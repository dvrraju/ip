// sh_l2_ctrl: L2 controller (version 1, one request at a time)
//
// Parameters:
//   TAG_RD_DEL  - tag array read latency in clk cycles; 0 = combinational
//   DATA_RD_DEL - data array read latency
//   LRU_RD_DEL  - LRU array read latency
//   RST_LOW     - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC   - reset type:  1 = asynchronous, 0 = synchronous
//
// Serves the request held by sh_l2_intake, then pulses req_done:
//
//   INIT      clear every set (tag invalid, LRU 0), once after reset
//   IDLE      wait for a request; a bad one goes straight to RSP (SLVERR)
//   LOOKUP    read the set's tags and LRU bits
//   COMPARE   hit: RD_HDR (read) or WR_HIT (write), and update LRU
//             miss: pick a victim; WB if it is dirty, then FULL_WR (write of
//             a whole line) or FILL_REQ
//   RD_HDR    send the read response header
//   RD_DATA   send len + 1 words from the data array
//   WR_HIT    merge the buffered words into the line, set dirty
//   WB_HDR    send the writeback request header
//   WB_DATA   send the victim's words
//   FILL_REQ  send the fill request
//   FILL_WAIT write fill words as they arrive; wait for the fill and any
//             writeback response
//   FULL_WR   write the whole line from the write buffer (no fill)
//   INSTALL   write the new tag and LRU; then replay the request (LOOKUP),
//             or RSP for a whole-line write
//   RSP       send a header-only response (write done, or an error)
//
// Data array reads go through sh_lib_fifo_rd_pipe, which hides DATA_RD_DEL.
// The fill and writeback use NoC tags FILL_TAG and WB_TAG. A writeback that
// returns an error pulses wb_err_evt.

module sh_l2_ctrl
  import ip_top_pkg::ADDR_W;
  import sh_noc_pkg::*;
  import sh_l2_pkg::*;
#(
  parameter int unsigned TAG_RD_DEL  = 0,
  parameter int unsigned DATA_RD_DEL = 0,
  parameter int unsigned LRU_RD_DEL  = 0,
  parameter logic        RST_LOW     = 1'b1,
  parameter logic        RST_ASYNC   = 1'b1
) (
  input  logic                         clk,
  input  logic                         rst,
  // Request from sh_l2_intake
  input  logic                         req_vld,
  /* verilator lint_off UNUSEDSIGNAL */
  input  hdr_t                         req_hdr,     // dst and status unused
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic                         req_bad,
  input  logic                         req_full,
  input  logic [WORDS-1:0][FLIT_W-1:0] wbuf_data,
  input  logic [WORDS-1:0][STRB_W-1:0] wbuf_strb,
  output logic                         req_done,
  // Responses to agents
  output logic                         t_rsp_vld,
  input  logic                         t_rsp_rdy,
  output flit_t                        t_rsp_flit,
  // Fills and writebacks
  output logic                         i_req_vld,
  input  logic                         i_req_rdy,
  output flit_t                        i_req_flit,
  input  logic                         i_rsp_vld,
  output logic                         i_rsp_rdy,
  input  flit_t                        i_rsp_flit,
  // Tag array
  output logic                         tag_mem_wr_en,
  output logic [SET_W-1:0]             tag_mem_wr_addr,
  output logic [TAG_ROW_W-1:0]         tag_mem_wr_data,
  output logic [NUM_WAYS-1:0]          tag_mem_wr_mask,
  output logic                         tag_mem_rd_en,
  output logic [SET_W-1:0]             tag_mem_rd_addr,
  input  logic [TAG_ROW_W-1:0]         tag_mem_rd_data,
  // Data array
  output logic                         data_mem_wr_en,
  output logic [DATA_AW-1:0]           data_mem_wr_addr,
  output logic [FLIT_W-1:0]            data_mem_wr_data,
  output logic [STRB_W-1:0]            data_mem_wr_strb,
  output logic                         data_mem_rd_en,
  output logic [DATA_AW-1:0]           data_mem_rd_addr,
  input  logic [FLIT_W-1:0]            data_mem_rd_data,
  // LRU array
  output logic                         lru_mem_wr_en,
  output logic [SET_W-1:0]             lru_mem_wr_addr,
  output logic [LRU_W-1:0]             lru_mem_wr_data,
  output logic                         lru_mem_rd_en,
  output logic [SET_W-1:0]             lru_mem_rd_addr,
  input  logic [LRU_W-1:0]             lru_mem_rd_data,
  // Status
  output logic                         wb_err_evt
);

  typedef enum logic [3:0] {
    S_INIT, S_IDLE, S_LOOKUP, S_COMPARE, S_RD_HDR, S_RD_DATA, S_WR_HIT,
    S_WB_HDR, S_WB_DATA, S_FILL_REQ, S_FILL_WAIT, S_FULL_WR, S_INSTALL, S_RSP
  } state_e;

  localparam int unsigned LKP_DEL = (TAG_RD_DEL > LRU_RD_DEL) ? TAG_RD_DEL : LRU_RD_DEL;
  localparam int unsigned CNT_MAX = (NUM_SETS > LKP_DEL + 1) ? NUM_SETS : LKP_DEL + 1;
  localparam int unsigned CNT_W   = $clog2(CNT_MAX + WORDS + 1);
  localparam int unsigned WCNT_W  = WORD_W + 1;   // counts 0 .. WORDS

  // ---------------------------------------------------------------------------
  // Request fields
  // ---------------------------------------------------------------------------

  logic [SET_W-1:0]    r_set;
  logic [L2_TAG_W-1:0] r_tag;
  logic [WORD_W-1:0]   r_word;
  logic [WCNT_W-1:0]   r_words;   // words the request covers (len + 1)

  assign r_set   = addr_set(req_hdr.addr);
  assign r_tag   = addr_tag(req_hdr.addr);
  assign r_word  = addr_word(req_hdr.addr);
  assign r_words = WCNT_W'(req_hdr.len) + 1'b1;

  // ---------------------------------------------------------------------------
  // State and registers
  // ---------------------------------------------------------------------------

  state_e                      state,      state_d;
  logic [CNT_W-1:0]            cnt,        cnt_d;
  logic [TAG_ROW_W-1:0]        tag_row;
  logic [LRU_W-1:0]            lru;
  logic [WAY_W-1:0]            way,        way_d;
  logic                        way_en;
  rsp_status_e                 rsp_st,     rsp_st_d;
  logic                        rsp_st_en;
  logic                        wb_pend;
  logic                        fill_done;
  logic                        fill_err;
  logic                        in_fill;
  logic [WCNT_W-1:0]           fill_cnt;
  tag_entry_t [NUM_WAYS-1:0]   row;

  assign row = tag_row;

  sh_lib_ff #(
    .WIDTH     ($bits(state_e)),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC),
    .RST_VAL   (S_INIT)
  ) u_state (
    .clk (clk),
    .rst (rst),
    .en  (1'b0),
    .d   (state_d),
    .q   (state)
  );

  sh_lib_ff #(
    .WIDTH     (CNT_W),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_cnt (
    .clk (clk),
    .rst (rst),
    .en  (1'b0),
    .d   (cnt_d),
    .q   (cnt)
  );

  // Tag row and LRU bits, captured when their reads return
  sh_lib_ff #(
    .WIDTH  (TAG_ROW_W),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_tag_row (
    .clk (clk),
    .rst (1'b0),
    .en  (state == S_LOOKUP && cnt == CNT_W'(TAG_RD_DEL)),
    .d   (tag_mem_rd_data),
    .q   (tag_row)
  );

  sh_lib_ff #(
    .WIDTH  (LRU_W),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_lru (
    .clk (clk),
    .rst (1'b0),
    .en  (state == S_LOOKUP && cnt == CNT_W'(LRU_RD_DEL)),
    .d   (lru_mem_rd_data),
    .q   (lru)
  );

  sh_lib_ff #(
    .WIDTH  (WAY_W),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_way (
    .clk (clk),
    .rst (1'b0),
    .en  (way_en),
    .d   (way_d),
    .q   (way)
  );

  sh_lib_ff #(
    .WIDTH  ($bits(rsp_status_e)),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_rsp_st (
    .clk (clk),
    .rst (1'b0),
    .en  (rsp_st_en),
    .d   (rsp_st_d),
    .q   (rsp_st)
  );

  // ---------------------------------------------------------------------------
  // Tag compare and victim selection
  // ---------------------------------------------------------------------------

  logic [NUM_WAYS-1:0] hit_vec;
  logic [NUM_WAYS-1:0] valid_vec;
  logic                hit;
  logic [WAY_W-1:0]    hit_way;
  logic [WAY_W-1:0]    victim;
  logic [WAY_W-1:0]    acc_way;
  logic [LRU_W-1:0]    lru_upd;
  logic                vict_dirty;

  always_comb begin
    hit_way = '0;
    for (int w = 0; w < NUM_WAYS; w++) begin
      valid_vec[w] = row[w].valid;
      hit_vec[w]   = row[w].valid && (row[w].tag == r_tag);
      if (hit_vec[w]) hit_way = WAY_W'(w);
    end
  end

  assign hit        = (hit_vec != '0);
  assign acc_way    = (state == S_COMPARE) ? hit_way : way;
  assign vict_dirty = row[victim].valid && row[victim].dirty;

  sh_lib_plru #(
    .WAYS (NUM_WAYS)
  ) u_plru (
    .state     (lru),
    .valid     (valid_vec),
    .victim    (victim),
    .acc_way   (acc_way),
    .state_upd (lru_upd)
  );

  // ---------------------------------------------------------------------------
  // Data read stream (read hits and writebacks)
  // ---------------------------------------------------------------------------

  logic              st_on;        // stream active in this state
  logic              st_wb;        // 1 = writeback, 0 = read hit
  logic [WCNT_W-1:0] st_total;
  logic [WCNT_W-1:0] st_issued;
  logic [WCNT_W-1:0] st_popped;
  logic              st_issue;
  logic              st_vld;
  logic              st_rdy;
  logic              st_pop;
  logic              st_last;
  logic [FLIT_W-1:0] st_data;
  logic [WORD_W-1:0] st_word;

  assign st_wb    = (state == S_WB_HDR || state == S_WB_DATA);
  assign st_on    = st_wb || state == S_RD_HDR || state == S_RD_DATA;
  assign st_total = st_wb ? WCNT_W'(WORDS) : r_words;
  assign st_word  = (st_wb ? '0 : r_word) + WORD_W'(st_issued);
  assign st_pop   = st_vld && st_rdy;
  assign st_last  = (st_popped == st_total - 1'b1);

  sh_lib_ff #(
    .WIDTH     (WCNT_W),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_st_issued (
    .clk (clk),
    .rst (rst),
    .en  (1'b0),
    .d   (st_on ? st_issued + WCNT_W'(st_issue) : '0),
    .q   (st_issued)
  );

  sh_lib_ff #(
    .WIDTH     (WCNT_W),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_st_popped (
    .clk (clk),
    .rst (rst),
    .en  (1'b0),
    .d   (st_on ? st_popped + WCNT_W'(st_pop) : '0),
    .q   (st_popped)
  );

  sh_lib_fifo_rd_pipe #(
    .WIDTH      (FLIT_W),
    .MEM_RD_DEL (DATA_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_rd_pipe (
    .clk         (clk),
    .rst         (rst),
    .mem_avail   (st_on && st_issued != st_total),
    .issue       (st_issue),
    .mem_rd_en   (data_mem_rd_en),
    .mem_rd_data (data_mem_rd_data),
    .rd_vld      (st_vld),
    .rd_rdy      (st_rdy),
    .rd_data     (st_data)
  );

  assign data_mem_rd_addr = {r_set, way, st_word};

  // ---------------------------------------------------------------------------
  // Fill and writeback responses (always accepted)
  // ---------------------------------------------------------------------------

  /* verilator lint_off UNUSEDSIGNAL */
  hdr_t i_rsp_hdr;     // only typ, tag and status are used
  /* verilator lint_on UNUSEDSIGNAL */
  logic i_rsp_take;
  logic fill_word;     // a fill data word is written this cycle
  logic wb_rsp;        // writeback response header
  logic fill_rsp;      // fill response header
  logic fill_end;      // last flit of the fill response
  logic fill_req_go;   // fill request accepted
  logic wb_req_go;     // writeback request header accepted

  assign i_rsp_rdy  = 1'b1;
  assign i_rsp_take = i_rsp_vld;
  assign i_rsp_hdr  = get_hdr(i_rsp_flit);
  assign wb_rsp     = i_rsp_take && i_rsp_flit.head && i_rsp_hdr.typ == PKT_WR_RSP
                      && i_rsp_hdr.tag == WB_TAG;
  assign fill_rsp   = i_rsp_take && i_rsp_flit.head && i_rsp_hdr.typ == PKT_RD_RSP
                      && i_rsp_hdr.tag == FILL_TAG;
  assign fill_word  = i_rsp_take && !i_rsp_flit.head && in_fill;
  assign fill_end   = (fill_rsp && i_rsp_flit.tail) || (fill_word && i_rsp_flit.tail);
  assign wb_err_evt = wb_rsp && i_rsp_hdr.status != RSP_OK;

  sh_lib_ff #(
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_wb_pend (
    .clk (clk),
    .rst (rst),
    .en  (wb_req_go || wb_rsp),
    .d   (wb_req_go),
    .q   (wb_pend)
  );

  sh_lib_ff #(
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_in_fill (
    .clk (clk),
    .rst (rst),
    .en  (fill_rsp || fill_word),
    .d   (!i_rsp_flit.tail),      // until the fill response's last flit
    .q   (in_fill)
  );

  sh_lib_ff #(
    .WIDTH     (WCNT_W),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_fill_cnt (
    .clk (clk),
    .rst (rst),
    .en  (fill_rsp || fill_word),
    .d   (fill_rsp ? '0 : fill_cnt + 1'b1),
    .q   (fill_cnt)
  );

  sh_lib_ff #(
    .WIDTH     (2),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_fill_st (
    .clk (clk),
    .rst (rst),
    .en  (fill_req_go || fill_end),
    .d   (fill_req_go ? 2'b00 : {1'b1, fill_rsp && i_rsp_hdr.status != RSP_OK}),
    .q   ({fill_done, fill_err})
  );

  // ---------------------------------------------------------------------------
  // Response and memory request flits
  // ---------------------------------------------------------------------------

  hdr_t rsp_hdr;
  hdr_t mem_hdr;

  always_comb begin
    rsp_hdr        = '0;
    rsp_hdr.dst_x  = req_hdr.src_x;
    rsp_hdr.dst_y  = req_hdr.src_y;
    rsp_hdr.typ    = (req_hdr.typ == PKT_RD_REQ) ? PKT_RD_RSP : PKT_WR_RSP;
    rsp_hdr.tag    = req_hdr.tag;
    rsp_hdr.len    = req_hdr.len;
    rsp_hdr.status = (state == S_RSP) ? rsp_st : RSP_OK;

    mem_hdr        = '0;
    mem_hdr.len    = LEN_W'(WORDS - 1);
    mem_hdr.status = RSP_OK;
    if (state == S_WB_HDR) begin
      mem_hdr.typ  = PKT_WR_REQ;
      mem_hdr.tag  = WB_TAG;
      mem_hdr.addr = line_addr(row[way].tag, r_set);
    end else begin
      mem_hdr.typ  = PKT_RD_REQ;
      mem_hdr.tag  = FILL_TAG;
      mem_hdr.addr = line_addr(r_tag, r_set);
    end
  end

  always_comb begin
    t_rsp_vld  = 1'b0;
    t_rsp_flit = '0;
    i_req_vld  = 1'b0;
    i_req_flit = '0;
    st_rdy     = 1'b0;

    case (state)
      S_RD_HDR: begin
        t_rsp_vld       = 1'b1;
        t_rsp_flit.head = 1'b1;
        t_rsp_flit.data = hdr_to_data(rsp_hdr);
      end
      S_RD_DATA: begin
        t_rsp_vld       = st_vld;
        t_rsp_flit.tail = st_last;
        t_rsp_flit.data = st_data;
        st_rdy          = t_rsp_rdy;
      end
      S_RSP: begin
        t_rsp_vld       = 1'b1;
        t_rsp_flit.head = 1'b1;
        t_rsp_flit.tail = 1'b1;
        t_rsp_flit.data = hdr_to_data(rsp_hdr);
      end
      S_WB_HDR: begin
        i_req_vld       = 1'b1;
        i_req_flit.head = 1'b1;
        i_req_flit.data = hdr_to_data(mem_hdr);
      end
      S_WB_DATA: begin
        i_req_vld       = st_vld;
        i_req_flit.tail = st_last;
        i_req_flit.strb = '1;
        i_req_flit.data = st_data;
        st_rdy          = i_req_rdy;
      end
      S_FILL_REQ: begin
        i_req_vld       = 1'b1;
        i_req_flit.head = 1'b1;
        i_req_flit.tail = 1'b1;
        i_req_flit.data = hdr_to_data(mem_hdr);
      end
      default: ;
    endcase
  end

  assign wb_req_go   = (state == S_WB_HDR)   && i_req_rdy;
  assign fill_req_go = (state == S_FILL_REQ) && i_req_rdy;

  // ---------------------------------------------------------------------------
  // Array writes
  // ---------------------------------------------------------------------------

  tag_entry_t new_entry;

  always_comb begin
    // Tag array
    tag_mem_wr_en   = 1'b0;
    tag_mem_wr_addr = r_set;
    tag_mem_wr_mask = '0;
    new_entry       = '0;
    case (state)
      S_INIT: begin
        tag_mem_wr_en   = 1'b1;
        tag_mem_wr_addr = SET_W'(cnt);
        tag_mem_wr_mask = '1;
      end
      S_WR_HIT: begin
        // Mark the line dirty once
        tag_mem_wr_en   = (cnt == '0) && !row[way].dirty;
        tag_mem_wr_mask = NUM_WAYS'(1) << way;
        new_entry       = '{valid: 1'b1, dirty: 1'b1, tag: r_tag};
      end
      S_INSTALL: begin
        tag_mem_wr_en   = !wb_pend;
        tag_mem_wr_mask = NUM_WAYS'(1) << way;
        new_entry       = '{valid: 1'b1, dirty: req_full, tag: r_tag};
      end
      default: ;
    endcase
    tag_mem_wr_data = {NUM_WAYS{new_entry}};

    // LRU array
    lru_mem_wr_en   = 1'b0;
    lru_mem_wr_addr = r_set;
    lru_mem_wr_data = lru_upd;
    case (state)
      S_INIT: begin
        lru_mem_wr_en   = 1'b1;
        lru_mem_wr_addr = SET_W'(cnt);
        lru_mem_wr_data = '0;
      end
      S_COMPARE: lru_mem_wr_en = hit;
      S_INSTALL: lru_mem_wr_en = !wb_pend;
      default: ;
    endcase

    // Data array (one write port)
    data_mem_wr_en   = 1'b0;
    data_mem_wr_addr = {r_set, way, WORD_W'(0)};
    data_mem_wr_data = '0;
    data_mem_wr_strb = '0;
    if (fill_word) begin
      data_mem_wr_en   = 1'b1;
      data_mem_wr_addr = {r_set, way, WORD_W'(fill_cnt)};
      data_mem_wr_data = i_rsp_flit.data;
      data_mem_wr_strb = '1;
    end else if (state == S_WR_HIT) begin
      data_mem_wr_en   = 1'b1;
      data_mem_wr_addr = {r_set, way, r_word + WORD_W'(cnt)};
      data_mem_wr_data = wbuf_data[WORD_W'(cnt)];
      data_mem_wr_strb = wbuf_strb[WORD_W'(cnt)];
    end else if (state == S_FULL_WR) begin
      data_mem_wr_en   = 1'b1;
      data_mem_wr_addr = {r_set, way, WORD_W'(cnt)};
      data_mem_wr_data = wbuf_data[WORD_W'(cnt)];
      data_mem_wr_strb = '1;
    end
  end

  // Lookup reads
  assign tag_mem_rd_en   = (state == S_LOOKUP) && (cnt == '0);
  assign tag_mem_rd_addr = r_set;
  assign lru_mem_rd_en   = (state == S_LOOKUP) && (cnt == '0);
  assign lru_mem_rd_addr = r_set;

  // ---------------------------------------------------------------------------
  // Next state
  // ---------------------------------------------------------------------------

  always_comb begin
    state_d   = state;
    cnt_d     = cnt;
    way_d     = way;
    way_en    = 1'b0;
    rsp_st_d  = RSP_OK;
    rsp_st_en = 1'b0;
    req_done  = 1'b0;

    case (state)
      S_INIT: begin
        cnt_d = cnt + 1'b1;
        if (cnt == CNT_W'(NUM_SETS - 1)) begin
          state_d = S_IDLE;
          cnt_d   = '0;
        end
      end

      S_IDLE: begin
        cnt_d = '0;
        if (req_vld) begin
          if (req_bad) begin
            state_d   = S_RSP;
            rsp_st_d  = RSP_SLVERR;
            rsp_st_en = 1'b1;
          end else begin
            state_d = S_LOOKUP;
          end
        end
      end

      S_LOOKUP: begin
        cnt_d = cnt + 1'b1;
        if (cnt == CNT_W'(LKP_DEL)) begin
          state_d = S_COMPARE;
          cnt_d   = '0;
        end
      end

      S_COMPARE: begin
        way_en = 1'b1;
        if (hit) begin
          way_d   = hit_way;
          state_d = (req_hdr.typ == PKT_RD_REQ) ? S_RD_HDR : S_WR_HIT;
        end else begin
          way_d   = victim;
          state_d = vict_dirty ? S_WB_HDR : (req_full ? S_FULL_WR : S_FILL_REQ);
        end
      end

      S_RD_HDR: begin
        if (t_rsp_rdy) state_d = S_RD_DATA;
      end

      S_RD_DATA: begin
        if (st_pop && st_last) begin
          state_d  = S_IDLE;
          req_done = 1'b1;
        end
      end

      S_WR_HIT: begin
        cnt_d = cnt + 1'b1;
        if (cnt == CNT_W'(req_hdr.len)) begin
          state_d   = S_RSP;
          cnt_d     = '0;
          rsp_st_d  = RSP_OK;
          rsp_st_en = 1'b1;
        end
      end

      S_WB_HDR: begin
        if (i_req_rdy) state_d = S_WB_DATA;
      end

      S_WB_DATA: begin
        if (st_pop && st_last) state_d = req_full ? S_FULL_WR : S_FILL_REQ;
      end

      S_FILL_REQ: begin
        if (i_req_rdy) state_d = S_FILL_WAIT;
      end

      S_FILL_WAIT: begin
        if (fill_done && !wb_pend) begin
          if (fill_err) begin
            state_d   = S_RSP;
            rsp_st_d  = RSP_SLVERR;
            rsp_st_en = 1'b1;
          end else begin
            state_d = S_INSTALL;
          end
        end
      end

      S_FULL_WR: begin
        cnt_d = cnt + 1'b1;
        if (cnt == CNT_W'(WORDS - 1)) begin
          state_d = S_INSTALL;
          cnt_d   = '0;
        end
      end

      S_INSTALL: begin
        if (!wb_pend) begin
          if (req_full) begin
            state_d   = S_RSP;
            rsp_st_d  = RSP_OK;
            rsp_st_en = 1'b1;
          end else begin
            state_d = S_LOOKUP;   // replay: now a hit
          end
        end
      end

      S_RSP: begin
        if (t_rsp_rdy) begin
          state_d  = S_IDLE;
          req_done = 1'b1;
        end
      end

      default: state_d = S_IDLE;
    endcase
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (i_rsp_take && i_rsp_flit.head && !wb_rsp && !fill_rsp)
      $error("%m: unexpected memory response (type %0d, tag %0d)", i_rsp_hdr.typ, i_rsp_hdr.tag);
    if (i_rsp_take && !i_rsp_flit.head && !in_fill)
      $error("%m: unexpected memory response data flit");
    if (fill_word && fill_cnt >= WCNT_W'(WORDS))
      $error("%m: fill response longer than a line");
  end
`endif

endmodule
