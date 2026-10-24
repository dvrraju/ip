// sh_l2_ctrl: L2 controller (version 2: hit-under-miss, multiple misses)
//
// Parameters:
//   TAG_RD_DEL  - tag array read latency in clk cycles; 0 = combinational
//   DATA_RD_DEL - data array read latency
//   LRU_RD_DEL  - LRU array read latency
//   PF_RD_DEL   - pending FIFO memory read latency
//   RST_LOW     - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC   - reset type:  1 = asynchronous, 0 = synchronous
//
// The pipeline takes the oldest pending FIFO entry that is ready to go
// (NEW, READY or NEEDFILL, with no dependency outstanding), reads its command
// into the active register and handles it. Because a command waits for the
// one before it to the same line (sh_l2_pfifo), the pipeline never holds two
// commands of one line at a time:
//
//   INIT      clear every set (tag invalid, LRU 0), once after reset
//   IDLE      pick an entry; NEEDFILL goes straight to FILL_REQ
//   PF_RD     read the entry from the pending FIFO memory
//   CHECK     a failed fill or a bad request (crosses a line, not a
//             request): RSP with SLVERR
//   LOOKUP    read the set's tags and LRU bits
//   COMPARE   hit: RD_HDR (read) or WR_HIT (write), and update LRU
//             miss: pick a victim that no outstanding fill has reserved (none:
//             NO_WAY); invalidate it; WB if it is dirty; then FULL_WR (a
//             write of a whole line) or the fill (FILL_REQ, or WB_WAIT if
//             the line's own writeback is still outstanding)
//   RD_HDR,
//   RD_DATA   read response: header, then len + 1 words from the data array
//   WR_HIT    merge the active write into the line, set dirty
//   WB_HDR,
//   WB_DATA   writeback of the victim (NoC tag = entry index)
//   FILL_REQ  send the fill (NoC tag = entry index); the entry waits in FILL
//   FULL_WR,
//   FULL_INST write the whole line and install it dirty
//   RSP       header-only response (write done, or an error)
//
// Responses and the pipeline's entry is freed when its response is sent.
//
// Fills return independently of the pipeline: their words are written into
// the reserved way as they arrive, then the tag is installed and the entry
// (and entries waiting on it) wakes up to replay. Writeback responses clear
// the entry's wb_pend and wake fills waiting on that line. Responses from
// memory may arrive in any order; each is matched by tag (entry index) and
// type. On a data array write conflict the pipeline wins and i_rsp waits.

module sh_l2_ctrl
  import ip_top_pkg::ADDR_W;
  import sh_noc_pkg::*;
  import sh_l2_pkg::*;
#(
  parameter int unsigned TAG_RD_DEL  = 0,
  parameter int unsigned DATA_RD_DEL = 0,
  parameter int unsigned LRU_RD_DEL  = 0,
  parameter int unsigned PF_RD_DEL   = 0,
  parameter logic        RST_LOW     = 1'b1,
  parameter logic        RST_ASYNC   = 1'b1
) (
  input  logic                         clk,
  input  logic                         rst,
  // Pending FIFO
  input  ent_t [PF_DEPTH-1:0]          ent,
  input  logic [PF_IW-1:0]             head,
  output logic                         upd_en,
  output logic [PF_IW-1:0]             upd_idx,
  output ent_t                         upd_ent,
  output logic                         inst_en,
  output logic [PF_IW-1:0]             inst_idx,
  output logic                         inst_err,
  output logic                         wbr_en,
  output logic [PF_IW-1:0]             wbr_idx,
  output logic                         release_en,
  output logic                         pf_mem_rd_en,
  output logic [PF_IW-1:0]             pf_mem_rd_addr,
  input  logic [PF_ENTRY_W-1:0]        pf_mem_rd_data,
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
    P_INIT, P_IDLE, P_PF_RD, P_CHECK, P_LOOKUP, P_COMPARE, P_RD_HDR, P_RD_DATA,
    P_WR_HIT, P_WB_HDR, P_WB_DATA, P_FILL_REQ, P_FULL_WR, P_FULL_INST, P_RSP
  } pstate_e;

  localparam int unsigned LKP_DEL = (TAG_RD_DEL > LRU_RD_DEL) ? TAG_RD_DEL : LRU_RD_DEL;
  localparam int unsigned DEL_MAX = (LKP_DEL > PF_RD_DEL) ? LKP_DEL : PF_RD_DEL;
  localparam int unsigned CNT_MAX = (NUM_SETS > DEL_MAX + 1) ? NUM_SETS : DEL_MAX + 1;
  localparam int unsigned CNT_W   = $clog2(CNT_MAX + WORDS + 1);
  localparam int unsigned WCNT_W  = WORD_W + 1;   // counts 0 .. WORDS

  // ---------------------------------------------------------------------------
  // Pipeline registers
  // ---------------------------------------------------------------------------

  pstate_e                      state, state_d;
  logic [CNT_W-1:0]             cnt,   cnt_d;
  logic [PF_IW-1:0]             act,   act_d;     // entry in the pipeline
  logic                         act_en;
  logic [WAY_W-1:0]             way,   way_d;     // hit way or victim
  logic                         way_en;
  rsp_status_e                  rsp_st, rsp_st_d;
  logic                         rsp_st_en;
  logic                         wb_go;            // after WB_DATA: 1 = fill, 0 = whole-line write
  logic                         wb_go_d;
  logic [TAG_ROW_W-1:0]         tag_row;
  logic [LRU_W-1:0]             lru;
  logic [PF_ENTRY_W-1:0]        areq;             // active request, as in the buffer
  tag_entry_t [NUM_WAYS-1:0]    row;

  assign row = tag_row;

  sh_lib_ff #(
    .WIDTH     ($bits(pstate_e)),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC),
    .RST_VAL   (P_INIT)
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

  sh_lib_ff #(
    .WIDTH  (PF_IW),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_act (
    .clk (clk),
    .rst (1'b0),
    .en  (act_en),
    .d   (act_d),
    .q   (act)
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
    .WIDTH  ($bits(rsp_status_e) + 1),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_rsp_st (
    .clk (clk),
    .rst (1'b0),
    .en  (rsp_st_en),
    .d   ({rsp_st_d, wb_go_d}),
    .q   ({rsp_st, wb_go})
  );

  sh_lib_ff #(
    .WIDTH  (PF_ENTRY_W),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_areq (
    .clk (clk),
    .rst (1'b0),
    .en  (state == P_PF_RD && cnt == CNT_W'(PF_RD_DEL)),
    .d   (pf_mem_rd_data),
    .q   (areq)
  );

  sh_lib_ff #(
    .WIDTH  (TAG_ROW_W),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_tag_row (
    .clk (clk),
    .rst (1'b0),
    .en  (state == P_LOOKUP && cnt == CNT_W'(TAG_RD_DEL)),
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
    .en  (state == P_LOOKUP && cnt == CNT_W'(LRU_RD_DEL)),
    .d   (lru_mem_rd_data),
    .q   (lru)
  );

  // ---------------------------------------------------------------------------
  // Active request
  // ---------------------------------------------------------------------------

  /* verilator lint_off UNUSEDSIGNAL */
  hdr_t                         a_hdr;      // dst and status unused
  logic [PF_SLOTS-1:0][SLOT_W-1:0] a_slot;
  /* verilator lint_on UNUSEDSIGNAL */
  logic [WORDS-1:0][FLIT_W-1:0] a_data;
  logic [WORDS-1:0][STRB_W-1:0] a_strb;
  ent_t                         a_ent;
  logic [LINE_W-1:0]            a_line;
  logic [SET_W-1:0]             a_set;
  logic [L2_TAG_W-1:0]          a_tag;
  logic [WORD_W-1:0]            a_word;
  logic [WCNT_W-1:0]            a_words;
  logic                         a_bad;
  logic                         a_full;
  logic                         full_ok;    // whole-line write that may skip the fill
  logic                         strb_all;

  assign a_slot = areq;
  assign a_hdr  = hdr_t'(a_slot[0][HDR_W-1:0]);

  always_comb begin
    strb_all = 1'b1;
    for (int i = 0; i < WORDS; i++) begin
      {a_strb[i], a_data[i]} = a_slot[i + 1][STRB_W + FLIT_W - 1:0];
      strb_all &= (a_strb[i] == '1);
    end
  end

  assign a_ent   = ent[act];
  assign a_line  = a_ent.line;
  assign a_set   = line_set(a_line);
  assign a_tag   = line_tag(a_line);
  assign a_word  = addr_word(a_hdr.addr);
  assign a_words = WCNT_W'(a_hdr.len) + 1'b1;
  assign a_bad   = !(a_hdr.typ inside {PKT_RD_REQ, PKT_WR_REQ})
                   || (32'(a_word) + 32'(a_hdr.len) > WORDS - 1);
  assign a_full  = (a_hdr.typ == PKT_WR_REQ) && (a_word == '0)
                   && (32'(a_hdr.len) == WORDS - 1) && strb_all;

  // A line whose writeback is still outstanding must not be written back
  // again before it completes, so a whole-line write to it takes the fill path
  assign full_ok = a_full && !wbm_vld;

  // ---------------------------------------------------------------------------
  // Pick: the oldest entry that is ready to go
  // ---------------------------------------------------------------------------

  logic              pick_vld;
  logic [PF_IW-1:0] pick;

  always_comb begin
    logic [PF_IW-1:0] idx;
    pick_vld = 1'b0;
    pick     = head;
    for (int k = 0; k < PF_DEPTH; k++) begin
      idx = head + PF_IW'(k);
      if (!pick_vld && !ent[idx].dep && ent[idx].state inside {E_NEW, E_READY, E_NEEDFILL}) begin
        pick_vld = 1'b1;
        pick     = idx;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Searches across the FIFO (victim selection and the fill decision)
  // ---------------------------------------------------------------------------

  logic [NUM_WAYS-1:0] reserved;      // ways of the set reserved by outstanding fills
  logic                wbm_vld;       // the line's writeback is outstanding
  logic [PF_IW-1:0]     wbm_idx;

  always_comb begin
    reserved = '0;
    wbm_vld  = 1'b0;
    wbm_idx  = '0;
    for (int i = 0; i < PF_DEPTH; i++) begin
      if (ent[i].prim && ent[i].state != E_FREE && PF_IW'(i) != act
          && line_set(ent[i].line) == a_set)
        reserved[ent[i].way] = 1'b1;
      if (ent[i].wb_pend && ent[i].wb_line == a_line && !(wbr_en && wbr_idx == PF_IW'(i))) begin
        wbm_vld = 1'b1;
        wbm_idx = PF_IW'(i);
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Tag compare and victim selection
  // ---------------------------------------------------------------------------

  logic [NUM_WAYS-1:0] hit_vec;
  logic [NUM_WAYS-1:0] valid_vec;
  logic                hit;
  logic [WAY_W-1:0]    hit_way;
  logic [WAY_W-1:0]    plru_way;
  logic [WAY_W-1:0]    acc_way;
  logic [LRU_W-1:0]    lru_upd;
  logic                vict_vld;
  logic [WAY_W-1:0]    victim;
  logic                vict_dirty;

  always_comb begin
    hit_way = '0;
    for (int w = 0; w < NUM_WAYS; w++) begin
      valid_vec[w] = row[w].valid;
      hit_vec[w]   = row[w].valid && (row[w].tag == a_tag);
      if (hit_vec[w]) hit_way = WAY_W'(w);
    end
  end

  assign hit     = (hit_vec != '0);
  assign acc_way = (state == P_COMPARE) ? hit_way : way;

  // Reserved ways count as valid, so an invalid unreserved way goes first
  sh_lib_plru #(
    .WAYS (NUM_WAYS)
  ) u_plru (
    .state     (lru),
    .valid     (valid_vec | reserved),
    .victim    (plru_way),
    .acc_way   (acc_way),
    .state_upd (lru_upd)
  );

  always_comb begin
    vict_vld = !reserved[plru_way];
    victim   = plru_way;
    for (int w = NUM_WAYS - 1; w >= 0; w--) begin
      if (!vict_vld && !reserved[w]) begin
        vict_vld = 1'b1;
        victim   = WAY_W'(w);
      end
    end
  end

  assign vict_dirty = row[victim].valid && row[victim].dirty;

  // ---------------------------------------------------------------------------
  // Data read stream (read hits and writebacks)
  // ---------------------------------------------------------------------------

  logic              st_on;
  logic              st_wb;
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

  assign st_wb    = (state == P_WB_HDR || state == P_WB_DATA);
  assign st_on    = st_wb || state == P_RD_HDR || state == P_RD_DATA;
  assign st_total = st_wb ? WCNT_W'(WORDS) : a_words;
  assign st_word  = (st_wb ? '0 : a_word) + WORD_W'(st_issued);
  assign st_pop   = st_vld && st_rdy;
  assign st_last  = (st_popped == st_total - 1'b1);

  sh_lib_ff #(
    .WIDTH     (2 * WCNT_W),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_st_cnt (
    .clk (clk),
    .rst (rst),
    .en  (1'b0),
    .d   (st_on ? {st_issued + WCNT_W'(st_issue), st_popped + WCNT_W'(st_pop)} : '0),
    .q   ({st_issued, st_popped})
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

  assign data_mem_rd_addr = {a_set, way, st_word};

  // ---------------------------------------------------------------------------
  // Fill and writeback responses (independent of the pipeline)
  // ---------------------------------------------------------------------------

  /* verilator lint_off UNUSEDSIGNAL */
  hdr_t              r_hdr;         // only typ, tag and status are used
  /* verilator lint_on UNUSEDSIGNAL */
  logic [PF_IW-1:0] r_idx;
  logic              r_take;
  logic              r_fill_hdr;
  logic              r_wb_hdr;
  logic              r_fill_word;
  logic              in_fill;
  logic [PF_IW-1:0] fill_idx;
  logic [WCNT_W-1:0] fill_cnt;
  logic              inst_pend;
  logic              inst_pend_err;
  logic [PF_IW-1:0] inst_pend_idx;
  logic              inst_set;
  logic              pipe_data_wr;
  logic              pipe_tag_wr;
  /* verilator lint_off UNUSEDSIGNAL */
  ent_t              f_ent;         // entry of the fill in progress: set and way used
  /* verilator lint_on UNUSEDSIGNAL */

  assign r_hdr       = get_hdr(i_rsp_flit);
  assign r_idx       = r_hdr.tag[PF_IW-1:0];
  assign i_rsp_rdy   = !pipe_data_wr && !inst_pend;
  assign r_take      = i_rsp_vld && i_rsp_rdy;
  assign r_fill_hdr  = r_take && i_rsp_flit.head && r_hdr.typ == PKT_RD_RSP;
  assign r_wb_hdr    = r_take && i_rsp_flit.head && r_hdr.typ == PKT_WR_RSP;
  assign r_fill_word = r_take && !i_rsp_flit.head && in_fill;
  assign f_ent       = ent[fill_idx];

  assign wbr_en      = r_wb_hdr;
  assign wbr_idx     = r_idx;
  assign wb_err_evt  = r_wb_hdr && r_hdr.status != RSP_OK;

  // A fill ends with its last word, or with a header-only error response
  assign inst_set = (r_fill_hdr && i_rsp_flit.tail) || (r_fill_word && i_rsp_flit.tail);

  sh_lib_ff #(
    .WIDTH     (1 + PF_IW + WCNT_W),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_fill (
    .clk (clk),
    .rst (rst),
    .en  (r_fill_hdr || r_fill_word),
    .d   (r_fill_hdr ? {!i_rsp_flit.tail, r_idx, WCNT_W'(0)}
                     : {!i_rsp_flit.tail, fill_idx, fill_cnt + 1'b1}),
    .q   ({in_fill, fill_idx, fill_cnt})
  );

  // Install waits for a cycle in which the pipeline is not writing tags
  assign inst_en  = inst_pend && !pipe_tag_wr;
  assign inst_idx = inst_pend_idx;
  assign inst_err = inst_pend_err;

  sh_lib_ff #(
    .WIDTH     (2 + PF_IW),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_inst (
    .clk (clk),
    .rst (rst),
    .en  (inst_set || inst_en),
    .d   (inst_set ? {1'b1, r_fill_hdr && r_hdr.status != RSP_OK, r_fill_hdr ? r_idx : fill_idx}
                   : {1'b0, inst_pend_err, inst_pend_idx}),
    .q   ({inst_pend, inst_pend_err, inst_pend_idx})
  );

  // ---------------------------------------------------------------------------
  // Response and memory request flits
  // ---------------------------------------------------------------------------

  hdr_t rsp_hdr;
  hdr_t mem_hdr;

  always_comb begin
    rsp_hdr        = '0;
    rsp_hdr.dst_x  = a_hdr.src_x;
    rsp_hdr.dst_y  = a_hdr.src_y;
    rsp_hdr.typ    = (a_hdr.typ == PKT_RD_REQ) ? PKT_RD_RSP : PKT_WR_RSP;
    rsp_hdr.tag    = a_hdr.tag;
    rsp_hdr.len    = a_hdr.len;
    rsp_hdr.status = (state == P_RSP) ? rsp_st : RSP_OK;

    mem_hdr        = '0;
    mem_hdr.tag    = TAG_W'(act);
    mem_hdr.len    = LEN_W'(WORDS - 1);
    mem_hdr.status = RSP_OK;
    if (state == P_WB_HDR) begin
      mem_hdr.typ  = PKT_WR_REQ;
      mem_hdr.addr = line_addr(row[way].tag, a_set);
    end else begin
      mem_hdr.typ  = PKT_RD_REQ;
      mem_hdr.addr = line_addr(a_tag, a_set);
    end
  end

  always_comb begin
    t_rsp_vld  = 1'b0;
    t_rsp_flit = '0;
    i_req_vld  = 1'b0;
    i_req_flit = '0;
    st_rdy     = 1'b0;

    case (state)
      P_RD_HDR: begin
        t_rsp_vld       = 1'b1;
        t_rsp_flit.head = 1'b1;
        t_rsp_flit.data = hdr_to_data(rsp_hdr);
      end
      P_RD_DATA: begin
        t_rsp_vld       = st_vld;
        t_rsp_flit.tail = st_last;
        t_rsp_flit.data = st_data;
        st_rdy          = t_rsp_rdy;
      end
      P_RSP: begin
        t_rsp_vld       = 1'b1;
        t_rsp_flit.head = 1'b1;
        t_rsp_flit.tail = 1'b1;
        t_rsp_flit.data = hdr_to_data(rsp_hdr);
      end
      P_WB_HDR: begin
        i_req_vld       = 1'b1;
        i_req_flit.head = 1'b1;
        i_req_flit.data = hdr_to_data(mem_hdr);
      end
      P_WB_DATA: begin
        i_req_vld       = st_vld;
        i_req_flit.tail = st_last;
        i_req_flit.strb = '1;
        i_req_flit.data = st_data;
        st_rdy          = i_req_rdy;
      end
      P_FILL_REQ: begin
        i_req_vld       = 1'b1;
        i_req_flit.head = 1'b1;
        i_req_flit.tail = 1'b1;
        i_req_flit.data = hdr_to_data(mem_hdr);
      end
      default: ;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Array accesses
  // ---------------------------------------------------------------------------

  tag_entry_t new_entry;

  // Tag writes by the pipeline (a fill install waits for a cycle without one)
  always_comb begin
    case (state)
      P_INIT:      pipe_tag_wr = 1'b1;
      P_COMPARE:   pipe_tag_wr = !hit && vict_vld;                              // invalidate the victim
      P_WR_HIT:    pipe_tag_wr = (cnt == '0) && !row[way].dirty;                // mark dirty once
      P_FULL_INST: pipe_tag_wr = 1'b1;
      default:     pipe_tag_wr = 1'b0;
    endcase
  end

  always_comb begin
    // Tag array: the pipeline first, then a fill install
    tag_mem_wr_addr = a_set;
    tag_mem_wr_mask = '0;
    new_entry       = '0;
    case (state)
      P_INIT: begin
        tag_mem_wr_addr = SET_W'(cnt);
        tag_mem_wr_mask = '1;
      end
      P_COMPARE: begin
        tag_mem_wr_mask = NUM_WAYS'(1) << victim;
      end
      P_WR_HIT, P_FULL_INST: begin
        tag_mem_wr_mask = NUM_WAYS'(1) << way;
        new_entry       = '{valid: 1'b1, dirty: 1'b1, tag: a_tag};
      end
      default: ;
    endcase
    tag_mem_wr_en = pipe_tag_wr;
    if (inst_en && !inst_pend_err) begin
      tag_mem_wr_en   = 1'b1;
      tag_mem_wr_addr = line_set(ent[inst_pend_idx].line);
      tag_mem_wr_mask = NUM_WAYS'(1) << ent[inst_pend_idx].way;
      new_entry       = '{valid: 1'b1, dirty: 1'b0, tag: line_tag(ent[inst_pend_idx].line)};
    end
    tag_mem_wr_data = {NUM_WAYS{new_entry}};

    // LRU array
    lru_mem_wr_en   = 1'b0;
    lru_mem_wr_addr = a_set;
    lru_mem_wr_data = lru_upd;
    case (state)
      P_INIT: begin
        lru_mem_wr_en   = 1'b1;
        lru_mem_wr_addr = SET_W'(cnt);
        lru_mem_wr_data = '0;
      end
      P_COMPARE:   lru_mem_wr_en = hit;
      P_FULL_INST: lru_mem_wr_en = 1'b1;
      default: ;
    endcase

    // Data array: the pipeline first, then fill words
    pipe_data_wr     = (state == P_WR_HIT) || (state == P_FULL_WR);
    data_mem_wr_en   = 1'b0;
    data_mem_wr_addr = {a_set, way, WORD_W'(0)};
    data_mem_wr_data = '0;
    data_mem_wr_strb = '0;
    if (state == P_WR_HIT) begin
      data_mem_wr_en   = 1'b1;
      data_mem_wr_addr = {a_set, way, a_word + WORD_W'(cnt)};
      data_mem_wr_data = a_data[WORD_W'(cnt)];
      data_mem_wr_strb = a_strb[WORD_W'(cnt)];
    end else if (state == P_FULL_WR) begin
      data_mem_wr_en   = 1'b1;
      data_mem_wr_addr = {a_set, way, WORD_W'(cnt)};
      data_mem_wr_data = a_data[WORD_W'(cnt)];
      data_mem_wr_strb = '1;
    end else if (r_fill_word) begin
      data_mem_wr_en   = 1'b1;
      data_mem_wr_addr = {line_set(f_ent.line), f_ent.way, WORD_W'(fill_cnt)};
      data_mem_wr_data = i_rsp_flit.data;
      data_mem_wr_strb = '1;
    end
  end

  assign tag_mem_rd_en   = (state == P_LOOKUP) && (cnt == '0);
  assign tag_mem_rd_addr = a_set;
  assign lru_mem_rd_en   = (state == P_LOOKUP) && (cnt == '0);
  assign lru_mem_rd_addr = a_set;
  assign pf_mem_rd_en   = (state == P_PF_RD) && (cnt == '0);
  assign pf_mem_rd_addr = act;

  // ---------------------------------------------------------------------------
  // Next state
  // ---------------------------------------------------------------------------

  always_comb begin
    ent_t e;

    state_d    = state;
    cnt_d      = cnt;
    act_d      = act;
    act_en     = 1'b0;
    way_d      = way;
    way_en     = 1'b0;
    rsp_st_d   = rsp_st;
    wb_go_d    = wb_go;
    rsp_st_en  = 1'b0;
    upd_en     = 1'b0;
    upd_idx    = act;
    release_en = 1'b0;
    e          = a_ent;

    case (state)
      P_INIT: begin
        cnt_d = cnt + 1'b1;
        if (cnt == CNT_W'(NUM_SETS - 1)) begin
          state_d = P_IDLE;
          cnt_d   = '0;
        end
      end

      P_IDLE: begin
        cnt_d = '0;
        if (pick_vld) begin
          act_d   = pick;
          act_en  = 1'b1;
          upd_en  = 1'b1;
          upd_idx = pick;
          e       = ent[pick];
          state_d = (e.state == E_NEEDFILL) ? P_FILL_REQ : P_PF_RD;
          e.state = E_ACTIVE;
        end
      end

      P_PF_RD: begin
        cnt_d = cnt + 1'b1;
        if (cnt == CNT_W'(PF_RD_DEL)) begin
          state_d = P_CHECK;
          cnt_d   = '0;
        end
      end

      P_CHECK: begin
        if (a_ent.err || a_bad) begin
          state_d   = P_RSP;
          rsp_st_d  = RSP_SLVERR;
          rsp_st_en = 1'b1;
        end else begin
          state_d = P_LOOKUP;
        end
      end

      P_LOOKUP: begin
        cnt_d = cnt + 1'b1;
        if (cnt == CNT_W'(LKP_DEL)) begin
          state_d = P_COMPARE;
          cnt_d   = '0;
        end
      end

      P_COMPARE: begin
        way_en = 1'b1;
        if (hit) begin
          way_d   = hit_way;
          state_d = (a_hdr.typ == PKT_RD_REQ) ? P_RD_HDR : P_WR_HIT;
        end else if (!vict_vld) begin
          upd_en  = 1'b1;
          e.state = E_NO_WAY;
          e.prim  = 1'b0;
          state_d = P_IDLE;
        end else begin
          way_d     = victim;
          wb_go_d   = !full_ok;
          rsp_st_en = 1'b1;
          if (!full_ok) begin
            upd_en = 1'b1;
            e.prim = 1'b1;
            e.way  = victim;
          end
          if (vict_dirty) begin
            state_d = P_WB_HDR;
          end else if (full_ok) begin
            state_d = P_FULL_WR;
          end else if (wbm_vld) begin
            e.state   = E_WB_WAIT;
            e.wait_on = wbm_idx;
            state_d   = P_IDLE;
          end else begin
            state_d = P_FILL_REQ;
          end
        end
      end

      P_RD_HDR: begin
        if (t_rsp_rdy) state_d = P_RD_DATA;
      end

      P_RD_DATA: begin
        if (st_pop && st_last) begin
          upd_en     = 1'b1;
          release_en = a_ent.prim;
          e.state    = E_FREE;
          e.prim     = 1'b0;
          state_d    = P_IDLE;
        end
      end

      P_WR_HIT: begin
        cnt_d = cnt + 1'b1;
        if (cnt == CNT_W'(a_hdr.len)) begin
          state_d   = P_RSP;
          cnt_d     = '0;
          rsp_st_d  = RSP_OK;
          rsp_st_en = 1'b1;
        end
      end

      P_WB_HDR: begin
        if (i_req_rdy) begin
          upd_en    = 1'b1;
          e.wb_pend = 1'b1;
          e.wb_line = addr_line(line_addr(row[way].tag, a_set));
          state_d   = P_WB_DATA;
        end
      end

      P_WB_DATA: begin
        if (st_pop && st_last) begin
          if (!wb_go) begin
            state_d = P_FULL_WR;
          end else if (wbm_vld) begin
            upd_en    = 1'b1;
            e.state   = E_WB_WAIT;
            e.wait_on = wbm_idx;
            state_d   = P_IDLE;
          end else begin
            state_d = P_FILL_REQ;
          end
        end
      end

      P_FILL_REQ: begin
        if (i_req_rdy) begin
          upd_en  = 1'b1;
          e.state = E_FILL;
          state_d = P_IDLE;
        end
      end

      P_FULL_WR: begin
        cnt_d = cnt + 1'b1;
        if (cnt == CNT_W'(WORDS - 1)) begin
          state_d = P_FULL_INST;
          cnt_d   = '0;
        end
      end

      P_FULL_INST: begin
        state_d   = P_RSP;
        rsp_st_d  = RSP_OK;
        rsp_st_en = 1'b1;
      end

      P_RSP: begin
        if (t_rsp_rdy) begin
          upd_en     = 1'b1;
          release_en = a_ent.prim;
          e.state    = E_FREE;
          e.prim     = 1'b0;
          state_d    = P_IDLE;
        end
      end

      default: state_d = P_IDLE;
    endcase

    upd_ent = e;
  end

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (r_fill_hdr && ent[r_idx].state != E_FILL)
      $error("%m: fill response for entry %0d, which is not waiting for a fill", r_idx);
    if (r_wb_hdr && !ent[r_idx].wb_pend)
      $error("%m: writeback response for entry %0d, which has no writeback outstanding", r_idx);
    if (r_take && !i_rsp_flit.head && !in_fill)
      $error("%m: unexpected memory response data flit");
    if (r_fill_word && fill_cnt >= WCNT_W'(WORDS))
      $error("%m: fill response longer than a line");
  end
`endif

endmodule
