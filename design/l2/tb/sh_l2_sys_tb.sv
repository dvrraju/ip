// sh_l2_sys_tb: system test of sh_l2 in sh_noc_mesh
//
// The L2 sits at tile (L2_Y, L2_X). Every other tile T is an agent (initiator)
// and the memory that owns T's address region (target).
//
// Agents issue NUM_TXN random reads and writes each, with up to 2**TAG_W
// outstanding, to cacheable regions (through the L2), non-cacheable regions
// (straight to the owner) and unmapped addresses (DECERR at the source).
// Word ownership keeps the check race-free while still sharing lines: the
// 8-byte word at address a belongs to tile (a / 8) % NUM_TILES, and an agent
// only writes bytes of its own words. Several agents' partial writes
// therefore land in the same L2 lines. Each agent checks the bytes of its own
// words in every read against a reference memory updated in issue order;
// requests from one agent to one address take one path (to the L2 or to the
// owner) and stay in order.
//
// Memory owners check routing: requests to a cacheable region come only from
// the L2 (fills and writebacks), requests to a non-cacheable region never do.
// Response sources are checked too (L2, owner, or the agent itself for
// DECERR). A few cacheable requests cross a line and must get RSP_SLVERR.

module sh_l2_sys_tb
  import ip_top_pkg::ADDR_W;
  import sh_noc_pkg::*;
  import sh_l2_pkg::*;
#(
  parameter int unsigned BUF_DEPTH    = 4,
  parameter logic        ASYNC        = 1'b1,
  parameter int unsigned CDC_DEPTH    = 4,
  parameter int unsigned L2_RD_DEL    = 0,    // read latency of all L2 memories
  parameter int unsigned NUM_TXN      = 150,  // transactions per agent
  parameter int unsigned SPAN_LINES   = 16,   // lines used at the start of each region
  parameter int unsigned UNMAPPED_PCT = 3,
  parameter int unsigned CROSS_PCT    = 3,    // cacheable requests that cross a line
  parameter int unsigned TX_PCT       = 80,
  parameter int unsigned RX_PCT       = 80,
  parameter int unsigned MEM_DLY      = 4
);

  localparam int unsigned NT    = NUM_TILES;
  localparam int unsigned NTAGS = 1 << TAG_W;
  localparam int unsigned BAW   = (BUF_DEPTH > 1) ? $clog2(BUF_DEPTH) : 1;
  localparam int unsigned CAW   = (CDC_DEPTH > 1) ? $clog2(CDC_DEPTH) : 1;

  initial begin
    if (!L2_PRESENT) $fatal(1, "sh_l2_sys_tb: needs SH_NOC_L2_PRESENT");
    if (NT > 256)    $fatal(1, "sh_l2_sys_tb: at most 256 tiles");
  end

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  function automatic int unsigned tile_id(logic [X_W-1:0] x, logic [Y_W-1:0] y);
    return 32'(y) * COLS + 32'(x);
  endfunction

  function automatic int unsigned word_owner(logic [ADDR_W-1:0] a);
    return 32'((a / STRB_W) % NT);
  endfunction

  logic [7:0] ref_mem  [logic [ADDR_W-1:0]];   // as agents see it
  logic [7:0] back_mem [logic [ADDR_W-1:0]];   // memory owners' contents

  function automatic logic [7:0] init_byte(logic [ADDR_W-1:0] a);
    return 8'(a ^ (a >> 8) ^ (a >> 16) ^ (a >> 24) ^ 8'h96);
  endfunction

  function automatic logic [7:0] ref_byte(logic [ADDR_W-1:0] a);
    return ref_mem.exists(a) ? ref_mem[a] : init_byte(a);
  endfunction

  function automatic logic [7:0] back_byte(logic [ADDR_W-1:0] a);
    return back_mem.exists(a) ? back_mem[a] : init_byte(a);
  endfunction

  function automatic logic [ADDR_W-1:0] unmapped_addr();
    logic [ADDR_W-1:0] a;
    for (int k = 0; k < 10000; k++) begin
      a = ADDR_W'({$urandom, $urandom}) & ~ADDR_W'(STRB_W - 1);
      if (!addr_decode(a).hit) return a;
    end
    $fatal(1, "sh_l2_sys_tb: no unmapped address found");
    return '0;
  endfunction

  // ---------------------------------------------------------------------------
  // Clocks and reset
  // ---------------------------------------------------------------------------

  logic clk = 1'b0;
  logic rst_n = 1'b0;

  always #5ns clk = ~clk;

  logic  [ROWS-1:0][COLS-1:0] tile_clk;
  logic  [ROWS-1:0][COLS-1:0] tile_rst;
  logic  [ROWS-1:0][COLS-1:0] i_req_vld, i_req_rdy, i_rsp_vld, i_rsp_rdy;
  logic  [ROWS-1:0][COLS-1:0] t_req_vld, t_req_rdy, t_rsp_vld, t_rsp_rdy;
  flit_t [ROWS-1:0][COLS-1:0] i_req_flit, i_rsp_flit, t_req_flit, t_rsp_flit;

  logic [ROWS-1:0][COLS-1:0][1:0][5:0]                  buf_mem_wr_en, buf_mem_rd_en;
  logic [ROWS-1:0][COLS-1:0][1:0][5:0][BAW-1:0]         buf_mem_wr_addr, buf_mem_rd_addr;
  logic [ROWS-1:0][COLS-1:0][1:0][5:0][FLIT_LINK_W-1:0] buf_mem_wr_data, buf_mem_rd_data;
  logic [ROWS-1:0][COLS-1:0][3:0]                       cdc_mem_wr_en, cdc_mem_rd_en;
  logic [ROWS-1:0][COLS-1:0][3:0][CAW-1:0]              cdc_mem_wr_addr, cdc_mem_rd_addr;
  logic [ROWS-1:0][COLS-1:0][3:0][FLIT_LINK_W-1:0]      cdc_mem_wr_data, cdc_mem_rd_data;

  sh_noc_mesh #(
    .BUF_DEPTH (BUF_DEPTH),
    .ASYNC     (ASYNC),
    .CDC_DEPTH (CDC_DEPTH)
  ) u_mesh (
    .clk             (clk),
    .rst             (rst_n),
    .tile_clk        (tile_clk),
    .tile_rst        (tile_rst),
    .i_req_vld       (i_req_vld),
    .i_req_rdy       (i_req_rdy),
    .i_req_flit      (i_req_flit),
    .i_rsp_vld       (i_rsp_vld),
    .i_rsp_rdy       (i_rsp_rdy),
    .i_rsp_flit      (i_rsp_flit),
    .t_req_vld       (t_req_vld),
    .t_req_rdy       (t_req_rdy),
    .t_req_flit      (t_req_flit),
    .t_rsp_vld       (t_rsp_vld),
    .t_rsp_rdy       (t_rsp_rdy),
    .t_rsp_flit      (t_rsp_flit),
    .buf_mem_wr_en   (buf_mem_wr_en),
    .buf_mem_wr_addr (buf_mem_wr_addr),
    .buf_mem_wr_data (buf_mem_wr_data),
    .buf_mem_rd_en   (buf_mem_rd_en),
    .buf_mem_rd_addr (buf_mem_rd_addr),
    .buf_mem_rd_data (buf_mem_rd_data),
    .cdc_mem_wr_en   (cdc_mem_wr_en),
    .cdc_mem_wr_addr (cdc_mem_wr_addr),
    .cdc_mem_wr_data (cdc_mem_wr_data),
    .cdc_mem_rd_en   (cdc_mem_rd_en),
    .cdc_mem_rd_addr (cdc_mem_rd_addr),
    .cdc_mem_rd_data (cdc_mem_rd_data)
  );

  // ---------------------------------------------------------------------------
  // Scoreboard totals
  // ---------------------------------------------------------------------------

  int unsigned completed = 0;
  int unsigned n_cached  = 0;
  int unsigned n_direct  = 0;
  int unsigned n_decerr  = 0;
  int unsigned n_slverr  = 0;
  int unsigned n_l2_reqs = 0;   // fills and writebacks seen by memory owners
  int unsigned errors    = 0;

  // ---------------------------------------------------------------------------
  // Tiles
  // ---------------------------------------------------------------------------

  for (genvar y = 0; y < ROWS; y++) begin : g_row
    for (genvar x = 0; x < COLS; x++) begin : g_col
      localparam int unsigned T     = y * COLS + x;
      localparam bit          IS_L2 = (T == L2_TILE);
      localparam realtime     HALF  = (3.1 + 0.7 * ((T * 7) % 13)) * 1ns;

      logic tclk = 1'b0;
      logic tsc;

      always #(HALF) tclk = ~tclk;
      assign tile_clk[y][x] = tclk;
      assign tile_rst[y][x] = rst_n;
      assign tsc            = ASYNC ? tclk : clk;

      // NoC memories
      for (genvar n = 0; n < 2; n++) begin : g_net
        for (genvar p = 0; p < 6; p++) begin : g_buf
          sh_noc_tb_mem #(.WIDTH(FLIT_LINK_W), .DEPTH(BUF_DEPTH)) u_mem (
            .wr_clk  (clk), .wr_en (buf_mem_wr_en[y][x][n][p]), .wr_addr (buf_mem_wr_addr[y][x][n][p]),
            .wr_data (buf_mem_wr_data[y][x][n][p]), .rd_clk (clk), .rd_en (buf_mem_rd_en[y][x][n][p]),
            .rd_addr (buf_mem_rd_addr[y][x][n][p]), .rd_data (buf_mem_rd_data[y][x][n][p])
          );
        end
      end
      for (genvar f = 0; f < 4; f++) begin : g_cdc
        localparam bit FROM_TILE = (f == 0 || f == 3);
        sh_noc_tb_mem #(.WIDTH(FLIT_LINK_W), .DEPTH(CDC_DEPTH)) u_mem (
          .wr_clk  (FROM_TILE ? tsc : clk), .wr_en (cdc_mem_wr_en[y][x][f]), .wr_addr (cdc_mem_wr_addr[y][x][f]),
          .wr_data (cdc_mem_wr_data[y][x][f]), .rd_clk (FROM_TILE ? clk : tsc), .rd_en (cdc_mem_rd_en[y][x][f]),
          .rd_addr (cdc_mem_rd_addr[y][x][f]), .rd_data (cdc_mem_rd_data[y][x][f])
        );
      end

      if (IS_L2) begin : g_l2
        // ---------------------------------------------------------------------
        // The L2 and its arrays
        // ---------------------------------------------------------------------
        logic                 tag_wr_en, tag_rd_en, data_wr_en, data_rd_en, lru_wr_en, lru_rd_en;
        logic [SET_W-1:0]     tag_wr_addr, tag_rd_addr, lru_wr_addr, lru_rd_addr;
        logic [TAG_ROW_W-1:0] tag_wr_data, tag_rd_data;
        logic [NUM_WAYS-1:0]  tag_wr_mask;
        logic [DATA_AW-1:0]   data_wr_addr, data_rd_addr;
        logic [FLIT_W-1:0]    data_wr_data, data_rd_data;
        logic [STRB_W-1:0]    data_wr_strb;
        logic [LRU_W-1:0]     lru_wr_data, lru_rd_data;
        logic                   pf_wr_en, pf_rd_en;
        logic [PF_IW-1:0]       pf_wr_addr, pf_rd_addr;
        logic [PF_ENTRY_W-1:0] pf_wr_data, pf_rd_data;
        logic [PF_SLOTS-1:0]    pf_wr_mask;
        logic                 wb_err;
        logic                 rq_rdy, rp_vld, iq_vld, ip_rdy;
        flit_t                rp_flit, iq_flit;

        assign t_req_rdy[y][x]  = rq_rdy;
        assign t_rsp_vld[y][x]  = rp_vld;
        assign t_rsp_flit[y][x] = rp_flit;
        assign i_req_vld[y][x]  = iq_vld;
        assign i_req_flit[y][x] = iq_flit;
        assign i_rsp_rdy[y][x]  = ip_rdy;

        sh_l2 #(
          .TAG_RD_DEL  (L2_RD_DEL),
          .DATA_RD_DEL (L2_RD_DEL),
          .LRU_RD_DEL  (L2_RD_DEL),
          .PF_RD_DEL   (L2_RD_DEL)
        ) u_l2 (
          .clk              (tsc),
          .rst              (rst_n),
          .t_req_vld        (t_req_vld[y][x]),
          .t_req_rdy        (rq_rdy),
          .t_req_flit       (t_req_flit[y][x]),
          .t_rsp_vld        (rp_vld),
          .t_rsp_rdy        (t_rsp_rdy[y][x]),
          .t_rsp_flit       (rp_flit),
          .i_req_vld        (iq_vld),
          .i_req_rdy        (i_req_rdy[y][x]),
          .i_req_flit       (iq_flit),
          .i_rsp_vld        (i_rsp_vld[y][x]),
          .i_rsp_rdy        (ip_rdy),
          .i_rsp_flit       (i_rsp_flit[y][x]),
          .tag_mem_wr_en    (tag_wr_en),
          .tag_mem_wr_addr  (tag_wr_addr),
          .tag_mem_wr_data  (tag_wr_data),
          .tag_mem_wr_mask  (tag_wr_mask),
          .tag_mem_rd_en    (tag_rd_en),
          .tag_mem_rd_addr  (tag_rd_addr),
          .tag_mem_rd_data  (tag_rd_data),
          .data_mem_wr_en   (data_wr_en),
          .data_mem_wr_addr (data_wr_addr),
          .data_mem_wr_data (data_wr_data),
          .data_mem_wr_strb (data_wr_strb),
          .data_mem_rd_en   (data_rd_en),
          .data_mem_rd_addr (data_rd_addr),
          .data_mem_rd_data (data_rd_data),
          .lru_mem_wr_en    (lru_wr_en),
          .lru_mem_wr_addr  (lru_wr_addr),
          .lru_mem_wr_data  (lru_wr_data),
          .lru_mem_rd_en    (lru_rd_en),
          .lru_mem_rd_addr  (lru_rd_addr),
          .lru_mem_rd_data  (lru_rd_data),
          .pf_mem_wr_en     (pf_wr_en),
          .pf_mem_wr_addr   (pf_wr_addr),
          .pf_mem_wr_data   (pf_wr_data),
          .pf_mem_wr_mask   (pf_wr_mask),
          .pf_mem_rd_en     (pf_rd_en),
          .pf_mem_rd_addr   (pf_rd_addr),
          .pf_mem_rd_data   (pf_rd_data),
          .wb_err           (wb_err),
          .wb_err_clr       (1'b0)
        );

        sh_l2_tb_mem #(.WIDTH(TAG_ROW_W), .DEPTH(NUM_SETS), .MASK_W(NUM_WAYS), .RD_DEL(L2_RD_DEL)) u_tag (
          .clk   (tsc), .wr_en (tag_wr_en), .wr_addr (tag_wr_addr), .wr_data (tag_wr_data), .wr_mask (tag_wr_mask),
          .rd_en (tag_rd_en), .rd_addr (tag_rd_addr), .rd_data (tag_rd_data)
        );
        sh_l2_tb_mem #(.WIDTH(FLIT_W), .DEPTH(NUM_SETS * NUM_WAYS * WORDS), .MASK_W(STRB_W), .RD_DEL(L2_RD_DEL)) u_data (
          .clk   (tsc), .wr_en (data_wr_en), .wr_addr (data_wr_addr), .wr_data (data_wr_data), .wr_mask (data_wr_strb),
          .rd_en (data_rd_en), .rd_addr (data_rd_addr), .rd_data (data_rd_data)
        );
        sh_l2_tb_mem #(.WIDTH(LRU_W), .DEPTH(NUM_SETS), .MASK_W(1), .RD_DEL(L2_RD_DEL)) u_lru (
          .clk   (tsc), .wr_en (lru_wr_en), .wr_addr (lru_wr_addr), .wr_data (lru_wr_data), .wr_mask (1'b1),
          .rd_en (lru_rd_en), .rd_addr (lru_rd_addr), .rd_data (lru_rd_data)
        );
        sh_l2_tb_mem #(.WIDTH(PF_ENTRY_W), .DEPTH(PF_DEPTH), .MASK_W(PF_SLOTS), .RD_DEL(L2_RD_DEL)) u_pf (
          .clk   (tsc), .wr_en (pf_wr_en), .wr_addr (pf_wr_addr), .wr_data (pf_wr_data), .wr_mask (pf_wr_mask),
          .rd_en (pf_rd_en), .rd_addr (pf_rd_addr), .rd_data (pf_rd_data)
        );

        always @(posedge tsc) begin
          if (rst_n && wb_err) begin
            $error("L2 reported a writeback error"); errors++;
          end
        end

      end else begin : g_agent
        // ---------------------------------------------------------------------
        // Agent (initiator)
        // ---------------------------------------------------------------------
        typedef struct {
          bit                busy;
          pkt_type_e         typ;
          logic [LEN_W-1:0]  len;
          logic [ADDR_W-1:0] addr;
          rsp_status_e       status;
          int unsigned       src;        // expected response source tile
          logic [FLIT_W-1:0] data [$];   // expected read data (own words only)
        } txn_t;

        txn_t        txn [NTAGS];
        flit_t       ireq_q [$];
        int unsigned n_issued = 0;
        logic        iq_vld;
        flit_t       iq_flit;
        logic        ip_rdy;

        assign i_req_vld[y][x]  = iq_vld;
        assign i_req_flit[y][x] = iq_flit;
        assign i_rsp_rdy[y][x]  = ip_rdy;

        always @(posedge tsc or negedge rst_n) begin
          hdr_t              h;
          addr_dec_t         dec;
          int                tag, s;
          int unsigned       r, word, nwords;
          logic [ADDR_W-1:0] wa;
          logic [STRB_W-1:0] st;
          logic [FLIT_W-1:0] d;
          bit                crosses;
          if (!rst_n) begin
            iq_vld <= 1'b0;
            for (int t = 0; t < NTAGS; t++) txn[t].busy = 1'b0;
          end else if (!iq_vld || i_req_rdy[y][x]) begin
            if (ireq_q.size() == 0 && n_issued < NUM_TXN) begin
              tag = -1;
              s   = $urandom_range(NTAGS - 1);
              for (int k = 0; k < NTAGS; k++) if (tag < 0 && !txn[(s + k) % NTAGS].busy) tag = (s + k) % NTAGS;
              if (tag >= 0) begin
                h       = hdr_t'({$urandom, $urandom});   // junk in dst/src
                h.typ   = $urandom_range(1) ? PKT_WR_REQ : PKT_RD_REQ;
                h.tag   = TAG_W'(tag);
                h.status = RSP_OK;
                crosses = 0;
                if ($urandom_range(99) < UNMAPPED_PCT) begin
                  h.addr = unmapped_addr();
                  nwords = $urandom_range(MAX_LEN, 1);
                end else begin
                  r       = $urandom_range(NUM_REGIONS - 1);
                  word    = $urandom_range(WORDS - 1);
                  crosses = ($urandom_range(99) < CROSS_PCT) && word > 0;
                  nwords  = crosses ? $urandom_range(MAX_LEN, WORDS - word + 1)
                                    : $urandom_range(WORDS - word, 1);
                  h.addr  = ADDR_MAP[r].base + ADDR_W'($urandom_range(SPAN_LINES - 1)) * LINE_BYTES
                            + ADDR_W'(word) * STRB_W;
                end
                h.len = LEN_W'(nwords - 1);
                dec   = addr_decode(h.addr);

                txn[tag].busy = 1'b1;
                txn[tag].typ  = (h.typ == PKT_RD_REQ) ? PKT_RD_RSP : PKT_WR_RSP;
                txn[tag].len  = h.len;
                txn[tag].addr = h.addr;
                txn[tag].data = {};
                if (!dec.hit) begin
                  txn[tag].status = RSP_DECERR;
                  txn[tag].src    = T;
                end else if (dec.cacheable) begin
                  txn[tag].status = crosses ? RSP_SLVERR : RSP_OK;
                  txn[tag].src    = L2_TILE;
                end else begin
                  txn[tag].status = RSP_OK;
                  txn[tag].src    = tile_id(dec.x, dec.y);
                end
                n_issued++;

                ireq_q.push_back('{head: 1'b1, tail: (h.typ == PKT_RD_REQ), strb: '0, data: hdr_to_data(h)});
                for (int i = 0; i < nwords; i++) begin
                  wa = h.addr + ADDR_W'(i) * STRB_W;
                  if (h.typ == PKT_WR_REQ) begin
                    d  = FLIT_W'({$urandom, $urandom, $urandom, $urandom});
                    st = (word_owner(wa) == T) ? STRB_W'($urandom) : '0;   // only own words
                    ireq_q.push_back('{head: 1'b0, tail: (i == nwords - 1), strb: st, data: d});
                    if (txn[tag].status == RSP_OK)
                      for (int b = 0; b < STRB_W; b++) if (st[b]) ref_mem[wa + ADDR_W'(b)] = d[8*b +: 8];
                  end else if (txn[tag].status == RSP_OK) begin
                    for (int b = 0; b < STRB_W; b++) d[8*b +: 8] = ref_byte(wa + ADDR_W'(b));
                    txn[tag].data.push_back(d);
                  end
                end
              end
            end
            if (ireq_q.size() != 0 && $urandom_range(99) < TX_PCT) begin
              iq_flit <= ireq_q.pop_front();
              iq_vld  <= 1'b1;
            end else begin
              iq_vld  <= 1'b0;
            end
          end
        end

        // Response checker
        bit          in_pkt = 0;
        int unsigned rtag, rbeat;

        always @(posedge tsc or negedge rst_n) begin
          hdr_t  h;
          flit_t f;
          if (!rst_n) begin
            ip_rdy <= 1'b0;
          end else begin
            if (i_rsp_vld[y][x] && ip_rdy) begin
              f = i_rsp_flit[y][x];
              if (!in_pkt) begin
                h    = get_hdr(f);
                rtag = 32'(h.tag);
                if (!f.head || !txn[rtag].busy) begin
                  $error("tile %0d: unexpected response (tag %0d)", T, rtag); errors++;
                end else if (h.typ != txn[rtag].typ || h.len != txn[rtag].len || h.status != txn[rtag].status
                             || tile_id(h.src_x, h.src_y) != txn[rtag].src) begin
                  $error("tile %0d tag %0d addr %h: got typ %0d len %0d status %0d src %0d, expected typ %0d len %0d status %0d src %0d",
                         T, rtag, txn[rtag].addr, h.typ, h.len, h.status, tile_id(h.src_x, h.src_y),
                         txn[rtag].typ, txn[rtag].len, txn[rtag].status, txn[rtag].src);
                  errors++;
                end else if (f.tail != (txn[rtag].data.size() == 0)) begin
                  $error("tile %0d tag %0d: response framing wrong", T, rtag); errors++;
                end
                rbeat  = 0;
                in_pkt = !f.tail;
              end else begin
                if (f.head || rbeat >= txn[rtag].data.size()) begin
                  $error("tile %0d tag %0d: extra or misframed data flit", T, rtag); errors++;
                end else if (word_owner(txn[rtag].addr + ADDR_W'(rbeat) * STRB_W) == T
                             && f.data != txn[rtag].data[rbeat]) begin
                  $error("tile %0d: read %h beat %0d got %h, expected %h", T, txn[rtag].addr, rbeat,
                         f.data, txn[rtag].data[rbeat]);
                  errors++;
                end
                rbeat++;
                in_pkt = !f.tail;
              end
              if (f.tail) begin
                case (txn[rtag].status)
                  RSP_DECERR: n_decerr++;
                  RSP_SLVERR: n_slverr++;
                  default:    if (txn[rtag].src == L2_TILE) n_cached++; else n_direct++;
                endcase
                txn[rtag].busy = 1'b0;
                completed++;
              end
            end
            ip_rdy <= ($urandom_range(99) < RX_PCT);
          end
        end

        // ---------------------------------------------------------------------
        // Memory owner (target)
        // ---------------------------------------------------------------------
        typedef struct {
          flit_t       flits [$];
          int unsigned delay;
        } mrsp_t;

        mrsp_t       mrsp_q [$];
        bit          mq_in_pkt = 0;
        hdr_t        mq_hdr;
        int unsigned mq_beat;
        logic        rq_rdy, rp_vld;
        flit_t       rp_flit;

        assign t_req_rdy[y][x]  = rq_rdy;
        assign t_rsp_vld[y][x]  = rp_vld;
        assign t_rsp_flit[y][x] = rp_flit;

        always @(posedge tsc or negedge rst_n) begin
          flit_t             f;
          hdr_t              r;
          mrsp_t             m;
          addr_dec_t         dec;
          logic [ADDR_W-1:0] wa;
          logic [FLIT_W-1:0] d;
          int unsigned       src;
          if (!rst_n) begin
            rq_rdy <= 1'b0;
          end else begin
            if (t_req_vld[y][x] && rq_rdy) begin
              f = t_req_flit[y][x];
              if (!mq_in_pkt) begin
                mq_hdr  = get_hdr(f);
                mq_beat = 0;
                dec     = addr_decode(mq_hdr.addr);
                src     = tile_id(mq_hdr.src_x, mq_hdr.src_y);
                if (!f.head || !dec.hit || tile_id(dec.x, dec.y) != T) begin
                  $error("tile %0d: request for %h delivered here", T, mq_hdr.addr); errors++;
                end else if (dec.cacheable != (src == L2_TILE)) begin
                  $error("tile %0d: %0s request for %h came from tile %0d", T,
                         dec.cacheable ? "cacheable" : "non-cacheable", mq_hdr.addr, src);
                  errors++;
                end
                if (src == L2_TILE) n_l2_reqs++;
              end else begin
                wa = mq_hdr.addr + ADDR_W'(mq_beat) * STRB_W;
                for (int b = 0; b < STRB_W; b++) if (f.strb[b]) back_mem[wa + ADDR_W'(b)] = f.data[8*b +: 8];
                mq_beat++;
              end
              mq_in_pkt = !f.tail;
              if (f.tail) begin
                r        = '0;
                r.dst_x  = mq_hdr.src_x;
                r.dst_y  = mq_hdr.src_y;
                r.typ    = (mq_hdr.typ == PKT_RD_REQ) ? PKT_RD_RSP : PKT_WR_RSP;
                r.tag    = mq_hdr.tag;
                r.len    = mq_hdr.len;
                r.status = RSP_OK;
                m.flits  = {};
                m.delay  = $urandom_range(MEM_DLY);
                m.flits.push_back('{head: 1'b1, tail: (r.typ == PKT_WR_RSP), strb: '0, data: hdr_to_data(r)});
                if (r.typ == PKT_RD_RSP) begin
                  for (int i = 0; i <= 32'(r.len); i++) begin
                    wa = mq_hdr.addr + ADDR_W'(i) * STRB_W;
                    for (int b = 0; b < STRB_W; b++) d[8*b +: 8] = back_byte(wa + ADDR_W'(b));
                    m.flits.push_back('{head: 1'b0, tail: (i == 32'(r.len)), strb: '0, data: d});
                  end
                end
                mrsp_q.push_back(m);
              end
            end
            rq_rdy <= ($urandom_range(99) < RX_PCT);
          end
        end

        always @(posedge tsc or negedge rst_n) begin
          if (!rst_n) begin
            rp_vld <= 1'b0;
          end else if (!rp_vld || t_rsp_rdy[y][x]) begin
            rp_vld <= 1'b0;
            if (mrsp_q.size() != 0) begin
              if (mrsp_q[0].delay != 0) begin
                mrsp_q[0].delay--;
              end else begin
                rp_flit <= mrsp_q[0].flits.pop_front();
                rp_vld  <= 1'b1;
                if (mrsp_q[0].flits.size() == 0) void'(mrsp_q.pop_front());
              end
            end
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Test control
  // ---------------------------------------------------------------------------

  initial begin
    automatic int unsigned expected = (NT - 1) * NUM_TXN;
    automatic longint      cycles   = 0;
    $display("sh_l2_sys_tb: %0dx%0d mesh, L2 at tile %0d (%0d bytes, %0d ways, read latency %0d), ASYNC=%0d, %0d transactions",
             ROWS, COLS, L2_TILE, L2_BYTES, NUM_WAYS, L2_RD_DEL, ASYNC, expected);
    #33ns rst_n = 1'b1;
    while (completed < expected && cycles < 64'(expected) * 2000 + 64'(NUM_SETS) * 4) begin
      @(posedge clk);
      cycles++;
    end
    repeat (500) @(posedge clk);
    if (errors == 0 && completed == expected)
      $display("PASS: %0d transactions (%0d through the L2, %0d direct, %0d DECERR, %0d SLVERR), %0d L2 fills/writebacks, %0d NoC cycles",
               completed, n_cached, n_direct, n_decerr, n_slverr, n_l2_reqs, cycles);
    else
      $display("FAIL: %0d errors, %0d / %0d transactions completed", errors, completed, expected);
    $finish;
  end

endmodule
