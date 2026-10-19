// sh_l2_tb: self-checking unit test for sh_l2
//
// An agent model sends NUM_REQ random requests on t_req and checks each
// response on t_rsp against a reference memory (bytes, in request order):
//   - reads of 1..WORDS flits within a line: data must be the last value
//     written to each byte
//   - writes with random strobes, and whole-line writes (all strobes set)
//   - requests that cross a line: RSP_SLVERR, no effect
//   - requests to "bad" lines (every BAD_MOD-th line): the memory model fails
//     their fills, so they get RSP_SLVERR and have no effect
// Addresses fall in SPAN_LINES lines, so a small cache evicts constantly.
// The L2 serves requests in order, so responses are expected in order.
//
// A memory model on i_req/i_rsp holds the backing store. It checks that the
// L2 only sends whole-line fills and writebacks (all strobes set), answers
// with random delays, and fails WB_ERR_PCT % of writebacks (the data is still
// stored). wb_err is checked every cycle against a model of the sticky bit,
// with random wb_err_clr pulses.
//
// The arrays are sh_l2_tb_mem models with the configured read latencies.

module sh_l2_tb
  import ip_top_pkg::ADDR_W;
  import sh_noc_pkg::*;
  import sh_l2_pkg::*;
#(
  parameter int unsigned TAG_RD_DEL  = 0,
  parameter int unsigned DATA_RD_DEL = 0,
  parameter int unsigned LRU_RD_DEL  = 0,
  parameter int unsigned NUM_REQ     = 4000,
  parameter int unsigned SPAN_LINES  = 64,     // distinct lines addressed
  parameter int unsigned STRIDE      = 1,      // lines between addressed lines (NUM_SETS: all in one set)
  parameter int unsigned BAD_MOD     = 37,     // every BAD_MOD-th line fails fills; 0 = none
  parameter int unsigned FULL_PCT    = 10,     // whole-line writes
  parameter int unsigned CROSS_PCT   = 3,      // requests that cross a line
  parameter int unsigned WB_ERR_PCT  = 5,      // writebacks answered with an error
  parameter int unsigned TX_PCT      = 80,     // chance per cycle a flit is offered
  parameter int unsigned RX_PCT      = 80,     // chance per cycle a flit is accepted
  parameter int unsigned MEM_DLY     = 6       // max memory response delay, cycles
);

  localparam logic [ADDR_W-1:0] BASE = ADDR_W'(32'h4000_0000);

  // ---------------------------------------------------------------------------
  // Clock, reset, DUT
  // ---------------------------------------------------------------------------

  logic clk = 1'b0;
  logic rst_n = 1'b0;

  always #5ns clk = ~clk;

  logic                 t_req_vld, t_req_rdy;
  flit_t                t_req_flit;
  logic                 t_rsp_vld, t_rsp_rdy;
  flit_t                t_rsp_flit;
  logic                 i_req_vld, i_req_rdy;
  flit_t                i_req_flit;
  logic                 i_rsp_vld, i_rsp_rdy;
  flit_t                i_rsp_flit;
  logic                 tag_mem_wr_en, tag_mem_rd_en;
  logic [SET_W-1:0]     tag_mem_wr_addr, tag_mem_rd_addr;
  logic [TAG_ROW_W-1:0] tag_mem_wr_data, tag_mem_rd_data;
  logic [NUM_WAYS-1:0]  tag_mem_wr_mask;
  logic                 data_mem_wr_en, data_mem_rd_en;
  logic [DATA_AW-1:0]   data_mem_wr_addr, data_mem_rd_addr;
  logic [FLIT_W-1:0]    data_mem_wr_data, data_mem_rd_data;
  logic [STRB_W-1:0]    data_mem_wr_strb;
  logic                 lru_mem_wr_en, lru_mem_rd_en;
  logic [SET_W-1:0]     lru_mem_wr_addr, lru_mem_rd_addr;
  logic [LRU_W-1:0]     lru_mem_wr_data, lru_mem_rd_data;
  logic                 wb_err, wb_err_clr;

  sh_l2 #(
    .TAG_RD_DEL  (TAG_RD_DEL),
    .DATA_RD_DEL (DATA_RD_DEL),
    .LRU_RD_DEL  (LRU_RD_DEL)
  ) u_dut (
    .clk              (clk),
    .rst              (rst_n),
    .t_req_vld        (t_req_vld),
    .t_req_rdy        (t_req_rdy),
    .t_req_flit       (t_req_flit),
    .t_rsp_vld        (t_rsp_vld),
    .t_rsp_rdy        (t_rsp_rdy),
    .t_rsp_flit       (t_rsp_flit),
    .i_req_vld        (i_req_vld),
    .i_req_rdy        (i_req_rdy),
    .i_req_flit       (i_req_flit),
    .i_rsp_vld        (i_rsp_vld),
    .i_rsp_rdy        (i_rsp_rdy),
    .i_rsp_flit       (i_rsp_flit),
    .tag_mem_wr_en    (tag_mem_wr_en),
    .tag_mem_wr_addr  (tag_mem_wr_addr),
    .tag_mem_wr_data  (tag_mem_wr_data),
    .tag_mem_wr_mask  (tag_mem_wr_mask),
    .tag_mem_rd_en    (tag_mem_rd_en),
    .tag_mem_rd_addr  (tag_mem_rd_addr),
    .tag_mem_rd_data  (tag_mem_rd_data),
    .data_mem_wr_en   (data_mem_wr_en),
    .data_mem_wr_addr (data_mem_wr_addr),
    .data_mem_wr_data (data_mem_wr_data),
    .data_mem_wr_strb (data_mem_wr_strb),
    .data_mem_rd_en   (data_mem_rd_en),
    .data_mem_rd_addr (data_mem_rd_addr),
    .data_mem_rd_data (data_mem_rd_data),
    .lru_mem_wr_en    (lru_mem_wr_en),
    .lru_mem_wr_addr  (lru_mem_wr_addr),
    .lru_mem_wr_data  (lru_mem_wr_data),
    .lru_mem_rd_en    (lru_mem_rd_en),
    .lru_mem_rd_addr  (lru_mem_rd_addr),
    .lru_mem_rd_data  (lru_mem_rd_data),
    .wb_err           (wb_err),
    .wb_err_clr       (wb_err_clr)
  );

  sh_l2_tb_mem #(.WIDTH(TAG_ROW_W), .DEPTH(NUM_SETS), .MASK_W(NUM_WAYS), .RD_DEL(TAG_RD_DEL)) u_tag_mem (
    .clk (clk), .wr_en (tag_mem_wr_en), .wr_addr (tag_mem_wr_addr), .wr_data (tag_mem_wr_data),
    .wr_mask (tag_mem_wr_mask), .rd_en (tag_mem_rd_en), .rd_addr (tag_mem_rd_addr), .rd_data (tag_mem_rd_data)
  );
  sh_l2_tb_mem #(.WIDTH(FLIT_W), .DEPTH(NUM_SETS * NUM_WAYS * WORDS), .MASK_W(STRB_W), .RD_DEL(DATA_RD_DEL)) u_data_mem (
    .clk (clk), .wr_en (data_mem_wr_en), .wr_addr (data_mem_wr_addr), .wr_data (data_mem_wr_data),
    .wr_mask (data_mem_wr_strb), .rd_en (data_mem_rd_en), .rd_addr (data_mem_rd_addr), .rd_data (data_mem_rd_data)
  );
  sh_l2_tb_mem #(.WIDTH(LRU_W), .DEPTH(NUM_SETS), .MASK_W(1), .RD_DEL(LRU_RD_DEL)) u_lru_mem (
    .clk (clk), .wr_en (lru_mem_wr_en), .wr_addr (lru_mem_wr_addr), .wr_data (lru_mem_wr_data),
    .wr_mask (1'b1), .rd_en (lru_mem_rd_en), .rd_addr (lru_mem_rd_addr), .rd_data (lru_mem_rd_data)
  );

  // ---------------------------------------------------------------------------
  // Memory contents: reference (as agents see it) and backing store
  // ---------------------------------------------------------------------------

  logic [7:0] ref_mem  [logic [ADDR_W-1:0]];
  logic [7:0] back_mem [logic [ADDR_W-1:0]];

  // Value of a byte nobody has written
  function automatic logic [7:0] init_byte(logic [ADDR_W-1:0] a);
    return 8'(a ^ (a >> 8) ^ (a >> 16) ^ 8'h3C);
  endfunction

  function automatic logic [7:0] ref_byte(logic [ADDR_W-1:0] a);
    return ref_mem.exists(a) ? ref_mem[a] : init_byte(a);
  endfunction

  function automatic logic [7:0] back_byte(logic [ADDR_W-1:0] a);
    return back_mem.exists(a) ? back_mem[a] : init_byte(a);
  endfunction

  function automatic bit bad_line(logic [ADDR_W-1:0] a);
    return BAD_MOD != 0 && ((a >> OFF_W) % BAD_MOD) == BAD_MOD - 1;
  endfunction

  // ---------------------------------------------------------------------------
  // Agent: requests
  // ---------------------------------------------------------------------------

  typedef struct {
    pkt_type_e         typ;
    logic [TAG_W-1:0]  tag;
    logic [LEN_W-1:0]  len;
    logic [ADDR_W-1:0] addr;
    rsp_status_e       status;
    logic [FLIT_W-1:0] data [$];   // expected read data
  } exp_t;

  flit_t       req_q [$];    // flits waiting to be sent
  exp_t        exp_q [$];    // responses expected, in order
  int unsigned n_sent     = 0;
  int unsigned n_done     = 0;
  int unsigned n_err_rsp  = 0;
  int unsigned n_full     = 0;
  int unsigned errors     = 0;

  // Builds one random request, updates the reference, queues its flits
  task automatic make_request();
    hdr_t              h;
    exp_t              e;
    int unsigned       line, word, nwords;
    logic [STRB_W-1:0] strb;
    logic [FLIT_W-1:0] d;
    bit                full, crosses;

    line  = $urandom_range(SPAN_LINES - 1);
    full  = 0;
    crosses = 0;

    h        = '0;
    h.src_x  = X_W'($urandom);
    h.src_y  = Y_W'($urandom);
    h.typ    = $urandom_range(1) ? PKT_WR_REQ : PKT_RD_REQ;
    h.tag    = TAG_W'(n_sent);

    if ($urandom_range(99) < CROSS_PCT && WORDS > 1) begin
      crosses  = 1;
      word   = $urandom_range(WORDS - 1, 1);
      nwords = $urandom_range(MAX_LEN, WORDS - word + 1);
    end else if (h.typ == PKT_WR_REQ && $urandom_range(99) < FULL_PCT) begin
      full   = 1;
      word   = 0;
      nwords = WORDS;
      while (bad_line(BASE + ADDR_W'(line) * STRIDE * LINE_BYTES)) line = $urandom_range(SPAN_LINES - 1);
    end else begin
      word   = $urandom_range(WORDS - 1);
      nwords = $urandom_range(WORDS - word, 1);
    end

    h.addr = BASE + ADDR_W'(line) * STRIDE * LINE_BYTES + ADDR_W'(word) * STRB_W;
    h.len  = LEN_W'(nwords - 1);

    e.typ    = (h.typ == PKT_RD_REQ) ? PKT_RD_RSP : PKT_WR_RSP;
    e.tag    = h.tag;
    e.len    = h.len;
    e.addr   = h.addr;
    e.status = (crosses || bad_line(h.addr)) ? RSP_SLVERR : RSP_OK;
    if (full) n_full++;

    req_q.push_back('{head: 1'b1, tail: (h.typ == PKT_RD_REQ), strb: '0, data: hdr_to_data(h)});

    for (int i = 0; i < nwords; i++) begin
      logic [ADDR_W-1:0] wa;
      wa = h.addr + ADDR_W'(i) * STRB_W;
      if (h.typ == PKT_WR_REQ) begin
        d    = FLIT_W'({$urandom, $urandom, $urandom, $urandom});
        strb = full ? '1 : STRB_W'($urandom);
        req_q.push_back('{head: 1'b0, tail: (i == nwords - 1), strb: strb, data: d});
        if (e.status == RSP_OK) begin
          for (int b = 0; b < STRB_W; b++) if (strb[b]) ref_mem[wa + ADDR_W'(b)] = d[8*b +: 8];
        end
      end else if (e.status == RSP_OK) begin
        for (int b = 0; b < STRB_W; b++) d[8*b +: 8] = ref_byte(wa + ADDR_W'(b));
        e.data.push_back(d);
      end
    end

    exp_q.push_back(e);
    n_sent++;
  endtask

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      t_req_vld <= 1'b0;
    end else if (!t_req_vld || t_req_rdy) begin
      if (req_q.size() == 0 && n_sent < NUM_REQ) make_request();
      if (req_q.size() != 0 && $urandom_range(99) < TX_PCT) begin
        t_req_flit <= req_q.pop_front();
        t_req_vld  <= 1'b1;
      end else begin
        t_req_vld  <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Agent: responses
  // ---------------------------------------------------------------------------

  bit          rsp_in_pkt = 0;
  int unsigned rsp_beat;
  exp_t        cur;

  always @(posedge clk or negedge rst_n) begin
    hdr_t h;
    if (!rst_n) begin
      t_rsp_rdy <= 1'b0;
    end else begin
      if (t_rsp_vld && t_rsp_rdy) begin
        if (!rsp_in_pkt) begin
          h = get_hdr(t_rsp_flit);
          if (exp_q.size() == 0) begin
            $error("unexpected response, tag %0d", h.tag); errors++;
          end else begin
            cur = exp_q.pop_front();
            if (!t_rsp_flit.head || h.typ != cur.typ || h.tag != cur.tag || h.len != cur.len
                || h.status != cur.status) begin
              $error("response for addr %h: got typ %0d tag %0d len %0d status %0d, expected typ %0d tag %0d len %0d status %0d",
                     cur.addr, h.typ, h.tag, h.len, h.status, cur.typ, cur.tag, cur.len, cur.status);
              errors++;
            end
            if (t_rsp_flit.tail != (cur.data.size() == 0)) begin
              $error("response for addr %h: framing wrong", cur.addr); errors++;
            end
            if (cur.status != RSP_OK) n_err_rsp++;
            rsp_beat   = 0;
            rsp_in_pkt = !t_rsp_flit.tail;
            if (t_rsp_flit.tail) n_done++;
          end
        end else begin
          if (t_rsp_flit.head || rsp_beat >= cur.data.size()) begin
            $error("response for addr %h: extra or misframed data flit", cur.addr); errors++;
          end else if (t_rsp_flit.data != cur.data[rsp_beat]) begin
            $error("read addr %h beat %0d: got %h, expected %h", cur.addr, rsp_beat,
                   t_rsp_flit.data, cur.data[rsp_beat]);
            errors++;
          end
          rsp_beat++;
          if (t_rsp_flit.tail) begin
            if (rsp_beat != cur.data.size()) begin
              $error("read addr %h: %0d data flits, expected %0d", cur.addr, rsp_beat, cur.data.size());
              errors++;
            end
            rsp_in_pkt = 0;
            n_done++;
          end
        end
      end
      t_rsp_rdy <= ($urandom_range(99) < RX_PCT);
    end
  end

  // ---------------------------------------------------------------------------
  // Memory model
  // ---------------------------------------------------------------------------

  typedef struct {
    flit_t       flits [$];
    int unsigned delay;
    bit          wb_error;
  } mrsp_t;

  mrsp_t       mrsp_q [$];
  bit          mreq_in_pkt = 0;
  hdr_t        mreq_hdr;
  int unsigned mreq_beat;
  int unsigned n_fills = 0;
  int unsigned n_wbs   = 0;
  int unsigned n_wb_errs = 0;

  assign i_req_rdy = 1'b1;   // varied back-pressure comes from i_rsp timing

  always @(posedge clk) begin
    flit_t f;
    mrsp_t r;
    hdr_t  h;
    if (rst_n && i_req_vld && i_req_rdy) begin
      f = i_req_flit;
      if (!mreq_in_pkt) begin
        mreq_hdr  = get_hdr(f);
        mreq_beat = 0;
        if (!f.head || mreq_hdr.len != LEN_W'(WORDS - 1) || mreq_hdr.addr[OFF_W-1:0] != '0
            || !((mreq_hdr.typ == PKT_RD_REQ && mreq_hdr.tag == FILL_TAG && f.tail)
                 || (mreq_hdr.typ == PKT_WR_REQ && mreq_hdr.tag == WB_TAG && !f.tail))) begin
          $error("bad memory request: typ %0d tag %0d len %0d addr %h", mreq_hdr.typ, mreq_hdr.tag,
                 mreq_hdr.len, mreq_hdr.addr);
          errors++;
        end
        if (mreq_hdr.typ == PKT_RD_REQ) begin
          n_fills++;
          r.flits    = {};
          r.delay    = $urandom_range(MEM_DLY);
          r.wb_error = 0;
          h          = '0;
          h.typ      = PKT_RD_RSP;
          h.tag      = mreq_hdr.tag;
          h.len      = mreq_hdr.len;
          h.status   = bad_line(mreq_hdr.addr) ? RSP_SLVERR : RSP_OK;
          r.flits.push_back('{head: 1'b1, tail: (h.status != RSP_OK), strb: '0, data: hdr_to_data(h)});
          if (h.status == RSP_OK) begin
            for (int i = 0; i < WORDS; i++) begin
              logic [FLIT_W-1:0] d;
              for (int b = 0; b < STRB_W; b++)
                d[8*b +: 8] = back_byte(mreq_hdr.addr + ADDR_W'(i * STRB_W + b));
              r.flits.push_back('{head: 1'b0, tail: (i == WORDS - 1), strb: '0, data: d});
            end
          end
          mrsp_q.push_back(r);
        end
        mreq_in_pkt = !f.tail;
      end else begin
        if (f.head || f.strb != '1) begin
          $error("writeback addr %h beat %0d: bad flit (strobes %h)", mreq_hdr.addr, mreq_beat, f.strb);
          errors++;
        end
        if (bad_line(mreq_hdr.addr)) begin
          $error("writeback of a line that could never be filled: %h", mreq_hdr.addr); errors++;
        end
        for (int b = 0; b < STRB_W; b++)
          back_mem[mreq_hdr.addr + ADDR_W'(mreq_beat * STRB_W + b)] = f.data[8*b +: 8];
        mreq_beat++;
        if (f.tail) begin
          if (mreq_beat != WORDS) begin
            $error("writeback addr %h: %0d flits", mreq_hdr.addr, mreq_beat); errors++;
          end
          n_wbs++;
          r.flits    = {};
          r.delay    = $urandom_range(MEM_DLY);
          r.wb_error = ($urandom_range(99) < WB_ERR_PCT);
          h          = '0;
          h.typ      = PKT_WR_RSP;
          h.tag      = mreq_hdr.tag;
          h.len      = mreq_hdr.len;
          h.status   = r.wb_error ? RSP_SLVERR : RSP_OK;
          r.flits.push_back('{head: 1'b1, tail: 1'b1, strb: '0, data: hdr_to_data(h)});
          mrsp_q.push_back(r);
          mreq_in_pkt = 0;
        end
      end
    end
  end

  // Responses go out in order, each after its delay
  logic err_evt;     // an error writeback response is accepted this cycle

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      i_rsp_vld <= 1'b0;
    end else begin
      if (i_rsp_vld && i_rsp_rdy) begin
        i_rsp_vld <= 1'b0;
        if (i_rsp_flit.tail) void'(mrsp_q.pop_front());
      end
      if ((!i_rsp_vld || i_rsp_rdy) && mrsp_q.size() != 0) begin
        if (i_rsp_vld && i_rsp_rdy && i_rsp_flit.tail) begin
          // next packet starts next cycle
        end else if (mrsp_q[0].delay != 0) begin
          mrsp_q[0].delay--;
        end else begin
          i_rsp_flit <= mrsp_q[0].flits.pop_front();
          i_rsp_vld  <= 1'b1;
          if (mrsp_q[0].flits.size() == 0 && mrsp_q[0].wb_error) n_wb_errs++;
        end
      end
    end
  end

  assign err_evt = i_rsp_vld && i_rsp_rdy && i_rsp_flit.head
                   && get_hdr(i_rsp_flit).typ == PKT_WR_RSP && get_hdr(i_rsp_flit).status != RSP_OK;

  // ---------------------------------------------------------------------------
  // wb_err: model of the sticky bit, random clears
  // ---------------------------------------------------------------------------

  logic exp_wb_err = 1'b0;

  always @(posedge clk) begin
    if (rst_n) begin
      if (wb_err !== exp_wb_err) begin
        $error("wb_err is %0b, expected %0b", wb_err, exp_wb_err); errors++;
      end
      exp_wb_err <= err_evt || (exp_wb_err && !wb_err_clr);
      wb_err_clr <= ($urandom_range(99) < 2);
    end else begin
      wb_err_clr <= 1'b0;
    end
  end

  // ---------------------------------------------------------------------------
  // Test control
  // ---------------------------------------------------------------------------

  initial begin
    automatic longint cycles = 0;
    $display("sh_l2_tb: %0d bytes, %0d ways, %0d sets, %0d B lines, read latency tag/data/lru=%0d/%0d/%0d, %0d requests over %0d lines",
             L2_BYTES, NUM_WAYS, NUM_SETS, LINE_BYTES, TAG_RD_DEL, DATA_RD_DEL, LRU_RD_DEL,
             NUM_REQ, SPAN_LINES);
    #33ns rst_n = 1'b1;
    while (n_done < NUM_REQ && cycles < 64'(NUM_REQ) * 400 + 64'(NUM_SETS) + 10000) begin
      @(posedge clk);
      cycles++;
    end
    repeat (200) @(posedge clk);
    if (exp_q.size() != 0) begin
      $error("%0d responses never arrived", exp_q.size()); errors++;
    end
    if (errors == 0 && n_done == NUM_REQ)
      $display("PASS: %0d requests (%0d error responses, %0d whole-line writes), %0d fills, %0d writebacks (%0d failed), %0d cycles",
               n_done, n_err_rsp, n_full, n_fills, n_wbs, n_wb_errs, cycles);
    else
      $display("FAIL: %0d errors, %0d / %0d requests completed", errors, n_done, NUM_REQ);
    $finish;
  end

endmodule
