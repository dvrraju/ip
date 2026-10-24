// sh_l2_tb: self-checking unit test for sh_l2
//
// An agent model sends NUM_REQ random requests on t_req, with up to
// 2**TAG_W outstanding (unique tags), and checks each response on t_rsp by
// tag, in whatever order they come back:
//   - reads of 1..WORDS flits within a line: data must be the last value
//     written to each byte
//   - writes with random strobes, and whole-line writes (all strobes set)
//   - requests that cross a line: RSP_SLVERR, no effect
//   - requests to "bad" lines (every BAD_MOD-th line): the memory model fails
//     their fills, so they get RSP_SLVERR and have no effect
// Addresses fall in SPAN_LINES lines (STRIDE lines apart), so lines are
// shared by many outstanding requests and a small cache evicts constantly.
// The reference memory is updated in request order: the L2 completes
// requests to one line in arrival order, so this is exact even though
// responses to different lines come back out of order.
//
// A memory model on i_req/i_rsp holds the backing store and answers fills
// and writebacks out of order, after random delays. Writeback data is
// applied, and fill data read, only when the response is sent, so a fill
// that overtakes its line's writeback would return stale data; the model
// also flags such a fill directly. It checks that the L2 only sends
// whole-line fills and writebacks with distinct tags, and fails WB_ERR_PCT %
// of writebacks (the data is still stored). wb_err is checked every cycle
// against a model of the sticky bit, with random wb_err_clr pulses.
//
// The arrays and the pending FIFO are sh_l2_tb_mem models with the
// configured read latencies.

module sh_l2_tb
  import ip_top_pkg::ADDR_W;
  import sh_noc_pkg::*;
  import sh_l2_pkg::*;
#(
  parameter int unsigned TAG_RD_DEL  = 0,
  parameter int unsigned DATA_RD_DEL = 0,
  parameter int unsigned LRU_RD_DEL  = 0,
  parameter int unsigned PF_RD_DEL   = 0,
  parameter int unsigned NUM_REQ     = 4000,
  parameter int unsigned SPAN_LINES  = 64,     // distinct lines addressed
  parameter int unsigned STRIDE      = 1,      // lines between addressed lines (NUM_SETS: all in one set)
  parameter int unsigned BAD_MOD     = 37,     // every BAD_MOD-th line fails fills; 0 = none
  parameter int unsigned FULL_PCT    = 10,     // whole-line writes
  parameter int unsigned CROSS_PCT   = 3,      // requests that cross a line
  parameter int unsigned WB_ERR_PCT  = 5,      // writebacks answered with an error
  parameter int unsigned TX_PCT      = 80,     // chance per cycle a flit is offered
  parameter int unsigned RX_PCT      = 80,     // chance per cycle a flit is accepted
  parameter int unsigned MEM_DLY     = 30,     // max memory response delay, cycles
  parameter int unsigned MAX_OUT     = 1 << TAG_W   // agent's outstanding request limit
);

  localparam logic [ADDR_W-1:0] BASE  = ADDR_W'(32'h4000_0000);
  localparam int unsigned       NTAGS = 1 << TAG_W;

  // ---------------------------------------------------------------------------
  // Clock, reset, DUT
  // ---------------------------------------------------------------------------

  logic clk = 1'b0;
  logic rst_n = 1'b0;

  always #5ns clk = ~clk;

  logic                   t_req_vld, t_req_rdy;
  flit_t                  t_req_flit;
  logic                   t_rsp_vld, t_rsp_rdy;
  flit_t                  t_rsp_flit;
  logic                   i_req_vld, i_req_rdy;
  flit_t                  i_req_flit;
  logic                   i_rsp_vld, i_rsp_rdy;
  flit_t                  i_rsp_flit;
  logic                   tag_mem_wr_en, tag_mem_rd_en;
  logic [SET_W-1:0]       tag_mem_wr_addr, tag_mem_rd_addr;
  logic [TAG_ROW_W-1:0]   tag_mem_wr_data, tag_mem_rd_data;
  logic [NUM_WAYS-1:0]    tag_mem_wr_mask;
  logic                   data_mem_wr_en, data_mem_rd_en;
  logic [DATA_AW-1:0]     data_mem_wr_addr, data_mem_rd_addr;
  logic [FLIT_W-1:0]      data_mem_wr_data, data_mem_rd_data;
  logic [STRB_W-1:0]      data_mem_wr_strb;
  logic                   lru_mem_wr_en, lru_mem_rd_en;
  logic [SET_W-1:0]       lru_mem_wr_addr, lru_mem_rd_addr;
  logic [LRU_W-1:0]       lru_mem_wr_data, lru_mem_rd_data;
  logic                   pf_mem_wr_en, pf_mem_rd_en;
  logic [PF_IW-1:0]       pf_mem_wr_addr, pf_mem_rd_addr;
  logic [PF_ENTRY_W-1:0] pf_mem_wr_data, pf_mem_rd_data;
  logic [PF_SLOTS-1:0]    pf_mem_wr_mask;
  logic                   wb_err, wb_err_clr;

  sh_l2 #(
    .TAG_RD_DEL  (TAG_RD_DEL),
    .DATA_RD_DEL (DATA_RD_DEL),
    .LRU_RD_DEL  (LRU_RD_DEL),
    .PF_RD_DEL   (PF_RD_DEL)
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
    .pf_mem_wr_en     (pf_mem_wr_en),
    .pf_mem_wr_addr   (pf_mem_wr_addr),
    .pf_mem_wr_data   (pf_mem_wr_data),
    .pf_mem_wr_mask   (pf_mem_wr_mask),
    .pf_mem_rd_en     (pf_mem_rd_en),
    .pf_mem_rd_addr   (pf_mem_rd_addr),
    .pf_mem_rd_data   (pf_mem_rd_data),
    .wb_err           (wb_err),
    .wb_err_clr       (wb_err_clr)
  );

  sh_l2_tb_mem #(.WIDTH(TAG_ROW_W), .DEPTH(NUM_SETS), .MASK_W(NUM_WAYS), .RD_DEL(TAG_RD_DEL)) u_tag_mem (
    .clk     (clk), .wr_en (tag_mem_wr_en), .wr_addr (tag_mem_wr_addr), .wr_data (tag_mem_wr_data),
    .wr_mask (tag_mem_wr_mask), .rd_en (tag_mem_rd_en), .rd_addr (tag_mem_rd_addr), .rd_data (tag_mem_rd_data)
  );
  sh_l2_tb_mem #(.WIDTH(FLIT_W), .DEPTH(NUM_SETS * NUM_WAYS * WORDS), .MASK_W(STRB_W), .RD_DEL(DATA_RD_DEL)) u_data_mem (
    .clk     (clk), .wr_en (data_mem_wr_en), .wr_addr (data_mem_wr_addr), .wr_data (data_mem_wr_data),
    .wr_mask (data_mem_wr_strb), .rd_en (data_mem_rd_en), .rd_addr (data_mem_rd_addr), .rd_data (data_mem_rd_data)
  );
  sh_l2_tb_mem #(.WIDTH(LRU_W), .DEPTH(NUM_SETS), .MASK_W(1), .RD_DEL(LRU_RD_DEL)) u_lru_mem (
    .clk     (clk), .wr_en (lru_mem_wr_en), .wr_addr (lru_mem_wr_addr), .wr_data (lru_mem_wr_data),
    .wr_mask (1'b1), .rd_en (lru_mem_rd_en), .rd_addr (lru_mem_rd_addr), .rd_data (lru_mem_rd_data)
  );
  sh_l2_tb_mem #(.WIDTH(PF_ENTRY_W), .DEPTH(PF_DEPTH), .MASK_W(PF_SLOTS), .RD_DEL(PF_RD_DEL)) u_pf_mem (
    .clk     (clk), .wr_en (pf_mem_wr_en), .wr_addr (pf_mem_wr_addr), .wr_data (pf_mem_wr_data),
    .wr_mask (pf_mem_wr_mask), .rd_en (pf_mem_rd_en), .rd_addr (pf_mem_rd_addr), .rd_data (pf_mem_rd_data)
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
    bit                busy;
    pkt_type_e         typ;
    logic [LEN_W-1:0]  len;
    logic [ADDR_W-1:0] addr;
    rsp_status_e       status;
    logic [FLIT_W-1:0] data [$];   // expected read data
  } exp_t;

  flit_t       req_q [$];    // flits waiting to be sent
  exp_t        exp_tab [NTAGS];
  int unsigned n_out      = 0;
  int unsigned n_sent     = 0;
  int unsigned n_done     = 0;
  int unsigned n_err_rsp  = 0;
  int unsigned n_full     = 0;
  int unsigned n_ooo      = 0;    // responses that overtook an older request
  int unsigned errors     = 0;
  int unsigned issue_seq [NTAGS];

  // Builds one random request with tag `tag`, updates the reference, queues its flits
  task automatic make_request(int unsigned tag);
    hdr_t              h;
    exp_t              e;
    int unsigned       line, word, nwords;
    logic [STRB_W-1:0] strb;
    logic [FLIT_W-1:0] d;
    bit                full, crosses;

    line    = $urandom_range(SPAN_LINES - 1);
    full    = 0;
    crosses = 0;

    h        = '0;
    h.src_x  = X_W'($urandom);
    h.src_y  = Y_W'($urandom);
    h.typ    = $urandom_range(1) ? PKT_WR_REQ : PKT_RD_REQ;
    h.tag    = TAG_W'(tag);

    if ($urandom_range(99) < CROSS_PCT && WORDS > 1) begin
      crosses = 1;
      word    = $urandom_range(WORDS - 1, 1);
      nwords  = $urandom_range(MAX_LEN, WORDS - word + 1);
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

    e.busy   = 1;
    e.typ    = (h.typ == PKT_RD_REQ) ? PKT_RD_RSP : PKT_WR_RSP;
    e.len    = h.len;
    e.addr   = h.addr;
    e.status = (crosses || bad_line(h.addr)) ? RSP_SLVERR : RSP_OK;
    e.data   = {};
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

    exp_tab[tag]   = e;
    issue_seq[tag] = n_sent;
    n_out++;
    n_sent++;
  endtask

  always @(posedge clk or negedge rst_n) begin
    int tag, s;
    if (!rst_n) begin
      t_req_vld <= 1'b0;
      for (int t = 0; t < NTAGS; t++) exp_tab[t].busy = 0;
    end else if (!t_req_vld || t_req_rdy) begin
      if (req_q.size() == 0 && n_sent < NUM_REQ && n_out < MAX_OUT) begin
        tag = -1;
        s   = $urandom_range(NTAGS - 1);
        for (int k = 0; k < NTAGS; k++) if (tag < 0 && !exp_tab[(s + k) % NTAGS].busy) tag = (s + k) % NTAGS;
        if (tag >= 0) make_request(tag);
      end
      if (req_q.size() != 0 && $urandom_range(99) < TX_PCT) begin
        t_req_flit <= req_q.pop_front();
        t_req_vld  <= 1'b1;
      end else begin
        t_req_vld  <= 1'b0;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Agent: responses (any order, matched by tag)
  // ---------------------------------------------------------------------------

  bit          rsp_in_pkt = 0;
  int unsigned rsp_beat;
  int unsigned rtag;

  task automatic finish(int unsigned tag);
    for (int t = 0; t < NTAGS; t++)
      if (exp_tab[t].busy && t != tag && issue_seq[t] < issue_seq[tag]) begin
        n_ooo++;
        break;
      end
    exp_tab[tag].busy = 0;
    n_out--;
    n_done++;
  endtask

  always @(posedge clk or negedge rst_n) begin
    hdr_t h;
    if (!rst_n) begin
      t_rsp_rdy <= 1'b0;
    end else begin
      if (t_rsp_vld && t_rsp_rdy) begin
        if (!rsp_in_pkt) begin
          h    = get_hdr(t_rsp_flit);
          rtag = 32'(h.tag);
          if (!t_rsp_flit.head || !exp_tab[rtag].busy) begin
            $error("unexpected response, tag %0d", rtag); errors++;
          end else begin
            if (h.typ != exp_tab[rtag].typ || h.len != exp_tab[rtag].len || h.status != exp_tab[rtag].status) begin
              $error("response for addr %h tag %0d: got typ %0d len %0d status %0d, expected typ %0d len %0d status %0d",
                     exp_tab[rtag].addr, rtag, h.typ, h.len, h.status, exp_tab[rtag].typ, exp_tab[rtag].len,
                     exp_tab[rtag].status);
              errors++;
            end
            if (t_rsp_flit.tail != (exp_tab[rtag].data.size() == 0)) begin
              $error("response for addr %h: framing wrong", exp_tab[rtag].addr); errors++;
            end
            if (exp_tab[rtag].status != RSP_OK) n_err_rsp++;
            rsp_beat   = 0;
            rsp_in_pkt = !t_rsp_flit.tail;
            if (t_rsp_flit.tail) finish(rtag);
          end
        end else begin
          if (t_rsp_flit.head || rsp_beat >= exp_tab[rtag].data.size()) begin
            $error("response for addr %h: extra or misframed data flit", exp_tab[rtag].addr); errors++;
          end else if (t_rsp_flit.data != exp_tab[rtag].data[rsp_beat]) begin
            $error("read addr %h beat %0d: got %h, expected %h", exp_tab[rtag].addr, rsp_beat,
                   t_rsp_flit.data, exp_tab[rtag].data[rsp_beat]);
            errors++;
          end
          rsp_beat++;
          if (t_rsp_flit.tail) begin
            if (rsp_beat != exp_tab[rtag].data.size()) begin
              $error("read addr %h: %0d data flits, expected %0d", exp_tab[rtag].addr, rsp_beat,
                     exp_tab[rtag].data.size());
              errors++;
            end
            rsp_in_pkt = 0;
            finish(rtag);
          end
        end
      end
      t_rsp_rdy <= ($urandom_range(99) < RX_PCT);
    end
  end

  // ---------------------------------------------------------------------------
  // Memory model: answers out of order
  // ---------------------------------------------------------------------------

  typedef struct {
    bit                 wb;
    logic [TAG_W-1:0]   tag;
    logic [ADDR_W-1:0]  addr;
    logic [FLIT_W-1:0]  wdata [$];   // writeback data, applied when answered
    int unsigned        delay;
    bit                 wb_error;
  } mreq_t;

  mreq_t       pend [$];              // requests not answered yet
  flit_t       mrsp_flits [$];        // response being sent
  bit          mreq_in_pkt = 0;
  mreq_t       cur_wb;
  int unsigned n_fills   = 0;
  int unsigned n_wbs     = 0;
  int unsigned n_wb_errs = 0;

  assign i_req_rdy = 1'b1;

  // Is a request of this kind / line / tag outstanding?
  function automatic bit pending(bit wb, logic [ADDR_W-1:0] addr, bit by_addr, logic [TAG_W-1:0] tag);
    foreach (pend[k])
      if (pend[k].wb == wb && (by_addr ? pend[k].addr == addr : pend[k].tag == tag)) return 1;
    return 0;
  endfunction

  always @(posedge clk) begin
    flit_t f;
    hdr_t  h;
    mreq_t r;
    if (rst_n && i_req_vld && i_req_rdy) begin
      f = i_req_flit;
      if (!mreq_in_pkt) begin
        h = get_hdr(f);
        if (!f.head || h.len != LEN_W'(WORDS - 1) || h.addr[OFF_W-1:0] != '0
            || !((h.typ == PKT_RD_REQ && f.tail) || (h.typ == PKT_WR_REQ && !f.tail))) begin
          $error("bad memory request: typ %0d tag %0d len %0d addr %h", h.typ, h.tag, h.len, h.addr);
          errors++;
        end
        r.wb       = (h.typ == PKT_WR_REQ);
        r.tag      = h.tag;
        r.addr     = h.addr;
        r.wdata    = {};
        r.delay    = $urandom_range(MEM_DLY);
        r.wb_error = r.wb && ($urandom_range(99) < WB_ERR_PCT);
        if (pending(r.wb, '0, 0, r.tag)) begin
          $error("memory request tag %0d reused while outstanding", r.tag); errors++;
        end
        if (pending(1, r.addr, 1, '0)) begin
          $error("%0s for line %h sent while its writeback is outstanding", r.wb ? "writeback" : "fill", r.addr);
          errors++;
        end
        if (r.wb) begin
          if (bad_line(r.addr)) begin
            $error("writeback of a line that could never be filled: %h", r.addr); errors++;
          end
          cur_wb = r;
        end else begin
          pend.push_back(r);
          n_fills++;
        end
        mreq_in_pkt = !f.tail;
      end else begin
        if (f.head || f.strb != '1) begin
          $error("writeback addr %h: bad data flit (strobes %h)", cur_wb.addr, f.strb); errors++;
        end
        cur_wb.wdata.push_back(f.data);
        if (f.tail) begin
          if (cur_wb.wdata.size() != WORDS) begin
            $error("writeback addr %h: %0d flits", cur_wb.addr, cur_wb.wdata.size()); errors++;
          end
          pend.push_back(cur_wb);
          n_wbs++;
          mreq_in_pkt = 0;
        end
      end
    end
  end

  // Picks a random request whose delay has run out, carries it out, answers it
  always @(posedge clk or negedge rst_n) begin
    int          ready [$];
    int          k;
    mreq_t       r;
    hdr_t        h;
    logic [FLIT_W-1:0] d;
    if (!rst_n) begin
      i_rsp_vld <= 1'b0;
    end else begin
      if (i_rsp_vld && i_rsp_rdy) i_rsp_vld <= 1'b0;
      foreach (pend[j]) if (pend[j].delay != 0) pend[j].delay--;
      if (!i_rsp_vld || i_rsp_rdy) begin
        if (mrsp_flits.size() == 0) begin
          ready = {};
          foreach (pend[j]) if (pend[j].delay == 0) ready.push_back(j);
          if (ready.size() != 0) begin
            k = ready[$urandom_range(ready.size() - 1)];
            r = pend[k];
            pend.delete(k);
            h     = '0;
            h.tag = r.tag;
            h.len = LEN_W'(WORDS - 1);
            if (r.wb) begin
              for (int i = 0; i < WORDS; i++)
                for (int b = 0; b < STRB_W; b++)
                  back_mem[r.addr + ADDR_W'(i * STRB_W + b)] = r.wdata[i][8*b +: 8];
              h.typ    = PKT_WR_RSP;
              h.status = r.wb_error ? RSP_SLVERR : RSP_OK;
              if (r.wb_error) n_wb_errs++;
              mrsp_flits.push_back('{head: 1'b1, tail: 1'b1, strb: '0, data: hdr_to_data(h)});
            end else begin
              h.typ    = PKT_RD_RSP;
              h.status = bad_line(r.addr) ? RSP_SLVERR : RSP_OK;
              mrsp_flits.push_back('{head: 1'b1, tail: (h.status != RSP_OK), strb: '0, data: hdr_to_data(h)});
              if (h.status == RSP_OK) begin
                for (int i = 0; i < WORDS; i++) begin
                  for (int b = 0; b < STRB_W; b++) d[8*b +: 8] = back_byte(r.addr + ADDR_W'(i * STRB_W + b));
                  mrsp_flits.push_back('{head: 1'b0, tail: (i == WORDS - 1), strb: '0, data: d});
                end
              end
            end
          end
        end
        if (mrsp_flits.size() != 0) begin
          i_rsp_flit <= mrsp_flits.pop_front();
          i_rsp_vld  <= 1'b1;
        end
      end
    end
  end

  logic err_evt;     // an error writeback response is accepted this cycle

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
    $display("sh_l2_tb: %0d bytes, %0d ways, %0d sets, %0d B lines, %0d-entry pending FIFO, read latency tag/data/lru/req=%0d/%0d/%0d/%0d, %0d requests over %0d lines",
             L2_BYTES, NUM_WAYS, NUM_SETS, LINE_BYTES, PF_DEPTH, TAG_RD_DEL, DATA_RD_DEL, LRU_RD_DEL,
             PF_RD_DEL, NUM_REQ, SPAN_LINES);
    #33ns rst_n = 1'b1;
    while (n_done < NUM_REQ && cycles < 64'(NUM_REQ) * 400 + 64'(NUM_SETS) + 10000) begin
      @(posedge clk);
      cycles++;
    end
    repeat (200) @(posedge clk);
    if (n_out != 0) begin
      $error("%0d responses never arrived", n_out); errors++;
    end
    if (pend.size() != 0) begin
      $error("%0d memory requests never answered", pend.size()); errors++;
    end
    if (errors == 0 && n_done == NUM_REQ)
      $display("PASS: %0d requests (%0d out of order, %0d error responses, %0d whole-line writes), %0d fills, %0d writebacks (%0d failed), %0d cycles",
               n_done, n_ooo, n_err_rsp, n_full, n_fills, n_wbs, n_wb_errs, cycles);
    else
      $display("FAIL: %0d errors, %0d / %0d requests completed", errors, n_done, NUM_REQ);
    $finish;
  end

endmodule
