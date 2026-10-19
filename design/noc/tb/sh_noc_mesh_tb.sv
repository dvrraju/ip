// sh_noc_mesh_tb: self-checking random-transaction test for sh_noc_mesh
//
// Every tile is an initiator and a target, on its own clock when ASYNC = 1.
//
// Initiator: issues NUM_TXN random reads and writes (1..MAX_LEN data flits)
// to random addresses, UNMAPPED_PCT % of them unmapped, with up to 2**TAG_W
// outstanding. Header dst/src fields are filled with junk to check that the
// NI replaces them. Each response is checked against its tag: type, len,
// source tile (the target, or itself for DECERR), status, and read data.
//
// Target: checks each request arrived at the tile its address maps to, and
// its write data; then returns one response (read data for reads). Response
// src is filled with junk to check that the NI stamps it.
//
// Write data and read data are functions of (source, tag, address, beat), so
// both ends can check without sharing state. The test passes when every
// transaction has completed once, and the targets saw exactly the mapped ones.
//
// The FIFO memory ports are connected to sh_noc_tb_mem models with the
// configured read latency.

module sh_noc_mesh_tb
  import ip_top_pkg::ADDR_W;
  import sh_noc_pkg::*;
#(
  parameter int unsigned BUF_DEPTH      = 4,
  parameter int unsigned BUF_MEM_RD_DEL = 0,
  parameter logic        ASYNC          = 1'b1,
  parameter int unsigned CDC_DEPTH      = 4,
  parameter int unsigned CDC_MEM_RD_DEL = 0,
  parameter logic        RST_LOW        = 1'b1,
  parameter logic        RST_ASYNC      = 1'b1,
  parameter int unsigned NUM_TXN        = 100,  // transactions per initiator
  parameter int unsigned UNMAPPED_PCT   = 10,   // requests to unmapped addresses
  parameter int unsigned TX_PCT         = 70,   // chance per cycle a flit is offered
  parameter int unsigned RX_PCT         = 70    // chance per cycle a flit is accepted
);

  localparam int unsigned NT     = NUM_TILES;
  localparam int unsigned NTAGS  = 1 << TAG_W;
  localparam int unsigned BAW    = (BUF_DEPTH > 1) ? $clog2(BUF_DEPTH) : 1;
  localparam int unsigned CAW    = (CDC_DEPTH > 1) ? $clog2(CDC_DEPTH) : 1;

  initial begin
    if (NT > 256) $fatal(1, "sh_noc_mesh_tb: at most 256 tiles");
  end

  // ---------------------------------------------------------------------------
  // Data patterns
  // ---------------------------------------------------------------------------

  function automatic logic [FLIT_W-1:0] wr_data(int unsigned src, int unsigned tag,
                                                logic [ADDR_W-1:0] addr, int unsigned beat);
    return FLIT_W'({8'(src), 8'(beat), 16'(tag), 32'(addr)});
  endfunction

  function automatic logic [FLIT_W-1:0] rd_data(logic [ADDR_W-1:0] addr, int unsigned beat);
    return FLIT_W'({~32'(addr), 16'hDA7A, 16'(beat)});
  endfunction

  function automatic int unsigned tile_id(logic [X_W-1:0] x, logic [Y_W-1:0] y);
    return 32'(y) * COLS + 32'(x);
  endfunction

  // Random address that is (or is not) in the address map
  function automatic logic [ADDR_W-1:0] rand_addr(bit mapped);
    logic [ADDR_W-1:0] a;
    longint unsigned   size;
    int unsigned       r;
    if (mapped) begin
      r    = $urandom_range(NUM_REGIONS - 1);
      size = longint'(ADDR_MAP[r].last) - longint'(ADDR_MAP[r].base) + 1;
      return ADDR_MAP[r].base + ADDR_W'(longint'({$urandom, $urandom}) % size);
    end
    for (int k = 0; k < 10000; k++) begin
      a = ADDR_W'({$urandom, $urandom});
      if (!addr_decode(a).hit) return a;
    end
    $fatal(1, "sh_noc_mesh_tb: could not find an unmapped address");
    return '0;
  endfunction

  // ---------------------------------------------------------------------------
  // Clocks and reset
  // ---------------------------------------------------------------------------

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  logic rst;

  always #5ns clk = ~clk;
  assign rst = RST_LOW ? rst_n : !rst_n;

  logic  [ROWS-1:0][COLS-1:0] tile_clk;
  logic  [ROWS-1:0][COLS-1:0] tile_rst;
  logic  [ROWS-1:0][COLS-1:0] i_req_vld;
  logic  [ROWS-1:0][COLS-1:0] i_req_rdy;
  flit_t [ROWS-1:0][COLS-1:0] i_req_flit;
  logic  [ROWS-1:0][COLS-1:0] i_rsp_vld;
  logic  [ROWS-1:0][COLS-1:0] i_rsp_rdy;
  flit_t [ROWS-1:0][COLS-1:0] i_rsp_flit;
  logic  [ROWS-1:0][COLS-1:0] t_req_vld;
  logic  [ROWS-1:0][COLS-1:0] t_req_rdy;
  flit_t [ROWS-1:0][COLS-1:0] t_req_flit;
  logic  [ROWS-1:0][COLS-1:0] t_rsp_vld;
  logic  [ROWS-1:0][COLS-1:0] t_rsp_rdy;
  flit_t [ROWS-1:0][COLS-1:0] t_rsp_flit;

  logic [ROWS-1:0][COLS-1:0][1:0][5:0]                  buf_mem_wr_en;
  logic [ROWS-1:0][COLS-1:0][1:0][5:0][BAW-1:0]         buf_mem_wr_addr;
  logic [ROWS-1:0][COLS-1:0][1:0][5:0][FLIT_LINK_W-1:0] buf_mem_wr_data;
  logic [ROWS-1:0][COLS-1:0][1:0][5:0]                  buf_mem_rd_en;
  logic [ROWS-1:0][COLS-1:0][1:0][5:0][BAW-1:0]         buf_mem_rd_addr;
  logic [ROWS-1:0][COLS-1:0][1:0][5:0][FLIT_LINK_W-1:0] buf_mem_rd_data;
  logic [ROWS-1:0][COLS-1:0][3:0]                       cdc_mem_wr_en;
  logic [ROWS-1:0][COLS-1:0][3:0][CAW-1:0]              cdc_mem_wr_addr;
  logic [ROWS-1:0][COLS-1:0][3:0][FLIT_LINK_W-1:0]      cdc_mem_wr_data;
  logic [ROWS-1:0][COLS-1:0][3:0]                       cdc_mem_rd_en;
  logic [ROWS-1:0][COLS-1:0][3:0][CAW-1:0]              cdc_mem_rd_addr;
  logic [ROWS-1:0][COLS-1:0][3:0][FLIT_LINK_W-1:0]      cdc_mem_rd_data;

  // ---------------------------------------------------------------------------
  // DUT
  // ---------------------------------------------------------------------------

  sh_noc_mesh #(
    .BUF_DEPTH      (BUF_DEPTH),
    .BUF_MEM_RD_DEL (BUF_MEM_RD_DEL),
    .ASYNC          (ASYNC),
    .CDC_DEPTH      (CDC_DEPTH),
    .CDC_MEM_RD_DEL (CDC_MEM_RD_DEL),
    .RST_LOW        (RST_LOW),
    .RST_ASYNC      (RST_ASYNC)
  ) u_dut (
    .clk             (clk),
    .rst             (rst),
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

  int unsigned issued_mapped = 0;   // requests sent to mapped addresses
  int unsigned target_rcvd   = 0;   // requests received by targets
  int unsigned completed     = 0;   // responses received by initiators
  int unsigned errors        = 0;

  // ---------------------------------------------------------------------------
  // Tiles
  // ---------------------------------------------------------------------------

  for (genvar y = 0; y < ROWS; y++) begin : g_row
    for (genvar x = 0; x < COLS; x++) begin : g_col
      localparam int unsigned T = y * COLS + x;
      // Tile clock periods spread between ~6 and ~17 ns
      localparam realtime HALF = (3.1 + 0.7 * ((T * 7) % 13)) * 1ns;

      logic tclk = 1'b0;
      logic tsc;                 // clock the tile logic runs on

      always #(HALF) tclk = ~tclk;
      assign tile_clk[y][x] = tclk;
      assign tile_rst[y][x] = rst;
      assign tsc            = ASYNC ? tclk : clk;

      // -----------------------------------------------------------------------
      // Memories
      // -----------------------------------------------------------------------

      for (genvar n = 0; n < 2; n++) begin : g_net
        for (genvar p = 0; p < 6; p++) begin : g_buf
          sh_noc_tb_mem #(.WIDTH(FLIT_LINK_W), .DEPTH(BUF_DEPTH), .RD_DEL(BUF_MEM_RD_DEL)) u_mem (
            .wr_clk  (clk),
            .wr_en   (buf_mem_wr_en[y][x][n][p]),
            .wr_addr (buf_mem_wr_addr[y][x][n][p]),
            .wr_data (buf_mem_wr_data[y][x][n][p]),
            .rd_clk  (clk),
            .rd_en   (buf_mem_rd_en[y][x][n][p]),
            .rd_addr (buf_mem_rd_addr[y][x][n][p]),
            .rd_data (buf_mem_rd_data[y][x][n][p])
          );
        end
      end

      // CDC FIFOs 0 and 3 are written on the tile side, 1 and 2 on clk
      for (genvar f = 0; f < 4; f++) begin : g_cdc
        localparam bit FROM_TILE = (f == 0 || f == 3);
        sh_noc_tb_mem #(.WIDTH(FLIT_LINK_W), .DEPTH(CDC_DEPTH), .RD_DEL(CDC_MEM_RD_DEL)) u_mem (
          .wr_clk  (FROM_TILE ? tsc : clk),
          .wr_en   (cdc_mem_wr_en[y][x][f]),
          .wr_addr (cdc_mem_wr_addr[y][x][f]),
          .wr_data (cdc_mem_wr_data[y][x][f]),
          .rd_clk  (FROM_TILE ? clk : tsc),
          .rd_en   (cdc_mem_rd_en[y][x][f]),
          .rd_addr (cdc_mem_rd_addr[y][x][f]),
          .rd_data (cdc_mem_rd_data[y][x][f])
        );
      end

      // Each tile drives its own signals, mapped onto the shared vectors, so
      // no vector is driven from several clocks
      logic  ireq_vld;
      flit_t ireq_flit;
      logic  irsp_rdy;
      logic  treq_rdy;
      logic  trsp_vld;
      flit_t trsp_flit;

      assign i_req_vld[y][x]  = ireq_vld;
      assign i_req_flit[y][x] = ireq_flit;
      assign i_rsp_rdy[y][x]  = irsp_rdy;
      assign t_req_rdy[y][x]  = treq_rdy;
      assign t_rsp_vld[y][x]  = trsp_vld;
      assign t_rsp_flit[y][x] = trsp_flit;

      // -----------------------------------------------------------------------
      // Initiator
      // -----------------------------------------------------------------------

      typedef struct {
        bit                busy;
        pkt_type_e         typ;
        logic [LEN_W-1:0]  len;
        logic [ADDR_W-1:0] addr;
        bit                mapped;
        int unsigned       target;
      } txn_t;

      txn_t        txn [NTAGS];
      flit_t       ireq_q [$];         // flits of the request being sent
      int unsigned n_issued = 0;

      // Request generator and sender
      always @(posedge tsc or negedge rst_n) begin
        hdr_t      h;
        addr_dec_t dec;
        int        tag;
        int        s;
        if (!rst_n) begin
          ireq_vld <= 1'b0;
          for (int t = 0; t < NTAGS; t++) txn[t].busy = 1'b0;
        end else if (!ireq_vld || i_req_rdy[y][x]) begin
          if (ireq_q.size() == 0 && n_issued < NUM_TXN) begin
            tag = -1;
            s   = $urandom_range(NTAGS - 1);
            for (int k = 0; k < NTAGS; k++) begin
              if (tag < 0 && !txn[(s + k) % NTAGS].busy) tag = (s + k) % NTAGS;
            end
            if (tag >= 0) begin
              h        = hdr_t'({$urandom, $urandom});   // junk in dst/src
              h.typ    = $urandom_range(1) ? PKT_WR_REQ : PKT_RD_REQ;
              h.tag    = TAG_W'(tag);
              h.len    = LEN_W'($urandom_range(MAX_LEN - 1));
              h.status = RSP_OK;
              h.addr   = rand_addr($urandom_range(99) >= UNMAPPED_PCT);
              dec      = addr_decode(h.addr);

              txn[tag].busy   = 1'b1;
              txn[tag].typ    = h.typ;
              txn[tag].len    = h.len;
              txn[tag].addr   = h.addr;
              txn[tag].mapped = dec.hit;
              txn[tag].target = dec.hit ? tile_id(dec.x, dec.y) : T;
              n_issued++;
              if (dec.hit) issued_mapped++;

              ireq_q.push_back('{head: 1'b1, tail: (h.typ == PKT_RD_REQ), data: hdr_to_data(h)});
              if (h.typ == PKT_WR_REQ) begin
                for (int i = 0; i <= 32'(h.len); i++) begin
                  ireq_q.push_back('{head: 1'b0, tail: (i == 32'(h.len)),
                                     data: wr_data(T, tag, h.addr, i)});
                end
              end
            end
          end
          if (ireq_q.size() != 0 && $urandom_range(99) < TX_PCT) begin
            ireq_flit <= ireq_q.pop_front();
            ireq_vld  <= 1'b1;
          end else begin
            ireq_vld  <= 1'b0;
          end
        end
      end

      // Response checker
      bit          irsp_in_pkt = 0;
      int unsigned irsp_tag;
      int unsigned irsp_beat;

      always @(posedge tsc or negedge rst_n) begin
        hdr_t        h;
        flit_t       f;
        rsp_status_e exp_st;
        if (!rst_n) begin
          irsp_rdy <= 1'b0;
        end else begin
          if (i_rsp_vld[y][x] && irsp_rdy) begin
            f = i_rsp_flit[y][x];
            if (!irsp_in_pkt) begin
              h        = get_hdr(f);
              irsp_tag = 32'(h.tag);
              exp_st   = txn[irsp_tag].mapped ? RSP_OK : RSP_DECERR;
              if (!f.head) begin
                $error("tile %0d: response flit without a header", T); errors++;
              end else if (!txn[irsp_tag].busy) begin
                $error("tile %0d: response for idle tag %0d", T, irsp_tag); errors++;
              end else if (32'(h.dst_x) != x || 32'(h.dst_y) != y) begin
                $error("tile %0d: response addressed to (%0d,%0d)", T, h.dst_x, h.dst_y); errors++;
              end else if (h.typ != ((txn[irsp_tag].typ == PKT_RD_REQ) ? PKT_RD_RSP : PKT_WR_RSP)
                           || h.len != txn[irsp_tag].len || h.status != exp_st
                           || tile_id(h.src_x, h.src_y) != txn[irsp_tag].target) begin
                $error("tile %0d tag %0d: bad response typ=%0d len=%0d status=%0d src=%0d (expected src %0d, status %0d)",
                       T, irsp_tag, h.typ, h.len, h.status, tile_id(h.src_x, h.src_y),
                       txn[irsp_tag].target, exp_st);
                errors++;
              end else if (f.tail != !has_data(h)) begin
                $error("tile %0d tag %0d: response framing wrong", T, irsp_tag); errors++;
              end
              irsp_beat   = 0;
              irsp_in_pkt = !f.tail;
              if (f.tail) begin
                txn[irsp_tag].busy = 1'b0;
                completed++;
              end
            end else begin
              if (f.head || f.data != rd_data(txn[irsp_tag].addr, irsp_beat)
                  || f.tail != (irsp_beat == 32'(txn[irsp_tag].len))) begin
                $error("tile %0d tag %0d: bad read data beat %0d", T, irsp_tag, irsp_beat); errors++;
              end
              irsp_beat++;
              if (f.tail) begin
                irsp_in_pkt        = 0;
                txn[irsp_tag].busy = 1'b0;
                completed++;
              end
            end
          end
          irsp_rdy <= ($urandom_range(99) < RX_PCT);
        end
      end

      // -----------------------------------------------------------------------
      // Target
      // -----------------------------------------------------------------------

      flit_t       trsp_q [$];         // response flits waiting to be sent
      bit          treq_in_pkt = 0;
      hdr_t        treq_hdr;
      int unsigned treq_beat;

      // Request checker; queues one response per request
      always @(posedge tsc or negedge rst_n) begin
        flit_t     f;
        hdr_t      r;
        addr_dec_t dec;
        bit        done;
        if (!rst_n) begin
          treq_rdy <= 1'b0;
        end else begin
          if (t_req_vld[y][x] && treq_rdy) begin
            f    = t_req_flit[y][x];
            done = 1'b0;
            if (!treq_in_pkt) begin
              treq_hdr  = get_hdr(f);
              dec       = addr_decode(treq_hdr.addr);
              treq_beat = 0;
              if (!f.head) begin
                $error("tile %0d: request flit without a header", T); errors++;
              end else if (!dec.hit || 32'(dec.x) != x || 32'(dec.y) != y
                           || 32'(treq_hdr.dst_x) != x || 32'(treq_hdr.dst_y) != y) begin
                $error("tile %0d: request for address %h delivered here", T, treq_hdr.addr); errors++;
              end else if (tile_id(treq_hdr.src_x, treq_hdr.src_y) >= NT) begin
                $error("tile %0d: request from tile %0d", T, tile_id(treq_hdr.src_x, treq_hdr.src_y)); errors++;
              end
              treq_in_pkt = !f.tail;
              done        = f.tail;
              if (f.tail != (treq_hdr.typ == PKT_RD_REQ)) begin
                $error("tile %0d: request framing wrong", T); errors++;
              end
            end else begin
              if (f.head || f.data != wr_data(tile_id(treq_hdr.src_x, treq_hdr.src_y),
                                              treq_hdr.tag, treq_hdr.addr, treq_beat)
                  || f.tail != (treq_beat == 32'(treq_hdr.len))) begin
                $error("tile %0d: bad write data beat %0d from tile %0d tag %0d", T, treq_beat,
                       tile_id(treq_hdr.src_x, treq_hdr.src_y), treq_hdr.tag);
                errors++;
              end
              treq_beat++;
              treq_in_pkt = !f.tail;
              done        = f.tail;
            end

            if (done) begin
              target_rcvd++;
              r        = hdr_t'({$urandom, $urandom});   // junk in src
              r.dst_x  = treq_hdr.src_x;
              r.dst_y  = treq_hdr.src_y;
              r.typ    = (treq_hdr.typ == PKT_RD_REQ) ? PKT_RD_RSP : PKT_WR_RSP;
              r.tag    = treq_hdr.tag;
              r.len    = treq_hdr.len;
              r.status = RSP_OK;
              r.addr   = '0;
              trsp_q.push_back('{head: 1'b1, tail: (r.typ == PKT_WR_RSP), data: hdr_to_data(r)});
              if (r.typ == PKT_RD_RSP) begin
                for (int i = 0; i <= 32'(r.len); i++) begin
                  trsp_q.push_back('{head: 1'b0, tail: (i == 32'(r.len)),
                                     data: rd_data(treq_hdr.addr, i)});
                end
              end
            end
          end
          treq_rdy <= ($urandom_range(99) < RX_PCT);
        end
      end

      // Response sender
      always @(posedge tsc or negedge rst_n) begin
        if (!rst_n) begin
          trsp_vld <= 1'b0;
        end else if (!trsp_vld || t_rsp_rdy[y][x]) begin
          if (trsp_q.size() != 0 && $urandom_range(99) < TX_PCT) begin
            trsp_flit <= trsp_q.pop_front();
            trsp_vld  <= 1'b1;
          end else begin
            trsp_vld  <= 1'b0;
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Test control
  // ---------------------------------------------------------------------------

  initial begin
    automatic int unsigned expected = NT * NUM_TXN;
    automatic longint      cycles   = 0;

    $display("sh_noc_mesh_tb: %0dx%0d mesh, FLIT_W=%0d, MAX_LEN=%0d, TAG_W=%0d, ASYNC=%0d, BUF_DEPTH=%0d, read latency buf/cdc=%0d/%0d, %0d transactions",
             ROWS, COLS, FLIT_W, MAX_LEN, TAG_W, ASYNC, BUF_DEPTH, BUF_MEM_RD_DEL,
             CDC_MEM_RD_DEL, expected);

    #33ns rst_n = 1'b1;

    while (completed < expected && cycles < 64'(expected) * MAX_LEN * 100) begin
      @(posedge clk);
      cycles++;
    end
    // Let anything extra (duplicates) show up
    repeat (500) @(posedge clk);

    if (target_rcvd != issued_mapped) begin
      $error("targets received %0d requests, %0d were sent to mapped addresses",
             target_rcvd, issued_mapped);
      errors++;
    end

    if (errors == 0 && completed == expected)
      $display("PASS: %0d transactions (%0d mapped, %0d DECERR) in %0d NoC cycles",
               completed, issued_mapped, completed - issued_mapped, cycles);
    else
      $display("FAIL: %0d errors, %0d / %0d transactions completed", errors, completed, expected);
    $finish;
  end

endmodule
