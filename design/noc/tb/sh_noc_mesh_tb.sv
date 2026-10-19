// sh_noc_mesh_tb: self-checking random-traffic test for sh_noc_mesh
//
// Every tile sends NUM_PKTS packets to random tiles (itself included), on
// its own clock when ASYNC = 1, with random inject gaps and eject
// back-pressure. The payload is {src, dst, seq}; each receiver checks that
// a packet arrived at the right tile and in order for its (src, dst) pair,
// and the test checks that every packet sent was received.
//
// The FIFO memory ports are connected to sh_lib_beh_mem_mrnw instances
// (combinational read), standing in for the memories used in a real design.

module sh_noc_mesh_tb #(
  parameter int unsigned ROWS      = 3,
  parameter int unsigned COLS      = 4,
  parameter int unsigned DATA_W    = 32,
  parameter int unsigned BUF_DEPTH = 4,
  parameter logic        ASYNC     = 1'b1,
  parameter int unsigned CDC_DEPTH = 4,
  parameter logic        RST_LOW   = 1'b1,
  parameter logic        RST_ASYNC = 1'b1,
  parameter int unsigned NUM_PKTS  = 300,
  parameter int unsigned INJ_PCT   = 60,   // chance per cycle a tile offers a packet
  parameter int unsigned EJ_PCT    = 70    // chance per cycle a tile accepts a packet
);

  localparam int unsigned NT  = ROWS * COLS;
  localparam int unsigned X_W = (COLS > 1) ? $clog2(COLS) : 1;
  localparam int unsigned Y_W = (ROWS > 1) ? $clog2(ROWS) : 1;
  localparam int unsigned FW  = Y_W + X_W + DATA_W;
  localparam int unsigned BAW = (BUF_DEPTH > 1) ? $clog2(BUF_DEPTH) : 1;
  localparam int unsigned CAW = (CDC_DEPTH > 1) ? $clog2(CDC_DEPTH) : 1;

  initial begin
    if (DATA_W < 32) $fatal(1, "sh_noc_mesh_tb: DATA_W must be >= 32");
    if (NT > 256)    $fatal(1, "sh_noc_mesh_tb: at most 256 tiles");
  end

  // ---------------------------------------------------------------------------
  // Clocks and reset
  // ---------------------------------------------------------------------------

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  logic rst;

  always #5ns clk = ~clk;
  assign rst = RST_LOW ? rst_n : !rst_n;

  logic [ROWS-1:0][COLS-1:0]             tile_clk;
  logic [ROWS-1:0][COLS-1:0]             tile_rst;
  logic [ROWS-1:0][COLS-1:0]             inj_vld;
  logic [ROWS-1:0][COLS-1:0]             inj_rdy;
  logic [ROWS-1:0][COLS-1:0][X_W-1:0]    inj_dst_x;
  logic [ROWS-1:0][COLS-1:0][Y_W-1:0]    inj_dst_y;
  logic [ROWS-1:0][COLS-1:0][DATA_W-1:0] inj_data;
  logic [ROWS-1:0][COLS-1:0]             ej_vld;
  logic [ROWS-1:0][COLS-1:0]             ej_rdy;
  logic [ROWS-1:0][COLS-1:0][DATA_W-1:0] ej_data;

  logic [ROWS-1:0][COLS-1:0][5:0]          buf_mem_wr_en;
  logic [ROWS-1:0][COLS-1:0][5:0][BAW-1:0] buf_mem_wr_addr;
  logic [ROWS-1:0][COLS-1:0][5:0][FW-1:0]  buf_mem_wr_data;
  logic [ROWS-1:0][COLS-1:0][5:0]          buf_mem_rd_en;
  logic [ROWS-1:0][COLS-1:0][5:0][BAW-1:0] buf_mem_rd_addr;
  logic [ROWS-1:0][COLS-1:0][5:0][FW-1:0]  buf_mem_rd_data;
  logic [ROWS-1:0][COLS-1:0]               inj_mem_wr_en;
  logic [ROWS-1:0][COLS-1:0][CAW-1:0]      inj_mem_wr_addr;
  logic [ROWS-1:0][COLS-1:0][FW-1:0]       inj_mem_wr_data;
  logic [ROWS-1:0][COLS-1:0]               inj_mem_rd_en;
  logic [ROWS-1:0][COLS-1:0][CAW-1:0]      inj_mem_rd_addr;
  logic [ROWS-1:0][COLS-1:0][FW-1:0]       inj_mem_rd_data;
  logic [ROWS-1:0][COLS-1:0]               ej_mem_wr_en;
  logic [ROWS-1:0][COLS-1:0][CAW-1:0]      ej_mem_wr_addr;
  logic [ROWS-1:0][COLS-1:0][DATA_W-1:0]   ej_mem_wr_data;
  logic [ROWS-1:0][COLS-1:0]               ej_mem_rd_en;
  logic [ROWS-1:0][COLS-1:0][CAW-1:0]      ej_mem_rd_addr;
  logic [ROWS-1:0][COLS-1:0][DATA_W-1:0]   ej_mem_rd_data;

  // ---------------------------------------------------------------------------
  // DUT
  // ---------------------------------------------------------------------------

  sh_noc_mesh #(
    .ROWS      (ROWS),
    .COLS      (COLS),
    .DATA_W    (DATA_W),
    .BUF_DEPTH (BUF_DEPTH),
    .ASYNC     (ASYNC),
    .CDC_DEPTH (CDC_DEPTH),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_dut (
    .clk             (clk),
    .rst             (rst),
    .tile_clk        (tile_clk),
    .tile_rst        (tile_rst),
    .inj_vld         (inj_vld),
    .inj_rdy         (inj_rdy),
    .inj_dst_x       (inj_dst_x),
    .inj_dst_y       (inj_dst_y),
    .inj_data        (inj_data),
    .ej_vld          (ej_vld),
    .ej_rdy          (ej_rdy),
    .ej_data         (ej_data),
    .buf_mem_wr_en   (buf_mem_wr_en),
    .buf_mem_wr_addr (buf_mem_wr_addr),
    .buf_mem_wr_data (buf_mem_wr_data),
    .buf_mem_rd_en   (buf_mem_rd_en),
    .buf_mem_rd_addr (buf_mem_rd_addr),
    .buf_mem_rd_data (buf_mem_rd_data),
    .inj_mem_wr_en   (inj_mem_wr_en),
    .inj_mem_wr_addr (inj_mem_wr_addr),
    .inj_mem_wr_data (inj_mem_wr_data),
    .inj_mem_rd_en   (inj_mem_rd_en),
    .inj_mem_rd_addr (inj_mem_rd_addr),
    .inj_mem_rd_data (inj_mem_rd_data),
    .ej_mem_wr_en    (ej_mem_wr_en),
    .ej_mem_wr_addr  (ej_mem_wr_addr),
    .ej_mem_wr_data  (ej_mem_wr_data),
    .ej_mem_rd_en    (ej_mem_rd_en),
    .ej_mem_rd_addr  (ej_mem_rd_addr),
    .ej_mem_rd_data  (ej_mem_rd_data)
  );

  // ---------------------------------------------------------------------------
  // Scoreboard
  // ---------------------------------------------------------------------------

  int unsigned sent_cnt [NT][NT];   // [src][dst]
  int unsigned rcv_cnt  [NT][NT];
  int unsigned total_rcv = 0;
  int unsigned errors    = 0;

  // ---------------------------------------------------------------------------
  // Per-tile clock, memories, traffic source and sink
  // ---------------------------------------------------------------------------

  for (genvar y = 0; y < ROWS; y++) begin : g_row
    for (genvar x = 0; x < COLS; x++) begin : g_col
      localparam int unsigned T = y * COLS + x;
      // Tile clock periods spread between ~6 and ~17 ns
      localparam realtime HALF = (3.1 + 0.7 * ((T * 7) % 13)) * 1ns;

      logic tclk = 1'b0;
      logic tile_side_clk;

      always #(HALF) tclk = ~tclk;
      assign tile_clk[y][x] = tclk;
      assign tile_rst[y][x] = rst;
      assign tile_side_clk  = ASYNC ? tclk : clk;   // clock the tile logic runs on

      // Memories
      for (genvar p = 0; p < 6; p++) begin : g_buf_mem
        sh_lib_beh_mem_mrnw #(.WIDTH(FW), .DEPTH(BUF_DEPTH), .RD_DEL(0)) u_mem (
          .clk     (clk),
          .rst     (rst),
          .wr_en   (buf_mem_wr_en[y][x][p]),
          .wr_addr (buf_mem_wr_addr[y][x][p]),
          .wr_data (buf_mem_wr_data[y][x][p]),
          .rd_en   (buf_mem_rd_en[y][x][p]),
          .rd_addr (buf_mem_rd_addr[y][x][p]),
          .rd_data (buf_mem_rd_data[y][x][p]),
          .rd_vld  ()
        );
      end

      sh_lib_beh_mem_mrnw #(.WIDTH(FW), .DEPTH(CDC_DEPTH), .RD_DEL(0)) u_inj_mem (
        .clk     (tile_side_clk),
        .rst     (rst),
        .wr_en   (inj_mem_wr_en[y][x]),
        .wr_addr (inj_mem_wr_addr[y][x]),
        .wr_data (inj_mem_wr_data[y][x]),
        .rd_en   (inj_mem_rd_en[y][x]),
        .rd_addr (inj_mem_rd_addr[y][x]),
        .rd_data (inj_mem_rd_data[y][x]),
        .rd_vld  ()
      );

      sh_lib_beh_mem_mrnw #(.WIDTH(DATA_W), .DEPTH(CDC_DEPTH), .RD_DEL(0)) u_ej_mem (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (ej_mem_wr_en[y][x]),
        .wr_addr (ej_mem_wr_addr[y][x]),
        .wr_data (ej_mem_wr_data[y][x]),
        .rd_en   (ej_mem_rd_en[y][x]),
        .rd_addr (ej_mem_rd_addr[y][x]),
        .rd_data (ej_mem_rd_data[y][x]),
        .rd_vld  ()
      );

      // Traffic source. Each tile drives its own signals, mapped onto the
      // shared vectors below, so no vector is driven from several clocks.
      int unsigned       issued = 0;
      logic              src_vld;
      logic [X_W-1:0]    src_dst_x;
      logic [Y_W-1:0]    src_dst_y;
      logic [DATA_W-1:0] src_data;
      logic              snk_rdy;

      assign inj_vld[y][x]   = src_vld;
      assign inj_dst_x[y][x] = src_dst_x;
      assign inj_dst_y[y][x] = src_dst_y;
      assign inj_data[y][x]  = src_data;
      assign ej_rdy[y][x]    = snk_rdy;

      always @(posedge tile_side_clk or negedge rst_n) begin
        int unsigned d;
        if (!rst_n) begin
          src_vld <= 1'b0;
        end else if (!src_vld || inj_rdy[y][x]) begin
          if (issued < NUM_PKTS && $urandom_range(99) < INJ_PCT) begin
            d = $urandom_range(NT - 1);
            src_dst_x <= X_W'(d % COLS);
            src_dst_y <= Y_W'(d / COLS);
            src_data  <= DATA_W'({8'(T), 8'(d), 16'(sent_cnt[T][d])});
            src_vld   <= 1'b1;
            sent_cnt[T][d]++;
            issued++;
          end else begin
            src_vld <= 1'b0;
          end
        end
      end

      // Traffic sink
      always @(posedge tile_side_clk or negedge rst_n) begin
        int unsigned s;
        if (!rst_n) begin
          snk_rdy <= 1'b0;
        end else begin
          if (ej_vld[y][x] && snk_rdy) begin
            s = 32'(ej_data[y][x][31:24]);
            if (32'(ej_data[y][x][23:16]) != T) begin
              $error("tile %0d: got packet for tile %0d (from %0d)", T, ej_data[y][x][23:16], s);
              errors++;
            end else if (ej_data[y][x][15:0] != 16'(rcv_cnt[s][T])) begin
              $error("tile %0d: from %0d expected seq %0d, got %0d",
                     T, s, rcv_cnt[s][T], ej_data[y][x][15:0]);
              errors++;
            end
            rcv_cnt[s][T]++;
            total_rcv++;
          end
          snk_rdy <= ($urandom_range(99) < EJ_PCT);
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Test control
  // ---------------------------------------------------------------------------

  initial begin
    automatic int unsigned expected = NT * NUM_PKTS;
    automatic longint      cycles   = 0;

    $display("sh_noc_mesh_tb: %0dx%0d mesh, ASYNC=%0d, BUF_DEPTH=%0d, %0d packets",
             ROWS, COLS, ASYNC, BUF_DEPTH, expected);

    #33ns rst_n = 1'b1;

    while (total_rcv < expected && cycles < 64'(expected) * 200) begin
      @(posedge clk);
      cycles++;
    end
    // Let anything extra (duplicates) show up
    repeat (200) @(posedge clk);

    for (int s = 0; s < NT; s++) begin
      for (int d = 0; d < NT; d++) begin
        if (sent_cnt[s][d] != rcv_cnt[s][d]) begin
          $error("%0d -> %0d: sent %0d, received %0d", s, d, sent_cnt[s][d], rcv_cnt[s][d]);
          errors++;
        end
      end
    end

    if (errors == 0 && total_rcv == expected)
      $display("PASS: %0d packets delivered in %0d NoC cycles", total_rcv, cycles);
    else
      $display("FAIL: %0d errors, %0d / %0d packets received", errors, total_rcv, expected);
    $finish;
  end

endmodule
