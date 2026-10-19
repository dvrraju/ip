// sh_noc_mesh: ROWS x COLS 2D mesh network-on-chip
//
// Parameters:
//   ROWS           - number of rows (M); y = 0 .. ROWS-1, y grows North
//   COLS           - number of columns (N); x = 0 .. COLS-1, x grows East
//   DATA_W         - payload width in bits
//   BUF_DEPTH      - router input buffer depth (>= 4 for full link throughput,
//                    >= BUF_MEM_RD_DEL + 5 when BUF_MEM_RD_DEL > 0)
//   BUF_MEM_RD_DEL - read latency of the buf_mem_* memories; 0 = combinational
//   ASYNC          - 1 = each tile has its own clock (tile_clk[y][x]),
//                    0 = tiles run on clk and tile_clk/tile_rst are ignored
//   CDC_DEPTH      - entries in each tile-side FIFO (power of 2 >= 2 when ASYNC)
//   SYNC_DELAY     - synchronizer flops per pointer crossing (ASYNC = 1 only)
//   CDC_MEM_RD_DEL - read latency of the inj_mem_* / ej_mem_* memories, in
//                    the read side's clock; 0 = combinational
//   RST_LOW        - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC      - reset type:  1 = asynchronous, 0 = synchronous
//   X_W, Y_W       - destination coordinate widths (derived, do not override)
//   FW, BAW, CAW   - flit width / buffer and CDC FIFO address widths (derived,
//                    do not override)
//
// Tile (y, x) injects with inj_* (dst_x/dst_y select the receiving tile) and
// receives with ej_*. Packets are single flits, routed XY (East/West first,
// then North/South). Packets between the same source and destination arrive
// in order. A tile must keep draining ej_* or the network backs up.
// tile_rst must be asserted together with rst.
//
// No memories are instantiated here. Every buffer is a FIFO whose storage is
// brought out on a memory port to be connected outside: one write port, and
// one read port whose data (mem_rd_data) is the word at mem_rd_addr *_RD_DEL
// read-clock cycles after mem_rd_en (same cycle when the latency is 0):
//   buf_mem_*[y][x][p] - BUF_DEPTH x FW on clk; p = 0..4 router input buffer
//                        (Local, N, E, S, W), p = 5 NI eject buffer
//   inj_mem_*[y][x]    - CDC_DEPTH x FW,     write tile_clk[y][x], read clk
//   ej_mem_*[y][x]     - CDC_DEPTH x DATA_W, write clk, read tile_clk[y][x]
// (with ASYNC = 0 every memory is on clk)

module sh_noc_mesh #(
  parameter int unsigned ROWS           = 2,
  parameter int unsigned COLS           = 2,
  parameter int unsigned DATA_W         = 32,
  parameter int unsigned BUF_DEPTH      = 4,
  parameter int unsigned BUF_MEM_RD_DEL = 0,
  parameter logic        ASYNC          = 1'b1,
  parameter int unsigned CDC_DEPTH      = 4,
  parameter int unsigned SYNC_DELAY     = 2,
  parameter int unsigned CDC_MEM_RD_DEL = 0,
  parameter logic        RST_LOW        = 1'b1,
  parameter logic        RST_ASYNC      = 1'b1,
  parameter int unsigned X_W            = (COLS > 1) ? $clog2(COLS) : 1,
  parameter int unsigned Y_W            = (ROWS > 1) ? $clog2(ROWS) : 1,
  parameter int unsigned FW             = Y_W + X_W + DATA_W,
  parameter int unsigned BAW            = (BUF_DEPTH > 1) ? $clog2(BUF_DEPTH) : 1,
  parameter int unsigned CAW            = (CDC_DEPTH > 1) ? $clog2(CDC_DEPTH) : 1
) (
  input  logic                                    clk,
  input  logic                                    rst,
  // Per-tile clocks and resets
  input  logic [ROWS-1:0][COLS-1:0]               tile_clk,
  input  logic [ROWS-1:0][COLS-1:0]               tile_rst,
  // Inject
  input  logic [ROWS-1:0][COLS-1:0]               inj_vld,
  output logic [ROWS-1:0][COLS-1:0]               inj_rdy,
  input  logic [ROWS-1:0][COLS-1:0][X_W-1:0]      inj_dst_x,
  input  logic [ROWS-1:0][COLS-1:0][Y_W-1:0]      inj_dst_y,
  input  logic [ROWS-1:0][COLS-1:0][DATA_W-1:0]   inj_data,
  // Eject
  output logic [ROWS-1:0][COLS-1:0]               ej_vld,
  input  logic [ROWS-1:0][COLS-1:0]               ej_rdy,
  output logic [ROWS-1:0][COLS-1:0][DATA_W-1:0]   ej_data,
  // Buffer memories (clk)
  output logic [ROWS-1:0][COLS-1:0][5:0]          buf_mem_wr_en,
  output logic [ROWS-1:0][COLS-1:0][5:0][BAW-1:0] buf_mem_wr_addr,
  output logic [ROWS-1:0][COLS-1:0][5:0][FW-1:0]  buf_mem_wr_data,
  output logic [ROWS-1:0][COLS-1:0][5:0]          buf_mem_rd_en,
  output logic [ROWS-1:0][COLS-1:0][5:0][BAW-1:0] buf_mem_rd_addr,
  input  logic [ROWS-1:0][COLS-1:0][5:0][FW-1:0]  buf_mem_rd_data,
  // Inject CDC FIFO memories
  output logic [ROWS-1:0][COLS-1:0]               inj_mem_wr_en,
  output logic [ROWS-1:0][COLS-1:0][CAW-1:0]      inj_mem_wr_addr,
  output logic [ROWS-1:0][COLS-1:0][FW-1:0]       inj_mem_wr_data,
  output logic [ROWS-1:0][COLS-1:0]               inj_mem_rd_en,
  output logic [ROWS-1:0][COLS-1:0][CAW-1:0]      inj_mem_rd_addr,
  input  logic [ROWS-1:0][COLS-1:0][FW-1:0]       inj_mem_rd_data,
  // Eject CDC FIFO memories
  output logic [ROWS-1:0][COLS-1:0]               ej_mem_wr_en,
  output logic [ROWS-1:0][COLS-1:0][CAW-1:0]      ej_mem_wr_addr,
  output logic [ROWS-1:0][COLS-1:0][DATA_W-1:0]   ej_mem_wr_data,
  output logic [ROWS-1:0][COLS-1:0]               ej_mem_rd_en,
  output logic [ROWS-1:0][COLS-1:0][CAW-1:0]      ej_mem_rd_addr,
  input  logic [ROWS-1:0][COLS-1:0][DATA_W-1:0]   ej_mem_rd_data
);

  localparam int unsigned P_L = 0;
  localparam int unsigned P_N = 1;
  localparam int unsigned P_E = 2;
  localparam int unsigned P_S = 3;
  localparam int unsigned P_W = 4;

  // Router port signals, [y][x][port]. Outputs on the mesh edge go nowhere.
  logic [4:0]          r_in_vld   [ROWS][COLS];
  logic [4:0][FW-1:0]  r_in_flit  [ROWS][COLS];
  /* verilator lint_off UNUSEDSIGNAL */
  logic [4:0]          r_in_crd   [ROWS][COLS];
  logic [4:0]          r_out_vld  [ROWS][COLS];
  logic [4:0][FW-1:0]  r_out_flit [ROWS][COLS];
  /* verilator lint_on UNUSEDSIGNAL */
  logic [4:0]          r_out_crd  [ROWS][COLS];

  for (genvar y = 0; y < ROWS; y++) begin : g_row
    for (genvar x = 0; x < COLS; x++) begin : g_col

      sh_noc_router #(
        .X_W            (X_W),
        .Y_W            (Y_W),
        .DATA_W         (DATA_W),
        .X              (x),
        .Y              (y),
        .BUF_DEPTH      (BUF_DEPTH),
        .BUF_MEM_RD_DEL (BUF_MEM_RD_DEL),
        .RST_LOW        (RST_LOW),
        .RST_ASYNC      (RST_ASYNC)
      ) u_router (
        .clk             (clk),
        .rst             (rst),
        .in_vld          (r_in_vld[y][x]),
        .in_flit         (r_in_flit[y][x]),
        .in_crd          (r_in_crd[y][x]),
        .out_vld         (r_out_vld[y][x]),
        .out_flit        (r_out_flit[y][x]),
        .out_crd         (r_out_crd[y][x]),
        .buf_mem_wr_en   (buf_mem_wr_en[y][x][4:0]),
        .buf_mem_wr_addr (buf_mem_wr_addr[y][x][4:0]),
        .buf_mem_wr_data (buf_mem_wr_data[y][x][4:0]),
        .buf_mem_rd_en   (buf_mem_rd_en[y][x][4:0]),
        .buf_mem_rd_addr (buf_mem_rd_addr[y][x][4:0]),
        .buf_mem_rd_data (buf_mem_rd_data[y][x][4:0])
      );

      sh_noc_ni #(
        .X_W            (X_W),
        .Y_W            (Y_W),
        .DATA_W         (DATA_W),
        .BUF_DEPTH      (BUF_DEPTH),
        .BUF_MEM_RD_DEL (BUF_MEM_RD_DEL),
        .ASYNC          (ASYNC),
        .CDC_DEPTH      (CDC_DEPTH),
        .SYNC_DELAY     (SYNC_DELAY),
        .CDC_MEM_RD_DEL (CDC_MEM_RD_DEL),
        .RST_LOW        (RST_LOW),
        .RST_ASYNC      (RST_ASYNC)
      ) u_ni (
        .clk             (clk),
        .rst             (rst),
        .tile_clk        (tile_clk[y][x]),
        .tile_rst        (tile_rst[y][x]),
        .inj_vld         (inj_vld[y][x]),
        .inj_rdy         (inj_rdy[y][x]),
        .inj_dst_x       (inj_dst_x[y][x]),
        .inj_dst_y       (inj_dst_y[y][x]),
        .inj_data        (inj_data[y][x]),
        .ej_vld          (ej_vld[y][x]),
        .ej_rdy          (ej_rdy[y][x]),
        .ej_data         (ej_data[y][x]),
        .rtr_in_vld      (r_in_vld[y][x][P_L]),
        .rtr_in_flit     (r_in_flit[y][x][P_L]),
        .rtr_in_crd      (r_in_crd[y][x][P_L]),
        .rtr_out_vld     (r_out_vld[y][x][P_L]),
        .rtr_out_flit    (r_out_flit[y][x][P_L]),
        .rtr_out_crd     (r_out_crd[y][x][P_L]),
        .ejb_mem_wr_en   (buf_mem_wr_en[y][x][5]),
        .ejb_mem_wr_addr (buf_mem_wr_addr[y][x][5]),
        .ejb_mem_wr_data (buf_mem_wr_data[y][x][5]),
        .ejb_mem_rd_en   (buf_mem_rd_en[y][x][5]),
        .ejb_mem_rd_addr (buf_mem_rd_addr[y][x][5]),
        .ejb_mem_rd_data (buf_mem_rd_data[y][x][5]),
        .inj_mem_wr_en   (inj_mem_wr_en[y][x]),
        .inj_mem_wr_addr (inj_mem_wr_addr[y][x]),
        .inj_mem_wr_data (inj_mem_wr_data[y][x]),
        .inj_mem_rd_en   (inj_mem_rd_en[y][x]),
        .inj_mem_rd_addr (inj_mem_rd_addr[y][x]),
        .inj_mem_rd_data (inj_mem_rd_data[y][x]),
        .ej_mem_wr_en    (ej_mem_wr_en[y][x]),
        .ej_mem_wr_addr  (ej_mem_wr_addr[y][x]),
        .ej_mem_wr_data  (ej_mem_wr_data[y][x]),
        .ej_mem_rd_en    (ej_mem_rd_en[y][x]),
        .ej_mem_rd_addr  (ej_mem_rd_addr[y][x]),
        .ej_mem_rd_data  (ej_mem_rd_data[y][x])
      );

      // Each link: flits from the neighbour's facing output, credits from the
      // neighbour's facing input. Edge ports are tied off.
      if (y < ROWS - 1) begin : g_n
        assign r_in_vld[y][x][P_N]  = r_out_vld[y+1][x][P_S];
        assign r_in_flit[y][x][P_N] = r_out_flit[y+1][x][P_S];
        assign r_out_crd[y][x][P_N] = r_in_crd[y+1][x][P_S];
      end else begin : g_n_edge
        assign r_in_vld[y][x][P_N]  = 1'b0;
        assign r_in_flit[y][x][P_N] = '0;
        assign r_out_crd[y][x][P_N] = 1'b0;
      end

      if (x < COLS - 1) begin : g_e
        assign r_in_vld[y][x][P_E]  = r_out_vld[y][x+1][P_W];
        assign r_in_flit[y][x][P_E] = r_out_flit[y][x+1][P_W];
        assign r_out_crd[y][x][P_E] = r_in_crd[y][x+1][P_W];
      end else begin : g_e_edge
        assign r_in_vld[y][x][P_E]  = 1'b0;
        assign r_in_flit[y][x][P_E] = '0;
        assign r_out_crd[y][x][P_E] = 1'b0;
      end

      if (y > 0) begin : g_s
        assign r_in_vld[y][x][P_S]  = r_out_vld[y-1][x][P_N];
        assign r_in_flit[y][x][P_S] = r_out_flit[y-1][x][P_N];
        assign r_out_crd[y][x][P_S] = r_in_crd[y-1][x][P_N];
      end else begin : g_s_edge
        assign r_in_vld[y][x][P_S]  = 1'b0;
        assign r_in_flit[y][x][P_S] = '0;
        assign r_out_crd[y][x][P_S] = 1'b0;
      end

      if (x > 0) begin : g_w
        assign r_in_vld[y][x][P_W]  = r_out_vld[y][x-1][P_E];
        assign r_in_flit[y][x][P_W] = r_out_flit[y][x-1][P_E];
        assign r_out_crd[y][x][P_W] = r_in_crd[y][x-1][P_E];
      end else begin : g_w_edge
        assign r_in_vld[y][x][P_W]  = 1'b0;
        assign r_in_flit[y][x][P_W] = '0;
        assign r_out_crd[y][x][P_W] = 1'b0;
      end

    end
  end

endmodule
