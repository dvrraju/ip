// sh_noc_router: 5-port mesh router, XY routing, credit-based links
//
// Parameters:
//   X_W, Y_W       - widths of the destination x / y fields in a flit
//   DATA_W         - payload width in bits
//   X, Y           - this router's coordinates (x grows East, y grows North)
//   BUF_DEPTH      - input buffer entries per port; also the credit count each
//                    output starts with, so every receiver must use the same depth
//   BUF_MEM_RD_DEL - input buffer memory read latency; 0 = combinational
//   RST_LOW        - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC      - reset type:  1 = asynchronous, 0 = synchronous
//   FW, BAW        - flit width / buffer address width (derived, do not override)
//
// Buffer storage is external: buf_mem_*[p] is input port p's buffer memory
// (BUF_DEPTH x FW, one write port, one read port, clk).
//
// Ports are indexed 0 = Local, 1 = North, 2 = East, 3 = South, 4 = West.
// Flit layout: {dst_y, dst_x, data}. Every flit is a whole packet.
//
// Each input has a BUF_DEPTH buffer. A flit at the head of a buffer is routed
// X first, then Y, then to Local. Each output has a round-robin arbiter over
// the inputs, and sends (registered) when it holds a credit for the next
// router's buffer. Credits come back one per flit as that buffer drains.
//
// The credit loop is 4 cycles (BUF_MEM_RD_DEL + 5 with read latency), and
// BUF_DEPTH at least that keeps a link at full rate. XY routing has no cyclic channel dependencies, so the mesh is
// deadlock-free as long as every Local output is eventually drained.

module sh_noc_router #(
  parameter int unsigned X_W            = 2,
  parameter int unsigned Y_W            = 2,
  parameter int unsigned DATA_W         = 32,
  parameter int unsigned X              = 0,
  parameter int unsigned Y              = 0,
  parameter int unsigned BUF_DEPTH      = 4,
  parameter int unsigned BUF_MEM_RD_DEL = 0,
  parameter logic        RST_LOW        = 1'b1,
  parameter logic        RST_ASYNC      = 1'b1,
  parameter int unsigned FW             = Y_W + X_W + DATA_W,
  parameter int unsigned BAW            = (BUF_DEPTH > 1) ? $clog2(BUF_DEPTH) : 1
) (
  input  logic                clk,
  input  logic                rst,
  // Inputs: flits in, credits back to the upstream router
  input  logic [4:0]          in_vld,
  input  logic [4:0][FW-1:0]  in_flit,
  output logic [4:0]          in_crd,
  // Outputs: flits out, credits from the downstream router
  output logic [4:0]          out_vld,
  output logic [4:0][FW-1:0]  out_flit,
  input  logic [4:0]          out_crd,
  // Input buffer memories
  output logic [4:0]          buf_mem_wr_en,
  output logic [4:0][BAW-1:0] buf_mem_wr_addr,
  output logic [4:0][FW-1:0]  buf_mem_wr_data,
  output logic [4:0]          buf_mem_rd_en,
  output logic [4:0][BAW-1:0] buf_mem_rd_addr,
  input  logic [4:0][FW-1:0]  buf_mem_rd_data
);

  localparam int unsigned NP  = 5;
  localparam int unsigned P_L = 0;
  localparam int unsigned P_N = 1;
  localparam int unsigned P_E = 2;
  localparam int unsigned P_S = 3;
  localparam int unsigned P_W = 4;

  logic [NP-1:0]          buf_vld;
  logic [NP-1:0]          buf_pop;
  logic [NP-1:0][FW-1:0]  buf_flit;
  logic [NP-1:0][NP-1:0]  route;     // [input][output], one-hot per input
  logic [NP-1:0][NP-1:0]  req;       // [output][input]
  logic [NP-1:0][NP-1:0]  gnt;       // [output][input]
  logic [NP-1:0]          has_crd;
  logic [NP-1:0]          send;
  logic [NP-1:0][FW-1:0]  send_flit;

  // ---------------------------------------------------------------------------
  // Input buffers and route computation
  // ---------------------------------------------------------------------------

  for (genvar i = 0; i < NP; i++) begin : g_in
    logic [X_W-1:0] dst_x;
    logic [Y_W-1:0] dst_y;

    sh_noc_in_buf #(
      .WIDTH      (FW),
      .DEPTH      (BUF_DEPTH),
      .MEM_RD_DEL (BUF_MEM_RD_DEL),
      .RST_LOW    (RST_LOW),
      .RST_ASYNC  (RST_ASYNC)
    ) u_buf (
      .clk         (clk),
      .rst         (rst),
      .in_vld      (in_vld[i]),
      .in_flit     (in_flit[i]),
      .crd         (in_crd[i]),
      .out_vld     (buf_vld[i]),
      .out_rdy     (buf_pop[i]),
      .out_flit    (buf_flit[i]),
      .mem_wr_en   (buf_mem_wr_en[i]),
      .mem_wr_addr (buf_mem_wr_addr[i]),
      .mem_wr_data (buf_mem_wr_data[i]),
      .mem_rd_en   (buf_mem_rd_en[i]),
      .mem_rd_addr (buf_mem_rd_addr[i]),
      .mem_rd_data (buf_mem_rd_data[i])
    );

    assign dst_x = buf_flit[i][DATA_W +: X_W];
    assign dst_y = buf_flit[i][DATA_W + X_W +: Y_W];

    // On edge routers some of these compares are constant (e.g. dst_x < 0)
    /* verilator lint_off CMPCONST */
    /* verilator lint_off UNSIGNED */
    always_comb begin
      route[i] = '0;
      if      (dst_x > X_W'(X)) route[i][P_E] = 1'b1;
      else if (dst_x < X_W'(X)) route[i][P_W] = 1'b1;
      else if (dst_y > Y_W'(Y)) route[i][P_N] = 1'b1;
      else if (dst_y < Y_W'(Y)) route[i][P_S] = 1'b1;
      else                      route[i][P_L] = 1'b1;
    end
    /* verilator lint_on UNSIGNED */
    /* verilator lint_on CMPCONST */

    always_comb begin
      buf_pop[i] = 1'b0;
      for (int o = 0; o < NP; o++) buf_pop[i] |= gnt[o][i];
    end
  end

  // ---------------------------------------------------------------------------
  // Output arbitration and link registers
  // ---------------------------------------------------------------------------

  for (genvar o = 0; o < NP; o++) begin : g_out
    always_comb begin
      for (int i = 0; i < NP; i++) req[o][i] = buf_vld[i] && route[i][o] && has_crd[o];
    end

    sh_lib_rr_arb #(
      .N         (NP),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_arb (
      .clk (clk),
      .rst (rst),
      .req (req[o]),
      .adv (1'b1),
      .gnt (gnt[o])
    );

    assign send[o] = (gnt[o] != '0);

    always_comb begin
      send_flit[o] = '0;
      for (int i = 0; i < NP; i++) send_flit[o] |= {FW{gnt[o][i]}} & buf_flit[i];
    end

    sh_noc_crd_cnt #(
      .DEPTH     (BUF_DEPTH),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_crd_cnt (
      .clk     (clk),
      .rst     (rst),
      .send    (send[o]),
      .crd     (out_crd[o]),
      .has_crd (has_crd[o])
    );

    sh_lib_ff #(
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_vld (
      .clk (clk),
      .rst (rst),
      .en  (1'b0),
      .d   (send[o]),
      .q   (out_vld[o])
    );

    sh_lib_ff #(
      .WIDTH  (FW),
      .EN     (1'b1),
      .RST_EN (1'b0)
    ) u_flit (
      .clk (clk),
      .rst (1'b0),
      .en  (send[o]),
      .d   (send_flit[o]),
      .q   (out_flit[o])
    );
  end

endmodule
