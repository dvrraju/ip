// sh_noc_router: 5-port wormhole mesh router, XY routing, credit-based links
//
// Parameters:
//   X, Y           - this router's coordinates (x grows East, y grows North)
//   BUF_DEPTH      - input buffer entries per port; also the credit count each
//                    output starts with, so every receiver must use the same depth
//   BUF_MEM_RD_DEL - input buffer memory read latency; 0 = combinational
//   RST_LOW        - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC      - reset type:  1 = asynchronous, 0 = synchronous
//   BAW            - buffer address width (derived, do not override)
//
// Flits and packets are as defined in sh_noc_pkg. Ports are indexed
// 0 = Local, 1 = North, 2 = East, 3 = South, 4 = West.
//
// Buffer storage is external: buf_mem_*[p] is input port p's buffer memory
// (BUF_DEPTH x FLIT_LINK_W, one write port, one read port, clk).
//
// Each input has a BUF_DEPTH buffer. A head flit is routed X first, then Y,
// then to Local, and the route is kept for the rest of its packet. Each
// output arbitrates round-robin among the head flits routed to it; the winner
// keeps the output until its tail flit has gone (wormhole), so packets never
// interleave on a link. An output sends (registered) only when it holds a
// credit for the next buffer.
//
// The credit loop is 4 cycles (BUF_MEM_RD_DEL + 5 with read latency), and
// BUF_DEPTH at least that keeps a link at full rate. Buffers need not hold a
// whole packet. XY routing has no cyclic channel dependencies, so a network
// is deadlock-free as long as every Local output is eventually drained.

module sh_noc_router
  import sh_noc_pkg::*;
#(
  parameter int unsigned X              = 0,
  parameter int unsigned Y              = 0,
  parameter int unsigned BUF_DEPTH      = 4,
  parameter int unsigned BUF_MEM_RD_DEL = 0,
  parameter logic        RST_LOW        = 1'b1,
  parameter logic        RST_ASYNC      = 1'b1,
  parameter int unsigned BAW            = (BUF_DEPTH > 1) ? $clog2(BUF_DEPTH) : 1
) (
  input  logic                         clk,
  input  logic                         rst,
  // Inputs: flits in, credits back to the upstream router
  input  logic  [4:0]                  in_vld,
  input  flit_t [4:0]                  in_flit,
  output logic  [4:0]                  in_crd,
  // Outputs: flits out, credits from the downstream router
  output logic  [4:0]                  out_vld,
  output flit_t [4:0]                  out_flit,
  input  logic  [4:0]                  out_crd,
  // Input buffer memories
  output logic  [4:0]                  buf_mem_wr_en,
  output logic  [4:0][BAW-1:0]         buf_mem_wr_addr,
  output logic  [4:0][FLIT_LINK_W-1:0] buf_mem_wr_data,
  output logic  [4:0]                  buf_mem_rd_en,
  output logic  [4:0][BAW-1:0]         buf_mem_rd_addr,
  input  logic  [4:0][FLIT_LINK_W-1:0] buf_mem_rd_data
);

  localparam int unsigned NP  = 5;
  localparam int unsigned P_L = 0;
  localparam int unsigned P_N = 1;
  localparam int unsigned P_E = 2;
  localparam int unsigned P_S = 3;
  localparam int unsigned P_W = 4;

  logic  [NP-1:0]         buf_vld;
  logic  [NP-1:0]         buf_pop;
  flit_t [NP-1:0]         buf_flit;
  logic  [NP-1:0][NP-1:0] route;     // [input][output], one-hot per input
  logic  [NP-1:0][NP-1:0] gnt;       // [output][input]
  logic  [NP-1:0]         has_crd;
  logic  [NP-1:0]         send;
  flit_t [NP-1:0]         send_flit;

  // ---------------------------------------------------------------------------
  // Input buffers and route computation
  // ---------------------------------------------------------------------------

  for (genvar i = 0; i < NP; i++) begin : g_in
    /* verilator lint_off UNUSEDSIGNAL */
    hdr_t          hdr;        // only dst_x/dst_y are used
    /* verilator lint_on UNUSEDSIGNAL */
    logic [NP-1:0] xy_route;   // route of the head flit at the buffer output
    logic [NP-1:0] pkt_route;  // route of the packet in progress

    sh_noc_in_buf #(
      .WIDTH      (FLIT_LINK_W),
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

    assign hdr = get_hdr(buf_flit[i]);

    // On edge routers some of these compares are constant (e.g. dst_x < 0)
    /* verilator lint_off CMPCONST */
    /* verilator lint_off UNSIGNED */
    always_comb begin
      xy_route = '0;
      if      (hdr.dst_x > X_W'(X)) xy_route[P_E] = 1'b1;
      else if (hdr.dst_x < X_W'(X)) xy_route[P_W] = 1'b1;
      else if (hdr.dst_y > Y_W'(Y)) xy_route[P_N] = 1'b1;
      else if (hdr.dst_y < Y_W'(Y)) xy_route[P_S] = 1'b1;
      else                          xy_route[P_L] = 1'b1;
    end
    /* verilator lint_on UNSIGNED */
    /* verilator lint_on CMPCONST */

    // Data flits follow the route their head flit took
    sh_lib_ff #(
      .WIDTH     (NP),
      .EN        (1'b1),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_pkt_route (
      .clk (clk),
      .rst (rst),
      .en  (buf_pop[i] && buf_flit[i].head),
      .d   (xy_route),
      .q   (pkt_route)
    );

    assign route[i] = buf_flit[i].head ? xy_route : pkt_route;

    always_comb begin
      buf_pop[i] = 1'b0;
      for (int o = 0; o < NP; o++) buf_pop[i] |= gnt[o][i];
    end
  end

  // ---------------------------------------------------------------------------
  // Output allocation, arbitration and links
  // ---------------------------------------------------------------------------

  for (genvar o = 0; o < NP; o++) begin : g_out
    logic          locked;     // a packet holds this output
    logic [NP-1:0] owner;      // input that holds it (one-hot)
    logic [NP-1:0] head_req;
    logic [NP-1:0] arb_gnt;

    // New packets compete only while the output is free
    always_comb begin
      for (int i = 0; i < NP; i++) begin
        head_req[i] = !locked && has_crd[o] && buf_vld[i] && buf_flit[i].head && route[i][o];
      end
    end

    sh_lib_rr_arb #(
      .N         (NP),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_arb (
      .clk (clk),
      .rst (rst),
      .req (head_req),
      .adv (1'b1),
      .gnt (arb_gnt)
    );

    assign gnt[o]  = locked ? (owner & buf_vld & {NP{has_crd[o]}}) : arb_gnt;
    assign send[o] = (gnt[o] != '0);

    always_comb begin
      send_flit[o] = '0;
      for (int i = 0; i < NP; i++) send_flit[o] |= {FLIT_LINK_W{gnt[o][i]}} & buf_flit[i];
    end

    // Held from a head flit until its tail flit is sent
    sh_lib_ff #(
      .EN        (1'b1),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_locked (
      .clk (clk),
      .rst (rst),
      .en  (send[o]),
      .d   (!send_flit[o].tail),
      .q   (locked)
    );

    sh_lib_ff #(
      .WIDTH     (NP),
      .EN        (1'b1),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_owner (
      .clk (clk),
      .rst (rst),
      .en  (send[o]),
      .d   (gnt[o]),
      .q   (owner)
    );

    sh_noc_tx #(
      .WIDTH     (FLIT_LINK_W),
      .DEPTH     (BUF_DEPTH),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_tx (
      .clk      (clk),
      .rst      (rst),
      .in_vld   (send[o]),
      .in_rdy   (has_crd[o]),
      .in_flit  (send_flit[o]),
      .out_vld  (out_vld[o]),
      .out_flit (out_flit[o]),
      .crd      (out_crd[o])
    );
  end

endmodule
