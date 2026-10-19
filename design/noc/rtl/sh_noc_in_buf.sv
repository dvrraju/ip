// sh_noc_in_buf: credit-based link receiver (input buffer)
//
// Parameters:
//   WIDTH      - flit width in bits
//   DEPTH      - buffer entries; the sender's credit counter must start at DEPTH
//   MEM_RD_DEL - buffer memory read latency in clk cycles; 0 = combinational
//   RST_LOW    - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC  - reset type:  1 = asynchronous, 0 = synchronous
//   AW         - memory address width (derived from DEPTH, do not override)
//
// Storage is external: mem_* is the sh_lib_fifo memory interface (one write
// port, one read port with MEM_RD_DEL latency, DEPTH x WIDTH, clocked by clk).
//
// in_vld has no ready: the sender only sends when it holds a credit, so the
// buffer always has room. Each flit popped from the buffer returns one credit
// on crd (registered, one-cycle pulse).

module sh_noc_in_buf #(
  parameter int unsigned WIDTH      = 8,
  parameter int unsigned DEPTH      = 4,
  parameter int unsigned MEM_RD_DEL = 0,
  parameter logic        RST_LOW    = 1'b1,
  parameter logic        RST_ASYNC  = 1'b1,
  parameter int unsigned AW         = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic             clk,
  input  logic             rst,
  // Link from the sender
  input  logic             in_vld,
  input  logic [WIDTH-1:0] in_flit,
  output logic             crd,
  // Buffered flits
  output logic             out_vld,
  input  logic             out_rdy,
  output logic [WIDTH-1:0] out_flit,
  // Memory interface
  output logic             mem_wr_en,
  output logic [AW-1:0]    mem_wr_addr,
  output logic [WIDTH-1:0] mem_wr_data,
  output logic             mem_rd_en,
  output logic [AW-1:0]    mem_rd_addr,
  input  logic [WIDTH-1:0] mem_rd_data
);

  /* verilator lint_off UNUSEDSIGNAL */
  logic wr_rdy;   // only checked in simulation
  /* verilator lint_on UNUSEDSIGNAL */

  sh_lib_fifo #(
    .WIDTH      (WIDTH),
    .DEPTH      (DEPTH),
    .MEM_RD_DEL (MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_fifo (
    .clk         (clk),
    .rst         (rst),
    .wr_vld      (in_vld),
    .wr_rdy      (wr_rdy),
    .wr_data     (in_flit),
    .rd_vld      (out_vld),
    .rd_rdy      (out_rdy),
    .rd_data     (out_flit),
    .mem_wr_en   (mem_wr_en),
    .mem_wr_addr (mem_wr_addr),
    .mem_wr_data (mem_wr_data),
    .mem_rd_en   (mem_rd_en),
    .mem_rd_addr (mem_rd_addr),
    .mem_rd_data (mem_rd_data)
  );

  sh_lib_ff #(
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_crd (
    .clk (clk),
    .rst (rst),
    .en  (1'b0),
    .d   (out_vld && out_rdy),
    .q   (crd)
  );

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (in_vld && !wr_rdy) $error("%m: flit received with no buffer space (credit overflow)");
  end
`endif

endmodule
