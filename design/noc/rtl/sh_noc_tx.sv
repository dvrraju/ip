// sh_noc_tx: credit-based link transmitter
//
// Parameters:
//   WIDTH     - flit width in bits
//   DEPTH     - receiver buffer entries (credits at reset)
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//
// in_rdy is high while a credit is left; a flit is sent (in_vld && in_rdy)
// on the registered link one cycle later. The receiver (sh_noc_in_buf)
// returns one credit on crd per flit it drains.

module sh_noc_tx #(
  parameter int unsigned WIDTH     = 8,
  parameter int unsigned DEPTH     = 4,
  parameter logic        RST_LOW   = 1'b1,
  parameter logic        RST_ASYNC = 1'b1
) (
  input  logic             clk,
  input  logic             rst,
  // Flits to send
  input  logic             in_vld,
  output logic             in_rdy,
  input  logic [WIDTH-1:0] in_flit,
  // Link to the receiver
  output logic             out_vld,
  output logic [WIDTH-1:0] out_flit,
  input  logic             crd
);

  logic send;

  assign send = in_vld && in_rdy;

  sh_noc_crd_cnt #(
    .DEPTH     (DEPTH),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_crd_cnt (
    .clk     (clk),
    .rst     (rst),
    .send    (send),
    .crd     (crd),
    .has_crd (in_rdy)
  );

  sh_lib_ff #(
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_vld (
    .clk (clk),
    .rst (rst),
    .en  (1'b0),
    .d   (send),
    .q   (out_vld)
  );

  sh_lib_ff #(
    .WIDTH  (WIDTH),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_flit (
    .clk (clk),
    .rst (1'b0),
    .en  (send),
    .d   (in_flit),
    .q   (out_flit)
  );

endmodule
