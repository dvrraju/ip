// sh_noc_crd_cnt: credit counter for the sending side of a credit-based link
//
// Parameters:
//   DEPTH     - receiver buffer entries (initial and maximum credit count)
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//
// send takes one credit, crd returns one. has_crd is high while at least one
// credit is left; only send when it is high.

module sh_noc_crd_cnt #(
  parameter int unsigned DEPTH     = 4,
  parameter logic        RST_LOW   = 1'b1,
  parameter logic        RST_ASYNC = 1'b1
) (
  input  logic clk,
  input  logic rst,
  input  logic send,
  input  logic crd,
  output logic has_crd
);

  localparam int unsigned CW = $clog2(DEPTH + 1);

  logic [CW-1:0] cnt;

  assign has_crd = (cnt != '0);

  sh_lib_ff #(
    .WIDTH     (CW),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC),
    .RST_VAL   (CW'(DEPTH))
  ) u_cnt (
    .clk (clk),
    .rst (rst),
    .en  (send ^ crd),
    .d   (send ? cnt - 1'b1 : cnt + 1'b1),
    .q   (cnt)
  );

`ifndef SYNTHESIS
  always_ff @(posedge clk) begin
    if (send && !has_crd)                  $error("%m: send with no credit");
    if (crd && !send && cnt == CW'(DEPTH)) $error("%m: credit returned with counter full");
  end
`endif

endmodule
