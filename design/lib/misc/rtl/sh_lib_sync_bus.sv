// sh_lib_sync_bus: multi-bit synchronizer, one sh_lib_sync per bit
//
// Parameters:
//   WIDTH     - number of bits
//   DELAY     - number of synchronizer flops per bit (minimum 2)
//   RST_EN    - reset:       1 = resettable stages, 0 = non-resettable (rst is ignored)
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//   RST_VAL   - per-bit value loaded into every stage while reset is asserted
//
// Each bit is synchronized independently, so bits that change together may
// arrive in different cycles. Only use this for independent bits, gray-coded
// values, or values held stable long enough to settle.

module sh_lib_sync_bus #(
  parameter int unsigned      WIDTH     = 1,
  parameter int unsigned      DELAY     = 3,
  parameter logic             RST_EN    = 1'b1,
  parameter logic             RST_LOW   = 1'b1,
  parameter logic             RST_ASYNC = 1'b1,
  parameter logic [WIDTH-1:0] RST_VAL   = '0
) (
  input  logic             clk,
  input  logic             rst,
  input  logic [WIDTH-1:0] d,
  output logic [WIDTH-1:0] q
);

  for (genvar i = 0; i < WIDTH; i++) begin : g_bit
    sh_lib_sync #(
      .DELAY     (DELAY),
      .RST_EN    (RST_EN),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC),
      .RST_VAL   (RST_VAL[i])
    ) u_sync (
      .clk (clk),
      .rst (rst),
      .d   (d[i]),
      .q   (q[i])
    );
  end

endmodule
