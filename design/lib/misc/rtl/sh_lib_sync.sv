// sh_lib_sync: single-bit multi-stage synchronizer
//
// Parameters:
//   DELAY     - number of synchronizer flops (minimum 2)
//   RST_EN    - reset:       1 = resettable stages, 0 = non-resettable (rst is ignored)
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//   RST_VAL   - value loaded into every stage while reset is asserted
//
// d is asynchronous to clk; q is d synchronized to clk after DELAY cycles.

module sh_lib_sync #(
  parameter int unsigned DELAY     = 3,
  parameter logic        RST_EN    = 1'b1,
  parameter logic        RST_LOW   = 1'b1,
  parameter logic        RST_ASYNC = 1'b1,
  parameter logic        RST_VAL   = 1'b0
) (
  input  logic clk,
  input  logic rst,
  input  logic d,
  output logic q
);

  if (DELAY < 2) begin : g_check
    $error("sh_lib_sync: DELAY must be at least 2, got %0d", DELAY);
  end

  // Stage 0 samples d; each later stage samples the one before it.
  logic [DELAY-1:0] sync_q;

  sh_lib_ff #(
    .WIDTH     (DELAY),
    .EN        (1'b0),
    .RST_EN    (RST_EN),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC),
    .RST_VAL   ({DELAY{RST_VAL}})
  ) u_sync_ff (
    .clk (clk),
    .rst (rst),
    .en  (1'b0),
    .d   ({sync_q[DELAY-2:0], d}),
    .q   (sync_q)
  );

  assign q = sync_q[DELAY-1];

endmodule
