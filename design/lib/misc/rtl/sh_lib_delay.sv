// sh_lib_delay: delays a bus by a fixed number of clock cycles
//
// Parameters:
//   WIDTH     - data width in bits
//   DELAY     - number of pipe stages; 0 = q is d (combinational pass-through)
//   RST_EN    - reset:       1 = resettable stages, 0 = non-resettable (rst is ignored)
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//   RST_VAL   - value loaded into every stage while reset is asserted

module sh_lib_delay #(
  parameter int unsigned      WIDTH     = 1,
  parameter int unsigned      DELAY     = 1,
  parameter logic             RST_EN    = 1'b1,
  parameter logic             RST_LOW   = 1'b1,
  parameter logic             RST_ASYNC = 1'b1,
  parameter logic [WIDTH-1:0] RST_VAL   = '0
) (
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic             clk,
  input  logic             rst,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [WIDTH-1:0] d,
  output logic [WIDTH-1:0] q
);

  // pipe[0] is the input; pipe[i] is the output of stage i
  logic [WIDTH-1:0] pipe [DELAY+1];

  assign pipe[0] = d;

  for (genvar i = 1; i <= DELAY; i++) begin : g_stage
    sh_lib_ff #(
      .WIDTH     (WIDTH),
      .RST_EN    (RST_EN),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC),
      .RST_VAL   (RST_VAL)
    ) u_ff (
      .clk (clk),
      .rst (rst),
      .en  (1'b0),
      .d   (pipe[i-1]),
      .q   (pipe[i])
    );
  end

  assign q = pipe[DELAY];

endmodule
