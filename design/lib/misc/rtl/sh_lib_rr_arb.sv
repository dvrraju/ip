// sh_lib_rr_arb: round-robin arbiter
//
// Parameters:
//   N         - number of requesters
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//
// gnt is one-hot (or zero when there are no requests) and combinational from
// req. Priority starts at the requester after the last one granted while adv
// was high; after reset requester 0 has the highest priority. Drive adv high
// when the grant is actually used, so an unused grant does not lose its turn.

module sh_lib_rr_arb #(
  parameter int unsigned N         = 4,
  parameter logic        RST_LOW   = 1'b1,
  parameter logic        RST_ASYNC = 1'b1
) (
  input  logic         clk,
  input  logic         rst,
  input  logic [N-1:0] req,
  input  logic         adv,
  output logic [N-1:0] gnt
);

  localparam int unsigned IW = (N > 1) ? $clog2(N) : 1;

  logic [IW-1:0] last;
  logic [IW-1:0] gnt_idx;

  always_comb begin
    int unsigned idx;
    gnt     = '0;
    gnt_idx = last;
    for (int unsigned k = 1; k <= N; k++) begin
      idx = 32'(last) + k;
      if (idx >= N) idx -= N;
      if (gnt == '0 && req[idx]) begin
        gnt[idx] = 1'b1;
        gnt_idx  = IW'(idx);
      end
    end
  end

  sh_lib_ff #(
    .WIDTH     (IW),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC),
    .RST_VAL   (IW'(N - 1))
  ) u_last (
    .clk (clk),
    .rst (rst),
    .en  (adv && (req != '0)),
    .d   (gnt_idx),
    .q   (last)
  );

endmodule
