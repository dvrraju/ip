// sh_lib_ff: generic parameterized flip-flop
//
// Parameters:
//   WIDTH     - data width in bits
//   EN        - enable:      1 = q loads d only when en is high, 0 = en is ignored
//   RST_EN    - reset:       1 = resettable flop, 0 = non-resettable (rst is ignored)
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//   RST_VAL   - value loaded into q while reset is asserted
//
// Reset takes priority over enable. Tie off rst/en when they are disabled.

module sh_lib_ff #(
  parameter int unsigned      WIDTH     = 1,
  parameter bit               EN        = 1'b0,
  parameter bit               RST_EN    = 1'b1,
  parameter bit               RST_LOW   = 1'b1,
  parameter bit               RST_ASYNC = 1'b1,
  parameter logic [WIDTH-1:0] RST_VAL   = '0
) (
  input  logic             clk,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic             rst,
  input  logic             en,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [WIDTH-1:0] d,
  output logic [WIDTH-1:0] q
);

  logic [WIDTH-1:0] d_nxt;

  if (EN) begin : g_en
    assign d_nxt = en ? d : q;
  end else begin : g_no_en
    assign d_nxt = d;
  end

  // The reset edge in an async sensitivity list must be a literal, so each
  // sense/type combination gets its own always_ff block.
  if (!RST_EN) begin : g_no_rst
    always_ff @(posedge clk) begin
      q <= d_nxt;
    end
  end else if (RST_ASYNC && RST_LOW) begin : g_async_low
    always_ff @(posedge clk or negedge rst) begin
      if (!rst) q <= RST_VAL;
      else      q <= d_nxt;
    end
  end else if (RST_ASYNC) begin : g_async_high
    always_ff @(posedge clk or posedge rst) begin
      if (rst) q <= RST_VAL;
      else     q <= d_nxt;
    end
  end else begin : g_sync
    localparam bit RST_ACT = !RST_LOW;
    always_ff @(posedge clk) begin
      if (rst == RST_ACT) q <= RST_VAL;
      else                q <= d_nxt;
    end
  end

endmodule
