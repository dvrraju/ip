// sh_lib_sync_bus_en: multi-bit synchronizer with a source-side enable pulse
//
// Parameters:
//   WIDTH     - data width in bits
//   DELAY     - number of synchronizer flops on the req/ack paths (minimum 2)
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high (both domains)
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous (both domains)
//   RST_VAL   - value of the data registers while reset is asserted
//
// A src_en pulse while src_busy is low captures src_data and starts a
// toggle req/ack handshake. dst_data is loaded from the captured value once
// the req reaches the destination, and dst_vld pulses for one dst_clk cycle
// with the new data. src_busy is high from the cycle after src_en until the
// ack returns; src_en pulses while busy are ignored.
//
// The captured data is held stable while it crosses, so the src_data_q ->
// dst_data path is not a synchronizer and should be constrained (e.g.
// max delay) rather than timed as a normal path. Reset both domains together.
// All flops are always reset (there is no RST_EN): the req/ack handshake must
// start from a known state or it can lock up.

module sh_lib_sync_bus_en #(
  parameter int unsigned      WIDTH     = 1,
  parameter int unsigned      DELAY     = 3,
  parameter logic             RST_LOW   = 1'b1,
  parameter logic             RST_ASYNC = 1'b1,
  parameter logic [WIDTH-1:0] RST_VAL   = '0
) (
  // Source domain
  input  logic             src_clk,
  input  logic             src_rst,
  input  logic             src_en,
  input  logic [WIDTH-1:0] src_data,
  output logic             src_busy,
  // Destination domain
  input  logic             dst_clk,
  input  logic             dst_rst,
  output logic [WIDTH-1:0] dst_data,
  output logic             dst_vld
);

  // ---------------------------------------------------------------------------
  // Source domain
  // ---------------------------------------------------------------------------

  logic             src_load;
  logic             src_req;
  logic             src_ack;
  logic [WIDTH-1:0] src_data_q;

  assign src_load = src_en && !src_busy;
  assign src_busy = src_req ^ src_ack;

  // Toggles once per accepted transfer
  sh_lib_ff #(
    .EN        (1'b1),
    .RST_EN    (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_src_req (
    .clk (src_clk),
    .rst (src_rst),
    .en  (src_load),
    .d   (!src_req),
    .q   (src_req)
  );

  // Holds the data stable while it crosses to the destination
  sh_lib_ff #(
    .WIDTH     (WIDTH),
    .EN        (1'b1),
    .RST_EN    (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC),
    .RST_VAL   (RST_VAL)
  ) u_src_data (
    .clk (src_clk),
    .rst (src_rst),
    .en  (src_load),
    .d   (src_data),
    .q   (src_data_q)
  );

  // ---------------------------------------------------------------------------
  // Destination domain
  // ---------------------------------------------------------------------------

  logic dst_req;
  logic dst_req_q;
  logic dst_load;

  sh_lib_sync #(
    .DELAY     (DELAY),
    .RST_EN    (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_req_sync (
    .clk (dst_clk),
    .rst (dst_rst),
    .d   (src_req),
    .q   (dst_req)
  );

  // Edge detect on the synchronized req; dst_req_q is also the ack
  sh_lib_ff #(
    .RST_EN    (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_dst_req_q (
    .clk (dst_clk),
    .rst (dst_rst),
    .en  (1'b0),
    .d   (dst_req),
    .q   (dst_req_q)
  );

  assign dst_load = dst_req ^ dst_req_q;

  sh_lib_ff #(
    .WIDTH     (WIDTH),
    .EN        (1'b1),
    .RST_EN    (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC),
    .RST_VAL   (RST_VAL)
  ) u_dst_data (
    .clk (dst_clk),
    .rst (dst_rst),
    .en  (dst_load),
    .d   (src_data_q),
    .q   (dst_data)
  );

  // Pulses in the same cycle dst_data shows the new value
  sh_lib_ff #(
    .RST_EN    (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_dst_valid (
    .clk (dst_clk),
    .rst (dst_rst),
    .en  (1'b0),
    .d   (dst_load),
    .q   (dst_vld)
  );

  // ---------------------------------------------------------------------------
  // Ack back to the source domain
  // ---------------------------------------------------------------------------

  sh_lib_sync #(
    .DELAY     (DELAY),
    .RST_EN    (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_ack_sync (
    .clk (src_clk),
    .rst (src_rst),
    .d   (dst_req_q),
    .q   (src_ack)
  );

endmodule
