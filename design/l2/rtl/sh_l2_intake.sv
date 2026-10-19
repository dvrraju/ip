// sh_l2_intake: collects one complete agent request for the L2
//
// Parameters:
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//
// Takes a request packet from t_req: the header, then for a write its data
// flits into a WORDS-entry write buffer (data and strobes, in flops; entry i
// holds data flit i). Once the tail flit is in, req_vld goes high and stays
// high, with t_req stalled, until the controller pulses req_done.
//
//   req_bad  - not a read or write request, or it crosses a line
//              (first word + len > WORDS - 1); data flits beyond the buffer
//              are consumed and dropped
//   req_full - a write that covers the whole line with every strobe set

module sh_l2_intake
  import sh_noc_pkg::*;
  import sh_l2_pkg::*;
#(
  parameter logic RST_LOW   = 1'b1,
  parameter logic RST_ASYNC = 1'b1
) (
  input  logic                         clk,
  input  logic                         rst,
  // Requests from the NI
  input  logic                         t_req_vld,
  output logic                         t_req_rdy,
  input  flit_t                        t_req_flit,
  // Buffered request
  output logic                         req_vld,
  output hdr_t                         req_hdr,
  output logic                         req_bad,
  output logic                         req_full,
  output logic [WORDS-1:0][FLIT_W-1:0] wbuf_data,
  output logic [WORDS-1:0][STRB_W-1:0] wbuf_strb,
  input  logic                         req_done
);

  localparam int unsigned BEAT_W = LEN_W + 1;

  logic              take;
  logic [BEAT_W-1:0] beat;     // index of the next data flit
  logic              strb_all;

  assign t_req_rdy = !req_vld;
  assign take      = t_req_vld && t_req_rdy;

  // Complete request held until the controller is done with it
  sh_lib_ff #(
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_vld (
    .clk (clk),
    .rst (rst),
    .en  ((take && t_req_flit.tail) || req_done),
    .d   (!req_done),
    .q   (req_vld)
  );

  sh_lib_ff #(
    .WIDTH  ($bits(hdr_t)),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_hdr (
    .clk (clk),
    .rst (1'b0),
    .en  (take && t_req_flit.head),
    .d   (get_hdr(t_req_flit)),
    .q   (req_hdr)
  );

  sh_lib_ff #(
    .WIDTH     (BEAT_W),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_beat (
    .clk (clk),
    .rst (rst),
    .en  (take),
    .d   (t_req_flit.head ? '0 : beat + 1'b1),
    .q   (beat)
  );

  // Write buffer
  for (genvar i = 0; i < WORDS; i++) begin : g_wbuf
    sh_lib_ff #(
      .WIDTH  (STRB_W + FLIT_W),
      .EN     (1'b1),
      .RST_EN (1'b0)
    ) u_entry (
      .clk (clk),
      .rst (1'b0),
      .en  (take && !t_req_flit.head && beat == BEAT_W'(i)),
      .d   ({t_req_flit.strb, t_req_flit.data}),
      .q   ({wbuf_strb[i], wbuf_data[i]})
    );
  end

  // Request checks
  always_comb begin
    strb_all = 1'b1;
    for (int i = 0; i < WORDS; i++) strb_all &= (wbuf_strb[i] == '1);
  end

  assign req_bad  = !(req_hdr.typ inside {PKT_RD_REQ, PKT_WR_REQ})
                    || (32'(addr_word(req_hdr.addr)) + 32'(req_hdr.len) > WORDS - 1);
  assign req_full = (req_hdr.typ == PKT_WR_REQ) && (addr_word(req_hdr.addr) == '0)
                    && (32'(req_hdr.len) == WORDS - 1) && strb_all;

endmodule
