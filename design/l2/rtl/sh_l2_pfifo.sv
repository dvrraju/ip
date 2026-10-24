// sh_l2_pfifo: L2 pending FIFO (circular FIFO of PF_DEPTH entries)
//
// Parameters:
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//
// Each command (request) from t_req is written into the entry at the tail of
// the FIFO: its header into slot 0 of the external pending FIFO memory and
// its data flits into slots 1 .. WORDS (data flits beyond a line are
// dropped; such a command crosses a line and is rejected later). The entry's
// state (ent_t) is kept here in flops and goes from LOAD to NEW when the last
// flit is in. t_req stalls when every entry is in use.
//
// Dependencies: when a command is accepted, the FIFO is looked up for
// pending commands to the same line. If there are any, the new command's
// dep flag is set with dep_id = the youngest of them, so the commands to a
// line form a chain and each depends on one earlier command. A command with
// dep set is not picked. The flag clears when the command it depends on
// completes (its entry is freed after its response, whatever the status).
//
// Entries are freed in any order (state FREE). The head moves past freed
// entries; a freed entry is reused only once the head has passed it, and not
// while a writeback it caused is still outstanding (its NoC tag is the
// entry index).
//
// The controller changes an entry with upd_*. On top of that, events wake
// parked entries (also when the controller parks an entry in the same
// cycle):
//   upd_*     - an entry is freed: every entry depending on it clears dep
//   inst_*    - the fill of entry inst_idx is done (inst_err: it failed):
//               that entry FILL -> READY
//   wbr_*     - the writeback caused by entry wbr_idx is done: its wb_pend
//               clears, and every entry in WB_WAIT on it -> NEEDFILL
//   release   - a reserved way was released: every NO_WAY entry -> NEW

module sh_l2_pfifo
  import sh_noc_pkg::*;
  import sh_l2_pkg::*;
#(
  parameter logic RST_LOW   = 1'b1,
  parameter logic RST_ASYNC = 1'b1
) (
  input  logic                         clk,
  input  logic                         rst,
  // Commands from the NI
  input  logic                         t_req_vld,
  output logic                         t_req_rdy,
  input  flit_t                        t_req_flit,
  // Pending FIFO memory, write side
  output logic                         pf_mem_wr_en,
  output logic [PF_IW-1:0]             pf_mem_wr_addr,
  output logic [PF_ENTRY_W-1:0]        pf_mem_wr_data,
  output logic [PF_SLOTS-1:0]          pf_mem_wr_mask,
  // Entry state
  output ent_t [PF_DEPTH-1:0]          ent,
  output logic [PF_IW-1:0]             head,
  // Updates
  input  logic                         upd_en,
  input  logic [PF_IW-1:0]             upd_idx,
  input  ent_t                         upd_ent,
  input  logic                         inst_en,
  input  logic [PF_IW-1:0]             inst_idx,
  input  logic                         inst_err,
  input  logic                         wbr_en,
  input  logic [PF_IW-1:0]             wbr_idx,
  input  logic                         release_en
);

  if (PF_DEPTH < 2 || (1 << PF_IW) != PF_DEPTH || PF_DEPTH > (1 << TAG_W)) begin : g_check
    $error("sh_l2_pfifo: PF_DEPTH must be a power of 2 between 2 and 2**TAG_W, got %0d", PF_DEPTH);
  end

  localparam int unsigned BEAT_W = LEN_W + 1;

  // ---------------------------------------------------------------------------
  // Pointers and intake
  // ---------------------------------------------------------------------------

  logic [PF_IW:0]     head_p;     // with a wrap bit
  logic [PF_IW:0]     tail_p;
  logic                full;
  logic                loading;    // a request's data flits are arriving
  logic [PF_IW-1:0]    load_idx;
  logic [BEAT_W-1:0]   beat;
  logic                take;
  logic                alloc;      // header taken into the tail entry
  logic [PF_IW-1:0]    alloc_idx;
  logic                load_done;  // last data flit taken
  logic                advance;
  hdr_t                in_hdr;

  assign head      = head_p[PF_IW-1:0];
  assign full      = (tail_p - head_p) == (PF_IW + 1)'(PF_DEPTH);
  assign t_req_rdy = loading || !full;
  assign take      = t_req_vld && t_req_rdy;
  assign in_hdr    = get_hdr(t_req_flit);
  assign alloc     = take && t_req_flit.head;
  assign alloc_idx = tail_p[PF_IW-1:0];
  assign load_done = take && !t_req_flit.head && t_req_flit.tail;
  assign advance   = (head_p != tail_p) && ent[head].state == E_FREE && !ent[head].wb_pend;

  sh_lib_ff #(
    .WIDTH     (PF_IW + 1),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_tail (
    .clk (clk),
    .rst (rst),
    .en  (alloc),
    .d   (tail_p + 1'b1),
    .q   (tail_p)
  );

  sh_lib_ff #(
    .WIDTH     (PF_IW + 1),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_head (
    .clk (clk),
    .rst (rst),
    .en  (advance),
    .d   (head_p + 1'b1),
    .q   (head_p)
  );

  sh_lib_ff #(
    .WIDTH     (1 + PF_IW + BEAT_W),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_load (
    .clk (clk),
    .rst (rst),
    .en  (take),
    .d   (t_req_flit.head ? {!t_req_flit.tail, alloc_idx, BEAT_W'(0)}
                          : {!t_req_flit.tail, load_idx, beat + 1'b1}),
    .q   ({loading, load_idx, beat})
  );

  // Pending FIFO memory writes: the header into slot 0, data flit i into
  // slot 1 + i
  always_comb begin
    pf_mem_wr_en   = take && (t_req_flit.head || beat < BEAT_W'(WORDS));
    pf_mem_wr_addr = t_req_flit.head ? alloc_idx : load_idx;
    pf_mem_wr_mask = '0;
    if (t_req_flit.head) begin
      pf_mem_wr_mask[0] = 1'b1;
      pf_mem_wr_data    = {PF_SLOTS{SLOT_W'(in_hdr)}};
    end else begin
      for (int i = 0; i < WORDS; i++) pf_mem_wr_mask[i + 1] = (beat == BEAT_W'(i));
      pf_mem_wr_data = {PF_SLOTS{SLOT_W'({t_req_flit.strb, t_req_flit.data})}};
    end
  end

  // ---------------------------------------------------------------------------
  // Dependency lookup for the command being accepted
  // ---------------------------------------------------------------------------

  logic              freeing;     // an entry is freed this cycle
  logic              dep_vld;
  logic [PF_IW-1:0]   dep_idx;

  assign freeing = upd_en && upd_ent.state == E_FREE;

  // Youngest pending entry of the same line: scan back from the tail
  always_comb begin
    logic [PF_IW-1:0] idx;
    dep_vld = 1'b0;
    dep_idx = '0;
    for (int k = 1; k < PF_DEPTH; k++) begin
      idx = alloc_idx - PF_IW'(k);
      if (!dep_vld && ent[idx].state != E_FREE && ent[idx].line == addr_line(in_hdr.addr)
          && !(freeing && upd_idx == idx)) begin
        dep_vld = 1'b1;
        dep_idx = idx;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Entry state
  // ---------------------------------------------------------------------------

  ent_t [PF_DEPTH-1:0] ent_d;

  always_comb begin
    for (int i = 0; i < PF_DEPTH; i++) begin
      ent_d[i] = ent[i];

      if (alloc && alloc_idx == PF_IW'(i)) begin
        ent_d[i]       = '0;
        ent_d[i].state  = t_req_flit.tail ? E_NEW : E_LOAD;
        ent_d[i].line   = addr_line(in_hdr.addr);
        ent_d[i].dep    = dep_vld;
        ent_d[i].dep_id = dep_idx;
      end
      if (load_done && load_idx == PF_IW'(i)) ent_d[i].state = E_NEW;

      if (upd_en && upd_idx == PF_IW'(i)) ent_d[i] = upd_ent;

      // Wake-ups, applied to the state after any update this cycle
      if (freeing && ent_d[i].dep && ent_d[i].dep_id == upd_idx) ent_d[i].dep = 1'b0;
      if (inst_en && inst_idx == PF_IW'(i) && ent_d[i].state == E_FILL) begin
        ent_d[i].state = E_READY;
        ent_d[i].err   = inst_err;
      end
      if (wbr_en) begin
        if (wbr_idx == PF_IW'(i)) ent_d[i].wb_pend = 1'b0;
        if (ent_d[i].state == E_WB_WAIT && ent_d[i].wait_on == wbr_idx) ent_d[i].state = E_NEEDFILL;
      end
      if (release_en && ent_d[i].state == E_NO_WAY) ent_d[i].state = E_NEW;
    end
  end

  sh_lib_ff #(
    .WIDTH     (PF_DEPTH * $bits(ent_t)),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_ent (
    .clk (clk),
    .rst (rst),
    .en  (1'b0),
    .d   (ent_d),
    .q   (ent)
  );

`ifndef SYNTHESIS
  // The commands of a line form one chain: only its oldest has no dependency
  always_ff @(posedge clk) begin
    for (int i = 0; i < PF_DEPTH; i++)
      for (int j = i + 1; j < PF_DEPTH; j++)
        if (ent[i].state != E_FREE && ent[j].state != E_FREE && !ent[i].dep && !ent[j].dep
            && ent[i].line == ent[j].line)
          $error("%m: entries %0d and %0d of line %h both have no dependency", i, j, ent[i].line);
  end
`endif

endmodule
