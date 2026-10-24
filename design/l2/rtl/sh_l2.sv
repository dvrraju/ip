// sh_l2: unified, transparent, write-back L2 cache (version 2: hit-under-miss,
// multiple outstanding misses)
//
// Parameters:
//   TAG_RD_DEL  - tag array read latency in clk cycles; 0 = combinational
//   DATA_RD_DEL - data array read latency
//   LRU_RD_DEL  - LRU array read latency
//   PF_RD_DEL   - pending FIFO memory read latency
//   RST_LOW     - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC   - reset type:  1 = asynchronous, 0 = synchronous
//
// Geometry comes from sh_l2_pkg and the packet format from sh_noc_pkg. The
// L2 connects to its tile's NI: t_req/t_rsp (agents' requests to cacheable
// regions, and their responses) and i_req/i_rsp (fills and writebacks to the
// region owners). See design/l2/doc/l2_spec.md and l2_uarch.md.
//
// Requests wait in a pending FIFO of PF_DEPTH entries (t_req stalls when
// it is full). The pipeline takes the oldest entry that is ready: hits are
// served while misses are outstanding, up to PF_DEPTH misses can be
// outstanding, and requests to the same line complete in arrival order;
// responses to different lines may complete out of order. A miss writes back
// a dirty victim, fills the line, installs it and replays as a hit. Writes
// are write-back and write-allocate; a write of a whole line with every
// strobe set allocates without a fill. A request that crosses a line, or a
// fill that returns an error, gets a header-only RSP_SLVERR response.
// Memory may answer fills and writebacks in any order (they are tagged with
// the entry index); a line is not refilled before its writeback completes.
//
// No memories are instantiated here. Each array is a 1R1W memory port: a
// write happens on the clk edge when *_wr_en is high; *_rd_data is the entry
// at *_rd_addr *_RD_DEL cycles after *_rd_en (same cycle when 0), and a read
// of an entry written in the same cycle returns the old value.
//   tag_mem_*  - NUM_SETS x TAG_ROW_W; entry = NUM_WAYS x {valid, dirty, tag},
//                way 0 in the low bits; wr_mask selects the ways written
//   data_mem_* - NUM_SETS * NUM_WAYS * WORDS x FLIT_W, address {set, way,
//                word}; wr_strb selects the bytes written
//   lru_mem_*  - NUM_SETS x LRU_W tree pseudo-LRU bits
//   pf_mem_*  - PF_DEPTH x PF_ENTRY_W pending FIFO; an entry is
//                PF_SLOTS slots of SLOT_W bits (slot 0 the header, slot 1 + i
//                data flit i as {strb, data}); wr_mask selects the slots
//
// wb_err is set when a writeback returns an error (the dirty data is lost)
// and stays set until wb_err_clr is pulsed or reset.

module sh_l2
  import sh_noc_pkg::*;
  import sh_l2_pkg::*;
#(
  parameter int unsigned TAG_RD_DEL  = 0,
  parameter int unsigned DATA_RD_DEL = 0,
  parameter int unsigned LRU_RD_DEL  = 0,
  parameter int unsigned PF_RD_DEL   = 0,
  parameter logic        RST_LOW     = 1'b1,
  parameter logic        RST_ASYNC   = 1'b1
) (
  input  logic                 clk,
  input  logic                 rst,
  // Agents' requests and responses (NI target port)
  input  logic                 t_req_vld,
  output logic                 t_req_rdy,
  input  flit_t                t_req_flit,
  output logic                 t_rsp_vld,
  input  logic                 t_rsp_rdy,
  output flit_t                t_rsp_flit,
  // Fills and writebacks (NI initiator port)
  output logic                 i_req_vld,
  input  logic                 i_req_rdy,
  output flit_t                i_req_flit,
  input  logic                 i_rsp_vld,
  output logic                 i_rsp_rdy,
  input  flit_t                i_rsp_flit,
  // Tag array
  output logic                 tag_mem_wr_en,
  output logic [SET_W-1:0]     tag_mem_wr_addr,
  output logic [TAG_ROW_W-1:0] tag_mem_wr_data,
  output logic [NUM_WAYS-1:0]  tag_mem_wr_mask,
  output logic                 tag_mem_rd_en,
  output logic [SET_W-1:0]     tag_mem_rd_addr,
  input  logic [TAG_ROW_W-1:0] tag_mem_rd_data,
  // Data array
  output logic                 data_mem_wr_en,
  output logic [DATA_AW-1:0]   data_mem_wr_addr,
  output logic [FLIT_W-1:0]    data_mem_wr_data,
  output logic [STRB_W-1:0]    data_mem_wr_strb,
  output logic                 data_mem_rd_en,
  output logic [DATA_AW-1:0]   data_mem_rd_addr,
  input  logic [FLIT_W-1:0]    data_mem_rd_data,
  // LRU array
  output logic                 lru_mem_wr_en,
  output logic [SET_W-1:0]     lru_mem_wr_addr,
  output logic [LRU_W-1:0]     lru_mem_wr_data,
  output logic                 lru_mem_rd_en,
  output logic [SET_W-1:0]     lru_mem_rd_addr,
  input  logic [LRU_W-1:0]     lru_mem_rd_data,
  // Pending FIFO
  output logic                   pf_mem_wr_en,
  output logic [PF_IW-1:0]       pf_mem_wr_addr,
  output logic [PF_ENTRY_W-1:0] pf_mem_wr_data,
  output logic [PF_SLOTS-1:0]    pf_mem_wr_mask,
  output logic                   pf_mem_rd_en,
  output logic [PF_IW-1:0]       pf_mem_rd_addr,
  input  logic [PF_ENTRY_W-1:0] pf_mem_rd_data,
  // Status
  output logic                 wb_err,
  input  logic                 wb_err_clr
);

  if (WORDS > MAX_LEN) begin : g_check_len
    $error("sh_l2: a line (%0d flits) must fit in one NoC packet (MAX_LEN %0d)", WORDS, MAX_LEN);
  end
  if (NUM_WAYS < 2 || (1 << WAY_W) != NUM_WAYS) begin : g_check_ways
    $error("sh_l2: NUM_WAYS must be a power of 2 >= 2, got %0d", NUM_WAYS);
  end

  ent_t [PF_DEPTH-1:0] ent;
  logic [PF_IW-1:0]     head;
  logic                 upd_en;
  logic [PF_IW-1:0]     upd_idx;
  ent_t                 upd_ent;
  logic                 inst_en;
  logic [PF_IW-1:0]     inst_idx;
  logic                 inst_err;
  logic                 wbr_en;
  logic [PF_IW-1:0]     wbr_idx;
  logic                 release_en;
  logic                 wb_err_evt;

  sh_l2_pfifo #(
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_pfifo (
    .clk            (clk),
    .rst            (rst),
    .t_req_vld      (t_req_vld),
    .t_req_rdy      (t_req_rdy),
    .t_req_flit     (t_req_flit),
    .pf_mem_wr_en   (pf_mem_wr_en),
    .pf_mem_wr_addr (pf_mem_wr_addr),
    .pf_mem_wr_data (pf_mem_wr_data),
    .pf_mem_wr_mask (pf_mem_wr_mask),
    .ent            (ent),
    .head           (head),
    .upd_en         (upd_en),
    .upd_idx        (upd_idx),
    .upd_ent        (upd_ent),
    .inst_en        (inst_en),
    .inst_idx       (inst_idx),
    .inst_err       (inst_err),
    .wbr_en         (wbr_en),
    .wbr_idx        (wbr_idx),
    .release_en     (release_en)
  );

  sh_l2_ctrl #(
    .TAG_RD_DEL  (TAG_RD_DEL),
    .DATA_RD_DEL (DATA_RD_DEL),
    .LRU_RD_DEL  (LRU_RD_DEL),
    .PF_RD_DEL   (PF_RD_DEL),
    .RST_LOW     (RST_LOW),
    .RST_ASYNC   (RST_ASYNC)
  ) u_ctrl (
    .clk              (clk),
    .rst              (rst),
    .ent              (ent),
    .head             (head),
    .upd_en           (upd_en),
    .upd_idx          (upd_idx),
    .upd_ent          (upd_ent),
    .inst_en          (inst_en),
    .inst_idx         (inst_idx),
    .inst_err         (inst_err),
    .wbr_en           (wbr_en),
    .wbr_idx          (wbr_idx),
    .release_en       (release_en),
    .pf_mem_rd_en     (pf_mem_rd_en),
    .pf_mem_rd_addr   (pf_mem_rd_addr),
    .pf_mem_rd_data   (pf_mem_rd_data),
    .t_rsp_vld        (t_rsp_vld),
    .t_rsp_rdy        (t_rsp_rdy),
    .t_rsp_flit       (t_rsp_flit),
    .i_req_vld        (i_req_vld),
    .i_req_rdy        (i_req_rdy),
    .i_req_flit       (i_req_flit),
    .i_rsp_vld        (i_rsp_vld),
    .i_rsp_rdy        (i_rsp_rdy),
    .i_rsp_flit       (i_rsp_flit),
    .tag_mem_wr_en    (tag_mem_wr_en),
    .tag_mem_wr_addr  (tag_mem_wr_addr),
    .tag_mem_wr_data  (tag_mem_wr_data),
    .tag_mem_wr_mask  (tag_mem_wr_mask),
    .tag_mem_rd_en    (tag_mem_rd_en),
    .tag_mem_rd_addr  (tag_mem_rd_addr),
    .tag_mem_rd_data  (tag_mem_rd_data),
    .data_mem_wr_en   (data_mem_wr_en),
    .data_mem_wr_addr (data_mem_wr_addr),
    .data_mem_wr_data (data_mem_wr_data),
    .data_mem_wr_strb (data_mem_wr_strb),
    .data_mem_rd_en   (data_mem_rd_en),
    .data_mem_rd_addr (data_mem_rd_addr),
    .data_mem_rd_data (data_mem_rd_data),
    .lru_mem_wr_en    (lru_mem_wr_en),
    .lru_mem_wr_addr  (lru_mem_wr_addr),
    .lru_mem_wr_data  (lru_mem_wr_data),
    .lru_mem_rd_en    (lru_mem_rd_en),
    .lru_mem_rd_addr  (lru_mem_rd_addr),
    .lru_mem_rd_data  (lru_mem_rd_data),
    .wb_err_evt       (wb_err_evt)
  );

  // Sticky writeback error; a new error wins over a clear in the same cycle
  sh_lib_ff #(
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_wb_err (
    .clk (clk),
    .rst (rst),
    .en  (wb_err_evt || wb_err_clr),
    .d   (wb_err_evt),
    .q   (wb_err)
  );

endmodule
