// sh_l2: unified, transparent, write-back L2 cache (version 1, blocking)
//
// Parameters:
//   TAG_RD_DEL  - tag array read latency in clk cycles; 0 = combinational
//   DATA_RD_DEL - data array read latency
//   LRU_RD_DEL  - LRU array read latency
//   RST_LOW     - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC   - reset type:  1 = asynchronous, 0 = synchronous
//
// Geometry comes from sh_l2_pkg and the packet format from sh_noc_pkg. The
// L2 connects to its tile's NI: t_req/t_rsp (agents' requests to cacheable
// regions, and their responses) and i_req/i_rsp (fills and writebacks to the
// region owners). See design/l2/doc/l2_spec.md and l2_uarch.md.
//
// Requests are served one at a time, in arrival order: a hit, or a miss
// (writeback of a dirty victim, fill, install, replay as a hit). Writes are
// write-back and write-allocate; a write of a whole line with every strobe
// set allocates without a fill. A request that crosses a line, or a fill that
// returns an error, gets a header-only RSP_SLVERR response.
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

  logic                         req_vld;
  hdr_t                         req_hdr;
  logic                         req_bad;
  logic                         req_full;
  logic [WORDS-1:0][FLIT_W-1:0] wbuf_data;
  logic [WORDS-1:0][STRB_W-1:0] wbuf_strb;
  logic                         req_done;
  logic                         wb_err_evt;

  sh_l2_intake #(
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_intake (
    .clk        (clk),
    .rst        (rst),
    .t_req_vld  (t_req_vld),
    .t_req_rdy  (t_req_rdy),
    .t_req_flit (t_req_flit),
    .req_vld    (req_vld),
    .req_hdr    (req_hdr),
    .req_bad    (req_bad),
    .req_full   (req_full),
    .wbuf_data  (wbuf_data),
    .wbuf_strb  (wbuf_strb),
    .req_done   (req_done)
  );

  sh_l2_ctrl #(
    .TAG_RD_DEL  (TAG_RD_DEL),
    .DATA_RD_DEL (DATA_RD_DEL),
    .LRU_RD_DEL  (LRU_RD_DEL),
    .RST_LOW     (RST_LOW),
    .RST_ASYNC   (RST_ASYNC)
  ) u_ctrl (
    .clk              (clk),
    .rst              (rst),
    .req_vld          (req_vld),
    .req_hdr          (req_hdr),
    .req_bad          (req_bad),
    .req_full         (req_full),
    .wbuf_data        (wbuf_data),
    .wbuf_strb        (wbuf_strb),
    .req_done         (req_done),
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
