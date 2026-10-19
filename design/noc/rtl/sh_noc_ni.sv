// sh_noc_ni: network interface between a tile and the request and response
// networks at its mesh position
//
// Parameters:
//   X, Y           - this tile's coordinates
//   IS_L2          - 1 = the L2 cache is this tile's agent: its requests
//                    (fills, writebacks) always go to the region owner
//   BUF_DEPTH      - router input buffer depth (credits for the inject links)
//                    and depth of the two eject buffers here
//   BUF_MEM_RD_DEL - eject buffer memory read latency (clk); 0 = combinational
//   ASYNC          - 1 = tile_clk is asynchronous to clk (dual-clock FIFOs)
//                    0 = tile_clk/tile_rst are ignored; the tile runs on clk/rst
//   CDC_DEPTH      - entries in each tile-side FIFO (power of 2 >= 2 when ASYNC)
//   SYNC_DELAY     - synchronizer flops per pointer crossing (ASYNC = 1 only)
//   CDC_MEM_RD_DEL - CDC FIFO memory read latency, in the read side's clock
//   RST_LOW        - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC      - reset type:  1 = asynchronous, 0 = synchronous
//   BAW, CAW       - eject buffer / CDC FIFO address widths (derived, do not
//                    override)
//
// Tile side (tile_clk), all valid/ready flit streams in the sh_noc_pkg format:
//   i_req - initiator requests out. The NI decodes each header's addr with
//           the address map and fills in dst and src; the tile's values for
//           those fields are ignored. dst is the region owner, or the L2 tile
//           for a cacheable region when an L2 is present and this is not the
//           L2's NI. A request to an unmapped address is dropped here (with
//           its data flits) and answered on i_rsp with a header-only response
//           with status RSP_DECERR.
//   i_rsp - responses to the initiator.
//   t_req - requests to this tile as a target.
//   t_rsp - target responses out. The tile sets dst (the request's src); the
//           NI fills in src.
// NoC side (clk): credit-based links to the Local ports of the request
// network (req_*) and response network (rsp_*).
//
// Storage is external; each memory has one write port and one read port:
//   ejb_mem_*[n] - eject buffer of network n (0 = request, 1 = response),
//                  BUF_DEPTH x FLIT_LINK_W, write and read clk
//   cdc_mem_*[f] - CDC FIFO f, CDC_DEPTH x FLIT_LINK_W:
//                  0 = i_req (write tile_clk, read clk)
//                  1 = i_rsp (write clk,      read tile_clk)
//                  2 = t_req (write clk,      read tile_clk)
//                  3 = t_rsp (write tile_clk, read clk)
// (with ASYNC = 0 every memory is on clk)

module sh_noc_ni
  import sh_noc_pkg::*;
#(
  parameter int unsigned X              = 0,
  parameter int unsigned Y              = 0,
  parameter logic        IS_L2          = 1'b0,
  parameter int unsigned BUF_DEPTH      = 4,
  parameter int unsigned BUF_MEM_RD_DEL = 0,
  parameter logic        ASYNC          = 1'b1,
  parameter int unsigned CDC_DEPTH      = 4,
  parameter int unsigned SYNC_DELAY     = 2,
  parameter int unsigned CDC_MEM_RD_DEL = 0,
  parameter logic        RST_LOW        = 1'b1,
  parameter logic        RST_ASYNC      = 1'b1,
  parameter int unsigned BAW            = (BUF_DEPTH > 1) ? $clog2(BUF_DEPTH) : 1,
  parameter int unsigned CAW            = (CDC_DEPTH > 1) ? $clog2(CDC_DEPTH) : 1
) (
  // NoC domain
  input  logic                        clk,
  input  logic                        rst,
  // Tile domain
  input  logic                        tile_clk,
  input  logic                        tile_rst,
  // Initiator
  input  logic                        i_req_vld,
  output logic                        i_req_rdy,
  input  flit_t                       i_req_flit,
  output logic                        i_rsp_vld,
  input  logic                        i_rsp_rdy,
  output flit_t                       i_rsp_flit,
  // Target
  output logic                        t_req_vld,
  input  logic                        t_req_rdy,
  output flit_t                       t_req_flit,
  input  logic                        t_rsp_vld,
  output logic                        t_rsp_rdy,
  input  flit_t                       t_rsp_flit,
  // Request network Local port
  output logic                        req_inj_vld,
  output flit_t                       req_inj_flit,
  input  logic                        req_inj_crd,
  input  logic                        req_ej_vld,
  input  flit_t                       req_ej_flit,
  output logic                        req_ej_crd,
  // Response network Local port
  output logic                        rsp_inj_vld,
  output flit_t                       rsp_inj_flit,
  input  logic                        rsp_inj_crd,
  input  logic                        rsp_ej_vld,
  input  flit_t                       rsp_ej_flit,
  output logic                        rsp_ej_crd,
  // Eject buffer memories
  output logic [1:0]                  ejb_mem_wr_en,
  output logic [1:0][BAW-1:0]         ejb_mem_wr_addr,
  output logic [1:0][FLIT_LINK_W-1:0] ejb_mem_wr_data,
  output logic [1:0]                  ejb_mem_rd_en,
  output logic [1:0][BAW-1:0]         ejb_mem_rd_addr,
  input  logic [1:0][FLIT_LINK_W-1:0] ejb_mem_rd_data,
  // CDC FIFO memories
  output logic [3:0]                  cdc_mem_wr_en,
  output logic [3:0][CAW-1:0]         cdc_mem_wr_addr,
  output logic [3:0][FLIT_LINK_W-1:0] cdc_mem_wr_data,
  output logic [3:0]                  cdc_mem_rd_en,
  output logic [3:0][CAW-1:0]         cdc_mem_rd_addr,
  input  logic [3:0][FLIT_LINK_W-1:0] cdc_mem_rd_data
);

  localparam int unsigned F_I_REQ = 0;
  localparam int unsigned F_I_RSP = 1;
  localparam int unsigned F_T_REQ = 2;
  localparam int unsigned F_T_RSP = 3;

  // Clock and reset of the tile side of the CDC FIFOs
  logic t_clk;
  logic t_rst;

  assign t_clk = ASYNC ? tile_clk : clk;
  assign t_rst = ASYNC ? tile_rst : rst;

  // ---------------------------------------------------------------------------
  // Initiator requests: tile -> CDC FIFO -> decode/stamp -> request network
  // ---------------------------------------------------------------------------

  logic      ireq_vld;
  logic      ireq_rdy;
  flit_t     ireq_flit;
  hdr_t      ireq_hdr;
  hdr_t      ireq_hdr_out;
  addr_dec_t ireq_dec;
  logic      ireq_drop;      // current flit belongs to a rejected packet
  logic      drop_q;         // inside a rejected packet, after its header
  logic      req_tx_vld;
  logic      req_tx_rdy;
  flit_t     req_tx_flit;
  logic      err_vld;        // DECERR response waiting for i_rsp
  flit_t     err_flit;
  hdr_t      err_hdr;
  logic      err_set;
  logic      err_clr;

  sh_lib_cdc_fifo #(
    .WIDTH      (FLIT_LINK_W),
    .DEPTH      (CDC_DEPTH),
    .ASYNC      (ASYNC),
    .SYNC_DELAY (SYNC_DELAY),
    .MEM_RD_DEL (CDC_MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_i_req_fifo (
    .wr_clk      (t_clk),
    .wr_rst      (t_rst),
    .wr_vld      (i_req_vld),
    .wr_rdy      (i_req_rdy),
    .wr_data     (i_req_flit),
    .rd_clk      (clk),
    .rd_rst      (rst),
    .rd_vld      (ireq_vld),
    .rd_rdy      (ireq_rdy),
    .rd_data     (ireq_flit),
    .mem_wr_en   (cdc_mem_wr_en[F_I_REQ]),
    .mem_wr_addr (cdc_mem_wr_addr[F_I_REQ]),
    .mem_wr_data (cdc_mem_wr_data[F_I_REQ]),
    .mem_rd_en   (cdc_mem_rd_en[F_I_REQ]),
    .mem_rd_addr (cdc_mem_rd_addr[F_I_REQ]),
    .mem_rd_data (cdc_mem_rd_data[F_I_REQ])
  );

  assign ireq_hdr  = get_hdr(ireq_flit);
  assign ireq_dec  = addr_decode(ireq_hdr.addr);
  assign ireq_drop = ireq_flit.head ? !ireq_dec.hit : drop_q;

  always_comb begin
    ireq_hdr_out = ireq_hdr;
    if (L2_PRESENT && !IS_L2 && ireq_dec.cacheable) begin
      ireq_hdr_out.dst_x = X_W'(L2_X);
      ireq_hdr_out.dst_y = Y_W'(L2_Y);
    end else begin
      ireq_hdr_out.dst_x = ireq_dec.x;
      ireq_hdr_out.dst_y = ireq_dec.y;
    end
    ireq_hdr_out.src_x = X_W'(X);
    ireq_hdr_out.src_y = Y_W'(Y);
  end

  assign req_tx_vld       = ireq_vld && !ireq_drop;
  assign req_tx_flit.head = ireq_flit.head;
  assign req_tx_flit.tail = ireq_flit.tail;
  assign req_tx_flit.strb = ireq_flit.strb;
  assign req_tx_flit.data = ireq_flit.head ? hdr_to_data(ireq_hdr_out) : ireq_flit.data;

  // A rejected header waits until the previous DECERR response has gone
  assign ireq_rdy = ireq_drop ? !(ireq_flit.head && err_vld) : req_tx_rdy;

  sh_lib_ff #(
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_drop (
    .clk (clk),
    .rst (rst),
    .en  (ireq_vld && ireq_rdy),
    .d   (ireq_drop && !ireq_flit.tail),
    .q   (drop_q)
  );

  // DECERR response for a rejected request, back to this tile
  always_comb begin
    err_hdr        = '0;
    err_hdr.dst_x  = X_W'(X);
    err_hdr.dst_y  = Y_W'(Y);
    err_hdr.src_x  = X_W'(X);
    err_hdr.src_y  = Y_W'(Y);
    err_hdr.typ    = (ireq_hdr.typ == PKT_RD_REQ) ? PKT_RD_RSP : PKT_WR_RSP;
    err_hdr.tag    = ireq_hdr.tag;
    err_hdr.len    = ireq_hdr.len;
    err_hdr.status = RSP_DECERR;
  end

  assign err_set = ireq_vld && ireq_rdy && ireq_drop && ireq_flit.head;

  sh_lib_ff #(
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_err_vld (
    .clk (clk),
    .rst (rst),
    .en  (err_set || err_clr),
    .d   (err_set),
    .q   (err_vld)
  );

  sh_lib_ff #(
    .WIDTH  (FLIT_LINK_W),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_err_flit (
    .clk (clk),
    .rst (1'b0),
    .en  (err_set),
    .d   ({1'b1, 1'b1, STRB_W'(0), hdr_to_data(err_hdr)}),
    .q   (err_flit)
  );

  sh_noc_tx #(
    .WIDTH     (FLIT_LINK_W),
    .DEPTH     (BUF_DEPTH),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_req_tx (
    .clk      (clk),
    .rst      (rst),
    .in_vld   (req_tx_vld),
    .in_rdy   (req_tx_rdy),
    .in_flit  (req_tx_flit),
    .out_vld  (req_inj_vld),
    .out_flit (req_inj_flit),
    .crd      (req_inj_crd)
  );

  // ---------------------------------------------------------------------------
  // Initiator responses: response network + DECERR responses -> CDC FIFO -> tile
  // ---------------------------------------------------------------------------

  logic  rsp_ej_buf_vld;
  logic  rsp_ej_buf_rdy;
  flit_t rsp_ej_buf_flit;
  logic  net_busy;       // a network response packet is part-way through
  logic  sel_err;
  logic  irsp_vld;
  logic  irsp_rdy;
  flit_t irsp_flit;

  sh_noc_in_buf #(
    .WIDTH      (FLIT_LINK_W),
    .DEPTH      (BUF_DEPTH),
    .MEM_RD_DEL (BUF_MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_rsp_ej_buf (
    .clk         (clk),
    .rst         (rst),
    .in_vld      (rsp_ej_vld),
    .in_flit     (rsp_ej_flit),
    .crd         (rsp_ej_crd),
    .out_vld     (rsp_ej_buf_vld),
    .out_rdy     (rsp_ej_buf_rdy),
    .out_flit    (rsp_ej_buf_flit),
    .mem_wr_en   (ejb_mem_wr_en[1]),
    .mem_wr_addr (ejb_mem_wr_addr[1]),
    .mem_wr_data (ejb_mem_wr_data[1]),
    .mem_rd_en   (ejb_mem_rd_en[1]),
    .mem_rd_addr (ejb_mem_rd_addr[1]),
    .mem_rd_data (ejb_mem_rd_data[1])
  );

  // A DECERR response goes in between network response packets
  assign sel_err        = err_vld && !net_busy;
  assign irsp_vld       = sel_err || rsp_ej_buf_vld;
  assign irsp_flit      = sel_err ? err_flit : rsp_ej_buf_flit;
  assign rsp_ej_buf_rdy = !sel_err && irsp_rdy;
  assign err_clr        = sel_err && irsp_rdy;

  sh_lib_ff #(
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_net_busy (
    .clk (clk),
    .rst (rst),
    .en  (rsp_ej_buf_vld && rsp_ej_buf_rdy),
    .d   (!rsp_ej_buf_flit.tail),
    .q   (net_busy)
  );

  sh_lib_cdc_fifo #(
    .WIDTH      (FLIT_LINK_W),
    .DEPTH      (CDC_DEPTH),
    .ASYNC      (ASYNC),
    .SYNC_DELAY (SYNC_DELAY),
    .MEM_RD_DEL (CDC_MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_i_rsp_fifo (
    .wr_clk      (clk),
    .wr_rst      (rst),
    .wr_vld      (irsp_vld),
    .wr_rdy      (irsp_rdy),
    .wr_data     (irsp_flit),
    .rd_clk      (t_clk),
    .rd_rst      (t_rst),
    .rd_vld      (i_rsp_vld),
    .rd_rdy      (i_rsp_rdy),
    .rd_data     (i_rsp_flit),
    .mem_wr_en   (cdc_mem_wr_en[F_I_RSP]),
    .mem_wr_addr (cdc_mem_wr_addr[F_I_RSP]),
    .mem_wr_data (cdc_mem_wr_data[F_I_RSP]),
    .mem_rd_en   (cdc_mem_rd_en[F_I_RSP]),
    .mem_rd_addr (cdc_mem_rd_addr[F_I_RSP]),
    .mem_rd_data (cdc_mem_rd_data[F_I_RSP])
  );

  // ---------------------------------------------------------------------------
  // Target requests: request network -> CDC FIFO -> tile
  // ---------------------------------------------------------------------------

  logic  req_ej_buf_vld;
  logic  req_ej_buf_rdy;
  flit_t req_ej_buf_flit;

  sh_noc_in_buf #(
    .WIDTH      (FLIT_LINK_W),
    .DEPTH      (BUF_DEPTH),
    .MEM_RD_DEL (BUF_MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_req_ej_buf (
    .clk         (clk),
    .rst         (rst),
    .in_vld      (req_ej_vld),
    .in_flit     (req_ej_flit),
    .crd         (req_ej_crd),
    .out_vld     (req_ej_buf_vld),
    .out_rdy     (req_ej_buf_rdy),
    .out_flit    (req_ej_buf_flit),
    .mem_wr_en   (ejb_mem_wr_en[0]),
    .mem_wr_addr (ejb_mem_wr_addr[0]),
    .mem_wr_data (ejb_mem_wr_data[0]),
    .mem_rd_en   (ejb_mem_rd_en[0]),
    .mem_rd_addr (ejb_mem_rd_addr[0]),
    .mem_rd_data (ejb_mem_rd_data[0])
  );

  sh_lib_cdc_fifo #(
    .WIDTH      (FLIT_LINK_W),
    .DEPTH      (CDC_DEPTH),
    .ASYNC      (ASYNC),
    .SYNC_DELAY (SYNC_DELAY),
    .MEM_RD_DEL (CDC_MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_t_req_fifo (
    .wr_clk      (clk),
    .wr_rst      (rst),
    .wr_vld      (req_ej_buf_vld),
    .wr_rdy      (req_ej_buf_rdy),
    .wr_data     (req_ej_buf_flit),
    .rd_clk      (t_clk),
    .rd_rst      (t_rst),
    .rd_vld      (t_req_vld),
    .rd_rdy      (t_req_rdy),
    .rd_data     (t_req_flit),
    .mem_wr_en   (cdc_mem_wr_en[F_T_REQ]),
    .mem_wr_addr (cdc_mem_wr_addr[F_T_REQ]),
    .mem_wr_data (cdc_mem_wr_data[F_T_REQ]),
    .mem_rd_en   (cdc_mem_rd_en[F_T_REQ]),
    .mem_rd_addr (cdc_mem_rd_addr[F_T_REQ]),
    .mem_rd_data (cdc_mem_rd_data[F_T_REQ])
  );

  // ---------------------------------------------------------------------------
  // Target responses: tile -> CDC FIFO -> stamp src -> response network
  // ---------------------------------------------------------------------------

  logic  trsp_vld;
  logic  trsp_rdy;
  flit_t trsp_flit;
  hdr_t  trsp_hdr_out;
  flit_t rsp_tx_flit;

  sh_lib_cdc_fifo #(
    .WIDTH      (FLIT_LINK_W),
    .DEPTH      (CDC_DEPTH),
    .ASYNC      (ASYNC),
    .SYNC_DELAY (SYNC_DELAY),
    .MEM_RD_DEL (CDC_MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_t_rsp_fifo (
    .wr_clk      (t_clk),
    .wr_rst      (t_rst),
    .wr_vld      (t_rsp_vld),
    .wr_rdy      (t_rsp_rdy),
    .wr_data     (t_rsp_flit),
    .rd_clk      (clk),
    .rd_rst      (rst),
    .rd_vld      (trsp_vld),
    .rd_rdy      (trsp_rdy),
    .rd_data     (trsp_flit),
    .mem_wr_en   (cdc_mem_wr_en[F_T_RSP]),
    .mem_wr_addr (cdc_mem_wr_addr[F_T_RSP]),
    .mem_wr_data (cdc_mem_wr_data[F_T_RSP]),
    .mem_rd_en   (cdc_mem_rd_en[F_T_RSP]),
    .mem_rd_addr (cdc_mem_rd_addr[F_T_RSP]),
    .mem_rd_data (cdc_mem_rd_data[F_T_RSP])
  );

  always_comb begin
    trsp_hdr_out       = get_hdr(trsp_flit);
    trsp_hdr_out.src_x = X_W'(X);
    trsp_hdr_out.src_y = Y_W'(Y);
  end

  assign rsp_tx_flit.head = trsp_flit.head;
  assign rsp_tx_flit.tail = trsp_flit.tail;
  assign rsp_tx_flit.strb = trsp_flit.strb;
  assign rsp_tx_flit.data = trsp_flit.head ? hdr_to_data(trsp_hdr_out) : trsp_flit.data;

  sh_noc_tx #(
    .WIDTH     (FLIT_LINK_W),
    .DEPTH     (BUF_DEPTH),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_rsp_tx (
    .clk      (clk),
    .rst      (rst),
    .in_vld   (trsp_vld),
    .in_rdy   (trsp_rdy),
    .in_flit  (rsp_tx_flit),
    .out_vld  (rsp_inj_vld),
    .out_flit (rsp_inj_flit),
    .crd      (rsp_inj_crd)
  );

endmodule
