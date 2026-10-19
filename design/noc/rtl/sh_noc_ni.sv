// sh_noc_ni: network interface between a tile and its router's Local port
//
// Parameters:
//   X_W, Y_W       - widths of the destination x / y fields in a flit
//   DATA_W         - payload width in bits
//   BUF_DEPTH      - router input buffer depth (credits for the inject link) and
//                    depth of the eject buffer here
//   BUF_MEM_RD_DEL - eject buffer memory read latency (clk); 0 = combinational
//   ASYNC          - 1 = tile_clk is asynchronous to clk (dual-clock FIFOs)
//                    0 = tile_clk/tile_rst are ignored; the tile runs on clk/rst
//   CDC_DEPTH      - entries in each tile-side FIFO (power of 2 >= 2 when ASYNC)
//   SYNC_DELAY     - synchronizer flops per pointer crossing (ASYNC = 1 only)
//   CDC_MEM_RD_DEL - CDC FIFO memory read latency, in the read side's clock
//   RST_LOW        - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC      - reset type:  1 = asynchronous, 0 = synchronous
//   FW, BAW, CAW   - flit width / eject buffer and CDC FIFO address widths
//                    (derived, do not override)
//
// Tile side (tile_clk): valid/ready inject of {dst_y, dst_x, data} and
// valid/ready eject of data. The packet carries no source; put it in the
// payload if the receiver needs it.
// NoC side (clk): credit-based links to and from the router's Local port.
//
// Storage is external; each memory has one write port and one read port:
//   ejb_mem_* - eject buffer,   BUF_DEPTH x FW,     write clk,      read clk
//   inj_mem_* - inject FIFO,    CDC_DEPTH x FW,     write tile_clk, read clk
//   ej_mem_*  - eject FIFO,     CDC_DEPTH x DATA_W, write clk,      read tile_clk
// (with ASYNC = 0 every memory is on clk)

module sh_noc_ni #(
  parameter int unsigned X_W            = 2,
  parameter int unsigned Y_W            = 2,
  parameter int unsigned DATA_W         = 32,
  parameter int unsigned BUF_DEPTH      = 4,
  parameter int unsigned BUF_MEM_RD_DEL = 0,
  parameter logic        ASYNC          = 1'b1,
  parameter int unsigned CDC_DEPTH      = 4,
  parameter int unsigned SYNC_DELAY     = 2,
  parameter int unsigned CDC_MEM_RD_DEL = 0,
  parameter logic        RST_LOW        = 1'b1,
  parameter logic        RST_ASYNC      = 1'b1,
  parameter int unsigned FW             = Y_W + X_W + DATA_W,
  parameter int unsigned BAW            = (BUF_DEPTH > 1) ? $clog2(BUF_DEPTH) : 1,
  parameter int unsigned CAW            = (CDC_DEPTH > 1) ? $clog2(CDC_DEPTH) : 1
) (
  // NoC domain
  input  logic              clk,
  input  logic              rst,
  // Tile domain
  input  logic              tile_clk,
  input  logic              tile_rst,
  // Tile inject
  input  logic              inj_vld,
  output logic              inj_rdy,
  input  logic [X_W-1:0]    inj_dst_x,
  input  logic [Y_W-1:0]    inj_dst_y,
  input  logic [DATA_W-1:0] inj_data,
  // Tile eject
  output logic              ej_vld,
  input  logic              ej_rdy,
  output logic [DATA_W-1:0] ej_data,
  // To the router's Local input
  output logic              rtr_in_vld,
  output logic [FW-1:0]     rtr_in_flit,
  input  logic              rtr_in_crd,
  // From the router's Local output
  input  logic              rtr_out_vld,
  input  logic [FW-1:0]     rtr_out_flit,
  output logic              rtr_out_crd,
  // Eject buffer memory
  output logic              ejb_mem_wr_en,
  output logic [BAW-1:0]    ejb_mem_wr_addr,
  output logic [FW-1:0]     ejb_mem_wr_data,
  output logic              ejb_mem_rd_en,
  output logic [BAW-1:0]    ejb_mem_rd_addr,
  input  logic [FW-1:0]     ejb_mem_rd_data,
  // Inject CDC FIFO memory
  output logic              inj_mem_wr_en,
  output logic [CAW-1:0]    inj_mem_wr_addr,
  output logic [FW-1:0]     inj_mem_wr_data,
  output logic              inj_mem_rd_en,
  output logic [CAW-1:0]    inj_mem_rd_addr,
  input  logic [FW-1:0]     inj_mem_rd_data,
  // Eject CDC FIFO memory
  output logic              ej_mem_wr_en,
  output logic [CAW-1:0]    ej_mem_wr_addr,
  output logic [DATA_W-1:0] ej_mem_wr_data,
  output logic              ej_mem_rd_en,
  output logic [CAW-1:0]    ej_mem_rd_addr,
  input  logic [DATA_W-1:0] ej_mem_rd_data
);

  // ---------------------------------------------------------------------------
  // Inject: tile -> CDC FIFO -> credit-gated link to the router
  // ---------------------------------------------------------------------------

  logic          inj_q_vld;
  logic [FW-1:0] inj_q_flit;
  logic          has_crd;
  logic          send;

  sh_lib_cdc_fifo #(
    .WIDTH      (FW),
    .DEPTH      (CDC_DEPTH),
    .ASYNC      (ASYNC),
    .SYNC_DELAY (SYNC_DELAY),
    .MEM_RD_DEL (CDC_MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_inj_fifo (
    .wr_clk      (ASYNC ? tile_clk : clk),
    .wr_rst      (ASYNC ? tile_rst : rst),
    .wr_vld      (inj_vld),
    .wr_rdy      (inj_rdy),
    .wr_data     ({inj_dst_y, inj_dst_x, inj_data}),
    .rd_clk      (clk),
    .rd_rst      (rst),
    .rd_vld      (inj_q_vld),
    .rd_rdy      (send),
    .rd_data     (inj_q_flit),
    .mem_wr_en   (inj_mem_wr_en),
    .mem_wr_addr (inj_mem_wr_addr),
    .mem_wr_data (inj_mem_wr_data),
    .mem_rd_en   (inj_mem_rd_en),
    .mem_rd_addr (inj_mem_rd_addr),
    .mem_rd_data (inj_mem_rd_data)
  );

  assign send = inj_q_vld && has_crd;

  sh_noc_crd_cnt #(
    .DEPTH     (BUF_DEPTH),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_crd_cnt (
    .clk     (clk),
    .rst     (rst),
    .send    (send),
    .crd     (rtr_in_crd),
    .has_crd (has_crd)
  );

  sh_lib_ff #(
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_inj_vld (
    .clk (clk),
    .rst (rst),
    .en  (1'b0),
    .d   (send),
    .q   (rtr_in_vld)
  );

  sh_lib_ff #(
    .WIDTH  (FW),
    .EN     (1'b1),
    .RST_EN (1'b0)
  ) u_inj_flit (
    .clk (clk),
    .rst (1'b0),
    .en  (send),
    .d   (inj_q_flit),
    .q   (rtr_in_flit)
  );

  // ---------------------------------------------------------------------------
  // Eject: router -> credit-returning buffer -> CDC FIFO -> tile
  // ---------------------------------------------------------------------------

  logic          ej_buf_vld;
  logic          ej_buf_rdy;
  /* verilator lint_off UNUSEDSIGNAL */
  logic [FW-1:0] ej_buf_flit;   // coordinates are dropped
  /* verilator lint_on UNUSEDSIGNAL */

  sh_noc_in_buf #(
    .WIDTH      (FW),
    .DEPTH      (BUF_DEPTH),
    .MEM_RD_DEL (BUF_MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_ej_buf (
    .clk         (clk),
    .rst         (rst),
    .in_vld      (rtr_out_vld),
    .in_flit     (rtr_out_flit),
    .crd         (rtr_out_crd),
    .out_vld     (ej_buf_vld),
    .out_rdy     (ej_buf_rdy),
    .out_flit    (ej_buf_flit),
    .mem_wr_en   (ejb_mem_wr_en),
    .mem_wr_addr (ejb_mem_wr_addr),
    .mem_wr_data (ejb_mem_wr_data),
    .mem_rd_en   (ejb_mem_rd_en),
    .mem_rd_addr (ejb_mem_rd_addr),
    .mem_rd_data (ejb_mem_rd_data)
  );

  sh_lib_cdc_fifo #(
    .WIDTH      (DATA_W),
    .DEPTH      (CDC_DEPTH),
    .ASYNC      (ASYNC),
    .SYNC_DELAY (SYNC_DELAY),
    .MEM_RD_DEL (CDC_MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_ej_fifo (
    .wr_clk      (clk),
    .wr_rst      (rst),
    .wr_vld      (ej_buf_vld),
    .wr_rdy      (ej_buf_rdy),
    .wr_data     (ej_buf_flit[DATA_W-1:0]),
    .rd_clk      (ASYNC ? tile_clk : clk),
    .rd_rst      (ASYNC ? tile_rst : rst),
    .rd_vld      (ej_vld),
    .rd_rdy      (ej_rdy),
    .rd_data     (ej_data),
    .mem_wr_en   (ej_mem_wr_en),
    .mem_wr_addr (ej_mem_wr_addr),
    .mem_wr_data (ej_mem_wr_data),
    .mem_rd_en   (ej_mem_rd_en),
    .mem_rd_addr (ej_mem_rd_addr),
    .mem_rd_data (ej_mem_rd_data)
  );

endmodule
