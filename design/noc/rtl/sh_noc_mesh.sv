// sh_noc_mesh: ROWS x COLS 2D mesh NoC with request and response networks
//
// Mesh size, flit width, packet format and the system address map come from
// sh_noc_pkg. Module parameters only set the implementation:
//   BUF_DEPTH      - router input buffer depth (>= 4 for full link throughput,
//                    >= BUF_MEM_RD_DEL + 5 when BUF_MEM_RD_DEL > 0)
//   BUF_MEM_RD_DEL - read latency of the buf_mem_* memories; 0 = combinational
//   ASYNC          - 1 = each tile has its own clock (tile_clk[y][x]),
//                    0 = tiles run on clk and tile_clk/tile_rst are ignored
//   CDC_DEPTH      - entries in each tile-side FIFO (power of 2 >= 2 when ASYNC)
//   SYNC_DELAY     - synchronizer flops per pointer crossing (ASYNC = 1 only)
//   CDC_MEM_RD_DEL - read latency of the cdc_mem_* memories, in the read
//                    side's clock; 0 = combinational
//   RST_LOW        - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC      - reset type:  1 = asynchronous, 0 = synchronous
//   BAW, CAW       - buffer and CDC FIFO address widths (derived, do not
//                    override)
//
// Each tile (y, x) is an initiator and a target (see sh_noc_ni). With
// L2_PRESENT, tile (L2_Y, L2_X) is the L2 cache: it receives agents' requests
// to cacheable regions and sends its fills and writebacks to the owners.
//   i_req_* - requests out, addressed by the system address map
//   i_rsp_* - responses back, one per request
//   t_req_* - requests in
//   t_rsp_* - responses out, one per request, dst = the request's src
// Requests travel on the request network and responses on a separate
// response network, so responses are never blocked behind requests. Both
// networks use XY routing with wormhole switching; packets between the same
// source and destination arrive in order. Initiators must keep draining
// i_rsp_* and targets t_req_*, or their network backs up.
// tile_rst must be asserted together with rst.
//
// No memories are instantiated here. Every buffer is a FIFO whose storage is
// brought out on a memory port to be connected outside: one write port, and
// one read port whose data (mem_rd_data) is the word at mem_rd_addr *_RD_DEL
// read-clock cycles after mem_rd_en (same cycle when the latency is 0):
//   buf_mem_*[y][x][n][p] - BUF_DEPTH x FLIT_LINK_W on clk; network n
//                           (0 = request, 1 = response); p = 0..4 router input
//                           buffer (Local, N, E, S, W), p = 5 NI eject buffer
//   cdc_mem_*[y][x][f]    - CDC_DEPTH x FLIT_LINK_W; f = 0 i_req (write
//                           tile_clk, read clk), 1 i_rsp (write clk, read
//                           tile_clk), 2 t_req (write clk, read tile_clk),
//                           3 t_rsp (write tile_clk, read clk)
// (with ASYNC = 0 every memory is on clk)

module sh_noc_mesh
  import sh_noc_pkg::*;
#(
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
  input  logic                                                 clk,
  input  logic                                                 rst,
  // Per-tile clocks and resets
  input  logic  [ROWS-1:0][COLS-1:0]                           tile_clk,
  input  logic  [ROWS-1:0][COLS-1:0]                           tile_rst,
  // Initiator
  input  logic  [ROWS-1:0][COLS-1:0]                           i_req_vld,
  output logic  [ROWS-1:0][COLS-1:0]                           i_req_rdy,
  input  flit_t [ROWS-1:0][COLS-1:0]                           i_req_flit,
  output logic  [ROWS-1:0][COLS-1:0]                           i_rsp_vld,
  input  logic  [ROWS-1:0][COLS-1:0]                           i_rsp_rdy,
  output flit_t [ROWS-1:0][COLS-1:0]                           i_rsp_flit,
  // Target
  output logic  [ROWS-1:0][COLS-1:0]                           t_req_vld,
  input  logic  [ROWS-1:0][COLS-1:0]                           t_req_rdy,
  output flit_t [ROWS-1:0][COLS-1:0]                           t_req_flit,
  input  logic  [ROWS-1:0][COLS-1:0]                           t_rsp_vld,
  output logic  [ROWS-1:0][COLS-1:0]                           t_rsp_rdy,
  input  flit_t [ROWS-1:0][COLS-1:0]                           t_rsp_flit,
  // Buffer memories (clk)
  output logic  [ROWS-1:0][COLS-1:0][1:0][5:0]                 buf_mem_wr_en,
  output logic  [ROWS-1:0][COLS-1:0][1:0][5:0][BAW-1:0]        buf_mem_wr_addr,
  output logic  [ROWS-1:0][COLS-1:0][1:0][5:0][FLIT_LINK_W-1:0] buf_mem_wr_data,
  output logic  [ROWS-1:0][COLS-1:0][1:0][5:0]                 buf_mem_rd_en,
  output logic  [ROWS-1:0][COLS-1:0][1:0][5:0][BAW-1:0]        buf_mem_rd_addr,
  input  logic  [ROWS-1:0][COLS-1:0][1:0][5:0][FLIT_LINK_W-1:0] buf_mem_rd_data,
  // CDC FIFO memories
  output logic  [ROWS-1:0][COLS-1:0][3:0]                      cdc_mem_wr_en,
  output logic  [ROWS-1:0][COLS-1:0][3:0][CAW-1:0]             cdc_mem_wr_addr,
  output logic  [ROWS-1:0][COLS-1:0][3:0][FLIT_LINK_W-1:0]     cdc_mem_wr_data,
  output logic  [ROWS-1:0][COLS-1:0][3:0]                      cdc_mem_rd_en,
  output logic  [ROWS-1:0][COLS-1:0][3:0][CAW-1:0]             cdc_mem_rd_addr,
  input  logic  [ROWS-1:0][COLS-1:0][3:0][FLIT_LINK_W-1:0]     cdc_mem_rd_data
);

  localparam int unsigned N_REQ = 0;
  localparam int unsigned N_RSP = 1;

  if (HDR_W > FLIT_W) begin : g_check
    $error("sh_noc_mesh: header (%0d bits) does not fit in a %0d-bit flit", HDR_W, FLIT_W);
  end

  // Local ports of both networks, [network][y][x]
  logic  [ROWS-1:0][COLS-1:0] inj_vld  [2];
  flit_t [ROWS-1:0][COLS-1:0] inj_flit [2];
  logic  [ROWS-1:0][COLS-1:0] inj_crd  [2];
  logic  [ROWS-1:0][COLS-1:0] ej_vld   [2];
  flit_t [ROWS-1:0][COLS-1:0] ej_flit  [2];
  logic  [ROWS-1:0][COLS-1:0] ej_crd   [2];

  // ---------------------------------------------------------------------------
  // Request and response networks
  // ---------------------------------------------------------------------------

  for (genvar n = 0; n < 2; n++) begin : g_net
    logic [ROWS-1:0][COLS-1:0][4:0]                  mem_wr_en;
    logic [ROWS-1:0][COLS-1:0][4:0][BAW-1:0]         mem_wr_addr;
    logic [ROWS-1:0][COLS-1:0][4:0][FLIT_LINK_W-1:0] mem_wr_data;
    logic [ROWS-1:0][COLS-1:0][4:0]                  mem_rd_en;
    logic [ROWS-1:0][COLS-1:0][4:0][BAW-1:0]         mem_rd_addr;
    logic [ROWS-1:0][COLS-1:0][4:0][FLIT_LINK_W-1:0] mem_rd_data;

    sh_noc_net #(
      .BUF_DEPTH      (BUF_DEPTH),
      .BUF_MEM_RD_DEL (BUF_MEM_RD_DEL),
      .RST_LOW        (RST_LOW),
      .RST_ASYNC      (RST_ASYNC)
    ) u_net (
      .clk             (clk),
      .rst             (rst),
      .loc_in_vld      (inj_vld[n]),
      .loc_in_flit     (inj_flit[n]),
      .loc_in_crd      (inj_crd[n]),
      .loc_out_vld     (ej_vld[n]),
      .loc_out_flit    (ej_flit[n]),
      .loc_out_crd     (ej_crd[n]),
      .buf_mem_wr_en   (mem_wr_en),
      .buf_mem_wr_addr (mem_wr_addr),
      .buf_mem_wr_data (mem_wr_data),
      .buf_mem_rd_en   (mem_rd_en),
      .buf_mem_rd_addr (mem_rd_addr),
      .buf_mem_rd_data (mem_rd_data)
    );

    for (genvar y = 0; y < ROWS; y++) begin : g_row
      for (genvar x = 0; x < COLS; x++) begin : g_col
        assign buf_mem_wr_en[y][x][n][4:0]   = mem_wr_en[y][x];
        assign buf_mem_wr_addr[y][x][n][4:0] = mem_wr_addr[y][x];
        assign buf_mem_wr_data[y][x][n][4:0] = mem_wr_data[y][x];
        assign buf_mem_rd_en[y][x][n][4:0]   = mem_rd_en[y][x];
        assign buf_mem_rd_addr[y][x][n][4:0] = mem_rd_addr[y][x];
        assign mem_rd_data[y][x]             = buf_mem_rd_data[y][x][n][4:0];
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Network interfaces
  // ---------------------------------------------------------------------------

  for (genvar y = 0; y < ROWS; y++) begin : g_row
    for (genvar x = 0; x < COLS; x++) begin : g_col
      logic [1:0]                  ejb_wr_en;
      logic [1:0][BAW-1:0]         ejb_wr_addr;
      logic [1:0][FLIT_LINK_W-1:0] ejb_wr_data;
      logic [1:0]                  ejb_rd_en;
      logic [1:0][BAW-1:0]         ejb_rd_addr;
      logic [1:0][FLIT_LINK_W-1:0] ejb_rd_data;

      sh_noc_ni #(
        .X              (x),
        .Y              (y),
        .IS_L2          (L2_PRESENT && x == L2_X && y == L2_Y),
        .BUF_DEPTH      (BUF_DEPTH),
        .BUF_MEM_RD_DEL (BUF_MEM_RD_DEL),
        .ASYNC          (ASYNC),
        .CDC_DEPTH      (CDC_DEPTH),
        .SYNC_DELAY     (SYNC_DELAY),
        .CDC_MEM_RD_DEL (CDC_MEM_RD_DEL),
        .RST_LOW        (RST_LOW),
        .RST_ASYNC      (RST_ASYNC)
      ) u_ni (
        .clk             (clk),
        .rst             (rst),
        .tile_clk        (tile_clk[y][x]),
        .tile_rst        (tile_rst[y][x]),
        .i_req_vld       (i_req_vld[y][x]),
        .i_req_rdy       (i_req_rdy[y][x]),
        .i_req_flit      (i_req_flit[y][x]),
        .i_rsp_vld       (i_rsp_vld[y][x]),
        .i_rsp_rdy       (i_rsp_rdy[y][x]),
        .i_rsp_flit      (i_rsp_flit[y][x]),
        .t_req_vld       (t_req_vld[y][x]),
        .t_req_rdy       (t_req_rdy[y][x]),
        .t_req_flit      (t_req_flit[y][x]),
        .t_rsp_vld       (t_rsp_vld[y][x]),
        .t_rsp_rdy       (t_rsp_rdy[y][x]),
        .t_rsp_flit      (t_rsp_flit[y][x]),
        .req_inj_vld     (inj_vld[N_REQ][y][x]),
        .req_inj_flit    (inj_flit[N_REQ][y][x]),
        .req_inj_crd     (inj_crd[N_REQ][y][x]),
        .req_ej_vld      (ej_vld[N_REQ][y][x]),
        .req_ej_flit     (ej_flit[N_REQ][y][x]),
        .req_ej_crd      (ej_crd[N_REQ][y][x]),
        .rsp_inj_vld     (inj_vld[N_RSP][y][x]),
        .rsp_inj_flit    (inj_flit[N_RSP][y][x]),
        .rsp_inj_crd     (inj_crd[N_RSP][y][x]),
        .rsp_ej_vld      (ej_vld[N_RSP][y][x]),
        .rsp_ej_flit     (ej_flit[N_RSP][y][x]),
        .rsp_ej_crd      (ej_crd[N_RSP][y][x]),
        .ejb_mem_wr_en   (ejb_wr_en),
        .ejb_mem_wr_addr (ejb_wr_addr),
        .ejb_mem_wr_data (ejb_wr_data),
        .ejb_mem_rd_en   (ejb_rd_en),
        .ejb_mem_rd_addr (ejb_rd_addr),
        .ejb_mem_rd_data (ejb_rd_data),
        .cdc_mem_wr_en   (cdc_mem_wr_en[y][x]),
        .cdc_mem_wr_addr (cdc_mem_wr_addr[y][x]),
        .cdc_mem_wr_data (cdc_mem_wr_data[y][x]),
        .cdc_mem_rd_en   (cdc_mem_rd_en[y][x]),
        .cdc_mem_rd_addr (cdc_mem_rd_addr[y][x]),
        .cdc_mem_rd_data (cdc_mem_rd_data[y][x])
      );

      // NI eject buffers are entry 5 of each network's buffer memories
      for (genvar n = 0; n < 2; n++) begin : g_ejb
        assign buf_mem_wr_en[y][x][n][5]   = ejb_wr_en[n];
        assign buf_mem_wr_addr[y][x][n][5] = ejb_wr_addr[n];
        assign buf_mem_wr_data[y][x][n][5] = ejb_wr_data[n];
        assign buf_mem_rd_en[y][x][n][5]   = ejb_rd_en[n];
        assign buf_mem_rd_addr[y][x][n][5] = ejb_rd_addr[n];
        assign ejb_rd_data[n]              = buf_mem_rd_data[y][x][n][5];
      end
    end
  end

endmodule
