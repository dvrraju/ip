// sh_noc_net: one ROWS x COLS mesh of sh_noc_router (sizes from sh_noc_pkg)
//
// Parameters:
//   BUF_DEPTH      - router input buffer depth; the Local receivers outside
//                    (loc_out_*) must also have BUF_DEPTH entries
//   BUF_MEM_RD_DEL - router buffer memory read latency; 0 = combinational
//   RST_LOW        - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC      - reset type:  1 = asynchronous, 0 = synchronous
//   BAW            - buffer address width (derived, do not override)
//
// loc_in_*[y][x] is the credit-based link into router (y, x)'s Local input
// (loc_in_crd returns its credits); loc_out_*[y][x] is the link out of its
// Local output (loc_out_crd takes credits back from the receiver).
// buf_mem_*[y][x][p] is router (y, x)'s input buffer memory for port p
// (0 = Local, 1 = N, 2 = E, 3 = S, 4 = W), BUF_DEPTH x FLIT_LINK_W on clk.

module sh_noc_net
  import sh_noc_pkg::*;
#(
  parameter int unsigned BUF_DEPTH      = 4,
  parameter int unsigned BUF_MEM_RD_DEL = 0,
  parameter logic        RST_LOW        = 1'b1,
  parameter logic        RST_ASYNC      = 1'b1,
  parameter int unsigned BAW            = (BUF_DEPTH > 1) ? $clog2(BUF_DEPTH) : 1
) (
  input  logic                                               clk,
  input  logic                                               rst,
  // Local ports
  input  logic  [ROWS-1:0][COLS-1:0]                         loc_in_vld,
  input  flit_t [ROWS-1:0][COLS-1:0]                         loc_in_flit,
  output logic  [ROWS-1:0][COLS-1:0]                         loc_in_crd,
  output logic  [ROWS-1:0][COLS-1:0]                         loc_out_vld,
  output flit_t [ROWS-1:0][COLS-1:0]                         loc_out_flit,
  input  logic  [ROWS-1:0][COLS-1:0]                         loc_out_crd,
  // Router buffer memories
  output logic  [ROWS-1:0][COLS-1:0][4:0]                    buf_mem_wr_en,
  output logic  [ROWS-1:0][COLS-1:0][4:0][BAW-1:0]           buf_mem_wr_addr,
  output logic  [ROWS-1:0][COLS-1:0][4:0][FLIT_LINK_W-1:0]   buf_mem_wr_data,
  output logic  [ROWS-1:0][COLS-1:0][4:0]                    buf_mem_rd_en,
  output logic  [ROWS-1:0][COLS-1:0][4:0][BAW-1:0]           buf_mem_rd_addr,
  input  logic  [ROWS-1:0][COLS-1:0][4:0][FLIT_LINK_W-1:0]   buf_mem_rd_data
);

  localparam int unsigned P_L = 0;
  localparam int unsigned P_N = 1;
  localparam int unsigned P_E = 2;
  localparam int unsigned P_S = 3;
  localparam int unsigned P_W = 4;

  // Router port signals, [y][x][port]. Outputs on the mesh edge go nowhere.
  logic  [4:0] r_in_vld   [ROWS][COLS];
  flit_t [4:0] r_in_flit  [ROWS][COLS];
  /* verilator lint_off UNUSEDSIGNAL */
  logic  [4:0] r_in_crd   [ROWS][COLS];
  logic  [4:0] r_out_vld  [ROWS][COLS];
  flit_t [4:0] r_out_flit [ROWS][COLS];
  /* verilator lint_on UNUSEDSIGNAL */
  logic  [4:0] r_out_crd  [ROWS][COLS];

  for (genvar y = 0; y < ROWS; y++) begin : g_row
    for (genvar x = 0; x < COLS; x++) begin : g_col

      sh_noc_router #(
        .X              (x),
        .Y              (y),
        .BUF_DEPTH      (BUF_DEPTH),
        .BUF_MEM_RD_DEL (BUF_MEM_RD_DEL),
        .RST_LOW        (RST_LOW),
        .RST_ASYNC      (RST_ASYNC)
      ) u_router (
        .clk             (clk),
        .rst             (rst),
        .in_vld          (r_in_vld[y][x]),
        .in_flit         (r_in_flit[y][x]),
        .in_crd          (r_in_crd[y][x]),
        .out_vld         (r_out_vld[y][x]),
        .out_flit        (r_out_flit[y][x]),
        .out_crd         (r_out_crd[y][x]),
        .buf_mem_wr_en   (buf_mem_wr_en[y][x]),
        .buf_mem_wr_addr (buf_mem_wr_addr[y][x]),
        .buf_mem_wr_data (buf_mem_wr_data[y][x]),
        .buf_mem_rd_en   (buf_mem_rd_en[y][x]),
        .buf_mem_rd_addr (buf_mem_rd_addr[y][x]),
        .buf_mem_rd_data (buf_mem_rd_data[y][x])
      );

      // Local port
      assign r_in_vld[y][x][P_L]  = loc_in_vld[y][x];
      assign r_in_flit[y][x][P_L] = loc_in_flit[y][x];
      assign loc_in_crd[y][x]     = r_in_crd[y][x][P_L];
      assign loc_out_vld[y][x]    = r_out_vld[y][x][P_L];
      assign loc_out_flit[y][x]   = r_out_flit[y][x][P_L];
      assign r_out_crd[y][x][P_L] = loc_out_crd[y][x];

      // Each link: flits from the neighbour's facing output, credits from the
      // neighbour's facing input. Edge ports are tied off.
      if (y < ROWS - 1) begin : g_n
        assign r_in_vld[y][x][P_N]  = r_out_vld[y+1][x][P_S];
        assign r_in_flit[y][x][P_N] = r_out_flit[y+1][x][P_S];
        assign r_out_crd[y][x][P_N] = r_in_crd[y+1][x][P_S];
      end else begin : g_n_edge
        assign r_in_vld[y][x][P_N]  = 1'b0;
        assign r_in_flit[y][x][P_N] = '0;
        assign r_out_crd[y][x][P_N] = 1'b0;
      end

      if (x < COLS - 1) begin : g_e
        assign r_in_vld[y][x][P_E]  = r_out_vld[y][x+1][P_W];
        assign r_in_flit[y][x][P_E] = r_out_flit[y][x+1][P_W];
        assign r_out_crd[y][x][P_E] = r_in_crd[y][x+1][P_W];
      end else begin : g_e_edge
        assign r_in_vld[y][x][P_E]  = 1'b0;
        assign r_in_flit[y][x][P_E] = '0;
        assign r_out_crd[y][x][P_E] = 1'b0;
      end

      if (y > 0) begin : g_s
        assign r_in_vld[y][x][P_S]  = r_out_vld[y-1][x][P_N];
        assign r_in_flit[y][x][P_S] = r_out_flit[y-1][x][P_N];
        assign r_out_crd[y][x][P_S] = r_in_crd[y-1][x][P_N];
      end else begin : g_s_edge
        assign r_in_vld[y][x][P_S]  = 1'b0;
        assign r_in_flit[y][x][P_S] = '0;
        assign r_out_crd[y][x][P_S] = 1'b0;
      end

      if (x > 0) begin : g_w
        assign r_in_vld[y][x][P_W]  = r_out_vld[y][x-1][P_E];
        assign r_in_flit[y][x][P_W] = r_out_flit[y][x-1][P_E];
        assign r_out_crd[y][x][P_W] = r_in_crd[y][x-1][P_E];
      end else begin : g_w_edge
        assign r_in_vld[y][x][P_W]  = 1'b0;
        assign r_in_flit[y][x][P_W] = '0;
        assign r_out_crd[y][x][P_W] = 1'b0;
      end

    end
  end

endmodule
