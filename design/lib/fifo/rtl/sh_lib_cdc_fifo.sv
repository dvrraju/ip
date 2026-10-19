// sh_lib_cdc_fifo: FIFO that is either dual-clock or single-clock by parameter
//
// Parameters:
//   WIDTH      - data width in bits
//   DEPTH      - number of entries (power of 2 >= 2 when ASYNC = 1)
//   ASYNC      - 1 = sh_lib_async_fifo between wr_clk and rd_clk
//                0 = sh_lib_fifo on wr_clk/wr_rst; rd_clk/rd_rst are ignored
//                    and must be the same clock/reset
//   SYNC_DELAY - synchronizer flops per pointer crossing (ASYNC = 1 only)
//   MEM_RD_DEL - memory read latency in read-side clock cycles; 0 = combinational
//   RST_LOW    - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC  - reset type:  1 = asynchronous, 0 = synchronous
//   AW         - memory address width (derived from DEPTH, do not override)
//
// Storage is external (see sh_lib_fifo / sh_lib_async_fifo): the memory
// write port is in wr_clk and the read port in rd_clk (both in wr_clk when
// ASYNC = 0).

module sh_lib_cdc_fifo #(
  parameter int unsigned WIDTH      = 8,
  parameter int unsigned DEPTH      = 4,
  parameter logic        ASYNC      = 1'b1,
  parameter int unsigned SYNC_DELAY = 2,
  parameter int unsigned MEM_RD_DEL = 0,
  parameter logic        RST_LOW    = 1'b1,
  parameter logic        RST_ASYNC  = 1'b1,
  parameter int unsigned AW         = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic             wr_clk,
  input  logic             wr_rst,
  input  logic             wr_vld,
  output logic             wr_rdy,
  input  logic [WIDTH-1:0] wr_data,
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic             rd_clk,
  input  logic             rd_rst,
  /* verilator lint_on UNUSEDSIGNAL */
  output logic             rd_vld,
  input  logic             rd_rdy,
  output logic [WIDTH-1:0] rd_data,
  // Memory interface
  output logic             mem_wr_en,
  output logic [AW-1:0]    mem_wr_addr,
  output logic [WIDTH-1:0] mem_wr_data,
  output logic             mem_rd_en,
  output logic [AW-1:0]    mem_rd_addr,
  input  logic [WIDTH-1:0] mem_rd_data
);

  if (ASYNC) begin : g_async
    sh_lib_async_fifo #(
      .WIDTH      (WIDTH),
      .DEPTH      (DEPTH),
      .SYNC_DELAY (SYNC_DELAY),
      .MEM_RD_DEL (MEM_RD_DEL),
      .RST_LOW    (RST_LOW),
      .RST_ASYNC  (RST_ASYNC)
    ) u_fifo (
      .wr_clk      (wr_clk),
      .wr_rst      (wr_rst),
      .wr_vld      (wr_vld),
      .wr_rdy      (wr_rdy),
      .wr_data     (wr_data),
      .rd_clk      (rd_clk),
      .rd_rst      (rd_rst),
      .rd_vld      (rd_vld),
      .rd_rdy      (rd_rdy),
      .rd_data     (rd_data),
      .mem_wr_en   (mem_wr_en),
      .mem_wr_addr (mem_wr_addr),
      .mem_wr_data (mem_wr_data),
      .mem_rd_en   (mem_rd_en),
      .mem_rd_addr (mem_rd_addr),
      .mem_rd_data (mem_rd_data)
    );
  end else begin : g_sync
    sh_lib_fifo #(
      .WIDTH      (WIDTH),
      .DEPTH      (DEPTH),
      .MEM_RD_DEL (MEM_RD_DEL),
      .RST_LOW    (RST_LOW),
      .RST_ASYNC  (RST_ASYNC)
    ) u_fifo (
      .clk         (wr_clk),
      .rst         (wr_rst),
      .wr_vld      (wr_vld),
      .wr_rdy      (wr_rdy),
      .wr_data     (wr_data),
      .rd_vld      (rd_vld),
      .rd_rdy      (rd_rdy),
      .rd_data     (rd_data),
      .mem_wr_en   (mem_wr_en),
      .mem_wr_addr (mem_wr_addr),
      .mem_wr_data (mem_wr_data),
      .mem_rd_en   (mem_rd_en),
      .mem_rd_addr (mem_rd_addr),
      .mem_rd_data (mem_rd_data)
    );
  end

endmodule
