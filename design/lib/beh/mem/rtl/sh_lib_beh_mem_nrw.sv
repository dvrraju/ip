// sh_lib_beh_mem_nrw: behavioral memory with N read/write (RW) ports, single clock
//
// Full-word-write version of sh_lib_beh_mem_nrw_bw (all bit enables tied high).
// See sh_lib_beh_mem_nrw_bw for parameter and port behavior.
//
// Parameters:
//   WIDTH     - data width in bits
//   DEPTH     - number of words
//   NUM_RW    - number of RW ports (N)
//   RD_DEL    - read latency in clk cycles; 0 = combinational read
//   RST_EN    - read-data reset: 1 = resettable read data flops, 0 = no reset
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//   RST_VAL   - value of every read flop (and so rd_data) while reset is asserted
//   AW        - address width (derived from DEPTH, do not override)

module sh_lib_beh_mem_nrw #(
  parameter int unsigned      WIDTH     = 32,
  parameter int unsigned      DEPTH     = 256,
  parameter int unsigned      NUM_RW    = 1,
  parameter int unsigned      RD_DEL    = 1,
  parameter logic             RST_EN    = 1'b1,
  parameter logic             RST_LOW   = 1'b1,
  parameter logic             RST_ASYNC = 1'b1,
  parameter logic [WIDTH-1:0] RST_VAL   = '0,
  parameter int unsigned      AW        = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic                         clk,
  input  logic                         rst,
  input  logic [NUM_RW-1:0]            en,
  input  logic [NUM_RW-1:0]            we,
  input  logic [NUM_RW-1:0][AW-1:0]    addr,
  input  logic [NUM_RW-1:0][WIDTH-1:0] wr_data,
  output logic [NUM_RW-1:0][WIDTH-1:0] rd_data,
  output logic [NUM_RW-1:0]            rd_vld
);

  sh_lib_beh_mem_nrw_bw #(
    .WIDTH     (WIDTH),
    .DEPTH     (DEPTH),
    .NUM_RW    (NUM_RW),
    .RD_DEL    (RD_DEL),
    .RST_EN    (RST_EN),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC),
    .RST_VAL   (RST_VAL),
    .AW        (AW)
  ) u_mem (
    .clk     (clk),
    .rst     (rst),
    .en      (en),
    .we      (we),
    .addr    (addr),
    .wr_data (wr_data),
    .wen     ('1),
    .rd_data (rd_data),
    .rd_vld  (rd_vld)
  );

endmodule
