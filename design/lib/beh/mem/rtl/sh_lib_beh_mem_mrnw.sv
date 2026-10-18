// sh_lib_beh_mem_mrnw: behavioral memory with M read and N write ports, single clock
//
// Full-word-write version of sh_lib_beh_mem_mrnw_bw (all bit enables tied high).
// See sh_lib_beh_mem_mrnw_bw for parameter and port behavior.
//
// Parameters:
//   WIDTH     - data width in bits
//   DEPTH     - number of words
//   NUM_RD    - number of read ports (M)
//   NUM_WR    - number of write ports (N)
//   RD_DEL    - read latency in clk cycles; 0 = combinational read
//   RST_EN    - read-data reset: 1 = resettable read data flops, 0 = no reset
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//   RST_VAL   - value of every read flop (and so rd_data) while reset is asserted
//   AW        - address width (derived from DEPTH, do not override)

module sh_lib_beh_mem_mrnw #(
  parameter int unsigned      WIDTH     = 32,
  parameter int unsigned      DEPTH     = 256,
  parameter int unsigned      NUM_RD    = 1,
  parameter int unsigned      NUM_WR    = 1,
  parameter int unsigned      RD_DEL    = 1,
  parameter logic             RST_EN    = 1'b1,
  parameter logic             RST_LOW   = 1'b1,
  parameter logic             RST_ASYNC = 1'b1,
  parameter logic [WIDTH-1:0] RST_VAL   = '0,
  parameter int unsigned      AW        = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic                         clk,
  input  logic                         rst,
  // Write ports
  input  logic [NUM_WR-1:0]            wr_en,
  input  logic [NUM_WR-1:0][AW-1:0]    wr_addr,
  input  logic [NUM_WR-1:0][WIDTH-1:0] wr_data,
  // Read ports
  input  logic [NUM_RD-1:0]            rd_en,
  input  logic [NUM_RD-1:0][AW-1:0]    rd_addr,
  output logic [NUM_RD-1:0][WIDTH-1:0] rd_data,
  output logic [NUM_RD-1:0]            rd_vld
);

  sh_lib_beh_mem_mrnw_bw #(
    .WIDTH     (WIDTH),
    .DEPTH     (DEPTH),
    .NUM_RD    (NUM_RD),
    .NUM_WR    (NUM_WR),
    .RD_DEL    (RD_DEL),
    .RST_EN    (RST_EN),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC),
    .RST_VAL   (RST_VAL),
    .AW        (AW)
  ) u_mem (
    .clk       (clk),
    .rst       (rst),
    .wr_en     (wr_en),
    .wr_addr   (wr_addr),
    .wr_data   (wr_data),
    .wr_wen    ('1),
    .rd_en     (rd_en),
    .rd_addr   (rd_addr),
    .rd_data   (rd_data),
    .rd_vld    (rd_vld)
  );

endmodule
