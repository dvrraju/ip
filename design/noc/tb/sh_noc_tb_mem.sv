// sh_noc_tb_mem: simulation model of a 1-write / 1-read memory for the
// sh_noc_mesh FIFO memory ports
//
// Parameters:
//   WIDTH  - data width in bits
//   DEPTH  - number of words
//   RD_DEL - read latency in rd_clk cycles; 0 = combinational read
//   AW     - address width (derived from DEPTH, do not override)
//
// Writes happen on wr_clk. With RD_DEL > 0, a read is sampled on the rd_clk
// edge where rd_en is high and its word is on rd_data RD_DEL cycles later,
// for that one cycle only: every other cycle rd_data is random, so a FIFO
// that samples at the wrong time fails.

module sh_noc_tb_mem #(
  parameter int unsigned WIDTH  = 8,
  parameter int unsigned DEPTH  = 4,
  parameter int unsigned RD_DEL = 0,
  parameter int unsigned AW     = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic             wr_clk,
  input  logic             wr_en,
  input  logic [AW-1:0]    wr_addr,
  input  logic [WIDTH-1:0] wr_data,
  input  logic             rd_clk,
  input  logic             rd_en,
  input  logic [AW-1:0]    rd_addr,
  output logic [WIDTH-1:0] rd_data
);

  logic [WIDTH-1:0] mem [DEPTH];

  always @(posedge wr_clk) begin
    if (wr_en) begin
      if (32'(wr_addr) >= DEPTH) $error("%m: write address %0d out of range", wr_addr);
      mem[wr_addr] <= wr_data;
    end
  end

  if (RD_DEL == 0) begin : g_comb
    assign rd_data = mem[rd_addr];
  end else begin : g_pipe
    logic             vld  [RD_DEL];
    logic [WIDTH-1:0] data [RD_DEL];

    always @(posedge rd_clk) begin
      vld[0]  <= rd_en;
      data[0] <= rd_en ? mem[rd_addr] : WIDTH'({4{$urandom}});
      for (int i = 1; i < RD_DEL; i++) begin
        vld[i]  <= vld[i-1];
        data[i] <= vld[i-1] ? data[i-1] : WIDTH'({4{$urandom}});
      end
    end

    assign rd_data = vld[RD_DEL-1] ? data[RD_DEL-1] : ~data[RD_DEL-1];
  end

endmodule
