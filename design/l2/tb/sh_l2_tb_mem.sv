// sh_l2_tb_mem: simulation model of a 1R1W memory with a write mask, for the
// sh_l2 array ports
//
// Parameters:
//   WIDTH  - data width in bits
//   DEPTH  - number of entries
//   MASK_W - write mask bits; each covers WIDTH / MASK_W data bits
//   RD_DEL - read latency in clk cycles; 0 = combinational read
//   AW     - address width (derived, do not override)
//
// Writes happen on the clk edge where wr_en is high, only for the slices
// whose wr_mask bit is set. With RD_DEL > 0 a read is sampled on the edge
// where rd_en is high and its entry is on rd_data RD_DEL cycles later, for
// that one cycle only: every other cycle rd_data is random, so a design that
// samples at the wrong time fails. A read in the same cycle as a write of the
// same entry returns the old value.

module sh_l2_tb_mem #(
  parameter int unsigned WIDTH  = 8,
  parameter int unsigned DEPTH  = 4,
  parameter int unsigned MASK_W = 1,
  parameter int unsigned RD_DEL = 0,
  parameter int unsigned AW     = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic              clk,
  input  logic              wr_en,
  input  logic [AW-1:0]     wr_addr,
  input  logic [WIDTH-1:0]  wr_data,
  input  logic [MASK_W-1:0] wr_mask,
  input  logic              rd_en,
  input  logic [AW-1:0]     rd_addr,
  output logic [WIDTH-1:0]  rd_data
);

  localparam int unsigned SLICE = WIDTH / MASK_W;

  logic [WIDTH-1:0] mem [DEPTH];

  initial begin
    for (int i = 0; i < DEPTH; i++) mem[i] = WIDTH'({8{$urandom}});   // contents start unknown
  end

  always @(posedge clk) begin
    if (wr_en) begin
      if (32'(wr_addr) >= DEPTH) $error("%m: write address %0d out of range", wr_addr);
      for (int m = 0; m < MASK_W; m++) begin
        if (wr_mask[m]) mem[wr_addr][m*SLICE +: SLICE] <= wr_data[m*SLICE +: SLICE];
      end
    end
  end

  if (RD_DEL == 0) begin : g_comb
    assign rd_data = mem[rd_addr];
  end else begin : g_pipe
    logic             vld  [RD_DEL];
    logic [WIDTH-1:0] data [RD_DEL];

    always @(posedge clk) begin
      if (rd_en && 32'(rd_addr) >= DEPTH) $error("%m: read address %0d out of range", rd_addr);
      vld[0]  <= rd_en;
      data[0] <= rd_en ? mem[rd_addr] : WIDTH'({8{$urandom}});
      for (int i = 1; i < RD_DEL; i++) begin
        vld[i]  <= vld[i-1];
        data[i] <= vld[i-1] ? data[i-1] : WIDTH'({8{$urandom}});
      end
    end

    assign rd_data = vld[RD_DEL-1] ? data[RD_DEL-1] : ~data[RD_DEL-1];
  end

endmodule
