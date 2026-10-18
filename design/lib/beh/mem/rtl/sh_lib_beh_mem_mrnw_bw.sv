// sh_lib_beh_mem_mrnw_bw: behavioral bit-writable memory with M read and N write
// ports, single clock
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
//
// Write port j: wr_en[j]=1 writes wr_data[j] to wr_addr[j], only the bits
//               where wr_wen[j] is 1; other bits keep their value.
// Read port i:  rd_en[i]=1 reads rd_addr[i]; rd_data[i] shows the word
//               RD_DEL cycles later and holds it until the next read on that
//               port reaches the output. rd_vld[i] is high in each cycle
//               rd_data[i] shows a newly read word.
//
// Same cycle, same address:
//   - several write ports: per bit, the highest-numbered port with that bit
//     enabled wins.
//   - read and write: the read returns the old data (RD_DEL >= 1). With
//     RD_DEL = 0 the read is combinational, so the new data shows after the
//     write clock edge.
// The reset only applies to the read path; memory contents are never reset.
// rd_vld always resets to 0 (RST_EN only affects the read data), so rst must
// be connected when RD_DEL > 0.

module sh_lib_beh_mem_mrnw_bw #(
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
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic                         rst,
  /* verilator lint_on UNUSEDSIGNAL */
  // Write ports
  input  logic [NUM_WR-1:0]            wr_en,
  input  logic [NUM_WR-1:0][AW-1:0]    wr_addr,
  input  logic [NUM_WR-1:0][WIDTH-1:0] wr_data,
  input  logic [NUM_WR-1:0][WIDTH-1:0] wr_wen,
  // Read ports
  input  logic [NUM_RD-1:0]            rd_en,
  input  logic [NUM_RD-1:0][AW-1:0]    rd_addr,
  output logic [NUM_RD-1:0][WIDTH-1:0] rd_data,
  output logic [NUM_RD-1:0]            rd_vld
);

  logic [WIDTH-1:0] mem [DEPTH];

  // Written bit by bit; later ports override earlier ones on a collision
  always_ff @(posedge clk) begin
    for (int j = 0; j < NUM_WR; j++) begin
      if (wr_en[j]) begin
        for (int b = 0; b < WIDTH; b++) begin
          if (wr_wen[j][b]) mem[wr_addr[j]][b] <= wr_data[j][b];
        end
      end
    end
  end

  for (genvar i = 0; i < NUM_RD; i++) begin : g_rd_port
    if (RD_DEL == 0) begin : g_rd_comb
      assign rd_data[i] = mem[rd_addr[i]];
      assign rd_vld[i]  = rd_en[i];
    end else begin : g_rd_pipe
      logic [WIDTH-1:0] rd_data_q;

      sh_lib_ff #(
        .WIDTH     (WIDTH),
        .EN        (1'b1),
        .RST_EN    (RST_EN),
        .RST_LOW   (RST_LOW),
        .RST_ASYNC (RST_ASYNC),
        .RST_VAL   (RST_VAL)
      ) u_rd_ff (
        .clk (clk),
        .rst (rst),
        .en  (rd_en[i]),
        .d   (mem[rd_addr[i]]),
        .q   (rd_data_q)
      );

      // Remaining RD_DEL-1 cycles of latency
      sh_lib_delay #(
        .WIDTH     (WIDTH),
        .DELAY     (RD_DEL - 1),
        .RST_EN    (RST_EN),
        .RST_LOW   (RST_LOW),
        .RST_ASYNC (RST_ASYNC),
        .RST_VAL   (RST_VAL)
      ) u_rd_dly (
        .clk (clk),
        .rst (rst),
        .d   (rd_data_q),
        .q   (rd_data[i])
      );

      // Read request delayed to line up with rd_data
      sh_lib_delay #(
        .DELAY     (RD_DEL),
        .RST_LOW   (RST_LOW),
        .RST_ASYNC (RST_ASYNC)
      ) u_vld_dly (
        .clk (clk),
        .rst (rst),
        .d   (rd_en[i]),
        .q   (rd_vld[i])
      );
    end
  end

endmodule
