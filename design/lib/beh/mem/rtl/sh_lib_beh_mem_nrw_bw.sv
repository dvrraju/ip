// sh_lib_beh_mem_nrw_bw: behavioral bit-writable memory with N read/write (RW)
// ports, single clock
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
//
// Per port i:
//   en[i]=1, we[i]=1: write wr_data[i] to addr[i], only the bits where
//                     wen[i] is 1; other bits keep their value.
//   en[i]=1, we[i]=0: read addr[i]; rd_data[i] shows the word RD_DEL cycles
//                     later and holds it until the next read on that port
//                     reaches the output. A write does not change rd_data[i].
//   rd_vld[i] is high in each cycle rd_data[i] shows a newly read word.
//
// Same cycle, same address:
//   - several ports writing: per bit, the highest-numbered port with that
//     bit enabled wins.
//   - one port reading while another writes: the read returns the old data
//     (RD_DEL >= 1). With RD_DEL = 0 the new data shows after the clock edge.
// The reset only applies to the read path; memory contents are never reset.
// rd_vld always resets to 0 (RST_EN only affects the read data), so rst must
// be connected when RD_DEL > 0.

module sh_lib_beh_mem_nrw_bw #(
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
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic                         rst,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic [NUM_RW-1:0]            en,
  input  logic [NUM_RW-1:0]            we,
  input  logic [NUM_RW-1:0][AW-1:0]    addr,
  input  logic [NUM_RW-1:0][WIDTH-1:0] wr_data,
  input  logic [NUM_RW-1:0][WIDTH-1:0] wen,
  output logic [NUM_RW-1:0][WIDTH-1:0] rd_data,
  output logic [NUM_RW-1:0]            rd_vld
);

  logic [WIDTH-1:0] mem [DEPTH];

  // Written bit by bit; later ports override earlier ones on a collision
  always_ff @(posedge clk) begin
    for (int i = 0; i < NUM_RW; i++) begin
      if (en[i] && we[i]) begin
        for (int b = 0; b < WIDTH; b++) begin
          if (wen[i][b]) mem[addr[i]][b] <= wr_data[i][b];
        end
      end
    end
  end

  for (genvar i = 0; i < NUM_RW; i++) begin : g_port
    if (RD_DEL == 0) begin : g_rd_comb
      assign rd_data[i]  = mem[addr[i]];
      assign rd_vld[i] = en[i] && !we[i];
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
        .en  (en[i] && !we[i]),
        .d   (mem[addr[i]]),
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
        .d   (en[i] && !we[i]),
        .q   (rd_vld[i])
      );
    end
  end

endmodule
