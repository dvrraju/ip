// sh_lib_fifo: single-clock first-word-fall-through FIFO
//
// Parameters:
//   WIDTH     - data width in bits
//   DEPTH     - number of entries (any value >= 1)
//   RST_LOW   - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC - reset type:  1 = asynchronous, 0 = synchronous
//   AW        - memory address width (derived from DEPTH, do not override)
//
// wr_vld/wr_rdy and rd_vld/rd_rdy are valid/ready handshakes. rd_data shows
// the oldest entry whenever rd_vld is high. A full FIFO does not accept a
// write in the same cycle as a read (wr_rdy depends only on state).
//
// Storage is external: connect mem_* to a DEPTH x WIDTH memory with one write
// port (written on the clk edge when mem_wr_en is high) and one combinational
// read port (mem_rd_data is the word at mem_rd_addr in the same cycle).

module sh_lib_fifo #(
  parameter int unsigned WIDTH     = 8,
  parameter int unsigned DEPTH     = 4,
  parameter logic        RST_LOW   = 1'b1,
  parameter logic        RST_ASYNC = 1'b1,
  parameter int unsigned AW        = (DEPTH > 1) ? $clog2(DEPTH) : 1
) (
  input  logic             clk,
  input  logic             rst,
  // Write side
  input  logic             wr_vld,
  output logic             wr_rdy,
  input  logic [WIDTH-1:0] wr_data,
  // Read side
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

  localparam int unsigned CW = $clog2(DEPTH + 1);

  logic          push;
  logic          pop;
  logic [AW-1:0] wr_ptr;
  logic [AW-1:0] rd_ptr;
  logic [CW-1:0] cnt;

  assign wr_rdy = (cnt != CW'(DEPTH));
  assign rd_vld = (cnt != '0);
  assign push   = wr_vld && wr_rdy;
  assign pop    = rd_vld && rd_rdy;

  sh_lib_ff #(
    .WIDTH     (AW),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_wr_ptr (
    .clk (clk),
    .rst (rst),
    .en  (push),
    .d   ((wr_ptr == AW'(DEPTH - 1)) ? '0 : wr_ptr + 1'b1),
    .q   (wr_ptr)
  );

  sh_lib_ff #(
    .WIDTH     (AW),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_rd_ptr (
    .clk (clk),
    .rst (rst),
    .en  (pop),
    .d   ((rd_ptr == AW'(DEPTH - 1)) ? '0 : rd_ptr + 1'b1),
    .q   (rd_ptr)
  );

  sh_lib_ff #(
    .WIDTH     (CW),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_cnt (
    .clk (clk),
    .rst (rst),
    .en  (push ^ pop),
    .d   (push ? cnt + 1'b1 : cnt - 1'b1),
    .q   (cnt)
  );

  assign mem_wr_en   = push;
  assign mem_wr_addr = wr_ptr;
  assign mem_wr_data = wr_data;
  assign mem_rd_en   = rd_vld;
  assign mem_rd_addr = rd_ptr;
  assign rd_data     = mem_rd_data;

endmodule
