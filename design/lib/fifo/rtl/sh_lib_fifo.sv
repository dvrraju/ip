// sh_lib_fifo: single-clock first-word-fall-through FIFO
//
// Parameters:
//   WIDTH      - data width in bits
//   DEPTH      - memory entries (any value >= 1)
//   MEM_RD_DEL - memory read latency in clk cycles; 0 = combinational read
//   RST_LOW    - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC  - reset type:  1 = asynchronous, 0 = synchronous
//   AW         - memory address width (derived from DEPTH, do not override)
//
// wr_vld/wr_rdy and rd_vld/rd_rdy are valid/ready handshakes. rd_data shows
// the oldest entry whenever rd_vld is high. A full FIFO does not accept a
// write in the same cycle as a read (wr_rdy depends only on state).
//
// Storage is external: connect mem_* to a DEPTH x WIDTH memory with one write
// port (written on the clk edge when mem_wr_en is high) and one read port:
// mem_rd_data is the word at mem_rd_addr MEM_RD_DEL cycles after mem_rd_en
// (in the same cycle when MEM_RD_DEL = 0).
//
// With MEM_RD_DEL > 0, entries are read ahead into a small flop buffer (see
// sh_lib_fifo_rd_pipe), so the FIFO holds up to DEPTH + MEM_RD_DEL + 2
// entries and an entry takes MEM_RD_DEL + 1 extra cycles to reach rd_*.

module sh_lib_fifo #(
  parameter int unsigned WIDTH      = 8,
  parameter int unsigned DEPTH      = 4,
  parameter int unsigned MEM_RD_DEL = 0,
  parameter logic        RST_LOW    = 1'b1,
  parameter logic        RST_ASYNC  = 1'b1,
  parameter int unsigned AW         = (DEPTH > 1) ? $clog2(DEPTH) : 1
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
  logic          issue;      // entry read out of the memory
  logic          mem_avail;
  logic [AW-1:0] wr_ptr;
  logic [AW-1:0] rd_ptr;
  logic [CW-1:0] cnt;        // entries in the memory

  assign wr_rdy    = (cnt != CW'(DEPTH));
  assign mem_avail = (cnt != '0);
  assign push      = wr_vld && wr_rdy;

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
    .en  (issue),
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
    .en  (push ^ issue),
    .d   (push ? cnt + 1'b1 : cnt - 1'b1),
    .q   (cnt)
  );

  assign mem_wr_en   = push;
  assign mem_wr_addr = wr_ptr;
  assign mem_wr_data = wr_data;
  assign mem_rd_addr = rd_ptr;

  sh_lib_fifo_rd_pipe #(
    .WIDTH      (WIDTH),
    .MEM_RD_DEL (MEM_RD_DEL),
    .RST_LOW    (RST_LOW),
    .RST_ASYNC  (RST_ASYNC)
  ) u_rd_pipe (
    .clk         (clk),
    .rst         (rst),
    .mem_avail   (mem_avail),
    .issue       (issue),
    .mem_rd_en   (mem_rd_en),
    .mem_rd_data (mem_rd_data),
    .rd_vld      (rd_vld),
    .rd_rdy      (rd_rdy),
    .rd_data     (rd_data)
  );

endmodule
