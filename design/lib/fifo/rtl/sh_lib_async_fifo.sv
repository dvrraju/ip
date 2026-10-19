// sh_lib_async_fifo: dual-clock first-word-fall-through FIFO (gray-code pointers)
//
// Parameters:
//   WIDTH      - data width in bits
//   DEPTH      - number of entries (power of 2, minimum 2)
//   SYNC_DELAY - synchronizer flops on each pointer crossing (minimum 2)
//   RST_LOW    - reset sense: 1 = active-low, 0 = active-high (both domains)
//   RST_ASYNC  - reset type:  1 = asynchronous, 0 = synchronous (both domains)
//   AW         - memory address width (derived from DEPTH, do not override)
//
// wr_vld/wr_rdy (wr_clk) and rd_vld/rd_rdy (rd_clk) are valid/ready handshakes.
// Each side sees the other side's pointer SYNC_DELAY+ cycles late, so wr_rdy
// and rd_vld are conservative: a slot freed by a read (or an entry added by a
// write) shows up on the other side a few cycles later.
//
// Storage is external: connect mem_* to a DEPTH x WIDTH memory with a write
// port in wr_clk (written when mem_wr_en is high) and a combinational read
// port (mem_rd_data is the word at mem_rd_addr, used in rd_clk). Reading
// across the clock boundary is safe because an entry is not visible to the
// reader until its write pointer has crossed. Reset both domains together.

module sh_lib_async_fifo #(
  parameter int unsigned WIDTH      = 8,
  parameter int unsigned DEPTH      = 4,
  parameter int unsigned SYNC_DELAY = 2,
  parameter logic        RST_LOW    = 1'b1,
  parameter logic        RST_ASYNC  = 1'b1,
  parameter int unsigned AW         = $clog2(DEPTH)
) (
  // Write domain
  input  logic             wr_clk,
  input  logic             wr_rst,
  input  logic             wr_vld,
  output logic             wr_rdy,
  input  logic [WIDTH-1:0] wr_data,
  // Read domain
  input  logic             rd_clk,
  input  logic             rd_rst,
  output logic             rd_vld,
  input  logic             rd_rdy,
  output logic [WIDTH-1:0] rd_data,
  // Memory interface: write port in wr_clk, read port in rd_clk
  output logic             mem_wr_en,
  output logic [AW-1:0]    mem_wr_addr,
  output logic [WIDTH-1:0] mem_wr_data,
  output logic             mem_rd_en,
  output logic [AW-1:0]    mem_rd_addr,
  input  logic [WIDTH-1:0] mem_rd_data
);

  if (DEPTH < 2 || (1 << AW) != DEPTH) begin : g_check
    $error("sh_lib_async_fifo: DEPTH must be a power of 2 >= 2, got %0d", DEPTH);
  end

  function automatic logic [AW:0] bin2gray(logic [AW:0] b);
    return b ^ (b >> 1);
  endfunction

  function automatic logic [AW:0] gray2bin(logic [AW:0] g);
    logic [AW:0] b;
    for (int i = 0; i <= AW; i++) b[i] = ^(g >> i);
    return b;
  endfunction

  // ---------------------------------------------------------------------------
  // Write domain
  // ---------------------------------------------------------------------------

  logic        push;
  logic [AW:0] wr_bin;
  logic [AW:0] wr_gray;
  logic [AW:0] rd_gray_s;
  logic [AW:0] rd_bin_s;
  logic        pop;
  logic [AW:0] rd_bin;
  logic [AW:0] rd_gray;
  logic [AW:0] wr_gray_s;

  assign rd_bin_s = gray2bin(rd_gray_s);
  // Full when the pointers differ only in the wrap bit
  assign wr_rdy   = ((wr_bin ^ rd_bin_s) != {1'b1, AW'(0)});
  assign push     = wr_vld && wr_rdy;

  sh_lib_ff #(
    .WIDTH     (AW + 1),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_wr_bin (
    .clk (wr_clk),
    .rst (wr_rst),
    .en  (push),
    .d   (wr_bin + 1'b1),
    .q   (wr_bin)
  );

  // Registered gray copy, so only one bit changes per crossing
  sh_lib_ff #(
    .WIDTH     (AW + 1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_wr_gray (
    .clk (wr_clk),
    .rst (wr_rst),
    .en  (1'b0),
    .d   (bin2gray(wr_bin)),
    .q   (wr_gray)
  );

  sh_lib_sync_bus #(
    .WIDTH     (AW + 1),
    .DELAY     (SYNC_DELAY),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_rd_gray_sync (
    .clk (wr_clk),
    .rst (wr_rst),
    .d   (rd_gray),
    .q   (rd_gray_s)
  );

  // ---------------------------------------------------------------------------
  // Read domain
  // ---------------------------------------------------------------------------

  assign rd_vld = (bin2gray(rd_bin) != wr_gray_s);
  assign pop    = rd_vld && rd_rdy;

  sh_lib_ff #(
    .WIDTH     (AW + 1),
    .EN        (1'b1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_rd_bin (
    .clk (rd_clk),
    .rst (rd_rst),
    .en  (pop),
    .d   (rd_bin + 1'b1),
    .q   (rd_bin)
  );

  sh_lib_ff #(
    .WIDTH     (AW + 1),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_rd_gray (
    .clk (rd_clk),
    .rst (rd_rst),
    .en  (1'b0),
    .d   (bin2gray(rd_bin)),
    .q   (rd_gray)
  );

  sh_lib_sync_bus #(
    .WIDTH     (AW + 1),
    .DELAY     (SYNC_DELAY),
    .RST_LOW   (RST_LOW),
    .RST_ASYNC (RST_ASYNC)
  ) u_wr_gray_sync (
    .clk (rd_clk),
    .rst (rd_rst),
    .d   (wr_gray),
    .q   (wr_gray_s)
  );

  // ---------------------------------------------------------------------------
  // Memory interface
  // ---------------------------------------------------------------------------

  assign mem_wr_en   = push;
  assign mem_wr_addr = wr_bin[AW-1:0];
  assign mem_wr_data = wr_data;
  assign mem_rd_en   = rd_vld;
  assign mem_rd_addr = rd_bin[AW-1:0];
  assign rd_data     = mem_rd_data;

endmodule
