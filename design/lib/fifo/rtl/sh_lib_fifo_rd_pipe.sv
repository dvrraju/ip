// sh_lib_fifo_rd_pipe: FIFO read side for a memory with read latency
//
// Parameters:
//   WIDTH      - data width in bits
//   MEM_RD_DEL - memory read latency in clk cycles: mem_rd_data holds the word
//                addressed while mem_rd_en was high MEM_RD_DEL cycles earlier.
//                0 = combinational read.
//   RST_LOW    - reset sense: 1 = active-low, 0 = active-high
//   RST_ASYNC  - reset type:  1 = asynchronous, 0 = synchronous
//   OB_DEPTH   - output buffer entries (derived, do not override)
//
// mem_avail is high while the memory holds an entry not yet read; issue is
// high in each cycle that entry is read (mem_rd_en), and the FIFO then moves
// its read pointer, which also frees the memory slot.
//
// MEM_RD_DEL = 0: a pass-through; issue is the pop and rd_data is mem_rd_data.
// MEM_RD_DEL > 0: reads are issued ahead into an OB_DEPTH = MEM_RD_DEL + 2
// entry flop buffer, which rd_* drains, so rd_data still shows the head entry
// whenever rd_vld is high and back-to-back pops run at full rate. An entry
// reaches rd_* MEM_RD_DEL + 1 cycles after mem_avail goes high.

module sh_lib_fifo_rd_pipe #(
  parameter int unsigned WIDTH      = 8,
  parameter int unsigned MEM_RD_DEL = 1,
  parameter logic        RST_LOW    = 1'b1,
  parameter logic        RST_ASYNC  = 1'b1,
  parameter int unsigned OB_DEPTH   = MEM_RD_DEL + 2
) (
  /* verilator lint_off UNUSEDSIGNAL */
  input  logic             clk,
  input  logic             rst,
  /* verilator lint_on UNUSEDSIGNAL */
  // Memory side
  input  logic             mem_avail,
  output logic             issue,
  output logic             mem_rd_en,
  input  logic [WIDTH-1:0] mem_rd_data,
  // FIFO read side
  output logic             rd_vld,
  input  logic             rd_rdy,
  output logic [WIDTH-1:0] rd_data
);

  if (MEM_RD_DEL == 0) begin : g_comb
    assign mem_rd_en = mem_avail;
    assign issue     = mem_avail && rd_rdy;
    assign rd_vld    = mem_avail;
    assign rd_data   = mem_rd_data;

  end else begin : g_pipe
    localparam int unsigned PW = (OB_DEPTH > 1) ? $clog2(OB_DEPTH) : 1;
    localparam int unsigned CW = $clog2(OB_DEPTH + 1);

    logic             pop;
    logic             arrive;
    logic [CW-1:0]    occ;       // entries in flight plus entries buffered
    logic [CW-1:0]    ob_cnt;    // entries buffered
    logic [PW-1:0]    wr_ptr;
    logic [PW-1:0]    rd_ptr;
    logic [WIDTH-1:0] ob [OB_DEPTH];

    // Only read when the data is sure to have a buffer slot when it arrives
    assign issue     = mem_avail && (occ != CW'(OB_DEPTH));
    assign mem_rd_en = issue;
    assign rd_vld    = (ob_cnt != '0);
    assign pop       = rd_vld && rd_rdy;
    assign rd_data   = ob[rd_ptr];

    // Marks the cycle each read's data is on mem_rd_data
    sh_lib_delay #(
      .DELAY     (MEM_RD_DEL),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_arrive (
      .clk (clk),
      .rst (rst),
      .d   (issue),
      .q   (arrive)
    );

    sh_lib_ff #(
      .WIDTH     (CW),
      .EN        (1'b1),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_occ (
      .clk (clk),
      .rst (rst),
      .en  (issue ^ pop),
      .d   (issue ? occ + 1'b1 : occ - 1'b1),
      .q   (occ)
    );

    sh_lib_ff #(
      .WIDTH     (CW),
      .EN        (1'b1),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_ob_cnt (
      .clk (clk),
      .rst (rst),
      .en  (arrive ^ pop),
      .d   (arrive ? ob_cnt + 1'b1 : ob_cnt - 1'b1),
      .q   (ob_cnt)
    );

    sh_lib_ff #(
      .WIDTH     (PW),
      .EN        (1'b1),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_wr_ptr (
      .clk (clk),
      .rst (rst),
      .en  (arrive),
      .d   ((wr_ptr == PW'(OB_DEPTH - 1)) ? '0 : wr_ptr + 1'b1),
      .q   (wr_ptr)
    );

    sh_lib_ff #(
      .WIDTH     (PW),
      .EN        (1'b1),
      .RST_LOW   (RST_LOW),
      .RST_ASYNC (RST_ASYNC)
    ) u_rd_ptr (
      .clk (clk),
      .rst (rst),
      .en  (pop),
      .d   ((rd_ptr == PW'(OB_DEPTH - 1)) ? '0 : rd_ptr + 1'b1),
      .q   (rd_ptr)
    );

    for (genvar k = 0; k < OB_DEPTH; k++) begin : g_ob
      sh_lib_ff #(
        .WIDTH  (WIDTH),
        .EN     (1'b1),
        .RST_EN (1'b0)
      ) u_ob (
        .clk (clk),
        .rst (1'b0),
        .en  (arrive && (wr_ptr == PW'(k))),
        .d   (mem_rd_data),
        .q   (ob[k])
      );
    end
  end

endmodule
