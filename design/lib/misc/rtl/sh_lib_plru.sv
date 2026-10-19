// sh_lib_plru: tree pseudo-LRU victim selection and update (combinational)
//
// Parameters:
//   WAYS - number of ways (power of 2, >= 2)
//
// The state is a binary tree of WAYS-1 bits stored in heap order: node n has
// children 2n+1 and 2n+2, and the leaves are ways 0 .. WAYS-1 from left to
// right. A node bit of 0 means the next victim is in its left subtree, 1 the
// right subtree.
//
//   victim    - the lowest-numbered way whose valid bit is 0, if any;
//               otherwise the way reached by following the bits from the root
//   state_upd - state after an access to way acc_way: every node on the path
//               to acc_way points away from it

module sh_lib_plru #(
  parameter int unsigned WAYS = 8,
  parameter int unsigned WW   = $clog2(WAYS)   // do not override
) (
  input  logic [WAYS-2:0] state,
  input  logic [WAYS-1:0] valid,
  output logic [WW-1:0]   victim,
  input  logic [WW-1:0]   acc_way,
  output logic [WAYS-2:0] state_upd
);

  if (WAYS < 2 || (1 << WW) != WAYS) begin : g_check
    $error("sh_lib_plru: WAYS must be a power of 2 >= 2, got %0d", WAYS);
  end

  always_comb begin
    int unsigned node;
    logic        found;

    // Invalid ways first
    node   = 0;
    found  = 1'b0;
    victim = '0;
    for (int unsigned w = 0; w < WAYS; w++) begin
      if (!found && !valid[w]) begin
        found  = 1'b1;
        victim = WW'(w);
      end
    end

    // Otherwise walk the tree
    if (!found) begin
      for (int unsigned lvl = 0; lvl < WW; lvl++) begin
        node = 2 * node + 1 + 32'(state[node]);
      end
      victim = WW'(node - (WAYS - 1));
    end
  end

  always_comb begin
    int unsigned node;
    logic        dir;

    state_upd = state;
    node      = 0;
    for (int unsigned lvl = 0; lvl < WW; lvl++) begin
      dir             = acc_way[WW - 1 - lvl];   // 0 = left, 1 = right
      state_upd[node] = !dir;
      node            = 2 * node + 1 + 32'(dir);
    end
  end

endmodule
