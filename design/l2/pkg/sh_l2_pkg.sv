// sh_l2_pkg: unified L2 cache geometry
//
// Configuration (each can be overridden with a +define):
//   L2_BYTES   - capacity in bytes
//   NUM_WAYS   - associativity (power of 2, >= 2)
//   LINE_BYTES - line size in bytes; a line is LINE_BYTES / (FLIT_W / 8) flits
//                and must fit in one NoC packet (<= MAX_LEN flits)
//
// The L2's tile is set in sh_noc_pkg (L2_X, L2_Y), since the NoC routes to it.
//
// Address split (defaults: 1 MB, 8 ways, 64 B lines, 64-bit flits):
//   [ L2_TAG_W | SET_W | WORD_W | BYTE_W ] = [ 15 | 11 | 3 | 3 ]

`ifndef SH_L2_BYTES
  `define SH_L2_BYTES (1024 * 1024)
`endif
`ifndef SH_L2_WAYS
  `define SH_L2_WAYS 8
`endif
`ifndef SH_L2_LINE_BYTES
  `define SH_L2_LINE_BYTES 64
`endif

package sh_l2_pkg;

  import ip_top_pkg::ADDR_W;
  import sh_noc_pkg::FLIT_W;
  import sh_noc_pkg::STRB_W;
  import sh_noc_pkg::TAG_W;

  // ---------------------------------------------------------------------------
  // Configuration
  // ---------------------------------------------------------------------------

  localparam int unsigned L2_BYTES   = `SH_L2_BYTES;
  localparam int unsigned NUM_WAYS   = `SH_L2_WAYS;
  localparam int unsigned LINE_BYTES = `SH_L2_LINE_BYTES;

  // ---------------------------------------------------------------------------
  // Derived
  // ---------------------------------------------------------------------------

  localparam int unsigned WORDS    = LINE_BYTES / STRB_W;               // flits per line
  localparam int unsigned NUM_SETS = L2_BYTES / (LINE_BYTES * NUM_WAYS);

  localparam int unsigned BYTE_W   = $clog2(STRB_W);
  localparam int unsigned WORD_W   = (WORDS > 1) ? $clog2(WORDS) : 1;
  localparam int unsigned OFF_W    = $clog2(LINE_BYTES);
  localparam int unsigned SET_W    = (NUM_SETS > 1) ? $clog2(NUM_SETS) : 1;
  localparam int unsigned IDX_W    = (NUM_SETS > 1) ? $clog2(NUM_SETS) : 0;  // set bits in the address
  localparam int unsigned WAY_W    = $clog2(NUM_WAYS);
  localparam int unsigned L2_TAG_W = ADDR_W - OFF_W - IDX_W;

  // Data array address: {set, way, word}
  localparam int unsigned DATA_AW  = SET_W + WAY_W + WORD_W;

  // Tag array: one row per set, all ways side by side
  typedef struct packed {
    logic                valid;
    logic                dirty;
    logic [L2_TAG_W-1:0] tag;
  } tag_entry_t;

  localparam int unsigned TAG_ENTRY_W = $bits(tag_entry_t);
  localparam int unsigned TAG_ROW_W   = NUM_WAYS * TAG_ENTRY_W;

  // Tree pseudo-LRU bits per set
  localparam int unsigned LRU_W = NUM_WAYS - 1;

  // NoC tags of the L2's own requests
  localparam logic [TAG_W-1:0] FILL_TAG = TAG_W'(0);
  localparam logic [TAG_W-1:0] WB_TAG   = TAG_W'(1);

  // ---------------------------------------------------------------------------
  // Address fields
  // ---------------------------------------------------------------------------

  function automatic logic [SET_W-1:0] addr_set(logic [ADDR_W-1:0] a);
    return (IDX_W == 0) ? '0 : SET_W'(a >> OFF_W);
  endfunction

  function automatic logic [L2_TAG_W-1:0] addr_tag(logic [ADDR_W-1:0] a);
    return L2_TAG_W'(a >> (OFF_W + IDX_W));
  endfunction

  function automatic logic [WORD_W-1:0] addr_word(logic [ADDR_W-1:0] a);
    return (WORDS == 1) ? '0 : WORD_W'(a >> BYTE_W);
  endfunction

  // Line address from a tag and set
  function automatic logic [ADDR_W-1:0] line_addr(logic [L2_TAG_W-1:0] t, logic [SET_W-1:0] s);
    logic [ADDR_W-1:0] a;
    a = ADDR_W'(t) << (OFF_W + IDX_W);
    if (IDX_W > 0) a |= ADDR_W'(s) << OFF_W;
    return a;
  endfunction

endpackage
