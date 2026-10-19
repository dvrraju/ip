// sh_noc_pkg: mesh NoC configuration, packet format and system address map
//
// Configuration (each can be overridden with a +define, e.g. for regressions):
//   ROWS, COLS - mesh size; tile (y, x), x grows East, y grows North
//   FLIT_W     - flit payload width in bits
//   MAX_LEN    - maximum data flits in one packet
//   TAG_W      - transaction tag width (up to 2**TAG_W outstanding per initiator)
//   L2_PRESENT - 1 = a unified L2 cache sits at tile (L2_Y, L2_X)
//   L2_X, L2_Y - L2 tile (default: the middle of the mesh)
//
// Packets
// -------
// A packet is one header flit followed by 0..MAX_LEN data flits. Every flit
// carries head/tail framing bits and byte strobes next to its FLIT_W payload;
// the header is the first flit (head = 1) and the last flit has tail = 1 (a
// header-only packet has both). In write data flits, strb[i] = 1 means byte i
// of data is written; strobes are ignored in all other flits and should be 0.
// Write addresses are aligned to a flit (FLIT_W/8 bytes).
//
// There are two networks, one per direction:
//   request network  (initiator -> target): PKT_RD_REQ, PKT_WR_REQ
//   response network (target -> initiator): PKT_RD_RSP, PKT_WR_RSP
//
//   type        data flits                      sent by
//   PKT_WR_REQ  len + 1 (write data)            initiator tile
//   PKT_RD_REQ  none (len + 1 requested)        initiator tile
//   PKT_RD_RSP  len + 1 (read data) if RSP_OK,  target tile
//               none if an error
//   PKT_WR_RSP  none                            target tile
//
// Each request gets exactly one response packet. The response copies the
// request's tag and len, and its dst is the request's src.
//
// Header fields (header flit payload bits [HDR_W-1:0], the rest are zero):
//   dst_x/dst_y - destination tile. Requests: filled in by the source NI from
//                 the address map (or the L2 tile, see below). Responses: set
//                 by the target tile to the request's src.
//   src_x/src_y - source tile, filled in by the source NI
//   typ         - packet type (pkt_type_e)
//   tag         - transaction tag chosen by the initiator
//   len         - number of data flits - 1
//   status      - response status (rsp_status_e); 0 in requests
//   addr        - request address (ADDR_W); 0 in responses
//
// Address map
// -----------
// ADDR_MAP lists address regions [base, last] (inclusive), the tile of the
// agent that owns each, and whether the region is cacheable. The first
// matching region wins. A request whose address matches no region is rejected
// by the source NI: it never enters the network and the initiator gets a
// response with status RSP_DECERR and no data.
//
// With L2_PRESENT, an agent's request to a cacheable region goes to the L2
// tile instead of the owner; the L2's own fills and writebacks go to the
// owner. The L2 has no address space of its own, so no region names its tile.
//
// The default map gives each tile t other than the L2 tile the 1/16th at the
// start of the t-th equal slice of the address space; the rest is unmapped.
// Regions of even-numbered tiles are cacheable. Replace ADDR_MAP for a real
// system.

`ifndef SH_NOC_ROWS
  `define SH_NOC_ROWS 4
`endif
`ifndef SH_NOC_COLS
  `define SH_NOC_COLS 4
`endif
`ifndef SH_NOC_FLIT_W
  `define SH_NOC_FLIT_W 64
`endif
`ifndef SH_NOC_MAX_LEN
  `define SH_NOC_MAX_LEN 16
`endif
`ifndef SH_NOC_TAG_W
  `define SH_NOC_TAG_W 4
`endif
`ifndef SH_NOC_L2_PRESENT
  `define SH_NOC_L2_PRESENT ((`SH_NOC_ROWS) * (`SH_NOC_COLS) > 1)
`endif
`ifndef SH_NOC_L2_X
  `define SH_NOC_L2_X ((`SH_NOC_COLS) / 2)
`endif
`ifndef SH_NOC_L2_Y
  `define SH_NOC_L2_Y ((`SH_NOC_ROWS) / 2)
`endif

package sh_noc_pkg;

  import ip_top_pkg::ADDR_W;

  // ---------------------------------------------------------------------------
  // Configuration
  // ---------------------------------------------------------------------------

  localparam int unsigned ROWS    = `SH_NOC_ROWS;
  localparam int unsigned COLS    = `SH_NOC_COLS;
  localparam int unsigned FLIT_W  = `SH_NOC_FLIT_W;
  localparam int unsigned MAX_LEN = `SH_NOC_MAX_LEN;
  localparam int unsigned TAG_W   = `SH_NOC_TAG_W;

  localparam bit          L2_PRESENT = `SH_NOC_L2_PRESENT;
  localparam int unsigned L2_X       = `SH_NOC_L2_X;
  localparam int unsigned L2_Y       = `SH_NOC_L2_Y;

  // Derived
  localparam int unsigned NUM_TILES = ROWS * COLS;
  localparam int unsigned X_W       = (COLS > 1) ? $clog2(COLS) : 1;
  localparam int unsigned Y_W       = (ROWS > 1) ? $clog2(ROWS) : 1;
  localparam int unsigned LEN_W     = (MAX_LEN > 1) ? $clog2(MAX_LEN) : 1;
  localparam int unsigned STRB_W    = FLIT_W / 8;
  localparam int unsigned L2_TILE   = L2_Y * COLS + L2_X;

  // ---------------------------------------------------------------------------
  // Packet format
  // ---------------------------------------------------------------------------

  typedef enum logic [1:0] {
    PKT_RD_REQ = 2'd0,
    PKT_WR_REQ = 2'd1,
    PKT_RD_RSP = 2'd2,
    PKT_WR_RSP = 2'd3
  } pkt_type_e;

  typedef enum logic [1:0] {
    RSP_OK     = 2'd0,
    RSP_SLVERR = 2'd2,   // target error
    RSP_DECERR = 2'd3    // address not in the map (generated by the source NI)
  } rsp_status_e;

  typedef struct packed {
    logic [Y_W-1:0]    dst_y;
    logic [X_W-1:0]    dst_x;
    logic [Y_W-1:0]    src_y;
    logic [X_W-1:0]    src_x;
    pkt_type_e         typ;
    logic [TAG_W-1:0]  tag;
    logic [LEN_W-1:0]  len;
    rsp_status_e       status;
    logic [ADDR_W-1:0] addr;
  } hdr_t;

  localparam int unsigned HDR_W = $bits(hdr_t);

  typedef struct packed {
    logic              head;
    logic              tail;
    logic [STRB_W-1:0] strb;
    logic [FLIT_W-1:0] data;
  } flit_t;

  /* verilator lint_off UNUSEDPARAM */
  localparam int unsigned FLIT_LINK_W = $bits(flit_t);   // for flits carried as plain vectors
  /* verilator lint_on UNUSEDPARAM */

  // Helpers take whole flits/headers and use only some fields
  /* verilator lint_off UNUSEDSIGNAL */

  // Header carried in a head flit's payload
  function automatic hdr_t get_hdr(flit_t f);
    return hdr_t'(f.data[HDR_W-1:0]);
  endfunction

  function automatic logic [FLIT_W-1:0] hdr_to_data(hdr_t h);
    return FLIT_W'(h);
  endfunction

  // Whether a packet with this header has data flits after it
  function automatic logic has_data(hdr_t h);
    return (h.typ == PKT_WR_REQ) || (h.typ == PKT_RD_RSP && h.status == RSP_OK);
  endfunction

  /* verilator lint_on UNUSEDSIGNAL */

  // ---------------------------------------------------------------------------
  // System address map
  // ---------------------------------------------------------------------------

  typedef struct packed {
    logic [ADDR_W-1:0] base;
    logic [ADDR_W-1:0] last;    // inclusive
    logic [Y_W-1:0]    y;
    logic [X_W-1:0]    x;
    logic              cacheable;
  } addr_region_t;

  localparam int unsigned NUM_REGIONS = NUM_TILES - (L2_PRESENT ? 1 : 0);

  // Default map: the address space is sliced evenly by tile, and the first
  // 1/16th of each slice is mapped
  localparam int unsigned SLICE_LOG2 = ADDR_W - ((NUM_TILES > 1) ? $clog2(NUM_TILES) : 0);

  function automatic addr_region_t [NUM_REGIONS-1:0] default_addr_map();
    addr_region_t [NUM_REGIONS-1:0] map;
    int unsigned                    r;
    r = 0;
    for (int unsigned t = 0; t < NUM_TILES; t++) begin
      if (!(L2_PRESENT && t == L2_TILE)) begin
        map[r].base      = ADDR_W'(t) << SLICE_LOG2;
        map[r].last      = map[r].base + ((ADDR_W'(1) << (SLICE_LOG2 - 4)) - 1'b1);
        map[r].x         = X_W'(t % COLS);
        map[r].y         = Y_W'(t / COLS);
        map[r].cacheable = (t % 2 == 0);
        r++;
      end
    end
    return map;
  endfunction

  localparam addr_region_t [NUM_REGIONS-1:0] ADDR_MAP = default_addr_map();

  typedef struct packed {
    logic           hit;
    logic           cacheable;
    logic [Y_W-1:0] y;
    logic [X_W-1:0] x;
  } addr_dec_t;

  // First matching region wins
  function automatic addr_dec_t addr_decode(logic [ADDR_W-1:0] addr);
    addr_dec_t dec;
    dec = '0;
    for (int unsigned r = 0; r < NUM_REGIONS; r++) begin
      if (!dec.hit && addr >= ADDR_MAP[r].base && addr <= ADDR_MAP[r].last) begin
        dec.hit       = 1'b1;
        dec.cacheable = ADDR_MAP[r].cacheable;
        dec.x         = ADDR_MAP[r].x;
        dec.y         = ADDR_MAP[r].y;
      end
    end
    return dec;
  endfunction

endpackage
