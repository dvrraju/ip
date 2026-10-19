# L2 Cache Micro-Architecture (version 1, blocking)

Companion to `l2_spec.md`. Defaults: 1 MB, 8 ways, 64 B lines (8 flits
of 64 bits), 2048 sets, 32-bit addresses. Array read latency `RD_DEL`
(0..N) is a parameter.

## 1. Block diagram

```
                       sh_l2  (one clock: the L2 tile's clock)
   NoC NI (target side)                                         NoC NI (initiator side)
  ┌───────────────────────────────────────────────────────────────────────────────────┐
  │                                                                                   │
  │  t_req ──►┌──────────────┐ hdr  ┌───────────────────────────┐                     │
  │           │ REQ INTAKE   │─────►│                           │  ┌───────────────┐  │
  │           │ header reg   │      │       CONTROL FSM         │─►│ MEM REQ GEN   │──┼─► i_req
  │           │ write buffer │─────►│  (one request at a time)  │  │ fill RD_REQ   │  │
  │           │ 8 x {strb,d} │ wbuf │                           │  │ wb   WR_REQ   │  │
  │           │ line check   │      │  INIT IDLE LOOKUP COMPARE │  └──────▲────────┘  │
  │           └──────────────┘      │  RD_HIT WR_HIT WB FILL    │         │ wb data   │
  │                                 │  INSTALL RESP REJECT      │         │           │
  │  t_rsp ◄──┌──────────────┐      │                           │  ┌──────┴────────┐  │
  │           │ RSP GEN      │◄─────│                           │◄─│ MEM RSP SINK  │◄─┼── i_rsp
  │           │ header + rd  │ ctl  └──┬────────┬────────┬──────┘  │ fill data,    │  │
  │           │ data stream  │         │        │        │         │ wb ack/error  │  │
  │           └──────▲───────┘         │        │        │         └──────┬────────┘  │
  │                  │ rd data         │        │        │                │fill data  │
  │           ┌──────┴────────────┐    │        │        │                │           │
  │           │ DATA READ STREAM  │◄───┘        │        │                │           │
  │           │ (sh_lib_fifo_rd_  │ word reads  │        │                │           │
  │           │  pipe: hides      │             │        │                │           │
  │           │  RD_DEL)          │             │        │                │           │
  │           └──────▲────────────┘             │        │                │           │
  │                  │                          │        │                │           │
  │  ┌───────────────┴──────────┐ ┌─────────────▼──┐ ┌───▼──────────┐     │           │
  │  │ DATA array port (1R1W)   │ │ TAG array port │ │ LRU array    │     │           │
  │  │ 131072 x 64, byte strb   │ │ 2048 x 136,    │ │ 2048 x 7     │     │           │
  │  │ wr: hit merge / fill ◄───┼─┼─ way mask      │ │ tree PLRU    │     │           │
  │  └──────────▲───────────────┘ └────────────────┘ └──────────────┘     │           │
  │             └─────────────────────────────────────────────────────────┘           │
  │                                                                                   │
  │  wb_err ◄── sticky ◄── MEM RSP SINK (writeback error)        wb_err_clr ──►       │
  └───────────────────────────────────────────────────────────────────────────────────┘
          │ tag_mem_*            │ data_mem_*           │ lru_mem_*
          ▼                      ▼                      ▼
     external 1R1W memories (instantiated outside sh_l2), read latency RD_DEL
```

## 2. Interfaces

| Group | Signals | Notes |
|---|---|---|
| Clock/reset | `clk`, `rst` | L2 tile clock; the NI does the CDC to the NoC |
| Agent requests in | `t_req_vld/rdy/flit` | from the L2 tile's NI target port |
| Agent responses out | `t_rsp_vld/rdy/flit` | dst = request src; NI stamps src |
| Memory requests out | `i_req_vld/rdy/flit` | fills and writebacks; NI routes by `ADDR_MAP` (`IS_L2`) |
| Memory responses in | `i_rsp_vld/rdy/flit` | fill data, writeback acks |
| Tag array | `tag_mem_{wr_en, wr_addr, wr_data, wr_mask, rd_en, rd_addr, rd_data}` | `wr_mask` per way |
| Data array | `data_mem_{wr_en, wr_addr, wr_data, wr_strb, rd_en, rd_addr, rd_data}` | `wr_strb` per byte |
| LRU array | `lru_mem_{wr_en, wr_addr, wr_data, rd_en, rd_addr, rd_data}` | |
| Status | `wb_err` (out), `wb_err_clr` (in) | sticky writeback error |

Flits are `sh_noc_pkg::flit_t` extended with `strb[FLIT_W/8]` (spec 3.1).

## 3. Array organization

**Address split (defaults):**

```
 31            17 16           6 5     3 2   0
┌────────────────┬──────────────┬───────┬─────┐
│  tag (15)      │  set (11)    │ word  │ byte│
└────────────────┴──────────────┴───────┴─────┘
```

**Tag array** (depth `NUM_SETS`, one entry per set, all ways side by side):

```
 entry[set] = { way7, way6, ..., way0 },  wayN = { valid, dirty, tag[14:0] }   (8 x 17 = 136 bits)
```

Writes use `wr_mask` (one bit per way) so a single way is updated.

**Data array** (one 64-bit word per entry):

```
 data_addr = { set[10:0], way[2:0], word[2:0] }        (17 bits, 131072 entries)
```

A line is 8 consecutive entries. Writes use `wr_strb` (byte enables) so
a partial write updates only its bytes.

**LRU array** (depth `NUM_SETS`): 7-bit tree pseudo-LRU per set.

```
            b0
         /      \
       b1        b2           bit = 0: next victim is in the left subtree
      /  \      /  \          bit = 1: next victim is in the right subtree
    b3   b4   b5   b6
    /\   /\   /\   /\
   w0 w1 w2 w3 w4 w5 w6 w7
```

- **Victim:** the lowest-numbered invalid way if any; otherwise follow
  the bits from `b0`.
- **Update on access (hit or install) to way w:** set the 3 bits on w's
  path to point away from w.

## 4. Control FSM

```
            reset
              │
              ▼
          ┌───────┐  walk all sets: tag = 0 (invalid), lru = 0
          │ INIT  │──────────────────────────────┐
          └───────┘                              │ done
                                                 ▼
   ┌──────────────────────────────────────► ┌────────┐ ◄─────────────────────────┐
   │                                        │  IDLE  │                           │
   │                                        └───┬────┘                           │
   │                     request in intake      │                                │
   │              (write: whole packet buffered)│                                │
   │                    ┌───────────────────────┴───────┐                        │
   │       crosses line │                               │ ok                     │
   │                    ▼                               ▼                        │
   │              ┌──────────┐                    ┌──────────┐                   │
   │              │  REJECT  │ SLVERR             │  LOOKUP  │ read tag + lru,   │
   │              └────┬─────┘                    └────┬─────┘ wait RD_DEL       │
   │                   │                               ▼                         │
   │                   │                         ┌──────────┐                    │
   │                   │                         │ COMPARE  │ hit? victim?       │
   │                   │                         └─┬──┬──┬──┘                    │
   │                   │        read hit ┌─────────┘  │  └──────┐ miss           │
   │                   │                 ▼            ▼         ▼                │
   │                   │           ┌─────────┐  ┌─────────┐  victim dirty?       │
   │                   │           │ RD_HIT  │  │ WR_HIT  │   yes │      │ no    │
   │                   │           │ stream  │  │ merge   │       ▼      │       │
   │                   │           │ words   │  │ words,  │  ┌────────┐  │       │
   │                   │           │ to RSP  │  │ dirty=1 │  │   WB   │  │       │
   │                   │           └────┬────┘  └────┬────┘  │ stream │  │       │
   │                   │                │            │       │ victim │  │       │
   │                   │                │            │       └───┬────┘  │       │
   │                   │                │            │           ▼       ▼       │
   │                   │                │            │        ┌──────────────┐   │
   │                   │                │            │        │    FILL      │   │
   │                   │                │            │        │ full-line wr:│   │
   │                   │                │            │        │  from wbuf   │   │
   │                   │                │            │        │ else RD_REQ, │   │
   │                   │                │            │        │  write words │   │
   │                   │                │            │        │  as they     │   │
   │                   │                │            │        │  arrive      │   │
   │                   │                │            │        └──────┬───────┘   │
   │                   │                │            │     fill error│  │ ok     │
   │                   │                │            │     (SLVERR)  │  ▼        │
   │                   │                │            │               │ ┌───────┐ │
   │                   │                │            │               │ │INSTALL│ │
   │                   │                │            │               │ │tag,lru│ │
   │                   │                │            │               │ └───┬───┘ │
   │                   │                │            │               │     │     │
   │                   │                │            │               │     └─────┼─► LOOKUP
   │                   │                │            │               │  (replay: │   (now a hit)
   │                   ▼                ▼            ▼               ▼  the hit  │
   │                 ┌─────────────────────────────────────────────────┐  path   │
   └─────────────────│ RESP: send header (+ read data), update lru     │  does   │
                     └─────────────────────────────────────────────────┘  the    │
                                                                         access) │
```

Key choices:

- **Replay after a miss.** A miss only brings the line in (WB, FILL,
  INSTALL) and then goes back to LOOKUP, where the request now hits and
  takes the normal RD_HIT or WR_HIT path. One data path for every access;
  the cost is one extra lookup (about `RD_DEL + 2` cycles) per miss.
- **Full-line write miss** (all 64 strobes set): FILL writes the line
  from the write buffer instead of reading memory, then INSTALL marks it
  dirty; the replay is skipped and the request goes to RESP.
- **Writeback before fill, both complete before INSTALL.** The victim is
  streamed out as a `PKT_WR_REQ` (tag 1), the fill is issued as a
  `PKT_RD_REQ` (tag 0) right after it, and INSTALL waits for both
  responses. A writeback error sets `wb_err` and does not stop the request.
- **Fill error:** an error response carries no data, so no fill words
  were written. The line is not installed and the victim way is left
  as it was (if it was dirty it has been written back but stays marked
  dirty, so it is simply written back again on a later eviction). The
  agent gets a header-only `RSP_SLVERR`.
- **REJECT** consumes nothing more (the write buffer already holds the
  whole packet) and sends a header-only `RSP_SLVERR`.

## 5. Datapaths

**Request intake.** Takes the header flit, then for writes the
`len + 1` data flits into the write buffer (8 x {strb[7:0], data[63:0]},
flops, indexed by `word - first_word`). The line check is
`first_word + len > 7` → REJECT. The FSM starts only once the whole
packet is in, so later stages never wait on the agent.

**Data read stream.** RD_HIT and WB read 8 (or `len + 1`) consecutive
data entries. Reads are issued through `sh_lib_fifo_rd_pipe`, which
issues ahead into a `RD_DEL + 2` flop buffer and hands out words with
valid/ready, so `t_rsp` or `i_req` back-pressure never loses data and
the stream runs one word per cycle when not stalled. Its output is
steered to RSP GEN (read hit) or MEM REQ GEN (writeback, strobes all 1).

**Data writes.** One write port, used by exactly one of:
- WR_HIT: words `first_word .. first_word + len` from the write buffer,
  with their strobes.
- FILL: each fill flit as it arrives from `i_rsp` (all strobes 1), or the
  write buffer for a full-line write.

**Tag/LRU writes.** INIT (clear), INSTALL (new tag, valid, dirty = 0 or
1 for full-line write), WR_HIT (dirty = 1, only if not already dirty),
and an LRU update on every hit and install.

## 6. Hazards

The FSM handles one request at a time, so the only hazards are between
its own steps:

| Hazard | Why it is safe |
|---|---|
| Tag write then the replay LOOKUP of the same set | INSTALL writes in one cycle; LOOKUP reads in a later cycle, so it sees the new entry (1R1W read-during-write returns old data only in the same cycle). |
| Fill writes then replay reads of the same line | All fill writes are done before INSTALL, before the replay. |
| Victim read (WB) then fill writes to the same way | FILL starts after the WB stream has issued all 8 reads; data reads complete `RD_DEL` cycles after issue and the read port returns old data even in the write cycle. |
| WB and fill reordered in the NoC | Different line addresses; both responses are awaited before INSTALL. |
| Request to a line whose writeback is in flight | Cannot happen in v1: the next request starts only after the previous one is complete. |

## 7. Latency (no back-pressure, `RD_DEL` = L)

| Case | L2 cycles from full request in intake to first response flit |
|---|---|
| Read hit | about `2L + 4` |
| Write hit | about `L + 4` (header-only response after the merge) |
| Read miss, clean victim | read hit + fill round trip + `L + 3` (replay) |
| Read miss, dirty victim | as above + 8 cycles to stream the victim out |
| Full-line write miss | `L + 3` + 8 (write the line) + install, no memory read |

NoC hops and the NI CDC add to these.

## 8. Parameters (`sh_l2_pkg` and module)

| Name | Where | Default |
|---|---|---|
| `L2_BYTES`, `NUM_WAYS`, `LINE_BYTES` | `sh_l2_pkg` | 1 MB, 8, 64 |
| `L2_PRESENT`, `L2_X`, `L2_Y` | `sh_noc_pkg` | tile of the L2 (the NoC routes on it) |
| `NUM_SETS`, tag/set/word widths | `sh_l2_pkg` (derived) | 2048, 15/11/3 |
| `FILL_TAG`, `WB_TAG` | `sh_l2_pkg` | 0, 1 |
| `TAG_RD_DEL`, `DATA_RD_DEL`, `LRU_RD_DEL` | module | 0 |
| `RST_LOW`, `RST_ASYNC` | module | 1, 1 |

## 9. Module breakdown

| Module | Contents |
|---|---|
| `sh_l2` | top: wiring, sticky `wb_err` |
| `sh_l2_intake` | header register, write buffer, line and whole-line checks |
| `sh_l2_ctrl` | FSM, tag compare, victim select, PLRU update, data read stream, response generator, fill/writeback requests and `i_rsp` handling |
| `sh_lib_fifo_rd_pipe` | reused for the data read stream |
| `sh_lib_plru` (lib) | tree PLRU victim and update, any power-of-2 ways |

The response and memory request generators planned as separate modules
are small muxes on the FSM state, so they live in `sh_l2_ctrl`.

FSM state names in the RTL: `S_RD_HDR`/`S_RD_DATA` are RD_HIT (header,
then data), `S_WB_HDR`/`S_WB_DATA` are WB, `S_FILL_REQ`/`S_FILL_WAIT` are
FILL, `S_FULL_WR` is the whole-line write, and `S_RSP` covers RESP for
writes and REJECT/error responses.
