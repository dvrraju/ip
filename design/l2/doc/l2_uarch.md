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

---

# Version 2: hit-under-miss and multiple outstanding misses

Status: implemented (`sh_l2_pfifo`, `sh_l2_ctrl`).

## V2.1 Goal

Version 1 serves one command at a time, so one agent's miss (a memory
round trip, possibly with a writeback) stalls every agent. Version 2 keeps
the lookup and data paths busy while misses are outstanding:

- **Hit-under-miss:** hits are served while fills are in flight.
- **Miss-under-miss:** up to `PF_DEPTH` misses are outstanding at once.
- **Commands to the same line stay in order:** each command depends on
  the one before it to the same line and is not started until that one has
  completed.

Responses to commands for different lines may complete out of order.
Agents match responses by tag.

## V2.2 Pending FIFO

The single-command intake of v1 becomes the **pending FIFO**: `PF_DEPTH`
entries (a `sh_l2_pkg` parameter, default 16, at most 2^`TAG_W` because
fills and writebacks are tagged with the entry index). A command lives in
one entry from arrival until its response is sent; a command that misses
stays in its entry, so the pending FIFO is also the miss tracker. When
every entry is in use, `t_req` is not ready and the NoC holds further
commands.

**Organization.** The FIFO is circular: commands are allocated at the
tail, entries are freed in any order, and the head advances past freed
entries. An entry's age is its distance from the head. A freed entry is
reused only once the head has passed it (and its writeback, if any, has
been answered), so an old miss at the head lowers the usable depth until
it completes.

**Storage.** The pending FIFO memory is an external memory port:
`PF_DEPTH` entries, each holding a whole command in `WORDS + 1` slots of
`SLOT_W` bits (slot 0 the header, slot 1 + i data flit i as {strb, data};
648 bits at defaults), with one write-mask bit per slot, so each flit is
written into its slot as it arrives. When the pipeline picks an entry it
reads it once (latency `PF_RD_DEL`) into the active command register.

Per entry, in flops (searched across all entries): state, line address,
`dep` and `dep_id`, reserved way (`prim`, `way`), `wait_on` (for
`WB_WAIT`), `err`, and `wb_pend` / `wb_line` for a writeback the entry
caused.

```
 t_req ──► accept at the tail ──────────────► ┌──────────────────────────────────┐
           dependency lookup: youngest         │ PENDING FIFO (PF_DEPTH entries)  │
           pending entry of the same line      │ head (oldest) .. tail            │
           -> dep, dep_id                      │ memory: command + write data     │
                                               │ flops: state, line, dep, dep_id, │
                                               │ prim, way, wait_on, err,         │
                                               │ wb_pend, wb_line                 │
                                               └───┬──────────────────▲───────────┘
                     pick: oldest entry with no dep │                 │ free: clear dep of
                     (NEW / READY / NEEDFILL)       ▼                 │ dependents; fill /
                                               ┌──────────┐           │ writeback done: wake
                                               │ LOOKUP   │           │
                                               │ COMPARE  │───────────┘
                                               └────┬─────┘
             hit ┌──────────────────┬───────────────┴───┐ miss
                 ▼                  ▼                   ▼
            ┌─────────┐       ┌──────────┐        ┌───────────┐
            │ RD_HIT  │       │ WR_HIT   │        │ ALLOCATE  │──► i_req
            │ → t_rsp │       │ → ack    │        │ victim WB │    (WB, fill)
            └─────────┘       └──────────┘        │ fill req  │
                                                  └───────────┘
 i_rsp ──► FILL RETURN: write fill words into the reserved way, install the
           tag, entry -> READY; writeback responses clear wb_pend
```

## V2.3 Dependencies

When a command is accepted, the pending FIFO is looked up for pending
commands (any state but `FREE`) to the same line. If there are any, the
new command's `dep` flag is set and `dep_id` records the **youngest** of
them. The commands to a line therefore form a chain, each depending on
exactly one earlier command. A command with `dep` set is never picked.
When a command completes (its response is sent and its entry freed,
whatever its status), every entry with `dep_id` = that entry clears `dep`.

Consequences:

- At most one command per line is past its dependency at any time, so the
  pipeline never holds two commands of one line, and there are no
  secondary misses.
- A command that depends on a miss starts only after that miss has
  filled, replayed and responded, so it finds the line in the cache.
- If the command it depends on failed (`RSP_SLVERR`), the dependent still
  starts; it looks up the line itself.

Cost: one line-address comparator per entry at accept time (26 bits at
defaults), plus a backward priority scan from the tail; the dependency
clear compares `dep_id` (log2 bits) per entry.

## V2.4 Entry states

| State | Meaning |
|---|---|
| `FREE` | unused |
| `LOAD` | receiving its flits from `t_req` |
| `NEW` | complete; may be picked once `dep` is clear |
| `ACTIVE` | in the pipeline |
| `WB_WAIT` | missed; its line's writeback is still outstanding, so the fill waits for that writeback's response |
| `NEEDFILL` | missed; that writeback is answered, ready to send its fill |
| `FILL` | missed; fill outstanding |
| `NO_WAY` | missed, but every way of its set is reserved by outstanding fills; back to `NEW` when a way is released |
| `READY` | its fill is done (or failed): replays through the pipeline |

- **Pick:** the oldest entry (nearest the head) with `dep` clear in
  `NEW`, `READY` or `NEEDFILL`.
- **Miss:** pick a victim way that no other entry has reserved (else
  `NO_WAY`), reserve it (`prim`, `way`), invalidate its tag, write the
  victim back if dirty (NoC tag = entry index; record `wb_line`, set
  `wb_pend`), then send the fill with NoC tag = entry index (`FILL`), or
  wait in `WB_WAIT` if the line's own writeback is still outstanding.
- **Fill returns:** words are written into the reserved way as they
  arrive; on the last word the tag (valid, clean) is installed and the
  entry becomes `READY` (with `err` on a fill error, which installs
  nothing).
- **Replay:** a `READY` entry goes through the pipeline again: it hits
  and completes as in v1, or with `err` gets a header-only `RSP_SLVERR`.
  Its way reservation is released when it completes.
- **Whole-line write miss:** no fill; after any victim writeback it writes
  the line from the active command and installs it dirty, then responds.
  If the line's previous writeback is still outstanding it takes the fill
  path instead, so a line never has two writebacks outstanding.

## V2.5 Memory ordering

Memory owners may handle requests and return responses in any order. The
only ordering the L2 depends on is a dirty victim's **writeback** reaching
memory before a later **fill of the same line**. The L2 enforces it: a
writeback is tagged with the index of the entry that caused it (the
response type, `PKT_WR_RSP` vs `PKT_RD_RSP`, tells it apart from that
entry's fill); the entry records the victim's line and `wb_pend` until the
writeback is answered; and before a miss sends its fill, its line is
compared with every pending writeback's line. On a match it waits in
`WB_WAIT` and sends the fill when that writeback's response arrives.

## V2.6 Hazards

| Hazard | Handling |
|---|---|
| Fill install writes the tag in the same cycle a lookup reads that set (1R1W returns old data) | The command being looked up is of another line (commands of the filling line depend on it). The reserved way is excluded from victim selection, so the stale view of it does no harm. |
| Two lookups to the same set back to back, the first allocating a way | The pipeline finishes allocation (victim tag invalidated) before the next lookup reads tags. |
| Fill data overwriting a way that a hit is reading | The way's tag is invalidated at allocation, so no hit selects it. |
| Tag or data write port wanted by the pipeline and a fill in the same cycle | The pipeline wins; a fill word or install waits a cycle (`i_rsp` is held). Fills always drain. |
| Pending FIFO full of misses | `t_req` back-pressures; fills drain without the pipeline, then replays free entries. No deadlock. |
| An accepted command's predecessor completes in the same cycle | The lookup ignores an entry being freed that cycle. |

## V2.7 NoC tags of the L2's requests

| Request | Tag |
|---|---|
| Fill for entry i | i |
| Writeback caused by entry i | i (its response is `PKT_WR_RSP`, the fill's `PKT_RD_RSP`) |

## V2.8 Cost (defaults: 16 entries, 64-bit flits)

- Flops: about 16 x 72 bits of per-entry state, plus the active command
  register (648 bits, as v1's intake buffer).
- Logic: 16 line comparators at accept (dependency lookup), 16 against
  pending writeback lines, 16 set comparators (reserved ways), 16
  `dep_id` comparators, rotating priority encoders over 16 entries.
- External memory: the pending FIFO, 16 x 648 bits, 1R1W, 9-bit write
  mask.

## V2.9 Hook for maintenance operations (next step)

Flush and invalidate will enter the pipeline as another command source.
"Flush all" waits for the pending FIFO's misses to drain, then walks every
set and way, writing back dirty lines.

## V2.10 Verification

- Unit test: up to 16 outstanding commands with responses matched by tag;
  a memory model that answers fills and writebacks out of order, applies
  writeback data and reads fill data only when it answers (so a fill that
  overtook its writeback would return stale data), and flags such a fill
  directly; buffer depths 2, 4 and 16; read latency on every memory;
  same-line hotspots.
- A simulation check in `sh_l2_pfifo`: of the pending commands of one
  line, only one may have no dependency.
- System test in the mesh, as for v1.
- Mutation checks: dependency never set, dependency on the oldest instead
  of the youngest, dependency cleared at pick instead of completion,
  reserved ways not skipped, picking the newest entry, not waiting for a
  line's writeback.
