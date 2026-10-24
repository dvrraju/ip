# L2 Cache Specification (draft)

Status: version 2 implemented (`design/l2/rtl`): hit-under-miss and up to
`PF_DEPTH` outstanding misses; see `l2_uarch.md`, Version 2.

## 1. Overview

The L2 is a single, unified, transparent, write-back cache attached to
the mesh NoC (`sh_noc_mesh`) at one tile.

- **Unified:** one L2 instance caches every cacheable region for every
  agent. There is one copy of each cached line, so no coherence protocol
  is needed. (Agents with private caches of their own are outside this
  spec.)
- **Transparent:** agents read and write system addresses as before. No
  agent addresses the L2, and the L2 has no address space of its own.
- **Cacheable regions only:** requests to non-cacheable regions (device
  registers, etc.) bypass the L2 and go straight to the region's owner.

```
 agent tile                  L2 tile                  memory controller tile
 ┌─────────┐  cacheable    ┌───────────┐  miss /       ┌──────────────┐
 │ agent   │──── req ─────►│    L2     │── writeback ─►│ owner of the │
 │   NI    │◄─── rsp ──────│    NI     │◄── fill ──────│  region (NI) │
 └─────────┘               └───────────┘               └──────────────┘
      │       non-cacheable request (bypasses the L2)          ▲
      └────────────────────────────────────────────────────────┘
```

The L2 occupies its own mesh tile, set by `L2_PRESENT`, `L2_X`, `L2_Y` in
`sh_noc_pkg` (the NoC routes on them, so they live with the NoC
configuration; default: the middle of the mesh). Its NI's target port
receives cacheable requests and its initiator port sends fills and
writebacks.

Note: every cacheable access goes through the L2 tile's one NoC port, so
that link (one flit per cycle in each direction) bounds the total
cacheable bandwidth. Place the L2 centrally to keep hop counts low.

## 2. System address map

`sh_noc_pkg::ADDR_MAP` stays the system map: each region names the tile
of the agent that owns it. Each agent owns a unique address range.
Regions gain one attribute:

| Field       | Meaning                                              |
|-------------|------------------------------------------------------|
| `base`      | first address of the region                          |
| `last`      | last address (inclusive)                             |
| `x`, `y`    | tile of the owning agent                             |
| `cacheable` | 1 = accesses go through the L2; 0 = bypass the L2    |

A request to an address in no region is rejected at the source NI with
`RSP_DECERR`, as today.

## 3. NoC changes

1. **Byte enables.** Every flit gets a strobe field, so the link is
   `{head, tail, strb[FLIT_W/8], data}`. Strobes matter only in write data
   flits (1 = byte is written); elsewhere they are ignored and should be 0.
2. **`cacheable` region attribute** (section 2).
3. **NI routing by source.** The NI gets a parameter `IS_L2`.

   | Request from     | Address       | Routed to                         |
   |------------------|---------------|-----------------------------------|
   | agent NI         | cacheable     | the L2 tile                       |
   | agent NI         | non-cacheable | region owner from `ADDR_MAP`      |
   | agent NI         | unmapped      | rejected, `RSP_DECERR`            |
   | L2 NI (`IS_L2`)  | any           | region owner from `ADDR_MAP`      |

   Responses route back to the request's `src` as today. An agent's
   response to a cacheable request therefore comes from the L2 tile; the
   agent matches it by tag, as it already must.

## 4. Agent-side transactions (arriving at the L2)

Requests use the existing packet format (`PKT_RD_REQ` / `PKT_WR_REQ`).

- `addr` is aligned to a flit (8 bytes); partial bytes within a flit are
  selected by strobes.
- A cacheable request must not cross a 64-byte line, so it is at most
  8 flits. A request that crosses is rejected: it gets `RSP_SLVERR`
  (header-only) and has no effect on the cache. Its write data, if any,
  is consumed and dropped.
- A read returns `len + 1` full flits. A write returns a header-only
  write response.
- One response per request, as the NoC requires.

## 5. Policies

| Policy        | Choice                                                     |
|---------------|------------------------------------------------------------|
| Write hit     | update the line, mark it dirty (write-back)                |
| Write miss    | allocate (write-allocate): fetch the line, then merge the  |
|               | write. If the write covers the whole line (all 64 strobes  |
|               | set), skip the fetch and fill the line from the write.     |
| Read miss     | fetch the line, then respond                               |
| Eviction      | dirty victim is written back as one full-line `PKT_WR_REQ` |
| Replacement   | tree pseudo-LRU per set; invalid ways are used first       |
| Fill          | one `PKT_RD_REQ` with `len = 7` at the line address        |

**Memory errors.** If a fill returns an error, the line is not allocated
and the agent gets `RSP_SLVERR`. If a writeback returns an error, the
line is already gone, so no agent can be told; instead the L2 sets a
sticky status bit:

| Port         | Dir | Meaning                                          |
|--------------|-----|--------------------------------------------------|
| `wb_err`     | out | set by any writeback error; stays set            |
| `wb_err_clr` | in  | 1-cycle pulse clears `wb_err` (reset clears it too) |

A writeback error in the same cycle as `wb_err_clr` leaves `wb_err` set.

## 6. Concurrency

Requests wait in a pending FIFO of `PF_DEPTH` entries (default 16, at
most 2^`TAG_W`); `t_req` is not ready when it is full. Hits are served
while misses are outstanding, and up to `PF_DEPTH` misses can be
outstanding. Requests to the same line complete in the order they arrive
at the L2: when a command is accepted it records a dependency on the
youngest pending command to the same line, and starts only after that
command has completed. Responses to requests for different lines may
complete out of order (agents match responses by tag).

Memory may answer the L2's fills and writebacks in any order. The L2 does
not refill a line until that line's writeback has been answered.

Version 1 handled one request at a time.

The pending FIFO is a further external memory port: `PF_DEPTH`
entries, each holding a whole request (header and a line of write data
with strobes).

## 7. Geometry (`sh_l2_pkg`)

| Parameter    | Default | Derived (defaults)                          |
|--------------|---------|---------------------------------------------|
| `L2_BYTES`   | 1 MB    |                                             |
| `NUM_WAYS`   | 8       |                                             |
| `LINE_BYTES` | 64      | 8 flits of 64 bits                          |
|              |         | `NUM_SETS = 1M / (64 * 8) = 2048`           |
|              |         | offset `addr[5:0]`, index `addr[16:6]`      |
|              |         | tag `addr[31:17]` (15 bits)                 |
| `L2_X`, `L2_Y` |       | tile of the L2 (in `sh_noc_pkg`)            |

## 8. Storage

No memories are instantiated in the L2. Each array is brought out as a
two-port memory port (one read port, one write port, 1R1W) with a
read-latency parameter, like the NoC FIFOs. Read and write ports can be
used in the same cycle; a read of the address being written in that
cycle returns the old data.

| Array | Depth                        | Width                              | Write mask |
|-------|------------------------------|------------------------------------|------------|
| Tag   | `NUM_SETS`                   | `NUM_WAYS x (tag + valid + dirty)` | per way    |
| Data  | `NUM_SETS x NUM_WAYS x 8`    | `FLIT_W` (one flit)                | per byte   |
| LRU   | `NUM_SETS`                   | `NUM_WAYS - 1` bits                | none       |
| Pending FIFO | `PF_DEPTH`         | `(WORDS + 1) x SLOT_W` (header slot + 8 x {strb, data}) | per slot |

Default sizes: tag 2048 x 136 bits, data 131072 x 64 bits (1 MB),
LRU 2048 x 7 bits, pending FIFO 16 x 648 bits (9 slots of 72).

## 9. Reset and initialization

After reset the L2 clears every valid bit by walking the tag array
(`NUM_SETS` cycles) before it accepts requests. Requests arriving during
this time wait in the NoC.

## 10. Optional features (not in version 1)

- Maintenance operations: flush all / invalidate all, flush by address.
- ECC or parity on the tag and data arrays.
- Performance counters: hits, misses, writebacks, stall cycles.
- Hit-under-miss and multiple outstanding misses (section 6).

## 11. Verification plan

- Unit test of the L2 with a model agent and a model memory: random
  reads and writes (partial, full-line, line-crossing) checked against a
  reference memory, so every read must return the last value written.
  Small configurations (e.g. 2 sets, 2 ways) force evictions.
- System test in the mesh: several agents, the L2, a memory controller
  model, cacheable and non-cacheable regions, unmapped addresses.
- Read-latency settings 0..3 on all arrays.
- Mutation checks, as for the NoC.
