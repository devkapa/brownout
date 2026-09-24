# brownout core v2 (Rust/WASM) — contract

The partitioned event-driven transient engine, compiled to WebAssembly. The
plan and the locked owner decisions live in the devolt repo:
`memory/decisions/2026-09-24-brownout-rust-wasm-core.md` (plan) and
`...-handoff.md` (state and hard-won detail). This file is the technical
contract the TypeScript front end and the Rust back end are both written
against. Change it first, then both sides.

**Status: W0 (scaffolding).** The kernel implements only a static slice —
`ground`, `resistor` and grounded ideal `voltage_source` — so the whole
pipeline (compile, upload, partition, solve, read back, facade, equivalence
harness, packaging) can be proven end to end. The event kernel, rails,
closed-form and stepped partitions arrive in W1.

## Layout

| Path | Owns |
|---|---|
| `core/` | The Rust crate `brownout-core` (lib `brownout_core`). No crates.io dependencies and no wasm-bindgen: raw C-ABI exports, hand-written TS glue. |
| `core/rust-toolchain.toml` | The pinned toolchain and the `wasm32-unknown-unknown` target. |
| `scripts/build-core.mjs` | Builds the wasm and copies it to `src/core/brownout-core.wasm` (gitignored) and, with `--dist`, to `dist/core/`. |
| `src/core/` | TS front end: program format, SimCircuit compiler, wasm loader, `CoreEngine` facade, engine selection. |
| `src/core.ts` | The `brownout/core` entry. |

The `.wasm` is a build output. It is never committed; CI builds it, and the
npm tarball carries it under `dist/core/`.

## Division of labour

- **TS front end** owns catalog and spec resolution, parameter defaults, net
  building (`buildNets`, so net ids are identical to the classic engine's),
  the compiler, the wasm loader and worker glue, the SPICE netlist parser and
  the instrument DSP. It compiles a `SimCircuit` into a flat program (below).
  It does no physics. The MCU emulators stay in TS only until they are
  ported: avr8js moves into the core in W4, rp2040js in W7. The micro:bit
  (MicroPython already compiled to WASM, in an iframe) stays a JS host that
  pushes pin levels in.
- **Rust back end** owns fixed-node derivation, partitioning, the solvers,
  the event kernel, every device kernel's physics and, once ported, the MCU
  instruction loops. From W6 it also owns the DC operating point and
  small-signal AC, and the classic engine is deleted.

## Program format, version 1

One `Float64Array`. Every value is an f64; integers are carried as exact
doubles (all counts and indices are far below 2^53). The TS and Rust sides
ship in one package, so there is no cross-version compatibility: the version
number is a sanity check, and any layout change bumps it on both sides at
once.

```
header   MAGIC, VERSION, nodeCount N, deviceCount D
devices  D records, in component order (see below)
trailer  MAGIC
```

- `MAGIC` = `0x424F4331` ("BOC1"), `VERSION` = `1`.
- **Nodes.** `N >= 1`. Node 0 is the datum: the net `buildNets` names `gnd`
  when there is one, otherwise a synthetic node with no pins. Every other net
  from `buildNets` becomes nodes `1..N-1`, in `buildNets` order. Node
  classes (fixed, rail, free) are derived in Rust, not sent.
- **Device record.**
  ```
  kindCode, componentIndex, pinCount, pin node x pinCount,
  paramCount, param x paramCount, blobCount, blob x blobCount
  ```
  `componentIndex` is the index in `circuit.components`. The blob is empty in
  W0; from W1 it carries ROM/RAM contents and IC pin maps. Components that
  stamp nothing (`ground`) produce no record.
- **Kind codes (v1).**

  | Code | Kind | Pins (in order) | Params |
  |---|---|---|---|
  | 1 | `resistor` | `pins[0]`, `pins[1]` | `R = max(1e-3, Number(params.resistance ?? 1000))` |
  | 2 | `voltage_source` | `pins[0]` (+), `pins[1]` (−) | `V = Number(params.voltage ?? 5)` |

  Both mirror the classic models exactly (`devices/passives.ts`
  `resistorModel`, `devices/sources.ts` `voltageSourceModel`, which stamps
  `pins[0]` as + and `pins[1]` as −). A component with fewer than two pins is
  a compile failure, not a silent skip.

The reader validates everything: magic, version, counts, every node index
`< N`, known kind codes, the exact pin and param counts per kind, every param
finite, every resistor's `R > 0`, the trailer, and no trailing data. A
malformed program is error `-1`, never a panic; a kind code that is a valid
integer but names no kind is `-2`. (The compiler already clamps `R >= 1e-3`,
so a non-positive `R` can only come from a hand-built program, not a compiled
circuit — the reader's `R > 0` check is defense in depth, the same as its
finiteness check. The compiler also refuses non-finite parameters before they
reach a program, so the circuit falls back to the classic engine and the core
never has to define NaN physics.)

## C ABI, version 1

All exports are `extern "C"` with `#[no_mangle]`. The module has **zero
imports** and exports its `memory`, the functions below, and nothing else
except the two inert linker globals `__heap_base` and `__data_end`, which
rustc exports from every wasm32-unknown-unknown cdylib.
`scripts/build-core.mjs` fails the build on any other import or export.

| Export | Signature | Meaning |
|---|---|---|
| `bo_abi_version` | `() -> u32` | `1`. The TS loader refuses a mismatch. |
| `bo_alloc` | `(len_f64: u32) -> *mut f64` | Scratch f64 buffer, zeroed. |
| `bo_free` | `(ptr: *mut f64, len_f64: u32)` | Frees a `bo_alloc` buffer. |
| `bo_engine_create` | `() -> u32` | New engine handle; `0` is never a valid handle. Handles are never reused. |
| `bo_engine_destroy` | `(h: u32)` | Frees the engine. Unknown handles are ignored. |
| `bo_engine_load` | `(h: u32, ptr: *const f64, len_f64: u32) -> i32` | Load a program, derive fixed nodes, partition, solve at t = 0. `0` or an error code. |
| `bo_engine_advance` | `(h: u32, t_end: f64) -> i32` | Advance to `t_end` (seconds, absolute). |
| `bo_engine_time` | `(h: u32) -> f64` | Current simulation time. |
| `bo_engine_node_count` | `(h: u32) -> u32` | `N` of the loaded program. |
| `bo_engine_node_voltages` | `(h: u32) -> *const f64` | `N` volts, node 0 = 0. |
| `bo_engine_device_count` | `(h: u32) -> u32` | `D` of the loaded program. |
| `bo_engine_device_currents` | `(h: u32) -> *const f64` | `D` amps, one per device record, in record order. |
| `bo_engine_stat` | `(h: u32, i: u32) -> f64` | Counters: `0` partitions, `1` static solves, `2` largest partition size, `3` fixed nodes. Unknown `i` gives `NaN`. |
| `bo_error_ptr` / `bo_error_len` | `() -> *const u8` / `() -> u32` | UTF-8 message for the most recent non-zero return. |

**Error codes:** `-1` malformed program, `-2` unknown kind code, `-3`
unsupported topology (the TS side falls back to the classic engine), `-4`
solve failed (a zero or non-finite pivot, or a published node voltage or
device current that came out non-finite), `-5` unknown handle or no program
loaded, `-6` `t_end` is not finite, or is earlier than the current time.

**Memory.** Any call may grow the wasm memory, which detaches every JS view
of `memory.buffer`. The TS glue re-derives each view from `memory.buffer`
after every call and never caches one across calls. Pointers returned by the
`*_voltages` / `*_currents` exports are valid until the next call on that
engine.

**Traps.** The crate builds with `panic = "abort"`, so a Rust panic is a wasm
trap (`WebAssembly.RuntimeError`). The instance may be mid-mutation after a
trap, so the TS side treats it as poisoned: every later call throws until the
core is re-initialised.

## W0 kernel semantics

1. **Fixed nodes.** Node 0 is fixed at 0 V. A `voltage_source` with one
   terminal fixed fixes the other: `V(+) = V(−) + V`. Propagate until nothing
   changes, recording each source as the parent of the node it fixed. A source
   whose terminals are both already fixed (a loop of ideal sources, including
   a source shorted to itself) is `-3`, whether or not the voltages agree —
   the classic MNA is singular there too. A source with neither terminal
   reachable is `-3` (floating ideal source).
2. **Node shunt.** Every node except node 0 carries `NODE_RSHUNT_G = 1e-12` S
   to the datum, exactly as the classic engine stamps it on every node row.
   It keeps an isolated net solvable (it reads 0 V) and it is included in
   every KCL sum below.
3. **Partitions.** Union-find over free nodes, joined by every resistor whose
   two terminals are both free. A resistor with one fixed end stamps the fixed
   voltage into the right-hand side; one with both ends fixed stamps nothing.
   The union-find itself is iterative (path halving, union by size): no input
   — however long a chain of unions it produces — makes it recurse to a depth
   that depends on the input.
4. **Solve.** Dense LU with partial pivoting per partition, **LINPACK
   layout**: at step k swap only columns `k..n-1` of the two rows, keep the
   multipliers in place, and interleave the recorded swaps with forward
   elimination when applying. (Full-row swaps including the stored multipliers
   are wrong with an interleaved apply — the prototype hit this.) A pivot that
   is zero or non-finite is `-4`. After the solve and the current pass (point
   5), the load also fails with `-4` if any published node voltage or device
   current is not finite, naming the first offending node or device index —
   a finite pivot at every step does not rule out overflow in the fixed-
   voltage propagation, the right-hand side, or the currents themselves.
5. **Currents.** A resistor publishes `(V[pins[0]] − V[pins[1]]) / R`, as
   classic does. A `voltage_source` publishes the classic MNA branch value:
   the current flowing **into its + terminal** from the circuit, i.e. negative
   while it delivers power. Compute it by KCL at the node the source fixed,
   children before parents in the propagation forest:
   - `R_u` = sum over resistors of the current leaving node `u`, plus the
     shunt current `1e-12 * V[u]`;
   - a child source `c` at `u` contributes `+i_c` if `c`'s + terminal is `u`
     and `−i_c` if its − terminal is `u`;
   - if source `s` fixed its + terminal `u`: `i_s = −(R_u + children)`;
     if it fixed its − terminal `u`: `i_s = R_u + children`.
6. **Time.** `advance(t_end)` with `t_end` below the current time by more than
   `1e-15` is `-6`. A non-finite `t_end` is also `-6`, with the message
   "t_end must be finite" — checked first, since a comparison against NaN is
   always false and would otherwise let it through and become the clock.
   Everything in W0 is static, so advancing only moves time.

## Hard-won rules for the kernel (from the prototype)

- Accept a threshold crossing only at `t > now + 1e-15`; rounding can put a
  node back on the threshold and re-schedule it forever.
- Guard closed-form re-aiming until the first segment exists (`segReady`), or
  it produces NaN.
- Keep a pending plan's capacitor history separate from the committed `vc`.
- Bidirectional parts (74LS245) register every A and B pin as a reader as well
  as creating drivers.
