# brownout core v2 (Rust/WASM) — contract

The partitioned event-driven transient engine, compiled to WebAssembly. The
plan and the locked owner decisions live in the devolt repo:
`memory/decisions/2026-09-24-brownout-rust-wasm-core.md` (plan) and
`...-handoff.md` (state and hard-won detail). This file is the technical
contract the TypeScript front end and the Rust back end are both written
against. Change it first, then both sides.

**Status: W1c done (the digital core and the 11 combinational ICs); W1d
(sequential ICs, memories, datasheet delays and the W1c enable retrofit)
contracted.** W0 proved the pipeline with a
static slice (`ground`, `resistor`, grounded ideal `voltage_source`). W1 builds
the event kernel and the Ben Eater path; its locked owner decisions and
dispatch plan are in the devolt repo,
`memory/decisions/2026-09-25-brownout-core-w1-plan.md`. The classic behaviour
the kernel mirrors is `core/spec/w1-classic-semantics.md`, and the datasheet
delays are `core/spec/w1-datasheet-delays.md`.

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

## Program format, version 3

One `Float64Array`. Every value is an f64; integers are carried as exact
doubles (all counts, indices and 32-bit hash states are far below 2^53). The
TS and Rust sides ship in one package, so there is no cross-version
compatibility: v2 replaces v1 outright, and any layout change bumps the
version on both sides at once.

```
header   MAGIC, VERSION, nodeCount N, deviceCount D,
         optionCount O, (key, value) x O,
         digitalSlotCount S, channelSlotCount C
devices  D records, in component order
trailer  MAGIC
```

- `MAGIC` = `0x424F4331` ("BOC1"), `VERSION` = `3`. W1d bumped it from 2,
  because the 74ls157 and 74ls245 records gained a delay tail.
- **Nodes.** As in v1: node 0 is the datum, the net `buildNets` names `gnd`
  when there is one, otherwise a synthetic node with no pins; every other net
  becomes nodes `1..N-1` in `buildNets` order. Node classes (fixed, rail,
  free) are derived in Rust, never sent.
- **Options.** `(key, value)` pairs, each key at most once:

  | Key | Name | Values | Default |
  |---|---|---|---|
  | 1 | `INTEGRATION` | `0` backward Euler, `1` trapezoidal | `0` |
  | 2 | `NO_SOLUTION_CACHE` | `0` / `1` (exactness switch) | `0` |
  | 3 | `NO_RAIL_SHORTCUT` | `0` / `1` (exactness switch) | `0` |
  | 4 | `FORCE_STEPPED` | `0` / `1` (tests only) | `0` |
  | 5 | `AMBIENT_C` | finite °C | `25` |
  | 6 | `FLOATING_MODEL` | `0` classic hash (the only value in W1) | `0` |
  | 7 | `START_TIME` | finite seconds `>= 0`: the absolute time the loaded program starts at | `0` |

  `START_TIME` exists because classic keeps absolute `simTime` across a
  reload, and time-dependent behaviour keys on it: floating-input buckets
  (§1.5) and waveform source phase. The facade passes the time it carries
  over, so Rust's clock is always absolute and no time base is layered on
  top.

- **Readout slot counts.** `S` digital slots and `C` channel slots (see
  Readouts). The compiler assigns every slot; Rust only writes numbers.
- **Device record.**
  ```
  kindCode, componentIndex, pinCount, pin record x pinCount,
  paramCount, param x paramCount, blobCount, blob x blobCount
  ```
  A **pin record** is four values: `node, flags, openHash, deadHash`.
  - `node`: the pin's node index, `< N`.
  - `flags`: bit 0 `OPEN` (the pin is open by classic's rule, spec §1.6: its
    net holds only this component and no wire touches this component's pins
    on it; the compiler computes it, G8); bit 1 `OPEN_HIGH` (the pin is in
    classic's `IC_OPEN_HIGH_PINS`, spec §1.5). Every other bit must be 0.
  - `openHash`: the FNV-1a-32 state (basis `0x811c9dc5`, per UTF-16 code
    unit `h ^= unit; h = h * 0x01000193 mod 2^32`) after hashing the string
    `${componentId}:${pinId}:`. `deadHash`: the same after
    `${componentId}:${pinId}:undefined:`. Rust finishes classic's key by
    hashing the bucket's decimal digits (ASCII, one unit each) onto the
    state. The component and pin id strings never reach Rust. Both are
    integers in `[0, 2^32)`; the compiler emits them for every pin.
  - Pin **order** is fixed per kind in the kind table (G7): some kinds are
    positional (`pins[0]`, `pins[1]`, as classic reads them), others list
    pins by id in the table's order. Rust knows every pin's role from its
    position alone.
  - `params` and `blob` are per kind (kind table). Every param and blob value
    is finite; a sentinel is an explicit code, never NaN. Where a kind
    publishes digital or channel readouts, its blob starts with the index of
    its first slot of each kind it uses.
- **Validation.** The reader checks everything: magic, version, counts, each
  option key known, not repeated and its value in range, every node index
  `< N`, flags with no unknown bit, hash states integral and in range, known
  kind codes, the exact pin, param and blob shape per kind, every param and
  blob value finite, every slot index `< S` or `< C`, the trailer, and no
  trailing data. Malformed is `-1`, never a panic; a well-formed but unknown
  kind code is `-2`. Per-kind rules (for example resistor `R > 0`) live in
  `core/src/kinds.rs`. A bad option inside the program is `-1` (the program
  is malformed); the same bad value passed to `bo_engine_set_option` after
  load is `-8` (an invalid argument).
- **Slot indices.** A kind that publishes digital readouts declares in
  `kinds.rs` how many slots a record owns (`digital_slots`); its first slot
  is `blob[0]`, and the reader checks that `blob[0]` is an integer and
  `blob[0] + count <= S`. The W1c ICs are the first kinds that publish any.

## Kind codes and records

Codes are grouped in blocks by family. A kind's pin order, params and blob are
written into this table by the orchestrator before the sub-wave that ports it
is dispatched; a code without a record layout is reserved and rejected as
`-2`.

| Block | Codes |
|---|---|
| Passives and switches | 1 `resistor`, 3 `capacitor`, 4 `switch`, 5 `push_button`, 6 `spdt_switch`, 7 `dip_switch`, 8 `potentiometer`, 9 `trimmer`, 10 `fuse`, 11 `ldr`, 12 `thermistor` |
| Sources | 2 `voltage_source` (kept from v1), 20 `battery_pack`, 21 `bench_psu`, 22 `clock`, 23 `clock_gen`, 24 `pulse_source`, 25 `pulse_gen` |
| LEDs and displays | 30 `led`, 31 `rgb_led`, 32 `bicolor_led`, 33 `seg7_cc`, 34 `seg7_ca` |
| Timers | 40 `ne555` |
| Combinational ICs | 50 `74ls00`, 51 `74ls04`, 52 `74ls08`, 53 `74ls32`, 54 `74ls86`, 55 `74ls157`, 56 `74ls283`, 57 `74hc138`, 58 `74hc14`, 59 `74ls47`, 60 `cd4511`, 61 `74ls245` |
| Sequential ICs | 70 `74ls161`, 71 `74ls173`, 72 `74hc74`, 73 `74hc595`, 74 `74hc165`, 75 `cd4017`, 76 `cd4060` |
| Memories | 80 `74ls189`, 81 `28c16`, 82 `28c256` |

**Record layouts** (added per sub-wave):

| Code | Kind | Pins, in order | Params | Blob | Added |
|---|---|---|---|---|---|
| 1 | `resistor` | `pins[0]`, `pins[1]` (positional) | `R = max(1e-3, Number(params.resistance ?? 1000))`, `R > 0` | empty | W0 |
| 2 | `voltage_source` | `pins[0]` (+), `pins[1]` (−) (positional) | `V = Number(params.voltage ?? 5)` | empty | W0 |
| 3 | `capacitor` | `pins[0]`, `pins[1]` (positional) | `[C, esr, leakage]`: `C = max(1e-15, Number(params.capacitance ?? 1e-6))`; `esr` and `leakage` by classic's `modelParam` exact-uid rule (spec §7.2.6), `esr` 0 when absent or invalid, `leakage` 0 meaning none (classic's infinite parallel resistance) | empty | W1b |
| 4 | `switch` | `pins[0]`, `pins[1]` (positional) | `[closed]`: `Boolean(params.closed) ? 1 : 0` (the string `"0"` counts as closed, as in classic); the closed contact is a literal 1 mΩ | empty | W1b; live param 0 |
| 5 | `push_button` | `pins[0]`, `pins[1]` (positional; `buildNets` already merges `a≡a2`, `b≡b2`) | `[closed]`, as `switch` | empty | W1b; live param 0 |
| 6 | `spdt_switch` | `com`, `b`, `c` (by id) | `[sel]`: `Number(params.position ?? 0) === 0 ? 0 : 1` (0 or absent selects `b`; anything else, NaN included, selects `c`) | empty | W1b; live param 0 |
| 7 | `dip_switch` | `a1, b1, ..., ak, bk` (by id) | `[k, sw1..sw8]`: `k` is `positions` rounded and clamped to `[2, 8]` as classic does; `sw_i = Number(params["sw"+i] ?? 0) !== 0 ? 1 : 0`, inert past `k` | empty | W1b; live params 1-8 |
| 50-59, 61 | the 11 combinational ICs: `74ls00`, `74ls04`, `74ls08`, `74ls32`, `74ls86`, `74ls157`, `74ls283`, `74hc138`, `74hc14`, `74ls47`, `74ls245` | `vcc`, `gnd` (the pins classic's `supplyPinsForComponent` resolves), then the kind's logic pins in `icPinMap(kind)` order, by id (table below) | the 13 IC params below, then the kind's delay tail (74ls157: 1 pair, 74ls245: 2 pairs, the other nine: none) | `[slotBase, pinFlags x L]`, `L` the logic-pin count | W1c; tail W1d |
| 70-72 | `74ls161`, `74ls173`, `74hc74` | `vcc`, `gnd`, then the kind's logic pins by id (sequential table below) | the 13 IC params, then the kind's delay tail | `[slotBase, pinFlags x L]` | W1d |
| 80 | `74ls189` | `vcc`, `gnd`, then the memory logic pins by id (memory table below) | the 13 IC params, then the kind's delay tail | `[slotBase, pinFlags x L]` | W1d |
| 81, 82 | `28c16`, `28c256` | `vcc`, `gnd`, then the memory logic pins by id | the 13 IC params, the delay tail, then `tWC` and `sdp` | `[slotBase, pinFlags x L, word x W]` (ROM image below) | W1d |

**IC logic pins**, after `vcc` and `gnd` (`icPinMap`, src/sim/engine/digital.ts;
`i` input, `c` control, `o` output). `cd4511` (code 60) is W1h.

| Kind | `L` | Logic pins | Digital slots |
|---|---|---|---|
| `74ls00`, `74ls08`, `74ls32`, `74ls86` | 12 | `1a 1b 1y 2a 2b 2y 3a 3b 3y 4a 4b 4y` (y `o`) | 4 |
| `74ls04`, `74hc14` | 12 | `1a 1y 2a 2y 3a 3y 4a 4y 5a 5y 6a 6y` (y `o`) | 6 |
| `74ls157` | 14 | `sel /oe` (`c`), `1a 1b 1y 2a 2b 2y 3a 3b 3y 4a 4b 4y` (y `o`) | 4 |
| `74ls283` | 14 | `a1 b1 a2 b2 a3 b3 a4 b4 c0` (`i`), `s1 s2 s3 s4 c4` (`o`) | 5 |
| `74hc138` | 14 | `a0 a1 a2` (`i`), `e1n e2n e3` (`c`), `y0n..y7n` (`o`) | 8 |
| `74ls47` | 14 | `b c` (`i`), `lt_n rbo_n rbi_n` (`c`), `d a` (`i`), `e_out d_out c_out b_out a_out g_out f_out` (`o`) | 7 |
| `74ls245` | 18 | `dir /oe` (`c`), `a1 b1 a2 b2 ... a8 b8` | 16 (every `a` and `b` pin; the evaluator picks the direction) |

A digital slot belongs to each output-capable pin in pin order, keyed
`<componentId>/<pinId>`. The compiler declines an IC (the circuit falls back to
classic with reason `topology:ic-pins:<componentId>`) when its supply pins do
not both resolve (classic's supply-less path: a stale uid, or a kind missing
from the injected library), or when the component's pin ids are not exactly
the two supply pins plus the kind's logic pins. Catalog parts always carry the
full set; the decline keeps classic's missing-pin defaults (`?? 0` for gate
inputs, `?? 1` for 74ls47 controls) out of the core, where an absent pin would
read node 0.

**IC params** (indices in `kinds.rs` `ic_param` and `kinds.ts`
`CORE_IC_PARAM`). Every value is finite, and the compiler resolves each one with
classic's own helpers, exported from `sim-engine.ts` for this, never re-derived:
- `0 vccMin = specs?.vcc_range?.min ?? 2.0`, `1 vccMax = specs?.vcc_range?.max ?? 18.0`.
- `2 famFlags`: bit 0 `TTL` (`logic_family` contains `"TTL"`: pHigh 0.7 and a
  1 MΩ floating pull, else 0.5 and 10 MΩ); bit 1 `CMOS_RATIO`
  (`logic_family` contains `"HC-CMOS"` or `"CMOS-4000"`: `thresholdsFor`'s
  ratio rule).
- `3 vilFixed`, `4 vil`, `5 vihFixed`, `6 vih`: when `CMOS_RATIO` is clear and
  `specs.v_input` exists, `vilFixed = 1, vil = v_il_max` if `v_il_max` is
  present, likewise `vih` from `v_ih_min`; otherwise the flag is 0 and the
  value 0, and the formula applies.
- `7 schmitt = 1` when `specs.v_input.v_t_plus != null` (74hc14 among these
  kinds); then `8 vtPlus = v_t_plus` and `9 vtMinus = v_t_minus ?? 1.6`;
  otherwise all three are 0.
- `10 nominal = specs?.vcc_range?.nominal ?? 5` (the Schmitt scale).
- `11 ioMax = specs.io_max` when it is `> 0`, else 0 (meaning R_out = 50 Ω).
- `12 delay = propagationDelayForComponent(comp)`, seconds, `>= 0`.

**IC blob.** `slotBase`, then `pinFlags[i]` for logic pin `i`: bit 0
`FLOATING_INPUT` (classic's `_shouldTreatOpenPinAsDigitalInput`: the catalog
pin function is in `FLOATING_INPUT_FUNCTIONS`, or the `icPinMap` role is
input, clock or control), bit 1 `OPEN_COLLECTOR` (catalog function
`open_collector`: the 74ls47 segments). No other bit is used. The pin-flag
check covers `blob[1..=L]` only; an EEPROM's ROM words follow and are checked
as words.

**Sequential and memory IC logic pins**, after `vcc` and `gnd` (catalog pin
ids; `r` read, `o` driven). `74hc74` follows `icPinMap`; `74ls161`,
`74ls173`, `74ls189` and the EEPROMs have no `icPinMap` entry, so this table
fixes their order: controls, then data, then outputs in bit order.

| Kind | `L` | Logic pins | Drivers | Digital slots |
|---|---|---|---|---|
| `74ls161` | 14 | `/clr clk /load enp ent a b c d` (r), `qa qb qc qd rco` (o) | 5 | 5: `<id>` (the count, a bare key), `qa qb qc qd` |
| `74ls173` | 14 | `/clr clk m n g1 g2 d1 d2 d3 d4` (r), `q1 q2 q3 q4` (o) | 4 | 4: `q1..q4` |
| `74hc74` | 12 | `clr1_n d1 clk1 pre1_n` (r), `q1 q1_n q2_n q2` (o), `pre2_n clk2 d2 clr2_n` (r) | 4 | 4: `q1 q1_n q2 q2_n` (classic's key order) |
| `74ls189` | 14 | `a0 a1 a2 a3 d1 d2 d3 d4 /cs /we` (r), `/o1 /o2 /o3 /o4` (o) | 4 | 4: `/o1../o4` (keys `<id>//o1`) |
| `28c16` | 22 | `a0..a10 /oe /ce /we` (r), `io0..io7` (o and r) | 8 | 8: `io0..io7` |
| `28c256` | 26 | `a0..a14 /oe /ce /we` (r), `io0..io7` (o and r) | 8 | 8: `io0..io7` |

Bit 0 of every bus is its first pin. The compiler declines exactly as for a
W1c IC (`topology:ic-pins:<componentId>`). For codes 70-82:
- params `7 schmitt`, `8 vtPlus`, `9 vtMinus` are 0 whatever the catalog says,
  because classic reads these kinds through `logicHigh`/`logicHighH`, never
  its Schmitt branch. The reader rejects `schmitt = 1` here;
- `OPEN_COLLECTOR` is never set (classic drives them push-pull through
  `stampDigitalOutput`), and the reader rejects it;
- `FLOATING_INPUT` for the kinds with no `icPinMap` roles is the catalog
  pin-function test alone. It covers every read pin and, through `tri_state`
  and `io`, the 173's `q` pins and the memories' outputs.

**IC delay tail** (owner decision 3; figures in
`core/spec/w1-datasheet-delays.md`). After the 13 IC params come `P` delay
pairs. Pair `k` is params `13 + 2k` (`dH`) and `14 + 2k` (`dL`), in seconds,
finite and `>= 0`; a kind's further params follow the tail. A pair is indexed
by the driven level the transition involves: a level path gives `dH = tPLH`,
`dL = tPHL`; an enable path `dH = tPZH`, `dL = tPZL`; a disable path
`dH = tPHZ`, `dL = tPLZ`.

| Kind | `P` | Pairs, in order | Typical ns (`dH`, `dL`) |
|---|---|---|---|
| `74ls157` | 1 | `strobe` | 13, 14 |
| `74ls245` | 2 | `oeEnable`, `oeDisable` | 25, 27; 15, 15 |
| `74ls161` | 4 | `clkQ`, `clrQ`, `clkRco`, `entRco` | 13, 18; 20, 20 (tPHL only published); 20, 18; 9, 9 |
| `74ls173` | 4 | `clkQ`, `clrQ`, `mnEnable`, `mnDisable` | 17, 22; 26, 26 (tPHL only); 15, 18; 11, 11 |
| `74hc74` | 2 | `clkQ`, `preClrQ` | 20, 20; 20, 20 (the 4.5 V column: TI's HC tables have no 5 V column) |
| `74ls189` | 5 | `addrO`, `csEnable`, `csDisable`, `weEnable`, `weDisable` | 37, 37; 10, 10; the rest unpublished for the LS part, so param 12 (20 ns) |
| `28c16`, `28c256` | 5 | `addrIo`, `ceEnable`, `ceDisable`, `oeEnable`, `oeDisable` | 150, 150; 150, 150; 50, 50; 70, 70; 50, 50 (datasheet MAXIMA: none publishes a typical) |

The other nine W1c kinds carry no tail, and every one of their transitions
uses param 12 in both directions. The EEPROMs' further params are
`23 tWC = 10e-3` (classic's `T_WC`, both kinds) and `24 sdp =
kind === "28c256" && Number(params.sdpBypass ?? 0) === 0 ? 1 : 0` (0 on a
28c16).

**Resolution** (the compiler). With `f(x) = specs.switching_ns[path][x] *
1e-9` when that value is a finite number `>= 0` (`specs` from
`specsForComponent`) and `d = param 12`:
`dH = f(h) ?? f(l) ?? d` and `dL = f(l) ?? f(h) ?? d`. So a path published in
one direction only carries its figure both ways, and an unpublished path, or a
catalog without the field (devolt's injected catalog until it gains it), gets
classic's family default. A missing figure never declines the circuit.

**Catalog field** `electrical_specs.switching_ns` (`src/circuit/types.ts`):
`Record<path, { tplh?, tphl?, tpzh?, tpzl?, tphz?, tplz? }>` in nanoseconds,
plus a documentation-only `switching_source`. It is added per part to
`src/parts/default-parts.json` and to the test catalog. Only the core
compiler reads it; classic keeps `prop_delay_ns` and the family prefix.

**ROM image** (EEPROMs). `size` is 2048 (`28c16`) or 32768 (`28c256`), and
`W = size / 4` (512 or 8192 words).
- TS decodes `params.contents` exactly as classic's `load()` does, through a
  helper exported from `sim-engine.ts` that classic's `load()` also calls:
  fill `size` bytes with 0xFF, overlay `atob(String(params.contents ?? ""))`
  truncated to `size`, and keep all 0xFF when `atob` throws.
- Word `k` is `b[4k] + b[4k+1]*2^8 + b[4k+2]*2^16 + b[4k+3]*2^24`:
  little-endian, an exact integer in `[0, 2^32)`, built with shifts and
  `>>> 0`, never a host-endian view. The reader checks every word.
- Rust never sees base64, a short image, or the contents hash.

**Pins by id.** A pin id the component lacks maps to node 0, because classic
maps an absent pin to its reference node (spec §7.2.7). One exception: classic
stamps nothing at all for an `spdt_switch` with fewer than 3 declared pins
(`if (pins.length < 3) return`), so the compiler declines one
(`malformed:<componentId>:too-few-pins`, the circuit runs on classic) rather
than emit a contact classic never stamps.

**Currents** (`elementI`, one per record):
- `capacitor`: series plus leakage current, `pins[0]` to `pins[1]`.
- `switch`, `push_button`: the contact current `pins[0]` to `pins[1]`, 0 when
  open.
- `spdt_switch`: the current through the contact in use, `com` to the
  selected throw. This is the Q-O26 fix, landed in classic first; classic
  used to report `com` to `b` at every position.
- `dip_switch`: the sum of the closed positions' contact currents, `a_i` to
  `b_i`.
- The W1c and W1d ICs: `0`, as classic (no current model).

## C ABI, version 2

All exports are `extern "C"` with `#[no_mangle]`. The module has **zero
imports** and exports its `memory`, the functions below, and nothing else
except the two inert linker globals `__heap_base` and `__data_end`, which
rustc exports from every wasm32-unknown-unknown cdylib.
`scripts/build-core.mjs` fails the build on any other import or export, so an
export lands in its list in the same change that adds it.

| Export | Signature | Meaning | Since |
|---|---|---|---|
| `bo_abi_version` | `() -> u32` | `2`. The TS loader refuses a mismatch. | v1 |
| `bo_alloc` / `bo_free` | `(len_f64: u32) -> *mut f64` / `(ptr, len_f64)` | Scratch f64 buffer, zeroed; freed with the same length. | v1 |
| `bo_engine_create` / `bo_engine_destroy` | `() -> u32` / `(h)` | Handles; `0` is never valid; never reused. | v1 |
| `bo_engine_load` | `(h, ptr: *const f64, len_f64: u32) -> i32` | Load a program and settle it at `START_TIME` (kernel semantics, point 6). | v1 |
| `bo_engine_advance` | `(h, t_end: f64) -> i32` | Process every event with `t <= t_end`, then set time to `t_end`. | v1 |
| `bo_engine_time` | `(h) -> f64` | Current simulation time. | v1 |
| `bo_engine_node_count` / `bo_engine_node_voltages` | `(h) -> u32` / `(h) -> *const f64` | `N` volts, node 0 = 0. | v1 |
| `bo_engine_device_count` / `bo_engine_device_currents` | `(h) -> u32` / `(h) -> *const f64` | One current per device record, in record order. | v1 |
| `bo_engine_stat` | `(h, i: u32) -> f64` | Counters (below). Unknown `i` gives `NaN`. | v1 |
| `bo_error_ptr` / `bo_error_len` | `() -> *const u8` / `() -> u32` | UTF-8 message for the most recent non-zero return. | v1 |
| `bo_engine_set_option` | `(h, key: u32, value: f64) -> i32` | Change an option after load. Keys 2 and 3 take effect at the next event; keys 1 and 4-7 are load-time only (`-8` after load). `FORCE_STEPPED` is load-time only because it decides partition classification, which is fixed at load. | v2 (W1a) |
| `bo_engine_next_event_time` | `(h) -> f64` | Time of the earliest pending event, `+Infinity` when none. | v2 (W1a) |
| `bo_engine_next_digital_due_time` | `(h) -> f64` | Time of the earliest pending driver commit (a live token; a commit held by the power-loss hold is not pending), `+Infinity` when none. Classic's `nextScheduledEventTime()` counterpart for a host that clamps its steps onto digital due times; it ignores stepped, crossing and bucket events. | v2 (W1c) |
| `bo_engine_digital_slot_count` / `bo_engine_digital_slots` | `(h) -> u32` / `(h) -> *const f64` | `S` values; `NaN` means the key is absent this pass. | v2 (W1a) |
| `bo_engine_channel_slot_count` / `bo_engine_channel_slots` | `(h) -> u32` / `(h) -> *const f64` | `C` per-channel currents (amps). | v2 (W1a) |
| `bo_engine_trace_enable` | `(h, on: u32) -> i32` | Record every digital slot change as `(t, slot, value)`. Harness instrumentation only; W2's host traces are separate. | v2 (W1a) |
| `bo_engine_trace_ptr` / `bo_engine_trace_len` / `bo_engine_trace_clear` | `(h) -> *const f64` / `(h) -> u32` (f64 count, a multiple of 3) / `(h)` | Read and reset the trace. | v2 (W1a) |
| `bo_engine_set_params` | `(h, ptr: *const f64, len_f64: u32) -> i32` | `(deviceIndex, paramIndex, value)` triples for each kind's live params (owner decision 1), applied as one event at the current time with all other state kept. | v2 (W1b) |
| `bo_engine_seed_memory` | `(h, device: u32, ptr: *const f64, len_f64: u32) -> i32` | Replace a 74ls189's 16 stored nibbles (owner decision 2). | v2 (W1d) |
| `bo_engine_take_eeprom_updates` | `(h) -> i32` | Collect and clear every dirty EEPROM image; returns the update buffer's length in f64 (`>= 0`), `-5` with no program. | v2 (W1d) |
| `bo_engine_eeprom_updates_ptr` | `(h) -> *const f64` | The buffer from the last take, valid until the next call on that engine. | v2 (W1d) |

**`bo_engine_set_params`** (W1b, owner decision 1). The buffer holds
`(deviceIndex, paramIndex, value)` triples.
- The call is all or nothing: every triple is validated before any is
  applied. A device index out of range, a param index outside that kind's
  live params (`kinds.rs` `live_params`), or a value that fails the kind's
  check is `-8`, and nothing changes.
- The changes apply as ONE event at the current time. The affected devices'
  stamps change, their partitions are marked dirty, and the delta cycle runs
  at that instant. All other state is kept: register contents, capacitor
  charge, pending delays. This reproduces the digital outcome of classic's
  carry-forward reload.
- If that delta cycle fails (`-4` or `-7`), the param changes roll back too,
  along with the rest of the instant, and the call returns the code. A
  failed toggle leaves the engine exactly as it was.

**Stat indices:** `0` partitions, `1` static solves, `2` largest partition,
`3` fixed nodes, `4` events processed, `5` delta passes, `6` maximum delta
depth, `7` solution-cache hits, `8` closed-form segments, `9` crossings,
`10` stepped steps, `11` Newton iterations, `12` IC evaluations, `13` driver
commits, `14` rail moves, `15` stepped partitions holding two or more
capacitors (decision 14's count; `FORCE_STEPPED` does not change it).

**Error codes:** `-1` malformed program; `-2` unknown kind code; `-3`
unsupported topology (the TS side falls back to classic; so do `-4` and `-7`
returned by `bo_engine_load`, because owner decision 8 sends a failed t=0
settle to classic); `-4` solve failed
(singular, non-finite pivot, a non-finite published value, or an IC fixed
point that does not converge); `-5` unknown
handle or no program loaded; `-6` `t_end` not finite or earlier than the
current time; `-7` delta-cycle livelock (the per-instant pass cap was
exceeded; the host treats it as non-convergence); `-8` invalid argument (an
unknown option key, a value out of range, a load-time option after load, a
device or param index out of range, or a param that is not live).

**`bo_engine_seed_memory`** (W1d, owner decision 2).
- `device` is a device record index. The buffer holds 16 values; value `i`
  is the nibble stored at address `i` in true sense (classic's `mem{i}`; the
  `/o` pins show its complement), an integer in `[0, 15]`.
- Only a `74ls189` accepts a seed. Any other kind, an index out of range,
  `len_f64 != 16`, or a bad value is `-8`, validated in full before anything
  changes; `-5` before a successful load.
- It replaces the 16 nibbles and nothing else: `lastWE`, drivers and pending
  events are kept.
- It applies as ONE event at the current time, like `bo_engine_set_params`:
  the device is re-evaluated in a delta cycle there, its readout shows the new
  word at once, and an output the new word changes requests it with the
  `addrO` pair. `-4` or `-7` rolls the contents back with the instant.
- A later load rebuilds every 189 at 0xF. Classic has no counterpart; the
  harness keeps writing classic's `icState` (test-only).

**`bo_engine_take_eeprom_updates`** (W1d). It collects, in device order,
every EEPROM whose `dirty` flag is set, clears those flags, and writes the
buffer: per EEPROM `deviceIndex, W, word x W`, the whole image packed as the
ROM blob is. `dirty` is set only when a write completes (even to the same
byte); neither load nor a seed sets it, and it rolls back with a failed
instant. The facade's `CoreEngine.takeEepromUpdates()` returns classic's
`SimEngine.takeEepromUpdates()` shape: component id to a fresh
`Uint8Array(size)`.

**A failed advance.** When `bo_engine_advance` returns `-4` or `-7`, the
engine keeps the state of the last instant it completed, and
`bo_engine_time` reports that instant. The host treats this as
non-convergence, as classic's `lastConverged = false`: it keeps the last
good readouts and reports how far time actually got. It is not a load
error. The stat counters are the exception: they keep the failed attempt's
counts, as classic keeps its failed diagnostics after restoring state.

**Memory.** Any call may grow the wasm memory, which detaches every JS view of
`memory.buffer`. The TS glue re-derives each view after every call and never
caches one; every pointer is read as unsigned (`>>> 0`). Pointers returned by
the readout exports are valid until the next call on that engine.

**Traps.** The crate builds with `panic = "abort"`, so a Rust panic is a wasm
trap. The TS side poisons the instance on a trap, and on any non-core error
thrown during an ABI call (a wasm stack overflow surfaces as a JS
`RangeError`): every later call throws until the core is re-initialised.

## Readouts

**Keys are built in TS; Rust publishes numbers.** The compiler emits a slot
table alongside the program: digital slot `i` maps to a `digitalState` key
(for example `u1/qa`, or a bare `u1` for a counter value), and channel slot
`j` maps to `(componentId, channel)`. The facade assembles the classic-shaped
records from it:
- `getNetV()`: every net, plus classic's unconditional `gnd: 0`.
- `getElementI()`: the key set classic's `_updateElementI` publishes (spec
  §10.5), with the device current where the kind has a current model and
  classic's `0` where it has none (`default: out[id] = 0`).
- `digitalState`: rebuilt from the slots after every advance; a `NaN` slot is
  an absent key, because classic rebuilds `digitalState` from scratch every
  pass (spec §10.9 lists every kind's keys and when they appear).
- `getElementChannelI()`: from the channel slots.

## Kernel semantics: the static solve (W0)

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
   two terminals are both free (W1b widens the union; see "Kernel semantics:
   partitions and dynamic solves (W1b)"). A resistor with one fixed end stamps
   the fixed voltage into the right-hand side; one with both ends fixed stamps
   nothing. The union-find itself is iterative (path halving, union by size): no input
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
   - `R_u` = sum over resistors and switch-family contacts of the current
     leaving node `u` through their live stamps, plus each capacitor's
     published current leaving `u`, plus the shunt current `1e-12 * V[u]`;
   - a child source `c` at `u` contributes `+i_c` if `c`'s + terminal is `u`
     and `−i_c` if its − terminal is `u`;
   - if source `s` fixed its + terminal `u`: `i_s = −(R_u + children)`;
     if it fixed its − terminal `u`: `i_s = R_u + children`.
6. **Time.** `advance(t_end)` with `t_end` below the current time by more than
   `1e-15` is `-6`. A non-finite `t_end` is also `-6`, with the message
   "t_end must be finite" — checked first, since a comparison against NaN is
   always false and would otherwise let it through and become the clock.
   Everything in W0 is static, so advancing only moves time.

## Kernel semantics: partitions and dynamic solves (W1b)

1. **Union.** The W0 union-find also joins, whatever their state:
   - both pins of a capacitor, when both are free;
   - both pins of every switch-family contact, when both are free, open or
     closed;
   - every free pin of one switch-family device with every other free pin of
     it, even when the contact between them runs through a fixed pin (an
     `spdt_switch` whose `com` sits on a source, a `dip_switch` whose `a`
     side is a rail).

   So every device's free pins share exactly one partition. Partition
   membership is fixed at load: a toggle only changes what is stamped, never
   which partition a node is in, and `bo_engine_set_params` marks that one
   partition dirty.
2. **Inert capacitor.** A capacitor whose two pins are on the same node
   stamps nothing, does not count toward its partition's classification, and
   publishes `0`, as classic does (its companion stamps cancel).
2a. **Capacitor across two fixed nodes** (a decoupling capacitor on an
   ideal supply; distinct nodes, so not rule 2). It belongs to no partition
   and is not `-3`: its terminal voltage `dV = V(pins[0]) - V(pins[1])` is
   fixed, so it evolves on its own, exactly.
   - `esr > 0`: `vc(t) = dV + (vc(t0) - dV) * exp(-(t - t0) / (esr * C))`
     from `vc = 0` at `START_TIME`, and the series current is
     `(dV - vc) / esr`.
   - `esr = 0`: `vc = dV` from the load instant on, and the series current
     is 0.
   - It publishes the series current plus the leakage `dV / leakage` (0 when
     `leakage` is 0, meaning none), `pins[0]` to `pins[1]`, and that current
     enters the KCL of the sources that fix its nodes (W0 point 5).
   - Known divergence: classic reports its load seed's `C * dV / 1e-12` at
     load and `C * dV / h` on its first step when `esr = 0`. Both are
     artifacts of its step size; the core reports the exact value.
   - It rolls back with the rest of the instant (rule 7). No fixed-node
     voltage is live in W1b, so nothing re-aims it after load yet.
3. **Classification** (decision 14). A partition with no counted capacitor
   is static. One with exactly one is closed-form. One with two or more is
   stepped. `FORCE_STEPPED` (load-time only) makes every closed-form
   partition stepped.
4. **Static partitions** keep a solution cache keyed on driver modes, the
   rail band, the floating bits and an epoch that bumps only on a real value
   change. `NO_SOLUTION_CACHE` bypasses it.
5. **Initial state at load.** Every capacitor starts at `vc = 0` at
   `START_TIME`, in closed-form and stepped partitions alike; carrying charge
   across a reload is W2. A stepped partition's clock starts at
   `START_TIME`, so a load integrates nothing and costs the same at any
   start time.
6. **Closed form** evaluates the exact exponential between events, and
   lands threshold crossings analytically. **Stepped** partitions use
   classic's controller (BE step doubling; trapezoidal with a BE anchor when
   `INTEGRATION` is `1`), land on every `t_end`, and refine crossings by
   bisection. When an event (a driver commit, a reset, a `set_params`)
   dirties a stepped partition, its catch-up to the event instant integrates
   with the stamps in force BEFORE the event. The new stamps apply from the
   instant on.
7. **Rollback.** A failed instant (`-4`, `-7`, in `advance` or
   `set_params`) restores every piece of solver state the attempt touched:
   published arrays, each static partition's own solution and cache, the
   closed-form and stepped partition state, and any crossing or step event
   the attempt scheduled. Nothing from the failed attempt reappears later.

## Kernel semantics: the digital core (W1c)

References are to `core/spec/w1-classic-semantics.md`.

1. **Power** (§1.3), per IC, from its supply-node voltages:
   `vSraw = V(vcc) - V(gnd)`; `powered = !OPEN(vcc) && !OPEN(gnd) &&
   vSraw >= vccMin*0.9 && vSraw <= vccMax*1.1`, the products computed in f64
   at run time; `vS = max(0, vSraw)`; `R_out = ioMax > 0 && vS > 0 ?
   clamp(vS / (4*ioMax), 50, 1000) : 50`. Thresholds follow `thresholdsFor`
   at `vS`: with `CMOS_RATIO`, `vil = max(0.2, 0.3 vS)` and
   `vih = max(0.4, 0.7 vS)`; otherwise the fixed value when its flag is set,
   else `max(0.8, 0.3 vS)` and `max(2.0, 0.6 vS)`.
2. **Partitions.** An IC's driver stamps couple each output-capable pin to its
   own supply nodes, and a floating pull's target follows both supply
   voltages (point 6). So the
   union-find also joins each free output-capable pin, and each free `OPEN`
   `FLOATING_INPUT` pin, with every free supply pin of the same IC. Reading a
   pin joins nothing. When both supply pins are fixed (an ideal supply), no
   IC pin joins anything, and every output node partitions with its own load.
3. **Stamps that depend on power** are an IC's driver conductances (`1/R_out`
   to its own `vcc` or `gnd` node), its floating pulls, and whether it stamps
   at all. Classic recomputes them from every Newton iterate with no Jacobian
   term, which is a fixed-point iteration, and the core does the same: a
   partition holding a free IC supply node re-stamps from the latest iterate
   until classic's Newton test passes (`|dx| <= 1e-6 + 1e-3 |x|` per node, at
   most 50 iterations; failure is `-4`). A dynamic partition holding a free IC
   supply node is stepped, because R_out then moves with its state. So is a
   dynamic partition holding a 74ls245 `a`/`b` pin: those pins are read and
   driven, a closed-form crossing watch cannot follow a pin its own IC drives,
   and a stepped partition re-reads its ICs after every step. When every
   supply node is fixed, the power is constant and the solve is direct.
4. **Reads** (§1.4), at each reader evaluation, from the solved voltages,
   relative to `V(gnd)`:
   - an `OPEN_HIGH` pin that is `OPEN` reads 1;
   - otherwise an `OPEN` pin reads the floating model with the open key;
   - a Schmitt part reads `v >= vtPlus*s` HIGH and `v <= vtMinus*s` LOW,
     with `s = nominal > 0 ? vS/nominal : 1`; in between it holds its
     per-pin state (default LOW);
   - any other pin reads `v >= vih` HIGH and `v <= vil` LOW; in between it
     reads the floating model with the dead key.

   A reader evaluation is classic's `commit = true`, so it may write Schmitt
   state. An unpowered IC reads nothing. An IC reads its supply nodes and
   every logic pin's node. A reader whose pin sits on a closed-form or stepped
   partition registers that pin's live thresholds (`vil`/`vih`, or the scaled
   `vtMinus`/`vtPlus`, and the power window on the supply) with the
   partition's crossing watch, so it is re-evaluated at the crossing instant
   (accepted only at `t > now + 1e-15`), not at the next unrelated event. An
   output-capable pin is not watched: its level never changes the kind's
   result (the 74ls245 aside, rule 3), and it sits on its own driver. A
   stepped partition instead re-reads every IC reading a node that moved,
   after every step, including the steps a horizon clamp takes (W1a point 5).
   A watch's thresholds come from the IC's live supply voltages. So an IC
   whose supply sits in a static partition is watched too, and its watches
   are re-aimed whenever that supply moves. Only an IC with a supply node in
   a dynamic partition relies on the stepped re-read instead.
5. **Floating model** (§1.5, owner decision 9): `bucket = floor((t + 1e-12) /
   0.01)` of the evaluation time; `n = finish(hashState, bucket) /
   0xffffffff`; HIGH when `n < pHigh`. At each bucket boundary, every
   partition holding an `OPEN` `FLOATING_INPUT` pin of a powered IC is marked
   dirty (its pull target may change), and every IC holding such a pin or
   reading a dead-band pin is re-evaluated.
6. **Floating pulls** (§1.6): while powered, each `OPEN` pin with
   `FLOATING_INPUT` and not `OPEN_HIGH` gets `g = 1/(TTL ? 1e6 : 1e7)` toward
   `V(gnd) + (high ? vS : 0)`, `high` from the open-key hash, so the pin
   voltage and its read agree. It is stamped as classic stamps it, as a
   Norton branch to the datum: `g` from the pin to node 0 plus an injected
   `g * target`, with the target taken from the latest iterate. So the pull's
   current never passes through the IC's supply source. A conductance to the
   `vcc` or `gnd` node gives the same pin voltage, but it would bill a pulled
   pin that its own IC drives (a 173 `q`, a 189 `/o`, an EEPROM `io`, a 245
   `a`/`b` left unconnected) to the supply source.
7. **Evaluation** (§3.6): the 74ls245 drives neither side when `/oe` reads 1,
   and otherwise drives one side and requests HIZ on the other; the 74ls157
   drives every output LOW when `/oe` (the strobe `/G`) reads 1 (W1d,
   Q-O64). Otherwise the kind's truth table over the read levels, with
   74ls47's precedence `rbo_n` over `lt_n` over `rbi_n` and its partial glyphs
   for 10-15, and the 74hc138 driving every output HIGH when disabled. Each
   output-capable pin in the result requests its level through its driver
   (§2.4, the kernel's inertial FSM, with the `delay` param; `delay = 0`
   commits in the next delta). A pin the result does not drive requests HIZ
   through the same FSM, with its disable delay (W1d, delays point 4); no IC
   requests a reset. Every evaluation counts toward
   stat 12 (IC evaluations), which need not equal classic's
   `digitalEvaluationCount` (D27).
8. **Output stage** (§1.7, §3.7): while powered, a committed HIGH stamps
   `1/R_out` from the pin to the `vcc` node and a committed LOW to the `gnd`
   node. An `OPEN_COLLECTOR` pin committed HIGH is released: it stamps nothing.
   An unpowered IC stamps nothing at all: no drivers, no pulls. An output
   on the datum node stamps nothing and draws no source current, as
   classic's `_stampDigitalOutput` returns for a pin on the reference net.
   An output on any other fixed node stamps into its supply node's partition
   when that supply is free, and its current enters the source KCL. A commit
   on a fixed-node driver re-publishes the source currents at that instant.
9. **Power loss** (owner decision 8, Q-O4): while an IC is unpowered its
   drivers keep their committed level and in-flight transition frozen; a
   commit that comes due meanwhile is held, not applied. On the first powered
   evaluation the partition is solved with the pre-outage committed levels,
   and any held transition whose due time has passed commits in the next
   delta, as classic's first powered pass does.
10. **Readouts** (codes 50-61). A digital slot holds the pin's committed level
    (electrical: HIGH, or released for open-collector, is 1) while the pin is
    driven, and `NaN` (an absent key) while the IC is unpowered, or the pin is
    committed HIZ or has never committed. Codes 70-82 publish evaluator state
    instead (W1d).

## Kernel semantics: datasheet delays and enables (W1d)

1. **Delay set.** A `DriverRequest::Level` carries `delays: DelaySet { lh,
   hl, zh, zl, hz, lz }` (seconds) in place of W1c's single `delay`. The
   evaluator builds one set per output-capable pin, from the pin's output
   class and the cause of the request (point 3). A kind with no tail sends
   `DelaySet::uniform(param 12)`, so its timing is unchanged. Figures are
   fixed at load and do not scale with supply or load (classic, §1.3).
2. **Pick.** `Driver::request` picks exactly one delay with a pure
   `fsm::pick(committed, requested, delays)` beside the FSM, then runs the
   unchanged §2.4 FSM. The pick keys on the COMMITTED level, because the pin
   is still physically there.

   | committed → requested | delay |
   |---|---|
   | none → any | 0 (first observation) |
   | LOW → HIGH | `lh` |
   | HIGH → LOW | `hl` |
   | HIZ → HIGH | `zh` |
   | HIZ → LOW | `zl` |
   | HIGH → HIZ | `hz` |
   | LOW → HIZ | `lz` |

   A request for the pending target keeps its due time; a request for the
   committed level cancels. Load settling's zero override zeroes the whole
   set, and `Kernel::request_driver` takes `DelaySet::uniform(delay)`.
3. **Cause.** Each IC keeps `lastReads`, its logic-pin reads at its previous
   powered evaluation (none before the first).
   - A level transition's `lh`/`hl` come from the pair of the cause that
     produced the new target: an input whose read differs from `lastReads`,
     or a state change the evaluation made (a clock edge, a clear, a write, a
     completed EEPROM write, a seeded word, a polling toggle).
   - With several causes the larger figure for that direction wins: a
     datasheet's output-valid time is the latest of its constraints. With no
     identified cause (a first powered evaluation, or a read that changed
     nothing the table names), the default path applies: the first entry in
     the kind's `lh`/`hl` cell below (the 157's param 12, the 161's `clkQ`,
     the 189's `addrO`). A `DelaySet` entry the kind can never select (a pin
     that never goes HIZ) carries that same default, never 0.
   - `zh`/`zl`/`hz`/`lz` come from the output class's enable and disable
     pairs; where a kind has several enables, from the one that changed.
   - `lastReads` is written at every powered evaluation, frozen-settle passes
     included (it only chooses delays, which are zero while settling). It is
     in the evaluator's snapshot, so it rolls back with a failed instant; a
     load drops it.

   | Kind | Outputs | `lh`/`hl` by cause | `zh`/`zl` | `hz`/`lz` |
   |---|---|---|---|---|
   | `74ls157` | `1y..4y` | param 12; `strobe` when the `/oe` read changed | - | - |
   | `74ls245` | `a1..a8`, `b1..b8` | param 12 (data) | `oeEnable`, from `/oe` or `dir` | `oeDisable`, from `/oe` or `dir` |
   | `74ls161` | `qa..qd` | `clkQ` (a clock edge: count or load); `clrQ` (`/clr`) | - | - |
   | `74ls161` | `rco` | `clkRco` (the count changed on an edge); `clrQ` (a clear); `entRco` (`ent` changed alone) | - | - |
   | `74ls173` | `q1..q4` | `clkQ` (a clock edge); `clrQ` (`/clr`, active HIGH) | `mnEnable` | `mnDisable` |
   | `74hc74` | `q1 q1_n q2 q2_n` | `clkQ` (a clock edge); `preClrQ` (`/pre` or `/clr`) | - | - |
   | `74ls189` | `/o1../o4` | `addrO` (the address or the stored word changed) | `csEnable` (`/cs` fell); `weEnable` (`/we` rose) | `csDisable` (`/cs` rose); `weDisable` (`/we` fell) |
   | `28c16`, `28c256` | `io0..io7` | `addrIo` (the address, the byte or a polling bit changed) | `ceEnable` (`/ce` fell); `oeEnable` (`/oe` fell) | `ceDisable` (`/ce` rose); `oeDisable` (`/oe` rose) |

4. **Tri-state is a HIZ request, never a reset.** An output the evaluator
   does not drive requests `Mode::Hiz` through the same FSM, as the
   prototype's three-mode `drive()` did (§2.17):
   - disable: from HIGH or LOW, HIZ commits at `now + hz` or `now + lz`, and
     until then the pin keeps driving its committed level;
   - enable: from HIZ, the level commits at `now + zh` or `now + zl`, and until
     then the pin stamps nothing;
   - a glitch back before the commit cancels it (the pin never releases, or
     never drives); a new level mid-transition supersedes it with a fresh due
     time picked from the committed level;
   - a committed HIZ stamps nothing, exactly like an uncommitted driver;
   - no IC requests `DriverRequest::Reset` any more (the variant stays for
     its kernel tests), so after load no pin re-seeds with zero delay.
5. **74ls245.** Evaluation as classic: `/oe` HIGH requests HIZ on every
   `a`/`b` pin; `/oe` LOW with `dir` HIGH drives `b_i = a_i` and requests HIZ
   on `a_i`, and `dir` LOW the reverse. Level changes use param 12. A `dir`
   flip with `/oe` LOW disables the old side with `oeDisable` and enables the
   other with `oeEnable`: the datasheet has no DIR row, and DIR gates the same
   output enables. Every disable figure is shorter than every enable figure
   (15 < 25 ns), so the new side cannot commit a value read back through the
   old side's still-driven pin: the old side's HIZ commit re-evaluates the
   245, and the enable target is re-picked before its due time.
6. **74ls157 strobe** (Q-O64; classic fixed in the same change, spec delta
   1): `/oe` HIGH forces every `y` LOW, and the outputs never go HIZ.
   Strobe-caused transitions use `strobe` (13 ns release, 14 ns force); data
   and select changes keep param 12 (classic's 20 ns). Classic takes its one
   20 ns package delay for all of them.
7. **State across calls.** A driver's FSM state (committed level, pending
   transition, token) survives `bo_engine_set_params` and `bo_engine_seed_memory`,
   rolls back with a failed instant, and is rebuilt uncommitted by a load, so
   the load settle is all first observations at zero delay. Classic carries
   the `delay:/pending:/due:` slots across `load()` by id and kind; in the
   core that is W2 (load carry-forward). HIZ commits are ordinary commits:
   held while unpowered (W1c point 9), counted in stat 13, and reported by
   `bo_engine_next_digital_due_time`.
8. **Readouts of the 157 and 245** stay the committed pin level (W1c point
   10): a 245 pin shows its old level while a disable is in flight and `NaN`
   while an enable is; the 157's four keys are always present while powered.

## Kernel semantics: sequential ICs (W1d)

References are to spec §4.1 and §4.2.1-4.2.3 and classic's
`devices/digital-ics.ts`.

1. **Shared machinery.** Codes 70-72 are ICs for W1c points 1-6, 8 and 9:
   power, partitions, power-dependent stamps, reads (no Schmitt branch; the
   74hc74's `pre1_n clr1_n pre2_n clr2_n` are `OPEN_HIGH`), the floating model
   and pulls (an open 173 `q` pin is pulled whether or not it is driven,
   because classic stamps pulls before outputs), the output stage and the
   power-loss hold. A general IC predicate replaces the combinational-only
   one wherever W1c tests it.
2. **Register state** lives in the evaluator, one record per device: 161
   `{count 0-15, lastClk}`, 173 `{q1..q4, lastClk}`, 74hc74 `{q1, q1n,
   lastClk1, q2, q2n, lastClk2}` (Q̄ is stored so that Q = Q̄ = 1 can exist).
   Power-on values are classic's `defaultIcState`: all 0 except `q1n = q2n =
   1`. The records are in the evaluator's snapshot (a failed instant restores
   them), `bo_engine_set_params` keeps them, and a load starts from power-on
   values (classic carries them by id; W2). No export seeds a register.
3. **One evaluation** of a powered device: (a) read every logic pin; (b) when
   edges are live, apply the update (point 5) and set every `lastClk` to its
   clock's read level; while settling with edge history frozen, skip (b)
   entirely; (c) compute each driver's target (point 6); (d) request it with
   the delays of the delays section. An unpowered device reads, writes and
   requests nothing, so its state and `lastClk` freeze and the first powered
   evaluation compares the clock with the pre-outage `lastClk` (Q-O4).
4. **Edges** (owner decision 6, D22). A rising edge is `clkRead &&
   !lastClk`, `clkRead` read through W1c point 4 (thresholds, floating model,
   dead-band key) at every event that can change a read, not once per step.
   So a clock entering the dead band is an edge whenever the dead key hashes
   HIGH, an open clock can take an edge at a bucket boundary, and a pulse
   narrower than a classic step counts. `lastClk` is written at every live
   evaluation whatever branch applies, so an edge that arrives while a clear
   holds is consumed. Every other input is sampled at the evaluation that
   detects the edge, after that delta's commits.
5. **Update**, in priority order (classic's `updateDigital`):
   - `74ls161`: `/clr` reads 0 → count = 0 (asynchronous, level-sensitive,
     at every evaluation); else a rising edge with `/load` reading 0 → count =
     `a + 2b + 4c + 8d`; else a rising edge with `enp` and `ent` both 1 →
     count = (count + 1) & 0xF; else hold.
   - `74ls173`: the pin named `/clr` is active HIGH (classic): it reads 1 →
     q1..q4 = 0; else a rising edge with `g1` and `g2` both reading 0 → qi =
     di; else hold. M and N do not enter the update.
   - `74hc74`, per flip-flop: `/pre` and `/clr` both 0 → Q = Q̄ = 1; `/clr`
     0 → Q = 0, Q̄ = 1; `/pre` 0 → Q = 1, Q̄ = 0; a rising edge → Q = D, Q̄ =
     !D; else hold.
6. **Targets** (classic's stamp): 161 `qa..qd` = count bits 0-3, `rco` =
   count == 15 && ENT reads 1 (ENP never gates RCO); 173 `qi` = bit i when M
   and N both read 0, else HIZ; 74hc74 the stored Q and Q̄ with point 5's
   asynchronous priorities applied from the current reads (a no-op once (b)
   ran, but it drives the frozen settle).
7. **Load** (decision 8). The frozen settle drives classic's stamp view of
   the power-on state, with the live paths live (161 RCO from ENT, 173 M/N,
   74hc74 `/pre` and `/clr`). The one edge pass then runs point 5 with every
   `lastClk` at 0: a clock already reading HIGH gives classic's phantom rising
   edge (optimization-equivalence's count 1 at 1 ms), and a held clear clears.
   The requests of that pass carry their real delays, so the phantom edge's Q
   changes land at `START_TIME + clkQ`, where classic's land on its next
   solve; the slots at `START_TIME` already show the post-edge state.
8. **Readouts** (Q-O8, decision 6: mirror classic). A slot holds the register
   value after the latest evaluation, not a driver level: it changes at the
   evaluation that changes the state, ahead of the pin by the delay (classic
   leads by one solve). The 161 publishes its count under the bare key
   `<id>` and bits 0-3 under `qa..qd` (no `rco` key); the 173 shows its
   register while M or N tri-states the pins. Every slot is `NaN` while the
   device is unpowered.
9. **Events and stats.** A register needs no events of its own: W1c's watches
   on every read pin and its bucket events wake it. Its output commits are
   ordinary commits, and every evaluation counts toward stat 12.

## Kernel semantics: memories (W1d)

References are to spec §4.2.4-4.2.5.

1. **Shared machinery.** As for the sequential ICs. The EEPROM's `io` pins are
   driven and also read (sampled at a latch), so they follow the 74ls245
   rule: never crossing-watched, and a dynamic partition holding one is
   stepped (W1c point 3).
2. **State at load.** 189: `mem[0..15] = 0xF`, `lastWE = 1`. EEPROM: the
   blob's image, `lastWeHigh = lastCeHigh = true`, no pending write,
   `pollReads = 0`, SDP locked at step 0, `dirty = false`. So the settle's edge
   pass sees no write edge (no phantom write, as classic).
3. **74ls189**, one powered evaluation, in order:
   1. `csLow = !/cs`, `weHigh = /we`.
   2. Write, with edges live only: when `csLow && weHigh && lastWE == 0`,
      `mem[addr] = D` (`d1` is bit 0), address and D read now (the trailing
      edge of `/we`). Then `lastWE = weHigh`, whatever `/cs` reads.
   3. Drive: while `csLow && weHigh`, `/oN` requests HIGH when bit `N-1` of
      `mem[addr]` is 0 (inverted outputs); otherwise every `/oN` requests HIZ.
   4. Publish: while `csLow`, slot `N` is 1 when bit `N-1` of `mem[addr]` is
      0, else 0, write mode included (classic); otherwise `NaN`.
4. **28c16 and 28c256**, one powered evaluation, in order (`mask = size - 1`):
   1. Completion: a pending write with `now >= completesAt` stores
      `bytes[pendingAddr] = pendingByte`, sets `dirty`, clears the pending
      write and `pollReads = 0`.
   2. Latch, with edges live only: `weLatch = weHigh && !lastWeHigh && ceLow
      && oeHigh`, `ceLatch = ceHigh && !lastCeHigh && !weHigh && oeHigh`. On
      either, with no write pending, sample the address and `io0..io7` now
      and run SDP. An accepted write sets `pendingAddr = addr & mask`,
      `pendingByte`, `completesAt = now + tWC`, `pollReads = 0`, and schedules
      a `Timer` for the device at `completesAt`. An edge while a write is
      pending is ignored and never reaches SDP. Then `lastWeHigh = weHigh`,
      `lastCeHigh = ceHigh`.
   3. SDP (when `sdp = 1`) is classic's `consumeEepromWrite` verbatim:
      unlocked accepts; otherwise a write matching `[(0x5555, 0xAA), (0x2AAA,
      0x55), (0x5555, 0xA0)][sdpStep]` advances the step (the third match
      unlocks and resets the step), a mismatch resets the step, and in both
      cases the byte is dropped. The unlock lasts until the next load.
   4. Toggle bit: while a write is pending, an evaluation at which `/oe` and
      `/ce` both read LOW, having not both read LOW at the previous powered
      evaluation, adds 1 to `pollReads` (one read cycle, the datasheet's
      "successive reads toggle I/O6"). Classic adds 1 per accepted step (a
      step-grid artifact the core cannot reproduce; a divergence).
   5. Drive: while `oeLow && ceLow`, `ioK` requests bit K of `bytes[addr]`;
      with a write pending and `addr == pendingAddr`, io7 requests the
      complement of `pendingByte` bit 7 and io6 requests `pollReads & 1` (DATA
      polling). Otherwise every `ioK` requests HIZ.
   6. Publish: while `oeLow && ceLow` and no write is pending, slot K is bit K
      of `bytes[addr]` (never a polling bit); otherwise `NaN`.
5. **Write completion** (Q-O7). The latch's `Timer` only guarantees an
   evaluation at `completesAt`; the completion itself is step 4.1, so an
   evaluation then for any other reason completes the write and the `Timer`
   finds nothing to do. A `Timer` that finds the IC unpowered leaves the
   completion to its first powered evaluation. The kernel evaluates a device
   in the delta in which its live `Timer` pops; `bo_engine_next_event_time`
   reports it and `bo_engine_next_digital_due_time` does not (classic's
   `nextScheduledEventTime` never sees `writeCompletesAt`).
6. **Readouts** mirror classic (Q-O8): a slot holds the evaluator's view at
   its latest evaluation, not the committed pin level, and runs ahead of the
   pins by the delay. `NaN` while unpowered.
7. **Rollback.** A failed instant restores the stored contents, edge
   history, pending write, `pollReads`, SDP progress, `dirty`, `lastReads` and
   the readouts. The EEPROM image is shared copy-on-write, so the per-instant
   evaluator snapshot never copies 32 KiB; only a completion writes it.
8. **Reload.** Nothing carries across `bo_engine_load` in W1: RAM returns to
   0xF and the EEPROM to its contents image. Classic's carry rules (189 by
   id; EEPROM by contents hash and size) are W2's load carry-forward.

## Kernel semantics: events (W1a)

1. **Event order.** One heap ordered by `(t, seq)` with `f64::total_cmp`;
   `seq` is a global insertion counter, so events at equal `t` run first in,
   first out.
2. **Drivers.** A driver's commit (a commit to HIZ included: the driver stops
   driving, as a tri-stated or direction-flipped pin does) and any other change to what it
   stamps all mark its partition dirty for the next re-solve; readers of that
   partition are then re-evaluated. Every IC output is a driver: a conductance to its IC's own
   supply pin nodes (so load current debits the rails, spec §1.7) with mode
   `HIZ`, `HIGH` or `LOW`. A requested change goes through the inertial rules
   of spec §2.4, as one pure function: the first observation has zero delay;
   a glitch shorter than the delay is swallowed; a re-glitch gets a fresh due
   time; at most one transition is in flight per pin. Each pending commit
   carries a token; a newer request for the same driver supersedes it. Each
   request carries a delay set, and the driver picks one delay from its
   committed level and the requested level (W1d, delays point 2).
3. **Delta cycles** (owner decision 4). At an instant `t`:
   1. apply every due commit at `t`, in `(t, seq)` order, marking the
      partitions they touch dirty;
   2. re-solve the dirty partitions, then evaluate each affected reader once;
   3. zero-delay outputs scheduled at `t` form the next delta. Repeat.

   More than 1000 deltas at one instant is `-7`, never a silent exit.
4. **Crossings.** A threshold crossing is accepted only at `t > now + 1e-15`.
5. **Advance.** Process every event with `t <= t_end`, then set time to
   `t_end`. Stepped partitions land exactly on `t_end` (a horizon clamp), so a
   readout at `t_end` is a solved value, not an interpolation. The readers
   whose nodes the clamp moved are evaluated in a delta cycle at `t_end`, so
   a digital readout at `t_end` reflects them, as if a step had landed there.
   Interpolated acquisition is W2.
6. **Load settling** (owner decision 8, and the W1-0 spike). Three steps, at
   `START_TIME`:
   1. Settle the network to a fixed point with every register's edge history
      FROZEN: parts drive their current state but never act on an edge or
      update `lastClk` or `lastWE`. Level-sensitive asynchronous inputs (a
      74hc74's `/pre` and `/clr`, a 161's ENT to RCO, a 173's M/N) change the
      outputs, as classic's seed stamp does, but never the register state;
      the edge pass does the state writes. Delays do not apply while settling; this
      is finding the consistent initial state, not simulating time.
   2. Run exactly ONE edge pass: every reader is evaluated once with edges
      live. Classic's phantom rising edge (`lastClk` seeded 0 while the clock
      is already HIGH) is kept.
   3. Whatever that edge pass requests is ordinary simulation from then on:
      it goes through the inertial rules with its real delays and is drained
      at `START_TIME` by the normal delta cycle, exactly as an advance to
      `START_TIME` would. It is never re-run as further zero-delay settle
      passes.

   Classic's 1 ps seed and rescue ladder are not reproduced. The spike showed
   that re-running edge-acting passes during settling lets a transient
   glitch clock a register or write RAM; step 3 is what rules that out.
7. **Floating buckets** (owner decision 9). A floating or dead-band read uses
   classic's key with `bucket = floor((t + 1e-12) / 0.01)` of the event time,
   and every device that reads such a pin gets an event at each bucket
   boundary.

## Hard-won rules for the kernel (from the prototype)

- Accept a threshold crossing only at `t > now + 1e-15`; rounding can put a
  node back on the threshold and re-schedule it forever.
- Guard closed-form re-aiming until the first segment exists (`segReady`), or
  it produces NaN.
- Keep a pending plan's capacitor history separate from the committed `vc`.
- Bidirectional parts (74LS245) register every A and B pin as a reader as well
  as creating drivers.
