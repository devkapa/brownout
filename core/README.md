# brownout core v2 (Rust/WASM) — contract

The partitioned event-driven transient engine, compiled to WebAssembly. The
plan and the locked owner decisions live in the devolt repo:
`memory/decisions/2026-09-24-brownout-rust-wasm-core.md` (plan) and
`...-handoff.md` (state and hard-won detail). This file is the technical
contract the TypeScript front end and the Rust back end are both written
against. Change it first, then both sides.

**Status: W1e done (NE555, LEDs, 7-segment displays; checkpoint S1); W1f
(rails and sources) contracted; W1g (thermal, failures and warnings)
contracted.** W0
proved the pipeline with a
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
  | 3 | `NO_RAIL_SHORTCUT` | `0` / `1` (exactness switch; load-time only from W1f) | `0` |
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
  blob value finite, every slot index `< S` or `< C` (from each kind's slot
  bases, below), the trailer, and no trailing data. Malformed is `-1`, never
  a panic; a well-formed but unknown kind code is `-2`. Per-kind rules (for
  example resistor `R > 0`, or a `led`'s `n > 0`, `isSet` and `burnt` in
  `{0, 1}` and `is = 0` when `isSet = 0`) live in `core/src/kinds.rs`. A bad option inside the program is `-1` (the program
  is malformed); the same bad value passed to `bo_engine_set_option` after
  load is `-8` (an invalid argument).
- **Slot indices.** A kind declares in `kinds.rs` how many digital slots
  (`digital_slots`) and channel slots (`channel_slots`) a record owns. Its
  blob starts with the first slot of each kind it uses, digital before
  channel: `blob[0]` for the first kind used, `blob[1]` for the second. The
  reader checks that each base is an integer and `base + count <= S`
  (digital) or `<= C` (channel). No two devices' digital ranges overlap, and
  no two devices' channel ranges overlap; the reader rejects an overlap as
  malformed (`-1`), because the digital readout publishes only the slots whose
  value changed, which is exact only while each slot has a single writer. The
  W1c ICs are the first kinds with digital slots, codes 30-34 the first with
  channel slots.

## Kind codes and records

Codes are grouped in blocks by family. A kind's pin order, params and blob are
written into this table by the orchestrator before the sub-wave that ports it
is dispatched; a code without a record layout is reserved and rejected as
`-2`.

| Block | Codes |
|---|---|
| Passives and switches | 1 `resistor`, 3 `capacitor`, 4 `switch`, 5 `push_button`, 6 `spdt_switch`, 7 `dip_switch`, 8 `potentiometer`, 9 `trimmer`, 10 `fuse`, 11 `ldr`, 12 `thermistor`, 13 `ptc_fuse` |
| Sources | 2 `voltage_source` (kept from v1), 20 `battery_pack`, 21 `bench_psu`, 22 `clock`, 23 `clock_gen`, 24 `pulse_source`, 25 `pulse_gen`, 26 `section_anchor` (synthetic: emitted by the compiler, never a component kind) |
| LEDs and displays | 30 `led`, 31 `rgb_led`, 32 `bicolor_led`, 33 `seg7_cc`, 34 `seg7_ca` |
| Timers | 40 `ne555` |
| Combinational ICs | 50 `74ls00`, 51 `74ls04`, 52 `74ls08`, 53 `74ls32`, 54 `74ls86`, 55 `74ls157`, 56 `74ls283`, 57 `74hc138`, 58 `74hc14`, 59 `74ls47`, 60 `cd4511`, 61 `74ls245` |
| Sequential ICs | 70 `74ls161`, 71 `74ls173`, 72 `74hc74`, 73 `74hc595`, 74 `74hc165`, 75 `cd4017`, 76 `cd4060` |
| Memories | 80 `74ls189`, 81 `28c16`, 82 `28c256` |
| Slow physics | 90 `rating`, 91 `thermal` (both synthetic: emitted by the compiler, never a component kind) |

The W1e rows and sections cite classic by file and line at `3b22a3e`: `SE`
`src/sim/engine/sim-engine.ts`, `SC` `src/sim/engine/devices/semiconductors.ts`,
`DI` `src/sim/engine/devices/digital-ics.ts`, `EL` `src/sim/engine/elements.ts`,
`NW` `src/sim/engine/newton.ts`, `GR` `src/sim/engine/graph.ts` and `HL`
`src/host/headless.ts`. Where one differs from the spec's 2026-09-24
citation, the code decides.

The W1f rows and sections cite classic at `2b5faf1` (its `sim-engine.ts`
moved after `3b22a3e`): `SE` `src/sim/engine/sim-engine.ts`, `SO`
`src/sim/engine/devices/sources.ts`, `BP` `src/sim/battery-physics.ts`, `WF`
`src/sim/engine/digital.ts` (the waveform functions), `GR`
`src/sim/engine/graph.ts` and `HL` `src/host/headless.ts`; `P3` is the
prototype's `brownout-core-prototype/js/pec3.mjs`. "W1f Qn" is ruling `n` of
the W1f contract review (orchestrator and manager, 2026-09-30; devolt repo,
`memory/progress/brownout-core-w1e-drafts/w1f-draft.md`, section 10), and
"probe Pn" or "probe Qn" is a classic probe in that draft's section 8.

The W1g rows and sections cite classic at `2b5faf1` too, with W1f's keys
plus `PA` `src/sim/engine/devices/passives.ts`, `SC`
`src/sim/engine/devices/semiconductors.ts` and `TP` `src/sim/thermal-physics.ts`
(all three unchanged since `3b22a3e`). The classic they describe carries the
four brownout 0.6.1 fixes of W1g step 0, each on its own branch, which change
lines cited at `2b5faf1`: `fix/output-sag-direction-debounce` (`7b1056d`, the
`output_sag` debounce, W1g Q1), `fix/resistor-default-overload` (`fe9b38a`,
the resistor's and the fuse's absent-param defaults, Q-O27 and W1g Q3),
`fix/cold-start-thermal-warnings` (`efceffb`, Q-O42) and
`fix/74ls47-released-segment-sag` (`d1dee17`, Q-O49). "W1g Qn" is ruling
`n` of the W1g contract review (manager, 2026-10-01; devolt repo,
`memory/progress/brownout-core-w1e-drafts/w1g-draft.md`, section 11), and
"probe F", "probe S" or "probe C" is a classic probe in that draft's section
10.

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
| 10 | `fuse` | `pins[0]`, `pins[1]` (positional, PA:695-696) | `[rNormal, iRating]` (Fuse and PTC records below) | empty | W1g |
| 13 | `ptc_fuse` | `pins[0]`, `pins[1]` (positional, PA:760-761) | `[rNormal, iHold]` (Fuse and PTC records below) | empty | W1g |
| 20 | `battery_pack` | `pos`, `neg` (by id, SO:146-150) | the 9 battery params (Battery record below) | the OCV table `[n, (soc, volts) x n]` | W1f |
| 21 | `bench_psu` | `pins[0]` (+), `pins[1]` (−) (positional, SO:226-227, 276-281) | `[V, iLimit]` (Bench PSU record) | empty | W1f |
| 22, 23 | `clock`, `clock_gen` | `pins[0]` (+), `pins[1]` (−) (positional) | `[freq, duty, vHigh, vLow, delay]` | `[slotBase]`, 1 digital slot | W1f |
| 24, 25 | `pulse_source`, `pulse_gen` | `pins[0]` (+), `pins[1]` (−) (positional) | `[v1, v2, td, tr, tf, pw, per]` | `[slotBase]`, 1 digital slot | W1f |
| 26 | `section_anchor` (synthetic) | one pin, the anchor node | none | empty | W1f |
| 30 | `led` | `pins[0]` (anode), `pins[1]` (cathode) (positional, SC:356-357) | the 6 `led` params (LED records below) | empty | W1e |
| 31 | `rgb_led` | `r_a`, `g_a`, `b_a`, `com_k` (by id, SC:618-623) | the 6 `rgb_led` params | `[channelBase]`, 3 channel slots | W1e |
| 32 | `bicolor_led` | `a1`, `a2`, `k` (by id, SC:551-552) | the 5 `bicolor_led` params | `[channelBase]`, 2 channel slots | W1e |
| 33, 34 | `seg7_cc`, `seg7_ca` | `a b c d e f g dp com com2` (by id; segments in channel order, DI:1206) | the 2 `seg7` params | `[channelBase]`, 8 channel slots | W1e |
| 40 | `ne555` | `8` (vcc), `1` (gnd), then `2 3 4 5 6 7` (by id; NE555 record below) | the 13 IC params, with 7-9, 11 and 12 set to 0 | `[slotBase, 0 x 6]` | W1e |
| 50-59, 61 | the 11 combinational ICs: `74ls00`, `74ls04`, `74ls08`, `74ls32`, `74ls86`, `74ls157`, `74ls283`, `74hc138`, `74hc14`, `74ls47`, `74ls245` | `vcc`, `gnd` (the pins classic's `supplyPinsForComponent` resolves), then the kind's logic pins in `icPinMap(kind)` order, by id (table below) | the 13 IC params below, then the kind's delay tail (74ls157: 1 pair, 74ls245: 2 pairs, the other nine: none) | `[slotBase, pinFlags x L]`, `L` the logic-pin count | W1c; tail W1d |
| 70-72 | `74ls161`, `74ls173`, `74hc74` | `vcc`, `gnd`, then the kind's logic pins by id (sequential table below) | the 13 IC params, then the kind's delay tail | `[slotBase, pinFlags x L]` | W1d |
| 80 | `74ls189` | `vcc`, `gnd`, then the memory logic pins by id (memory table below) | the 13 IC params, then the kind's delay tail | `[slotBase, pinFlags x L]` | W1d |
| 81, 82 | `28c16`, `28c256` | `vcc`, `gnd`, then the memory logic pins by id | the 13 IC params, the delay tail, then `tWC` and `sdp` | `[slotBase, pinFlags x L, word x W]` (ROM image below) | W1d |
| 90 | `rating` (synthetic) | none | `[device, pMax, rPower]` (Rating record below) | empty | W1g |
| 91 | `thermal` (synthetic) | none | `[device, rThetaJA, tauS, hasShutdown, tripC, restartC]` (Thermal record below) | empty | W1g |

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

**LED and display records** (codes 30-34).
- **Pins.** A pin id the component lacks maps to node 0 with `OPEN` set (Pins
  by id, below): classic's `_pinNode` returns the reference node for an
  absent id (SE:5036-5038) and `_isOpenPin` reports it open (SE:5040-5044).
  `OPEN_HIGH` is never set on codes 30-34 (`IC_OPEN_HIGH_PINS` names no LED
  kind). The pin-count declines are in Pins by id; `seg7` has no pin-count
  gate in classic and none here.
- **Params** (indices in `kinds.rs` `led_param`, `rgb_param`,
  `bicolor_param`, `seg7_param` and `kinds.ts` `CORE_LED_PARAM`,
  `CORE_RGB_LED_PARAM`, `CORE_BICOLOR_LED_PARAM`, `CORE_SEG7_PARAM`). The
  compiler resolves every value with classic's own expression (§6.5),
  `specs` being `specsForComponent(comp)` (SE:1373-1375, the value
  `ctx.electricalSpecs` returns outside the failure pass, SE:1972-1977).
  Each value is the 25 °C catalog-facing number; Rust applies temperature
  (LEDs point 2).

  | Kind | Index: name = compiler expression | Classic |
  |---|---|---|
  | `led` | `0 vf = Number(p.vf ?? specs?.vf ?? 1.8)`; `1 n = Number(p.n ?? 2)`; `2 iRated = Number(p.iRated ?? specs?.if_max ?? 0.02)`; `3 isSet = p.Is !== undefined ? 1 : 0`; `4 is = isSet ? Number(p.Is) : 0`; `5 burnt = p.burnt ? 1 : 0` (JS truthiness: the string `"0"` is burnt) | SC:358, 369-378 |
  | `rgb_led` | `0 vfR = Number(p.vf_r ?? 1.8)`; `1 vfG = Number(p.vf_g ?? 2.0)`; `2 vfB = Number(p.vf_b ?? 3.0)`; `3 iRated = Number(p.iRated ?? 0.02)`; `4 ifMax = Number(p.iRated ?? specs?.if_max ?? 0.02)`; `5 burnt` as `led` | SC:615-628, SC:481 |
  | `bicolor_led` | `0 vf1 = Number(p.vf1 ?? 1.8)`; `1 vf2 = Number(p.vf2 ?? 2.0)`; `2 iRated = Number(p.iRated ?? 0.02)`; `3 ifMax = Number(p.iRated ?? specs?.if_max ?? 0.02)`; `4 burnt` as `led` | SC:548-557, SC:481 |
  | `seg7_cc`, `seg7_ca` | `0 vf = Number(p.vf ?? specs?.vf ?? 2.0)`; `1 iRated = Number(p.iRated ?? specs?.if_max ?? 0.02)` | DI:1208-1210 |

  - Precedence is §6.5's: params over specs over the literal for `led` and
    `seg7`; params over the literal only for the `rgb_led` and
    `bicolor_led` stamps, which never read specs. `n` and the `Is` override
    exist only on `led` (§6.3); the other kinds use `n = 2` and derive `Is`
    from `(Vf, iRated)` always.
  - `ifMax` is the Q-O24 damage limit (SC:481): `params.iRated ??
    specs?.if_max ?? 0.02`. For `led` it is the same expression as param 2,
    so `led` carries no separate `ifMax` and W1g reads param 2. For
    `rgb_led` and `bicolor_led` it differs from param 3 whenever `iRated` is
    absent and the resolved catalog `if_max` is not 0.02, because the stamp
    ignores specs and the failure hook does not. Nothing reads `ifMax` in
    W1e.
  - `seg7` has no `burnt` and no `ifMax`: it ignores `burnt` and has no
    failure surface (Q-O22, DI:1195-1302).
  - The compiler declines `malformed:<componentId>:non-finite-<param>` when
    any resolved value is not finite, and
    `malformed:<componentId>:non-positive-n` when `led` param 1 is `<= 0`
    (classic then divides by `n·Vt = 0` or inverts the junction, and fails
    its solve). The reader repeats both rules (`-1`), plus `isSet` and
    `burnt` in `{0, 1}` and `is = 0` when `isSet = 0`. Any other finite
    value is accepted, `iRated <= 0` included (it gives `Is <= 0`; classic
    stamps it as is). Through W1f a plain `led` with an exact thermal
    identity was declined too (LEDs point 3); W1g lifts that decline (LED
    records: the `led-thermal` decline is lifted).
  - `params.color`, `color1`, `color2` are read by nothing (§6.5).
- **Channel slots.** `kinds.rs` `KindShape.channel_slots`: 3 for `rgb_led`,
  2 for `bicolor_led`, 8 for `seg7_cc` and `seg7_ca`, 0 elsewhere. The
  compiler allocates them in component order through the existing
  `channelSlots` allocator (`compile/index.ts`), consecutively per
  component, and writes the first into the blob. Keys are
  `(componentId, channel)`:

  | Kind | Channel `j` | Classic |
  |---|---|---|
  | `rgb_led` | `0 r_a`, `1 g_a`, `2 b_a` (against `com_k`) | SE:4503-4519 |
  | `bicolor_led` | `0 a1`, `1 a2` (against `k`) | SE:4520-4535 |
  | `seg7_cc`, `seg7_ca` | `0 a` .. `6 g`, `7 dp` | SE:4536-4571 |

  `led` publishes no channel: `getElementChannelI` has no entry for it
  (SE:4495-4590).
- **Digital slots, live params.** None: classic publishes no
  `digitalState` for these kinds (LEDs point 13), and none has a live
  param.

**NE555 record** (code 40).
- **Pins.** By id, in the IC layout: `8` and `1`, the pins
  `supplyPinsForComponent` resolves for a 555 (the catalog's `vcc` and `gnd`
  functions, or its `"8"`/`"1"` fallback without a catalog entry,
  SE:1387-1394), then `2 3 4 5 6 7`. So `IcRecord` types the record
  unchanged, and its logic pins are:

  | Logic index | Pin | Role |
  |---|---|---|
  | 0 | `2` TRIG | read |
  | 1 | `3` OUT | driven (stage `Ne555Out`) |
  | 2 | `4` RST | read |
  | 3 | `5` CV | carried; never read or stamped |
  | 4 | `6` THR | read |
  | 5 | `7` DIS | driven (stage `Ne555Dis`) |

  The 555 also reads pins 8 and 1 (power and the reference). The compiler
  declines the component (`topology:ic-pins:<componentId>`) unless its pin
  ids are exactly `1` to `8` and `supplyPinsForComponent` returns `vcc = "8"`
  and `gnd = "1"`. Classic's power gate reads the catalog's supply pins while
  its stamp hard-codes pins 1 and 8 (DI:184-187); the core does not
  reproduce a catalog in which the two differ.
- **Pin flags.** `OPEN` by G8. Rust reads it on pins 8 and 1 only (the power
  gate); an open TRIG, THR or RST reads its node voltage, 0 V through the
  shunt (Q-O16). `OPEN_HIGH` is never set: `ne555` is not in
  `IC_OPEN_HIGH_PINS`.
- **Params.** The 13 IC params, resolved by the helper the ICs use
  (`icBaseParams`, from `specsForComponent`), then `7 schmitt`, `8 vtPlus`,
  `9 vtMinus`, `11 ioMax` and `12 delay` set to 0. The reader rejects any
  other value there.
  - The kernel reads `0 vccMin` and `1 vccMax`: the power window, 4.5 and 16
    from the catalog, 2.0 and 18.0 without an entry.
  - `2 famFlags` to `6 vih` are carried for W1g's output-sag scan of pin 3,
    which compares it against `thresholdsFor` at `vS` (§5.3). The catalog
    entry has no `logic_family` and no `v_input`, so that is the generic
    formula.
  - `10 nominal` is carried and not read.
  - The zeros record what classic computes and ignores for this kind: an
    `outputResistance` from `io_max` (the OUT stage is a fixed 20 Ω) and a
    package delay (decision 7: no propagation delay). Forcing them to 0
    keeps a later reader from mistaking them for the OUT stage.
- **Blob.** `[slotBase, pinFlags x 6]`, every flag 0; the reader rejects any
  other value. `FLOATING_INPUT` is never set, because classic's 555 stamps no
  floating pull (§5.2). `OPEN_COLLECTOR` is never set, because DIS is its own
  driver stage, not a released-HIGH logic output.
- **Digital slots.** One, at `blob[0]`, keyed by the bare component id
  (classic's `digitalState[id]`, DI:250). No channel slots.
- **Live params.** None.

`kinds.rs`: `KindShape { code: NE555, name: "ne555", pins: 8, pins_max: 8,
params: 13, blob: BlobShape::Exact(7), check: ne555_check, live_params: &[],
digital_slots: 1, channel_slots: 0 }`, where `ne555_check` is `ic_check` plus
the zero rules above. `kinds.ts`: `{ pins: 8, pinsMax: 8, params: 13, blob: {
exact: 7 }, liveParams: [], digitalSlots: 1 }`.

No format version bump and no ABI version change for W1e. A layout for a
reserved code changes no existing record: a v3 reader without it rejected
codes 30-34 and 40 as `-2`.

**Source records** (codes 20-26).
- **Pins.** Codes 21-25 read `pins[0]`/`pins[1]` by position, as classic's
  `stampIndependentVoltageSource` and the bench_psu stamps do. Code 20 reads
  `pos`/`neg` by id (`ctx.pinNode(comp.id, "pos")`, SO:148-149). The declines
  are in Pins by id. `OPEN` and `OPEN_HIGH` are computed as for every pin
  (G8) and never read by Rust for these kinds.
- **Digital slots.** Codes 22-25 own one, at `blob[0]`, keyed by the bare
  component id (classic's `ds[comp.id]`). Codes 20, 21 and 26 own none. No
  source kind has channel slots or live params.

**Battery record** (code 20). The compiler resolves the model with classic's
own code, never re-derived: a helper exported from `sim-engine.ts` that
`_batteryModel` (SE:1857-1899) and `_batteryOperatingPoint` (SE:1901-1915)
also call, bit-identical.

| Index | Name | Compiler expression (classic) |
|---|---|---|
| 0 | `ocvScale` | `nominalVoltageScale`: `configuredVoltage / profile.nominalVoltageV`, or 1 when nominal `<= 0` (SE:1892-1894) |
| 1 | `rBase` | `finiteNonNegative(referenceInternalResistanceOhm, profile.freshInternalResistanceOhm)` (BP:412-415), with `referenceInternalResistanceOhm = configuredResistance` (SE:1871-1874, 1895) |
| 2 | `tempFactor` | `max(Fmin, exp(alpha * (T_ref - T)))` with `T` the load's ambient clamped to the profile's operating range, non-finite giving `T_ref` (BP:420-434). The ambient is `environment.temperatureC ?? 25` (SE:1836-1838), constant per load. |
| 3 | `depA` | `profile.resistance.depletionScale` |
| 4 | `depP` | `profile.resistance.depletionExponent` |
| 5 | `depK` | `profile.resistance.kneeSoc` |
| 6 | `capacityAh` | `max(1e-12, Number(params.capacityAh ?? profile.referenceCapacityAh))`, which also replaces the profile's reference capacity (SE:1883-1889) |
| 7 | `soc0` | `clampSoc(clamp01(Number(params.charge ?? 1)))` (SE:1902, BP:307-309, 439-445): NaN gives 0 |
| 8 | `profileCode` | 0 generic, 1 `alkaline-aa-4s`, 2 `alkaline-pp3-9v`, 3 `lithium-cr2032`. Rust ignores it; TS keeps the id and label for the readout. |

- **Identity** (§8.3.1). `profile` is `BATTERY_PROFILES_BY_CATALOG_UID[
  catalogUid.trim()]` when that exists, else the generic fallback built with
  `fallbackNominalVoltageV = configuredVoltage`, `fallbackCapacityAh =
  Number(params.capacityAh ?? 1)` and `fallbackFreshInternalResistanceOhm =
  configuredResistance ?? 0` (SE:1866-1882, BP:311-382). Catalog defaults
  (`voltage`, `rInternal`) apply only when `catalogUid` is set and resolves
  exactly (`partFor`, SE:1177-1202, read at SE:1866). The compiler never
  infers an identity, and never applies the host-side legacy alias
  (`legacyBatteryPackUid`, §8.3.1 (b)): a `{voltage: 5}` battery is generic
  unless a host persisted `catalogUid: "supply-5v"` (Q-O38). So:
  - identity-less, no `rInternal`: `rBase = 0`, an ideal rail (probe P2);
  - identity-less with `rInternal`: generic shape (A 0.5, K 0.05, T_ref
    25 °C, x2.5 at −20 °C), `rBase = rInternal` (probe Q1);
  - `supply-5v` with `{voltage: 5}`: AA_4S, `ocvScale = 5/6`, fresh OCV
    5.3333 V, `rBase = 0.9` from the catalog, 0.8041 Ω at 25 °C and 0.9 Ω at
    21 °C (probe P3; Q-O33's 21 °C reference, mirrored).
- **Blob: the OCV table.** `[n, soc_0, v_0, ..., soc_{n-1}, v_{n-1}]`, the
  profile's `ocvCurve` UNSCALED (volts before `ocvScale`), `n >= 1`. Generic
  is `[2, 0, 0, 1, nominal]`; AA_4S and PP3 are the alkaline cell curve x4
  and x6 (8 points, BP:147-158, 192, 225); CR2032 is 9 points (BP:259-269).
  Rust interpolates exactly as `batteryOpenCircuitVoltageV` (BP:384-403):
  `soc <= soc_0` gives `v_0`; the first point with `soc <= soc_i` bounds the
  segment; a zero-width segment gives `v_i`; past the end gives `v_{n-1}`.
- **Validation** (`kinds.rs`, `-1`): every value finite; `ocvScale >= 0`;
  `rBase >= 0`; `tempFactor > 0`; `depA >= 0`, `depP >= 0`, `depK > 0`; `capacityAh >=
  1e-12`; `soc0` in `[0, 1]`; `profileCode` in `{0, 1, 2, 3}`; `n` a
  positive integer with blob length `1 + 2n`, every `soc_i` in `[0, 1]` and
  non-decreasing. The compiler declines
  `malformed:<componentId>:non-finite-<param>` for a non-finite resolved
  value (a non-numeric `voltage` makes classic's OCV NaN), and
  `malformed:<componentId>:non-finite-ocvTable` for a non-finite table
  entry (no shipped profile has one). `depP` has no decline: every profile's
  is a positive constant.
- A battery param edit is a reload, as in classic; SoC carry-forward across a
  reload is W2 (§8.3.4, D25), so until then the core restarts at `soc0`.

**Bench PSU record** (code 21).
- **Params.** `0 V = Number(params.voltage ?? 5)`, `1 iLimit =
  Number(params.iLimit ?? 0)` (SO:218-219, 294). No catalog default is ever
  read: a `supply-var` part with no `iLimit` is an ideal source (probe P6;
  Q-W18). A non-finite `V` declines `non-finite-voltage`. An `iLimit` that is
  not `> 0` (negative, zero, NaN) is emitted as 0, classic's ideal path
  (SO:274-283, `iLimitPsu > 0` false); `+Infinity` declines
  `non-finite-iLimit`.
- **Two forms, decided at load.** `iLimit = 0`: an ideal source, handled
  exactly as code 2 (W0 points 1 and 5), with no device state. `iLimit > 0`:
  a hub candidate (Rails point 1), with device state `reg` (C ABI, device
  state).
- **Validation** (`kinds.rs`, `-1`): `V` finite; `iLimit` finite and `>=
  0` (the compiler never emits a negative one).
- Regime carry-forward across a reload is W2 (Q-O37).

**Waveform source records** (codes 22-25).
- **Clock** (SO:360-364, SE:6188-6197): `0 freq = max(1e-6,
  Number(frequency ?? 1000))`, `1 duty = Number(duty ?? 0.5)`, `2 vHigh =
  Number(v_high ?? 5)`, `3 vLow = Number(v_low ?? 0)`, `4 delay =
  Number(delay ?? 0)`. The period is `1 / freq`, computed in Rust exactly as
  classic passes it to `clockVoltage`.
- **Pulse** (SO:386-392, SE:6200-6211): `0 v1 = Number(v1 ?? 0)`, `1 v2 =
  Number(v2 ?? 5)`, `2 td = Number(td ?? 0)`, `3 tr = Number(tr ?? 1e-6)`,
  `4 tf = Number(tf ?? 1e-6)`, `5 pw = Number(pw ?? 5e-4)`, `6 per =
  Number(per ?? 1e-3)`. A `per` that is not finite and positive is one-shot
  (WF:51); the compiler emits it as 0.
- A battery whose resolved profile is none of the four `profileCode` names
  declines `unsupported-feature:battery-profile:<profileId>`: Rust carries
  the table, but the readout's id and label could not round-trip.
- Params come from `comp.params` only; no catalog default is read (§8.8).
- The compiler declines `non-finite-<param>` for any other non-finite value,
  `negative-<param>` for `tr`, `tf` or `pw < 0`, and `truncated-pulse` when
  `per > 0` and `per < (tr + pw) + tf`, summed left to right as classic's
  `tr + pw + tf` (a cycle cut short, WF:52-56; Sources point 3). The reader
  repeats these rules (`-1`), and also requires `freq >= 1e-6` for a clock
  and `per >= 0` for a pulse (the compiler's own output).

**Section anchor record** (code 26, synthetic). The `bd90847` port (spec,
"Deltas since the 2026-09-24 baseline"; W1f Q8). The compiler computes
classic's isolated sections exactly as `_compileIsolatedSectionAnchors`
(SE:4690-4735): a union over each component's pins (no W1 kind declares
`galvanicGroups`, so every component is one group), with the reference net
as ground. For each section that does not contain the reference net, it
emits one record whose single pin is the anchor: the return pin
(`sourceReturnPinId`, GR:56-59: `neg`, else `gnd`) of the first
`SOURCE_KINDS` component in circuit order with that pin in the section, else
the section's lowest net row. `componentIndex` is the index of the component
owning that pin (the first one in circuit order for the lowest-row
fallback). Records are emitted after every component record, in ascending
anchor row, the order classic sorts its anchors in. Rust fixes the anchor
node at 0 V (Rails point 1), where classic ties it to ground through 1 S
(`ISOLATED_SECTION_ANCHOR_G`), so the core reads 0 V where classic reads
about 1e-11 V (probe Q4). No readout is published for the record: its entry
in `bo_engine_device_currents` is the anchor's source-KCL current (W0 point
5, the section's shunt draw, about 0 A), and the facade gives it no
`elementI` key. An ownerless net, one none of whose pins belongs to a
component of the circuit (only wires to absent components), declines
`malformed:<componentId>:unknown-component` for each absent id: the anchor
needs an owning pin, the compiler never throws on a user circuit (one
that meets the `SimCircuit` types: a non-string `catalogUid` throws in
classic's `load` too), and
classic loads such a circuit (it anchors the ghost net as one more
section). A ghost pin on a net that also has a real pin still compiles, as
before W1f, and classic anchors nothing extra for it. Before W1f an
ownerless net compiled with no anchor; it now stays with classic.

`kinds.rs` (`BATTERY_PACK` to `PULSE_GEN` exist; `SECTION_ANCHOR = 26` is
new), with `live_params: &[]` and `channel_slots: 0` throughout:
- `battery_pack`: `pins: 2, pins_max: 2, params: 9, blob: BlobShape::Table,
  check: battery_check, digital_slots: 0`;
- `bench_psu`: `pins: 2, pins_max: 2, params: 2, blob: Exact(0),
  digital_slots: 0`;
- `clock`, `clock_gen`: `pins: 2, pins_max: 2, params: 5, blob: Exact(1),
  digital_slots: 1`;
- `pulse_source`, `pulse_gen`: `pins: 2, pins_max: 2, params: 7, blob:
  Exact(1), digital_slots: 1`;
- `section_anchor`: `pins: 1, pins_max: 1, params: 0, blob: Exact(0),
  digital_slots: 0`.

`BlobShape::Table` is new: `blob[0] = n`, length `1 + 2n`. The reader checks
the shape; `battery_check` checks the values. `kinds.ts` mirrors every shape.
No format version bump and no ABI version change for W1f: a v3 reader
rejected codes 20-26 as `-2`.

**Fuse and PTC records** (codes 10 and 13; W1g Q3). Both are two-terminal
and read `pins[0]` and `pins[1]` by position, as `fuseModel` and
`ptcFuseModel` do. Params come from `comp.params` only, with classic's own
literals (the catalog's `fuse` and `ptc-fuse` defaults are the same numbers,
§9.8):
- `fuse`: `0 rNormal = max(0.001, Number(params.rNormal ?? 0.1))` (PA:694),
  `1 iRating = Number(params.iRating ?? 1)`. The `?? 1` is the catalog
  default and the 0.6.1 fix (`fix/resistor-default-overload`); classic read
  `?? 0` at `2b5faf1` (PA:713), so a fuse with no `iRating` never tripped.
  An `iRating <= 0` is emitted as it is: it means no limit, as classic's
  current ratio is 0 then (PA:715).
- `ptc_fuse`: `0 rNormal = max(0.001, Number(params.rNormal ?? 0.5))`
  (PA:755), `1 iHold = max(1e-6, Number(params.iHold ?? 0.5))` (PA:774).
- The compiler declines `malformed:<componentId>:too-few-pins` with fewer
  than 2 pins, and `malformed:<componentId>:non-finite-<param>` for a
  non-finite resolved value. The reader repeats finiteness (`-1`) and
  requires `rNormal >= 0.001`, and for a PTC `iHold >= 1e-6`.
- The stamps (W1g point 12): an intact fuse is `1/rNormal` between its pins,
  and a failed one stamps nothing (PA:697-699). A PTC is `1/rNormal`, or
  `1/(rNormal * 200)` while tripped, with the product taken first as classic
  stamps it (PA:759).
- No digital or channel slots, no live params, and no thermal identity.

**Rating record** (code 90, synthetic). One per `resistor` record, emitted
by `src/core/compile/thermal.ts`. W1h's `potentiometer`, `trimmer` and
`resistor_array` carry `p_max` in their own records.
- `device`: the resistor's record index. `componentIndex` is the resistor's
  component index.
- `pMax = specs?.p_max` when it is a finite number, else `0`, with `specs`
  from `specsForComponent(comp)`: the value the failure pass reads through
  `ctx.electricalSpecs` (PA:101-102, SE:1960-1961 and 6281-6285). A
  `pMax <= 0` means no limit, as classic's `powerRatio` is 0 when
  `localLimit` is absent or not `> 0` (PA:138).
- `rPower = Number(params.resistance ?? 1000)`: the resistance classic
  multiplies the squared current by (PA:107), unfloored. The `?? 1000` is
  the 0.6.1 fix (`fix/resistor-default-overload`, Q-O27). At `2b5faf1`
  classic read `?? 0`, so a resistor with no `resistance` param never
  overloaded (probe F `resistorNoParam`: 50 V across the 1 kΩ default,
  2.5 W, no latch in 2 s). It is finite whenever the resistor compiles
  (its own `non-finite-resistance` decline).
- The power is `P = I² · rPower` with `I = (V(pins[0]) − V(pins[1])) / R`,
  `R` the resistor record's stamped value, and `0` when `rPower <= 0`
  (PA:106-109; W1g point 6).
- Every resistor gets one, identity or not: classic's failure hook does not
  depend on thermal identity (§9.3.1, Q-O47).

**Thermal record** (code 91, synthetic). One per device with an exact
thermal identity: the compiler emits it iff `thermalProfileForComponent(comp)`
(SE:1404) is defined, which takes an explicit `catalogUid`, a kind match and
an exact-catalog source (§9.2.3). An identity-less part fails and warns but
never heats (Q-O47).
- In W1 the kinds are `resistor` (uid `resistor`) and plain `led` (any LED
  uid, §9.3.1). A thermal identity on any other kind declines the circuit
  with `unsupported-feature:thermal:<componentId>`. None is reachable in W1:
  the other profiles are W3's regulators and op-amps.
- `rThetaJA = profile.network.rThetaJA_CPerW` and `tauS =
  thermalTimeConstantS(profile.network)` (TP:452-459): 340 and 8 for the
  resistor, 560 and 2 for the LED (TP:285-324).
- `hasShutdown` is 1 when `profile.shutdown` is set, and then `tripC` and
  `restartC` carry its `tripTemperatureC` and `restartTemperatureC`.
  Otherwise all three are 0. No W1 profile has a shutdown.
- The profile id, label, derating limits and static warnings stay in TS
  (Readouts). Rust needs only the one-pole and the shutdown.

**Reader rules for codes 90 and 91.** They come after every component
record and after W1f's code 26 anchors, in ascending `device`, 90 before 91
for one device. `device` names an earlier non-synthetic record: a `resistor`
for 90, a `resistor` or `led` for 91. At most one of each code per device,
and exactly one 90 per resistor. Every value is finite, with `rThetaJA > 0`,
`tauS > 0`, `hasShutdown` in `{0, 1}`, `tripC = restartC = 0` when
`hasShutdown = 0`, and `restartC < tripC` otherwise. `kinds.rs` checks the
shapes and values; `program.rs` checks the order and the `device`
references, which span records. A synthetic record publishes nothing: its
`bo_engine_device_currents` entry is 0, and the facade gives it no
`elementI` key.

**LED records: the `led-thermal` decline is lifted.** LEDs point 3's decline
(`unsupported-feature:led-thermal:<componentId>`, `src/core/compile/leds.ts`)
is removed. A plain `led` with an exact identity compiles like any other,
plus a code 91 record. The damage limit needs no new param: `led` reads
param 2 (`iRated`, the Q-O24 expression), and `rgb_led` and `bicolor_led`
read their `ifMax` param (LEDs point 15).

**IC records: the output-sag scan.** Classic scans every component whose
specs carry `vcc_range` (SE:6296). Every IC kind that reaches the core has a
catalog entry (a supply-less IC declines, `topology:ic-pins`), and every W1
and W1h IC entry in `default-parts.json` carries `vcc_range` (all 23 kinds
checked). So every IC record is scanned, and no param changes. The compiler
declines an IC whose resolved specs lack `vcc_range`
(`unsupported-feature:no-vcc-range:<componentId>`); that is unreachable
today and keeps the rule true. The thresholds come from IC params 2-6 at the
live `vS` (W1c point 1). The NE555 record carries them for exactly this
(NE555 record, "`2 famFlags` to `6 vih`").

`kinds.rs` (`FUSE = 10` exists; `PTC_FUSE = 13`, `RATING = 90` and
`THERMAL = 91` are new), with `live_params: &[]`, `digital_slots: 0` and
`channel_slots: 0` throughout:
- `fuse`, `ptc_fuse`: `pins: 2, pins_max: 2, params: 2, blob: Exact(0)`,
  with `fuse_check` and `ptc_fuse_check`;
- `rating`: `pins: 0, pins_max: 0, params: 3, blob: Exact(0)`;
- `thermal`: `pins: 0, pins_max: 0, params: 6, blob: Exact(0)`, with
  `thermal_check`.

`kinds.ts` mirrors every shape, and `CORE_KIND_CODES` gains `ptc_fuse: 13`.
No format version bump and no ABI version change for W1g: a v3 reader
rejected codes 13, 90 and 91 as `-2`, and code 10 had no shape.

**Pins by id.** A pin id the component lacks maps to node 0, because classic
maps an absent pin to its reference node (spec §7.2.7). The exceptions are
kinds classic stamps nothing for when pins are missing, which the compiler
declines (`malformed:<componentId>:too-few-pins`, the circuit runs on
classic) rather than emit a stamp classic never makes:
- an `spdt_switch` with fewer than 3 declared pins (`if (pins.length < 3)
  return`);
- a `led` with fewer than 2, a `bicolor_led` with fewer than 3 and an
  `rgb_led` with fewer than 4 (SC:355, SC:547, SC:614). A 1-pin `led` also
  has no `elementI` key (SE:6233), but a 2-pin `bicolor_led` or a 2- or
  3-pin `rgb_led` still publishes `elementI` and channel currents computed
  from the absent pins' reference node (SC:563-584, SE:4503-4535), a split
  the core does not reproduce;
- a source of codes 20-25 with fewer than 2 pins: classic stamps nothing but
  still allocates an all-zero branch row (§8.1.1, Q-W17);
- a `fuse` or `ptc_fuse` with fewer than 2 pins (W1g; classic reads
  `pins[1].id` unguarded, PA:696, 761).

One decline is not of this kind: a `battery_pack` whose `pos` or `neg` is
absent declines `malformed:<componentId>:missing-pin`, because classic maps
the absent pin to its reference node and stamps a shorted battery.

**Currents** (`elementI`, one per record):
- `capacitor`: series plus leakage current, `pins[0]` to `pins[1]`.
- `switch`, `push_button`: the contact current `pins[0]` to `pins[1]`, 0 when
  open.
- `spdt_switch`: the current through the contact in use, `com` to the
  selected throw. This is the Q-O26 fix, landed in classic first; classic
  used to report `com` to `b` at every position.
- `dip_switch`: the sum of the closed positions' contact currents, `a_i` to
  `b_i`.
- The LED kinds, from the published voltages with the junction law of LEDs
  point 2, GMIN excluded (§6.1). These are Q-O23's aggregation conventions,
  unchanged (D31):
  - `led`: the signed current `I(V(pins[0]) - V(pins[1]))`, anode to
    cathode (SC:463-464). It is at least `-Is`. `0` when `burnt`.
  - `bicolor_led`: `max(I1, I2)` of the unclamped channel currents
    (SC:582); it can go slightly negative. `0` when `burnt`.
  - `rgb_led`: `max(0, Ir, Ig, Ib)`, the running maximum starting at 0
    (SC:647-658). `0` when `burnt`.
  - `seg7_cc`, `seg7_ca`: the sum, in channel order `a..dp`, of
    `max(0, I_seg)` over the segments that have a junction (LEDs point 1);
    `0` when no common pin is stamped (DI:1247-1268).
- The W1c and W1d ICs: `0`, as classic (no current model).
- `ne555`: the current delivered out of pin 3 to its load,
  `-(gLo (V3 - V1) + gHi (V3 - V8))` with `gHi = f/20`, `gLo = (1-f)/20` and
  `f` the fraction of the committed OUT level (NE555 point 7); 0 while
  unpowered (§5.2 "Element current"). The DIS current is not reported.
- `battery_pack`, `bench_psu`, `clock`, `clock_gen`, `pulse_source`,
  `pulse_gen`: classic's branch value `x[k]`, the current entering the
  source at `pins[0]`/`pos` from the circuit, negative while it delivers
  (SO:74-82, §8.1.1). It includes the node shunts the source feeds (probe
  P2: `−0.500000000005` A into 10 Ω at 5 V). A bench_psu in CC publishes
  exactly `−dir·iLimit` (SO:258-260; probe P5).
- `section_anchor`: no `elementI` key (Section anchor record).

W1g adds (W1g point 12):
- `resistor`: `0` once `resistor_overload` has latched (PA:191-194).
- `fuse`: `(V(pins[0]) − V(pins[1])) / rNormal`, and `0` once
  `fuse_tripped` has latched (PA:701-710).
- `ptc_fuse`: `(V(pins[0]) − V(pins[1])) / R`, with `R = rNormal` or
  `rNormal * 200` while tripped (PA:764-772).
- `led`, `rgb_led`, `bicolor_led`: `0` once `led_failed` has latched, as
  when `burnt`.
- `rating`, `thermal`: no `elementI` key (Rating and Thermal records).

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
| `bo_engine_stat` | `(h, i: u32) -> f64` | Counters (below). Unknown `i` gives `NaN`. Index 11 counts static iterations too from W1e (below). | v1 |
| `bo_error_ptr` / `bo_error_len` | `() -> *const u8` / `() -> u32` | UTF-8 message for the most recent non-zero return. | v1 |
| `bo_engine_set_option` | `(h, key: u32, value: f64) -> i32` | Change an option after load. Key 2 takes effect at the next event; keys 1 and 3-7 are load-time only (`-8` after load). `FORCE_STEPPED` and `NO_RAIL_SHORTCUT` are load-time only because each decides partition classification or membership, which is fixed at load. | v2 (W1a); key 3 load-time only from W1f |
| `bo_engine_next_event_time` | `(h) -> f64` | Time of the earliest pending event, `+Infinity` when none. | v2 (W1a) |
| `bo_engine_next_digital_due_time` | `(h) -> f64` | Time of the earliest pending driver commit (a live token; a commit held by the power-loss hold is not pending), `+Infinity` when none. Classic's `nextScheduledEventTime()` counterpart for a host that clamps its steps onto digital due times; it ignores stepped, crossing and bucket events. | v2 (W1c) |
| `bo_engine_digital_slot_count` / `bo_engine_digital_slots` | `(h) -> u32` / `(h) -> *const f64` | `S` values; `NaN` means the key is absent this pass. | v2 (W1a) |
| `bo_engine_channel_slot_count` / `bo_engine_channel_slots` | `(h) -> u32` / `(h) -> *const f64` | `C` per-channel currents (amps). All of a component's slots are `NaN` while its `getElementChannelI` entry is absent (a burnt `rgb_led` or `bicolor_led`; `led_failed` from W1g). | v2 (W1a); `NaN` rule W1e |
| `bo_engine_trace_enable` | `(h, on: u32) -> i32` | Record every digital slot change as `(t, slot, value)`. Harness instrumentation only; W2's host traces are separate. | v2 (W1a) |
| `bo_engine_trace_ptr` / `bo_engine_trace_len` / `bo_engine_trace_clear` | `(h) -> *const f64` / `(h) -> u32` (f64 count, a multiple of 3) / `(h)` | Read and reset the trace. | v2 (W1a) |
| `bo_engine_set_params` | `(h, ptr: *const f64, len_f64: u32) -> i32` | `(deviceIndex, paramIndex, value)` triples for each kind's live params (owner decision 1), applied as one event at the current time with all other state kept. | v2 (W1b) |
| `bo_engine_seed_memory` | `(h, device: u32, ptr: *const f64, len_f64: u32) -> i32` | Replace a 74ls189's 16 stored nibbles (owner decision 2). | v2 (W1d) |
| `bo_engine_take_eeprom_updates` | `(h) -> i32` | Collect and clear every dirty EEPROM image; returns the update buffer's length in f64 (`>= 0`), `-5` with no program. | v2 (W1d) |
| `bo_engine_eeprom_updates_ptr` | `(h) -> *const f64` | The buffer from the last take, valid until the next call on that engine. | v2 (W1d) |
| `bo_engine_device_state` | `(h, device: u32) -> i32` | Collect one device's runtime state into the engine's state buffer; returns its length in f64 (`>= 0`, the per-kind layout below). `-8` for an index out of range or a kind with no device state, `-5` with no program. | v2 (W1f) |
| `bo_engine_device_state_ptr` | `(h) -> *const f64` | The buffer from the last collect, valid until the next call on that engine. | v2 (W1f) |
| `bo_engine_reset_failures` | `(h) -> i32` | Classic's `resetFailures()`, applied as ONE event at the current time (W1g point 13). `0`; `-5` with no program; `-4` or `-7` when the re-solve at that instant fails, and then the reset rolls back and the engine is exactly as it was. | v2 (W1g) |
| `bo_engine_records` | `(h, channel: u32) -> i32` | Collect one channel's records into the engine's record buffer; returns its length in f64, a multiple of 7. Channel `0` failures, `1` warnings (empty in W1), `2` tripped PTCs, `3` the latch log. `-8` for any other channel, `-5` with no program. Channels 0-2 are read-only: they project to the current time and commit nothing. Collecting channel 3 empties it (below). | v2 (W1g) |
| `bo_engine_records_ptr` | `(h) -> *const f64` | The buffer from the last collect, valid until the next call on that engine. | v2 (W1g) |

**`NO_RAIL_SHORTCUT` is load-time only from W1f** (W1f Q10). It was live
through W1e. Exact mode makes the rail node free and merges every partition
that reads it (Rails point 9), and partition membership changes only at a
load. So key 3 after load is `-8`, as keys 1 and 4-7 are. This is a
behaviour change of a public export, part of 1.0.0. No TS caller sets it
after load (the S1 script and the compiler pass it in the program); the two
Rust tests that did (`abi.rs:796`, `engine.rs:955`) change with it.

**`bo_engine_device_state`** (W1f Q11). One generic pair, so that W3's
regulators and later stateful kinds reuse it; it replaces the plan's
`bo_engine_battery_state` and separate bench_psu `reg` read. Layouts, per
kind:

| Kind | Length | Values, in order |
|---|---|---|
| `battery_pack` | 6 | `0 soc`, `1 openCircuitVoltageV` (scaled: `OCV(soc) * ocvScale`), `2 internalResistanceOhm` (`R(soc)`, unscaled), `3 remainingCapacityCoulombs` (`capacityAh * 3600 * soc`), `4 dischargedCoulombs`, `5 rejectedRechargeCoulombs` (SE:4312-4331) |
| `bench_psu`, `iLimit > 0` | 1 | `0 reg`: 0 (CV) or 2 (CC) |
| `bench_psu`, `iLimit = 0` | 0 | none |
| `resistor`, `led` with a code 91 record | 3 | `0 temperatureC`, `1 dissipatedPowerW`, `2 thermalShutdown` (0 or 1) (W1g point 3) |
| `resistor`, `led` without one | 0 | none |
| `ptc_fuse` | 3 | `0 tripped` (0 or 1), `1 tripStress`, `2 recoveryTime` (W1g point 10) |

- Every battery value is projected to the current time without committing
  it (Rails point 6), so a collect never changes the simulation. So is
  every W1g value (W1g point 16).
- **Extending a layout.** The returned length is the layout's version. A
  later wave extends a kind's layout only by appending values, never by
  reordering, removing or reusing an index, and records the new length in
  this table. The TS wrapper checks the returned length against the length
  it expects for the kind (a mismatch throws, as an ABI version mismatch
  does) and exposes typed accessors that name each value, so no caller
  indexes raw offsets.
- The buffer is valid until the next call on that engine (Memory, below).

**`bo_engine_records`** (W1g). One record is 7 f64: `device, code, pin,
since, value, limit, aux`.
- `device` is the device record index. `pin` is the pin's index in that
  device's pin records (for an IC, `vcc` and `gnd` are 0 and 1), or `-1` for
  none (classic's `pinId` absent).
- **Failure codes** (channels 0 and 3): `1 resistor_overload`,
  `2 fuse_tripped`, `3 led_failed`, `4 output_sag`. **Warning codes**
  (channel 1): `1 transistor_overcurrent` (W3), `2 missing_flyback`
  (reserved: the facade derives it, W1g point 11). **PTC** (channel 2):
  `1 ptc_tripped`, with `since` the trip time and `value = limit = aux = 0`.
  A PTC trip is never a failure or a warning (Q-O45).
- `aux` is the `output_sag` direction (`1` HIGH, `0` LOW), and `0`
  otherwise. Every W1 failure record has a `value` and a `limit` (classic
  sets both for all four kinds).
- **Order.** Records come in latch order, the order classic's `failures`
  map iterates. A latched `output_sag` keeps its place while it stays
  latched (classic re-`set`s an existing key, SE:6325). A clear or an
  unpublished direction flip followed by a re-latch moves it to the end, as
  classic's delete and set do (W1g point 9).
- **Channel 3, the latch log** (harness instrumentation). Every failure key
  `(device, code, pin)` that has latched at least once since the previous
  collect of channel 3, or since load, appears once, in the order of its
  first latch in that span, with that latch's `since` (the latch time, for
  `output_sag` too), `value`, `limit` and `aux`. Collecting it empties it,
  as `bo_engine_take_eeprom_updates` does. It holds at most one record per
  key, so a host that never collects it costs a bounded amount. It rolls
  back with a failed instant; `bo_engine_reset_failures` leaves it alone,
  and a load empties it. It exists so that the harness sees every latch over
  a run, not a poll (Equivalence harness rules, W1g).

**`bo_engine_reset_failures`** (W1g, W1g point 13). No argument. It applies
as one event at the current time, like `bo_engine_set_params`: it does not
advance time, and there is no 1 ps seed. A failed re-solve (`-4`, `-7`)
rolls the whole reset back, which is a recorded divergence from classic
(W1g Q6; Equivalence harness rules, W1g). `-5` before a successful load.

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
capacitors (decision 14's count; `FORCE_STEPPED` does not change it), `16`
rail sensitivity computations (pec3's `railSens`), `17` rail scans (pec3's
`railScans`), `18` rail events processed, `19` source edges processed, `20`
reader flips found at a rail scan, `21` slow interval closes, `22` slow
events fired, `23` latches, `24` junction re-arms. Stat 11 counts every Newton iteration of
a stepped step (one for a linear partition) and, from W1e, every iteration
of an iterated static partition:
LEDs point 6, the polish included, and W1c's static fixed point, which W1c
did not count. Stat 14 was reserved from W1a and counts from W1f (Rails
point 3); stats 16-20 are W1f's (Rails points 3 and 4, Sources points 2 and
3, and for 20 the reader-scan divergence under Equivalence harness rules).
Stats 16-18 and 20 stay 0 in exact mode. Stats 21-24 are W1g's (W1g point
18).

**Error codes:** `-1` malformed program; `-2` unknown kind code; `-3`
unsupported topology, which from W1f includes the hub classification cases
of Rails point 1 (the TS side falls back to classic; so do `-4` and `-7`
returned by `bo_engine_load`, because owner decision 8 sends a failed t=0
settle to classic); `-4` solve failed
(singular, non-finite pivot, a non-finite published value, or a partition's
Newton iteration that does not converge: a static partition within 50
iterations, a stepped partition at the `H_MIN` floor); `-5` unknown
handle or no program loaded; `-6` `t_end` not finite or earlier than the
current time; `-7` delta-cycle livelock (the per-instant pass cap was
exceeded; the host treats it as non-convergence); `-8` invalid argument (an
unknown option key, a value out of range, a load-time option after load, a
device or param index out of range, a param that is not live, a device
state read of a kind with none, or an unknown record channel). W1f and W1g
add no code.

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
- `getElementChannelI()`: from the channel slots. A component whose slots
  are `NaN` is omitted, as classic omits a burnt or failed `rgb_led` or
  `bicolor_led` (SE:4503, 4520); `buildElementChannelI`
  (`src/core/readout.ts`) skips it rather than write the `NaN` through.

W1f adds:
- `getNetV()`: the rail node publishes `u`; nodes of hub partitions publish
  their rail-corrected voltage (Rails point 3).
- `digitalState`: a clock or pulse slot holds 1 while the source's value is
  `> (vHigh + vLow) / 2` (clock) or `> (v1 + v2) / 2` (pulse), else 0,
  strictly greater (SE:6195, 6209). The slot is never `NaN`: classic
  publishes the key on every pass.
- `getBatteryState(id)`: `undefined` unless `id` is a `battery_pack` record;
  otherwise classic's `BatteryRuntimeReadout` (SE:540-551): the six numbers
  from `bo_engine_device_state`, plus `profileId` and `label` from the
  compiler's profile (the generic one is `"generic-primary"` and `"Generic
  primary battery (fallback)"`, probe P2), and `modelTemperatureC` and
  `temperatureWasClamped` computed by the compiler from the ambient and the
  profile's range (BP:492-510; constant per load).
- `getIcState(id)`: for a `bench_psu` with `iLimit > 0`, a fresh `{ reg }`
  object (classic returns its live `icState` object, SE:4466-4468; a caller
  mutating it is not supported). `undefined` for a bench_psu with `iLimit =
  0`, as classic creates no entry (SE:2854-2855). For every other kind the
  facade returns `undefined` in W1f; classic's IC `icState` objects are not
  mirrored (W1f Q11; whether to mirror them is W2's call, and no host reads
  them).
- Both read device state through the typed accessors (C ABI,
  `bo_engine_device_state`), never raw offsets. Neither method is on
  `EngineReadout` (`src/core/surface.ts`); both live on `CoreEngine` for
  gate 1 (`battery-engine-physics`, `wave5-regulators`). No devolt host calls
  either (only comments in `packages/simcore/src/canvas/renderer.ts`).

W1g adds (records from `bo_engine_records`; the pure decoding, keys and
message builders live in `src/core/failures.ts`, which the facade calls):
- **`getFailures()`**: a fresh object in record order (channel 0), keyed by
  classic's `failureKey` (SE:928-945): `${kind}\0${componentId}\0${pinId}`,
  with `pinId` `""` when absent. Each record becomes `{componentId, kind,
  since, value, limit, message}` in classic's key order, with `pinId` after
  `kind` on `output_sag` only (classic sets it only there, SE:6328). The
  compiler keeps each record's pin ids in pin-record order, so the facade
  names `pin`. Through W1f this was always `{}`.
- **Messages** come from builders that copy classic's template literals
  character for character (G10), applied to the record's own `value` and
  `limit`:
  - `resistor_overload`: `` `${id} dissipated ${value.toFixed(3)} W through a ${limit.toFixed(3)} W rating long enough to fail open.` `` (PA:155; the pot and trimmer segment form, PA:156, and `resistor_array`'s per-channel one, PA:949, are W1h's);
  - `fuse_tripped`: `` `${id} carried ${(value * 1000).toFixed(1)} mA through a ${(limit * 1000).toFixed(1)} mA fuse rating and tripped open.` `` (PA:731);
  - `led_failed`: `` `${id} exceeded its ${(limit * 1000).toFixed(1)} mA LED rating with ${(value * 1000).toFixed(1)} mA long enough to fail open.` `` (SC:508);
  - `output_sag`: the SE:6333 template, `HIGH` or `LOW` from `aux`.
- **`getWarnings()`, `takeNewWarnings()`**: the facade's static warnings
  (W1g point 11) plus channel 1 (empty in W1), keyed by classic's
  `warningKey`. The announce queue lives in TS and works as classic's
  (SE:2372-2381, 5536-5544): a warning is queued once when it latches, a
  still-true warning keeps its `since` and is not queued again, and a key
  that is gone is pruned.
- **`getPtcTripped()`**: a fresh `Set` of the component ids in channel 2.
- **`getThermalState(id)`**: `undefined` unless the component has a code 91
  record. Otherwise the facade reads `[T, P, shutdown]` from
  `bo_engine_device_state` and calls classic's own pure
  `stepThermalDevice(profile, {temperatureC: T, thermalShutdown},
  {ambientTemperatureC: AMBIENT_C, dissipatedPowerW: P, dtSeconds: 0})`
  (TP:506-572). With `dt = 0` it returns `T` unchanged,
  `targetTemperatureC = Ta + P·Rθ`, the derating, the per-state warnings and
  `withinContinuousLimits`, exactly as classic computes them. The readout is
  `{temperatureC: T, thermalShutdown (Rust's flag), catalogUid, profileId,
  dissipatedPowerW: P, targetTemperatureC, allowedPowerW,
  withinContinuousLimits, warnings: [...new Set([...profile.warnings,
  ...stepped.warnings])], label}` (SE:6045-6053, 2398-2405), so every string
  is classic's by construction. At load `T = Ta`, and the warnings include
  the profile's, as classic's cold start does since 0.6.1 even when its
  seed solve is rejected (Q-O42, `fix/cold-start-thermal-warnings`).
  Q-O52's stale-profile case cannot arise in W1: nothing carries across a
  load (W1g point 15).
- **`getPartTemperature(id)`**: `T` from the device state, else `undefined`
  (the thermistor's body temperature is W1h's).
- **`resetFailures()`**: calls `bo_engine_reset_failures`. On `0` it clears
  the announce queue, re-derives the static warnings with `since` = the
  current time and queues them again (SE:2407-2448). On `-4` or `-7` it sets
  `lastConverged = false` and changes nothing else (W1g point 13).
- **`takeLatchedFailures()`**: channel 3 as `SimFailure` objects, each with
  its latch's `since`. Harness instrumentation, classic has no counterpart,
  and it is NOT public API: marked `@internal` in the declarations, left
  out of the 1.0 public API surface and of the changeset (manager ruling).
- `getElementI()`: a failed `resistor`, `fuse` or LED-family device reads `0`
  (Currents). `getElementChannelI()` omits a failed `rgb_led` or
  `bicolor_led`, whose channel slots are `NaN` (LEDs point 13).
- `getFailures`, `getWarnings` and `takeNewWarnings` are already on
  `EngineReadout`. `getPtcTripped`, `getThermalState`, `getPartTemperature`,
  `resetFailures` and `takeLatchedFailures` live on `CoreEngine` only (W1g
  Q5), for gate 1, as W1f's methods do; the worker path is W2.

## Kernel semantics: the static solve (W0)

1. **Fixed nodes.** Node 0 is fixed at 0 V. A `voltage_source` with one
   terminal fixed fixes the other: `V(+) = V(−) + V`. Propagate until nothing
   changes, recording each source as the parent of the node it fixed. A source
   whose terminals are both already fixed (a loop of ideal sources, including
   a source shorted to itself) is `-3`, whether or not the voltages agree —
   the classic MNA is singular there too. A source with neither terminal
   reachable is `-3` (floating ideal source). From W1f every
   `section_anchor` node is fixed at 0 V before propagation starts, so a
   source in an isolated section propagates from its anchor (Rails point 1).
2. **Node shunt.** Every node except node 0 carries `NODE_RSHUNT_G = 1e-12` S
   to the datum, exactly as the classic engine stamps it on every node row.
   It keeps an isolated net solvable (it reads 0 V) and it is included in
   every KCL sum below.
3. **Partitions.** Union-find over free nodes, joined by every resistor whose
   two terminals are both free (W1b widens the union; see "Kernel semantics:
   partitions and dynamic solves (W1b)"; W1e adds the anode and cathode of
   every stamped LED junction whose two nodes are both free, LEDs point 4).
   A resistor with one fixed end stamps
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
     published current leaving `u`, plus each stamped LED junction's branch
     current `I(vA - vK) + GMIN_D * (vA - vK)` leaving its anode and entering
     its cathode (LEDs point 12), plus the shunt current `1e-12 * V[u]`;
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
     side is a rail);
   - the anode and cathode of every stamped LED junction whose two nodes are
     both free (W1e, LEDs point 4).

   So every device's free pins share exactly one partition, except an IC
   with both supplies fixed (W1c point 2) and an LED-family device, whose
   junctions are placed one by one. `compile_dyn_shape` and `static_part`
   place junctions by node, not by owning device; code that resolves a
   device's owner (`owner_of`, `owned_by_static`) never sees LED kinds.
   Partition membership is fixed at load: a toggle only changes what is
   stamped, never which partition a node is in, and `bo_engine_set_params`
   marks that one partition dirty.
2. **Inert capacitor.** A capacitor whose two pins are on the same node
   stamps nothing, does not count toward its partition's classification, and
   publishes `0`, as classic does (its companion stamps cancel). An LED
   junction whose two pins share a node is the same: it stamps nothing, does
   not classify and carries `I(0) = 0` (W1e, LEDs point 1).
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
     voltage is live in W1b, so nothing re-aims it after load yet; from W1f a
     rail move or a source edge re-aims it (Rails point 7).
   - An LED junction across two fixed nodes is the same case (W1e, LEDs
     point 4): it belongs to no partition and is not `-3`, publishes
     `I(dV)` and enters the source KCL.
3. **Classification** (decision 14). A partition with no counted capacitor
   is static. One with exactly one is closed-form. One with two or more is
   stepped. `FORCE_STEPPED` (load-time only) makes every closed-form
   partition stepped. A partition holding a stamped LED junction is
   nonlinear (W1e, LEDs point 5): static when it holds no counted
   capacitor, stepped otherwise, never closed-form. In code,
   `partition::build` gains a per-partition `holds_junction` and adds such
   partitions to `must_step`; `closed_form::build` asserts it never
   receives one.
4. **Static partitions** keep a solution cache keyed on driver modes, the
   rail band, the floating bits and an epoch that bumps only on a real value
   change. Each partition holds up to 256 keys (W1e), so a partition whose
   drive alternates between configurations (an IC output clocked HIGH, LOW,
   HIGH) hits on every repeat; a partition's cache is cleared when it is
   full. A hit replays the vector stored for that exact key.
   `NO_SOLUTION_CACHE` bypasses it. A nonlinear static partition
   keeps the cache too (W1e, LEDs point 7): `NO_SOLUTION_CACHE` stays
   bit-identical for linear partitions, and a nonlinear one agrees within
   1 µV (the polish residual). W1f fills the rail band, gates a hub
   partition's hit on its distance from the stored rail (Rails point 3) and
   adds each feeding waveform source's level (Rails point 7).
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
   instant on. A step whose Newton does not converge (a partition holding a
   stamped LED junction or a free IC supply node) is rejected and retried at
   `max(H_MIN, 0.25 h)`, `-4` only at the floor, and the re-aim seed
   warm-starts from the committed `x` (W1e, LEDs point 8). This changes
   W1c's free-supply stepped partitions too, which returned `-4` at once.
7. **Rollback.** A failed instant (`-4`, `-7`, in `advance` or
   `set_params`) restores every piece of solver state the attempt touched:
   published arrays (the channel array included, W1e, LEDs point 16), each
   static partition's own solution and cache, the closed-form and stepped
   partition state, and any crossing or step event the attempt scheduled. Nothing from the failed attempt reappears later.

## Kernel semantics: the digital core (W1c)

References are to `core/spec/w1-classic-semantics.md`.

"An IC" below includes code 40, the NE555, for points 1-3, 8 and 9 (NE555
point 1). In code: `digital::is_ne555`, `is_ic` includes it,
`output_capable(40) = [1, 5]`, and `is_read_and_driven(40)` is false.
Partitioning (`partition::build`), the stepped promotion, the reader index,
`ic_powered` and the power-loss hold then cover the 555 with no change of
their own, and the source KCL does with W1a point 2's branch fold.

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
   term, which is a fixed-point iteration, and the core does the same in an
   iterated static partition: one that holds a free IC supply node or (W1e)
   a stamped LED junction. It re-stamps from the latest iterate until
   classic's Newton test passes (`|dx| <= 1e-6 + 1e-3 * max(|x_new|,
   |x_old|)` per node, classic's own form, at most 50 iterations; failure is
   `-4`). One holding a junction then runs one polish iteration (LEDs point
   6.4); one holding a free IC supply and no junction does not. Static and
   stepped partitions share one loop: `static_part` calls `newton::solve`,
   which gains a non-finite-iterate check and a `polish` argument, and
   `linear` is `!(holds_free_ic_supply || holds_junction)`. A dynamic
   partition holding a free IC
   supply node is stepped, because R_out then moves with its state. So is a
   dynamic partition holding a 74ls245 `a`/`b` pin: those pins are read and
   driven, a closed-form crossing watch cannot follow a pin its own IC drives,
   and a stepped partition re-reads its ICs after every step. So is a
   dynamic partition holding a watched read pin (point 4) of an IC whose
   `vcc` or `gnd` is free in a stepped partition: a closed-form crossing is
   solved once, for one threshold, and that supply moves the threshold
   between events. In code, `partition::build` promotes such a closed-form
   partition after the first classification, in one pass (a promoted
   partition holds no free IC supply, so it makes no new stepped supply, and
   a static partition is never made dynamic); a 555's watched pins are TRIG,
   RST and THR, so a bypass capacitor on CV promotes nothing. When every
   supply node is fixed, the power is constant and the solve is direct.

   A driver on a fixed node other than the datum belongs to no partition, but
   its stamp lands in the partition of whichever supply is free: HIGH into
   `vcc`, LOW into `gnd`, a 555's OUT into both, its DIS into `gnd`.
   `devices::ics::register` therefore registers it against the partition of
   its free `vcc`, else its free `gnd`, so its commit dirties that partition
   and re-solves the supply at the commit instant. A driver dirties exactly
   one partition, so an IC with both supplies free and an output-capable pin
   on such a node has its two supplies unioned by `partition::build`. A pin on
   the datum stamps nothing (point 8) and needs neither.
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
   partition's crossing watch (a 555 registers NE555 point 5's thresholds on
   pins 2, 4 and 6 instead, and on a stepped partition whatever its supply),
   so it is re-evaluated at the crossing instant
   (accepted only at `t > now + 1e-15`), not at the next unrelated event. An
   output-capable pin is not watched: its level never changes the kind's
   result (the 74ls245 aside, rule 3), and it sits on its own driver. A
   stepped partition instead re-reads every IC reading a node that moved,
   after every step, including the steps a horizon clamp takes (W1a point 5).
   A watch's thresholds come from the IC's live supply voltages. So an IC
   whose supply sits in a static partition is watched too, and its watches
   are re-aimed whenever that supply moves. A pin on a stepped partition is
   watched whatever its supply, as a 555's is, and one on a closed-form
   partition only when the supply is fixed or static (point 3 promotes any
   other). In code, `register_ic_read_pin_watches` skips code 40 in its
   generic loop and registers `digital::ne555::watch_thresholds`, and
   `add_watch_dependent_partitions` re-aims the partitions holding a watched
   pin (pins 2, 4 and 6 of a 555) of an IC whose supply is static, or is
   stepped and being re-solved (its trial is discarded, and a dependent's
   crossing was aimed at it). `watch_node` and `ThresholdFn` serve stepped
   partitions.

   A supply that moves moves the threshold, so a stepped crossing is found
   against the supply's value at the crossing instant. The thresholds are read
   at the start and end of each planned step and at each bisection trial, from
   that instant's state: this partition's nodes from the trial, and a supply in
   another stepped partition from that partition's pending trial (a parabola
   through its last two commits and the trial's end, a line right after a
   discontinuity). The watching partition's step never runs past that trial's
   end, and before it plans, and before the IC is read at one of its steps,
   each supply partition is brought to that instant by a real step from its
   last commit and planned afresh. So the IC reads the supply the crossing was
   aimed at, and the miss is the supply partition's own step-control
   accuracy, within W0's tolerance on the threshold (100 uV + 1e-3
   relative). A supply cycle between partitions plans the innermost against
   the trial it already has. Partitions otherwise advance independently, each
   on its own `Step` events, and an IC evaluated at an event of one of them
   reads the others as last published.

   What guards this mechanism is the Rust tests, not a fixture. Fixture 40
   (a 555 whose watched pin and free supply sit in different stepped
   partitions) is a smoke test: it passes against classic, and also passes
   with the supply dependency removed, because classic's fixed step cannot
   resolve the difference. Fixture 38 (the same layout for an hc14) fails
   without gap 1's promotion and watch, but has not been checked against the
   supply dependency alone. These tests fail when a piece is removed:
   - in `solve/ic_tests.rs`:
     `gap1_a_read_pin_in_the_supplys_own_stepped_partition_flips_at_the_moving_thresholds_crossing`,
     `gap1_a_closed_form_pin_beside_a_quiet_stepped_supply_is_promoted_and_flips_at_the_analytic_time`,
     `gap1_a_read_pin_in_another_partition_meets_a_threshold_that_is_still_moving`,
     `gap1_a_supply_re_solve_re_aims_the_dependent_watch_to_an_earlier_crossing` and
     `gap1_a_555_thr_pin_in_another_partition_resets_at_the_moving_thresholds_crossing`;
   - in `solve/stepped.rs`:
     `a_threshold_that_reads_another_node_of_the_partition_is_taken_at_the_trial_instant`,
     the four `a_supply_trace_*` tests (the parabola read) and
     `a_step_never_runs_past_the_end_of_a_supply_trial_however_long_its_own_control_would_take`;
   - in `devices/ics.rs`, for the fixed-node driver: the four `gap2_*` tests.
5. **Floating model** (§1.5, owner decision 9): `bucket = floor((t + 1e-12) /
   0.01)` of the evaluation time; `n = finish(hashState, bucket) /
   0xffffffff`; HIGH when `n < pHigh`. At each bucket boundary, every
   partition holding an `OPEN` `FLOATING_INPUT` pin of a powered IC is marked
   dirty (its pull target may change), and every IC holding such a pin or
   reading a dead-band pin is re-evaluated. A 555 takes no part: it never
   reads through the floating model and holds no `FLOATING_INPUT` pin. In
   code, `any_powered_ic_needs_bucket_watch` and `bucket_boundary` skip code
   40; otherwise a 555 pin inside the generic band `(max(0.8, 0.3 vS),
   max(2.0, 0.6 vS))` (every astable's timing node, most of each period)
   would schedule a bucket event every 10 ms and wake the 555 for nothing.
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
   A 555's OUT and DIS stamp by their stages (NE555 point 7), with the same
   datum and fixed-node rules.
9. **Power loss** (owner decision 8, Q-O4): while an IC is unpowered its
   drivers keep their committed level and in-flight transition frozen; a
   commit that comes due meanwhile is held, not applied. On the first powered
   evaluation the partition is solved with the pre-outage committed levels,
   and any held transition whose due time has passed commits in the next
   delta, as classic's first powered pass does.
10. **Readouts** (codes 50-61). A digital slot holds the pin's committed level
    (electrical: HIGH, or released for open-collector, is 1) while the pin is
    driven, and `NaN` (an absent key) while the IC is unpowered, or the pin is
    committed HIZ or has never committed. Code 40 (NE555 point 12) and codes
    70-82 (W1d) publish evaluator state instead.

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

## Kernel semantics: LEDs and displays (W1e)

References are to spec chapter 6, §10.5-10.6 and §10.10; classic line
citations are at `3b22a3e` ("Kind codes and records").

1. **Junctions.** Every kind is a set of junctions, each an anode node, a
   cathode node and a law (point 2). A kind's junctions are fixed at load:
   - `led`: one, `pins[0]` to `pins[1]`.
   - `bicolor_led`: `a1` to `k`, then `a2` to `k`, both always, even when an
     anode pin is `OPEN` (§6.6).
   - `rgb_led`: `r_a`, `g_a`, `b_a`, each to `com_k`, always.
   - `seg7_cc` / `seg7_ca`: the common node is `com` when `com` is not
     `OPEN`, else `com2` when `com2` is not `OPEN`, else none (DI:1217-1224,
     point 14). With a common node, each segment that is not `OPEN` is a
     junction (DI:1226), with the segment as anode and the common node as
     cathode on `seg7_cc`, the reverse on `seg7_ca` (DI:1228). `dp` is a
     segment like the others.
   - A `burnt` device has no junctions: it stamps nothing (SC:358-360,
     SC:548-550, SC:615-617).

   A junction is **stamped** when its two nodes differ. One whose two pins
   share a node stamps nothing (its companion cancels, as a self-shorted
   capacitor's does), does not count toward classification and carries
   `I(0) = 0`.

2. **The law** (§6.1; EL:693-786, SE:156-193). Per junction, at temperature
   `T` (°C), from the resolved params:
   - `vt = K_B_OVER_Q * max(1, T + 273.15)`, `K_B_OVER_Q = 8.617333262145e-5`
     (SE:109, 156-159);
   - `vf = max(0.05, vf25 + (-0.002) * (T - 25))` (SE:111, 162-168);
   - `vtn = n * vt`, with `n = 2` for every kind but `led`;
   - `Is = iRated * exp(-vf / (n * vt))` (EL:755-762), or, for a `led` with
     `isSet = 1`, classic's `saturationCurrentAtTemperature(is, T, n)`
     (SE:175-193): `nn = max(0.1, n)`, `t = max(1, T + 273.15)`,
     `tNom = 298.15`, `x = clamp((1.11 / (nn * K_B_OVER_Q)) * (1/tNom - 1/t),
     -80, 80)`, `Is = clamp(is * (t/tNom)^(3/nn) * exp(x), 1e-30, 1)`;
   - `vSat = min(max(40 * vtn, vf + 5 * vtn), 80 * vtn)` (SC:381-382), the
     temperature-adjusted `vf` in it, the `Is` override notwithstanding.

   The current law, `I(v)`: `c = min(v, vSat)`, `e = exp(c / vtn)`,
   `I = Is * (e - 1) + ((Is * e) / vtn) * max(0, v - vSat)`: the exponential
   below `vSat`, its tangent above (EL:772-786).

   The companion about the previous iterate `vPrev` (EL:693-742):
   `v = vPrev > vSat ? vSat : vPrev`, `gd = (Is * exp(v / vtn)) / vtn`,
   `id = I(vPrev)`, stamped conductance `g = gd + GMIN_D` with
   `GMIN_D = 1e-12`, Norton offset `ieq = id - gd * vPrev`, so the branch
   current anode to cathode is `g * (vA - vK) + ieq` (the
   `stamp_two_terminal` convention, `solve/mod.rs`). GMIN carries no offset
   (EL:721-725).

   Rust evaluates these in classic's operation order. `f64::exp` and `powf`
   need not match V8's `Math.exp` and `Math.pow` to the last bit (D13); the
   goldens against classic compare them within 4 ulp.

3. **Temperature.** `T` is `AMBIENT_C` (option 5) for every junction in W1e.
   The multi-junction kinds are ambient-only forever (`ctx.junctionVt()` /
   `junctionVf()`, SE:1818-1828). Self-heating is limited to a plain `led`
   with an exact thermal identity (Q-O21, §6.3), and it is W1g. Until then
   the compiler declines such a `led`
   (`unsupported-feature:led-thermal:<componentId>`, the circuit runs on
   classic): one whose `catalogUid` classic's `_thermalProfile` accepts
   (SE:1830-1849). It decides with `thermalProfileForComponent(comp)`, the
   body of `_thermalProfile` without its cache, exported from `sim-engine.ts`
   for this and for W1g. So every `led` that reaches the core has no thermal
   state, and no known divergence is registered for it. W1g removes the
   decline when it ports self-heating: it feeds that LED's held junction
   temperature into point 2 and re-solves its partition when the held value
   moves (D19, orchestrator lock). The tempco is the fixed `-2 mV/°C`
   (`DEFAULT_VF_TEMPCO`); no param overrides it. The law's inputs are fixed
   at load in W1e, so each junction's `(vtn, Is, vSat)` is computed once.

4. **Partitions.** The union-find also joins the anode and cathode of every
   stamped junction whose two nodes are both free, junction by junction, as a
   resistor's two ends are joined (the prototype's `join(d.a, d.k)`,
   `pec3.mjs:264`). A device's other pins are not joined through it: a
   `bicolor_led`, `rgb_led` or `seg7` whose common pin is fixed puts each
   channel in the partition of its own drive. So an LED-family device can
   touch several partitions, as a W1c IC with both supplies fixed can (W1c
   point 2). Each partition stamps exactly the junctions that touch it; a
   junction's two nodes are each fixed or free in that one partition, by the
   union rule. A burnt device joins nothing.

   A stamped junction with both nodes fixed belongs to no partition and is
   not `-3` (an LED straight across an ideal source): its current is `I(dV)`
   with `dV` fixed, and it enters the source KCL (point 12). No fixed-node
   voltage is live in W1e, so it is constant; from W1f it re-publishes at
   each rail move or source edge (Rails point 7).

5. **Classification** (amends W1b point 3). A partition holding a stamped
   junction is **nonlinear**. A nonlinear partition with no counted
   capacitor is static and Newton-iterated (point 6). A nonlinear partition
   with one or more counted capacitors is stepped: it is never closed-form,
   because the exact exponential needs a linear network. So an LED plus a
   capacitor is stepped. Classification is fixed at load: a W1g `led_failed`
   latch removes a junction's stamp but not the classification.
   `FORCE_STEPPED` and stat 15 are unchanged (stat 15 counts partitions with
   two or more capacitors, junctions or not).

6. **The nonlinear static solve.** Classic's Newton (NW:73-130), per
   partition:
   1. Start from the partition's last published voltages: 0 V before its
      first solve, as classic's first solve starts from `netV = {gnd: 0}`
      (SE:2942, 5650-5656); afterwards whatever it last published, as
      classic warm-starts every solve from the previous accepted `netV`
      (SE:5650-5669).
   2. Each iteration restamps the partition from scratch: node shunts,
      resistors, switch contacts and non-IC drivers as the direct solve
      does, every IC driver and pull from the iterate (W1c point 3), and
      every junction's companion about the iterate (point 2). Then one dense
      LU solve (W0 point 4). A zero or non-finite pivot, or a non-finite
      entry in the new iterate (NW:109-111), is `-4` at once.
   3. The iterate passes when every node `i` of the partition has
      `|x'_i - x_i| <= 1e-6 + 1e-3 * max(|x'_i|, |x_i|)` (NW:112-114,
      classic's own `max` form). At most 50 iterations (SE:5680,
      `maxIter: 50`); a partition that has not passed by then is `-4`
      ("Newton did not converge within 50 iterations"), and the instant
      rolls back (W1b point 7).
   4. **Polish.** When the test passes, the partition runs exactly one more
      iteration from the passing iterate and publishes its result. That
      iteration is not tested; only a pivot or a non-finite entry fails it.
      Classic needs no such step because it re-solves every partition at
      every accepted step, warm-started from the last: each quiet classic
      step is one more Newton iteration from an already converged point, so
      one quiet step after any change classic sits where the core's polish
      puts it, and a step later at the fixed point to rounding (point 11).
      The core solves a static partition once per change, so it takes that
      iteration at once. Measured on the drafting probes: the passing
      iterate is up to 145 µV from the fixed point and its reported junction
      current up to 5.6 times the amps tolerance (a `seg7_cc` with all eight
      segments lit through 330 Ω each and a shared 100 Ω common resistor, at
      -40 °C, warm-started 0.2 V off the fixed point); after the polish, at
      most 0.39 µV and 0.015 times the tolerance, over 18 000 one-node cases
      and 180 two- and nine-node cases.
   5. Stat 11 counts every iteration, the polish included.

7. **The solution cache** (amends W1b point 4). A nonlinear static partition
   with no free IC supply node keeps the cache, as the plan's static class
   says ("cached per driver configuration and rail band") and as the
   prototype did (`pec3.mjs:507-529`). The key is unchanged: driver modes,
   rail band, floating bits and the epoch; every junction input is fixed at
   load in W1e. A miss runs point 6 and stores the polished result; a hit
   publishes the stored vector with no iteration. The epoch is solver-wide
   (`static_part.rs` `CacheKey`, bumped by `apply_params` on a real change
   to a static partition's switch), so such a change makes every static
   partition's next solve a miss, nonlinear or not; a board clocked by a 555
   or an IC oscillator never moves it. A partition holding a free IC supply
   node bypasses the cache as before (W1c point 3), junctions or not. With
   `NO_SOLUTION_CACHE`, a nonlinear partition is solved by point 6 from its
   last published voltages, so it agrees with the cached run within 1 µV
   (the polish residual, at most 0.39 µV measured), not bit for bit; linear
   partitions stay bit-identical. W1g leaves the key alone and instead
   clears a partition's cache at every slow stamp change (a latch, a
   junction re-arm, a PTC trip or recovery, a reset), since the key carries
   no failure or law state (W1g point 12).

8. **Stepped partitions with junctions** (amends W1b point 6). Each BE or
   trap step runs the same Newton as points 6.1-6.3 (`newton::solve` with
   `linear = false`), stamping every capacitor companion, IC driver and
   pull, and junction companion about the iterate, with no polish, exactly
   as classic solves each step. Warm starts are unchanged: the committed `x`
   for the coarse and first half legs, the first half's result for the
   second (`stepped.rs` `plan_step`). Three changes:
   - A step whose Newton does not pass within 50 iterations is rejected, not
     `-4`: `h` becomes `max(H_MIN, 0.25 * h)`, the partition is marked
     changing, and the step-doubling trio is retried, as classic's host does
     (`nextFactor: 0.25`, HL:609-617, 627-630; retry at HL:819-831). A step
     that fails at `h <= 1.01 * H_MIN` is `-4`, classic's `hitMinStep`
     (HL:827-829). A zero or non-finite pivot stays `-4` at once (W1b). This
     covers every stepped Newton, W1c's free-supply stepped partitions
     included, which returned `-4` at once before W1e (`stepped.rs`
     `take_step`).
   - A crossing-refinement trial (`refine_crossing_h`) that does not
     converge ends the refinement at the best `h` found so far.
   - The re-aim seed (`solve_initial_x`, the `1e-12` BE step at load and at
     every discontinuity) warm-starts from the committed `x`, not from zero.
     It is not a step and has no smaller `h` to retry: non-convergence there
     is `-4`.

   The crossing watch and the stepped re-read of ICs (W1c point 4) are
   unchanged: both already work on whatever a step solves.

9. **With the W1c fixed point.** A partition holding both a free IC supply
   node and a junction runs one loop: every iteration restamps the IC
   drivers and pulls and the junction companions from the same iterate, with
   the test and cap of point 6. That is classic's single global iteration
   restricted to the partition. Such a static partition polishes (it holds a
   junction) and bypasses the cache (it holds a free supply). A static
   partition with a free IC supply and no junction keeps W1c's behaviour,
   with no polish, save the test's form (W1c point 3).

10. **Topology changes.** Membership is fixed at load. A switch toggle, a
    `bo_engine_set_params` batch, a driver commit (an IC output driving an
    LED changes level, or goes HIZ) or a floating-bucket boundary marks the
    partition dirty, as for any partition. A dirty nonlinear static
    partition re-runs point 6 from its pre-change voltages, which is where
    classic's first step after the change starts too; the epoch bump on a
    real change keeps a toggle from hitting a pre-toggle cache entry. A
    dirty stepped partition catches up with its pre-change stamps, then
    re-seeds from its committed `x` (point 8) and restarts at `POST_EVENT_H`
    with the BE anchor (W1b point 6).

11. **Per-partition Newton against classic's global Newton.** Classic solves
    every unknown of the circuit in one Newton (§10.5-10.6, D24). After
    fixed-node elimination the core's partitions are exactly the diagonal
    blocks of that system: a fixed node's value comes from its source row
    alone, and every IC coupling that depends on the iterate (driver
    `R_out`, pulls) lies inside one partition by W1c point 2's union rule. So
    in exact arithmetic a partition's iterates are the ones classic computes
    for its rows, from the same warm start. Digital levels are not part of
    the iteration: the core settles them by delta cycles (events point 3),
    and classic's zero-delay paths inside Newton (§2.5, G11) are W1c's
    accepted difference, not this wave's. The iterates differ in five
    places, none beyond W0's tolerances:
    - **Stopping.** Classic stops only when every unknown passes on the
      same iteration, source branch currents included, so a partition that
      converges early takes more iterations in classic than here. The polish
      (point 6.4) and classic's per-step re-solve both land on the fixed
      point: 0.39 µV worst case measured against W0's 100 µV + 1e-3
      relative, 0.015 of the amps tolerance.
    - **The cap.** Classic's 50 is shared by the whole circuit, the core's
      is per partition. Since a partition's iterates are the same, a
      partition converges in the core whenever it converges in classic. The
      core can converge where classic does not (another block stalls
      classic), which only turns a classic failure into a core result.
    - **Re-solves.** Classic re-solves a quiet LED partition at every step
      (a polish each time); the core solves it once per change, or serves it
      from the cache. The polished result and the cached result are both
      that fixed point.
    - **Right after an event.** Classic's first step after a change
      publishes its unpolished iterate for one step; the core publishes the
      polished one at once. The quiet-instant rule (gate 2b) keeps such
      samples out of comparisons.
    - **Failures and health.** Classic retries a non-converged step at
      `0.25 h` (HL:609-617); static partitions have no `h`, so the core
      returns `-4` where classic would retry the same failing solve down to
      `hitMinStep`, and stepped partitions retry exactly as classic (point
      8). Classic also rejects a solve whose relative residual exceeds
      `1e-8` (SE:5695-5699); the core checks pivots and finiteness only (W0
      point 4), which can only accept a solve classic's global matrix
      rejects for conditioning. Per-partition LU and global LU also round
      differently (D13).

    The stepped case keeps W1b's accepted stepping divergence: a stepped
    partition picks its own steps, and classic's plain path also scales `h`
    by Newton effort (`iters > 15`: 0.7, `iters <= 3`: 1.5, HL:620), which
    the core's step doubling does not use.

12. **Currents and the source KCL** (amends W0 point 5). Every stamped
    junction, in a partition or across two fixed nodes, contributes its
    branch current `I(vA - vK) + GMIN_D * (vA - vK)` leaving the anode and
    entering the cathode to `R_u` in the source KCL walk
    (`recompute_source_currents`). At the published voltages that equals the
    companion current classic's source branch carries at convergence, within
    Newton tolerance. The datum end of a junction feeds no source, as for
    every other device. `elementI` ("Currents") excludes GMIN and never
    enters the KCL walk, since for three kinds it is an aggregate, not a
    branch current.

13. **Readouts.**
    - Classic publishes no `digitalState` and no `displayState` for any of
      the five kinds (no `updateDigital` hook, §6.6, §10.9): no digital
      slots. `getDisplayState()` stays `{}` in W1 (W1 plan, W1e "Display
      readout"); hosts derive lit segments from channel currents.
    - Each record's `elementI` ("Currents") and channel values are pure
      functions of the published voltages and the device's `burnt` flag
      (W1g adds `led_failed`). The solver computes them in one device pass,
      beside `recompute_source_currents`, when the engine prepares a
      readout (`PartitionSolver::prepare_readout`, at the end of `load`,
      `advance`, `set_params` and `seed_memory`, success or failure). That is
      identical to computing them at every point that publishes voltages
      (`resolve_dirty`, `handle_dynamic_event`, `sync_dynamic_to`), which the
      solver still does whenever a derived value could be non-finite, since
      that would fail the instant. A device none of whose pin voltages
      changed bitwise since its last pass may be skipped: the result would
      be identical.
    - Channel values: `rgb_led` and `bicolor_led` channel `j` is
      `max(0, I_j)` (SE:4517, 4533); `seg7` channel `j` is `max(0, I_j)` for
      a stamped segment and `0` for an `OPEN` segment or when no common pin
      is stamped (SE:4557-4569). While an `rgb_led` or `bicolor_led` is
      `burnt`, every one of its channel slots is `NaN`, meaning its entry is
      absent (SE:4503, 4520: classic omits it). `seg7` never is (no `burnt`
      gate, Q-O22).
    - Channel values use the same ambient law as the stamp: classic's
      `getElementChannelI` uses ambient `Vt` and `Vf` (SE:4500, 4513, 4529,
      4542) and the same param precedence (§10.10), identical to what the
      stamp uses for these three kinds.
    - The published values are those of the current instant. Classic
      computes `getElementChannelI` on demand from `_lastX` (SE:4495-4499),
      the last accepted solve, which is the same thing.
    - In code, the solver gains a published channel array (`fn
      channels(&self) -> &[f64]` on `PartitionSolver`, default empty),
      written by the device pass and carried in `PartitionedSolver`'s
      `dynamic_snapshot`. The device pass is a `PartitionedSolver`-level
      pass like `recompute_source_currents`, because an LED device's current
      is not one partition's stamp. `Engine` copies the array into
      `Readout`'s channel slots after `load` and after every `advance`,
      `set_params` and `seed_memory`, success or failure, as it drains
      digital writes. `check_finite` accepts a `NaN` channel only on a
      device that is absent (`burnt`; W1g adds failed); any other non-finite
      channel value is `-4`.

14. **The com/com2 tie and the com-open check** (§6.9). `buildNets` unions
    `com` and `com2` of every `seg7` into one net (GR:160-167), so both pin
    records carry the same node, and the compiler's `OPEN` flags follow
    classic's rule on that net: wiring either lead makes both non-open (G8).
    The core stamps from the flags alone (point 1): `com`, else `com2`, else
    nothing, which is classic's order (DI:1217-1224, SE:4549-4555). The
    `com2` fallback is reachable only when the pin list lacks `com`. With
    neither stamped the device stamps nothing, publishes `elementI = 0` and
    eight zero channels. A source wired from `com` to `com2` is shorted to
    itself: that is `-3` (W0 point 1), so the circuit falls back to classic,
    where `seg7-load.test.ts:127-148` expects `lastConverged === false`.

15. **The damage limit (Q-O24).** Classic's fix is in (`3b22a3e`):
    `ifMax = params.iRated ?? specs?.if_max ?? 0.02` for `led`, `rgb_led`
    and `bicolor_led` (SC:481). W1e ports the resolution only: the `led`
    record's `iRated` and the `rgb_led`/`bicolor_led` `ifMax` carry it,
    validated finite, and nothing reads them. W1g ports the latch on top:
    stress rate `max(0, r² - 1)` with `r = peakI / ifMax` (0 when
    `ifMax <= 0`), `peakI = |elementI|`, for `rgb_led` and `bicolor_led` also
    the maximum `|channel|`, recovery 2/s, threshold 24 (SC:472-511,
    §9.5.1); the latch removes every junction of the device, zeroes its
    `elementI`, makes its channels (if any) `NaN`, and marks every partition
    it touches dirty; the message is classic's (SC:508). `seg7` never fails
    (Q-O22). `led-damage-limit-irated.test.ts` needs the latch, so it joins
    gate 1 at W1g.

16. **Rollback** (amends W1b point 7). The channel array is published solver
    state: a failed instant restores it with the voltages and currents, and
    restores (or clears) any per-device memo point 13 keeps.

17. **What LEDs do not do.** They create no drivers, schedule no events,
    register no crossing watches and are not readers. They have no live
    params. AC small-signal stamps (`shockleyConductanceAtOp`, SC:95-106)
    are W6, thermal state and failures W1g, telemetry and load carry-forward
    W2 (§6.8), and `led`'s role as a flyback clamp (`FLYBACK_CLAMP_KINDS`,
    §6.10) belongs to the `missing_flyback` port. A nonlinear partition on a
    hub rail is W1f's, against point 6 (the prototype stamped diodes in
    `_railSens` and re-solved a nonlinear partition past 25 mV of rail
    drift, `pec3.mjs:478`).

## Kernel semantics: NE555 (W1e)

References are to spec chapter 5, classic's `ne555Model` (DI:178-308) and
decision 7; classic line citations are at `3b22a3e` ("Kind codes and
records").

1. **Shared machinery.** Code 40 is an IC for W1c points 1-3, 8 and 9:
   power, partitions, power-dependent stamps and the stepped promotion, the
   output stage (with point 7's stages) and the power-loss hold, all through
   its `IcRecord`. It takes no part in W1c points 4-7 and 10: it reads no
   logic level, has no floating model, pull or bucket event, no truth table
   and no delay, and it publishes its latch. The points below replace them.
   Its evaluation lives in `digital/ne555.rs`, dispatched by the IC
   evaluator as its own family: `devices/ics.rs` gains
   `Family::Ne555 { out_high }`, registration creates OUT and DIS with their
   stages, evaluation branches to `digital::ne555` before any `levels::read`
   and leaves `last_reads` alone, and `on_settled` publishes its family slot
   as the W1d families do. Small-signal AC (DI:275-308) and the DC operating
   point's regime signature (§5.3) are W6.
2. **Power and reference** (§5.2 "Power detection"; W1c point 1).
   `powered = !OPEN(8) && !OPEN(1) && vccMin*0.9 <= V(8) - V(1) <= vccMax*1.1`
   (4.05-17.6 V with the catalog, 1.8-19.8 V without), and
   `vS = max(0, V(8) - V(1))`.
   - Every comparator input is relative to pin 1: `vTrig = V(2) - V(1)`,
     `vThr = V(6) - V(1)`, `vRst = V(4) - V(1)` (DI:227-229).
   - `vUpper = (2/3)*vS` and `vLower = (1/3)*vS`, computed exactly as
     classic writes them (`(2.0 / 3.0) * vs`, DI:230-231).
   - R_out and the logic thresholds of W1c point 1 are not used.
   - CV (pin 5) is never read or stamped, so an external voltage on it does
     not move the thresholds. Input bias currents are not modelled (decision
     7).
3. **Comparators and latch** (§5.2, DI:245-247). The state is one bit,
   `outHigh`, with power-on value 1 (`{outHigh: true}`, DI:233). One
   powered evaluation resolves the following, all comparisons strict, first
   match wins:
   1. `vRst < 0.5`, in absolute volts above pin 1, not a supply fraction: 0.
   2. Otherwise `vTrig < vLower`: 1.
   3. Otherwise `vThr > vUpper`: 0.
   4. Otherwise the stored `outHigh`.

   So SET beats RESET when both comparators are true (the monostable hold,
   DI:235-244), and a node exactly on a threshold changes nothing. With
   edges live (every evaluation except load settling's frozen passes,
   point 9) the result is written to `outHigh`. An unpowered 555 reads,
   writes and requests nothing, so `outHigh` holds (§5.2 "Unpowered").
   An unwired RST reads 0 V (its node holds only the shunt; §5.2 "Open
   RST", Q-O16), so OUT is LOW from load on, as in classic, with no special
   case.
4. **Requests.** Each powered evaluation requests, with
   `DelaySet::uniform(0)`, OUT HIGH or LOW as the result, and DIS LOW when
   the result is 0 or HIGH (released) when it is 1. Both commit in the next
   delta at the same instant (W1a point 3). OUT and DIS therefore switch at
   the instant of the evaluation that changes the latch: there is no output
   lag (decision 7, Q-O19), and no propagation delay.
5. **Crossing watches** (decision 7, D11; W1c point 4's mechanism). At load
   the solver registers these watches on each of a 555's read pins that sits
   on a stepped partition, and on each that sits on a closed-form partition
   when pins 8 and 1 are both fixed or on static partitions:
   - pin 2 (TRIG) at `V(1) + vLower`, rising and falling;
   - pin 6 (THR) at `V(1) + vUpper`, rising and falling;
   - pin 4 (RST) at `V(1) + 0.5`, rising and falling.

   The `ThresholdFn` reads V(1) and V(8) from the published array at every
   re-aim or step plan, whatever the power state, so a threshold always
   follows the live supply. Pins 8, 1, 3, 5 and 7 are never watched. TRIG and
   THR on one node (an astable) are two registrations; the earliest crossing
   wins.
   - **Why both directions.** The latch can change on RST falling (reset)
     and rising (release to TRIG and THR), on TRIG falling (set) and rising
     (a held reset appears), and on THR rising (reset). THR falling cannot
     change it, but it is watched too, so that a watched node's published
     voltage always lies on the same side of every threshold as its exact
     value. The 555 is also evaluated at events that do not touch its
     watched partitions (a TRIG button, a supply re-solve, its own OUT
     commit), and there it reads a closed-form node's voltage as last
     published. The invariant makes that read give the exact comparator
     result.
   - **Closed form** (W1b point 6). The crossing time is analytic. The
     crossed node lands 1e-9 V past the threshold, so the strict comparison
     fires, and the 555 is evaluated at that instant. Its OUT and DIS commits
     re-stamp and re-aim at the same instant in the next delta, anchored at
     the exact `vc`, so the trajectory stays continuous. An astable is exact
     up to the landing nudge: each landing moves the trajectory 1e-9 V along
     itself, a few picoseconds.
   - **Echo crossings.** After a SET the node sits 1e-9 V below `vLower` and
     is now rising, so the TRIG watch fires again about `1e-9 V / slope`
     later (accepted when that is later than `now + 1e-15`); after a RESET
     the THR watch fires the same way, falling. The 555 evaluates and holds.
     An astable takes four crossings per period (two switches, two echoes);
     stat 9 counts all four. A latch-aware watch (the prototype's THR only
     while HIGH, TRIG only while LOW, `pec3.mjs:1060-1065`) would avoid the
     echoes, but a `ThresholdFn` cannot see evaluator state, and it would
     break the invariant above.
   - **Stepped** (W1b point 6; the promotion of point 6). A step that shows
     a crossing is bisected onto it, landing within 1e-9 V past the
     threshold, and every step re-reads its readers. Crossing times carry
     the stepped integration error; exact timing needs closed form.
   - **Moving thresholds.** The thresholds move only with V(8) and V(1).
     - Both supply nodes fixed (an ideal supply): the thresholds are
       constant and every crossing is exact.
     - A supply node on a static partition: the node moves only at events.
       When that partition re-solves, every closed-form or stepped partition
       holding one of this 555's watched pins is re-aimed in the same
       resolve, so its crossings are rescheduled against the new thresholds
       (W1c point 4). The 555 is also a reader of pins 8 and 1, so it is
       evaluated at that instant, which catches a threshold that moves
       across a node that did not move.
     - A supply node on a dynamic partition: that partition is stepped
       (point 6), and the watches are registered all the same. Each step
       plan reads the thresholds from the supply at the trial's own instant,
       and a step that shows a crossing is bisected onto it, so the
       threshold is the supply's value at the crossing (W1c point 4). W1c's
       ICs register the same watch on a stepped partition. A stepped
       crossing is found by bisecting real steps, with no backward solve. A
       supply on a hub rail is W1f's.
6. **Partitions** (W1c points 2-3). OUT (logic index 1) and DIS (5) are the
   output-capable pins: each free one joins every free supply pin. TRIG,
   RST, THR and CV join nothing through the 555.
   - With an ideal supply (pins 8 and 1 fixed), the timing network
     partitions by its own resistors and capacitor. An astable's or
     monostable's timing partition holds one capacitor and is closed-form,
     and OUT partitions with its load.
   - A dynamic partition holding a free 555 supply node is stepped: OUT's
     fraction and the power gate move with its state. With a free VCC the
     timing network joins it through RA (or the monostable's R), so its
     timing is stepped until W1f treats such a rail as a hub. A partition
     with a free 555 supply node and no capacitor stays static and runs W1c
     point 3's fixed-point iteration, so a fixture that needs a stepped
     555 supply puts a capacitor in that partition.
   - The monostable ties DIS to THR. Unlike W1c's read-and-driven pins, that
     does not force stepping: the DIS commit follows a THR crossing and
     moves the node away from it, so no crossing races its own commit. The
     only consequence is point 5's echo.
7. **OUT and DIS stamps** (§5.2 "OUT stage" and "DIS"; DI:180-219;
   `_stampRailReferencedOutput`, SE:5369-5380). OUT and DIS are drivers with
   their own stages (W1a point 2). While the 555 is powered, each stamps
   from its committed level and the live `vS`:
   - OUT: `f = vS > 0 ? (HIGH ? 1 - min(0.5, 0.1/vS) : min(0.5, 0.1/vS)) : 0`,
     clamped to [0, 1]. It stamps `f/20` S from pin 3 to pin 8 when
     `f > 1e-12`, and `(1-f)/20` S from pin 3 to pin 1 when
     `f < 1 - 1e-12`. Unloaded, OUT sits at `V(8) - 0.1` HIGH and
     `V(1) + 0.1` LOW, and `f(1-f)vS/20` flows from pin 8 to pin 1 in either
     state (4.9 mA at 5 V): classic's stand-in for bipolar idle draw
     (DI:197-202); R = 20 Ω and the 0.1 V headroom are DI:86-87.
     Classic stamps the resistances `20/f` and `20/(1-f)`; the core stamps
     the conductances, a difference of an ulp (D13).
   - DIS: committed LOW stamps `1/10` S from pin 7 to pin 1; committed HIGH
     stamps nothing (released).
   - A pin on node 0 stamps nothing (classic returns for `node < 0`, and
     tests `pDis >= 0`). An unpowered 555 stamps neither pin.
   - Both enter the source KCL at fixed nodes (W0 point 5), the idle current
     included, so a supply's published current carries the 555's draw.
   - **Re-stamp triggers.** A commit of OUT or DIS marks that driver's
     partition dirty (W1a point 2). Every solve of a partition holding either
     pin recomputes `f` and the power gate from the solved supply voltages,
     or, with a free supply, from the latest fixed-point iterate (W1c
     point 3). Nothing else is cached. The static solution cache keys on
     committed modes, which fix the stamp when both supply nodes are fixed;
     a partition holding a free supply node bypasses the cache.
8. **Element current** (§5.2, DI:252-273). A 555's `elementI` is the
   current delivered out of pin 3: `-(gLo (V3 - V1) + gHi (V3 - V8))`, with
   `gHi = f/20`, `gLo = (1-f)/20` and `f` from the committed OUT level at the
   published `vS`; 0 while unpowered. The DIS current is not reported. It is
   recomputed wherever the source currents are (every resolve, crossing,
   step and horizon clamp), so it follows a moving supply or load:
   `PartitionedSolver` writes it in its global post-pass, beside
   `recompute_source_currents`. The 555 is the first IC with a nonzero
   current. With OUT on node 0 it is still this formula, as classic
   publishes it, although nothing is stamped: a phantom `f·vS/20` (0.245 A
   HIGH at 5 V). No fixture wires OUT to ground.
9. **Load settling** (W1a point 6 and decision 8, amended for the 555).
   1. Every 555 starts at `outHigh = 1`. Classic's carry by id is W2. In each
      frozen pass, a powered 555 evaluates point 3 without writing `outHigh`
      and requests OUT and DIS from the result. A comparator active at the
      settled voltages drives the output; in the hold region the stored
      value does. Because nothing is written, a transient glitch during the
      frozen passes cannot set or reset the latch.
   2. The edge pass writes `outHigh` from point 3 once, on the settled
      network.
   3. Its requests have zero delay and drain at `START_TIME` in the ordinary
      delta cycle. From then on the 555 is ordinary simulation.

   This departs from classic's seed, whose stamp shows the default latch
   whatever the comparators say (the one-solve lag, §5.2 "Stamp: committed
   latch only"; classic publishes 4.803922 V on OUT at load with RST open,
   into 1 kΩ at 5 V). Decision 7 removes that lag, and the W1-0 post-spike
   owner ruling accepted the consequence: a register clocked by a 555 whose
   comparators resolve 0 at load (RST held low, or THR above `vUpper` with
   TRIG above `vLower`) takes no phantom edge in the core, where classic
   takes one (harness registration
   `w1-0-startup-phantom-edge-74hc74-555-rst-low`, widened in W1e from RST
   held low to that case).

   What stays of classic's startup is the power-on HIGH in the hold region.
   A monostable with TRIG high and THR low at load fires one pulse from
   `vc = 0` (0.110406 s in classic, 0.109861 s in the model, for 100 kΩ and
   1 µF). An astable's first HIGH runs from `vc = 0` to `vUpper` (ln 3 in
   place of ln 2). A register clocked by an astable 555 still takes
   classic's phantom rising edge, because TRIG below `vLower` at load
   resolves HIGH, the same as classic's default.
10. **Power loss** (W1c point 9; §5.2 "Unpowered"). While unpowered, a 555
    stamps nothing and reads, writes and requests nothing. `outHigh` holds,
    the slot is `NaN`, the current is 0, and a commit that comes due is held.
    The 555 is a reader of pins 8 and 1, so the re-solve that restores power
    evaluates it, and that first powered evaluation resolves point 3 from the
    held `outHigh`.
11. **Rollback** (W1b point 7). `outHigh` is evaluator state and is in the
    evaluator's snapshot. OUT and DIS, their pending commits, the timing
    partition's segment or plan, its scheduled crossings and the published
    voltages and currents roll back with a failed instant as they do for any
    IC. `bo_engine_set_params` keeps `outHigh`, and a load rebuilds every 555
    at `outHigh = 1`. A zero-delay loop through the 555 (OUT feeding TRIG and
    THR through resistors only, which classic oscillates at its step rate)
    is `-7`, W1c's rule, and the circuit falls back to classic.
12. **Readouts** (§5.2; §10.9).
    - Classic publishes `digitalState[id] = outHigh ? 1 : 0` after every
      accepted solve while powered, the load seed's included, and drops the
      key while unpowered (DI:249-250; `ds` is rebuilt every pass). Its
      `elementI[id]` is computed after the latch update (SE:5723-5724), so
      it pairs the new latch with voltages solved under the previous one.
      Its OUT voltage runs one solve behind the latch. It publishes no
      channel current and no display state.
    - The core's slot holds `outHigh` at every completed instant while the
      555 is powered, and `NaN` otherwise. The trace records each change at
      the crossing instant. OUT's voltage and the element current follow
      the latch at the same instant.
    - The two agree at every quiet instant. They differ on the one classic
      solve after each switch (the 10 ns guard in the worker, the rest of
      the step in headless, a whole step when the split finds no fraction),
      and at the load frame when the comparators resolve 0 at load. With RST
      open, into 1 kΩ at 5 V, classic publishes V(out) 4.803922 V and
      `elementI` -0.2352 A beside `digitalState` 0; the core publishes
      0.098039 V and 9.8039e-5 A.
    - Stat 12 counts 555 evaluations, stat 9 its crossings, stat 13 its
      commits. There are no step breakpoints in W1 (decision 7: W2 derives
      them from event times). W1g's output-sag scan reads the committed OUT
      level; without the lag, the step-counted debounce loses its reason
      (Q-O43).
13. **Harness comparison** (decision 6; gate 2e; S1 criterion 3).
    1. **Digital.** `<id>` is in every 555 fixture's projection, and a
       555-clocked fixture takes its cycle boundary from it
       (`findClockComponentId`, `test/core/equivalence.test.ts:446`). States
       compare per cycle after alignment; absolute times are never compared.
    2. **Period.** The core's switch times come from its trace, at exact
       event times. With `r_k` the k-th 0-to-1 time of `<id>`, every period
       `T_k = r_(k+1) - r_k` for `k >= 1` (the power-on cycle excluded), the
       HIGH and LOW times from the matching 1-to-0 times, and the first HIGH
       from `START_TIME`, each agree with the sidecar to 1e-6 relative.
       Classic's are recorded against the same sidecar and do not gate (D11;
       a probe of the SAP-1 clock astable measured classic +1.58% per
       period).
    3. **Sidecar.** It is model-exact and computed by the sidecar generator,
       never by core code. Take `vU = (2/3) vS`, `vL = (1/3) vS` and
       `vS = V(8) - V(1)`, thresholds relative to pin 1. The capacitor sees
       the Thevenin equivalent `(V∞c, Rc)` of its network with DIS released,
       and `(V∞d, Rd)` with DIS a 10 Ω resistor from pin 7 to pin 1. Both
       include the 1e-12 S shunt on every free node. Then:
       - `t_high = Rc C ln((V∞c - vL) / (V∞c - vU))`,
         `t_low = Rd C ln((vU - V∞d) / (vL - V∞d))`, `T = t_high + t_low`;
       - the first HIGH, from `vc = 0`, is `Rc C ln(V∞c / (V∞c - vU))`;
       - a monostable pulse from start voltage `v0` is
         `Rc C ln((V∞c - v0) / (V∞c - vU))`, with `v0 = 0` at power-on and
         `v0 = V∞d` after a full discharge.

       For the textbook astable (RA from pin 8 to DIS, RB from DIS to the
       capacitor), with shunts dropped: `Rc = RA + RB` and `V∞c = vS`, so
       `t_high = (RA + RB) C ln 2`; `Rd = RB + RA·10/(RA + 10)` and
       `V∞d = vS·10/(RA + 10)`, so `t_low = Rd C ln((2 - 3k)/(1 - 3k))` with
       `k = 10/(RA + 10)`. Dropping the shunts moves the W1e astable
       fixtures' periods by at most 3e-8 relative. It moves a 1 MΩ
       monostable by 8.2e-7, and a re-triggered monostable's `v0` moves its
       pulse by 9.1e-6 (1 MΩ) and 9.1e-5 (100 kΩ), so the generator keeps
       both terms. The 1e-9 V landing nudge moves a half period by 1e-9 V
       over the threshold slope, below 1e-8 relative for these fixtures. ESR
       is 0 without a `catalogUid`.
    4. **Analog.** The engines drift apart in phase (1.58% per period on the
       SAP-1 clock astable), so after the first half period classic's sample instants,
       which the core replays (`test/core/equivalence/stimuli.ts`), no
       longer sample the same phase. In a 555 fixture, the timing network's
       nodes (TRIG, THR, DIS, the capacitor) compare against the sidecar's
       exponential at the core's instants, within W0's tolerance. Everything
       else (OUT and its load, the 555's `elementI`, the supply current, and
       what OUT drives) compares at phase-aligned quiet instants: the
       midpoint of each engine's own k-th HIGH and LOW.
       - The supply current carries the timing network's own branch current
         (the charging current through RA), which follows the capacitor's
         phase: classic's capacitor is up to 60 mV off at its own midpoint,
         30 µA on the SAP-1 clock's 14.9 mA. So a source's current compares
         with the timing resistors' current leaving its + net added back
         (W0 point 5's sign), which leaves what the 555 and its load draw.
         The timing resistors and capacitor are held to the sidecar instead.
       - A 555 the generator cannot model (anything but resistors, the
         capacitor and pins 2, 6 and 7 on its timing network, or RST neither
         on the supply nor on its own RC) gets no sidecar. It compares with
         classic at phase-aligned instants, timing nodes included, with no
         Richardson forgiveness.
       - Under a scripted stimulus (gate 2d), the timing network is held to
         the sidecar at every sample, and a button on TRIG is a trigger the
         sidecar follows. A sample where the engines disagree on a 555's
         level, and agree again at the very next sample, is classic's
         one-solve lag (point 12). It is registered as
         `classic-555-one-solve-lag` for what OUT and DIS reach through
         passives, and for the sources. A disagreement that lasts longer is
         never excused.

**Known gaps (W1e follow-up): closed.** The two W1c gaps, a watched pin whose
supply is free in a stepped partition and a driver on a fixed node whose stamp
lands in a free supply's partition, are W1c points 3 and 4; fixtures 38 and
39 cover them.

## Kernel semantics: rails and sources (W1f)

References are to spec chapter 8, §10.1 and the delta table; classic line
citations are at `2b5faf1`, and "W1f Qn" is a ruling ("Kind codes and
records"). Constants, all from P3:35-38 unless marked new:

| Name | Value | Use |
|---|---|---|
| `RAIL_DELTA` | `1e-6` V | the finite-difference step of the sensitivity |
| `RAIL_RESOLVE_V` | `0.025` V | re-solve a nonlinear hub partition once the rail has moved this far since its solve |
| `RAIL_BUCKET_V` | `0.05` V | the rail band in the static cache key |
| `RAIL_SCAN_V` | `0.02` V | re-queue the readers of hub partitions once the rail has moved this far since the last scan |
| `RAIL_MOVE_EPS` | `1e-12` V | a smaller change of `u` is not a move (P3:468) |
| `RAIL_DRIFT_V` | `1e-5` V (new) | the implied rail motion that schedules a rail event (Rails point 4) |
| `RAIL_CC_RESOLVE_V` | `1e-6` V (new) | the re-solve band while a PSU is in CC with no rail capacitor (Rails point 3) |
| `RAIL_TAU_MAX` | `1e-4` s (new) | the largest battery rail time constant `r·C_rail` the core accepts (Rails point 7) |
| `RAMP_STEP_RATIO` | `0.01` (new) | the longest pulse ramp, relative to the source's event timescale, that is a midpoint step (Sources point 3) |

### Rails

1. **Hub classification** (at load, before partitioning). A **hub
   candidate** is every `battery_pack` and every `bench_psu` with `iLimit >
   0`. A `bench_psu` with `iLimit = 0` is an ideal source like code 2.
   - **Anchors first.** Every `section_anchor` node is fixed at 0 V before
     propagation, as a 0 V source from the datum. So a source whose section
     has no conductive path to the datum is no longer floating: it
     propagates from its anchor (probe Q4: a second battery reads
     −1.1e-11 V at its own `neg` in classic; the core reads 0). A source
     joined to the datum only through resistors is still floating and still
     `-3` (Q-O39 stands; the anchor covers only isolated sections). A loop
     of ideal sources is still `-3`.
   - **At most one hub** (W1 limit, orchestrator lock). Two or more
     candidates: `-3`, "more than one hub rail (W1 limit)".
   - With one candidate, its `pos` node becomes the **rail node** and its
     `neg` node the **return**. `-3` unless all hold:
     - the return is fixed at 0 V: node 0, or an anchor node;
     - the rail node is not node 0, not the return, and not fixed by any
       ideal source (a hub in parallel with an ideal source is a loop
       classic cannot solve either, except through `R`);
     - no ideal source (codes 2, 21 with `iLimit = 0`, 22-25) has a pin on
       the rail node (it would propagate from a moving node);
     - for a battery, `R(soc0)·C_rail < RAIL_TAU_MAX`, where `C_rail` is the
       summed capacitance of the rail capacitors (point 7).
   - The rail node is neither fixed nor free. The union-find skips it as it
     skips fixed nodes, so a device between the rail and a free node puts
     only the free node in a partition, and an IC whose `vcc` is the rail
     joins none of its pins to it (W1c point 2): partitions stay as small as
     on an ideal supply. This is what the hub buys.
   - In exact mode (point 9) the same rules apply, and then the rail node is
     made free.

2. **The rail equation.** Every **hub partition** (one with any stamp that
   reads the rail node: a conductance, an IC driver or pull whose supply is
   the rail, an LED junction, a capacitor, a switch contact) keeps, from its
   latest solve at rail voltage `u_p`: its draw `I_p` (the current flowing
   from the rail node into the partition), its slope `g_p = dI_p/du` and its
   node sensitivities `s_p = dx/du` (P3:437-448). Every **direct device**
   (all of its pins on fixed nodes or the rail, at least one on the rail: a
   resistor or closed switch contact from the rail to a fixed node, an LED
   junction across them, an IC driver on a fixed node stamping into a rail
   supply, the rail node's own `NODE_RSHUNT_G`, the leakage of a rail
   capacitor) is evaluated exactly at any `u`: `I_d(u)`, `g_d(u)`. A rail
   capacitor itself is not a term (point 7). With
   `A0 = Σ_p (I_p − g_p u_p) + Σ_d (I_d(u) − g_d(u) u)` and `G = Σ_p g_p +
   Σ_d g_d(u)`, the rail solves (P3:462-484):
   - battery: `u = (E − r A0) / (1 + r G)` with `E = OCV(soc) * ocvScale`
     and `r = R(soc)` (point 6); `r = 0` gives `u = E`;
   - bench_psu in CV: `u = V`;
   - bench_psu in CC: `u = (dir * iLimit − A0) / G`, `dir = V < 0 ? −1 : 1`
     (SO:228). `G >= 1e-12` always, from the rail node's own shunt. With a
     rail capacitor the CC rail is a closed form instead (point 4).

   The direct devices are iterated inside the solve (a scalar Newton on
   `u`, to `|Δu| <= 1e-12`, at most 50 iterations, else `-4`); they cost no
   partition solve. `A0` and `G`'s partition terms are running sums updated
   per contribution, with a full recompute every 4096 updates and after any
   restore (P3:450-460, 465).

3. **Sensitivity, publication and re-solve.** After a hub partition solves
   (static, a closed-form segment, a stepped commit), its `(I_p, g_p, s_p)`
   come from three residual stamps and one re-solve (P3:437-448): residuals
   `F0 = r(x, u)` and `F1 = r(x, u + RAIL_DELTA)`; `s = −A⁻¹ (F1 − F0) /
   RAIL_DELTA` through the partition's existing factors (`lu_apply`); `I1 =
   I(x + s·RAIL_DELTA, u + RAIL_DELTA)`; `g = (I1 − I0) / RAIL_DELTA`. For a
   Newton partition `A` is the last iteration's Jacobian. Stat 16 counts
   each computation.
   - A hub node's published voltage is `x + s (u − u_p)` (P3:486-506); the
     rail node's is `u`. A closed-form hub partition publishes its segment
     re-aimed at `u` instead, with `s = 0` (P3:558-569).
   - After every rail solve, a **nonlinear** hub partition (one holding a
     stamped LED junction or a free IC supply) whose `|u − u_p|` exceeds
     `RAIL_RESOLVE_V` (`RAIL_CC_RESOLVE_V` while the hub PSU is in CC with
     no rail capacitor) is re-solved at the new `u`, and the rail solves
     again. The loop runs inside one `resolve_dirty` call until no partition
     is past its band, at most 50 passes (then `-4`, as a static Newton that
     does not converge). A linear partition, and a partition whose draw
     depends on `u` only through an IC driver's `R_out`, keeps its
     first-order correction (pec3's rule; its second-order error is about
     20 µV on the SAP-1, P3:36). CC needs the tighter band because `u` there
     is set by the draws alone: a 25 mV linearisation error on an LED is a
     14% current error.
   - **The 25 mV band is measured** (W1f Q6). F1 runs an LED straight across
     a 0.9 Ω battery, with no resistor, in default and in exact mode and
     records the difference here. If it is outside W0's tolerance (volts
     100 µV plus 1e-3 relative, amps 1 nA plus 1e-3 relative),
     `RAIL_RESOLVE_V` becomes 1 mV.
   - **Rail moves.** A solve that changes `u` by more than `RAIL_MOVE_EPS`
     is a move (stat 14). On a move (P3:469-483):
     - every closed-form hub partition, and every closed-form partition
       holding a watched pin of an IC whose `vcc` or `gnd` is the rail, is
       re-aimed at the current `u` from its current `vc` (P3:570-574), which
       reschedules its crossings;
     - rule 2a capacitors and LED junctions across the rail and a fixed node
       re-aim (point 7);
     - when `|u − u_scan| > RAIL_SCAN_V`: `u_scan = u` (stat 17); every
       nonlinear static hub partition past `RAIL_RESOLVE_V` is marked dirty;
       every reader of a watched node of the other static hub partitions,
       and every IC whose `vcc` or `gnd` is the rail, is queued for this
       delta. A 555 whose supply is the rail is queued too. Each queued
       reader whose logic-pin reads differ from its previous powered
       evaluation's counts once in stat 20: that flip may be up to
       `RAIL_SCAN_V` of rail late (Equivalence harness rules, the reader
       scan).
   - **Static cache.** A static hub partition's key includes the band
     `round(u / RAIL_BUCKET_V)`; an entry stores `x`, `s`, `I_p`, `g_p` and
     `u_p`, and a hit is taken only when `|u − u_p| <= RAIL_RESOLVE_V`
     (P3:507-531). A hit replays the contribution and publishes `x + s (u −
     u_p)`. So cache on and cache off agree within the rail-correction bound
     on hub fixtures, as row B1's verify expects.

4. **Rail events** (W1f Q1; pec3 has none). Between events the static draws
   are constant, but a closed-form hub partition's draw follows its
   capacitor, and SoC moves `E` and `r`. So after every rail solve at `t0`,
   the rail holds its **implied trajectory** `u*(t)`: the equation of point
   2 with each closed-form hub contribution at `I_p(t) = a_p + b_p vc_p(t) +
   g_p (u − u_p)` (its segment's `vc_p(t)`, P3:575-579) and the battery's
   `E(t)`, `r(t)` from the held discharge (point 6). One `Rail` event is
   pending at a time, versioned; every rail solve supersedes it. It is due
   at the earliest `t > t0 + 1e-15` at which any of these is reached:
   - **drift:** `|u*(t) − u(t0)| >= RAIL_DRIFT_V`;
   - **power window:** `u*(t)` crosses `V(gnd) + vccMin*0.9` or `V(gnd) +
     vccMax*1.1` of an IC (555 included) whose `vcc` is the rail and whose
     `gnd` is fixed, computed as W1c point 1 computes them;
   - **CV entry** (PSU in CV): the implied draw `I*(t) = A0(t) + G V`
     reaches `|I*| > iLimit`;
   - **CC exit** (PSU in CC): `dir·u*(t) >= 0.98 |V|` and `dir·(A0(t) + G V)
     <= iLimit` both hold (point 5);
   - **SoC:** `soc` reaches the next OCV-table knot below it, or 0.

   **Locating the time.** Each threshold is a level of one scalar function
   of `t`: `u*(t)` in volts, or the draw `I*(t)` in amps (CV entry, the
   exit's current test). For each function, each moving term (a closed-form
   contribution, the SoC) gets an equal share of the smallest distance from
   its value at `t0` to that function's nearest threshold, capped at
   `RAIL_DRIFT_V` for `u*`. A term's time to use its share is closed form:
   a log for an exponential segment (P3:580-608's form), linear for SoC
   within a table segment. The event is the earliest of these times. With
   one moving term the event lands on the threshold exactly; with several,
   successive events close on it geometrically, and a distance below
   `1e-12 * max(1, |value|)` counts as reached. At a `Rail` event (stat 18):
   SoC advances to `t`, every closed-form hub contribution is refreshed at
   its `vc(t)`, the rail solves (point 2), the regime rules run (point 5),
   and a new event is scheduled. A stepped hub partition contributes afresh
   at each committed step, so the rail solves at its step events; a
   threshold it crosses within a step is found at that step's end, as
   classic finds it (a step granularity, not an exact time).

   So the rail is exact at events and never further than `RAIL_DRIFT_V` from
   its implied value between them, the power gate flips at the exact
   instant the rail crosses it (a digital fact, D1), and CC entry and exit
   are exact state transitions (Q-O36). Cost on the SAP-1 battery variant:
   about 75 drift events per clock half-period from the clock 555's timing
   network (a 1 mA swing through 0.9 Ω); each re-aims three closed-form
   partitions and solves a scalar.

   **The CC rail with a rail capacitor** (W1f Q2). In CC a rail capacitance
   `C_rail` is the whole dynamics: a PSU limiting into a capacitor ramps it
   at `iLimit/C` (probe P9). So while a PSU with `C_rail > 0` is in CC, the
   rail is a closed form of its own, `C_rail du/dt = dir·iLimit − (A0 + G
   u)`: `u(t) = u∞ + (u(t0) − u∞) e^(−G (t − t0) / C_rail)` with `u∞ = (dir·
   iLimit − A0) / G`, and the linear ramp `u(t0) + (dir·iLimit − A0 − G
   u(t0)) (t − t0) / C_rail` while `G (t − t0) / C_rail < 1e-9`. It starts
   from `u = V` at CC entry (continuous), is published live, and each rail
   capacitor carries `C·du/dt`. Its own motion adds four thresholds to the
   rail event: `u_scan ± RAIL_SCAN_V`, the nearest nonlinear hub
   partition's `u_p ± RAIL_RESOLVE_V` (the CC band `RAIL_CC_RESOLVE_V` does
   not apply here: the capacitor, not the draws, sets `u`), `RAIL_DRIFT_V`
   of motion since the last re-aim when a closed-form hub partition exists,
   and the exit (point 5). Each is a log (or a line) on the rail's own
   segment.

5. **Bench PSU regimes** (Q-O36: classic's state machine, exact; entry at a
   current crossing; the hub tolerance applies to voltages only). The state
   is `reg`: 0 (CV) or 2 (CC), starting at 0 at load. The rules run at
   every rail solve (so at load, at every instant that touches the rail,
   and at every rail event), at most one transition per PSU per solve:
   - **CV → CC** when `|I| > iLimit`, strictly (SO:319), where `I = A0 + G V`
     is the draw at the setpoint, unless the CC solution would have
     `dir·u_CC >= max(0, |V|) − (1e-9 + 1e-6 max(1, |V|))`, which keeps CV
     (the compliance clamp, SE:1793-1815). The instant is the event where
     the draw first exceeds the limit: a commit, a toggle, load, or a rail
     event at the crossing. The rail then solves in CC at the same instant.
   - **CC → CV** when `dir·u >= 0.98 |V|` (the unscaled setpoint, SO:294,
     315) **and** the draw at the setpoint `dir·(A0 + G V) <= iLimit` (W1f
     Q7). With a rail capacitor, a jump to `V` would draw without bound, so
     the exit needs `dir·u >= |V| − (1e-9 + 1e-6 max(1, |V|))` instead
     (probe P9: classic stays in CC, −0.1 A, up to 5.0 V). The rail then
     solves in CV at the same instant.
   - In CC the PSU publishes `elementI = −dir·iLimit` exactly; in CV, `−I`.
   - The rules are pure functions (`devices/sources.rs`): `regime_step(reg,
     i_at_v, u_cc, u_rail, V, iLimit)` and `regime_step_rail_cap` with the
     same arguments. `u_cc` is the rail voltage of the CC solution, which the
     CV → CC clamp tests (in CV the rail is `V`, so the caller computes the
     CC candidate; passing the CV rail would make the clamp keep CV
     forever); `u_rail` is the present rail, which the CC → CV exit tests.
     An `iLimit` that is not `> 0` has no regime.

   The second condition on exit is not in classic's commit, but it gives
   classic's electrical trajectory. Classic's commit exits at 98% (reg 0),
   and its next solve re-enters CC inside the solve (the entry clamp,
   SE:1778-1791) while the CV current still exceeds the limit, so its `reg`
   alternates 0, 2, 0, ... every step while the solution stays CC (probe P7:
   12 `reg` changes in 600 steps, current −0.1 A throughout, CV only once
   the CV current falls to 0.1 A at `vc = 4.0 V`). A literal copy would
   re-enter at the exit instant forever (`-7`). Known divergence
   (`w1f-cc-exit-reg`, Equivalence harness rules): the core's `reg` reads 2
   through that window where classic flickers. Only `reg` differs: the
   settled output voltage and current match classic within the ordinary
   criterion there. Classic's in-solve entry tolerance (`iLimit + 1e-12 +
   1e-6 iLimit`) is not reproduced: in the band between it and `iLimit`
   classic enters one step late, the core at once. Scale is 1 (Q-O40): the
   setpoint is never scaled.

6. **Battery chemistry and SoC** (§8.3.2, D16, B2; `battery-physics.ts`,
   not pec3's fixed `E` and `R`, P3:159).
   - `OCV(soc)`: the table of the record, interpolated as BP:384-403.
   - `R(soc) = (rBase * dep) * tempFactor`, `dep = 1 + depA * (1 − soc)^depP
     / (soc + depK)`, in classic's order (BP:417-418, 436). `soc` is clamped
     to `[0, 1]` first. With `depP = 2` the power is `x * x`, as V8's
     `Math.pow(x, 2)`.
   - `E = OCV(soc) * ocvScale`, `r = R(soc)`. `rBase = 0` makes `r = 0`
     for every `soc`: an ideal rail at `u = E` that still drains.
   - **Coulomb counting per event interval** (orchestrator lock; W1f Q9).
     SoC is advanced from `t_last` to `t` at every rail solve and every rail
     event, and only there. A read (device state, the published `u`)
     projects it to the current time without committing it, so reading
     never changes the simulation, and the host's choice of `t_end` changes
     nothing but the horizon clamp's own steps. The charge `Q` delivered out
     of `pos` over `[t_last, t]` is the exact integral of the implied rail
     current: static and direct terms held (exact between events), each
     closed-form hub term integrated in closed form (`(a + b·inf)·Δt +
     b·(vc0 − inf)·(e^(FΔt) − 1)/F`, and `(a + b vc0) Δt` when `F = 0`),
     each stepped term held from its last commit (one step, like classic).
     Then, exactly as `advancePrimaryBatteryState` (BP:447-482) with `Q` in
     place of `I·dt`: `requested = max(0, Q)`, `rejected += max(0, −Q)`,
     `accepted = min(requested, capacityAh·3600·soc)`, `soc' = clampSoc(soc
     − accepted / (capacityAh·3600))`, `discharged += accepted`.
   - Classic counts `max(0, I_end)·h` per accepted step, with the step's
     start SoC in the stamp (SO:146-177): an explicit rule that drains too
     fast by O(h). Probe P4 (a 0.1 mAh cell into 100 Ω, 24% drained in 2 s
     at h = 1e-3): classic's SoC is 1.44e-5 below the exact ODE, 72 µV of
     OCV; the core's integral is closer. At 1 Ah and the SAP-1's draw the
     difference is below 1 µV. The harness treats it as a
     closer-than-classic case (Equivalence harness rules).
   - **Load.** No time passes at load (owner decision 8): SoC stays `soc0`
     and `discharged` stays 0, where classic's 1 ps seed commits `I·1e-12`
     (probe P1: `4.587e-13` C). Known divergence, far below every tolerance.
   - **Empty.** At `soc = 0` the OCV is `v_0` (0 V for every profile) and
     `R` is `rBase·(1 + depA/depK)·tempFactor` (11x for generic): the rail
     collapses as in classic. Further discharge is simply not accepted
     (BP:459-465).
   - Temperature is fixed per load; battery self-heating is a non-goal
     (BP:40-42).

7. **Live fixed nodes** (W1b rule 2a noted "no fixed-node voltage is live
   in W1b"). From W1f a fixed node's voltage can change: the rail node at
   every move, and a waveform source's nodes at every edge or through a
   resolved ramp (Sources point 3).
   - Every stamp reads a non-free node through one live boundary array, not
     a value baked at build. A change re-runs the fixed-node propagation for
     the sources it touches (a `voltage_source` stacked on a clock node moves
     with it) and marks dirty every partition that reads a changed node.
     Closed-form and stepped partitions catch up to the instant with the old
     boundary first (W1b rule 6). The W0 source KCL re-publishes.
   - **Rule 2a across a moving node** (W1f Q2). A capacitor whose two pins
     are on fixed or rail nodes re-aims at each change: `vc` is continuous
     and the new `dV` sets its target (`esr > 0`: the exponential with
     `esr·C`; `esr = 0`: `vc = dV` at once, current 0, W1b rule 2a's known
     divergence). An LED junction across such nodes re-publishes `I(dV)`.
     A **rail capacitor** (one pin on the rail, the other on a fixed node)
     is this case too in CV and on a battery, with `esr` of any value (two
     corpus circuits carry `cap-ceramic` parts with `esr` 0.05 across a
     PSU): its current enters the published source current, as rule 2a's
     does, but not the rail equation, and the charge it moves is not counted
     in SoC. In CV, and on a battery with `r = 0`, this is exact. On a
     battery with `r > 0` the rail's own `r·C_rail` time constant is not
     modelled (90 ns for 0.9 Ω and 100 nF); above `RAIL_TAU_MAX` it would
     show at quiet instants, so the load is `-3` (point 1). In CC the rail
     capacitance makes the rail a closed form (point 4), with each
     capacitor's `esr` ignored there (5 ns for the corpus parts).
   - **Watches.** A change re-aims every watch whose thresholds read the
     changed node, as a moving static supply does (W1c point 4,
     `add_watch_dependent_partitions`).
   - **Static cache.** The key of a static partition that reads a waveform
     source's node includes that source's level index (0 or 1), as a driver
     mode is included, so a partition fed by a clock hits its cache on the
     second cycle instead of missing on every edge.

8. **Rollback.** A failed instant (`-4`, `-7`) restores, with everything W1b
   rule 7 names: `u`, `u_scan`, every hub contribution and the running sums,
   `reg`, the battery state (`soc`, `t_last`, the held discharge,
   `discharged`, `rejected`), the live boundary array, each waveform
   source's edge counter and level, the pending `Rail` and `Source` events,
   and the device-state buffer.

9. **Exact mode** (`NO_RAIL_SHORTCUT = 1`, load-time only, W1f Q10;
   orchestrator lock: "merges hub-coupled partitions with the rail node as
   an unknown"; kept as locked, W1f Q3).
   - The rail node is a free node. Every partition that reads it, and every
     direct device, joins one partition with it, classified by the usual
     rules (on the SAP-1 it holds nearly the whole board and is stepped).
   - The source stamps in that partition: a battery with `r > 0` as a
     Norton, `1/r` from the rail to the return plus `E/r` injected, with `E`
     and `r` held per step from the SoC at the step's start (classic's
     cadence, SO:141-156); a battery with `r = 0` and a PSU in CV replace
     the rail row by `u = E` or `u = V`; a PSU in CC keeps the KCL row and
     injects `dir·iLimit`. The source current is the Norton branch current,
     or the replaced row's KCL residual.
   - No sensitivity, band, scan or rail event: the partition solves the
     rail with everything else. SoC advances per committed step with the
     committed current; the regime rules of point 5 run per committed step
     on the solved `u` and draw.
   - Cost: the merged partition is a dense Newton over the board, the same
     class of work as classic's global Newton (83-112 s CPU per simulated
     second on the SAP-1, build log 2026-09-30) or worse, since every commit
     instant lands a step. So S2 compares against it over a declared window,
     the first 20 clock cycles (Equivalence harness rules), and the rail
     fixtures cover the rest. F1 measures its cost on that window and
     records it here.

### Sources

1. **Identity rules** are the compiler's (Battery record): explicit uid
   only, catalog defaults only with an exact uid, the generic fallback
   otherwise, no legacy alias, the 21 °C reference of the exact profiles
   kept (at 25 °C `supply-5v` is 0.8041 Ω, probe P3). Nothing in Rust
   depends on identity.

2. **Clock edges** (§8.5; decision 6, Q-O34: exact in every host, since the
   core owns time). The level at any `t` is classic's `clockVoltage(t,
   1/freq, duty, vHigh, vLow, delay)` (WF:12-23): `vLow` before `delay`,
   then `vHigh` while `((t − delay) % P) / P < duty`. The core never samples
   it between edges; it schedules them:
   - `duty >= 1`: `vHigh` from `delay` on, one edge at `delay` if `delay >
     START_TIME`. `duty <= 0`: `vLow` always, no edge.
   - Otherwise rising edges at `delay + n P` and falling edges at `delay +
     n P + duty·P`, for every integer `n` with the edge time `> START_TIME`
     and `>= delay` (a negative `delay` is a phase shift, WF:20-21). Each
     time is computed from `n`, never accumulated.
   - At an edge event the level is the edge's own (HIGH after a rising
     edge), never `clockVoltage` of the edge time: fmod can put an edge
     instant on the wrong side (probe P8: `(0.011 % 0.001) / 0.001 =
     0.99999...`, so classic reads LOW at t = 11 ms, where the rising edge
     is due). The two agree everywhere except on edge instants themselves.
   - The level at load is the level after the last edge whose time (the
     same expression as the schedule) is `<= START_TIME`, and `vLow` when
     there is none. So every edge is either in the load level or scheduled,
     never both and never neither. This is `clockVoltage(START_TIME)`
     except when `START_TIME` is itself an edge instant, where fmod can read
     the pre-edge level: a literal `clockVoltage` there would lose the edge
     (at 1 kHz, 1,467 of the 2,999 start times `m · 1e-3` sit on a rising
     edge that reads LOW, and the clock stayed LOW for its whole first HIGH
     half-period). Classic reads that level for one solve and takes the
     edge on its next; the core takes it at `START_TIME` (a register it
     clocks sees the phantom edge at load instead of a real one a step
     later: same count, the instant within the edge-instant rule).
   - The test is on the computed edge time, not the decimal one. A
     `START_TIME` that the computed `delay + n P` overshoots by an ulp
     loads the pre-edge level and takes the edge one ulp later (399 of the
     2,999 starts `m / 1000` at 1 kHz); either way the edge happens once.
     Edges apply by index, `[0, k)` for the first `k` whose time is `>
     START_TIME`, which is "every edge at or before" because the times
     never fall with `k`: a falling edge's time is `min(delay + n P +
     duty·P, delay + (n + 1) P)`, both terms computed from `n`. Without the
     `min`, a `duty` with `(1 − duty)·P` under an ulp of the time (1 kHz,
     `duty` 0.9999999999999999, `t` 0.019: the fall computes to
     0.019000000000000003, after the next rise at 0.019) would put a fall
     after its successor. Clamped, the fall shares the rise's instant and
     keeps its place before it in the schedule, so the clock reads HIGH
     there, as classic's `clockVoltage` does everywhere but on that one
     instant.
   - Past `2^52` periods (`MAX_INDEX`), where edge times no longer resolve,
     the level at load is `clockVoltage(START_TIME)` and no edge is
     scheduled, as for a pulse past its search range.
   - At an edge (stat 19) the source's node takes the new value (Rails
     point 7), the slot updates, and the next edge is scheduled.

3. **Pulse edges and ramps** (§8.6; W1f Q5). `pulseVoltage` (WF:37-57) is
   piecewise linear. Each ramp, the rise `tr` and the fall `tf` separately,
   is either a midpoint step or resolved, decided at load from the params
   against the source's **event timescale** `T_ev`, the shortest flat
   segment of its cycle: `min(pw, per − tr − pw − tf)` when periodic, `pw`
   when one-shot.
   - **Threshold.** A ramp of length `t_r` is a midpoint step when `t_r <=
     RAMP_STEP_RATIO · T_ev`, and resolved otherwise. A ramp of length 0 is
     a step at its instant whatever `T_ev`. The catalog's `pulse-source`
     defaults (1 µs ramps, `pw` 500 µs, `per` 1 ms: 0.002 of `T_ev`) and
     every corpus pulse source (1 µs ramps, `pw` 30 ms or more) are steps; a
     user-authored 1 ms rise with `pw` 4 ms and `per` 10 ms (0.25 of `T_ev`)
     is resolved.
   - **Why this bound.** Below it a reader on the node flips at most
     `t_r / 2 <= 0.005 T_ev` from its exact instant, inside the same flat
     segment, so no per-cycle state changes. An RC load of time constant τ
     differs after the ramp from its exact response by `(t_r/τ)² / 24` of
     the swing to leading order, decaying with τ: below 1e-3 of the swing
     for `τ >= 6.5 t_r`, and a faster load has settled within a few τ, long
     before the segment's quiet instants. Classic's headless host never
     resolves such a ramp either (`per/10` steps, HL:150-153).
   - **Midpoint steps.** Per cycle `c` (only `c = 0` when one-shot) from
     `td + c·per`: `v1 → v2` at `+ tr/2` (`+ 0` when `tr = 0`), `v2 → v1`
     at `+ tr + pw + tf/2` (`+ tr + pw` when `tf = 0`); `v1` before `td`.
     Each time is `(td + c·per) + offset`, the offset independent of `c`
     (`tr/2`, `tr`, `(tr + pw)`, `(tr + pw) + tf/2`, `(tr + pw) + tf`), so
     never accumulated. Events of one instant keep the schedule order, the
     rise's before the fall's: a triangle (`pw = 0`) puts the rise's end and
     the fall's start on one instant, and a zero-length ramp's step shares
     its instant with the next corner. `v1 = v2`: no edge.
     `pw = 0` with `tr = tf = 0`: no edge (classic never leaves `v1`). The
     digital slot flips at the same instants classic's readout does, the
     ramp midpoints (probe P8: 0 at 0.2 µs and 1 at 0.6 µs of a 1 µs rise).
     The analog value differs only inside a ramp.
   - **Resolved ramps.** A resolved ramp never collapses to a step:
     - the node carries `pulseVoltage`'s exact linear value through the
       ramp. A `Source` event lands on each corner (the ramp's start and
       end) and one on its midpoint (the slot); between the corners the live
       boundary value (Rails point 7) is linear in `t`, read at the instant
       of each stamp and read;
     - every partition that reads the node, directly or through a source
       stacked on it, is classified stepped at load, whatever it holds
       (membership and classification stay fixed at load). It lands a step
       on each corner and restarts there at `POST_EVENT_H` with the BE
       anchor, as after any discontinuity, and each step reads the boundary
       at its own instant;
     - a reader pin on the node itself gets a crossing watch on the ramp:
       the threshold crossing time is linear in `t` and exact (accepted only
       at `t > now + 1e-15`). The digital slot flips at the ramp midpoint,
       as on the step path;
     - a capacitor or LED junction across the node and another fixed node
       (W1b rule 2a) is `-3`: its ramp response is not ported.
   - **Truncated cycles** (`per > 0` and `per < tr + pw + tf`) are declined
     by the compiler (Waveform source records).
   - **Fixtures, one side each** (F4, each against classic with a sidecar):
     - below: `pulse-161`, a pulse at the catalog defaults clocking a
       `74ls161` and driving a 1 kΩ, 1 µF RC, compared per cycle and against
       the RC's sidecar. The corpus's `engine-test-04` and `engine-test-07`
       join this side once W1h ports the `74hc595` and `74hc165` they feed
       (until then they fall back to classic);
     - above: `pulse-ramp-1ms`, `{v1 0, v2 5, td 0, tr 1e-3, tf 1e-3, pw
       4e-3, per 1e-2}` driving the same RC and a `74hc14` input on the
       pulse node, against a sidecar holding the RC's exact trapezoid
       response and the hc14's crossing times on the ramp. It must fail with
       the ramp seeded to collapse to a midpoint step.

4. **Load settling** (owner decision 8, events point 6). The rail joins the
   frozen settle: each settle pass solves the rail after the partitions and
   runs the regime rules, and a rail move over `1e-9` V or a regime change
   counts as a change of the fixed point (P3:746). So a PSU overloaded at
   load starts in CC, as classic's seed latches it (probe P5: `reg 2`, 1 V,
   −0.1 A at load). A clock takes the level after its last edge at or
   before `START_TIME` (point 2); a pulse takes the level of its edge model
   at `START_TIME` (the level after its last event at or before it, or
   `pulseVoltage(START_TIME)` inside a resolved ramp). A register such a
   source clocks sees classic's phantom edge if that level is HIGH. No SoC
   advances. The rail and source events are
   scheduled after settling.

5. **Kernel events** (events point 1). Two solver-owned kinds, dispatched
   like `Bucket` in phase 1 of the delta cycle: `Rail { version }` and
   `Source { device, version }`. Each returns dirty partitions and readers,
   as `bucket_boundary` does; a stale version drops silently.
   `bo_engine_next_event_time` includes them;
   `bo_engine_next_digital_due_time` does not (it stays drivers only).

6. **Failures and warnings.** None of these kinds fails (§8.4.3, §8.7). The
   `missing_flyback` rail-kind rule (`FLYBACK_RAIL_KINDS`, §8.7) is W1g's.
   Small-signal AC stamps are W6.

## Kernel semantics: thermal, failures and warnings (W1g)

References are to spec chapter 9 (§9.1-9.10), §6.7, §7.2.5 and §7.8;
classic line citations are at `2b5faf1`, "classic" is classic with the
0.6.1 fixes, and "W1g Qn" is a ruling ("Kind codes and records").
Constants:

| Name | Value | Use |
|---|---|---|
| `RESISTOR_DAMAGE_THRESHOLD` | `1` | the `resistor_overload` latch (SE:120) |
| `FUSE_I2T_THRESHOLD` | `0.3` | the `fuse_tripped` latch (SE:121) |
| `LED_DAMAGE_THRESHOLD` | `24` | the `led_failed` latch (SE:122) |
| `THERMAL_BAND_C` | `0.01` °C (new, W1g Q4) | the held junction temperature's band (point 4) |
| `OUTPUT_SAG_WINDOW` | `1e-8` s (new, W1g Q2) | how long an `output_sag` must persist in one direction (point 9) |

1. **Scope and ownership** (W1g Q9). A **slow device** is any device W1g
   integrates:
   - every `resistor` with `pMax > 0` or a thermal identity;
   - every `fuse` and `ptc_fuse` (W1g Q3);
   - every `led`, `rgb_led` and `bicolor_led` (damage, and heat for an
     identified `led`);
   - every IC (`output_sag`);
   - from their records on, W1h's `potentiometer`, `trimmer`,
     `resistor_array` and `thermistor`.

   `seg7_cc` and `seg7_ca` never fail (Q-O22), and a `resistor` with `pMax
   <= 0` and no identity is not a slow device.

   All slow state lives in one `slow::SlowState` owned by
   `PartitionedSolver`, with one `Slow` event kind and one post-delta hook
   (points 14 and 17). A latch is a stamp change that only the solver can
   make (point 12), and the solver already holds the device records, the
   junction laws, the published voltages and each dynamic partition's
   trajectory. Classic's state maps (`failureStress`, `failures`, `warnings`,
   `ptcs`, `thermalDevices`, SE:547-577) become per-device entries in
   `SlowState`.

   **Slow physics computes its own currents.** It never reads `elementI`,
   channel currents or the source KCL. Since perf round 1 those are derived
   only when the engine prepares a readout (P1), and the static currents are
   incremental (P3), so no event inside an advance can see them, and a
   whole-circuit current pass per event would undo P3. Each slow device
   computes the currents it needs from node voltages with its own law
   (points 6-10).

2. **Intervals and publishes.** Each integrator is lazy: it holds its value
   at `t0` and what drives it (a rate, or a thermal target). It is closed,
   meaning brought exactly to a later time, only when needed.
   - **Publishes.** A device's inputs change only when a partition it
     touches **publishes** new voltages:
     - a static re-solve (a commit, a toggle, `set_params`, a bucket
       boundary, a W1g stamp change);
     - a closed-form segment start or re-aim;
     - a stepped step commit, the horizon landing included;
     - for a device whose nodes are all fixed (a resistor or LED straight
       across ideal sources), a fixed-node voltage change (W1f's rail moves
       and source edges) and the load.
   - At the end of every completed instant, after the last delta pass
     (point 14), `SlowState::settle(now)` visits the slow devices of every
     partition that published during that instant:
     1. it closes each integrator over `[t0, now]` with what drove it over
        that interval;
     2. it reads the new voltages and computes the new drive;
     3. it schedules the device's next event, if any.
   - What drove it depends on the partition class:
     - **Static and fixed-pair:** the pre-publish drive, constant over the
       interval, so the close is exact.
     - **Closed-form:** the segment's own exponential trajectory, closed in
       closed form (points 3 and 5).
     - **Stepped:** the drive at the step's end voltages, held over the step:
       classic's backward rule (`rate·h` from the accepted step's solution,
       SE:5510-5531, 6029-6054).
   - A readout projects to `now` without closing anything (point 16).
   - Slow state never influences step size: the stepped controller ignores
     it. Classic's `estimateStepError` compares thermal and stress terms
     (§9.3.8); D21 is not ported.

3. **Package thermal state** (§9.2-9.3). Per identified device `(T0, t0,
   P)`, with `Ta = AMBIENT_C` and `T∞ = Ta + P·rThetaJA`.
   - **Static and fixed-pair:** `T(t) = T∞ + (T0 − T∞)·exp(−(t − t0)/tauS)`,
     classic's formula (TP:520-524), evaluated once per interval rather than
     once per step (D16). It is exact for a constant `P`, and splitting an
     interval changes `T` only by rounding.
   - **Closed-form** (resistors only: an LED is never in a closed-form
     partition, LEDs point 5). The resistor voltage is `V(s) = a +
     b·e^{−s/τc}` over the segment (`s = t − t_seg`), so `P(s) = (a² +
     2ab·e^{−s/τc} + b²·e^{−2s/τc}) / R`, and the one-pole has the forced
     solution `T(s) = Ta + (Rθ/R)·a²·(1 − e^{−s/tauS}) + (T(0) − Ta)·e^{−s/tauS}
     + (Rθ/R)·Σ_m c_m·e^{−s/tauS}·(s/tauS)·φ(−δ_m s)`, with `c_1 = 2ab`, `c_2 =
     b²`, `δ_m = m/τc − 1/tauS` and `φ(x) = expm1(x)/x` (`φ(0) = 1`). Each
     `m`-term is `c_m·(e^{−m s/τc} − e^{−s/tauS})/(1 − m·tauS/τc)` rewritten so
     that it never divides by a small difference: at `δ_m = 0` it is the
     resonant form `c_m·(s/tauS)·e^{−s/tauS}`, and near it there is no
     cancellation and no switch-over threshold. This is exact and does not
     depend on when the interval is read or split.
   - **Stepped:** `P` from the step's end voltages, held over the step, with
     the formula above per step: classic's own cadence, since a stepped
     partition steps anyway.
   - **Power** (§9.3.4, SE:5947-6025):
     - `resistor`: `V²/R`, with `R` the stamped resistance (`max(1e-3,
       resistance ?? 1000)`, SE:5958), and `0` once `resistor_overload` has
       latched;
     - `led`: `max(0, V·I(V))` with the junction law at the held temperature
       (point 4), and `0` once `led_failed` has latched or while `burnt`
       (SE:5962-5968).
   - **Readout:** `T(now)`, the one-pole value, not the held temperature,
     and `P` of the current interval.
   - **Shutdown:** evaluated on `T` with classic's hysteresis (TP:526-538).
     With `hasShutdown`, an analytic slow event at the time `T` reaches
     `tripC` (or `restartC` while shut down). No W1 kind has one.
   - `resetFailures` leaves `T` and the shutdown flag alone (Q-O41). At load
     `T0 = Ta`: the core has no carry-forward (point 15).

4. **LED junction coupling** (D19, orchestrator lock: hold and re-arm; W1g
   Q4). Only a plain `led` with a thermal identity couples; every other
   junction stays at `AMBIENT_C` (LEDs point 3).
   - **The held temperature `T_hold`.** Each such junction evaluates the law
     of LEDs point 2 at `T_hold` (`vt`, `vf(T)`, `Is(T)` and `vSat` all at
     `T_hold`) for its stamp, its current, its power and its stress. This is
     what classic does with its committed temperature (SC:365-367,
     SE:5972-5985). At load `T_hold = Ta`.
   - **Static partitions and fixed-pair junctions.** `T_hold` changes only
     at a **thermal boundary**: the instant `T(t)` reaches `T_hold ±
     THERMAL_BAND_C`, a fixed 0.01 °C (W1g Q4). Given the interval's `(T0,
     t0, T∞)` the boundary is analytic, `t_b = t0 + tauS·ln((T0 − T∞) /
     (T_hold ± THERMAL_BAND_C − T∞))`, on the side `T∞` lies; there is none
     while `|T∞ − T_hold| < THERMAL_BAND_C`.
     1. It is scheduled as a slow event (point 14).
     2. At `t_b`: `T_hold := T(t_b)`, the law is recomputed, the device's
        stamps change (point 12), and the partition re-solves at `t_b`.
     3. The new voltages give a new `P` and a new boundary.

     A re-solve for any other reason (a commit, a toggle) does NOT move
     `T_hold`, so a boundary-free re-solve can still hit the solution cache,
     which is cleared only when `T_hold` actually moves.
   - **Stepped partitions.** `T_hold := T` after every committed step,
     classic's one-step lag exactly (SC:365-367 reads the committed
     temperature, which the previous step wrote). This forces no event and
     no re-seed, so a thermal boundary never becomes a discontinuity in a
     stepped partition.
   - **Why 0.01 °C** (the analog tolerance applies):
     - At a fixed junction voltage, `∂ln I/∂T = (0.002 + (vf − v)/T_K) /
       (n·vt)`: 0.039/°C at the rated current, up to about 0.06/°C well
       below it; the `Is` override path gives about −0.05/°C. So a held
       temperature at most 0.01 °C stale moves a junction's current by at
       most 6e-4 relative, inside W0's 1e-3 with room for the Newton
       residual.
     - That is the worst case, an LED straight across a source. With a
       series resistor the sensitivity is about 6e-4/°C (probe F `ledHeat`:
       +1.3% over a 21 °C rise through 150 Ω), and the band error is about
       6e-6.
     - Classic's own lag is one step of heating, 23 µs of `τ = 2 s` on the
       SAP-1.
   - **Cost:** one re-solve per 0.01 °C of drift, about 2 100 over a 21 °C
     warm-up (probe F `ledHeat`), then almost none once `T` settles. An LED
     toggled faster than `τ` ripples below the band (5.5 m°C for a 1 ms
     toggle at 11 °C of rise). Stat 24 counts the re-arms; it is measured on
     the catalogUid SAP-1 (83 LEDs) at W1z.
   - **The junction memo** (perf round 1's `memo_junction_current`, keyed on
     the two terminal voltages' bits). A re-arm changes the law without
     changing the voltages. So each junction carries a `law_gen: u32`,
     bumped at every re-arm, latch and reset, and the memo's key includes it:
     a re-armed law never reuses a stale memo. The law parameters `(vtn, Is,
     vSat)` become per-junction state for identified LEDs; every other
     junction keeps its load-time values.

5. **Stress integrators and analytic threshold events** (D16, D18;
   §9.5.1). Classic's dwell integrator (SE:5510-5531): `S' = rate` when
   `rate > 0`, else `S' = −recovery`, floored at 0; it latches at `S >=
   threshold`, and once latched it does nothing. The per-kind rate, recovery
   and threshold are in points 6-8; Rust computes each rate in classic's
   operation order.
   - **Constant rate** (static, fixed-pair, and within a stepped step): `S(t)
     = S0 + rate·(t − t0)`. With `rate > 0` the latch time `t_x = t0 +
     (threshold − S0)/rate` is scheduled as a slow event. A bleeding
     integrator schedules nothing, and its entry is dropped at the time it
     reaches 0, lazily.
   - **Closed-form segments** (resistors and fuses): `rate(s) = max(0,
     P(s)/pMax − 1)`, or `(I(s)/iRating)² − 1` for a fuse, with `P(s)` the
     squared exponential of point 3. `P` is monotone on at most two pieces,
     split where the device's voltage crosses zero, so `{rate > 0}` is at
     most two intervals and `S(s)` has a closed form on each. `t_x` is found
     by safeguarded Newton on that closed form, monotone wherever `rate > 0`,
     to 1e-12 relative, and scheduled. W1h's multi-segment kinds (the pot's
     two track segments) meet the same requirement with their own pieces.
   - **Stepped partitions** use the step's end rate over the step (point 2).
     A latch falls on the end of the step in which `S` reaches the threshold,
     with `since` that step's end: classic's rule (`since = simTime + h`,
     PA:150; W1g Q11). This is the one place a latch time is quantized to a
     step.
   - **The event is the latch.** At `t_x` the latch happens without
     re-testing `S >= threshold`: rounding can leave `S` one ulp short and
     reschedule forever (the hard-won crossing rule's twin). A publish before
     `t_x` bumps the device's token, which makes the pending event stale, and
     schedules afresh. A `t_x <= now + 1e-15` at scheduling time latches at
     once, inside the current instant (point 14).
   - **`since`** of an integrated kind is its latch time, and `value` is the
     drive at that instant: the power or the current. Neither changes
     afterwards (classic: `if failures.has(key) return`, SE:5518).
   - Classic's integrators are the exact solution of the same ODE sampled
     per step, so the core's trip times are the analytic ones. Probe F,
     classic on the harness host: the resistor at 2× latches at 1.0017965 s
     against 1.0 analytic; the fuse at 2× at 0.1017965 s against 0.1; the
     LED overdrive at 2.7066605 s, a 64 µs step end, against 2.7066528 s. The
     core is held to the analytic time and classic's is recorded (Q-O31).

6. **`resistor_overload`** (PA:95-159; §7.2.5).
   - Rate `max(0, P/pMax − 1)`, recovery `1`, threshold
     `RESISTOR_DAMAGE_THRESHOLD`.
   - `P = I²·rPower` with `I = ΔV/R` (Rating record), and `0` when `rPower
     <= 0`. The rate is `0` when `pMax <= 0`. `rPower` is classic's
     `resistance ?? 1000` (Q-O27, `fix/resistor-default-overload`).
   - Record: `value = P` at the latch, `limit = pMax`, `pin = −1`.
   - Effect: an open (point 12).
   - A resistor at exactly 2× (5 V across 50 Ω, `p_max` 0.25 W) latches at
     `t = 1` (probe F).

7. **`led_failed`** (SC:472-511; §6.7; LEDs point 15, Q-O24).
   - Rate `max(0, r² − 1)` with `r = peakI/ifMax` (`0` when `ifMax <= 0`),
     recovery `2`, threshold `LED_DAMAGE_THRESHOLD`.
   - `ifMax` is `led` param 2 and the `rgb_led` or `bicolor_led` `ifMax`
     param.
   - `peakI` comes from the device's own junction currents at the published
     voltages (the law at `T_hold` for an identified `led`, ambient
     otherwise), GMIN excluded, aggregated as classic does:
     - `led`: `|I|`;
     - `bicolor_led`: `max(|max(I1, I2)|, max(0, I1), max(0, I2))`;
     - `rgb_led`: `max(0, Ir, Ig, Ib)`.

     These are the `elementI` and channel rules of LEDs "Currents" and point
     13, fed through classic's `max(|elementI|, ...|channel|)` (SC:482-491).
   - A `burnt` device has `peakI = 0` (it stamps nothing), so its stress
     only bleeds. `seg7` has no damage surface.
   - Record: `value = peakI` at the latch, `limit = ifMax`, `pin = −1`.
     `params.burnt` is never written (`failure-states.test.ts:192-226`).
   - Effect: point 12.
   - Probe F `ledOver`: a red LED (no uid, `vf` 1.8, `if_max` 0.02) on 50 Ω
     from 5 V carries 62.824 mA, `r = 3.1412`, rate 8.8670/s, and latches at
     2.7066528 s analytic.

8. **`fuse_tripped`** (PA:690-750; §7.8; W1g Q3).
   - Rate `max(0, (I/iRating)² − 1)` with `I = |ΔV|/rNormal`, recovery
     `0.3`, threshold `FUSE_I2T_THRESHOLD`. The rate is `0` when `iRating <=
     0`. An absent `iRating` is classic's `?? 1` (Fuse and PTC records).
   - Record: `value = I` at the latch, `limit = iRating`, `pin = −1`.
   - Effect: an open (point 12).
   - At 2× (1 A through a 0.5 A fuse) the latch is at `t = 0.1` (probe F:
     classic 0.1018 at a 10 ms step end).

9. **`output_sag`** (SE:6274-6365, targets SE:6375-6555; §9.5.2).
   - **Targets.** For a powered IC (W1c point 1), every driver whose stage
     is `Logic` or `Ne555Out`, committed `HIGH` or `LOW`, is a target,
     except an `OPEN_COLLECTOR` pin committed `HIGH` (released). This one
     rule reproduces classic's per-kind table for every W1 and W1h kind:
     - the 555's pin 3 only (DIS is `Ne555Dis`);
     - a 74LS47 segment only while it sinks;
     - disabled 173, 189, EEPROM and 595 outputs, which are committed `HIZ`;
     - the 74LS245's driven side.

     A pin committed `HIZ` or never committed, a released one, or any pin of
     an unpowered IC is not a target, and its state clears. That covers
     classic's unpowered clear (SE:6357, 6367-6373) and its cleanup sweep
     (SE:6345-6355), which since 0.6.1 also clears a released
     open-collector output (Q-O49, `fix/74ls47-released-segment-sag`).
   - **Committed, not live, targets** (W1g Q12: accepted, bounded). Classic
     lists combinational outputs from the live evaluation with the delayed
     level (SE:6396-6400). So inside a pending enable or disable delay it
     can list a pin the core still holds `HIZ`, or drop one the core still
     drives. The difference lasts less than the pending delay, and once the
     delay expires the committed state is identical. A fixture pins it, and
     it is registered (`w1g-sag-committed-target`, Equivalence harness
     rules).
   - **Predicate.** `actual = V(pin) − V(gnd)`, compared with the thresholds
     at the live `vS` (W1c point 1; `digital::power::ic_power`). `HIGH` sags
     when `actual < vih`, `LOW` when `actual > vil`, strictly, as classic
     (SE:6304-6306).
   - **Window** (W1g Q2, Q-O43: `W = OUTPUT_SAG_WINDOW = 1e-8` s, recorded
     here; the owner sees it at W1z). Per `(IC, pin)` the state is `Ok`,
     `Pending{dir, t_s}` or `Latched{dir}`, `dir` the commanded direction.
     Only `Latched` is published. At each scan (below):
     - no sag: `Ok`, which clears a latch;
     - a sag in direction `dir` while `Ok`, or while `Pending` or `Latched`
       in the other direction: `Pending{dir, now}`, with a slow event at
       `now + OUTPUT_SAG_WINDOW`;
     - a sag in direction `dir` while `Pending{dir}` or `Latched{dir}`: no
       change.

     At the event, a current `Pending{dir, t_s}` whose predicate still holds
     in direction `dir` latches. So after a flip the latch is unpublished
     until the new direction has persisted for the window: a latched sag
     whose commanded direction flips while the pin still sags is unpublished
     and re-latches once the new direction has held for `W`, and an output
     that sags in both directions but flips before every window ends never
     latches. That is classic's direction-aware debounce, counted in time
     instead of steps (Direction, below).
     - "Still holds" is read at the event time. Static: the voltages are
       unchanged unless a publish, which re-evaluated, intervened.
       Closed-form: `V(t_s + W)` from the segment, which is monotone within
       a segment, so one reading decides whether the sag lasted the whole
       window. Stepped: the last step's value, and each step is itself a
       publish.
     - The scan runs in `settle` for the ICs whose output or supply
       partitions published this instant: once per instant, after the last
       delta pass, so a mid-delta transient has zero duration and never
       counts.
   - **Why `W = 10 ns`:**
     1. Classic latches at `seen >= OUTPUT_SAG_DEBOUNCE_STEPS = 2`
        observations in one direction (SE:131, 6318-6335, with `7b1056d`):
        two consecutive accepted steps, which are at least `H_MIN = 1e-8`
        apart (HL:88). So classic latches nothing that has not persisted
        across at least 10 ns.
     2. The load seed counts as the first observation (§9.5.2), and
        HeadlessRunner's first step is `H_INITIAL = 1e-8` (HL:90). Classic
        latches a sag present at load at `t = 1e-8` (probe F `sag`: 74LS00
        1Y into 100 Ω, 1.9512 V against `vih` 2.0, latched at 1e-8). The
        core, with `Pending` from `START_TIME`, latches at `START_TIME +
        1e-8`, the same instant.
     3. On capacitively loaded edges, W = 10 ns splits edges as classic does
        wherever classic's integration resolves the edge (probe C, a 74LS04
        into `C`, driven by a 555; run before the debounce fix, and G4's
        fixture re-measures it on fixed classic):
        - 10 nF (crossings 0.80 µs and 2.86 µs): classic latches every edge
          (26 in 20 ms), and so does the core;
        - 100 pF (model crossings 8.0 ns HIGH, 28.6 ns LOW): classic latches
          13 LOW edges and 1 HIGH, the power-on edge under BE; the core
          latches the LOW edges only;
        - 30 pF (LOW crossing 8.6 ns): classic still latches 13 LOW edges,
          because its 10 ns BE step leaves the node at 1.6 V against the
          exact 0.59 V, above `vil`. The core, closer to the model, latches
          none: a Q-O31-style "closer to the analytic answer" case, recorded
          here.
     4. The step count existed to hide the one-solve lag between a committed
        level and the voltage solved under the previous stamp (Q-O19; the
        test comment at `failure-states.test.ts:318-326`). The core has no
        such lag (decision 7: it stamps what it commits), so what W must
        keep is the minimum persistence, not the lag cover.
   - **Direction** (W1g Q1). The window restarts when the commanded
     direction changes, as classic's count does since 0.6.1
     (`fix/output-sag-direction-debounce`, `7b1056d`): the count carries the
     commanded direction, a flip restarts it at 1, and below 2 the failure is
     not published. At `2b5faf1` the count was keyed by pin only, so a
     ripple-counter decoder glitch read "commanded LOW, pin still HIGH" and,
     one step later, "commanded HIGH, pin at the glitch's LOW", and latched
     a false sag for one step: the S1 running board's `output_sag` on
     `U41.y2n` (the draft's section 8). The core never sees that pair, since
     it stamps what it commits. With the fix, the S1 running classic
     reference is rerun and both engines' failure sets are expected empty
     (point 19).
   - **Record**, projected at collect time:
     - `since = now` (classic re-sets `since = simTime + h` every step while
       latched, SE:6329, so at any readout it is the current time);
     - `value = actual` and `limit = vih` or `vil`, both at `now`;
     - `aux` = direction, `pin` = the pin's index.

     A latched sag whose predicate no longer holds at `now` (a closed-form
     segment crossed back since the last publish) is not reported, and its
     state clears at the next publish. The record never alters a stamp
     (SE:6309-6313, comment). `output_sag` stays a failure kind (Q-O44).
   - **Load and reset.** A sag present after load settling is `Pending` from
     `START_TIME`; after `resetFailures`, from the reset instant (classic
     counts both seeds as one observation, D17).

10. **PTC** (PA:751-830; §9.6), classic's separate channel (Q-O45; W1g Q3).
    Per device `(tripped, S, Rt)`, with `rNormal` and `iHold` from the
    record.
    - `I = |ΔV|/R`, with `R = rNormal`, or `rNormal * 200` while tripped.
    - Not tripped, `I >= iHold`: `S' = max(0, I² − iHold²)`, with a trip
      event at `S = 3·iHold·iHold·0.1`, in that operation order (PA:789).
      The trip zeroes `S` and `Rt`.
    - Not tripped, `I < iHold`: `S' = −iHold²·0.5`, floored at 0.
    - Tripped and cool, meaning `I < iHold/2` and `I·I·rNormal·200 <
      iHold·iHold·rNormal·0.1` in classic's operation order (PA:805-807;
      about `I < 0.02236·iHold`): `Rt' = 1`, with a recovery event at `Rt =
      2.0` (PA:809) that clears all three.
    - Tripped and not cool: `Rt = 0`, set at the publish that observes it.
    - In a closed-form segment `I` is monotone, so each condition flips at
      most once per segment, at a root scheduled like a threshold.
    - A trip or a recovery is a stamp change (point 12), never a failure or
      warning record.
    - Recovery starts at the trip instant itself when the tripped current is
      already cool. Probe F `ptc`: 1 A through `iHold` 0.5 A trips at 0.1 s
      (classic 0.1018). At 0.5 V the tripped current is 5 mA, which is
      cool, and it recovers 2.0 s after the trip, not after the later drop.
    - Readout: channel 2 and `bo_engine_device_state`.

11. **Warnings** (§9.7). No Rust producer exists in W1:
    `transistor_overcurrent` is W3, and `missing_flyback` is a static
    topology check (SE:5581-5627) over nets, kinds and the inductance param,
    run at `load()` and `resetFailures()`. So the facade derives it with
    classic's own logic, through a pure helper exported from classic's
    `sim-engine.ts` that `_evaluateStaticWarnings` itself calls
    (behaviour-neutral, on the wave branch; W1g Q7), as
    `thermalProfileForComponent` is exported.
    - `since` is `START_TIME` at load and the current time at a reset.
    - `value` is the inductance for an inductor.
    - The message is classic's (SE:5616).
    - A reload re-latches and re-announces it: no carry-forward in W1
      (point 15).

    Its windings are the inductor (W1h), the relay and the dc_motor (later
    waves), and its clamp kinds include `led` (SE:975-977). Its rail-kind
    rule (`FLYBACK_RAIL_KINDS`, §8.7; Sources point 6) is in the same
    helper. W1g ships the facade path; it fires live once an inductor
    compiles.

12. **What a latch does afterwards** (§9.5.2; SE:5498 `_hasFailure`).
    - **The stamp.** A failed `resistor` or `fuse` stamps nothing: an open,
      not a large resistance (`failure-states.test.ts:69-96`). A failed LED
      device stamps no junction at all. A PTC trip or recovery changes its
      resistance. A thermal re-arm changes one junction's law. `output_sag`
      changes nothing.
    - **Currents.** A failed device's `elementI` is 0 and its channels are
      `NaN`, meaning absent. `check_finite` accepts `NaN` channels on a
      failed device, as on a `burnt` one (LEDs point 13), and `led::readout`
      receives the real failed flag instead of `false`.
    - **Partitions and classification.** Membership and classification are
      fixed at load (W1b point 1, LEDs point 5). An open can split a
      partition into two islands inside one LU. Every node's `NODE_RSHUNT_G`
      keeps it solvable, and an isolated island reads 0 V through the shunt
      (`failure-states.test.ts:69-96` asserts exactly that). A nonlinear
      partition whose last junction failed stays nonlinear; its Newton
      converges in one iteration plus the polish.
    - **One entry point.** Every slow change that alters a stamp (a latch, a
      re-arm, a PTC trip or recovery, a reset) goes through
      `mark_device_changed(device)`, which does what `apply_params` does for
      a toggle, minus the value write:
      - it marks the device's stamps dirty, or sets the static solver's
        `full` flag: the static currents are incremental (P3), and would
        otherwise miss a change that moves a current without moving a node
        voltage;
      - it marks every partition the device touches dirty;
      - it **clears those partitions' solution caches**, since the cache key
        carries no failure or law state, and an entry stored under the
        pre-latch stamps would otherwise be served after a reset;
      - it bumps the junction's `law_gen` (point 4).

      For a device on two fixed nodes (no partition) it recomputes that
      device's current and republishes the source currents at that instant
      (W1c point 8's fixed-node rule).
    - **The instant.** The change applies at the latch instant, and the
      dirty partitions re-solve in a delta pass at that same instant (point
      14). A stepped partition catches up with its pre-change stamps,
      re-seeds and restarts at `POST_EVENT_H` (LEDs point 10).
    - **Classic's latching step.** Classic applies the open from the step
      after the latch, and that step's heat is still computed on the intact
      part (§9.3.3). The core opens at `t_x`, and its power is 0 from `t_x`.
      The two agree as `h → 0`.

13. **`resetFailures`** (SE:2407-2448; §9.5.4).
    - It clears every latch, every stress integrator (the PTC's included),
      every `output_sag` state and window, and every Rust warning.
    - It resets every PTC (`tripped = false`, `S = Rt = 0`).
    - It leaves `T`, `T_hold` and the shutdown flag alone (Q-O41), and the
      latch log (C ABI, `bo_engine_records`).
    - It applies as ONE event at the current time, as `bo_engine_set_params`
      does: `mark_device_changed` for each device whose stamp returns, then
      the delta cycle at that instant. It does not advance time (Q-O28), and
      there is no 1 ps seed.
    - A device still over its rating starts integrating from 0 at the reset
      instant. Probe F `reset`: classic re-latches exactly 1.0 s after the
      reset.
    - **A failed re-solve** (W1g Q6). `-4` or `-7` rolls the reset back with
      the instant, as `set_params` rolls back: the engine is exactly as it
      was before the call. Classic instead keeps its maps cleared and marks
      the seed non-converged (the comment at SE:2421-2431; the seed at
      SE:2439). This is a recorded divergence
      (`w1g-reset-failures-failed-seed`), reachable only when un-failing a
      part makes the circuit unsolvable. A test reaches it and asserts that
      BOTH engines report non-convergence (`lastConverged = false`).

14. **Order within an instant** (amends events point 3).
    1. The delta cycle runs to quiescence as before: commits, re-solves,
       readers.
    2. `settle(now)` (point 2) closes and re-drives the slow devices of every
       partition published this instant, and runs the `output_sag` scan.
    3. It returns the partitions dirtied by any latch due at once (`t_x <=
       now + 1e-15`). If there are any, another delta cycle runs at `now`,
       then `settle` again.

    A device latches at most once per instant, and `output_sag` never
    dirties anything, so this terminates; the 1000-pass cap still applies.
    `on_settled` (digital slots) runs after the last `settle`. Slow events
    (threshold, window expiry, thermal boundary, PTC trip and recovery) are
    one event kind, `EventKind::Slow { device, token }`. Popped with a
    current token, it runs `handle_slow_event`, which closes the device to
    `now`, applies the latch, re-arm or window decision, and returns any
    dirty partitions for that delta. Ties at one instant keep the heap's
    FIFO order (events point 1).

15. **Rollback and load** (amends W1b point 7).
    - **Rollback.** Everything in `SlowState` is instant state and rolls back
      bit for bit with a failed instant (`-4`/`-7` in `advance`,
      `set_params`, `seed_memory` and `reset_failures`): each device's
      integrators, tokens and latch flags (`led_failed` included), its
      `output_sag` state, each identified junction's `T_hold`, law and
      `law_gen`, the PTC state, the latch order and the latch log.
      - It lives in the solver's instant checkpoint (`begin_instant`,
        `end_instant` and `rollback_instant` on `PartitionSolver`), as a
        journal of first-touch copies per instant, the pattern
        `DeviceEvaluator::begin_instant` documents. The common instant
        touches no slow entry (on the uid-less SAP-1 every resistor and LED
        sits at rate 0), so a completed instant pays nothing.
      - The slow events a failed instant scheduled or popped roll back with
        the kernel heap, as crossings do.
      - A rollback leaves the static solver's `full` flag set (perf round
        2's rule), so the incremental currents recompute after a restored
        latch.
    - **Load** (W1g Q8). A load starts every slow device fresh: `T = T_hold =
      Ta`, no stress, no latch, every PTC untripped, an empty latch log.
      Classic carries failures, stress, warnings, PTC and thermal state
      across `load()` by id and kind (SE:2690-2733, 2829-2850). That is W2's
      carry-forward (D25; Plan W1z "Deferred to W2"), and the gate 1 cases
      that reload mid-failure are classified `W2`.

16. **Readouts and the derived currents.** `bo_engine_records` and the
    thermal and PTC device state are computed on demand from `SlowState` and
    the published voltages, projected to `now` without committing anything.
    The solver's derived readouts (source currents, LED `elementI` and
    channels, prepared once at the end of `load`, `advance`, `set_params`,
    `seed_memory` and now `reset_failures`, P1) read `SlowState` only for
    the failed flags and the PTC resistance. Nothing flows the other way
    (point 1).

17. **What W1g needs from the perf-wave and W1f files.** Kept to these
    hooks; everything else is in `slow/*`, `digital/sag.rs` and
    `devices/{passives,led}.rs`.
    - `events.rs`: `EventKind::Slow { device: DeviceId, token: u32 }`.
    - `kernel.rs`:
      - the `Slow` arm in `apply_due_events`: if `solver.slow_is_current(
        device, token)`, extend `dirty` with `solver.handle_slow_event(device,
        time)?`;
      - after `run_deltas` quiesces, point 14's loop around
        `solver.slow_settle(time, &drivers)`;
      - `reset_failures_as_event`, a copy of `apply_params_as_event`'s
        envelope around `solver.reset_failures()`.
    - `solve/mod.rs` (`PartitionSolver`, all default no-ops, so the kernel's
      mock solvers compile unchanged): `slow_settle`, `handle_slow_event`,
      `slow_is_current`, `reset_failures`, `collect_records(channel, out)`
      and `slow_device_state(device, out)`.
      - `PartitionedSolver` owns `slow: SlowState`, pushes each publishing
        partition onto a `published: Vec<PartitionId>` that `slow_settle`
        drains, and carries `SlowState` in its instant checkpoint.
      - It passes each LED device's failed flag to `led::readout`.
      - Every stamp path reads the failed flags and the PTC resistance
        through one accessor: the static stamps, `stamp_linear_base` (closed
        form and stepped), the junction stamps and the KCL walk.
    - `solve/static_part.rs`: `mark_device_changed(device)` (point 12) and
      the memo's `law_gen` key (point 4).
    - `solve/closed_form.rs`: `node_trajectory(node) -> Option<(v_inf, v0,
      t0, tau)>` for the current segment (points 3, 5 and 9).
    - `solve/stepped.rs`: one setter for a junction's law and stamped flag,
      because a stepped partition owns its `DynShape` (`T_hold` after a step,
      a failed junction); nothing else, since steps publish through
      `handle_dynamic_event`.
    - `partition.rs`: `fuse` and `ptc_fuse` join their two free pins, as a
      resistor does (W0 point 3).
    - `devices/mod.rs`: the `fuse` and `ptc_fuse` arms of `stamp` and
      `current`.
    - `engine.rs`, `abi.rs`, `program.rs`, `kinds.rs`, `readout.rs`: the
      three exports, codes 10, 13, 90 and 91, and the failed-device `NaN`
      rule in `check_finite`. `scripts/build-core.mjs` lists the three
      exports.
    - `devices/ics.rs` and the rest of `digital/*`: none edited. The sag scan
      (`digital/sag.rs`) reads the kernel's drivers, `ic_powered` and
      `digital::power::ic_power`.

18. **Stats** (C ABI): `21` slow interval closes, one device's integrators
    brought to a publish instant; `22` slow events fired (threshold, window
    expiry, thermal boundary, PTC trip and recovery), current tokens only;
    `23` latches (failures, `output_sag` latches, PTC trips and recoveries);
    `24` junction re-arms, a thermal LED's law recomputed at a new held
    temperature. They keep failed-attempt counts, as the others do.

19. **Harness comparison.**
    1. **Gate 2c** (Plan 3.4.2). Each fixture first shows the failure firing
       on classic, with its firing time recorded. Then the core's trip time
       is held to the sidecar's analytic time within `1e-9 s + 1e-9`
       relative, with classic's recorded (Q-O31). The classic values are
       probe F's, at `2b5faf1`; none of these cases moves with the 0.6.1
       fixes.
       - resistor at 2× `p_max`: analytic 1.0 s; classic 1.0018 s;
       - fuse at 2× (1 A, `iRating` 0.5): analytic 0.1 s; classic 0.1018 s;
       - PTC at 2× hold (1 A, `iHold` 0.5): trip analytic 0.1 s, classic
         0.1018 s; at 0.5 V, recovery 2.0 s after the trip;
       - LED overdrive: analytic 2.7066528 s; classic 2.7066605 s;
       - 74LS00 1Y into 100 Ω: classic and core both latch at 1e-8, value
         1.9512195 V, limit 2.0;
       - the capacitive edge (probe C at 100 pF): LOW edges latch, HIGH edges
         do not;
       - battery rail out of the power window, and bench_psu CC entry and
         exit: W1f's fixtures, gated here on the sag clearing and returning
         with power;
       - self-heating resistor with a uid: `T(4 s) = 110 − 85·e^{−0.5} =
         58.44489392442616`, classic within 5.5e-12;
       - self-heating LED with a uid, 150 Ω from 5 V: classic 21.3117 mA at
         1 ms, 21.5875 mA and 46.1619 °C at 10 s. The sidecar integrates the
         model's own ODE, since the coupled rise has no closed form;
       - `resetFailures`: re-latch 1.0 s after the reset;
       - the failed reset (point 13): both engines report non-convergence;
       - the committed-target case (point 9): an enable or a disable delay
         into a sagging load, where the sets differ only inside the delay.

       A failure set compares `(componentId, kind, pinId)` exactly,
       `value`/`limit` within decision 6's tolerance, and `message` against
       classic's template applied to the core's own numbers. Temperatures
       compare within 1e-3 of the rise above ambient plus 1e-3 °C.
    2. **The S1 running item** (manager ruling: must pass by W1z; W1g Q1).
       Classic's `output_sag` on `U41.y2n` was its pin-keyed debounce (point
       9, Direction). With fixed classic the S1 running classic reference is
       rerun (the cache key moves with the classic commit), and both
       engines' failure sets are expected empty. Criterion 5 polls classic
       at every accepted step (`4d6fdb8`) and reads the core's latch log at
       every advance end (Equivalence harness rules, W1g).
    3. **Gate 1 joins** (classification rows):
       - `led-damage-limit-irated` (both cases; manager ruling);
       - `failure-states`: the resistor overload, fuse, LED and both
         `output_sag` cases become portable; the reload-mid-failure case is
         `W2` (W1g Q8); the potentiometer and resistor_array cases wait for
         W1h's records;
       - `thermal-engine-physics`: resistor one-pole, identity-less,
         kind-mismatched uid, LED feedback and failed-open LED become
         portable; the rejected-trial case (`saveState`) is `W2`; the 7805
         and op-amp cases are W3;
       - `catalog-identity` (`led`: its decline was `led-thermal`);
       - `wave3-protection`'s `ptc_fuse` cases (W1g Q3);
       - the 0.6.1 fixes' own tests (`output-sag-direction`,
         `resistor-default-overload`, `thermal-cold-start-warnings`,
         `ls47-released-segment-sag`), each case whose kinds compile;
       - `engine-warnings`' `missing_flyback` cases join as their kinds
         compile (the inductor W1h, the relay and dc_motor later);
         `transistor_overcurrent` is W3.

## Kernel semantics: events (W1a)

1. **Event order.** One heap ordered by `(t, seq)` with `f64::total_cmp`;
   `seq` is a global insertion counter, so events at equal `t` run first in,
   first out. W1f adds two solver-owned kinds, `Rail { version }` and
   `Source { device, version }`, dispatched like `Bucket` in phase 1 of the
   delta cycle (Sources point 5). W1g adds one more, `Slow { device, token
   }`, dispatched in phase 1 too and stale when the device's token has moved
   on (W1g point 14).
2. **Drivers.** A driver's commit (a commit to HIZ included: the driver stops
   driving, as a tri-stated or direction-flipped pin does) and any other change to what it
   stamps all mark its partition dirty for the next re-solve; readers of that
   partition are then re-evaluated. Every IC output is a driver with mode
   `HIZ`, `HIGH` or `LOW`. Its stage sets what a committed mode stamps,
   always between the pin and its own IC's supply pin nodes, so load current
   debits the rails (spec §1.7): a logic output (W1c point 8), or a 555's
   OUT or DIS (NE555 point 7). In code, `Driver` gains `stage: DriverStage {
   Logic, Ne555Out, Ne555Dis }` (default `Logic`), and
   `solve::ic_driver_target` becomes `ic_driver_branches`, returning at most
   two `(target, g)` pairs that every caller folds
   (`static_part::stamp_ics_into`, `stamp_ics_into_dyn`,
   `PartitionedSolver::recompute_source_currents`; closed-form and stepped
   partitions reach it only through `stamp_ics_into_dyn`). `hi_level` and
   `lo_level` stay for non-IC drivers: a 555's OUT fraction depends on the
   live `vS`, so it is computed at stamp time from the stage, not written
   there. A requested change goes through the inertial rules
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

   More than 1000 deltas at one instant is `-7`, never a silent exit. From
   W1g, once the cycle quiesces the slow settle runs, and a latch due at
   once starts another cycle at the same instant (W1g point 14).
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
      is finding the consistent initial state, not simulating time. A 555's
      latch is not edge history: in a frozen pass a powered 555 drives its
      comparator result without writing the latch (NE555 point 9). Unlike
      the other level-sensitive inputs named here, this departs from
      classic's seed stamp (decision 7).
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

## Equivalence harness rules

Under scripted stimuli (gate 2d) a scripted circuit has no
closed form, so a classic-versus-core mismatch in an analog readout (`netV`,
`elementI`) can be explained by a Richardson reference R, and only by one of
the two rules below. An explained mismatch is a known-divergence hit under its
own id, never a silent pass. Digital and time mismatches are never touched.

- **R, the reference.** Classic is replayed against itself: at the fixed step
  h, and again at h/2, both backward Euler (order 1). At an instant,
  `R = 2 c(h/2) - c(h)`, which cancels backward Euler's leading error. R is
  a numerical stand-in for the true answer, not the answer.
- **`richardson-classic-self-reference`.** The core is within the ordinary
  tolerance of R (volts: 100 uV plus 0.1% of the larger value; amps: 1 nA
  plus 0.1%) and at least as close to R as classic is.
- **`richardson-closer-than-classic`.** A mismatch the first rule does not
  take is explained when BOTH hold: the core is strictly closer to R than
  classic is (`|core - R| < |classic - R|`), and R has converged. The
  convergence guard: classic is replayed a third time at h/4,
  `R' = 2 c(h/4) - c(h/2)`, and `|R' - R|` is within the ordinary tolerance
  at R's magnitude (the same volt and amp bounds). The h/4 replay runs only
  when a mismatch needs it, at most once per entry. If it fails, or has no
  sample at the instant, the mismatch stays a failure. Each hit records
  `|core - R|`, `|classic - R|` and `|R' - R|`.

Both rules record their hit's errors against the tolerance at R's
magnitude. For each rule, `scripts/core-equivalence.ts` prints the largest
`|core - R|` and `|classic - R|` in units of that tolerance next to its hit
count, so the size of what a rule accepts stays visible.

Delta (W1e, manager ruling 2026-09-29): the closer-than-classic rule, from plan accuracy gate 2.

W1f adds (rails and sources, "Kernel semantics: rails and sources (W1f)"):
- **Battery drain.** SoC, `dischargedCoulombs` and the OCV compare against
  an analytic sidecar where one exists (the ODE of Rails point 6, solved by
  the sidecar generator), else under `richardson-closer-than-classic`, since
  classic's per-step count is O(h). `dischargedCoulombs` uses the amps
  tolerance integrated over the run: `(1 nA + 1e-3·|I|avg)·T`.
- **Regime** (W1f Q7). `reg` is not a compared readout; the harness compares
  the electrical result (`netV`, `elementI`). Known divergence
  `w1f-cc-exit-reg`: where classic's `reg` flickers 0, 2, 0 on a CC exit,
  the core's reads 2 (Rails point 5). Only `reg` may differ: the settled
  output voltage and current in and after that window match classic within
  the ordinary criterion, and this id never excuses a mismatch in them.
  Gate 1's `wave5-regulators` asserts `reg` at points outside the window.
- **Source edges.** Clock and pulse edges are compared per cycle (decision
  6). Headless classic lags an edge by up to one step (`1/(10 f)` for a
  clock, `per/10` for a pulse, HL:141-168); the core does not.
- **The reader scan** (W1f Q12). Known divergence
  `w1f-rail-scan-late-flip`: a reader whose pin moves with the rail against
  a fixed threshold (an LS input on a divider off the rail) is re-evaluated
  at the next rail scan, so it can flip up to `RAIL_SCAN_V` of rail late
  (Rails point 3; the power window is exact, Rails point 4). It is measured,
  not assumed:
  - a Rust test pins the bound: on a slowly draining battery, such a reader
    flips when the rail has moved at most `RAIL_SCAN_V` past its exact
    crossing, plus the one rail move that crossed the band (`RAIL_DRIFT_V`
    on a smooth drain), and never before the crossing;
  - stat 20 counts every reader flip found at a rail scan, and
    `scripts/core-equivalence.ts` lists by name every corpus and fixture
    circuit whose run ends with stat 20 above 0, counted as hits of this id.
- **The 555 period on a moving rail** (W1f Q4). NE555 point 13.2's 1e-6
  against the model-exact sidecar assumes a constant supply. The
  **moving-rail set** is exactly:
  - every 555 with pin 8 or pin 1 on the rail node of a `battery_pack` hub;
  - every 555 with pin 8 or pin 1 on the rail node of a `bench_psu` hub
    whose PSU is in CC at any instant of the compared cycles.

  Neither engine has a closed form there. For that set only, each period,
  HIGH and LOW time compares at 1e-5 relative against a refined classic run
  (fixed 1 µs trapezoidal steps, the first 20 cycles, cached as the classic
  references are), and the closed form at each cycle's mean rail is
  reported alongside at 1e-4, without gating. Every other 555 with a
  sidecar keeps criterion 3 at 1e-6: both supply pins on fixed nodes (an
  ideal source, a `bench_psu` with `iLimit = 0`), or on a `bench_psu` hub
  that stays in CV (`u = V`) through the compared cycles.
- **S2, the SAP-1 on the battery.** The board is `gen-sap1 max battery`:
  `BAT1 {voltage: 5, rInternal: 0.9}`, no uid, so generic: `R = 0.9·dep`,
  `OCV = 5·soc`, 1 Ah, 25 °C (factor 1). Every IC's `vcc` is the rail; no
  capacitor sits across it. The plan's seven criteria (§3.4.4) per variant,
  with:
  1. machine state per cycle: unchanged;
  2. node voltages at quiet instants: W0's tolerance, the rail node
     included; the supply current judged as S1 judges it (the residual less
     the display's draw);
  3. the 555 period: every 555 on the board is in the moving-rail set, so
     1e-5 against the refined classic run, the mean-rail closed form
     reported at 1e-4;
  4. the display digits equal OUT: unchanged;
  5. failure and warning sets: as S1 (criterion 5 on the running variant
     is W1g's item);
  6. core against core-exact (Rails point 9), over the declared window, the
     first 20 clock cycles (W1f Q3): identical per cycle, voltages at
     phase-aligned instants within a measured bound. P3:36's 20 µV is the
     rail's share only: exact mode steps the timing networks (they join the
     merged partition), so their step-control error, of the order of W0's
     tolerance, dominates the bound;
  7. real time: unchanged; the rail's own cost is reported apart (the
     battery variant against the ideal one on the same trace length).

  Two S2-only checks: the battery state at the end of the run (SoC and
  `dischargedCoulombs`, tolerance above), and the rail's minimum and
  maximum over the run against classic's (the sag range).

W1g adds (thermal, failures and warnings, "Kernel semantics: thermal,
failures and warnings (W1g)"):
- **Failure and warning sets over a run.** Classic is sampled at every
  accepted step (S1 criterion 5, `4d6fdb8`): a latch that lasts one step is
  otherwise caught about once in fifty. The core side reads the latch log
  (`bo_engine_records` channel 3, `CoreEngine.takeLatchedFailures()`) and
  `takeNewWarnings()` at every advance end, so it sees every latch over the
  run, never a sparse poll of `getFailures()`.
- **Integrated latch times** (Q-O31). The core is held to the sidecar's
  analytic time within `1e-9 s + 1e-9` relative; classic's time is recorded
  and does not gate. `output_sag`'s `since` is the readout time and is not
  compared.
- **Failure sets** compare `(componentId, kind, pinId)` exactly, `value` and
  `limit` within decision 6's tolerance, and `message` against classic's
  template applied to the core's own numbers (W1g point 19).
- **The failed reset** (W1g Q6). Known divergence
  `w1g-reset-failures-failed-seed`: when the re-solve after
  `resetFailures()` fails, the core rolls the reset back, where classic
  keeps its failures cleared over a non-converged seed (W1g point 13). Both
  engines report `lastConverged = false`, and a gate 2c fixture asserts it;
  the id excuses only the failure and warning sets after that call, never
  the convergence flag.
- **Committed against live sag targets** (W1g Q12). Known divergence
  `w1g-sag-committed-target`: inside a pending enable or disable delay,
  classic's `output_sag` targets follow the live evaluation and the core's
  the committed drivers (W1g point 9). It is bounded: a hit must lie inside
  that driver's pending delay, and the sets must agree once the delay
  expires. A gate 2c fixture pins it, and it fails with a hit outside the
  delay.

## Hard-won rules for the kernel (from the prototype, W1e, and the W1f and W1g probes)

- Accept a threshold crossing only at `t > now + 1e-15`; rounding can put a
  node back on the threshold and re-schedule it forever.
- Guard closed-form re-aiming until the first segment exists (`segReady`), or
  it produces NaN.
- Keep a pending plan's capacitor history separate from the committed `vc`.
- Bidirectional parts (74LS245) register every A and B pin as a reader as well
  as creating drivers.
- Polish a static Newton solve once after it passes: classic re-solves every
  step, so its steady state is the fixed point, not the first passing
  iterate.
- Watch every threshold a reader compares against in both directions, so a
  node's published voltage never sits across a threshold from its exact
  value; a reader evaluated at an unrelated event then reads the right side.
- Never re-evaluate a waveform at its own edge instant to learn its new
  level: fmod rounding puts some edges on the wrong side (probe P8).
- An exact CC exit needs the CV current back under the limit, not only the
  98% voltage, or the state machine re-enters at the exit instant forever
  (probe P7).
- A probe load must stay under the resistor's default rating: classic's
  `resistor_overload` opens a 2 W load within a second and the source then
  reads its node shunts only (probe P1's first run, 10 Ω on 5 V).
- An IC given a `catalogUid` that is not in the library takes classic's
  degraded path (powered, drives to `params.vcc ?? 5`), not the rail (probe
  Q3's first run: `ic-74ls00` is the uid, not `74ls00`).
- A slow event is its own latch: never re-test a stress, window or boundary
  condition with `>=` at the event's own time. Rounding leaves it one ulp
  short and it reschedules forever.
- Clear a partition's solution cache at every slow stamp change. The keys
  carry no failure or law state, so a pre-latch entry is a valid key again
  after a reset.
- A thermal LED's held temperature moves only at a band boundary (static)
  or a step commit (stepped), never at an unrelated re-solve. Otherwise
  every commit misses the cache.
- The `output_sag` window is per pin AND direction. A pin-keyed count
  latches a glitch's LOW-then-HIGH as one sag (the S1 `U41.y2n` latch, W1g
  point 9).

## Deferred past 1.0.0

- Hysteresis at the IC supply floor, a classic change too: the W1c fixed point
  returns -4 (fallback to classic) when a free supply sits near the power
  floor. Accepted for 1.0.0 (manager, 2026-09-29).
- LED plus capacitor transients: both engines sit up to about 10x the
  tolerance from the Richardson reference ("Make a Capacitor Fade an LED",
  which passes as `richardson-closer-than-classic`). Investigate step
  control and the LED Newton's accuracy after 1.0.0 (manager, 2026-09-29).
