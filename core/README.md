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

## Program format, version 4

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
carry    carryLength L, value x L          (v4 only)
trailer  MAGIC
```

- `MAGIC` = `0x424F4331` ("BOC1"), `VERSION` = `4`. W1d bumped it from 2 to
  3, because the 74ls157 and 74ls245 records gained a delay tail; W2c to 4
  for the `CARRY` block. A v3 program (no `CARRY` block) still loads, as a
  cold load (ruling R100 (a): the ABI stays 2, and a host on the v3 writer
  keeps working).
- **`CARRY` block** (W2c, rulings R57 D8 (a) and D9 (a)). The state a
  reload carries, which `bo_engine_load` seeds before its settle (see
  Carry). `L = 0` is a cold load. Its values are records `device, family,
  version, len, value x len`; `NaN` is a value here (none, or start fresh),
  unlike params and blobs. The reader checks every record: whole, its
  device `< D`, at most one per device and family (latches excepted), and a
  known family at the current version on a device of its kind, with at
  least its layout's values, each in range. A malformed block is `-1`, and
  nothing is seeded. A family or version the core does not know is skipped,
  and a longer record's tail is a later layout's.
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
| 2 | `voltage_source` | `pins[0]` (+), `pins[1]` (−) (positional) | `V = Number(params.voltage ?? 5)` | empty | W0; live param 0 (W2c, Live V) |
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
- A battery param edit is a reload, as in classic; the reload carries its
  SoC onto the same authored charge and part (W2c, Carry).

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
- Across a reload the regime is not carried but derived again at the
  settle from the carried charges (W2c, Carry): CV/CC is a function of the
  state.

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
ownerless net compiled with no anchor; it now stays with classic. A
component id that two components share declines `malformed:<id>:duplicate-id`
once per shared id, for every kind: the core's tables are keyed by id, and
classic's load keys by id too (a shared source id leaves its matrix
singular; shared passive ids alias as parallel stamps). No corpus circuit
has one.

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
  2.5 W, no latch in 2 s). A resistance `<= 0` (or `-Infinity`) emits
  `rPower = 0`, so the resistor rates 0 W, as classic's power is
  `resistance > 0 ? I·I·resistance : 0` (ruling R18 (a);
  `ratingResistance`, `src/core/compile/thermal.ts`). So the record is
  finite whenever the resistor compiles; a `NaN` or `+Infinity` resistance
  is the resistor's own `non-finite-resistance` decline.
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

All exports are `extern "C"` with `#[no_mangle]`. The module has **one
import**, `env.bo_host_now_ms: () -> f64`, the host's clock in milliseconds
(`performance.now()`), which only `bo_engine_advance_for` reads (ruling R86);
the TS loader supplies it at every load, and a host without a clock supplies
one that reads NaN, which turns the wall budget off. It exports its `memory`, the functions below, and nothing else
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
| `bo_engine_advance_for` | `(h, t_end: f64, budget_ms: f64) -> i32` | `bo_engine_advance` under a wall budget (ruling R86, "A yielding advance" below): `0` at `t_end`, `1` when it yielded short of it. | v2, R86 |
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
| `bo_engine_snapshot` | `(h) -> u32` | Copy the whole engine into the handle's snapshot table ("Snapshots" below) and return the snapshot's id; `0` for an unusable handle, with the message. | v2 (W2a) |
| `bo_engine_restore` | `(h, id: u32) -> i32` | Put snapshot `id` back; it stays restorable. `-8` for an id this handle does not hold, `-5` for an unusable handle. | v2 (W2a) |
| `bo_engine_snapshot_drop` | `(h, id: u32) -> i32` | Free snapshot `id`. `-8` for an id this handle does not hold (a second drop among them), `-5` for an unusable handle. | v2 (W2a) |
| `bo_engine_snapshot_bytes` | `(h) -> f64` | The handle's snapshot table in bytes, each shared memory image counted once; `NaN` for an unusable handle. | v2 (W2a) |
| `bo_engine_carry_take` | `(h) -> i32` | Collect every device's carry records ("Carry" below) and return their length in f64; `-5` for an unusable handle, `-4`/`-7` when bringing the partitions to the engine's time fails. Optional, detected by presence. | v2 (W2c) |
| `bo_engine_carry_ptr` | `(h) -> *const f64` | The buffer from the last take, valid until the next call on that engine. Optional. | v2 (W2c) |
| `bo_engine_acquire` | `(h, nodes: *const f64, n: u32, origin: f64, interval: f64, limit: u32) -> i32` | Arm the observer on `n` node indices, sampling at `origin + k * interval` from the engine's time ("Acquisition" below); `interval` 0 disarms, `limit` 0 holds 2^20. `-8` for a bad node, origin or interval, `-5` for an unusable handle. Optional. | v2 (W2b) |
| `bo_engine_acquire_take` | `(h) -> i32` | Take every final sample, `[count, maxSpan, overflowed, dropped, fallbacks x n, t x count, node-major values]`; the length in f64 (`4` disarmed), `-5` for an unusable handle. Optional. | v2 (W2b) |
| `bo_engine_acquire_ptr` | `(h) -> *const f64` | The buffer from the last take, valid until the next call on that engine. Optional. | v2 (W2b) |
| `bo_engine_charge_arm` | `(h, devices: *const f64, n: u32) -> i32` | Arm the charge on `n` device indices from the engine's time ("Charge" below); `n` 0 disarms. `-8` for a bad device, `-5` for an unusable handle. Optional. | v2 (W2b) |
| `bo_engine_take_charge` | `(h) -> i32` | Take the charge, `[t0, t1, per device its charge then its channels', per device its final time, per device its KCL flag]`, the sums cleared; the length in f64 (`2` disarmed), `-5` for an unusable handle. Optional. | v2 (W2b) |
| `bo_engine_charge_ptr` | `(h) -> *const f64` | The buffer from the last take, valid until the next call on that engine. Optional. | v2 (W2b) |

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
  it (Rails point 6; in exact mode, the held current over the time since
  the last committed step, Rails point 9), so a collect never changes the
  simulation. So is every W1g value (W1g point 16).
- Both `-8` cases, an index out of range and a kind with no device state,
  leave the buffer empty, so `bo_engine_device_state_ptr` never hands back
  an earlier collect's values after a failed one.
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
  (reserved: the facade derives it, W1g point 11), `3
  rail_ic_power_chatter` (a chip sliding along its power floor, W1c point
  1a: `value` the floor in V, `limit` the PSU's limit in A or `NaN`). **PTC** (channel 2):
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
rolls the whole reset back, where classic keeps its failures cleared over a
non-converged seed (W1g Q6; W1g point 13). `-5` before a successful load.

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
events fired, `23` latches, `24` junction re-arms, `25` power holds taken,
`26` power holds released, `27` stepped partitions fallen back from the
trapezoid to backward Euler (INT-ACC, R81 (1); Kernel semantics W1b 6a),
`28` the charge's intervals whose source KCL closed only approximately
(W2b, "Charge": several stepped hub partitions on a CC rail with no
capacitor).
Stat 11 counts every Newton iteration of
a stepped step (one for a linear partition) and, from W1e, every iteration
of an iterated static partition:
LEDs point 6, the polish included, and W1c's static fixed point, which W1c
did not count. Stat 14 was reserved from W1a and counts from W1f (Rails
point 3); stats 16-20 are W1f's (Rails points 3 and 4, Sources points 2 and
3, and for 20 the reader-scan divergence under Equivalence harness rules).
Stats 16-18 and 20 stay 0 in exact mode. Stats 21-24 are W1g's (W1g point
18). Stats 25 and 26 are F5's (W1c point 1a; ruling D8): a hold counts when
an IC goes to held unpowered and a release when it comes back free; the
probes count nothing, and neither does dropping every hold at `set_params`,
`seed_memory` or `resetFailures`.

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

**A yielding advance** (rulings R86, R94 (a)). `bo_engine_advance_for(h,
t_end, budget_ms)` runs as `bo_engine_advance` does, reading the host clock
at its start, at its end, and every `check_stride` completed instants. The
stride starts at 1, shrinks at once to what the measured time per instant
allows within the budget, and grows back by a quarter a read up to 64
(`YIELD_CHECK_EVERY`), so a call overruns its budget by about one instant at
most. Once a read finds `budget_ms` gone and at least one instant has run in
the call, it returns `1` between two instants: the state, readouts and
`bo_engine_time` are the last completed instant's, with no horizon clamp. A
host step stays open from its first call until it reaches `t_end`; a later
advance (either call) to the same `t_end` goes on with the same host step,
whose span the hold guard reads (R54 (a)) and whose events the cap counts,
so a run split by yields is bit for bit the same run as one without
(`a_run_split_by_budget_yields_is_bit_identical_to_the_same_run_unsplit`).
Any host input ends the host step: `set_params`, `seed_memory`,
`set_option` and `reset_failures`.

The facade's `CoreEngine.advanceFor(tEnd, budgetMs)` (W2h) returns
`"reached"` (as `advance`), `"yielded"` (`simTime` read back, readouts of
that instant, `lastConverged` still true) or `"failed"` (`-4`/`-7` exactly
as `advance` takes them, with `lastAdvanceFailure` naming the code and the
core's message; never a yield). A module without the export runs `advance`
instead and warns once per engine, since a plain advance cannot yield.

Two checks decline with `-7`, the livelock's code, keeping the last
completed instant:
- **without a clock** (`bo_engine_advance`, or a budget that is off): a host
  step that has taken `HOST_STEP_EVENT_CAP` (20M) x max(1, its span in
  simulated seconds) events without reaching `t_end`; asking again declines
  again, at the same event count on every run. The busiest legitimate
  advance of the cold fuzz takes 956k events, and fixture 26's oscillator at
  100 pF takes 24.75M in `advance(3)` (cap 60M);
- **with a clock**, the only check: less than `PROGRESS_FLOOR` (0.01)
  simulated seconds per wall second over `PROGRESS_WINDOW_MS` (30 s) of wall
  time spent inside budgeted advances, judged at every read, the call's end
  read included. Time between calls does not count; one read adds at most
  `BUSY_READ_CAP_MS` (250 ms) or four budgets, whichever is more (a longer
  gap is a pause, such as a suspend); a clock that steps back re-anchors the
  call's reads. An edit does not restart the window (it moves no simulated
  time); a decline does, so a host that goes on is judged afresh. The slow
  legitimate boards bottom out at 0.003-0.03 s/s for 1-2 s of start-up; the
  freezes ran at 4e-4 to 9e-4 s/s for 50-200 s.
A `budget_ms` that is not positive, or a clock that reads no finite time,
never yields and leaves the floor off and the cap on.

**Snapshots** (W2a; W2 design 2.3, ruling D7 (a)). `bo_engine_snapshot`
copies the whole engine: time, the event heap, drivers and held commits, the
partition solvers with their integrator history and hold table, the IC
evaluator, slow state with its records and warnings ledger, the memories,
the readouts (slots, channels and the harness trace), the options and an
open host step with its event count and check stride. `bo_engine_restore`
puts all of that back, so a run from a restored snapshot is bit for bit the
run that followed the snapshot, any number of times. Snapshots are taken and
restored between calls only, so no instant is ever half done in one.
- **Not restored: the stats.** They are telemetry and read on monotonically
  across a restore: each counter keeps counting from its live value, and
  stat 6 (a maximum) never reads below the largest it has shown. The counters
  themselves go back with the state, and `bo_engine_stat` adds a per-stat
  offset that the restore sets to the published value before it minus the
  restored count, so logic that reads a counter, R86's event cap, counts
  against the same events it was taken with, and a restored run stays bit
  for bit the same. Classic restores 7
  diagnostics with its state; that is a registered divergence.
- **Not restored: R86's busy clock.** It is wall time. The progress window
  re-anchors at the restore, to (busy now, the restored time), since a
  restore moves simulated time back while wall time runs on.
- **Ids.** One counter for the module issues them and never reuses one. A
  load starts the handle's table afresh, a failed one too (which keeps the
  old program), so every id taken before it, like
  one dropped or one from another handle, reads `-8`. The core sets no count
  or byte cap: WASM memory never shrinks, so the host owns the lifetime (the
  facade drops a snapshot on `dispose()` or when its object is collected).
- **Copy-on-write tables.** The tables fixed at load (stamps, IC records,
  node maps, the slow model and the slow tables' entries, each partition's
  solution cache) and the EEPROM images are shared between the live engine
  and its snapshots, and copied at the first write after a snapshot: a
  latch, a host edit, a new cache entry or a law re-arm. So a snapshot
  copies only the state the run changes; on the SAP-1, taken a frame apart,
  about 125 KiB each (the table, shared tables included, was 610 KiB a
  snapshot before). `bo_engine_snapshot_bytes` walks the table: each
  snapshot's own heap, and each allocation the snapshots share once,
  whether the live engine shares it too or not.
- **Scratch buffers copy empty.** Work buffers whose contents never outlive
  one use (the kernel's delta lists, the solver's partition and reader
  lists, the source-current fold's pair) are empty between uses, so a
  snapshot holds none of them; each use takes an empty buffer (asserted in
  debug builds) and hands it back cleared.

**Memory.** Any call may grow the wasm memory, which detaches every JS view of
`memory.buffer`. The TS glue re-derives each view after every call and never
caches one; every pointer is read as unsigned (`>>> 0`). Pointers returned by
the readout exports are valid until the next call on that engine.

**Traps.** The crate builds with `panic = "abort"`, so a Rust panic is a wasm
trap. The TS side poisons the instance on a trap, and on any non-core error
thrown during an ABI call (a wasm stack overflow surfaces as a JS
`RangeError`): every later call throws until the core is re-initialised.

## Carry (W2c)

A reload carries state the way classic's `load()` does (SE:2807-3200). The
core reads it off the old engine, the host maps it onto the new program's
devices (`src/core/carry.ts`: by component id, a capacitor also by its two
nets' fingerprint, each kind-gated), and the new program's `CARRY` block
(program v4) seeds the load. Ids and nets stay in TS (ruling R57 D8 (a)).

- **Host** (`CoreEngine`, `src/core/carry.ts`). `load()` onto a loaded
  program takes a carry, maps it (component id; a capacitor also by its
  nets' fingerprint, its sign with its orientation; each record kind-gated:
  an IC, its slow state and latches onto the same kind, a battery onto the
  same authored charge and part, a thermal model's temperature onto the
  same part and profile, an EEPROM onto the same authored `contents`) and
  writes program v4; a cold load stays v3. An EEPROM so carried loads the
  old engine's current image as its ROM blob, so a written byte survives
  even when the host has taken it, while new authored contents start
  afresh, as classic's `contentsHash`. Warnings of components whose id and
  kind survive carry, not announced again. A load given `options.simTime`,
  `coldLoad()` (time 0, as classic's) and a core without the carry exports
  carry nothing. A take that fails (its clamp to the engine's time, `-4` or
  `-7`) is the load's failure, never a silent cold load: the facade throws
  `CoreUnsupportedError` with `carry:` reasons and holds no program, as
  after a load-time solve failure, so the host routes the edit as it routes
  one (to classic, whose own load carries) or calls `coldLoad()`.
- **Cost** (2026-11-12, against 26ed5ee, 5 interleaved reps on a quiet
  machine). A carrying reload of the SAP-1 takes 14.7 ms (ideal) and 18.1
  ms (battery) of CPU against 11.7 and 15.5 ms for a cold one before W2c
  (the take, the map and the seed). A cold reload does no carry work: the
  facade keeps only a component-level copy of the circuit for the next
  reload (a whole `structuredClone` had cost 1 ms of an SAP-1 load) and
  decodes no EEPROM image, and the core's load is unchanged natively (0.035
  ms on fixture 03 against 0.035). Cold SAP-1 reload 9.8 ms against 9.7 ms
  before W2c, fixture 03 0.64 against 0.61 ms (3 reps each, not in a quiet
  window). The rated SAP-1 run (R43) is unchanged within noise: battery at
  steady state +0.9 % running and -0.9 % frozen (7 paired reps, 30 s
  warm-up). The wasm grows 6.4 % (6.2 % gzipped).
- **Take.** `bo_engine_carry_take(h) -> i32` collects the records and
  returns their length (or an error code), `bo_engine_carry_ptr(h)` the
  buffer, valid until the next call on the engine. Both are optional
  exports, detected by presence (R100 (a)). The take brings a copy of the
  engine's dynamic partitions to `bo_engine_time` (as `advance(time)` does),
  so a carry taken where an `advance_for` yielded is the one a plain stop
  there takes, and the live engine is not touched.
- **Records** `device, family, version, len, value x len`, version 1:

  | Family | Values |
  |---|---|
  | 1 capacitor | `vc`, the charge's voltage (behind any `esr`) |
  | 2 IC | `kind, tag, family values, reads, slots, drivers` (`carry::ic`) |
  | 3 battery | `bo_engine_device_state`'s battery layout |
  | 4 bench PSU (limited) | `reg` (0 CV, 2 CC) |
  | 5 slow | `failed, stress, temperature, shutdown, ptc_tripped, ptc_stress, ptc_rt, t_hold` |
  | 6 latch | `code, pin, since, value, limit, aux`, one per latched failure |
  | 7 output_sag windows | `n, (state, dir, t_s) x n` per output pin: 0 none, 1 pending since `t_s`, 2 latched |

  `NaN` is a quantity the device has none of; times are absolute. A
  capacitor's charge is the running solver's: a `cc_switch` partition's
  from whichever copy runs it, and a rail capacitor's in CC from the rail's
  segment (its charge less what the closed-form draws took since it was
  aimed), which its fixed pair does not hold there.
- **Seed**, at `START_TIME` before the settle: a capacitor's `vc` into the
  solver that holds it (a stepped partition's first solve then seeds its
  nodes from it, a closed form's first segment starts from it, a fixed
  pair's anchor is it); a battery's SoC as its `soc0` (so the rail's
  open-circuit voltage and every fixed node follow it) and its discharge
  counts; the slow state (stress, temperature and held junction
  temperature, PTC trip state and timer, failed flags), then the stamps it
  implies, so the settle's first solve sees an open fuse or a tripped PTC;
  the latches in record order, not logged again (the host announced them
  before), and the `output_sag` windows (a pending one ends where it would
  have, a latched one stays published). A value for an integrator the new device lacks is dropped, and
  `NaN` starts that quantity fresh. A PSU's regime is not seeded: CV/CC
  is a function of the state with no hysteresis (CC exactly while the CV
  draw would pass the limit, CV once the CC rail reaches the setpoint), so
  the settle derives it from the carried charges, as classic's first step
  does (2575 tpl 2 reloaded in CC comes back in CC, a test pins it). A PSU
  model that gains hysteresis must carry its regime.
  An IC takes its family state (registers, a 189's nibbles, an EEPROM's
  write cycle, a 555's latch, Schmitt-held levels), its last reads and
  slot readout, and its output drivers: each committed level, and a
  transition in flight as a `Commit` at its absolute due time (an EEPROM
  write in flight likewise wakes its device when it completes). An EEPROM
  takes its write cycle only onto the same image (hash and length); the
  image is the program's ROM blob, so a host carries a written image by
  loading it there, and onto another image the IC starts afresh, drivers
  included, as classic's does. A record whose counts do not fit its IC
  makes the load `-1`. The settle then runs as on a cold load, except that
  a carried driver is no first observation: its transitions take their
  real delays, not the settle's zero override, as classic's seed solve
  times them from the carried `delay:` slots (classic's seed solve sits 1
  ps after the load time, the core's at it). The carried edge history
  (`lastClk`, `lastWE`, `/WE` and `/CE`) makes an edge the reload made act
  in the edge pass. The integrator
  history is not carried, so the first step is backward Euler, as classic
  forces after a load. The cold-device span widens over carried charges
  (see the thermal, failures and warnings section).

## Live V (W2c)

A `voltage_source`'s `V` (param 0) is live (ruling R57 D12 (a)):
`bo_engine_set_params` sets it as a source edge at the engine's time, one
event like every set, which ends the host step in flight (R86).

- **The edge.** Every node the source fixes, and every node a source stacked
  on it fixes, takes its new value (`PartitionedSolver::source_params_at`).
  What reads one re-solves as at any edge: the partitions whose stamps read
  it, the ICs reading it (for a supply, the partitions watching their read
  pins too), the rule-2a capacitors on it (re-aimed, their charge
  continuous), and the rail where it is a direct node. The static solver
  folds a constant node into its partitions' bases, so those fold afresh,
  and the solution caches (static and closed-form) take a new epoch: no
  key holds a constant node. Every slow device is driven afresh at the
  settle, as at load.
- **IC power.** A set decides the power of each IC whose supply it moves
  as a reload onto the new `V` does. An IC on constant supplies takes its
  `output_sag` power afresh at the new values (and back at a rollback):
  out of its window it drives nothing, so its outputs latch no sag. A static
  partition holding a free IC supply that reads a moved node solves from
  0 V, as at load, not from the published values: such a supply can hold
  two consistent states (powered and drawing through its resistor, or off
  and drawing nothing), and a solve from the old values kept the old one
  (corpus 39 at 7 V: 4.83 V powered, where classic and a reload read 7 V
  off). A stepped partition's seed already decides it as the reload's does.
- **Cold devices.** The cold decisions (R38 (a)) hold under the span at
  load. A set takes the span again from the sources' values then, widened
  by the charges the capacitors hold (as a carried load's, D10 (a)); while
  it is within the load's, every cold device stays cold, and past it every
  cold device turns hot (a superset of those now stressable).
- **Equivalence.** Classic has no live set: its answer to an edit is its
  `load()`, which carries, so the reference for a set is a carrying reload
  onto the new `V`. A set runs on as that reload does: digital identical,
  analog within W0, and the same failure records (`solve/live_v_tests.rs`,
  and each switch kind's live param likewise). Measured across the corpus
  (W2cV's sweep, 2026-11-12: every voltage source of 25 fixtures set to 0,
  half, 1.7 times plus 1 V and minus itself less 2 V, at four times each):
  1040 set and reload pairs, every readout and failure equal within W0.
  Before the IC power rule above, 358 of them latched `output_sag` that the
  reload, and classic's own edit, did not.
- **A failed set** (`-4`, `-7`) rolls back with its instant: the source,
  the nodes it fixed and the static solver's folded bases.
- A battery or a bench PSU stays a reload.

## Acquisition (W2b)

A scope reads the model's trajectory on a grid without moving it (rulings
R57 D1-D3 (a), R104; `core/src/observe.rs`, `solve/observe_tests.rs`).

- **Arm.** `bo_engine_acquire(h, nodes, n, origin, interval, limit)`
  samples the nodes at `origin + k * interval` for whole `k` (the times
  computed so, never summed), from the first at or after the engine's time.
  Re-arming drops what was pending, `interval` 0 disarms, a load disarms.
- **Take.** `[count, maxSpan, overflowed, dropped, fallbacks x n, t x count,
  node-major values]`, every sample whose value is final, in time order;
  taking empties the queue. At most `limit` samples are held (2^20 by
  default): past that the newest are dropped and counted, the oldest kept;
  every index, a full queue's skip and the first after arming too, is
  decided by the grid times themselves, not by a rounded quotient (W2bV),
  and an interval too fine for `u64` indices saturates rather than wraps.
  `maxSpan` is the widest stepped step a sample was read across; a node's
  fallbacks count its samples read linear past W0 (R104's solve failed, or
  inside the step in flight when the observer armed, below).
- **A sample** at `t_s` between two completed instants is the model's state
  there, read off what the solver holds: a fixed node's value (a running
  ramp's line), a static node held, a closed-form node its segment's exact
  value, the hub rail its model between events (the CC segment, the cell
  path, the CV setpoint; where nothing models it, linear between its solves,
  ruling (a)) and a static hub node its line in it, a stepped node between
  the two commits around it: linear over a backward Euler step, cubic
  Hermite on each capacitor's charge over a trapezoidal step or TR-BDF2
  stage, the node from the charges where cheap (a plate on a fixed node, a
  linear partition with one capacitor), else, for an iterated partition, a
  scratch solve at the Hermite charges (R104 (b); a failed solve reads linear
  and counts as a fallback). On an instant a sample reads the state after it
  (D2). A stepped node's sample waits for its later commit, so a yield can
  leave samples pending; the horizon clamp flushes them. A step planned
  before the observer armed kept no points: its plates read the cubic
  Hermite across the whole step from its ends' capacitor currents (W2bV:
  linear, 11.4 W0 off a run armed from the start; Hermite 0.49, at a
  voltage's W0 of 100 uV + 1e-3 |v|). The limit: an iterated partition's
  other nodes read linear across that one step, since their scratch solves
  need the step's own context, gone by then; measured 3.40 W0 (an LED's
  cathode with no capacitor at turn-on), each such sample inside the step
  counted on the node's fallbacks (a must-close item), the samples after it
  as any.
- **Passive.** Nothing is stepped, cut or re-solved for a sample (an
  iterated partition's scratch solve keeps nothing): a probed run is the
  unprobed run bit for bit.
- **Interactions.** A rolled-back instant reads no sample. A run split by
  `advance_for` yields reads the same samples. A snapshot carries the queue
  and the arming (a restore retakes from there). A host's edit (a live V
  set, any `bo_engine_set_params`) is an instant at the engine's time: samples on and
  after it see the new value, so one at that time the horizon clamp had
  already read is read again after the edit (W2bV, D2); one a take already
  delivered stays the host's, with the value the run had then.

## Charge (W2b)

Light means: the integral of each armed device's published current, and of
each of its channel slots, over time (ruling R57 D4 (a); `solve/charge.rs`,
`solve/charge_tests.rs`). A mean is a charge over its span.

- **Arm.** `bo_engine_charge_arm(h, devices, n)`, from the engine's time on;
  re-arming drops what was not taken, `n` 0 disarms, a load disarms. A
  device's charge does not depend on what else is armed: when anything that
  follows the hub rail is armed, every stepped hub partition keeps its steps
  for the charge, since any of them may carry the rail (W2bV).
  Unarmed, nothing runs but an `Option` test at each instant; a charged run
  is the uncharged run bit for bit.
- **Take.** `[t0, t1, then per device its charge and its channel slots' in
  channel order, then per device the time its charge is final to, then per
  device its `kcl_approximate` flag]`. A take carries each device's charge
  from the final time of the take that last delivered it to its final time
  now, exactly, and nothing past it: no later take corrects what one
  delivered. A final time is `t1` after `advance` (the horizon clamp
  commits every partition there and solves the rail). After an
  `advance_for` yield a device a stepped partition publishes is final to
  that partition's last commit (its charge is its committed steps; the rest
  of its step comes with a later take), and what a carrier folds (below) to
  the carrier's last commit. Two kinds are withheld whole, their final time
  left where their last delivery put it and their charge `0` in the take,
  until a take can deliver them exactly: a device with an interval on a rail
  nothing models (folded at the rail as last solved, waiting for the next
  solve to correct it, below), and a source until every term its walk sums
  reaches `t1` (its stepped partitions and the carrier committed there, no
  such interval waiting): a step of one partition straddles another's
  commit, so its walk has no earlier time it reaches whole. W2bV: the final
  time had stopped at a held interval's start while its charge (at the held
  rail) reached `t1`, a cell's mean after a yield up to 2x high. A host
  divides each device's charge by the span from its previous final time to
  this one, never assumes `[t0, t1]`, and reads no mean from a zero span.
- **Cost** (ruling R107 (a)). The static solver's copy and the boundary
  writer mark the held devices whose nodes or currents they write, and only
  those refold, so the charge costs what an instant changes, not what is
  armed (a debug build checks at every fold that no unmarked device moved);
  a held LED's last two states are kept, so a logic output toggling it costs
  no exponential (a debug build checks each memo hit against the readout).
  The observer reads and locates its nodes only at an instant with a grid
  time due. Measured on the rated SAP-1 (ideal, running; 20 simulated
  seconds a case, 5 interleaved reps, quiet machine, 4c532a1;
  `pnpm run bench:probe-cost`): unarmed 13.52 ms per simulated second, 2
  probes at 10 kS/s +5.0 %, the means (its 83 LEDs and 4 displays) +7.0 %,
  4 probes at 10 kS/s with the means +12.3 %. Unarmed, the core is gated
  within noise of its base (the sap1-cost gate; +0.7 % measured against
  2f62f88); armed, the budget case must stay within +15 %
  (`bench:probe-cost -- --gate`). Where wall time is too noisy to read a few
  per cent (a desktop with a browser running), count instructions retired
  instead (`/usr/bin/time -l` on one case run in-process, `node --import tsx`,
  30 simulated seconds, head and base interleaved; run spread about 0.4 %):
  after the W2bV fixes (68bc58b, 4c532a1 the base) unarmed -0.25 % (equal:
  nothing new runs unarmed), the budget case +0.8 %, so about +13 % over
  unarmed against 4c532a1's measured +12.3 %. What remains at 10 kS/s is a floor per
  instant and per take, not per sample (an optimisation item below).
- **Folding.** At the kernel's `observe` calls, between instants: the steps
  committed since, the held currents the last instant changed, and the
  interval up to the next instant on the state the last one left. A step
  commits inside its instant's window and goes with it on a rollback, so a
  rolled-back instant adds nothing; the interval before a failed instant is
  final and stays.

Each solver's charge is exact in its own model. Where it is not, the
error is bounded below against W0's current tolerance over the span, `(1
nA + 1e-3 |I|) t`, which is the charge's W0.

| Device | Charge | Bound |
|---|---|---|
| In a static partition, or across fixed nodes | its published current, held between instants, times the time it held (refolded only when it moves) | exact |
| A resistor or switch across fixed nodes, one on a running ramp | its current at the interval's mean voltages (the ramp's midpoint) | exact (affine) |
| A resistor, switch, LED or 555 on the hub rail or a static hub partition's line in it | held while the rail holds between its events; while it moves, its current at the rail model's mean (the CC segment's or cell path's integral, `CapSeg::mean`) | exact for a resistor or switch; an LED or 555, nonlinear, by `I''/8 (dV)^2` with `dV` the line's move over the interval (within W0 while `dV` stays under about `0.09 n Vt`, 4.6 mV at n = 2) |
| In a closed form | each current at the analytic means of `vc`, the rail's move `du` and the ramp's time (all affine); the capacitor `C dvc` | exact; a draining cell's held and curved terms and a curved segment's `vc du` by composite five-point Gauss-Legendre, at most `4e-10` of the term (panels of two time constants) |
| In a stepped partition | its integrator's own quadrature over the step's points: backward Euler's end, the trapezoid's two ends, TR-BDF2's `k (gamma h / 2)(i_0 + i_g) + h_2 i_1`, each point read against the boundary and the CC rail the step solved with; a capacitor's is `C dvc` to rounding | exact; an iterated partition's points meet KCL to its Newton tolerance (measured 2e-7 to 5e-7 of the charge at an LED's nodes) |
| A capacitor across fixed nodes | `C dvc` on its exponential, or its share of a CC rail's charge | exact |
| A voltage source, PSU or anything else the forest fixes a node with | KCL over the charges of what leaves the nodes it fixes, walked as `recompute_source_currents` walks; a PSU in CC is `-dir iLimit` for as long as it is | exact; see the open item below |

- **A rail nothing models** (a cell or CC rail moved by stepped draws between
  its solves; ruling (a)). The interval is folded at the rail as last solved
  with its sensitivity to the rail kept (every term on the line is affine in
  it); when the next solve comes, the rail's mean there, linear between the
  two solves, corrects it. Measured: 1.5e-8 W0 from the linear rail over 5 ms
  and 1/60 s (cell rail, stepped; 0.94 W0 before the correction).
- **Beside a sliding chip** (rulings R92 and R103 (b)). An LED, a 555's
  OUT, a junction or a sliding chip's own pins in a partition the chip
  touches are read, in every class above and in a source's KCL walk, as
  their publish reads them: the two solves' values weighted by the duty
  (`led_readout_at`, `ne555_out_current`), not the device's law at the
  blended voltages; a held LED's key carries the duty and the two solves'
  junction voltages. Measured on HG1's CC ramp
  (`the_charge_of_an_led_beside_a_sliding_gate_is_its_two_solves_weighted_by_the_duty`):
  the LED's charge over [1.47, 1.60] ms within W0 of the rail's KCL by
  hand (read by its law at the blended voltages it charged 1.02e-7 C
  against 4.48e-7). In a capacitor-free stepped partition the two solves
  are the partition's last committed ones carried along the rail, so a
  point inside a step reads them as at its end (the duty is constant
  within a step; what else moves is a running ramp over one step).
- **A failed LED-family channel** reads NaN (absent, as classic's) and
  carries no current: it adds nothing, the charge before the failure stays
  (W2bV: the take's channel charges went NaN).
- **A floating pull's target** moves only at its bucket edges, which are
  instants, and a step is stamped at its start: exact.
- **Arming inside a step** weighs the straddling step's quadrature by the
  part of it after the arming (a step planned before arming kept no points:
  its end over that part), within one step's charge.
- Measured against fine references (host steps of 0.5 us, the same boards):
  cell rail 1.4e-7 W0; cell rail stepped 0.72 W0 and a CC rail with no
  capacitor, stepped, 0.84 W0, both the integrators' own trajectory
  differences (a stepped capacitor's charge is exact `C dvc` of either run).
- **A CC rail with no capacitor** is set by the stepped hub partition's
  draws, solved at each point of its steps. With one stepped hub partition
  the rail's lines (a static hub partition's nodes, what reads the rail)
  fold at that partition's points with its weights, as its draws do, so
  the PSU's `-dir iLimit t` is the KCL of what leaves the rail (was 1.29 W0
  apart with the lines linear between the rail's solves). A line's charge
  then carries the steps' own quadrature error, as the draws' does: on the
  test board 2.3 W0 from the fine run's, which is the PSU's and the stepped
  resistor's own gap from theirs (the coarse run leaves CC at its own
  instant). With several stepped hub partitions each step solves the rail
  against the others' draws held where they last committed, so no one rail
  trajectory exists: the lines fold linear between the rail's solves, and
  KCL holds to the steps' coupling error (measured 1.88 W0, where the
  stepped capacitors' own exact charges are 2.1-2.3 W0 from the fine run's;
  `charge_rail_carrier`). Said loudly: the take flags `kcl_approximate` (`1`)
  on each armed source whose walk reads the rail, for a span holding such an
  interval, and stat `28` counts the intervals. Bound: the coupling error of
  the steps, 1.88 W0 measured; a must-close item.
- **A CC rail with capacitors and stepped hub partitions** (ruling R68): each
  step carries the rail capacitors as a companion on its rail row and hands
  their charge to the rail solve after it commits, which re-aims the CC
  segment from that charge; between solves the segment is only the frozen
  draws' estimate. With one stepped hub partition its steps carry the rail:
  the capacitors fold at its points at their companion current, the lines
  with them, so a capacitor's charge is `C dV` of its readout and the PSU's
  `-dir iLimit t` the KCL of what leaves the rail to the step's Newton
  tolerance (measured 8e-7 W0; on the segment they were 1.2e-2 of `C dV` and
  1-7 W0 apart, W2bV). With several, the capacitors fold on the segment and
  take each re-aim's jump as their own charge, so they stay `C dV`; the lines
  fold on the segment, and the PSU's KCL is flagged and counted as above
  (measured up to 0.52 W0).

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
- Each method finds its kind from the compiler's own tables on
  `CompiledCircuit` (exported from `brownout/core`): `batteries`, per
  `battery_pack` component id, the profile and temperature statics
  `getBatteryState` adds, and `benchPsuLimits`, per `bench_psu` component
  id, the record's compiled `iLimit` (0 for an ideal source), which picks
  the layout `getIcState` expects back, so the facade reads the value Rust
  was given rather than re-deriving it from the circuit.
- A returned length other than the layout's means the TS and Rust halves
  were built from different contracts: `CoreInitError`, as an ABI version
  mismatch. A negative code cannot come from a loaded program and a kind
  that has a layout either (a Rust/TS drift): `CoreLoadError` with Rust's
  message.
- A circuit with two hub candidates (two batteries, or a battery and a
  `bench_psu` with `iLimit > 0`) is a load error, `-3` (Rails point 1),
  which the facade raises as `CoreUnsupportedError`: the circuit falls back
  to classic, and neither method is reached.
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
  (W1g point 11) plus channel 1 (`rail_ic_power_chatter`), keyed by classic's
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
- **`saveState()` / `restoreState(s)`** (W2a, "Snapshots" under the C ABI):
  `saveState()` takes `bo_engine_snapshot` plus the facade's own clock,
  `lastConverged` and warnings ledger with its announce queue, and returns
  an opaque `CoreSnapshot`; `restoreState(s)` puts all of it back and
  republishes the readouts. It throws for a snapshot of another engine, a
  disposed one, or one taken before the last `load()` (a failed load
  included). `CoreSnapshot.dispose()` frees the core's copy (a second call
  is a no-op); one collected without a `dispose()` is freed by a
  `FinalizationRegistry`. `snapshotBytes()` reads
  `bo_engine_snapshot_bytes`. The stats are not part of a snapshot.

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
   `INTEGRATION` is `1`, and in a linear partition by default, 6a), land on every `t_end`, and refine crossings by
   bisection. When an event (a driver commit, a reset, a `set_params`)
   dirties a stepped partition, its catch-up to the event instant integrates
   with the stamps in force BEFORE the event. The new stamps apply from the
   instant on. A step whose Newton does not converge (a partition holding a
   stamped LED junction or a free IC supply node) is rejected and retried at
   `max(H_MIN, 0.25 h)`, `-4` only at the floor, and the re-aim seed
   warm-starts from the committed `x` (W1e, LEDs point 8). This changes
   W1c's free-supply stepped partitions too, which returned `-4` at once.
6a. **The integrator (INT-ACC: rulings R77, R78, R79, R81, R88).**
   - **Trapezoid by default** (R79 (a)). A linear stepped partition (no
     stamped junction, no free IC supply, no rail-supplied IC pin when it
     steps the CC rail) steps by the trapezoidal rule under either
     `INTEGRATION` value; the first step after a discontinuity stays
     backward Euler, as classic's trap mode has it. A partition that reads
     the PSU's rail (`hub`), or in exact mode holds it, keeps backward
     Euler (`on_rail`): the rail jumps at a regime change with no
     discontinuity for the partition, which backward Euler steps through
     and the trapezoid carries as a ring; the trapezoid there also let a
     rail's stiff draw ring (cold 761/5, 207/5). `voltage_span_bound` gives
     no bound when any partition may step by the trapezoid (`may_trap`).
     Divergence from classic, whose default is backward Euler everywhere.
   - **The series current compared** (R77 (a)). In a trapezoidal run the
     step-doubling estimate also compares each capacitor's series current,
     the next step's history (classic's `capsI`, `ABS_CURRENT` 10 nA plus
     `REL_TOL`), so a ring the charges hide is caught. A current that rings
     (over the two halves it steps down and back up about the current it
     rides on) is not what a shorter step improves: it does not set the
     next step's growth (R88), and above the floor a step failing on rings
     alone is taken (R81 (3), the ring-aware R77). Classic feeds the whole
     capsI ratio to its next factor and fails on it, and crawls: cold 287/3
     (a 14 nA ring on a 6.6 uA current, 1.5 million steps to 0.25 s), and
     `INTEGRATION` trap's draining cell in exact mode (15 million steps to
     0.6 s). Recorded divergences. At the floor a failing ring falls back.
   - **The floor step checked** (R78 (a)). A step at the floor (`h <= 2.01
     H_MIN`) is still taken as the one plain step it always was, but its
     two halves (below the floor) estimate it, and a failing estimate holds
     the next step at the floor instead of doubling it unchecked. Growth
     stays x2 (classic x1.5) and the floor restart at an edge (classic: a
     10 ns guard step, then its old step); a recorded divergence.
   - **TR-BDF2 in place of a backward Euler step** (R96, R98). A step that
     would be backward Euler, except the one after a discontinuity (a CV/CC
     regime switch is one), a floor step, a step on a CC rail (its rail
     capacitors and coupling bound have no TR-BDF2 form) and a step of
     exact mode's cell partition (its E and r are held from the SoC at each
     step's start, a first-order coupling longer steps stretch), is
     TR-BDF2: a trapezoidal stage to `gamma h` (`gamma = 2 - sqrt(2)`), then a
     BDF2 stage, solved as a backward Euler step over `h (1 - gamma) / (2 -
     gamma)` from the history `k vc_gamma - m vc_0`. Second order and
     L-stable: a junction's or a chip supply's partition keeps the damping
     backward Euler gave it and gains the trapezoid's order. Steps grow by
     the order-2 factor. A cell's charge bill (Rails point 9) has the same
     quadrature ready, `k (gamma h / 2) (i_0 + i_gamma) + h_2 i_1` (each
     capacitor's `C dv`), for when its partition steps by TR-BDF2. Classic
     steps backward Euler; a recorded divergence.
   - **The stiff fallback** (R81 (1)). A trapezoidal step that fails its
     estimate even at the floor meets a mode the trapezoid does not damp:
     the partition goes on by backward Euler until its next discontinuity,
     under either `INTEGRATION` value. Stat `27` counts the fallbacks.
     Classic's trap mode crawls there instead (cold 213/1, 582/5, 686/5);
     a recorded divergence.
   - Measured on the cold 1-800 hub-2 sweep's moved boards (every 16th, 156,
     against the fine reference): 12 closer, 142 within 0.05 W0, 2 farther:
     300/0 (a resistor's burn latch, ruling R80's subject) and 214/5 (1.0 to
     1.59 W0 at 6.3 ms, a trap-option board's start-up; divergence log,
     R88). Cold 61/5 declines (`-4`) where it ran: it sat at a chip's power
     edge already (fixed-host failures 4 of 20 before), under R78/R88: it is
     no R53 case (its 74LS00 chatters 1.3 to 1.9 V over its 5.775 V ceiling,
     40 ns holds, nowhere near the edge W1c point 1a's chatter entry slides
     at); 505/5 did at the rail Newton's edge until R80's stepped stress.
     Cold 360/3 runs where it declined: its decline was the trapezoid's.
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
   else `max(0.8, 0.3 vS)` and `max(2.0, 0.6 vS)`. An IC under a power hold
   (point 1a) is unpowered whatever its supply reads; `powered` above is its
   verdict only while it is free. In code every site that judges power goes
   through `digital::power::verdict` with the solver's hold table.
1a. **The power hold** (F5; classic's H8, `fix/power-off-stall`, 0.6.3;
   rulings D1-D4, D8, R44 (a), R45 (a), R51, R54 (a)). An IC whose own draw pulls its
   supply out of its power window, while its absence lets the supply
   recover, has no consistent power state, and a solve that iterates over it
   alternates. Each such solve counts every gated IC's verdict flips between
   its iterations: the rail loop's passes, the rail Newton's iterates, an
   iterated static partition's Newton iterates (point 3) and, across
   instants, `Rail` events and a hub partition's own steps and crossings in
   a row (Rails point 3). When the solve fails, or
   the rail loop cycles, with an IC flipped at least twice, the solve runs
   again in place (no rollback, no 1 ps step: an instant is classic's 1 ps
   probe already):
   1. the alternating ICs held powered, every other gated IC of that hub or
      partition at its last reading (one already held off stays so);
   2. those reading unpowered there starve themselves; on a CC rail
      capacitor's segment every alternating IC does when, powered, its draw
      turns the segment down, whatever its supply reads (ruling R44 (a): the
      capacitors hold the rail through both probes, and what the draw
      changes is the segment's slope);
   3. those are probed again held unpowered, which gives the drop their own
      draw causes;
   4. the solve runs with them held unpowered and the rest as they were;
      that is the instant's solution. An instant has no length, so a part
      whose supply reads low either way is held too (ruling D2 (a)); classic
      holds that one only at its host's floor step.

   No starving IC at step 2: on the rail, a probe in which every rail IC's
   free verdict is the one it was probed at is a consistent state, and the
   instant's solution, provided it passes the ordinary convergence check: the
   partitions that read the alternating ICs solve again from it with the
   holds as they were, each within Newton's tolerance (ruling R45 (a); the
   free passes alternated only on the way to it, as on G2bv's bc_ic seed 97,
   a rail landed at the power edge and re-solved a drift width under it).
   Classic rejects that step instead (its `partsHold`). Otherwise, and in an
   iterated static partition, no starving IC is `-4`, as is a probe, a check
   or a held solve that fails. A stepped partition's Newton runs no
   procedure: its failures stay its own (`-4` at `H_MIN`).

   **Release** (rulings D1 (a), R51 (b)). Classic's H8 holds a chip for one
   host step and retries it powered at the next; so does the core, once
   per host step (an `advance` call): at the first re-solve after the last
   call's end that reaches a held IC (on the rail, a rail loop whose passes
   solved, once it has settled; in an iterated static partition, its
   solve), or else at the horizon clamp ending the call, every IC held off
   is probed held powered, the rest as they were. At any other re-solve
   only one whose supply, held off, passed its release level is (below).
   Probed at every event re-solve instead, cold fuzz 34 tpl 0's 555, held
   while the rail it starves climbs 10 uV a `Rail` event (one each 20 ns to
   70 us), took 79,000 failed retries, each two rail passes more: 0.6 s for
   the sweep's 1.2 s, 0.16 s once per host step (pin
   `a_held_chip_is_retried_once_per_host_step`). The flag that arms the
   retry is set at load and at each clamp and journaled with the table. In
   exact mode the rail sits in a static partition, and a clamp retry that
   re-solves it is a solve of the rail's partition: the regime rules run on
   it, so a chip freed in a CV probe whose draw takes the PSU past its limit
   enters CC and starves again (cold 621 tpl 2, whose fine reference held
   126 mA in CV against 99.1 mA). One that
   carries itself there (its free verdict powered), or whose supply reads
   unpowered both probed powered and held off (its own draw no longer
   decides), is free again and reads at once (point 9); starving again at
   a later instant, it is held again with a fresh drop. The others stay held, and the probe
   leaves no trace, bit for bit: the static probe publishes nothing (the
   clamp re-solves a static partition holding a held IC only when its probe,
   from the published solution, frees or moves a hold; re-solved only to be
   probed, the partition moved by ulps), and a rail probe that frees nothing
   puts the solver's state back as it was, the hub's running sums included
   (not re-summed, as a whole restore re-sums them, Rails point 2: ulps on
   12 of the cold fuzz's 83 boards whose hold is never released). Re-solved
   back instead, a stepped partition re-seeded at each of its steps
   restarted its step each time, G2bv's bc_ic 178 hub 2 at 20 ns. Pin:
   `a_retry_that_frees_nothing_leaves_the_run_bit_for_bit_as_it_was` (a
   run whose every retry frees nothing against one with no retry, to the
   bit at every host step). On the
   rail, one freed that alternates again within the instant has its draw
   still deciding: the retry found no release, and it stays held, the
   solver put back as by a retry that frees nothing. Freed and held again
   at each retry, FXH's cold 830 tpl 3 (freed only because its supply read
   unpowered both ways; its PSU in CC with the gate held and CV without)
   and cold 696 tpl 2 and 677 tpl 3 (carrying themselves in the probe, but
   turning a CC rail capacitor's segment down once free, ruling R44 (a))
   chained their holds one sliding episode a host step, to the guard's
   `-4` (pin
   `the_host_steps_retry_reaches_only_static_partitions_and_a_release_that_alternates_at_once_is_none`). So a
   hold ends once its cause is gone, whatever the supply did (F5V's stale
   hold: a 74LS04 whose load vanished stayed dark because its supply,
   unpowered, could never pass the level its old drop set). The level stays
   the rail's trigger: the window edge the IC starved at, moved inward by
   the drop (the supply held unpowered less held powered) and a margin,
   two reached widths `η` of that edge, as a `Rail` event threshold located
   like a power-window edge (Rails point 4). A retry that still starves past
   its level takes a new one from the fresh drop with twice the margin (up
   to the drop): each probe solves the rail to its own tolerance, and a
   supply landing a few `η` short at each level two `η` further on never got
   there (cold fuzz 111 tpl 2). A chip held on a CC rail capacitor's
   segment is retried only once ruling R44 (a)'s draw threshold passes (the
   draw at its level plus what it adds within the limit): within an instant
   the capacitors carry it whatever its draw. Every hold is dropped,
   uncounted, at a successful `set_params`, `seed_memory` or
   `resetFailures`, where the circuit itself changes: a ping-pong still there
   re-holds at once. Stats 25 and 26 count holds taken and released (C ABI;
   classic's `_holdOffCounts.accepted` counts the first; a retry that stays
   held counts neither). The table, each hold's release and each IC's
   episode (below) roll back with a failed instant (Rails point 8). Nothing
   is paid while no IC is held: the table is empty and every verdict site
   takes one branch. Pins: the plumbing in `digital/power.rs`
   (`the_verdict_is_the_holds_and_the_window_tests_only_while_free`) and
   `devices/ics.rs`
   (`a_held_off_ic_on_an_ideal_supply_stamps_reads_and_commits_nothing_and_its_hold_rolls_back`),
   the procedure in `solve/rail_tests.rs`, listed under Equivalence harness
   rules, the power-gate ping-pong.

   **The slide** (ruling R92, R53; `hold.rs`, `HoldKind::Duty`). Since
   R92 the hold is the procedure's answer only where no state fits for
   another reason. A single starving rail IC whose edge **attracts**
   (powered, its supply leaves its window; held unpowered, it comes back:
   on a CC rail capacitor's segment, powered the segment falls and
   unpowered it rises, ruling R44 (a)'s case) slides along that edge
   instead of being held: it is powered, its logic runs, and every output
   drive (and so its supply draw) is scaled by a duty `x` in `[0, 1]`,
   Filippov's convex combination of its powered and unpowered stamps (an
   unpowered chip stamps nothing). `x` is the extra unknown that pins its
   supply at the edge: solved within the instant by the rail loop off a
   segment (the supply at the edge), and on a segment by the aim below.
   A chip whose supply reads low both ways is still held (ruling D2 (a)),
   one that carries itself is free, one fed by a foreign edge (F5V2's
   pulse-fed gate) is held each phase, and a self-fed chip (below) never
   slides. While it slides (`keep_sliding`) its duty is solved again at an
   instant that changed a driver or a source and once its supply has
   drifted `slide_drift` (W0) off the edge, a `Rail` watch on both levels;
   a slide that needs more than `SLIDE_FOLLOW_MAX` re-solves in a host step
   is held as before R92 for that host step and re-entered at the next
   (`hold::reenter_slides`). A slide is no hold: stats 25 and 26 do not
   count it. It is dropped with
   every hold at a `set_params`, `seed_memory` or `resetFailures`, and rolls
   back with a failed instant. Exact mode does not slide yet: its chips
   are held as before.

   **Two states beside a slide** (ruling R103 (b); `static_part.rs`,
   `two_state_view`). The averaged stamp is Filippov's average for what
   the chip's draw does to its supply, but not for a node with no
   capacitor beside it: in the resolved chatter such a node sits at the
   chip's powered solve or its unpowered one, never between, so its mean
   is the duty's weighting of the two, not the solve of the averaged
   stamp. A static hub partition touching one sliding chip (duty strictly
   between 0 and 1) is therefore solved twice, the chip held at duty 1 and
   held off, each the ordinary cached solve under that hold (neither
   depends on the duty, so a slide's re-solves replay them), and publishes
   the duty's blend of the two: its voltages and sensitivities, the draw
   and slope the rail sees, and every current through it from each
   state's own voltages, the junctions in both KCL walks, an LED's
   current and channels, an IC's driver and pull currents and a 555's OUT
   (the chip's stamps carry current only powered), an LED's stress peak
   and latch value and its thermal power. Its band is the narrower of the
   two. A stepped hub partition with no capacitor of its own (stepped
   only because it reads a running ramp) is algebraic too, and is solved
   the same way at each seed and step (`stepped.rs`, `solve_two_state`),
   its sensitivity per state at commit, its draw in a slide's trial per
   state. On a CC rail the two states share the one rail the step solves:
   both states' nodes and the rail are the unknowns, the rail row the
   duty's weighting of the two states' rail rows with the rest of the
   rail's draw, its capacitors and the limit (`take_cc_two_state`), which
   is Filippov's form exactly. Each state solved on a CC rail row of its
   own put two rails 6.5 mV apart over a 10 ns step on cold 2575 tpl 2,
   and the blended draw the rail loop read drifted off the limit between
   its solves (2.3 times the events, the slide given up after one source
   pulse). Cold 2575 tpl 2's node 2 (105.7 ohm from the rail, 820 kohm
   to ground, OUT LOW of the sliding 555) slides at x 0.539526, node 2
   1.028632 V, both the hand values (0.516649 and 0.5557 V through the
   averaged stamp, 419 W0 under), at 78.8k events and 3545 slide solves
   to 1.2 s against 78.7k and 3570. A partition beside two sliding chips
   keeps the averaged stamps; one with a capacitor keeps them too, the
   convex combination of currents its capacitors integrate. HG1's CC ramp: the
   duty that keeps the rail at 4.275 V is 0.359346 (1.8182 / 5.0597 mA,
   the LED's powered current; 0.135077 through the averaged stamp), and
   1y reads 2.29 V over 1.50-1.55 ms against the fine reference's 2.32 V
   (exact mode resolving the chatter, its powered fraction 0.6458; 3.005 V
   through the averaged stamp, 280 W0 over). The cell of the slide test
   slides at 0.212133 (0.030755), 1y's mean 0.8 V either way.

   **The warning** (ruling R92, R53; `slow::warnings`). A chip sliding along
   its power floor is a board that cannot power that chip, which classic
   reports by declining (R89's `rail_ic_power_chatter`). The core runs on
   and says so: at the end of each rail loop every chip still sliding at its
   floor is latched once on `bo_engine_records` channel 1 as code 3
   (`rail_ic_power_chatter`, classic R89's code): `since` the instant,
   `value` the floor (V), `limit` the PSU's current limit (A) when the rail
   is a PSU in CC there, else `NaN` (a cell, or a PSU in CV through a
   series part), `pin` -1, `aux` 0. Latched where the loop has settled,
   so a slide a probe or a retry tried and gave back latches nothing, and
   through the slow state's journal, so a failed instant, a restored
   snapshot and `resetFailures` treat it as they treat a failure (a slide
   still running after a reset latches again at the next loop's end). The
   facade builds Dev's text from it (`failures.ts`,
   `railIcPowerChatterMessage`, the one place it is worded): the message
   `U1 isn't getting enough power to work properly.` and the warning's
   `detail` line `Supply stuck at 1.8 V (limit 72 mA)`, its limit clause
   only where a limit applies, both numbers to three significant figures;
   classic R89 is to give the same. A slide at the ceiling latches nothing
   until its message is worded. Pins:
   `a_chip_sliding_along_its_floor_latches_its_warning_once_naming_its_floor_and_the_supplys_limit`
   (`solve/rail_tests.rs`) and `test/core/ic-engine.test.ts`'s
   `rail_ic_power_chatter`.

   **The chatter entry** (rulings R92, R95 (b), R97 (b); `hold.rs`). A
   chip whose draw reaches its supply through a capacitor that carries it
   within any instant (cold 757 tpl 0: a 555's OUT sinking from behind a
   0.64 ohm fuse and 4.94 mF) moves neither end of the slide's own solve, so
   its edge shows only as a chatter. From the hold that follows its second
   short self-released one, with its supply within W0 of the edge, it slides
   at the powered fraction of the last held-then-powered cycle; that cycle's
   held and powered drift rates are the duty's first bracket, at 0 and 1
   (Filippov's drift is the duty's blend of them). Its duty is then
   corrected across instants: once the supply has drifted W0 off the edge,
   the drift rate measured since the last correction puts the duty on one
   side of its zero-drift duty, the bracket keeps the newest point on each
   side, and the next duty is the bracket's interpolation to the rate that
   brings the supply back to the edge over that span, or the bracket's
   midpoint when the same end moved at the last correction too. A bracket
   end that a new measurement contradicts goes back to the chatter's own
   phase. Not a secant: one across two corrections read the board's own
   change between them as the duty's (757's PTC fuse tripping, ten times
   the slope); not a fixed slope: 22 tpl 2's chatter showed its powered
   phase only as a sub-microsecond esr step, -65,000 V/s against the
   slide's few V/s. A correction waits for the supply to cross back over
   the edge since the last one, or to drift a further W0 out, a `Rail` watch
   on that level: a rate read right after a correction is the settling at
   the band's level (1.9 V/s over 83 ns took 757's duty to 1). The drift is
   measured across driver changes, which a chip toggling as it slides makes
   at its own pace. A chip that, fully powered, still drifts as held (or off,
   as powered) leaves the slide. On a CC rail capacitor's segment that is
   checked directly (`hold::chatter_attracts`, R53-P11, R99W's defect): at
   the entry and at every correction the segment's flat residual is probed
   at duty 0 and at duty 1, each end's draw read at the edge with the
   stepped hub partitions' draw over the span, as the class-B exit reads
   them; the slide holds only while fully on carries the supply out of its
   window and fully off back in, and leaves (freed, as any exit, the hold
   procedure deciding from there) where both carry it the same way. Each
   end's turning point is kept as a threshold on the duty's draw at the
   edge (each end's residual moves with it volt for volt), watched as a
   class-B slide's draw exits are and checked at each instant, so the exit
   comes as the board crosses it and not at the next correction. Unchecked,
   cold 2575 tpl 2 under R99W's ramp-following CC segment entered a chatter
   slide at 23.187 ms as switch 14's pulse dragged the rail through the
   555's 1.8 V floor whatever the duty, its corrections halved the duty at
   each W0 of the fall (3e-128 at 0.59 V, 636 W0 under, a correction and a
   `Rail` event at each), and the run crawled a microsecond of simulated
   time per 1.4 ms of wall until it stalled at 0 V; now every stop history
   (1 to 33 us stops from 19.2 or 22.4 ms, and event by event) runs the fall
   in under 0.2 s of wall, its slides within 1.0 W0 at every stop. A
   chatter slide's corrections also count per simulated time toward the
   follow cap (`SLIDE_FOLLOW_MAX` per `SLIDE_FOLLOW_SPAN`,
   `Episode::corrections`),
   whatever the host step, so one the exit does not catch is given up as
   any slide that cannot be followed, never a hang. Pin:
   `a_chatter_slide_leaves_where_its_edge_stops_attracting_at_every_host_cadence`
   (a pulse dragging a PSU's rail across a 74LS00's floor both ways, the
   slide forced to chatter by the `ForceChatter` seam: the exits at the
   pulse's 3.475 V and 4.7565 V by hand, at the first instant past them,
   at R53V2's cadences and two stop histories). Off a segment the measured
   drift alone decides, as before. 757 tpl 0 slid so from its third hold;
   since open item (c) it slides on its capacitor's segment from its first
   (0.17284 s, x 0.741), runs to 1.2 s at x = 0.643 against 0.6443 by hand,
   then 0.78 once its PTC fuse trips at 0.338 s (0.787 by hand), its rail
   within W0 of 1.8 V from 0.19 s, the trip included, at 0.1 to 1 ms stops
   (1.0004 W0 at 0.92 s at 10 ms stops); as it enters, 1.11 to 1.36 W0 (the
   relative-degree-2 must-close item). On a capacitor's segment whose aim back at the edge needs more
   than the limit leaves at any duty, yet one holds it flat, the slide takes
   the end that turns it back fastest (off under the edge, on over it); cold
   766 tpl 0 then slides to 0.507 s instead of 0.191 s. **A chip with a
   loop from an output back into the logic that drives it**, through
   passive parts with no capacitor (`hold::self_fed`, at build: resistive
   parts, switches in either state, fuses and LEDs, stopping at fixed nodes
   and at the chip's own supply pins; an output drives the outputs whose
   inputs it reaches, by `digital::output_inputs` where the model computes
   each output from its own gate's inputs, the quad gates and hex
   inverters, else every input; a loop is a cycle among them, an output on
   its own inputs included) never slides (ruling R97 (b); a 74LS00's 1Y on
   its own 2A and 2B with 2Y open is no loop, R53V): its inputs read its own averaged
   output between their thresholds, and 22 tpl 2's 555 (OUT on its TRIG and
   THR) rang at 1e9 events a second; it chatters to the guard's `-4` as
   before, the message naming the ruling (also 688 tpl 5's 555). Open item:
   a self-feedback chip at a starving edge, to be checked against a longer
   fine reference.

   **Filippov's exit** (ruling R92; `hold::slide_exits`). A slide solved
   within the instant (not a chatter slide) leaves where an end of its
   duty's bracket stops bracketing: duty 1 no longer turning the supply
   back (or the segment down), duty 0 no longer letting it come back. Each
   solve keeps the residuals of its probes at duty 0 and 1 (probing an end
   its search skipped, then solving the duty again) and turns them into a
   watch on what the board moves at a fixed duty: off a CC rail capacitor's
   segment the supply, each end's residual being its own supply's distance
   past the edge (the cold 4.06 V cell: the powered end 8.06e-11 V past,
   freed 8.10e-12 s after the crossing, the old hold's release level
   floor + drop + 2 eta; left to the W0 re-solves it sat at x = 2e-11 for
   0.54 ms, and a watch placed from the bracket's slope, 0.341 V per unit
   duty, came as late); on a segment the rail's draw at the edge, each
   end's draw being the duty's plus the two residuals' difference (the rail
   capacitor holds the aim the same at every probe), watched along the
   segment's closed-form terms as R44 (a)'s held chips are. The re-solve at
   the watch decides. An end already past its exit at the solve (the aim
   alone keeping it) gets no watch: one would fire at once, and cold 766
   tpl 0 was held 51 times by 0.219 s. HG1's CC ramp (5 V, 20 mA, 1 uF and
   100 ohm into 10 uF, a 74LS00's 1y into an LED): duty 1 at 1.638121 ms
   against 1.638516 ms by hand (the rail up to W0 over the edge charges the
   10 uF faster, up to 0.57 us); found at the W0 re-solves alone, about 22
   us apart, the duty sat at 0.942 to 1.6475 ms, 1y 7.4 W0 under the fine
   reference over the 5.8 us past the exit. From the exit to 1.70 ms
   against exact mode at 0.2 us (its episode guard lifted in a scratch
   build; it declines at 1.449 ms otherwise): rail 0.73 W0, 1y 0.52 W0, the
   LED's current 0.05 W0.

   **The aim on a segment** (`hold::segment_aim`). Off the edge by `off`,
   the duty aims the segment back at the edge over the span the supply took
   to drift off: `C u' = dir (I - D)` with the draw moving with the rail by
   its own conductance `g = dD/du`, read at the solve (the direct devices'
   slope and the static partitions' conductance: a stepped or closed-form
   partition's is its capacitors' companions at its step, a capacitance on
   the aim's scale; counted, cold 766 tpl 0's read 4e6 S and its duty swung
   between its ends), so the duty's draw at the edge `w` (the draw's line
   at the supply carried to the edge) is the limit plus `dir g off / (exp(g span / C) - 1)`, and
   `dir C off / span` as `g` goes to 0 (also for `g <= 0`). Read as a
   current at the supply instead, the aim set the segment's equilibrium at
   `off (1 - C / (g span))`: cold 2575 tpl 2 (a 555 on 11.9 nF, DIS on, `g`
   0.0575 S, `tau` = C / g = 0.2 us) sat 0.92 of W0 off, measured 1.71 mV
   against 1.75 by hand, back past W0 every 2.7 us, to `SLIDE_FOLLOW_MAX`
   in under a millisecond. Only within 2 W0 of the edge, where the draw's
   line holds: further off the supply is read as before (cold 627 tpl 4,
   its ground pin's step 6.95 V under its floor, balanced a draw the rail
   never had at a duty of 0.373 and stalled at 20 million events).
   2575 tpl 2 then slides at 0.51665, 0.328264 and 0.483920 by hand per
   switch state (within the band's 6.1e-4), its re-solves 472 to 198 over
   its run before re-entry after a fallback (1,736 with it, the slide
   re-entered and followed to its cap at each pulse edge, 13 holds against
   4 and 124 sliding stops against 1); 627 tpl 4 (13 re-solves against 12),
   766 tpl 0, 757 tpl 0, 175 tpl 2 and 696 tpl 2 run as before.

   **The aim through a stepped partition** (open item (c), closed; ruling
   R92, `hold::stepped_draw_over`, `hold::rail_settling`,
   `SteppedSolver::trial_rail_draw`). A stepped hub partition's capacitors
   carry the rail over a span its contribution (companions `C / h` at its
   own nanosecond step) does not show: cold 766 tpl 0's C17, 13.5 mF to a
   node 2.82 ohm and 105 uF hold to ground, read as the rail's own 2.66 uF,
   left the aim a thousand times short, a re-solve every 0.3 to 0.6 ms and
   the rail up to 2.2 W0 past the edge between them. The aim now reads each
   such partition's draw at the end of one backward Euler step over the
   span with the rail held at the edge, the chip at its duty (a trial: no
   state changes, `a_slides_trial_steps_leave_no_trace`), in place of its
   contribution, in the residual, the flat solve, the exit thresholds, the
   at-edge check and the "on its way" check alike. The span is the rail's
   own settling `C_eff / g_eff`, read once as the slide enters from the
   rail's admittance over 10 ms and 10 us (fitted to `g + C / T`), the chip
   at the duty its entry solved (read with the chip still at its unpowered
   probe, cold 2575 tpl 2's rail had 1e-12 S of its own and the fit took
   C17's 0.73 ms RC for the rail's 0.34 us: the slide could not follow
   C17's decay, and whether a re-entry slid depended on where the host
   stopped; `a_555_sliding_on_a_cc_rail_through_a_capacitor_s_transient_
   slides_alike_at_any_host_cadence`; a fit at the duty that is not
   positive keeps the entry's reading), at least
   `H_MIN`, and not bounded by the host step (757 tpl 0 at 0.1 ms stops,
   bounded: an aim over 0.1 ms no duty meets, the chip free from 0.1733 s to
   0.8 s, 4 s of wall; unbounded it slides from 0.1726 s, 0.25 s of wall);
   with no such partition, the span since the last solve, a host step at
   most, as before. A re-solve at an instant the slide's own commits
   changed aims over what is left of the last span (757: 2 mV off 10 ns
   after a solve, the residual at duty 0 was 224 A). "On its way" past the
   band needs the rail heading back now (the draw against the limit) as
   well as over the span: over the span alone, 2575 tpl 2's 11.9 nF rail,
   solved at -1.9 mV at a pulse edge, ran on to +2.3 mV 0.16 us later.
   766 tpl 0 under the trapezoid: within W0 at every schedule (worst
   0.993 W0 at 1 ms stops), re-solves about 1.8 ms apart (270 from 0.25 to
   1 s at 1 ms stops); its flat-at-an-end run 0.30 W0 at 0.5 s (1.55
   before). 2575 tpl 2: 78,733 events against 91,057, wall 2.05 s
   against 2.00 s; with the entry's settling read at the duty (R53-P10)
   107,684 events and 3,574 slide solves against 78,832 and 3,543, the
   extra events in switch 21's pulse phase (0.38 to 1.1 s).

   **Solving again at the edge** (ruling R113 (1), R53-P11; `keep_sliding`'s
   `at_edge`, `Episode::solved_imbalance`, `EDGE_FLAT`). A slide on a CC
   rail capacitor's segment whose supply is back at its edge is solved
   flat there again only where the flat solve's inputs changed: the last
   solve left the segment off flat by more than `EDGE_FLAT` (1e-2) of W0,
   read as the offset its excess draw holds the segment at (`excess / g`,
   `g` the rail draw's conductance), or the draw has moved by W0/2 of it
   since (a discrete flip is solved again anyway). A rail its own aim
   brings back is no new input: cold 2575 tpl 2 under R99's
   ramp-following segment was solved flat at each of C17's 0.44 us steps
   (0.2 uA of excess against 0.0577 S, 3.5 uV), 170 solves in 77 us, to
   the follow cap and a fallback left Off at the 4.98 V setpoint, and
   whether its rail landed within two reached widths of the edge, a
   difference of 1e-11 V, decided it; 1 ms host steps took 114 re-solves
   to 0.2 s against STOPS_2575's 61, now 57 to 58 at every schedule. HG1's
   CC ramp, whose solves at the edge each halve the draw's excess, still
   converges to its flat duty 0.359346 within 2 ns (0.3593472; stopped at
   W0/2 it sat at 0.359662). Pins:
   `a_slide_solved_flat_at_its_edge_is_solved_there_again_only_when_its_inputs_move`,
   `hg1s_cc_ramp_solves_its_slide_flat_at_the_edge_until_the_segment_is_flat`.

   **A hold retried as a slide** (ruling R113 (2), `hold::reenter_slides`).
   A chip held on a CC rail capacitor's segment is retried only once ruling
   R44 (a)'s draw threshold passes, which reads the draw at the release
   level (or the setpoint in CV) with only the chip's own draw on the way
   to the edge. Where it would slide (not self-fed) and its edge attracts
   at the segment's ends (`chatter_end_rates`), it is retried each host
   step: past its release level it is freed, as R51 (b)'s retry frees a
   chip, and the segment carries its rail down to the edge where the
   procedure slides it; at or short of it, its slide is solved from the
   hold (`slide_on_rail`) and kept only where it slides. A rail held by a
   pulse through 100 ohm while slides were off sat Off from 4.05 ms to
   9.78 ms with its edge attracting; now it slides from the next host step,
   at the hand duty `(8 mA + (Vp - 4.275) / 100) / 12.815 mA` within W0
   (`a_chip_held_where_no_slide_was_possible_slides_once_its_edge_attracts`).
   Freed and left to the passes instead, cold 757 tpl 0's 555 reloaded at
   0.25 s (W2c) went free under its floor and stayed free at 0.3 s.

   **A slide in its supply's own partition** (rulings R92, R113 (3);
   R53-P11, R53-P12; `static_part.rs` `own_two_state`, `stepped.rs`
   `solve_own_two_state`). A chip supplied through a series resistor, an
   RC filter or an LED, its supply node in a static or stepped partition
   rather than on the hub rail, slides where its edge attracts (held
   powered its draw takes the supply out of its window; held off the
   supply comes back) instead of declining at the episode guard (R53V2:
   40 of its 42 fed-supply variants, "held 51 times in one sliding
   episode"). It enters where the partition's hold procedure finds it
   starving with its edge attracting (`hold_in`, `hold_around`) and then
   slides with R103 (b)'s two-state duty: the partition is solved with the
   chip powered (duty 1) and unpowered, its supply pinned at the edge in
   each (the supply's row replaced by `v = edge`, through the Newton, the
   hub polish, the sensitivity and the refinement), and the duty `x` is
   the one that balances the supply node's KCL between them, `x F_on + (1
   - x) F_off = 0` (Filippov's convex combination). The published state
   is the blend; the device currents read each state's voltages
   (`two_state_view`), so the chip's own stamps carry `x` times their
   powered current. On a hub the draw, its slope and the sensitivity
   carry the duty's own slope in the rail, `dx/du (on - off)`, and the
   band is the narrower of the states' and of the rail moves that take
   `x` to 0 or 1 or leave its second-order term past W0. A stepped
   partition (a capacitor on or behind the supply) solves each step in
   the two states at once: unknowns both states' nodes, the duty, and on
   a CC rail the rail; a node a capacitor touches is shared by the two
   (its charge does not jump between them), its row the duty's weighting
   of the two states' rows, as the CC rail's is (`take_cc_two_state`);
   the duty's row is the supply's KCL weighted alike, its capacitor's
   current included. Where no duty in [0, 1] balances it (both states
   carry the supply the same way) the chip is freed (powered, its supply
   moves into its window) or held off, and the partition is solved as any
   other. The averaged stamp the first build used (the chip's
   conductances scaled by the duty) put the 4.3 V / 5 ohm / 100 ohm board
   at x 0.555079; two-state it slides at I_feed / I_on = 3.238095 mA /
   3.523915 mA = 0.918891, the output load setting it (0.485510 into 470
   ohm). Its entry takes the edge the hold procedure found the powered
   supply past (`entry_edge`, `entry_floor`): read from the supply alone, a
   switch toggled under an RC-fed chip at 5.3 V took the ceiling, nearer,
   and held the chip off above its floor (10 of 200 fuzz boards). Measured
   at R53V2's six host cadences on the four variants (series R, R + C, RC
   then R, LED): the supply within 1.4e-15 V of the edge at every stop, the
   duty alike across cadences to 1e-13 on three, and on the RC then R board
   to 5.6e-6 at 2 ms, inside its capacitor's transient (1y 46 uV), 3e-12
   after. R53V2's rail fuzz (1,260 boards x 4 cadences): no class changes
   against R53-P11's base. R53-P11's own-partition fuzz (200 boards x 4
   cadences): 163 slide, 2 cadence mismatches (10 with the averaged duty),
   7 errors (as averaged; 66 before any such slide), and no silent wrong
   state but the probe's own false positive (32 LED-fed boards whose chip,
   free, is unpowered under its floor). Pins:
   `a_chip_fed_through_a_series_resistor_slides_at_the_duty_its_output_load_sets`,
   `a_chip_on_an_rc_filtered_supply_slides_at_the_duty_its_output_load_sets`,
   `a_chip_with_a_capacitor_on_its_supply_and_without_one_slides_at_one_duty`,
   `a_chip_behind_an_rc_and_a_second_resistor_slides_as_its_capacitor_relaxes`,
   `a_chip_fed_through_an_led_slides_at_the_duty_its_output_load_sets`,
   `a_chip_sliding_in_its_supplys_own_partition_slides_alike_at_every_host_cadence`.
   Open: two chips sliding in one partition would need four states, and
   neither is solved in two (`own_slide_of` finds none), so each keeps
   the duty it entered at, averaged stamps, as two chips beside one
   partition do.

   **Entry at the crossing** (ruling R113 4 (a), R102 (a)'s cut in default
   mode; `stepped.rs`). A step of a stepped partition holding a free chip's
   supply is cut where that supply crosses a window edge, by regula falsi
   on the step as exact mode's is: the chip's verdict flips at that commit
   and the step after it, whose Newton finds the edge attracting, enters
   the slide. Uncut, `sp_board` (a) from a 0.3 ms host stop took a 157 us
   step to 4.10 V judged off and slid from 1.13 ms (20 us stops: 0.70 ms);
   cut, they enter at 0.587808 ms and 0.593997 ms, 1.7 and 7.9 us after
   the floor's crossing at 0.586145 ms (within the 31 us a powered supply
   takes to fall W0 under the floor), the supply within W0 at entry
   (`an_rc_fed_chip_enters_its_slide_where_its_supply_crosses_its_floor_at_every_host_cadence`).
   The 7.9 us is the coarse schedule's own charge-up, not the cut: each cut
   lands on its schedule's crossing to the cut's tolerance and the slide
   enters 10 ns later at 3.959998701 V on both; the 5 ohm cell's drop held
   at the last commit over a stepped hub partition's step (Rails point 4)
   leaves the coarse charge-up 11.3 mV (2.8 W0) behind the analytic curve
   at its crossing (15.8 mV at worst; 1.1 mV from an ideal source), an
   open item of its own. A seed whose free chip's supply sits on its edge
   (within the cut's tolerance, read from its capacitor's charge where one
   holds it) takes the two-state test there and slides from that instant
   (`stepped::seed_on_edge`, R53V3's F2): the commit a cut ends on, and a
   reload mid-slide, which W2c seeds with power judged afresh. Seeded
   powered there, the capacitor holding the supply through the seed,
   `sp_board` (a) reloaded at its slide published 1y powered, 85.9 W0 over
   the blend, until the step after
   (`a_chip_sliding_on_a_capacitor_slides_from_the_instant_of_a_reload_and_of_a_cut`).
   Cost (SAP-1, `--gate`, 3 reps, against 8c9b11e):
   steady ideal +0.7 % frozen and running, battery -0.8 % and -0.3 %;
   start-up within -1.7 % to +1.3 %; the SAP-1 publishes bit for bit, its
   partitions crossing no free chip's edge.

   **The hold-episode guard** (rulings R51, permanent, and R54 (a)). A chip
   whose supply slides along its power edge (powered, its draw takes the
   supply out of its window; held off, the supply comes back) is held,
   freed and held again at the pace of its own edge, each hold right and
   none settling: on the cold fuzz that pace ran from femtoseconds to
   milliseconds, up to 110,000 holds in the sweep's 1.5 s, the run crawling
   or one advance frozen for 16 s. An IC's **episode** counts its holds as
   each is released: a hold counts when it lasted less than one host step
   (the `advance` call it ends in) and its release was the chip's own; a
   hold that lasted a host step or more (classic's H8 cadence) ends the
   episode, as does a release that a foreign driver change in the instant
   caused (a waveform source's edge, a commit by another device; a switch
   is a `set_params`, which drops every hold: the evidence NE555 point 11's
   latch test reads, ruling R49 (d)). A hold taken again with no release
   between, or a retry that moves a held chip's level, is the same hold. The
   release that counts the 51st short hold of one episode declines `-4`
   (the board falls back to classic). R51's first rule (each hold within
   100 times the last one's length) both fired on a legitimate run and
   missed a fast chatter (F5V2): a 4.5 <-> 5.5 V pulse feeding a 74LS00
   through 30 ohm holds it each 6 ms low phase and the edge frees it, and
   was declined at its 51st phase, 0.4955 s (now each phase is an episode
   of its own: 1 s run, 101 holds, 100 releases); cold 577 tpl 5 and 670
   tpl 2 (10 ns held, about 1 us powered) took 10,000 to 13,000 holds and
   3 to 4.5 s to the wall cap (now `-4` at their 51st release, within the
   first host step). Measured at F5m's tree (drafts/f5m-logs): the cold
   fuzz 1-800's 5,600 runs have 33 guard declines, every one a `-4` at
   c5b1a13 (22 rail loop, 11 rail Newton), none of them a board that ran at
   c5b1a13 or 1a84f72, and the slowest 1.7 s (482 tpl 0, as at 1a84f72);
   206 runs that complete take holds, at most 108 (527 tpl 5, where 66 of
   the 107 released lasted a host step or more, so no episode passed 4). Cold 24 tpl 0 and 248
   tpl 5, which ran at bcbce27 with one hold, decline: their chips relax at
   the edge (24: 40 us held, 11 us powered; 248: 49 and 29 us), each hold
   far shorter than the sweep's host steps and released by the chip's own
   supply (named exit-gate exceptions, ruling R55 2; "What F5 leaves"; cold
   688 tpl 5 joins them, R59 (2), sliding at a 555's floor in 51 holds, and
   482 tpl 0, R64 (3), sliding at its gate's floor to 0.104 s; and,
   R65 (a), cold 271 tpl 4 and 597 tpl 5, which R63 (2)'s pace trigger
   promotes to the stepped path and which then meet a 74LS00's power
   floor: 271/4's CC rail Newton fails, 597/5 slides in 47 holds to
   1.36 s). Pins: `a_gate_sliding_along_its_power_edge_declines_after_fifty_short_holds_in_one_episode`,
   `an_episode_counts_only_short_holds_its_chip_released_itself`,
   `a_gate_fed_from_a_pulse_is_held_each_low_phase_and_runs`,
   `the_stale_hold_boards_run_a_second_of_pulses` and
   `a_chip_chattering_at_its_edge_in_ten_nanosecond_holds_declines_within_its_first_episode`
   (`solve/rail_tests.rs`).
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
   |x_old|)` per node, classic's own form, at most 50 iterations). One
   holding a free IC supply that fails so runs once more from the same
   start, its Jacobian taking the supply terms restamping misses (each
   driver conductance's slope in its supply, each pull's target), as the
   hub polish does (point 9 of LEDs); a stepped partition's step does the
   same. Frozen, a 555's OUT at a low supply contracted the iteration by
   only 0.89 a step, and another board 2-cycled across the power gate (cold
   fuzz seeds 1120 tpl 2 and 1325 tpl 0, F5e). Not at the first try: the
   full step can overshoot a solution just inside the power window out of
   it, where the frozen one creeps up to it (FXBv's fed gates). The power
   hold (point 1a) reads the first try's verdicts. Failure of both is `-4`.
   One holding a junction then runs one polish iteration (LEDs point
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
   state. An unpowered IC reads nothing. An IC held unpowered (point 1a)
   reads nothing either, and a rail IC held so watches its release level
   instead of its power window (Rails point 4). An IC reads its supply nodes and
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
   delta, as classic's first powered pass does. A power hold (point 1a) is
   an outage: its release is the first powered evaluation.
10. **Readouts** (codes 50-61). A digital slot holds the pin's committed level
    (electrical: HIGH, or released for open-collector, is 1) while the pin is
    driven, and `NaN` (an absent key) while the IC is unpowered (held
    unpowered included, point 1a), or the pin is committed HIZ or has never
    committed. Code 40 (NE555 point 12) and codes
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
     included (it only chooses delays, which are zero while settling for a
     driver not carried). It is in the evaluator's snapshot, so it rolls
     back with a failed instant; a cold load drops it, and a carried one
     seeds it.

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
   rolls back with a failed instant, and is rebuilt uncommitted by a cold
   load, so its settle is all first observations at zero delay. Classic
   carries the `delay:/pending:/due:` slots across `load()` by id and kind;
   so does the core's carried load (W2c, "Carry"), whose carried drivers
   keep their real delays through the settle. HIZ commits are ordinary commits:
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
   with an exact thermal identity (Q-O21, §6.3), and it is W1g. Until W1g
   the compiler declined such a `led`
   (`unsupported-feature:led-thermal:<componentId>`, the circuit ran on
   classic): one whose `catalogUid` classic's `_thermalProfile` accepts
   (SE:1830-1849), decided with `thermalProfileForComponent(comp)`, the
   body of `_thermalProfile` without its cache, exported from `sim-engine.ts`
   for this and for W1g. So through W1f no `led` that reached the core had
   thermal state, and no known divergence was registered for it. W1g
   removed the decline (G3 dispatch 2; "LED records: the `led-thermal`
   decline is lifted") when it ported self-heating: it feeds that LED's held
   junction temperature into point 2 and re-solves its partition when the
   held value moves (D19, orchestrator lock; W1g point 4). The tempco is
   the fixed `-2 mV/°C` (`DEFAULT_VF_TEMPCO`); no param overrides it. The law's inputs are fixed
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
   with no polish, save the test's form (W1c point 3). A hub partition (one
   on a rail) holding a free IC supply or a junction goes on to the
   solution itself: Newton steps on the exact Jacobian, the stamp matrix
   plus, with a free supply, the supply terms `R_out` restamping misses,
   until the step is rounding (rulings R19 (a), FXC, and R20 (a), FXD,
   2026-10-21; Rails point 3's band below, `static_part::polish_hub` for
   the stop rule). A step that stalls short of rounding, or 8 steps without
   arriving, is `-4`. On a stall the solve keeps, of every iterate the
   polish evaluated and the stepped one, the one with the smallest residual
   (each node's KCL imbalance over its own conductance, in volts), and has
   arrived when the step that iterate asks for is within the residual's
   rounding; a solve whose rounding floor (Rails point 3's band) passes
   0.01 of W0's tolerance is then refined, the residual summed element by
   element with compensated sums, the correction solved through the
   sensitivity's factors (ruling R23, FXE 2026-10-21).

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
    is `-7`, W1c's rule, and the circuit falls back to classic. The
    evidence is the latch's own, the same in every mode (ruling R49 (d),
    F5g): a latch that flips and then flips back within one rollback window
    (an instant, a parameter change, a memory seed), with no other device's
    driver changing between the two flips, has no consistent state there,
    since the window's solves are deterministic and the state it returns to
    is the one that flipped it, so the window is declined at once rather
    than at the delta cap. A flip back after another device changed answers
    that change, not the latch's own output: a delta-cycle hazard (one edge
    reaching a zero-delay NAND by paths of unequal depth, its one-delta
    glitch on TRIG setting the latch and THR taking it back) settles, and
    that flip counts as a first one
    (`a_zero_delay_hazard_into_a_555_settles_its_instant_rather_than_declining`). This holds where the second solve
    differs from the first within W0 and lets the cycle quiet: in exact
    mode a 555 astable whose OUT load drags a CC rail below half its
    setpoint (Known follow-ups (W1g), "What F5 leaves") reset and set within
    an instant and held high on a rail its re-solve put 5.5e-5 V above the
    step's, and each step after it re-planned the reset at its start until
    NP2's guard declined the crawl with `-4`. A latch due at once (a slow
    device's, W1g point 14) starts the evidence afresh: a flip back after
    it answers a changed circuit, not the 555's own output. Load settling
    has no window and keeps the cap alone. A latch that flips once per
    instant, its input held on a threshold by its own output across
    instants (DIS on TRIG through a capacitor, THR above its threshold:
    cold fuzz seed 145 tpl 4, instants 5 to 70 fs apart), is not this
    evidence and still declines with NP2's `-4` in both modes.
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

References are to spec chapter 8, §10.1 and the delta table (a bare `Dn`
is that table's row); classic line citations are at `2b5faf1`, and "W1f Qn"
is a ruling ("Kind codes and records"). R-numbers are the coordinator's W1f
rulings (from 2026-10-15), on the owner's priority: physics as accurate as
possible while fast enough to run a whole breadboard computer. "Ruling R3
(b)" is option (b) of the third; R2b revisited R2's question.

**Who found what.** Handles in this section and in the harness rules name
the W1f worker that built or found a rule; each one's commit on
`wip/w1f-int` names it (`WIP(W1f F1 round 9): ...`), and a verifier's or
investigator's findings land in the commit of the round that took them. F1
is the rails worker: F1 round 1 (`12c7db6`) to round 15, F1rN its round N
(rounds 14 and 15 on `wip/w1f-r14`, merged at `95c5e7e`). Verifiers: F1v23
(rounds 2-3), V4X (round 4, FX1 and FX2), V5 (round 5), V6 (rounds 6-7,
exact mode, ramps, the merges), V8 (round 8), V9 (round 9), V11 (rounds
11-12), FX45v (FX4 and FX5). FX1-FX5 and EX1 are the exact-mode fixes
(Rails point 9); F3b the device-state exports; F4 the W1f fixtures and
harness; F6 the resolved-ramp readers (Sources point 3); INF1 the
investigation of an infinite published voltage (taken in round 7); INV2
that of the SAP-1's rail-loop cycle at 2.897 s (taken in round 9); HG1 the
ping-pong hang fix; NP1 the no-progress guard, and NP2 its redefinition by
state. F5 is the ping-pong hold (W1c point 1a), on `wip/f5`: F5a the hold
table and its verdict, F5b the procedure, its triggers and releases (rounds
2 and 3: R44, R45), F5c the CC seed's charge balance (R28), F5d the contract
and the harness.

Constants, all from P3:35-38 unless marked new:

| Name | Value | Use |
|---|---|---|
| `RAIL_DELTA` | `1e-6` V | the finite-difference step of the sensitivity |
| `RAIL_RESOLVE_V` | `0.025` V | the cap of a curvature band, and the band of a curved hub partition without one outside CC; an affine closed-form segment's band is not capped (Rails point 3) |
| `RAIL_BUCKET_V` | `0.05` V | the rail band in the static cache key |
| `RAIL_SCAN_V` | `0.02` V | re-queue the readers of hub partitions once the rail has moved this far since the last scan |
| `RAIL_MOVE_EPS` | `1e-12` V | a smaller change of `u` is not a move (P3:468) |
| `RAIL_DRIFT_V` | `1e-5` V (new) | the implied rail motion that schedules a rail event, the frozen CC segment's drift bound, and on a draining cell's coupled path the path's own motion (Rails point 4); a CC rail capacitor's coupled path has no drift watch |
| `COUPLE_PACE` | `8` (new) | an aim of `x` time constants may spend `x / COUPLE_PACE` of the coupling error earlier aims left (Rails point 3, the coupling band) |
| `MERGED_MAX` | `16` (new) | the most distinct rates the merged crossing search takes; a sum of more is searched by windows (Rails point 4) |
| `RAIL_CC_RESOLVE_V` | `1e-6` V (new) | the re-solve band while a PSU is in CC with no rail capacitor (Rails point 3) |
| `RAIL_CC_CAP_RESOLVE_V` | `1e-3` V (new) | the band of a curved hub partition without a curvature band while a PSU ramps a rail capacitor in CC (Rails point 3; none lacks one since ruling R7 (a)) |
| `REACHED_REL` | `1e-12` (new) | the reached-width `η = REACHED_REL·max(1, abs(threshold))`: a rail-event watch aims `2η` past a threshold and counts it reached within `η` of that aim, so at `η` or more past it (Rails point 4); a PSU's capacitor demand re-enters CC only past `iLimit + REACHED_REL·max(1, iLimit)` (Rails point 5) |
| `RAIL_TAU_MAX` | `1e-4` s (new) | the largest battery rail time constant `r·C_rail` the core accepts (Rails point 7) |
| `RAMP_STEP_RATIO` | `0.01` (new) | the longest pulse ramp, relative to the source's event timescale, that is a midpoint step (Sources point 3) |
| `slide_drift(edge)` | `1e-4 + 1e-3·abs(edge)` V (new, ruling R92) | W0's voltage tolerance at a sliding chip's edge: its duty is solved again once its supply drifts this far, and a `Rail` watch sits on both levels (W1c point 1a, the slide) |
| `SLIDE_FOLLOW_MAX` | `100` per host step, or per `SLIDE_FOLLOW_SPAN` (10 ms) of a longer one, whichever allows more (new, ruling R92; per time since R53V) | duties solved again in one host step before a slide is given up as one that cannot be followed: the chip is held as before the ruling for that host step only, and the next host step's retry runs the hold procedure for it again (`hold::reenter_slides`), which slides it again or puts the solver back (held until the circuit changed, cold 2575 tpl 2's 555 sat unpowered in CV for 0.63 s after one source ramp). Cold 230 tpl 2 (a stepped hub partition on a CC rail moving the rail past W0 at each 210 ns step), 559 tpl 0 and `a_rail_ic_at_its_power_edge_on_a_cc_rail_capacitor_is_held_as_on_the_segment` ran under the hold and declined through `SLIDE_SOLVES_MAX` without it; G's boards (175/2, 696/2, 627/4) take at most 42 |
| `SLIDE_SOLVES_MAX` | `1000` per host step (new, ruling R92), scaled with `SLIDE_FOLLOW_MAX` | the backstop past which a slide's re-solves decline `-4` (never reached while `SLIDE_FOLLOW_MAX` gives a slide up first) |
| `SLIDE_FOLLOW_SPAN` | `10e-3` s (new, R53V) | the simulated time per 100 re-solves a long host step may also make (`hold::slide_follow_cap`). The cap was set at the app's cadence, a host step a frame (1/60 s): 100 re-solves in 16.7 ms, one per 167 us. Counted per host step only, it gave a slide up at long host steps that frame steps followed: cold 766 tpl 0 under the trapezoid re-solved its duty every 0.3 to 0.6 ms (before open item (c) closed), about 160 times across a 50 ms host step, and lost its slide at 0.25 s at stops 0.2, 0.25, ... 1.0 s, kept at 1 ms stops. Per 10 ms the allowance at any host step is never below the frame cadence's pace (one per 100 us). A slide given up is held as before ruling R92 and retried at its release level within the host step as any hold (kept from that retry, 766's chip sat off while its rail climbed to 2.3 to 3.6 V) |

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
       summed capacitance of the rail capacitors (point 7);
     - no capacitor runs from the rail node to a node a waveform source
       moves, directly or through a source stacked on it (point 7).
   - The rail node is neither fixed nor free. The union-find skips it as it
     skips fixed nodes, so a device between the rail and a free node puts
     only the free node in a partition, and an IC whose `vcc` is the rail
     joins none of its pins to it (W1c point 2): partitions stay as small as
     on an ideal supply. This is what the hub buys.
   - In exact mode (point 9) the same rules apply, every `-3` above
     included, and then the rail node is made free.

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
   (static, a closed-form segment, a stepped commit), it hands the rail its
   `(I_p, g_p, s_p)` at the solve's rail `u_p`. Stat 16 counts each
   computation.
   - **Curved partitions** (rulings R7 (a) and R8 (a), 2026-10-15). A
     hub partition is **curved** when it holds a stamped junction, a free IC
     supply, or a logic pin of an IC with exactly one supply on the rail:
     that IC's drivers stamp `R_out = vs / (4 io_max)` (a 555 its headroom
     split), which follows the rail even into a linear network. A
     closed-form partition is curved by the last test alone. Every other hub
     partition is **linear**: its solution is affine in `u`, so its
     first-order correction is exact.
   - **The sensitivity of a curved partition** is the exact derivative of
     its solution at `x` itself (F1r6, after V5 finding 2: the last Newton
     iteration's Jacobian and a forward difference left `s` 4.2e-4 off, a
     first-order error): `s = −J⁻¹ dF/du`, `J` the Jacobian of the residual
     stamps linearised about `x` and factored afresh, `dF/du` their rail
     column plus the IC stamps that follow the rail other than through a
     conductance (a driver's conductance moving with its supply, a floating
     pull aimed at the rail; with a free IC supply, the whole exact `dF/dz`
     over every unknown and the rail), and `g = (dI/dx) s + dI/du` from the
     same matrix. No difference is taken, so `s` is exact to rounding. A
     closed-form segment takes it at its `vc = 0` and `vc = 1` solutions, so
     at a given `vc` it publishes `x0 + dx vc + (s0 + s1 vc) (u − u_seg)`,
     exact to first order in the rail at that `vc`. Its capacitor's
     trajectory under a known rail path (point 4: a CC rail capacitor's, a
     draining cell's) is another matter: it takes the rail's pull on the
     capacitor's current at the aim's `vc`, and the bilinear term that leaves
     out is bounded by the coupling band below.
   - **The sensitivity of a linear partition** is P3:437-448's: residuals
     `F0 = r(x, u)` and `F1 = r(x, u + RAIL_DELTA)`, `s = −A⁻¹ (F1 − F0) /
     RAIL_DELTA` through the partition's own factors, `I1 = I(x +
     s·RAIL_DELTA, u + RAIL_DELTA)`, `g = (I1 − I0) / RAIL_DELTA`. Each
     difference `F(z') − F(z)` is formed as `(A' − A) z' + A (z' − z) − (b'
     − b)` with `z' − z` the intended step, so a linear stamp contributes
     `A (z' − z)` exactly: whole residuals left their rounding, about 1e-6 S
     of false slope through a closed 1000 S switch on a 5 V rail.
   - **Power at the solve's rail** (INF1, F1r7). Every sensitivity judges a
     rail IC's power gate at the solve's `u`, whatever the rail unknown
     reads: the residual stamps, the explicit `dF/du` terms, the closed-form
     segments, and the direct IC draws of point 2 (`direct_at`). Power flips
     only at rail events (point 4). A gate judged at `u + RAIL_DELTA` with
     the rail within `RAIL_DELTA` under a floor differenced an unpowered
     driver against a powered one (about −2.4e14 S per output) and sent a CC
     segment to `−inf`. `R_out` and the thresholds still follow the unknown.
   - **Publication.** A hub node's published voltage is `x + s (u − u_p)`
     (P3:486-506); the rail node's is `u`. A closed-form hub partition
     publishes its segment re-aimed at `u` (P3:558-569), as above.
   - **The band** (ruling R2b (b), 2026-10-15; `solve/curvature.rs`).
     Each curved partition sizes its own band at its solve: the widest `b <=
     RAIL_RESOLVE_V` at which a conservative second-order bound, `sup |f''|
     b² / 2` over the band, keeps every quantity it publishes (its node
     voltages, its resistors' and switches' currents, its junctions'
     currents, its draw) inside W0's tolerance (volts 100 µV plus 1e-3
     relative, amps 1 nA plus 1e-3 relative). The suprema are taken over the
     band, never read at the solve, so the bound holds through a junction's
     knee from either side. It measures the correction from the solution,
     so every iterated hub solve is polished on to the solution first (LEDs
     point 9; ruling R20 (a), FXD 2026-10-21: a 16 mA gate into 1000 Ω, an
     LED and 47 Ω published its passing iterate, 6.3e-9 V off, its LED
     current 2e-10 A off, 6.5e5 times its bound at a 3e-9 V rail step), and
     each quantity's bound adds the solve's rounding floor
     (`curvature::Floors`, ruling R23 (i), FXE 2026-10-21): its first-order
     distance from the solution through the exact Jacobian, `|L J⁻¹ r|`
     plus the residual's and the sensitivity's rounding through `|J⁻¹|`,
     plus its own publication's rounding (a GMIN-held cluster sat 1.8e-6 V
     off against a bound of 1.2e-14 V). A
     stepped hub partition's seed (`SEED_H`, at a discontinuity) takes one
     Newton iteration past its passing iterate (ruling R25 (a), FXF): through
     `C / SEED_H` its capacitors magnify a tolerance's offset into amps of
     draw (G2bv's seed 396: a 330 A LED loop through a rail capacitor, its
     draw +-0.45 A off, the rail loop alternating 0.38 V to its `-4`). Its terms:
     - a junction `j`, the rest of the partition seen across it as a
       driving-point resistance `R_j` and an open-circuit sensitivity `a_j`:
       at most `a_j² φ(R_j g_j) / vtn` on any voltage, `φ(y) = y / (1 +
       y)³` (peak `4/27` at the knee, `y = 1/2`), and `a_j² g_j / (vtn (1 +
       R_j g_j)³)` on any current. With several junctions `|a_j|` is bounded
       by the sensitivity network's span and `R_j` lies between its values
       with every other junction off and at its steepest (Rayleigh);
     - the rail-supplied drivers, grouped by node and supply: their own
       second derivative, through the driving-point resistance at their
       node, and their conductance's motion over the band, which moves the
       rest's resistances (Rayleigh) (ruling R4 (a), 2026-10-15). A
       power-window edge is no kink: the bound holds within the power state
       of the solve, and a flip re-solves what the IC stamps into (Rail
       moves, below);
     - a partition holding a free IC supply takes a perturbation bound
       instead (`supply_band`), valid while its element system contracts
       (a Collatz-Wielandt test that the radius is below 1). The iterated
       solve passes Newton's tolerance short of the solution (`R_out`
       restamped at the last iterate's supply converges linearly; up to
       9.6e-4 V, 0.51 of W0, on FXBv's worst board), so the solve is
       polished on to the solution by exact Newton (LEDs point 9;
       ruling R19 (a), FXC 2026-10-21): the bound measures the
       correction from the solution, so an offset at the solve would sit
       outside the band's budget. The bound still measures every span from
       the solve and bounds the solution's distance from it through the
       residual there (`RailSens::res`, closed like the slopes), with the
       first-order term that distance makes, `k1 |u - u_p|` (FXB: before
       the polish a correction 3e-7 V from its solve was 126x a bound
       without it); after the polish both are at rounding;
     - a closed-form segment (ruling R8 (a)) takes the larger of the bounds
       with its capacitor held at either end of the voltages the segment can
       reach from `vc` (a source behind `1/esr`, or a stiff 1e6 S at `esr =
       0`), each quantity checked at its least magnitude over that range; a
       segment rebuilds past its band, and on a reuse from outside that
       range. The range is the hull of `vc`, its asymptote, and the
       asymptote the segment's own model gives a rail `RAIL_RESOLVE_V` either
       side, which is monotone in the rail while the segment decays (F1r11,
       V9 LOW: taken at `vc = 0`'s sensitivity, `vc` ran 0.137 V past it);
       built under a known rail path it is widened by W0's tolerance, the
       most the coupling band lets the published `vc` stray. A segment that
       grows, or could stop decaying inside that reach, has only `vc` and
       the band `RAIL_MOVE_EPS` (fail closed, below); one whose current does
       not depend on `vc` is sized at `vc` alone;
     - the draw by `DrawRule`, judged from the stamps the solve holds (its
       devices, fixed at load, and its IC drivers' committed branches, each
       change of which re-solves it), never from a value: `Zero` when no
       stamp touches the rail (the draw is 0 at every rail), `ShuntOnly` when
       only IC branches to the rail touch a partition in which nothing else
       conducts (the draw is the node shunts' current, and its error
       theirs), else `General`, bounded as a branch current.

     Two exits from the bound. **Affine drivers** (F1r9; closed-form
     segments only). A closed-form hub partition with no junction whose
     driver groups each keep a constant conductance and far-end sum on their
     present piece (a 555's discharge transistor, its OUT pair, whose split
     cancels) is affine in the rail between its drivers' kinks:
     - with no driver kinking inside its IC's power window
       (`IcRail::affine`, checked at the segment's build), the segment keeps
       the linear segment's terms and is a linear partition: it never
       re-solves for the rail;
     - otherwise it is curved, and `segment_band` returns the distance to
       the nearest such kink, just short of it, not capped at
       `RAIL_RESOLVE_V` (infinite if `segment_band` finds none). A segment
       with an infinite band is exact at any rail and never re-solves for it
       in any regime, `RAIL_CC_RESOLVE_V` included
       (`ClosedFormSolver::hubs_past_band`); a finite one takes the regime's
       rule below.

     Without the first form corpus 07 and 08 cost 38 % and 29 % more. A
     static or stepped partition has no such exit: its band is
     `Bound::band`'s, bisected inside `(0, RAIL_RESOLVE_V]` and ending short
     of any driver's kink, so at most 25 mV whatever its drivers. **Fail
     closed** (F1r6): where the bound cannot be formed (a rest that does not
     factor, a reach below `RAIL_MOVE_EPS`) or holds at no band, the band is
     `RAIL_MOVE_EPS`, so the partition re-solves at every move; never the
     flat band. On a rail that moves continuously (a CC rail capacitor's
     ramp, a draining cell's path) that is a `Rail` event every 1e-15 s,
     which no guard declines since each one moves the rail, so the bound
     must not fail where it can close: the sensitivity bound over the band
     (`Bound::reach`) assumes `|x'| < m0` and holds where it yields less,
     and `m0`, twice the larger of the solve's sensitivity and 1, is
     doubled past what it yielded, up to `REACH_TRIES` (6) times, where it
     does not close (FXH, W1gV4's frozen seeds 84 and 20: a 74LS00's
     output pulled to about twice its supply by fixed sources yields
     terminals just past 2 at every band, and the run froze at the gate's
     4.275 V floor on a CC ramp and on a cell's path; pinned by
     `a_gate_output_held_far_above_its_supply_keeps_a_band_on_a_cc_ramp`).
   - **The coupling band** (F1r11, `couple_band`, after V9's MEDIUM 1;
     generalised in rounds 14 and 15). A curved closed-form hub partition
     driven by a known rail path (point 4: a CC rail capacitor's, a draining
     cell's coupled path) holds its sensitivities at its aim's `vc_a`, so
     its current leaves out `di1du δ (vc − vc_a)`, `δ = u − u_a` the rail's
     motion since the aim. The capacitor's error obeys `C e' = (di_a +
     di1du δ) e + di1du δ (vm − vc_a)`, `vm` the published trajectory, and
     is bounded in closed form while the decay `|di_a| − |di1du| |δ|` stays
     positive: the transient's share, the rail's own share `Σ |w| J(f, k,
     τ)`, a pending crossing's landing (the scheduler holds a curved node's
     sensitivity at `vc_a` too), and what earlier aims under the same drive
     left, decaying. `|δ|` is at most the path's own excursion so far: its
     largest magnitude at `τ` or at any turn before, the turns being the
     zeros of its slope `Σ s e^(−kτ)` (`rails::path_turns`; one mode: `|s|
     φ_k(τ)` itself; a path of more than `MERGED_MAX` rates keeps `D(τ) = Σ
     |s| φ_k(τ)`). Each quantity the segment publishes (its nodes, its
     stamped devices' currents, the capacitor's, the draw) is held within
     W0's tolerance at its least magnitude over the voltages and rails it
     can take in each interval of a geometric grid in `τ`, up to the path's
     own first reach of the band's edge (whose rail event aims again) or a
     pending crossing (whose landing does), or for ever where neither comes.
     The first interval that fails sets `couple`, the excursion by then (at
     least the rail's reach in 1 ns and 1000 reached-widths, so the events
     keep moving), and the rail event at `u_a ± couple`, a threshold of the
     path's watch (point 4), aims the partition again. On a path of several
     modes, which may reach that excursion only at a turn where no passage
     fires, the aim also expires at the time it fails (at least 1 ns after
     the aim), and the rail schedules its next event no later than the
     earliest expiry. There an affine segment (an infinite band) takes the
     coupling band too (V11's B256: 2.65x W0 in a gate's transient). An aim
     may spend `x / COUPLE_PACE` (8) of what earlier aims left, `x` its time
     constants, so it never leaves the next aim nothing; where their error
     alone has used a quantity's tolerance up (near its zero), it may spend
     none and ends there, the rail aiming again until the segment's own
     decay frees some (FIN1: 165 such grants in the 768-board sweep before
     the fix). Two holes remain, neither seen to fire in the 768-board
     sweep: where the bound does not close (or the segment does not decay)
     the next aim starts afresh, dropping the inherited error, and the
     coupling band's floor applies even when nothing is left (Known
     follow-ups). Bounding `|δ|` by `D(τ)`,
     as round 14 did, outgrows a path that turns back: the check stopped
     where the rail never went and nothing guarded the aim past it (F1r14:
     a 150 Ω gate's settled current 275x W0; then F1r15's 5 residual CC
     boards). Measured: 818x W0 to 0.66 on one of V9's boards and 6.39x to
     0 on its P1d (F1r11); F1r11's 768-board CC sweep and V11's boards
     none over W0 at round 15 (V11's B162 and B256 0.13 and 0.11). A
     draining cell's fallback drive (point 4) takes no coupling band.
   - **Re-solve.** After every rail solve, each curved hub partition whose
     `|u − u_p|` exceeds its band under the regime's rule is re-solved at
     the new `u`, and the rail solves again:
     - while the hub PSU is in CC with no rail capacitor, `RAIL_CC_RESOLVE_V`
       for every curved partition, whatever its own band, except a
       closed-form segment with an infinite band (affine, above), which is
       exact at any rail: `u` there is set by the draws alone, and a 25 mV
       linearisation error on an LED is a 14% current error (a stepped one
       steps with the rail, so its band holds across its steps: point 4,
       the CC step);
     - in every other regime, battery included, its own band (orchestrator,
       on F1r5: accuracy first, the cost measured small); `RAIL_RESOLVE_V`
       for one without a band, `RAIL_CC_CAP_RESOLVE_V` while a PSU ramps a
       rail capacitor in CC (F1v23 finding 3, ruling R2 (a), kept as
       the fallback by R2b (b)). Since R7 (a) every curved kind sizes a
       band, so neither fallback is reached.

     A linear partition never re-solves for the rail (except at a CC exit,
     point 5). The loop runs inside one `resolve_dirty` call until no
     partition is past its band, at most `RAIL_LOOP_MAX` (50) passes, then
     `-4` ("rail: the rail loop did not settle within 50 passes").
   - **The rail loop's cycle guard** (INV2, F1r9). A pass whose rail lands
     within `RAIL_MOVE_EPS` of where a pass before the last one put it is a
     cycle of cache replays: two entries on either side of a band's edge (a
     driver's `R_out` kink), each one's correction placing the rail just
     inside the other's band, alternate without end (the SAP-1 on its
     battery at 4.8 V, a 74LS245's 24 mA `io_max` kink, 2.897 s). From that
     pass on, a static hub partition past its band takes only a solve at
     that very rail, and the loop settles as a Newton iteration does. A
     closed-form segment past its band already builds at the very rail, and
     a stepped partition solves afresh.
   - **The power hold's trigger** (F5, W1c point 1a). The passes count each
     rail IC's power-verdict flips, and the rail Newton keeps its last three
     iterates. W1c point 1a's procedure runs in place when the passes reach
     `RAIL_LOOP_MAX` with a rail IC flipped at least twice, when a pass that
     moved the rail lands it where it was two passes before with a flip on
     this pass and the one before (an IC whose own flip steps the rail back
     across its edge; F1r14's slow inrush, its supply current through the
     rail capacitors' esr, 17 nV against an edge landed two reached widths
     past, is one), or when the rail Newton fails with a rail IC's verdict
     (`direct_at` judges a direct IC at each iterate) flipped at both of its
     last two steps. Across instants HG1's count runs it (Equivalence
     harness rules, the power-gate ping-pong). With no flip the loop's own
     `-4` stands: a cycle of cache replays at an `R_out` kink flips no gate
     and is the cycle guard's.
   - **Measured.** The 25 mV cap (W1f Q6, F1 round 1): an LED straight
     across a 0.9 Ω battery, default against exact mode, 4e-16 V and 2.5e-14
     A; the hub-partition variant (the LED plus 1 Ω) 2.69e-6 V and 1.43e-6
     A, inside W0's tolerance, so `RAIL_RESOLVE_V` stayed 25 mV. The bound:
     a junction at its knee errs 0.9985 of it (a point estimate there was
     exceeded 1.92x); V6's 176 adversarial cases reached at most 0.99995 of
     it, and V8's 210 boards at most 0.36 of W0's tolerance. The SAP-1 on
     its battery reads within 2.7e-5 V of a re-solve-every-move reference
     (F1r7; 1.34e-3 V before R4 (a)).
   - **Cost bars** (the rulings' own). Ruling R2b (b): corpus 07 and 08
     switched on (load and the CC inrush) cost no more than classic, about
     15 and 8 ms; measured 7.3 and 4.2 ms at round 5, about 8.6 and 5.1 ms
     at round 9, about 8.5 and 4.9 ms at round 11, 26.4-27.1 and 17.8-18.4
     ms at round 14 (60 % of it in the term-by-term crossing search, point
     4), and 6.9 and 4.5 ms at round 15 with the merged-rate search. Rulings
     R4 (a) and R7 (a): the SAP-1 run on its battery costs at most 10 % more
     than at round 5 (`3ca4004`, the baseline each later round measured
     against), else the coordinator is told; measured within about 1 % at
     round 9 (+1.1 % frozen, +0.5 % running), and at round 15, on the cell's
     coupled path (point 4), 1157 and 1345 ms of CPU per simulated second
     against `3ca4004`'s 2092 and 2331 (45 % less). Ruling R17's condition:
     the SAP-1 on its ideal supply within 10 % of `3ca4004` too; measured at
     steady state (5 s of warm-up, 9 interleaved runs) at 9.86 against 9.93
     ms per simulated second frozen (−0.7 %) and 12.81 against 12.74 running
     (+0.5 %). A cost gate must warm up a few seconds first: V8 compiling the
     wasm (848 KB against 814 KB) lands inside a 0.1 s warm-up window, which
     read the same build 10 % slower.
   - **Rail moves.** A solve that changes `u` by more than `RAIL_MOVE_EPS`
     is a move (stat 14). On a move (P3:469-483):
     - every closed-form hub partition, and every closed-form partition
       holding a watched pin of an IC whose `vcc` or `gnd` is the rail or a
       static hub partition's node (F1v23 finding 4: such a threshold moves
       on that node's published line), is re-aimed at the current `u` from
       its current `vc` (P3:570-574), which reschedules its crossings; under
       a CC rail capacitor the aim is the segment's value now and the
       segment drives it, and on a draining cell it follows the cell's path
       or its drive (point 4). Every stepped hub partition re-publishes;
     - an IC on the rail whose power gate the move flipped is queued, and
       the partitions its drivers and pulls stamp into re-solve (they stamp
       only while powered);
     - rule 2a capacitors and LED junctions across the rail and a fixed node
       re-aim (point 7);
     - when `|u − u_scan| > RAIL_SCAN_V`: `u_scan = u` (stat 17); every
       reader of a node of a static hub partition, and every IC whose `vcc`
       or `gnd` is the rail (a 555 included), is queued for this delta. Each
       queued reader whose logic-pin reads differ from its previous powered
       evaluation's counts once in stat 20: that flip may be up to
       `RAIL_SCAN_V` of rail late (Equivalence harness rules, the reader
       scan).
   - **Static cache.** A static hub partition's key includes the band
     `round(u / RAIL_BUCKET_V)` and the power gate of each IC on the rail
     that touches it (a power window can sit inside one band, and an
     unpowered IC stamps nothing). An entry stores `x`, `s`, `I_p`, `g_p`,
     `u_p` and the band (P3:507-531), and the same-key entries are searched
     for one whose `|u − u_p|` is within its band under the regime's rule,
     the rule the re-solve applies, so a hit never answers where the loop
     would re-solve (V4X: CC with no rail capacitor took a 25 mV hit under a
     1 µV band, cache on and off 1.09e-5 V apart). At a CC exit (point 5)
     and once the cycle guard fires, only a solve at that very rail answers
     (band 0). A partition holding a free IC supply is never cached. A hit
     replays the contribution and publishes `x + s (u − u_p)`, so cache on
     and cache off agree within the rail-correction bound on hub fixtures,
     as row B1's verify expects (F4: at most 8.2e-10 V apart on the W1f
     fixtures).

4. **Rail events** (W1f Q1; pec3 has none). Between events the static draws
   are constant, but a closed-form hub partition's draw follows its
   capacitor, and SoC moves `E` and `r`. So after every rail solve at `t0`,
   the rail holds its **implied trajectory** `u*(t)`: the equation of point
   2 with each closed-form hub contribution at `I_p(t) = a_p + b_p vc_p(t) +
   g_p (u − u_p)` (its segment's `vc_p(t)`, P3:575-579) and the battery's
   `E(t)`, `r(t)` from the held discharge (point 6). One `Rail` event is
   pending at a time, versioned; every rail solve supersedes it. It is due
   at the earliest `t > t0 + 1e-15` at which any of these is reached:
   - **drift:** `|u*(t) − u(t0)| >= RAIL_DRIFT_V` (on a draining cell's
     coupled path, the path's own motion; a CC rail capacitor's coupled
     path has no drift threshold, and its frozen form a bound of its own:
     both below);
   - **power window:** `u*(t)` crosses `V(gnd) + vccMin*0.9` or `V(gnd) +
     vccMax*1.1` of an IC (555 included) whose `vcc` is the rail and whose
     `gnd` is fixed, computed as W1c point 1 computes them;
   - **CV entry** (PSU in CV): the implied draw `I*(t) = A0(t) + G V`
     reaches `|I*| > iLimit`;
   - **CC exit** (PSU in CC): `dir·(A0(t) + G V) <= iLimit`, the draw at
     the setpoint back within the limit; with a rail capacitor, `dir·u*(t)
     >= |V|` (point 5);
   - **SoC:** `soc` reaches the next OCV-table knot below it, or 0;
   - **power hold release** (F5, W1c point 1a): an IC held unpowered whose
     `vcc` is the rail and `gnd` fixed watches, in place of its power
     window, `V(gnd)` plus its release level: the window edge it starved
     at, moved inward by the drop its own draw caused (its supply held
     unpowered less held powered, from the probes) and two reached widths
     of that edge. A chip held while a PSU with a rail capacitor is in CC
     (on a segment, where the capacitors hold the rail through both probes,
     so the drop is 0 bar the esr) also waits for a **draw threshold**
     (ruling R44 (a)): the draw at its edge `V(gnd) + level`, `dir` signed,
     plus what its being powered adds (the probes' difference) reaching
     `iLimit`, located like the CC exit's draw test. Either event is a
     trigger, not the release itself (ruling R51 (b), W1c point 1a): the
     settled rail loop it brings retries the held rail ICs past their level
     (a pass in passing does not), and on a segment only one past its draw
     threshold; the one that carries itself reads at once, the passes go on
     with its stamps, and if it starves again it is held again with a fresh
     drop. A retry that still starves past the level moves the level (a
     fresh drop, twice the margin). Besides, once per host step every held
     rail IC is retried (W1c point 1a).

   **Locating the time.** Each threshold is a level of one scalar function
   of `t`: `u*(t)` in volts, or the draw `I*(t)` in amps (CV entry, the
   exit's current test). For each function, each moving term (a closed-form
   contribution, the SoC) gets an equal share of the smallest distance from
   its value at `t0` to that function's nearest threshold, capped at
   `RAIL_DRIFT_V` for `u*`. A term's time to use its share is closed form:
   a log for an exponential segment (P3:580-608's form), linear for SoC
   within a table segment. The event is the earliest of these times. With
   one moving term the event lands on the threshold exactly; with several,
   successive events close on it geometrically. Each threshold is aimed at
   two reached-widths past it, on the far side, so a landing is past it
   however the last bit rounds (a gate tested with `>=` or `>` alike), and
   within one reached-width of that aim, `REACHED_REL * max(1, |threshold|)`,
   it counts as reached. At a `Rail` event (stat 18):
   SoC advances to `t`, every closed-form hub contribution is refreshed at
   its `vc(t)`, the rail solves (point 2), the regime rules run (point 5),
   and a new event is scheduled. A stepped hub partition contributes afresh
   at each committed step, so the rail solves at its step events; a
   threshold it crosses within a step is found at that step's end, as
   classic finds it (a step granularity, not an exact time). The CC exit of
   a PSU with no rail capacitor is the exception (point 3, the CC step): a
   step whose rail passes the setpoint is cut where it reaches it.

   **A seed's capacitor currents** (W2c, the ruling on option (A)). A
   stepped seed (`SEED_H`: a load, a re-aim at a discontinuity, a CC
   promotion) publishes each capacitor that holds a charge at the current
   of its branch formulation: the capacitor an ideal source of its `vc` in
   series with its esr, its current an unknown of one linear solve with the
   rest of the partition linearised at the seed's solution
   (`stepped::seed_branch_currents`). Read through the companion instead,
   it is `C / SEED_H` times the node's offset from the charge, and a node an
   ulp off a settled charge read up to `C / 1 ps x ulp(v) / 2`: 1.9 nA on
   corpus 35's 10 uF at a reload (the true current 1e-14 A), 0.44 uA for 1 mF
   at 5 V. The seed's node voltages and the charges are unchanged; a
   capacitor at `vc = 0` (every cold load) keeps the companion's current,
   which has no cancellation, bit for bit. **Known divergence**: classic's
   own seed is the 1 ps companion, so at a load onto charged capacitors (its
   carrying reload) and at a re-aim its published capacitor currents carry
   up to `C / 1 ps x ulp / 2` of that error; the core's are the true current
   to the currents' own rounding, closer than classic's by that bound.

   **The CC seed** (F5c, ruling R28 (a)). A stepped hub partition's seed
   (`SEED_H`, BE at 1 ps) stamps each capacitor as `C / SEED_H` siemens (up
   to 2e10 S), and a draw read through that conductance carries the solve's
   rounding magnified the same way: `eps C / (SEED_H g)` on the rail, 2e-6 V
   for G2bv's 213 uF behind 0.03 S, past the 1 µV CC band, so the rail loop
   re-seeded to the same rails without settling (11 of G2bv's 9,000 runs and
   2 of the cold fuzz's sample, all `-4`). So a seed whose partition holds a
   capacitor with a pin on the rail takes its draw from a charge balance, in
   every regime: every branch at the rail but its capacitors, plus each rail
   capacitor's current read from KCL at its far node (over the whole cluster
   the rail reaches through capacitors when the far node holds more); summed,
   the rail row and the cluster's node rows are the same equation with the
   capacitors' terms cancelled exactly (`stepped::ChargeBalance`). A cluster
   with a capacitor to a fixed node has no KCL there and keeps the
   linearised draw. On a CC rail with no rail capacitor the seed also solves
   the rail as an unknown with its partition, as the CC step below does,
   with that balance as its rail row: held at the rail the loop last solved,
   a seed whose stamps curve in the rail (cold fuzz 456 tpl 0: a gate's
   output on it, 13 Ω and an LED off it) re-solved to the same three rails
   in turn. Such a seed hands the loop the rail it solved and the draw its
   rail row held there (`dir iLimit` less the rest's draw), so the loop
   finds that rail again; its slope judges a rail IC's power at the loop's
   rail, as the solve did (F5h). Published at its own rail instead, a seed
   whose gate was powered in the solve and off there (G2bv's bc_ic 178 hub
   2: 1.69 V powered, 4.94 V not), or whose draw was read through 1e9 S of
   `C / SEED_H` on a path from the rail to a fixed node (cold fuzz 603 tpl
   3, 5.2e-7 A), left the loop's rail where it was, past the 1 µV band, and
   the seed re-solved to the same rail for 50 passes. Pins in
   `solve/rail_tests.rs`, G2bv's bc 453 hub 2 (an advance
   `-4` at 0.95 ms) and bc_ic 345 hub 2 (a load `-4`), each now run to 3 s:
   `a_g2bv_capacitor_on_a_cc_rail_seeds_at_the_rail_it_solves_with` and
   `a_g2bv_capacitor_on_a_cc_rail_loads_at_the_rail_it_solves_with`; F5h's
   `a_cc_hub_seed_publishes_its_draw_at_the_power_its_solve_held` (178),
   `a_cc_hub_seed_near_a_power_edge_lets_the_rail_move` (bc_ic 6 hub 2) and
   `a_cc_hub_seed_with_a_capacitor_path_off_the_rail_publishes_the_draw_its_rail_row_holds`
   (603). On a CC rail with capacitors (ruling R68) the seed's rail row
   also holds their current at the seed, `(u - q) / esr_eff` from the
   charge `q` they hold, and its draw is the row's less that current; at
   `esr_eff = 0` the rail is the charge itself and the draw is read as off
   a CC rail (the CC step with a rail capacitor, below).

   **The CC step** (FXF, after G2bv's frozen tab). While a bench PSU with
   no rail capacitor is in CC, the rail is set by the draws alone, so it
   moves with a stepped hub partition's own state (an LED charging a
   capacitor at the limit carries the rail up with the capacitor). Held as
   a boundary over each step, it went stale within the step, and the 1 µV
   CC band (point 3) re-solved the partition each time the rail rose 1 µV,
   each re-solve a discontinuity restarting the step at `POST_EVENT_H`
   (G2bv's seed 177: `{9.362 V, 0.3 A}`, an LED charging 29 mF, steps of
   4e-8 s, 0.1 s simulated in 27 s). So such a partition steps with the
   rail as an unknown (`stepped::CcRail`): the residual stamp's rail row
   (its draw, as the sensitivity reads it) plus the rest of the rail's
   draw, every other contribution and the direct devices linearised at the
   last rail solve, equals `dir·iLimit`, solved by Newton and then on to
   the solution to rounding (at most 4 more Newton iterations): the rail
   loop solves the rail from the step's draw and compares it with the
   step's own against the band and the setpoint, and an iterate one polish
   short still left them 1.3e-8 V apart. A partition holding a pin of a
   rail-supplied IC iterates its CC step (and its CC seed) even where it
   would solve a step in one linear pass off a CC rail: the IC's `R_out`
   (`vS / 4 ioMax`) and targets move with the rail, now an unknown, and
   one pass about a seed's 5.29 V start landed G2bv's bc_ic 425 hub 2
   rail 8.5 mV over the one its gate's stamps hold there (F5l: the power
   hold's retry freed the gate 0.22 s before classic, and every step after
   it drew the rail under the floor, flipped the gate twice and restarted,
   10 ns steps; pin `a_cc_step_solves_the_rail_its_gates_stamps_move_with`
   in `solve/rail_tests.rs`, classic's values within W0). A rail IC's
   power gate is judged at the step's starting rail. A step whose CC rail ends past the setpoint
   is cut, by regula falsi on the rail's distance past it from the shortest
   step (`H_MIN`), to land within `CROSS_TOLERANCE_V` past it, so the
   regime rules leave CC at that commit (re-solving the whole step in CV
   instead put its end on a trajectory CV never took, whose draw at `V` was
   over the limit again: the exit landed a step late, 16 mV of capacitor on
   FXF's round-3 board). A CC rail already past the setpoint at `H_MIN`
   (the draws fell below the limit since the last rail solve, as when a
   load is switched out) is an exit at the step's start, and the step
   solves in CV with the rail at `V`; cutting toward the last rail solve's
   rail, which is no CC solution there, made steps of 1e-12 s and hung the
   host. The rail loop still solves the rail and checks every band after
   each commit. A stepped partition re-solved only because the rail moved
   past its band keeps its step (`SteppedSolver::reaim`): with two stepped
   partitions on one CC rail each holds the other's draw where it last
   committed, so each commit of one re-solves the other, and restarting
   it at `POST_EVENT_H` stepped FXEFv's two-branch board at `H_MIN` (2.77e6
   steps, 18.8 s per 0.1 s; now 514 steps). That hold is a first-order
   coupling error between the branches, which the step doubling cannot see
   (both halves hold the same draws): kept, the steps grew until the rail
   moved by far more than W0 across one, 39x and 43x W0 off a fine
   reference on FXEFv's two- and three-branch boards (W1gV3). So while
   another stepped hub partition shares the CC rail, a step is also limited
   to a rail move of W0 across it (ruling R39 (1), `stepped::
   coupling_ratio`): the move is twice the second half step's (the step's
   own rail at its start can sit a re-solve's offset from the published
   one), and since the error is first order in `h` it enters the
   controller as its square (backward Euler). Measured on FXEFv's `cc`
   sweeps (power LEDs, 40 boards each): the worst node off the fine
   reference went from 30.6/39.0/43.1x to 30.6/6.5/9.0x W0 (tpl 9/10/12),
   at up to 4.4k steps and 0.07 s per 0.1 s simulated; what remains is the
   backward-Euler controller's own error (Known follow-ups (W1g), R32).
   Measured on seed 177: 4 s simulated in about 4 ms (729 steps against
   5.3e6 for 0.2 s), node voltages within 0.09 of W0 of exact mode and
   0.19 of a fine reference (exact mode advanced in 1 µs frames, 0.14);
   the LED's current after the exit 9.9x and 21.8x W0 off them, as the
   base build is (21.9x) and exact mode against the reference (16.2x): a
   1 mV step-controller error on the capacitor through the LED's
   exponential.

   **The CC step with a rail capacitor** (ruling R68, superseding R47 (a)
   for this case; F5c's R42 prototype, ported). With rail capacitors the
   CC rail was the capacitors' segment, a boundary over each step, and a
   stepped hub partition's draw was held on it linearised (the frozen
   form): a partition coupled to the rail through microfarads of small esr
   took the rail's microsecond motion as a draw it could not follow. The
   cold fuzz's 347 tpl 5 (a {3.08 V, 332 mA} PSU, 1.75 uF of rail capacitor
   behind 4.74 Ω, 562 uF on a switch contact to the rail and 12.6 mF from
   a second node to it, 0.67 uF of 19 mΩ to a third) ran 2824 W0 off its
   fine reference in the start-up inrush and 239 W0 after a switch's CC
   entry, where exact mode at the same host stops is within 3.6. So while
   no closed-form hub partition is active on the rail (their coupled path
   is the segment then, which a step holding them frozen would not keep),
   the CC step also carries the capacitors (`stepped::RailCap`): from
   their C-weighted charge `q` and current `i` at the step's start, `u =
   q' + esr_eff i'` with `C_rail q' = C_rail q + h i'` (backward Euler) or
   `+ h (i + i') / 2` (the trapezoid), a companion of `1 / (esr_eff + h /
   C_rail)` (or `h / 2 C_rail`) on the rail row; the step doubling compares
   the two charges as it compares a capacitor's, and the second half starts
   from the charge the first left. A step starts from its partition's own
   last committed charge when it starts there, else from the segment's
   charge at its start. Each commit keeps its charge, and the latest hands
   it to the rail solve after it in place of the segment's, once
   (`SteppedSolver::take_rail_charge`); the step's other terms are summed
   afresh from the contributions, not read from the running sums (F5c: a
   step cut inside an inrush handed the rail 5e15 S, whose removal left
   0.4 A of rounding in them). At a CC entry a hub partition already
   standing at the instant plans its pending trial again on the CC rail
   (`SteppedSolver::replan`): solved against the rail before the entry as a
   boundary, and past no band when linear, it went on to commit that trial
   in CC (347 tpl 5's load stepped 10 ns at the setpoint, 30 uC into 562
   uF through a 332 mA limit). It keeps its state and sensitivity: a
   re-seed changes the draw's rounding, which solves the rail again, and a
   rail capacitor of no esr entered at the setpoint, the rail its charge,
   leaves CC at once there (CC and CV in turn for the loop's 50 passes on
   `a_cc_exit_ties_a_capacitor_with_no_esr_at_the_setpoint`). Two
   rules keep a rail IC's power on such a step's rail. A re-solve's
   catch-up steps on the CC rail its pending trial was solved on, as a CC
   exit's does: judged at the rail published after the change, cold 216
   tpl 5's 74LS00, powered over the step, was judged off at the falling
   edge that ended it, the step carried the rail back over the edge, and
   the chip flipped at every instant 1e-14 s apart, never twice within one,
   so the power hold never triggered (3.2M steps for 7 ms;
   `a_rail_ic_at_its_power_edge_on_a_cc_rail_capacitor_is_held_as_on_the_segment`).
   And a step whose rail reaches a rail IC's power edge (its window's
   edges, a held one's release level) is cut there, by regula falsi from
   the shortest step as at the setpoint: one 2.6 ms step of cold 505 tpl 5
   carried the rail 0.13 V past a 74LS00's floor, the powered chip pulled
   it back at 3e5 V/s, and the rail Newton failed at the floor (`-4`;
   `a_cc_step_on_a_rail_capacitor_is_cut_at_a_rail_ics_power_edge`). A
   step starts from the latest charge at its start: a hand-off not yet
   taken, else the segment's if aimed there, else the partition's own; and
   a trapezoid from the partition's own last current, never the segment's
   slope. Cold 581 tpl 5 re-seeded on a segment its rail solve had aimed
   before the catch-up's commit, 6 uV off the partition's capacitors, which
   a capacitor loop through the rail turned into 4.5 A, reflected by the
   trapezoid at every step: the rail sat up to 2.35 mV below the node a 1
   mohm switch holds it at
   (`a_rail_capacitor_loop_through_a_stepped_partition_keeps_the_rail_on_its_switch`).
   The fine reference's own limit shows on cold 84 tpl 1: an LED from the
   rail to a clock's node drains 2.65 uF at a clock fall, an integral of
   14.71 against its 24 by hand (a 37 ns exponential through its 71.5 S
   line past the knee), so it survives the first fall and burns in the
   second; exact mode latched it at the first, its stepped rail partition
   taking the end rate over the 2.07 ms step that ended at the edge
   (`an_led_draining_a_cc_rail_capacitor_carries_the_stress_its_own_drain_gives`). One stepped
   partition on such a rail now steps with it as exact mode does (within
   0.4 W0 of the fine reference where the frozen form was 650 W0 off,
   `a_stepped_hub_partition_steps_a_cc_rail_capacitor_with_it`); with two,
   each holds the other's draw where it last committed, the coupling error
   of two stepped partitions on one CC rail above (rulings R32, R37, R39),
   up to 21 W0 on R68's two-partition board against 3632 before
   (`two_stepped_hub_partitions_on_a_cc_rail_capacitor_keep_near_the_fine_reference`).

   **Superseded events** (F1r11, after V9's MEDIUM 3). A host advancing
   every few nanoseconds through a CC ramp solves the rail at each horizon
   clamp, and each solve supersedes the pending `Rail` event with a new
   version due at the same threshold: about 60,000 stale versions popped
   within 1 ps of each other at 4.32 V, each run as an instant, and the
   no-progress guard (Equivalence harness rules) declined with `-4`. A
   superseded event at the head of the heap is now dropped between
   instants, as its handler would drop it inside one (W1a events point 5):
   it is no instant, so it neither runs nor counts toward the guard. Stat 4
   still counts it. V11 found it cannot lose a live event: the state is the
   last completed instant's, as a rollback leaves it, so an event stale now
   stays stale after any rollback. Pinned by
   `a_host_stepping_nanoseconds_through_a_cc_ramp_is_not_a_stall`.

   So the rail is exact at events and never further than `RAIL_DRIFT_V` from
   its implied value between them (a CC rail capacitor's coupled path, below,
   is that value), the power gate flips at the exact instant
   the rail crosses it (a digital fact, delta D1), and CC entry and exit are
   exact state transitions (Q-O36). Cost on the SAP-1 battery variant,
   measured (F1r8, before the cell's coupled path below): about 132k `Rail`
   events per simulated second, mostly
   drift events from the 555s' timing and CV networks (closed-form hub RCs
   through 0.9 Ω), each re-aiming the closed forms and solving a scalar,
   about 16.8 µs of CPU apiece (W1f known follow-ups, perf round 3).

   **The CC rail with a rail capacitor** (W1f Q2). In CC a rail capacitance
   `C_rail` is the whole dynamics: a PSU limiting into a capacitor ramps it
   at `iLimit/C` (probe P9). So while a PSU with `C_rail > 0` is in CC, the
   rail is a closed form of its own (`rails::CapSeg`). With closed-form hub
   partitions on the rail it is their coupled path (below, ruling R15 (a));
   the single exponential here is the form of a rail with none, and the
   fallback:
   - **The form.** Its state is the capacitors' C-weighted charge `q`,
     `C_rail dq/dt = I_cap`, with `I_cap = dir·iLimit − (A0 + G u)` their
     current, and the rail sits a static `esr_eff·I_cap` above it (manager
     ruling 2026-10-07, F1 finding C; classic shows the offset), `esr_eff =
     Σ esr_k·C_k² / C_rail²` (a lone capacitor's own `esr`). Both are linear
     in `u`, so `u = α q + β` with `α = 1 / (1 + G esr_eff)` and `β = α
     esr_eff (dir·iLimit − A0)`, and `u` is the same exponential as `q`:
     `u(t) = u∞ + (u(t0) − u∞) e^(−k (t − t0))` with `u∞ = (dir·iLimit − A0)
     / G` and rate `k = α G / C_rail`, not `G / C_rail` (F1r10: the core's
     form is the exact coupled solution with esr and a resistive load, to
     5.8e-12 V and 4e-9 relative in the capacitor's current, inside the
     `esr·C` window too). Ruling R10 (b) (2026-10-16) asked for that exact
     solution after F4's fixture 45 read the rail up to 8.2e-5 V above it;
     it is recorded as a correction, not a change: the core was already
     exact, and F4's sidecar treated the switch as ideal where both engines
     carry a 1 mΩ contact and the node shunt. Fixed, the sidecar agrees, and
     fixture 51 samples inside the 50 µs `esr·C` window after a CC exit
     (Equivalence harness rules). It is evaluated as its increment from
     `u(t0)`, `u(t0) + slope·∫0^(t−t0) e^(−k s) ds` with `slope = α
     (dir·iLimit − A0 − G u(t0)) / C_rail` (the textbook form rounds at the
     scale of `u∞`, which on a lightly loaded rail is `iLimit / G`,
     megavolts), and is the straight line `u(t0) + slope (t − t0)` while `k
     (t − t0) < RAIL_CAP_LINEAR` (1e-9). At `esr_eff = 0`, `α = 1` and `β =
     0` exactly.
   - **At a rail solve** the charge is continuous, and the rail solves `u =
     q + esr_eff (dir·iLimit − draw(u))` with the direct devices exact (the
     scalar Newton of point 2), so it steps with the draws through the
     offset; the segment re-aims from there. The charge starts at `V` at a
     CC entry from a load, so the rail starts `esr_eff·I_cap` from `V`
     (short of it in the PSU's direction `dir`, the draw being past the
     limit), and from the rail the capacitors hold when their own charge was
     the demand that entered CC (point 5: their charge shared at the rail,
     `V − dir·Σ(C_k/C_rail)·d_k` over the capacitors short of `V`). The
     `esr·C` transient that sets the offset (nanoseconds) is not modelled.
     The horizon clamp brings the segment to `t_end` with a rail solve.
   - It is published live, and each rail capacitor carries its share
     `C_k·dq/dt`.
   - **Thresholds.** Its own motion replaces the drift threshold with these:
     the power-window edges above; the rail voltages at which an IC's
     generic thresholds kink (at a supply difference of 2.667 V for `max(0.8,
     0.3 vs)`, 3.333 V for `max(2.0, 0.6 vs)`, 0.571 and 0.667 V for the
     CMOS ratios' clamps), for each IC whose supply difference moves with
     the rail along a line (F1v23 finding 4); the
     setpoint (the exit, point 5); `u_scan ± RAIL_SCAN_V`; `u_p ± band` of
     every curved hub partition, its band under this regime's rule (point 3;
     `RAIL_CC_RESOLVE_V` does not apply here: the capacitor, not the draws,
     sets `u`); and `u_a ± couple` of every driven curved closed-form
     partition (the coupling band, point 3), whose expiry, on a path of
     several modes, is due as well. On the single exponential each is a log
     (or a line) on the segment alone: sharing the distance with the
     closed-form draws made every event close on a threshold geometrically,
     and a fast inrush ran out of time resolution short of it. On the coupled
     path each is a first passage of the path, which may turn: the nearest
     threshold above and the nearest below, whichever the path reaches first
     (`RailModes::passage`). The path holds nothing frozen, so it has no
     drift threshold.
   - **The coupled path** (ruling R15 (a), F1r14). While a PSU ramps a rail
     capacitor in CC with closed-form hub partitions on the rail, the rail
     and those partitions advance together between solves, on the exact
     solution of the linear system they form at the solve, each partition
     linearised at the aim it takes there (`rails::coupled_modes`):
     `C_rail q' = dir·iLimit − A0 − G u − Σ b (vc − vc_a)`, `u = α (q +
     esr_eff (dir·iLimit − A0 − Σ b vc))`, and each `vc' = f (vc − vc_a) +
     γ (u − u_a) + r0`, `γ = (di0/du + di1du·vc) / C`. Eliminating the
     partitions, the rail is `u(t0 + τ) = u0 + Σ s φ_k(τ)` over the
     system's modes `−k`, with `φ_k(τ) = ∫0^τ e^(−k r) dr`: the zeros of the
     secular function `F(λ) = Σ w / (λ − f) − m(λ)`, `w = −b γ >= 0`, `m =
     (λ + k_q) / (α Q(λ))`, `Q = (α + esr_eff C_rail (λ + k_q)) / C_rail`,
     `k_q = α G / C_rail`. `W` falls between its poles and `m` rises (with a
     pole of its own at esr > 0), so one root lies between each two
     consecutive poles, one between the last and 0 while the rail is stable
     (`F(0) < 0`), and at esr 0 one below the first: as many as the system
     has states, each found from its nearer pole, so a weakly coupled
     partition's mode keeps its distance from its own rate to the last bit.
     Each mode's share is the residue `s = (B − r0_q / Q) / F'`, `B = Σ b r0
     / (λ − f)`, `r0_q = q'(t0)`; a partition that moves its draw but
     follows no rail (`w = 0`) is a forcing at its own rate. The shares sum
     to the rail's slope, `α (r0_q − esr_eff Σ b r0)`, which is checked to
     1e-9 relative. The capacitors' charge follows through the esr pole,
     and each rail capacitor carries its share `C_k dq/dt`. A rail of no
     closed-form hub partition keeps the single exponential above.
   - **Following the path.** Every closed-form hub partition, and every
     closed-form partition holding a watched pin of an IC on the rail or on
     a static hub node, is aimed at the path's value now and takes it as a
     known drive. A hub partition follows it by its own closed form, `vc =
     inf + (vc_a − inf) e^(fτ) + Σ w J(f, k, τ)`, `w = γ s` per mode and
     `J(f, k, τ) = ∫0^τ e^(f (τ − s)) φ_k(s) ds`, its nodes publishing
     `x0(u_a) + (dx0/du)(u − u_a) + dx vc`. That is the coupled solution's
     partition part, so a linear partition is exact to rounding and the
     rail a solve finds between events is the path's own: no drift charge,
     no owed step. What is left is second order in the rail's motion since
     an aim: a curved partition's bilinear term, `di1du (u − u_a)(vc −
     vc_a)`, which the coupling band bounds (point 3: its sensitivities are
     held at the aim's `vc_a`, its nodes publish with `dx/du` at the present
     `vc`, and the crossing scheduler holds `dx/du` at `vc_a`), and the
     linearisation of the direct devices and static partitions at the
     solve. A watched-pin partition (`w = 0`) sees the pin's thresholds move
     with `u`. Every threshold the compiler emits is affine in the supply
     between the power-window edges and the generic kinks above, so each
     edge and kink is a rail event that aims again, and a supply on a
     static hub node moves with the rail on that node's published line.
     Measured (F1r14, F1r15): F1r12's 108 RC-on-rail boards and F1r11's 768
     CC sweep none over W0 (27 and 83 over before round 12, worst 99x and
     160x), and V11's boards (R12's hang B1070 and its 5 false `-4`s
     included) all run, none over W0, with about 6x fewer rail events on
     CC than under rounds 12 and 13's drift caps (below).
   - **Crossings on the path.** Under one mode a crossing is the first root
     of node less threshold, which has at most one extremum
     (`rails::driven_crossing`). Under several it is the first root of a sum
     of monotone terms that moves with the rail (each mode's line, the
     capacitor's own exponential and its response to each mode,
     `rails::ExpSum`), and the search takes the sum by rate (F1r15):
     `e^(fτ) = 1 + f φ_(−f)`, `J(f, k) = (φ_(−f) − φ_k) / (f + k)`, so `g =
     c0 + Σ d φ_λ` over distinct rates, and `g' = Σ d e^(−λτ)` is an
     exponential sum whose zeros are at most its coefficients' sign changes
     in order of rate (Descartes' rule), each found between the zeros of the
     next derivative (Rolle, `exp_zeros`). The sum is monotone between its
     turns: each piece is one evaluation at its end, and the piece holding
     the crossing is solved by Newton kept in its bracket, to the last bit.
     A capacitor within `1e-6` relative of resonance with a mode (a weakly
     coupled partition beside its own mode) takes the confluent form `ψ_λ =
     ∫0^τ s e^(−λs) ds` at their mean rate, as do two rates within `1e-6` of
     each other, the error it leaves added to the search's rounding margin.
     A sum of more than `MERGED_MAX` (16) rates is searched by windows: each
     bounded term by term and skipped where its range excludes a root, as
     round 14 searched every sum. A node and a threshold riding the same rail
     cancel only in the sum, so those windows bisected to the narrowest, and
     the search was 60 % of corpus 07's switch-on, 3x its bar (point 3, cost
     bars). **The landing rule** (F1r14): a sum already at or past 0 at the
     aim (a landed node can sit its last bits on the wrong side) must first
     come back past 0 by more than 64 ulps of its scale before a crossing
     counts, so the root it has just come through is never landed again.
   - **The frozen form** (the fallback). Where the system is not one the
     modal form takes (a partition with `w < 0`, a capacitor that does not
     decay, a mode unstable, degenerate or failing the slope check), the
     segment is the single exponential above and holds the closed-form
     draws where they stood at its aim, as before ruling R15 (F1 round 3,
     option (i), chosen over re-aiming every closed form each
     `RAIL_RESOLVE_V`; both were measured inside the criterion):
     - **Drift-charge payback.** The segment holds the closed-form draws'
       `b vc` where they stood at its aim (`cf_frozen`) while their
       capacitors move on. The charge their motion takes since is known in
       closed form, and the next rail solve takes it off the capacitors'
       charge (`q = charge(t) − drift`). A `DriftBound` watch solves the rail
       again before that could move it `RAIL_DRIFT_V`: after `t` the draws
       have moved at most `min(c_sat, c1 t) + c2 t²` amps, and an amp moves
       the rail at most `α (esr_eff + t / C_rail)` (F1r4: 4.8e-6 V with the
       watch, 2.2e-4 V without). What that leaves a partition's current is
       the gap rulings R12 and R13 targeted (below).
     - **Solve before a crossing.** The rail a driven partition is aimed
       with trails the one a solve gives by the owed step, `α·drift + α
       esr_eff Σ b Δvc` (the drift charge and the esr offset's share of the
       draws' own motion, F1v23 finding 2). Landing first would put the node
       `CROSS_NUDGE_V` (1e-9 V, the closed-form crossing's landing nudge)
       past a threshold riding the stale rail, and the solve the crossing
       causes would step the threshold back across it: the same crossing
       again a moment later. So when the owed step is at least
       `CROSS_NUDGE_V`, the rail solves first, re-aims what it drives and
       reschedules the crossing; since every threshold and hub node moves
       with the supply by a factor in `[0, 1]`, a crossing the solve does
       not put back short of its threshold lands at once. A coupled path
       owes no step and never takes this solve.
   - **Rulings R12 and R13, backed out** (ruling R15 (a)). Round 12 bounded
     the frozen form's drift per curved partition (`drift_width`, `drift_cap`:
     the rail solved again before its drift could put a published current
     past W0), and round 13 extended it to linear partitions; on F1r11's 768
     sweep and F1r12's 108 boards both cleared W0, at 4x and 11x the rail
     events. V11 found it unsound (a hang, B1070, a CC exit whose rail sat
     still while `Rail` events came 1.14 ps apart; 5 false `-4`s; 21 boards
     still over W0, 8 worse than before, the drift's premise false), and
     ruling R15 (a) replaced both by the coupled path. Their tests stay,
     each now asserting W0 on the coupled path.
   - **A flip against the ramp** (F1r14; replaced by W1c point 1a, ruling
     D4 (a)). A rail IC whose own power flip steps the rail back across its
     window's edge has no consistent state at that instant. F1r14 held the
     flip that ran against the CC segment's motion; since F5 the passes'
     alternation triggers the power hold (point 3), which holds the IC off
     until its supply carries it. F1r14's pins pass unchanged.
   - **Not modelled** (ruling R11 (a), 2026-10-16). Rail capacitors of
     different `esr` exchange charge in a fast mode of their own after a
     step, a transient lasting about 5 µs for the measured pair; the
     C-weighted form leaves it out. Measured: one capacitor's current 11.5 %
     off 0.5 µs after the step, the rail within 8.1e-6 V. It is a W3 analog
     backlog item (W1f known follow-ups).

   **The cell's coupled path** (ruling R16 (B), F1r15; `rails::cell_modes`,
   `CELL_COUPLED` in `solve/hub.rs`). On a battery with closed-form hub
   partitions the rail is algebraic between solves: `u = u0 + ρ τ + κ Σ b
   (vc − vc(t0))`, `κ = −r / (1 + r G)` (0 at `r = 0`), `ρ` the drain's line,
   `(dE/dsoc − (dR/dsoc)·draw) / (1 + r G)` times `dsoc/dt` (the implied
   trajectory's SoC term; `rails::CellDrive`). With each closed-form hub
   partition linearised at the aim it takes at the solve (`vc' = f e + γ
   (u − u0) + r0`, `e = vc − vc(t0)`), the partitions are a diagonal system
   with a rank-one coupling, and the rail's slope transforms to `(ρ / s + κ
   B(s)) / D(s)`, `D = 1 − Σ c / (s − f)`, `c = κ b γ`. With every `c` of
   one sign, `D` has one zero between each two consecutive poles and one
   beyond them (above the last for `c > 0`, stable only while `D(0) > 0`;
   below the first for `c < 0`), each found from its nearer pole by Newton
   kept in its bracket. Each mode's share is its residue, `s = (ρ / λ + κ
   B(λ)) / D'(λ)`; the drain adds the line `ρ / D(0)`, and a partition that
   follows no rail (`c = 0`) its own rate; the shares are checked against
   the rail's slope, `ρ + κ Σ b r0`. The partitions follow that path as they
   follow a CC rail capacitor's (above), each at its own rate with its own
   pull on the cell inside the path: every partition is driven by every
   other's motion, exactly to the system's first order. A curved segment
   takes each mode's pull at its asymptote and its transient's share, `we
   e^(fτ) χ_k(τ)`, `χ_k = ∫ φ_k` (a falling gate's settled current was 104x
   W0 at `vc_a`'s slope, its rail barely moving, so no band's event came
   while the transient ran), and keeps its coupling band (point 3). The
   rail's watch is the path's first passages of the power windows, the
   kinks, and the curved closed-form partitions' band and coupling edges,
   capped at `RAIL_DRIFT_V`, which bounds the linearisation of the static
   partitions' and direct devices' draws and the curvature of `E(soc)` and
   `R(soc)`; the SoC knot is watched on its own line. A partition's
   crossings are searched only until the path first moves `RAIL_DRIFT_V`,
   where the rail solves and aims it again. The horizon clamp solves a
   cell's rail whenever its closed-form partitions follow it (path or
   drive), so the rail node published at a frame is the rail they
   followed. The SoC integral (point 6) holds each partition's draw on its
   own exponential between solves, leaving out its response to the path,
   whose charge over a drift-capped interval is second order. Measured
   (F1r15): F1r13's 636-board draining-cell probe none over W0 (374 at
   `e211c55`), and the SAP-1 on its battery 45 % faster than at `3ca4004`
   (point 3, cost bars). Before it (round 14, ruling R15 (a)) each
   partition was driven by its own pull and the drain's line alone, and the
   others' motion entered only first order: 193 of the multi-partition
   probe's 600 boards over W0, worst 118x.

   **The cell's fallback drive.** Couplings of mixed sign, an unstable one,
   or a system `cell_modes` otherwise declines keep round 14's drive: each
   partition takes its own pull on the cell, `κ b`, into its rate and its
   nodes at the aim, and the drain's line as a ramp drive (`J(f, 0, τ)`, a
   curved segment adding its transient's `we τ² e^(fτ) / 2`); its
   crossings are roots of `rails::line_crossing` searched to twice the
   time the line takes to move `RAIL_DRIFT_V`; the rail's watch is the
   implied trajectory with each partition's exponential and the line, its
   rate widened by what the partitions following it add (`J <= τ / −f`),
   capped at `RAIL_DRIFT_V`; the others' motion stays first order under
   it, and the SoC integral takes each partition's response to the line.
   That drive takes no coupling band.

   **Known divergence** (F1r15, ruling R17 (a); `w1f-cell-multi-partition`,
   Equivalence harness rules). Between rail solves on a battery the path
   takes the static partitions' and direct devices' draws linearised at the
   solve while the rail moves up to `RAIL_DRIFT_V`, and what that leaves is
   second order in the rail's motion. On a cell with a curved and a second
   partition it can still exceed W0's tolerance: the 600-board
   multi-partition cell probe has 12 boards over W0 (worst 20x, a 22 Ω
   partition's current beside a 150 Ω gate charging 145 nF on a 5 Ω cell)
   and 10 whose rail loop does not settle at setup (point 3's `-4`),
   against 193 and 38 on round 14's drive and 433 and 38 before it. A cap of 1 µV brings
   the measured rows down about fivefold, but the rail loop then fails to
   settle on some boards; a cap derived from W0 was the other option
   offered. Recorded as a bounded
   divergence; the proper fix is perf round 3's (W1f known follow-ups; F5
   did not take it).

5. **Bench PSU regimes** (Q-O36: classic's state machine, exact; entry at a
   current crossing; the hub tolerance applies to voltages only). The state
   is `reg`: 0 (CV) or 2 (CC), starting at 0 at load. The rules run at
   every rail solve (so at load, at every instant that touches the rail,
   and at every rail event), at most one transition per PSU per solve:
   - **CV → CC** when `|I| > iLimit`, strictly (SO:319), where `I = A0 + G
     V` is the draw at the setpoint, unless the CC solution reaches the
     setpoint, `dir·u_cc >= |V|`, which keeps CV (the compliance clamp,
     SE:1793-1815, with no tolerance band for a bench PSU since
     `fix/psu-railcap-cc-entry`: a `dir·u_cc` any amount short of `|V|`
     enters). `u_cc` is the CC solution's settled rail, `(dir·iLimit −
     A0)/G` (point 2), with or without a rail capacitor: the capacitor only
     sets how fast the rail gets there. For a linear rail `dir·u_cc >= |V|`
     is the same as `dir·(A0 + G V) <= iLimit`, so the clamp never vetoes an
     entry; it is the guard for a removed load (the CC candidate far past
     `V` in the PSU's direction, `dir`). The instant is the event where the
     draw first exceeds the limit: a commit, a toggle, load, or a rail event
     at the crossing. The rail then solves in CC at the same instant.
   - **The CC operating point on a non-monotone draw** (defect G, ruled
     2026-10-31). A chip powered only inside its window draws there and not
     past it, so the draw need not rise with the rail across a window edge,
     and the CC Newton from the setpoint can settle past the setpoint on the
     chip's unpowered draw, where the clamp kept CV however far the draw at
     the setpoint was past the limit (ACC-EV's probe: a 74LS00 sinking 32 mA
     through 1Y on its own `{5 V, 50 mA}` rail beside 125 Ω held CV at 72
     mA, its CC Newton at the unpowered 6.25 V), or step over the segment
     holding the first root to a lower one. The clamp's `u_cc` is therefore
     the highest rail at or below the setpoint, in `dir`, where the draw
     meets the limit: the first one reached coming down from the setpoint
     (`rails::cc_entry_rail`). The Newton's root stands when no free rail
     IC's window edge lies between it and the setpoint (`power_edges`: each
     edge of a rail IC with one supply pin on the rail, the other pin read
     where it stands; a held IC's verdict does not move with the rail);
     otherwise the segments between edges are walked down from the setpoint,
     each read two reached widths inside its lower edge: the first whose
     draw there is within the limit holds the root, solved there by the rail
     Newton bracketed to the segment, and that root is the CC rail the entry
     starts from. An edge where the draw falls within the limit only across
     it is no operating point: the rail lands on that chip's power edge,
     which is the power hold's case (W1c point 1a): the search fails with
     its last evaluations either side of the edge, which the hold's trigger
     reads as the chip alternating. In the probe the chip is held off and
     the PSU is CV at 5 V, 40 mA. The hold's probes, its held solve and a
     release solve the rail again whatever partitions they dirty: a chip
     whose pins are all on the rail or fixed nodes dirties none, its direct
     draw alone reading the hold, and a pass that found the rail clean
     solved nothing (the verifier's repros: CV past the limit with the chip
     free, and a CC rail published at 2.25 V with 45 mA delivered against
     55 mA fed in). An edge at the setpoint itself counts (just under it the
     draw can be another power state's); an edge whose lower band reaches
     the Newton's root down to it is no landing (the draw within the limit
     there is that root's); and a landing's last three evaluations are
     `up`, `dn`, `up` whether or not the walk read `up` first.
     A PSU with a rail capacitor (ruling R68)
     takes the same search, an edge landing included as a CC candidate
     below the setpoint: its rail lands nowhere at the instant (CC starts
     from the capacitors' charge, at the setpoint, and the draws carry it
     down to the edge, where the hold meets it as on any CC ramp), and the
     Newton's candidate kept CV past the limit (G's probe on 10 uF: 72 mA
     from a 50 mA supply). So that entry holds, CC with a rail capacitor
     leaves at the setpoint only with the draw there within the limit (point
     4; a rail that climbed to it had it so). And a chip held there is
     retried, at its release level or the host step, only while its powered
     draw would fit: R44 (a)'s draw threshold, at its edge on a CC
     segment and at the setpoint in CV, where a release passed on the way up
     re-entered CC at once (cold 696 tpl 2's 555, held 51 times within a host
     step, the hold-episode guard's `-4`; 175 tpl 2 and 627 tpl 4 alike). G's
     probe on 10 uF ends as without the capacitor, the chip held off and CV
     at 5 V, 40 mA
     (`a_chip_powered_only_inside_its_window_on_a_rail_capacitor_is_held_off_as_without_one`,
     `a_555_at_its_floor_on_a_cc_rail_capacitor_is_held_once_and_the_runs_complete`).
     Within a segment the draw is
     taken to rise with the rail. Below every edge, a Newton that cannot
     reach the lowest segment's root from its top has it bracketed by steps
     doubling down (a HIGH stage into a node below ground draws more as the
     rail falls, to its `R_out` floor: cold 247 tpl 2's chip held powered by
     the hold's probe has its one root at -2.68 V), and a draw that reaches
     the limit nowhere below leaves the Newton from the setpoint as it was:
     a chip whose drivers sit in a hub partition is linearised in `A0` and
     `G` at its present power, so its edge moves nothing in the scalar draw,
     and the rail loop's passes judge it at the rail they move to (cold 461
     tpl 2). Passes that settle the PSU in CV past its limit, with a free
     rail IC's power edge under the setpoint, are no operating point: the
     curve under that edge is the linearised one, not the circuit's (ruling
     R74, cold 471 tpl 2 and 673 tpl 0: a 74LS00 sourcing into a node below
     ground kept 64 mA under its floor on the curve and drew nothing in the
     circuit; CV at 190 and 76 mA on 55 and 23.5 mA PSUs). The rail loop
     then lands at the highest such edge (`cv_past_limit_landing`), and the
     hold has a rule of its own: a powered probe that settles the PSU in CV
     past the limit means the chips starve, their supply read two reached
     widths under that edge; one its own window turns off in the unpowered
     probe is freed, not held; a retry whose probe settles in CV past the
     limit carries nothing. Held off, the PSU is in CV at the setpoint
     delivering 0 and 5.87 mA, exact mode's answer. A landing that each
     clock rise recreates (cold 360 tpl 3: a gate whose ground pin a
     capacitor holds at the rise draws 140 mA even at its floor's rail) is
     held and freed once a host step and declines by the episode guard;
     ruling R76 (b) keeps that decline (the R53 list) rather than an
     exception for edges the circuit recreates. A general fix of the linearised scalar curve (the hub
     partitions re-solved under trial rails, as exact mode's search does) is
     a later item. Pins:
     `rails::tests::the_cc_entry_takes_the_highest_root_at_or_below_the_setpoint_and_lands_on_a_power_edge_as_an_error`,
     `a_chip_whose_draw_takes_its_rail_past_the_limit_only_inside_its_window_is_held_off_at_its_power_edge`
     and `a_cc_entry_takes_the_highest_operating_point_at_or_below_the_setpoint`
     (`solve/rail_tests.rs`, both modes).
   - **A rail capacitor short of the setpoint is demand** (probe P9, found
     by F1; manager ruling 2026-10-06). A rail capacitor is not a term of
     `A0` or `G` (point 7), but the CV → CC test adds the draw it would take
     at the setpoint while it holds the rail short of `V`. With the deficit
     `d = dir·(V − V_f − vc) > 0` (`V_f` its fixed pin's voltage, `vc` its
     charge, both in the PSU's direction), that draw is `d / esr` for `esr >
     0` and unbounded for `esr = 0`. So a load's discharged capacitor puts
     the PSU in CC at once, as in fixed classic (whose capacitor starts at 0
     V and whose 1 ps load seed sees the charging current), and the rail
     ramps at the limit by point 4's closed form until the exit below. The
     compliance clamp then reads the rail those capacitors hold, not the
     settled CC rail, which with no load is far past `V` (`iLimit / G` at
     the node shunt) and would keep CV. A capacitor at or past the setpoint
     (in `dir`) adds nothing: one carried charged across a reload stays CV.
     In CV the deficit is zero (`esr = 0`, rule 2a) or decays with `esr·C`,
     a moving term of the CV-entry watch (point 4), so a supply inside its
     limit never re-enters. A total past the limit against the PSU's
     direction (`dir·I < −iLimit`) is a source back-driving the rail, which
     the CV supply sinks (ruling R40): the CC solution at the instant, the
     charge plus its current's drop across the esr, is past the setpoint
     there, so the clamp keeps CV without reading the charge (FXI, bug D: a
     pulse's edge through 2.2 Ω back-drove the rail by 16.6 A net of a
     capacitor's 3.2 A demand, and the clamp, reading the 0.37 V charge,
     entered CC at a 6.2 V CC rail). Contract tests: 100 µF on a 5 V, 0.1 A
     PSU rises at 1000 V/s and reaches 5 V at about 5 ms, matching classic
     within the normal criterion; the same capacitor charged to 5 V at load
     stays CV.
   - **CC → CV** when the CC solution reaches the setpoint, which for a
     linear rail is the draw at the setpoint back within the limit,
     `dir·(A0 + G V) <= iLimit` (classic since `fix/psu-cc-exit`: its
     commit leaves CC exactly when the solve's own compliance clamp selected
     CV, so there is one threshold, not two). Without a rail capacitor the
     present rail in CC is the CC solution, so CC also exits once `dir·u >=
     |V|`, the entry's own clamp: the draw test is that clamp only while the
     draw rises with the rail, and an IC that powers off past its window
     does not (FXH, W1gV4's bug A: a 555 on the rail, its DIS on through
     its RST tied to OUT, past the limit at the setpoint; its CC rail,
     solved from a short's 0.2 mV with the 555 unpowered, went to `iLimit`
     over the node shunt, 2.1e11 V, where the 555 is past its window, and
     the draw test held it there; now it leaves for CV and re-enters at the
     powered CC rail, 2.047 V, exact mode's). A CC Newton that cannot
     settle past the setpoint, with the draw there within the limit, gives
     the setpoint (F5f: cold fuzz seed 22 tpl 4, a fuse tripping in CC sent
     the iterates to 3.1e10 V over the node shunt, one ulp apart, and the
     advance declined with `-4`). With a free rail IC's window edge between
     the CC rail and the setpoint the draw test is no test of the CC
     solution at all, the draw at the setpoint being another power state's:
     there the rail itself is the exit, `dir·u >= |V|`, as in exact mode
     (defect C, cold 364 tpl 2: a 74LS00 off on a 3.43 V CC rail, which once
     powered at the 4.881 V setpoint back-feeds the rail through its
     clock-high outputs, its draw there within the limit; the default exit
     read that and left CC at each clock-high phase, 472.9 W0 against the
     fine reference, which stays CC with the chip off; pin
     `a_cc_rail_below_a_chips_window_stays_in_cc_when_the_chip_would_feed_it_at_the_setpoint`).
     Its twin: a CC solution past the setpoint while the draw at the
     setpoint is past the limit passed the setpoint only on another power
     state's draw (a chip off past its window), and CV there would deliver
     past the limit, so CC takes the entry's operating point instead, coming
     down from the setpoint (ruling G), and leaves for CV only when that too
     reaches the setpoint (cold 267 tpl 2: exited to CV, the loop settled
     with no partition to re-solve, 114.8 mA from a 62.7 mA PSU through each
     clock-low phase; pin
     `a_cc_exit_whose_cv_draw_is_past_the_limit_enters_again_at_once`, both
     modes). A search there that fails other than by an edge landing leaves
     the exit as it was, and a CC continuation Newton that fails takes the
     entry's operating point instead, its own failure standing (solved
     again, so its iterates are the hold's trigger's) only when that search
     fails too other than by an edge landing: a HIGH stage into a node below
     ground draws less as the rail rises, and a pass whose partitions moved
     left the Newton from the last rail nowhere to settle (cold 536 tpl 2,
     which exact mode and 416537e hold in CC at 3.417 V). With a rail capacitor, a
     jump to `V` would draw without bound, so the exit is the rail itself
     reaching the setpoint, `dir·u >= |V|` (probe P9: classic stays in CC,
     −0.1 A, up to 5.0 V). The rail then solves in CV at the same
     instant, and every hub partition, linear or curved, static, stepped or
     closed-form, re-solves at the new rail there instead of carrying a
     first-order correction from where it was last solved (manager ruling
     2026-10-07, F1 finding D: a linear partition solved at power-on and
     extrapolated 0.725 V read 1.1 nA where classic reads 0.01 nA). A
     static one's cache answers there only with a solve at the setpoint
     itself (point 3): a hit within its band would be that correction again,
     and a ramp's narrow bands store entries just below the setpoint, so
     cache on and cache off would part by its error.
   - **The exit's tie** (ruling R1 (a), 2026-10-15; F1v23 finding 1).
     With a rail capacitor the supply current is continuous across the
     exit: in CV the capacitors take what the limit leaves of the CV draw at
     the setpoint, `I_cap = dir·iLimit − draw(V)`, each its share `C_k /
     C_rail` as it took its share of the segment's current, held one esr
     drop below the rail (`vc = V − V_f − esr_k·(C_k/C_rail)·I_cap`, in the
     PSU's direction), from where it relaxes to `dV` as rule 2a's
     exponential (at `esr = 0` its charge is the rail and its current 0,
     rule 2a's divergence). The CV → CC test reads that charge back as
     demand, so it sees the limit itself rather than the segment's current
     at a rail that landed a reached-width past the setpoint. Since a charge
     rounds at the rail's scale, which `1/esr` magnifies, each held charge
     is stepped a last bit toward the rail, at most `TIE_STEPS` (8) times,
     until the test reads no more than the limit, and the rail loop ties
     again after each later pass of the instant, at the draws its re-solves
     settle to. A capacitor's demand then enters CC only past `iLimit +
     REACHED_REL·max(1, iLimit)`: a real change of the draws, never the
     rounding of a re-solve. Without the tie a CC exit with `esr > 0`
     re-entered CC at the same instant in 42 of 96 cases, leaving CC 4 to 20
     µs late with 2 to 10 mV of overshoot.
   - **An exit the instant forces keeps the charge** (FXI, bug D). The
     supply current is continuous only where the segment brought the rail to
     the setpoint. A CC solution carried past it by a step of the draws at
     the instant (a source edge or toggle back-driving the rail through the
     capacitors' esr), with the segment's rail still short of the setpoint
     there (`dir·u_seg < |V|`, read before the solve re-aims it), is no ramp
     reaching its end: the supply current steps, as in classic, and each
     capacitor with `esr > 0` keeps the segment's charge (`vc = q − V_f` in
     the PSU's direction), from where it relaxes to `dV` as rule 2a's
     exponential; no tie follows at that instant. The tie would hold it the
     whole step's current times its esr below the rail: on W1gV4's cold seed
     704 a pulse edge back-drove a 5.06 V rail by 19.8 A and the 1.44 Ω
     capacitor, charged to 0.57 V, was held at −23.7 V, from which the next
     CC entry published −22.4 V (seed 985: a clock tied to the rail by a
     switch, −6093 V held, −5960 V published). Classic, exact mode and the
     core now agree on seed 985's linear parts (rail minimum 82 µV) and on a
     reduced board within 2 mV
     (`a_cc_exit_a_back_driven_edge_forces_keeps_the_rail_capacitors_charge`).
   - In CC the PSU publishes `elementI = −dir·iLimit` exactly; in CV, `−I`.
   - The rules are pure functions (`devices/sources.rs`): `regime_step(reg,
     i_at_v, u_cc, u_rail, V, iLimit)` and `regime_step_rail_cap` with the
     same arguments. `u_cc` is the rail voltage of the CC solution, which the
     CV → CC clamp tests (in CV the rail is `V`, so the caller computes the
     CC candidate; passing the CV rail would make the clamp keep CV
     forever); `u_rail` is the present rail, which the CC → CV exit tests
     (without a rail capacitor beside the draw at the setpoint).
     An `iLimit` that is not `> 0` has no regime.

   Before the 0.6.2 fixes classic exited at 98% of the setpoint and
   re-entered inside the next solve, so its `reg` flickered every step on a
   slight overload and its adaptive step locked at 10-15 ns (probe P12);
   with a rail capacitor its compliance band held an overloaded supply in
   CV past its limit (probes P10, P11). Both are fixed in classic
   (`fix/psu-cc-exit`, `fix/psu-railcap-cc-entry`), and the rules above are
   fixed classic's, so `reg` is compared like any state. Classic's in-solve
   entry tolerance (`iLimit + 1e-12 + 1e-6 iLimit`) is not reproduced: in
   the band between it and `iLimit` classic enters one step late, at its
   commit (at most 1 ppm over the limit for that step), the core at once. A
   rail landing exactly on the setpoint at a classic step boundary may read
   either regime there. Scale is 1 (Q-O40): the setpoint is never
   scaled.

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
     b·(vc0 − inf)·(e^(FΔt) − 1)/F`, and `(a + b vc0) Δt` when `F = 0`;
     under the cell's fallback drive plus its response to the drain's line,
     and on the cell's coupled path without its response to the path, Rails
     point 4), each stepped term held from its last commit (one step, like
     classic).
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
     capacitance makes the rail a closed form (point 4), with the
     capacitors' `esr` entering as a static offset and its nanosecond
     transient not modelled. A rail capacitor whose far pin is a waveform
     source's node, directly or through a source stacked on it, is not
     handled (an edge there does not solve the rail, and the CC segment
     ignores the far node moving), so such a load is `-3` (point 1) until it
     is (a decline added with F1 round 2, the manager informed, 2026-10-07),
     in exact mode too: the hub is classified before exact mode frees the
     rail node.
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
   and the device-state buffer; from F5, the power-hold table and each
   hold's release (W1c point 1a), and with the hub's state its held draws
   (ruling R44 (a)) and HG1's count; from R68, each stepped hub
   partition's committed rail-capacitor charge and its pending trial's, and
   the charge the latest commit hands the rail solve (pin
   `a_rolled_back_instant_restores_the_cc_rail_capacitors_charge_a_stepped_commit_hands_over`). A hold the static solver took in the
   failed instant is undone with it, and so is the retry's arming, in the
   owner and in the static solver (pin
   `a_retry_taken_in_an_instant_that_then_fails_is_armed_again`). The
   guard's yardsticks are not state: the host step's length is set at each
   `advance` call and the instant's foreign changes are cleared as each
   instant opens.

9. **Exact mode** (`NO_RAIL_SHORTCUT = 1`, load-time only, W1f Q10;
   orchestrator lock: "merges hub-coupled partitions with the rail node as
   an unknown"; kept as locked, W1f Q3).
   - The rail node is a free node. Every partition that reads it, and every
     direct device, joins one partition with it, classified by the usual
     rules (on the SAP-1 it holds nearly the whole board and is stepped),
     except that it is never closed-form (the source's SoC and regime
     advance per committed step, so it needs steps) and a battery's is
     stepped whatever it holds: SoC is state, and `E` and `r` move with it.
     The owner (Dev) ruled a battery partition with `r > 0` stepped
     (2026-10-15); FX1 found that `r = 0`, where `u = E(soc)`, froze the
     same way and extended the rule to it (`partition.rs:583-601`). A static
     partition, solved only at events, held the rail and the draw at their
     load values and charged SoC at that frozen current (an LED and 220 Ω on
     the generic cell: 4.9869 V against classic's 4.7918 V at 1 s, 3 % more
     discharged). With nothing else to step, the partition steps under the
     stepped controller's own limits, so a host that advances in long jumps
     sees steps up to `H_MAX` (10 ms) and their error (FX1: about 0.5 of
     W0's tolerance at an LED's cathode).
   - The source stamps in that partition: a battery with `r > 0` as a
     Norton, `1/r` from the rail to the return plus `E/r` injected, with `E`
     and `r` held per step from the SoC at the step's start (classic's
     cadence, SO:141-156); a battery with `r = 0` and a PSU in CV replace
     the rail row by `u = E` or `u = V`; a PSU in CC keeps the KCL row and
     injects `dir·iLimit`. The source current is the Norton branch current,
     or the replaced row's KCL residual, the network linearised about the
     solution with the capacitor companions the solve used.
   - No sensitivity, band, scan or rail event: the partition solves the
     rail with everything else.
   - **Plan after the SoC charge** (FX1, FX2). The battery partition's next
     step is planned only once the step just committed is charged to SoC
     and the source restamped from it: a plan made at the commit took `E`
     and `r` a step late. A plan that reads the held partition's trial (an
     IC it supplies) waits after it too, and the held plans run each after
     the held ones that supply it. The supplies a held plan reads are still
     caught up to the instant first, on the drivers in force before the
     event (W1b rule 6), which only that call has; a held supply is not
     caught up again. Since the per-step restamp below, this hold is
     redundant for `E` and `r` (FX45v: 208 runs bit-identical without it);
     it stays as the release backstop and for the order in which the
     scheduled steps are pushed.
   - **Restamp per step** (FX5). One call can commit several steps of the
     battery partition (the horizon clamp, a catch-up before a
     discontinuity). Each committed step is charged with its own charge and
     the source restamped from the SoC it leaves, so the next step, in the
     same call or the next, takes `E` and `r` from it (one call against one
     step per call: 1.389 V apart before, bit-identical after).
   - **Billing** (ruling R5, 2026-10-15, and FX4). Ruling R5 (b) chose
     billing per solved half-step at its own end current, for charge
     conservation. Under the trapezoid that does not conserve (FX4: −0.1 to
     −7 %), so the orchestrator applied the integrator's own quadrature,
     within the ruling's invariant, and informed the coordinator, who did
     not object. SoC advances per committed step by the charge the source
     delivered over it in the integrator's own quadrature, per solved
     sub-step: `I_end·h_sub` under backward Euler, `(I_start + I_end)/2 ·
     h_sub` under the trapezoid, over each half of a step-doubled step. It
     conserves charge with the integrator (24 cases, both integrators,
     within 1e-10; FX45v's 72 more, LED, switched and charging, within the
     probe's own Newton tolerance), where classic's `I_end·h` per step loses
     up to 1.9e-2 under backward Euler and 1.2e-1 under the trapezoid in an
     inrush. A PSU's partition is not billed.
   - **Regime rules** (EX1). Point 5's rules run on the solved rail and
     draw (CV → CC when `dir·draw > iLimit`, CC → CV when `dir·u >= |V|`) at
     every committed step of the rail's partition, and also at its seed and
     at any re-solve at the instant of the last commit: run only for a step
     committed after the last, a PSU stayed CV through its seed, so its
     first step charged a discharged rail capacitor to the setpoint (4.98 V
     at 2.5 µs against classic's 0.0047 V), and a toggle entered CC a step
     late. A transition re-solves the partition at once. The entry is
     point 5's, compliance clamp included. The partition holds no CC
     candidate, so the entry is tried: while the draw rises with the rail
     the CC solution is past the setpoint exactly when the draw at it is
     within the limit in the PSU's own direction, so a draw past the limit
     against it (a source back-feeding the rail) keeps CV without a solve,
     as in classic and default mode (FXH, W1gV4's bug B: `{5 V, 0.1 A}`, a
     9 V source behind 10 Ω and 1k, entered CC on `|draw|` = 0.395 A and
     held the 9.901 V CC solution; with a rail capacitor it alternated CC and
     CV each step, the capacitor's current growing to kiloamps, and the
     rail-capacitor fixtures 44-47, 50, 51 and 60 re-entered CC for a step on
     the capacitor's discharge just after their exit). A draw over the limit
     in the PSU's direction enters CC and re-solves, and a CC solution at or
     past the setpoint is no current limit: CV again at the same instant,
     re-solved (W1gV4's cold seed 839: a 555 whose window the CC rail leaves
     drew nothing past it, and the rail published 2.3e10 V, `iLimit` over
     the node shunt, every other step). Between the entry's CC solve and that
     clamp the operating point is point 5's, the highest at or below the
     setpoint (ruling G; the fine reference shared defect G): the CC
     solution stands when no free IC with a supply pin on the rail has a
     window edge between it and the setpoint; otherwise the draw curve is
     read by trial solves of the rail's partition with the rail held at a
     trial voltage (the CV stamp there; no hold procedure or release runs in
     a trial), walked down from the setpoint as default mode walks it. A
     segment holding the root has it found by the Illinois method on the
     trials and the CC solve then runs from the trial at it; an edge landing
     re-solves in CC from the state before the trials with that edge's ICs
     put to the iterated partition's hold procedure (point 1a), as if its
     Newton had seen them alternate (`StaticPartitionSolver::edge_hold`).
     Where that cannot take the landing, and always on a stepped rail
     partition, whose Newton runs no hold procedure, the owner runs point 1a
     on its own table (`exact_edge_hold`): probed held powered, probed held
     unpowered, then held off, each a re-solve of the partitions the chips
     touch. The landing itself shows they starve (the trial just under the
     edge read the draw within the limit), so a powered probe that fails to
     solve takes them as starving with their supply read at that trial's
     rail (REF, cold 794 tpl 4: a 555 held powered has no CC solution its
     Newton reaches, and the clamp then kept CV at 1.28 A on a 455 mA PSU
     through every clock-low phase). Only a procedure that cannot take it
     either leaves the state before the search, rather than decline the
     instant; a search that keeps the CC solution puts every piece of state
     back, bit for bit. A stepped rail partition's CC solve at the entry can
     fail outright (the chip on above its edge, off below); the search then
     runs from the CV state with no Newton root to keep, and fails with that
     solve only if it places nothing. A CC exit whose CV draw is past the limit enters again at the
     same instant through the same search (a static rail partition only:
     a stepped one's CV draw just after its exit is its rail capacitors'
     current); left in CV, nothing re-solved the rail's partition (cold 267
     tpl 2). A stepped rail partition is searched as a static one is, each
     trial a re-solve that restarts its step at the instant (REF, cold 789
     tpl 3: unsearched, a 74LS00 sourcing into a clock's low phase held CV at
     147 mA on a 107 mA PSU, where the landing holds it off at 27 mA). A chip
     held there is retried by the owner (ruling R51 (b): once per host step,
     or once its supply passes its release level, `exact_retry_stepped`):
     probed free with the regime rules and the entry's search run on the
     probe, it is free if it carries itself there, else the state goes back
     bit for bit. A stepped partition's seed or minimum step whose Newton
     fails with a gated IC's verdict alternating runs point 1a inside the
     stepped solver (`stepped::hold_around`: probed powered, probed
     unpowered, held off), the holds entered in the owner's table after the
     call (`take_stepped_holds`) and retried as above (REF, cold 656 tpl 3:
     a 555 fed through 115 ohm from a 2.1 V rail draws 22 mA at its 1.8 V
     floor where its feed carries 1.7 mA; it slides along its edge, and the
     episode guard declines it, ruling R54 (a), where the seed's Newton had
     failed outright). Such a board re-solves twice at each
     commit while its draw stays over the limit: about 2.7 s of CPU for 10
     ms of a 74LS00 gate on a `{5 V, 20 mA}` PSU (1.8 s before, ping-ponging
     the rail). A back-fed PSU in CV sinks without limit, as classic's model
     does; a supply that cannot sink is not modelled.
   - **A step cut at every regime crossing** (ruling R102 (a)). A
     stepped partition's Newton judges each free chip whose supply it
     holds at the step's end, for the whole step (W1c point 3), and a PSU
     in CC leaves it only at a commit whose rail reached the setpoint
     (point 5's exit, `dir u >= |V|`). So a step whose end crosses a chip's
     power-window edge, or a CC rail's setpoint, is cut where it lands,
     just past it (`CROSS_TOLERANCE_V`), by regula falsi on the step length
     over the farthest crossing's signed distance, as a CC step on rail
     capacitors is cut at its setpoint and its rail ICs' edges in default
     mode (ruling R68). The verdict flips, or the PSU leaves CC, at that
     commit. A chip held (point 1a) watches nothing here: its release is
     the owner's retry. The CC stamp carries the setpoint for it
     (`ExactKind::Inject`). Uncut, coarse cold 757 tpl 0 carried its rail
     to 1.840 V across a 13 ms step, a 555 judged off until 21 W0 past its
     1.8 V floor (cut, it powers at 1.800035 V; the board then chatters at
     the floor and, with no slide on this base, declines through R54 (a)'s
     guard inside that host step, at 0.1893 s before); and cold 149 tpl 5's
     CC rail ran a 7 ms clamp step past its 9.42 V setpoint, the CV
     re-solve at the commit leaving its timing node 0.6 V off the fine
     reference (299 W0; cut, 0.001 W0). The independent verification of
     R102, R105 and R106 (EXACTV) found no defect and recorded two costs of
     the cut: on a CC rail, a chip at its power edge now reaches the edge
     where it chatters, so exact mode, which has no slide yet (R53's
     exact-mode piece), declines through R54 (a)'s guard earlier than
     before (757/0, 648/5, 733/3, 735/3, the fine reference with it); and
     on boards with a tiny rail, up to 2.2 times the Newton iterations per
     step, with no crawl. A CC rail already past its
     setpoint within the shortest step is an exit there: the step is that
     shortest one (corpus 47 at 1 ms host steps: the 5 ms stop landed a
     few ulps short of the setpoint, still CC, and the next step ran the
     esr-free capacitor to 6 V, its exit seed publishing C dV / 1 ps = 1e8
     A from the PSU). A chip's edge within the shortest step is left
     uncut: a chip sitting on its edge would step at the floor (cold 701
     tpl 2 ran 4.7 times its wall that way).
   - **A hold at a step's start** (ruling R106 (a)). A stepped partition
     whose step from an instant holds a chip off there (point 1a inside the
     stepped solver, `stepped::hold_around` at the floor) leaves that step
     pending with the hold already in the owner's table, while what it
     published at the instant was solved before it: the chip unpowered,
     its outputs at the powered solution, KCL broken at every sample until
     the step commits (the CC ramp's `1y` at 3.599 V and its LED lit at
     1,516 held samples in 30 us; cold 2575 tpl 2's node 2, OUT and DIS,
     at 0.38 V against the 1.87 V its divider holds, 11,620 of 11,628 held
     samples to 30.2 ms). Each such partition is re-solved at the instant
     under the table (`exact_resolve_held`, flagged by the partition's
     `pend_held`), as at any power change; the chip's outputs and pulls
     sit in its free supply's partition (W1c point 2), so that one
     re-solve is the downstream one. Default mode solves the rail's chips
     in the rail loop and was already clean (R53-P8 checked).
   - An `r = 0` battery publishes at the load instant the 1 ps seed's
     capacitor current, `C·E / 1e-12` (5e6 A for 1 µF), as classic does;
     default mode publishes 0 there.
   - Cost: the merged partition is a dense Newton over the board, the same
     class of work as classic's global Newton (83-112 s CPU per simulated
     second on the SAP-1, build log 2026-09-30) or worse, since every commit
     instant lands a step. So S2 compares against it over a declared window,
     the first 20 clock cycles (Equivalence harness rules), and the rail
     fixtures cover the rest. Measured on that window (F1 round 1, the SAP-1
     battery variant, 41.7 ms): 8.47 s of CPU frozen and 13.07 s running,
     against 0.14 s in default mode, about 200-300 s of CPU per simulated
     second. The exact-mode fixes since (FX1-FX5, EX1) moved it within noise
     (8.5-9.0 s frozen, 12.5-13.5 s running).
   - **Block elimination over the rail** (ruling R105 part 2). The merged
     partition's branches meet only at the rail: the battery SAP-1's 323
     nodes fall into 217 blocks, the largest 20, once the rail is taken
     out, where the dense LU of the whole was 79 % of the samples. Each
     Newton iteration of the rail's partition (stepped or static) factors
     by `lu::lu_factor_hub`: the connected components of the system's
     pattern without the rail, each a dense LU with partial pivoting
     within it, and the rail from the Schur complement, one scalar,
     `s = a_rr - sum a_rk A_kk^-1 a_kr`. A source row (`u = V`) is such a
     row too. It applies at 32 unknowns or more and when the blocks'
     summed cubes are at most a quarter of the whole's; a singular block or
     complement factors the whole as before. Its answers are the whole
     LU's to rounding, as far as Newton's own tolerance lets a rounding
     show: the pin
     `exact_mode_solves_the_rails_partition_block_by_block_as_it_solves_it_whole`
     takes the same steps with every reading within 1e-9 of the whole's
     (past 1e-13 V and 1e-15 A), and so does every fixture in exact mode
     at 1 ms frames; the battery SAP-1 (running, 0.25 s), whose Newtons
     stop at classic's tolerance a rounding apart, sits within 7.6e-8 V
     of the whole's (8e-4 of W0) and its capacitor currents within 0.5 of
     W0's 1 nA, not growing. Its CPU there: 23.4 s to 4.0 s for 0.05 s
     (running), 12.1 to 3.6 s (frozen), measured on a loaded machine.
   - **What a capacitor current resolves to** (rulings R109, R111, R112).
     A capacitor current through a path of conductance `g` is resolvable
     only to about `g` times the voltage rounding: on very stiff paths
     W0's 1 nA floor is below that. A capacitor's published current is its
     companion's over its last committed step, whose conductance is `C /
     h`: after a step of 1 ps at a clock edge that is 1e6 S per uF, and the
     battery SAP-1's C4 read 2.4 nA apart between two solves a rounding
     apart (block elimination against the whole LU; the four instants were
     each a 1 ps step, `|di| = C / h |dv|` within the ulps). So blocks and
     whole are held to each current within 1e-9 of the reading plus `g`
     times the voltage bound, a thousandth of W0, with `g` the device's
     own path (`exact_mode_blocks_and_whole_agree_within_each_paths_conditioning`).
     The companion current is the right one to publish: at a committed
     step of a linear partition it is the branch formulation's (each
     capacitor a source of its charge, `stepped::branch_currents`) within
     1e-9 and 1e-15 A, and in an iterated partition within the conductance
     at its nodes times Newton's tolerance, the two standing equally far
     from a fine reference (`a_published_capacitor_current_is_the_branch_formulations_at_its_step`).
     At an instant where a waveform source steps (a clock edge) the
     published current is the right-hand limit, the current after the
     edge: a fine reference read as `C dv/dt` backward over its last
     microsecond reads the left-hand one there (5 mA against 50 nA on a 1
     kohm clock feed), so a comparison at an edge reads the reference from
     the right.

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
     before the segment's quiet instants. The bound rests on that, not on
     classic: its headless host stepped over such a ramp (`per/10` steps,
     HL:150-153) until `685179c`, and now lands a step on each corner
     (Equivalence harness rules, source edges).
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
     - a partition that reads the node, directly or through a source
       stacked on it, is classified at load (membership and classification
       stay fixed at load). A **linear closed-form reader** stays
       closed-form (ruling R3 (b), 2026-10-15): one W1b point 3 and
       LEDs point 5 classify closed-form, that is not a hub partition, and
       that holds neither a stamp of an IC supplied from a ramping node nor
       a watched read pin of an IC supplied from a ramping node, the rail or
       a hub partition's node (a threshold moving between events). Its
       network is linear, so `x0` and `i0` are affine in each ramping node
       `v`, with `dx0/dv` and `di0/dv` solved exactly through the segment's
       own factors, and between corners `v` is a straight line of slope
       `b`. So from an aim at `t_a` its nodes are `x0_a + r_x τ + dx vc` and
       `vc = inf + (vc_a − inf) e^(fτ) + (r_i / C) J(f, 0, τ)`, with `r = Σ
       b · d/dv`: Rails point 4's drive with the line `φ_0(τ) = τ` in place
       of the rail's exponential. Each corner of a ramp it reads aims it
       again from its exact `vc` (the midpoint changes no slope and does
       not), and a crossing on its nodes is the exact root of that
       trajectory, searched up to the next corner, landing as every
       closed-form crossing lands;
     - every other reader (nonlinear, two or more capacitors, or excluded
       above) is stepped, whatever it holds. It lands a step on each corner
       and restarts there at `POST_EVENT_H` with the BE anchor, as after
       any discontinuity, and each step reads the boundary at its own
       instant. Ruling R3's second half: a nonlinear or multi-capacitor
       reader stays stepped and is checked against classic under the
       Richardson rules where classic converges (Equivalence harness
       rules). The rail-coupled exclusion above is F6's narrowing of R3
       (b), which the coordinator accepted (2026-10-15): the rail's drive
       under a CC rail capacitor (Rails point 4) is an exponential, and a
       crossing on the exponential plus the ramp's line is outside
       `rails::driven_crossing`'s single-extremum argument. Composing the
       two drives with a crossing solver for both is a W3 follow-up (W1f
       known follow-ups);
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

### Known follow-ups (W1f)

Recorded at the W1f integration. None changes a rule above until it lands.

- Coupling band holes (V11, FIN1): `err_at` returns 0 where the bound
  does not close and `CoupleAim::of` drops a segment that does not decay,
  so the next aim starts afresh without the inherited error; the floor
  (the larger of D(1 ns) and 1000 reached widths) applies even when the
  tolerance is used up. Neither fired in the 768-board CC sweep after
  FIN1; fixing either needs a decay-free bound or a floor that yields
  without risking sub-ns rail events (a ruling).
- **Perf round 3** (the coordinator, 2026-10-15, with ruling R7): the SAP-1
  on its battery at 1x real time or better, 5x the stretch goal, a
  first-class target. It ran at about 0.4x: 2.1-2.4 s of CPU per
  simulated second (V6; 2.2 s in F1r8's profile), against about 20 ms on
  the ideal supply (V6; 29 ms in F1r8's profile). Round 15's coupled cell
  path (Rails point 4) brought it to 1.16-1.35 s (F1r15), about 0.8x; F1r8's
  profile below predates it and has not been taken again. F1r8's profile,
  per simulated second, battery against ideal: 311k events against 21k,
  132k `Rail` events against none, 147k rail moves, 576k IC evaluations against 43k, 445k
  delta passes against 22k, 4.6k hub re-solves, about 16.8 µs of CPU per
  rail event. Two cheap structural causes:
  - `direct_at`'s IC-draw closure (`solve/hub.rs`) is 44.5 % of self time:
    on every call inside the rail Newton it walks every rail IC and driver
    twice, at `u` and `u + RAIL_DELTA`, where only the drivers with both
    ends non-free contribute, a set fixed at load that can be precomputed;
  - the rail-event rate: an event per `RAIL_DRIFT_V` (10 µV) of drift from
    the closed-form hub RCs (the 555 timing and CV capacitors through
    0.9 Ω), each republishing every static hub partition (13 % of the time)
    and re-aiming the closed forms (about 4 IC evaluations per event);
    malloc and free take 6 %.
- **The multi-partition cell divergence** (ruling R17 (a); Rails point 4,
  known divergence `w1f-cell-multi-partition`): 12 of the 600
  multi-partition cell probe's boards over W0, worst 20x, from the static
  and direct draws linearised at the solve while a battery's rail drifts up
  to `RAIL_DRIFT_V` between solves. The proper fix is perf round 3's (the
  coordinator, with ruling R17; F5 did not take it); a cap of 1 µV needs
  the rail loop's settling fixed first.
- **Stepped hub partitions on a rail they move** (FXF, after G2bv's
  sweep; not fixed, options to the coordinator). (1) A cell: a stepped
  partition holding a capacitor to the rail on a battery with `r > 0`
  steps with the rail held, and its seeds (`SEED_H`, `C / 1e-12`) and
  steps disagree on the rail by more than its band, so it re-solves and
  restarts at `POST_EVENT_H` (G2bv's sweep: 6 battery boards at under 0.01x
  real time); the CC step's rail row with the cell's Norton, `(E - u) / r`,
  runs one at 136x but 967x W0 off a fine reference in a cap current
  early on (exact mode 39x), the other stepped partition held. Items (2)
  and (3) of FXF's list were F5's and are fixed: the CC seed's rounding
  (Rails point 4, the CC seed; ruling R28 (a)) and the 24 rail-loop cycles
  on boards with a 74LS00 on the rail, which F5D traced to power-verdict
  flips (W1c point 1a), not `R_out` kinks.
- **Coupled stepped partitions on one CC rail** (FXF round 3, FXEFv).
  Each stepped hub partition's CC step holds the other partitions' draws
  where they last committed, a first-order coupling error between them
  (FXEFv's two-branch board 3.4x to 5.1x W0 from exact mode, 6.4x off a fine
  reference where exact mode is 3.4x). Stepping them jointly on the shared
  rail unknown (one step size, as exact mode merges them) would remove it;
  that is a change to the stepped solver's per-partition clocks.
- **CC band events from a closed-form draw** (FXF round 3, diagnosed, not
  fixed). With no rail capacitor in CC, a closed-form hub partition's draw
  moves the rail continuously, and a curved static hub partition (an LED
  and resistor) has the flat 1 µV band (`RAIL_CC_RESOLVE_V`), whose edges
  are rail events: one every 1 µV of the ramp (FXEFv's `cc 128 tpl=1`:
  980k `Rail` events, 2.9 s per 0.1 s; 3 V of ramp at 1 µV). Options: (a)
  the curvature band in CC too, with the rail's own error `δI / G` added
  to its budget (events per W0-sized band, about a thousandfold fewer); (b)
  a coupled path for the CC rail without a capacitor (the cell's form,
  `κ = -1 / G`), with the static curved partitions' draws linearised and
  their band edges as passages; (c) leave it (a ramp of milliseconds).
- **The `R_out` clamp-kink band** (F1r7, INV2): a curvature band ends at a
  driver's `R_out` kink (`io_max`), so about 58 re-solves pile up at it on
  the SAP-1 (its rail near 4.8 V). A first-order jump term across the kink
  would widen the band (filed, not taken).
- **The closed-form RC's CC-exit lag** (F1r10): with a rail capacitor and a
  closed-form RC hub partition, the CC exit lands 49 ns late, inside
  `RAIL_DRIFT_V` over the rail's slope (200 ns there), which puts the
  capacitor's current 0.99 of W0's tolerance off at the start of the exit
  window. It passed with no margin, by `RAIL_DRIFT_V`'s design (Rails
  point 4); no decision has been asked. Measured on the frozen form: since
  round 14 the rail and the RC advance on their coupled path, whose exit is
  a first passage with no drift behind it, but the lag has not been
  measured again, and the test
  (`a_cc_rail_capacitor_with_esr_and_a_closed_form_rc_hub_partition_follow_their_coupled_ode`)
  still allows `RAIL_DRIFT_V` over the slope and describes the frozen form.
- **Exact mode's late power gate** (V6, recorded, not investigated): in
  exact mode a rail IC's power gate turns on about 1 µs late at its floor.
  Exact mode has no rail events (Rails point 9), so nothing lands on the
  crossing.
- **Corpus 06's two fixed points** (V5): its worst current moved from
  3.4e-17 to 2.6e-13 A at round 5 because its solve has two Newton
  fixed points 4e-12 V apart, and which one it lands on depends on the
  cache entry the CC exit replays; not the curvature correction. It still
  passes.
- **psuTight's host frames** (INF1): on INF1's probe board `psuTight`
  (a probe, in no test), the default-mode `v1` at 20 ms varies by about
  6e-8 V with how the host frames the run. Not investigated.
- **The reader-scan listing** (Equivalence harness rules, the reader
  scan): `scripts/core-equivalence.ts` does not yet list the circuits whose
  run ends with stat 20 above 0.
- **W3 analog backlog** (from W1f): (1) compose a rail-coupled linear
  closed-form reader of a resolved ramp, the CC segment's exponential drive
  plus the ramp's line, with a crossing solver for both (Sources point 3;
  until then such a reader is stepped); (2) model the fast `esr·C`
  redistribution mode between rail capacitors of different `esr` (ruling
  R11 (a), 2026-10-16: a transient lasting about 5 µs after a step, one
  capacitor's current 11.5 % off 0.5 µs into it, the rail within 8.1e-6 V;
  Rails point 4).

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
       interval, so the close is exact. On a moving rail, the rail's own
       trajectory (ruling R30, below).
     - **Closed-form:** the segment's own exponential trajectory, closed in
       closed form (points 3 and 5). On a segment a known rail drives
       (Rails point 4) the trajectory carries the rail's terms too, which
       no single exponential holds; such a device is re-anchored instead
       (ruling R21, below).
     - **Stepped:** the mean of the drives at the step's own start and end
       states, held over the step (rulings R80 (a), R84 (B)), where classic
       holds the end drive (its backward rule, `rate·h` from the accepted
       step's solution, SE:5510-5531, 6029-6054; divergence D37). The rate is
       `q − zero` for the ratio `q` a kind integrates (`P/pMax` or
       `(I/limit)^2`, zero 1; a PTC's `I^2`, zero `I_hold^2`;
       `StressDrive::Constant::zero`), and `q` relaxes exponentially across an
       RC step, so where either end accumulates the step holds the
       logarithmic mean of the two `q`s less `zero` (`StressDrive::over_step`,
       `log_mean`); a step whose `q` crosses the level (the drive carries `q`,
       `StressDrive::Constant::q`) accrues to the crossing, at the fraction
       of the step exponential `q` spends above it, and bleeds the rest at
       the idle end's recovery (read at the level, a charging 555's R1
       accrued through every such step and never bled: cold 323 tpl 6 burnt
       at 0.78 s for 1.02 s, 178 tpl 6 5 % early); where neither end
       accumulates it bleeds at the mean recovery. That mean is exact for `q
       = A e^(kt)` and over-counts `q = (a + b e^(−t/τ))^2` (`ab ≥ 0`) by at
       most `exp((h/τ)^2/16) − 1`; the backward rule under-counts a falling
       rate by about `h/2τ_q` and burnt a 555 astable's timing resistors 5-17
       % late in exact mode, where default mode's closed form met the hand
       integral (cold 104, 282, 7, 570 tpl 6:
       `a_stepped_resistors_stress_is_the_mean_of_its_steps_start_and_end_states`),
       and a clock-driven LED's drain spikes a whole clock fall late (cold
       414 tpl 0: 7.77 per fall for 9.6 by hand). The first step after a
       re-seed has no start state of its own and keeps its end drive. The
       start state is the partition's own nodes at the step's start and its
       outside reads (a CC rail among them) as the step before ended
       (`Partition::end_reads`): read as published at the commit, a rail
       other partitions had moved since set 0.26 V across a 0.98 ohm fuse
       (cold 474 tpl 5 tripped a fuse that by hand never carries 4.2 mA:
       `a_stepped_steps_start_state_reads_its_outside_nodes_where_the_step_before_ended`),
       and the trial's own `u0` is the rail it was planned from, a step
       behind.
       The step a stepped stressed device (its ratio at least half its zero
       level) lets its partition take is held to a ratio change of `e^0.5`
       (ruling R90 (a), `STRESS_STEP_RATIO`): from the step before, its log
       rate held (`stress_reach`, a horizon on the next step, never the
       error controller), and on the step itself, a trial over it planned
       again (R68's `SteppedSolver::replan`) under a reach cut in proportion
       with a margin, at most four times (`stress_replan`), and not while a
       rail IC is held. Without it a CV tail falling 16-fold in one step at
       50 ms host steps latched cold 535 tpl 2's LED at 19.45 ms, where by
       hand it never latches
       (`a_stepped_leds_cv_tail_at_long_host_steps_is_stepped_finely_enough_not_to_latch`,
       `a_stress_replan_leaves_the_state_a_plan_at_its_reach_would_have`,
       `a_stress_replan_stops_at_its_cap_on_a_ratio_that_keeps_falling`).
       Over the step only: an idle visit (R38
       (a)) writes nothing and leaves `t0` behind, so up to the start of
       the step the visit closes each integrator on the drive it had; and
       after a re-solve at `now` (an event's, a promotion's) what is
       published is no step's end, and the drive starts at `now` (defect B,
       cold 668/1: a clock edge put 11.2 V across a rated 84.7 ohm idle on
       a settled 8.6 uF since 9.3 ms, and 47/s held back to then burnt it
       at the edge, S = 7, where its discharge gives 0.016). Its latch
       (stress or PTC trip, ruling R71 (b)) is scheduled from the drive the
       step ended with, so it fires where the integral crosses inside the
       step that follows, the device's stepped partitions first brought to
       it with the stamps they had; a latch only the step's own end drive
       reaches fires at that commit. When the predicted latch comes, the
       step then running decides: its trial's own mean drive crosses before
       then, or the latch waits for that step (cold 535 tpl 2: an LED's CC
       rate held forward past the CC exit latched it at 15.0 ms where by hand
       its damage tops out at 22.8 of 24). At the step's end with the end rate,
       cold 39 tpl 4's R20 latched 2.1 ms late and took R17 with it, 74
       ms before the fine reference's R17. Each committed step (a catch-up's
       and a clamp's included) closes the integrators at its own end
       voltages, its CC rail and the boundary it was solved against
       (`SteppedSolver::trail`), before a re-seed or a republish at the moved
       rail replaces what is published (ACC-A: cold 507 tpl 4's R7-6 latched
       at 0.239 s against 0.4006 s by hand; cold 55 tpl 2's PTC tripped at the
       host's step, 2.0 ms at 1 ms hosts against 1.135 ms by hand, now 1.24
       ms in default mode and 1.135 in exact mode).
   - **Re-anchoring** (rulings R21, R29, R30, 2026-10-21). A device whose
     voltage a moving rail drives takes, at each visit, a drive anchored at
     the visit, and is visited again before what that drive drops can move
     it by more than its tolerance.
     - **Closed-form, linear segment (R29).** The device's own voltage `V =
       Σ ± pin` (each pin a node of the partition, the rail, or a fixed
       value; `ClosedFormSolver::driven_device`). With the rail's motion
       taken as its tangent at the anchor (`du = du_n + S_n δ`, the
       capacitor's forcing `F_n + W_n δ`), `V = V∞ + L·δ + (V_n − V∞)·e^(fδ)`
       exactly, with `L = A·S_n + D·β`, `β = −W_n/f`.
       - A device that tracks the rail (a resistor from the rail into the
         capacitor on a linear ramp) has `L = 0`: its voltage is one
         exponential, the lag `S·tau` in its asymptote, and only the rail's
         curvature re-anchors it.
       - Otherwise the asymptote is `V∞ − L/f`, which leaves `V_n` on `V`'s
         own slope.
       - Bound: `|L|·(δ − (1 − e^(fδ))/|f|) + |A|·K_s·δ²/2 + |D|·K_w·min(δ³/6,
         δ²/(2|f|))`, where `K_s = Σ|s|·k·e^(−kτn)` and `K_w = Σ|w|·k·e^(−kτn)`
         bound the rail's curvature.
       - Tolerance: `DEVICE_TOL_V + DEVICE_TOL_REL·|V_n|` (`1e-8` V and
         `1e-4`, a tenth of W0's relative part). It is on the device's own
         voltage, so a millivolt-scale lag is held to `1e-4` of itself.
     - **Closed-form, curved or folded segment (R21).** An IC on the rail,
       or a cell's coupled path. Each pin is anchored as an exponential
       through its exact value: the segment's own asymptote, or the rail's
       exact value `u_a + du(τn)` for a pin on the rail.
       - Tolerance: half of `DRIVE_TOL_V + DRIVE_TOL_REL·|v|` per pin
         (`1e-5` V and `1e-4`).
       - Deadline: `(L(τn + δ) + |D(τn)|/tau)·δ` (`closed_form::CurvedBound`);
         for the rail pin, `tol / S`.
     - **Static and fixed-pair (R30).** Over its pins, `V = c + σ·u`, each
       pin being the rail, a static hub node's line `a + s·u`
       (`StaticPartitionSolver::hub_node_line`), or a fixed value.
       - The CC rail capacitor's segment without coupled modes is one
         exponential, `u = u0 + slope·φ_k(t − t0)`. With every line exact (a
         linear hub partition), `V` is that exponential: exact, never
         re-anchored, and the same however the host chunks its advances.
       - It is held as a ramp (`ExpVoltage::ramp`): `V = V0 + b·expm1(−s/τ)`
         from its exact start `V0 = c + σ·u0`, `b = −σ·slope/k`. On a lightly
         loaded rail the asymptote `u0 + slope/k` is `iLimit·R` volts (5e9 V
         for 10 GΩ on a 0.5 A PSU), far past where CV ends the segment, and
         the `a + b·e^(−s/τ)` form cancelled: a latch 14x early at 10 GΩ, a
         temperature below ambient (W1gV3). The closed forms read a ramp from
         `V0` and its slope: the stress gain from the piece's start, `(kV_p²
         − c)d − 2kV_pW·τ·g1(d/τ) + kW²·τ·g2(d/τ)` with `g1`, `g2` the
         integrals of `expm1` and its square (series below 1); the level
         crossing by `ln_1p((level − V0)/b)`; the temperature as a series in
         `u/τ` over the one-pole's moments of `(s/u)^n` while `u/τ ≤ 0.5`
         (past it `a` and `b` are the size of `V`). Every term is then the
         size of the quantity it adds to, at any resistance: latches within
         1e-9 from 1 MΩ to 1 TΩ, temperatures within 1e-7 of the rise from
         1 kΩ to 100 TΩ.
       - Otherwise (coupled modes, `k = 0`, a curved hub partition) `V` is
         held at `c + σ·u(t_a)` until `tol / (|σ|·S)`, `S = Σ|s_k(t_a)|`.
       - Without a segment the rail moves only at its own events, which
         publish, and nothing changes.
       - A new segment (`RailState::cap_seg`, compared at every settle)
         re-drives every static and fixed-pair device the rail reaches.
     - **Anchors are taken at each visit.** Nothing is kept between
       visits: every visit anchors afresh at its own instant. A
       closed-form device is visited only at a re-aim, its own event or its
       anchor, so a visit rarely falls inside an earlier anchor's span.
       - A host's chunk end resyncs the coupled path, which is a new
         segment. So on the CC coupled path, re-anchored devices still
         differ between chunkings within their tolerance.
     - **Quiet devices.** A re-anchor is scheduled only after a visit that
       wrote a drive, so a device whose drive writes nothing (under its
       rating) is never re-anchored. It is visited again when something it
       reads publishes.
     - **The deadline** is never less than `REDRIVE_MIN_DT = 2·NO_PROGRESS_DT`
       after the visit.
     - **Counts.**
       - A 1 µs RC that tracks a rail ramping at 100 V/s re-anchors a
         handful of times in 20 ms.
       - A device off the rail (`L ≠ 0`) re-anchors about 1,500 times a
         second while its voltage grows.
     - **The event.** It is a `Slow` event whose token carries
       `events::SLOW_ANCHOR` (the top bit). It re-drives the device and
       fires nothing (`PartitionSolver::handle_slow_anchor`). A device's own
       events never carry the bit, and `slow_is_current` compares the rest.
       It rolls back with the heap and the slow journal, as other slow
       events do.
     - **The guard.** An instant whose only events were re-anchors moved no
       node by design, and the no-progress guard counts it neither way.
   - **Cold devices** (ruling R38 (a), 2026-10-22). A rated resistor, a
     fuse or a plain `led`, with no thermal identity, whose stress cannot
     grow at any voltage the circuit can put across it is cold: its next
     check is never. No publish notes it, no visit (forced or not) touches
     it, and it schedules nothing. Its integrator stays at 0 under the idle
     drive, which is what each visit would have left it at, so its record
     is that of visiting it: it never latches, cold or visited. Every other
     output is bit for bit that of visiting it on the SAP-1 and on every
     cold circuit of G2e's fuzz (31,486 circuits; on the 8,074 with a cold
     device that both runs finished, every voltage at every frame matched
     the visited twin's), but this is measured,
     not promised: a visit can split an interval where a skipped one does
     not, so a re-anchored segment may round differently (W1gV4 saw 2 of
     2,160 current-limited fuzz circuits differ by 1e-13 V before such
     circuits lost their bound).
     - **The most it can see** (`settle::voltage_span_bound`, fixed at load:
       no source has a live param). Walking the propagation forest from
       node 0, each fixed node's (and the rail's) range is its parent's plus
       its source's: a value (an ideal PSU's too); a clock's or pulse's two
       levels; a section anchor's 0; a battery's `[0, max E]` over its OCV
       table. Every free node stays within `[lo, hi]` of those (the maximum
       principle): every other element is downhill (resistances, switch
       contacts, an IC output's `R_out` to its own `vcc` or `gnd`, the 555's,
       the shunts to node 0, junctions, a floating pull to its IC's `vcc` or
       `gnd` potential), so the highest free node cannot rise past every
       source by itself, and a capacitor with its other pin on a fixed node
       that never moves discharges from there. So: no bound for a kind not
       in that list, or for a capacitor between two free nodes or from a
       free node to a moving one (a charge pump). The voltage across any
       device is then at most `hi − lo`.
     - **No bound with a current-limited PSU** (W1gV4 finding 1, G2e). Its
       rail is a solved node, not a held one: a supply that cannot sink lets
       another source lift it past `V`, and a wrong rail solve has published
       it far outside `[0, V]` (209.5 V on an 11.29 V supply once a shorting
       switch opened beside a 555; 9.901 V on a 5 V one back-driven through
       10 Ω in exact mode), where a cold load missed the latch its visited
       twin took. A bound read off a solve is only as sound as the solve, so
       a program holding one has no cold device. The SAP-1 (ideal or
       battery) holds none.
     - **No bound under the trapezoidal rule** (W1gV4 finding 2, G2e).
       `INTEGRATION` 1 steps every stepped partition by the trapezoidal
       rule, which has no maximum principle: past a stiff edge it rings, and
       a capacitor charged through an LED lands volts past its supply and
       stays there while it discharges (3.63 V over 12.31 V for 0.3 s on
       fuzz seed 860; 15.8 V on 12 V in the test). Backward Euler, the
       closed forms and the static solves keep every free node inside. A
       program with a stepped partition under `INTEGRATION` 1 has no bound
       anywhere, since a node it overshoots can be an IC's supply that
       another partition's outputs follow.
     - **A carried start** (W2c, ruling R57 D10 (a)). On a cold load every
       capacitor starts at 0 V and every node inside the span (W1b point
       5), which the maximum principle needs: it bounds where a node can
       go, not where it starts. A carried capacitor can start outside the
       new program's span (a source lowered under a charged one): its free
       node sits off its fixed one by its `vc` until it discharges. So the
       span and `|v|` bound are both widened by the sum of the carried
       charges' `|vc|` (`carry::abs_vc_sum`): 12 V carried onto a 5 V
       circuit makes a resistor rated past 5 V's power hot. A carried
       battery SoC does not move the battery's range, which is taken over
       the whole OCV table.
     - **Series chains** (`settle::resistance_volts_bound`). A resistance
       joined to others through free nodes nothing else touches (two pins
       each) carries the chain's one current, less the node shunts' leak:
       at most `R_d (span/R_total + m·NODE_RSHUNT_G·max(|lo|, |hi|))`.
     - **An LED** (`settle::led_never_stressed`) never carries more than
       `ifMax` forward when its chain's resistances could not hold it: past
       `ifMax` its junction is past its safe voltage (LEDs' fast path) and
       the resistances hold at least `R_others (ifMax − m·G·max(|lo|,
       |hi|))`, together more than the span. Its reverse current is at most
       `Is`, which the safe voltage holds to `ifMax`.
     - **The test** (`settle::never_stressed`): the device's own rate, in
       its constant and its closed-form (`k V² − c`) form, both monotone in
       `|V|`, is not positive at that bound, or an LED's chain test holds,
       with the span widened by 1 % plus 10 mV (`COLD_MARGIN_REL`,
       `COLD_MARGIN_V`: W0's tolerance and the backward-Euler controller's
       error let a published voltage stray past the exact one). The margin
       is a tolerance on a sound bound, never cover for an overshoot that
       grows with the circuit; those have no bound (above).
     - **Hot devices behave as before.** A finite earliest latch time is not
       a next check: the stress a device could gain between visits is not
       known, so honouring it would delay a latch. A thermal identity keeps
       a device hot (its readout reports the interval's power, to `1e-9`
       W), and so do PTCs and the other LED kinds. One addition for hot
       devices: a closed-form segment whose voltage, monotone, stays under
       the rate's level from the visit on is idle like a constant drive
       (`accumulates_after`), so the visit stores nothing, unless it is a
       rail-driven anchor (R21, R29), whose exponential holds only until its
       re-anchor.
     - **The SAP-1**: all 115 rated resistors are cold (220 Ω at 5 V is 114
       mW of 250; the 555's 1 mΩ timing resistor carries its share of a 1
       kΩ chain, microvolts), and so are its 83 LEDs (each behind 220 Ω:
       20 mA would need 1.8 V + 4.4 V).
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
     changing the voltages. So each device carries a `law_gen: u32`, bumped
     by `mark_device_changed` at every re-arm, latch and reset (point 12),
     and the memo's key includes it: a re-armed law never reuses a stale
     memo. `law_gen` rolls back with its instant, so a generation number can
     be reused after a rollback for a different law; the memo is a cache,
     not instant state, so `rollback_instant` clears it whenever the window
     bumped any `law_gen`. The law parameters `(vtn, Is, vSat)` are a pure
     function of `T_hold` for identified LEDs (recomputed, not stored, after
     a rollback restores `T_hold`); every other junction keeps its
     load-time values.

5. **Stress integrators and analytic threshold events** (D16, D18;
   §9.5.1). Classic's dwell integrator (SE:5510-5531): `S' = rate` when
   `rate > 0`, else `S' = −recovery`, floored at 0; it latches at `S >=
   threshold`, and once latched it does nothing. The per-kind rate, recovery
   and threshold are in points 6-8; Rust computes each rate in classic's
   operation order.
   - **Constant rate** (static, fixed-pair, and within a stepped step): `S(t)
     = S0 + rate·(t − t0)`. With `rate > 0` the latch time `t_x = t0 +
     (threshold − S0)/rate` is scheduled as a slow event. A bleeding
     integrator schedules nothing; at 0 it stays at 0 (the floor), and its
     zero time is computable but never an event.
   - **Closed-form segments** (resistors and fuses): `rate(s) = max(0,
     P(s)/pMax − 1)`, or `(I(s)/iRating)² − 1` for a fuse, with `P(s)` the
     squared exponential of point 3. The rate is `k·V(s)² − c` (resistor `k
     = rPower/(R²·pMax)`, `c = 1`; fuse `k = 1/(rNormal·iRating)²`, `c = 1`;
     PTC `k = 1/R²`, `c = iHold²`), positive exactly where `|V(s)| > √(c/k)`.
     `V(s)` is monotone over a segment, so `{rate > 0}` is at most two
     disjoint intervals (`V > L` and `V < −L`), and `S(s)` has a closed form
     on each. `t_x` is found
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
     in direction `dir` latches. The event only marks the window due; the
     decision is taken in `settle`, after the instant's delta cycle (point
     14), against the state then: it latches only if the pin is still a
     target commanded in `dir` and its predicate holds. A commit at the same
     instant that flips the commanded direction restarts the window
     instead, as classic's count restarts on the flip at that step, so no
     zero-length latch reaches the record or the latch log. So after a flip the latch is unpublished
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
        - 100 pF (model crossings 8.0 ns HIGH, 28.6 ns LOW): classic and
          the core both latch the 13 LOW edges and no HIGH one (G4's
          re-measure on classic with K0's `fix/output-sag-capacitive-edge`;
          probe C, before it, also saw classic latch the power-on HIGH edge
          under BE);
        - 30 pF (LOW crossing 8.6 ns): both latch none (G4, after K0; probe
          C saw classic latch 13 LOW edges, its 10 ns BE step leaving the
          node at 1.6 V against the exact 0.59 V). No divergence is
          registered. Fixtures 62 and 63 pin both counts.
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
    - In a closed-form segment `I` is monotone, so the trip condition flips
      at most once per segment, at a root scheduled like a threshold. The
      cool test `|V| < level` can begin and end in one segment (a `V`
      sweeping through 0), so it is a window `Cool{from, until}`: recovery
      needs 2.0 s inside it, and the window ending resets `Rt`.
    - A trip or a recovery is a stamp change (point 12), never a failure or
      warning record.
    - Recovery starts at the trip instant itself when the tripped current is
      already cool. Probe F `ptc`: 1 A through `iHold` 0.5 A trips at 0.1 s
      (classic 0.1018). At 0.5 V the tripped current is 5 mA, which is
      cool, and it recovers 2.0 s after the trip, not after the later drop.
    - Readout: channel 2 and `bo_engine_device_state`.

11. **Warnings** (§9.7). One Rust producer exists: `rail_ic_power_chatter`
    (code 3), a chip sliding along its power floor (W1c point 1a, the
    warning; ruling R92). The facade latches channel 1's records into its
    ledger at each `getWarnings()` and `takeNewWarnings()`, as classic's
    `_latchWarning` does. `transistor_overcurrent` is W3, and `missing_flyback` is a static
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
      SE:2439). Both report `lastConverged = false`. It is reachable only
      when un-failing a part makes the circuit unsolvable, and no circuit
      is known to make both engines' re-solves fail (ruling D7 (b), F5): a
      reset only closes paths (a failed part comes back), and since F5 the
      core holds a self-starving chip off as classic's H8 does (W1c point
      1a), so G4's candidate, a restored shunt pulling a resistor-fed
      74LS00's supply onto its floor, converges on both. The registered
      divergence `w1g-reset-failures-failed-seed` and its detector were
      deleted as unreachable (Equivalence harness rules, the failed reset).
      The rollback is pinned by the core's own tests through a seeded
      failure: `a_reset_whose_re_solve_fails_leaves_the_engine_exactly_as_it_was`
      (`solve/slow_tests.rs`, a fuse closing a zero-delay ring, `-7`) and,
      for the facade, `CoreEngine.resetFailures when the re-solve fails
      (-4)` (`test/core/engine-failures.test.ts`).

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
      `output_sag` state, each identified junction's `T_hold` (its law
      follows from it) and `law_gen`, the PTC state, the latch order and the
      latch log. The junction memo is cleared instead (point 4).
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
    expiry, thermal boundary, PTC trip and recovery) and R21 re-anchors
    (point 2), current tokens only;
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
         into a sagging load, where the sets differ only inside the delay;
       - an enable into a sagging load (ruling R35 (a)): the core's latch at
         the enable's read plus `tPZH` plus `OUTPUT_SAG_WINDOW`, classic's
         later latch recorded.

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
       - `engine-warnings`' `missing_flyback` cases are W3 (W1g round 2
         ruling): every one switches its winding with a W3 transistor, so
         none joins with W1h's inductor; `transistor_overcurrent` is W3.

### W1g cost

Ruling R22 (a), 2026-10-21: W1g lands under a TEMPORARY exception of +11 %
of CPU on the ideal SAP-1 running board over `3ca4004` (measured quietly at
G2b round 3: running +11.0 %, frozen +9.3 %; W1f's `fb96acb` was +4.8 % and
+3.7 %), against the +10 % ceiling. Perf round 3 must bring it back under
+10 %, and nothing after W1g may widen it.

Ruling R38 (a), 2026-10-22: those figures came from `.prog` files compiled
before G3's rating records. Every resistor carries one, 115 on the SAP-1
(265 devices to 380), and the slow physics they switch on cost `306987c`
some 23 % frozen and 26 % running on the ideal board, 16 % and 18 % on the
battery one (the table; W1gV3 measured +21.4/+28.9 and +19.5/+23.9). Cold
devices (point 2) take it back: every rated resistor and every LED on the
SAP-1 is cold, so it runs as the unrated board does, with no LED visit
either. R38's target, inside +11 % on both boards, frozen and running, is
met: +9.3 % and +9.4 % ideal, −38.6 % and −35.8 % battery.

The rated run below (G2d, 2026-10-22; 9 interleaved reps, 1-minute load
median 3.5, 1.8 to 18): CPU ms per simulated second, medians, and in
brackets the median of the per-rep ratios to `3ca4004`.

| SAP-1 | `3ca4004` | `fb96acb` | `306987c` | G2d |
|---|---|---|---|---|
| ideal, frozen | 10.14 | 10.65 (+4.4 %) | 12.35 (+23.4 %) | 11.55 (+9.3 %) |
| ideal, running | 12.87 | 13.21 (+3.0 %) | 16.29 (+26.1 %) | 14.08 (+9.4 %) |
| battery, frozen | 2107 | 1147 (−45.9 %) | 2446 (+15.8 %) | 1333 (−38.6 %) |
| battery, running | 2354 | 1313 (−43.3 %) | 2825 (+18.1 %) | 1512 (−35.8 %) |

Cold LEDs take 2 to 5 points of it: with cold resistors alone the same
protocol measured +13.9 % frozen and +11.2 % running on the ideal board.

The gate is the rated cost run, `pnpm run bench:sap1-cost -- --gate
<label>=<tree> ...` (`scripts/perf/sap1-cost.ts`): each tree loads the
generator's boards through its own compiler, as a host does, so the program
timed carries the rating records; a tree whose compiler knows the record but
emits none is refused. Ideal and battery, frozen and running, interleaved
reps (9 by default), each tree in a fresh process, 1 s measured, the
1-minute load logged; a change is the median of each rep's ratio to the
baseline's run beside it. A cost in this README is that run's, never a
native `.prog`'s.

Ruling R43 (b), 2026-10-26: R22's ceiling is judged at **steady state**,
30 s of warm-up on every side (`--gate`, which is `--warmup=30`), 9
interleaved reps, the paired median. At the earlier 5 s warm-up the wasm is
still tiering up: F5a's build read 1.5-2 % slower than its head at 5 s and
0.7 % at 30 s, with a CPU profile 0.2 % apart and no hot spot. The **5 s
start-up** figure is always reported beside the gate, since start-up is what
a user feels opening a circuit: the same process measures 1 s after 5 s of
warm-up (`--startup`, default 5) on its way to 30 s, so both figures come
from one load. F5's landing proposes a start-up budget with its `3ca4004`
baseline. Every figure above was taken at 5 s of warm-up, the protocol of
its time; the next gate run takes both.

### Known follow-ups (W1g)

Recorded at the W1g integration. None changes a rule above until it lands.

- **What F5 leaves** (W1f's follow-up, widened; rulings R23 (iii), R28 (a),
  R34 (a), D5 (b), R46 (a), R47 (a)). F5 ported classic's H8 (W1c point
  1a), took the CC seed's rounding (Rails point 4, the CC seed) and the
  power-edge `-4`s (FXB's `k6 s39` and FXDv's edge board among them). Its
  exit gate is no `-4` on the fuzz sweep (G2bv's generators, 9,000 runs)
  except the stepped-Newton cases, which get their own diagnosis. Left:
  - the 555 astable on a CC PSU (cold fuzz tpl 6; rulings D5 (b), R49):
    when the output's load pulls the CC rail below half the setpoint, no
    latch state is consistent while the capacitor sits between 2/3 of the
    CC rail and 1/3 of the setpoint (high, the threshold resets it; low,
    the rail leaves CC for the setpoint and the trigger sets it). Both modes
    now report the zero-delay loop as `-7` by the latch's own evidence
    (NE555 point 11; R49 (d), F5g) and the board falls back to classic: of
    tpl 6's 1,500 hub-2 boards, exact mode's 23 `-4` and 6 `-7` became 1
    and 28 (the `-4` left is seed 95's load-time stepped Newton), default
    mode's 38 `-7` are unchanged, and tpl 4 and tpl 0 moved in neither
    mode. Known divergence `w1f-ne555-cc-latch-band` (Equivalence harness
    rules) until R49 (a), a bench PSU output capacitance, gives the rail a
    state to hold;
  - named exceptions to F5's exit gate (ruling R50 (C), F5g): five
    default-mode rail-Newton `-4`s on the cold fuzz's tpl 4, each a
    `bench_psu` with a rail capacitor. Since then (F5m's tree): 542, 889 and
    49 run their 1.2 s; 722 declines at 1.34 ms through the hold-episode
    guard (W1c point 1a, ruling R54 (a): its compliance 2-cycle below holds
    and frees the 74LS00 every few microseconds, 51 short holds); 985 is
    still the rail Newton's `-4` at 0.47 s. As ruled:
    - seeds 542, 722, 889 and 985: `rails::next_regime`'s CV -> CC
      compliance candidate (Rails point 5), the CC rail solved without its
      capacitor, 2-cycles on a direct chip's power (a 555 with DIS on its
      own rail in 542, 889, 985; a 74LS00 in 722), each iterate judging it
      afresh (G 0.006 against 0.156 S on 542). F5b's hold probes the chip,
      but held off it returns the PSU to CV, so no draw condition is kept
      (ruling R44 (a) applies on a CC segment only), and the CV rail,
      already past the release level inside the window, frees it within
      the same loop;
    - seed 49: R45 (a)'s check re-solves the CC rail capacitor's rail from
      its charge `q`, not from the accepted rail (held powered, 1.2e-11 V
      above the 74LS00's 4.275 V edge), and the free Newton 2-cycles across
      the edge.

    Re-measuring them is R49 (a)'s first gate (a bench PSU output
    capacitance). Any left then go to A2 (a candidate that fails enters CC)
    and B (R45's check from the accepted rail), whose prototypes are in
    devolt's `memory/progress/brownout-core-w1e-drafts/f5g-logs/`;
  - named exceptions to F5's exit gate (ruling R55 2): cold seeds 24 tpl
    0 and 248 tpl 5, chips relaxing at their power edge (24: 40 us held,
    11 us powered; 248: 49 and 29 us) under the sweep's 7 ms host step,
    each release their own supply's, so ruling R54 (a)'s guard counts every
    hold and declines `-4` at the 51st: they fall back to classic until R51
    (c)'s sliding hold. Seed 24 also fails under the fine reference (exact
    mode, 1 us host steps) at 0.208 s;
  - and cold seed 688 tpl 5 (ruling R59 (2)) until R51 (c): a {8.64 V,
    0.107 A} PSU with no rail capacitor and a 555 on the rail, whose CC
    ramp reaches the 555's 1.8 V floor at 11.8 ms. The CC step aimed at its
    own rail lands at 1.78 V, the next crosses 1.80 V, and the chip's draw
    pulls the rail back under its floor: it slides there in 51 holds of
    10-40 ns and declines `-4` at 11.909 ms, as it did at da167d2, before
    R59's promotion: the decline came with da167d2's aim of a crossing
    inside a CC step at the step's own rail, before which a crossing
    misaimed at the published rail let one 2 ms step jump the floor
    unheld (R59 (2)). Exact mode
    at the sweep's host runs it with no hold; the fine reference (exact
    mode, 1 us host steps) fails Newton at 11.791 ms, at the same edge;
  - and cold seed 482 tpl 0 (ruling R64 (3)) until R51 (c): its gate on a
    CC rail with no rail capacitor slides along its power floor once the
    stepped hub partitions re-solve at each move of the CC rail (R59 (1)
    (B)), 51 short holds, `-4` at 0.104 s (at da167d2 the same guard at
    0.068 s);
  - and cold seeds 271 tpl 4 and 597 tpl 5 (ruling R65 (a)) until R51
    (c): R63 (2)'s pace trigger promotes each to the stepped path, as it
    should (597/5 at 6.27 ms, where the closed form reads 3.7 W0 off the
    fine reference and the stepped path 0.145), and the stepped path then
    meets a 74LS00's power floor. On 271/4 the CC rail Newton fails: the
    chip's edge verdict reads off, on, on across the iterates. 597/5
    slides along the floor in 47 holds, 1.36 s. The fine reference (exact
    mode, 1 us host steps) fails Newton on both, at 175 ms (271/4) and
    35.3 ms (597/5);
  - named exceptions to F5's exit gate (ruling R54, F5m): two default-mode
    declines on the cold fuzz that fail under the fine reference too (exact
    mode, 1 us host steps), each since f2d93ae:
    - seed 12 tpl 4: `-7` at 23.4 ms. A 555 (vcc 144 ohm off a CC rail,
      DIS on the rail itself, TRIG and THR fed from a clock's node through
      resistors), held off from 14.7 ms, is freed by the clock's edge at
      23.4 ms, and that instant's delta cycle does not settle within
      `DELTA_CAP` (a livelock, `-7`);
    - seed 654 tpl 3: stepped `-4` at `H_MIN` at 0.135 s. A 74LS00 whose
      ground pin is a 2.8 mF capacitor's node (a stepped partition holding
      its free supply), held off at load and freed at 6.6 ms; at 0.135 s
      that partition's step does not converge at 10 ns, and a stepped
      partition's Newton runs no hold procedure (W1c point 1a).
  - named exceptions to F5's exit gate (ruling R52 (a), F5j): four
    default-mode rail-loop `-4`s on the cold fuzz with no IC flip, each
    the same at c5b1a13. A CV <-> CC 2-cycle: the partition's tangent,
    taken at one regime's rail, is carried to the other regime's decision
    across a strongly nonlinear draw:
    - seed 479 tpl 2: the tangent at the setpoint puts the CC rail at
      1.06 V; re-solved there it reads the draw at the setpoint as 11.8 mA,
      under the 35.2 mA limit, so CV, where the exact draw is 76.6 mA;
    - seed 679 tpl 1: an LED directly on the rail; CC from the tangent at
      2.11 V finds a spurious root at 137 V, past the setpoint, where the
      555 is over-voltage;
    - seed 78 tpl 4: inrush into uncharged capacitors at load, the tangent
      17 A, CC at 2.97 V, then CV;
    - seed 509 tpl 3: a CC rail-capacitor tangent 2-cycle (1.7996 <->
      1.8863 V) about a kink in the draw (exact mode `-4` too).

    A bracketed CC root on draws re-solved at each iterate (a rail-loop
    safeguard on the failing path) is perf round 3's (ruling R52 (b)), as
    are exact mode's CV over the limit on seed 479 (CV at 5.152 V drawing
    76.6 mA for at least 60 ms, reported ok; that round's first fix) and
    the rail loop's two-back test, which compares bits and so misses a
    2-cycle whose CC-capacitor charge moves by ulps;
  - **the CC drift crawl** (F5m, F5V2 finding 3), fixed by promotion
    (rulings R55 (A2), R59 (1) (B), R60 (3) and (4), R63 (2), R64, R65;
    F5n-F5p, F5r). A PSU in CC with no rail capacitor whose rail a
    closed-form hub partition's capacitor carries through a small
    resistance (742 tpl 3: 5.8 uF through a 0.45 ohm PTC fuse) ramped by
    `RAIL_DRIFT_V` (10 uV) a `Rail` event, each re-aiming the closed forms
    (Rails point 4's drift cap): 2.2M events and 13 s for 50 us of ramp.
    Such a partition (`PartitionBuild::cc_switch`: closed-form, on a
    bench PSU's rail with no rail capacitor, outside exact mode) loads
    closed-form and is also compiled stepped, the idle copy parked
    (nothing steps, publishes or schedules it, its events stale). In CC it
    steps with the rail as an unknown (`stepped::CcRail`) once its rail
    crawls, and is a closed form again once the PSU is back in CV
    (`hub::sync_cc_kinds`, after each rail solve). Promoted at load, a
    closed form that never sees CC lost its exact crossings and CC boards
    whose rail barely moves ran up to 100x slower (F5o, R59 prototype), so
    it promotes lazily, at the next rail solve after either trigger:
    `CC_CRAWL_EVENTS` (16) `Rail` events within one closed-form time
    constant (the slowest, `-1 / F`; R60 (3), K measured over 59 boards
    from 0 to 1024), or the rail's pace: a move at a new instant over the
    interval since the last, times that time constant, past W0 (R63 (2);
    cold 72 tpl 4, whose rail another partition moves at its commits a
    millisecond apart with no `Rail` event between, read 95 W0 off the
    fine reference closed-form and is now within W0 of the always-stepped
    run). Each switch hands the charge across at `now` (the stepped copy
    starts from the closed form's `vc(now)`, the closed form resumes from
    the stepped copy's), the kinds and the trigger's state are journaled
    with the window and carried in the snapshot. Hysteresis within an
    instant: a partition promoted at an instant stays stepped through it
    (a CC exit there is the stepped copy's charge balance against the
    closed form's linearised draw; handed back and forth, cold 794 tpl 3
    switched 51 times at 13.04 us and the rail loop gave up), and the
    next rail solve still in CV demotes it. A regime change at an
    internal event brings every stepped hub partition short of it there
    on the rail it was on, before the re-solves (and a demotion) take the
    new one: leaving CC, the CC rail its pending step was solved on
    (`SteppedSolver::catch_up_on_cc_rail`, F5r); entering CC, the rail
    before the solve, as a boundary, each partition so brought there
    restarting at the entry as all do at an exit (ruling R66 (1); its
    trial spent, cold 574 tpl 5's `Step` event committed an empty one).
    Brought there after
    the rail moved, a partition had stepped its whole uncommitted gap on
    the new rail (F5pV: 3 ms of a capacitor settled at 2 V charged at 5
    V, 1346 W0; at the entry, 41.3 W0 under FORCE_STEPPED, both as at
    da167d2). The same holds with no regime change when the CC rail steps
    at an event that changed what it sees (a source edge, a re-solve of
    dirty partitions, `rail_jump`): every stepped hub partition short of
    the event is brought to it on the CC rail its pending trial was
    planned on and restarts there, and a dirty one re-solved before the
    rail loop is brought there the same way (defect D: one load put on a
    CC rail, a 10 uF settled at 3.5 V stepped its 1.9 ms gap on the new
    rail at the next host stop, 1.46 V, 566 W0, in both modes; two edges
    at one instant swapping two loads, 647 W0 in default mode). On a rail
    capacitor a host's live edit moves the CC segment rather than the
    rail, and a stepped hub partition already at the edit's instant plans
    its trial again on the new segment, as at a CC entry (ruling R89 (a):
    cold 2575 tpl 2's trials, planned at a host stop before a switch freed
    the rail from a source at -2.776 V, survived the edit, and the next
    stop's clamp stepped the rail 0.22 ms with its 555's power judged at
    the old rail, to 749.6 V and an 82M-event chase). A
    partition's own step, a `Rail` event and the horizon clamp move the
    rail as the draws drift, which each step's coupling bound covers, and
    keep the re-aim (FXEFv). A promoted partition's CC steps hold
    their error `CC_STEP_TOL_SCALE` (8) times under the controller's
    tolerance (R60 (4): G2bv's bc_ic 97 hub 2 crept 1.48 W0 off the fine
    reference at the sweep's host stops, exact mode 1.50 there, the host
    cadence's floor; 0.70 W0 at 8 times, while 16 slid cold 482 tpl 0 to
    the hold guard); a partition stepped from load keeps the controller's
    own. The cold 1-800 hub 2 sweep fixed 44 crawls (742 runs its 1.2 s
    in 0.08 s, within W0 of the fine reference through the ramp), and 230
    tpl 2 and 527 tpl 5, which declined under the R59 prototype, run:
    230/2's CC seed judged a held 74LS00 at a rail where its supply
    difference reads 0, and `SupplyCurve::derivs` now gives a constant
    piece's slope as 0 there, not 0 / 0; lazy promotion keeps 527/5
    closed-form, where stepping from the CC entry declines at 10.4 ms
    (R64). R64 and R65 name what it leaves: cold 482 tpl 0, 271 tpl 4 and
    597 tpl 5, exit-gate exceptions above. A coupled
    closed-form path for this case (A1) stays perf round 3's exact
    upgrade. Two boards F5m filed beside it are other mechanisms: 304 tpl
    4's 555 drives its own trigger and oscillates at 6.1 us (a ring at
    its own speed, ruling R46's kind; since F5n a crossing inside a CC
    step is refined against the step's own rail, 26 -> 20 steps a period),
    and 503 tpl 3's stepped hub partition rides a CC rail capacitor's
    segment with its draw frozen (R42's shape, ruling R47): each 9 uV
    band pass restarts it at `H_MIN`, and keeping its step instead (a
    prototype) left the rail 947 W0 off the fine reference, so the
    restarts are what keep it coupled until R42 (b);
  - **cold 34 tpl 3 at the stepped solver's floor** (ruling R66 (2), R39):
    at 0.4185 s a switch lets its CC rail climb from 0 to the 12.98 V
    setpoint in 11 us inside one host advance, and the clamp's steps after
    the exit cut are solved in CV (`CcStep::cv`, cold 75 tpl 3's fix) at
    the controller's backward-Euler tolerance, up to 1.7 ms on node 2's 28
    ms decay: 0.4598 V at 0.452 s against the fine reference's 0.4522, up
    to 13.5 W0 worse than da167d2 to 0.54 s. Exact mode at the same host
    stops reads 0.45973 there and agrees within 0.07 W0 at every instant
    to 1.2 s; da167d2 began each clamp step in CC again and crawled the
    clamp in nanosecond steps (0.59 s against 0.007 s), exact by accident.
    Narrowing it is a tighter stepped tolerance, not this case's;
  - **sliding at a power edge** (ruling R51 (c), R53): G2bv's bc_ic 426
    hub 1 (a cell's rail climbing through a 74LS00's 4.275 V floor, from
    1.14 s) and fz_ic 6 hub 2 (a CC rail held at the same floor for its
    first 70 ms) chatter there: each instant powers the chip for a pass
    and settles, on and off in turn, a rail-loop re-solve restarting the
    stepped hub partitions at `H_MIN` at each flip (0.10x and 0.13x real
    time). Exact mode slides through both at 1.3x and 14x real time.
    Holding the chip off at the flip (a trigger prototyped in F5n) left
    426 190 W0 off exact mode, which keeps the chip partly powered on
    average, and the powered side chattered on: R53's Duty(x) hold is
    their fix. Cold 617 tpl 5 is a relaxation of the same kind (its 555's
    output on its own rail collapses it, about 400 holds a second) whose
    falls at 1 kV/s re-anchor its two rated rail resistors every 172 ns
    (ruling R30's tolerance), 7M slow events a second of fall;
  - **the hold cadence on a CC rail capacitor** (ruling R70, the R44
    class): cold 482 tpl 4's 74LS00, whose 1Y and 4Y drive a clock's node
    from a CC rail over 588.6 uF, is held off at its 4.275 V floor and freed
    only at R44 (a)'s draw threshold or a host step's retry, so the
    capacitors carry the rail past the floor with the chip off: default
    mode reads 4.29 V and 4.67 V at 0.9896 s and 1.2 s where the hand
    integration with the chip powered inside its window gives 4.1714 and
    4.2516 V (exact mode within W0 of it). R53's duty hold, letting the
    chip slide at its edge, is the fix; the hand oracle is the ignored
    `cold_482_tpl4_default_mode_follows_the_hand_rail` (exact mode's,
    `cold_482_tpl4_exact_mode_follows_the_hand_rail`, passes);
  - **a 555 held off on its own CC rail capacitor's ramp** (rulings G with
    R68, R91 (a); R89 (a)'s finding): cold 2575 tpl 2's 555 has VCC, TRIG,
    THR and DIS on a {4.984 V, 106.6 mA} PSU's rail over 11.9 nF; powered
    at its 1.8 V edge it loads the rail past the limit, and off nothing
    holds the rail under the edge, so no power state holds. Default mode
    holds it off at the edge once per episode (each switch toggle starts
    one) and the rail goes on to CV at 4.984 V, held off there by
    convention; exact mode declines at 30.0846 ms through R54 (a)'s guard,
    and classic stalls at 1.81 V (a decline with a diagnostic on its
    `fix/rail-ic-power-chatter`). R53's sliding hold settles the physics
    (`a_cc_rail_freed_by_a_switch_holds_its_555_off_at_the_power_edge_once_per_toggle`).
    Cold 3593 tpl 3 is of the same kind: a 555 on a {5.24 V, 54 mA} PSU's
    rail that chatters at its edge from 0.98 ms, where the fine exact run
    declines (R54 (a)'s guard) and classic stalls at 0.97 ms; default mode
    runs its 1.2 s, and R89 (a)'s re-plan moves it by up to 4 W0 after its
    switch toggle at 0.687 s, with no reference to judge either;
  - rail excursions on the cold fuzz (1-800, hub 2, 5,600 runs; F5m's
    tree): 9 runs publish a rail more than 0.1 mV outside their sources'
    range, all 9 as at 1a84f72 and 8 the same size as at c5b1a13
    (seeds 34/3, 45/5, 226/4, 245/5, 259/5, 570/3, 594/4, 781/4 exact);
    the ninth, 654 tpl 5 (a `-4` at c5b1a13), runs with its rail 2.9 V
    under ground, the bench PSU that cannot sink (its fine reference -2.96
    V; the PSU sink model is R40's backlog);
  - the waveform-coupled CC rail (R42): known divergence
    `w1f-cc-rail-waveform-coupling` (Equivalence harness rules), its gate in
    perf round 3 right after the integrator change (ruling R47 (a));
  - **seed 226's LED path** (ruling R47 (a)), its own diagnosis, outside
    F5: W1gV4's cold fuzz seed 226 tpl 4 (a `{8.99 V, 91 mA}` PSU in CC with
    a 0.16 µF rail capacitor of esr 0.14 Ω, a -2.0 to 7.3 V pulse on node 3,
    an LED from that node to the rail, 0.92 mF from node 5 to it, 18 Ω and
    341 Ω from the rail to node 5) runs 19 W0 off its fine reference at 5
    ms and 720-860 W0 at 27-33 ms in default mode (the rail at 5.12 V
    against 2.78 V at 28.2 ms), and R42's prototype does not move it. The
    suspect is the LED carrying the pulse's edges onto the CC rail. Pinned
    by the ignored test
    `seed_226_default_mode_stays_within_w0_of_a_fine_reference`
    (`solve/rail_tests.rs`), which fails today and is the target;
  - **a gate that rings at its own delay** (ruling R46 (a)): a gate wired
    to its own input oscillates at its propagation delay, which is the
    physics, and the core takes every edge as an event, so such a board
    runs slower than real time: G2bv's bc_ic seed 97 rings at 81 ns and
    runs at 0.03x, fz_ic seed 401 at 27 ns and 0.02x
    (`a_gate_tied_to_its_own_input_is_held_at_load_then_rings_at_its_own_delay`).
    A recorded performance limit, not a crawl: its instants lie a delay
    apart, far past NP2's `NO_PROGRESS_DT`. A steady-ring fast-forward (a
    periodic orbit recognised and advanced whole) is a perf round 3
    candidate;
  - of three of the cold fuzz's partition-Newton `-4`s in F5D's sample
    that F5 left, 1120 tpl 2 and 1325 tpl 0 pass since F5e (W1c point 3's
    second try); seed 790 tpl 0's `-7` at 40.3 µs is a zero-delay loop
    through its 555 (DIS on TRIG with no timing capacitor, THR on a rail
    above 2/3 of its supply: DIS low sets the latch, released it resets),
    the `-7` of NE555 point 11, which the F5 core is the first to reach:
    c5b1a13's partition-Newton `-4` stopped it at 32.5 µs. Forced to exact
    mode the board stops earlier, at the first reset (31.3 µs), with a
    stepped-Newton `-4`, its 555's supply 1.82 V there (fed through 786 Ω)
    against the 1.8 V power edge: the stepped-Newton class F5 leaves to its
    own diagnosis, not the latch loop;
  - the failed-reset fixture (W1g Q6, ruling R34 (a)) has no circuit, and
    `w1g-reset-failures-failed-seed` is deleted as unreachable (ruling D7
    (b); Equivalence harness rules, the failed reset).
- **Battery boards with a stepped hub partition** (ruling R26 (a); W1f's
  "Stepped hub partitions on a rail they move", item 1): still slower than
  real time. A follow-up in the perf round 3 window, after F5, with the
  cell's modes; options go to the coordinator. The CC step's static Norton
  row was rejected (967x W0 off a fine reference).
- **Stepped junction currents** (ruling R27 (a)): 10x to 22x W0 off a
  small-step reference (the step controller's 1 mV against an LED's
  exponential; pre-existing; exact mode 16x). Current-aware error control
  in perf round 3; numbers to the coordinator before a commit
  that costs the SAP-1 noticeably.
- **Multi-capacitor stepped hub partitions** (ruling R31 (a); W1gV2):
  step-control error of 14x to 21x W0 in thermal readings and 17x in
  volts. Folded into R27's step control in perf round 3, after a
  rail-shortcut diagnosis that comes first and is fixed as a W1f bug as
  soon as it is diagnosed.
- **Two stepped partitions on one CC rail** (rulings R32 (a), R37 (a), R39
  (1); W1f's "Coupled stepped partitions on one CC rail"): a known
  divergence at its measured worst. The coupling step control (Rails point
  4, the CC step) took the worst node off a fine reference on FXEFv's `cc`
  boards (power LEDs, 40 each) from 39.0x and 43.1x W0 to 6.5x and 9.0x
  (tpl 10, two branches; tpl 12, three), all faster than real time; tpl 9
  (two branches, one switched) stays at 30.6x on seed 34, the
  backward-Euler controller's own error on a single branch (R27/R31's
  class: one LED and capacitor branch alone is up to 6.2x off, 28 of 40
  boards past W0). `two_stepped_partitions_on_one_cc_rail_keep_their_steps`
  bounds its board at 3.5x W0 of exact mode (measured 3.44x, 5.1x before);
  `stepped_partitions_sharing_a_cc_rail_bound_the_rail_move_across_each_step`
  holds seed 21's two- and three-branch boards within 1 W0 of exact mode's
  own distance from the fine reference (1.10x and 3.20x against 7.12x and
  5.26x; 27.8x before). Perf round 3 owns what is left: exact mode's
  CC-transition outliers, second-order stepping and joint stepping.
- **The CC flat band's rail-event rate** (ruling R33 (a); W1f's "CC band
  events from a closed-form draw", option (a)): the curvature band in CC,
  with the rail's own `δI / G` in its budget, in perf round 3.
- **Rounding floors** (R23, FXE): the stepped and closed-form hub bands
  carry no rounding floor; linear (direct) hub solves are not gated for
  refinement; and the floor's overstatement drives the refinement's cost
  on extreme boards (`k8 s1017`: 1.21 s against 0.69 s).
- **Chunking on the CC coupled path** (G2b round 6; point 2,
  Re-anchoring): a host's chunk end resyncs the coupled path, a new
  segment, so re-anchored devices there differ between chunkings, within
  their tolerance: 9e-8 relative for a rated RC, and 2e-5 to 6.7e-5
  relative (0.07 of W0) for the held fallback, static devices beside
  coupled RCs or behind a curved hub partition (W1gV3).
  Making them identical needs W1f's clamp resync to keep the segment's
  identity across a chunk end.
- **Cold devices and W2's carry-forward** (W1gV4, G2e): the span bound
  (point 2, "Cold devices") assumes every node starts inside it. A load
  that carries capacitor voltages forward must re-derive the bound over the
  carried starts or load every device hot.
- **Cold devices on current-limited rails** (W1gV4, G2e): none until a
  bound is proved that holds even when the rail solve is wrong; FXH fixes
  the rail solves themselves.

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
   `t_end`. A superseded event at the head of the heap (a stale token or
   version) is dropped between instants, as its handler would drop it
   inside one, so it is no instant (W1f, Rails point 4's superseded
   events); stat 4 still counts it. Stepped partitions land exactly on
   `t_end` (a horizon clamp), so a readout at `t_end` is a solved value,
   not an interpolation. The readers whose nodes the clamp moved are
   evaluated in a delta cycle at `t_end`, so a digital readout at `t_end`
   reflects them, as if a step had landed there.
   Interpolated acquisition is W2.
6. **Load settling** (owner decision 8, and the W1-0 spike). Three steps, at
   `START_TIME`:
   1. Settle the network to a fixed point with every register's edge history
      FROZEN: parts drive their current state but never act on an edge or
      update `lastClk` or `lastWE`. Level-sensitive asynchronous inputs (a
      74hc74's `/pre` and `/clr`, a 161's ENT to RCO, a 173's M/N) change the
      outputs, as classic's seed stamp does, but never the register state;
      the edge pass does the state writes. Delays do not apply while settling; this
      is finding the consistent initial state, not simulating time. A pass
      whose commits or resets land only on drivers on fixed nodes still
      re-solves, as a delta pass does, so the source currents and the
      solver's driver table take them. A 555's
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
  the electrical result (`netV`, `elementI`). Against fixed classic (0.6.2's
  PSU fixes) the two regimes agree except inside classic's 1 ppm entry
  tolerance and on a rail that lands on the setpoint at a step boundary
  (Rails point 5); there is no registered divergence for `reg`.
- **Source edges.** Clock and pulse edges are compared per cycle (decision
  6). Headless classic lagged an edge by up to one step (`1/(10 f)` for a
  clock, `per/10` for a pulse, HL:141-168) until `685179c`
  (`fix/headless-source-breakpoints`, 0.6.3): it now lands a step 10 ns
  before each source event and crosses it in a 10 ns guard step, as the
  browser worker does, so both classic hosts place an edge within one guard
  step. The core places it exactly.
- **The reader scan** (W1f Q12). Known divergence
  `w1f-rail-scan-late-flip`: a reader whose pin moves with the rail against
  a fixed threshold (an LS input on a divider off the rail) is re-evaluated
  at the next rail scan, so it can flip up to `RAIL_SCAN_V` of rail late
  (Rails point 3; the power window is exact, Rails point 4). The id names
  the divergence; it is not an entry of
  `test/core/equivalence/known-divergences.ts`, whose entries excuse
  classic-against-core mismatches inside a structural digital cone grown
  from seed components, which a late flip anywhere on a rail does not fit.
  It is measured, not assumed:
  - the Rust test
    `a_reader_of_a_static_hub_partition_flips_at_the_next_scan_within_the_scan_band`
    (`solve/rail_tests.rs`) pins the bound: a {5 V, 10 Ω} battery charges
    10 µF through 100 Ω, so its rail rises smoothly, and a Schmitt NAND
    supplied apart reads a divider off the rail. It flips when the rail has
    moved at most `RAIL_SCAN_V` past its exact crossing, plus the one rail
    move that crossed the band (`RAIL_DRIFT_V` on a smooth rail), never
    before the crossing, and stat 20 counts the flip once;
  - stat 20 counts every reader flip found at a rail scan. Listing the
    corpus and fixture circuits whose run ends with stat 20 above 0 in
    `scripts/core-equivalence.ts` is NOT implemented (W1f known
    follow-ups).
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

  **As implemented (F4), and OPEN.** A 555 on a limited `bench_psu`'s rail
  keeps its 1e-6 sidecar only when the core's regime readout is CV at the
  load and at 2,000 evenly spaced instants over the compared span, across
  every phase (`psuStaysCv`; a discharged rail capacitor enters CC at the
  load itself). A 555 in the moving-rail set gets no sidecar and is listed
  as unmodelled (for a declared sidecar, a fixture error). The refined
  classic run above is NOT implemented, so a 555 on a battery or on a PSU
  that enters CC (corpus 07 and 08 among them) has no period check yet, and
  S2's criterion 3 waits on it.
- **The power-gate ping-pong** (F1; manager rulings 2026-10-06 and
  2026-10-07; F5). An IC whose own draw sags a weak supply back across its
  power window has no consistent power state (repro: 4.279 V, r 5 Ω, a
  74LS00 driving 1 kΩ). Classic resolves it with H8 (`fix/power-off-stall`,
  0.6.3); the core ports H8 as W1c point 1a (F5): the solve that alternates
  probes the IC held powered and held unpowered and holds off one that
  starves itself, retrying it once per host step as classic does (ruling
  R51 (b)), and declines `-4` past 50 holds shorter than a host step in
  one sliding episode (the guard of rulings R51, R54 (a); W1c point 1a).
  Its forms:
  - **Within an instant** (no rail capacitor, or one whose esr steps the
    rail with the IC's draw): the rail loop's passes, the rail Newton's
    iterates or an iterated static partition's Newton iterates alternate.
    Since F5 these settle with the IC held off; they declined with `-4`
    before. Pins,
    each now asserting the hold and its release, in `solve/rail_tests.rs`:
    `an_ic_whose_own_draw_sags_the_rail_back_across_its_power_window_is_held_off`,
    `the_power_gate_pingpong_board_through_5_ohm_is_held_off_until_its_rail_carries_it`,
    and the fuzz boards that declined (G2bv's bc_ic seed 20 hub 1, the cold
    fuzz's seeds 237 tpl 3, 12 tpl 4, 785 tpl 0 and 830 tpl 3, FXB's k6 s39
    and FXDv's edge board):
    `a_g2bv_gate_starving_its_cell_at_load_is_held_off_and_again_after_a_toggle`,
    `a_rail_newton_alternating_a_555s_power_gate_holds_it_off_until_the_rail_passes_its_release`,
    `an_iterated_partition_alternating_a_555s_power_gate_holds_it_off`,
    `the_fed_gate_on_a_cc_ramp_is_held_at_its_floor_and_released_at_the_next_re_solve`,
    `the_fxh_boards_that_declined_at_load_load_with_their_gate_held_off`,
    `the_fxdv_edge_board_loads_held_off_between_its_floor_and_its_carrying_rail`,
    `a_gate_hovering_at_its_floor_on_a_cc_rail_is_held_once` (G2bv's bc_ic
    97) and
    `a_gate_tied_to_its_own_input_is_held_at_load_then_rings_at_its_own_delay`
    (fz_ic 401, which then rings at its own delay: Known follow-ups (W1g),
    ruling R46 (a)). A probe in which every alternating IC carries itself and
    every rail IC reads as probed is the instant's solution (the free
    passes alternated only on the way to it), provided its state passes the
    ordinary convergence check (ruling R45 (a); W1c point 1a):
    `a_probe_every_alternating_gate_carries_itself_in_is_accepted_when_its_state_passes_the_residual_check`
    and, through a seeded fault, `a_probe_whose_state_fails_the_residual_check_is_not_accepted`.
    F1r14's flip-against-the-ramp hold (Rails point 4) is replaced by it
    (ruling D4 (a)); its pins pass unchanged.
  - **Across instants** (HG1; F5, ruling R44 (a)). With a rail capacitor of
    esr 0 the rail is continuous and every instant settles: a flip turns the
    segment back, and the next power-window event lands the rail two
    reached-widths past the same edge the other way femtoseconds later (5.3
    and 9.4 fs apart, the rail within 8.6e-12 V of a 74LS00's 4.275 V floor
    on a {5 V, 20 mA} PSU ramping 1 µF, the gate driving an LED and 330 Ω).
    Two `Rail` events in a row, each flipping the rail ICs' gates back to
    where the one before flipped them from, run W1c point 1a's procedure at
    that instant; on a segment a chip that, powered, turns the segment down
    starves itself whatever its supply reads, and its release waits for the
    draw threshold (Rails point 4). On a rail its partitions move (a PSU in
    CC, a cell) a hub partition's own step or crossing moves the rail as a
    `Rail` event does and counts as one (F5m; a PSU in CV pins its rail, and
    there the count costs nothing): a stepped
    hub partition on a CC rail at a gate's floor flipped the gate at each
    10 ns step, the step starting powered ending under the floor and the
    next over it, and ran free between the holds its rail loops took (cold
    301 tpl 2: 49 holds and an 11.5 s first advance of 6.3 ms; 204 tpl 5,
    279 tpl 5, G2bv's fz_ic 6 hub 2 alike). A `Rail` event or hub step that
    flips nothing ends the count, except on a CC rail capacitor's segment,
    where up to 16 of the stepped hub partition's own steps that flip
    nothing, each starting from the gates the last flip left, keep it
    standing (ruling R82 (B')): the capacitor holds the rail through a flip,
    the partition restarts at `H_MIN`, and the steps that regrow from there
    carry the rail back across the edge (cold 757 tpl 0: a 555 in reset
    sinking 82.8 mA through a 71.8 mA supply chattered at its 1.8 V floor
    every 170 µs, 11 to 13 steps between flips and no hold, 6,805 steps in a
    45 ms host step; it then declined through the guard, and since ruling
    R95 (b) it slides, since open item (c) from its first hold, W1c point 1a; off a segment cold 108
    tpl 5's 74LS00 throws a capacitorless rail to 7.1 V and back at its
    floor, which exact mode runs through:
    `a_rail_ic_whose_draw_reaches_its_rail_through_a_capacitor_slides_at_its_floor`,
    `a_rail_ic_flipping_off_a_rail_capacitor_is_not_counted_across_its_steps`).
    A source edge ends it too, and so does another partition's event when
    the rail solves again in that call (a clock swinging a load across a
    floor every half period is no ping-pong); a commit does not (the gate's own outputs and a zero-delay reader of them
    answer the ping-pong), nor does the horizon clamp. The repro is held at the floor crossing,
    1.4421592 ms, and freed at 1.62047 ms when the PSU leaves CC, 472 events
    to 3 ms (it declined at 424 before F5). Pins:
    `a_cc_ramp_whose_ic_load_turns_the_rail_back_under_its_floor_holds_the_gate_off_until_the_supply_carries_it`,
    `a_zero_delay_reader_of_a_ping_ponging_gate_reads_it_held_off_then_powered`
    and G2bv's fz_ic seed 275,
    `the_hg1_fuzz_board_holds_its_gate_until_the_ramp_carries_it`
    and cold 301 tpl 2,
    `a_chip_flipped_at_each_step_of_the_hub_partition_it_draws_from_is_held`
    (`solve/rail_tests.rs`). `w1f-power-gate-pingpong` is deleted.
  - **Any other crawl** (NP1, redefined by NP2; ruling R6 (a), 2026-10-15:
    a frozen tab is never acceptable). Completed instants each within
    `NO_PROGRESS_DT` (1 ns) of the one before form a run, and inside a run
    each instant is judged by what it did to the state, not by its spacing.
    It made no progress if it moved no published node voltage, turned one
    back (moved it against that node's own last move in the run), or ran
    only `Rail` events and left the rail node where it was (a `Rail` event
    exists to move the rail; the node is `PartitionSolver::rail_node()`).
    A move is a change of more than `RAIL_MOVE_EPS` (1e-12 V), as it is for
    the rail (Rails point 3; FXF, after G2bv): a node the circuit holds
    still is published from terms at the board's scale and turns by their
    rounding, and on G2bv's seed 400 (a CC ramp through 2.62 µF into a
    0.26 Ω short, `Rail` events 0.08 ns apart) the short's node turned by
    1e-15 to 4e-15 V at most instants and declined the run at 1.16 µs. A
    cycle within the floor moves nothing, so it still counts every instant.
    An instant that applied a waveform source's edge (a clock's, a pulse's)
    progressed whatever it turned, and each node's direction starts afresh
    after it: the edges are the board's scheduled input, not the circuit
    chasing itself, and a clock has no frequency cap. Once `NO_PROGRESS_MAX`
    (10,000) instants of a run have made no progress, the next instant
    inside the run declines with the same `-4` before it runs, leaving the
    last completed instant's state. Instants that progress neither count
    nor reset the count, so a cycle cannot hide its turns between them;
    only an instant outside the window ends the run. A livelock repeats
    states, and a node that returns to a value must turn back on the way,
    so any cycle counts at least one instant a period, and a crawl that
    moves nothing counts every one. The count is kept across `advance`
    calls (a host stepping event by event makes one instant per call), is
    untouched by a failed instant and by the horizon clamp, and resets at a
    successful `set_params` or `seed_memory`. A counter with one floor,
    the rounding of a published value: past it the sign of each change is
    exact, and it changes nothing in a run it does not stop.

    Time alone cannot tell a livelock from fast dynamics, which is why NP1's
    rule (10,000 instants in a row each within 1 ps of the one before) was
    replaced: a 50 ns RC inrush off a 5 Ω cell lands some 28,000 rail
    events 0.7 ps apart, and NP1 declined each of F1r13's 36 such boards in
    its first microsecond (a false `-4`, against R6), while V11's B1070
    repeated a rail that did not move every 1.14 ps, just past NP1's
    window. 1 ns is a decade under the stepped solver's smallest step
    (`stepped::H_MIN`, 1e-8 s). The longest run of instants inside it
    measured on a legitimate board was 82,763 (the inrush); the most
    no-progress instants one run counted, 3 (NP2, 2026-10-16). Pins: the
    kernel's mock tests (`kernel.rs`:
    `a_fast_monotone_burst_runs_however_dense_and_a_cycle_of_any_length_declines`,
    which holds B1070's shape, and
    `a_crawl_of_femtosecond_instants_declines_with_the_guards_minus_4_and_keeps_the_last_instant`), and in
    `solve/rail_tests.rs`
    `a_fast_rc_inrush_off_a_5_ohm_cell_runs_to_the_end_on_a_coarse_host`,
    `a_gigahertz_clock_into_an_rc_and_gates_runs_for_any_number_of_edges`
    (1 and 10 GHz, 40,000 and 400,000 edges) and
    `a_cc_exit_that_pinned_the_rail_now_runs_on_a_coarse_host`: B1070
    itself runs to 250 ms on a 612 µs host and event by event, since the
    coupled CC path (Rails point 4) leaves it no stall.

- **Several closed-form partitions on a cell** (F1r15; ruling R17 (a)).
  Known divergence `w1f-cell-multi-partition`: on a battery hub whose rail
  feeds a curved closed-form partition and a second one, a published
  current can sit past W0's tolerance of the board's own ODE between rail
  solves, since the cell's coupled path takes the static and direct draws
  linearised at the solve while the rail moves up to `RAIL_DRIFT_V` (Rails
  point 4). Bounded as measured: 12 of the 600-board multi-partition cell
  probe over W0, worst 20x. The id names the divergence; it is not an
  entry of `test/core/equivalence/known-divergences.ts`, and the harness
  does not yet look for it on the corpus or the fixtures. It goes when the
  fix scheduled for perf round 3 lands (W1f known follow-ups).
- **The waveform-coupled CC rail** (R42, F5c; ruling R47 (a)). Known
  divergence `w1f-cc-rail-waveform-coupling`: a bench PSU in CC with a rail
  capacitor (0.16 to 0.9 µF), and a waveform edge reaching the rail
  through a dynamic hub partition (W1gV4's cold fuzz, hub 2: seeds 226 tpl
  4, 245, 1084, 1429 and 1359), runs off a fine reference (exact mode,
  horizon clamp at most 1 µs; ruling D6 (a)). With a rail capacitor the CC
  step's rail-as-unknown is off (`sync_stepped_cc`), so a stepped
  partition rides the frozen form with its draw linearised, and the rail's
  own fast mode (`R C_rail`, 0.5 to 3 µs) is integrated at whatever the host
  and the rail events sample. Worst node off the reference at a 0.1 ms
  host, in W0: 1429 5845, 1084 5.5e4, 226 819, 245 12, 1359 21; exact mode
  itself is host-dependent there (1084's sampled rail minimum -3.88, -1.47
  and -0.003 V at 0.1, 3 and 50 ms hosts). F5c's prototype extended the CC
  step to rail capacitors and promoted to stepped any closed-form hub
  partition holding a capacitor on the rail (it also needed the hub's
  running sums re-summed and the load's CC rail-capacitor charge in
  parity; kept out of the tree, in devolt's
  `memory/progress/brownout-core-w1e-drafts/f5c-r28-r42-prototype.patch`):
  1429 went to 15-25 W0 across 0.1 ms, 3 ms and 50 ms hosts (exact mode
  10-18), 1084 to 1.7e4 (exact 64), 226 stayed near 800, and 245 and 1359
  stayed at 12-25 W0 in every mode, the integrator's own floor. So R42's
  gate moves to perf round 3, judged right after the integrator change,
  against D6's reference (default mode within W0 of it). The id names the
  divergence; it is not an entry of
  `test/core/equivalence/known-divergences.ts`, and the harness does not
  look for it on the corpus or the fixtures. Seed 226 has a diagnosis of
  its own (Known follow-ups (W1g)).
- **The 555's latch band on a CC rail** (ruling R49 (d), F5g). Known
  divergence `w1f-ne555-cc-latch-band`: a 555 astable on a bench PSU whose
  OUT load drags the CC rail below half the setpoint has no consistent
  latch state while its capacitor sits between 2/3 of the dragged rail and
  1/3 of the setpoint (NE555 point 11; Known follow-ups (W1g)). Classic
  oscillates the latch at its step rate there; the core declines the
  instant with `-7` in every mode and the board falls back to classic, so
  no core reading of such a board is compared (cold fuzz tpl 6: 28 exact
  and 38 default boards of 1,500). The id names the divergence; it is not
  an entry of `test/core/equivalence/known-divergences.ts`, and the
  harness does not look for it on the corpus or the fixtures. It goes when
  R49 (a), a bench PSU output capacitance, lands after F5: the rail then
  has a state to hold through the latch's switch.
- **CC exits against the exact rail** (probe P9; the owner's ruling (a),
  2026-10-15). On a `psu-rail` fixture (a limited PSU, a rail capacitor and
  switched resistive loads), every sample from a CC exit to the next CC
  entry, the post-exit `esr·C` relaxation included, compares against the
  sidecar's exact value, not classic (an analytic sidecar, as the 555's
  timing network has; no new tolerance); the samples before the exit stay
  classic against core, and the core is held to the exact rail at every
  sample it replays, on the stimuli path too. Classic's backward Euler at
  the harness's step misses that 5 µs mode (P9 with `esr` 0.05, 5 µs after
  the exit: the core 0.03679 A, exactly `0.1 e^-1`; classic 0.09782 A). The
  sidecar holds the switch's 1 mΩ contact and the node shunt, as both
  engines do (F1r10; without them it read the core's correct rail 8.2e-5 V
  off); fixture 51 samples 0.6 to 50.6 µs after an exit, inside the window.
- **A resolved ramp against the exact trapezoid** (ruling R3 (b),
  2026-10-15). `pulse-ramp-1ms`'s RC is held to the sidecar's exact
  response to the trapezoid at W0's tolerance (the core is 4.4e-15 V from
  it). Classic is 2.6e-2 V off there, and Richardson on classic converges
  only at 2.5e-5 (F4), so classic does not gate those samples: the sidecar
  explains its mismatch (W1 plan decision 3).
- **Harness choices** (ruling R9, 2026-10-16: all four of F4's
  accepted). A Schmitt input's flip instants on a resolved ramp, and its
  output's, compare within `1e-9 s + 1e-9` relative of the sidecar's
  (borrowed from W1g's integrated latch times); an IC's power-gate flips on
  a sagging PSU rail are judged against the exact sidecar's crossings
  (classic's adaptive steps land them 39 to 148 µs late); P9's rule above
  applies on the stimuli path too; SoC's tolerance is the coulomb tolerance
  divided by the capacity. `optimization-equivalence` stays classic-only.
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
  compared. An `output_sag` latch is held to its analytic time too, read
  from the latch log (`OUTPUT_SAG_WINDOW` after the load, a reset, a power
  return or an enable's commit). Classic latching later than that (its
  two-step debounce on growing steps: 263 µs after a reset, fixture 56; 737
  µs after an enable commit, fixture 65) is a classic accuracy shortfall
  recorded in the fixture's `classicS`, never an excused divergence (ruling
  R35 (a)).
- **Failure sets** compare `(componentId, kind, pinId)` exactly, `value` and
  `limit` within decision 6's tolerance, and `message` against classic's
  template applied to the core's own numbers (W1g point 19). The tolerance
  is in the kind's unit: volts for `output_sag` and amps for `fuse_tripped`
  and `led_failed`, as above; watts for `resistor_overload`, `1e-9 W` plus
  `1e-3` relative (ruling R36 (a)), derived by analogy with W0's amps
  tolerance of `1 nA` plus `1e-3` relative. If a small-current case ever
  passes only because of the absolute floor, revisit it with the power-law
  bound `|dP| <= |V|·dI + |I|·dV`.
- **The failed reset** (W1g Q6; ruling D7 (b), F5). When the re-solve after
  `resetFailures()` fails, the core rolls the reset back, where classic
  keeps its failures cleared over a non-converged seed (W1g point 13). It
  was registered as `w1g-reset-failures-failed-seed`, with a detector that
  excused only the failure and warning sets after that call, never the
  convergence flag, waiting for a gate 2c fixture on which both engines
  fail the re-solve (ruling R34 (a)). F5d's bounded search found none: the
  cold fuzz's 42 partition-Newton `-4`s (F5D's sample) re-run on the F5
  core, 39 to the end and none at a reset (two `-4`s and one `-7` left, all
  on boards with a 555 on a fed supply, one at a switch toggle, which opens
  a path where a reset can only close one); 720 boards of G4's shape (a
  74LS00, a 555 or two 74LS00s on a resistor-fed node, a shunt failing at
  6 V and restored at 4.5 to 5.2 V, resistor or LED loads) and 56,020
  random boards of mixed families (74LS00, 74LS04, 74HC14 and 555 on one or
  two fed nodes, an LED on the supply, a shunt failing at 6-8 V and
  restored at 2.5-5.7 V; 20,596 of them with the shunt failed before the
  reset): no reset whose re-solve failed on either engine. One board failed
  on both before any reset (the core at load, classic in its run, both at
  6.5 V with the shunt in place), and driven so that the shunt was the
  part restored, both resets converged. So the id and its detector are deleted as unreachable; a
  mismatch after a failed reset now fails the gate, which is where a
  circuit that reaches it should surface.
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
- An exact CC exit needs the CV current back under the limit, not a voltage
  fraction below the setpoint, or the state machine re-enters at the exit
  instant forever (probe P7; classic had exactly that flicker until
  `fix/psu-cc-exit`).
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

## Open items (must close before 1.0.0)

- **Multi-hub rail charge closure** (W2b, "Charge"). Several stepped hub
  partitions on a CC rail (with no capacitor, or stepping its capacitors as
  their companion) each solve the rail against the others' draws held, so
  no one rail trajectory exists and a source's charge closes its KCL only to
  the steps' coupling error (measured 1.88 W0 on
  `two_stepped_hubs_on_a_cc_rail_with_no_capacitor_fold_its_lines_linear_between_its_solves`,
  0.52 W0 with a rail capacitor), flagged `kcl_approximate` in the take and
  counted in stat 28. Closing it needs the stepped hub partitions on such a
  rail solved together.

- **The step planned before the observer armed** (W2b, "Acquisition",
  W2bV 6a). It kept no points, so an iterated partition's nodes no
  capacitor's voltage gives read linear across it: measured 3.40 W0 (an
  LED's cathode at turn-on), counted on the node's fallbacks so a host sees
  it. Closing it needs the step's knots kept (or the step re-planned
  passively) whenever it may be armed into.

- **Relative-degree-2 slide aim** (zero rail velocity at the edge; R53,
  open item (c)'s successor). A slide's aim brings the rail back to its edge
  over the rail's settling, a backward Euler trial's end-of-span draw, but
  sets no condition on the rail's velocity there. Where the duty reaches
  the rail only through a capacitor (cold 757 tpl 0: the 555's sink on
  node 2, 4.94 mF behind a 0.64 ohm fuse), the rail runs on past W0 as the
  slide enters: 1.114 W0 at the test's stops (0.17746 s), 1.304, 1.363 and
  1.336 W0 at 1, 0.5 and 0.1 ms stops, within W0 from 0.19 s (1.0004 W0
  at 0.92 s at 10 ms stops). Re-solving
  more often (a band of 0.2 to 0.5 W0), a trapezoid aim and a span of the
  fast mode (0.9 ms) were measured and do not hold it. The same run-on
  takes node 2 past its own W0 (1.856 mV at 1.756 V) at single stops in
  the steady window after the rail is back within W0: against the hand
  1.75593 V over 0.2-0.33 s it peaks at 1.18 W0 (0.2125 s, 0.1 ms
  stops), 1.02 (1 ms), 0.95 (0.5 ms), its mean +0.42 to +0.59 W0.
  Centring alone does not fix it: the band's centring aim (a = -W0 / 2,
  R53-P9's option B) on this slide too takes node 2's mean to -0.03 to
  -0.06 W0 but its peaks to 1.09-1.21 W0, with 2 to 4 times the
  re-solves, 766 tpl 0 to 0.99984 W0 at 1 ms stops, and the duty 0.024
  off its hand value at 0.25 s; it is not taken. An aim at zero rail
  velocity at the edge, from node 2's trajectory in the trial, would.
  `a_rail_ic_whose_draw_reaches_its_rail_through_a_capacitor_slides_at_its_floor`
  is its expected failure, pinned at 1.6 W0 over all four schedules
  (`..._slides_within_the_relative_degree_2_band`).
- **BE stepped capacitor-current error control** (INT-ACC, rulings R93, R96).
  A backward Euler stepped partition's step-doubling bounds its node
  voltages to W0 per step but never its capacitors' series currents, a
  small decaying difference: fixture 38's supply partition (5 kohm into
  100 nF, holding a 74HC14's free supply) reads `cd` 218 W0 off the fine
  reference before INT-ACC and 323 W0 after it, its voltages 0.74 and 1.1
  W0 (R79's longer input steps no longer cut its steps through
  `sync_supplies`). Comparing the series current against classic's 10 nA in
  every backward Euler partition cost +64 % cold-sweep steps, and in the
  supply partitions alone +24 %, both with new wall caps. Fixture 38 was an
  expected failure of the equivalence gate linked to this item; with
  TR-BDF2 it passes again and the expectation is gone. A current check relative to the
  partition's own current scale was measured and dropped (+9.7 % cold-sweep
  steps and new crawls at k = 1). TR-BDF2 (W1b 6a) took fixture 38's current
  from 326 to 16 W0 and cold 360/3's gate ground pin from 9.6 to 0.4 W0 after
  each clock edge (R98 brought TR-BDF2 onto the PSU's rail). Left to do:
  (1) fixture 38's 16 W0 against W0 (the Rust test
  `a_backward_euler_supply_partition_keeps_its_capacitor_current_to_the_
  closed_form` keeps its 5 W0 bound as an expected failure); (2) a CC step
  (its rail capacitors, `CapStep`, and the coupling bound have no TR-BDF2
  form) and exact mode's cell partition (E and r held from the SoC at each
  step's start: under TR-BDF2 a 5 V, 0.9 ohm cell draining into 100 nF and
  100 ohm read its rail 3.8 W0 off a fine reference at 0.6 s, 1.5 under
  backward Euler), which still step by backward Euler.
- **Default-mode CC-rail accuracy** (ruling R99 (a)). Against an independent
  truth (exact mode at 0.25 and 0.125 us host steps, extrapolated;
  `CC_TRUTH`, int-acc-logs/truth/cc-truth.txt), default mode's readings on
  the four CC-rail boards sit past W0, worst default / FORCE_STEPPED: the cc
  entry with one capacitor 1.83 / 4.89 W0, with two 29.6 / 29.6, the cc exit
  0.93 / 2.34, a second load on the CC rail 3.10 / 8.62, two branches on one
  CC rail 1.10, three 3.18. Each case is an expected failure
  (`cc_*_within_w0_of_the_truth`) and is pinned no farther from the truth
  than at 8c5cde7. Cold 2575 tpl 2 (W2c's R101, 2026-11-10) is a fifth: against a
  fine reference (exact mode, trapezoid, the host stepping 1 us), its
  readings across 19-25 ms (from STOPS_2575[1], the host step to
  STOPS_2575[2]) sit up to 31.9 W0 off, and 190 W0 inside the swing at 19.23
  ms (the CC rail-edge capacitor, device 9, -2.6415 V against -2.6379 V at
  19.2365 ms). A host stopping at each instant and one yielding there read
  the same, bit for bit, so this is accuracy, not host cadence (D6).
  `zz_r101_2575_cadence_against_fine` (ignored) re-measures it.
  Three causes (R99 diagnosis, 2026-11-13):
  (1) CC steps take backward Euler (R96 left them so), and its error piles
  up past the per-step W0 control: the entries, the exit, the second load,
  and node 6 of 2575 tpl 2 decaying through C17 after each pulse ramp
  (`cold_2575_tpl2_after_each_ramp_within_w0_of_the_truth`, an expected
  failure, 24.8 W0 at 24.5 ms against the truth `TRUTH_2575`, pinned no
  farther than 0898801's 27.5). Left to do: TR-BDF2 on CC steps, the rail
  capacitors' `CapStep` with its BDF2 history, under a step rule matched to
  its order and a cost gate.
  (2) Several stepped hub partitions on one CC rail each solve the rail
  against the others' draws held (the branches): left to do, stepping them
  together with the rail as a shared unknown.
  (3) A pulse ramp moving a direct device's draw (2575 tpl 2's switch 14,
  1 mohm from the rail to the ramping source) was held where the rail last
  solved, by the rail capacitor's segment and by the stepped partitions'
  CC steps, and a `Rail` event, which published no ramp, read it stale.
  Fixed (R99 (3)): a `Rail` event publishes the running ramps at its
  instant, the segment follows the ramp's draw rate (`direct_ramp`) onto a
  line (`CapSeg::ramp`; not while a chip slides on the rail, whose duty
  holds it), a CC step reads the rest's draw at its own end
  (`CcStep::a_o_at`), the CV and CC watches move with it, the rail solves
  again at each ramp corner, and the coupling bound leaves the ramp's own
  rail motion out (`coupling_ratio`). Through both of 2575 tpl 2's ramps
  every reading is within W0 of the truth at every host cadence tried
  (`a_cc_rail_a_pulse_ramp_drives_reads_within_w0_of_the_truth_wherever_
  the_host_stops`, 0.83 W0 at worst; 684 W0 at 0898801), with R53's exit
  for a slide whose edge stops attracting (the falling ramp drags the 555's
  rail through its 1.8 V edge inside a host step). Not yet ramp-aware: a CC rail capacitor's
  coupled path (closed-form hub partitions on the rail, `modes`) and a
  static hub partition whose boundary a ramp moves, which hold their
  draws as before.
  Exact mode's esr-0 rail capacitor at a CC exit (corpus 47 at 1 ms
  host steps: its charge to 6 V at 6 ms in CC, the exit seed publishing
  about 1e8 A) is closed: a CC rail past its setpoint within the shortest
  step leaves CC there (Rails point 9, ruling R102 (a)).
- **Threshold-aware battery rail cadence** (rulings R105, R108). A cell's
  rail re-solves at every 10 uV of its coupled path's motion
  (`RAIL_DRIFT_V`), about 32,000 times a simulated second on the battery
  SAP-1 and most of its cost (1.0-1.4 s of CPU per simulated second
  natively). Re-solving instead on the path's own cadence (R105 C2: a time
  step `sqrt(2 RAIL_DRIFT_V / |rho_dot|)` for the drain line's change, 10 uV
  only with direct draws, every curved partition's band and the scan band as
  rail thresholds, the cell's charge integrated along the path) cut that
  cost by 51-58 % and kept the gate's tests within a tenth of W0's
  allowance, but on 700 battery cold boards (0.3 s, against exact mode at a
  1 us host step) it moved every 555 astable on a cell (template 6) far
  past today's error (74/6: 0.06 to 31,354 W0 on its voltages; 45/6's rail
  0.001 to 16.4 W0), plus five boards' currents (26/5 0.31 to 1.09 W0):
  the 555's thresholds ride the rail, and its timing capacitor's crossings
  were scheduled on the drain line against thresholds read at the last
  solve, so the oscillation slips in phase. Closing it needs the cadence to
  re-solve wherever a watched IC's rail-borne threshold could be crossed
  within it. Variants A, B, C and C2 and their numbers: drafts
  `battery-rail-cadence.md`; patches in int-acc-logs/r105.
- **Exact-mode coarse step accuracy** (ruling R102 (a), measured on
  wip/exact at 3cefe1d). Exact mode at the cold sweep's host stops (1-800,
  hub 2, its own exact-mode boards) against its own fine reference (exact
  mode at 1 us host steps, scored at the coarse stops): of the 276 boards
  R102 moved, 42 still read past W0 somewhere (79 did before it), worst
  first: 254/0 97,024, 467/4 5,640 (`-4`), 249/5 812, 289/2 650, 741/3 470,
  744/2 78, 450/1 65, 454/0 32, 447/5 13.5, 758/2 9.7, 88/2 7.1, 375/5 6.6,
  434/5 5.8, 782/3 5.4, 191/5 4.3, 175/5 3.3, 293/0 3.1, 616/5 2.9 (`-7`),
  93/0 2.9, 274/3 2.1, 493/0 2.0, 504/3 1.9, 500/5 1.9, 107/3 1.9, 639/4
  1.8, 224/0 1.8, 761/3 1.6, 679/5 1.5, 109/5 1.5, 24/3 1.4, 469/3 1.4,
  522/5 1.4, 376/3 1.4, 336/0 1.4, 676/4 1.4, 781/4 1.3, 498/5 1.2, 770/5
  1.2, 391/3 1.1, 238/3 1.1, 466/1 1.1, 342/3 1.05 W0 (the sweep and its
  scoring: drafts/exact-logs/r102b-cmp-x.final.txt, r102d-cmp-x-12.txt).
  The boards the R102 work looked at fall in three classes: (1) step
  placement on a node a clock or pulse swings (224/0, 1.2 to 1.8 W0 at
  different stops on both builds); (2) R53's no-slide domain, a chip
  chattering at its floor whose held-off phases a stop can land in (467/4,
  the PSU in CV at its setpoint there); (3) step error that was there
  before any cut, the same reading at the same stop on both builds (249/5,
  289/2 and 744/2 at the first stop, 254/0, 450/1), or lowered by R102 but
  not to W0 (741/3 582 to 470, 758/2 117 to 9.7, 88/2 70 to 7.1). The rest
  are not classified yet.

## Deferred past 1.0.0

- **The observer's and the charge's per-take floor** (W2b, ruling R107
  (a)). A host taking samples and charge once a frame pays a fixed cost per
  take and per instant (the take's buffer through the ABI, the facade's
  records, the next-time check), about +5 % on the rated SAP-1 at 10 kS/s
  whatever the rate. Batching takes across frames should come with the next
  host-API revision.

- A self-feedback chip at a starving edge (ruling R97 (b), W1c point 1a's
  chatter entry): it does not slide and declines `-4` through the episode
  guard; check against a longer fine reference whether its chatter has a
  usable average (cold 22 tpl 2, 688 tpl 5).
- A slide's duty moving in time within a step during a source ramp (ruling
  R92): the duty is constant between its solves, so where the board's draw
  drifts fast against the rail's own conductance the supply leaves W0 within
  a microsecond and the slide falls back for that host step. Cold 2575 tpl 2
  with switch 21 closed: each 1.11 ms edge of the source's pulse (-2.776 to
  10.62 V) drifts the draw through 105.7 ohm at about 114 A/s against g
  about 0.04 S, the segment's equilibrium moving at about 2.6 kV/s, so a
  constant duty holds W0 for about 0.8 us (re-solves came 0.87 us apart);
  following the edge would take about 1,200 re-solves (600 at the best
  constant-duty aim) against `SLIDE_FOLLOW_MAX`'s 100. A duty `x0 + x' t`
  from the measured drift, its stamps following `x(t)` inside the step,
  would follow it. Meanwhile re-entry could wait until the drift measured
  at the fallback has dropped below what one constant duty can hold, saving
  about 100 re-solves per ramp edge (each re-entry during an edge follows to
  the cap first).
- Hysteresis at the IC supply floor, a classic change too: the W1c fixed point
  returns -4 (fallback to classic) when a free supply sits near the power
  floor. Accepted for 1.0.0 (manager, 2026-09-29).
- LED plus capacitor transients: both engines sit up to about 10x the
  tolerance from the Richardson reference ("Make a Capacitor Fade an LED",
  which passes as `richardson-closer-than-classic`). Investigate step
  control and the LED Newton's accuracy after 1.0.0 (manager, 2026-09-29).
