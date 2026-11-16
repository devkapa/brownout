# W3 classic semantics: what the Rust core must mirror (analog kinds)

## About this document

**Purpose.** This document specifies what the classic brownout TypeScript
engine does for the 22 analog part kinds that the Rust/WASM core ports in wave
W3 (`devolt/memory/progress/brownout-core-w1e-drafts/plan-1.0.0.md` section
2.1: W3-D1, D2, T, O, C, F, R, X). It is the W3 counterpart of
`core/spec/w1-classic-semantics.md` and follows its shape: each kind has its
terminals and parameters, the equations as classic evaluates them with every
constant cited, its dynamics and state, Newton behaviour, regimes and events,
small-signal AC, diagnostics, the tests that pin it, and a numbered list of
questions. The readers are the W3 implementation workers and the W3 reviewers.

**Status.** All 22 kinds are written. Nothing was built or run for this
document except a handful of arithmetic checks (the numbers in sections 2.1
to 2.3, 12 and 10.1 were computed with a throwaway Node script, not by the
engine); every other fact was read from the code. A fact marked "inferred" was
read but not executed.

**Authority rule.** As in W1: the classic TS source is the authority. Every
fact is cited `file:line`. Plan decision 7 (plan-1.0.0.md section 5, "Reference
where classic is wrong or absent", recommendation (b), not yet ruled) applies
to everything below: where classic disagrees with the datasheet or with
physics, this document says so and gives both figures, but it does not fix
classic. The port mirrors classic unless a question below is ruled otherwise.
"Datasheet figure" entries are recalled from the named manufacturer's
datasheet and were not re-fetched; each is marked "recalled" and should be
checked before it is used to justify a change.

**Which tree.** The code read is brownout `wip/v1` at commit
`0ef1169a419ab1bc168830b8e9d8f06691367b53` (worktree branch `wip/sx3`, one new
file, this one). Line numbers refer to that tree. The six kinds the bundled
catalog does not carry (njfet, pjfet, scr, triac, opto_npn, analog_switch) are
catalogued only in devolt's `packages/catalog/parts.json` (devolt commit
`6d3c6074`); lines for them cite that file as `devolt:parts.json:LINE`.

**Conventions.**
- Paths are relative to the brownout repository root unless they start with
  `devolt/`. Abbreviations for files:
  `SE` = `src/sim/engine/sim-engine.ts`,
  `EL` = `src/sim/engine/elements.ts`,
  `NW` = `src/sim/engine/newton.ts`,
  `SC` = `src/sim/engine/devices/semiconductors.ts`,
  `AR` = `src/sim/engine/devices/amps-regulators.ts`,
  `MF` = `src/sim/engine/devices/model-families.ts`,
  `DR` = `src/sim/engine/devices/index.ts` (the registry),
  `AS` = `src/sim/adaptive-step.ts`,
  `TP` = `src/sim/thermal-physics.ts`,
  `CAT` = `src/parts/default-parts.json`,
  `W1` = `core/spec/w1-classic-semantics.md`,
  `PLAN` = plan-1.0.0.md, `DK` = dk-design.md (both under
  `devolt/memory/progress/brownout-core-w1e-drafts/`),
  `ACC` = `w3-regulator-acceptance-cases.md` in the same folder.
- "Tests" are under `test/sim/engine/` unless a path is given. Test titles are
  quoted only where the exact claim matters.
- `Vt` is the junction thermal voltage `K_B_OVER_Q * (T + 273.15)`; at the
  default 25 C it is 25.6926 mV (not the 25.85 mV some comments and default
  arguments use; section 1.1).
- Branch-current sign: for every kind that owns a branch row (op-amp units,
  linear_reg, lm317, dcdc_converter) `x[k]` is the current entering the
  branch-owning output node's stamp row with the voltage-source convention, so
  it is NEGATIVE while the device delivers current (sections 10.1, 13.2).
- "Open pin" means `ctx.isOpenPin` (W1 section 6.9): a pin on a single-
  component net with no explicit wire.
- Wave placement tags `[W3-X]` follow PLAN 2.1.
- Questions are `Q-W3-n`. Rulings already made are cited by their home:
  PLAN section 5 (decisions 1 to 16, none yet ruled as of PLAN's date, with a
  recommendation each) and DK section 5 (questions 1 to 11, with
  recommendations). Where a question below is answered by one of those, it
  cites it instead of re-asking.

### The 22 kinds

| Kind(s) | Model object | File:lines | Params read | Shared family | Catalog parts |
|---|---|---|---|---|---|
| diode, schottky_diode | `diodeModel` | SC:513-524 (body SC:349-465) | 4 (`vf`, `n`, `iRated`, `Is`) | with `led`, one body `stampShockleyJunction`; no failure hook | `diode-rectifier` CAT:269, `diode-schottky` CAT:367 |
| zener_diode | `zenerDiodeModel` | SC:137-210 | 5 (`vf`, `vz`, `n`, `iRated`, `iz_knee`) | breakdown law `stampBreakdownDiode` EL:886 (with tvs) | `diode-zener` CAT:300 |
| tvs_diode | `tvsDiodeModel` | SC:212-341 | 4 (`vbr`, `bidirectional`, `iRated`, `iz_knee`) | breakdown law (with zener) | `tvs-diode` CAT:334 |
| bjt_npn, bjt_pnp | `bjtModel` | SC:736-882 | 6 (`Is`, `betaF`, `betaR`, `nF`, `nR`, `earlyVoltage`) | Ebers-Moll core `bjtLinearization` EL:973 (shared with opto_npn) | `bjt-npn` CAT:971, `bjt-pnp` CAT:1014 |
| nmos, pmos | `mosfetModel` | SC:884-1040 | 6 (`vto`, `k`, `lambda`, `cgs`, `cgd`, `bodyDiodeVf`) | square law shared with JFET by copy (EL:1206, EL:1839) | `nmos` CAT:1057, `pmos` CAT:1098 |
| njfet, pjfet | `jfetModel` | MF:329-432 | 4 (`vto`, `idss`, `lambda`, `Is`) | square law like MOS; two gate diodes | none in CAT; `devolt:parts.json:2007, 2066` |
| scr | `scrModel` | MF:474-591 | 5 (`vgt`, `igt`, `ih`, `vtm`, `ron`) | latch pattern shared with triac, analog_switch | none in CAT; `devolt:parts.json:2113` |
| triac | `triacModel` | MF:593-686 | 5 (same) | thyristor latch, bidirectional | none in CAT; `devolt:parts.json:2164` |
| opto_npn | `optoNpnModel` | MF:719-883 | 5 (`ctr`, `vf`, `iRated`, `betaF`, `Is`) | LED junction + BJT core | none in CAT; `devolt:parts.json:8103` |
| analog_switch | `analogSwitchModel` | MF:1135-1190 | 2 (`ron`, `roff`) | committed latch like the thyristors | none in CAT; `devolt:parts.json:2215` |
| lm358, mcp6002 | `dualOpAmpModel` | AR:759-1017 | 6 (`openLoopGain`, `gbw`, `slewRate`, `sourceCurrentLimit`, `sinkCurrentLimit`, `outputResistance`) | op-amp unit machinery AR:66-300 | `lm358` CAT:6469, `mcp6002` CAT:6573 |
| lm386 | `lm386Model` | AR:1019-1174 | 6 (`gain` replaces `openLoopGain`; same five others) | same unit machinery | `lm386` CAT:6677 |
| lm393 | `lm393Model` | AR:1176-1275 | 0 (hysteresis is a literal) | open-collector output via `ctx.stampDigitalOutput` | `lm393` CAT:6781 |
| tl431 | `tl431Model` | SC:1042-1164 | 0 (all literals) | none (a smooth VCCS, not a regime device) | `tl431` CAT:7102 |
| linear_reg | `linearRegModel` | AR:1277-1408 | 4 (`vout`, `vdropout`, `iLimit`, `quiescentCurrent`) | regime machinery shared with lm317 and dcdc_converter | `reg-7805` CAT:6861, `reg-ams1117-33` CAT:6921, `reg-ams1117-50` CAT:6981 |
| lm317 | `lm317Model` | AR:1410-1530 | 4 (`vref`, `vdropout`, `iLimit`, `quiescentCurrent`) | as linear_reg | `lm317` CAT:7041 |
| dcdc_converter | `dcdcConverterModel` | AR:461-757 | 6 (`vout`, `eta`, `iLimit`, `vinMin`, `vinMax`, `vdropout`) | active-set clamps shared with regulators | `dcdc-buck-5v` CAT:7209 |

Registration (`DR`): zener, tvs, diode, led, bicolor, rgb, bjt, mosfet, tl431
in the SEMICONDUCTORS block (`DR:138-146`), dcdc_converter first and then dual
op-amp, lm386, lm393, linear_reg, lm317 in the AMPS block (`DR:151-156`), and
jfet, scr, triac, opto_npn, analog_switch in the MODEL-FAMILIES block at the
end (`DR:193-198`). Registration order does not decide stamp order: the stamp
list order is circuit component order (section 1.3).

### Cross-cutting findings

These are the findings that most change how W3 should be built. Each is
developed in the chapter named.

1. **The four "regime devices" are three different things.** The linear
   regulators, the dcdc_converter and the bench-PSU-style CC logic share one
   per-solve active-set mechanism (section 1.4) that is part of the *solve*,
   not a post-step event: an entry clamp, a compliance clamp and a headroom
   selection are re-decided at every Newton iterate and cleared per solve.
   The thyristors, the analog switch and the lm393 are the opposite: a
   committed latch decided after the accepted solve from that solve's
   voltages, so their output changes one accepted step late and classic never
   locates the instant. The op-amp is a third kind: a continuous BE pole with
   an active-set clamp on the target. W3-G should therefore be two mechanisms,
   not one (sections 1.4, 6.5, 10.5, 11.5, 13.5).
2. **The lm393 is a latch with a one-step lag, not a finite-gain linear
   element.** DK question 6 (op-amp form) asks SX-3 which it is: it is a latch
   (section 11). Its output stamp comes from the committed `out1`/`out2` and
   the decision uses the accepted solve's voltages, so the DK 1.5
   cross-partition driver-commit form is the natural port; the 1 mV
   hysteresis and the 1-step lag are classic behaviour that a driver commit
   with a datasheet delay would replace.
3. **TL431 is smooth, so it belongs with the kernels, not W3-G.** Classic
   models it as one exponential VCCS plus a 1 Mohm REF bias (section 12): no
   minimum cathode voltage, no latch, no regime. PLAN row W3-C lists it with
   the comparator and "needs W3-G"; it does not.
4. **The six unnamed kinds have no bundled catalog entry**, so every default
   the model reads is an in-code literal, and every catalog value in this
   document for them comes from devolt's catalog. The core's compile layer
   cannot get them from `default-parts.json` (section "The 22 kinds" above,
   sections 5 to 9).
5. **Many parameter defaults in code differ from the catalog's.** The BJT
   reads `Is` with fallback 1e-16 while the catalog says 1e-14; the MOSFET
   reads `lambda` with fallback 0 while the catalog says 0.02; `earlyVoltage`,
   `cgs`, `cgd` and the op-amp-style `vdropout` of the buck module go through
   `modelParam` (catalog default only with an exact `catalogUid`) while their
   neighbours do not. A saved circuit without those params behaves differently
   from a freshly placed part. The port's compiler must reproduce each read
   rule per parameter (section 1.2 table; Q-W3-1).
6. **Newton limiting exists only for BJT junctions (and the opto's internal
   BJT).** MOSFET, JFET, diodes and thyristor gates use no limiter; the
   MOSFET square law has kinks at `vOV = 0` and `vDS = vOV` and a reverse
   conduction branch that is not symmetric (section 4). DK question 10
   (classic's limiting where it has one, SPICE's otherwise) is therefore
   concretely: pnjlim for BJT and opto, nothing for the rest.
7. **Several classic behaviours are non-physical and worth deciding before the
   port copies them**: the TVS forward drop at its rated current is 1.43 V,
   not the 1.1 V its comment says (section 2.3); the MOSFET conducts in
   reverse through the triode formula with `vOV` measured from the *named*
   source (section 4); the regulators' acceptance cases and H7 (ACC, section
   13); the LM393 sink is not current-limited and the part has no delay
   (section 11); a thyristor's conducting law is a voltage source of `vtm`
   below `vtm` for one step (section 6).
8. **Op-amp AC is the continuous-time pole, the transient is backward Euler.**
   They agree only for `omega*h` small (section 10.6). A derived small-signal
   stamp (DK 1.4) from the discrete kernel would differ from classic's
   `acStamp`; a recorded override is needed, as DK 1.4 already says.
9. **The dcdc_converter AC stamp deliberately ignores its negative
   incremental input resistance** and the input is an AC open (section 14.6);
   a derived stamp would differ.
10. **Package thermal for these kinds is a one-pole lag read one step late**:
    shutdown of a regulator is decided from the committed temperature at the
    *next* stamp. Only seven exact catalog uids have a profile; nothing else
    in W3 heats (section 1.7, 10, 13).

---

## 1. Shared machinery

Everything in this chapter is used by more than one kind. Chapters 2 to 14
refer back to it.

### 1.1 Constants and helper functions

All in `SE` unless marked.

| Name | Value | Line |
|---|---|---|
| `K_B_OVER_Q` | 8.617333262145e-5 V/K | 125 |
| `NOMINAL_TEMP_C` | 25 | 126 |
| `DEFAULT_VF_TEMPCO` | -0.002 V/C | 127 |
| `N_DIODE` | 1.0 | 130 |
| `N_LED` | 2.0 | 131 |
| `IRATED_LED` | 0.02 A | 132 |
| `NODE_RSHUNT_G` | 1e-12 S on every node | 101 |
| `GATE_R_OUT` | 50 ohm | 1155 |
| `AC_GMIN` | 1e-12 S | SC:85 |
| `DC_OP_COMPANION_H` | 1e-12 s | 702 |
| `DC_OP_NEWTON_MAX_ITER` | 1000 | 736 |

- **Thermal voltage.** `thermalVoltage(T) = K_B_OVER_Q * max(1, T + 273.15)`
  (`SE:183-186`). At 25 C: 0.0256926 V. The default argument `Vt = 0.02585`
  in `stampDiodeShockley`, `shockleyIsFromVf`, `stampBJT`, `bjtCurrents`
  (`EL:700, 759, 1085, 1138`) is never used by a W3 handler: every handler
  passes `ctx.junctionVt()` or `thermalVoltage(...)`. Only `shockleyIsFromVf`'s
  and `stampBJT`'s defaults would matter to a caller that omitted them, and
  none does.
- **Ambient temperature.** `ctx.ambientTempC()` and `ctx.junctionVt()` are
  `circuit.environment.temperatureC ?? 25` and `thermalVoltage` of it
  (`SE:2269-2279`); `ctx.junctionVf(vf25)` is
  `max(0.05, vf25 - 0.002*(T - 25))` (`SE:189-196`). Only the plain `led` kind
  reads a self-heated junction temperature (W1 section 6.3, `SC:363-368`); the
  diode, schottky, zener, TVS, BJT, MOSFET, JFET, thyristor and opto kinds
  use the ambient only.
- **Saturation current with temperature**: `saturationCurrentAtTemperature(
  Is25, T, n)` (`SE:202-221`): `tNom = 298.15`, `t = max(1, T + 273.15)`,
  `n' = max(0.1, n)`, `exponent = clamp((1.11 / (n' * K_B_OVER_Q)) *
  (1/tNom - 1/t), -80, 80)`, result
  `clamp(Is25 * (t/tNom)^(3/n') * exp(exponent), 1e-30, 1)`. It equals `Is25`
  at 25 C exactly (clamped to [1e-30, 1]).
- **pnjlim critical voltage** `vCritFor(Is, VtN) = VtN * ln(VtN / (max(1e-30,
  Is) * sqrt(2)))` (`SE:178-180`).
- **Forward-voltage-anchored saturation current**
  `shockleyIsFromVf(Vf, iRated, n, Vt) = iRated * exp(-Vf / (n*Vt))`
  (`EL:755-762`). The anchor is "the junction passes `iRated` at `Vf`", valid
  only while `Vf <= vSat` (the tangent guard below); where `Vf > vSat` the
  anchor is not honoured (section 2.3, TVS).

### 1.2 Parameter resolution rules

A device handler reads parameters in one of four ways. The core's compiler
must reproduce each per parameter.

1. `Number(comp.params.X ?? literal)`: instance param, else an in-code
   literal. The catalog's `default_params` are NOT consulted. The host
   copies `default_params` into the instance at placement, so a freshly
   placed catalog part carries the catalog value, but a saved or hand-built
   circuit that lacks the key gets the literal.
2. `ctx.electricalSpecs(comp)?.X`: the part's `electrical_specs` through
   `partFor` (W1 section 1.2; `SE:1457-1488`). It resolves without a
   `catalogUid` when the kind has one candidate or the discriminating params
   pick one; an explicit but unknown or wrong-kind uid resolves to nothing.
3. `modelParam(comp, key, fallback)` (`SE:1491-1500`): the instance param if
   present; else, only when `catalogUid` names a part of this kind exactly,
   that part's `default_params[key]`; else the fallback.
4. `ctx.catalogPart(comp)?.uid`: a decision on exact catalog identity
   (the dcdc buck, `AR:321-325`).

Per-parameter rules for every W3 kind (D = literal default):

| Kind | Param | Rule | Default |
|---|---|---|---|
| diode, schottky | `vf` | 1 then 2 then D | 0.7 |
| | `n` | 1 | 1 |
| | `iRated` | 1 then 2 (`if_max`) then D | 0.1 |
| | `Is` | 1, no default (absent means derive from vf) | none |
| zener | `vf` | 1 then 2 then D | 0.7 |
| | `vz` | 1 | 5.1 |
| | `n` | 1 | 1 |
| | `iRated` | 1 then 2 (`if_max`) then D | 0.1 |
| | `iz_knee` | 1 (the catalog `electrical_specs.iz_knee` is not read) | 0.005 |
| tvs | `vbr` | 1 | 6.8 |
| | `bidirectional` | 1 (nonzero = true) | 0 |
| | `iRated` | 1 (specs not read) | 0.1 |
| | `iz_knee` | 1 | 0.005 |
| bjt | `Is`, `betaF`, `betaR`, `nF`, `nR` | 1 | 1e-16, 100, 1, 1, 1 |
| | `earlyVoltage` | 3 | 0 |
| mos | `vto`, `k`, `lambda` | 1 | 0.7, 0.02, 0 |
| | `cgs`, `cgd` | 3 | 0, 0 |
| | `bodyDiodeVf` | 1 | 0.7 |
| jfet | `vto`, `idss`, `lambda`, `Is` | 1 | -2, 0.01, 0, 1e-14 |
| scr, triac | `vgt`, `igt`, `ih`, `vtm`, `ron` | 1 | 0.7, 0.005, 0.005, 1.2, 0.1 |
| opto_npn | `ctr`, `vf`, `iRated`, `betaF`, `Is` | 1 | 1.0, 1.2, 0.02, 100, 1e-16 |
| analog_switch | `ron`, `roff` | 1 | 100, 1e9 |
| lm358, mcp6002, lm386 | six op-amp params | 1 with per-kind defaults | table in 10.1 |
| lm386 | `gain` | 1 | 20 |
| linear_reg | `vout`, `vdropout`, `iLimit` | 1 | 5.0, 2.0, 1.0 |
| | `quiescentCurrent` | 1 (explicit) else exact-uid table | 0 |
| lm317 | `vref`, `vdropout`, `iLimit` | 1 | 1.25, 2.0, 1.5 |
| dcdc_converter | `vout`, `iLimit`, `eta` | 1 | 5.0, 2.0, 0.85 |
| | `vinMin`, `vinMax` | 1 then 2 (`vcc_range`) then D | 7, 35 |
| | `vdropout` | 3 (only for uid `dcdc-buck-5v`) | 1.5 |

- **Pins.** The diode family, zener, tvs, BJT and MOSFET take pins BY
  POSITION: anode `pins[0]`, cathode `pins[1]` (`SC:140-144, 354-357`); BJT
  `[C, B, E]` (`SC:739, 746-748`); MOSFET `[D, G, S]` (`SC:887, 893-895`).
  The JFET, thyristors, opto, analog switch, TL431, regulators and dcdc take
  pins BY ID (`"d","g","s"`, `"a","k","g"`, `"mt1","mt2","g"`,
  `"led_a","led_k","c","e"`, `"a","b","ctrl"`, `"ref","anode","cathode"`,
  `"in","gnd"|"adj","out"`, `"in_pos","in_neg","out_pos","out_neg"`). The
  op-amps and lm393 take fixed catalog pin ids `"1"` to `"8"`.
- **`burnt` and `led_failed`.** A truthy `params.burnt`, or (diode, schottky,
  zener) a latched `led_failed` failure, turns the part into an open circuit
  with `elementI = 0`. The TVS reads `burnt` only. Nothing in the engine sets
  either for these kinds (W1 section 6.7, "The engine never writes `burnt`").
- **Open-pin gate.** Only the BJT checks `isOpenPin` (base) and returns
  without any stamp (`SC:741, 811-815`).
- **Uncatalogued kinds.** Six kinds exist in `COMPONENT_KINDS` and the
  registry with no bundled catalog part: `src/circuit/types.ts:142-155`,
  `DR:193-198`. They are reachable from SPICE import (`J` cards give njfet,
  `src/sim/engine/spice/netlist.ts:1045-1056, 1119-1125`) and from devolt's
  catalog. A part without a `catalogUid` for these kinds gets in-code
  literals only; `electricalSpecs` returns nothing.

### 1.3 One solve: order, Newton and convergence

- **Stamp order.** `_stampAll` (`SE:6971`) stamps the static list first (none
  of the W3 kinds) and then the dynamic list in circuit component order
  (`SE:5787-5807`; `SE:6992-6997` dispatches `deviceModel.stamp`). Float
  accumulation order into the matrix is semantics (the header of `SC:1-44`
  says so); a bit-exact port must stamp in circuit order. The homotopy overlay
  adds `_homotopyGminG` on every node diagonal before the devices
  (`SE:6981-6985`).
- **Newton** (`NW:73-131`; `SE:6786-6821`). Each iteration: `resetToBase`, then
  stamp every device at the previous iterate `x`, then solve. Default `maxIter`
  for a transient solve is 50 (`SE:6806`); ladder solves of the DC
  operating point use up to 1000 (`SE:736`). Convergence test per unknown
  `|a-b| <= vAbsTol + vRelTol*max(|a|,|b|)` with `vAbsTol = 1e-6`,
  `vRelTol = 1e-3` (`NW:80-82, 112-116`), over EVERY unknown, including
  branch currents (a regulator branch current of 1 A converges at 1 mA; at
  1 mA at 2 uA). A non-finite entry aborts with `converged: false`
  (`NW:109-111`). A converged iteration is refused if
  `limitedThisIteration()` reports that pnjlim clamped a junction
  (`NW:121-126`; set by the BJT and opto stamps via
  `ctx.markJunctionLimitedThisIteration`, `SC:777`, `MF:789`).
- **Start values.** `xInit` is the previous committed node voltages by net id
  (zero on the first solve) and, for internal solver rows (the opto base), the
  previous `_lastX` (`SE:6676-6702`).
- **Per-solve resets.** `_runNewton` clears the BJT previous-junction map,
  the compliance and entry clamp sets and the headroom selections before each
  solve (`SE:6786-6796`).
- **After an accepted solve** (`SE:6741-6779`): net voltages are published;
  `_updateState` runs each model's `commitState` (regime and latch decisions,
  gate history), then the digital pass, then `_updateElementI` runs each
  model's `updateCurrent`, then the package thermal step and the failure
  pass. A rejected solve (`lastConverged` false) commits nothing (`SE:6733`).
  The order matters: the thyristor, analog switch and opto publish a current
  committed in `commitState` because `updateCurrent` runs after it.
- **Time stepping.** Backward Euler by default; `useTrapThisSolve()` selects
  the trapezoidal companion for capacitors and inductors only (`SE:1989-1991`).
  None of the W3 kinds integrates with trapezoid: the MOSFET gate capacitances
  are BE in every mode (`SC:910-914`), the op-amp pole is a BE recurrence
  (`EL:1349-1389`), everything else is memoryless or a committed latch.

### 1.4 Current-limit and headroom active sets (regulators, dcdc)

Three per-solve sets decide CC and compliance inside Newton, cleared at the
start of every solve (`SE:2150-2260` for the definitions; `SE:6792-6797` for
the clear). They are the same machinery as bench_psu (W1 section 8.4.1).
Keys are strings such as `linear_reg:<id>`.

- **Entry clamp** `useCurrentLimitEntryClamp(key, branchCurrentGuess, limit)`
  (`SE:2210-2224`): once true it stays true for the solve. It latches when
  `|guess| > limit + 1e-12 + 1e-6*limit` (non-finite guess or `limit <= 0`
  never latches).
- **Compliance clamp** `useCurrentLimitComplianceClamp(key, outputGuess,
  complianceMagnitude, toleranceBelow)` (`SE:2225-2250`): once true it stays
  true. The first call per key per solve returns false and only marks the key
  as tried (so a CC trial is always made once, `SE:2236-2240`). Afterwards it
  latches when `outputGuess >= max(0, compliance) - tolerance`, with
  `tolerance = toleranceBelow ?? (1e-9 + 1e-6*max(1, compliance))`. The
  regulators and the dcdc pass `toleranceBelow = 0`.
- **Headroom selection** `regulatorHeadroomRegime(key, acceptedRegime,
  headroomGuess, regulatedVoltage, dropoutVoltage)` (`SE:2168-2208`): the first
  call per key per solve returns the accepted regime mapped to {0 REG, 1
  DROPOUT, 3 OFF} (CC is mapped to 0), so the accepted branch establishes the
  rail candidate first. Later calls compute `required = 3` if
  `headroom <= max(0, dropout)`, else `1` if `headroom < regulated + dropout`,
  else `0`, and keep a monotone capability order within the solve:
  REG (2) > DROPOUT (1) > OFF (0); the selection can only move down.

The regime committed after the solve is a separate, per-kind decision
(sections 13.3, 14.3); the sets above only choose which linear branch is
stamped for the iterate at hand.

### 1.5 What the step controller sees

`estimateStepError` (`AS:150-452`, W1 section 10.14) compares the coarse and
refined snapshots. For the W3 kinds:
- `icState` keys with a tolerance: `i1`, `i2`, `iin` at `ABS_CURRENT = 10 nA`;
  `u1`, `u2`, `d1`, `d2`, `supplyV`, `vm` at `ABS_VOLTAGE = 100 uV`
  (`AS:136-148`).
- Every other `icState` key is compared exactly, and a mismatch scores
  Infinity (`AS:454-461`): `reg` (linear_reg, lm317, dcdc), `reg1`, `reg2`,
  `limit1`, `limit2` (op-amps), `out1`, `out2` (lm393), `on`, `pol`
  (thyristors, switch). A regime flip between the full step and the two half
  steps rejects the step, so regime changes are *bracketed by step halving*
  when the host uses the checked step (headless `H_MIN = 1e-8`,
  `src/host/headless.ts:81`; the worker's floor is W1 section 10.16), not
  located by root finding. At the host floor classic accepts the step.
- `mosfetGates` `vgs`, `vgd` at `ABS_VOLTAGE` (`AS:416-430` region of the
  snapshot walk).
- `capCurrents` (thyristor and switch principal current) are snapshot-carried
  but not error-walked (W1 section 10.8; comment at `MF:540-545`).

### 1.6 Small-signal AC

`acStamp(ctx, comp, ac, omega)` runs once per frequency after a DC operating
point (W1 section 10.1; `ctx.opVoltage(node)` reads the held OP). Every W3
kind carries one (`ac-isolation.test.ts:343-395`, the census). General rules:
- `AC_GMIN = 1e-12` is added to every junction conductance, to mirror the
  transient stamp's GMIN (`SC:80-85`).
- Committed regimes (`icState`) select the same piecewise-linear branch the
  last transient step stamped; nothing re-decides a regime at AC.
- Where the transient stamp is a Norton source plus a conductance, the
  constant part differentiates away.
- Where the stamp uses the iterate `xGuess` (every junction), AC uses the OP
  voltages instead (no limiting).

### 1.7 Package thermal, failures and warnings for these kinds

- **Thermal.** `_thermalComponents` holds only components whose
  `_thermalProfile` resolves, which needs an exact `catalogUid`
  (`SE:2281-2300`, W1 section 6.3). The static exact profiles in W3 are
  `reg-7805` (`lm7805-to220`, 23.9 C/W, tau 15 s, shutdown 150/135 C,
  rated 15 W, `TP:163-176`), `reg-ams1117-33` and `reg-ams1117-50`
  (`ams1117-sot223-module`, 90 C/W, tau 8 s, 165/145 C, 1 W,
  `TP:185-197`), `lm317` (`lm317-to220-module`, 37.9 C/W, tau 15 s, 150/135 C,
  20 W, `TP:208-219`), `lm358` (`lm358-pdip8`, 80.9 C/W, tau 12 s, no
  shutdown), `mcp6002` (`mcp6002-pdip8`, 85 C/W, 12 s), `lm386`
  (`lm386-pdip8`, 53.4 C/W, 12 s) (`TP:229-280`, mapping `TP:326-336`).
  Step: `T' = T_target + (T - T_target)*exp(-dt/tau)`,
  `T_target = T_amb + P*rTheta` (`TP:513-525`); shutdown latches at
  `T >= trip` and clears at `T <= restart` (`TP:526-535`). The thermal update
  runs once per accepted step, after the commit, and not in DC-OP pseudo-steps
  (`SE:6756-6764`). No W3 diode, transistor, thyristor, comparator or converter
  has a thermal profile.
- **Dissipation** (`SE:7210-7290`): linear regulators
  `(vin - vout)+ * |x[k]| + (vin - vref)+ * Iq`, zero while shut down; analog
  ICs (`analog-ic` class) `vSupply * Iq + sum over units of
  (-i)*(vcc - vout)` if sourcing, `i*(vout - gnd)` if sinking, over the
  catalog's `analog_out` pins.
- **Failures.** No W3 kind has an overload failure, and none latches
  `led_failed` or `burnt`. The failure pass includes components that have an
  `updateFailures` hook (bjt, mosfet) or `electrical_specs.vcc_range`
  (lm358, mcp6002, lm386, lm393, dcdc_converter, via `SE:5770-5774`); for the
  `vcc_range` kinds it runs the output-sag scan, which only the lm393
  contributes targets to (section 11.7).
- **Warnings.** `SimWarningCode` has three members (`SE:657`):
  `transistor_overcurrent` (bjt, mos), `missing_flyback` (a switching kind on
  a winding; `FLYBACK_SWITCH_KINDS` includes bjt_npn, bjt_pnp, nmos, pmos;
  `FLYBACK_CLAMP_KINDS` includes diode, schottky_diode, zener_diode,
  tvs_diode; `SE:1069-1081`), and `rail_ic_power_chatter` (supply-gated parts;
  the lm393 is in `SUPPLY_GATED_KINDS`, `SE:1383-1389`).

### 1.8 The two junction laws

Both are used by several kinds and are evaluated at the previous iterate.

**Guarded Shockley** (`EL:693-742, 772-786`), `VtN = n*Vt`, `vSat` supplied
by the caller or `40*VtN`:
- `i(v) = Is*(exp(min(v,vSat)/VtN) - 1) + (Is*exp(min(v,vSat)/VtN)/VtN) *
  max(0, v - vSat)`
- companion: `gd = Is*exp(min(vPrev,vSat)/VtN)/VtN`, `g = gd + 1e-12`,
  `ieq = i(vPrev) - gd*vPrev`; stamp `+g` at (i,i), (j,j), `-g` at (i,j), (j,i),
  `b[i] -= ieq`, `b[j] += ieq`, nodes `< 0` skipped.
- The branch current at the fixed point is `i(v) + 1e-12*v`: the GMIN is a
  real shunt from the origin, not a Norton offset (`EL:721-724`).
- The caller's vSat: diode/schottky `min(max(40*VtN, Vf + 5*VtN), 80*VtN)`
  (`SC:381-382`); every other W3 caller of `stampDiodeShockley` passes either
  nothing (`40*VtN`: JFET gates, MOSFET body diode) or `raisedJunctionVSat`
  `min(max(40*VtN, anchorV + 5*VtN), 700*VtN)` (`MF:118-120`: thyristor
  gates, opto LED).

**Breakdown law** (`EL:815-852, 886-945`), `v = V(anode) - V(cathode)`:
- `vSatF = vSatFOverride ?? 40*nF*Vt`, `vSatZ = 5*nZ*Vt` (128.46 mV at nZ = 1,
  25 C); `vF = min(v, vSatF)`; `revArg = -(v + vz)`;
  `revC = min(revArg, vSatZ)`
- `gFwd = Is_f*exp(vF/VtNF)/VtNF`; `gRev = izKnee*exp(revC/VtNZ)/VtNZ`
- `iFwd = Is_f*(exp(vF/VtNF) - 1) + gFwd*max(0, v - vSatF)`;
  `iRev = izKnee*exp(revC/VtNZ) + gRev*max(0, revArg - vSatZ)`
- `i = iFwd - iRev`; `g = gFwd + gRev + 1e-12`; `ieq = i - (gFwd + gRev)*v`.
- There is no `-1` in the reverse term: the prefactor IS the knee current
  (comment `EL:788-798`), so `i(-vz) = -izKnee` (to the forward leakage).

---

## 2. Diodes: diode, schottky_diode, zener_diode, tvs_diode

[W3-D1: diode, schottky; W3-D2: zener, tvs.]

### 2.1 diode and schottky_diode

#### 2.1.1 Terminals and parameters

Pins `a` (anode, `pins[0]`) and `k` (cathode, `pins[1]`); the stamp returns
with fewer than two pins (`SC:355`). The two kinds are one case body
(`stampShockleyJunction`, `SC:349-384`), identical to the `led` body without
the LED branches: `isLed = comp.kind === "led"` is false (`SC:363`), so the
junction temperature is ambient, `n` defaults to `N_DIODE`, `vf` to 0.7 and
`iRated` to 0.1. The two kinds are registered as `diodeModel` (kinds
`["schottky_diode", "diode"]`, `SC:513-524`) so that no `updateFailures` hook
exists for them (`SC:19-25`).

| Parameter | Read at | Default | Catalog |
|---|---|---|---|
| `vf` (V, at `iRated`, 25 C) | `SC:369-372` | `specs.vf ?? 0.7` | diode-rectifier `0.7` (CAT:272), diode-schottky `0.3` (CAT:370) |
| `n` | `SC:373` | 1 | not in default_params |
| `iRated` (A) | `SC:374` | `specs.if_max ?? 0.1` | `if_max` 1 for both (CAT:295, 393) |
| `Is` (A at 25 C) | `SC:375-378` | derived from `vf`, `iRated` | not in default_params |

Catalog `electrical_specs`: diode-rectifier `vr_max 1000, vf 0.7, if_max 1,
vf_tol 0.05`; diode-schottky `vr_max 40, vf 0.3, if_max 1`. A bare `diode`
without a uid resolves to its only candidate and therefore gets `if_max = 1`
(W1 section 1.2, "only-candidate"), so its default rated current is 1 A, not
the literal 0.1 A. `vr_max` is not read by the engine (only by a live
diagnostic, section 2.1.7).

Derived quantities at 25 C (computed): `Vt = 25.6926 mV`; diode
(`vf 0.7, iRated 1, n 1`): `Is = 1.4708e-12 A`, `vSat = 1.027703 V`, drop
0.5817 V at 10 mA and 0.5225 V at 1 mA; schottky (`vf 0.3`): `Is = 8.4908e-6
A`, drop 0.1817 V at 10 mA and 0.1227 V at 1 mA.

#### 2.1.2 Equations

With `T = ambientTempC()` (`SC:365-367`), `Vt = thermalVoltage(T)`,
`Vf = max(0.05, vf25 - 0.002*(T - 25))` (`SC:369-372`), `VtN = n*Vt`:
- `Is = saturationCurrentAtTemperature(params.Is, T, n)` if `params.Is` is
  defined, else `iRated*exp(-Vf/VtN)` (`SC:375-378`).
- `vSat = min(max(40*VtN, Vf + 5*VtN), 80*VtN)` (`SC:381-382`). The comment
  says the 80 cap prevents f64 overflow.
- Terminal current (anode to cathode) `i(vd)` and the companion as in section
  1.8, `vd = V(a) - V(k)` from `xGuess`.
- Reverse: `i -> -Is` (with `Is*(exp(v/VtN) - 1)`); GMIN adds `1e-12*v`.
- Reported element current `commitShockleyJunctionCurrent` (`SC:429-465`):
  `shockleyDiodeCurrent(vd, Is, n, Vt, vSat)` re-evaluated at the converged
  `x`, signed and unclamped, excluding the GMIN shunt, 0 when `burnt` or
  `led_failed`. It matches the stamp's own branch current only to Newton
  tolerance.

#### 2.1.3 Dynamics and state

None. No junction or diffusion capacitance, no reverse recovery, no
temperature state (the junction is at ambient), no failure state. The
integration method is irrelevant. W1 section 6.10 states the same for the LED.

#### 2.1.4 Newton

No limiter: `SE` and `EL:686-691` say diodes are "current-driven (a series
resistor or follow-on conductance always bounds the loop)" and that pnjlim
"creates oscillation" (W1 section 6.2). The only protection is the tangent
extension above `vSat` and the 1e-12 GMIN. Linearisation point: the previous
iterate. Start value: the previous committed node voltages. The guard
is engaged only at absurd currents: for the 1 A / 0.7 V diode `vSat = 1.0277 V`
corresponds to 3.46e5 A, and above it the slope is 1.3475e7 S (74 nohm). A
diode directly across an ideal source therefore asks Newton for currents up
to 1e5 A and beyond before the tangent bounds the step, which is why every
real circuit has a series resistance (the stamp's own comment, `EL:686-691`).

#### 2.1.5 Regimes and events

No discrete regime. Forward, reverse (blocking at `-Is + 1e-12 v`) and the
tangent branch above `vSat` are one smooth function. No reverse breakdown:
`vr_max` does not clamp (a 1000 V diode blocks 10 kV).

#### 2.1.6 Small-signal AC

`acStampShockleyJunction` (`SC:394-426`): `gd + AC_GMIN` between anode and
cathode, with `gd = shockleyConductanceAtOp(vd_OP, Is, n, Vt, vSat)`
(`SC:95-106`) = `Is*exp(min(v,vSat)/VtN)/VtN`, the exact derivative of the law
including the constant tangent slope above `vSat`, with identical
parameter derivation. No susceptance. It IS the derivative of the transient
model; no disagreement.

#### 2.1.7 Diagnostics

No engine warning or failure. `diode` and `schottky_diode` are in
`FLYBACK_CLAMP_KINDS` (`SE:1073-1076`), so a diode on a winding's net
suppresses `missing_flyback`. The live diagnostics helper (not engine code,
`test/helpers/diagnostics/live.ts:482-508`) raises "reverse-overvoltage" for a
diode or schottky whose reverse voltage exceeds `vr_max * 1.1`
(`RATED_LIMIT_WARNING_MULTIPLIER = 1.1`, `live.ts:35`).

#### 2.1.8 Tests that pin it

- `physics-reference-benchmarks.test.ts:332-366` "diode and LED temperature
  direction": a silicon diode's forward drop falls by 0.33 V +/- 30 mV from
  -40 C to 125 C (this fixes `Vf(T)` and `Is(T)` together).
- `physics-reference-benchmarks.test.ts:368-447` "guarded junction overdrive":
  tangent-extended diode telemetry consistent with the solved branch.
- `dc-op-parity.test.ts:95` (`diodeLedCircuit`): DC-OP parity.
- `spice-ngspice-crossval.test.ts:431-462` case 3, half-wave rectifier with a
  smoothing capacitor against ngspice (`.tran`).
- `wave3-protection.test.ts` and `extended-integration.test.ts` use the diode
  as a clamp and in the mega-fixture; `ac-small-signal.test.ts` has no diode
  case.

#### 2.1.9 Questions

- **Q-W3-1 (parameter reads: literal versus catalog).** Section 1.2: `Is`,
  `betaF`, `betaR`, `nF`, `nR`, `vto`, `k`, `lambda` are read with in-code
  fallbacks different from the catalog's default_params (BJT `Is`: 1e-16 in
  code, 1e-14 in the catalog; MOSFET `lambda`: 0 versus 0.02; `earlyVoltage`
  and `cgs`/`cgd` through `modelParam`, so only an exact-uid save gets the
  catalog value). **Recommend** the port's compile layer reproduces each
  read rule exactly (a table like section 1.2) and logs, as W1e did for IC
  params, any component whose resolved value differs from the catalog default;
  changing a fallback is a catalog/migration decision for Dev, not a port
  decision.
- **Q-W3-2 (junction GMIN semantics).** The stamped diagonal is `gd + 1e-12`
  but the companion `ieq` uses `gd`: a 1e-12 S shunt to the origin sits in
  the branch (section 1.8), and the reported element current excludes it.
  The same pattern holds for the breakdown law, the MOSFET channel (GMIN as
  a floor on `gds`, not a shunt), and the TL431. **Recommend** mirroring this
  exactly (it is invisible above 1 pA but moves cut-off node potentials: a
  node with only a reverse diode and the 1e-12 node shunt sits at the
  midpoint).
- **Q-W3-3 (what `vf` means).** In classic `vf` is the forward voltage AT
  `iRated` (default `if_max`, 1 A for both catalog diodes), not at 10 mA.
  Datasheet figures (recalled): 1N4007 forward voltage about 0.93 V typical
  and 1.1 V max at 1 A (Vishay/ON); classic gives 0.7 V at 1 A. 1N5819
  Schottky 0.45 V typical, 0.6 V max at 1 A (ON Semiconductor, recalled);
  classic gives 0.3 V at 1 A. Classic's curve matches neither at 1 A and
  matches the 1N4007 only near 10 mA (classic 0.58 V; datasheet curve
  roughly 0.6 to 0.7 V). **Recommend** (decision 7 (a)/(b)): the port
  reproduces classic, the catalog value is the thing to correct, owner call.
- **Q-W3-4 (no series resistance).** The law has no `Rs`: the dynamic
  resistance is `VtN/i` all the way up (2.6 mohm at 10 A) until the tangent
  guard at `vSat = 1.0277 V` (3.46e5 A for the 1 A / 0.7 V diode, so the guard
  never engages in a physical circuit). Classic's drop at 10 A is
  `Vt*ln(10/Is) = 0.759 V`; a real 1N4007 (series resistance of the order of
  tens of mohm, recalled) drops well over 1 V at 10 A. **Recommend**
  reproduce, and put the guard constants (`vSat`, the tangent slope) in the
  golden so the Rust port's overflow behaviour is fixed.
- **Q-W3-5 (no capacitance, no recovery).** `Cj` (1N4007: 15 pF typ; recalled)
  and reverse recovery (1N4007 trr about 2 us; recalled) are absent. Not an
  accuracy gate under decision 3 (rectifier `.tran` within 5 percent RMS of
  swing), so **recommend** keep absent and list in the model disclosure
  (already done in `devolt/apps/sim/lib/sim/model-disclosure.ts`).

### 2.2 zener_diode

#### 2.2.1 Terminals and parameters

Pins `a` (`pins[0]`), `k` (`pins[1]`) (`SC:140-144`); `burnt` or latched
`led_failed` gives an open circuit (`SC:145-147`, `SC:171`, `SC:188`).

| Parameter | Read at | Default | Catalog |
|---|---|---|---|
| `vf` | `SC:150` through `ctx.junctionVf` (ambient tempco) | `specs.vf ?? 0.7` | 0.7 (CAT:303) |
| `vz` (V) | `SC:151` | 5.1 | 5.1 (CAT:304) |
| `n` (forward only) | `SC:152` | 1 | none |
| `iRated` | `SC:153` | `specs.if_max ?? 0.1` | `if_max` 0.1 (CAT:327) |
| `iz_knee` (A) | `SC:158` | 0.005 | not in default_params; `electrical_specs.iz_knee = 0.005` is NOT read |
| `nZ` | literal `1.0` at `SC:159` | 1 | none |

`params.Is` is not honoured by the zener. `p_max` is 1 W in the catalog
(read only by the live diagnostic).

#### 2.2.2 Equations

`Is_f = shockleyIsFromVf(Vf, iRated, nF, Vt)` with `Vf = junctionVf(vf)`
(`SC:154`). The breakdown law of section 1.8 with `Is_f`, `izKnee`, `nF`,
`nZ = 1`, `vz`, and NO `vSatFOverride` (`SC:160`), so
`vSatF = 40*nF*Vt = 1.0277 V`.
- At 25 C: `Is_f = 1.4708e-13 A` (0.1 A at 0.7 V); reverse knee 5 mA at
  `v = -vz`; one decade of reverse current per `Vt*ln 10 = 59.2 mV`.
- Reverse tangent: beyond overdrive `vSatZ = 0.12846 V` the current is
  `izKnee*e^5 = 0.7421 A` there and rises linearly with slope
  `0.7421/0.025693 = 28.88 S` (34.6 mohm).
- Reported current: `breakdownDiodeCurrent` (`EL:815-852`; `SC:162-182`),
  same law, no GMIN.

#### 2.2.3 Dynamics, state, Newton, regimes

None, none, none (no limiter, previous iterate), and three smooth zones
(forward conduction, blocking, breakdown) with no discrete event. Reverse
leakage below the knee is `izKnee*exp(-(|v|-vz)/Vt)`: below `1e-9 A` at 3 V
reverse for a 5.1 V part (the test at `wave3-protection.test.ts:124` asserts
< 1 nA).

#### 2.2.4 Small-signal AC

`SC:183-209`: `breakdownConductanceAtOp(...)` (`SC:115-135`) `+ AC_GMIN`:
`gFwd + gRev` from the clamped exponentials, which equals the slope of the
tangent extension beyond each guard. Derivative of the transient law; no
disagreement. Same parameter derivation as the stamp.

#### 2.2.5 Diagnostics

No engine diagnostics. `zener_diode` is a flyback clamp kind. The live
helper "zener-over-dissipation" fires when `|vAK * i| > 1.1 * p_max`
(`live.ts:547-585`), with `p_max = 1 W` for the zener and `0.4 W` for the TVS.

#### 2.2.6 Tests that pin it

`wave3-protection.test.ts`: forward conduction 4.0 to 5.0 mA through a
5 V / 1 kohm circuit (`:75-110`); reverse blocking, less than 1 nA at 3 V,
node near supply (`:113-160`); regulation at `vz`, cathode about 5.1 V with
about 8 mA (`:163-196`); the diagnostics interlock (`:488-543`).
`ac-isolation.test.ts` census.

#### 2.2.7 Questions

- **Q-W3-6 (what `vz` means; stiffness).** Classic anchors `vz` at 5 mA
  (`iz_knee`), the forward knee current at `vf` for `iRated`. A datasheet
  zener is specified at a test current: 1N4733A, 5.1 V at 49 mA, dynamic
  impedance 7 ohm at 49 mA, knee 1 mA at 550 ohm (recalled, ON Semiconductor
  1N47xx). Classic's slope at 49 mA is `Vt/I = 0.52 ohm`, 13 times stiffer
  than 7 ohm, and the voltage at 49 mA is `5.1 + 0.059 = 5.159 V`. The load
  regulation of a classic zener is therefore optimistic by an order of
  magnitude. **Recommend** reproduce (decision 7, rec (b) does not apply:
  there is no analytic answer other than classic's own law); report to Dev as a
  catalog/model improvement (add a series resistance `rz` and a test-current
  anchor), not a port task.
- **Q-W3-7 (the zener forward diode is `iRated`-anchored at 0.1 A and
  `vSatF` is not raised).** For `vf <= 1.0277 V` the anchor is honoured. For
  `vf > 1.0277 V` (an authored high-`vf` zener) the exponential guard bites
  before `vf` and the forward drop at `iRated` is larger than `vf` (the
  same defect as the TVS, section 2.3). **Recommend** reproduce and add to
  the golden a case with `vf = 1.2`.

### 2.3 tvs_diode

#### 2.3.1 Terminals and parameters

Pins `a`, `k` positional (`SC:218-219`). Only `burnt` gates it (`SC:220`;
`led_failed` is not read: nothing sets it).

| Parameter | Read at | Default | Catalog |
|---|---|---|---|
| `vbr` (V) | `SC:224` | 6.8 | 6.8 (CAT:337) |
| `bidirectional` (nonzero = true) | `SC:225` | 0 | 0 (CAT:338) |
| `iRated` | `SC:226` | 0.1 (specs NOT read) | none |
| `iz_knee` | `SC:227` | 0.005 | not in default_params; `electrical_specs.iz_knee 0.005` not read |
| forward `vf` | literal `junctionVf(1.1)` at `SC:232`, `params.vf` ignored | 1.1 | none |
| `n` | `N_DIODE` literal | 1 | none |
| `nZ` | literal 1 at `SC:228` | 1 | none |

Catalog specs: `p_max 0.4`, `vr_max 6.8` (equal to `vbr`, not the stand-off
voltage), `iz_knee 0.005`.

#### 2.3.2 Equations

**Unidirectional** (`bidirectional = 0`, `SC:230-234`): the breakdown law with
`Is_f = shockleyIsFromVf(junctionVf(1.1), iRated, 1, Vt)`, `izKnee`, `nF = 1`,
`nZ = 1`, `vz = vbr`, no vSat override, i.e. the zener with fixed 1.1 V
forward voltage.
- At 25 C: `Is_f = 2.5477e-20 A`, `vSatF = 1.0277 V`.
- **Finding.** `vSatF (1.0277 V) < Vf (1.1 V)`, so the exponential is
  clamped before the anchor: `i(1.1 V) = 22.9 mA`, not `iRated = 100 mA`.
  The forward drop at 100 mA is 1.4304 V (arithmetic: guard current
  5.997 mA, tangent slope 0.2334 S). The comment says "Vf ~ 1.1 V like a
  rectifier"; classic delivers 1.43 V at 0.1 A.

**Bidirectional** (`SC:235-270`): symmetric overdrive exponentials, no
Shockley term:
- `vSatOver = 5*nZ*Vt`; `argPos = vd - vbr`; `argNeg = -(vd + vbr)`;
  `ePos = exp(min(argPos, vSatOver)/(nZ*Vt))`, `eNeg` likewise;
  `gPos = izKnee*ePos/(nZ*Vt)`, `gNeg` likewise
- `iPos = izKnee*ePos + gPos*max(0, argPos - vSatOver)`, `iNeg` likewise;
  `i = iPos - iNeg`
- `g = gPos + gNeg + 1e-12`; `ieq = i - (gPos + gNeg)*vd`; stamped directly
  (`SC:265-269`), not through `stampBreakdownDiode`.
- No forward Shockley diode: `i(0) = izKnee*(exp(-vbr/Vt) - exp(-vbr/Vt))`
  = 0, and the current is odd in `vd` exactly.
Reported current: `SC:272-307`, same expressions.

#### 2.3.3 Dynamics, state, Newton, regimes

None; no limiter; smooth. At 25 C the device conducts less than `1e-20 A` at
`0.8*vbr`: there is no stand-off leakage (a real SMAJ6.8A leaks 1 uA at
`Vrwm = 5.8 V`, recalled).

#### 2.3.4 Small-signal AC

`SC:309-340`: unidirectional uses `breakdownConductanceAtOp`; bidirectional
`gPos + gNeg` with the clamped exponentials (the tangent slope beyond the
guard); `+ AC_GMIN`. Derivative of the transient law (the bidirectional
transient `g` omits no term). No disagreement.

#### 2.3.5 Diagnostics and tests

As the zener (live helper, `p_max = 0.4 W`). Tests: `wave3-protection.test.ts`
blocking at 5 V below `vbr` (`:199-224`), unidirectional clamping at 12 V,
cathode about 6.8 V and about 23 mA (`:226-257`), bidirectional symmetric
clamping forward and reversed (`:259-316`), the TVS over-dissipation
diagnostic (`:488-521`).

#### 2.3.6 Questions

- **Q-W3-8 (TVS forward drop, section 2.3.2).** Classic's unidirectional TVS
  forward drop at its 100 mA anchor is 1.43 V, the anchor (1.1 V) is not
  honoured. **Recommend** reproduce unchanged (the core's breakdown-law port
  must not "fix" it by raising vSat; a golden at `v = 1.1` and `0.1 A`
  fixes it) and tell Dev.
- **Q-W3-9 (clamp voltage versus the datasheet).** SMAJ6.8A (recalled):
  `Vbr 6.45 to 7.14 V at 10 mA`, `Vc 10.5 V at Ipp 38.1 A`, 400 W peak
  (10.5 V * 38.1 A), `Vrwm 5.8 V`. Classic: knee at 5 mA (not 10 mA) and a
  34.6 mohm tangent past 0.742 A; at 38.1 A the clamp is
  `6.8 + 0.128 + (38.1 - 0.742)/28.88 = 8.22 V` versus 10.5 V, 22 percent
  low. The catalog `p_max 0.4 W` looks like the 400 W peak figure scaled by
  1000; the diagnostic therefore fires at 0.44 W continuous, which is a
  steady-state, not a pulse, number. **Recommend** reproduce; flag the
  catalog's `p_max` and `vr_max` (which equals `vbr`, not `Vrwm`).
- **Q-W3-10 (breakdown-law curvature for W3-D2).** DK 1.6 gives a two-
  terminal kernel for the zener and TVS with a curvature band bound per law.
  The classic law has TWO exponentials with different `VtN` (`nF*Vt` forward,
  `nZ*Vt` reverse, equal here) and a tangent extension at each guard, so a
  `f''/f'` bound must take the larger of `1/(nF*Vt)` and `1/(nZ*Vt)`
  (`1/Vt = 38.9 /V` at 25 C). **Recommend** `NARROWEST_BAND` until measured
  (DK question 9 (a)).

---

## 3. Bipolar transistors: bjt_npn, bjt_pnp

[W3-T.] One model object, `bjtModel` (`SC:736-882`), kinds `["bjt_npn",
"bjt_pnp"]`, polarity `pol = +1` for NPN and `-1` for PNP (`SC:750`).

### 3.1 Terminals and parameters

Pins by position `[C, B, E]` (`SC:739-748`; catalog ids `c`, `b`, `e`,
functions collector, base, emitter). The stamp, the readout and the AC stamp
return (stamping and reading nothing) with fewer than three pins, and the
stamp and AC stamp also return when the BASE pin is open (`ctx.isOpenPin`,
`SC:743, 804, 849`): a BJT with an unwired base is absent, collector and
emitter float and the current reads 0.

| Parameter | Read at | Rule (section 1.2) | Default | Catalog (`bjt-npn` CAT:971, `bjt-pnp` CAT:1014) |
|---|---|---|---|---|
| `Is` (A at 25 C) | `SC:751` | 1 | **1e-16** | 1e-14 both |
| `betaF` | `SC:752` | 1 | 100 | 100 |
| `betaR` | `SC:753` | 1 | 1 | 1 |
| `nF` | `SC:754` | 1 | 1 | 1 |
| `nR` | `SC:755` | 1 | 1 | 1 |
| `earlyVoltage` (V) | `SC:756` (through `modelParam`) | 3 | 0 (no Early effect) | 75 (npn), 100 (pnp) |

Specs: `i_c_max 0.6` A both, `beta_typical 100` (only `i_c_max` is read, by
the overcurrent warning, section 3.7). `Is` temperature scaling uses `nF` for
both junctions (`SE:3489-3496`). The per-load junction cache
(`_rebuildJunctionCache`, `SE:3482-3504`) stores for each BJT, at the ambient
temperature of that load: `IsT = saturationCurrentAtTemperature(Is, T, nF)`,
`VtNF = nF*Vt`, `VtNR = nR*Vt`, `vCritBE = vCritFor(IsT, VtNF)`,
`vCritBC = vCritFor(IsT, VtNR)` (note both critical voltages use `IsT`, which
was scaled with `nF`). Examples at 25 C: with `Is = 1e-14`, `vCrit = 0.7253 V`;
with `Is = 1e-16`, `vCrit = 0.8436 V`.

### 3.2 Equations

All from `bjtLinearization` (`EL:973-1041`), shared by the stamp
(`stampBJT`, `EL:1071-1123`), the readout (`bjtCurrents`, `EL:1128-1162`) and
the AC stamp. Node voltages `vB`, `vC`, `vE`; `Vt = junctionVt()`:
- `vBE = pol*(vB - vE)`, `vBC = pol*(vB - vC)`
- `nfVt = nF*Vt`, `nrVt = nR*Vt`, `vSatF = 40*nfVt`, `vSatR = 40*nrVt` (1.0277 V
  at `n = 1`, 25 C)
- `vf = min(vBE, vSatF)`, `vr = min(vBC, vSatR)`: a HARD clamp, with no
  tangent continuation (unlike the diode law; section 3.9).
- `eF = exp(vf/nfVt)`, `eR = exp(vr/nrVt)`; `iF = Is*(eF - 1)`,
  `iR = Is*(eR - 1)`; `gF = Is*eF/nfVt`, `gR = Is*eR/nrVt`; `bRR = 1 + 1/betaR`.
- Forward Early effect (`EL:1012-1015`): `va = earlyVoltage` if finite and
  positive, else 0; `earlyRaw = 1 + pol*(vC - vE)/va` if `va > 0` else 1;
  `earlyScale = max(0, earlyRaw)`; `earlySlope = 1/va` if `va > 0` and
  `earlyRaw > 0`, else 0. The multiplier applies to `iF` only, not to `iR`.
- Currents into the terminals (transport form):
  `icF = iF*earlyScale - iR*bRR`; `ibF = iF/betaF + iR/betaR`;
  `ib = pol*ibF`; `ic = pol*icF`; `ie = -(ib + ic)`.
- Jacobian (row = terminal KCL, column = node voltage; identical for NPN and
  PNP, the polarity factors cancel):
  - `gbB = gF/betaF + gR/betaR`, `gbC = -gR/betaR`, `gbE = -gF/betaF`
  - `gcB = gF*earlyScale - gR*bRR`, `gcC = gR*bRR + iF*earlySlope`,
    `gcE = -gF*earlyScale - iF*earlySlope`
  - `geX = -(gbX + gcX)` for `X in {B, C, E}` (derived from KCL so every
    column conserves current even with the Early term)
- Companion (`EL:1113-1120`): `iBeq = ib - gbB*vB' - gbC*vC' - gbE*vE'`, and
  likewise `iCeq`, `iEeq`; `b[B] -= iBeq`, `b[C] -= iCeq`, `b[E] -= iEeq`;
  the nine conductances are added at (row, col) when both are `>= 0`. The
  primed voltages are the LIMITED effective voltages of section 3.4, not the
  raw iterate.
- No GMIN is stamped (the Jacobian has no `1e-12` term, and the AC stamp has
  none either). A cut-off BJT's base and collector are held only by the
  `1e-12 S` node shunt.
- Reported element current (`SC:800-835`): `bjtCurrents(...).ic` at the raw
  converged `x`, with `IsT` recomputed from `ctx.ambientTempC()` and
  `nF`: the COLLECTOR current into the collector terminal. For a PNP it is
  negative in forward-active operation (current leaves the collector
  terminal). Returns 0 with an open base.
- Not modelled (catalog `notes`, CAT:971 and following): series resistances
  `rb`, `re`, `rc`, junction capacitance, transit time, high-injection
  roll-off, breakdown, noise, self-heating, reverse Early.

### 3.3 Dynamics and state

None: the BJT is memoryless. The only state is the Newton limiter's
previous-junction map (section 3.4), which is per-solve scratch
(`_vPrevBJT`, cleared at `SE:6787` and in `_rebuildJunctionCache`), never
committed and never snapshotted.

### 3.4 Newton

- **pnjlim on each junction independently**, then the node voltages are
  rebuilt around the fixed emitter (`SC:758-781`):
  `vBElim = pnjlim(vBEraw, prev.vBE, VtNF, vCritBE)`,
  `vBClim = pnjlim(vBCraw, prev.vBC, VtNR, vCritBC)`;
  `vB_eff = vE + pol*vBElim`; `vC_eff = vB_eff - pol*vBClim`;
  the stamp then uses `(vB_eff, vC_eff, vE)` as the linearisation point for
  both the Jacobian and the companion sources.
- `pnjlim` (`NW:133-162`): if `vNew > vCrit` and `|vNew - vOld| > 2*VtN`:
  for `vOld > 0` `arg = 1 + (vNew - vOld)/VtN`, result
  `vOld + VtN*ln(arg)` if `arg > 0` else `vCrit`; for `vOld <= 0` result
  `VtN*ln(vNew/VtN)`; otherwise `vNew` unchanged.
- `prev` is `ctx.bjtPrevJunction(id) ?? {vBE: vBEraw, vBC: vBCraw}` and is
  set to the limited values every stamp (`SC:771, 779`); on the first
  iteration of every solve the map is empty, so pnjlim is a no-op there and
  limits from the second iteration on.
- If either junction was limited (`vBElim !== vBEraw || vBClim !== vBCraw`)
  the stamp calls `ctx.markJunctionLimitedThisIteration()` (`SC:776-778`), and
  the Newton loop refuses to accept a delta-x-converged iteration while the
  flag is set (`NW:121-126`). The flag is reset at the start of every
  iteration (`SE:6807-6810`).
- Start value: the previous committed node voltages. A fresh solve with
  nothing wired is `vBE = 0`, which `pnjlim` leaves alone.
- DC operating point: the engine's gmin and source-stepping ladder
  (W1 section 10) wraps these same stamps; the BJT has no ladder-specific
  code. `dc-homotopy-pathological.test.ts:182-244` pins a bistable BJT latch
  cold start.

### 3.5 Regimes and events

No discrete regime: cut-off, active, saturation and reverse-active are one
smooth law. The only piecewise features are the `vSat` clamps and the
`max(0, earlyRaw)` floor (collector current from `iF` vanishes at
`vCE <= -VA`). Nothing is committed, nothing is detected, nothing is stepped
over.

At 25 C, `Is = 1e-14`, `betaF = 100`, `betaR = 1`, forcing `Ic = 150 mA` with
`Ib = 15 mA` gives `Vbe = 0.7837 V`, `Vbc = 0.7171 V`, `Vce(sat) = 66.6 mV`
(solved from the transport equations; `Vce(sat)` depends only on `Ic/Ib`,
`betaF` and `betaR`, not on `Is` or on the absolute currents).

### 3.6 Small-signal AC

`SC:840-881`: the nine `lin` entries of `bjtLinearization` at the raw OP
voltages, `Is` temperature-scaled as in the cache, `nF`, `nR`, Early included,
no capacitances, no GMIN. At a converged OP `pnjlim` is inert, so this IS the
transient Jacobian; no disagreement. (A frequency-flat transistor: no `Cbe`,
`Cbc`, so the AC gain does not roll off.)

### 3.7 Diagnostics

`transistor_overcurrent` (`SC:690-731`, `SC:837-839`): a SimWarning, never a
failure and never a stamp change. `limit = electricalSpecs(comp).i_c_max`
(0.6 A for both catalog parts; none, no warning); `current = |elementCurrent|`
(the collector current). `recordAccumulatedWarning(code, id, "", rate = 1 if
current > limit else 0, recovery 0.25, h, dwell 1e-4, ...)`: a leaky integrator
that fills at rate 1 and drains at 0.25 per second of simulated time, latching
when it reaches `1e-4` accepted seconds of net over-rating. Message:
"`id` is carrying N mA of collector current against its M mA rating."
(W1 section 9.7.3.) `missing_flyback` counts `bjt_npn` and `bjt_pnp` as
switching kinds (`SE:1069-1072`).

### 3.8 Tests that pin it

- `physics-reference-benchmarks.test.ts:449-474`: NPN forced-bias collector
  current equals the forward-active Ebers-Moll value within 1 percent
  (`Is = 1e-16`, 0.7 V).
- `semiconductor-parasitics.test.ts:102-217`: the Early effect: transport
  current slope `VCE/VA` and PNP symmetry (`:103`), the historical result
  exactly when `VA` is absent or zero (`:130`), the collector Jacobian against
  finite differences of the reported current (`:138`), the readout through the
  engine (`:170`), the catalog default applied to an older identified save
  (`:193`).
- `ac-small-signal.test.ts:494-552`: common-emitter gain equals
  `-gm*(Rc || ro)` from the engine's own OP within 2 percent.
- `engine-warnings.test.ts:110-242`: `transistor_overcurrent`: latches once
  when over the rating and reports value against limit, silent at 90 percent,
  needs dwell (the load seed and one step do not announce), does not change the
  trajectory, rolls back with `restoreState`, carries across a reload.
- `spice-ngspice-crossval.test.ts:464-510` case 4: BJT common-emitter `.op`
  and `.ac` against a live ngspice.
- `dc-homotopy-pathological.test.ts:182-244`: the bistable BJT latch.
- `wave6-hbridge`, `wave6-relay`, `wave7-dcmotor`, `wave7-servo-stepper`
  drive loads through BJTs.

### 3.9 Questions

- **Q-W3-11 (`Is` default 1e-16 versus catalog 1e-14).** A BJT with no `Is`
  param behaves as `Is = 1e-16`: `Vbe = 0.769 V` at 1 mA, against 0.651 V at the
  catalog's `1e-14` (computed). The `bjt-npn` and `bjt-pnp` catalog entries
  carry `Is = 1e-14`; a freshly placed part has the catalog value, a
  hand-built or SPICE-imported circuit without `Is` gets `1e-16`. The 2N2222
  SPICE value is 14.34 fA (recalled). **Recommend** Q-W3-1: reproduce the
  literal in the compiler, keep a test that pins the 0.1 V gap.
- **Q-W3-12 (hard clamp versus tangent).** The BJT clamps `exp` at
  `vSat = 40*n*Vt` and freezes both current and conductance
  (`Is*(e^40 - 1)` = 2.35e3 A for `Is = 1e-14`), while the diode, the JFET
  gate, the opto LED and the MOS body diode extend along the tangent. Beyond
  1.0277 V of `vBE` the BJT Jacobian is constant, so a Newton iterate sitting
  there only moves by the linear solve. **Recommend** reproduce each as is;
  do not unify (DK `MAX_T`/`Eval` kernels evaluate per device, so this is a
  per-kernel `eval` detail, but the golden must exercise `vBE = 1.2 V`).
- **Q-W3-13 (no series resistance: `Vce(sat)` far below the datasheet).**
  Classic gives `Vce(sat) = 66.6 mV` at `Ic/Ib = 10`. The 2N2222A datasheet
  (ON Semiconductor, recalled) guarantees `VCE(sat) <= 0.3 V` at 150 mA /
  15 mA, with typical values around 0.15 V; the difference is the collector
  and emitter bulk resistance the model lacks. This is documented in the
  catalog `notes`, so it is a known limitation. **Recommend** reproduce.
- **Q-W3-14 (open base removes the whole device).** `isOpenPin(base)` returns
  from the stamp, so a BJT with a floating base stamps nothing, while a base
  tied through any high resistance stamps the full model. A real transistor
  with an open base still conducts `Iceo` (nA scale). **Recommend** reproduce
  (compile as an absent device).
- **Q-W3-15 (cache versus live params).** The stamp reads `IsT`, `VtN*` and
  the critical voltages from the load-time cache, while `updateCurrent` and
  `acStamp` recompute them from the live `params` and ambient temperature
  (`SC:812-835, 840-870`). The two agree unless a param is edited in place
  without `load()`, which direct API callers can do (comments at
  `SE:7694` allow it for sag poles). The core fixes params at load, so
  **recommend** no action; note it in the compile layer's contract.
- **Q-W3-16 (limiter choice for the kernel).** DK question 10 asks classic's
  limiting where it has one, SPICE's otherwise: for the BJT classic's is
  pnjlim as in section 3.4 with the extra "converged only if not limited"
  rule. **Recommend** port exactly that (state: the previous limited
  `(vBE, vBC)` per junction per solve in `DeviceKernel::limit`'s scratch, not in
  `State`).

---

## 4. MOSFETs: nmos, pmos

[W3-T.] One model object, `mosfetModel` (`SC:884-1040`), kinds `["nmos",
"pmos"]`, `pol = +1` / `-1` (`SC:897`). Level-1 Shichman-Hodges channel, an
intrinsic body diode (bulk tied to source) and lumped `Cgs`, `Cgd`.

### 4.1 Terminals and parameters

Pins by position `[D, G, S]` (`SC:887-895`; catalog ids `d`, `g`, `s`). No
open-pin gate: an unwired gate is just a floating row (held by the node shunt).

| Parameter | Read at | Rule | Default | Catalog |
|---|---|---|---|---|
| `vto` (V, positive magnitude for both polarities) | `SC:898` | 1 | 0.7 | 0.7 |
| `k` (A/V^2, `(KP/2)(W/L)`) | `SC:899` | 1 | 0.02 | 0.02 |
| `lambda` (1/V) | `SC:900` | 1 | **0** | 0.02 |
| `cgs` (F) | `SC:904` | 3 | 0 | nmos 5.5e-11, pmos 4e-11 |
| `cgd` (F) | `SC:905` | 3 | 0 | nmos 5e-12, pmos 2e-11 |
| `bodyDiodeVf` (V, at 1 A) | `SC:924` through `ctx.junctionVf` | 1 | 0.7 | none |

Specs: `i_c_max 5` A both (`nmos` CAT:1057, `pmos` CAT:1098), read by the
overcurrent warning. `cgs`/`cgd` are zeroed in a `dcSolveMode()` solve
(`SC:904-905`), "the gate is open at the operating point". Neither `vto` nor `k`
is temperature dependent.

### 4.2 Equations

`vGS = pol*(vG - vS)`, `vDS = pol*(vD - vS)`, `vOV = vGS - vto`
(`EL:1228-1231`); `GMIN_M = 1e-12`. At the previous iterate:
- **cutoff** (`vOV <= 0`): `id = 0`, `gm = 0`, `gds = GMIN_M`.
- **triode** (`vOV > 0` and `vDS < vOV`, which INCLUDES every negative `vDS`):
  `r = 2*vOV*vDS - vDS^2`, `m = 1 + lambda*vDS`; `id = K*r*m`;
  `gm = 2*K*vDS*m`; `gds = K*(2*vOV - 2*vDS)*m + K*r*lambda`, floored at
  `GMIN_M`.
- **saturation** (`vOV > 0`, `vDS >= vOV`): `m = 1 + lambda*vDS`;
  `id = K*vOV^2*m`; `gm = 2*K*vOV*m`; `gds = K*vOV^2*lambda`, floored at
  `GMIN_M`.
- The boundaries are C1-continuous at `vDS = vOV` (current, `gm`, `gds` agree
  on both sides). At `vOV = 0` the law is NOT continuous for `vDS < 0` (finding
  below).
- Actual drain current `idActual = pol*id`.
- Stamp (`EL:1262-1281`): `(d,g) += gm`, `(d,d) += gds`, `(d,s) += -gm - gds`,
  `(s,g) += -gm`, `(s,d) += -gds`, `(s,s) += gm + gds`; companion
  `iDeq = idActual - (gm*vG + gds*vD + (-gm - gds)*vS)` with the RAW node
  voltages (no polarity factors: they cancel in the Jacobian), `b[d] -= iDeq`,
  `b[s] += iDeq`. The gate row has no stamp.
- **Body diode** (`EL:1283-1305`): NMOS anode = source, cathode = drain,
  `vBd = vS - vD`; PMOS anode = drain, cathode = source, `vBd = vD - vS`.
  `stampDiodeShockley(bdAnode, bdCathode, vBd, bodyDiodeIs, 1.0, thermalV)`
  with `bodyDiodeIs = shockleyIsFromVf(bodyDiodeVf, 1.0, 1.0, thermalV)`
  (i.e. exactly 1 A at `bodyDiodeVf`; `1.4708e-12` at 0.7 V, 25 C) and the
  default `vSat = 40*Vt`. Nothing else (no `Cjb`, no `Rb`).
- **Gate charge** (`EL:1307-1320`): only when `gateDynamics.h > 0` and the
  capacitance is `> 0` (clamped `max(0, finite)`): `stampCapacitor(g, s, cgs,
  h, vgsPrev)` and `stampCapacitor(g, d, cgd, h, vgdPrev)`, the backward-Euler
  companion (`EL:164-176, 217-231`): conductance `C/h`, history current
  `(C/h)*vPrev` (`b[i] += hist`, `b[j] -= hist`). `vgsPrev`, `vgdPrev` come from
  `ctx.state.mosfetGates.get(id) ?? {vgs: 0, vgd: 0}` (`SC:906`): the
  previous ACCEPTED solve's `(vG - vS, vG - vD)`, set by `commitState`
  (`SC:935-945`) and never by a Newton iterate. BE in both integration
  modes (`SC:908-914`).
- **Reported element current** (`SC:946-975`): `idChannel` (`mosDrainCurrent`,
  `EL:1784-1804`, the same law, no GMIN) `+ bodyDrainCurrent + cgdDrainCurrent`:
  `bodyDrainCurrent = -pol*shockleyDiodeCurrent(bodyVoltage, bodyDiodeIs, 1, vt)`
  with `bodyVoltage = vS - vD` (NMOS) or `vD - vS` (PMOS), default `vSat`;
  `cgdDrainCurrent = -cgd*((vG - vD) - previousVgd)/h` (0 for `h <= 0`),
  `previousVgd` the PRE-commit snapshot (`ctx.previousMosfetGate`,
  `SE:6751`). The `Cgs` current never reaches the drain and is not included.
  So the readout satisfies KCL at the drain (the body diode and the Miller
  current are in it) but not at the source. Positive = into the drain terminal.

### 4.3 Dynamics and state

The only state is `mosfetGates[id] = {vgs, vgd}` (the two history voltages),
initialised `{0, 0}` at `load()` and carried across `load()` for a same-id,
same-kind component (`SE:3130-3141`), in the snapshot and rollback
(`SE:2616, 2678, 2724`), and compared by the step controller at
`ABS_VOLTAGE` (W1 section 10.14). Integration: backward Euler on `Cgs` and
`Cgd` always; the trapezoidal rule is never used here (`SC:908-914`: "threading
branch currents through stampMOSFET is not the trivial change Wave A2
allows").

At the first step after `load()` a gate at 5 V sees `vgsPrev = 0` unless the
load seed already committed the gate (the seed solve runs `commitState`, so it
normally does: `dcSolveMode` zeroes only the capacitances, not the commit).

### 4.4 Newton

No limiter of any kind (no `fetlim`): the square-law kinks at `vOV = 0` and
`vDS = vOV` are evaluated at the previous iterate and the region can flip
between iterations with no memory or hysteresis. `limitedThisIteration` is
never set by the MOSFET. Start value: the previous committed node voltages.
The pathological cold start `dc-homotopy-pathological.test.ts:246-299`
(cross-coupled NMOS pair) is the only test of this behaviour and it passes via
the engine's homotopy ladder, not via the device.

### 4.5 Regimes and events

Three analytic regions chosen from the iterate every Newton iteration, never
committed: no detection of the instant, no event. The step controller does not
see a region change (nothing is committed), only the resulting voltage error.

**Findings in the law** (computed):
1. *Reverse channel conduction is asymmetric.* For `vDS < 0` with `vOV > 0`
   the law is the triode formula with `vOV` measured from the NAMED source
   terminal, `id = K*(2*vOV*vDS - vDS^2)*(1 + lambda*vDS)`. A physical MOSFET
   is source/drain symmetric; the reverse current here is
   `K*(2*vOV*|vDS| + vDS^2)*(1 - lambda*|vDS|)` in magnitude, larger than the
   forward value at the same `|vDS|` (for `vGS = 5 V`, `vto = 0.7`, `K = 0.02`,
   `lambda = 0.02`: `+0.356 A` at `vDS = +3 V` (triode), `-0.654 A` at
   `vDS = -3 V`).
2. *`(1 + lambda*vDS)` goes negative for `vDS < -1/lambda`* (-50 V at
   `lambda = 0.02`): the channel current then reverses sign.
3. *Discontinuity at `vOV = 0` for `vDS < 0`.* Crossing `vGS = vto` the current
   jumps by `-K*vDS^2*(1 + lambda*vDS)` (about 9.7 mA at `vDS = -0.7 V`,
   0.17 A at `-3 V`), because cut-off is exactly 0 while the triode formula at
   `vOV = 0+` is not.
These are visible only when the drain is below the source (a freewheeling or
reversed device); the body diode takes over beyond roughly `-0.7 V`.

### 4.6 Small-signal AC

`SC:979-1039`: `gm`, `gds` from the same piecewise law (the AC code
re-derives them with identical expressions, `gds` floored at `AC_GMIN`),
stamped as the six VCCS entries; the body diode's guarded slope
`shockleyConductanceAtOp(vBd, bodyDiodeIs, 1, vt) + AC_GMIN` between its anode
and cathode; and the gate charge as true `j*omega*C`
(`stampAcAdmittance(ac, g, s, 0, omega*cgs)` and `(g, d, 0, omega*cgd)`,
`SC:1035-1038`), with `max(0, ...)`. It is the derivative of the transient
model with the BE companion replaced by the exact capacitor: they agree in the
limit `omega*h -> 0` (the BE capacitor admittance is `C/h*(1 - e^{-j*omega*h})`
sampled, not `j*omega*C`). No other disagreement.

### 4.7 Diagnostics

`transistor_overcurrent` as in section 3.7 with `limit = i_c_max = 5 A` and
`current = |element current|` (the drain current including body and Miller
parts, `SC:837, 976`; message says "drain current"). `missing_flyback` counts
`nmos`, `pmos` as switching kinds.

### 4.8 Tests that pin it

- `physics-reference-benchmarks.test.ts:476-490`: NMOS triode (within 0.5
  percent) and cut-off (below 1 nA) currents against the level-1 equations
  (`VTO = 1`, `K = 0.01`, `VGS = 5`, `VDS = 2`).
- `semiconductor-parasitics.test.ts:219-403`: Cgs and Cgd as conservative BE
  companions (`:220`), the closed-form BE charge of a resistively driven gate
  and exact rollback (`:254`), the catalog `cgs`/`cgd` default on an older save
  (`:294`), Miller coupling of a drain step (`:325`), the body-diode current in
  the drain readout (`:357`) and its temperature consistency (`:379`).
- `ac-small-signal.test.ts:554-625`: common-source Miller pole within 5
  percent.
- `engine-warnings.test.ts:144-160`: overcurrent for a MOSFET.
- `dc-homotopy-pathological.test.ts:246-299`: the NMOS latch.
- `wave6-relay`, `wave6-hbridge`, `wave7-*`: MOSFET and BJT drivers.

### 4.9 Questions

- **Q-W3-17 (reverse conduction, section 4.5 findings 1 to 3).** The
  port can mirror the asymmetric reverse law bit for bit, or make the
  channel symmetric by swapping the roles of D and S when `vDS < 0`. Decision 7
  (b) points at the analytic answer; the analytic answer for a symmetric
  device is the symmetric law, and classic's differs by a large factor. The
  circuits that reach it (H-bridge freewheel, synchronous rectifier) are also
  the ones with the body diode in parallel. **Recommend** mirror classic for
  W3-T (gate 1 identity) and open a divergence entry for the symmetric law,
  decided after the extended-semis ngspice cases; do NOT fix silently.
- **Q-W3-18 (catalog `lambda` 0.02 versus code default 0).** A MOSFET placed
  from the catalog has `lambda = 0.02` (output conductance `K*vOV^2*0.02`);
  one without the key has 0. Cited by Q-W3-1.
- **Q-W3-19 (no Newton limiter).** SPICE limits the gate voltage step
  (`fetlim`) and the drain-source step (`limvds`). Classic has neither; DK
  question 10's "SPICE's otherwise" would add them. **Recommend** the port
  adds none in W3-T (bit-identity with classic and the cold-start tests, which
  pass without), and revisits only if the 6-cadence harness shows a
  convergence loss; record it as a divergence if added.
- **Q-W3-20 (gate capacitance model).** `Cgs`/`Cgd` are constant. A real
  2N7000: `Ciss` about 20 to 60 pF, `Crss` about 5 pF, `Coss` about 25 pF
  (recalled): classic's `55 pF` and `5 pF` are in range but the real `Cgd`
  rises steeply at low `vDS`. Not an accuracy-gate item. **Recommend**
  reproduce.
- **Q-W3-21 (the readout satisfies KCL at the drain, not the source).** An
  `elementI` consumer summing source current gets `Cgs` and body diode
  mismatches. **Recommend** the core publishes the same drain-terminal
  number (it is the telemetry contract) and nothing else.
- **Q-W3-22 (body diode `vSat` is not raised).** The body diode uses
  `vSat = 40*Vt = 1.0277 V` while its 1 A anchor is at 0.7 V, so it is the
  generic diode; unlike the TVS (section 2.3) the anchor is below `vSat` and
  honoured. No action; listed so the golden pins the 1 A anchor.

---

## 5. JFETs: njfet, pjfet

[W3-F, "on the MOS pattern".] `jfetModel` (`MF:329-432`), kinds `["njfet",
"pjfet"]`, `pol = +1` / `-1`. A depletion-mode Shichman-Hodges channel plus two
physical gate junction diodes. No catalog entry in `default-parts.json`; the
devolt catalog has `njfet` (`devolt:parts.json:2007`) and `pjfet`
(`devolt:parts.json:2066`).

### 5.1 Terminals and parameters

Pins by id `d`, `g`, `s` (`MF:337-340`); fewer than three pins returns.

| Parameter | Read at | Rule | Default | devolt catalog |
|---|---|---|---|---|
| `vto` (V) | `MF:319` | 1, coerced `-max(0.05, abs(vto))` | -2 | njfet -3, pjfet -4.5 |
| `idss` (A) | `MF:320` | 1, `max(1e-9, .)` | 0.01 | njfet 0.003, pjfet 0.022 |
| `lambda` (1/V) | `MF:324` | 1, `max(0, .)` | 0 | njfet 0.0033, pjfet 0.001 |
| `Is` (gate junctions, A) | `MF:325` | 1, `max(1e-30, .)` | 1e-14 | none |

`beta = idss/vto^2` (`MF:323`), so `Id(vGS = 0) = idss` in saturation. `vto`
is forced negative for both channel types: a positive authored value is
treated as `-|vto|`. Specs in the devolt catalog: njfet `i_c_max 0.005`,
`p_max 0.31`; pjfet `i_c_max 0.07`, `p_max 0.35`; the engine reads neither (no
overcurrent warning for the JFET).

### 5.2 Equations

Channel: `jfetLinearization` (`EL:1839-1874`), identical to the MOSFET law of
section 4.2 with `vto -> vto (negative)`, `K -> beta`: `vGS = pol*(vG - vS)`,
`vDS = pol*(vD - vS)`, `vOV = vGS - vto`; cutoff `vOV <= 0`, triode
`vDS < vOV`, saturation otherwise; `GMIN_J = 1e-12` floors `gds`. The stamp
(`stampJFET`, `EL:1884-1918`) is the MOSFET's six-entry VCCS pattern and
companion without a body diode or gate charge. The findings of section 4.5
(asymmetric reverse conduction, `(1 + lambda*vDS)` sign, the jump at
`vOV = 0` for `vDS < 0`) apply unchanged.

Gate (`MF:351-370`): two guarded Shockley diodes, `Is = isGate`, `n = N_DIODE`,
`Vt = ctx.junctionVt()`, default `vSat = 40*Vt` (`stampDiodeShockley`, section
1.8, no raised vSat):
- n-JFET: gate-source anode `g`, cathode `s`; gate-drain anode `g`, cathode `d`.
- p-JFET: gate-source anode `s`, cathode `g`; gate-drain anode `d`, cathode `g`.
Each is evaluated at its own junction voltage `V(anode) - V(cathode)` of the
iterate (`MF:357-368`).

Reported current (`MF:374-392`): `channel - pol*gateDrain`, with
`gateDrain = shockleyDiodeCurrent(vGd, isGate, N_DIODE, vt)`,
`vGd = vG - vD` (n) or `vD - vG` (p), default `vSat`; the drain-terminal
current (channel plus the gate-drain junction's contribution at the drain).
The gate-source junction never reaches the drain node and is not reported.

### 5.3 Dynamics and state

None. No `Cgs`/`Cgd` ("unlike the MOSFET model there is no Cgs/Cgd here at
all", `devolt/apps/sim/lib/sim/model-disclosure.ts`), no junction capacitance,
no self-heating, no state.

### 5.4 Newton

No limiter (the comment at `MF:343-345` repeats the diode rationale: both gate
junctions are "current-limited by the surrounding circuit"). Previous iterate,
previous committed voltages as the start.

### 5.5 Regimes and events

As the MOSFET: three channel regions chosen from the iterate, no commit. The
gate junctions are reverse-biased in normal operation (about `1e-12 S`
leakage each) and conduct when the gate is driven about 0.6 V past the channel.

### 5.6 Small-signal AC

`MF:394-432`: channel `gm`, `gds` from `jfetLinearization` (so the GMIN floor
is included) as the six entries, and the two gate junctions'
`shockleyConductanceAtOp + AC_GMIN` between `g,s` and `g,d`. The derivative of
the transient stamp; no disagreement.

### 5.7 Diagnostics

None.

### 5.8 Tests that pin it

`extended-semis.test.ts:203-290` ("njfet — Shichman-Hodges channel physics"):
`idss` at `vGS = 0` in saturation within 1 percent (`:204`), pinch-off leakage
only (`:213`), saturation transfer curve against
`beta*(vGS - vto)^2*(1 + lambda*vDS)` within 1 percent (`:223`), forward gate
drive conducts through the gate junctions (`:244`), common-source gain
`gm*RL` within 2 percent (`:277`). `spice-ngspice-crossval.test.ts:550-581`
case 6: self-biased JFET `.op` against ngspice. `extended-integration.test.ts`
mega-fixture. SPICE import: `spice/netlist.ts:1045-1056, 1119-1125`
(`J` cards give `njfet`; model type `pjf` gives `pjfet`).

### 5.9 Questions

- **Q-W3-23 (shared reverse-conduction defect).** The JFET inherits
  section 4.5's three findings. A JFET is symmetric (source/drain
  interchangeable); classic's reverse mode is not. Same recommendation as
  Q-W3-17.
- **Q-W3-24 (`idss`/`vto` cannot both match a datasheet).** The devolt
  catalog's `model-disclosure` already says that a JFET's typical `gm` and
  typical cut-off cannot both be matched by the square law; the pjfet
  defaults (`vto -4.5`, `idss 0.022`) are a "cross-checked midpoint" for the
  J175 whose datasheet gives no typical values (devolt `model-disclosure.ts`
  `pjfet` entry). **Recommend** none for the port.
- **Q-W3-25 (the six uncatalogued kinds and `catalogUid`).** None of njfet,
  pjfet, scr, triac, opto_npn, analog_switch has a bundled catalog entry, so
  the core's compile layer, which gets its catalog from the same bundle,
  has no `default_params` for them and no `electrical_specs`; devolt's
  catalog (injected at runtime through `setPartLibrary`) has them.
  **Recommend** the compile layer reads literals only (as classic does) and
  PLAN decision 6 (the unnamed kinds into W3, recommended yes) adds them to
  the support table; a catalog sync between brownout's bundle and devolt's
  is a separate item.
- **Q-W3-26 (one `Is` for both gate junctions).** The gate-source and
  gate-drain diodes share `Is` (default 1e-14: a 0.651 V drop at 1 mA, as a
  silicon junction). A real JFET's two junctions are one structure, so this is
  reasonable. **Recommend** none; the golden pins both anchors.

---

## 6. SCR

[W3-F, "latching regimes", needs W3-G.] `scrModel` (`MF:474-591`), kind
`"scr"`. A committed-regime latch: the conducting/blocking decision is made
AFTER the accepted solve from that solve's voltages, and the stamp of the NEXT
solve uses it. No catalog entry in `default-parts.json`
(`devolt:parts.json:2113`).

### 6.1 Terminals and parameters

Pins by id `a` (anode), `k` (cathode), `g` (gate) (`MF:477-480`); fewer than
three pins returns from stamp, commit and AC.

| Parameter | Read at | Clamp | Default | devolt catalog |
|---|---|---|---|---|
| `vgt` (V) | `MF:460` | `max(0.3, .)` | 0.7 | 0.8 |
| `igt` (A) | `MF:461` | `max(1e-6, .)` | 0.005 | 0.0002 |
| `ih` (A) | `MF:466` | `max(1e-6, .)` | 0.005 | 0.005 |
| `vtm` (V, on-state intercept) | `MF:467` | `max(0.1, .)` | 1.2 | 1.25 |
| `ron` (ohm) | `MF:468` | `max(1e-3, .)` | 0.1 | 0.1 |

(`thyristorParams`, `MF:459-472`.) Derived: `isGate =
shockleyIsFromVf(vgt, igt, N_DIODE, Vt)` (so the gate junction passes exactly
`igt` at `vgt`); `vSatGate = raisedJunctionVSat(vgt, N_DIODE*Vt)
= min(max(40*Vt, vgt + 5*Vt), 700*Vt)` (`MF:118-120`). Constant
`THYRISTOR_BLOCKING_G = 1e-9` S (`MF:441`). devolt catalog `i_c_max 0.8` (not
read).

### 6.2 Equations

State `on = (icState.on ?? 0) >= 0.5` (committed at the previous accepted
solve; `defaultIcState("scr") = {on: 0}`, `SE:1672-1679`).
- **Gate junction** (stamped in BOTH regimes, `MF:499-508`): guarded Shockley
  diode anode `g`, cathode `k`, `Is = isGate`, `n = 1`, `Vt`, `vSat = vSatGate`,
  at `vG - vK` of the iterate.
- **Principal path** (`MF:509-515`): blocking: `stampResistor(a, k,
  1/THYRISTOR_BLOCKING_G)` (1 Gohm, current `1e-9*vAK`); conducting:
  `stampResistor(a, k, ron)` plus `stampCurrentSource(a, k, vtm/ron)` (a source
  of `vtm/ron` INTO node `a`, out of `k`), i.e. `i(a->k) = (vAK - vtm)/ron`.
  There is no dependence on the iterate in either regime: the principal stamp
  is linear and fixed for the whole solve.
- No anode-gate path, no gate-to-anode junction, no breakover (blocking holds
  any voltage), no dv/dt triggering (devolt `model-disclosure.ts`, `scr`
  entry).
- Reported current (`MF:558-562`): `capCurrents[id]`, committed in
  `commitState`, below.

### 6.3 Dynamics and state

State: `icState[id] = {on}` (0 or 1), plus the derived readout
`capCurrents[id]`. `icState` is carried across `load()` for a same-id,
same-kind component, is in the step snapshot, and is error-compared EXACTLY by
the step controller (`on` is not in the tolerance table; section 1.5).
`capCurrents` is snapshot-carried but excluded from the error walk
(`MF:540-545` comment). No integration state, no turn-on or turn-off time.

### 6.4 Newton

Only the gate diode depends on the iterate: guarded Shockley, no limiter, the
raised `vSatGate` (section 1.8). The principal branch is linear. So within a
solve the circuit is a diode plus a fixed resistor; the regime cannot flip
inside Newton. At a DC operating point the regime loop `_dcRegimeSettle`
(`SE:3730-3756`) re-solves while the committed `icState` signature
(`SE:3773-3796`, every `icState` slot quantised to 1e-9) changes, up to the
caller's `maxOuterRegimeIters`, then accepts the last converged solve.

### 6.5 Regimes and events

Two regimes, `blocking` and `conducting`, decided in `commitState`
(`MF:517-557`) from the ACCEPTED solution `x`:
1. `vAk = vA - vK`.
2. `iPrincipal = on ? (vAk - vtm)/ron : vAk*1e-9` evaluated under the regime
   the solve was STAMPED with (the entry latch) and stored in
   `capCurrents` (so the published current is the one the stamp carried, and a
   series neighbour sees the same number on a flip step, `MF:536-547`).
3. `iGate = shockleyDiodeCurrent(vG - vK, isGate, N_DIODE, Vt, vSatGate)`;
   `gateDriven = iGate >= igt && (vG - vK) >= vgt` (both are the same
   condition by construction of `isGate`; both asserted).
4. Blocking: becomes conducting iff `gateDriven && vAk > vtm` (strict).
   Conducting: becomes blocking iff `iPrincipal < ih` (strict), UNLESS
   `gateDriven && iPrincipal > 0` (a maintained gate holds it). So a reverse-
   biased on-state (`iPrincipal` negative, which includes every `vAk < vtm`)
   drops at once, which is the commutation rule.
5. `icState.set(id, {on: next})`.

Latency: the stamp that follows the commit uses the new regime, so the
principal current changes one accepted step after the gate reaches the
threshold, and the first conducting step carries the full `(vAk - vtm)/ron`.
Classic never locates the instant. Under the checked step (`AS`), the
exact-compare of `on` between the full step and two half steps rejects and
halves the step until the event time is resolved to the host floor (headless
`H_MIN = 1e-8`, W1 section 10.15), which is bracketing, not root finding.
A conducting SCR with `vAk < vtm` is a one-step 1.25 V source (the Norton
`vtm/ron` term holds the anode at `vtm` above the cathode through `ron`)
before the next commit drops it.

### 6.6 Small-signal AC

`MF:563-591`: `stampAcAdmittance(a, k, on ? 1/ron : 1e-9)` (the Norton offset
differentiates away) plus the gate junction's `shockleyConductanceAtOp(vGK_OP,
isGate, N_DIODE, Vt, vSatGate) + AC_GMIN` between `g` and `k`. It selects the
committed regime exactly as the transient stamp does, so it is the derivative
of the transient stamp: no disagreement.

### 6.7 Diagnostics

None: no failure, no warning, no entry in any kind list. The latching has no
`IL` (latching current), no surge limit.

### 6.8 Tests that pin it

`extended-semis.test.ts:404-680`: the lifecycle "blocks, fires on gate, latches
after gate removal, drops out below ih, re-blocks" (`:405`), bitwise
determinism across two fresh runs (`:436`), no firing at half the rated gate
current (`:442`), firing with an authored `vgt` of 2.5 V above the generic
exponential cap (`:484`), a maintained gate sustaining conduction below `ih`
without chatter (`:524`), series-KCL on flip steps (`:566`),
`dcOperatingPoint` settling the latch bit-identically (`:607-629`), the
conducting small-signal model (`:630-680`), and a same-id kind swap `scr` ->
`triac` starting from the fresh kind's cold latch (`:681`). The census tests
`ac-isolation.test.ts:343-395` and `extended-integration.test.ts:266-312`.

### 6.9 Questions

- **Q-W3-27 (latching versus the datasheet).** `vgt`/`igt` are the datasheet's
  MAXIMUM trigger values (the part is guaranteed to fire at or below them),
  and classic fires exactly there; `ih` likewise a maximum. A real part's
  typical triggers are lower and it also needs the anode current to exceed the
  latching current `IL` (about 1.5 to 2 `IH`, recalled) before the gate may be
  removed; classic has no `IL` and latches on any anode voltage above `vtm`.
  `vtm` is a zero-current intercept where the datasheet quotes a drop at the
  rated current (devolt `model-disclosure.ts`: "the low-current drop is
  overestimated"). **Recommend** reproduce; record the three omissions in the
  disclosure the core carries.
- **Q-W3-28 (find the instant or not).** DK 1.5 puts thyristors in the
  regime-device class with `switching` distances `i_G - I_GT` and
  `i_A - I_H`, cutting the step at the crossing. Classic does not (section
  6.5): the regime changes one accepted step late. Decision 7 (b) (closer to
  the analytic answer) would favour locating the crossing; gate 1 (identity)
  would not. **Recommend** W3-G locates the crossing (DK 1.5) and registers
  the divergence, since the instant the SCR fires moves the whole downstream
  waveform by up to one host step; the lifecycle test asserts phases, not
  times, so it still passes. This is DK's design question restated, not new.
- **Q-W3-29 (the one-step source).** After a flip with `vAk < vtm` the
  conducting stamp is a voltage source of `vtm` through `ron`, not a
  device-off state. In classic that lasts exactly one accepted step; with
  step locating it must last zero. **Recommend** the kernel's conducting
  `eval` is `(vAk - vtm)/ron` as classic, and the regime rule drops it at the
  next commit; a located crossing then makes the point moot.
- **Q-W3-30 (strictness at the boundaries).** `vAk > vtm` and `iPrincipal <
  ih` are strict, `iGate >= igt` and `vGK >= vgt` are not. The port must use
  the same comparison operators (a golden with the gate exactly at `vgt`).
  **Recommend** reproduce exactly.

---

## 7. Triac

[W3-F.] `triacModel` (`MF:593-686`). The SCR's latch made bidirectional. No
bundled catalog entry (`devolt:parts.json:2164`).

### 7.1 Terminals and parameters

Pins by id `mt1`, `mt2`, `g` (`MF:596-600`); the gate is referenced to `mt1`.
Parameters are the SCR's five through the same `thyristorParams`
(`MF:459-472`; same clamps and defaults); devolt catalog values `vgt 1`,
`igt 0.01`, `ih 0.015`, `vtm 1.4`, `ron 0.05` (`i_c_max 4`). `isGate` and
`vSatGate` as the SCR. Blocking conductance `1e-9` S.

### 7.2 Equations

State `{on, pol}` (`defaultIcState("triac") = {on: 0, pol: 1}`), `pol = +1` if
`(st.pol ?? 1) >= 0` else `-1`. `v12 = V(mt1) - V(mt2)`.
- **Gate** (`MF:608-612`): an anti-parallel pair of guarded Shockley diodes
  between `g` and `mt1`: `stampDiodeShockley(g, mt1, vGate, isGate, 1, Vt,
  vSatGate)` and `stampDiodeShockley(mt1, g, -vGate, ...)` with
  `vGate = V(g) - V(mt1)` of the iterate. A bidirectional gate: a real triac
  fires on either gate polarity, with no quadrant asymmetry (devolt
  `model-disclosure.ts` states the Q4 omission).
- **Principal** (`MF:613-621`): blocking: `stampResistor(mt1, mt2, 1e9 ohm)`;
  conducting: `stampResistor(mt1, mt2, ron)` plus `stampCurrentSource(mt1,
  mt2, pol*vtm/ron)`, i.e. `i(mt1->mt2) = (v12 - pol*vtm)/ron`: the `vtm` drop
  always opposes the latched direction.
- Reported current: `capCurrents[id]`, mt1 -> mt2 (`MF:664-668`).

### 7.3 Dynamics and state

`icState = {on, pol}`, both compared exactly by the step controller;
`capCurrents` as the SCR. `pol` is frozen at trigger time and changes only at
the next trigger.

### 7.4 Newton

The two gate diodes only, no limiter, raised `vSatGate`. Principal linear. A
`dcOperatingPoint` settles the latch through the regime loop as the SCR.

### 7.5 Regimes and events

`commitState` (`MF:623-663`):
1. `v12 = V(mt1) - V(mt2)`; `iPrincipal = on ? (v12 - pol*vtm)/ron : v12*1e-9`
   under the stamped regime, into `capCurrents`.
2. `vGate = V(g) - V(mt1)`; `iGate = shockleyDiodeCurrent(vGate, ...) -
   shockleyDiodeCurrent(-vGate, ...)` (the pair's net current);
   `gateDriven = |iGate| >= igt && |vGate| >= vgt`.
3. Blocking: `on` iff `gateDriven && |v12| > vtm`; then `pol = (v12 >= 0 ? 1 :
   -1)`. Conducting: off iff `pol*iPrincipal < ih`, unless `gateDriven &&
   pol*iPrincipal > 0`. A reversal (`pol*iPrincipal <= 0`) always
   commutates, and a still-driven gate re-fires the opposite polarity from the
   blocking state at the next commit once `|v12| > vtm`.
4. `icState.set(id, {on, pol})`.
Same latency and no-instant-location as the SCR (section 6.5). Note that
`pol` is derived from `v12` at the commit that fires it, one step after the
gate threshold is met, so an AC waveform near its zero crossing may fire on
the wrong polarity's half for one step.

### 7.6 Small-signal AC

`MF:669-686`: `1/ron` or `1e-9` between `mt1` and `mt2`; both anti-parallel gate
slopes `shockleyConductanceAtOp(vGate) + shockleyConductanceAtOp(-vGate) +
AC_GMIN` between `g` and `mt1`. Derivative of the transient stamp; no
disagreement.

### 7.7 Diagnostics

None.

### 7.8 Tests that pin it

`extended-semis.test.ts:811-942`: "with a held gate it conducts BOTH mains
polarities with the correct sign" (`:812`), "latches through the half-cycle
after the gate pulse, then commutates at the zero crossing" (`:830`), "a driven
gate cannot latch an unbiased MT pair into a phantom vtm rail" (`:853`);
`ac-isolation.test.ts:391`; `extended-integration.test.ts`.

### 7.9 Questions

- **Q-W3-31 (quadrants, `IGT`, and `IH` from the datasheet).** Classic fires
  at one `igt` in all four quadrants. A BT136-600 (recalled, WeEn/NXP):
  `IGT` 5 to 35 mA by quadrant (Q4 up to 70 mA), `VGT` 1.5 V max, `IH`
  6 mA typical and 15 mA max, `VT` 1.7 V at 5 A. Classic with the devolt
  defaults fires at 10 mA everywhere. Documented omission. **Recommend**
  reproduce.
- **Q-W3-32 (`pol` latched from the commit step's `v12`).** When the gate
  threshold is met just before a zero crossing the polarity latched is the
  sign at the commit. **Recommend** reproduce (W3-G's located crossing makes
  it follow the exact instant; same recommendation as Q-W3-28).
- **Q-W3-33 (the commutation state `pol*iPrincipal < ih`).** Because
  `iPrincipal` includes the `-pol*vtm/ron` offset, the hold boundary is
  `v12 = pol*(vtm + ih*ron)` (1.4 + 0.015 * 0.05 = 1.40075 V for the devolt
  defaults), not zero. A triac on 50 Hz mains therefore drops out `vtm`
  before the zero crossing, as a real one does within its `IH`.
  **Recommend** reproduce.

---

## 8. Optocoupler: opto_npn

[W3-F; PLAN lists it with jfet as depending on W3-T.] `optoNpnModel`
(`MF:719-883`). An input LED, an internal-node phototransistor, and a
photocurrent from the LED's COMMITTED current. No bundled catalog entry; the
devolt part `opto-4n35` is at `devolt:parts.json:8103`.

### 8.1 Terminals and parameters

Pins by id `led_a`, `led_k`, `c`, `e` (`MF:725-736`); fewer than four pins
returns. One INTERNAL solver node `b` (`internalNodes: () => ["b"]`,
`MF:690, 723`), allocated per load after the net rows (W1 section 7.15; DK
1.3 "internal unknowns"); `ctx.internalNode(id, "b")`. The devolt part's
`b` and `nc` pins are footprint pins that reach nothing: the package base is
internal and an external base pin is only a floating net with no stamp
(`model-disclosure.ts`, `opto_npn` excluded list). Galvanic groups
`[led_a, led_k]`, `[c, e, b]`, `[nc]` (`MF:694`) tell the section-anchor logic
that the input and output sides are isolated (W1 delta `bd90847`).

| Parameter | Read at | Clamp | Default | devolt catalog |
|---|---|---|---|---|
| `ctr` (output current per LED current) | `MF:712` | `max(0, .)` | 1.0 | 1 |
| `vf` (LED, V at `iRated`) | `MF:708` through `ctx.junctionVf` | none | 1.2 | 1.2 |
| `iRated` (A) | `MF:709` | none | 0.02 | 0.01 |
| `betaF` | `MF:713` | `max(1, .)` | 100 | 100 |
| `Is` (phototransistor) | `MF:776` | none | 1e-16 | 1e-16 |

`isLed = shockleyIsFromVf(vfLed, iRated, N_LED = 2, Vt)`; `vSatLed =
raisedJunctionVSat(vfLed, N_LED*Vt) = min(max(40*N_LED*Vt, vfLed + 5*N_LED*Vt),
700*N_LED*Vt)` (`MF:118-120, 710-715`). Specs in
the devolt catalog (`vf 1.2`, `if_max 0.05`, `i_c_max 0.05`, `p_max 0.07`)
are not read.

### 8.2 Equations

- **Input LED** (`MF:758-767`): guarded Shockley anode `led_a`, cathode `led_k`,
  `n = 2` (LED), `Is = isLed`, `vSat = vSatLed`.
- **Output transistor** (`MF:769-800`): Ebers-Moll NPN on `(b, c, e)` via
  `stampBJT` with `polarity = 1`, `Is = IsT = saturationCurrentAtTemperature(
  Is, ambient, 1)`, `betaF = p.betaF`, `betaR = 1`, `nF = nR = 1`, `Vt`,
  `earlyVoltage = 0`; the junction laws and Jacobian are section 3.2 with
  those constants. Newton limiting: `pnjlim` on `vBE = vB - vE` and
  `vBC = vB - vC` with `VtN = N_DIODE*Vt`, `vCrit = vCritFor(IsT, VtN)`,
  `prev = ctx.bjtPrevJunction(id) ?? raw` (the same per-id map the BJTs use,
  keyed by this component id); there is no per-load cache for this kind (the
  engine builds it for `bjt_npn`/`bjt_pnp` only), so the values are computed
  per stamp. `markJunctionLimitedThisIteration` is called when either junction
  is limited (`MF:789`).
- **Photocurrent** (`MF:796-800`): `Ip = (ctr/betaF)*max(0, iLedCommitted)`
  with `iLedCommitted = icState.i1`, the LED current of the LAST ACCEPTED solve;
  `stampCurrentSource(b, c, Ip)`: `Ip` enters the base node and is drawn from
  the collector node (the CB photodiode path). In forward-active operation the
  collector collects `betaF*Ip = ctr*iLed`, and the external collector current
  is `ctr*iLed*(1 + 1/betaF)` (one percent above `ctr*iLed` at `betaF = 100`:
  `Ip` is drawn from the collector node as well).
- The matrix has NO entry coupling the LED nodes to the output nodes: the
  isolation is structural.
- Commit (`MF:802-814`): `icState.set(id, {i1: shockleyDiodeCurrent(vLed,
  isLed, N_LED, Vt, vSatLed)})`, the signed LED current at the accepted
  voltages (the `max(0, .)` is taken when the next stamp reads it).
- Reported current (`MF:815-819`): `icState.i1`, LED anode -> cathode.

### 8.3 Dynamics and state

State `icState = {i1}` with a deliberate name: the step controller maps `i1`
to the ABSOLUTE current tolerance (10 nA, section 1.5) instead of the
exact-match rule (comment at `SE:1682-1687`). Plus the internal `b` row's
warm start from `_lastX` (`SE:6676-6702`). There are no capacitances: no LED
or phototransistor response time (a 4N35 has `tr`, `tf` of about 7 to 10 us,
recalled), no dark current.

The ONE-STEP LAG is documented and deliberate (`MF:727-748`): a same-solve
coupling "would add led-side columns to output-side Newton rows and change
the sparse pattern class of every opto circuit". At a DC operating point
`i1` settles through the regime loop (`SE:3730-3756`; a continuous slot
quantised to 1e-9 in the signature).

### 8.4 Newton

LED: none (guarded Shockley). Transistor: pnjlim as the BJT. The photocurrent
is a constant per solve (from the committed `i1`).

### 8.5 Regimes and events

No discrete regime; saturation is the transistor's. The coupling from the LED
to the output is one accepted step late, so an input edge reaches the
collector one step later (up to the host's step, 10 us at the usual
transient; the step controller does not see a lag because `i1` differs by
less than its tolerance only once it settles).

### 8.6 Small-signal AC

`MF:820-883`: LED `shockleyConductanceAtOp(vLed_OP, isLed, N_LED, Vt, vSatLed)
+ AC_GMIN` between `led_a` and `led_k`; the nine Ebers-Moll entries on
`(b, c, e)` from `bjtLinearization` at the OP voltages (`Is` scaled, Early 0);
and the coupling `gp = (ctr/betaF)*gdLed`: `G[b][led_a] -= gp`,
`G[b][led_k] += gp`, `G[c][led_a] += gp`, `G[c][led_k] -= gp`
(`MF:858-882`). **This is not the derivative of the transient stamp**: the
transient has NO cross-isolation entries (the photocurrent is the lagged
`i1`), so its Jacobian has `gp = 0`. The AC stamp linearises the
instantaneous idealisation, justified by the DC settle placing `i1` at its
fixed point (comment `MF:830-848`). The `max(0, .)` clamp on a reverse LED is
also dropped.

### 8.7 Diagnostics

None.

### 8.8 Tests that pin it

`extended-semis.test.ts:943-1017`: `ctr*iLed` to the collector within 5
percent at 10 mA drive (`:944`), dark with the LED unpowered (`:958`), no DC
input-output path, "a 100 V common-mode tie carries only shunt leakage"
(`:968`); `isolated-sections.test.ts` (galvanic groups and section anchors);
`ac-isolation.test.ts` and `extended-integration.test.ts` (census, mega
fixture, dense/sparse parity); `ac-small-signal.test.ts` does not exercise it.

### 8.9 Questions

- **Q-W3-34 (AC disagrees with transient by design).** Section 8.6. A kernel
  that derives AC from `eval` (DK 1.4) would produce `gp = 0` unless the
  kernel's `eval` reads the LED current in the same solve. **Recommend**
  record an override: the opto kernel's AC stamp is hand-written as classic's,
  and note that the transient port may also couple in-solve if W3-N allows an
  LED-to-output coupling (it would remove the lag and change every opto
  trajectory by one step: a divergence to register, not default).
- **Q-W3-35 (one-step lag).** Keep (identity with classic) or couple in-solve
  (the physics: no lag). **Recommend** keep for W3-F gate 1; decide separately
  with the `ctr` units below.
- **Q-W3-36 (`ctr` is the datasheet minimum, fixed).** 4N35 (recalled): CTR
  100 percent minimum at `IF = 10 mA`, `VCE = 10 V`; typical much higher and
  falling with `IF` and temperature; classic uses one number at all currents.
  Documented. **Recommend** reproduce.
- **Q-W3-37 (`vf`, `iRated` vs the catalog's `if_max`).** The kind reads
  `iRated` (default 0.02) and ignores `electrical_specs.if_max`. The devolt
  part carries `iRated 0.01`. Anchoring the LED at `iRated` makes `vf` the drop
  at 10 mA for the catalog part and at 20 mA for a bare part, a 35 mV
  difference at `n = 2`. **Recommend** reproduce.
- **Q-W3-38 (internal node convergence).** The `b` row is convergence-tested
  like every unknown (DK 1.3: "convergence-tested, `n` grows by `INTERNAL`").
  Classic's start value for it is `_lastX`. **Recommend** the kernel
  reproduces both.

---

## 9. Analog switch: analog_switch

[W3-F.] `analogSwitchModel` (`MF:1135-1190`). A transmission gate with a
committed control level. No bundled catalog entry (`devolt:parts.json:2215`:
`analog-switch`, `ron 100`, `roff 1e9`, no specs).

### 9.1 Terminals and parameters

Pins by id `a`, `b`, `ctrl` (`MF:1140-1150`); fewer than three pins returns.
No supply pins: the switch is "self-contained", the control level is
ground-referenced by construction (comment `MF:1113-1119`).

| Parameter | Read at | Rule | Default |
|---|---|---|---|
| `ron` (ohm) | `MF:1126` | `max(1e-3, .)` | 100 |
| `roff` (ohm) | `MF:1123-1132` | `roff > 0 ? max(1, roff) : 1e9` (zero, negative or NaN falls back to the default) | 1e9 |

Constants: `ANALOG_SWITCH_VIH = 2.0` V, `ANALOG_SWITCH_VIL = 0.8` V
(`MF:1120-1121`), not authorable.

### 9.2 Equations

`on = (icState.on ?? 0) >= 0.5` (default 0, `defaultIcState("analog_switch")
= {on: 0}`). Stamp: `stampResistor(a, b, on ? ron : roff)` (`MF:1137-1152`);
the `ctrl` pin has no stamp (a pure sense input). Reported channel current
`capCurrents[id] = (V(a) - V(b))/(prev ? ron : roff)` under the regime the solve
was stamped with (`MF:1153-1171`), a -> b.

### 9.3 Dynamics and state

`icState = {on}` and `capCurrents`. No charge injection, no switching time, no
feedthrough, no resistance variation with signal level.

### 9.4 Newton

None: the stamp is a fixed resistor per solve.

### 9.5 Regimes and events

Committed after the solve (`MF:1164-1169`): with `vCtrl = V(ctrl)` (an ABSOLUTE
node voltage, not referenced to anything): `vCtrl >= 2.0` gives `on = true`,
`vCtrl <= 0.8` gives `on = false`, in between keeps `prev`. A floating
`ctrl` pin reads about 0 V through the node shunt and is off. The resistance
changes at the NEXT solve, one accepted step after the control crosses a
threshold; the instant is not located (bracketed by the step controller's exact
compare of `on`, section 1.5).

### 9.6 Small-signal AC

`MF:1177-1190`: `1/(on ? ron : roff)` between `a` and `b`: the transient stamp
exactly. No disagreement.

### 9.7 Diagnostics and tests

None. `extended-semis.test.ts:1018-1075`: ctrl high passes the divider
through `ron` within 1 percent (`:1019`), ctrl low isolates (`:1032`), "an
authored roff of 0 fails toward isolation" (`:1043`), a mid-run ctrl flip
replays bitwise after a snapshot rollback (`:1059`).

### 9.8 Questions

- **Q-W3-39 (fixed TTL thresholds, no supply).** Devolt's
  `model-disclosure.ts` already states the model "cannot represent a switch
  whose threshold tracks its supply" (CD4066B thresholds scale with `VDD`;
  74HC4066 `VIL`/`VIH` are 0.3/0.7 of `VCC`; recalled). A 3.3 V part driven
  from a 3.3 V logic pin reaches 3.3 V >= 2.0, so it works; a 1.8 V logic does
  not turn it on. **Recommend** reproduce; if a supplied variant is wanted it
  is a new kind, not a change.
- **Q-W3-40 (`ron` vs datasheet).** CD4066B `RON` about 470 ohm at `VDD =
  5 V`, 180 ohm at 10 V, 125 ohm at 15 V (TI, recalled), 74HC4066 about 40 ohm
  at 4.5 V; classic's 100 ohm is a mid value and does not vary with signal
  level. **Recommend** none.
- **Q-W3-41 (event timing of the control).** One accepted step of lag
  between the control crossing and the resistance change. For a comparator
  or clock driving the control that is up to one host step (10 us at the usual
  step). **Recommend** in W3-G the located crossing (as Q-W3-28) when `ctrl`
  is driven by a partition-boundary event; otherwise reproduce the lag.

---

## 10. Op-amps: lm358, mcp6002, lm386

[W3-O; DK 2.3 recommends the in-partition form.] Two model objects:
`dualOpAmpModel` for `lm358` and `mcp6002` (`AR:759-1017`, two units per
package) and `lm386Model` for `lm386` (`AR:1019-1174`, one unit). They share
the parameter reader `opAmpModelParams` (`AR:114-137`), the step-target
helpers (`AR:144-258`), the supply-return and quiescent helpers
(`AR:267-299`) and the AC unit stamp (`AR:380-459`). The electrical core is
`stampOpAmp` (`EL:1495-1554`) and the pole helpers (`EL:1349-1446`).

### 10.1 Terminals and parameters

**Pins** (fixed catalog ids, `AR:800-806, 855-860, 1052-1056`):

| Kind | Unit | Out | In- | In+ | Supply |
|---|---|---|---|---|---|
| lm358, mcp6002 | 1 | `1` | `2` | `3` | V- `4` (gnd), V+ `8` (vcc) |
| lm358, mcp6002 | 2 | `7` | `6` | `5` | same package pins |
| lm386 | 1 | `5` | `2` | `3` | gnd `4`, vcc `6`; pins `1`, `7`, `8` passive and NOT read |

Supply pins come from `supplyPinsForComponent` (catalog `vcc`/`gnd`
functions). Each unit owns one MNA branch row, keyed `<id>:op0`, `<id>:op1`
(`branchRows`, `AR:764, 1022`), always allocated so the matrix is never
singular even when the part is unpowered.

**Parameters** (`opAmpModelParams`, `AR:114-137`; defaults `AR:75-107`;
`nonNegativeParam` returns the fallback for a non-finite or negative value,
`AR:109-112`):

| Parameter | lm358 | mcp6002 | lm386 | Rule |
|---|---|---|---|---|
| `openLoopGain` A0 | 1e5 | 1e5 | (see `gain`) | `max(1, nonNeg(.))`; lm386 uses the fixed gain |
| `gain` (lm386 only) | | | 20 | `max(1, Number(params.gain ?? 20))` (`AR:1034`) |
| `gbw` (Hz) | 1e6 | 1e6 | 6e6 | `nonNeg`; **0 disables the pole** (flat VCVS) |
| `slewRate` (V/s) | 3e5 | 6e5 | 3e5 | `nonNeg`; **0 disables the slew limiter** |
| `sourceCurrentLimit` (A) | 0.04 | 0.023 | 0.25 | `nonNeg`; **0 disables** the source limit |
| `sinkCurrentLimit` (A) | 0.02 | 0.023 | 0.25 | `nonNeg`; **0 disables** the sink limit |
| `outputResistance` (ohm) | 50 | 50 | 2 | `max(0.01, nonNeg(.))` |

Catalog (`lm358` CAT:6469, `mcp6002` CAT:6573, `lm386` CAT:6677)
`default_params` carry the same numbers as the table, with the lm386's `gain
20, gbw 6e6, slewRate 3e5, source/sinkCurrentLimit 0.25, outputResistance 2`.
Specs: `vcc_range` lm358 `{3, 5, 32}`, mcp6002 `{1.8, 5, 6}`, lm386
`{4, 6, 12}`; `quiescent_current_a` 0.0007, 0.0002, 0.004; `p_max` 0.83, 1.18,
1.25 W.

**Power** (`ctx.icPowerInfo`, W1 section 1.3, `SE:6035-6090`): the part is
`powered` when both supply pins are wired (not open, no `_noSupplyReturnIds`
entry) and `0.9*min <= vSupply <= 1.1*max` of `vcc_range`: lm358 2.7 to
35.2 V, mcp6002 1.62 to 6.6 V, lm386 3.6 to 13.2 V. `power.vcc` and
`power.gnd` are the node voltages of the supply pins at the iterate (the
package ground need not be 0 V).

**Output limits** (absolute node volts): `vHigh = vcc - 1.5` (lm358),
`vcc - 0.02` (mcp6002) (`AR:777-781`), `vcc - 0.5` (lm386, `AR:1043`);
`vLow = gnd + 0.02` (lm358, mcp6002), `gnd + 0.5` (lm386). The lm386's
`vOffset = vmid = (vcc + gnd)/2` (`AR:1040`); the dual's `vOffset = 0`.

### 10.2 Equations

All per unit. `u` is the dominant-pole ("internal gain stage") voltage;
`uPrev` its committed value (section 10.3); `vd = V(in+) - V(in-)` from the
iterate; `h` the step.

1. **Pole companion** (`opAmpDominantPoleCompanion`, `EL:1349-1374`):
   `gain = max(1e-12, |A0|)`, `poleHz = gbw/gain`; if `gbw <= 0` or `h <= 0`:
   `{dGain = A0, hist = 0, off = 1}` (the instantaneous VCVS); else
   `lambda = 2*pi*poleHz*h`, `denom = 1 + lambda`, `dGain = (lambda/denom)*A0`,
   `hist = 1/denom`, `off = lambda/denom`. This is the backward-Euler step of
   `du/dt = wp*(A0*vd + vOffset - u)` with `wp = 2*pi*gbw/A0`:
   `u = hist*uPrev + dGain*vd + off*vOffset`. At the LM358's 10 Hz pole and
   `h = 1 us`: `lambda = 6.283e-5`, `dGain = 6.28`, `hist = 0.99994`.
2. **Unconstrained target** (`opAmpTransientTarget`, `EL:1377-1389`) is that
   expression with the iterate's `vd`.
3. **Slew limit** (`opAmpSlewLimitedTarget`, `EL:1392-1407`): if `slewRate > 0`
   and `h > 0`: `uPrev + clamp(target - uPrev, -SR*h, +SR*h)`; else unchanged.
4. **Rail bound**: `bounded = clamp(slewLimited, vLow, vHigh)`
   (`opAmpStepTarget`, `AR:144-174`). `fixedTarget = bounded` if
   `|bounded - unconstrained| > 1e-12*max(1, |unconstrained|, |bounded|)`,
   else `null`. Slew is applied BEFORE the rail bound.
5. **Newton active set** (`opAmpNewtonFixedTarget`, `AR:183-200`): the
   clamp is admitted only when the iterate's implied pole voltage
   `internalGuess = V_out_guess - Rout*x[k]_guess` lies on either
   branch's own solution: `tol = 1e-8*max(1, |unconstrained|, |bounded|,
   |internalGuess|)`; active iff `|internalGuess - unconstrained| <= tol` or
   `|internalGuess - bounded| <= tol`. Otherwise the stamp stays in the
   unconstrained (voltage-operation) form. Unpowered:
   `fixedTarget = power.gnd` regardless.
6. **Current constraint** (`opAmpCurrentConstraint`, `AR:218-258`), `null` if
   unpowered. Candidate mode from `opAmpCurrentLimitMode(committedLimit,
   x[k]_guess, srcLimit, snkLimit)` (`EL:1417-1446`): non-finite gives 0;
   from mode -1 stay while `natural <= -0.98*src` (`src > 0`); from mode +1 stay
   while `natural >= 0.98*snk`; else enter -1 if `natural <= -src` (inclusive)
   or +1 if `natural >= snk`; else 0. Then admissibility, tolerance
   `1e-8*max(1, |Vout_guess|, |compliance|)`:
   mode -1: `constraint = -src`, `compliance = bounded + Rout*constraint`,
   admissible iff `Vout_guess <= compliance + tol`;
   mode +1: `constraint = +snk`, `compliance = bounded + Rout*constraint`,
   admissible iff `Vout_guess >= compliance - tol`.
   `naturalCurrent` uses `stampOpAmp`'s sign: `x[k]` is current entering the
   output stage, NEGATIVE while sourcing.
7. **Stamp** (`stampOpAmp`, `EL:1495-1554`), unit row `k`, always `add(out, k,
   +1)`:
   - current constraint active: `add(k, k, 1)`, `b[k] = constraint` (the row
     pins `x[k]`), nothing else.
   - else `add(k, out, 1)` and, if `Rout > 0`, `add(k, k, -Rout)`; then
     - saturated (`fixedTarget` non-null): `b[k] = fixedTarget` (inputs
       uncoupled; the absolute voltage is passed, not re-derived through the
       gain);
     - else linear with `transient` (always, here): `add(k, in+, -dGain)`,
       `add(k, in-, +dGain)`, `b[k] = hist*uPrev + off*vOffset`.
   The linear row reads `V_out = Rout*x[k] + dGain*vd + hist*uPrev +
   off*vOffset`: with `x[k] < 0` while sourcing, `V_out = u_target - Rout*|I|`.
8. **Supply return** (`stampOpAmpSupplyReturn`, `AR:267-282`): when powered
   with both supply pins, `railPin = committedCurrent > 0 ? gnd : vcc` and
   `add(railNode, k, -1)`; `committedCurrent` is `currentConstraint` if one is
   active, else the last accepted `i1`/`i2`. The output current is debited from
   V+ when sourcing and credited to V- when sinking, with the sign of the last
   ACCEPTED step in voltage operation.
9. **Quiescent current** (`stampIcQuiescentCurrent`, `AR:285-299`): when
   powered, `analogIcQuiescentCurrent(comp)` (`SE:1707-1710`, the catalog's
   `quiescent_current_a`, per package, not per unit) is a constant current load
   from V+ to V-: `stampCurrentSource(gndNode, vccNode, I)`.
10. **Seed rule** (`AR:788-792, 1037-1039`): `h <= 1e-12` AND the pole
    state non-finite (`u1` and `u2` for the dual, `u1` for the lm386) swaps
    `gbwHz` and `slewRate` to 0 in `stepModel` (instantaneous VCVS without slew),
    so the load seed initialises the operating point directly instead of
    slewing from zero. A live reload carries finite `u1/u2` and keeps full
    dynamics.
11. **Element current** (`AR:988-999`, `1150-1158`): `x[k]` of unit 1 only,
    the unit-2 current is not published (`getElementChannelI` has no op-amp
    entry). Negative while sourcing.

Input pins draw no current (no `Ib`, no `Ios`, no input capacitance).

### 10.3 Dynamics and state

`icState[id]`: dual `{reg1, d1, u1, limit1, i1, reg2, d2, u2, limit2, i2,
supplyV}`; lm386 `{reg1, d1, u1, limit1, i1, supplyV}`
(`defaultIcState`, `SE:1612-1622`; `NaN` for `d*`, `u*` initially). The commit
(`AR:910-987, 1106-1149`) REPLACES the whole record each accepted solve:
- `d` = `V(in+) - V(in-)` accepted;
- `u` = `!powered ? power.gnd : currentConstraint !== null ? drive.bounded :
  V_out - Rout*x[k]` (the settled pole voltage), where `drive` and the
  constraint are recomputed from the ACCEPTED `x`, `uPrev` and `st.limit`;
- `limit` = 0 / -1 (source) / +1 (sink) from the constraint's sign;
- `i` = `x[k]`;
- `reg` = `u >= vHigh ? 1 : u <= vLow ? 2 : 0` (used only by the AC stamp);
- `supplyV` = `power.vSupply`.
`uPrev` at the stamp is `finite(u) ? u : power.gnd` (dual: a fresh part's pole
starts at ITS GROUND, not at 0 V or `vmid`) and `finite(u) ? u : vmid` for the
lm386. Integration is backward Euler in closed form (item 1); the pole is NOT
an MNA unknown: it is a committed state advanced algebraically, with the
output stage's row solving it implicitly through `dGain`. The step controller
compares `u*`, `d*`, `supplyV` at 100 uV, `i*` at 10 nA, and `reg*`, `limit*`
exactly (section 1.5).

At a DC operating point the pole of a mid-slew amplifier moves `SR*1e-12 s` per
re-solve, i.e. hundreds of 1e-9 quanta, so the regime loop's signature
(`SE:3773-3796`) never rests and `_dcRegimeSettle` takes its accept-the-last-
converged exit (comment `SE:3762-3771`): "the OP is the resting point GIVEN the
carried device state, poles included".

### 10.4 Newton

No limiter and no per-iteration damping; convergence over every unknown
including `x[k]` at `1e-6 + 1e-3*max`. The three clamps (rail, slew, current)
are active-set decisions re-made from the iterate at every stamp
(items 5 and 6), with the hysteresis only on the current mode (0.98) and none
on rail or slew. Start values: previous node voltages; `x[k]` from the previous
solution's row. Feedback loops close inside one Newton solve through the matrix
(loop gain up to 1e5), which is why DK 2.3 recommends the in-partition form
(decision 5 (a)).

### 10.5 Regimes and events

Five operating states per unit, none committed as a discrete event except the
current mode and `reg`:
- **linear** (pole): the BE row of item 7.
- **slew-limited**: `|target - uPrev| > SR*h`: the output moves exactly
  `SR*h` per step (a fixed target, `bounded` from the slew-limited value).
  A step is the unit of slew: at `h = 1 us`, 0.3 V.
- **rail** high or low: `fixedTarget = vHigh | vLow`.
- **current-limited** source or sink: row `x[k] = -src | +snk`; the output
  voltage falls out of KCL; hysteresis band 0.98.
- **unpowered**: output pinned to `power.gnd`, no quiescent draw, no rail
  routing.
The slew/rail decision depends on the iterate and needs at least two Newton
iterations (item 5); the current-mode decision needs the iterate's `x[k]` and
the committed mode. Classic never locates the instant a clamp is entered or
left; the step controller rejects a step whose coarse and refined `limit*` or
`reg*` disagree, and recovers by halving down to the host's floor. Recovery
from a rail "relaxes to linear, never jumps to the opposite rail" because the
slew limit applies before the rail bound (comment `AR:139-143`).

### 10.6 Small-signal AC

`acStampOpAmpUnit` (`AR:380-459`) per unit, from the COMMITTED `reg`, `limit`
and `i` slots (so it needs a prior transient or `dcOperatingPoint` commit):
`add(out, k, 1)`; if powered, `add(railNode, k, -1)` with the rail chosen by
the committed current sign; then
- powered and `limit != 0`: `add(k, k, 1)` only (`dI = 0`: no admittance, no
  drive);
- else `add(k, out, 1)` and `add(k, k, -Rout)`;
  - unpowered: `add(k, gnd, -1)`;
  - `reg = 1`: `add(k, vcc, -1)`; `reg = 2`: `add(k, gnd, -1)` (the pinned stage
    tracks its rail node with derivative 1; the inputs are uncoupled);
  - linear: `A(jw) = A0/(1 + jw/wp)`, `wp = 2*pi*gbw/A0` (`gbw <= 0` gives the
    flat `A0`): `ratio = w/wp`, `aRe = A0/(1 + ratio^2)`, `aIm = -A0*ratio/(1 +
    ratio^2)`; `add(k, in+, -aRe, -aIm)`, `add(k, in-, +aRe, +aIm)`; for the lm386
    (`midSupplyBias`) additionally `add(k, vcc, -aRe/(2*A0), -aIm/(2*A0))` and the
    same on `gnd`.
The slew limiter has no small-signal contribution; the quiescent source is
constant.

**Where it disagrees with the transient stamp.** The transient is the BE
recurrence (`hist`, `dGain` depend on `h`); AC is the exact continuous pole.
They agree only for `w*h` small: for the LM358 follower at 100 kHz, `|A|` is
10.00 continuous and 10.17 for the BE recurrence at `h = 1 us` (10.002 at
`h = 100 ns`) (computed). A stamp derived from the discrete kernel (DK 1.4)
would carry that `h`-dependent error into a frequency response that classic
computes exactly. Also, the AC stamp uses `A0/(1 + jw/wp)` for the
instantaneous case `gbw = 0` as the flat gain, matching the transient.

### 10.7 Diagnostics

No engine warning or failure of its own. The failure pass includes these
kinds (they have `vcc_range`), but `_drivenOutputTargets` has no case for them
(`SE:7800-8004`), so no `output_sag`. Package thermal (exact uid only,
section 1.7): `power = vSupply*Iq + sum over units of (-i)*(vcc - vout)` if
sourcing or `i*(vout - gnd)` if sinking; the profiles have no shutdown, so
the only product is `warnings` text and `allowedPowerW`. `rail_ic_power_chatter`
applies to supply-gated kinds only (`SUPPLY_GATED_KINDS`), which excludes the
op-amps.

### 10.8 Tests that pin it

- `wave4-opamps.test.ts`: catalog (`:36-70`), LM358 follower (`:72`: 2.0 V
  within 10 mV), gain 2 (`:126`), inverting with mid-rail reference (`:180`),
  saturation at `vcc - 1.5` (`:241`) and MCP6002 at `vcc - 0.02` (`:289`),
  MCP6002 follower at 1.8 V (`:323`), LM386 DC bias (`:365`) and its
  saturation to `VS - 0.5` (`:434`), an unpowered op-amp stays non-singular
  (`:465`), unit independence (`:505`), accuracy at A = 1e5 (`:558`),
  recovery from saturation (`:596`).
- `opamp-dynamics.test.ts`: the BE recurrence satisfies the ODE exactly
  (`:115`); slew limit and current-limit hysteresis (`:133`); real-sine
  closed-loop bandwidth, flat below GBW and -3 dB at unity (`:145`); slew and
  rollback (`:195`); source/sink current limiting, same-solve entry on a hard
  load (`:225`), caps (`:276, 293`), recovery in the solve a load is removed
  (`:320`); output energy returned to the rails (`:347`, `:360`).
- `ac-small-signal.test.ts:626-786`: inverting amplifier flat at `-R2/R1`,
  roll-off at `GBW/noise gain` within 5 percent (`:639`); zero AC gain at a
  rail-saturated OP (`:740`); rail ripple through the pinned stage at
  `RL/(RL + Rout)` (`:766`).
- `physics-reference-benchmarks.test.ts:775`: op-amp output and quiescent
  current withdrawn from the supply rail.
- `dc-homotopy-pathological.test.ts:364-428`: LM358 open-loop comparator at
  microvolt offsets lands at the rail without regime oscillation.
- `thermal-engine-physics.test.ts`: exact-profile heating of analog ICs.

### 10.9 Questions

- **Q-W3-42 (pole form: in-partition internal state).** DK 1.5 and PLAN
  decision 5 (a). Classic's pole is NOT an unknown: it is a committed state
  folded into the output row's right-hand side (`hist*uPrev`) and gain
  (`dGain`). A kernel with a `charge` on an internal node `p` (DK 1.5,
  "stateful kernels") would add one unknown per unit and make the pole
  convergence-tested; classic's `x[k]` row is the only unknown. They are the
  same discrete equation (`u = V_out - Rout*x[k]`), so **recommend** the port
  either keeps classic's state-in-RHS (identical floats) or adds the unknown
  with BE `q = Cp*v`, `Cp = 1/(2*pi*poleHz*Rp)` chosen to reproduce `hist` and
  `dGain`; DK's form is a re-derivation and must be proven bit-comparable
  against `opamp-dynamics.test.ts:115`.
- **Q-W3-43 (the clamps' non-smooth active set).** Items 5 and 6 are
  heuristics tuned to the Newton path (admit the unconstrained solution first;
  hysteresis 0.98; compliance tolerance 1e-8). A kernel with regime `eval`s
  (DK 1.5, up to 8 flips) replaces the heuristics with a proper active-set
  loop; results differ at the clamp boundaries within solver tolerance but not
  in the converged state. **Recommend** port the proper loop and register it as
  a divergence only if a gate-2 case moves.
- **Q-W3-44 (what the instant of a clamp entry means).** Section 10.5. A
  located crossing (DK 1.5 `switching` distances: `vd*A0 + ... - vHigh`) cuts
  the step at the exact time the output reaches the rail; classic steps over
  it and the controller brackets. **Recommend** locate (decision 7 (b)).
- **Q-W3-45 (AC from the discrete kernel).** Section 10.6 and cross-cutting
  finding 8. **Recommend** keep a recorded override: AC uses the continuous
  `A(jw)`; DK 1.4 allows exactly this ("a recorded override").
- **Q-W3-46 (op-amp seed rule).** `h <= 1e-12` AND non-finite poles gives the
  instantaneous VCVS: the first accepted state is the DC solution with no
  dynamics. **Recommend** reproduce exactly (it is what makes `dcOperatingPoint`
  and AC start at the operating point).
- **Q-W3-47 (LM358 relaxation oscillator does not start; PLAN BL:3854).**
  Classic's relaxation oscillator on an LM358 stays at its seed in both
  engines (backlog item, `brownout-core-v2.md:3870`, `:4284`: "pre-existing
  op-amp seed quirk"). The cause is not established here. From the code the
  candidates are: (a) no input offset or bias current, so a perfectly
  symmetric start (`vd = 0`, capacitor at 0 V, output on the DC solution) has
  no perturbation to leave the metastable point; (b) the seed's pole start at
  the package ground with `vLow = gnd + 0.02` pinning the first output at
  20 mV, which a small feedback fraction may keep below the capacitor
  voltage. A real op-amp has a few mV of offset (LM358 `VIO` 2 mV typ, 7 mV
  max, recalled) that starts it.
  **Recommend** a probe in W3-O before port (one engine, `vitest --maxWorkers=1`
  on a minimal oscillator) and a decision between "reproduce (no start)" and
  "add a start rule: an input offset of 1 uV" under decision 7 (b); reproduce
  until the probe says which.
- **Q-W3-48 (swing and limits vs the datasheets, recalled).** LM358 (TI):
  output high `V+ - 1.5 V` at 2 mA load (classic exact; `V+ - 2 V` max),
  output low 5 mV typ at no load (classic `gnd + 20 mV`); output source 40 mA
  typ and sink 20 mA typ at the short-circuit limit (classic exact);
  slew 0.3 V/us (classic exact), GBW 0.7 to 1.1 MHz, unity-gain (classic
  1 MHz); input common-mode `0` to `V+ - 1.5 V`, not modelled; `VIO` 2 mV,
  `IB` 20 to 45 nA, not modelled. MCP6002 (Microchip): GBW 1 MHz, slew
  0.6 V/us, output swing within 25 mV of the rails at 10 kohm, source/sink
  23 mA at 5.5 V: classic's `vcc - 0.02` and 23 mA are in line. LM386-N:
  gain 20 with BW 300 kHz at `V+ = 6 V` (classic exact by construction),
  quiescent 4 mA at 6 V (classic exact), output swing within about 0.5 to
  1 V of the rails at 250 mA (classic 0.5 V). **Recommend** none; these are
  catalog numbers chosen from those datasheets.
- **Q-W3-49 (lm386 gain pins and input impedance).** The catalog notes that
  the gain is set by pins 1/8 (open = 20, 1.2 kohm = 50, capacitor = 200) but
  the model reads only `params.gain`; pins `1`, `7`, `8` are floating nets
  with no stamp, and the 50 kohm input impedance to ground and the internal
  1.35 kohm feedback network are absent. **Recommend** reproduce.
- **Q-W3-50 (supply-return sign lag).** In voltage operation the rail debited
  uses the LAST ACCEPTED current sign, so on the step where the output current
  changes sign the wrong supply pin carries it. KCL still holds (the branch
  couples to both `out` and one rail) but the supply-current split between V+
  and V- is a step stale. **Recommend** reproduce for W3-O identity; the kernel
  form decides the rail inside the solve if it can.

---

## 11. Comparator: lm393

[W3-C; needs W3-G.] `lm393Model` (`AR:1176-1275`). A dual open-collector
comparator modelled as a committed two-state output per unit; NOT a
finite-gain element and NOT a VCVS (no branch row).

### 11.1 Terminals and parameters

Pins: unit 1 `IN1+ = 3`, `IN1- = 2`, `OUT1 = 1`; unit 2 `IN2+ = 5`,
`IN2- = 6`, `OUT2 = 7`; `4` gnd; `8` vcc (catalog functions
`open_collector` for 1 and 7, `analog_in` for 2, 3, 5, 6). No params. Catalog
specs (CAT:6781): `vcc_range {2, 5, 36}`, `io_max 0.016`, no `default_params`.

Literals: `LM393_HYST = 0.001` V (`AR:1218`).

Power: `icPowerInfo` (section 10.1): powered for `vSupply` in `[1.8, 39.6]` V,
both supply pins wired. The output resistance of a sinking unit is
`power.outputResistance = max(50, min(1000, vSupply/(io_max*4)))`
(`SE:6071-6075`) = `vSupply/0.064` clamped: 50 ohm at 3.2 V and below,
78.125 ohm at 5 V, 187.5 ohm at 12 V, 562.5 ohm at 36 V. It is a function of
the iterate's supply voltage.

### 11.2 Equations

- **Stamp** (`AR:1195-1210`): returns if `!powered` (read from `xGuess`).
  `st = icState ?? defaultIcState("lm393") = {out1: NaN, out2: NaN}`. For
  each unit, if `st.outN === 0` (committed sinking):
  `ctx.stampDigitalOutput(comp, outPin, false, power)`
  (`SE:6299-6320`): if the gnd pin is wired, `stampResistor(outNode,
  gndPinNode, Rout)`, else the supply-less Norton fallback
  `g = 1/Rout` to `power.gnd + 0`. A released unit (`out = 1` or `NaN`) stamps
  nothing: high impedance, the external pull-up defines the net.
- **Element current** (`AR:1236-1247`): unit 1 only, when sinking and powered:
  `(V_out1 - power.gnd)/Rout`, positive into the output pin; else 0.
- No stamp from the input pins: the inputs draw no current and are sensed only
  at commit.

### 11.3 Dynamics and state

`icState = {out1, out2}` (1 = released, 0 = sinking, `NaN` = uninitialised,
treated as released). No propagation delay (the datasheet's 1.3 us typical
response, recalled), no slew, no input offset or bias, no output saturation
voltage (only the `Rout` drop).

### 11.4 Newton

None for the comparator: the stamp is a fixed resistor per solve given the
committed state; the inputs do not enter the matrix. Supply gating can chatter
across iterates when the part's own draw pulls its supply through the power
window (the `power-off-stall` machinery of W1 section 1.3: `lm393` is in
`SUPPLY_GATED_KINDS`, `SE:1383-1389`).

### 11.5 Regimes and events

Per unit, decided in `commitState` (`AR:1212-1235`) from the ACCEPTED `x`:
unpowered gives `{out1: 1, out2: 1}` for both; else `d1 = V(3) - V(2)`
(`d2 = V(5) - V(6)`): `d > +1 mV` gives 1 (released); `d < -1 mV` gives 0
(sinking); inside the band keeps the previous value (NaN keeps 1). A
`+-1 mV` hysteresis around `IN+ = IN-`. The new state is stamped from the NEXT
solve: **one accepted step of latency**, and the crossing instant is not
located (the step controller compares `out1`/`out2` exactly, section 1.5, so
a flip between the coarse and refined trial rejects the step and halves it).
The output cannot slew through intermediate values: it is binary.

**Conclusion for DK question 6 / PLAN decision 5.** Classic's comparator is a
committed latch with a one-step lag, so the cross-partition driver-commit form
of DK 1.5 (output stage in the output's partition, inputs watched, the switch
a driver commit) is the natural port: the output stage stamp is `Rout` to
V-, and the only coupling from input to output is the committed bit.

### 11.6 Small-signal AC

`AR:1248-1274`: unpowered: nothing; else for each sinking unit the output
conductance `1/Rout` between the output node and the gnd pin (a local
Norton self-conductance if the gnd pin is open, `AR:1266-1270`); no
input-to-output path; released stamps nothing. It is the derivative of the
transient stamp (the decision is a committed latch, so there is nothing to
derive from the input). `ac-small-signal.test.ts:838` ("a released
open-collector output stays hi-Z in the AC system").

### 11.7 Diagnostics

`output_sag` (the only W3 kind that contributes): `_drivenOutputTargets`
(`SE:7836-7845`) returns the SINKING units as `{pinId, high: false}`; the
failure pass (`SE:7570-7660`) compares `V_out - power.gnd` with
`vil = thresholdsFor(specs, vSupply).vil = max(0.8, 0.3*vSupply)` (the lm393
has no `logic_family` or `v_input`) and latches `output_sag` after two
consecutive steps (`OUTPUT_SAG_DEBOUNCE_STEPS = 2`, with the
direction-signed counter), clearing it when the unit releases (W1 delta
`d1dee17`). At 5 V a sink current above `0.8/78.125 = 10.2 mA` reads as a
sag. `rail_ic_power_chatter` applies (supply-gated).

### 11.8 Tests that pin it

`wave4-comparator-shunt.test.ts:36-370`: catalog (`:37`), released at
`IN+ > IN-` with the pull-up high (`:83`), sinking: `OUT1` about 39 mV from the
78.125 ohm / 10 kohm divider (`:143`), threshold crossing flips the state
(`:183`), hysteresis: a dither inside +-1 mV does not flip (`:238`), dual
independence (`:288`), unpowered releases (`:342`). `ls47-released-segment-
sag.test.ts` (the sag sweep covers open-collector outputs), `ac-small-signal`
`:787-877`, `isolated-section-stall.test.ts`, `rail-ic-power-chatter.test.ts`.

### 11.9 Questions

- **Q-W3-51 (form; DK question 6).** Answered in section 11.5: latch with a
  one-step lag, so the driver-commit form. **Recommend** accept DK's
  recommendation for the lm393.
- **Q-W3-52 (propagation delay).** Classic has none beyond the step lag.
  LM393 (TI, recalled): response time 1.3 us typ for a 100 mV step with 5 mV
  overdrive and 300 ns for a TTL-level step. The W1 accuracy standard keeps
  classic timing for everything but registers, tri-states and memories
  (`W1:About`); a driver commit needs SOME delay value for its event time.
  **Recommend** schedule the commit at the instant the crossing is located
  (zero delay), which is the physically nearest classic-equivalent timing, and
  leave the datasheet delay as an owner option (a divergence, like W1's Q-O1).
- **Q-W3-53 (hysteresis).** `+-1 mV` is not in the datasheet (a real LM393
  has none; `VIO` is about 2 mV typ, 5 mV max, recalled) but is what makes
  classic's behaviour well-defined near zero. The crossing is at `d = +1 mV`
  rising and `-1 mV` falling, an 'effective offset' of 1 mV one way and the
  other. **Recommend** reproduce.
- **Q-W3-54 (sink is not current-limited).** The sinking unit is a fixed
  resistance to V-; with a low pull-up the stage can carry far more than the
  16 mA `io_max` (which only sizes `Rout`); the real part saturates and the
  datasheet quotes `VOL` 250 mV typ at 4 mA, 400 mV max, and a minimum sink of
  6 mA at `VOL <= 1.5 V` (recalled). **Recommend** reproduce.
- **Q-W3-55 (input common-mode range).** The datasheet range is `0` to
  `V+ - 1.5 V` (the catalog says `VCC - 2 V`); classic compares any voltage.
  **Recommend** reproduce.
- **Q-W3-56 (unit 2's current is not published).** Only `out1`'s current
  appears in `elementI`. **Recommend** the same telemetry contract.

---

## 12. Shunt reference: tl431

[PLAN lists it under W3-C with "needs W3-G". It does not.] `tl431Model`
(`SC:1042-1164`). A single exponential VCCS from cathode to anode controlled
by `V(ref) - V(anode)`, plus a 1 Mohm bias. No latch, no regime, no state, no
branch row.

### 12.1 Terminals and parameters

Pins by id `ref`, `anode`, `cathode` (`SC:1076-1078`); if the cathode is on
the ground node (`cathNode < 0`) the stamp returns (nothing at all, not even
the bias resistor). No params. Catalog (CAT:7102): no `default_params`;
`p_max 0.5` (not read). Literals (`SC:1085-1088`): `TL431_VREF = 2.495` V,
`TL431_IK = 1e-3` A, `TL431_NZ = 0.5`, so `VtNZ = 0.5*Vt = 12.846 mV` at
25 C and the exponential guard `vSatZ = 5*VtNZ = 64.231 mV`.

### 12.2 Equations

`drive = V(ref) - V(anode) - 2.495` at the iterate (`SC:1093`);
`driveC = min(drive, vSatZ)`; `eZ = exp(driveC/VtNZ)`;
`gPhysical = 1e-3*eZ/VtNZ`; `I_ka = 1e-3*eZ + gPhysical*max(0, drive - vSatZ)`
(current from cathode to anode); `gZ = gPhysical + 1e-12`;
`Ieq = I_ka - gPhysical*(V(ref) - V(anode))`.
Stamp: `(cath, ref) += gZ`, `(cath, anode) -= gZ`, `b[cath] -= Ieq`,
`(anode, ref) -= gZ`, `(anode, anode) += gZ`, `b[anode] += Ieq`
(each guarded for ground rows); `stampResistor(ref, anode, 1e6)` always. The
current does NOT depend on `V(cathode)`; the `1e-12` is a transconductance
in the Jacobian that the companion `Ieq` omits, so the converged KCL carries
`I_ka + 1e-12*(Vref - Vanode)` (negligible).

Reported current (`SC:1123-1138`): `1e-3*exp(driveC/VtNZ) + slope*max(0, drive
- vSatZ)`, cathode to anode, no GMIN.

At 25 C: one decade of `I_ka` per 29.58 mV of `drive`; 1 mA at `drive = 0`,
10 mA at +29.6 mV, 100 mA at +59.2 mV; the tangent starts at 148.4 mA
(`1e-3*e^5`) with slope `11.55 S`. Dynamic resistance `VtNZ/I`: 12.8 ohm at
1 mA, 1.28 ohm at 10 mA, 0.128 ohm at 100 mA.

### 12.3 Dynamics and state

None. No compensation, no capacitance (a real TL431 is conditionally stable
with large output capacitance, recalled; not modelled), no thermal.

### 12.4 Newton

No limiter; the tangent guard bounds the exponential. The model is a
transconductance whose cathode current is independent of the cathode node, so
Newton converges through the circuit's own feedback (REF tied to a divider on
the cathode). With the REF held by something else the cathode node is
unconstrained (section 12.5).

### 12.5 Regimes and events

None discrete. **Finding: unbounded negative cathode.** Because `I_ka` does
not depend on `V(cathode)`, a cathode fed through a resistor from a rail with
REF only a few percent above `Vref` is driven absurdly negative
(computed, `Vs = 12 V`, 1 kohm pull-up): REF 2.52 V gives `I_ka = 7.0 mA`,
cathode 5.0 V; REF 2.55 V gives 72 mA, cathode -60 V; REF 2.6 V gives 619 mA,
cathode -607 V; REF 3 V gives 5.2 A, cathode -5229 V. With a 100 kohm pull-up
REF = 2.52 V already drives the cathode to -688 V. A real TL431 saturates
at about 1 to 2 V above the anode with REF well above `Vref` (the datasheet
`Vka(min)`) and clamps the current at 100 mA. Circuits that use the part as
a comparator or crowbar trigger (REF from a divider that is not fed back from
the cathode) are therefore wrong by hundreds of volts in classic. The shipped
tests use only the fed-back configurations.

### 12.6 Small-signal AC

`SC:1140-1163`: `stampAcAdmittance(ref, anode, 1e-6)`; `gZ = 1e-3*exp(driveC/
VtNZ)/VtNZ + AC_GMIN` and the same four VCCS entries (`cath/ref +gZ`,
`cath/anode -gZ`, `anode/ref -gZ`, `anode/anode +gZ`) at the OP's `drive`
(`SC:1156-1162`). The derivative of the transient stamp, GMIN included; no
disagreement. No output conductance (the cathode current is independent of
`V(cathode)`), so the AC output impedance of the part is infinite.

### 12.7 Diagnostics

None. (`p_max` is not read; there is no thermal profile.)

### 12.8 Tests that pin it

`wave4-comparator-shunt.test.ts:377-520`: `Vout = Vref*(1 + R1/R2)`: cathode
4.94 to 5.10 V and 5.9 to 8.1 mA with `R1 = R2 = 10 kohm` on 12 V through
1 kohm (`:402`: 4.99 V ideal plus about 50 mV of knee offset); REF tied to the
cathode gives 2.495 V within 30 mV (`:459`); REF tied to ground leaves the
cathode near 12 V (`:505`).

### 12.9 Questions

- **Q-W3-57 (no cathode-voltage dependence; section 12.5).** The port
  mirrors the unbounded behaviour or adds `Vka(min)` saturation and a 100 mA
  cap. The analytic answer (decision 7 (b)) is the saturated one: cathode
  >= anode + about 1 to 2 V (TI TL431: 1 to 100 mA operating range, recalled).
  A kernel with `limit` and a regime (anode-referenced saturation) is a
  genuine fix. **Recommend** mirror classic for W3-C gate 1 and register the
  divergence now: PLAN's decision 7 names exactly "classic is wrong or
  absent".
- **Q-W3-58 (anchor at 1 mA versus 10 mA).** The reference voltage is exact
  at 1 mA; the datasheet specifies `Vref` at `Vka = Vref`, `Ik = 10 mA` (TI,
  recalled). Classic is 29.6 mV (1.2 percent) high at 10 mA: 2.525 V. The
  catalog note quotes `+-0.5 percent` (B grade). **Recommend** reproduce.
- **Q-W3-59 (dynamic impedance).** Classic 1.28 ohm at 10 mA against the
  datasheet 0.22 ohm typ (0.5 max) (recalled): the load regulation of a
  classic TL431 is about six times worse than a real part. **Recommend**
  reproduce; flag to Dev.
- **Q-W3-60 (no reverse conduction).** A real part has an anode-to-cathode
  diode; classic has none. A TL431 with its cathode below its anode conducts
  nothing in reverse in classic (except the VCCS pull when REF is high, 12.5).
  **Recommend** reproduce.
- **Q-W3-61 (placement).** PLAN row W3-C lists the TL431 with the
  comparator behind W3-G. It needs only W3-N (a three-terminal smooth kernel)
  and could land with W3-T. **Recommend** move it.
- **Q-W3-62 (`cathNode < 0` returns everything).** With the cathode on the
  ground node no stamp is written, including the REF bias. **Recommend**
  reproduce (the case is degenerate).
