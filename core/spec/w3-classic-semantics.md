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
end (`DR:183-190`). Registration order does not decide stamp order: the stamp
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
  `DR:183-190`. They are reachable from SPICE import (`J` cards give njfet,
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
point (W1 section 10.x; `ctx.opVoltage(node)` reads the held OP). Every W3
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
