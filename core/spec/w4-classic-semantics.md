# W4 classic semantics: what the Rust core must mirror (electromechanical, audio, sensors, drivers, display chips, boards)

## About this document

**Purpose.** This document specifies what the classic brownout TypeScript
engine does for the 17 part kinds that the Rust/WASM core ports in wave W4
(`devolt/memory/progress/brownout-core-w1e-drafts/plan-1.0.0.md` section 2.2:
W4-E, W4-A, W4-D, W4-P, W4-S). It is the W4 counterpart of
`core/spec/w1-classic-semantics.md` and `core/spec/w3-classic-semantics.md` and
follows their shape: each kind has its terminals and parameters, the equations
as classic evaluates them with every constant cited, its dynamics and state,
Newton behaviour, regimes and events, small-signal AC, diagnostics, the tests
that pin it, and a numbered list of questions. The readers are the W4
implementation workers and the W4 reviewers. For the four boards the subject is
the board SHELL (supply paths, pin electrical models, how emulator events enter
the circuit and node voltages reach the emulator), not the CPU emulators.

**Status.** All 17 kinds are written; the status of each is "complete" in the
table below. Nothing was built or run for this document except a handful of
arithmetic checks (the numbers in the Uno/Nano path, the ULN/L293D clamp
diode, the servo and motor time constants were computed with a throwaway
Node script, not by the engine); every other fact was read from the code. A
fact marked "inferred" was read but not executed. avr8js and rp2040js are
not read here (their sources are under `node_modules`, which this task forbids
reading); where classic's behaviour depends on them it is marked as such.

**Authority rule.** As in W1 and W3: the classic TS source is the authority.
Every fact is cited `file:line`. Plan decision 7 (PLAN section 5, "Reference
where classic is wrong or absent", recommendation (b), not yet ruled) applies:
where classic disagrees with the datasheet or with physics, this document says
so and gives both figures, but it does not fix classic. The port mirrors
classic unless a question below is ruled otherwise. "Datasheet figure" entries
are recalled from the named manufacturer's datasheet and were not re-fetched;
each is marked "recalled" and must be checked before it is used to justify a
change.

**Which tree.** The code read is brownout `wip/v1` at commit
`33d3e1561808b16bb660bf07ae576acca1f057d0` (worktree branch `wip/sx4`, one new
file, this one; the W3 chapter is at the same commit). Line numbers refer to
that tree. All 17 kinds have a bundled catalog entry in
`src/parts/default-parts.json`.

**Conventions.**
- Paths are relative to the brownout repository root unless they start with
  `devolt/`. Abbreviations for files:
  `SE` = `src/sim/engine/sim-engine.ts`,
  `EL` = `src/sim/engine/elements.ts`,
  `EA` = `src/sim/engine/devices/electromech-audio.ts`,
  `DI` = `src/sim/engine/devices/digital-ics.ts`,
  `DR` = `src/sim/engine/devices/index.ts` (the registry),
  `SC` = `src/sim/engine/devices/semiconductors.ts`,
  `AD` = `src/sim/engine/arduino.ts` (the avr8js wrapper),
  `RP` = `src/sim/engine/rp2040.ts` (the rp2040js wrapper),
  `MC` = `src/sim/engine/mcu.ts` (the core contract and factory),
  `SP` = `src/sim/servo-physics.ts`,
  `PT` = `src/sim/engine/ptc.ts`,
  `AS` = `src/sim/adaptive-step.ts`,
  `HL` = `src/host/headless.ts`,
  `GR` = `src/sim/engine/graph.ts`,
  `AA` = `src/sim/engine/ac-analysis.ts`,
  `CAT` = `src/parts/default-parts.json`,
  `W1` = `core/spec/w1-classic-semantics.md`,
  `W3` = `core/spec/w3-classic-semantics.md`,
  `PLAN` = plan-1.0.0.md, `DK` = dk-design.md (both under
  `devolt/memory/progress/brownout-core-w1e-drafts/`).
- "Tests" are under `test/sim/engine/` unless a path is given. Test titles are
  quoted only where the exact claim matters.
- `Vt` is the junction thermal voltage `K_B_OVER_Q * (T + 273.15)`; at the
  default 25 C it is 25.6926 mV (W3 section 1.1). `ctx.junctionVt()` and
  `ctx.junctionVf(vf25)` are `SE:2273-2279`.
- Branch-current and stamp sign conventions: `stampCurrentSource(mna, i, j, I)`
  adds `+I` to the RHS of node `i` and `-I` to node `j` (`EL:138-141`), i.e. it
  injects `I` INTO `i` and removes it from `j`. A Norton current of `I` flowing
  `a -> b` through a branch is therefore stamped `stampCurrentSource(a, b, -I)`;
  this is what the relay, motor and stepper handlers do with a minus sign.
  `stampVSourceSeriesR(mna, i, j, k, V, R)` (`EL:83-125`) gives branch row `k`
  with `x[k]` = the current entering the source from its `+` node, NEGATIVE
  while the source delivers (W3 conventions).
- "Open pin" means `ctx.isOpenPin` (W1 section 1.6). A pin whose node is the
  reference net has node index `-1`; every `mna.add` / `addB` guard skips a
  negative index, so a handler need not special-case a grounded pin.
- Wave placement tags `[W4-X]` follow PLAN 2.2.
- Questions are `Q-W4-n`. Rulings already made are cited by their home:
  PLAN section 5 (decisions 1 to 16, none yet ruled as of PLAN's date, with a
  recommendation each) and DK section 5 (questions 1 to 11, with
  recommendations). Where a question below is answered by one of those it
  cites it instead of re-asking.

### The 17 kinds

| Kind(s) | Model | File:lines | Params read | Shares | Catalog part |
|---|---|---|---|---|---|
| relay | `relayModel` | EA:189-342 | 4 (`coilR`, `coilL`, `vPull`, `vDrop`) | RL Norton companion with dc_motor, stepper | `relay-spdt-5v` CAT:7263 |
| dc_motor | `dcMotorModel` | EA:344-479 | 7 (`windingR`, `windingL`, `Ke`, `Kt`, `inertia`, `friction`, `loadTorque`) | RL Norton companion | `dc-motor` CAT:8004 |
| servo | `servoModel` | EA:481-657 (+ SP:1-66 sidecar) | 9 names (`idleR`, `movingCurrent`, `nominalVoltage`, `minimumOperatingVoltage`, `maxSpeedDegPerSecond` with alias `speedDegPerSecond`, `minPulseMs`, `maxPulseMs`, `minAngle`, `maxAngle`) plus 2 catalog `vcc_range` reads | MCU pin-event decode with hcsr04 | `servo-sg90` CAT:8038 |
| stepper | `stepperModel` | EA:659-819 | 2 (`coilR`, `coilL`); catalog `stepsPerRev` is not read | RL Norton companion | `stepper-bipolar-nema17` CAT:8088 |
| buzzer | `buzzerModel` | EA:1146-1308 | 2 (`type`, `resistance`) | resistor + detector with speaker | `buzzer-active-5v` CAT:7630, `buzzer-passive-5v` CAT:7659 |
| speaker | `speakerModel` | EA:1310-1402 | 1 (`resistance`) | resistor + detector | `speaker-8ohm` CAT:7688 |
| hcsr04 | `hcsr04Model` | EA:821-990 | 2 (`distanceCm`, `noEcho`) | MCU pin-event decode with servo | `hcsr04` CAT:8344 |
| uln2003, uln2803 | `ulnModel` (one body, channel count by kind) | EA:992-1144 | 0 | committed latch + `stampClampDiode` with l293d, tb6612 | `uln2003a` CAT:7326, `uln2803a` CAT:7470 |
| l293d | `l293dModel` | EA:1435-1597 | 0 | H-bridge family with tb6612 | `l293d` CAT:7716 |
| tb6612 | `tb6612Model` | EA:1599-1769 | 0 | H-bridge family with l293d | `tb6612` CAT:7860 |
| hd44780 | `hd44780Model` | DI:1889-2092 | 0 | MCU display-event path with max7219 | `hd44780-lcd` CAT:8195 |
| max7219 | `max7219Model` | DI:2095-2266 | 0 | MCU display-event path | `max7219-matrix` CAT:8134 |
| arduino_uno | none: inline in `_stampAll` | SE:7005-7077 | 4 (`vcc`, `usb_power`, `idleCurrent`, `hex`) | with arduino_nano (one case body) | `ic-arduino-uno` CAT:5282 |
| arduino_nano | none: same case | SE:7005-7077 | 4 (same) | with arduino_uno | `ic-arduino-nano` CAT:5918 |
| raspberry_pi_pico | none: inline | SE:7078-7133 | 4 (`vcc`, `usb_power`, `idleCurrent`, `script`) | pin-drive loop with the AVR boards | `ic-raspberry-pi-pico` CAT:5534 |
| microbit | none: inline | SE:7134-7191 | 3 (`vcc`, `power`, `idleCurrent`); `breakout` is in the catalog but the engine does not read it | no MCU core in the engine | `ic-microbit` CAT:5866 |

Registration (`DR:171-186`): `hd44780Model` and `max7219Model` in the
digital-IC block, then relay, dc_motor, servo, stepper, hcsr04, uln, buzzer,
speaker, l293d, tb6612 in the ELECTROMECH-AUDIO block. Registration order does
not decide stamp order: the stamp list is circuit component order (W3 section
1.3). The four boards have NO device model: they are the three `case` bodies
of `_stampAll`'s fallthrough switch (`SE:7005`, `7078`, `7134`), the `case`
of `_updateElementI` (`SE:7534-7547`) and the helpers listed in section 14.

Which parts of the file header to trust. `EA:1-65` says every handler body was
"MOVED VERBATIM" from the engine's switch, "float accumulation order in the
matrix is semantics" and "the bitwise oracle suite pins these exact
trajectories". The port is not required to be bitwise; it is required to match
the cited behaviour within the gates the W4 packages set.

### Cross-cutting findings

These are the findings that most change how W4 should be built. Each is
developed in the chapter named.

1. **Almost nothing in W4 is a nonlinear device; almost everything is a
   committed decision applied one accepted step late.** Of the 13 non-board
   kinds, only the ULN, L293D and TB6612 output stages contain a nonlinearity
   at all (one guarded Shockley "catch diode" per output, section 1.4). The
   relay, motor and stepper are linear R+L Norton companions; the servo,
   buzzer, speaker and displays stamp a fixed resistor; the HC-SR04 stamps a
   resistor, a pull-down and a constant current. Every regime (relay energised,
   ULN sinking, bridge drive state, HC-SR04 ECHO level, servo moving) is
   decided in `commitState` from the accepted solve and stamped by the NEXT
   solve; no kind locates its instant, except that the HC-SR04's two edges are
   scheduled times that the host clamps its step to (section 8.5). The DK
   device-kernel interface therefore needs, for W4, no Jacobian beyond the
   one clamp diode and no charge; it needs the cross-partition driver commit
   (DK 1.5), event-scheduled edges, and per-kind committed state.
2. **The motor, servo and stepper mechanics are explicit side-integrators with
   one-step-late coupling, and they are "physical-time state".** `omega` is
   forward Euler with the back-EMF held at the previous step's value
   (section 3.3); the servo shaft is a rate limiter (`SP`); the stepper has no
   rotor at all, only a sign-pair state machine on the coil currents
   (section 5). All three hold at their entry values during every
   operating-point ladder (`SE:776-783`). The telemetry getters for them are
   absent on the core today (PLAN 1.2, D17).
3. **The MCU pin-event contract is the real interface of W4.** The servo and
   HC-SR04 decode cycle-timestamped `PinEvent`s from Uno, Nano and Pico
   (`includePico = true`); the HD44780 and MAX7219 decode them from Uno and
   Nano only (`includePico = false`, `SE:7356-7360`), so a Pico driving either
   display falls to the sampled path, which loses any E pulse or CLK edge
   shorter than a step (sections 12.5, 14.7). The events are handed out
   consumed-once, gated by `_mcuEdgesPending`, with a per-core boot-time
   baseline `_mcuBootSimTime` to turn cycles into seconds (section 1.5). The
   emulator core is not in `saveState` (section 14.10).
4. **Several classic behaviours are wrong against the datasheet or physics and
   worth deciding before the port copies them.** The ULN is a 5 ohm resistor
   (0.5 V at 100 mA against about 0.9 V recalled; section 9). The L293D and
   TB6612 are 2 ohm per side, so a motor between two outputs sees 4 ohm of
   output stage and a 4 V drop at 1 A, against about 2.6 V (L293D) and about
   0.5 ohm total (TB6612), recalled (sections 10, 11). The relay releases at
   1.5 V where its datasheet releases at about 0.5 V (section 2). The motor's
   `loadTorque` is a constant torque with no sign, so an unpowered motor with a
   load runs backward (section 3). The speaker detector reads silence for
   every host step of 5 ms or more (section 7). A Pico board is "powered"
   only by its `usb_power` flag, never by a supply on VSYS, VBUS or 3V3, and
   the Uno/Nano `VIN` and every board's RESET pin are dead pins (section 14).
5. **Boards: the emulator is stepped once per accepted electrical step and its
   outputs are held for the next interval.** The circuit sees a GPIO as a 50
   ohm Thevenin source to the real rails (or a 35 k / 65 k weak pull) whose
   level is the drive state AT THE END of the previous MCU step; PWM is
   therefore quantised to the step, which for any circuit with a Uno, Nano or
   Pico is capped at `MCU_H_MAX = 1e-4` (the headless runner; the worker's
   value is the same, `HL:25, 84`). A micro:bit has no engine core; its three
   to nine pins are driven by values the host pushes with `setMicrobitDrive`
   (section 14.9). The W4-S shell is the part of W4 with the most
   interactions with other kinds (supply gating, overvoltage, polyfuse).
6. **No W4 kind has a Newton limiter, a charge or a capacitance of its own.**
   The only memory elements are the coil inductances (BE companions) and
   committed latches. The step controller compares their state keys: `reg`
   style integer keys exactly and the current-like keys with tolerance
   (section 1.6), so a regime flip between the coarse and refined solves
   rejects the step (bracketing by halving, not location).
7. **Small-signal AC is hand-written per kind and never the derivative of the
   transient model.** The coils use the continuous `R + jwL` admittance where
   the transient is the backward-Euler `R + L/h`; the clamp diodes use the
   tangent slope at the held operating point; the committed regimes are read
   from state. Any circuit containing a Uno, Nano, Pico or micro:bit is
   rejected by the small-signal sweep (`AA:217-226`). A derived stamp (DK 1.4)
   would differ from classic wherever `omega * h` is not small (section 1.7).
8. **Dead and misleading code to not port**: the ULN's `_drivenOutputTargets`
   case is unreachable (the ULN catalog entry has no `vcc_range`, so the
   output-sag scan never runs for it, section 9.7); `L293D_VIL` is declared and
   never read (section 10.2); `servo` `sig` and the `speaker`/`buzzer` pins
   are read by position or by `pins[0]`/`pins[1]` where the catalog uses ids
   (sections 6, 7); `uln2003` appears in `PASSIVE_FAILURE_UPDATE_KINDS`
   (`SE:1417-1420`) with no failure model behind it; the HD44780 comment about
   "1M insurance stamps" (`DI:1991-1992`) describes a stamp that does not exist
   (`digital-input-impedance.test.ts` pins its absence, section 12.8).

---

## 1. Shared machinery

Everything in this chapter is used by more than one kind. Chapters 2 to 14
refer back to it.

### 1.1 Constants and helper functions

All in `SE` unless marked.

| Name | Value | Line |
|---|---|---|
| `NODE_RSHUNT_G` | 1e-12 S on every node | 101 |
| `ISOLATED_SECTION_ANCHOR_G` | 1 S, one node of each isolated section to ground | 114 |
| `AVR_PULLUP_R` | 35 000 ohm | 119 |
| `RP2040_PULL_R` | 65 000 ohm | 120 |
| `N_DIODE` | 1.0 | 130 |
| `OUTPUT_SAG_DEBOUNCE_STEPS` | 2 | 148 |
| `GATE_R_OUT` | 50 ohm | 1155 |
| `MCU_BOARD_IDLE_CURRENT_A` | uno 0.045, nano 0.020, pico 0.025, microbit 0.015 A | 1183-1190 |
| `HCSR04_MAX_STEP_H` | 100e-6 s | 1283 |
| `AC_GMIN` | 1e-12 S (`SC:85`) | |
| `DC_OP_COMPANION_H` | 1e-12 s | 702 |

- **`GATE_R_OUT = 50`** is the default push-pull output resistance of every
  "house" digital output and the board pin drivers; `_icPowerInfo` scales it
  per part with `io_max` (W1 section 1.3), but no W4 handler reads that scaled
  value: the HC-SR04 echo stage and the board pin drivers use the literal 50
  (`EA:850`, `SE:7067-7073`), the H-bridges and the ULN use their own 2 and 5.
- **`NODE_RSHUNT_G`** is added on every node diagonal (W1 section 1.6); a node
  that only a W4 pin touches (an unused relay contact, a released ULN output, a
  HIZ bridge output, an open servo `sig`) is held by this shunt alone and reads
  about 0 V.
- `ctx.junctionVf(0.7)` is `max(0.05, 0.7 - 0.002*(T - 25))` (`SE:189-196`,
  `2277`); at 25 C it is 0.7 V exactly.

### 1.2 Parameter resolution rules

Every W4 handler reads a parameter as `Number(comp.params.X ?? literal)`
(W3 section 1.2 rule 1): the catalog's `default_params` are NOT consulted by
the engine. The host copies `default_params` into the instance at placement,
so a freshly placed part carries the catalog value; a saved or hand-built
circuit that lacks the key gets the literal. For every parameter of W4 the
literal equals the catalog value (checked below), with these exceptions:

| Kind | Param | In code | In catalog | Note |
|---|---|---|---|---|
| servo | `nominalVoltage` | param, else `electricalSpecs.vcc_range.nominal`, else 4.8 (`EA:124-128`, `605-610`) | param 4.8 | one source of truth in practice |
| servo | `minimumOperatingVoltage` | param, else `vcc_range.min`, else 4.5 (`EA:611-614`) | param 4.5, `vcc_range.min` 4.5 | |
| servo | `maxSpeedDegPerSecond` | param, else `speedDegPerSecond`, else 600 (`EA:602-605`) | `maxSpeedDegPerSecond` 600 | alias only in code |
| stepper | `stepsPerRev` | not read | 200 | consumers (angle) are host-side |
| max7219, hd44780 | none | | | the catalog `icc_max` is not read; the model draws a fixed 150 or 200 ohm |

The board params `vcc`, `usb_power`, `power` and `idleCurrent` follow the same
rule (`SE:7015-7021`, `7087`, `7141`, `6417-6419`). `vcc` is read with
`_independentSourceMagnitude` for the source value (so source stepping scales
it) and as a plain number for the idle-load resistor's nominal voltage.

`specsForComponent(comp)` (`SE:1700-1702`) resolves the part's
`electrical_specs` through `partFor`; the W4 handlers that use it are the
servo (`vcc_range`), `_icPowerInfo` (all supplied kinds) and the failure pass.

### 1.3 The committed-state pattern and the order within one step

All thirteen non-board models follow one pattern. In `stamp` they read ONLY
committed state (`ctx.state.*`) and, for the clamp diodes, `xGuess`. In
`commitState` they read the accepted solution `x`, decide regimes and
integrate, and write `ctx.state.*`. The order inside one accepted step
(`W3 section 1.3`, `SE:6741-6779`): net voltages published; `_updateState`
(each model's `commitState`, in circuit order, `SE:7320-7340`); then
`_updateDigitalState` (the display decoders and the digital-update kinds); then
`_updateElementI` (each model's `updateCurrent`, which for W4 only READS the
state just committed); then thermal (none of W4) and the failure pass; then
`_updateStaticWarnings` is load-time only. After that, `_stepSolves` advances
`simTime` and calls `_advanceMcusAfterAcceptedSolve(h)` (`SE:4546`, `4570`,
`4626`), which samples inputs into every MCU core and steps it once (section
14.4). The MCU therefore runs AFTER the commit of its own step and BEFORE the
next stamp.

Consequences used throughout:
- A decision made in step `k` (a ULN input reading high, a bridge enable, the
  relay `energized` flag, the servo `moving` flag, the HC-SR04 `echoOut`)
  changes the stamp of step `k+1`, one accepted step later, however small that
  step is. At `h = 1e-8` (the host floor) the lag is 10 ns; at `h = 1e-2` it is
  10 ms.
- The state records are snapshotted by `saveState` (`SE:2608-2655`): `relays`,
  `motors`, `servos`, `steppers`, `hcsr04`, `lcds`, `icState` and `ptcs`.
  The MCU cores are NOT in it (section 14.10).
- The component state carries across `load()` by component id: relays,
  motors, servos, steppers, lcds, hcsr04 (`SE:3190-3259`); `icState` for the
  ULN/bridge/buzzer/speaker/max7219 through the generic `icState` carry, which
  keeps a record whose id and kind are unchanged. A servo's carried record is
  merged over `defaultServoState()` field by field and non-finite
  `targetAngle`, `velocity`, `supplyVoltage`, `loadResistance` are repaired
  (`SE:3208-3231`).

### 1.4 The two linear companions and the clamp diode

**R+L Norton companion (relay, dc_motor, stepper).** For a series R and L with
branch current `i` flowing a -> b, backward Euler gives
`v_ab = R*i + L*(i - i_prev)/h (+ Vbemf)`, so
`i = G*v_ab + G*((L/h)*i_prev - Vbemf)` with `G = 1/(R + L/h)`. Classic stamps
it as `stampResistor(a, b, R + L/h)` (conductance `G`) and
`stampCurrentSource(a, b, -I_eq)` with `I_eq = G*((L/h)*i_prev - Vbemf)`
(relay `EA:249-253`, motor `EA:403-409`, stepper `EA:709-713`). The relay and
stepper have `Vbemf = 0`. The stamp is linear in `x`, so Newton converges in
two iterations (one to move, one to confirm). The integration is backward Euler
always: none of the three uses `useTrapThisSolve`. In a DC solve
(`ctx.dcSolveMode()`) the companion is replaced by the plain resistor `R`
(relay `EA:235-243`, motor `EA:385-395`, stepper `EA:694-702`); the motor adds
the held back-EMF as `I_eqDc = -Vbemf/R` with `stampCurrentSource(m1, m2,
-I_eqDc)`.

**`stampClampDiode(ctx, anode, cathode, xGuess, vt)` (`EA:138-161`).** The
only nonlinearity in W4 (the board USB Schottkys, section 14.3, are the other
two uses of the same Shockley stamp). One diode from `anode` to `cathode`:
- `vf = ctx.junctionVf(0.7)`, rated current fixed at 0.1 A;
- `Is = shockleyIsFromVf(vf, 0.1, N_DIODE, vt) = 0.1*exp(-vf/(1.0*vt))`
  (`EL:755-762`); at 25 C `Is = 1.4708e-13` A;
- `vtN = N_DIODE * vt`; `vSat = min(max(40*vtN, vf + 5*vtN), 80*vtN)`; at 25 C
  `vSat = 1.02770` V (the `40 vtN` term wins);
- `vd = V(anode) - V(cathode)` from `xGuess`, then `stampDiodeShockley(mna,
  anode, cathode, vd, Is, N_DIODE, vt, vSat)` (`EL:693-742`): the current is
  `Is*(exp(min(vd, vSat)/vtN) - 1) + (Is*exp(vSat/vtN)/vtN)*max(0, vd - vSat)`
  (`shockleyDiodeCurrent`, `EL:772-786`; a tangent extension past `vSat`),
  conductance `g = Is*exp(min(vd, vSat)/vtN)/vtN + GMIN_D` with `GMIN_D =
  1e-12` (`EL:718-719`), and the Norton offset `ieq = id - gd*vd` stamped as
  `addB(anode, -ieq)`, `addB(cathode, +ieq)` (`EL:734-740`).
- At `vSat` the diode carries 3.46e4 A with slope 1.35e6 S: the clamp is a
  stiff but finite wall; it has no series resistance.
- There is no pnjlim limiting (W3 section 1.8): Newton relies on the `vSat`
  guard only. No W4 stamp calls `markJunctionLimitedThisIteration`.

The AC image is `acStampClampDiode` (`EA:169-183`): the same `Is`, `vSat`, with
`gd = shockleyConductanceAtOp(vd, Is, N_DIODE, vt, vSat)` at the held
operating-point voltage, plus `AC_GMIN`, stamped with
`stampAcAdmittance(ac, anode, cathode, gd + AC_GMIN, 0)` (`EA:182`).

### 1.5 MCU event plumbing: what the servo, HC-SR04 and displays read

Four pieces of engine machinery feed cycle-accurate edges to the W4 decoders.
All four are engine-owned (`EA:41-58`); the port needs the same contract.

1. **The pin-driver map.** `_buildArduinoPinDrivers(includePico)`
   (`SE:7361-7376`) maps every MCU I/O pin's MNA node to `{compId, pin}` for
   each uno/nano (and, only when `includePico`, each pico) component that has
   a core in `state.arduinos`. Last writer wins when two pins share a node
   (`SE:7356`). Two caches exist: `_mcuEventDrivers` (`includePico = true`,
   reached through `ctx.mcuEventDriverForNode`, `SE:2500-2507`; used by servo
   and HC-SR04 and by the clocked ICs' `mcuInputEdges`) and `_dispDrivers`
   (`includePico = false`, `SE:2482`; used only by `mergedDisplayEvents`). Both
   are reset per pass (`SE:7325`, `7387`).
2. **Consumed-once step events.** `ctx.mcuStepPinEvents(compId)`
   (`SE:4998-5001`) returns the core's `getStepPinEvents()` only while
   `_mcuEdgesPending` is true and arms `_mcuInputEdgesWanted`; otherwise it
   returns the shared empty list. `_advanceMcusAfterAcceptedSolve` sets
   `_mcuEdgesPending = true` after a step that recorded start levels
   (`SE:4744`) and `_updateDigitalState` clears it when the digital pass has
   run (`SE:7497`), so one MCU step's events reach each decoder once even when
   an NE555 splits the step (tests `mcu-servo-replay`, `mcu-display-replay`).
   A core that did not step (unpowered) has its event list cleared at the
   advance (`SE:4723-4733`, `clearStepPinEvents`).
3. **Boot-time baseline.** `PinEvent.cycle` is cumulative since the core
   booted, at the core's own clock: Uno/Nano 16 MHz (`AD:74`; `cpu.cycles` at
   the write), Pico 125 MHz (`RP:22`; `sim.clock.nanos / 8` so a WFI
   fast-forward still counts, `RP:364-381`). `_mcuClockHz(kind)` returns
   125 000 000 for the Pico and 16 000 000 for everything else (`EA:96-98`).
   `ctx.mcuBootSimTime(compId)` is the engine `simTime` at which the core was
   (re)built: set where `load()` constructs a new core and in `resetArduino`
   (`SE:3342`, `3351`, `3367`, `3375`, `5145-5152`). Event time is
   `bootT + cycle / clockHz` (`EA:555`, `897`), `bootT` defaulting to 0 when
   absent.
4. **The display variant.** `_mergedDisplayEvents(comp, pinNames, drivers)`
   (`SE:7385-7430`) maps the MCU pins on the part's protocol pins to the part's
   pin names; if the pins span more than one MCU, or none, it returns no events
   (the caller falls back to sampling); otherwise the events for those pins,
   sorted by cycle, as `{displayPin, level, cycle}`. The same `_mcuEdgesPending`
   gate applies.

The H11 readback shadow (`SE:4833-4930`, `mcuStepShadow`) applies to parts that
register one; none of the W4 kinds does (the 74HC165 and 74HC595 do).

### 1.6 What the step controller sees

`estimateStepError` (`src/sim/adaptive-step.ts`, W1 section 10.14, W3 section
1.5) compares the coarse and refined snapshots; the tolerances are
`ABS_VOLTAGE = 100 uV`, `ABS_CURRENT = 10 nA`, `ABS_SPEED = 1e-3`,
`ABS_ANGLE = 1e-4`, `ABS_TIME = 10 ns`, `ABS_PULSE_WIDTH_MS = 1e-5`,
`ABS_RESISTANCE = 1e-6`, `ABS_FREQUENCY = 1e-3` (`AS:21-30`). For W4:
- `relays` (`AS:254-265`): `iCoil` at `ABS_CURRENT`; `energized` exactly.
- `motors` (`AS:267-278`): `iWinding` at `ABS_CURRENT`, `omega` at
  `ABS_SPEED`.
- `servos` (`AS:280-317`): `lastSig` exactly; `riseT` at `ABS_TIME`;
  `pulseMs`, `targetAngle`, `angle` (`ABS_ANGLE`), `velocity` (`ABS_SPEED`);
  `moving` and `powered` exactly; `supplyVoltage` at `ABS_VOLTAGE`;
  `loadResistance` at `ABS_RESISTANCE`.
- `steppers` (`AS:319-337`): `iA`, `iB` at `ABS_CURRENT`; `phase`,
  `lastKnownPhase`, `position` exactly.
- `hcsr04` (`AS:339-372`): `trigLevel`, `phase`, `echoOut` exactly;
  `echoRiseAt`, `echoFallAt` at `ABS_TIME` absolute-only; `vccV` at
  `ABS_VOLTAGE`; the presence of `pendingEchoEvent` exactly.
- `icState` keys (`AS:454-461`, `icStateTolerance`): `vm`, `vPeak`, `vTrough`,
  `peakToPeak` at `ABS_VOLTAGE`; `lastCrossT`, `prevCrossT`, `windowStart` at
  `ABS_TIME` absolute; `detectedHz` at `ABS_FREQUENCY`; `envHi`, `envLo` are
  skipped (`IC_STATE_READOUT_ONLY`: "No stamp reads it, and it moves on every
  step by an amount that depends on h"); every other key, so the ULN `inN`, the
  bridge `out*`, the buzzer `lastSign` and `sounding`, the MAX7219 registers, is
  compared EXACTLY and a mismatch scores Infinity. A regime flip between the
  coarse and the two half steps therefore rejects the step and halves it:
  regimes are bracketed by halving, not located (W3 section 1.5).
- `lcds` (the HD44780 state) is not in the error snapshot (`SE:2679-2690`
  lists no `lcds`), so a display change never moves the step.
- The MCU circuits (uno, nano, pico) take NO trial replay: the headless runner
  and the worker decide `circuitSupportsStepReplay` false for them
  (`HL:111-117`) because the emulators cannot be rewound, so those circuits run
  the conservative controller capped at `MCU_H_MAX = 1e-4` (`HL:84`, `1097`). A
  micro:bit is NOT in that list (`HL:112-116`), so a circuit with only a
  micro:bit keeps error control and its host step cap.

### 1.7 Small-signal AC

`acStamp(ctx, comp, ac, omega)` runs once per frequency after a DC operating
point (W3 section 1.6). Every non-board W4 kind has one; none is derived from
the transient stamp; the W3 rule holds: committed regimes select the
piecewise branch the last transient step stamped. Specific notes per kind are in
the chapters. The boards have no AC model and
`runSmallSignalAc` throws for any circuit that contains one
(`AA:217-226`, `isMcuBoardKind(kind) || kind === "microbit"`). The census test
is `ac-isolation.test.ts:343-395` (cited by W3; every kind with a registered
model carries an `acStamp` or is listed as exempt).

### 1.8 DC operating point: the physical-time hold and the winding commit

- `DC_OP_PHYSICAL_TIME_HOLD_KINDS` (`SE:776-783`) holds `dc_motor`, `servo`,
  `stepper` and `hcsr04` (with `battery_pack` and `thermistor`) at their entry
  state through the whole ladder: while `_dcPhysicalTimeHold` is true their
  `commitState` is skipped (`SE:7334`). The relay is NOT in the set: its
  `commitState` runs during the ladder, so its `energized` flag can flip inside
  the OP solve (section 2.5).
- In DC mode the relay, motor and stepper stamp R only (section 1.4);
  `_commitDcOperatingPointState(viaDcStamps)` (`SE:4346-4400`) then overwrites
  the held currents: relay `iCoil = vCoil/coilR` (`SE:4361-4373`), motor
  `iWinding = (vWinding - Ke*omega_held)/windingR` (`SE:4375-4388`), stepper
  `iA = vA/coilR`, `iB = vB/coilR` (`SE:4389-4400`), and republishes
  `elementI`. Phase and position are left alone ("an OP is a bias solve, not a
  motion interval"). A pseudo-transient rung does not take this branch
  (`viaDcStamps` false): the companions already settled.
- The other W4 kinds are stateless in DC apart from their committed latches,
  which the OP solve updates through the ordinary `commitState`.

### 1.9 IC power gating, thresholds and floating inputs (cited, not repeated)

The H-bridges, `hd44780` and `max7219` use `_icPowerInfo` (W1 section 1.3):
powered iff supplies are wired and `vcc - gnd` lies in `[0.9*vcc_range.min,
1.1*vcc_range.max]`, `thresholdsFor(specs, vSupply)` for `vil`/`vih`, and
`ctx.logicHigh` / `_floatingLogicHigh` (W1 sections 1.4, 1.5) for open inputs.
The concrete windows and thresholds for the five W4 users:

| Part | Supply pins | Powered window | `vil` / `vih` (family) |
|---|---|---|---|
| l293d | `vcc1`, `gnd1` | [4.05, 7.70] V | generic: `max(0.8, 0.3 vS)` / `max(2.0, 0.6 vS)` (no `logic_family`, no `v_input`) |
| tb6612 | `vcc`, `gnd` | [2.43, 6.05] V | generic, same formula |
| hd44780 | `vdd`, `vss` | [4.05, 6.05] V | 0.6 / 2.2 from `v_input` (family `CMOS` is not `HC-CMOS`) |
| max7219 | `vcc`, `gnd` | [3.60, 6.05] V | 0.8 / 3.5 from `v_input` |
| hcsr04 | `vcc`, `gnd` | not read for stamping; own 3.0 V rule | n/a |

`uln2003`/`uln2803` have no `vcc` pin and no `vcc_range`, so
`_icPowerInfo` returns the "no supply pin" branch (`powered: true`,
`gnd = 0` literal) wherever it is called for them (W1 section 1.3); the ULN
handlers do not call it and read the `gnd` pin directly (section 9).
None of the W4 kinds is in `SUPPLY_GATED_KINDS` except `hd44780` and `max7219`
(`SE:1383-1389`), which makes them part of the `rail_ic_power_chatter` machinery
(W1 section 8) and of the `_noSupplyReturnIds` rule.

### 1.10 Failures and warnings across W4

- `SimFailureKind` has five members (`SE:635-640`); the W4 kinds can raise
  `output_sag` (l293d, tb6612 through `_drivenOutputTargets`, `SE:7931-7957`;
  the ULN case is unreachable, section 9.7) and `supply_overvoltage` (the four
  boards, section 14.6). There is no overload, thermal or `burnt` state for any
  W4 kind.
- `SimWarningCode` has three members (`SE:657`): `missing_flyback` is raised
  for `relay` and `dc_motor` (and `inductor`) through
  `inductiveWindingPins` (`SE:1084-1094`): relay `coil_a`/`coil_b`, motor
  `m1`/`m2`; the stepper is NOT checked by the engine (its catalog note and
  tests refer to the host's static rule engine, section 5.7, Q-W4-{stepfly}).
- `missingFlybackWarning` (`SE:1117-1148`) is evaluated at load from the graph
  (`_evaluateStaticWarnings`, `SE:6639-6660`): it fires if a winding's two
  nets differ and one of them, not `gnd` and not holding a pin of a
  `FLYBACK_RAIL_KINDS` part (`voltage_source`, `battery_pack`, `bench_psu` and
  the three MCU boards, `SE:1078-1082`), holds a pin of a
  `FLYBACK_SWITCH_KINDS` part (bjt_npn/pnp, nmos/pmos, switch, push_button,
  spdt_switch, push_dpdt, dip_switch, `SE:1069-1072`) and no
  `FLYBACK_CLAMP_KINDS` part (diode, schottky, zener, tvs, led,
  `SE:1073-1077`). The ULN, L293D and TB6612 are in neither set, so a winding
  on their outputs never fires: the engine's "suppression" by a driver is only
  that the driver is not a switch kind (`SE:1060-1068` says "a host's own rule
  engine owns those suppression rules"). The VM-powered condition of the tests
  (`wave6-hbridge.test.ts:916-995`, "fires when L293D VM is floating") belongs to
  the host static diagnostics, a test-support copy at
  `test/helpers/diagnostics/static.ts:918-969` (section 5.7), not to the
  engine: a relay on an L293D with a floating VM raises NO engine warning
  (Q-W4-{vmfly}).
- The output-sag pass (`SE:7553-7690`) runs for every component whose catalog
  entry has `vcc_range`; its debounce is two consecutive sagging steps, and its
  one-pole exemption `_outputSagPole` applies only to the drivers in
  `RC_OUTPUT_IC_KINDS` (`SE:1315-1320`), which contains no W4 kind.

---

## 2. Relay: relay

[W4-E.] `relayModel` (`EA:189-342`). A single-pole double-throw relay: an R+L
coil and a changeover contact pair. Catalog `relay-spdt-5v` (`CAT:7263`):
`default_params` `{coilR: 70, coilL: 0.05, vPull: 3.5, vDrop: 1.5}`,
`electrical_specs` `vcc_range {3, 5, 12}`, `coilR_min 70`, `vPull_max 3.5`,
`vDrop_min 1.5`. Registered with `galvanicGroups` (`EA:187`, `191`).

### 2.1 Terminals and parameters

Pins by id: `coil_a`, `coil_b`, `com`, `no`, `nc` (`EA:225-229`, all
`function: "passive"`). No supply pin.

| Parameter | Read at | Rule | Default | Catalog |
|---|---|---|---|---|
| `coilR` (ohm) | `EA:231`, `293`, `329` | `max(1, .)` | 70 | 70 |
| `coilL` (H) | `EA:232`, `294`, `330` | `max(1e-6, .)` | 0.05 | 0.05 |
| `vPull` (V) | `EA:295` | none (not clamped) | 3.5 | 3.5 |
| `vDrop` (V) | `EA:296` | none (not clamped, not checked against `vPull`) | 1.5 | 1.5 |

Constant: `CONTACT_R = 1e-3` ohm (`EA:263`), not authorable.

Galvanic groups (`EA:187`): `[coil_a, coil_b]` and `[com, no, nc]`. The engine
uses them when it decides which parts of the circuit are isolated sections
(`SE:5568-5610`): the coil and the contact set are two separate sections that
are NOT conductively joined by the part, so each isolated section gets the
`ISOLATED_SECTION_ANCHOR_G = 1` S reference at one of its nodes (`SE:114`).
The port's section analysis must honour the same two groups.

### 2.2 Equations

State per relay (`RelayEngineState`, `SE:262-266`): `iCoil` (A, the coil
current a -> b committed from the last accepted step) and `energized`
(boolean). Default `{iCoil: 0, energized: false}` (`EA:234`, `299`).

Stamp (`EA:192-271`), per solve with step `h`:
1. Coil, transient: `stampResistor(coil_a, coil_b, R + L/h)` and
   `stampCurrentSource(coil_a, coil_b, -I_hist)` with
   `G = 1/(R + L/h)`, `I_hist = G*(L/h)*iCoil` (`EA:245-253`). That is the
   backward-Euler companion of section 1.4 with no back-EMF.
2. Coil, DC solve (`ctx.dcSolveMode()`): `stampResistor(coil_a, coil_b, R)`
   only (`EA:235-243`).
3. Contacts (`EA:262-270`): `stampResistor(com, no, 1e-3)` if
   `energized`, else `stampResistor(com, nc, 1e-3)`. The other contact is not
   stamped: an unused `nc` (or `no`) node is held by `RSHUNT` alone.

There is no coil-to-contact coupling in the MNA: the contact state is a
committed flag.

Post-solve (`commitState`, `EA:286-313`), with `x` the accepted solution and
`prev` the committed state:
- `vAb = V(coil_a) - V(coil_b)`;
- `G = 1/(R + L/h)`; `I_hist = G*(L/h)*prev.iCoil`;
- `iCoilNew = G*vAb + I_hist` (the companion formula, NOT the current in the
  matrix: they are equal because the stamp is linear and exact);
- `absV = |vAb|`; `energized` becomes true if `!prev.energized && absV >=
  vPull`, becomes false if `prev.energized && absV < vDrop`, else holds
  (`EA:303-310`).

`updateCurrent` (`EA:314-320`) publishes `iCoil` (the coil current a -> b), not
a contact current. The telemetry row for a relay is `reference: "coil"`
(`src/sim/physics-telemetry.ts:153-154`).

### 2.3 Dynamics and state

- One state variable of electrical memory (`iCoil`, inductor current); time
  constant `L/R = 0.7143 ms` at the defaults; steady current at 5 V is
  `5/70 = 71.43 mA`.
- Backward Euler only; no trapezoid; `h` is the solve's step including 555
  sub-steps and the seed's `1e-12` (section 1.3).
- `energized` is a committed boolean with a voltage hysteresis band
  `[vDrop, vPull)`.
- Carries across `load()` by component id (`SE:3190-3196`); snapshotted in
  `saveState` (`relays`, `SE:2617`).
- No contact bounce, no pull-in or drop-out time, no armature motion, no
  contact welding, no contact current rating, no coil self-heating, no change of
  `L` with armature position (it is the constant `coilL`).

### 2.4 Newton

None to speak of: every stamp is linear in `x`, so the first iterate lands on
the answer and the second confirms (convergence test of W3 section 1.3,
`vAbsTol 1e-6 + vRelTol 1e-3`). The relay never reads `xGuess`
(`EA:192`: `_xGuess`). A relay sets no limiting flag.

### 2.5 Regimes and events

- **Energised / released**, decided after every accepted solve from `|v_coil|`
  against `vPull` and `vDrop`, applied to the NEXT solve's contact stamp. The
  instant is not located: classic only observes the end-of-step voltage, and
  the step controller rejects a step in which the coarse and refined solves
  disagree on `energized` (section 1.6), so the instant is bracketed by
  halving to the host's minimum step.
- The decision uses the coil VOLTAGE, not the coil current: with a
  transistor driver whose saturation voltage is small this is equivalent, but
  with a coil fed through a series resistor the pull-in is at `vPull` across
  the coil, not at the datasheet's current.
- **In the DC operating point** the relay is not held: the OP ladder runs
  `commitState` (section 1.8), so `energized` can flip during the ladder from
  the stage's coil voltage, and `iCoil` is then overwritten by `vCoil/coilR`
  (`SE:4361-4373`). The first transient step after `load()` therefore starts
  with a relay already in the regime the seed solve saw.
- The `load()` seed solve (`SE:3437`, `h = 1e-12`) also runs `commitState`: a
  coil driven directly by a source reads its full voltage at the seed (the
  coil companion at `L/h = 5e10` ohm carries no current), so the relay is
  energised before the first real step if that voltage reaches `vPull`.

### 2.6 Small-signal AC

`acStamp` (`EA:321-341`): the coil is the continuous admittance
`1/(R + jwL)` stamped as `stampAcAdmittance(ac, a, b, R/den, -xl/den)` with
`xl = w*L` and `den = R^2 + xl^2`; the committed contact pair is a
`1/1e-3 = 1000` S conductance between `com` and `no` (energised) or `nc`; the
other contact stamps nothing. This is written separately from the transient
stamp: the transient coil impedance is `R + L/h`, the AC one is `R + jwL`, and
they agree only for `w*h` small (section 1.7). The armature has no small-signal
model.

### 2.7 Diagnostics

- `missing_flyback` (section 1.10): fires for a relay whose `coil_a` or `coil_b`
  net holds a `FLYBACK_SWITCH_KINDS` pin (and not a rail or `gnd`) and no
  diode/LED/zener/TVS pin. Tests `wave6-relay.test.ts:427-475`: fires with an
  NPN and no diode (`:428`), does not fire with an anti-parallel diode
  (`:451`), does not fire across a fixed supply with no switch (`:475`).
- No failure state: a relay cannot burn out, weld or exceed a rating.

### 2.8 Tests that pin it

`wave6-relay.test.ts`: `:261` coil at 5 V reaches `5/70 = 71.4 mA` and energises;
`:315` 0 V de-energises, COM-NC closed; `:355` hysteresis (a 2 V coil that was
de-energised stays de-energised, between 1.5 and 3.5); `:388` RL time
constant (current at 1 tau between 50 and 95 percent of steady); flyback
diagnostics `:427-492`. `wave6-hbridge.test.ts:916-995` pin the flyback
suppression with an L293D. `ac-isolation.test.ts:343-395` (the census).
`state-rollback-derived.test.ts` and `optimization-equivalence.test.ts` cover
snapshot and the bitwise oracle (not read in detail).

### 2.9 Questions

- **Q-W4-{relayrelease} (release voltage, recalled).** Classic releases at
  `|v| < 1.5 V` (30 percent of the 5 V coil rating) and pulls in at 3.5 V (70
  percent). The common SRD-05VDC-SL-C datasheet (Songle, recalled): coil 70 ohm,
  nominal 71.4 mA, pull-in at most 75 percent of nominal (3.75 V), release at
  least 10 percent (0.5 V). Classic's pull-in is inside that figure; its
  release is 1.0 V above the datasheet's minimum. **Recommend** reproduce; the
  catalog parameters are the authored values and a user can edit them.
- **Q-W4-{relaystate} (decision from voltage, not current, and one step late).**
  See 2.5. For a coil driven through a bipolar transistor the base-emitter and
  saturation drops make the coil voltage a few tenths of a volt below the rail,
  which is harmless; for a coil on a resistive divider the decision follows the
  voltage the divider develops while the real contact follows the ampere-turns.
  Classic also applies the new contact state one accepted step later.
  **Recommend** reproduce for 1.0.0 (committed flag applied at the next solve
  by the device-kernel regime commit, DK 1.5) and record the one-step lag in
  the port's gate; an event-located instant is a post-1.0.0 improvement.
- **Q-W4-{relaydcop} (relay not in the OP physical-time hold).** Section 2.5:
  the relay's `commitState` runs inside the DC ladder, so the OP's regime
  signature (W1 section 2.15, PLAN 1.3 "regime signature of the operating
  point") includes `energized` and can differ between ladder stages.
  **Recommend** mirror (the OP then reflects the coil voltage actually
  solved), and add `relay` to the W6a regime signature list.
- **Q-W4-{relaynoreverse} (contact model).** A 1 mohm contact with no rating,
  no contact resistance variation and no dielectric: it passes any current and
  any voltage, and a `com` left floating is held by RSHUNT only (about 1e12
  ohm). Datasheet contact resistance is at most 100 mohm initial (SRD-05VDC,
  recalled) and the contact is rated 10 A at 250 VAC / 30 VDC. **Recommend**
  reproduce; a contact-rating warning would be new behaviour.

---

## 3. DC brushed motor: dc_motor

[W4-E.] `dcMotorModel` (`EA:344-479`). Series R+L winding with a back-EMF and
a rotor with inertia, viscous friction and a constant load torque. Catalog
`dc-motor` (`CAT:8004`): `default_params` `{windingR: 5, windingL: 0.002, Ke:
0.01, Kt: 0.01, inertia: 1e-5, friction: 1e-5, loadTorque: 0}`; no
`vcc_range`.

### 3.1 Terminals and parameters

Pins by id: `m1` (+), `m2` (-) (`EA:382-383`; `function: "passive"`).

| Parameter | Read at | Rule | Default |
|---|---|---|---|
| `windingR` (ohm) | `EA:375`, `427`, `467` | `max(0.001, .)` | 5 |
| `windingL` (H) | `EA:376`, `428`, `468` | `max(1e-9, .)` | 0.002 |
| `Ke` (V s/rad) | `EA:377`, `429` | none | 0.01 |
| `Kt` (N m/A) | `EA:430` | none | 0.01 |
| `inertia` J (kg m^2) | `EA:431` | `max(1e-12, .)` | 1e-5 |
| `friction` b (N m s/rad) | `EA:432` | `max(0, .)` | 1e-5 |
| `loadTorque` TL (N m) | `EA:433` | none | 0 |

`Ke` and `Kt` are independent parameters (a lossless motor has them equal in
SI units; classic does not enforce it).

### 3.2 Equations

State `MotorEngineState {iWinding, omega}` (`SE:284-287`); default
`{iWinding: 0, omega: 0}` (`EA:379`, `435`; `SE:3201-3204`). `omega` is rad/s,
signed.

Stamp (`EA:347-410`) with the COMMITTED `omega`:
- `Vbemf = Ke * omega` (`EA:380`);
- transient: `G = 1/(R + L/h)`, `I_eq = G*((L/h)*iWinding - Vbemf)`,
  `stampResistor(m1, m2, R + L/h)`, `stampCurrentSource(m1, m2, -I_eq)`
  (`EA:397-409`);
- DC solve: `stampResistor(m1, m2, R)` and `stampCurrentSource(m1, m2,
  Vbemf/R)` (that is `-I_eqDc` with `I_eqDc = -Vbemf/R`, `EA:391-394`): the
  held rotor speed's back-EMF is a Norton offset, the inductive history is
  dropped.

Commit (`commitState`, `EA:426-454`), with `h` the accepted step:
- `G = 1/(R + L/h)`, `Vbemf = Ke*prev.omega`, `I_eq = G*((L/h)*prev.iWinding -
  Vbemf)`;
- `iWindingNew = G*(V(m1) - V(m2)) + I_eq` (`EA:447`);
- `torqueNet = Kt*iWindingNew - b*prev.omega - TL` (`EA:450`);
- `omegaNew = prev.omega + (h/J)*torqueNet` (`EA:451`): forward Euler on the
  speed, using the NEW current and the OLD speed.

`updateCurrent` publishes `iWinding` (`EA:455-461`). Telemetry
(`physics-telemetry.ts:240-249`): `windingCurrentA`, `angularVelocityRadPerS`,
reference `winding`.

### 3.3 Dynamics and state

- Electrical: one backward-Euler inductor, `L/R = 0.4 ms` at the defaults.
- Mechanical: `J d(omega)/dt = Kt*i - b*omega - TL`, forward Euler with the
  back-EMF in the electrical row taken from the previous step's `omega`
  (explicit coupling). Defaults: no-load speed at 6 V is `6/(Ke + R*b/Kt) = 400
  rad/s` with `i = 0.4 A`; the electromechanical time constant is
  `J/(b + Kt*Ke/R) = 0.333 s`; stall current at 6 V is `6/5 = 1.2 A`. All four
  figures are asserted by tests (section 3.8).
- Reverse polarity gives negative `omega` (no direction clamp); there is no
  static friction, no commutation ripple, no brush drop, no armature reaction,
  no thermal state, no torque limit and no coupling of `omega` back to any
  other component (the comment on `SE:270-278`: "NO mechanical coupling").
- Stability of the explicit loop: the scheme is stable when `h*(b +
  Kt*Ke/R)/J` is below 2 for the speed equation alone, and the lagged
  back-EMF adds the electrical coupling term; at the defaults this is `3*h`, so
  the worst-case 10 ms host step gives 0.03. A motor with a much smaller `J`
  (a coreless motor, `J` near 1e-7 with `Ke = Kt = 0.05`, `R = 2`) has
  `Kt*Ke/(R*J) = 1.25e4 /s` and an `h` limit near 1.6e-4 s, below the quiet-circuit
  host step. Classic has no guard; the host's error control would see the
  oscillation as a large `omega` error and shrink `h` (`ABS_SPEED = 1e-3`),
  which is the only protection (section 1.6). See the stability question
  below.
- Carries across `load()` (`SE:3198-3205`); snapshotted (`motors`).
- Held at its entry state during the OP ladder (section 1.8): `commitState` is
  skipped, `_commitDcOperatingPointState` sets `iWinding = (vWinding -
  Ke*omega_held)/windingR` and keeps `omega` (`SE:4375-4388`). A motor placed
  with a source directly across it has an OP at the STALL current with
  `omega = 0` (or its held value) and runs up from there.

### 3.4 Newton

Linear: no `xGuess` read (`EA:347`: `_xGuess`), no limiter. Back-EMF is
constant within a solve.

### 3.5 Regimes and events

None: no regime, no event. The rotor integrates continuously; a direction
reversal passes through `omega = 0` with no discontinuity.

### 3.6 Small-signal AC

`acStamp` (`EA:462-478`): only the winding admittance `1/(R + jwL)` between
`m1` and `m2` (the same `R/den`, `-xl/den` form as the relay). The comment says
the held `omega` makes the back-EMF "a constant term that vanishes in small
signal; electromechanical coupling (the motional impedance) is deliberately
outside this compact AC model". A real motor's impedance near mechanical
resonance includes `Ke*Kt/(b + jwJ)` in series; it is absent.

### 3.7 Diagnostics

`missing_flyback` for `m1`/`m2` (`SE:1092-1093`, section 1.10): tests
`wave7-dcmotor.test.ts:489` (motor switched by a bare NPN fires) and `:517`
(motor on an L293D with VM powered is suppressed because the L293D is not a
switch kind). No overcurrent, stall or thermal failure.

### 3.8 Tests that pin it

`wave7-dcmotor.test.ts`: stall current with `Ke = 0` (`:233-266`: 6 V, `i =
1.2 A`); no-load steady state (`:268-316`: `omega` near 400 rad/s, `i` near 0.4
A after about five mechanical time constants); back-EMF reduces current
monotonically (`:318-367`); reverse polarity gives negative `omega`
(`:369-397`); L293D forward and reverse drive (`:399-478`); flyback
diagnostics (`:480-566`); convergence at `h = 50 us` for 100 steps and after a
reversal (`:568-600`). `ac-isolation` census.

### 3.9 Questions

- **Q-W4-{motorload} (`loadTorque` has no sign).** `torqueNet = Kt*i - b*omega
  - TL` subtracts `TL` for every `omega`: an unpowered motor with `TL > 0`
  accelerates to NEGATIVE speed (`omega = -TL/b` at equilibrium, `-100 rad/s`
  for `TL = 1e-3`, `b = 1e-5`), forever. A load torque (a gear train,
  friction, a driven load) opposes motion: it should be `TL*sign(omega)` with a
  stiction band, or the model should say TL is a one-way driving torque.
  **Recommend** mirror for the port (record as a registered divergence per PLAN
  decision 7), and open a classic fix separately; do not copy silently.
- **Q-W4-{motorstab} (explicit back-EMF and forward-Euler speed).** Section
  3.3. The port can integrate the mechanics implicitly (a 2x2 coupled system,
  i and omega solved together, which is better conditioned and removes the
  one-step back-EMF lag) or reproduce classic. **Recommend** reproduce
  forward Euler for the gate against classic, and add the implicit form behind
  the same state record only if a coreless-motor test shows an instability;
  note the `h` limit in the port's diagnostics.
- **Q-W4-{motorkt} (`Ke` and `Kt` independent).** A circuit with `Kt != Ke`
  violates energy conservation (the motor develops more mechanical power than
  it takes electrically, or the reverse). **Recommend** reproduce; add a
  catalog note rather than a new rule.
- **Q-W4-{motordc} (OP at stall).** Section 3.3: an OP with a held `omega = 0`
  gives the stall current as the operating point; the first transient steps then
  show the current falling as the motor spins up. For a motor in a circuit with
  a supply limit this OP can differ from the settled behaviour (the current
  limit may or may not hold at stall). **Recommend** mirror (it is the
  documented DC-OP hold, `SE:765-775`).

---

## 4. Hobby servo: servo

[W4-E.] `servoModel` (`EA:481-657`) plus the sidecar `advanceServoMotion`
(`SP:1-66`). A rate-limited position actuator driven by a decoded PWM
command. It is "intentionally not a rigid-body gear-train simulation:
acceleration, backlash, load torque, stall heating and control-loop hunting
remain outside the model" (`SP:1-9`). Catalog `servo-sg90` (`CAT:8038`):
`default_params` `{idleR: 330, movingCurrent: 0.15, maxSpeedDegPerSecond: 600,
nominalVoltage: 4.8, minimumOperatingVoltage: 4.5, minPulseMs: 1, maxPulseMs:
2, minAngle: 0, maxAngle: 180}`; `electrical_specs.vcc_range {4.5, 4.8, 6}`,
`if_max 0.7`.

### 4.1 Terminals and parameters

Pins by id: `sig` (`function: "input"`), `vplus` (`vcc`), `gnd` (`gnd`)
(`EA:492-493`, `511-512`). `sig` has no stamp: it is a genuinely high-impedance
input (`EA:487-491` and the catalog note: "no 1 Mohm pull-down is present";
`digital-input-impedance.test.ts` pins it). Open `sig` reads its level through
the 2.0 V threshold on a node held by RSHUNT alone, i.e. as low.

| Parameter | Read at | Rule | Default |
|---|---|---|---|
| `idleR` (ohm) | `EA:118` | `max(1, .)` | 330 |
| `movingCurrent` (A) | `EA:129` | `max(0, .)`; 0 means use `idleR` | 0.15 |
| `nominalVoltage` (V) | `EA:125-128`, `607-610` | `max(0.1, param ?? vcc_range.nominal ?? 4.8)` | 4.8 |
| `minimumOperatingVoltage` (V) | `EA:611-614` | `max(0, param ?? vcc_range.min ?? 4.5)` | 4.5 |
| `maxSpeedDegPerSecond` (deg/s) | `EA:602-605` | `max(0, param ?? speedDegPerSecond ?? 600)` | 600 |
| `minPulseMs`, `maxPulseMs` (ms) | `EA:520-521` | none; both unchecked | 1.0, 2.0 |
| `minAngle`, `maxAngle` (deg) | `EA:522-523` | none (the sidecar clamps `maxAngle >= minAngle`) | 0, 180 |

Constants: `SIG_THRESH = 2.0` V (`EA:515`), the TTL threshold "used
throughout the engine"; it is an ABSOLUTE comparison of `V(sig) - V(gnd)`, not a
fraction of the servo's own supply.

### 4.2 Equations

State `ServoEngineState` (`SE:315-326`): `lastSig` (0/1), `riseT` (s, NaN
until the first rising edge), `pulseMs` (NaN until the first pulse),
`targetAngle`, `angle`, `velocity` (deg/s), `moving`, `powered`,
`supplyVoltage`, `loadResistance` (ohm, the value STAMPED for the accepted
step). Default (`SE:331-344`): `lastSig 0, riseT NaN, pulseMs NaN, targetAngle
90, angle 90, velocity 0, moving false, powered false, supplyVoltage 0,
loadResistance NaN`.

**Stamp (`EA:484-496`).** Only the supply branch: `stampResistor(vplus, gnd,
servoLoadResistance)`, with the helper (`EA:112-131`):
`idleR = max(1, idleR_param)`; if the state is absent, or `!moving`, or
`|velocity| <= 1e-9`, or `!powered`, then `idleR`; otherwise
`nominalVoltage = max(0.1, param ?? vcc_range.nominal ?? 4.8)`,
`movingCurrent = max(0, param ?? 0.15)` and `R = movingCurrent > 0 ?
max(0.1, nominalVoltage/movingCurrent) : idleR`. At the defaults: idle `330`
ohm (15.2 mA at 5 V), moving `4.8/0.15 = 32` ohm (156 mA at 5 V). The load is
chosen from committed state, so the stamp never switches inside a Newton
iteration and lags the motion by one accepted step.

**Commit (`EA:509-631`).** In order:
1. `loadResistance = servoLoadResistance(...)` evaluated at the START of the
   commit from the OLD state (`EA:510`); it is the value that was stamped.
2. `vSig = V(sig) - V(gnd)`; `sampledSig = vSig >= 2.0 ? 1 : 0` (`EA:513-516`).
3. Driver lookup: `driver = ctx.mcuEventDriverForNode(sig node)` (the shared
   map with `includePico = true`, so Uno, Nano and Pico, section 1.5);
   `driverMcu = state.arduinos.get(driver.compId)`; `driverPowered =
   ctx.mcuPowered(driverComp, x)`; `driverPinEvents =
   ctx.mcuStepPinEvents(driver.compId)` filtered to `event.pin ===
   driver.pin && event.level !== null`; `finalDrive = driverMcu.pinDriveState(
   driver.pin)` (`EA:529-542`).
4. `hasEventDriver = driver && driverComp && driverMcu && driverPowered &&
   (finalDrive is "out-high" or "out-low" or driverPinEvents.length > 0)`
   (`EA:543-549`).
5. If `hasEventDriver`: edges are `{level: event.level, time: bootT +
   event.cycle/clockHz}` for each filtered event, sorted by time
   (`EA:551-558`); else if `sampledSig !== lastSig`: ONE edge
   `{level: sampledSig, time: ctx.simTime()}` (the step START; `EA:558-561`).
6. `applyEdge` (`EA:563-575`) over the edges in order: an edge whose level equals
   `lastSig` is ignored; a rising edge sets `riseT = edge.time`; a falling edge
   with a finite `riseT` and `edge.time >= riseT` sets `measuredMs =
   (edge.time - riseT)*1000`, `pulseMs = clamp(measuredMs, minPulse, maxPulse)`
   and `targetAngle = minAngle + ((pulseMs - minPulse)/max(1e-9, maxPulse -
   minPulse))*(maxAngle - minAngle)`; then `lastSig = edge.level`.
7. Reconcile (`EA:579-589`): when the MCU is the driver, `finalLevel` is 1 for
   `out-high`, 0 for `out-low`, `sampledSig` for any input state; if it
   differs from `lastSig`, one more edge `{level: finalLevel, time:
   ctx.simTime() + h}` is applied. This catches a pin that was set to its level
   with no non-null event (after a reload or reset).
8. `supplyVoltage = max(0, V(vplus) - V(gnd))` (`EA:592-596`).
9. Motion: `advanceServoMotion({angleDeg: prev.angle, targetAngleDeg: target,
   dtSeconds: h, maxSpeedDegPerSecond, supplyVoltageV, nominalSupplyVoltageV,
   minimumOperatingVoltageV, minAngleDeg, maxAngleDeg})` (`EA:598-617`).
10. New state: the decoded fields plus `angle = motion.angleDeg`, `velocity =
   motion.velocityDegPerSecond`, `moving = motion.moving && |velocity| > 1e-9`,
   `powered = motion.powered`, `supplyVoltage`, `loadResistance` (`EA:619-630`).

**Sidecar `advanceServoMotion` (`SP:28-65`).** With `finite(v, fallback)`
replacing non-finite values:
- `minAngle = finite(minAngleDeg, 0)`; `maxAngle = max(minAngle,
  finite(maxAngleDeg, 180))`; `angle = clamp(finite(angleDeg, 90), minAngle,
  maxAngle)`; `target = clamp(finite(targetAngleDeg, angle), minAngle,
  maxAngle)`;
- `dt = max(0, finite(dtSeconds, 0))`; `nominal = max(1e-9,
  finite(nominalSupplyVoltageV, 6))`; `minOp = max(0,
  finite(minimumOperatingVoltageV, 3))`; `supply = max(0, finite(
  supplyVoltageV, 0))` (the handler always passes 4.8 and 4.5, so the
  sidecar's own defaults 6 and 3 are unused);
- `powered = supply >= minOp`; `speedScale = powered ? clamp(supply/nominal, 0,
  1.5) : 0`;
- `maxSpeed = max(0, finite(maxSpeed, 0))*speedScale`; `error = target - angle`;
  `maxTravel = maxSpeed*dt`; `travel = clamp(error, -maxTravel, maxTravel)`;
  `nextAngle = clamp(angle + travel, minAngle, maxAngle)`; `velocity = dt > 0 ?
  travel/dt : 0`; `moving = |target - nextAngle| > 1e-9`.
At 5 V the speed is `600*5/4.8 = 625 deg/s`; a 0 to 180 degree move takes
0.288 s. Travel in one step is exactly `min(|error|, maxTravel)`: no
acceleration, no overshoot.

### 4.3 Dynamics and state

- The command is held between pulses ("loss of signal therefore does not
  teleport or reset the shaft", `SE:303-307`): there is no signal-loss timeout
  and no refresh requirement; one complete pulse commands the target
  forever.
- A pulse outside `[minPulse, maxPulse]` is clamped (0.5 ms reads as 0
  degrees; 3 ms as 180).
- The electrical supply load switches between two resistors by committed
  `moving && powered`; the shaft motion and the load are decoupled from the
  actual angle error (a stalled servo cannot draw more).
- The first valid pulse needs a rising edge followed by a falling edge: a
  signal held high from the start gives no target; the target stays at 90.
- Held at its entry state in the OP ladder (section 1.8); carried across
  `load()` (`SE:3206-3231`).

### 4.4 Newton

No `xGuess` read; the stamp is a fixed resistor per solve.

### 4.5 Regimes and events

- **Powered / unpowered** (`supply >= minOp`): below it `speedScale = 0` and the
  shaft stops while the target is retained.
- **Idle / moving** load: committed one step late (above).
- **Edge timing.** On the event path the edge times are exact to one MCU clock
  cycle (62.5 ns at 16 MHz, 8 ns at 125 MHz); the pulse width is independent of
  the host step (tests `servo-engine-physics.test.ts:96`, `mcu-servo-replay.test.ts:263`).
  On the sampled path the single edge per step is stamped at the STEP START
  (`ctx.simTime()`), so a width measured between a rising edge seen at the end
  of step `k` (timed at the start of step `k`) and a falling edge likewise has
  an error of up to one step on each side but cancels to the step-quantised
  width; with a square source whose edges the host lands on (source events,
  `HL:1118-1135`) it is exact. A pulse shorter than one host step is lost on the
  sampled path.
- **Which drivers get the event path.** `includePico = true`: Uno, Nano,
  Pico. A micro:bit does not (its pins are `setMicrobitDrive` levels, section
  14.9) so it takes the sampled path. A servo whose `sig` net has two MCU pins
  uses the last in `mcu.ioPins` order (last writer wins, `SE:7356`).

### 4.6 Small-signal AC

`acStamp` (`EA:644-656`): `1/servoLoadResistance` between `vplus` and `gnd`,
reading the committed state (so the AC load equals the load the OP's last
transient step stamped). `sig` stamps nothing. PWM decode and shaft motion
have no small-signal model.

### 4.7 Diagnostics

The catalog has `vcc_range`, so the output-sag scan visits the servo
(`SE:7553-7690`), but `_drivenOutputTargets` returns nothing for it; no failure
kind exists. Telemetry (`physics-telemetry.ts:250-271`): `angleDeg`,
`targetAngleDeg`, `velocityDegPerS`, `moving`, `powered`, `supplyVoltageV`,
`signalHigh`, `pulseWidthMs`, `lastRiseSimTimeS`, reference `supply`.

### 4.8 Tests that pin it

`wave7-servo-stepper.test.ts`: idle current `5/330 = 15.15 mA` (`:212`);
`sig` is high-impedance (`:251`: driving `sig` with 3.3 V does not disturb a 5
V supply); decode 1.5 ms to 90 deg (`:286`), 1.0 ms to 0 and 2.0 ms to 180
(`:361`, `:407`); target holds between pulses (`:451`); convergence (`:814`).
`servo-engine-physics.test.ts` (`:96-186`): cycle-precision decode of an
Arduino pulse "instead of 100 us / 18 degree steps" (`:96`), no teleport and
supply-scaled speed (`:125`), stop below the brownout voltage while keeping the
target (`:145`), the moving current drawn through the solved supply (`:158`),
exact replay after adaptive rollback (`:170`). `mcu-servo-replay.test.ts`
`:240-338` (events counted once under the 555 split; a servo and an HC-SR04 on
one MCU). `servo-physics.test.ts:4-66` (rate limit, exact landing, supply
scaling and brownout, angle clamps).

### 4.9 Questions

- **Q-W4-{servosample} (sampled-path edge quantisation).** For a PWM source
  that is not an MCU pin (a signal generator, a 555, a clock) the servo
  decodes at step ends, timing each edge at the step START; classic's own
  step controller does not clamp `h` to the 1 to 2 ms pulse structure, so at
  `h = H_MAX` the measured width is only as good as the source-event clamp
  (`HL:1118-1135` lands steps on square-source edges but a 555 is not a
  "source"). **Recommend** the core decodes servo `sig` from scheduled edges
  of its driver (a logic output's event, DK 1.5) so width is exact for any
  digital driver, and keeps the 2.0 V threshold sampling only for analog drives.
- **Q-W4-{servothresh} (absolute 2.0 V threshold).** A servo driven from a 1.8 V
  logic output never decodes; an SG90 accepts 3 to 5 V signals (recalled).
  **Recommend** reproduce.
- **Q-W4-{servoload} (load switches one step late and the stall load is
  absent).** The 32 ohm moving load is stamped for the step after the shaft
  starts moving and released the step after it lands; a servo that holds an
  angle against a load, or stalls, draws the idle 15 mA. SG90 stall is about
  650 mA and idle 10 mA (recalled). The catalog note says "stall mechanics/
  current limiting ... are not modelled", so this is by design. **Recommend**
  reproduce.
- **Q-W4-{servospeed} (speed scale up to 1.5x).** `speedScale` caps at 1.5, so a
  6 V supply gives `600*6/4.8 = 750 deg/s` (1.25x) and a 7.2 V rail the cap
  900 deg/s; no max-voltage failure exists even though `vcc_range.max` is 6.
  **Recommend** reproduce.
- **Q-W4-{servoevent} (the event path needs a powered MCU and ignores the
  signal's real voltage).** `hasEventDriver` requires `mcuPowered`; the decode
  takes the MCU's logical level even if the net is held low by a short (the
  electrical level never enters). **Recommend** reproduce; the board shell
  (section 14) is the source of truth for drive.

---

## 5. Bipolar stepper: stepper

[W4-E.] `stepperModel` (`EA:659-819`). Two independent R+L coils and a phase
state machine on the coil currents. No rotor model. Catalog
`stepper-bipolar-nema17` (`CAT:8088`): `default_params` `{coilR: 10, coilL:
0.01, stepsPerRev: 200}`.

### 5.1 Terminals and parameters

Pins by id: `a1`, `a2` (coil A), `b1`, `b2` (coil B) (`EA:689-692`, all
`function: "passive"`). Parameters: `coilR` `max(0.001, ., default 10)` and
`coilL` `max(1e-9, ., default 0.01)` (`EA:684-685`, `729-730`, `800-801`);
the catalog's `stepsPerRev` is NOT read by the engine (only the host reads it,
`SE:365-366`: `angle = position * 360 / stepsPerRev`).

### 5.2 Equations

State `StepperEngineState {iA, iB, phase, lastKnownPhase, position}`
(`SE:367-373`), default `{0, 0, -1, -1, 0}` (`EA:687`, `732`; `SE:3237-3241`).

Stamp (`EA:662-714`): for each coil two elements, exactly the relay's:
`stampResistor(a1, a2, R + L/h)`, `stampCurrentSource(a1, a2, -I_histA)` with
`G = 1/(R + L/h)`, `I_histA = G*(L/h)*iA`, and the same for B with `iB`. DC
solve: `stampResistor(a1, a2, R)` and `stampResistor(b1, b2, R)` only
(`EA:694-702`). No back-EMF ("v1 omits back-EMF for simplicity",
`EA:666-667`).

Commit (`EA:728-788`): `iANew = G*(V(a1) - V(a2)) + I_histA`, `iBNew` likewise;
`signA = 0 if |iANew| < 1e-3 else sign(iANew)` (the 1 mA `DEADBAND`, `EA:752`),
same for B. `newPhase` from the sign pair: (+,+) 0, (-,+) 1, (-,-) 2, (+,-) 3,
anything with a zero sign -1 (`EA:758-763`). `newLastKnownPhase = newPhase >= 0
? newPhase : prev.lastKnownPhase` (`EA:768`). Position: if `newPhase >= 0` and
`prev.lastKnownPhase >= 0`, `delta = (newPhase - prev.lastKnownPhase + 4) % 4`;
`delta == 1` gives `position++`, `delta == 3` gives `position--`, `0` and `2`
hold (`EA:770-781`). The new state is written whole (`EA:783-787`).

`updateCurrent` publishes `|iA| + |iB|` (`EA:789-795`); telemetry
`reference: "package-total"`, `aggregation: "absolute-sum"`, plus `coilCurrentAA`,
`coilCurrentBA`, `positionSteps`, `phaseIndex` (`physics-telemetry.ts:273-290`).

### 5.3 Dynamics and state

- Electrical: two independent backward-Euler inductors with `tau = L/R =
  1 ms` at the defaults; steady current per coil at 5 V is `0.5 A`.
- The "phase" machine is instantaneous and memoryless apart from
  `lastKnownPhase`. There is no rotor, no detent, no holding torque, no
  step loss, no resonance and no back-EMF: a stepper driven too fast simply
  counts every valid transition.
- Counted transitions are only the full-step, two-coils-on Gray sequence
  0 -> 1 -> 2 -> 3 -> 0 (forward) or the reverse. A one-coil-at-a-time (wave
  drive) or half-step sequence passes through `phase = -1` (a zero sign) and
  `lastKnownPhase` skips it: wave drive 0 -> (A only: -1) -> 1 counts as one
  step 0 -> 1 across the gap, so wave drive is read as the corresponding
  full-step motion. A repeated phase (`delta 0`) and a skip (`delta 2`) hold.
- The first valid phase after power-on sets `lastKnownPhase` and counts
  nothing (`prev.lastKnownPhase >= 0` is required).
- Held in the OP ladder; the OP sets `iA`, `iB` (`SE:4389-4400`) but not the
  phase fields.

### 5.4 Newton

Linear; no `xGuess` read.

### 5.5 Regimes and events

The phase is a committed integer evaluated each accepted step; changes are
bracketed by the controller's exact compare of `phase`, `lastKnownPhase` and
`position` (section 1.6). At a high step rate (the 1 mA deadband and the 1 ms
coil constant) two transitions inside one host step count as one (`delta 2`
holds) or are lost.

### 5.6 Small-signal AC

`acStamp` (`EA:796-818`): two `stampAcAdmittance(a1,a2,R/den,-xl/den)` and the
same for B with `xl = w*L`, `den = R^2 + xl^2`: the same continuous winding
admittance the relay and motor get; "Phase and position are step-counting
mechanics with no small-signal model".

### 5.7 Diagnostics

The ENGINE raises no `missing_flyback` for the stepper: `inductiveWindingPins`
knows only `inductor`, `relay` and `dc_motor` (`SE:1084-1094`). The stepper's
flyback finding (key `missing-flyback:stp:coilA`) comes from the host's static
rule engine, which the engine tests exercise through a test-support copy
(`test/helpers/diagnostics/static.ts:1132-1175` for the stepper coils;
header lines 1-3: "Test-support copy extracted from devolt"). That rule
engine, not the solver, owns the H-bridge suppression: `hasHBridgeClamp`
(`static.ts:918-969`) requires both coil nets to land on output pins of one
`l293d` or `tb6612` AND that part's VM pin (`vcc2` / `vm`) to be on a net with
at least two pins that holds a supply kind or a coil terminal. So the catalog
note "suppressed by L293D/TB6612 with VM powered" is a property of the app's
diagnostics, outside this port. (The test at
`wave7-servo-stepper.test.ts:728-812` asserts through `runDiagnostics`, not
through `SimEngine`.)

### 5.8 Tests that pin it

`wave7-servo-stepper.test.ts`: coil A reaches `V/R` (`:505-538`: 5 V, 10 ohm,
`iA` near 0.5 A after about ten `tau_L`); `elementI = |iA| + |iB|` (`:540-571`);
phase detection (`:573-633`: (+,+) is 0, (-,+) is 1, unenergised is -1);
position advances by 4 over the sequence 0,1,2,3,0 (`:635-698`); convergence
(`:700-726`); flyback diagnostics (`:728-812`).

### 5.9 Questions

- **Q-W4-{stepfly} (two flyback rule sets).** The engine's `missing_flyback`
  covers inductor, relay and dc_motor (a `SimWarning`, `SE:1117`); the host's
  static diagnostics cover relay, motor, stepper coil A/B and inductor, with
  the H-bridge suppression (`test/helpers/diagnostics/static.ts:918`). A user of
  the core sees the engine's warning only. **Recommend** the core ports only
  the engine's warning (`missingFlybackWarning` is already pure, `SE:1117`, and
  the core facade derives it from the graph, W3 section 1.7), and the stepper
  stays host-only; record the difference so the catalog note is corrected.
- **Q-W4-{stepmode} (half-step, wave drive and microstepping).** The counter
  treats any sign pair with a zero as a gap and counts only the full-step
  Gray sequence; a half-step drive (A+, A+B+, B+, ...) counts one step per
  full-step equivalent only if intermediate phases are seen. Microstepping
  (PWM currents) with all four signs present and changing slowly counts as
  full steps at sign changes only. **Recommend** reproduce.
- **Q-W4-{steprotor} (no rotor, no back-EMF).** A stepper with no back-EMF
  draws the full `V/R` at any speed; a real NEMA17 current falls with speed
  and a chopper driver limits it. Position leads or lags no real shaft.
  **Recommend** reproduce; the telemetry only reports commanded steps.
- **Q-W4-{stepdead} (1 mA deadband and sign-only phase).** Coil currents below
  1 mA read as zero; a driver releasing a coil to HIZ with a flyback tail
  reads zero within a few `tau`. Sign alone decides phase, so a coil at 1
  mA and one at 500 mA are the same phase. **Recommend** reproduce.

---

## 6. Buzzer: buzzer

[W4-A.] `buzzerModel` (`EA:1161-1308`), with its constants at `EA:1146-1159`. A
fixed resistor plus a post-solve "is it sounding" detector; there is no audio.
Catalog `buzzer-active-5v` (`CAT:7630`, `{type: "active", resistance: 32}`) and
`buzzer-passive-5v` (`CAT:7659`, `{type: "passive", resistance: 32}`); neither
has `vcc_range`.

### 6.1 Terminals and parameters

Pins by POSITION: `pins[0]` and `pins[1]` (ids `p1`, `p2` in the catalog); the
handler returns immediately when `pins.length < 2` (`EA:1175`, `1206`,
`1289`, `1298`). Voltage is `V(pins[0]) - V(pins[1])`.

| Parameter | Read at | Rule | Default |
|---|---|---|---|
| `resistance` (ohm) | `EA:1176`, `1290`, `1299` | `max(1, .)` | 32 |
| `type` | `EA:1210` | string; `"passive"` selects the tone detector, anything else is active | `"active"` |

Constants (`EA:1146-1154`): `PASSIVE_ENV_TAU = 0.1` s, `PASSIVE_HYSTERESIS =
0.1`, `PASSIVE_MIN_SWING = 0.05` V, `AUDIBLE_MIN_HZ = 20`, `AUDIBLE_MAX_HZ =
20000`; the active threshold `1.5` V is a literal (`EA:1274`).

### 6.2 Equations

Stamp (`EA:1164-1183`): `stampResistor(pins[0], pins[1], R)` for both types
(`R = max(1, resistance)`). Current published (`EA:1286-1294`): `(V(p1) -
V(p2))/R`. The buzzer is a plain linear resistor to the circuit; polarity does
not matter electrically.

Detector state in `icState` (default `{lastSign: NaN, lastCrossT: NaN,
prevCrossT: NaN, envHi: NaN, envLo: NaN, detectedHz: 0}`, `SE:1644-1646`; the
`sounding` key is added by the first commit). Per accepted step with `v =
V(p1) - V(p2)` (`EA:1205-1285`):

**Active.** `sounding = |v| >= 1.5 ? 1 : 0` (`EA:1273-1275`); the passive fields
are written back unchanged.

**Passive** (`EA:1218-1266`), with `h` the step, `stepStart = ctx.simTime()`,
`now = simTime + h`:
1. Envelope: if `envHi` and `envLo` are finite, `keep = exp(-h/0.1)`, `envHi =
   max(v, v + (prevHi - v)*keep)`, `envLo = min(v, v + (prevLo - v)*keep)`;
   else both equal `v` (`EA:1230-1237`). The envelope jumps to each new extreme
   and relaxes toward the signal with tau 0.1 s.
2. `swing = envHi - envLo`. If `swing >= 0.05`:
   - `mid = (envHi + envLo)/2`, `band = 0.1*swing`;
   - if an envelope existed and its previous swing `prevHi - prevLo < 0.05`
     (the drive has just started), `from = bandSide((prevHi + prevLo)/2, mid,
     band)` and, if nonzero, `lastSign = from` (so the first edge of a drive
     is not lost, `EA:1245-1248`);
   - `curSign = bandSide(v, mid, band)` with `bandSide(v, m, b) = v > m + b ?
     1 : v < m - b ? -1 : 0` (`EA:1157-1159`);
   - if `curSign != 0`: when `lastSign == -curSign` (a crossing), and
     `lastCrossT` is finite, `period = isFinite(prevCrossT) ? stepStart -
     prevCrossT : 2*(stepStart - lastCrossT)` and `hz = period > 0 ? 1/period :
     0`, `detectedHz = (20 <= hz <= 20000) ? hz : 0`; then `prevCrossT =
     lastCrossT`, `lastCrossT = stepStart` (`EA:1251-1261`); finally
     `lastSign = curSign`.
3. If `detectedHz > 0` and `now - lastCrossT > 2/detectedHz`, `detectedHz = 0`
   (`EA:1265`).
4. `sounding = detectedHz > 0 ? 1 : 0`.

The crossing time is `stepStart` (the START of the step in which the crossing
was first seen at the step's end), by design: "Backward Euler holds a step's
end state across the whole step, so a crossing seen at this step's end is timed
at its start" (`EA:1220-1225`), so that a clock edge crossed by a 10 ns guard
step and one first seen by the next full step read the same instant.

### 6.3 Dynamics and state

The only memory is the detector (the envelope, two crossing times, the last
band side, the estimate). Nothing in the stamp reads it: the part's electrical
behaviour is a constant resistor. `envHi`/`envLo` are excluded from the step
controller's error estimate; `lastCrossT`, `prevCrossT` and `detectedHz` are
compared with tolerance (section 1.6). Telemetry: `sounding` and, for passive,
`toneHz = detectedHz` (`physics-telemetry.ts:306-319`), `quality:
"behavioral-estimate"`.

### 6.4 Newton

Linear; no `xGuess`.

### 6.5 Regimes and events

Sounding / silent. The tone detector finds its crossings only at accepted-step
ends, so a drive whose edges the host steps over is read at the sampling
rate: at the MCU step cap `h = 1e-4` s the detector aliases above about
`1/(2h) = 5 kHz` (PLAN 1.3, row "max7219/hd44780 replay, micro:bit edge timing, buzzer
aliasing above ~5 kHz at 100 us"), and a tone near 4 kHz (period 2.5 steps) reads a jittering
`hz` because each crossing is seen at the next step boundary. The test suite
exercises a square clock source (whose edges the host lands on), not an MCU
`tone()`.

### 6.6 Small-signal AC

`acStamp` (`EA:1295-1307`): `1/R` between the two pins; no motional or
reactive part; the same for both types.

### 6.7 Diagnostics

None. (`vcc_range` is absent, so the output-sag scan skips it.)

### 6.8 Tests that pin it

`wave6-drivers-audio.test.ts`: active type sounds at 5 V and not at 0.5 V
(`:398-460`); passive detects Hz > 0 after a 1 kHz square for 30 steps
(`:461-517`); crossings relative to the drive's own midpoint (`:518-547`
`it.each` at `:535`); 440 Hz held when edges are crossed by guard steps and
full steps in turn (`:548`); stops sounding within two periods of the drive
stopping (`:580`); one press of a button (0 to 5 V to 0) does not report a tone
(`:591`).

### 6.9 Questions

- **Q-W4-{buzzalias} (detection at step ends).** Section 6.5. The core's
  event-exact edges (PLAN W4-A, "event-exact edges remove the replay") make
  the Schmitt detector see every logic edge at its true time, which would
  change `detectedHz` for MCU-driven tones from a step-quantised value to the
  exact one. **Recommend** use exact edge times for an MCU or logic driver and
  keep the sampled Schmitt for analog drive; the gate is "closer to the analytic
  answer" (PLAN decision 7 (b)).
- **Q-W4-{buzzthresh} (active threshold).** A literal 1.5 V against typical
  active buzzers that need 3 to 5.5 V and draw 25 to 35 mA (e.g. a 5 V module
  with a built-in oscillator, recalled); classic's resistor also draws `V/32`
  (156 mA at 5 V) while a real 5 V active buzzer draws about 30 mA, so the supply
  sags more than reality. **Recommend** reproduce; the catalog's `resistance`
  is authorable.
- **Q-W4-{buzzpolar} (polarity ignored).** The active buzzer sounds for
  `|v| >= 1.5` in either polarity; a real polarised active buzzer does not.
  **Recommend** reproduce.

---

## 7. Speaker: speaker

[W4-A.] `speakerModel` (`EA:1310-1402`). A fixed resistor and a 5 ms
peak-to-peak window. Catalog `speaker-8ohm` (`CAT:7688`, `{resistance: 8}`).

### 7.1 Terminals and parameters

Pins by position `pins[0]`, `pins[1]` (`p1`, `p2`); `resistance` `max(1, .,
default 8)` (`EA:1321`, `1383`, `1393`). Constants in the handler:
`SPK_WINDOW = 0.005` s and `SPK_THRESHOLD = 0.05` V (`EA:1345-1346`).

### 7.2 Equations

Stamp `stampResistor(p1, p2, R)` (`EA:1313-1328`); current `(V(p1)-V(p2))/R`
(`EA:1379-1387`). The comment says the voice-coil inductance is ignored.

Detector (`commitState`, `EA:1339-1378`), state `{vPeak, vTrough, windowStart,
peakToPeak, sounding}` (default `{NaN, NaN, NaN, 0}`, `SE:1652-1653`); with `v
= V(p1) - V(p2)` and `now = simTime + h`:
- first sample (`windowStart` is NaN): `windowStart = now`, `vPeak = vTrough =
  v`;
- else if `now - windowStart >= 0.005`: `peakToPeak = (isNaN(vPeak) ||
  isNaN(vTrough)) ? 0 : vPeak - vTrough`; `windowStart = now`; `vPeak = vTrough
  = v` (the current sample starts the next window and is NOT part of the
  committed window);
- else: running max/min of `v`.
`sounding = peakToPeak >= 0.05 ? 1 : 0` (`EA:1370`). The committed
`peakToPeak` is held until the next window closes.

### 7.3 Dynamics and state

Window-based; no relaxation. `sounding` therefore lags the signal by up to one
window (5 ms) after it starts and persists for up to one window after it stops.

### 7.4 Newton, regimes, events

Linear. Two regimes (`sounding` 0/1) decided once per window.

### 7.5 Small-signal AC

`1/R` between the pins (`EA:1388-1401`).

### 7.6 Diagnostics and tests

None. Tests `wave6-drivers-audio.test.ts:609-675`: resistive stamp `I = 5/8 =
0.625 A` (`:610`), signal present when `p2p >= 50 mV` after a 5 ms window
(`:636`).

### 7.7 Questions

- **Q-W4-{spkstep} (the detector breaks for host steps of 5 ms or more).** When
  every accepted step ends at least 5 ms after the window start
  (`h >= 5 ms`, which a quiet circuit reaches: `H_MAX = 1e-2`, `HL:83`), the
  window closes on every step, `vPeak` and `vTrough` were set to that single
  previous sample, and the committed `peakToPeak` is always 0: the speaker
  reads silent for any signal, however large. Under a changing source the
  headless runner caps `h` at `ACTIVE_H_MAX = 2e-3` (`HL:93`), which keeps
  several samples in a window; the cap is a host property, not the model's.
  **Recommend** reproduce for the gate, record it as a registered divergence,
  and make the core's window sample-count independent (an envelope on every
  accepted step regardless of `h`).
- **Q-W4-{spkload} (resistive only).** A real 8 ohm speaker is 6 to 7 ohm DC
  and 8 ohm nominal at 1 kHz with `Le` of order 50 uH to 1 mH and a resonance
  peak at tens to hundreds of Hz (recalled); classic is the DC resistance.
  **Recommend** reproduce (the catalog note says "AC reactance ... is ignored").

---

## 8. Ultrasonic ranger: hcsr04

[W4-A.] `hcsr04Model` (`EA:821-990`). An HC-SR04 module as an electrical
load, a TRIG pull-down input, a push-pull ECHO output and a timing state
machine; no acoustics. Catalog `hcsr04` (`CAT:8344`): `default_params
{distanceCm: 50, noEcho: 0}`, `electrical_specs.vcc_range {3, 5, 5.5}`.

### 8.1 Terminals and parameters

Pins by id: `vcc`, `trig` (`input`), `echo` (`output`), `gnd` (`EA:829-830`,
`847-848`, `866-867`, `878`).

| Parameter | Read at | Rule | Default |
|---|---|---|---|
| `distanceCm` | `EA:905` | `max(2, min(400, .))` | 50 |
| `noEcho` | `EA:906` | nonzero is true (out of range) | 0 |

Constants: `TRIG_R = 1e6` ohm (`EA:828`); the supply threshold `3.0` V
(`EA:843`, `869`, `966`, `979`); the quiescent current `0.002` A (`EA:855`,
`966`); the TRIG logic threshold `2.0` V (`EA:901`); the schedule constants
`250e-6` s, `58.0e-6` s per cm, `38e-3` s (`EA:914-915`); `GATE_R_OUT = 50` ohm
for the ECHO stage (`EA:850`).

### 8.2 Equations

State `Hcsr04EngineState` (`SE:422-430`): `trigLevel` (0/1), `phase` (`idle`,
`armed`, `pending`, `active`), `echoRiseAt`, `echoFallAt` (s, NaN when not
scheduled), `echoOut` (0/1), `vccV` (committed VCC-GND), `pendingEchoEvent`.
Default `{0, "idle", NaN, NaN, 0, 0, null}` (`SE:434-439`).

**Stamp** (`EA:824-857`), from committed state:
1. `stampResistor(trig, gnd, 1e6)` ALWAYS, powered or not (`EA:828-831`).
2. `physicallyPowered = state && vccV >= 3.0 && !isOpenPin(vcc) &&
   !isOpenPin(gnd)` (`EA:843-845`). If true:
   - `stampResistor(echo, echoOut ? vcc : gnd, 50)` (`EA:846-851`): a push-pull
     Thevenin output to the part's OWN supply node (so a 3.3 V rail gives a
     3.3 V high, "wide-supply");
   - `stampCurrentSource(gnd, vcc, 0.002)` (`EA:855`): by the sign convention of
     the header, 2 mA is injected into the `gnd` node and removed from `vcc`,
     i.e. the module draws 2 mA from VCC and returns it at GND; a constant
     current, not a resistor, so it does not fall with the supply (the comment:
     "stamped, rather than merely reported ... so batteries and finite-
     impedance supplies sag").
   When not powered neither element is stamped: ECHO is hi-Z (RSHUNT only).

**Commit** (`EA:863-961`), in order:
1. `vccVNew = V(vcc) - V(gnd)`; `vccOkNew = vccVNew >= 3.0`.
2. If `!vccOkNew`: reset `trigLevel = 0`, `phase = "idle"`, `echoRiseAt =
   echoFallAt = NaN`, `echoOut = 0` (`EA:873-877`); `pendingEchoEvent` is kept.
3. Else, TRIG edges: if `ctx.mcuEventDriverForNode(trig node)` finds a powered
   uno/nano/pico driver, the edges are that driver's non-null step events on its
   pin, at time `bootT + cycle/clockHz`; otherwise `vTrig = V(trig) - V(gnd)`,
   `curLevel = vTrig >= 2.0 ? 1 : 0`, and one edge `{curLevel, ctx.simTime()}`
   if `curLevel != trigLevel` (`EA:884-903`). Note: unlike the servo, the event
   path needs only a powered driver, not that the MCU currently drives the pin.
4. For each edge in order: if `phase == "idle" && trigLevel == 0 && edge.level
   == 1` then `phase = "armed"`; else if `phase == "armed" && trigLevel == 1 &&
   edge.level == 0` then schedule: `echoRiseAt = edge.time + 250e-6`, `width =
   noEcho ? 38e-3 : 58.0e-6*clamp(distanceCm, 2, 400)`, `echoFallAt = echoRiseAt
   + width`, `phase = "pending"`, and, when the ECHO net also carries a
   micro:bit edge pin (`hcsr04MicrobitBridge`, section 14.9), `pendingEchoEvent
   = {boardComponentId, pinId, delayUs: 250, widthUs: width*1e6}`
   (`EA:908-934`). Whatever the phase, `trigLevel = edge.level` after each
   edge (so retriggers while `pending`/`active` are ignored but tracked).
5. Fire: `now = ctx.simTime() + h`; if `phase == "pending" && now >=
   echoRiseAt` then `phase = "active"`, `echoOut = 1`, `echoRiseAt = NaN`; if
   `phase == "active" && now >= echoFallAt` then `phase = "idle"`, `echoOut =
   0`, `echoFallAt = NaN`, `trigLevel = 0` (`EA:944-955`). Both can fire in one
   step.
6. Store `{trigLevel, phase, echoRiseAt, echoFallAt, echoOut, vccV: vccVNew,
   pendingEchoEvent}`.

`updateCurrent` publishes `0.002` when `vccV >= 3.0`, else 0 (`EA:962-967`);
note it does not test the open-pin condition the stamp tests (Q below).

### 8.3 Dynamics and state

- All timing is event timing: the width of the echo is `58 us/cm * cm` exactly
  (2.9 ms at 50 cm; 23.2 ms at 400 cm; 116 us at 2 cm), and the delay from
  TRIG fall to ECHO rise is exactly 250 us, both measured in the TRIG edge's own
  time base (the MCU cycle on the event path; the step start on the net path).
- The sensor is a one-shot: ignored retriggers, no 60 ms measurement cycle,
  no minimum TRIG width (any rise followed by a fall arms and schedules; a real
  module wants at least 10 us), no maximum.
- `noEcho` gives a 38 ms pulse (the out-of-range timeout).
- The held state `echoOut` is the ECHO drive: 0 drives ECHO low through 50 ohm
  (not hi-Z) whenever powered.
- Held at its entry state in the OP ladder (`SE:781`). Carried across
  `load()` (`SE:3253-3259`): an in-flight echo survives a live `distanceCm`
  edit, and the new distance applies only to the next scheduled echo.

### 8.4 Newton

Linear: no `xGuess` read.

### 8.5 Regimes and events

- Phase machine `idle -> armed -> pending -> active -> idle`; `vcc < 3.0`
  forces `idle`.
- **The only device in W4 whose event instants classic locates.** The engine
  exposes the scheduled times (`nextScheduledEventTime()`, `SE:5262-5290`: the
  earliest `echoRiseAt` or `echoFallAt` greater than `simTime`, also the
  digital due-times), and the host clamps the next step to land on it
  (`HL:1107-1117`: `h = max(H_MIN, untilEvent)` when the event falls inside the
  step). Because the commit fires on `simTime + h >= time`, a step that ends
  exactly on the instant commits it; its `echoOut` then applies to the NEXT
  solve, so ECHO changes at the event instant to within `H_MIN = 10 ns` plus
  floating-point slack.
- **The birth step.** A schedule created in a step has no advance warning, so
  the host also caps every step at `HCSR04_MAX_STEP_H = 100 us` whenever an
  hcsr04 exists (`SE:5314-5317`, `HL:1136-1137`); otherwise a step longer than
  250 us could jump past `echoRiseAt` in its own birth step. This is part of
  the contract: the shell requires `h <= 100 us` for any circuit containing the
  part.
- Because it is the cap that protects the instant, a port with event-exact
  edges needs neither the cap nor `nextScheduledEventTime`; it needs the TRIG
  fall time (a logic-output event) and a scheduled ECHO edge (a heap event)
  with the delay and width constants above.

### 8.6 Small-signal AC

`acStamp` (`EA:968-989`): the TRIG pull-down `1/1e6` unconditionally; and, when
`physicallyPowered` by the same committed rule, `1/GATE_R_OUT` from ECHO to the
committed rail (VCC if `echoOut`, else GND). The 2 mA current source
differentiates away.

### 8.7 Diagnostics

Catalog `vcc_range` makes the failure pass call `_icPowerInfo` (supply pins
`vcc`/`gnd`, powered window [2.7, 6.05]) but `_drivenOutputTargets` has no
`hcsr04` case, so no output-sag is ever raised for ECHO. Telemetry
(`physics-telemetry.ts:292-305`): `phase`, `triggerHigh`, `echoHigh`,
`supplyVoltageV`, `echoRiseSimTimeS`, `echoFallSimTimeS`, `quality:
"behavioral-estimate"`, reference `supply`.

### 8.8 Tests that pin it

`hcsr04.test.ts`: net-threshold path (`:158-300`): arms on TRIG rise and
schedules on the fall at EXACTLY `tFall + 250 us` and `58 us/cm` (`:159`),
`noEcho` 38 ms (`:200`), retrigger ignored (`:214`), `VCC < 3.0` inert
(`:236`), 2 mA only while powered (`:249`), in-flight echo survives a
`distanceCm` reload (`:259`); event path with a real Uno hex (`:303-407`);
Pico end to end with MicroPython `machine.time_pulse_us`, 10 cm and 100 cm
within +-1 cm (`:408-457`); micro:bit echo-bridge event (`:458-498`).
`mcu-servo-replay.test.ts:175-338` (HC-SR04 events counted once under the 555
split). `ac-isolation` census.

### 8.9 Questions

- **Q-W4-{hcstamp} (stamp and commit disagree on the open-pin rule).** The
  stamp requires `!isOpenPin(vcc) && !isOpenPin(gnd)`; the commit and
  `updateCurrent` use only `V(vcc) - V(gnd) >= 3.0`. With `gnd` unwired the
  node floats near 0 V, `vccV` reads the full rail, the state machine runs and
  the module reports 2 mA, but nothing is drawn and ECHO is not driven.
  **Recommend** reproduce.
- **Q-W4-{hctrig} (no minimum TRIG width, retrigger, 60 ms cycle).**
  Datasheet (HC-SR04, Cytron/Elecfreaks, recalled): trigger pulse at least 10 us
  TTL, 8 x 40 kHz burst, ECHO width proportional to range, measurement cycle
  over 60 ms, range 2 to 400 cm, ECHO 150 us to 25 ms and 38 ms when nothing is
  detected. Classic accepts any pulse, has no cycle limit, and gives `58.0
  us/cm` (the speed-of-sound constant used by common libraries); the 250 us delay
  is classic's own (the real module is closer to 200 us plus the burst).
  **Recommend** reproduce.
- **Q-W4-{hcdraw} (echo is push-pull 50 ohm; 2 mA constant current).** The real
  module's ECHO is a 5 V TTL push-pull with a stronger stage; the quiescent
  draw is 15 mA typical (recalled), classic uses 2 mA "typical operating
  current". **Recommend** reproduce (catalog note: "deliberately stamped").
- **Q-W4-{hcstep} (the 100 us step cap).** Section 8.5. With event-exact edges
  the cap goes away, but the MCU step cap still applies to a Uno/Nano/Pico
  (`MCU_H_MAX`), so the observable ECHO resolution seen by firmware is bounded by
  the MCU's `h` either way (up to 100 us, 1.7 cm, between ECHO edge and the
  next sampling). **Recommend** that W4-A/W4-S state in the shell's contract that
  the edge reaches the emulator at the instant it occurs (event) rather than at
  the next step end (classic `setInputBit`).

---

## 9. Darlington sink arrays: uln2003, uln2803

[W4-D.] `ulnModel` (`EA:992-1144`): one model for both kinds, the channel count
chosen by `comp.kind` (`uln2003` 7, `uln2803` 8; `EA:1018`, `1071`, `1102`,
`1130`). Seven or eight open-collector Darlington sinks with a catch diode to
COM on each output. Catalogs `uln2003a` (`CAT:7326`) and `uln2803a`
(`CAT:7470`): `default_params {}`; `electrical_specs {logic_family:
"TTL-compatible", io_max: 0.5, v_input: {v_il_max: 1.5, v_ih_min: 2.5}}`; no
`vcc_range`.

### 9.1 Terminals and parameters

Pins by id: `in1..inN` (`function: input`), `gnd`, `com` (`passive`), `out1..outN`
(`open_collector`) (`EA:1022-1031`). No supply pin. No parameters are read.

Constants: `ULN_RON = 5` ohm (`EA:1016`, again `1101` and `1129`),
`ULN_TTL_VIH = 2.5` (`EA:1017`, declared and NEVER used), `ULN_VIH_COMMIT = 2.0`
V and `ULN_VIL_COMMIT = 1.5` V (`EA:1069-1070`), the clamp-diode constants of
section 1.4.

### 9.2 Equations

State in `icState`: `in1..inN`, each 1 (sinking), 0 (released) or NaN
(uninitialised) (`SE:1631-1638`, default NaN for every channel).

**Stamp** (`EA:995-1051`), for each channel `ch`:
- `outNode = pinNode(out_ch)`; skip the channel if `outNode < 0` (the output
  net is the reference);
- `stampClampDiode(out, com, xGuess, vt)` ALWAYS (anode OUT, cathode COM; the
  comment: "conducts only when an inductive turn-off lifts the output above the
  connected supply rail"), even with COM unwired (then the cathode node is held
  by RSHUNT only and the diode loads OUT with about 1e-12 S until OUT rises about
  a diode drop above the floating COM node);
- `sinking = !gndOpen && committedIn >= 0.5` (a NaN committed value gives
  false, `EA:1038-1039`); if sinking, `stampResistor(out, gnd, 5)`.
A released channel stamps nothing else: OUT is open-collector, hi-Z. The input
pins stamp nothing (the model has no input load).

**Commit** (`EA:1068-1093`): `gndV = V(gnd node)` (raw node voltage, `-1`
gives 0); for each channel `vIn = V(in_ch) - gndV`; `new = 1` if `vIn >=
2.0`; `0` if `vIn < 1.5`; else the previous value (`prev` NaN becomes 0)
(`EA:1082-1090`); the whole `icState` record is replaced. The
`ULN_TTL_VIH = 2.5` and the catalog's `v_ih_min = 2.5` are NOT used.

**Current published** (`EA:1094-1120`): the SUM over channels of `(V(out_ch) -
V(gnd))/5` for channels that are committed sinking and `gnd` wired, signed
(a channel whose OUT is below GND reports negative).

### 9.3 Dynamics and state

Memoryless apart from the committed input latch with a 0.5 V hysteresis band.
No delay, no base current, no storage time. Carried by the generic `icState`
carry.

### 9.4 Newton

The only nonlinearity is the per-channel clamp diode (section 1.4), a guarded
exponential with `vSat = 1.0277 V` and no pnjlim; with up to 8 diodes, each
stamped from `xGuess`. The sink resistor is linear.

### 9.5 Regimes and events

- **Sinking / released** per channel, decided from the accepted step's input,
  applied one accepted step later; brackets by halving (the `inN` keys are
  compared exactly, section 1.6).
- **The clamp** is a smooth device; no regime.
- Open `gnd` forces every channel released regardless of input.
- Open `in` pin: `V(in)` is held at about 0 V by RSHUNT, so reads as low (no
  floating-input model is used: the ULN never calls `_stampFloatingDigitalInputs`
  or `ctx.logicHigh`).

### 9.6 Small-signal AC

`acStamp` (`EA:1121-1143`): per channel `acStampClampDiode(out, com)` always, plus
`stampAcAdmittance(out, gnd, 1/5)` if sinking. "The inputs are sampled logic;
the sink decision is a committed latch, so no small-signal path exists from IN
to OUT."

### 9.7 Diagnostics

- The `_drivenOutputTargets` ULN case (`SE:7956-7970`) reports sinking
  channels as `{pinId: out_ch, high: false}`, but the output-sag scan is guarded by
  `if (specs?.vcc_range)` (`SE:7576`) and neither ULN catalog entry has a
  `vcc_range`, so the case is never reached and a ULN never raises `output_sag`
  (the sag limit would have been `vil = 1.5 V` on the sunk output, a current of
  300 mA through 5 ohm). `uln2003` also sits in `PASSIVE_FAILURE_UPDATE_KINDS`
  (`SE:1417-1420`) with no failure hook (`uln2803` does not).
- No overcurrent or thermal failure; no `missing_flyback` (the ULN is not a
  switch kind, section 1.10).

### 9.8 Tests that pin it

`wave6-drivers-audio.test.ts`: single channel sinking (`:165-264`): with 5 V
input and a pull-up the output sits at `V_out = 0.4545 V` (`:166`, the divider
of the load and 5 ohm), released with a low input the output goes to the
supply (`:207`), unwired GND never sinks (`:237`); channel independence
(`:267`); ULN2803 8-channel (`:313`); the COM catch-diode transient
(`it.each` at `:356`, `:355-396`).

### 9.9 Questions

- **Q-W4-{ulnron} (a 5 ohm resistor against a Darlington).** Classic gives
  `Vce = 0.5 V` at 100 mA, 1.0 V at 200 mA, 1.75 V at 350 mA, 2.5 V at 500 mA.
  The ULN2003A datasheet (TI, recalled): `VCE(sat)` about 0.9 V at 100 mA, 1.1
  V at 200 mA, 1.3 V at 350 mA (typical, IB 250 to 500 uA), absolute maximum
  collector current 500 mA per channel. Classic is below the datasheet at low
  current and above it at 350 mA and over, and has no current limit.
  **Recommend** reproduce (the file comment says "a deliberate
  simplification ... a future wave can replace with a Thevenin voltage-source").
- **Q-W4-{ulnthresh} (committed thresholds are not the catalog's).** The
  catalog says `v_il_max 1.5`, `v_ih_min 2.5`; the commit uses `>= 2.0` for high
  and `< 1.5` for low. ULN2003A datasheet: `VI(on)` 2.4 to 3 V at 200 to 300 mA
  (recalled). **Recommend** reproduce; delete the unused `ULN_TTL_VIH`.
- **Q-W4-{ulnclamp} (the clamp diode is the generic silicon fit, not the ULN
  catch diode).** 0.7 V at 100 mA with a 25.7 mV/decade slope gives 0.73 V at 350
  mA; the ULN2003A datasheet gives `VF` about 1.75 V at 350 mA (recalled) and
  the diode has series resistance. A real kick therefore clamps about 1 V
  higher. The same helper serves the L293D and TB6612. **Recommend** reproduce.
- **Q-W4-{ulninput} (no input load).** The ULN draws nothing from its input
  (real: about 0.9 mA at 3.85 V input, i.e. a 2.7 kohm series base resistor;
  recalled); an MCU pin driving 8 channels sees no load. **Recommend** reproduce.
- **Q-W4-{ulnlag} (one-step lag on the sink).** The released output of a
  channel follows its input one accepted step late (section 9.5). At `h` up to
  100 us (MCU circuits) or 10 ms (quiet circuits) this is visible in a fast
  PWM. **Recommend** the shell commits driver states as events at the input
  edge instant (DK 1.5).

---

## 10. Dual H-bridge: l293d

[W4-D.] `l293dModel` (`EA:1435-1597`), the committed-drive-state design
described in the header comment `EA:1404-1433`. Four push-pull outputs, two
enable groups, per-output freewheel diodes. Catalog `l293d` (`CAT:7716`):
`default_params {}`, `electrical_specs {vcc_range: {4.5, 5, 7}, io_max: 0.6}`.

### 10.1 Terminals and parameters

Pins by id: `en12`, `en34` (`enable`); `in1..in4` (`input`); `out1..out4`
(`output`); `gnd1..gnd4` (`gnd`; the four are one node, `GR:134-141`
`unionInternalPins`, "pins 4, 5, 12 and 13 are one exposed ground/heat-sink
structure"); `vcc1` (`vcc`, the logic supply, 4.5 to 7 V); `vcc2` (`passive`,
the motor supply "VM", 4.5 to 36 V per the catalog note). Supply roles:
`supplyPinsForComponent` returns `vcc1` / `gnd1` (W1 section 1.3). No
parameters are read.

Constants: `H_BRIDGE_RON = 2` ohm per output side, repeated at `EA:1456`,
`1546`, `1579`; `L293D_VIH = 2.0` V and `L293D_VIL = 1.5` V (`EA:1506-1507`) of
which `L293D_VIL` is NEVER read; the VM-valid threshold `1.5` V (`EA:1502`);
the drive codes `HIGH 1`, `LOW 0`, `HIZ -1` (`EA:1528-1530`); the clamp-diode
helper of section 1.4.

### 10.2 Equations

State in `icState`: `{out1, out2, out3, out4, vm}`, defaults `{-1, -1, -1, -1,
0}` (`SE:1657-1658`): every output starts HIZ.

**Stamp** (`EA:1438-1492`):
1. For each `outN`: `stampClampDiode(gnd1 -> outN)` and
   `stampClampDiode(outN -> vcc2)` (`EA:1465-1469`), ALWAYS, in every drive state
   including HIZ and with `vcc2` unwired (then its node is held by RSHUNT and
   the upper diode lifts that node as `outN` rises). Anode and cathode in
   that order: GND to OUT and OUT to VM.
2. For each output with committed state: `1` (HIGH): `stampResistor(outN, vcc2,
   2)` only if `vcc2` is not an open pin (`EA:1476-1480`); `0` (LOW):
   `stampResistor(outN, gnd1, 2)` (`EA:1481-1484`); `-1`: nothing.
Outputs whose node is the reference (`outNode < 0`) are skipped for the
resistor (`EA:1475`) but the diodes are still stamped with a `-1` index
(harmless).

**Commit** (`EA:1493-1538`):
- `power = ctx.icPowerInfo(comp, x)` (logic supply `vcc1`/`gnd1`);
- `vmOpen = isOpenPin(vcc2)`; `vm = vmOpen ? 0 : V(vcc2) - V(gnd1)`; `vmValid =
  power.powered && !vmOpen && vm >= 1.5` (`EA:1498-1502`);
- an input or enable reads high iff it is NOT an open pin and `V(pin) -
  V(gnd1) >= 2.0` (`EA:1512-1516`): a FIXED 2.0 V against `gnd1`, not the
  `thresholds.vih` of the supply, no hysteresis; an open pin is low;
- `outN = (!vmValid || !enable) ? HIZ : (inN ? HIGH : LOW)` with `en12` for
  `out1`, `out2` and `en34` for `out3`, `out4` (`EA:1532-1535`);
- `icState = {out1..out4, vm}`.

**Current published** (`EA:1539-1571`): the SUM over the four outputs of
`(V(vcc2) - V(outN))/2` for HIGH and `(V(outN) - V(gnd1))/2` for LOW. The motor
current of an H-bridge pair is counted at BOTH of its outputs (one source side
and one sink side), so the published element current is about twice the
current through a motor driven between two outputs, and it ignores the clamp
diodes. `HIGH` is summed without checking that `vcc2` is wired (the stamp
checks).

### 10.3 Dynamics and state

Memoryless apart from the committed drive states. No slew, no dead time, no
shoot-through (an input change swaps the committed pair in one step), no
thermal shutdown, no current limit. The clamp pair gives both freewheeling and
supply-pumping paths ("coast and supply pumping are solved electrically instead
of only diagnosed", `EA:1462-1464`).

### 10.4 Newton

Eight guarded Shockley diodes per part (two per output), each stamped from
`xGuess` with the `vSat` guard of section 1.4, no pnjlim; the output resistors
are linear. An inductive load that is switched off drives the diode pair across
a large Newton move; the guard keeps the exponent below `exp(40) ~ 2.4e17`
(`vSat/vtN = 40`), so no overflow but a very stiff diode.

### 10.5 Regimes and events

Per output HIGH / LOW / HIZ, decided from the ACCEPTED step's inputs and
applied one accepted step later. `vmValid` needs VM at least 1.5 V above `gnd1`
AND the logic supply powered. No located instant; the `out*` keys compare
exactly in the step controller (a flip between coarse and refined solves
rejects the step).

### 10.6 Small-signal AC

`acStamp` (`EA:1572-1596`): per output both clamp diodes' `acStampClampDiode`
tangent conductances at the held OP (anode/cathode orders as in the stamp),
plus `stampAcAdmittance(out, vcc2, 1/2)` for HIGH (when VM is wired) or `(out,
gnd1, 1/2)` for LOW. Not derived from the transient stamp; it reads the same
committed state.

### 10.7 Diagnostics

- **`output_sag`** (`SE:7973-7988` targets; scan `SE:7576-7660`): every
  non-HIZ output is a target; HIGH targets sag when `V(out) - V(gnd1) <
  thresholds.vih`, LOW targets when `> thresholds.vil`, for the LOGIC supply's
  `thresholds` (generic: at `vcc1 = 5 V`, `vil = 1.5`, `vih = 3.0`). A LOW
  output sags above 1.5 V, i.e. above 750 mA through 2 ohm; a HIGH output
  sags below 3 V whatever VM is, e.g. above 1 A with VM = 5 V, above 4.5 A with
  VM = 12 V. Two sagging steps are required (`OUTPUT_SAG_DEBOUNCE_STEPS = 2`).
  The comparison uses the logic family's thresholds on a motor-rail output,
  because that is all `_drivenOutputTargets` knows.
- No `missing_flyback` for loads on L293D outputs (the L293D is not a switch
  kind).

### 10.8 Tests that pin it

`wave6-hbridge.test.ts`: forward drive `V_out1 = 12*100/102 = 11.7647 V` with
100 ohm to ground and VM 12 V (`:268-325`); reverse `0.2353 V` (`:326`);
disabled gives HIZ (`:371`); motor direction `+-11.5294 V` (`:416-514`); VM
unconnected gives HIZ (`:515`); integrated freewheel transients (`:829-870`:
"clamps both sides of a winding after EN12 is released"); flyback suppression
through the host rule engine (`:916-1033`, section 5.7). `wave7-dcmotor.test.ts:
399-478` (a motor on an L293D).

### 10.9 Questions

- **Q-W4-{l293ron} (two ohm per side against the datasheet).** A motor
  between two outputs sees `2 + 2 = 4` ohm of bridge (4 V at 1 A, 2 V at 500 mA;
  the file comment justifies "0.7 V per transistor ... 1.4 ohm ... use 2 as the
  teaching value", and the catalog note says "1.4 V total drop at 1 A across
  both output transistors"). The L293D datasheet (TI, recalled): `VCE(sat)`
  source 1.4 V typical (1.8 max) at 1 A, sink 1.2 V typical (1.8 max) at 1 A, so
  about 2.6 V typical across the pair at 1 A and 1.2 V per side at 600 mA.
  Classic gives 4.0 V at 1 A and its catalog note contradicts its own constant.
  **Recommend** reproduce 2 ohm for the gate and fix the catalog sentence;
  record the divergence.
- **Q-W4-{l293dthr} (fixed 2.0 V input threshold).** `L293D_VIL` is dead; inputs use
  2.0 V from `gnd1` however the logic supply is set. The L293D datasheet (recalled):
  `VIH` 2.3 V min (logic supply 4.5 to 7 V), `VIL` 1.5 V max. A 2.0 V threshold
  accepts inputs the part would not guarantee. **Recommend** reproduce.
- **Q-W4-{l293sag} (output-sag thresholds from the logic supply).** Section 10.7:
  the sag test judges a VM-driven output by the logic supply's input
  thresholds. **Recommend** reproduce for the gate; a W4-D rework should pick
  limits from the output's own rail (a separate question for the host).
- **Q-W4-{bridgeI} (published element current counts both sides).** Section
  10.2. **Recommend** reproduce, and name the readout in the core's telemetry
  (`package-total`).
- **Q-W4-{vmfly} (the VM-powered suppression is in the host's rules, not in
  the engine).** Section 1.10: the engine raises no warning for a winding on an
  L293D at all, floating VM or not; `wave6-hbridge.test.ts:916-1033` tests
  the host's `runDiagnostics` copy. **Recommend** the port keeps the engine
  rule only and does not carry the host rule.
- **Q-W4-{bridgedead} (no shoot-through guard or dead time).** The committed
  pair changes in a single step; a real bridge has a dead time and, on the L293D,
  a shoot-through current spike. **Recommend** reproduce.

---

## 11. Dual H-bridge: tb6612

[W4-D.] `tb6612Model` (`EA:1599-1769`). The same committed push-pull bridge as
the L293D with a PWM-and-standby truth table. Catalog `tb6612` (`CAT:7860`):
`default_params {}`, `electrical_specs {vcc_range: {2.7, 3.3, 5.5}, io_max:
1.2}`.

### 11.1 Terminals and parameters

Pins by id: `ain1`, `ain2`, `bin1`, `bin2` (`input`); `pwma`, `pwmb`
(`input`); `stby` (`enable`); `ao1`, `ao2`, `bo1`, `bo2` (`output`); `vm`
(`passive`, the motor supply "2.5 to 13.5 V"); `vcc` (`vcc`, logic 2.7 to
5.5 V); `gnd`, `gnd2`, `gnd3` (`gnd`; one node, `GR:142-147`). Supply roles:
`vcc` / `gnd`. No parameters read. Constants: `TB_RON = 2` ohm (`EA:1612`,
`1721`, `1751`), the `vm` valid threshold `1.5` V (`EA:1652`), drive codes
`HIZ -1`, `HIGH 1`, `LOW 0` (`EA:1673-1675`). Default state `{ao1, ao2, bo1,
bo2: -1, vm: 0}` (`SE:1660-1661`).

### 11.2 Equations

**Stamp** (`EA:1602-1644`): identical in form to the L293D with `vm` and `gnd`:
for each of `ao1, ao2, bo1, bo2` the clamp pair `stampClampDiode(gnd -> out)`
and `stampClampDiode(out -> vm)` always, then `stampResistor(out, vm, 2)` for
HIGH (only if `vm` is wired) or `stampResistor(out, gnd, 2)` for LOW.

**Commit** (`EA:1645-1718`):
- `power = ctx.icPowerInfo(comp, x)`; `vmVolt = vmOpen ? 0 : V(vm) - V(gnd)`;
  `vmValid = power.powered && !vmOpen && vmVolt >= 1.5`;
- an input reads high iff not open and `V(pin) - V(gnd) >= power.thresholds.vih`
  (the supply-ratioed threshold: `max(2.0, 0.6*vSupply)` generic, so 2.0 V at the
  3.3 V logic supply, 3.0 V at 5 V; there is no hysteresis: `vil` is unused);
- channel A: if `!vmValid || !stby` both HIZ; else if `!pwma` both HIZ (coast,
  "PWM LOW -> coast (hi-Z). In DC model PWM is a binary enable"); else by
  `(ain1, ain2)`: `(1,0)` `ao1` HIGH `ao2` LOW; `(0,1)` `ao1` LOW `ao2` HIGH;
  `(1,1)` both LOW (brake); `(0,0)` both HIZ (coast) (`EA:1680-1696`);
- channel B: the same with `bin1`, `bin2`, `pwmb` (`EA:1698-1715`);
- `icState = {ao1, ao2, bo1, bo2, vm}`.

`updateCurrent` (`EA:1719-1746`) and `acStamp` (`EA:1747-1768`) are the L293D's
with the TB names.

### 11.3 to 11.6 Dynamics, Newton, regimes, AC

As section 10: no memory beyond the committed states; eight guarded diodes;
regimes HIGH/LOW/HIZ per output decided from the accepted step and applied one
step later; no located instant. **PWM is a binary enable evaluated at step
ends.** A PWM input at 20 kHz (period 50 us) sampled at step ends of 100 us or
10 ms is a sampled level: the part is in drive or in coast according to the
instant of the sample, not the duty cycle (the catalog note: "PWM duty-cycle
averaging for speed control is a Wave 7 motor-dynamics feature -- in this model
PWM is treated as a binary enable"). Brake `(1,1)` is decided with PWM high
only.

### 11.7 Diagnostics

`output_sag` as for the L293D (`SE:7989-8010`), with the logic supply's generic
thresholds at `vcc = 3.3 V`: `vil = max(0.8, 0.99) = 0.99`, `vih = max(2.0,
1.98) = 2.0`; a LOW output sags above 0.99 V, i.e. above 495 mA through 2 ohm.
No `missing_flyback` (not a switch kind).

### 11.8 Tests that pin it

`wave6-hbridge.test.ts`: forward `+11.5294 V` (`:560`), reverse `-11.5294 V`
(`:611`), brake `0 V` (`:661`), coast `0 V` (`:717`), `STBY` low gives HIZ
(`:772`), body-diode clamps during PWM coast (`:871`).

### 11.9 Questions

- **Q-W4-{tbbrake} (PWM low is coast; the datasheet brakes).** TB6612FNG
  function table (Toshiba, recalled): `PWM` low with `IN1/IN2 = H/L` or `L/H`
  gives SHORT BRAKE (both outputs low), `IN1/IN2 = H/H` gives short brake for
  any `PWM`, and `L/L` with `PWM` high gives stop (off). Classic coasts for `PWM`
  low. A chopped motor drive therefore recirculates through the body diodes
  (coast) instead of the low-side FETs (brake). **Recommend** reproduce for the
  gate and record the divergence; the correct table is a four-line change.
- **Q-W4-{tbron} (2 ohm against about 0.5 ohm).** TB6612FNG datasheet
  (recalled): `RON(H+L)` about 0.5 ohm typical, 0.7 max, at `VM` 5 V, 1 A. Classic
  is 4 ohm across a pair, eight times the datasheet. The catalog note
  admits "R_on = 2 ohm per output side (simplified)". **Recommend** reproduce.
- **Q-W4-{tbstby} (STBY and open inputs).** An open `stby`, `pwm`, `in` reads
  low (`isOpenPin`), consistent with the real part's internal pull-downs on
  `STBY` and the inputs (recalled); an open `vm` gives HIZ. **Recommend**
  reproduce.
- **Q-W4-{tbpwm} (PWM aliasing).** Section 11.3. A 20 kHz or 490 Hz PWM into
  a `PWM` pin gives a duty-dependent but step-quantised enable. The core's
  event edges (PLAN W4-D) would make drive/coast exact. **Recommend** the
  shell evaluates the enable at the PWM edges, not at step ends (DK 1.5), and the
  gate compares average motor current to the analytic duty-cycle value, not to
  classic (PLAN decision 7 (b)).

---

## 12. Character LCD controller: hd44780

[W4-P.] `hd44780Model` (`DI:1889-2092`). An HD44780 16x2 text LCD as a decoder
of E-falling-edge-latched bytes plus a behavioural supply load. All of the
analog behaviour is one resistor. Catalog `hd44780-lcd` (`CAT:8195`):
`default_params {}`, `electrical_specs {vcc_range {4.5, 5, 5.5}, logic_family
"CMOS", v_input {v_il_max 0.6, v_ih_min 2.2}, icc_max 0.003}`.

### 12.1 Terminals and parameters

Pins by id: `vss` (`gnd`), `vdd` (`vcc`), `v0` (`passive`: contrast, never
stamped), `rs`, `rw`, `e`, `d0..d7` (`input`), `a`, `k` (`passive`: backlight
anode and cathode, never stamped) (`CAT:8195`; `DI:1897-1900`, `1925`). No
parameters are read.

### 12.2 Equations

**Stamp** (`DI:1891-1903`): `power = ctx.icPowerInfo(comp, xGuess)`; if
`power.powered` and `vdd` is not an open pin and `vss` is not an open pin
(and `vddNode >= 0`), `stampResistor(vdd, vss, 200)`. Nothing else: the bus
pins are "high-impedance logic inputs ... Driven inputs are not burdened by
invented pull-downs" and no `_stampFloatingDigitalInputs` is called
(W1 section 1.6). The 200 ohm load is about 25 mA at 5 V "logic + backlight
draw" and does not depend on state, contrast or backlight. Powered window
[4.05, 6.05] V (section 1.9).

**State** `Hd44780State` (`SE:455-469`, default `SE:472-488`): `ddram`
(104 entries filled with `0x20`), `addr` 0, `fourBit` false, `nibblePhase` 0,
`hiNib` 0, `displayOn` false, `increment` true, `cgram` false, `twoLine`
false, `lastE`, `lastRS`, `lastData[8]` all 0. (The comment at `SE:459` says
the address counter "wraps within 0x00-0x27 / 0x40-0x67"; no code wraps it.)

**Decode** (`updateDigital`, `DI:1907-2069`), once per accepted step in the
digital pass:
1. If `!power.powered`: publish `{kind "hd44780", lines [16 spaces, 16 spaces],
   on false, cols 16, rows 2}`, keep the state, return (`DI:1915-1920`). The
   state is NOT reset by a power loss.
2. Events: `mergedDisplayEvents(comp, ["e","rs","d0".."d7"])` (section 1.5, item
   4; Uno and Nano drivers only).
3. **Event path** (`events.length > 0`, `DI:2013-2028`): for each `{displayPin,
   level}` in order: `lvl = level === null ? 0 : level` (a Hi-Z event is a
   LOW); `e`: if `lcdE == 1 && lvl == 0` call `latchOnEFalling()`, then `lcdE =
   lvl`; `rs`: `lcdRs = lvl`; `dN`: `lcdD[N] = lvl`.
4. **Sampled path** (`DI:2029-2041`): `cE = logicHigh(e)`, `lcdRs =
   logicHigh(rs)`, `lcdD[i] = logicHigh(di)` (the `ctx.logicHigh` of W1 section
   1.4, with the floating-input model for open pins); if `st.lastE == 1 && cE ==
   0`, `latchOnEFalling()`; `lcdE = cE`. At most ONE E falling edge per step. With 4-bit wiring `d0..d3` are open pins
   read through the floating-input model (W1 section 1.5), i.e. pseudo-random per
   10 ms bucket, which matters only before the part is in 4-bit mode.
5. `latchOnEFalling` (`DI:1993-2011`): in 8-bit mode (`!fourBit`) the byte is
   `d7..d0` and `processByte(byte, rs)` runs at once; in 4-bit mode phase 0
   captures `hiNib = d7 d6 d5 d4` and sets `nibblePhase = 1`; phase 1 forms
   `(hiNib << 4) | lowNib` from `d7..d4` and runs `processByte`, then phase 0.
6. `processByte` (`DI:1945-1985`): `rs = 1` (data): if `!cgram && addr <
   ddram.length` then `ddram[addr] = byte; addr += increment ? 1 : -1`;
   writes in CGRAM mode are discarded. `rs = 0` (command) in this priority:
   `0x01` clear (fill `0x20`, `addr = 0`, `cgram = false`); `(b & 0xfe) ==
   0x02` home (`addr = 0`, `cgram = false`); `(b & 0xfc) == 0x04` entry mode
   (`increment = bit 1`); `(b & 0xf8) == 0x08` display control (`displayOn = bit
   2`); `(b & 0xf0) == 0x10` cursor/display shift (accepted, ignored); `(b &
   0xe0) == 0x20` function set (`fourBit = (b & 0x10) == 0`, `twoLine = (b &
   0x08) != 0`); `(b & 0xc0) == 0x40` set CGRAM address (`cgram = true`); `(b &
   0x80) == 0x80` set DDRAM address (`addr = b & 0x7f`, `cgram = false`).
7. Commit and publish (`DI:2043-2068`): the working copies go back to
   `state.lcds`; `lines[0] = ddram[0x00..0x0f]`, `lines[1] = ddram[0x40..0x4f]`
   with each code mapped by `lcdCharToDisplay`: `0x20..0x7e` is its ASCII
   character, any other code a space; `on = displayOn`.

The power-on state is 8-bit mode; a 4-bit sketch's first `0x30`-style nibbles
decode as 8-bit bytes because `d0..d3` are never driven (read as 0 on the event
path, whose `lcdD[0..3]` stay 0), so `0x3 0x3 0x3 0x2` are the function-set
`0x30` (8-bit) three times and then `0x20` (4-bit), exactly the datasheet's
initialisation sequence. There is no busy flag, no execution time (every command
is instant), no `RW` read, no cursor, no blink, no display shift, no CGRAM
glyphs, no katakana and no contrast (`V0` and the backlight pins are
never read).

`updateCurrent` (`DI:2072-2081`): `(V(vdd) - V(vss))/200` when powered and both
supply pins wired, else 0. `acStamp` (`DI:2082-2091`): `1/200` between `vdd` and
`vss` when `icPowerInfoAtOp` says powered and the pins are wired.

### 12.3 Dynamics and state

None analog. Digital state carries across `load()` by deep copy (`SE:3244-3252`).
The display is not reset when it loses power (`DI:1915-1920`): re-applying the
supply shows the old DDRAM (Q below). `lcds` is snapshotted (`SE:2623`).

### 12.4 Newton

Linear: the 200 ohm load only; the power gate reads `xGuess` on every iterate
(`SUPPLY_GATED_KINDS`, `SE:1383-1389`, and the chatter machinery, W1 section 8).

### 12.5 Regimes and events

- **Powered / not powered** (`[4.05, 6.05]` V) per Newton iterate; when
  unpowered the display publishes dark and the decoder does nothing.
- **Event path (exact) vs sampled path.** The event path replays every
  cycle-ordered edge of the Uno's or Nano's pins inside one step (a whole
  command sequence), consumed once (`_mcuEdgesPending`). The sampled path sees
  one end-of-step snapshot and at most one `E` fall: it needs `E` to be
  HIGH at one step end and LOW at the next.
- **A Pico is not an event driver for a display.** `_dispDrivers` is built with
  `includePico = false` (`SE:2482`, `7356-7360`: "the existing max7219/hd44780
  display-decode call sites ... were never audited against Pico sub-step
  timing"). A Pico driving an HD44780 (or a MAX7219) therefore always takes
  the sampled path; a genuine `E` pulse of a few microseconds is shorter than
  the capped MCU step (100 us) and is never seen by the end-of-step sample, so
  such a display cannot be driven (Q below). The micro:bit is not an MCU core in
  the engine (section 14.9) and takes the sampled path too.

### 12.6 Small-signal AC

Only the `200` ohm supply load (section above).

### 12.7 Diagnostics

`vcc_range` puts the part through the failure pass; `_drivenOutputTargets`
returns nothing for it. `rail_ic_power_chatter` applies (W1 section 8).

### 12.8 Tests that pin it

`wave8-hd44780.test.ts`: event path with an Uno 4-bit sketch,
`hd44780_hello.hex`, `lines[0] === "Hello, World!   "`, `lines[1]` blank, `on`
true (`:91`); the sampled fallback with discrete sources writes `A` and sets
`on` (`:162`); display off command gives `on` false (`:312`); open `vdd` gives
`on` false and blank lines (`:389`). `mcu-display-replay.test.ts`: the message
shows under the 555 split and without (`:288-317`); events counted once;
after an Arduino loses power its last powered step's clocks count once then
stop, even with another powered Arduino (`:318-355`).
`digital-input-impedance.test.ts:81-110`: the part has no invented 1 Mohm
pull-down on its inputs, and the catalog copy matches (`:103`).

### 12.9 Questions

- **Q-W4-{lcdpico} (a Pico cannot drive the display).** Section 12.5.
  **Recommend** the shell passes events to the display for all MCU kinds
  (event-exact, PLAN W4-P: "event-exact decoders ... no replay"), recorded as a
  divergence from classic. Classic's failure to decode a Pico-driven LCD is a
  gap, not a behaviour a user relies on. Also the same decision for MAX7219.
- **Q-W4-{lcdpower} (no reset on power loss).** The decoder keeps DDRAM, the
  address, the mode and the display flag across a supply loss. The datasheet
  (Hitachi HD44780U, recalled) resets to 8-bit, one-line, display off,
  increment, after the internal reset circuit runs (needs a rise time and a
  40 ms wait); DDRAM contents at power-on are undefined; the display must be
  initialised again. **Recommend** reproduce (the host reloading the circuit
  does not reset it either).
- **Q-W4-{lcdtiming} (no busy flag or timing).** Every command is instantaneous,
  `RW` is unused, `E` pulse width and setup times are not checked, a command
  sent during the 1.52 ms clear or home (recalled) is not lost. Real sketches that
  forget delays still work in classic. **Recommend** reproduce; the catalog says
  "RW read-back is not supported in v1; tie RW to GND".
- **Q-W4-{lcdrender} (only 16x2 text, ASCII, no cursor, shift or CGRAM).**
  Characters outside `0x20..0x7e` render as a space; the HD44780's ROM maps
  `0x5c` to a yen sign and `0x7e`/`0x7f` to arrows (catalog note), custom
  characters `0x00..0x07` are blank. The renderer's `lines` ignores display
  shift (`0x18/0x1c`) and a one-line `N = 0` mode. **Recommend** reproduce.
- **Q-W4-{lcdaddr} (the address counter does not wrap or clamp low).** `addr`
  can grow past `0x67` (writes stop at `ddram.length`) and can go negative with a
  decrementing entry mode; `ddram[-1] = byte` would create a non-index property
  that the state copy drops. A real counter wraps within each line's range.
  **Recommend** reproduce for the gate and clamp in the port (record it).
- **Q-W4-{lcdload} (a constant 200 ohm load).** 25 mA at 5 V against the
  catalog's own `icc_max 0.003` (3 mA, logic only); a lit 16x2 backlight draws
  about 20 to 40 mA, and the logic supply 1.5 mA (recalled). The load does not
  change with the display state. **Recommend** reproduce.

---

## 13. LED matrix driver: max7219

[W4-P.] `max7219Model` (`DI:2095-2266`). A MAX7219 as a 16-bit serial register
decoder plus a behavioural supply load. Catalog `max7219-matrix` (`CAT:8134`):
`default_params {}`, `electrical_specs {vcc_range {4, 5, 5.5}, logic_family
"CMOS", v_input {v_il_max 0.8, v_ih_min 3.5}, icc_max 0.33}`.

### 13.1 Terminals and parameters

Pins by id: `din`, `clk`, `cs` (`input`), `vcc` (`vcc`), `gnd` (`gnd`)
(`CAT:8134`; `DI:2103-2104`). No `dout`, no `iset` pin, no segment or digit
pins: the part publishes pixel rows only, never an LED current. No parameters
are read.

### 13.2 Equations

**Stamp** (`DI:2097-2109`): if `power.powered` and `vcc`, `gnd` are wired,
`stampResistor(vcc, gnd, 150)`: a fixed 33 mA at 5 V "representing a
representative lit-matrix draw" (catalog). Powered window `[3.6, 6.05]` V
(section 1.9). The inputs are high-impedance; there is no floating-input stamp.

**State** (`defaultIcState("max7219")`, `SE:1665-1672`, set `icState`): `shift`,
`bits`, `dig0..dig7`, `decodeMode`, `intensity`, `scanLimit`, `shutdown`,
`displayTest` (all 0), `lastCLK` 0, `lastDIN` 0, `lastCS` 1 (CS idles high).

**Decode** (`updateDigital`, `DI:2112-2243`), per accepted step:
1. If `!power.powered`: publish `{rows: [dig0..dig7], intensity, on: false}`
   with the REGISTERS kept (the state is not reset), and return. Note: the
   "off" display shows its last digit registers in `rows` with `on = false`.
2. `merged = mergedDisplayEvents(comp, ["din","clk","cs"])` (section 1.5 item 4).
3. **Event path** (`DI:2165-2190`): for each event: `din`: `din = lvl`; `clk`:
   on a rising edge (`clk == 0 && lvl == 1`) `shift = ((shift << 1) | din) &
   0xffff`, `bits++`; `clk = lvl`; `cs`: on `cs == 1 && lvl == 0` `shift = 0`,
   `bits = 0` (frame start); on `cs == 0 && lvl == 1 && bits >= 16`
   `applyRegister((shift >> 8) & 0xff, shift & 0xff)`; `cs = lvl`. A Hi-Z event
   is level 0. `CLK` edges while `CS` is high still shift (no gating on CS).
4. **Sampled path** (`DI:2191-2213`): read `din`, `clk`, `cs` with
   `ctx.logicHigh`; handle CS first (fall: reset; rise with `bits >= 16`:
   latch), then a CLK rising edge relative to `st.lastCLK`: one edge of each kind
   per step at most.
5. `applyRegister(addr, data)` (`DI:2146-2163`): `0x01..0x08` set `dig0..dig7`
   (stored as raw bytes), `0x09` `decodeMode = data`, `0x0a` `intensity = data &
   0x0f`, `0x0b` `scanLimit = data & 0x07`, `0x0c` `shutdown = data & 1`, `0x0f`
   `displayTest = data & 1`; `0x00` and every other address are ignored.
6. Publish (`DI:2227-2242`): `on = shutdown == 1 || displayTest == 1`; per row `r`:
   `displayTest ? 0xff : !on ? 0 : (r <= scanLimit ? dig[r] : 0)`;
   `{kind "max7219", rows, intensity, on}`. `decodeMode` is stored and NOT
   applied: only raw segment/column bytes are rendered ("BCD mode (0x09) is
   stored but not rendered", catalog).

Power-on registers: all 0 with `shutdown 0`: the display is off until `0x0c01`
is written (`wave8-max7219.test.ts:217`). `updateCurrent` (`DI:2246-2255`):
`(V(vcc) - V(gnd))/150` when powered; `acStamp` (`DI:2256-2265`): `1/150`.

### 13.3 to 13.6 Dynamics, Newton, regimes, AC

No analog dynamics; linear (the 150 ohm load and the power gate); regimes
`powered/unpowered`, `shutdown`, `displayTest`, `scanLimit`; AC the 150 ohm
conductance. The event-versus-sampled structure, the Pico and micro:bit
exclusions and the carry/snapshot rules are those of section 12.5; `icState`
(not a dedicated map) holds the registers so the generic carry and snapshot
apply, and `lastCS`, `lastCLK`, `lastDIN` and `bits` compare exactly in the step
controller.

### 13.7 Diagnostics

`vcc_range` puts it through the failure pass; no output targets;
`rail_ic_power_chatter` applies (W1 section 8). No `Iset` or overcurrent model.

### 13.8 Tests that pin it

`wave8-max7219.test.ts`: event path with `max7219_frame.hex` after one 20 ms
step (`:176`); sampled fallback `0x0c01` latches normal operation with one edge
per step (`:217`); default shutdown off (`:286-352`); unpowered (open `vcc`)
reports `on` false (`:353`). `mcu-display-replay.test.ts:185-267` (each clock
edge counted once with and without a 555; with a clocked IC on one MCU
`:268`). `digital-input-impedance.test.ts:81-110`.

### 13.9 Questions

- **Q-W4-{maxpico} (same Pico and micro:bit gap as the LCD).** Section 12.5.
  **Recommend** as `Q-W4-{lcdpico}`.
- **Q-W4-{maxcs} (CLK shifts regardless of CS; latch needs at least 16 bits).**
  The datasheet (Maxim MAX7219, recalled) shifts on CLK rising only while
  `LOAD/CS` is low; classic shifts always and resets on the CS fall. A clock
  running while CS is high corrupts nothing because the next CS fall resets
  `shift`, but a frame of more than 16 bits (a daisy chain) latches its last 16
  bits, which is correct for the last device only. **Recommend** reproduce.
- **Q-W4-{maxreset} (no power-on reset, rows shown with `on = false`).** The
  registers survive a supply loss; at power-up the real chip clears its control
  registers (decode, intensity, scan limit to 0, shutdown on, test off) and the
  digit registers are random (recalled; classic zeroes them). **Recommend**
  reproduce.
- **Q-W4-{maxdecode} (decode mode and intensity are stored, not applied).**
  Section 13.2. The BCD "Code B" font is not rendered; `intensity` only
  reaches the renderer. **Recommend** reproduce (catalog says so).
- **Q-W4-{maxload} (150 ohm constant, no `Iset`).** 33 mA at 5 V against
  the catalog `icc_max 0.33` A; the real draw depends on `Iset` and the lit LEDs
  (up to 330 mA, recalled). **Recommend** reproduce.
