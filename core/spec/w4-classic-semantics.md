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
  `m1`/`m2`; the stepper is NOT checked, although its catalog note says the
  flyback diagnostic fires for it (Q-W4-13, and the test at
  `wave7-servo-stepper.test.ts:728-756` pins the stepper behaviour; section 5.7).
- `missingFlybackWarning` (`SE:1117-1148`) is evaluated at load from the graph
  (`_evaluateStaticWarnings`, `SE:6639-6660`): it fires if a winding's two
  nets differ and one of them, not `gnd` and not holding a pin of a
  `FLYBACK_RAIL_KINDS` part (`voltage_source`, `battery_pack`, `bench_psu` and
  the three MCU boards, `SE:1078-1082`), holds a pin of a
  `FLYBACK_SWITCH_KINDS` part (bjt_npn/pnp, nmos/pmos, switch, push_button,
  spdt_switch, push_dpdt, dip_switch, `SE:1069-1072`) and no
  `FLYBACK_CLAMP_KINDS` part (diode, schottky, zener, tvs, led,
  `SE:1073-1077`). The ULN, L293D and TB6612 are in neither set, so a winding
  on their outputs never fires (this is the "suppressed by an H-bridge with VM
  powered" of the tests): the suppression is only that the driver is not a
  switch kind. It does NOT check that VM is wired (see the test at
  `wave6-hbridge.test.ts:962` "fires when L293D VM is floating", which pins
  something else: Q-W4-14).
- The output-sag pass (`SE:7553-7690`) runs for every component whose catalog
  entry has `vcc_range`; its debounce is two consecutive sagging steps, and its
  one-pole exemption `_outputSagPole` applies only to the drivers in
  `RC_OUTPUT_IC_KINDS` (`SE:1315-1320`), which contains no W4 kind.
