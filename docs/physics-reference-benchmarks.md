# Physics reference benchmarks

Brownout's reference suite is in
`test/sim/engine/physics-reference-benchmarks.test.ts`. These are analytical
golden tests: expected values come from closed-form circuit laws, not values
copied from a previous simulator run.

Passing this suite demonstrates accuracy only inside the envelopes below. It
does not make the engine a substitute for SPICE, datasheet qualification, or
hardware measurement.

| Benchmark | Validity envelope | Acceptance budget |
| --- | --- | --- |
| Unloaded resistor divider | Ideal independent DC voltage source and two nominal, linear, lumped resistors; no wiring/contact resistance or meter load | Absolute output error no more than 0.2 µV |
| Loaded resistor divider | Same model, with one explicit linear shunt load | Absolute output error no more than 0.2 µV |
| RC step at one time constant | Ideal R and C, zero-impedance voltage step, no ESR, leakage, ESL, voltage coefficient, or dielectric absorption | Full-scale error no more than 0.3% at `h=τ/100`, 1.2% at `h=τ/20`, and 2.5% at `h=τ/10` |
| RL step at one time constant | Ideal R and L, zero-impedance voltage step, no winding resistance beyond the explicit R, saturation, core loss, hysteresis, or parasitic capacitance | Full-scale error no more than 0.3% at `h=τ/100`, 1.2% at `h=τ/20`, and 2.5% at `h=τ/10` |
| Diode temperature direction | Steady forward bias near the declared current anchor; ambient equals junction temperature; Shockley junction plus −2 mV/°C forward-drop coefficient | The −40°C to 125°C drop reduction is 0.33 V ±30 mV |
| LED temperature direction | Same junction model with LED emission coefficient; excludes optical output, wavelength drift, package thermal impedance, and self-heating | The −40°C to 125°C drop reduction is 0.33 V ±30 mV |
| Guarded diode overdrive | Forced-voltage compact silicon junction beyond the exponential overflow guard; the model uses a finite-slope tangent extension and excludes package resistance, high injection, heating, and destructive failure | Reported diode current and the solved source-branch current agree within one part per billion, apart from the declared 1 TΩ node shunt |
| NPN DC point | Ebers–Moll forward-active region at 25°C; forced terminal voltages; no Early effect, high-injection beta roll-off, breakdown, charge storage, or self-heating | Collector-current relative error no more than 1% |
| NMOS DC points | Static level-1/Shichman–Hodges model; forced terminal voltages; reverse-biased body diode; no subthreshold current, gate charge, capacitance, mobility temperature dependence, avalanche, or self-heating | Triode-current relative error no more than 0.5%; cutoff current below 1 nA |
| Passive DC KCL and power | One ideal independent source plus linear resistors; source current uses positive-into-source sign convention | Midpoint KCL residual no more than 0.1 nA; total signed-power residual no more than 0.5 nW |
| RC discrete conservation | Ideal series RC, backward Euler, `h=τ/100` | KCL residual no more than 10 nA; energy residual no more than `1e-6` of step source work after including `½C(ΔV)²` numerical damping |
| RL discrete conservation | Ideal series RL, backward Euler, `h=τ/100` | KVL residual no more than 1 µV; energy residual no more than `1e-6` of step source work after including `½L(ΔI)²` numerical damping |
| Digital output rail power | One 74HC14 inverter sourcing a 1 kΩ load at 5 V; catalog-derived finite output resistance; package static current excluded | VCC debit matches load current within 10 nA; signed source/load/output-stage power residual no more than 50 nW |
| NE555 rail power | One timer held HIGH into 1 kΩ at 5 V; compact 20 Ω two-rail output with 0.1 V headroom and divider idle draw | Rail debit and public output current each close within 20 nA; signed source/load/output-stage power residual no more than 50 nW |
| MCU board and GPIO current | micro:bit at 3.3 V with the model's declared 15 mA idle load and one HIGH GPIO driving 1 kΩ; firmware/peripheral-dependent draw excluded | USB branch debit equals idle plus GPIO load current within 10 nA |
| Op-amp rail current | LM358 follower at 5 V sourcing 1 kΩ; model's 0.7 mA whole-package typical quiescent draw; input bias and frequency effects excluded | Positive-supply debit equals quiescent plus delivered load current within 20 nA |

The backward-Euler damping terms are numerical integration loss. They are not
physical dielectric, winding, or core losses and must not be presented as such.

The active-device checks are enabled only because their output stages debit
the physical VCC/GND rails. They intentionally test the source branch itself;
display-only current estimates would not satisfy these contracts.

## Related first-principles suites

The following focused suites extend the same contract without pretending that
one golden file proves the whole engine:

| Suite | What it locks | Important boundary |
| --- | --- | --- |
| `passive-parasitics.test.ts` | Capacitor ESR/leakage and inductor DCR/core-loss companion identities | No ESL, dielectric absorption, saturation, hysteresis, skin effect or mutual coupling |
| `battery-engine-physics.test.ts` | Chemistry OCV, depletion resistance, coulomb counting, cold behavior and rollback | No diffusion recovery, Peukert-rate capacity, aging, rechargeable chemistry or internal heating |
| `opamp-dynamics.test.ts` | Dominant-pole backward-Euler identity, measured gain roll-off, slew clamps, source/sink limits, rail power and rollback | One pole with hard limits; no vendor transistor macro-model, noise, offset distribution or capacitive-load instability |
| `semiconductor-parasitics.test.ts` | BJT Early-effect Jacobian/readout consistency plus MOS Cgs/Cgd transients, Miller coupling, symmetry and rollback | Constant lumped capacitances; no voltage-dependent charge curves, subthreshold detail, breakdown, charge storage or noise |
| `thermal-engine-physics.test.ts` | Exact package identity, analytical one-pole heating, rollback, LED junction feedback and regulator shutdown/restart hysteresis | Lumped package-to-ambient fit; no heatsink geometry, airflow, thermal coupling or multi-pole transient impedance |
| `servo-engine-physics.test.ts` | Target/actual separation, finite supply-scaled travel, brownout, moving load and rollback | No acceleration, torque/stall, backlash, hunting, compliance or heating |
| `digital-input-impedance.test.ts` | Servo, MAX7219 and HD44780 inputs remain high impedance through a 100 MΩ source | Input capacitance, leakage distribution and ESD-clamp current are not modeled |
| `trap-integration.test.ts` | Trapezoidal accuracy against analytic LC/RLC trajectories, and the backward-Euler anchor at every marked discontinuity | Trapezoidal covers capacitors and inductors; MOSFET gate caps, the op-amp pole and relay/motor/stepper coils stay backward Euler |
| `dc-operating-point.test.ts`, `dc-homotopy-pathological.test.ts` | The true operating-point solve and the gmin-stepping / source-stepping / pseudo-transient rescue ladder | A circuit that still fails the convergence contract is reported unresolved, never approximated |
| `ac-small-signal.test.ts`, `ac-cross-validation.test.ts` | Linearized small-signal AC at the operating point, checked against transient numeric derivatives | Valid only near the linearized operating point; no noise analysis in any form |

The oscilloscope acquisition, coupling, quantization, provenance, gap and
alias-warning contracts live in the `scope-channels` and `scope-trigger`
suites plus the host application's own instrument tests. Those verify the
declared instrument domain; they do not add physical probe loading or
circuit-derived broadband noise.

## Cross-validation against ngspice

`test/sim/engine/spice-ngspice-crossval.test.ts` runs shared SPICE decks
through both Brownout's netlist runner and a real ngspice binary in batch
mode, and holds the two result sets to declared envelopes. The circuit lines
are byte-identical on both sides; the ngspice deck adds only output plumbing.
The suite skips when no ngspice binary is present (point `NGSPICE_BIN` at one
outside the probed locations), so it is a local/opt-in credibility check
rather than a CI gate.

| Analysis | Envelope |
| --- | --- |
| `.op`, linear circuits | `\|dV\|` no more than max(5 mV, 1% of the ngspice value) |
| `.op`, junction circuits | `\|dV\|` no more than max(20 mV, 5% of the ngspice value) |
| `.tran` | RMS(brownout − interpolated ngspice) below 2% of the ngspice trace's swing for linear circuits, 5% for junction circuits |
| `.ac` | Per-point magnitude within 1 dB and phase within 5 degrees |

The junction envelopes are wider because junction solves amplify small
model-constant differences: physical-constant vintages put the two engines'
thermal voltage a few ppm apart and the exponential multiplies that. Decks
therefore pin `.temp 25` and explicit `.model` parameters, and the ngspice
side pins `TNOM=25`, so both engines evaluate the same authored `IS` at the
same junction temperature. See `docs/spice-subset.md` for why matching
ngspice's default 27 °C instead would make agreement worse.

An envelope is a disclosed validity contract, not a tuning target. Never
weaken one to make a failing case pass: a breach is either a netlist/runner
mapping bug (fix it) or a legitimate model difference (document it in the
case comment, with the mechanism).

Measured 2026-07-17 against ngspice-46 (Homebrew, macOS arm64), all seven
cases inside envelope:

| Case | Result |
| --- | --- |
| Resistive divider `.op` | worst `\|dV\|` 9.60e-9 V at v(2) (envelope 4.00e-2 V) |
| RC lowpass step `.tran` | v(out) RMS 1.01e-2 V = 0.212% of 4.750 V swing (envelope 2%) |
| Half-wave rectifier `.tran` | v(out) RMS 2.79e-3 V = 0.064% of 4.351 V swing (envelope 5%) |
| BJT common-emitter `.op`/`.ac` | `.op` worst `\|dV\|` 1.50e-6 V at v(c); `.ac` v(c) worst `\|dMag\|` 2.56e-6 dB, worst `\|dPhase\|` 8.07e-10 deg |
| Series RLC ringdown `.tran` | v(n1) RMS 9.37e-3 V = 0.101% of 9.268 V swing (envelope 2%, t=0 excluded) |
| Self-biased JFET `.op` | worst `\|dV\|` 2.86e-8 V at v(d) (envelope 6.40e-1 V) |
| K-coupled transformer `.tran` | v(p) RMS 6.78e-5 V = 0.001% of 7.081 V swing; v(s) RMS 3.72e-4 V = 0.003% of 13.879 V swing (envelope 2%) |

Worst case across the corpus: `.op` 1.50e-6 V, `.tran` RMS 0.212% of swing,
`.ac` magnitude 2.56e-6 dB, `.ac` phase 8.07e-10 deg. These are agreement
figures for the documented subset on the decks above — they are not a general
claim of SPICE parity. The `.tran` residual is dominated by fixed-step
trapezoidal integration against ngspice's adaptive-step reference, which is a
method difference, not an error in either engine.

## Validation contract

The `docs/physics-model-boundaries.md` matrix states what each model does and
does not include. Passing the suites above demonstrates agreement only inside
each benchmark's stated validity envelope and error budget. Every new physics
model should add at least one analytical, datasheet-envelope, conservation, or
state/rollback regression before its documented claims are expanded.
