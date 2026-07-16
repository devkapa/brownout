# Physics model scope

Brownout is a deterministic, lumped-circuit simulation engine. It solves the
electrical circuit and selected thermal, mechanical, firmware, and instrument
models together. It is designed to explain cause and effect and to catch common
bench mistakes. It is not a field solver, a production SPICE sign-off tool, or
a substitute for a component datasheet and measurement.

Each device model declares its identity and an **Included / Not included**
envelope; host applications should surface that summary for each selected
component (de:volt's Inspector, which this engine was extracted from, does).
Results outside that stated scope are estimates.

## What a displayed value means

- A voltage or current is shown only when the engine produced a finite solved
  value. Missing telemetry is shown as unavailable, never converted to zero.
- Component current labels state their reference direction or aggregation.
  Multi-channel package totals are not presented as a single-pin current.
- Power is derived only where voltage and signed current share an unambiguous
  terminal reference.
- Temperatures are values from the declared lumped thermal profile. They are
  not package-surface measurements or a detailed junction-temperature map.
- A solver warning means the last operating point did not meet the numerical
  convergence or residual contract. Do not treat that frame as physical data.

## Electrical solver

Included:

- Modified nodal analysis with nonlinear Newton iteration, matrix
  equilibration, pivot/rank checks, residual validation, and rollback on a
  rejected step.
- Adaptive transient stepping with alignment to authored source edges.
  Rewindable circuits use step-doubling error control; live Arduino/Pico
  firmware circuits use a conservative 100 µs ceiling because the external CPU
  cores cannot be rewound for trial replays.
- Backward-Euler capacitor and inductor companions, including configured
  capacitor ESR/leakage and inductor winding/core-loss resistance. Trapezoidal
  companions are available as an opt-in integration method with backward-Euler
  anchoring at marked discontinuities.
- New and cold-reset capacitors/inductors begin with exactly zero stored
  voltage/current. Compact oscillator models start from their powered device
  transitions; the engine does not inject hidden energy into general passives.
- A 1 Tohm numerical node shunt used only to regularize otherwise floating
  matrices. Diagnostics call out isolated catalog inputs and explicit
  resistances near that numerical limit.
- Rail-referenced, finite-resistance digital and MCU outputs, real internal GPIO
  pulls, board idle current, IC quiescent current, and conservative supply-current
  routing for modeled active parts.
- Same-solve piecewise voltage/current limiting for declared bench supplies,
  regulators, converters, and op-amp outputs. A limiting regime is an
  instantaneous compact-model constraint, not a simulated control-loop waveform.
- Latched-open resistor, fuse, diode/LED and passive-network failures remove
  their electrical branch. Resetting a failure re-solves the restored topology
  before a new reading is published.
- A true DC operating point (`dcOperatingPoint()`) with a gmin-stepping,
  source-stepping, and pseudo-transient rescue ladder for hard nonlinear
  startups.

Not included:

- Distributed transmission lines, electromagnetic coupling, breadboard/contact
  resistance, wire inductance, antenna behavior, or PCB field effects.
- General shot, flicker, and Johnson noise propagated through a frequency-domain
  network.
- A topology-independent per-net Thevenin/output-resistance calculation.
  Connected floating islands or weak paths that are not an isolated catalog
  input or an explicit extreme resistor may not receive a high-impedance badge.
- Convergence for every pathological nonlinear startup. A circuit that still
  fails the convergence contract after the rescue ladder is surfaced as
  unresolved, never silently approximated.
- Arbitrary SPICE model cards or production corner qualification. The SPICE
  netlist subset and its per-parameter fidelity notes are documented in
  `docs/spice-subset.md`.

## Semiconductor and IC models

Included:

- Ambient-dependent junction thermal voltage and forward-voltage drift.
- Shockley diode/LED conduction, zener/TVS breakdown, Ebers-Moll BJT DC behavior
  with optional forward Early effect, level-1 MOSFET regions/body diodes, and
  lumped Cgs/Cgd Miller coupling within their disclosed envelopes.
- JFET (Shichman-Hodges), SCR/triac latching regimes, optocoupler, crystal
  (motional-branch), analog switch, and K-coupled inductors/transformers.
- Supply-aware regulators, converters, logic families, MCU GPIO, common drivers,
  timers, displays, and sensors with finite output strength where modeled.
- Op-amp dominant-pole bandwidth, slew-rate limiting, rail headroom, finite
  output resistance/current, quiescent current, and output-current limiting for
  the explicitly identified op-amp variants.

Not included:

- Semiconductor layout effects, avalanche energy, MOSFET subthreshold detail,
  voltage-dependent capacitance and complete gate-charge curves, BJT junction
  capacitance/charge storage/high-injection behavior, or every second-order
  datasheet limit.
- Op-amp input noise, full distortion spectra, input offset/bias-current
  distributions, common-mode phase reversal, or a vendor transistor-level
  macro-model.
- Switching-regulator ripple, EMI, control-loop stability, and magnetics
  saturation. DC-DC converters use an averaged power-flow model with
  instantaneous piecewise limits. The identified buck module is non-isolated,
  so its input and output negative terminals share one internal return. Its
  maximum-duty ceiling uses a fixed 1.5 V dropout assumption; real dropout
  varies with current, temperature, switch loss, diode loss and inductor DCR.

## Temperature and environment

Included:

- Ambient temperature feeds junction laws, thermistors, battery cold behavior,
  and any identified lumped package thermal profile.
- Thermistor self-heating uses its configured dissipation factor and thermal
  time constant.
- Identified regulators, op-amps, resistors, and indicator LEDs use a one-pole
  body/junction temperature model with explicit package assumptions. Regulators
  with declared protection use reversible thermal-shutdown hysteresis.
- Ambient light drives the LDR using a GL5528-class power-law curve.

Not included:

- Multi-pole transient thermal impedance, heatsink geometry, airflow, enclosure
  gradients, heat transfer between nearby components, or temperature-dependent
  aging.
- Humidity, moisture, pressure, magnetic field, or automatic optical coupling
  between an LED and light sensor.
- Electrolytic cold-aging, detailed resistor temperature coefficients, and a
  fully temperature-dependent transistor mobility/beta model unless a component
  disclosure says otherwise.

## Batteries and electromechanics

Included:

- Exact catalog battery identities use chemistry-shaped open-circuit voltage,
  depletion- and temperature-dependent internal resistance, and rollback-safe
  coulomb counting. Legacy/custom batteries stay on a labeled generic model.
- DC motors include winding resistance/inductance, back-EMF, torque, inertia,
  friction, and configured load torque.
- Hobby servos decode command pulses and move toward the target with a
  supply-dependent finite speed.
- Relays include coil inductance and pull-in/drop-out hysteresis; steppers track
  coil current and full-step phase.

Not included:

- Battery diffusion/recovery, Peukert-rate effects, self-discharge, aging,
  cell imbalance, rechargeable chemistry, and internal heat generation.
- Gear backlash, servo acceleration/stall torque/control hunting, brush arcing,
  motor magnetic saturation, bearing wear, and mechanical coupling between
  separate components.
- Relay contact bounce, contact wear/welding, and mechanical acoustic effects.

## Oscilloscope, logic analyzer, and analyses

Included:

- Uniform probe output inside the integration loop. Values between accepted
  solver states use linear dense interpolation; the reported effective rate and
  Nyquist limit are capped by the coarsest accepted solver interval in the
  acquisition. Source/event discontinuities are step-aligned, and gaps remain
  explicit.
- Per-channel DC or AC coupling, a declared 12-bit ADC, calibrated timebase,
  trigger/cursor/measurement provenance, and deterministic front-end display
  noise that is fixed for one acquisition.
- Logic-analyzer auto thresholds come from exact receiver VIL/VIH envelopes;
  mixed receiver families require a manual threshold, and the host should
  disclose the electrically undefined band around its midpoint display decision.
- Clean solved samples remain separate from optional display noise. Exports and
  measurements identify the domain they contain.
- DC operating point, parameter sweep, AC sine/DFT sweep, linearized
  small-signal AC, and seeded Monte Carlo analyses use convergence-checked
  steps and preserve exact catalog identity.

Not included:

- Physical probe capacitance/resistance and multimeter burden applied back into
  the circuit. Instruments are ideal observers unless the circuit explicitly
  includes the corresponding load.
- Analog front-end clipping/recovery detail, calibration drift, full ADC INL/DNL,
  alias-rejection filters, and circuit-derived broadband noise.
- Probabilistic receiver behavior, metastability, or a three-valued digital
  trace inside the VIL-to-VIH indeterminate band.
- Noise analysis in any form. The driven AC sweep is a transient measurement,
  so nonlinear results can depend on stimulus amplitude; the linearized
  small-signal AC analysis is amplitude-independent by construction but valid
  only near the operating point it linearizes.

## Validation contract

The analytical reference suite is documented in
`docs/physics-reference-benchmarks.md`, which also records the measured
agreement with ngspice on the documented SPICE subset. Passing it demonstrates
agreement only inside each benchmark's stated validity envelope and error
budget. Every new physics model should add at least one analytical,
datasheet-envelope, conservation, or state/rollback regression before its
documented claims are expanded.
