# brownout

Brownout is a mixed-signal circuit simulation engine in TypeScript. It solves
the circuit and the firmware together: a modified-nodal-analysis solver with
sparse LU and AMD ordering, trapezoidal/backward-Euler integration, a true DC
operating point with a homotopy rescue ladder, linearized small-signal AC, a
SPICE netlist subset, ~90 device models behind a public registry, and
cycle-accurate AVR (avr8js) and RP2040 (rp2040js) co-simulation with
thermal, battery, and failure physics attached to the same solve.

It runs in Node, the browser, and workers, with no native dependencies.

Extracted from the de:volt application's `@devolt/simcore` package. MIT.

**Status: 0.x.** The device registry is the stability surface; the API can
still move. What is here is covered by 1,602 tests across 88 files, run twice
per change — once per linear-solver backend.

## Install

```bash
npm install brownout            # engine only — no MCU emulators pulled in
npm install avr8js rp2040js     # optional peers, only if you co-simulate MCUs
```

Node >= 20.18.3. ESM-only (`"type": "module"`); CJS consumers can reach it
through dynamic `import()`. The floor is not arbitrary: the part library
imports its JSON with an import attribute (`with { type: "json" }`), which is a
SyntaxError before 20.10.0 and prints an ExperimentalWarning until 20.18.3 —
so 20.18.3 is the oldest runtime on which the package loads cleanly and
silently. Measured, not inferred:
[`docs/packaging-verification.md`](docs/packaging-verification.md).

## Quick start

A 5 V step into a 1 kΩ / 1 µF low-pass, stepped to one time constant:

```ts
import { SimEngine } from "brownout";

const engine = new SimEngine();
engine.setIntegrationMethod("trap");
engine.load({
  components: [
    { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
    { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
    { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6 } },
  ],
  wires: [
    { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
    { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "a" },
    { from_component: "c1", from_pin: "b", to_component: "v1", to_pin: "neg" },
  ],
});

const out = engine.getNetIdForPin("c1", "a")!;
for (let t = 0; t < 1e-3; t += 1e-5) engine.step(1e-5);
console.log(engine.netV[out].toFixed(4)); // 3.1605 — analytic 5*(1-1/e) = 3.1606
```

The engine has no first-class net input: components carry pins, wires join
pins, and `graph.ts` unions them into deterministic nets (`gnd`, `n0`, `n1`,
…). You drive the loop yourself — `step(h)` is the whole contract. For an
adaptive loop, `brownout/host` ships a reference `HeadlessRunner`.

Or bring a SPICE deck:

```ts
import { runSpice } from "brownout/spice";

const result = runSpice(`half-wave rectifier
v1 in 0 sin(0 5 1k)
d1 in out dmod
c1 out 0 10u
rl out 0 1k
.model dmod d(is=1e-14 n=1.8)
.temp 25
.tran 10u 5m
.end`);

result.warnings;                    // [] — every disclosed approximation lands here
result.tran!.nodeVoltages["out"];   // 501 samples, normalized to V(node) - V("0")
```

Runnable, self-checking versions of both — plus a custom device registered
from outside the package — live in [`examples/`](examples) (`pnpm run
examples`).

## Feature matrix

| Area | What ships | Where |
| --- | --- | --- |
| Linear solver | Dense and sparse MNA behind one `LinearSystem` interface, auto-selected at n=64. Sparse: CSC storage, AMD quotient-graph ordering, Gilbert-Peierls left-looking LU with threshold partial pivoting, numeric-only refactorization replay, O(nnz) substitution and residual check | `sim/engine/{linear-system,mna,sparse-mna,amd-ordering}.ts` |
| Nonlinear | Newton-Raphson, full re-stamp per iteration, mixed abs/rel tolerance, SPICE `pnjlim` junction limiting, non-finite iterates abort rather than propagate | `sim/engine/newton.ts` |
| Integration | Backward Euler (default) and opt-in trapezoidal for capacitors/inductors, with a BE anchor forced at every marked discontinuity | `setIntegrationMethod()`, `trap-integration.test.ts` |
| DC operating point | True OP solve with a gmin-stepping → source-stepping → pseudo-transient rescue ladder; the result reports which stage converged | `dcOperatingPoint()`, `dc-homotopy-pathological.test.ts` |
| Small-signal AC | Linearized at the DC OP via a bordered 2n complex solve reusing the sparse kernel; matches transient numeric derivatives to ~1e-10 at quasi-DC | `brownout/ac`, `ac-cross-validation.test.ts` |
| Large-signal AC | Driven sine sweep + single-bin DFT, with a not-settled flag when two measurement windows disagree by more than 0.5 dB | `brownout/analysis` |
| Devices | 90 registered kinds: passives with parasitics, sources, batteries, semiconductors, op-amps/comparators, regulators/converters, ~40 behavioral ICs with family-correct thresholds and delays, electromechanics, magnetics | `sim/engine/devices/*`, `listRegisteredKinds()` |
| Plugin API | `registerDeviceModel()` — compile/stamp/commit/telemetry/save-restore hooks, including `acStamp`. Third-party registration is proven end-to-end | `sim/engine/device-registry.ts`, `device-registry.test.ts` |
| SPICE subset | Netlist parser (R/C/L/K/V/I/D/Q/M/J, `.subckt`, `.model`), directive runner (`.op`/`.tran`/`.dc`/`.ac`), line-numbered errors, warnings for every unmapped parameter | `brownout/spice`, `docs/spice-subset.md` |
| Offline analyses | DC sweep, temperature sweep, AC sweep, seeded Monte Carlo (deterministic and order-independent), operating point, trace measurement, CSV | `brownout/analysis` |
| Physics sidecars | Chemistry battery model with coulomb counting, one-pole package thermal with regulator shutdown hysteresis, servo travel/brownout, i2t failure with latched-open branches | `brownout/physics` |
| MCU co-simulation | Cycle-accurate AVR and RP2040 with sub-step pin-event timing; firmware advances only after accepted electrical steps | `brownout/mcu` |
| Determinism | Analyses bit-reproducible; steps accepted only on converged Newton + non-singular matrix + finite values + relative residual <= 1e-8; rejected steps roll back and never advance firmware | `sim-engine.ts`, `state-rollback-derived.test.ts` |

### Cross-validation against ngspice

`test/sim/engine/spice-ngspice-crossval.test.ts` runs byte-identical circuit
decks through both engines and holds the results to declared envelopes.
Measured 2026-07-17 against ngspice-46 (Homebrew, macOS arm64), all seven
cases inside envelope:

| Case | Result | Envelope |
| --- | --- | --- |
| Resistive divider `.op` | worst dV **9.60e-9 V** at v(2) | 4.00e-2 V |
| RC lowpass step `.tran` | v(out) RMS 1.01e-2 V = **0.212%** of 4.750 V swing | 2% |
| Half-wave rectifier `.tran` | v(out) RMS 2.79e-3 V = **0.064%** of 4.351 V swing | 5% |
| BJT common-emitter `.op` / `.ac` | worst dV **1.50e-6 V** at v(c); worst dMag **2.56e-6 dB**, worst dPhase **8.07e-10 deg** | 20 mV / 1 dB / 5 deg |
| Series RLC ringdown `.tran` | v(n1) RMS 9.37e-3 V = **0.101%** of 9.268 V swing | 2%, t=0 excluded |
| Self-biased JFET `.op` | worst dV **2.86e-8 V** at v(d) | 6.40e-1 V |
| K-coupled transformer `.tran` | v(p) RMS **0.001%** of 7.081 V swing; v(s) RMS **0.003%** of 13.879 V swing | 2% |

Worst case across the corpus: `.op` 1.50e-6 V, `.tran` RMS 0.212% of swing,
`.ac` magnitude 2.56e-6 dB, `.ac` phase 8.07e-10 deg.

Read that for what it is: agreement with the reference implementation on
seven decks inside the documented subset, not a general claim of SPICE parity.
The `.tran` residual is dominated by fixed-step trapezoidal integration
against ngspice's adaptive-step reference — a method difference, not an error
in either engine. The suite skips when no ngspice binary is present, so it is
opt-in locally — but CI installs ngspice and fails the lane if the suite
skipped rather than ran, so every merge has cleared these envelopes against a
live reference (CI's ngspice is Ubuntu's packaged build, not the Homebrew 46
the table above was measured on). Envelopes and method notes:
`docs/physics-reference-benchmarks.md`.

### Scale

`pnpm run bench`, M-series Mac, Node 25. Engine end-to-end on a synthetic RC
ladder at h=10 µs, backend forced either way:

| ladder stages | dense ms/step | sparse ms/step | speedup |
| ---: | ---: | ---: | ---: |
| 200 | 0.316 | 0.184 | 1.72x |
| 800 | 3.112 | 0.756 | 4.12x |
| 1600 | 11.191 | 1.682 | 6.66x |

Fresh factorization on a circuit-like pattern (locality plus rails) at
n=6400: 10.6 ms, fill ratio 5.65. The honest counterpart: expander-class
patterns fill 96.75x at the same size, which equals exact minimum degree and
is inherent to the graph class, not a defect. Circuits are not expanders; a
pathological topology will still be slow.

## What Brownout is not

Read `docs/physics-model-boundaries.md` before trusting a number. The short
version:

- **Not a sign-off tool.** It is a deterministic, lumped-circuit engine. Not a
  field solver, not a substitute for a datasheet or a measurement, and not
  qualified for production corners.
- **Compact models only.** Level-1 Shichman-Hodges, Ebers-Moll with optional
  forward Early, Shockley junctions. No BSIM, no Gummel-Poon charge storage,
  no vendor `.model` ingestion beyond the tabled parameters in
  `docs/spice-subset.md`.
- **No noise analysis in any form.** No shot, flicker, or Johnson noise
  propagated through a frequency-domain network.
- **No distributed effects.** No transmission lines, EM coupling, wire
  inductance, contact resistance, or PCB field effects.
- **Instruments are ideal observers.** Probe capacitance and meter burden are
  not applied back into the circuit unless the circuit includes the load.
- **Every model declares its own envelope.** Each device documents what is
  Included and Not included; results outside that scope are estimates. A
  solver warning means the last operating point failed the convergence or
  residual contract — that frame is not physical data.

The engine is built to reject and report rather than silently emit garbage.
Where it cannot solve, it says so.

What it does that SPICE does not: run real firmware in-circuit with
transactional electrical coupling, brown out a servo from battery sag,
thermally shut down a regulator mid-transient. What SPICE does that it does
not: multi-order stiff integration, noise, vendor model ingestion, and the
device-model depth of a 50-year-old ecosystem.

## MCU support

Board kinds are ordinary components; their cores are driven by the same solve.
One converged electrical interval is committed first, then its endpoint inputs
are pushed into the core and the core advances exactly once — so a rejected
electrical trial never advances firmware, and each accepted endpoint is
sampled once.

| Board kind | Core | Emulator | Clock | Program | I/O surface |
| --- | --- | --- | --- | --- | --- |
| `arduino_uno`, `arduino_nano` | ATmega328P | avr8js (optional peer) | 16 MHz | Intel HEX | Digital ports B/C/D, ADC on A0-A7 (A6/A7 Nano-only), 35 kΩ internal pullups |
| `raspberry_pi_pico` | RP2040 | rp2040js (optional peer) | 125 MHz | MicroPython UF2 + raw-REPL script | GP0-GP22, GP25 (on-board LED), ADC on GP26-GP28, 65 kΩ internal pulls |
| `microbit` | — | not emulated in-engine | — | — | Edge pins P0/P1/P2 accept externally driven states via `setMicrobitDrive()` |

MCU pins stamp as rail-referenced Thevenin drivers whose complementary
conductances sum to 1/50 Ω against the actual VCC/GND package nodes, so rail
sag is physical and current is conserved at both rails. Pin modes: input
(hi-Z), input-pullup, input-pulldown, out-high, out-low. `PinEvent` cycle
timestamps give sub-step edge timing for servo PWM, HC-SR04 TRIG, and
MAX7219/HD44780 SPI decode.

Neither emulator is a hard dependency, and no runtime import of either exists
in the engine graph — verified against the built artifact. Register what you
need:

```ts
import { setAvr8Module, setRp2040Module, mcuFactory } from "brownout/mcu";

setAvr8Module(await import("avr8js"));
const uno = mcuFactory("arduino_uno", { hex: blinkHex });

// The Pico wrapper carries the vendored boot ROM and a static rp2040js
// import, so it lives behind its own subpath — load it lazily.
setRp2040Module((await import("brownout/mcu/rp2040")).RP2040Mcu);
```

An Arduino with firmware but no registered avr8js throws a clear
install-avr8js error (an explicit "run this" request should not look like a
solver bug). A Pico without firmware or module stays inert until both are
registered; a reload boots it.

Neither peer is needed to typecheck. `brownout/mcu` and `brownout/mcu/rp2040`
both resolve with avr8js/rp2040js absent, and they do so under
`skipLibCheck: false` — the published `.d.ts` files keep peer types out of
public seams (`arduino.d.ts` uses a structural stand-in for the avr8js module
namespace), so nothing in a public signature chases a package you did not
install.

## Entry points

| Entry | Contents |
| --- | --- |
| `brownout` | SimEngine, circuit domain types, device registry, linear-system backends, part library API |
| `brownout/spice` | SPICE netlist parser, `.model` mappers, directive runner |
| `brownout/ac` | Linearized small-signal AC at the DC operating point |
| `brownout/analysis` | Pure offline runners: DC/temp sweep, driven AC sweep, Monte Carlo, operating point, measurement, CSV |
| `brownout/physics` | Battery / thermal / servo sidecars + telemetry builder |
| `brownout/host` | Headless Node runner + the step-controller contract for out-of-tree adapters |
| `brownout/mcu` | MicrocontrollerCore contract, mcuFactory, firmware + emulator registration |
| `brownout/mcu/rp2040` | RP2040Mcu wrapper (statically imports rp2040js; load lazily) |

Anything not exported from these entries is internal. The `@brownout/*` npm
scope is reserved for standalone host adapters — nothing is published there
yet.

## Part library injection

The engine resolves component identity (`catalogUid`) against an injectable
part library. The bundled default (`src/parts/default-parts.json`) is an open
subset — engine-relevant fields only — sufficient for the engine corpus:
family-correct logic thresholds, regulator/LED/battery identities, thermal
package profiles. Hosts with their own catalogs inject them:

```ts
import { setPartLibrary, BUNDLED_DEFAULT_PARTS } from "brownout";

setPartLibrary([...BUNDLED_DEFAULT_PARTS, ...myParts]); // extend
setPartLibrary(myParts);                                // or replace
```

`PartDefinition` is model identity and nothing else — `uid`, `kind`,
`default_params`, `pin_layout`, `electrical_specs`, `spice_model`. That is the
complete set the engine reads, and the bundled JSON carries exactly those
fields. A product catalog usually holds much more (display names, palette and
teaching copy, BOM and purchasing data, tier flags); keep it on your own entry
type and inject that — the extra properties travel with your objects and the
engine ignores them, because TypeScript's excess-property check does not apply
to values passed through a variable.

Injection is reactive, not load-order-bound: resolvers, thermal LED profiles,
and the engine's identity cache all key on a library generation counter, so a
late `setPartLibrary()` call takes effect on the next lookup. Engines already
loaded keep the identities they resolved at `load()` time until reloaded.

## Development

```bash
pnpm install
pnpm run build          # tsc -> dist/ (js + d.ts + source maps, sources inlined)
pnpm run typecheck
pnpm test               # full engine corpus, dense linear backend (default)
pnpm run test:sparse    # same corpus with the sparse backend forced
pnpm run bench          # dense-vs-sparse linear backend scaling benchmark
pnpm run check:package  # publint + attw --pack (esm-only profile)
pnpm run examples       # build, then run examples/ against dist via the exports map
```

`pnpm run examples` is the exports-map gate: the examples import the package by
name (`brownout/host`, not `../src`), so they only resolve if the built `dist/`
and the exports map actually work for a consumer. That is also why they are
typechecked separately (`typecheck:examples`) — they need a build first, while
`pnpm run typecheck` must stay green on a fresh clone.

The corpus runs twice, once per linear-solver backend, via the
`SIMCORE_LINEAR_BACKEND=sparse` switch read by
`src/sim/engine/linear-system.ts`. The `SIMCORE_` prefix predates the brownout
name; it is engine API carried over as-is, so renaming it (with a deprecation
window) is deliberately deferred.

The ngspice cross-validation suite runs wherever an ngspice binary exists and
skips otherwise; point `NGSPICE_BIN` at one outside the probed locations. CI
installs ngspice and then greps the suite's own summary line to prove it ran —
a silent skip there would make a green build a lie about the headline claim.

`src/` deliberately mirrors `packages/simcore/src` in the de:volt monorepo
(`sim/engine/**`, `sim/*.ts` sidecars, `analysis/**`, `circuit/*.ts`) even
though the `sim/` prefix is redundant here. Both trees exist during the
transition and engine fixes may need to move in either direction; identical
paths keep every cross-repo diff a plain `diff -r`. The one mechanical
deviation: every relative import carries an explicit `.js` extension
(NodeNext), because the published `dist/` is plain tsc output that Node itself
must resolve.

`prepack` rebuilds `dist/` on every `npm pack` and `npm publish`. It has to:
`dist/` is gitignored, so without that hook a publish from a clean checkout
would produce a tarball with no code in it and no warning. `check:package` is
deliberately NOT wired to a lifecycle hook — `attw --pack` shells out to
`npm pack`, so from `prepack` it recurses forever, and from `prepublishOnly`
it breaks `npm publish --dry-run` (npm exports `npm_config_dry_run`, the nested
pack writes no tarball, and attw fails on the missing file). Run it directly,
in review or CI.

### Deliberately not extracted

Every solver, device, analysis and physics module from simcore is here. What
was left behind is app-domain — transport shells, app state, editor geometry —
and its absence is a decision, not an oversight:

| Left in de:volt | Why |
|---|---|
| `sim/sim.worker.ts` (1046), `sim/worker-client.ts` (280) | Browser `Worker` transport around the engine. A host picks its own transport; the engine stays loop-free and clock-free. |
| `sim/analysis.worker.ts` (118), `analysis/analysis-worker-client.ts` (128) | Same, for the offline runners. The runners themselves ship under `brownout/analysis` and are Node-safe. |
| `sim/live-store.ts` (446) | zustand store — app state management. |
| `sim/net-colors.ts` (57) | Canvas net palette. Presentation. |
| `circuit/migrate.ts` (124) | de:volt save-format migration. Tied to their schema history. |
| `circuit/microbit-breakout.ts` (91), `circuit/dip-switch-resize.ts` (117) | Editor placement/resize helpers. |
| `diagnostics/**` (~2085) | App-domain rule engine over engine output, not part of the solver. The engine corpus exercises it, so a copy lives in `test/helpers/diagnostics/` and is never published. |

The same line applies to types: `circuit/types.ts` models the document the
engine consumes and the catalog identity it resolves, not a host's editor
document or product catalog. See **Part library injection** below.

## Docs

- [`docs/physics-model-boundaries.md`](docs/physics-model-boundaries.md) —
  what each model does and does not include. The honest matrix.
- [`docs/physics-reference-benchmarks.md`](docs/physics-reference-benchmarks.md)
  — analytical golden tests with declared validity envelopes and error
  budgets, plus the ngspice cross-validation contract.
- [`docs/spice-subset.md`](docs/spice-subset.md) — the netlist grammar,
  directive support, `.model` parameter mapping, and per-device fidelity
  notes.
- [`docs/releasing.md`](docs/releasing.md) — the owner's release runbook:
  npm trusted publishing, provenance, and the changesets flow.
- [`CONTRIBUTING.md`](CONTRIBUTING.md) — the two test lanes, the bit-identity
  contract, and the rules for changing a number or citing prior art.

## License

MIT, Copyright (c) 2026 de:volt. See [LICENSE](LICENSE).

Third-party material and its licensing — avr8js (MIT), rp2040js (MIT), the
vendored RP2040 boot ROM (BSD-3-Clause, Copyright (c) 2020 Raspberry Pi
(Trading) Ltd), and the MicroPython test firmware (MIT) — is recorded in
[NOTICE](NOTICE).

Contributions: never copy code from GPL-licensed simulators (Falstad
CircuitJS1 in particular). A ported device model carries its license with it,
invisibly, and would contaminate this one.
