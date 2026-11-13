---
"brownout": minor
---

brownout now includes a second simulation engine, the core: the transient solver rewritten in Rust and compiled to WebAssembly. It is event-driven, so a board that is mostly logic chips (an 8-bit breadboard computer, for example) no longer costs a whole-circuit solve at every time step. The classic TypeScript engine is still in the package, unchanged, and it is the fallback for every circuit the core does not cover.

**What runs on the core.** A circuit runs on the core only when every part in it is one of these:

- resistors, capacitors, and the switches (`switch`, `push_button`, `spdt_switch`, `dip_switch`);
- `fuse` and `ptc_fuse`;
- the sources: `voltage_source`, `battery_pack`, `bench_psu`, `clock`, `clock_gen`, `pulse_source`, `pulse_gen`;
- `led`, `rgb_led`, `bicolor_led` and the 7-segment displays (`seg7_cc`, `seg7_ca`);
- the `ne555`;
- logic: `74ls00`, `74ls04`, `74ls08`, `74ls32`, `74ls86`, `74ls157`, `74ls283`, `74hc138`, `74hc14`, `74ls47`, `74ls245`, `74ls161`, `74ls173`, `74hc74`;
- memory: `74ls189`, `28c16`, `28c256`.

One part outside that list sends the whole circuit to the classic engine: transistors, diodes, op-amps, regulators, inductors, relays, motors, potentiometers, sensors, the remaining logic chips, and the Arduino, Pico and micro:bit boards are all classic-only in this release. The two engines never share a circuit. The core also turns down a few wirings of supported parts (more than one battery or current-limited supply in a circuit, for instance) and says why.

On the core, a run keeps what a host expects from classic: live switch and source-voltage changes without a reload, state carried across an edit (capacitor charge, latches, battery charge, failures, EEPROM contents), snapshots, failures and warnings, the EEPROM write readback, scope samples and LED light averages.

**Nothing changes until a host asks for it.** `HeadlessRunner` still defaults to the classic engine, and the analyses' `engine: "auto"` still resolves to classic, because brownout's own list of kinds cleared for automatic routing (`CORE_SUPPORTED_KINDS`) is empty in this release. A host opts in by naming the kinds it routes; `CORE_IMPLEMENTED_KINDS` is the list above:

```ts
import { initCore, CORE_IMPLEMENTED_KINDS } from "brownout/core";
import { HeadlessRunner } from "brownout/host";

await initCore(); // once; loads the wasm that ships in the package
const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
runner.load(circuit);
runner.activeEngine; // "core", or "classic" after a fallback
runner.engineFallbackReasons; // why, for example ["unsupported-kind:bjt_npn"]
```

`CoreEngine` (also from `brownout/core`) is the engine itself, for a host that runs its own loop. The sweep and Monte Carlo runners take `engine: "core"` and a `coreSupportedKinds` hook option in the same way.

**When the core cannot take a circuit**, `HeadlessRunner` loads it on classic instead and lists the reasons: `unsupported-kind:<kind>`, `unsupported-feature:<what>:<id>`, `malformed:<id>:<what>`, `topology:<message>`, `livelock:<message>`, `solve-failed:<message>`, `carry:<reason>` for an edit whose state could not be carried, and `core-not-initialized` or `core-poisoned` when the wasm is not ready. `CoreEngine.load()` throws `CoreUnsupportedError` with the same reasons.

**When the core fails in the middle of a run**, it stops at the last instant it completed and reports `lastConverged === false`, as classic does when a step does not converge; `HeadlessRunner.run()` returns `hitMinStep: true`. It does not switch engines by itself in the middle of a run: the host decides whether to reload the circuit on classic. A crash inside the wasm throws `CoreTrapError`; the module is then unusable until `initCore()` is awaited again.

**Things that differ on the core.**

- Results agree with classic within the tolerances the equivalence tests use (`core/README.md` in the repository, "Equivalence harness rules"). They are not bit-identical.
- There is no fixed time step. A fixed-step headless run samples on the grid you ask for, but the core chooses its own steps in between. The classic solver's diagnostics (Newton iterations, matrix size, residual) read zero.
- Step breakpoints (`captureStepBreakpoints`) are classic-only.
- The `SolverDiagnostics` message type (`sim/messages`, reached through `brownout/internals/*`) is now a union: classic's fields as before, or the core's counters under `core` when `engine === "core"`. TypeScript that reads classic's fields from it has to narrow on `engine` first.
- A `bench_psu` driven above its setpoint by another source still sinks current on the core. The classic engine no longer does (see the `bench_psu` entry in this release).
- The exact rail mode (`noRailShortcut`) is chosen when a circuit is loaded and cannot be changed afterwards.
- `CoreEngine.saveState()` returns a snapshot of the whole engine and `restoreState(snapshot)` puts it back. Call `snapshot.dispose()` when you are done with one; a `load()` retires every snapshot taken before it.

**Loading the wasm.** The binary is `dist/core/brownout-core.wasm`, about 1.9 MB (0.7 MB gzipped), also exported as `brownout/core.wasm`. `initCore()` finds it by itself in Node, and in webpack and Vite client or worker bundles. In a Next.js server bundle, add `brownout` to `serverExternalPackages` or pass the bytes to `initCore()` yourself. The wasm is loaded only when a host calls `initCore()`; a host that never does runs exactly as before.
