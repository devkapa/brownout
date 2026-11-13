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

**The core is on where it is supported, once a host has loaded it.** A host turns the core on by awaiting `initCore()`; a host that never does runs on the classic engine exactly as before. After that:

- The sweep and Monte Carlo runners default to `engine: "auto"`, so they run on the core whenever every part of the circuit is on brownout's supported list, and on classic otherwise. The result says which (`engine`) and why (`fallbackReasons`).
- `HeadlessRunner` still defaults to `"classic"`. Ask for `"auto"` to get the same choice there:

```ts
import { initCore } from "brownout/core";
import { HeadlessRunner } from "brownout/host";

await initCore(); // once; loads the wasm that ships in the package
const runner = new HeadlessRunner({ engine: "auto" });
runner.load(circuit);
runner.activeEngine; // "core", or "classic" after a fallback
runner.engineFallbackReasons; // why, for example ["unsupported-kind:bjt_npn"]
```

The supported list (`CORE_SUPPORTED_KINDS`) is the list of parts above: every kind the core runs has cleared the two-engine comparison. The two names stay separate because a kind added to the core later is implemented (`CORE_IMPLEMENTED_KINDS`) before it is supported, and until then a circuit holding it runs on classic under `"auto"`, with `unsupported-kind:<kind>` as the reason. A host can pass its own list as `coreSupportedKinds`.

`HeadlessRunner` takes three values for `engine`:

- `"classic"` (the default): the classic engine, always.
- `"auto"`: the core when it can take the circuit, classic otherwise, and classic again if the core fails part-way through a run (below).
- `"core"`: chooses at `load()` as `"auto"` does, but once a circuit is running on the core it stays there, and a failure is reported to the caller instead of being covered by classic.

`CoreEngine` (also from `brownout/core`) is the engine itself, for a host that runs its own loop. The sweep and Monte Carlo runners take `engine` and a `coreSupportedKinds` hook option in the same way; there `"core"` never falls back at all and throws when the core cannot run the analysis.

**When the core cannot take a circuit**, `HeadlessRunner` (on `"auto"` or `"core"`) loads it on classic instead and lists the reasons: `unsupported-kind:<kind>`, `unsupported-feature:<what>:<id>`, `malformed:<id>:<what>`, `topology:<message>`, `livelock:<message>`, `solve-failed:<message>`, `carry:<reason>` for an edit whose state could not be carried, and `core-not-initialized` or `core-poisoned` when the wasm is not ready. `CoreEngine.load()` throws `CoreUnsupportedError` with the same reasons.

**When the core fails in the middle of a run** (a solve that does not converge, logic that never settles, or a crash inside the wasm), what happens depends on what the host asked for:

- `HeadlessRunner` on `"auto"` finishes the run on classic. No state is carried from one engine to the other, so classic starts again from the load: it loads every circuit the runner gave the core, runs again every `run()` the core had completed, then the run that failed. The readings afterwards are the ones a classic runner gives for the same calls. `activeEngine` becomes `"classic"` and `engineFallbackReasons` says why, for example `["mid-run:solve-failed:<message>", "restarted-from-load"]` or `["core-trap", "restarted-from-load"]`. The cost is the time to simulate that history a second time, and `onSample` is called at the end of each core run instead of during it.
- A sweep or Monte Carlo run on `"auto"` is run again from its first point on classic, so one result never mixes engines. `onRestart(reasons)` is called first, and the result carries `engine: "classic"` and the reasons.
- On an explicit `"core"`, nothing is covered. `HeadlessRunner.run()` returns `hitMinStep: true` with the readings of the last instant the core completed, an analysis throws its convergence error, and a crash throws `CoreTrapError`.
- `CoreEngine` itself stops at the last instant it completed with `lastConverged === false` and `lastAdvanceFailure` set; it never throws for a failed solve.

After a crash inside the wasm the module is unusable until `initCore()` is awaited again; until then every core request falls back with `core-poisoned`.

**Things that differ on the core.**

- Results agree with classic within the tolerances the equivalence tests use (`core/README.md` in the repository, "Equivalence harness rules"). They are not bit-identical.
- There is no fixed time step. A fixed-step headless run samples on the grid you ask for, but the core chooses its own steps in between. The classic solver's diagnostics (Newton iterations, matrix size, residual) read zero.
- Step breakpoints (`captureStepBreakpoints`) are classic-only.
- The `SolverDiagnostics` message type (`sim/messages`, reached through `brownout/internals/*`) is now a union: classic's fields as before, or the core's counters under `core` when `engine === "core"`. TypeScript that reads classic's fields from it has to narrow on `engine` first.
- A `bench_psu` driven above its setpoint by another source still sinks current on the core. The classic engine no longer does (see the `bench_psu` entry in this release).
- The exact rail mode (`noRailShortcut`) is chosen when a circuit is loaded and cannot be changed afterwards.
- `CoreEngine.saveState()` returns a snapshot of the whole engine and `restoreState(snapshot)` puts it back. Call `snapshot.dispose()` when you are done with one; a `load()` retires every snapshot taken before it.

**Loading the wasm.** The binary is `dist/core/brownout-core.wasm`, about 1.9 MB (0.7 MB gzipped), also exported as `brownout/core.wasm`. `initCore()` finds it by itself in Node, and in webpack and Vite client or worker bundles. In a Next.js server bundle, add `brownout` to `serverExternalPackages` or pass the bytes to `initCore()` yourself. The wasm is loaded only when a host calls `initCore()`; a host that never does runs exactly as before.
