/**
 * Gate 1's suite adapter (devolt memory/decisions/2026-09-25-brownout-core-w1-plan.md,
 * section 3.4.1; worker A5's brief). A drop-in stand-in for classic
 * `SimEngine` (src/sim/engine/sim-engine.ts), aliased in for an allowlist of
 * existing suite files by vitest.core-suites.config.ts's resolver plugin --
 * those files are NEVER edited; they import `SimEngine`/`SimCircuit` exactly
 * as they always have; only WHICH module answers that import changes, and
 * only inside this dedicated vitest project.
 *
 * WHY THIS WRAPS HeadlessRunner, NOT CoreEngine DIRECTLY
 * HeadlessRunner's `engine: "core"` mode already implements exactly the
 * policy gate 1 needs: try the core for an explicit, caller-chosen kind set,
 * and report (via `activeEngine`/`engineFallbackReasons`) whether it actually
 * landed there or fell back to classic (src/host/headless.ts). Re-implementing
 * that routing/fallback logic against the bare `CoreEngine` facade here would
 * duplicate a policy that is already written, tested (test/host/headless-core.test.ts)
 * and reviewed. `coreSupportedKinds` is deliberately NOT `CORE_SUPPORTED_KINDS`
 * (kinds.ts) -- decision 15 keeps that empty for the whole of W1, on purpose,
 * because nothing has cleared devolt's own sim/learn/simcore suites yet
 * (gate 3). Gate 1 is part of what PROVES a kind, so it must be allowed to
 * route kinds `CORE_SUPPORTED_KINDS` has not blessed -- exactly the widened-set
 * use case `HeadlessRunnerOptions.coreSupportedKinds`'s own doc comment
 * describes ("this package's own tests"). Defaulting to `CORE_IMPLEMENTED_KINDS`
 * (also kinds.ts) means this adapter needs no edit as later sub-waves grow
 * that set -- it automatically offers more kinds to the core, and it is
 * classification.json (not this file) that decides which suites/cases are
 * actually exercised against the wider surface.
 *
 * NO SILENT FALLBACK (the worker brief's central requirement)
 * A test must not pass by quietly running on classic instead of the core --
 * that would make gate 1 vacuous (risk 3, section 3.6 of the W1 plan). Every
 * `load()`/`coldLoad()` call below asserts `activeEngine === "core"` and
 * throws `CoreSuiteFellBackError` (with the runner's own fallback reasons)
 * the instant it is not. See adapter.test.ts for the proof: an unsupported
 * kind is shown to make `load()` throw, not silently pass.
 *
 * WHAT IS -- AND IS NOT -- IMPLEMENTED HERE
 * The full `EngineReadout` surface (surface.ts) is implemented, since it is
 * cheap and well-specified (a pure delegation to `this._runner.readout`) and
 * useful to any future suite that only reads, never steps. `load`, `coldLoad`
 * and `step(h)` are implemented per the plan's own mechanism description
 * ("maps load, coldLoad, step(h) -> advance(simTime + h)"). The solver
 * diagnostics fields (`lastIters`, `lastMatrixSingular`, ...) are implemented
 * by delegating to `HeadlessRunner.snapshot().solver`, which is ALREADY the
 * single place that documents their W0/W1a "no core meaning yet" neutral
 * values (headless.ts's own `snapshot()` doc comment) -- this file does not
 * duplicate that convention, it reads it. `setIntegrationMethod` (added for
 * trap-integration.test.ts, once W1b's stepped partitions gave the core real
 * BE/trap dynamics to choose between) is implemented too -- see its own doc
 * comment for why it is a two-step (record, then apply on the next `step()`)
 * rather than an immediate call into `HeadlessRunner`. `getElementChannelI`
 * (added for wave3-passives.test.ts as a constant `{}`, provably right while no
 * multi-channel kind was implemented) delegates to the active readout since W1e:
 * rgb_led, bicolor_led and the 7-segment displays publish channel currents.
 * `getBatteryState` and `getIcState` (W1f, for battery-engine-physics and
 * wave5-regulators) delegate to the runner's `CoreEngine` and to nothing else,
 * as do W1g's `getPtcTripped`, `getThermalState`, `getPartTemperature` and
 * `resetFailures` (W1g Q5: on `CoreEngine` only, never `EngineReadout`).
 * `takeEepromUpdates` (W2d, for eeprom-write) delegates to the runner's
 * `CoreEngine` too, and `step()` first delivers the suite's in-place edits to
 * the circuit object it loaded (classic re-reads that reference every step):
 * live params through `setParams`, anything else as a thrown
 * `CoreSuiteEditNotDeliverableError` until W2c's carry and live V land.
 * Nothing else on classic `SimEngine` (saveState/restoreState/
 * captureErrorState/setMicrobitDrive/the dozens of kind-specific setters,
 * ...) is implemented: no case in classification.json currently needs any of
 * them, and inventing behaviour for a method no allowlisted test calls would
 * be exactly the kind of unverified guess the W1 plan's risk 15 warns about.
 * A suite file that needs one fails with a plain "not a function" TypeError,
 * which is the honest outcome -- add the method here, with a reason, when a
 * suite that needs it is added to classification.json.
 */

import { HeadlessRunner } from "../../../src/host/headless.js";
import { CoreEngine } from "../../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { liveParamDiff } from "../../../src/core/edit.js";
import type { EngineReadout } from "../../../src/core/surface.js";
import type {
  BatteryRuntimeReadout,
  SimCircuit as RealSimCircuit,
  SimFailure,
  SimWarning,
  SolverDiagnosticsSnapshot,
  ThermalRuntimeReadout,
} from "../../../src/sim/engine/sim-engine.js";
import type { Net } from "../../../src/sim/engine/graph.js";
import type { DisplayInfo } from "../../../src/sim/messages.js";

/**
 * Re-exported so an allowlisted suite file's own
 * `import { SimEngine, type SimCircuit } from ".../sim-engine.js"` keeps
 * typechecking unchanged after the alias swaps the VALUE import -- this is
 * the exact same classic `SimCircuit` type, not a redeclaration of it.
 */
export type SimCircuit = RealSimCircuit;

/**
 * Thrown by `load()`/`coldLoad()` when the circuit did not actually run on
 * the core. This is the adapter's no-silent-fallback detector: a test that
 * would otherwise have quietly exercised classic instead fails loudly here,
 * with the exact reasons HeadlessRunner recorded (`unsupported-kind:...`,
 * `topology:...`, `core-not-initialized`, `core-poisoned`, ...).
 */
export class CoreSuiteFellBackError extends Error {
  readonly reasons: readonly string[];

  constructor(reasons: readonly string[]) {
    super(
      "core-suites adapter: this circuit did NOT run on the core -- it fell back to classic " +
        `(reasons: ${reasons.length > 0 ? reasons.join(", ") : "<none reported>"}). A gate-1 ` +
        "test must never pass by silently exercising classic instead of the core. Either this " +
        "case's kinds are missing from CORE_IMPLEMENTED_KINDS (src/core/kinds.ts, owned by the " +
        "orchestrator) or the core rejected the circuit's topology/parameters -- classification.json " +
        "should mark this case \"not-yet\" until that is fixed, and vitest.core-suites.config.ts's " +
        "testNamePattern must not select it.",
    );
    this.name = "CoreSuiteFellBackError";
    this.reasons = reasons;
  }
}

/**
 * Thrown by `step()` when the suite edited the circuit object it loaded in a
 * way the adapter cannot deliver to the core yet. Classic re-reads the live
 * circuit every step, so an edit it cannot deliver would otherwise be dropped
 * and the case would pass or fail on a circuit the core never ran.
 *
 * - `live-v-pending`: only voltage_source V changed (code 2 param 0). Ruled
 *   live for W2 (D12) but not wired in Rust yet; `VOLTAGE_SOURCE_V_LIVE` in
 *   src/core/edit.ts is the flag the W2c Rust worker flips.
 * - `needs-carry`: a topology or non-live param change. Delivering it means a
 *   reload that carries state across (W2c, `src/core/carry.ts`); a cold reload
 *   would drop the capacitor charge and latches the case is checking.
 * - `not-applied`: `setParams` returned false (it skipped a change for a
 *   component the loaded program does not hold), so the edit did not land.
 */
export class CoreSuiteEditNotDeliverableError extends Error {
  readonly reason: "live-v-pending" | "needs-carry" | "not-applied";

  constructor(reason: "live-v-pending" | "needs-carry" | "not-applied", detail: string) {
    super(
      `core-suites adapter: the suite edited the loaded circuit in a way the core cannot take yet (${reason}): ` +
        `${detail}. A gate-1 case must not run a circuit the core was never given; classify it "not-yet" ` +
        "with this reason until W2c lands.",
    );
    this.name = "CoreSuiteEditNotDeliverableError";
    this.reason = reason;
  }
}

export interface CoreSuiteEngineOptions {
  /** Overrides which kinds this adapter is allowed to route to the core. Defaults to CORE_IMPLEMENTED_KINDS -- see the header comment for why NOT CORE_SUPPORTED_KINDS. */
  coreSupportedKinds?: ReadonlySet<string>;
}

/**
 * The `SimEngine` stand-in itself. Implements `EngineReadout` for the same
 * reason CoreEngine and classic SimEngine both do (test/core/engine.test.ts's
 * compile-time assertion) -- so a caller that only reads never needs to know
 * which of the three (classic, CoreEngine, this adapter) it is holding.
 */
export class SimEngine implements EngineReadout {
  private readonly _coreSupportedKinds: ReadonlySet<string>;
  private _runner: HeadlessRunner;
  /** Set by setIntegrationMethod(), applied on the NEXT step() -- see that method's own doc comment. */
  private _pendingIntegrationMethod: "be" | "trap" | undefined;
  /** The circuit object the suite passed to load(): classic re-reads it live every step, so a suite mutates it in place. */
  private _liveCircuit: SimCircuit | undefined;
  /** A deep copy of `_liveCircuit` as of the last load() or delivered step: what the core has actually been given. */
  private _delivered: SimCircuit | undefined;

  constructor(options?: CoreSuiteEngineOptions) {
    this._coreSupportedKinds = options?.coreSupportedKinds ?? CORE_IMPLEMENTED_KINDS;
    this._runner = this._freshRunner();
  }

  private _freshRunner(): HeadlessRunner {
    return new HeadlessRunner({ engine: "core", coreSupportedKinds: this._coreSupportedKinds });
  }

  /** Shared by load()/coldLoad(): drive the runner, then refuse a silent fallback. */
  private _loadOnCore(circuit: SimCircuit): void {
    this._runner.load(circuit);
    if (this._runner.activeEngine !== "core") {
      throw new CoreSuiteFellBackError(this._runner.engineFallbackReasons);
    }
    this._liveCircuit = circuit;
    this._delivered = structuredClone(circuit);
  }

  /**
   * Sends the suite's in-place edits since the last load() or step() to the
   * core. Live params (the switch family today) go through `setParams`, which
   * keeps all state; an edit with nothing to deliver is a no-op; anything else
   * throws, because dropping it would let the case run a circuit the core was
   * never given (see CoreSuiteEditNotDeliverableError).
   */
  private _deliverEdits(): void {
    const live = this._liveCircuit;
    const delivered = this._delivered;
    if (!live || !delivered) return;
    const diff = liveParamDiff(delivered, live);
    if (diff === null) {
      // Is V the only thing that blocks it? Then name the pending piece.
      const ifVWereLive = liveParamDiff(delivered, live, { voltageSourceLive: true });
      if (ifVWereLive !== null) {
        const ids = ifVWereLive.map((change) => change.componentId).join(", ");
        throw new CoreSuiteEditNotDeliverableError("live-v-pending", `voltage_source V changed on ${ids}`);
      }
      throw new CoreSuiteEditNotDeliverableError(
        "needs-carry",
        "a topology change or a param that is not live changed (liveParamDiff returned null)",
      );
    }
    if (diff.length > 0 && !this._coreEngine("setParams").setParams(diff)) {
      throw new CoreSuiteEditNotDeliverableError(
        "not-applied",
        `setParams skipped a change for ${diff.map((change) => change.componentId).join(", ")}`,
      );
    }
    this._delivered = structuredClone(live);
  }

  /** Classic `SimEngine.load()`: carries the facade clock forward (HeadlessRunner's own load() semantics -- see headless.ts). */
  load(circuit: SimCircuit): void {
    this._loadOnCore(circuit);
  }

  /**
   * Classic `SimEngine.coldLoad()` additionally resets `simTime` to 0. A
   * HeadlessRunner carries its facade clock forward across load() calls by
   * design (headless.ts's own doc comment), so "cold" here is implemented as
   * "start over with a brand-new runner" -- its very first load() always
   * begins at t=0 -- rather than as a special CoreEngine call. No case in
   * classification.json calls this yet; it exists for the mechanism's
   * documented shape (the W1 plan's own "maps load, coldLoad, step(h)"
   * description) and gets its own proof the day a suite that uses it is
   * added to the allowlist.
   */
  coldLoad(circuit: SimCircuit): void {
    this._runner = this._freshRunner();
    this._loadOnCore(circuit);
  }

  /**
   * Classic `SimEngine.step(h)` -> `CoreEngine.advance(simTime + h)`, via
   * HeadlessRunner's own core-mode run() (headless.ts's `_runCore`). Threads
   * any pending `setIntegrationMethod()` request through `run()`'s own
   * `integrationMethod` option every call -- cheap once applied (headless.ts's
   * `applyIntegrationMethod` no-ops when the method already matches) and the
   * ONLY call site this adapter has into that mechanism, since `load()`/
   * `coldLoad()` take no method argument of their own.
   *
   * A `step()` is not itself guarded by `_loadOnCore`'s anti-silent-fallback
   * check anywhere else in this file -- ordinarily nothing WOULD reload
   * mid-session after `load()`/`coldLoad()` already proved the core accepted
   * the circuit. But `applyIntegrationMethod` CAN reload (headless.ts: a
   * method switch at t=0 re-seeds through `load()` so the new method is baked
   * into the compiled program), and that reload can itself fall back to
   * classic. So this is the one `step()` needs its own fallback check too --
   * without it, a trap-mode circuit the core declines would silently keep
   * running on classic from here on, exactly the failure mode `load()`/
   * `coldLoad()` already refuse.
   */
  step(h: number): void {
    this._deliverEdits();
    this._runner.run({ durationS: h, integrationMethod: this._pendingIntegrationMethod });
    if (this._runner.activeEngine !== "core") {
      throw new CoreSuiteFellBackError(this._runner.engineFallbackReasons);
    }
  }

  /**
   * Classic `SimEngine.setIntegrationMethod(method)`. Classic applies this
   * IMMEDIATELY (it is a plain field write with a re-seed on the classic
   * engine object); `HeadlessRunner.applyIntegrationMethod` instead requires
   * an already-loaded circuit and is only reachable through `run()`'s own
   * `integrationMethod` option (headless.ts) -- there is no standalone
   * "set the method right now" entry point to call into directly, and a
   * suite file calling this BEFORE `load()` (trap-integration.test.ts's own
   * pattern: `engine.setIntegrationMethod(method); engine.load(circuit);`)
   * would have nothing loaded yet regardless. So this only records the
   * request; `step()` (above) is what actually applies it, on the runner's
   * own `run()` call, the same "in force before the load seed solve and the
   * first step's backward-Euler anchoring" instant headless.ts's own
   * `applyIntegrationMethod` doc comment requires (it throws if `simTime !==
   * 0`, so this is only ever legal before the first `step()` anyway --
   * exactly mirroring classic's "call it before stepping" contract, just
   * enforced one call later here).
   */
  setIntegrationMethod(method: "be" | "trap"): void {
    this._pendingIntegrationMethod = method;
  }

  // ---- EngineReadout -------------------------------------------------------

  get simTime(): number {
    return this._runner.readout.simTime;
  }

  get nets(): Net[] {
    return this._runner.readout.nets;
  }

  get lastConverged(): boolean {
    return this._runner.readout.lastConverged;
  }

  getNetV(): Record<string, number> {
    return this._runner.readout.getNetV();
  }

  getElementI(): Record<string, number> {
    return this._runner.readout.getElementI();
  }

  /**
   * Classic `SimEngine.getElementChannelI()`: per-channel currents for a
   * multi-channel device (rgb_led's 3 channels, bicolor_led's 2, a 7-segment
   * display's 8; resistor_array's 4 on classic only). `CoreEngine` has the
   * method (its channel slots, spec section 10.10) but `EngineReadout` does
   * not declare it, so this reads it off the active readout structurally: the
   * `CoreEngine` on the core, or classic `SimEngine` after a fallback (which
   * the no-silent-fallback check makes a caller bug anyway).
   *
   * This used to be a constant `{}`, provably right only while no
   * multi-channel kind was in `CORE_IMPLEMENTED_KINDS`. W1e added three, and
   * a stub would now make a case that reads a channel fail, and one that
   * guards on the record being present pass while checking nothing.
   */
  getElementChannelI(): Record<string, number[]> {
    const readout = this._runner.readout as EngineReadout & { getElementChannelI?: () => Record<string, number[]> };
    if (typeof readout.getElementChannelI !== "function") {
      throw new Error("core-suites adapter: the active readout has no getElementChannelI()");
    }
    return readout.getElementChannelI();
  }

  get digitalState(): Record<string, number> {
    return this._runner.readout.digitalState;
  }

  getFailures(): Record<string, SimFailure> {
    return this._runner.readout.getFailures();
  }

  getWarnings(): Record<string, SimWarning> {
    return this._runner.readout.getWarnings();
  }

  takeNewWarnings(): SimWarning[] {
    return this._runner.readout.takeNewWarnings();
  }

  getDisplayState(): Record<string, DisplayInfo> {
    return this._runner.readout.getDisplayState();
  }

  getNetIdForPin(componentId: string, pinId: string): string | undefined {
    return this._runner.readout.getNetIdForPin(componentId, pinId);
  }

  // ---- Device state (W1f): CoreEngine only, never EngineReadout ------------

  /**
   * Classic `SimEngine.getBatteryState(id)`, for battery-engine-physics.
   * `CoreEngine` has it and `EngineReadout` does not (core/README.md,
   * Readouts), so this asks the runner's `CoreEngine` itself.
   */
  getBatteryState(componentId: string): BatteryRuntimeReadout | undefined {
    return this._coreEngine("getBatteryState").getBatteryState(componentId);
  }

  /** Classic `SimEngine.getIcState(id)`, for wave5-regulators' `bench_psu` cases: `{ reg }` for a limited supply, as `getBatteryState` above. */
  getIcState(componentId: string): Record<string, number> | undefined {
    return this._coreEngine("getIcState").getIcState(componentId);
  }

  // ---- Thermal, failures and PTCs (W1g): CoreEngine only (W1g Q5) ---------

  /** Classic `SimEngine.getPtcTripped()`, for wave3-protection's `ptc_fuse` cases. */
  getPtcTripped(): ReadonlySet<string> {
    return this._coreEngine("getPtcTripped").getPtcTripped();
  }

  /** Classic `SimEngine.getThermalState(id)`, for thermal-engine-physics and thermal-cold-start-warnings. */
  getThermalState(componentId: string): ThermalRuntimeReadout | undefined {
    return this._coreEngine("getThermalState").getThermalState(componentId);
  }

  /** Classic `SimEngine.getPartTemperature(id)`, for thermal-engine-physics' identity-less case. */
  getPartTemperature(componentId: string): number | undefined {
    return this._coreEngine("getPartTemperature").getPartTemperature(componentId);
  }

  /**
   * Classic `SimEngine.takeEepromUpdates()`, for eeprom-write: component id to
   * a fresh image of every EEPROM written since the last call, then cleared.
   * The runner's `CoreEngine` has the same method and shape.
   */
  takeEepromUpdates(): Record<string, Uint8Array> {
    return this._coreEngine("takeEepromUpdates").takeEepromUpdates();
  }

  /**
   * Classic `SimEngine.resetFailures()`, for failure-states and
   * wave3-protection. The runner has no reset of its own: the call goes
   * straight to its `CoreEngine`, which re-solves at the current time
   * without advancing it (W1g point 13), so the runner's clock stays right.
   */
  resetFailures(): void {
    this._coreEngine("resetFailures").resetFailures();
  }

  /**
   * The runner's `CoreEngine`. Classic `SimEngine` has both device-state
   * methods too, so reading them structurally off the active readout, as
   * `getElementChannelI` does, would answer from classic after a fallback;
   * this refuses instead.
   */
  private _coreEngine(method: string): CoreEngine {
    const readout = this._runner.readout;
    if (!(readout instanceof CoreEngine)) {
      throw new Error(`core-suites adapter: ${method}() needs the core, but the runner's active engine is classic`);
    }
    return readout;
  }

  // ---- Classic solver diagnostics (no core meaning yet; see header) -------

  private get _solver(): SolverDiagnosticsSnapshot {
    return this._runner.snapshot().solver;
  }

  get lastIters(): number {
    return this._solver.lastIters;
  }

  get lastSolveUs(): number {
    return this._solver.lastSolveUs;
  }

  get lastMatrixSize(): number {
    return this._solver.lastMatrixSize;
  }

  get lastMatrixSingular(): boolean {
    return this._solver.lastMatrixSingular;
  }

  get lastMatrixIllConditioned(): boolean {
    return this._solver.lastMatrixIllConditioned;
  }

  get lastRelativeResidual(): number {
    return this._solver.lastRelativeResidual;
  }
}
