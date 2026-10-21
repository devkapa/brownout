/**
 * HeadlessRunner's `engine: "core"` option (src/host/headless.ts): routing
 * to brownout/core, falling back to classic, and the read-surface the two
 * modes share. See src/core/engine.test.ts for the core-vs-classic parity
 * checks themselves — this file is about the HOST's routing/fallback
 * behaviour, not the engine's numerics.
 */
import { afterEach, beforeAll, describe, expect, it, vi } from "vitest";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { HeadlessRunner } from "../../src/host/headless.js";
import { CoreEngine } from "../../src/core/engine.js";
import {
  coreCall,
  getWasmApi,
  initCore,
  __installCoreForTest,
  __resetCoreForTest,
  type CoreExports,
} from "../../src/core/wasm.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type SimComponent = SimCircuit["components"][number];
const part = (id: string, kind: string, pins: string[], params: Record<string, number> = {}): SimComponent => ({
  id,
  kind,
  pins: pins.map((pin) => ({ id: pin })),
  params,
});
const wire = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

const FIXTURES_CORE_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../fixtures/core");
function loadCoreFixture(name: string): SimCircuit {
  return JSON.parse(fs.readFileSync(path.join(FIXTURES_CORE_DIR, name), "utf8")) as SimCircuit;
}

const DIVIDER: SimCircuit = {
  components: [
    part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 10 }),
    part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
    part("r2", "resistor", ["a", "b"], { resistance: 1000 }),
  ],
  wires: [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "r2", "a"), wire("r2", "b", "vsrc", "neg")],
};

// A well-formed program the W0 kernel rejects at the ABI level: a loop of
// ideal sources (core/README.md's fixed-node rule #1).
const TOPOLOGY_UNSUPPORTED: SimCircuit = {
  components: [part("vs1", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("vs2", "voltage_source", ["pos", "neg"], { voltage: 9 })],
  wires: [wire("vs1", "pos", "vs2", "pos"), wire("vs1", "neg", "vs2", "neg")],
};

beforeAll(async () => {
  await initCore();
});

describe('engine: "core" — kind-support gate', () => {
  it("falls back to classic with unsupported-kind: reasons when coreSupportedKinds is the (empty, W0) default", () => {
    const runner = new HeadlessRunner({ engine: "core" });
    runner.load(DIVIDER);
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual(
      ["unsupported-kind:resistor", "unsupported-kind:voltage_source"].sort(),
    );
    // The fallback is a real classic load — everything still works.
    expect(runner.engine.getNetV()).toMatchObject({ n0: 10 });
  });

  it("lands on the core when coreSupportedKinds is widened (equivalence-harness / test use only)", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(DIVIDER);
    expect(runner.activeEngine).toBe("core");
    expect(runner.engineFallbackReasons).toEqual([]);
  });
});

describe('engine: "core" — topology fallback', () => {
  it("falls back to classic with a topology: reason when the W0 kernel rejects the circuit", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(TOPOLOGY_UNSUPPORTED);
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toHaveLength(1);
    expect(runner.engineFallbackReasons[0]).toMatch(/^topology:/);
    // The fallback still ran classic's own load — a loop of ideal sources is
    // singular there too (a genuine contradiction, 5 V forced equal to 9 V),
    // so classic reports it the way it reports any singular matrix rather
    // than throwing.
    expect(runner.engine.lastMatrixSingular).toBe(true);
    expect(runner.engine.lastConverged).toBe(false);
  });
});

describe('engine: "core" — not-initialised fallback (a genuinely fresh module)', () => {
  it("falls back with core-not-initialized when initCore() has never run", async () => {
    vi.resetModules();
    const { HeadlessRunner: FreshHeadlessRunner } = await import("../../src/host/headless.js");
    const { CORE_IMPLEMENTED_KINDS: FreshImplementedKinds } = await import("../../src/core/kinds.js");
    const runner = new FreshHeadlessRunner({ engine: "core", coreSupportedKinds: FreshImplementedKinds });
    runner.load(DIVIDER);
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual(["core-not-initialized"]);
  });
});

describe('engine: "core" — run(), snapshot(), readout, and the engine getter', () => {
  it("run() advances the core; snapshot's netV/elementI match a classic runner on the same circuit and duration", () => {
    const coreRunner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    coreRunner.load(DIVIDER);
    const result = coreRunner.run({ durationS: 1e-3 });
    expect(result.simTime).toBeCloseTo(1e-3, 15);
    expect(result.simulatedS).toBe(1e-3);
    expect(result.acceptedSteps).toBe(0); // no step loop in the core (W0)
    expect(result.errorControlled).toBe(false);

    const classicRunner = new HeadlessRunner(); // engine: "classic" (default)
    classicRunner.load(DIVIDER);
    classicRunner.run({ durationS: 1e-3 });

    const coreSnap = coreRunner.snapshot();
    const classicSnap = classicRunner.snapshot();
    expect(new Set(Object.keys(coreSnap.netV))).toEqual(new Set(Object.keys(classicSnap.netV)));
    for (const key of Object.keys(classicSnap.netV)) {
      expect(coreSnap.netV[key]).toBeCloseTo(classicSnap.netV[key]!, 6);
    }
    for (const key of Object.keys(classicSnap.elementI)) {
      expect(coreSnap.elementI[key]).toBeCloseTo(classicSnap.elementI[key]!, 6);
    }
    // Documented no-core-meaning-yet solver fields (see snapshot()'s doc comment).
    expect(coreSnap.solver).toEqual({
      lastIters: 0,
      lastConverged: true,
      lastSolveUs: 0,
      lastMatrixSize: 0,
      lastMatrixSingular: false,
      lastMatrixIllConditioned: false,
      lastRelativeResidual: 0,
    });
  });

  it("onSample fires exactly once, at the run's end, with h = durationS", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(DIVIDER);
    const samples: { simTime: number; h: number }[] = [];
    runner.run({ durationS: 2.5e-3, onSample: (s) => samples.push({ simTime: s.simTime, h: s.h }) });
    expect(samples).toHaveLength(1);
    expect(samples[0]!.h).toBe(2.5e-3);
    expect(samples[0]!.simTime).toBeCloseTo(2.5e-3, 15);
  });

  it("engine getter throws in core mode; readout works and matches getNetV/simTime", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(DIVIDER);
    expect(() => runner.engine).toThrow(/engine: "core"/);
    expect(runner.readout.simTime).toBe(runner.simTime);
    expect(runner.readout.getNetV()).toMatchObject({ n0: 10 });
    expect(runner.netIdFor("vsrc", "pos")).toBe(runner.readout.getNetIdForPin("vsrc", "pos"));
  });

  it("still enforces run()'s misuse checks in core mode (adaptive:false without fixedStepS)", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(DIVIDER);
    expect(() => runner.run({ durationS: 1e-3, adaptive: false })).toThrow(/fixedStepS/);
  });
});

describe('engine absent/"classic" — unchanged', () => {
  it("activeEngine is classic and there is never a fallback reason", () => {
    const runner = new HeadlessRunner();
    runner.load(DIVIDER);
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual([]);
    expect(runner.engine.getNetV()).toMatchObject({ n0: 10 });
    expect(runner.readout).toBe(runner.engine);
  });
});

// A well-formed program the core can compile but which uses a kind that is
// implemented (compiles) but not routed to the core by the (widened, test-
// only) coreSupportedKinds override used below — forces a live-edit fallback.
const WITH_UNSUPPORTED_KIND: SimCircuit = {
  components: [...DIVIDER.components, part("d1", "diode", ["a", "k"])],
  wires: DIVIDER.wires,
};

describe('engine: "core" — simTime carries across an engine switch (decision: time continuity across live-edit reloads)', () => {
  it("core -> classic -> core: simTime never resets or rewinds, matching a classic-only runner's own reload guarantee", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(DIVIDER);
    expect(runner.activeEngine).toBe("core");
    runner.run({ durationS: 1e-3 });
    expect(runner.simTime).toBeCloseTo(1e-3, 12);

    // A live edit adds a kind the core isn't routing: falls back to classic.
    // Before the fix, `runner.simTime` here read 0 (classic's own untouched
    // clock) instead of continuing from where the core left off.
    runner.load(WITH_UNSUPPORTED_KIND);
    expect(runner.activeEngine).toBe("classic");
    expect(runner.engineFallbackReasons).toEqual(["unsupported-kind:diode"]);
    expect(runner.simTime).toBeCloseTo(1e-3, 9);

    runner.run({ durationS: 1e-4 });
    expect(runner.simTime).toBeCloseTo(1.1e-3, 9);

    // The part is removed again: lands back on the core. Before the fix this
    // read whatever the CORE's own clock had last stood at (1e-3), silently
    // losing the 1e-4 s classic had just run.
    runner.load(DIVIDER);
    expect(runner.activeEngine).toBe("core");
    expect(runner.simTime).toBeCloseTo(1.1e-3, 9);
  });
});

describe('engine: "core" — run()\'s integrationMethod legality (decision: core-mode run() validates it exactly like classic)', () => {
  it("a mid-run method switch is rejected with classic's error text, even though the core landed successfully", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(DIVIDER);
    runner.run({ durationS: 1e-3 });
    expect(runner.activeEngine).toBe("core");

    // Before the fix, the core branch of run() returned before ever reaching
    // applyIntegrationMethod, so this silently succeeded instead of throwing.
    expect(() => runner.run({ durationS: 1e-3, integrationMethod: "trap" })).toThrow(
      /HeadlessRunner: cannot switch integrationMethod to "trap" at t=/,
    );
    expect(runner.simTime).toBeCloseTo(1e-3, 15); // the rejected call never ran
  });

  it("a legal t=0 integrationMethod request is recorded even in core mode, so a later fallback to classic honours it", () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(DIVIDER);
    runner.run({ durationS: 1e-3, integrationMethod: "trap" });
    expect(runner.activeEngine).toBe("core");

    // Live-edit to an unsupported kind: falls back to classic, carrying the
    // core's simTime over (non-zero from here on).
    runner.load(WITH_UNSUPPORTED_KIND);
    expect(runner.activeEngine).toBe("classic");

    // If the earlier "trap" request had NOT been recorded (the pre-fix
    // behaviour), _integrationMethod would still be the constructor default
    // "be", and requesting "be" again would be a no-op — no throw. Getting
    // the "cannot switch" error instead proves "trap" really is in force.
    expect(() => runner.run({ durationS: 1e-4, integrationMethod: "be" })).toThrow(
      /cannot switch integrationMethod to "be"/,
    );
  });
});

// w1a-fixes/ts.json finding 4 (HeadlessRunner half): before the fix,
// `_runCore` called `engine.advance(...)` with no try/catch, so a -4/-7
// result (which CoreEngine.advance now reports rather than throws) would
// have propagated a CoreLoadError straight out of run() — contradicting
// run()'s own documented contract ("Throws only on misuse; solver trouble is
// reported on the result").
describe('engine: "core" — a non-converged advance() is reported on the result, not thrown (decision: -4/-7 is non-convergence, not a load error)', () => {
  afterEach(async () => {
    __resetCoreForTest();
    await initCore();
  });

  function installNonConvergentCore(lastCompletedInstant: number): void {
    const buffer = new ArrayBuffer(1 << 16);
    const exports: CoreExports = {
      memory: { buffer },
      bo_abi_version: () => 2,
      bo_alloc: () => 0,
      bo_free: () => {},
      bo_engine_create: () => 1,
      bo_engine_destroy: () => {},
      bo_engine_load: () => 0,
      bo_engine_advance: () => -7, // delta-cycle livelock, every call
      bo_engine_time: () => lastCompletedInstant,
      bo_engine_node_count: () => 0,
      bo_engine_node_voltages: () => 0,
      bo_engine_device_count: () => 0,
      bo_engine_device_currents: () => 0,
      bo_engine_stat: () => 0,
      bo_error_ptr: () => 0,
      bo_error_len: () => 0,
      bo_engine_set_option: () => 0,
      bo_engine_next_event_time: () => Infinity,
      bo_engine_next_digital_due_time: () => Infinity,      bo_engine_digital_slot_count: () => 0,
      bo_engine_digital_slots: () => 0,
      bo_engine_channel_slot_count: () => 0,
      bo_engine_channel_slots: () => 0,
      bo_engine_trace_enable: () => 0,
      bo_engine_trace_ptr: () => 0,
      bo_engine_trace_len: () => 0,
      bo_engine_trace_clear: () => {},
      bo_engine_seed_memory: () => 0,
      bo_engine_take_eeprom_updates: () => 0,
      bo_engine_eeprom_updates_ptr: () => 0,
      bo_engine_device_state: () => 0,
      bo_engine_device_state_ptr: () => 0,
      bo_engine_reset_failures: () => 0,
      bo_engine_records: () => 0,
      bo_engine_records_ptr: () => 0,
      bo_engine_set_params: () => 0,
    };
    __installCoreForTest(exports);
  }

  it("run() does not throw; the result reports hitMinStep and how far time actually got, and lastConverged is false", () => {
    installNonConvergentCore(4e-4);
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(DIVIDER);
    expect(runner.activeEngine).toBe("core");

    let result: ReturnType<typeof runner.run> | undefined;
    expect(() => {
      result = runner.run({ durationS: 1e-3 });
    }).not.toThrow();

    expect(result!.hitMinStep).toBe(true);
    expect(result!.simulatedS).toBeCloseTo(4e-4, 15); // short of the requested 1e-3
    expect(result!.simTime).toBeCloseTo(4e-4, 15);
    expect(runner.readout.lastConverged).toBe(false);
  });
});

describe('engine: "core" — trap recovery (decision: HeadlessRunner rebuilds a stale CoreEngine after re-init instead of throwing forever)', () => {
  it("after a trap poisons the instance, the fallback reason is core-poisoned (not core-not-initialized), and the SAME runner lands back on the core once initCore() succeeds again", async () => {
    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(DIVIDER);
    expect(runner.activeEngine).toBe("core");

    // A synthetic trap — the public ABI has no supported way to provoke a
    // real Rust panic (see wasm.test.ts's own poison-flow tests), so this is
    // the seam core/README.md's "Traps" section exists to make testable.
    expect(() =>
      coreCall(() => {
        throw new (getWasmApi().RuntimeError)("synthetic trap for headless recovery test");
      }),
    ).toThrow();

    runner.load(DIVIDER);
    expect(runner.activeEngine).toBe("classic");
    // Not "core-not-initialized": the core WAS initialised and then poisoned,
    // which calls for a different host response (re-init, not first init).
    expect(runner.engineFallbackReasons).toEqual(["core-poisoned"]);

    await initCore(); // the documented recovery path

    // Before the fix, the runner's cached CoreEngine belonged to the dead
    // generation and every load() from here on threw CoreStaleInstanceError,
    // with no classic fallback, forever. It must instead rebuild the engine
    // and land back on the core exactly like a brand new runner would.
    runner.load(DIVIDER);
    expect(runner.activeEngine).toBe("core");
    expect(runner.engineFallbackReasons).toEqual([]);
    expect(runner.readout.getNetV()).toMatchObject({ n0: 10 });
  });
});

// w1b-review.md finding 7: `_tryLoadCore` never passed `integrationMethod`
// through to `CoreEngine.load()`, and `applyIntegrationMethod` returned
// immediately in core mode on the theory that "the core has no dynamics to
// seed" — true in W0/W1a, false once W1b gave capacitors and stepped
// partitions to the core (the program's INTEGRATION option now genuinely
// picks BE vs. trapezoidal). A core-mode runner constructed (or switched, at
// t=0) with integrationMethod "trap" silently ran BE instead.
describe('engine: "core" — integrationMethod reaches the core (W1b gave it real dynamics to seed)', () => {
  const TWO_CAP_LADDER = loadCoreFixture("14-two-capacitor-ladder.sim.json");

  it('a core-mode runner constructed with integrationMethod "trap" matches a CoreEngine loaded directly with trap, and differs from a BE load', () => {
    const beDirect = new CoreEngine();
    beDirect.load(TWO_CAP_LADDER, CORE_IMPLEMENTED_KINDS, { integrationMethod: "be" });
    beDirect.advance(1e-3);

    const trapDirect = new CoreEngine();
    trapDirect.load(TWO_CAP_LADDER, CORE_IMPLEMENTED_KINDS, { integrationMethod: "trap" });
    trapDirect.advance(1e-3);

    // Sanity: BE and trap must actually disagree on this circuit, or the
    // assertions below couldn't tell "ignored" from "honoured".
    expect(trapDirect.getNetV().n2).not.toBeCloseTo(beDirect.getNetV().n2!, 6);

    const runner = new HeadlessRunner({
      engine: "core",
      coreSupportedKinds: CORE_IMPLEMENTED_KINDS,
      integrationMethod: "trap",
    });
    runner.load(TWO_CAP_LADDER);
    expect(runner.activeEngine).toBe("core");
    runner.run({ durationS: 1e-3 });

    // Before the fix, _tryLoadCore's load() call never carried
    // integrationMethod, so the compiled program always used the INTEGRATION
    // default ("be") regardless of what the runner was constructed with —
    // this would have matched beDirect's n2, bit for bit, instead of trap's.
    expect(runner.readout.getNetV().n2).toBe(trapDirect.getNetV().n2);
    expect(runner.readout.getNetV().n2).not.toBe(beDirect.getNetV().n2);
  });

  it("switching BE to trap at t=0 in core mode takes effect (reloads the core program under the new method)", () => {
    const trapDirect = new CoreEngine();
    trapDirect.load(TWO_CAP_LADDER, CORE_IMPLEMENTED_KINDS, { integrationMethod: "trap" });
    trapDirect.advance(1e-3);

    const runner = new HeadlessRunner({ engine: "core", coreSupportedKinds: CORE_IMPLEMENTED_KINDS }); // default "be"
    runner.load(TWO_CAP_LADDER);
    expect(runner.activeEngine).toBe("core");

    // Before the fix, applyIntegrationMethod's `if (this._activeEngine ===
    // "core") return;` recorded the request but never reloaded the program,
    // so this run still solved under "be".
    runner.run({ durationS: 1e-3, integrationMethod: "trap" });
    expect(runner.activeEngine).toBe("core"); // the reload must not itself trigger a classic fallback
    expect(runner.readout.getNetV().n2).toBe(trapDirect.getNetV().n2);
  });

  it("the classic path is unchanged: a runner that falls back to classic (capacitor is not yet in the accuracy-cleared CORE_SUPPORTED_KINDS) still re-seeds under the requested method exactly as before this fix", () => {
    const runner = new HeadlessRunner({ engine: "core" }); // default coreSupportedKinds: CORE_SUPPORTED_KINDS, still empty in W0
    runner.load(TWO_CAP_LADDER);
    expect(runner.activeEngine).toBe("classic"); // capacitor isn't accuracy-cleared yet — always falls back
    runner.run({ durationS: 1e-3, integrationMethod: "trap" });

    const classicDirect = new HeadlessRunner({ integrationMethod: "trap" });
    classicDirect.load(TWO_CAP_LADDER);
    classicDirect.run({ durationS: 1e-3 });

    expect(runner.readout.getNetV().n2).toBe(classicDirect.readout.getNetV().n2);
  });
});
