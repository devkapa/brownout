/**
 * HeadlessRunner's `engine: "core"` option (src/host/headless.ts): routing
 * to brownout/core, falling back to classic, and the read-surface the two
 * modes share. See src/core/engine.test.ts for the core-vs-classic parity
 * checks themselves — this file is about the HOST's routing/fallback
 * behaviour, not the engine's numerics.
 */
import { beforeAll, describe, expect, it, vi } from "vitest";
import { HeadlessRunner } from "../../src/host/headless.js";
import { coreCall, getWasmApi, initCore } from "../../src/core/wasm.js";
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
  components: [...DIVIDER.components, part("led1", "led", ["a", "k"])],
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
    expect(runner.engineFallbackReasons).toEqual(["unsupported-kind:led"]);
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
