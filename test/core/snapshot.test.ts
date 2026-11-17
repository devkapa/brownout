/**
 * W2a: `CoreEngine.saveState()` / `restoreState()` (core/README.md,
 * "Snapshots"). Every synthetic fixture the core loads runs to a stop,
 * snapshots, runs on recording every readout at every stop, then restores
 * twice and must publish the same bits each time; the stats read on rather
 * than going back. Then the snapshot's lifetime: a load retires it,
 * `dispose()` frees it, and a forgotten one is freed when collected.
 */

import v8 from "node:v8";
import vm from "node:vm";
import { beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../src/core/engine.js";
import { CoreUnsupportedError } from "../../src/core/errors.js";
import { initCore } from "../../src/core/wasm.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import type { StimulusPhase } from "./equivalence/stimuli.js";
import { loadSyntheticCorpus, type LoadedSyntheticEntry } from "./equivalence/synthetic-fixtures.js";

beforeAll(async () => {
  await initCore();
});

/** Every number as its float64 bits, so NaN, -0 and the last ulp all count. */
function bits(value: unknown): string {
  const view = new DataView(new ArrayBuffer(8));
  return JSON.stringify(value, (_key, v: unknown) => {
    if (typeof v === "number") {
      view.setFloat64(0, v);
      return `${view.getUint32(0).toString(16)}:${view.getUint32(4).toString(16)}`;
    }
    if (v instanceof Uint8Array) return Array.from(v);
    if (v instanceof Set) return [...v].sort();
    return v;
  });
}

/** Everything a stop publishes. The queues (new warnings, latched failures, EEPROM updates) are drained, as a host drains them, at the same stops in every run. */
function seen(engine: CoreEngine): string {
  return bits({
    t: engine.simTime,
    converged: engine.lastConverged,
    netV: engine.getNetV(),
    elementI: engine.getElementI(),
    digital: engine.digitalState,
    channels: engine.getElementChannelI(),
    failures: engine.getFailures(),
    warnings: engine.getWarnings(),
    newWarnings: engine.takeNewWarnings(),
    latched: engine.takeLatchedFailures(),
    ptc: engine.getPtcTripped(),
    eeprom: engine.takeEepromUpdates(),
    nextEvent: engine.nextEventTime(),
    nextDigital: engine.nextDigitalDueTime(),
  });
}

const STAT_COUNT = 28;

/** One step of a fixture's run: the phase's `set` (if any) applied at its start, then an advance. */
interface Step {
  readonly t: number;
  readonly set?: StimulusPhase["set"];
}

/** A fixture's stops: each phase in eight advances, or the plain duration in 24. */
function stepsOf({ entry }: LoadedSyntheticEntry): Step[] {
  const steps: Step[] = [];
  if (entry.phases && entry.phases.length > 0) {
    let t = 0;
    entry.phases.forEach((phase, index) => {
      for (let k = 1; k <= 8; k++) {
        steps.push({ t: t + (phase.durationS * k) / 8, ...(k === 1 && index > 0 && phase.set ? { set: phase.set } : {}) });
      }
      t += phase.durationS;
    });
    return steps;
  }
  const duration = entry.durationS ?? 10e-3;
  for (let k = 1; k <= 24; k++) steps.push({ t: (duration * k) / 24 });
  return steps;
}

/** Phase 0's overrides folded into the circuit, as the stimulus replay loads it. */
function withFirstPhase({ entry, circuit }: LoadedSyntheticEntry): SimCircuit {
  const set = entry.phases?.[0]?.set;
  if (!set || set.length === 0) return circuit;
  return {
    ...circuit,
    components: circuit.components.map((c) => {
      const override = set.find((o) => o.id === c.id);
      return override ? { ...c, params: { ...c.params, ...override.params } } : c;
    }),
  };
}

function run(engine: CoreEngine, steps: readonly Step[]): string[] {
  return steps.map((step) => {
    if (step.set) engine.setParams(step.set.map((o) => ({ componentId: o.id, params: o.params })));
    engine.advance(step.t);
    return seen(engine);
  });
}

describe("CoreEngine snapshots: a restored run is the same run, bit for bit", () => {
  it("on every synthetic fixture the core loads, snapshotting a third of the way in", () => {
    let ran = 0;
    for (const loaded of loadSyntheticCorpus()) {
      const engine = new CoreEngine();
      try {
        engine.load(withFirstPhase(loaded));
      } catch (err) {
        if (err instanceof CoreUnsupportedError) continue;
        throw err;
      }
      const steps = stepsOf(loaded);
      const k = Math.floor(steps.length / 3);
      run(engine, steps.slice(0, k));
      const snapshot = engine.saveState();
      const atSnapshot = seen(engine);
      const first = run(engine, steps.slice(k));
      for (let again = 0; again < 2; again++) {
        const stats = engine.statCounters(STAT_COUNT);
        engine.restoreState(snapshot);
        expect(engine.statCounters(STAT_COUNT), `${loaded.entry.name}: the stats are not restored`).toEqual(stats);
        expect(seen(engine), `${loaded.entry.name}: the snapshot's readouts`).toBe(atSnapshot);
        expect(run(engine, steps.slice(k)), `${loaded.entry.name}: restore ${again}`).toEqual(first);
      }
      snapshot.dispose();
      engine.dispose();
      ran++;
    }
    expect(ran).toBeGreaterThanOrEqual(35);
  });

  it("brings back a 28C16's pending write: the restored run completes it and reports the image again (fixture 29)", () => {
    const loaded = loadSyntheticCorpus().find((e) => e.entry.name.startsWith("29-"))!;
    const engine = new CoreEngine();
    engine.load(withFirstPhase(loaded));
    const steps = stepsOf(loaded);
    // Fixture 29 latches its byte in phase 1; the write cycle (0.5 ms) is
    // still running at the end of phase 2.
    const k = 16;
    run(engine, steps.slice(0, k));
    const snapshot = engine.saveState();
    const updates = (rows: string[]) => rows.filter((row) => !row.includes('"eeprom":{}')).length;
    const first = run(engine, steps.slice(k));
    expect(updates(first), "the write completes after the snapshot and reports its image").toBeGreaterThan(0);
    engine.restoreState(snapshot);
    expect(run(engine, steps.slice(k))).toEqual(first);
    snapshot.dispose();
    engine.dispose();
  });
});

describe("CoreEngine snapshots: lifetime", () => {
  function engineOn29(): CoreEngine {
    const loaded = loadSyntheticCorpus().find((e) => e.entry.name.startsWith("29-"))!;
    const engine = new CoreEngine();
    engine.load(withFirstPhase(loaded));
    return engine;
  }

  it("a load retires every snapshot taken before it, and the table empties", () => {
    const engine = engineOn29();
    const snapshot = engine.saveState();
    expect(engine.snapshotBytes()).toBeGreaterThan(2048);
    engine.load(loadSyntheticCorpus().find((e) => e.entry.name.startsWith("29-"))!.circuit);
    expect(engine.snapshotBytes()).toBe(0);
    expect(() => engine.restoreState(snapshot)).toThrow(/before the last load/);
    snapshot.dispose();
    engine.dispose();
  });

  it("a snapshot restores only on its own engine, and not once disposed; a second dispose is a no-op", () => {
    const a = engineOn29();
    const b = engineOn29();
    const snapshot = a.saveState();
    expect(() => b.restoreState(snapshot)).toThrow(/another engine/);
    expect(snapshot.disposed).toBe(false);
    snapshot.dispose();
    expect(snapshot.disposed).toBe(true);
    expect(a.snapshotBytes()).toBe(0);
    expect(() => snapshot.dispose()).not.toThrow();
    expect(() => a.restoreState(snapshot)).toThrow(/disposed/);
    a.dispose();
    b.dispose();
  });

  it("disposing the engine first leaves the snapshot's dispose a quiet no-op", () => {
    const engine = engineOn29();
    const snapshot = engine.saveState();
    engine.dispose();
    expect(() => snapshot.dispose()).not.toThrow();
  });

  it("1,000 snapshots dropped without dispose() are freed once collected", async () => {
    v8.setFlagsFromString("--expose-gc");
    const gc = vm.runInNewContext("gc") as () => void;
    const engine = engineOn29();
    let held = 0;
    (() => {
      for (let i = 0; i < 1000; i++) engine.saveState();
      held = engine.snapshotBytes();
    })();
    expect(held).toBeGreaterThan(1000 * 1024);
    for (let round = 0; round < 50 && engine.snapshotBytes() > 0; round++) {
      gc();
      await new Promise((resolve) => setTimeout(resolve, 10));
    }
    expect(engine.snapshotBytes()).toBe(0);
    engine.dispose();
  });
});
