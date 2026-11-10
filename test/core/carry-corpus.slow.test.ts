/**
 * W2c: the carrying reload over the corpus (core/README.md, "Carry"; the W2c
 * plan's test 3). Every corpus circuit the core loads (brownout's fixtures,
 * the synthetic ones, and the external corpus when `BROWNOUT_CORE_CORPUS`
 * names one) is reloaded about a quarter of the way through its run:
 *
 * - **unchanged**: the core reloaded runs on as the core never reloaded, within
 *   the harness's tolerance (W0 for volts) and with identical digital state;
 * - **one param edited** (the first resistor's, else capacitor's, value up 10 %)
 *   and **one part added** (1 MOhm across the first two-pin part): the core's
 *   carrying reload against classic's (`SimEngine.load`, SE:2807-3200),
 *   wherever the two engines agree on the circuit run without a reload, which
 *   is the baseline the edit is measured from.
 */
import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { HeadlessRunner } from "../../src/host/headless.js";
import { CoreEngine } from "../../src/core/engine.js";
import { CoreUnsupportedError } from "../../src/core/errors.js";
import { initCore } from "../../src/core/wasm.js";
import { getPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";
import { loadBrownoutFixtureCorpus } from "./equivalence/brownout-fixtures.js";
import { loadSyntheticCorpus } from "./equivalence/synthetic-fixtures.js";
import { loadExternalCorpus } from "./equivalence/external-corpus.js";
import { compareSamples } from "./equivalence/compare.js";
import { DEFAULT_DURATION_S } from "./equivalence/sample.js";
import type { SampleReadout } from "./equivalence/types.js";

type Edit = (c: SimCircuit) => SimCircuit | null;

/**
 * The schedule: the reload about a quarter of the way in, then readings just
 * after it and on to D. Not at D/4 itself: fixtures put their events on round
 * fractions of D, and a reload at the very instant a regime changes (47's
 * capacitor reaching the setpoint at D/4) reads the seed solve's own
 * artifact on classic (its 1 ps companion), not the carry.
 */
function schedule(d: number): { reloadAt: number; times: number[] } {
  const r = 0.2537 * d;
  return { reloadAt: r, times: [d / 8, r, r + d / 1000, r + d / 50, d / 2, (3 * d) / 4, d] };
}

function readCore(e: CoreEngine): SampleReadout {
  return { simTime: e.simTime, netV: { ...e.getNetV() }, elementI: { ...e.getElementI() }, digitalState: { ...e.digitalState }, failures: e.getFailures(), warnings: e.getWarnings() };
}

/** The core's readings, reloading onto `next` at `reloadAt` (none: one run). The reading at `reloadAt` is the reload's. */
function coreRun(circuit: SimCircuit, next: SimCircuit | null, reloadAt: number, times: readonly number[]): SampleReadout[] {
  const e = new CoreEngine();
  try {
    e.load(structuredClone(circuit));
    const out: SampleReadout[] = [];
    for (const t of times) {
      e.advance(t);
      if (next && t === reloadAt) e.load(structuredClone(next));
      out.push(readCore(e));
    }
    return out;
  } finally {
    e.dispose();
  }
}

/** Classic's readings, the same way (`SimEngine.load` carries). */
function classicRun(circuit: SimCircuit, next: SimCircuit | null, reloadAt: number, times: readonly number[]): SampleReadout[] {
  const r = new HeadlessRunner();
  r.load(structuredClone(circuit));
  const out: SampleReadout[] = [];
  let prev = 0;
  for (const t of times) {
    const res = r.run({ durationS: t - prev, maxSteps: 2_000_000 });
    if (res.hitMinStep || res.hitStepCap) throw new Error(`classic stopped short of ${t}`);
    prev = t;
    if (next && t === reloadAt) r.load(structuredClone(next));
    const s = r.snapshot();
    out.push({ simTime: s.simTime, netV: s.netV, elementI: s.elementI, digitalState: s.digitalState, failures: s.failures, warnings: s.warnings });
  }
  return out;
}

const paramEdited: Edit = (c) => {
  const out = structuredClone(c);
  const r = out.components.find((k) => k.kind === "resistor" && Number.isFinite(Number(k.params.resistance)));
  if (r) {
    r.params.resistance = Number(r.params.resistance) * 1.1;
    return out;
  }
  const cap = out.components.find((k) => k.kind === "capacitor" && Number.isFinite(Number(k.params.capacitance)));
  if (cap) {
    cap.params.capacitance = Number(cap.params.capacitance) * 1.1;
    return out;
  }
  return null;
};

const partAdded: Edit = (c) => {
  const out = structuredClone(c);
  const host = out.components.find((k) => k.pins.length === 2 && out.wires.some((w) => w.from_component === k.id || w.to_component === k.id));
  if (!host) return null;
  out.components.push({ id: "zz_carry_r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1e6 } } as SimCircuit["components"][number]);
  out.wires.push(
    { from_component: "zz_carry_r", from_pin: "a", to_component: host.id, to_pin: host.pins[0]!.id },
    { from_component: "zz_carry_r", from_pin: "b", to_component: host.id, to_pin: host.pins[1]!.id },
  );
  return out;
};

function corpus(): { id: string; circuit: SimCircuit; d: number }[] {
  const out: { id: string; circuit: SimCircuit; d: number }[] = [];
  for (const { entry, circuit } of loadBrownoutFixtureCorpus()) out.push({ id: entry.id, circuit, d: entry.durationS ?? DEFAULT_DURATION_S });
  for (const { entry, circuit } of loadSyntheticCorpus()) out.push({ id: entry.id, circuit, d: entry.durationS ?? DEFAULT_DURATION_S });
  return out;
}

function firstMismatch(a: readonly SampleReadout[], b: readonly SampleReadout[]): string | null {
  const m = compareSamples(a, b).mismatches[0];
  return m ? `${m.category} ${m.id} at ${m.sampleTime}: ${String(m.classicValue)} against ${String(m.coreValue)}` : null;
}

function sweep(entries: { id: string; circuit: SimCircuit; d: number }[]): { loaded: number; failures: string[]; edited: number } {
  const failures: string[] = [];
  let loaded = 0;
  let edited = 0;
  for (const { id, circuit, d } of entries) {
    const { reloadAt, times } = schedule(d);
    let plain: SampleReadout[];
    try {
      plain = coreRun(circuit, null, reloadAt, times);
    } catch (err) {
      if (err instanceof CoreUnsupportedError) continue;
      failures.push(`${id}: the core's plain run threw ${String(err)}`);
      continue;
    }
    loaded++;
    // At the reload's own instant a settled capacitor's current is its seed
    // solve's KCL residual (35-led-rc-stepped: 1.9 nA where the run reads
    // 1e-14, past the 1 nA tolerance), not a carried quantity: that one
    // reading leaves the capacitors' currents out.
    const atReload = times.indexOf(reloadAt);
    const capIds = circuit.components.filter((k) => k.kind === "capacitor").map((k) => k.id);
    const withoutCaps = (samples: SampleReadout[]) =>
      samples.map((s, i) => (i === atReload ? { ...s, elementI: Object.fromEntries(Object.entries(s.elementI).filter(([k]) => !capIds.includes(k))) } : s));
    const same = firstMismatch(withoutCaps(plain), withoutCaps(coreRun(circuit, circuit, reloadAt, times)));
    if (same) failures.push(`${id} unchanged: ${same}`);

    let baseline: string | null;
    try {
      baseline = firstMismatch(classicRun(circuit, null, reloadAt, times), plain);
    } catch {
      continue;
    }
    if (baseline) continue;
    for (const [what, edit] of [["param edited", paramEdited], ["part added", partAdded]] as const) {
      const next = edit(circuit);
      if (!next) continue;
      let core: SampleReadout[];
      let classic: SampleReadout[];
      try {
        core = coreRun(circuit, next, reloadAt, times);
        classic = classicRun(circuit, next, reloadAt, times);
      } catch (err) {
        if (err instanceof CoreUnsupportedError) continue;
        failures.push(`${id} ${what}: threw ${String(err)}`);
        continue;
      }
      edited++;
      const diff = firstMismatch(classic, core);
      if (diff) failures.push(`${id} ${what}: ${diff}`);
    }
  }
  return { loaded, failures, edited };
}

describe("W2c: a carrying reload over the corpus", () => {
  it("brownout's fixtures and the synthetic ones", async () => {
    await initCore();
    const { loaded, failures, edited } = sweep(corpus());
    console.log(`carry corpus (local): ${loaded} loaded, ${edited} edited reloads against classic, ${failures.length} failure(s)`);
    expect(loaded).toBeGreaterThan(20);
    expect(failures).toEqual([]);
  });

  it.skipIf(!process.env.BROWNOUT_CORE_CORPUS)("the external corpus (BROWNOUT_CORE_CORPUS)", async () => {
    await initCore();
    const entries: { id: string; circuit: SimCircuit; d: number }[] = [];
    const previous = getPartLibrary();
    try {
      for (const dir of process.env.BROWNOUT_CORE_CORPUS!.split(":")) {
        const c = loadExternalCorpus(dir);
        setPartLibrary([...getPartLibrary(), ...c.parts]);
        for (const { entry, circuit } of c.entries) entries.push({ id: entry.id, circuit, d: entry.durationS ?? DEFAULT_DURATION_S });
      }
      const { loaded, failures, edited } = sweep(entries);
      console.log(`carry corpus (external): ${loaded} loaded, ${edited} edited reloads against classic, ${failures.length} failure(s)`);
      expect(failures).toEqual([]);
    } finally {
      setPartLibrary([...previous]);
    }
  });
});
