/**
 * Ruling R122 (a): the hand-judged mode (types.ts's `HandBoundSpec`).
 *
 * An entry carrying the marker is judged here instead of by the classic-
 * against-core comparison: the core's 555 edges, period, duty, pulse widths
 * and final count against the hand model (hand-555.ts) within the marker's
 * bound. Classic runs the same stops and its distance from the same hand
 * values is written to the entry's divergence rows, with numbers; it is never
 * asserted. Both engines are driven at the app's cadence (1/60 s) with a stop
 * on every phase boundary; the other cadences are test/core/battery-555-
 * corpus.test.ts's.
 *
 * Narrow by construction: `assertHandBoundApplies` refuses a circuit that has
 * no 555 on a battery pack's rail, and nothing here is reached by an entry
 * without the marker (run-entry.ts's first line of `runCorpusEntry`).
 *
 * The rail capacitor's published current is NOT judged and NOT excused: the
 * core reads exactly 0 for a capacitor directly across a cell's rail
 * (`publish_fixed_pair_capacitors`, core/src/solve/mod.rs; found by CORPUS,
 * fix in progress). It is listed in the report as a known failing reading.
 */
import { CoreEngine } from "../../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { buildCoreNodes, corePinKey } from "../../../src/core/compile/topology.js";
import { HeadlessRunner } from "../../../src/host/headless.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { handEdges, measuredQuantities, type HandEdges } from "./hand-555.js";
import type { CorpusEntry, EntryResult, HandBoundQuantities, HandBoundReport, HandBoundSpec, KnownDivergenceHit, KnownFailingReading, Mismatch } from "./types.js";

export const CLASSIC_555_ON_CELL_RAIL_ID = "classic-555-on-cell-rail";
const DECISION_REF =
  "R122 (a): where classic is shown to be the less accurate engine for a circuit class, the corpus judges the core against " +
  "hand values and records classic's distance (core/README.md, 'The 555 period on a moving rail')";
/** Classic's load-seed pulse (OUT low from load until its first solve settles, 76 us on these boards; README NE555 point 9) is not an edge of the circuit. */
const CLASSIC_SEED_S = 2e-4;
/** The app's frame, the cadence both engines are driven at here. */
const HOST_S = 1 / 60;

/** Refuses the marker on a circuit without a 555 whose pin 8 sits on a `battery_pack`'s positive node. */
export function assertHandBoundApplies(circuit: SimCircuit, spec: HandBoundSpec): void {
  const fail = (why: string): never => {
    throw new Error(`handBound (${spec.circuitClass}): refused, ${why}. The marker excuses a 555 on a cell's rail and nothing else.`);
  };
  if (spec.circuitClass !== "555 on a cell's rail") fail(`unknown circuit class "${String(spec.circuitClass)}"`);
  if (!spec.justification || spec.justification.trim().length === 0) fail("no justification for the bound");
  if (!(spec.bound.relative >= 0) || !(spec.bound.absoluteS >= 0)) fail("no bound");
  const chip = circuit.components.find((c) => c.id === spec.component);
  if (!chip || chip.kind !== "ne555") fail(`"${spec.component}" is not an ne555 in this circuit`);
  const built = buildCoreNodes(circuit);
  const supply = built.pinNode.get(corePinKey(spec.component, "8"));
  const fedByCell = circuit.components.some(
    (c) => c.kind === "battery_pack" && supply !== undefined && supply !== 0 && built.pinNode.get(corePinKey(c.id, "pos")) === supply,
  );
  if (!fedByCell) fail(`"${spec.component}" pin 8 is not on a battery_pack's positive node`);
  for (const id of [spec.counter, spec.railCapacitor]) {
    if (id !== undefined && !circuit.components.some((c) => c.id === id)) fail(`"${id}" is not in the circuit`);
  }
}

interface Run {
  up: number[];
  down: number[];
  count?: number;
  capCurrent: number[];
}

/** The stops: every frame, every phase boundary, and two early ones where the rail capacitor's reading is taken. */
function stopsFor(endS: number, boundaries: readonly number[]): number[] {
  const set = new Set<number>([1e-4, 1e-3, endS, ...boundaries]);
  for (let k = 1; k * HOST_S < endS - 1e-12; k++) set.add(k * HOST_S);
  return [...set].filter((t) => t > 0 && t <= endS + 1e-12).sort((a, b) => a - b);
}

function editsOf(entry: CorpusEntry): { t: number; id: string; params: Record<string, number | string> }[] {
  const out: { t: number; id: string; params: Record<string, number | string> }[] = [];
  let t = 0;
  for (const phase of entry.phases ?? []) {
    for (const s of phase.set ?? []) out.push({ t, id: s.id, params: { ...s.params } });
    t += phase.durationS;
  }
  return out;
}

function coreRun(circuit: SimCircuit, spec: HandBoundSpec, endS: number, stops: readonly number[], edits: ReturnType<typeof editsOf>): Run {
  const engine = new CoreEngine();
  try {
    engine.load(circuit, CORE_IMPLEMENTED_KINDS);
    engine.enableTrace(true);
    engine.clearTrace();
    const run: Run = { up: [], down: [], capCurrent: [] };
    const pending = edits.filter((e) => e.t > 0);
    for (const e of edits.filter((x) => x.t <= 0)) engine.setParams([{ componentId: e.id, params: e.params }]);
    for (const t of stops) {
      engine.advance(t);
      for (const ev of engine.readTrace()) {
        if (ev.key !== spec.component || ev.t <= 0) continue;
        (ev.value === 1 ? run.up : run.down).push(ev.t);
      }
      engine.clearTrace();
      if (spec.railCapacitor && t <= 1e-3 + 1e-12) run.capCurrent.push(engine.getElementI()[spec.railCapacitor] ?? 0);
      while (pending.length > 0 && pending[0]!.t <= t + 1e-12) {
        const e = pending.shift()!;
        engine.setParams([{ componentId: e.id, params: e.params }]);
      }
    }
    if (spec.counter) run.count = engine.digitalState[spec.counter];
    return run;
  } finally {
    engine.dispose();
  }
}

function classicRun(circuit: SimCircuit, spec: HandBoundSpec, endS: number, stops: readonly number[], edits: ReturnType<typeof editsOf>): Run {
  let current = circuit;
  const runner = new HeadlessRunner();
  const apply = (e: (typeof edits)[number]): void => {
    current = structuredClone(current);
    Object.assign(current.components.find((c) => c.id === e.id)!.params, e.params);
    runner.load(current);
  };
  runner.load(current);
  const run: Run = { up: [], down: [], capCurrent: [] };
  let previous = runner.engine.digitalState[spec.component];
  const pending = edits.filter((e) => e.t > 0);
  for (const e of edits.filter((x) => x.t <= 0)) apply(e);
  for (const t of stops) {
    const span = t - runner.simTime;
    if (span > 1e-15) {
      runner.run({
        durationS: span,
        onSample: () => {
          const v = runner.engine.digitalState[spec.component];
          if (v !== undefined && v !== previous) {
            (v === 1 ? run.up : run.down).push(runner.engine.simTime);
            previous = v;
          }
        },
      });
    }
    if (spec.railCapacitor && t <= 1e-3 + 1e-12) run.capCurrent.push(runner.snapshot().elementI[spec.railCapacitor] ?? 0);
    while (pending.length > 0 && pending[0]!.t <= t + 1e-12) apply(pending.shift()!);
  }
  if (spec.counter) run.count = runner.engine.digitalState[spec.counter];
  run.up = run.up.filter((t) => t > CLASSIC_SEED_S);
  run.down = run.down.filter((t) => t > CLASSIC_SEED_S);
  return run;
}

const allowed = (spec: HandBoundSpec, hand: number): number => spec.bound.relative * Math.abs(hand) + spec.bound.absoluteS;

function quantitiesOf(run: Run, spec: HandBoundSpec, hand: HandEdges): HandBoundQuantities {
  const worst = (a: readonly number[], b: readonly number[]): number => Math.max(0, ...a.slice(0, b.length).map((t, i) => Math.abs(t - b[i]!)));
  const m = measuredQuantities(run.up, run.down, spec.model);
  return {
    rises: run.up.length,
    falls: run.down.length,
    worstEdgeS: Math.max(worst(run.up, hand.up), worst(run.down, hand.down)),
    ...(m.periodS !== undefined ? { periodS: m.periodS } : {}),
    ...(m.duty !== undefined ? { duty: m.duty } : {}),
    ...(m.pulsesS ? { pulsesS: m.pulsesS } : {}),
    ...(spec.counter ? { finalCount: run.count } : {}),
  };
}

interface Verdict {
  /** Every quantity that is outside the bound, with the numbers. */
  readonly misses: { id: string; got: number; hand: number; allowed: number }[];
  /** The worst quantity as a fraction of its bound (counts that differ count as infinity). */
  readonly ratio: number;
}

function judge(run: Run, q: HandBoundQuantities, spec: HandBoundSpec, hand: HandEdges): Verdict {
  const misses: Verdict["misses"] = [];
  let ratio = 0;
  const note = (id: string, got: number, h: number, tol: number): void => {
    const r = Math.abs(got - h) / (tol > 0 ? tol : 1e-300);
    ratio = Math.max(ratio, r);
    if (Math.abs(got - h) > tol) misses.push({ id, got, hand: h, allowed: tol });
  };
  const count = (id: string, got: number, h: number): void => {
    if (got !== h) {
      ratio = Infinity;
      misses.push({ id, got, hand: h, allowed: 0 });
    }
  };
  count("rises", q.rises, hand.up.length);
  count("falls", q.falls, hand.down.length);
  // Each edge both lists hold, against its own allowance.
  for (const [name, got, h] of [["rise", run.up, hand.up], ["fall", run.down, hand.down]] as const) {
    got.slice(0, h.length).forEach((t, i) => note(`${name} ${i}`, t, h[i]!, allowed(spec, h[i]!)));
  }
  if (hand.periodS !== undefined) note("period", q.periodS ?? NaN, hand.periodS, allowed(spec, hand.periodS));
  if (hand.duty !== undefined) note("duty", q.duty ?? NaN, hand.duty, spec.bound.relative * hand.duty);
  hand.pulsesS?.forEach((w, i) => note(`pulse ${i}`, q.pulsesS?.[i] ?? NaN, w, allowed(spec, w)));
  if (spec.counter) count("final count", q.finalCount ?? NaN, hand.finalCount);
  return { misses, ratio };
}

/** The fixture's own hand values (its sidecar's) must be what the model derives: a stale sidecar is a fixture bug, not a verdict. */
function checkSidecarAgainstModel(spec: HandBoundSpec, hand: HandEdges): void {
  const same = (what: string, a: number | undefined, b: number | undefined): void => {
    if (a === undefined && b === undefined) return;
    if (a === undefined || b === undefined || Math.abs(a - b) > 1e-9 * Math.max(1, Math.abs(b))) {
      throw new Error(`handBound: the sidecar's ${what} (${String(a)}) is not the hand model's (${String(b)})`);
    }
  };
  same("rises", spec.hand.rises, hand.up.length);
  same("falls", spec.hand.falls, hand.down.length);
  same("period", spec.hand.periodS, hand.periodS);
  same("duty", spec.hand.duty, hand.duty);
  same("pulse", spec.hand.pulseS, hand.pulsesS?.[0]);
  if (spec.counter) same("final count", spec.hand.finalCount, hand.finalCount);
}

export function runHandBoundEntry(entry: CorpusEntry, circuit: SimCircuit): EntryResult {
  const spec = entry.handBound!;
  assertHandBoundApplies(circuit, spec);
  const endS = entry.durationS ?? (entry.phases ?? []).reduce((s, p) => s + p.durationS, 0);
  const hand = handEdges(spec.model, endS);
  checkSidecarAgainstModel(spec, hand);
  const edits = editsOf(entry);
  const stops = stopsFor(endS, edits.map((e) => e.t));

  let core: Run;
  const coreStart = performance.now();
  try {
    core = coreRun(circuit, spec, endS, stops, edits);
  } catch (err) {
    return { status: "error", kinds: entry.kinds, classicWallMs: null, coreWallMs: null, engine: "core", message: err instanceof Error ? err.message : String(err) };
  }
  const coreWallMs = performance.now() - coreStart;
  let classic: Run;
  const classicStart = performance.now();
  try {
    classic = classicRun(circuit, spec, endS, stops, edits);
  } catch (err) {
    return { status: "error", kinds: entry.kinds, classicWallMs: null, coreWallMs, engine: "classic", message: err instanceof Error ? err.message : String(err) };
  }
  const classicWallMs = performance.now() - classicStart;

  const handQ: HandBoundQuantities = {
    rises: hand.up.length,
    falls: hand.down.length,
    worstEdgeS: 0,
    ...(hand.periodS !== undefined ? { periodS: hand.periodS, duty: hand.duty } : {}),
    ...(hand.pulsesS ? { pulsesS: hand.pulsesS } : {}),
    ...(spec.counter ? { finalCount: hand.finalCount } : {}),
  };
  const coreQ = quantitiesOf(core, spec, hand);
  const classicQ = quantitiesOf(classic, spec, hand);
  const coreVerdict = judge(core, coreQ, spec, hand);
  const classicVerdict = judge(classic, classicQ, spec, hand);

  // Classic's distance: one divergence row per quantity outside the bound, never gated.
  const knownDivergences: KnownDivergenceHit[] = [];
  const seen = new Set<string>();
  for (const miss of classicVerdict.misses) {
    const family = miss.id.replace(/ \d+$/, "");
    if (seen.has(family)) continue; // one row per quantity: the first (earliest) edge that is out, the worst is in the report
    seen.add(family);
    const mismatch: Mismatch = {
      sampleTime: 0,
      category: "sidecar",
      id: `555 ${family}`,
      classicValue: miss.got,
      coreValue: coreQuantity(miss.id, core, coreQ),
      detail: `classic ${miss.got} against hand ${miss.hand} (allowed ${miss.allowed}); core ${String(coreQuantity(miss.id, core, coreQ))}`,
    };
    knownDivergences.push({
      mismatch,
      divergenceId: CLASSIC_555_ON_CELL_RAIL_ID,
      decisionRef: DECISION_REF,
      reason:
        `classic is ${Math.abs(miss.got - miss.hand)} away from the hand value at ${miss.id} (bound ${miss.allowed}); ` +
        `the core is within it: ${coreVerdict.misses.length === 0}`,
    });
  }

  const knownFailingReadings: KnownFailingReading[] = [];
  if (spec.railCapacitor) {
    const peak = (xs: readonly number[]): number => xs.reduce((m, x) => (Math.abs(x) > Math.abs(m) ? x : m), 0);
    knownFailingReadings.push({
      id: spec.railCapacitor,
      quantity: "elementI",
      classic: peak(classic.capCurrent),
      core: peak(core.capCurrent),
      note:
        "the core publishes 0 for a capacitor directly across a battery's rail (publish_fixed_pair_capacitors, core/src/solve/mod.rs); " +
        "not judged and not excused by the marker; fix in progress",
    });
  }

  const report: HandBoundReport = {
    circuitClass: spec.circuitClass,
    bound: spec.bound,
    justification: spec.justification,
    hand: handQ,
    core: coreQ,
    classic: classicQ,
    coreBoundRatio: coreVerdict.ratio,
    knownFailingReadings,
  };

  if (coreVerdict.misses.length === 0) {
    return {
      status: "pass",
      kinds: entry.kinds,
      classicWallMs,
      coreWallMs,
      maxAbsErrorVolts: 0,
      maxRelErrorVolts: 0,
      maxAbsErrorAmps: 0,
      maxRelErrorAmps: 0,
      knownDivergences,
      handBound: report,
    };
  }
  const mismatches: Mismatch[] = coreVerdict.misses.slice(0, 10).map((m) => ({
    sampleTime: 0,
    category: "sidecar",
    id: `555 ${m.id}`,
    classicValue: m.hand,
    coreValue: m.got,
    detail: `core ${m.got} against hand ${m.hand}: |difference| exceeds the bound ${m.allowed} (${spec.bound.relative} x value + ${spec.bound.absoluteS} s)`,
  }));
  return {
    status: "fail",
    kinds: entry.kinds,
    classicWallMs,
    coreWallMs,
    maxAbsErrorVolts: 0,
    maxRelErrorVolts: 0,
    maxAbsErrorAmps: 0,
    maxRelErrorAmps: 0,
    knownDivergences,
    mismatches,
    totalMismatches: coreVerdict.misses.length,
    handBound: report,
  };
}

/** The core's own value for the quantity a classic miss names, for the divergence row. */
function coreQuantity(id: string, core: Run, q: HandBoundQuantities): number | undefined {
  const edge = /^(rise|fall) (\d+)$/.exec(id);
  if (edge) return (edge[1] === "rise" ? core.up : core.down)[Number(edge[2])];
  if (id === "rises") return q.rises;
  if (id === "falls") return q.falls;
  if (id === "period") return q.periodS;
  if (id === "duty") return q.duty;
  if (id === "final count") return q.finalCount;
  const pulse = /^pulse (\d+)$/.exec(id);
  return pulse ? q.pulsesS?.[Number(pulse[1])] : undefined;
}
