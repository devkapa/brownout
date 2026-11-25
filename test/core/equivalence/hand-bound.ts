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
 * The rail capacitor's published current is judged too, against the core's
 * own rail (types.ts's `RailCapacitorReading`): the rail is algebraic and the
 * capacitor follows it, so it carries `C dV/dt` of the rail as published
 * (core/README.md Rails point 7, CAPI; it read exactly 0 before).
 */
import { CoreEngine } from "../../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { buildCoreNodes, corePinKey } from "../../../src/core/compile/topology.js";
import { HeadlessRunner } from "../../../src/host/headless.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { handEdges, measuredQuantities, type HandEdges, type PotAstableHandModel } from "./hand-555.js";
import type { CorpusEntry, EntryResult, HandBoundQuantities, HandBoundReport, HandBoundSpec, KnownDivergenceHit, Mismatch, RailCapacitorReading } from "./types.js";

export const CLASSIC_555_ON_CELL_RAIL_ID = "classic-555-on-cell-rail";
export const CLASSIC_555_POT_CLOCK_ID = "classic-555-pot-clock-step-split";
const DECISION_REF =
  "R122 (a): where classic is shown to be the less accurate engine for a circuit class, the corpus judges the core against " +
  "hand values and records classic's distance (core/README.md, 'The 555 period on a moving rail')";
const POT_DECISION_REF =
  "R122 (a), applied to a clock timed by a potentiometer (P0-SG's end-stop work, coordinator's addition): classic, stepped at a " +
  "fixed host step, splits a step once at a crossing, so a low phase shorter than its step holds for the rest of that step " +
  "(100 % position: +52 % at 100 us steps; ship-logs/p0-sg/clock555.log)";
/** Classic's load-seed pulse (OUT low from load until its first solve settles, 76 us on these boards; README NE555 point 9) is not an edge of the circuit. */
const CLASSIC_SEED_S = 2e-4;
/** The app's frame, the cadence both engines are driven at here. */
const HOST_S = 1 / 60;

/**
 * Refuses the marker on a circuit that is not of its class. "555 on a cell's rail": a 555 whose pin 8 sits on a
 * `battery_pack`'s positive node. "555 timed by a potentiometer": an astable 555 on an ideal source whose timing node
 * is reached through a `potentiometer` wired as a rheostat, the model's position, total and fixed resistors being the
 * circuit's own (a hand model of another clock is a fixture error, not a verdict).
 */
export function assertHandBoundApplies(circuit: SimCircuit, spec: HandBoundSpec): void {
  const fail = (why: string): never => {
    throw new Error(`handBound (${spec.circuitClass}): refused, ${why}. The marker excuses a 555 of its class and nothing else.`);
  };
  if (spec.circuitClass !== "555 on a cell's rail" && spec.circuitClass !== "555 timed by a potentiometer") fail(`unknown circuit class "${String(spec.circuitClass)}"`);
  if (!spec.justification || spec.justification.trim().length === 0) fail("no justification for the bound");
  if (!(spec.bound.relative >= 0) || !(spec.bound.absoluteS >= 0)) fail("no bound");
  const chip = circuit.components.find((c) => c.id === spec.component);
  if (!chip || chip.kind !== "ne555") fail(`"${spec.component}" is not an ne555 in this circuit`);
  const built = buildCoreNodes(circuit);
  const supply = built.pinNode.get(corePinKey(spec.component, "8"));
  if (spec.circuitClass === "555 on a cell's rail") {
    const fedByCell = circuit.components.some(
      (c) => c.kind === "battery_pack" && supply !== undefined && supply !== 0 && built.pinNode.get(corePinKey(c.id, "pos")) === supply,
    );
    if (!fedByCell) fail(`"${spec.component}" pin 8 is not on a battery_pack's positive node`);
  } else {
    if (spec.model.kind !== "astable-pot") fail("the hand model is not an astable-pot model");
    const model = spec.model as PotAstableHandModel;
    const pot = circuit.components.find((c) => c.kind === "potentiometer");
    if (!pot) fail("no potentiometer in the circuit");
    const timing = built.pinNode.get(corePinKey(spec.component, "6"));
    const discharge = built.pinNode.get(corePinKey(spec.component, "7"));
    const potNode = (pin: string): number | undefined => built.pinNode.get(corePinKey(pot!.id, pin));
    // A rheostat: cw on DIS, wiper on the timing node (ccw tied to the wiper, so the swept part carries the current).
    if (potNode("cw") !== discharge || potNode("wiper") !== timing || potNode("ccw") !== timing || timing === undefined || timing === 0) {
      fail(`the potentiometer "${pot!.id}" is not a rheostat from pin 7 (cw) to the timing node (wiper tied to ccw)`);
    }
    const sourceOnSupply = circuit.components.some((c) => c.kind === "voltage_source" && supply !== undefined && supply !== 0 && built.pinNode.get(corePinKey(c.id, "pos")) === supply);
    if (!sourceOnSupply) fail(`"${spec.component}" pin 8 is not on a voltage_source's positive node`);
    const num = (v: unknown): number => Number(v);
    if (num(pot!.params.rTotal) !== model.potTotalOhm || num(pot!.params.position) !== model.position) {
      fail(`the potentiometer is ${String(pot!.params.rTotal)} ohm at ${String(pot!.params.position)}, the model's ${model.potTotalOhm} ohm at ${model.position}`);
    }
    const cap = circuit.components.find((c) => c.kind === "capacitor" && built.pinNode.get(corePinKey(c.id, "a")) === timing);
    if (!cap || num(cap.params.capacitance) !== model.cF) fail(`the timing capacitor on pin 6 is not the model's ${model.cF} F`);
    const ra = circuit.components.find((c) => c.kind === "resistor" && built.pinNode.get(corePinKey(c.id, "a")) === supply && built.pinNode.get(corePinKey(c.id, "b")) === discharge);
    if (!ra || num(ra.params.resistance) !== model.raOhm) fail(`the resistor from the supply to pin 7 is not the model's ${model.raOhm} ohm`);
  }
  for (const id of [spec.counter, spec.railCapacitor]) {
    if (id !== undefined && !circuit.components.some((c) => c.id === id)) fail(`"${id}" is not in the circuit`);
  }
}

export interface Run {
  up: number[];
  down: number[];
  count?: number;
  /** The rail capacitor's published current at each stop. */
  capCurrent: number[];
  /** The core only: `C dV/dt` of its own published rail at each stop, the right-hand rate, and its resolution. */
  capRate?: number[];
  capResolution?: number[];
}

/** W0's current tolerance. */
const w0I = (want: number): number => 1e-9 + 1e-3 * Math.abs(want);
/**
 * How far past a stop the core's rail is read for its right-hand rate. The rail
 * publishes a move only past RAIL_MOVE_EPS (1e-12 V, README Rails point 3), so a
 * difference over `h` resolves the rate to about that over `h`: at 1 ns it was
 * 1e-3 V/s, 22 nA on 22 uF, 22 W0 of the 1 nA floor (fixture 79 at rest).
 * At 0.5 us the three-point one-sided difference below resolves it to
 * `5 RAIL_MOVE_EPS / (2 h)` = 5e-6 V/s and is exact to `k^2 h^2 / 3` of the rate
 * on a mode of rate `k` (7.5e-5 for the 10 nF control-pin mode, about 33 us).
 * A digital change inside the window (a 555 edge just after the stop, whose
 * rail jump is no rate) shortens it fivefold until none falls inside.
 */
const RATE_PROBE_S = [5e-7, 1e-7, 2e-8, 4e-9];
const RAIL_MOVE_EPS = 1e-12;

/**
 * `C dV/dt` of the core's published rail just after the stop, by `(-3 v0 + 4 v1 - v2) / (2 h)` over the rail read
 * at `t`, `t + h` and `t + 2h` on a snapshot that is then put back, so the run is the unprobed one. Returns the rate's
 * `C dV/dt` and the resolution it has from the rail's move threshold, `C 5 RAIL_MOVE_EPS / (2 h)`.
 */
export function rightHandCapRate(engine: CoreEngine, cap: { c: number; dv: (v: Record<string, number>) => number }, t: number): { rate: number; resolution: number } {
  const v0 = cap.dv(engine.getNetV());
  const digital = JSON.stringify(engine.digitalState);
  let out = { rate: NaN, resolution: Infinity };
  for (const h of RATE_PROBE_S) {
    const snap = engine.saveState();
    try {
      engine.advance(t + h);
      const v1 = cap.dv(engine.getNetV());
      engine.advance(t + 2 * h);
      const v2 = cap.dv(engine.getNetV());
      const still = JSON.stringify(engine.digitalState) === digital;
      out = { rate: (cap.c * (-3 * v0 + 4 * v1 - v2)) / (2 * h), resolution: (cap.c * 5 * RAIL_MOVE_EPS) / (2 * h) };
      if (still) return out;
    } finally {
      engine.restoreState(snap);
      snap.dispose();
    }
  }
  return out;
}

/** The stops: every frame, every phase boundary, and two early ones where the rail capacitor moves most. */
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

/** `V(pins a - b)` of a capacitor on the core's published nets, and its capacitance. */
export function capacitorOf(circuit: SimCircuit, id: string): { c: number; dv: (v: Record<string, number>) => number } {
  const built = buildCoreNodes(circuit);
  const net = (pin: string): string | null => built.netIdOfNode[built.pinNode.get(corePinKey(id, pin)) ?? 0] ?? null;
  const [a, b] = [net("a"), net("b")];
  const at = (v: Record<string, number>, n: string | null): number => (n === null ? 0 : (v[n] ?? 0));
  const c = Number(circuit.components.find((x) => x.id === id)!.params.capacitance);
  return { c, dv: (v) => at(v, a) - at(v, b) };
}

function coreRun(circuit: SimCircuit, spec: HandBoundSpec, endS: number, stops: readonly number[], edits: ReturnType<typeof editsOf>): Run {
  const engine = new CoreEngine();
  const cap = spec.railCapacitor ? capacitorOf(circuit, spec.railCapacitor) : undefined;
  try {
    engine.load(circuit, CORE_IMPLEMENTED_KINDS);
    engine.enableTrace(true);
    engine.clearTrace();
    const run: Run = { up: [], down: [], capCurrent: [], capRate: [], capResolution: [] };
    const pending = edits.filter((e) => e.t > 0);
    for (const e of edits.filter((x) => x.t <= 0)) engine.setParams([{ componentId: e.id, params: e.params }]);
    for (const t of stops) {
      engine.advance(t);
      for (const ev of engine.readTrace()) {
        if (ev.key !== spec.component || ev.t <= 0) continue;
        (ev.value === 1 ? run.up : run.down).push(ev.t);
      }
      engine.clearTrace();
      if (spec.railCapacitor && cap) {
        run.capCurrent.push(engine.getElementI()[spec.railCapacitor] ?? 0);
        const { rate, resolution } = rightHandCapRate(engine, cap, t);
        engine.clearTrace();
        run.capRate!.push(rate);
        run.capResolution!.push(resolution);
      }
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
    if (spec.railCapacitor) run.capCurrent.push(runner.snapshot().elementI[spec.railCapacitor] ?? 0);
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

export function judge(run: Run, q: HandBoundQuantities, spec: HandBoundSpec, hand: HandEdges): Verdict {
  const misses: Verdict["misses"] = [];
  let ratio = 0;
  const note = (id: string, got: number, h: number, tol: number): void => {
    // A quantity that was not measured (NaN: no complete cycle, a missing pulse) is a miss, never a pass.
    if (!Number.isFinite(got)) {
      ratio = Infinity;
      misses.push({ id, got, hand: h, allowed: tol });
      return;
    }
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
      divergenceId: spec.circuitClass === "555 timed by a potentiometer" ? CLASSIC_555_POT_CLOCK_ID : CLASSIC_555_ON_CELL_RAIL_ID,
      decisionRef: spec.circuitClass === "555 timed by a potentiometer" ? POT_DECISION_REF : DECISION_REF,
      reason:
        `classic is ${Math.abs(miss.got - miss.hand)} away from the hand value at ${miss.id} (bound ${miss.allowed}); ` +
        `the core is within it: ${coreVerdict.misses.length === 0}`,
    });
  }

  const railCapacitor = spec.railCapacitor ? railCapacitorReading(spec.railCapacitor, stops, core, classic) : undefined;
  const capMiss = railCapacitor !== undefined && !(railCapacitor.coreWorstW0 <= 1);

  const report: HandBoundReport = {
    circuitClass: spec.circuitClass,
    bound: spec.bound,
    justification: spec.justification,
    hand: handQ,
    core: coreQ,
    classic: classicQ,
    coreBoundRatio: coreVerdict.ratio,
    ...(railCapacitor ? { railCapacitor } : {}),
  };

  if (coreVerdict.misses.length === 0 && !capMiss) {
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
  if (capMiss) {
    const k = stops.indexOf(railCapacitor.coreWorstAtS);
    mismatches.push({
      sampleTime: railCapacitor.coreWorstAtS,
      category: "sidecar",
      id: `${railCapacitor.id} C dV/dt`,
      classicValue: core.capRate![k]!,
      coreValue: core.capCurrent[k]!,
      detail: `the rail capacitor reads ${core.capCurrent[k]} A against C dV/dt ${core.capRate![k]} A of the core's own rail: ${railCapacitor.coreWorstW0} of W0`,
    });
  }
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
    totalMismatches: coreVerdict.misses.length + (capMiss ? 1 : 0),
    handBound: report,
  };
}

/** The rail capacitor's reading at every stop: the core's against its own rail's `C dV/dt`, classic's recorded. */
export function railCapacitorReading(id: string, stops: readonly number[], core: Run, classic: Run): RailCapacitorReading {
  const peak = (xs: readonly number[]): number => xs.reduce((m, x) => (Math.abs(x) > Math.abs(m) ? x : m), 0);
  let worst = 0;
  let at = stops[0] ?? 0;
  core.capCurrent.forEach((i, k) => {
    const want = core.capRate![k]!;
    const r = Math.abs(i - want) / (w0I(want) + (core.capResolution?.[k] ?? 0));
    // A NaN reading or rate is a miss, never a pass.
    if (!(r <= worst)) {
      worst = Number.isNaN(r) ? Infinity : r;
      at = stops[k]!;
    }
  });
  return { id, stops: core.capCurrent.length, coreWorstW0: worst, coreWorstAtS: at, corePeak: peak(core.capCurrent), classicPeak: peak(classic.capCurrent) };
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
