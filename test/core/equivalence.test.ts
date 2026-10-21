/**
 * The gate: every kind that graduates into CORE_SUPPORTED_KINDS (kinds.ts)
 * must first show up here as "pass" across the whole corpus. Always runs
 * brownout's own fixtures (test/fixtures/circuits) plus this harness's own
 * synthetic W0 fixtures (test/fixtures/core); an external corpus only runs
 * when BROWNOUT_CORE_CORPUS names one or more directories (`:`-separated),
 * and nothing from it is ever written into this repo — CORE_EQUIV_REPORT
 * writes to a caller-given path, CORE_EQUIV_PRINT prints to the console, and
 * both are opt-in.
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../src/core/wasm.js";
import { CORE_SUPPORTED_KINDS } from "../../src/core/kinds.js";
import { buildCoreNodes, corePinKey } from "../../src/core/compile/topology.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { alignCycles, compareAlignedCycles, sampleCycles, type AlignmentSuccess, type CycleSample } from "./equivalence/align.js";
import { compareSamples } from "./equivalence/compare.js";
import { parseCorpusDirsEnv } from "./equivalence/external-corpus.js";
import { runEquivalenceHarness, type HarnessRun } from "./equivalence/harness.js";
import { renderMarkdownReport, toJsonReport } from "./equivalence/report.js";
import { explainAnalyticMismatches } from "./equivalence/run-entry.js";
import { collectClassicSamples, collectCoreSamples, DEFAULT_DURATION_S, sampleTimesFor } from "./equivalence/sample.js";
import { checkSupportedKindsGate } from "./equivalence/supported-kinds-gate.js";
import { loadSyntheticCorpus } from "./equivalence/synthetic-fixtures.js";
import { collectClassicDigitalTrace, type DigitalTraceEvent } from "./equivalence/trace.js";
import type { KindRow, SampleReadout } from "./equivalence/types.js";

// ---- small circuit-building helpers, local to this file (this worker owns
// no shared test-helper files; matches test/core/compile.test.ts's and
// test/core/engine.test.ts's own convention) --------------------------------

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

function circuit(components: SimComponent[], wires: SimCircuit["wires"] = []): SimCircuit {
  return { components, wires };
}

/** A 10 V / 1 kΩ / 1 kΩ divider with r1's resistance overridable — the self-test below perturbs it by 1%. */
function dividerWithR1(resistance: number): SimCircuit {
  return circuit(
    [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 10 }),
      part("gnd", "ground", ["gnd"]),
      part("r1", "resistor", ["a", "b"], { resistance }),
      part("r2", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    [wire("vsrc", "neg", "gnd", "gnd"), wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "r2", "a"), wire("r2", "b", "gnd", "gnd")],
  );
}

function omit(record: Record<string, number>, key: string): Record<string, number> {
  const copy = { ...record };
  delete copy[key];
  return copy;
}

let run: HarnessRun;

beforeAll(async () => {
  await initCore();

  const corpusDirs = parseCorpusDirsEnv(process.env.BROWNOUT_CORE_CORPUS);
  run = runEquivalenceHarness({ corpusDirs });

  if (process.env.CORE_EQUIV_REPORT) {
    fs.writeFileSync(process.env.CORE_EQUIV_REPORT, renderMarkdownReport(run));
  }
  if (process.env.CORE_EQUIV_PRINT === "1") {
    // eslint-disable-next-line no-console -- explicitly opt-in console output, per the harness's own contract.
    console.log(renderMarkdownReport(run));
  }
  // Keep toJsonReport reachable from a debugger/REPL against `run` without
  // importing it separately; not asserted on here.
  void toJsonReport;
});

describe("synthetic fixtures — declared outcome (assertion a)", () => {
  it("every synthetic fixture in test/fixtures/core gets its declared expected outcome", () => {
    expect(run.syntheticOutcomes.length).toBeGreaterThan(0);
    for (const { entry, expected, result } of run.syntheticOutcomes) {
      if (expected.status === "pass") {
        expect(result.status, `${entry.id}: expected "pass", got "${result.status}"`).toBe("pass");
      } else {
        expect(result.status, `${entry.id}: expected "core-declined", got "${result.status}"`).toBe("core-declined");
        if (expected.reasonPrefix && result.status === "core-declined") {
          const matched = result.reasons.some((reason) => reason.startsWith(expected.reasonPrefix!));
          expect(
            matched,
            `${entry.id}: expected a reason starting with "${expected.reasonPrefix}", got ${JSON.stringify(result.reasons)}`,
          ).toBe(true);
        }
      }
    }
  });
});

describe("the gate (assertion b)", () => {
  // checkSupportedKindsGate iterates CORE_SUPPORTED_KINDS itself (rather
  // than filtering the table's rows), so a supported kind with NO row at
  // all — no always-on entry contains it — fails the gate instead of
  // passing vacuously. See supported-kinds-gate.ts's header.
  it("every kind in CORE_SUPPORTED_KINDS has a row in the per-kind table, ran on at least one entry, with status \"pass\"", () => {
    const offenders = checkSupportedKindsGate(run.kindTable, CORE_SUPPORTED_KINDS);
    expect(
      offenders.map((o) => `${o.kind}: ${o.reason}`),
      "CORE_SUPPORTED_KINDS (src/core/kinds.ts) is the set of kinds a host is allowed to route " +
        "to the core in production. A kind only belongs there once this harness's per-kind table " +
        "shows it PASSING — a row that exists, ran on at least one entry, zero failures — across " +
        "the whole corpus. Every kind listed above failed that bar despite being in " +
        "CORE_SUPPORTED_KINDS: either the set was widened before the equivalence proof backs it " +
        "up, or a regression broke a previously-passing kind. This assertion is VACUOUSLY TRUE in " +
        "W0 — CORE_SUPPORTED_KINDS is empty until a kind actually clears this bar.",
    ).toEqual([]);
  });

  // The seam checkSupportedKindsGate exists for: prove a supported kind
  // absent from the table entirely is caught, without touching the real
  // CORE_SUPPORTED_KINDS or the always-on corpus. Before the fix, the old
  // "filter run.kindTable's rows" check could never flag a kind with no
  // row — this proves the replacement can.
  it("flags a supported kind that has no row at all in an injected per-kind table", () => {
    const passingResistorRow: KindRow = {
      kind: "resistor",
      implemented: true,
      supported: true,
      fixtures: 1,
      ran: 1,
      pass: 1,
      fail: 0,
      declined: 0,
      notPorted: 0,
      errored: 0,
      unstimulated: 0,
      status: "pass",
    };

    const offenders = checkSupportedKindsGate([passingResistorRow], new Set(["resistor", "bench_psu"]));

    expect(offenders.map((o) => o.kind)).toEqual(["bench_psu"]);
    expect(offenders[0]!.reason).toMatch(/no row/i);
  });

  it("flags a supported kind whose row exists but never ran (ran=0)", () => {
    const untestedDiodeRow: KindRow = {
      kind: "diode",
      implemented: true,
      supported: true,
      fixtures: 1,
      ran: 0,
      pass: 0,
      fail: 0,
      declined: 1,
      notPorted: 0,
      errored: 0,
      unstimulated: 0,
      status: "untested",
    };

    const offenders = checkSupportedKindsGate([untestedDiodeRow], new Set(["diode"]));

    expect(offenders.map((o) => o.kind)).toEqual(["diode"]);
    expect(offenders[0]!.reason).toMatch(/ran on no entry/i);
  });

  it("passes a supported kind whose row is present, ran, and passing", () => {
    const passingResistorRow: KindRow = {
      kind: "resistor",
      implemented: true,
      supported: true,
      fixtures: 1,
      ran: 1,
      pass: 1,
      fail: 0,
      declined: 0,
      notPorted: 0,
      errored: 0,
      unstimulated: 0,
      status: "pass",
    };

    expect(checkSupportedKindsGate([passingResistorRow], new Set(["resistor"]))).toEqual([]);
  });
});

describe("detector self-test — a clean sweep proves nothing unless the detector can fail (assertion c)", () => {
  // All three plant a mismatch through compareSamples() itself — the
  // harness's one internal seam for this (see compare.ts's header) — never
  // by editing src.

  it("fires on a genuine 1% resistor mismatch between two real engine runs, naming the affected net", () => {
    const times = sampleTimesFor(DEFAULT_DURATION_S);
    const classicSamples = collectClassicSamples(dividerWithR1(1000), times);
    const coreSamples = collectCoreSamples(dividerWithR1(1010), times); // r1 off by 1%

    const result = compareSamples(classicSamples, coreSamples);
    const netMismatches = result.mismatches.filter((m) => m.category === "netV");
    expect(netMismatches.length, "a 1% resistor mismatch must surface as at least one netV mismatch").toBeGreaterThan(0);
    for (const m of netMismatches) {
      expect(typeof m.id).toBe("string");
      expect(m.id.length).toBeGreaterThan(0);
    }
  });

  it("fires on a key-set mismatch (a net present on one side only)", () => {
    const times = sampleTimesFor(DEFAULT_DURATION_S);
    const base = dividerWithR1(1000);
    const classicSamples = collectClassicSamples(base, times);
    const coreSamples = collectCoreSamples(base, times);

    const droppedKey = Object.keys(coreSamples[0]!.netV).find((key) => key !== "gnd");
    expect(droppedKey).toBeDefined();
    const tampered: SampleReadout[] = coreSamples.map((sample, i) =>
      i === 0 ? { ...sample, netV: omit(sample.netV, droppedKey!) } : sample,
    );

    const result = compareSamples(classicSamples, tampered);
    const keySetMismatch = result.mismatches.find((m) => m.category === "netV" && m.id === droppedKey);
    expect(keySetMismatch, `expected a netV mismatch naming "${droppedKey}"`).toBeDefined();
    expect(keySetMismatch!.detail).toMatch(/present only on the classic side/);
  });

  it("fires on a digitalState difference, using a fabricated readout through the same seam", () => {
    const times = sampleTimesFor(DEFAULT_DURATION_S);
    const base = dividerWithR1(1000);
    const classicSamples = collectClassicSamples(base, times); // digitalState is always {} in W0
    const coreSamples = collectCoreSamples(base, times);

    const tampered: SampleReadout[] = coreSamples.map((sample, i) =>
      i === 0 ? { ...sample, digitalState: { clk: 1 } } : sample,
    );

    const result = compareSamples(classicSamples, tampered);
    const digitalMismatch = result.mismatches.find((m) => m.category === "digitalState" && m.id === "clk");
    expect(digitalMismatch, 'expected a digitalState mismatch naming "clk"').toBeDefined();
  });

  it("fires on a simTime mismatch (the two sides sampled at different instants), using a fabricated readout through the same seam", () => {
    const times = sampleTimesFor(DEFAULT_DURATION_S);
    const base = dividerWithR1(1000);
    const classicSamples = collectClassicSamples(base, times);
    const coreSamples = collectCoreSamples(base, times);

    // Nudge one core sample's own simTime label off of what it was actually
    // taken at — the values otherwise agree exactly, so only the new
    // simTime check (not netV/elementI) can catch this.
    const tampered: SampleReadout[] = coreSamples.map((sample, i) =>
      i === 1 ? { ...sample, simTime: sample.simTime + 1e-6 } : sample,
    );

    const result = compareSamples(classicSamples, tampered);
    const simTimeMismatch = result.mismatches.find((m) => m.category === "simTime");
    expect(simTimeMismatch, "expected a simTime mismatch").toBeDefined();
  });

  // W1g: the W1c exemption is gone (compare.ts's compareFailureKeys,
  // w1g-failures.ts). An empty core set is an answer and is held to
  // classic's, and a set the core reports that disagrees is caught as before.
  it("fires on a failure classic alone reports even when the core's failures side is completely empty (W1g has landed)", () => {
    const times = sampleTimesFor(DEFAULT_DURATION_S);
    const base = dividerWithR1(1000);
    const classicSamples = collectClassicSamples(base, times);
    const coreSamples = collectCoreSamples(base, times); // failures is always {} in W0/W1a-c

    const tampered: SampleReadout[] = classicSamples.map((sample, i) =>
      i === 0
        ? { ...sample, failures: { r1: { kind: "resistor_overload", componentId: "r1", since: 0, message: "fabricated for this test", value: 1, limit: 1 } } }
        : sample,
    );

    const result = compareSamples(tampered, coreSamples);
    const failureMismatch = result.mismatches.find((m) => m.category === "failures");
    expect(failureMismatch, "an empty core failure set is an answer since W1g (Equivalence harness rules, W1g)").toBeDefined();
  });

  it("still fires on a failures-presence difference when the core ALSO reports something (of its own) that disagrees with classic — the exemption never masks a real mismatch", () => {
    const times = sampleTimesFor(DEFAULT_DURATION_S);
    const base = dividerWithR1(1000);
    const classicSamples = collectClassicSamples(base, times);
    const coreSamples = collectCoreSamples(base, times);

    const classicTampered: SampleReadout[] = classicSamples.map((sample, i) =>
      i === 0
        ? { ...sample, failures: { r1: { kind: "resistor_overload", componentId: "r1", since: 0, message: "fabricated for this test", value: 1, limit: 1 } } }
        : sample,
    );
    // The core's side is NOT empty here — it reports a DIFFERENT failure key
    // of its own (fabricated, standing in for a future W1g false positive),
    // so the "completely empty" precondition compareFailureKeys checks does
    // not hold and the mismatch must still be reported.
    const coreTampered: SampleReadout[] = coreSamples.map((sample, i) =>
      i === 0
        ? { ...sample, failures: { r2: { kind: "output_sag", componentId: "r2", since: 0, message: "fabricated for this test", value: 1, limit: 1 } } }
        : sample,
    );

    const result = compareSamples(classicTampered, coreTampered);
    const failureMismatches = result.mismatches.filter((m) => m.category === "failures");
    expect(
      failureMismatches.map((m) => m.id).sort(),
      "both r1 (classic-only) and r2 (core-only) must still be reported — the exemption does not apply once the core's side is non-empty",
    ).toEqual(["r1", "r2"]);
  });
});

describe("gate 2e / decision 3 — analytic-closer detector (assertion c continued)", () => {
  // Fixture 11 (test/fixtures/core/11-rc-charge) declares an "rc" analytic
  // model in its .expected.json — loaded here exactly as the harness itself
  // loads it (loadSyntheticCorpus), never hand-constructed, so this test
  // proves the REAL declared model, not a stand-in for it.
  const fixture11 = loadSyntheticCorpus().find((e) => e.entry.name === "11-rc-charge")!;

  /** The net id `fixture11`'s own declared analytic node resolves to — the same resolution buildAnalyticTargets (run-entry.ts) itself uses. */
  function fixture11AnalyticNetId(): string {
    const model = fixture11.entry.analytic!;
    if (model.kind !== "rc") throw new Error("fixture 11 is expected to declare an \"rc\" analytic model");
    const built = buildCoreNodes(fixture11.circuit);
    const node = built.pinNode.get(corePinKey(model.node.component, model.node.pin));
    if (node === undefined) throw new Error("fixture 11's declared analytic node did not resolve to a net");
    const netId = built.netIdOfNode[node];
    if (!netId) throw new Error("fixture 11's declared analytic node resolved to the synthetic (netless) datum");
    return netId;
  }

  it("explains a genuine classic-BE-lag mismatch as analytic-closer (decision 3), on a REAL classic-vs-core run", () => {
    const times = sampleTimesFor(DEFAULT_DURATION_S); // [0, 2.5ms, 5ms, 10ms] -- short next to fixture 11's 100ms tau, so classic's lag is still visible
    const classicSamples = collectClassicSamples(fixture11.circuit, times);
    const coreSamples = collectCoreSamples(fixture11.circuit, times);

    const cmp = compareSamples(classicSamples, coreSamples);
    // Sanity: this is what makes the rest of the test meaningful -- if
    // compareSamples stops disagreeing here (a tighter classic stepper, a
    // different sample grid, ...), there is nothing left for the analytic
    // path to explain, and a "0 explained, 0 remaining" result would look
    // identical to "the detector is broken".
    expect(cmp.mismatches.some((m) => m.category === "netV"), "expected a real classic-vs-core netV mismatch on this fixture's short default sample grid").toBe(true);

    const { explained, remaining } = explainAnalyticMismatches(fixture11.entry, fixture11.circuit, cmp.mismatches);
    expect(explained.length).toBeGreaterThan(0);
    expect(remaining).toEqual([]);
    for (const hit of explained) {
      expect(hit.divergenceId).toBe("analytic:rc");
      expect(hit.reason).toMatch(/analytic answer/);
    }
  });

  it("does NOT explain away a core value perturbed away from the analytic answer, even on the declared node (the detector must still fire)", () => {
    const times = sampleTimesFor(DEFAULT_DURATION_S);
    const classicSamples = collectClassicSamples(fixture11.circuit, times);
    const coreSamples = collectCoreSamples(fixture11.circuit, times);
    const netId = fixture11AnalyticNetId();

    // +0.5 V at the t=5ms sample: far outside tolerance of BOTH classic and
    // the analytic answer (the untampered core value there is ~0.24 V) --
    // exactly the "core itself is wrong" case gate 2e must never launder.
    const tamperedIndex = 2;
    const tampered: SampleReadout[] = coreSamples.map((sample, i) =>
      i === tamperedIndex ? { ...sample, netV: { ...sample.netV, [netId]: sample.netV[netId]! + 0.5 } } : sample,
    );

    const cmp = compareSamples(classicSamples, tampered);
    const perturbedMismatch = cmp.mismatches.find((m) => m.category === "netV" && m.id === netId && m.sampleTime === tampered[tamperedIndex]!.simTime);
    expect(perturbedMismatch, "the perturbation itself must surface as a netV mismatch, or this test proves nothing").toBeDefined();

    const { explained, remaining } = explainAnalyticMismatches(fixture11.entry, fixture11.circuit, cmp.mismatches);
    expect(remaining).toContainEqual(perturbedMismatch);
    expect(explained.some((hit) => hit.mismatch === perturbedMismatch)).toBe(false);
  });

  it("leaves a mismatch on an UNDECLARED net or element untouched, even for an entry that declares an analytic model", () => {
    const times = sampleTimesFor(DEFAULT_DURATION_S);
    const classicSamples = collectClassicSamples(fixture11.circuit, times);
    const coreSamples = collectCoreSamples(fixture11.circuit, times);

    // A fabricated mismatch on a component this fixture doesn't even have --
    // guaranteed to name neither the declared node nor a declared current.
    const fabricated: SampleReadout["elementI"] = { ...coreSamples[0]!.elementI, "not-a-real-component": 1 };
    const tampered: SampleReadout[] = coreSamples.map((sample, i) => (i === 0 ? { ...sample, elementI: fabricated } : sample));

    const cmp = compareSamples(classicSamples, tampered);
    const fabricatedMismatch = cmp.mismatches.find((m) => m.category === "elementI" && m.id === "not-a-real-component");
    expect(fabricatedMismatch).toBeDefined();

    const { explained, remaining } = explainAnalyticMismatches(fixture11.entry, fixture11.circuit, cmp.mismatches);
    expect(remaining).toContainEqual(fabricatedMismatch);
    expect(explained.some((hit) => hit.mismatch === fabricatedMismatch)).toBe(false);
  });
});

describe("no entry in the always-on corpus ends in error (assertion d)", () => {
  it("brownout's fixtures and this harness's synthetic fixtures never report status \"error\"", () => {
    const alwaysOn = run.outcomes.filter(
      (o) => o.entry.origin === "brownout-fixture" || o.entry.origin === "synthetic",
    );
    expect(alwaysOn.length).toBeGreaterThan(0);
    const errored = alwaysOn.filter((o) => o.result.status === "error");
    expect(errored.map((o) => `${o.entry.id}: ${(o.result as { message?: string }).message ?? ""}`)).toEqual([]);
  });
});

// ---- gate 2a, validated before any digital kind exists -------------------
//
// "classic run adaptively vs classic run at a refined fixed step align as
// identical on the brownout fixtures that have digital state" (worker
// brief's Tests list). No kind here is in CORE_IMPLEMENTED_KINDS yet, so
// there is no real core-vs-classic digital comparison to run — this proves
// the ALIGNMENT MACHINERY ITSELF (trace.ts + align.ts) against two classic
// runs of the identical circuit on two different step grids, which is
// exactly what W1a's goal calls for: "a harness that aligns digital traces
// per Decision 6..., validated before any digital kind exists."
//
// The "core" side below is classic run again, at a much finer FIXED step —
// standing in for the core purely so this test exercises the real
// findRisingEdges/sampleCycles/alignCycles/compareAlignedCycles pipeline
// end to end, not a synthetic stand-in (align.test.ts already covers the
// pipeline's logic against hand-built synthetic traces per this worker's
// brief; this is the complementary "against something real" proof).

const CIRCUITS_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "fixtures", "circuits");

function loadCircuitFixture(name: string): SimCircuit {
  return JSON.parse(fs.readFileSync(path.join(CIRCUITS_DIR, `${name}.sim.json`), "utf8")) as SimCircuit;
}

/** The first component whose own digitalState key is a bare, driven clock/pulse level — the natural cycle boundary for that fixture. `null` when the fixture has no such source (nothing to align cycles on). */
function findClockComponentId(circuit: SimCircuit): string | null {
  const match = circuit.components.find((c) => c.kind === "clock" || c.kind === "pulse_source" || c.kind === "ne555");
  return match ? match.id : null;
}

/** The first cycle index (within `[0, boundCycles]`) whose state is unique in BOTH sequences — alignCycles's own precondition ("must match exactly once... in each"). `null` if no such index exists inside the bound. */
function findUniqueAnchorIndex(a: readonly CycleSample[], b: readonly CycleSample[], boundCycles: number): number | null {
  const limit = Math.min(a.length, b.length, boundCycles + 1);
  const countMatching = (cycles: readonly CycleSample[], target: string): number =>
    cycles.slice(0, limit).filter((c) => JSON.stringify(c.state) === target).length;
  for (let i = 0; i < limit; i++) {
    const candidate = JSON.stringify(a[i]!.state);
    if (countMatching(a, candidate) === 1 && countMatching(b, candidate) === 1) return i;
  }
  return null;
}

/**
 * Fixtures gate 2a checks, and how long to trace them for (long enough for
 * several cycles; short enough to stay fast). Every fixture LISTED here is
 * REQUIRED to produce a clean, uniquely-anchored, offset-0 comparison
 * (finding 11) — `checkDigitalAlignment` throws rather than skipping when
 * one doesn't, so an alignment failure or a missing anchor on a listed
 * fixture is a test FAILURE, not silently absorbed as "not checked".
 *
 * Excluded from the list entirely, each for a documented reason:
 * - 02-logic-ic-gate-lab and 06-memory-rom-lab have no autonomous
 *   clock/pulse/555 source at all (02's digitalState only drifts via the
 *   floating-input hash's 10ms buckets; 06 is a static ROM/RAM read with no
 *   stimulus in this plain run) — `findClockComponentId` would return
 *   `null` for either, and there is genuinely no "cycle" to align on, not a
 *   fixture-specific quirk worth chasing.
 * - 04-74hc595-shift-7seg: its `pulse_srclk` source has a 1 µs rise/fall
 *   time against an 80 ms period, and adaptive-vs-a-500µs-fixed-grid
 *   classic can count one FEWER accepted pulse by the ~7th cycle in a 0.64s
 *   window, shifting the 74HC595's shifted-in bit by one position from that
 *   point on. That is a genuine classic engine step-grid sensitivity on a
 *   very narrow source edge, independent of this harness — confirmed by
 *   hand while writing this test (an intermittent 1-mismatch result
 *   surfaced at cycle 8) — not something align.ts should paper over by
 *   excluding real mismatches after the fact.
 * - 07-74hc165-parallel-serial-io: the SAME class of genuine step-grid/edge-
 *   timing sensitivity as 04, confirmed by hand while fixing finding 11 — at
 *   the true (structurally-inferable) offset, cycles 2-6 agree between the
 *   adaptive and refined-fixed-step classic runs, but cycles 0 and 1
 *   disagree (adaptive: 0,1; refined: 0,0). Unlike 04, no single cycle
 *   within the offset bound even has a state unique to both traces (its
 *   shift pattern recurs with period 2), so `findUniqueAnchorIndex` cannot
 *   find an anchor to compare from in the first place — this is genuinely
 *   "no anchor exists", not a bug in the anchor search. Do not add 07 back
 *   to this list without re-deriving these numbers and either fixing the
 *   underlying step-grid sensitivity or choosing a projection/anchor that
 *   sidesteps it.
 */
const DIGITAL_ALIGNMENT_FIXTURES: ReadonlyArray<{ name: string; durationS: number }> = [
  { name: "03-ne555-74ls161-counter-7seg", durationS: 1.2 },
  { name: "05-register-bus-adder", durationS: 4.0 },
];

const REFINED_FIXED_STEP_S = 5e-4;
const OFFSET_BOUND_CYCLES = 6;

interface DigitalAlignmentCheck {
  readonly comparedCycles: number;
  readonly mismatchCount: number;
  readonly offsetCycles: number;
}

/**
 * Runs the alignment pipeline for one fixture. Finding 11: throws (a test
 * FAILURE, not a `null` skip) when the fixture has no clock/pulse/555
 * source, no cycle within the offset bound is state-unique on both sides
 * (no anchor), or `alignCycles` itself reports an alignment failure — every
 * one of those was previously swallowed into "not checked" instead of
 * failing the gate.
 */
function checkDigitalAlignment(name: string, durationS: number): DigitalAlignmentCheck {
  const circuit = loadCircuitFixture(name);
  const clockKey = findClockComponentId(circuit);
  if (!clockKey) {
    throw new Error(`checkDigitalAlignment(${name}): expected a clock/pulse/555 source to align cycles on, found none`);
  }

  const adaptiveTrace: DigitalTraceEvent[] = collectClassicDigitalTrace(circuit, durationS);
  const refinedTrace: DigitalTraceEvent[] = collectClassicDigitalTrace(circuit, durationS, { fixedStepS: REFINED_FIXED_STEP_S });

  const adaptiveCycles = sampleCycles(adaptiveTrace, clockKey);
  const refinedCycles = sampleCycles(refinedTrace, clockKey);

  const anchorIndex = findUniqueAnchorIndex(adaptiveCycles, refinedCycles, OFFSET_BOUND_CYCLES);
  if (anchorIndex === null) {
    throw new Error(
      `checkDigitalAlignment(${name}): no cycle within the first ${OFFSET_BOUND_CYCLES + 1} cycles has a state ` +
        "unique to BOTH traces — cannot find an anchor to align on",
    );
  }

  const alignment = alignCycles(adaptiveCycles, refinedCycles, adaptiveCycles[anchorIndex]!.state, OFFSET_BOUND_CYCLES);
  if (!alignment.ok) {
    throw new Error(`checkDigitalAlignment(${name}): alignment failed: ${alignment.reason}`);
  }

  const comparison = compareAlignedCycles(adaptiveCycles, refinedCycles, alignment as AlignmentSuccess);
  return { comparedCycles: comparison.comparedCycles, mismatchCount: comparison.mismatches.length, offsetCycles: alignment.offsetCycles };
}

describe("gate 2a: classic-adaptive vs classic-at-a-refined-fixed-step align as identical (brownout fixtures with digital state)", () => {
  it.each(DIGITAL_ALIGNMENT_FIXTURES)(
    "$name: aligns at offset 0 with zero mismatches across at least 2 compared cycles",
    ({ name, durationS }) => {
      const check = checkDigitalAlignment(name, durationS);
      // Finding 11: offset 0 is now actually asserted, not just implied by
      // the (never-checked) test title — two classic runs of the SAME
      // engine at two different step grids must anchor at the identical
      // cycle index; a nonzero offset here would mean the anchor state
      // recurred by coincidence, not that the runs are truly aligned.
      expect(check.offsetCycles, `${name}: expected alignment offset 0 between two classic runs of the same engine`).toBe(0);
      expect(check.mismatchCount, `${name}: expected zero mismatches between adaptive and refined-fixed-step classic`).toBe(0);
      expect(check.comparedCycles, `${name}: expected at least 2 compared cycles`).toBeGreaterThanOrEqual(2);
    },
  );
});
