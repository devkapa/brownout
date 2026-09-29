/**
 * Unit coverage for run-entry.ts's gate 2d stimuli path: an entry with
 * `phases` takes priority over the plain not-ported/pass/fail flow, replays
 * on classic through stimuli.ts, and — since W1b landed
 * `bo_engine_set_params` and put the switch family into
 * `CORE_IMPLEMENTED_KINDS` — now gets a REAL classic-vs-core comparison
 * (pass/fail) whenever every kind involved is core-implemented, or
 * "core-stimuli-pending" (never a plain "not-ported", and never a silent
 * "pass" that skipped the core side) whenever it is not.
 */

import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../../src/core/wasm.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { alignCycles, compareAlignedCycles, sampleCycles, type AlignmentSuccess } from "./align.js";
import { compareStimuliSamples } from "./compare.js";
import { extractNe555Sidecar } from "./analytic.js";
import {
  CACHE_AGREEMENT_VOLTS,
  checkNe555Periods,
  closedSegments,
  collectClassicSamplesCached,
  collectCoreDigitalTraceWithPhases,
  compareCacheOnOff,
  compareNe555Sidecar,
  EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR,
  explainRichardsonMismatches,
  isLocalCorpusOrigin,
  levelChanges,
  ne555LagSurface,
  ne555LagTimes,
  ne555Midpoints,
  ne555SupplyResidual,
  QuietInstantEvidenceError,
  resolveSampleTimes,
  RICHARDSON_DIVERGENCE_ID,
  runCorpusEntry,
  runSidecarChecks,
} from "./run-entry.js";
import { collectClassicSamples, collectCoreSamples } from "./sample.js";
import { replayStimuliOnClassic, replayStimuliOnCore, STIMULUS_FIXED_STEP_S, type StimulusPhase } from "./stimuli.js";
import { loadSyntheticCorpus } from "./synthetic-fixtures.js";
import { CoreEngine } from "../../../src/core/engine.js";
import { ledRcVoltages } from "./analytic.js";
import { collectClassicDigitalTrace, collectCoreDigitalTrace } from "./trace.js";
import type { CorpusEntry } from "./types.js";

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

/**
 * vsrc -> d1 -> r1 -> gnd. `diode` is NOT in CORE_IMPLEMENTED_KINDS
 * (kinds.ts) — this circuit is "not-ported" under the plain flow, and
 * "core-stimuli-pending" under the gate 2d stimuli path. (This "an unported
 * kind" role has moved three times now: `switch` played it until W1b put the
 * switch family into CORE_IMPLEMENTED_KINDS, then `capacitor` played it
 * until this same wave's later step landed the capacitor device/solver and
 * the orchestrator added it too; then `led` played it until W1e ported it,
 * and `diode` — in no wave's kind table — takes over now.)
 */
function unportedKindDivider(): SimCircuit {
  return {
    components: [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("d1", "diode", ["a", "k"], {}),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    wires: [
      wire("vsrc", "neg", "gnd", "gnd"),
      wire("vsrc", "pos", "d1", "a"),
      wire("d1", "k", "r1", "a"),
      wire("r1", "b", "gnd", "gnd"),
    ],
  };
}

/**
 * vsrc -> sw -> r1 -> gnd. `switch` IS in CORE_IMPLEMENTED_KINDS (kinds.ts,
 * W1b) — an entry toggling `sw` across phases now gets a REAL classic-vs-core
 * comparison through `runStimuliEntry`, not "core-stimuli-pending".
 */
function switchedDivider(closed: number | string = 0): SimCircuit {
  return {
    components: [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("sw", "switch", ["a", "b"], { closed }),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    wires: [
      wire("vsrc", "neg", "gnd", "gnd"),
      wire("vsrc", "pos", "sw", "a"),
      wire("sw", "b", "r1", "a"),
      wire("r1", "b", "gnd", "gnd"),
    ],
  };
}

// Some cases below (finding 10's end-to-end test) reach a real CoreEngine.
beforeAll(async () => {
  await initCore();
});

function baseEntry(overrides: Partial<CorpusEntry> = {}): CorpusEntry {
  return {
    id: "test/switched-divider",
    origin: "test",
    name: "switched-divider",
    durationS: null,
    kinds: ["ground", "resistor", "switch", "voltage_source"],
    stimulus: "verify-script",
    file: "/tmp/switched-divider.sim.json",
    ...overrides,
  };
}

describe("runCorpusEntry — gate 2d stimuli priority", () => {
  it("reports core-stimuli-pending for an entry with phases, instead of not-ported, even though `diode` is not core-implemented", () => {
    const entry = baseEntry({
      kinds: ["ground", "diode", "resistor", "voltage_source"],
      phases: [
        { set: [{ id: "d1", params: { forwardVoltage: 2.2 } }], durationS: 2e-3 },
        { set: [{ id: "d1", params: { forwardVoltage: 1.8 } }], durationS: 2e-3 },
      ],
    });

    const result = runCorpusEntry(entry, unportedKindDivider());

    expect(result.status).toBe("core-stimuli-pending");
    if (result.status === "core-stimuli-pending") {
      expect(result.phaseCount).toBe(2);
      expect(result.classicWallMs).not.toBeNull();
      expect(result.coreWallMs).toBeNull();
    }
  });

  it("falls through to the plain not-ported flow for the SAME circuit when the entry carries no phases", () => {
    const entry = baseEntry({ kinds: ["ground", "diode", "resistor", "voltage_source"], phases: undefined });
    const result = runCorpusEntry(entry, unportedKindDivider());
    expect(result.status).toBe("not-ported");
    if (result.status === "not-ported") {
      expect(result.missingKinds).toContain("diode");
    }
  });

  it("reports an error, not a silent pass or a thrown exception, when the classic-side replay itself fails", () => {
    const shorted: SimCircuit = {
      components: [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("gnd", "ground", ["gnd"])],
      wires: [wire("vsrc", "pos", "gnd", "gnd"), wire("vsrc", "neg", "gnd", "gnd")],
    };
    const entry = baseEntry({ kinds: ["ground", "voltage_source"], phases: [{ durationS: 1e-3 }] });

    const result = runCorpusEntry(entry, shorted);

    expect(result.status).toBe("error");
    if (result.status === "error") {
      expect(result.engine).toBe("classic");
    }
  });

  it("an empty phases array does not trigger the stimuli path (falls through to the plain flow)", () => {
    const entry = baseEntry({ kinds: ["ground", "diode", "resistor", "voltage_source"], phases: [] });
    const result = runCorpusEntry(entry, unportedKindDivider());
    expect(result.status).toBe("not-ported");
  });

  // ── W1b: `switch`/`push_button`/`spdt_switch`/`dip_switch` joined
  // CORE_IMPLEMENTED_KINDS and CoreEngine.setParams landed, so an entry whose
  // kinds are ALL core-implemented now gets a REAL classic-vs-core stimuli
  // comparison here, not just a "core-stimuli-pending" bookkeeping gesture. ──

  it("a switch-only stimulated circuit (a switch toggling a resistor divider across two phases) passes classic vs core", () => {
    const entry = baseEntry({
      kinds: ["ground", "resistor", "switch", "voltage_source"],
      phases: [
        { set: [{ id: "sw", params: { closed: 1 } }], durationS: 2e-3 },
        { set: [{ id: "sw", params: { closed: 0 } }], durationS: 2e-3 },
      ],
    });

    const result = runCorpusEntry(entry, switchedDivider(0));

    expect(result.status).toBe("pass");
    if (result.status === "pass") {
      expect(result.classicWallMs).not.toBeNull();
      expect(result.coreWallMs).not.toBeNull();
      expect(result.maxAbsErrorAmps).toBeLessThan(1e-6);
    }
  });

  it("a seeded fault (a phase applied only on the classic side) fails and names the phase index it diverged at", () => {
    // Both sides start from the SAME circuit (switch open) and phase 0 is a
    // no-op on both — real physics, identical on both engines. Phase 1
    // diverges ONLY in what gets applied: classic closes the switch, core's
    // corresponding phase stays a no-op. Neither replay is tampered — the
    // fault is a real, deliberate difference in what stimulus each engine
    // actually received, so the resulting mismatch is a genuine disagreement
    // for compareStimuliSamples (compare.ts) to catch, not a fabricated one.
    const circuit = switchedDivider(0);
    const noOpPhase: StimulusPhase = { durationS: 2 * STIMULUS_FIXED_STEP_S };
    const classicPhases: StimulusPhase[] = [
      noOpPhase,
      { set: [{ id: "sw", params: { closed: 1 } }], durationS: 2 * STIMULUS_FIXED_STEP_S },
    ];
    const corePhases: StimulusPhase[] = [noOpPhase, noOpPhase];

    const classicSamples = replayStimuliOnClassic(circuit, classicPhases);
    // Both phase arrays share the same per-phase durationS (only their
    // `set` overrides differ, which is the deliberate seeded fault below) —
    // classicSamples' timing is a valid schedule for corePhases too.
    const coreSamples = replayStimuliOnCore(circuit, corePhases, classicSamples);
    const cmp = compareStimuliSamples(classicSamples, coreSamples);

    expect(cmp.mismatches.length).toBeGreaterThan(0);
    // Phase 0 ran identically (switch stayed open) on both sides.
    expect(cmp.mismatches.some((m) => m.phaseIndex === 0)).toBe(false);
    // Phase 1 is where the two sides' stimuli actually diverged.
    const phase1Mismatches = cmp.mismatches.filter((m) => m.phaseIndex === 1);
    expect(phase1Mismatches.length).toBeGreaterThan(0);
    expect(phase1Mismatches.some((m) => m.category === "elementI" && m.id === "sw")).toBe(true);
  });

  it("a phase that sets a param which exists but is not live reports 'error', never a silent pass or 'core-stimuli-pending'", () => {
    // r1 (resistor) has no live params (CORE_RECORD_SHAPES.resistor.liveParams
    // is empty) — CoreEngine.setParams must reject this loudly, and
    // runStimuliEntry must surface that rejection as a real "error" entry,
    // not fold it into "not yet supported" (that status is reserved for
    // StimuliNotYetSupportedOnCoreError, an unrelated failure mode). The
    // override is on phase 1, not phase 0: phase 0's own `set` is folded into
    // the circuit before the initial load (investigation point 2, finding
    // 12), where "live" is not a constraint at all — only a LATER phase's
    // `set` ever reaches CoreEngine.setParams's liveness check.
    const entry = baseEntry({
      kinds: ["ground", "resistor", "switch", "voltage_source"],
      phases: [
        { durationS: 1e-3 },
        { set: [{ id: "r1", params: { resistance: 5000 } }], durationS: 1e-3 },
      ],
    });

    const result = runCorpusEntry(entry, switchedDivider(1));

    expect(result.status).toBe("error");
    if (result.status === "error") {
      expect(result.engine).toBe("core");
      expect(result.message).toMatch(/live/i);
      expect(result.classicWallMs).not.toBeNull();
    }
  });
});

/** vsrc -> r1 -> gnd. Every kind here is in CORE_IMPLEMENTED_KINDS (kinds.ts), so this reaches the plain pass/fail flow instead of "not-ported". */
function simpleDivider(): SimCircuit {
  return {
    components: [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    wires: [wire("vsrc", "neg", "gnd", "gnd"), wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "gnd", "gnd")],
  };
}

describe("finding 11 — Richardson-extrapolated forgiveness for the stimuli path", () => {
  // Fixture 13 (test/fixtures/core/13-rc-switch-toggle), loaded exactly as
  // the harness itself loads it (loadSyntheticCorpus, finding 12) — starts
  // OPEN and declares phases close-20ms-then-open-20ms, so this exercises
  // BOTH toggle directions on the section-11 circuit itself, not a
  // hand-rolled stand-in for it.
  const fixture13 = loadSyntheticCorpus().find((e) => e.entry.name === "13-rc-switch-toggle")!;

  it("the section-11 circuit (fixture 13's RC, close 20ms then open 20ms) passes gate 2d with every mismatch explained", () => {
    expect(fixture13.entry.phases?.length, "fixture 13 must declare phases for this test to mean anything").toBe(2);

    const result = runCorpusEntry(fixture13.entry, fixture13.circuit);

    expect(result.status).toBe("pass");
    if (result.status === "pass") {
      // Not a vacuous pass: compareStimuliSamples must have actually found
      // something for Richardson to explain, or this test would look
      // identical whether or not the detector does anything at all.
      expect(result.knownDivergences.length).toBeGreaterThan(0);
      expect(result.knownDivergences.every((hit) => hit.divergenceId === RICHARDSON_DIVERGENCE_ID)).toBe(true);
    }
  });

  it("a seeded core fault (one core sample offset by 10 mV) is NOT explained away — the detector must still fire", () => {
    const phases = fixture13.entry.phases!;
    const classicSamples = replayStimuliOnClassic(fixture13.circuit, phases);
    const coreSamples = replayStimuliOnCore(fixture13.circuit, phases, classicSamples);

    // Perturb the LAST sample's netV on every net by 10mV — comfortably
    // outside both the flat tolerance and any plausible Richardson estimate
    // near it, so this cannot pass by coincidence.
    const tamperedIndex = coreSamples.length - 1;
    const tampered = coreSamples.map((sample, i) =>
      i === tamperedIndex
        ? { ...sample, netV: Object.fromEntries(Object.entries(sample.netV).map(([k, v]) => [k, v + 0.01])) }
        : sample,
    );

    const cmp = compareStimuliSamples(classicSamples, tampered);
    // classic and core simTime labels agree closely (finding 10) but are not
    // necessarily bit-identical floats (independent accumulation on each
    // engine), so this matches by phase + a real time tolerance, not `===`.
    const taggedTime = tampered[tamperedIndex]!.simTime;
    const taggedPhase = tampered[tamperedIndex]!.phaseIndex;
    const seededMismatches = cmp.mismatches.filter(
      (m) => m.category === "netV" && m.phaseIndex === taggedPhase && Math.abs(m.sampleTime - taggedTime) < 1e-9,
    );
    expect(seededMismatches.length, "the 10mV perturbation itself must surface as a netV mismatch, or this test proves nothing").toBeGreaterThan(0);

    const { explained, remaining } = explainRichardsonMismatches(fixture13.circuit, phases, cmp.mismatches);
    for (const seeded of seededMismatches) {
      expect(remaining, `mismatch on ${seeded.id} at t=${seeded.sampleTime}`).toContainEqual(seeded);
      expect(explained.some((hit) => hit.mismatch === seeded)).toBe(false);
    }
  });

  it("leaves digitalState/simTime mismatches untouched, even when a matching netV/elementI mismatch at the same sample IS explained", () => {
    const phases = fixture13.entry.phases!;
    const classicSamples = replayStimuliOnClassic(fixture13.circuit, phases);
    const coreSamples = replayStimuliOnCore(fixture13.circuit, phases, classicSamples);
    const cmp = compareStimuliSamples(classicSamples, coreSamples);

    // Fabricate a digitalState mismatch alongside the real (Richardson-
    // explainable) netV/elementI ones this circuit actually produces.
    const fabricated = {
      sampleTime: cmp.mismatches[0]?.sampleTime ?? classicSamples[0]!.simTime,
      category: "digitalState" as const,
      id: "not-a-real-key",
      classicValue: 0,
      coreValue: 1,
      detail: "fabricated for this test",
      phaseIndex: 0,
    };

    const { explained, remaining } = explainRichardsonMismatches(fixture13.circuit, phases, [...cmp.mismatches, fabricated]);
    expect(remaining).toContainEqual(fabricated);
    expect(explained.some((hit) => hit.mismatch === fabricated)).toBe(false);
  });
});

describe("resolveSampleTimes — finding 10 (quiet-instant selection must not admit everything by default)", () => {
  it("throws when an analog entry has no digital-event evidence at all (guardEventTimes empty, no trace was ever attempted)", () => {
    const entry = baseEntry({ phases: undefined, analog: { minQuietInstants: 2, maxDelayS: 0 } });
    expect(() => resolveSampleTimes(entry, simpleDivider(), 0.01, [], undefined)).toThrow(QuietInstantEvidenceError);
  });

  it("throws when an analog entry's schedule outruns the span its event-time evidence actually covers", () => {
    const entry = baseEntry({ phases: undefined, analog: { minQuietInstants: 2, maxDelayS: 0 } });
    // Real event times exist, but only up to 0.5s — the analog schedule below asks for 1s.
    expect(() => resolveSampleTimes(entry, simpleDivider(), 1, [0.1, 0.2], 0.5)).toThrow(QuietInstantEvidenceError);
  });

  it("does not throw, and returns quiet instants, when real event-time evidence covers the whole schedule", () => {
    const entry = baseEntry({ phases: undefined, analog: { minQuietInstants: 2, maxDelayS: 0 } });
    // Events far outside [0, 0.01] never collide with the dense schedule's own candidates.
    const times = resolveSampleTimes(entry, simpleDivider(), 0.01, [-10, -9], 0.01);
    expect(times.length).toBeGreaterThanOrEqual(2);
  });

  it("a plain (non-analog) entry is unaffected — still returns the fixed 4-point default with no event evidence required", () => {
    const entry = baseEntry({ phases: undefined, analog: undefined });
    const times = resolveSampleTimes(entry, simpleDivider(), 0.01, [], undefined);
    expect(times).toEqual([0, 0.0025, 0.005, 0.01]);
  });

  /**
   * finding 10 (end to end): TODAY, `collectCoreDigitalTrace` always throws
   * `CoreTraceUnsupportedError` (no CoreEngine in this build implements the
   * trace ABI yet — trace.ts's own doc), so ANY entry that declares both
   * `digital` and `analog` gets zero real event-time evidence. Before the
   * fix, `runCorpusEntry` would have silently admitted every dense-schedule
   * candidate as "quiet" and produced a normal pass/fail. Now it refuses.
   */
  it("end-to-end via runCorpusEntry: an entry declaring both `digital` and `analog` refuses instead of silently comparing on every candidate, since this build's core trace ABI is unsupported", () => {
    const entry = baseEntry({
      phases: undefined,
      kinds: ["ground", "resistor", "voltage_source"],
      analog: { minQuietInstants: 2, maxDelayS: 0 },
      digital: { clockKey: "nonexistent", anchor: {}, offsetBoundCycles: 6, minComparedCycles: 2 },
    });
    expect(() => runCorpusEntry(entry, simpleDivider())).toThrow(QuietInstantEvidenceError);
  });
});

describe("classic reference cache location — finding 12 (an external corpus's derived data must never land inside the checkout)", () => {
  function listFilesRecursive(dir: string): string[] {
    if (!fs.existsSync(dir)) return [];
    const out: string[] = [];
    for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
      const full = path.join(dir, entry.name);
      if (entry.isDirectory()) out.push(...listFilesRecursive(full));
      else out.push(full);
    }
    return out;
  }

  const REPO_LOCAL_CACHE_DIR = path.join(process.cwd(), "node_modules", ".cache", "core-equivalence-classic-refs");
  const newFilesWritten: string[] = [];

  afterEach(() => {
    for (const file of newFilesWritten.splice(0)) fs.rmSync(file, { force: true });
  });

  it("isLocalCorpusOrigin: true only for the two fixed local literals CorpusEntry.origin documents", () => {
    expect(isLocalCorpusOrigin("brownout-fixture")).toBe(true);
    expect(isLocalCorpusOrigin("synthetic")).toBe(true);
    expect(isLocalCorpusOrigin("devolt")).toBe(false);
    expect(isLocalCorpusOrigin("some-other-producer")).toBe(false);
  });

  it("a LOCAL fixture's classic run is cached inside this repo's own gitignored node_modules/.cache", () => {
    const before = new Set(listFilesRecursive(REPO_LOCAL_CACHE_DIR));
    const entry = baseEntry({ id: `test/finding-12-local-${Date.now()}`, origin: "synthetic", phases: undefined });
    collectClassicSamplesCached(entry, simpleDivider(), [0]);
    const after = listFilesRecursive(REPO_LOCAL_CACHE_DIR);
    const created = after.filter((f) => !before.has(f));
    expect(created.length).toBeGreaterThan(0);
    newFilesWritten.push(...created);
  });

  it("finding 12: an EXTERNAL corpus entry's classic run is never written under this repo — it lands under the OS temp dir instead", () => {
    const repoTreeBefore = new Set(listFilesRecursive(REPO_LOCAL_CACHE_DIR));
    const externalTreeBefore = new Set(listFilesRecursive(EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR));

    const entry = baseEntry({ id: `devolt/finding-12-external-${Date.now()}`, origin: "devolt", phases: undefined });
    collectClassicSamplesCached(entry, simpleDivider(), [0]);

    const repoTreeAfter = listFilesRecursive(REPO_LOCAL_CACHE_DIR);
    expect(repoTreeAfter.filter((f) => !repoTreeBefore.has(f))).toHaveLength(0);

    const externalTreeAfter = listFilesRecursive(EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR);
    const created = externalTreeAfter.filter((f) => !externalTreeBefore.has(f));
    expect(created.length).toBeGreaterThan(0);
    expect(created.every((f) => f.startsWith(os.tmpdir()))).toBe(true);
    newFilesWritten.push(...created);
  });
});

/**
 * Fix 7 (review): fixture 19 (the 74HC14 relaxation oscillator) now declares
 * `digital` (decision 6's per-cycle alignment), which is only meaningful now
 * that `CoreEngine` implements the trace ABI (trace.ts's `enableTrace`/
 * `readTrace`/`clearTrace`) — before that it would have downgraded to
 * "unsupported" and compared nothing. These tests use the fixture's REAL
 * circuit and its own declared clockKey/anchor/offsetBoundCycles (never
 * synthetic dummy keys — align.test.ts already covers the alignment
 * machinery generically with synthetic traces) to prove: (a) the real
 * classic-vs-core alignment genuinely passes with real per-cycle data, and
 * (b) a seeded fault — the core's trace missing one whole toggle, exactly
 * what an oscillator with a wrong period/threshold would produce — is
 * actually caught, not silently absorbed by the alignment tolerance.
 */
describe("gate 2a real fixture — 19-74hc14-relaxation-oscillator (fix 7)", () => {
  const fixture19 = loadSyntheticCorpus().find((e) => e.entry.name === "19-74hc14-relaxation-oscillator")!;
  const spec = fixture19.entry.digital!;

  it("fixture 19 declares a digital spec, so it exercises the real per-cycle alignment, not the downgraded path", () => {
    expect(spec, "fixture 19 must declare `digital` for this describe block to mean anything").toBeDefined();
    expect(spec.clockKey).toBe("u1/1y");
  });

  it("aligns cleanly at offset 0 with zero mismatches, across at least the fixture's declared minimum cycles", () => {
    const durationS = spec.traceDurationS!;
    const classicTrace = collectClassicDigitalTrace(fixture19.circuit, durationS);
    const coreTrace = collectCoreDigitalTrace(fixture19.circuit, durationS);

    const classicCycles = sampleCycles(classicTrace, spec.clockKey, spec.projection);
    const coreCycles = sampleCycles(coreTrace, spec.clockKey, spec.projection);
    const alignment = alignCycles(classicCycles, coreCycles, spec.anchor, spec.offsetBoundCycles);
    expect(alignment.ok, alignment.ok ? "" : alignment.reason).toBe(true);

    const comparison = compareAlignedCycles(classicCycles, coreCycles, alignment as AlignmentSuccess);
    expect(comparison.mismatches).toEqual([]);
    expect(comparison.comparedCycles).toBeGreaterThanOrEqual(spec.minComparedCycles);
  });

  it("a seeded period error — one cycle's real, sampled state flipped — fails the per-cycle comparison", () => {
    const durationS = spec.traceDurationS!;
    const classicTrace = collectClassicDigitalTrace(fixture19.circuit, durationS);
    const coreTrace = collectCoreDigitalTrace(fixture19.circuit, durationS);

    const classicCycles = sampleCycles(classicTrace, spec.clockKey, spec.projection);
    const realCoreCycles = sampleCycles(coreTrace, spec.clockKey, spec.projection);
    expect(realCoreCycles.length).toBeGreaterThan(spec.minComparedCycles);

    // Seed the fault on the REAL, already-sampled core cycles (not the raw
    // event list): a single-key oscillator alternates 0/1 every cycle by
    // construction, so dropping a raw trace EVENT merges two half-cycles
    // instead of producing a per-cycle state mismatch — mirroring align.
    // test.ts's own "seeded fault: a bit flip" pattern (synthetic there;
    // this fixture's real, collected cycles here) is what actually exercises
    // what compareAlignedCycles inspects: cycle N (not compared to the
    // t=0/anchor cycle) has ITS state flipped, as if the core reported the
    // wrong logic level for one whole cycle — exactly what a wrong period or
    // wrong Schmitt threshold would produce.
    const flipIndex = Math.min(3, realCoreCycles.length - 1);
    const flippedCycle = realCoreCycles[flipIndex]!;
    const seededCoreCycles = realCoreCycles.map((c, i) =>
      i === flipIndex ? { ...c, state: { [spec.clockKey]: flippedCycle.state[spec.clockKey] === 1 ? 0 : 1 } } : c,
    );

    const alignment = alignCycles(classicCycles, seededCoreCycles, spec.anchor, spec.offsetBoundCycles);
    expect(alignment.ok, "the anchor (cycle 0) is untouched by flipping a LATER cycle").toBe(true);

    const comparison = compareAlignedCycles(classicCycles, seededCoreCycles, alignment as AlignmentSuccess);
    expect(comparison.mismatches.length, "a flipped cycle must surface as a real per-cycle mismatch, never be silently absorbed").toBeGreaterThan(0);
  });
});


// ─── W1e: NE555 entries (core/README.md NE555 point 13) and LED sidecars ──────
//
// Fixtures 31-36 (test/fixtures/core) run through the real harness in
// equivalence.test.ts; what is proven here is that each detector can FAIL: a
// clean sweep proves nothing unless a seeded fault turns it red.

function syntheticEntry(name: string) {
  const found = loadSyntheticCorpus().find((e) => e.entry.name === name);
  if (!found) throw new Error(`no synthetic fixture named ${name}`);
  return found;
}

describe("NE555 point 13.2 — the period gate, and the detector that must be able to fail", () => {
  const { entry, circuit } = syntheticEntry("31-ne555-astable-led");
  const spec = { component: "u1", mode: "astable", capacitor: "ct" } as const;
  const durationS = entry.durationS!;

  it("levelChanges drops repeats and absent (NaN) events; closedSegments leaves the open last level out; ne555Midpoints skips a runt and closes a monostable's tail", () => {
    const trace = [
      { t: 0, key: "u1", value: 1 },
      { t: 0, key: "other", value: 1 },
      { t: 1e-3, key: "u1", value: 1 },
      { t: 2e-3, key: "u1", value: 0 },
      { t: 3e-3, key: "u1", value: Number.NaN },
      { t: 3.000001e-3, key: "u1", value: 1 },
      { t: 3.000002e-3, key: "u1", value: 0 },
      { t: 5e-3, key: "u1", value: 1 },
    ];
    const changes = levelChanges(trace, "u1");
    expect(changes.map((c) => [c.t, c.high])).toEqual([[0, true], [2e-3, false], [3.000001e-3, true], [3.000002e-3, false], [5e-3, true]]);
    const segments = closedSegments(changes);
    expect(segments).toHaveLength(4);
    // the 1 us pulse between 3.000001 and 3.000002 ms is not a phase; the trailing open HIGH is not complete
    const mids = ne555Midpoints(changes, 6e-3, 1e-5, false);
    expect(mids).toHaveLength(3);
    [1e-3, 2.5000005e-3, 4.000001e-3].forEach((want, i) => expect(mids[i]).toBeCloseTo(want, 12));
    // a monostable's last LOW is open at the end of the run: its midpoint to the horizon is an instant too
    const mono = levelChanges([{ t: 0, key: "u1", value: 1 }, { t: 0.1, key: "u1", value: 0 }], "u1");
    expect(ne555Midpoints(mono, 0.3, 1e-5, true)).toEqual([0.05, 0.2]);
    expect(ne555Midpoints(mono, 0.3, 1e-5, false)).toEqual([0.05]);
  });

  it("the core's real trace of the SAP-1 clock passes against the model-exact sidecar, every phase and period to 1e-6", () => {
    const sidecar = extractNe555Sidecar(circuit, spec, durationS);
    const changes = levelChanges(collectCoreDigitalTrace(circuit, durationS), "u1");
    expect(checkNe555Periods({ key: "u1", changes, phases: sidecar.phases, minPhases: 16 })).toEqual([]);
  });

  it("the detector: a sidecar that drops RA from the discharge Thevenin (2.086374 ms, off by 0.7%) fails against the very same core trace", () => {
    const wrong = extractNe555Sidecar(circuit, spec, durationS, { dischargeDropsResistor: "ra" });
    const changes = levelChanges(collectCoreDigitalTrace(circuit, durationS), "u1");
    const mismatches = checkNe555Periods({ key: "u1", changes, phases: wrong.phases, minPhases: 16 });
    expect(mismatches.length).toBeGreaterThan(0);
    expect(mismatches.every((m) => m.category === "period")).toBe(true);
    expect(mismatches.some((m) => /LOW #1/.test(m.id))).toBe(true);
    expect(mismatches.some((m) => /period/.test(m.id))).toBe(true);
  });

  it("the detector: classic's own trace (its period is 2.1349 ms, +1.6%) fails the same gate, which is why classic's are recorded and not gated", () => {
    const sidecar = extractNe555Sidecar(circuit, spec, durationS);
    const changes = levelChanges(collectClassicDigitalTrace(circuit, durationS), "u1");
    const mismatches = checkNe555Periods({ key: "u1", changes, phases: sidecar.phases, minPhases: 16 });
    expect(mismatches.length).toBeGreaterThan(0);
    expect(mismatches.map((m) => m.id).join(" ")).toMatch(/first HIGH/);
  });

  it("too few completed phases, and a starting level that disagrees with the sidecar, are mismatches of their own", () => {
    const sidecar = extractNe555Sidecar(circuit, spec, durationS);
    const short = checkNe555Periods({ key: "u1", changes: [{ t: 0, high: true }, { t: 2.19722569e-3, high: false }], phases: sidecar.phases, minPhases: 5 });
    expect(short.map((m) => m.id)).toEqual(["u1: phases"]);
    const flipped = checkNe555Periods({ key: "u1", changes: [{ t: 0, high: false }, { t: 1e-3, high: true }], phases: sidecar.phases, minPhases: 0 });
    expect(flipped.map((m) => m.id)).toEqual(["u1: initial level"]);
  });
});

describe("NE555 point 13.4 — the timing network against the sidecar, and the supply residual", () => {
  const { circuit } = syntheticEntry("31-ne555-astable-led");
  const sidecar = extractNe555Sidecar(circuit, { component: "u1", mode: "astable", capacitor: "ct" }, 0.02);
  const t = 3.6057245e-3; // mid-HIGH of the second cycle, on the core's own clock

  it("the core's own readout at that instant agrees with the sidecar's exponentials (no mismatch)", () => {
    expect(compareNe555Sidecar(sidecar, collectCoreSamples(circuit, [t]))).toEqual([]);
  });

  it("the detector: a capacitor node 0.5 V off, or a timing resistor current 10% off, is a sidecar mismatch naming it", () => {
    const [sample] = collectCoreSamples(circuit, [t]);
    const capNet = [...sidecar.netIds].sort()[0]!;
    const netTampered = { ...sample!, netV: { ...sample!.netV, [capNet]: sample!.netV[capNet]! + 0.5 } };
    expect(compareNe555Sidecar(sidecar, [netTampered]).map((m) => m.id)).toEqual([`netV.${capNet}`]);
    const currentTampered = { ...sample!, elementI: { ...sample!.elementI, ra: sample!.elementI.ra! * 1.1 } };
    expect(compareNe555Sidecar(sidecar, [currentTampered]).map((m) => m.id)).toEqual(["elementI.ra"]);
  });

  it("ne555SupplyResidual adds the timing resistors' current leaving the source's + net back: the supply current with the charging branch removed", () => {
    const adjust = ne555SupplyResidual(circuit, sidecar.resistors);
    // vcc delivers 14.87 mA of which 1.1785 mA is RA's: the raw -0.0148738 A becomes -0.0136953 A.
    const out = adjust({ vcc: -0.0148738, ra: 0.0011785, rb: 0.0011785 });
    expect(out.vcc).toBeCloseTo(-0.0136953, 7);
    expect(out.ra).toBe(0.0011785); // the resistor's own current is left for the sidecar
    expect(adjust({ ra: 1 }).vcc).toBeUndefined();
  });
});

describe("NE555 entries in runCorpusEntry", () => {
  it("an entry with a 555 and no declared spec gets one from its wiring: fixture 31 without its `ne555` field still passes, on the derived sidecar", () => {
    const { entry, circuit } = syntheticEntry("31-ne555-astable-led");
    const auto: CorpusEntry = { ...entry, ne555: undefined };
    const result = runCorpusEntry(auto, circuit);
    expect(result.status).toBe("pass");
  });

  it("a declared fixture must name the 555 in its digital projection (point 13.1): one with no `digital` spec fails, naming that", () => {
    const { entry, circuit } = syntheticEntry("31-ne555-astable-led");
    const result = runCorpusEntry({ ...entry, digital: undefined }, circuit);
    expect(result.status).toBe("fail");
    if (result.status === "fail") expect(result.mismatches.some((m) => /NE555 point 13\.1/.test(m.detail))).toBe(true);
  });

  it("records classic's period drift against the sidecar as known divergences, never as failures", () => {
    const { entry, circuit } = syntheticEntry("31-ne555-astable-led");
    const result = runCorpusEntry(entry, circuit);
    expect(result.status).toBe("pass");
    if (result.status === "pass") {
      const drift = result.knownDivergences.filter((h) => h.divergenceId === "classic-555-period-drift");
      expect(drift.length).toBeGreaterThan(0);
      expect(drift[0]!.mismatch.detail).toMatch(/against the sidecar/);
    }
  });

  it("collectCoreDigitalTraceWithPhases follows a scripted trigger: fixture 36's second pulse starts at the button press, 0.5 s", () => {
    const { entry, circuit } = syntheticEntry("36-ne555-monostable-retrigger");
    const changes = levelChanges(collectCoreDigitalTraceWithPhases(circuit, entry.phases!), "u1");
    expect(changes.map((c) => c.high)).toEqual([true, false, true, false]);
    expect(changes[2]!.t).toBeCloseTo(0.5, 9);
    // the retriggered pulse runs from v0 = V_inf,d: 0.109851238 s
    expect(changes[3]!.t - changes[2]!.t).toBeCloseTo(0.109851238, 8);
  });

  it("ne555LagSurface reaches OUT and DIS through passives and stops at fixed nodes: fixture 32's timing parts are in, the supply is added, nothing past an IC is", () => {
    const { circuit } = syntheticEntry("32-ne555-monostable");
    const surface = ne555LagSurface(circuit, ["u1"]);
    expect([...surface.components].sort()).toEqual(["ct", "rl", "rt", "u1", "vcc"]);
    expect(surface.nets.size).toBe(2); // OUT, and DIS = THR
  });
});

describe("LEDs point 7 and 13 — the sidecar and the cache detectors", () => {
  const { entry, circuit } = syntheticEntry("34-led-family-static");

  it("the sidecar check of the static LED fixture passes against the core and classic's channel currents", () => {
    expect(runSidecarChecks(entry, circuit).mismatches).toEqual([]);
  });

  it("the detector: a sidecar with one cell's orientation flipped, or a burnt part that is not, is a mismatch", () => {
    const model = entry.sidecar!;
    if (model.kind !== "led-static") throw new Error("fixture 34 declares a led-static sidecar");
    const flipped = { ...entry, sidecar: { ...model, cells: model.cells.map((c, i) => (i === 0 ? { ...c, orientation: "cathode-node" as const } : c)) } };
    expect(runSidecarChecks(flipped, circuit).mismatches.length).toBeGreaterThan(0);
    // the same circuit with the burnt rgb_led un-burnt: the core now publishes channels the sidecar says are absent
    const unburnt: SimCircuit = { ...circuit, components: circuit.components.map((c) => (c.id === "bu_rgb" ? { ...c, params: { burnt: 0 } } : c)) };
    expect(runSidecarChecks(entry, unburnt).mismatches.some((m) => /channels\.bu_rgb/.test(m.id))).toBe(true);
  });

  it("the RC-fed LED is checked under trapezoidal integration against the ODE at mid-transient instants: passes, and the ODE tells a 2 kohm core from a 1 kohm one", () => {
    const rc = syntheticEntry("35-led-rc-stepped");
    expect(runSidecarChecks(rc.entry, rc.circuit).mismatches).toEqual([]);
    const model = rc.entry.sidecar!;
    if (model.kind !== "led-rc") throw new Error("fixture 35 declares a led-rc sidecar");
    // The core run on a circuit whose R is doubled, against the ODE of the original: mid-transient the two differ by far more than W0's tolerance.
    const doubled: SimCircuit = { ...rc.circuit, components: rc.circuit.components.map((c) => (c.id === "r1" ? { ...c, params: { resistance: 2000 } } : c)) };
    const engine = new CoreEngine();
    engine.load(doubled, undefined, { integrationMethod: "trap" });
    engine.advance(model.instantsS[3]!);
    const core = engine.getNetV()[engine.getNetIdForPin("d1", "a")!]!;
    engine.dispose();
    const ode = ledRcVoltages(rc.circuit, model, [model.instantsS[3]!])[0]!;
    expect(Math.abs(core - ode)).toBeGreaterThan(1e-2);
  });

  it("compareCacheOnOff agrees for a real LED circuit driven by a 555 (fixture 31: modes repeat, the cache hits), and the detector flags a cached sample 2 uV away", () => {
    const f31 = syntheticEntry("31-ne555-astable-led");
    const times = [3.6057245e-3, 4.6565e-3, 5.7074e-3];
    const cached = collectCoreSamples(f31.circuit, times);
    expect(compareCacheOnOff(f31.circuit, times, cached)).toEqual([]);
    const netId = Object.keys(cached[1]!.netV).find((k) => k !== "gnd")!;
    const tampered = cached.map((s, i) => (i === 1 ? { ...s, netV: { ...s.netV, [netId]: s.netV[netId]! + 2 * CACHE_AGREEMENT_VOLTS } } : s));
    const flagged = compareCacheOnOff(f31.circuit, times, tampered);
    expect(flagged.map((m) => m.id)).toEqual([`cache.netV.${netId}`]);
  });
});

describe("NE555 point 3 and 12 — an unwired RST (Q-O16), fixture 37", () => {
  const { circuit } = syntheticEntry("37-ne555-open-rst");

  it("RST reads 0 V, so OUT is LOW from load on: 0.0980392157 V across 1 kohm, 9.80392157e-5 A out of pin 3, the slot 0 and no edge in 20 ms", () => {
    const engine = new CoreEngine();
    try {
      engine.load(circuit);
      engine.advance(5e-3);
      expect(engine.getNetV()[engine.getNetIdForPin("u1", "3")!]).toBeCloseTo(0.0980392157, 9);
      expect(engine.getElementI().u1).toBeCloseTo(9.80392157e-5, 12);
      expect(engine.digitalState.u1).toBe(0);
    } finally {
      engine.dispose();
    }
    expect(levelChanges(collectCoreDigitalTrace(circuit, 0.02), "u1")).toEqual([{ t: 0, high: false }]);
  });

  it("classic publishes 4.803922 V on OUT at the load frame (its stamp lags the latch by one solve): the difference NE555 point 12 declares, which is why this fixture is sampled at quiet instants only", () => {
    const [frame] = collectClassicSamples(circuit, [0]);
    expect(frame!.netV[Object.keys(frame!.netV).find((k) => Math.abs(frame!.netV[k]! - 4.803922) < 1e-5)!]).toBeCloseTo(4.803922, 5);
  });
});

describe("classic's one-solve 555 lag (NE555 point 12)", () => {
  const at = (simTime: number, u1: number | undefined): { simTime: number; digitalState: Record<string, number> } => ({
    simTime,
    digitalState: u1 === undefined ? {} : { u1 },
  });

  it("excuses a disagreement on the 555's level that resolves at the very next sample, and only that", () => {
    const classic = [at(0, 1), at(1, 1), at(2, 0), at(3, 0)];
    const core = [at(0, 1), at(1, 0), at(2, 0), at(3, 0)];
    expect([...ne555LagTimes(["u1"], classic, core)]).toEqual([1]);
  });

  it("the detector: a standing difference (classic stays HIGH while the core is LOW), or one on the very last sample, is never excused", () => {
    const standing = ne555LagTimes(["u1"], [at(0, 1), at(1, 1), at(2, 1), at(3, 1)], [at(0, 1), at(1, 0), at(2, 0), at(3, 0)]);
    expect(standing.has(1)).toBe(false);
    expect(standing.size).toBe(0);
    expect(ne555LagTimes(["u1"], [at(0, 1), at(1, 1)], [at(0, 1), at(1, 0)]).size).toBe(0);
  });

  it("a key absent on one side counts as a disagreement (an unpowered 555 drops its key)", () => {
    expect([...ne555LagTimes(["u1"], [at(0, 1), at(1, undefined), at(2, 1)], [at(0, 1), at(1, 1), at(2, 1)])]).toEqual([1]);
  });
});

describe("NE555, classic side", () => {
  it("classic's monostable pulse ends later than the core's (fixture 32): the drift the period gate records instead of gating", () => {
    const { circuit } = syntheticEntry("32-ne555-monostable");
    const core = levelChanges(collectCoreDigitalTrace(circuit, 0.3), "u1");
    const classic = levelChanges(collectClassicDigitalTrace(circuit, 0.3), "u1");
    expect(classic[1]!.t).toBeGreaterThan(core[1]!.t);
  });
});
