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
import {
  collectClassicSamplesCached,
  EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR,
  explainRichardsonMismatches,
  isLocalCorpusOrigin,
  QuietInstantEvidenceError,
  resolveSampleTimes,
  RICHARDSON_DIVERGENCE_ID,
  runCorpusEntry,
} from "./run-entry.js";
import { replayStimuliOnClassic, replayStimuliOnCore, STIMULUS_FIXED_STEP_S, type StimulusPhase } from "./stimuli.js";
import { loadSyntheticCorpus } from "./synthetic-fixtures.js";
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
 * vsrc -> led1 -> r1 -> gnd. `led` is NOT in CORE_IMPLEMENTED_KINDS
 * (kinds.ts) — this circuit is "not-ported" under the plain flow, and
 * "core-stimuli-pending" under the gate 2d stimuli path. (This "an unported
 * kind" role has moved twice now: `switch` played it until W1b put the
 * switch family into CORE_IMPLEMENTED_KINDS, then `capacitor` played it
 * until this same wave's later step landed the capacitor device/solver and
 * the orchestrator added it too; `led` — W1e, not yet ported — takes over
 * now.)
 */
function unportedKindDivider(): SimCircuit {
  return {
    components: [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("led1", "led", ["a", "b"], {}),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    wires: [
      wire("vsrc", "neg", "gnd", "gnd"),
      wire("vsrc", "pos", "led1", "a"),
      wire("led1", "b", "r1", "a"),
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
  it("reports core-stimuli-pending for an entry with phases, instead of not-ported, even though `led` is not core-implemented", () => {
    const entry = baseEntry({
      kinds: ["ground", "led", "resistor", "voltage_source"],
      phases: [
        { set: [{ id: "led1", params: { forwardVoltage: 2.2 } }], durationS: 2e-3 },
        { set: [{ id: "led1", params: { forwardVoltage: 1.8 } }], durationS: 2e-3 },
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
    const entry = baseEntry({ kinds: ["ground", "led", "resistor", "voltage_source"], phases: undefined });
    const result = runCorpusEntry(entry, unportedKindDivider());
    expect(result.status).toBe("not-ported");
    if (result.status === "not-ported") {
      expect(result.missingKinds).toContain("led");
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
    const entry = baseEntry({ kinds: ["ground", "led", "resistor", "voltage_source"], phases: [] });
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
    const coreSamples = replayStimuliOnCore(circuit, corePhases);
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
    const coreSamples = replayStimuliOnCore(fixture13.circuit, phases);

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
    const coreSamples = replayStimuliOnCore(fixture13.circuit, phases);
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
