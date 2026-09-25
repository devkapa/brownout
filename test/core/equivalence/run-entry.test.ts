/**
 * Unit coverage for run-entry.ts's gate 2d stimuli path: an entry with
 * `phases` takes priority over the plain not-ported/pass/fail flow,
 * replays on classic through stimuli.ts, and reports
 * "core-stimuli-pending" once the (always-throwing, until W1b)
 * `replayStimuliOnCore` call is observed to throw
 * `StimuliNotYetSupportedOnCoreError` — never a plain "not-ported" (today)
 * or a silent "pass" (once every kind involved is core-implemented) that
 * would hide the fact the phases were never actually checked against the
 * core.
 */

import fs from "node:fs";
import os from "node:os";
import path from "node:path";
import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../../src/core/wasm.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import {
  collectClassicSamplesCached,
  EXTERNAL_CLASSIC_REFERENCE_CACHE_DIR,
  isLocalCorpusOrigin,
  QuietInstantEvidenceError,
  resolveSampleTimes,
  runCorpusEntry,
} from "./run-entry.js";
import type { CorpusEntry } from "./types.js";

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

/** vsrc -> sw -> r1 -> gnd. `switch` is NOT in CORE_IMPLEMENTED_KINDS (kinds.ts) — this circuit would be "not-ported" under the plain flow. */
function switchedDivider(): SimCircuit {
  return {
    components: [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("sw", "switch", ["a", "b"], { closed: 0 }),
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
  it("reports core-stimuli-pending for an entry with phases, instead of not-ported, even though `switch` is not core-implemented", () => {
    const entry = baseEntry({
      phases: [
        { set: [{ id: "sw", params: { closed: 1 } }], durationS: 2e-3 },
        { set: [{ id: "sw", params: { closed: 0 } }], durationS: 2e-3 },
      ],
    });

    const result = runCorpusEntry(entry, switchedDivider());

    expect(result.status).toBe("core-stimuli-pending");
    if (result.status === "core-stimuli-pending") {
      expect(result.phaseCount).toBe(2);
      expect(result.classicWallMs).not.toBeNull();
      expect(result.coreWallMs).toBeNull();
    }
  });

  it("falls through to the plain not-ported flow for the SAME circuit when the entry carries no phases", () => {
    const entry = baseEntry({ phases: undefined });
    const result = runCorpusEntry(entry, switchedDivider());
    expect(result.status).toBe("not-ported");
    if (result.status === "not-ported") {
      expect(result.missingKinds).toContain("switch");
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
    const entry = baseEntry({ phases: [] });
    const result = runCorpusEntry(entry, switchedDivider());
    expect(result.status).toBe("not-ported");
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
