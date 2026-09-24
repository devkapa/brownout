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
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../src/core/wasm.js";
import { CORE_SUPPORTED_KINDS } from "../../src/core/kinds.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { compareSamples } from "./equivalence/compare.js";
import { parseCorpusDirsEnv } from "./equivalence/external-corpus.js";
import { runEquivalenceHarness, type HarnessRun } from "./equivalence/harness.js";
import { renderMarkdownReport, toJsonReport } from "./equivalence/report.js";
import { collectClassicSamples, collectCoreSamples, DEFAULT_DURATION_S, sampleTimesFor } from "./equivalence/sample.js";
import { checkSupportedKindsGate } from "./equivalence/supported-kinds-gate.js";
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
