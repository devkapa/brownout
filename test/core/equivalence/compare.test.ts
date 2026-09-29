/**
 * Unit coverage for compare.ts's own detection logic, using hand-built
 * synthetic samples — no engine, no wasm — per this file's own precedent
 * (compare.ts's header: "the only way to test a detector without editing src
 * to break it"). Two things are proven here that no engine-backed test can
 * cleanly isolate:
 *
 * 1. Finding 12's stamp-after-commit-lag exemption actually explains the
 *    EXACT shape the investigation found (a netV mismatch at the sample
 *    where classic's digitalState just committed an IC output, resolved by
 *    the very next sample) — and ONLY that shape.
 * 2. A mismatch that looks superficially similar (same sample, same net) but
 *    does NOT resolve by the next sample — i.e., a seeded core fault — is
 *    never excused. This is the "prove a seeded fault still fails" check the
 *    fix's own brief asks for.
 */
import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { buildCoreNodes, corePinKey } from "../../../src/core/compile/topology.js";
import { comparePhaseAlignedSamples, compareStimuliSamples, STAMP_AFTER_COMMIT_DIVERGENCE_ID } from "./compare.js";
import type { StimulusStepSample } from "./stimuli.js";
import type { SampleReadout } from "./types.js";

/** One 74LS04 inverter (u1) with its 1y output wired to a resistor (r1) — just enough topology for `digitalOutputPins("74ls04")` to recognise "1y" as an output pin and for both pins to land on one shared net. */
function inverterCircuit(): SimCircuit {
  const pins = (ids: string[]) => ids.map((id) => ({ id }));
  return {
    components: [
      {
        id: "u1",
        kind: "74ls04",
        pins: pins(["vcc", "gnd", "1a", "1y", "2a", "2y", "3a", "3y", "4a", "4y", "5a", "5y", "6a", "6y"]),
        params: {},
      },
      { id: "r1", kind: "resistor", pins: pins(["a", "b"]), params: { resistance: 1000 } },
    ],
    wires: [{ from_component: "u1", from_pin: "1y", to_component: "r1", to_pin: "a" }],
  };
}

/** The net id u1's 1y pin actually landed on, resolved the same way the harness itself would — never hardcoded, since buildCoreNodes's net-naming is an implementation detail this test must not assume. */
function y1NetId(circuit: SimCircuit): string {
  const built = buildCoreNodes(circuit);
  const node = built.pinNode.get(corePinKey("u1", "1y"));
  if (node === undefined) throw new Error("test setup: u1/1y has no node");
  const netId = built.netIdOfNode[node];
  if (netId === null || netId === undefined) throw new Error("test setup: u1/1y's node has no net id");
  return netId;
}

function sample(
  simTime: number,
  digitalOutput: number,
  netV: number,
  netId: string,
): StimulusStepSample {
  return {
    phaseIndex: 0,
    simTime,
    netV: { [netId]: netV },
    elementI: {},
    digitalState: { "u1/1y": digitalOutput },
  };
}

describe("compareStimuliSamples — finding 12's stamp-after-commit-lag exemption", () => {
  it("explains a netV mismatch exactly at a classic IC-output commit, when both sides agree again at the NEXT sample", () => {
    const circuit = inverterCircuit();
    const netId = y1NetId(circuit);

    // Sample 0: both sides agree (output HIGH, net at 5V).
    // Sample 1: classic's digitalState just committed 1y LOW, but classic's
    // OWN netV is still the pre-commit 5V (spec §2.4's lag) while the core's
    // netV is already the post-commit 0V — a real mismatch, but exactly the
    // documented, transient one.
    // Sample 2: both sides agree again (net has caught up to 0V on classic
    // too) — this is what makes sample 1's mismatch "explained".
    const classic: StimulusStepSample[] = [
      sample(0, 1, 5, netId),
      sample(0.001, 0, 5, netId),
      sample(0.002, 0, 0, netId),
    ];
    const core: StimulusStepSample[] = [
      sample(0, 1, 5, netId),
      sample(0.001, 0, 0, netId),
      sample(0.002, 0, 0, netId),
    ];

    const result = compareStimuliSamples(classic, core, circuit);

    expect(result.mismatches).toEqual([]);
    expect(result.knownDivergences).toHaveLength(1);
    expect(result.knownDivergences[0]!.divergenceId).toBe(STAMP_AFTER_COMMIT_DIVERGENCE_ID);
    expect(result.knownDivergences[0]!.mismatch.category).toBe("netV");
    expect(result.knownDivergences[0]!.mismatch.id).toBe(netId);
    expect(result.knownDivergences[0]!.mismatch.sampleTime).toBeCloseTo(0.001, 9);
    expect(result.knownDivergences[0]!.reason).toMatch(/§2\.4/);
  });

  it("does NOT explain a mismatch that persists past the next sample — a seeded core fault still fails", () => {
    const circuit = inverterCircuit();
    const netId = y1NetId(circuit);

    // Same commit-at-sample-1 shape as above, but the core's netV is wrong
    // in a way that does NOT resolve by sample 2 (a stand-in for a genuine
    // core regression, not classic's one-sample lag).
    const classic: StimulusStepSample[] = [
      sample(0, 1, 5, netId),
      sample(0.001, 0, 5, netId),
      sample(0.002, 0, 0, netId),
    ];
    const core: StimulusStepSample[] = [
      sample(0, 1, 5, netId),
      sample(0.001, 0, 0, netId),
      sample(0.002, 0, 1.5, netId), // seeded fault: still wrong one sample later
    ];

    const result = compareStimuliSamples(classic, core, circuit);

    // Sample 1's mismatch is no longer excused, because the SAME (netV, netId)
    // pair still disagrees at sample 2 — and sample 2's own mismatch has no
    // "next sample" to be excused by (it's the last one), so it fails too.
    expect(result.knownDivergences).toEqual([]);
    expect(result.mismatches).toHaveLength(2);
    expect(result.mismatches.map((m) => m.sampleTime)).toEqual([0.001, 0.002]);
  });

  it("never explains anything when no circuit is given (the exemption is opt-in)", () => {
    const circuit = inverterCircuit();
    const netId = y1NetId(circuit);
    const classic: StimulusStepSample[] = [sample(0, 1, 5, netId), sample(0.001, 0, 5, netId), sample(0.002, 0, 0, netId)];
    const core: StimulusStepSample[] = [sample(0, 1, 5, netId), sample(0.001, 0, 0, netId), sample(0.002, 0, 0, netId)];

    const result = compareStimuliSamples(classic, core);

    expect(result.knownDivergences).toEqual([]);
    expect(result.mismatches).toHaveLength(1);
  });

  it("does not explain a netV mismatch on a net an IC output pin does NOT drive, even at a commit sample", () => {
    const circuit = inverterCircuit();
    const netId = y1NetId(circuit);
    // "unrelated" is a net id that never appears on any real net — stands in
    // for "some other net that happens to disagree at the same sample, for a
    // totally unrelated reason".
    const classic: StimulusStepSample[] = [
      { phaseIndex: 0, simTime: 0, netV: { [netId]: 5, unrelated: 1 }, elementI: {}, digitalState: { "u1/1y": 1 } },
      { phaseIndex: 0, simTime: 0.001, netV: { [netId]: 5, unrelated: 1 }, elementI: {}, digitalState: { "u1/1y": 0 } },
      { phaseIndex: 0, simTime: 0.002, netV: { [netId]: 0, unrelated: 1 }, elementI: {}, digitalState: { "u1/1y": 0 } },
    ];
    const core: StimulusStepSample[] = [
      { phaseIndex: 0, simTime: 0, netV: { [netId]: 5, unrelated: 1 }, elementI: {}, digitalState: { "u1/1y": 1 } },
      { phaseIndex: 0, simTime: 0.001, netV: { [netId]: 0, unrelated: 2 }, elementI: {}, digitalState: { "u1/1y": 0 } },
      // "unrelated" AGREES again at the next sample too — isolating that it
      // is excluded because it is off the commit's surface, not merely
      // because it never resolves (that's the previous test's point).
      { phaseIndex: 0, simTime: 0.002, netV: { [netId]: 0, unrelated: 1 }, elementI: {}, digitalState: { "u1/1y": 0 } },
    ];

    const result = compareStimuliSamples(classic, core, circuit);

    // The y1 net's own mismatch is explained; "unrelated" is not on the
    // commit's surface, so it still fails even though it resolves by the
    // next sample exactly like the explained one does.
    expect(result.knownDivergences).toHaveLength(1);
    expect(result.knownDivergences[0]!.mismatch.id).toBe(netId);
    expect(result.mismatches.map((m) => m.id)).toEqual(["unrelated"]);
  });
});


// ─── W1e: NE555 point 13.4, the phase-aligned comparison ────────────────────

describe("comparePhaseAlignedSamples (NE555 point 13.4)", () => {
  const sample = (simTime: number, over: Partial<SampleReadout> = {}): SampleReadout => ({
    simTime,
    netV: { out: 4.9, cap: 2.64, gnd: 0 },
    elementI: { vcc: -0.0148, ra: 0.0012 },
    digitalState: { u1: 1 },
    failures: {},
    warnings: {},
    ...over,
  });
  const options = { skipNetV: new Set(["cap"]), skipElementI: new Set(["ra"]) };

  it("never compares simTime (the engines are sampled at their own instants by design) and judges everything else as compareSamples does", () => {
    const result = comparePhaseAlignedSamples([sample(1.0e-3)], [sample(1.2e-3)], options);
    expect(result.mismatches).toEqual([]);
  });

  it("the detector: a level, a current or a digital state that differs is a mismatch, and its detail names both instants", () => {
    const core = sample(1.2e-3, { netV: { out: 4.5, cap: 2.64, gnd: 0 }, elementI: { vcc: -0.02, ra: 0.0012 }, digitalState: { u1: 0 } });
    const result = comparePhaseAlignedSamples([sample(1.0e-3)], [core], options);
    expect(result.mismatches.map((m) => `${m.category}:${m.id}`).sort()).toEqual(["digitalState:u1", "elementI:vcc", "netV:out"]);
    expect(result.mismatches[0]!.detail).toMatch(/classic sampled at t=0.001, core at t=0.0012/);
  });

  it("leaves the sidecar's nodes and elements out (a cap node 1 V apart is not this comparison's business), and removes the timing branch from a source through adjustElementI", () => {
    const core = sample(1.2e-3, { netV: { out: 4.9, cap: 3.64, gnd: 0 }, elementI: { vcc: -0.0148 - 0.0005, ra: 0.0012 + 0.0005 } });
    const raw = comparePhaseAlignedSamples([sample(1.0e-3)], [core], options);
    expect(raw.mismatches.map((m) => m.id)).toEqual(["vcc"]);
    const adjusted = comparePhaseAlignedSamples([sample(1.0e-3)], [core], {
      ...options,
      adjustElementI: (i) => ({ ...i, vcc: i.vcc! + i.ra! }),
    });
    expect(adjusted.mismatches).toEqual([]);
  });

  it("refuses a pair of different lengths (a harness bug, not a circuit result)", () => {
    expect(() => comparePhaseAlignedSamples([sample(1e-3)], [], options)).toThrow(/sample count mismatch/);
  });
});
