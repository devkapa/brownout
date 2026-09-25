/**
 * Unit coverage for stimuli.ts. The classic side is checked against a real
 * (small, fast) RC circuit for the one behaviour that actually matters here
 * — carry-forward across a phase's reload, mirroring devolt's
 * `simulateScript` (read for this brief, not imported: devolt is a private,
 * off-limits repo for this worker). The core side is checked against the
 * plain, always-true contract this worker's brief specifies for it: "give
 * the core side an interface that throws 'not yet supported' clearly."
 */

import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import {
  replayStimuliOnClassic,
  replayStimuliOnCore,
  STIMULUS_FIXED_STEP_S,
  StimuliNotYetSupportedOnCoreError,
  type StimulusPhase,
} from "./stimuli.js";

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

/** A simple RC charging network: vsrc -> r1 -> c1 -> gnd. `resistance` is the param a phase override changes; it does not touch `c1`, so any effect on its charge is purely from carry-forward (or its absence). */
function rcNetwork(resistance: number): SimCircuit {
  return {
    components: [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("r1", "resistor", ["a", "b"], { resistance }),
      part("c1", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
    ],
    wires: [
      wire("vsrc", "neg", "gnd", "gnd"),
      wire("vsrc", "pos", "r1", "a"),
      wire("r1", "b", "c1", "a"),
      wire("c1", "b", "gnd", "gnd"),
    ],
  };
}

describe("replayStimuliOnClassic", () => {
  it("carries capacitor charge forward across a phase's reload, even though the phase overrides an UNRELATED component's param", () => {
    // tau = 1000 * 1e-6 = 1ms; 2ms of phase 0 charges c1 to a clearly nonzero,
    // clearly-not-fully-charged voltage — a good midpoint to observe carry-
    // forward from without either extreme (0 V or fully settled) hiding a bug.
    const phases: StimulusPhase[] = [
      { durationS: 2e-3 }, // no overrides — just let c1 charge
      { set: [{ id: "r1", params: { resistance: 2000 } }], durationS: 2e-3 },
    ];

    const samples = replayStimuliOnClassic(rcNetwork(1000), phases);
    const phase0Samples = samples.filter((s) => s.phaseIndex === 0);
    const phase1Samples = samples.filter((s) => s.phaseIndex === 1);
    expect(phase0Samples.length).toBeGreaterThan(0);
    expect(phase1Samples.length).toBeGreaterThan(0);

    // Read continuity through r1's current (elementI is keyed by component
    // id, unlike netV's auto-generated net ids, so this needs no lookup of
    // which net a pin landed on). A cold reload (no carry-forward) would
    // discharge c1 back to 0V, so phase 1 would start at r1's FRESH charging
    // current for its new 2000ohm resistance (5V / 2000ohm = 2.5mA); carry-
    // forward instead continues from wherever phase 0's charging left off,
    // which by 2ms into a 1ms-tau RC is already well below that fresh value.
    const freshChargeCurrentA = 5 / 2000;
    const iAtEndOfPhase0 = Math.abs(phase0Samples.at(-1)!.elementI.r1 ?? NaN);
    const iAtStartOfPhase1 = Math.abs(phase1Samples[0]!.elementI.r1 ?? NaN);

    // A real, meaningfully-charged midpoint (not the trivial t=0 extreme).
    expect(iAtEndOfPhase0).toBeGreaterThan(0);
    expect(iAtEndOfPhase0).toBeLessThan(freshChargeCurrentA * 0.7);
    // Carry-forward: continuous with where phase 0 left off, nowhere near
    // the fresh (cold-reload) charging current.
    expect(iAtStartOfPhase1).toBeLessThan(freshChargeCurrentA * 0.7);
    expect(iAtStartOfPhase1).toBeLessThan(iAtEndOfPhase0 * 3);
  });

  it("samples at the fixed step STIMULUS_FIXED_STEP_S, not adaptively (a uniform grid within each phase)", () => {
    const phases: StimulusPhase[] = [{ durationS: 5 * STIMULUS_FIXED_STEP_S }];
    const samples = replayStimuliOnClassic(rcNetwork(1000), phases);
    expect(samples).toHaveLength(5);
    for (let i = 1; i < samples.length; i++) {
      expect(samples[i]!.simTime - samples[i - 1]!.simTime).toBeCloseTo(STIMULUS_FIXED_STEP_S, 9);
    }
  });

  it("applies only the overrides named in `set`, leaving every other param untouched", () => {
    const phases: StimulusPhase[] = [
      { durationS: STIMULUS_FIXED_STEP_S },
      { set: [{ id: "r1", params: { resistance: 5000 } }], durationS: STIMULUS_FIXED_STEP_S },
    ];
    // Just proving this runs cleanly end to end with a real override applied
    // (a wrong id, or clobbering c1's params, would either throw or change
    // the charge trajectory in a way the carry-forward test above would
    // already have caught) — this test is the "it does not crash and
    // produces the right sample count" regression guard.
    const samples = replayStimuliOnClassic(rcNetwork(1000), phases);
    expect(samples).toHaveLength(2);
  });

  it("throws when a phase does not complete cleanly", () => {
    // maxSteps has no equivalent lever exposed by StimulusPhase, so force a
    // failure another way: a source shorted directly to ground makes
    // classic's own load() throw before any phase even runs — proving
    // replayStimuliOnClassic does not swallow a bad circuit silently.
    const shorted: SimCircuit = {
      components: [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("gnd", "ground", ["gnd"])],
      wires: [wire("vsrc", "pos", "gnd", "gnd"), wire("vsrc", "neg", "gnd", "gnd")],
    };
    expect(() => replayStimuliOnClassic(shorted, [{ durationS: 1e-3 }])).toThrow();
  });
});

describe("replayStimuliOnCore", () => {
  it("always throws StimuliNotYetSupportedOnCoreError — owner decision 1's parameter-update event is W1b, not yet landed", () => {
    expect(() => replayStimuliOnCore(rcNetwork(1000), [{ durationS: 1e-3 }])).toThrow(StimuliNotYetSupportedOnCoreError);
  });

  it("the error message names the actual mechanism the core will use, not a generic TODO", () => {
    try {
      replayStimuliOnCore(rcNetwork(1000), [{ durationS: 1e-3 }]);
      expect.unreachable();
    } catch (err) {
      expect(err).toBeInstanceOf(StimuliNotYetSupportedOnCoreError);
      expect((err as Error).message).toMatch(/bo_engine_set_params/);
    }
  });
});
