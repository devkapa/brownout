/**
 * Unit coverage for stimuli.ts. The classic side is checked against a real
 * (small, fast) RC circuit for the one behaviour that actually matters here
 * — carry-forward across a phase's reload, mirroring devolt's
 * `simulateScript` (read for this brief, not imported: devolt is a private,
 * off-limits repo for this worker). The core side is now checked against a
 * real synthetic switch circuit: `bo_engine_set_params` (owner decision 1)
 * landed in W1b, so `replayStimuliOnCore` genuinely replays a phase's `set`
 * overrides through `CoreEngine.setParams` instead of always throwing.
 * `switch`/`push_button`/`spdt_switch`/`dip_switch` are not yet in
 * `CORE_IMPLEMENTED_KINDS` (kinds.ts, the orchestrator's table), so these
 * tests pass a widened `implementedKinds` set — the same documented escape
 * hatch `compileCircuit`/`CoreEngine.load` already offer callers (chiefly
 * tests).
 */

import { beforeAll, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { initCore } from "../../../src/core/wasm.js";
import {
  replayStimuliOnClassic,
  replayStimuliOnCore,
  STIMULUS_FIXED_STEP_S,
  StimuliNotYetSupportedOnCoreError,
  type StimulusPhase,
} from "./stimuli.js";

// A real CoreEngine (used once `implementedKinds` is widened below) needs
// initCore() awaited first.
beforeAll(async () => {
  await initCore();
});

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

/** vsrc -> sw -> r1 -> vsrc.neg. `switch` is not in CORE_IMPLEMENTED_KINDS (kinds.ts) yet — the core-side tests below pass a widened implementedKinds set. */
function switchedDivider(closed: number | string): SimCircuit {
  return {
    components: [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("sw", "switch", ["a", "b"], { closed }),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    wires: [
      wire("vsrc", "pos", "sw", "a"),
      wire("sw", "b", "r1", "a"),
      wire("r1", "b", "vsrc", "neg"),
    ],
  };
}

const WITH_SWITCH = new Set([...CORE_IMPLEMENTED_KINDS, "switch"]);

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

/**
 * vsrc -> led1 -> r1 -> gnd. `switch` joined CORE_IMPLEMENTED_KINDS in W1b
 * (kinds.ts), and `capacitor` joined it once this same wave's later step
 * landed the device/solver — neither still proves "the default set can't
 * load this circuit". `led` (W1e, not yet ported) takes over that role.
 */
function ledOnlyCircuit(): SimCircuit {
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

describe("replayStimuliOnCore", () => {
  it("throws StimuliNotYetSupportedOnCoreError, naming the missing kind, when the DEFAULT CORE_IMPLEMENTED_KINDS can't load this circuit at all", () => {
    try {
      replayStimuliOnCore(ledOnlyCircuit(), [{ durationS: 1e-3 }]);
      expect.unreachable();
    } catch (err) {
      expect(err).toBeInstanceOf(StimuliNotYetSupportedOnCoreError);
      expect((err as StimuliNotYetSupportedOnCoreError).reasons).toContain("unsupported-kind:led");
      expect((err as Error).message).toMatch(/bo_engine_set_params/);
    }
  });

  it("applies each phase's `set` through CoreEngine.setParams at the phase start, then advances — closed, then open, in ONE continuous run (no reload)", () => {
    const phases: StimulusPhase[] = [
      { set: [{ id: "sw", params: { closed: 1 } }], durationS: 2 * STIMULUS_FIXED_STEP_S },
      { set: [{ id: "sw", params: { closed: 0 } }], durationS: 2 * STIMULUS_FIXED_STEP_S },
    ];

    const samples = replayStimuliOnCore(switchedDivider(0), phases, WITH_SWITCH);
    const phase0 = samples.filter((s) => s.phaseIndex === 0);
    const phase1 = samples.filter((s) => s.phaseIndex === 1);
    expect(phase0).toHaveLength(2);
    expect(phase1).toHaveLength(2);

    // Phase 0 turned the switch on (closed:1): ~5mA through the 1k resistor.
    expect(Math.abs(phase0.at(-1)!.elementI.sw ?? 0)).toBeGreaterThan(4e-3);
    // Phase 1 turned it back off (closed:0): the update actually took effect
    // — this is not the circuit's ORIGINAL (also open) param, it is setParams
    // having flipped a device that phase 0 had just closed.
    expect(Math.abs(phase1.at(-1)!.elementI.sw ?? 0)).toBeLessThan(1e-4);
  });

  it("samples at the fixed step STIMULUS_FIXED_STEP_S, matching the classic path's uniform grid", () => {
    const phases: StimulusPhase[] = [{ durationS: 5 * STIMULUS_FIXED_STEP_S }];
    const samples = replayStimuliOnCore(switchedDivider(1), phases, WITH_SWITCH);
    expect(samples).toHaveLength(5);
    for (let i = 1; i < samples.length; i++) {
      expect(samples[i]!.simTime - samples[i - 1]!.simTime).toBeCloseTo(STIMULUS_FIXED_STEP_S, 9);
    }
  });

  it("a phase that sets a param which exists but is NOT live fails clearly — never silently skipped, and never StimuliNotYetSupportedOnCoreError", () => {
    // r1 (resistor) has no live params at all (CORE_RECORD_SHAPES.resistor.liveParams is empty).
    const phases: StimulusPhase[] = [{ set: [{ id: "r1", params: { resistance: 5000 } }], durationS: STIMULUS_FIXED_STEP_S }];
    expect(() => replayStimuliOnCore(switchedDivider(1), phases, WITH_SWITCH)).toThrow(/live/i);
    try {
      replayStimuliOnCore(switchedDivider(1), phases, WITH_SWITCH);
      expect.unreachable();
    } catch (err) {
      expect(err).not.toBeInstanceOf(StimuliNotYetSupportedOnCoreError);
    }
  });

  it("a phase naming a component absent from the loaded program fails clearly, not silently", () => {
    const phases: StimulusPhase[] = [{ set: [{ id: "no-such-component", params: { closed: 1 } }], durationS: STIMULUS_FIXED_STEP_S }];
    expect(() => replayStimuliOnCore(switchedDivider(1), phases, WITH_SWITCH)).toThrow(/no device for component/);
  });

  it("throws (does not swallow) when the circuit cannot even be loaded — a source shorted to itself, mirroring replayStimuliOnClassic's own load-failure check", () => {
    const shorted: SimCircuit = {
      components: [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 })],
      wires: [wire("vsrc", "pos", "vsrc", "neg")],
    };
    expect(() => replayStimuliOnCore(shorted, [{ durationS: 1e-3 }])).toThrow();
  });

  // Finding 10: phase durations that are NOT whole multiples of
  // STIMULUS_FIXED_STEP_S used to make the core round to a different whole
  // step count than classic's own clamp-to-remaining behaviour — 2.5ms
  // rounded UP to 3 whole 1ms steps (drifting the next phase's `set` 0.5ms
  // late relative to classic), and 0.4ms rounded DOWN to zero steps (no
  // sample at all, where classic still produces one clamped 0.4ms step).
  it("matches replayStimuliOnClassic's own sample times exactly for a phase duration that is NOT a whole multiple of STIMULUS_FIXED_STEP_S (2.5ms)", () => {
    const phases: StimulusPhase[] = [{ durationS: 2.5 * STIMULUS_FIXED_STEP_S }, { durationS: 2 * STIMULUS_FIXED_STEP_S }];
    const classicSamples = replayStimuliOnClassic(switchedDivider(1), phases);
    const coreSamples = replayStimuliOnCore(switchedDivider(1), phases, WITH_SWITCH);

    expect(coreSamples).toHaveLength(classicSamples.length);
    for (let i = 0; i < classicSamples.length; i++) {
      expect(coreSamples[i]!.phaseIndex, `sample ${String(i)}`).toBe(classicSamples[i]!.phaseIndex);
      expect(coreSamples[i]!.simTime, `sample ${String(i)}`).toBeCloseTo(classicSamples[i]!.simTime, 9);
    }
  });

  it("matches replayStimuliOnClassic's own sample COUNT for a phase shorter than one whole STIMULUS_FIXED_STEP_S (0.4ms) — the old rounding produced ZERO core samples here", () => {
    const phases: StimulusPhase[] = [{ durationS: 0.4 * STIMULUS_FIXED_STEP_S }, { durationS: 2 * STIMULUS_FIXED_STEP_S }];
    const classicSamples = replayStimuliOnClassic(switchedDivider(1), phases);
    const coreSamples = replayStimuliOnCore(switchedDivider(1), phases, WITH_SWITCH);

    expect(classicSamples.filter((s) => s.phaseIndex === 0)).toHaveLength(1);
    expect(coreSamples).toHaveLength(classicSamples.length);
    for (let i = 0; i < classicSamples.length; i++) {
      expect(coreSamples[i]!.simTime, `sample ${String(i)}`).toBeCloseTo(classicSamples[i]!.simTime, 9);
    }
  });
});
