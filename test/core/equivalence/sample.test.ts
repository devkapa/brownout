/**
 * Unit coverage for collectClassicSamples's own harness-error behaviour
 * (sample.ts's header). run-entry.ts turns any thrown error here into an
 * "error" EntryResult; these tests prove the collector itself refuses a
 * stalled run rather than silently returning a mislabelled sample, one
 * level below that plumbing.
 */

import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { DEFAULT_DURATION_S, collectClassicSamples, sampleTimesFor } from "./sample.js";

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

/** A 10 V source wired to ground on BOTH terminals — both fixed at 0 V while the source demands 10 V between them. Classic's MNA is singular here (core/README.md's W0 kernel semantics #1: the core rejects the same case as topology -3, "a loop of ideal sources, including a source shorted to itself"). */
function shortedSource(): SimCircuit {
  return circuit(
    [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 10 }), part("gnd", "ground", ["gnd"])],
    [wire("vsrc", "pos", "gnd", "gnd"), wire("vsrc", "neg", "gnd", "gnd")],
  );
}

describe("collectClassicSamples — schedule validation", () => {
  const divider = circuit(
    [
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 10 }),
      part("gnd", "ground", ["gnd"]),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    [wire("vsrc", "neg", "gnd", "gnd"), wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "gnd", "gnd")],
  );

  it("rejects an empty schedule — gate 2b's dense-and-quiet grid can be filtered down to nothing", () => {
    expect(() => collectClassicSamples(divider, [])).toThrow(/empty/);
  });

  it("rejects a non-ascending schedule", () => {
    expect(() => collectClassicSamples(divider, [0, 2, 1])).toThrow(/ascending/);
    expect(() => collectClassicSamples(divider, [0, 1, 1])).toThrow(/ascending/); // a duplicate is not STRICTLY ascending
  });
});

describe("collectClassicSamples", () => {
  it("throws a harness error instead of returning a sample from a run that hit the H_MIN floor or never converged", () => {
    const times = sampleTimesFor(DEFAULT_DURATION_S);
    expect(() => collectClassicSamples(shortedSource(), times)).toThrow(/hit the H_MIN floor|did not converge/);
  });

  it("still returns one sample per requested time for a well-behaved circuit (regression guard)", () => {
    const divider = circuit(
      [
        part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 10 }),
        part("gnd", "ground", ["gnd"]),
        part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
        part("r2", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      [
        wire("vsrc", "neg", "gnd", "gnd"),
        wire("vsrc", "pos", "r1", "a"),
        wire("r1", "b", "r2", "a"),
        wire("r2", "b", "gnd", "gnd"),
      ],
    );
    const times = sampleTimesFor(DEFAULT_DURATION_S);
    const samples = collectClassicSamples(divider, times);
    expect(samples).toHaveLength(times.length);
    samples.forEach((s, i) => expect(s.simTime).toBeCloseTo(times[i]!, 9));
  });
});
