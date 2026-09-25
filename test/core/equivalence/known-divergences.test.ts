/**
 * Unit coverage for known-divergences.ts's structural registry (orchestrator
 * review fix: seeds/cone computed from the circuit's own topology, not a
 * regex matched against Mismatch.id). Covers digitalOutputPins' two sources
 * (classic's icPinMap, and this file's own fallback table — each "naming its
 * source lines" per the fix brief), computeDivergenceCone's forward closure,
 * and classifyMismatches' three id-matching rules end to end on a synthetic
 * SAP-1-shaped circuit.
 */

import { describe, expect, it } from "vitest";
import { buildNets } from "../../../src/sim/engine/graph.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { classifyMismatches, computeDivergenceCone, digitalOutputPins, KNOWN_DIVERGENCES } from "./known-divergences.js";
import type { Mismatch } from "./types.js";

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

// ---- digitalOutputPins ------------------------------------------------------

describe("digitalOutputPins", () => {
  it("uses classic's own icPinMap for a kind it covers — 74hc74's q1/q1_n/q2/q2_n (src/sim/engine/digital.ts's IC_PIN_MAPS entry)", () => {
    expect(digitalOutputPins("74hc74")).toEqual(new Set(["q1", "q1_n", "q2", "q2_n"]));
  });

  it("falls back to this file's own table for 28c16/28c256's io0-io7 — source: src/sim/engine/devices/digital-ics.ts's eepromModel.stamp, the `for (let bit = 0; bit < 8; bit++) ctx.stampDigitalOutput(comp, `io${bit}`, ...)` loop", () => {
    const expected = new Set(["io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7"]);
    expect(digitalOutputPins("28c16")).toEqual(expected);
    expect(digitalOutputPins("28c256")).toEqual(expected);
  });

  it("is empty for a kind with no known digital outputs (LED, resistor, an unrecognised kind) — the 'stop propagation here' case", () => {
    expect(digitalOutputPins("led")).toEqual(new Set());
    expect(digitalOutputPins("resistor")).toEqual(new Set());
    expect(digitalOutputPins("not-a-real-kind")).toEqual(new Set());
  });
});

// ---- a synthetic SAP-1-shaped circuit for the cone/classification tests ---

/**
 * u_555 (ne555, RST on pin "4" tied low via `rstLow`) --OUT(pin "3")--> clk1
 * of u_ff (74hc74) --q1--> led1 (LED) --b--> r_after_led (resistor, beyond
 * the LED — must NEVER join the cone). r_unrelated sits off to the side,
 * wired to nothing the 555/FF/LED touch.
 */
function phantomEdgeCircuit(options: { rstLow: boolean; clockedByNe555: boolean }): SimCircuit {
  const components: SimComponent[] = [
    // A real source is needed so buildNets actually names a "gnd" net (node
    // 0) — graph.ts only does that from a SOURCE_KINDS component's own
    // return pin (or a handful of special-cased board kinds), NEVER merely
    // from a plain `ground`-kind component sitting on a net. Without this,
    // `gnd`'s own net would just be an ordinary "n_" net like any other, and
    // the "RST is on the datum/gnd net" branch of pinIsHeldLow would never
    // fire — exactly the mistake this test caught on its first run.
    part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
    part("gnd", "ground", ["gnd"]),
    part("u_555", "ne555", ["1", "2", "3", "4", "6", "7", "8"]),
    part("u_ff", "74hc74", ["clk1", "d1", "q1", "q1_n", "pre1_n", "clr1_n"]),
    part("led1", "led", ["a", "b"]),
    part("r_after_led", "resistor", ["a", "b"], { resistance: 220 }),
    part("r_unrelated", "resistor", ["a", "b"], { resistance: 1000 }),
  ];
  const wires: SimCircuit["wires"] = [
    wire("vcc", "neg", "gnd", "gnd"),
    wire("led1", "b", "r_after_led", "a"),
    wire("u_ff", "q1", "led1", "a"),
  ];

  if (options.clockedByNe555) {
    wires.push(wire("u_555", "3", "u_ff", "clk1"));
  } else {
    components.push(part("clk_src", "clock", ["out", "gnd"], { frequency: 1 }));
    wires.push(wire("clk_src", "out", "u_ff", "clk1"));
  }

  if (options.rstLow) {
    wires.push(wire("u_555", "4", "gnd", "gnd"));
  }
  // else: RST left floating — not held low.

  return circuit(components, wires);
}

describe("computeDivergenceCone", () => {
  it("is empty for no seeds", () => {
    const cone = computeDivergenceCone(phantomEdgeCircuit({ rstLow: true, clockedByNe555: true }), []);
    expect(cone.components).toEqual(new Set());
    expect(cone.seedPins.size).toBe(0);
  });

  it("grows forward from a seed's own output pins, and stops at a component with none (the LED) — the seed itself is NOT added to `components` (only its own pins are exempted, via seedPins)", () => {
    const cone = computeDivergenceCone(phantomEdgeCircuit({ rstLow: true, clockedByNe555: true }), [
      { componentId: "u_ff", outputPins: new Set(["q1", "q1_n"]) },
    ]);
    expect(cone.components.has("u_ff")).toBe(false); // never whole-package membership for a seed
    expect(cone.seedPins.get("u_ff")).toEqual(new Set(["q1", "q1_n"]));
    expect(cone.components.has("led1")).toBe(true); // joins: wired to u_ff's q1 output
    expect(cone.components.has("r_after_led")).toBe(false); // beyond the LED — propagation stopped there
    expect(cone.components.has("r_unrelated")).toBe(false);
    expect(cone.components.has("u_555")).toBe(false); // the 555 is not itself downstream of the seed
  });

  it("finding 7: a seed restricted to q1/q1_n never propagates through q2/q2_n's own net", () => {
    // u_ff2 has BOTH halves wired: clk1 (seeded) feeds led1 via q1; clk2
    // (NOT seeded — no ne555 involved) feeds led2 via q2. Only the q1 half's
    // seed should reach led1; led2 must never join the cone.
    const components: SimComponent[] = [
      part("u_ff2", "74hc74", ["clk1", "d1", "q1", "q1_n", "clk2", "d2", "q2", "q2_n", "pre1_n", "clr1_n", "pre2_n", "clr2_n"]),
      part("led1", "led", ["a", "b"]),
      part("led2", "led", ["a", "b"]),
    ];
    const wires: SimCircuit["wires"] = [wire("u_ff2", "q1", "led1", "a"), wire("u_ff2", "q2", "led2", "a")];
    const c = circuit(components, wires);

    const cone = computeDivergenceCone(c, [{ componentId: "u_ff2", outputPins: new Set(["q1", "q1_n"]) }]);
    expect(cone.components.has("led1")).toBe(true);
    expect(cone.components.has("led2")).toBe(false);
    expect(cone.seedPins.get("u_ff2")).toEqual(new Set(["q1", "q1_n"]));
  });
});

/**
 * u_555 (ne555, RST tied low) --OUT(pin "3")--> clk1 of u_ff (74hc74)
 * --q1--> led1. clk2 of the SAME u_ff package is driven by an unrelated
 * `clock` component --q2--> led2. Only clk1's half satisfies the owner's
 * ruling (finding 7: "ITS OWN clock is the RST-low 555's OUT") — clk2's
 * half must get no exemption at all, even though it shares a physical
 * package with the seeded half.
 */
function dualClockedHc74Circuit(): SimCircuit {
  const components: SimComponent[] = [
    part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
    part("gnd", "ground", ["gnd"]),
    part("u_555", "ne555", ["1", "2", "3", "4", "6", "7", "8"]),
    part("u_ff", "74hc74", ["clk1", "d1", "q1", "q1_n", "clk2", "d2", "q2", "q2_n"]),
    part("clk_src", "clock", ["out", "gnd"], { frequency: 1 }),
    part("led1", "led", ["a", "b"]),
    part("led2", "led", ["a", "b"]),
  ];
  const wires: SimCircuit["wires"] = [
    wire("vcc", "neg", "gnd", "gnd"),
    wire("u_555", "4", "gnd", "gnd"), // RST tied low
    wire("u_555", "3", "u_ff", "clk1"), // clk1: on the RST-low 555's OUT
    wire("clk_src", "out", "u_ff", "clk2"), // clk2: on an unrelated clock
    wire("u_ff", "q1", "led1", "a"),
    wire("u_ff", "q2", "led2", "a"),
  ];
  return circuit(components, wires);
}

describe("classifyMismatches — the phantom-edge registration (structural, not pattern-based)", () => {
  function mismatch(category: Mismatch["category"], id: string): Mismatch {
    return { sampleTime: 0, category, id, classicValue: 1, coreValue: 0, detail: "differs" };
  }

  it("finding 7: a package with one half on the RST-low 555 and the other half on a different clock — only the first half is known", () => {
    const c = dualClockedHc74Circuit();
    const mismatches: Mismatch[] = [
      mismatch("digitalState", "u_ff/q1"),
      mismatch("digitalState", "u_ff/q1_n"),
      mismatch("elementI", "led1"),
      mismatch("digitalState", "u_ff/q2"),
      mismatch("digitalState", "u_ff/q2_n"),
      mismatch("elementI", "led2"),
    ];

    const result = classifyMismatches(c, mismatches);

    expect(result.known.map((h) => `${h.mismatch.category}:${h.mismatch.id}`)).toEqual(
      expect.arrayContaining(["digitalState:u_ff/q1", "digitalState:u_ff/q1_n", "elementI:led1"]),
    );
    expect(result.known).toHaveLength(3);
    expect(result.unexplained.map((m) => `${m.category}:${m.id}`)).toEqual(
      expect.arrayContaining(["digitalState:u_ff/q2", "digitalState:u_ff/q2_n", "elementI:led2"]),
    );
    expect(result.unexplained).toHaveLength(3);
  });

  it("seeded fault covered: an RST-low 555 clocking a 74HC74 feeding an LED — FF keys, the LED's current, and the FF's output net are known; an unrelated component is not", () => {
    const c = phantomEdgeCircuit({ rstLow: true, clockedByNe555: true });
    const mismatches: Mismatch[] = [
      mismatch("digitalState", "u_ff/q1"),
      mismatch("digitalState", "u_ff/q1_n"),
      mismatch("elementI", "led1"),
      mismatch("netV", findNetIdForPin(c, "u_ff", "q1")),
      mismatch("elementI", "r_unrelated"),
      mismatch("digitalState", "r_unrelated"),
    ];

    const result = classifyMismatches(c, mismatches);

    const knownIds = result.known.map((h) => `${h.mismatch.category}:${h.mismatch.id}`);
    expect(knownIds).toEqual(
      expect.arrayContaining([
        "digitalState:u_ff/q1",
        "digitalState:u_ff/q1_n",
        "elementI:led1",
        `netV:${findNetIdForPin(c, "u_ff", "q1")}`,
      ]),
    );
    expect(result.known).toHaveLength(4);
    expect(result.unexplained.map((m) => `${m.category}:${m.id}`)).toEqual(
      expect.arrayContaining(["elementI:r_unrelated", "digitalState:r_unrelated"]),
    );
    expect(result.unexplained).toHaveLength(2);
    expect(result.known[0]!.divergence.id).toBe("w1-0-startup-phantom-edge-74hc74-555-rst-low");
  });

  it("seeded fault: the same circuit but RST is NOT tied low — nothing is known", () => {
    const c = phantomEdgeCircuit({ rstLow: false, clockedByNe555: true });
    const mismatches: Mismatch[] = [mismatch("digitalState", "u_ff/q1"), mismatch("elementI", "led1")];
    const result = classifyMismatches(c, mismatches);
    expect(result.known).toEqual([]);
    expect(result.unexplained).toHaveLength(2);
  });

  it("seeded fault: a 74HC74 clocked by some other source (not an ne555) — nothing is known, even with RST tied low elsewhere", () => {
    const c = phantomEdgeCircuit({ rstLow: true, clockedByNe555: false });
    const mismatches: Mismatch[] = [mismatch("digitalState", "u_ff/q1"), mismatch("elementI", "led1")];
    const result = classifyMismatches(c, mismatches);
    expect(result.known).toEqual([]);
    expect(result.unexplained).toHaveLength(2);
  });

  it("seeded fault: the cone stops at the LED — a mismatch on the component wired beyond it is never known", () => {
    const c = phantomEdgeCircuit({ rstLow: true, clockedByNe555: true });
    const result = classifyMismatches(c, [mismatch("elementI", "r_after_led")]);
    expect(result.known).toEqual([]);
    expect(result.unexplained).toHaveLength(1);
  });

  it("a board with no RST-low 555 at all gets no exemption — the cone is empty unless the seed condition holds", () => {
    // A totally unrelated circuit: no ne555, no 74hc74.
    const plain = circuit(
      [part("gnd", "ground", ["gnd"]), part("r1", "resistor", ["a", "b"], { resistance: 1000 })],
      [],
    );
    const result = classifyMismatches(plain, [mismatch("elementI", "r1")]);
    expect(result.known).toEqual([]);
  });

  it("never classifies a failures/warnings/simTime mismatch as known, even if its id names a cone member", () => {
    const c = phantomEdgeCircuit({ rstLow: true, clockedByNe555: true });
    const failureMismatch: Mismatch = { sampleTime: 0, category: "failures", id: "u_ff", classicValue: 1, coreValue: undefined, detail: "differs" };
    const result = classifyMismatches(c, [failureMismatch]);
    expect(result.known).toEqual([]);
    expect(result.unexplained).toEqual([failureMismatch]);
  });

  it("defaults to the real KNOWN_DIVERGENCES registry when none is given", () => {
    expect(() => classifyMismatches(phantomEdgeCircuit({ rstLow: true, clockedByNe555: true }), [])).not.toThrow();
    expect(KNOWN_DIVERGENCES.length).toBeGreaterThan(0);
  });
});

/** Test helper only: resolves the net id a (component, pin) sits on, via `buildNets` directly (not this module's internals), so the netV test case above names a real net independently rather than tautologically. */
function findNetIdForPin(c: SimCircuit, componentId: string, pinId: string): string {
  const nets = buildNets(c);
  const net = nets.find((n) => n.pins.some(([cid, pid]) => cid === componentId && pid === pinId));
  if (!net) throw new Error(`findNetIdForPin: no net contains ${componentId}.${pinId}`);
  return net.id;
}
