/**
 * Unit coverage for known-divergences.ts's structural registry (orchestrator
 * review fix: seeds/cone computed from the circuit's own topology, not a
 * regex matched against Mismatch.id). Covers digitalOutputPins' two sources
 * (classic's icPinMap, and this file's own fallback table — each "naming its
 * source lines" per the fix brief), computeDivergenceCone's forward closure,
 * and classifyMismatches' three id-matching rules end to end on a synthetic
 * SAP-1-shaped circuit. S1 adds the widened cone: the analog reach through
 * resistors, capacitors and LED-family junctions, the sources on the fixed
 * nodes it touches, a ripple counter's second half, and the two things it
 * must never do (cross a fixed node, pull in a register that only shares the
 * rail).
 */

import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../../src/core/wasm.js";
import { buildNets } from "../../../src/sim/engine/graph.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import {
  classifyMismatches,
  computeDivergenceCone,
  digitalOutputPins,
  KNOWN_DIVERGENCES,
  riseAt,
  schedulesCoincide,
  type RiseSchedule,
} from "./known-divergences.js";
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

  it("names the 74hc595's nine outputs, QH' (qh2) among them: classic drives it since ecdcede, the core as its ninth output (P0-IC)", () => {
    expect(digitalOutputPins("74hc595")).toEqual(new Set(["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh", "qh2"]));
  });

  it("names the registers, counter, RAM and 555 the SAP-1 is built from: core/README.md's sequential and memories tables, and NE555 point 4", () => {
    expect(digitalOutputPins("74ls173")).toEqual(new Set(["q1", "q2", "q3", "q4"]));
    expect(digitalOutputPins("74ls161")).toEqual(new Set(["qa", "qb", "qc", "qd", "rco"]));
    expect(digitalOutputPins("74ls189")).toEqual(new Set(["/o1", "/o2", "/o3", "/o4"]));
    expect(digitalOutputPins("ne555")).toEqual(new Set(["3", "7"]));
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

  it("grows forward from a seed's own output pins, and through the LED's junction to the resistor beyond it — the seed itself is NOT added to `components` (only its own pins are exempted, via seedPins)", () => {
    const c = phantomEdgeCircuit({ rstLow: true, clockedByNe555: true });
    const cone = computeDivergenceCone(c, [{ componentId: "u_ff", outputPins: new Set(["q1", "q1_n"]) }]);
    expect(cone.components.has("u_ff")).toBe(false); // never whole-package membership for a seed
    expect(cone.seedPins.get("u_ff")).toEqual(new Set(["q1", "q1_n"]));
    expect(cone.components.has("led1")).toBe(true); // joins: wired to u_ff's q1 output
    expect(cone.components.has("r_after_led")).toBe(true); // the analog reach crosses the LED junction (S1)
    expect(cone.nets.has(findNetIdForPin(c, "r_after_led", "a"))).toBe(true);
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

  it("the cone follows a driven net through an LED to the resistor beyond it (S1's analog reach), and still not to anything wired elsewhere", () => {
    const c = phantomEdgeCircuit({ rstLow: true, clockedByNe555: true });
    const result = classifyMismatches(c, [mismatch("elementI", "r_after_led"), mismatch("elementI", "r_unrelated")]);
    expect(result.known.map((h) => h.mismatch.id)).toEqual(["r_after_led"]);
    expect(result.unexplained.map((m) => m.id)).toEqual(["r_unrelated"]);
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


// ─── W1e: "any 555 whose comparators resolve 0 at load" (NE555 point 9) ─────
//
// The registration's seed is now NE555 point 3 evaluated on classic's own load
// frame, not just "RST tied low". Each circuit below is a powered 555 (5 V on
// pins 8 and 1) clocking one half of a 74HC74; only its wiring differs.

describe("the phantom-edge divergence, widened from 'RST held low' to any 555 that resolves 0 at load", () => {
  beforeAll(async () => {
    await initCore();
  });

  const ff = ["clr1_n", "d1", "clk1", "pre1_n", "q1", "q1_n", "gnd", "q2_n", "q2", "pre2_n", "clk2", "d2", "clr2_n", "vcc"];
  const chip = ["1", "2", "3", "4", "5", "6", "7", "8"];

  /** A 555 on 5 V clocking u_ff's clk1; `wiring` adds the pins under test. */
  function powered555(wiring: (w: SimCircuit["wires"], comps: SimComponent[]) => void): SimCircuit {
    const components: SimComponent[] = [
      part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("u_555", "ne555", chip),
      part("u_ff", "74hc74", ff),
    ];
    const wires: SimCircuit["wires"] = [
      wire("vcc", "neg", "gnd", "gnd"),
      wire("u_555", "8", "vcc", "pos"),
      wire("u_555", "1", "vcc", "neg"),
      wire("u_555", "3", "u_ff", "clk1"),
    ];
    wiring(wires, components);
    return circuit(components, wires);
  }

  const phantom: Mismatch = { sampleTime: 0, category: "digitalState", id: "u_ff/q1", classicValue: 1, coreValue: 0, detail: "differs" };
  const seeded = (c: SimCircuit): boolean => classifyMismatches(c, [phantom]).known.length === 1;

  it("RST on an uncharged RC (a power-on reset delay) resolves 0 at load: seeded", () => {
    const c = powered555((w, comps) => {
      comps.push(part("rr", "resistor", ["a", "b"], { resistance: 100000 }), part("cr", "capacitor", ["a", "b"], { capacitance: 1e-6 }));
      w.push(wire("rr", "a", "vcc", "pos"), wire("rr", "b", "u_555", "4"), wire("cr", "a", "u_555", "4"), wire("cr", "b", "vcc", "neg"));
      w.push(wire("u_555", "2", "u_555", "8"), wire("u_555", "6", "u_555", "1"));
    });
    expect(seeded(c)).toBe(true);
  });

  it("RST left open reads 0 V (the README's Q-O16): seeded", () => {
    const c = powered555((w) => {
      w.push(wire("u_555", "2", "u_555", "8"), wire("u_555", "6", "u_555", "1"));
    });
    expect(seeded(c)).toBe(true);
  });

  it("THR held above 2/3 vS with TRIG above 1/3 vS (both on the supply) resolves 0: seeded", () => {
    const c = powered555((w) => {
      w.push(wire("u_555", "4", "u_555", "8"), wire("u_555", "2", "u_555", "8"), wire("u_555", "6", "u_555", "8"));
    });
    expect(seeded(c)).toBe(true);
  });

  it("the seeded fault: an astable (TRIG = THR on an uncharged capacitor, RST on the supply) resolves 1 at load, so nothing is known", () => {
    const c = powered555((w, comps) => {
      comps.push(part("ct", "capacitor", ["a", "b"], { capacitance: 1e-6 }), part("ra", "resistor", ["a", "b"], { resistance: 1000 }));
      w.push(wire("u_555", "4", "u_555", "8"), wire("u_555", "2", "u_555", "6"), wire("u_555", "6", "ct", "a"), wire("ct", "b", "vcc", "neg"));
      w.push(wire("ra", "a", "vcc", "pos"), wire("ra", "b", "u_555", "7"), wire("u_555", "7", "u_555", "6"));
    });
    expect(seeded(c)).toBe(false);
  });

  it("the seeded fault: a monostable (TRIG high, THR low: the hold region, power-on HIGH) resolves 1 at load, so nothing is known", () => {
    const c = powered555((w) => {
      w.push(wire("u_555", "4", "u_555", "8"), wire("u_555", "2", "u_555", "8"), wire("u_555", "6", "u_555", "1"));
    });
    expect(seeded(c)).toBe(false);
  });

  it("an unpowered 555 resolves nothing: RST open, but pin 8 unwired, is not seeded", () => {
    const c = powered555((w) => {
      w.splice(w.findIndex((x) => x.from_pin === "8"), 1);
    });
    expect(seeded(c)).toBe(false);
  });
});


// ─── S1: the cone widened from the divider's own outputs to its whole downstream ─────────────────
//
// The SAP-1's frozen display: an RST-low 555 clocks a 74HC74 ripple counter, a 74HC138 selects a
// digit from the counter, and a resistor takes each decoder output to a seg7. The counter stays one
// count off forever, so everything below it differs by design. Beside it, on the same rail, an
// unrelated register (a 74LS173 driving an LED) is CPU state that must never be excused.

describe("the widened cone (S1): a frozen display's whole downstream, and nothing beside it", () => {
  const ff = ["clr1_n", "d1", "clk1", "pre1_n", "q1", "q1_n", "gnd", "q2_n", "q2", "pre2_n", "clk2", "d2", "clr2_n", "vcc"];
  const sel = ["a0", "a1", "a2", "e1n", "e2n", "e3", "y0n", "y1n", "y2n", "y3n", "y4n", "y5n", "y6n", "y7n", "gnd", "vcc"];
  const reg = ["/clr", "clk", "m", "n", "g1", "g2", "d1", "d2", "d3", "d4", "q1", "q2", "q3", "q4", "gnd", "vcc"];
  const seg7 = ["a", "b", "c", "d", "e", "f", "g", "dp", "com", "com2"];

  function displayBoard(): SimCircuit {
    const components: SimComponent[] = [
      part("psu", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("gnd", "ground", ["gnd"]),
      part("u_555", "ne555", ["1", "2", "3", "4", "5", "6", "7", "8"]),
      part("u_div", "74hc74", ff),
      part("u_sel", "74hc138", sel),
      part("r_dig", "resistor", ["a", "b"], { resistance: 220 }),
      part("dsp1", "seg7_cc", seg7),
      part("u_reg", "74ls173", reg),
      part("r_reg", "resistor", ["a", "b"], { resistance: 220 }),
      part("led_reg", "led", ["a", "k"]),
      part("r_rail", "resistor", ["a", "b"], { resistance: 1000 }),
      part("r_pull", "resistor", ["a", "b"], { resistance: 1000 }),
    ];
    const wires: SimCircuit["wires"] = [
      wire("psu", "neg", "gnd", "gnd"),
      // The 555: RST on the datum, so its comparators resolve 0 at load.
      wire("u_555", "8", "psu", "pos"),
      wire("u_555", "1", "psu", "neg"),
      wire("u_555", "4", "psu", "neg"),
      // The ripple counter: half 1 on the 555, half 2 on half 1's Q-bar.
      wire("u_555", "3", "u_div", "clk1"),
      wire("u_div", "q1_n", "u_div", "d1"),
      wire("u_div", "q1_n", "u_div", "clk2"),
      wire("u_div", "q2_n", "u_div", "d2"),
      wire("u_div", "vcc", "psu", "pos"),
      wire("u_div", "gnd", "psu", "neg"),
      // The digit select, and a resistor from its output to a seg7 segment, the common on the datum.
      wire("u_div", "q1", "u_sel", "a0"),
      wire("u_div", "q2", "u_sel", "a1"),
      wire("u_sel", "vcc", "psu", "pos"),
      wire("u_sel", "gnd", "psu", "neg"),
      wire("u_sel", "y0n", "r_dig", "a"),
      wire("r_dig", "b", "dsp1", "a"),
      wire("dsp1", "com", "psu", "neg"),
      // A resistor from the cone to the rail: the rail is a fixed node, which the cone never crosses.
      wire("u_sel", "y1n", "r_pull", "a"),
      wire("r_pull", "b", "psu", "pos"),
      // The unrelated register beside it: the same rail, its own output, its own LED.
      wire("u_reg", "vcc", "psu", "pos"),
      wire("u_reg", "gnd", "psu", "neg"),
      wire("u_reg", "q1", "r_reg", "a"),
      wire("r_reg", "b", "led_reg", "a"),
      wire("led_reg", "k", "psu", "neg"),
      wire("r_rail", "a", "psu", "pos"),
      wire("r_rail", "b", "psu", "neg"),
    ];
    return circuit(components, wires);
  }

  const q1Half = { componentId: "u_div", outputPins: new Set(["q1", "q1_n"]) };
  const net = (c: SimCircuit, componentId: string, pinId: string): string => findNetIdForPin(c, componentId, pinId);

  it("includes a 138 output driving a seg7 through a resistor: the decoder, the resistor, the seg7, and the nets between", () => {
    const c = displayBoard();
    const cone = computeDivergenceCone(c, [q1Half]);
    for (const id of ["u_sel", "r_dig", "dsp1"]) expect(cone.components.has(id), id).toBe(true);
    for (const [id, pin] of [["u_sel", "y0n"], ["r_dig", "b"], ["u_div", "q1"], ["u_div", "q1_n"]] as const) {
      expect(cone.nets.has(net(c, id, pin)), `${id}.${pin}`).toBe(true);
    }
  });

  it("excludes an unrelated register beside it on the same rail: its component, its LED chain, its output net", () => {
    const c = displayBoard();
    const cone = computeDivergenceCone(c, [q1Half]);
    for (const id of ["u_reg", "r_reg", "led_reg", "r_rail", "u_555"]) expect(cone.components.has(id), id).toBe(false);
    for (const [id, pin] of [["u_reg", "q1"], ["r_reg", "b"]] as const) expect(cone.nets.has(net(c, id, pin)), `${id}.${pin}`).toBe(false);
  });

  it("never crosses a fixed node: a resistor to the rail joins, the rail net and what sits on it do not", () => {
    const c = displayBoard();
    const cone = computeDivergenceCone(c, [q1Half]);
    expect(cone.components.has("r_pull")).toBe(true);
    expect(cone.nets.has(net(c, "psu", "pos"))).toBe(false);
    expect(cone.nets.has("gnd")).toBe(false);
    expect(cone.components.has("r_rail")).toBe(false);
  });

  it("never adds a source, though the cone draws from it: the supply feeds the whole board, so a supply-current mismatch is never excused here (a caller judges a residual instead)", () => {
    const cone = computeDivergenceCone(displayBoard(), [q1Half]);
    expect(cone.components.has("psu")).toBe(false);
  });

  it("reaches a ripple counter's second half through its own package: Q-bar of half 1 clocks half 2", () => {
    const cone = computeDivergenceCone(displayBoard(), [q1Half]);
    expect(cone.seedPins.get("u_div")).toEqual(new Set(["q1", "q1_n", "q2", "q2_n"]));
    // ... and the decoder's second address line, driven by half 2, is downstream too.
    expect(cone.components.has("u_sel")).toBe(true);
  });

  it("does not reach the second half when nothing of it sits on a net the seeded half drives", () => {
    const c = displayBoard();
    const wires = c.wires.filter((w) => !(w.from_pin === "q1_n" && w.to_pin === "clk2"));
    const cone = computeDivergenceCone({ ...c, wires }, [q1Half]);
    expect(cone.seedPins.get("u_div")).toEqual(new Set(["q1", "q1_n"]));
  });

  it("classifies every readout the frozen display's divergence explains as known, and CPU state beside it as unexplained", () => {
    const c = displayBoard();
    const m = (category: Mismatch["category"], id: string): Mismatch => ({ sampleTime: 0, category, id, classicValue: 1, coreValue: 0, detail: "differs" });
    const explained: Mismatch[] = [
      m("digitalState", "u_div/q1"),
      m("digitalState", "u_div/q2_n"), // the second half, through the ripple
      m("digitalState", "u_sel/y0n"),
      m("elementI", "r_dig"),
      m("elementI", "dsp1"),
      m("netV", net(c, "u_sel", "y0n")),
      m("netV", net(c, "r_dig", "b")), // between the resistor and the seg7: no digital driver at all
    ];
    const cpu: Mismatch[] = [
      m("digitalState", "u_reg/q1"),
      m("netV", net(c, "u_reg", "q1")),
      m("elementI", "r_reg"),
      m("elementI", "led_reg"),
      m("elementI", "r_rail"),
      m("elementI", "psu"), // the supply current: the display draws some, but so does the CPU
      m("netV", net(c, "psu", "pos")), // the rail itself
    ];
    const result = classifyMismatches(c, [...explained, ...cpu]);
    expect(result.known.map((h) => `${h.mismatch.category}:${h.mismatch.id}`).sort()).toEqual(explained.map((x) => `${x.category}:${x.id}`).sort());
    expect(result.unexplained.map((x) => `${x.category}:${x.id}`).sort()).toEqual(cpu.map((x) => `${x.category}:${x.id}`).sort());
  });

  it("detector: a board whose 555 does not resolve 0 at load has no cone, so the same mismatches are all unexplained", () => {
    const c = displayBoard();
    const running: SimCircuit = { ...c, wires: c.wires.map((w) => (w.from_component === "u_555" && w.from_pin === "4" ? { ...w, to_pin: "pos", to_component: "psu" } : w)) };
    // RST on the supply, TRIG and THR unwired (both read 0 V): TRIG below 1/3 vS resolves HIGH, so there is no phantom edge to excuse.
    const result = classifyMismatches(running, [{ sampleTime: 0, category: "digitalState", id: "u_sel/y0n", classicValue: 1, coreValue: 0, detail: "differs" }]);
    expect(result.known).toEqual([]);
    expect(result.unexplained).toHaveLength(1);
  });
});

describe("the capacitive enable-delay divergence (core/README.md, \"IC delay tail\")", () => {
  const entry = KNOWN_DIVERGENCES.find((d) => d.id === "ic-delay-tail-enable-carried-driver-capacitive-pin")!;
  const ls245 = ["dir", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "gnd", "b8", "b7", "b6", "b5", "b4", "b3", "b2", "b1", "/oe", "vcc"];
  const ls161 = ["/clr", "clk", "a", "b", "c", "d", "enp", "gnd", "rco", "qd", "qc", "qb", "qa", "/load", "ent", "vcc"];
  const board = (kind: "74ls245" | "74ls161", pin: string, loaded: boolean): SimCircuit =>
    circuit(
      [
        part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("u1", kind, kind === "74ls245" ? ls245 : ls161),
        part("cl", "capacitor", ["a", "b"], { capacitance: 100e-12 }),
        part("rl", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      [
        wire("vcc", "pos", "u1", "vcc"),
        wire("vcc", "neg", "u1", "gnd"),
        wire("u1", pin, loaded ? "cl" : "rl", "a"),
        wire(loaded ? "cl" : "rl", "b", "vcc", "neg"),
      ],
    );

  it("is registered", () => {
    expect(entry).toBeDefined();
    expect(entry.decisionRef).toContain("IC delay tail");
  });

  it.each([
    ["74ls245", "b1"],
    ["74ls245", "a4"],
    ["74ls161", "rco"],
  ] as const)("seeds the capacitor-loaded %s pin %s, and only it", (kind, pin) => {
    const seeds = entry.seeds(board(kind, pin, true));
    expect(seeds).toHaveLength(1);
    expect(seeds[0]!.componentId).toBe("u1");
    expect(seeds[0]!.outputPins).toEqual(new Set([pin]));
  });

  it.each([
    ["74ls245", "b1"],
    ["74ls161", "rco"],
  ] as const)("detector: the same %s pin %s on a resistor instead of a capacitor seeds nothing", (kind, pin) => {
    expect(entry.seeds(board(kind, pin, false))).toEqual([]);
  });

  it("detector: a capacitor on a 74LS161 count output (no delayed enable) seeds nothing", () => {
    expect(entry.seeds(board("74ls161", "qa", true))).toEqual([]);
  });

  it("excuses the loaded pin's net and the capacitor's current, and nothing on the rail", () => {
    const c = board("74ls245", "b1", true);
    const m = (category: Mismatch["category"], id: string): Mismatch => ({ sampleTime: 0, category, id, classicValue: 1, coreValue: 0, detail: "differs" });
    const pinNet = findNetIdForPin(c, "u1", "b1");
    const result = classifyMismatches(c, [m("netV", pinNet), m("elementI", "cl"), m("elementI", "vcc"), m("failures", "output_sag\u0000u1\u0000b1")]);
    expect(result.known.map((h) => `${h.mismatch.category}:${h.mismatch.id}`)).toEqual([`netV:${pinNet}`, "elementI:cl"]);
    expect(result.unexplained.map((x) => x.category)).toEqual(["elementI", "failures"]);
  });
});

// ---- R61 (a): a 74HC595 whose separate SRCLK and RCLK edges coincide ------

const HC595_PINS = ["vcc", "gnd", "/srclr", "srclk", "rclk", "/oe", "ser", "qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh", "qh2"];

type ClockDrive =
  | { kind: "pulse"; per: number; td: number; tr?: number; tf?: number; pw?: number; v2?: number }
  | { kind: "clock"; frequency: number; duty: number; delay?: number };

interface BoardOptions {
  srclk?: ClockDrive;
  rclk?: ClockDrive;
  tied?: boolean;
  /** A 555 whose OUT also sits on the SRCLK net. */
  ne555OnSrclk?: boolean;
  /** Feed the 595 and the LED from a battery_pack, with a far resistor on the battery rail. */
  batteryRail?: boolean;
  /** A second ideal supply in parallel with `vcc`. */
  parallelSupply?: boolean;
}

/**
 * engine-test-04's shape, cut down: SRCLK every 62.5 ms and RCLK every 500 ms,
 * both stepping at the same instants (0 s, 0.5 s, ...), `qb` lighting an LED
 * through a resistor, and `r_other` loading the rail with nothing to do with
 * the register.
 */
function hc595Board(options: BoardOptions = {}): SimCircuit {
  const drive = (id: string, d: ClockDrive) =>
    d.kind === "pulse"
      ? part(id, "pulse_source", ["pos", "neg"], { v1: 0, v2: d.v2 ?? 5, td: d.td, tr: d.tr ?? 0, tf: d.tf ?? 0, pw: d.pw ?? d.per / 2, per: d.per })
      : part(id, "clock", ["pos", "neg"], { frequency: d.frequency, duty: d.duty, delay: d.delay ?? 0 });
  const supply = options.batteryRail
    ? part("vcc", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 0.1, capacityAh: 2, charge: 1 })
    : part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 });
  const components = [
    supply,
    part("u1", "74hc595", HC595_PINS),
    drive("p_srclk", options.srclk ?? { kind: "pulse", per: 0.0625, td: 0 }),
    drive("p_rclk", options.rclk ?? { kind: "pulse", per: 0.5, td: 0 }),
    part("ser", "clock", ["out", "gnd"], { frequency: 1.7, duty: 0.55 }),
    part("r_qb", "resistor", ["a", "b"], { resistance: 1000 }),
    part("led_qb", "led", ["a", "k"]),
    part("r_other", "resistor", ["a", "b"], { resistance: 100 }),
  ];
  const wires = [
    wire("vcc", "pos", "u1", "vcc"),
    wire("vcc", "neg", "u1", "gnd"),
    wire("u1", "/srclr", "vcc", "pos"),
    wire("u1", "/oe", "u1", "gnd"),
    wire("p_srclk", "pos", "u1", "srclk"),
    wire("p_srclk", "neg", "u1", "gnd"),
    wire(options.tied ? "p_srclk" : "p_rclk", "pos", "u1", "rclk"),
    wire("p_rclk", "neg", "u1", "gnd"),
    wire("ser", "out", "u1", "ser"),
    wire("ser", "gnd", "u1", "gnd"),
    wire("u1", "qb", "r_qb", "a"),
    wire("r_qb", "b", "led_qb", "a"),
    wire("led_qb", "k", "u1", "gnd"),
    wire("r_other", "a", "vcc", "pos"),
    wire("r_other", "b", "u1", "gnd"),
  ];
  if (options.ne555OnSrclk) {
    components.push(part("t1", "ne555", ["1", "2", "3", "4", "5", "6", "7", "8"]));
    wires.push(wire("t1", "3", "u1", "srclk"), wire("t1", "8", "vcc", "pos"), wire("t1", "1", "u1", "gnd"));
  }
  if (options.batteryRail) {
    // A pull-up from `qb` to the rail, and a far load across the rail: a cone that crossed the rail would reach it.
    components.push(part("r_pu", "resistor", ["a", "b"], { resistance: 10000 }), part("r_far", "resistor", ["a", "b"], { resistance: 2200 }));
    wires.push(wire("r_pu", "a", "u1", "qb"), wire("r_pu", "b", "vcc", "pos"), wire("r_far", "a", "vcc", "pos"), wire("r_far", "b", "u1", "gnd"));
  }
  if (options.parallelSupply) {
    components.push(part("vcc2", "voltage_source", ["pos", "neg"], { voltage: 5 }));
    wires.push(wire("vcc2", "pos", "vcc", "pos"), wire("vcc2", "neg", "u1", "gnd"));
  }
  return circuit(components, wires);
}

function mm(category: Mismatch["category"], id: string, classicValue: unknown, coreValue: unknown, sampleTime = 0.625): Mismatch {
  return { sampleTime, category, id, classicValue, coreValue, detail: "differs" };
}

describe("w1h-74hc595-coincident-clock-capture (R61 (a))", () => {
  const registry = KNOWN_DIVERGENCES.filter((row) => row.id === "w1h-74hc595-coincident-clock-capture");
  const run = { endS: 2.5 };
  const netOf = (c: SimCircuit, comp: string, pin: string) => buildNets(c).find((n) => n.pins.some(([id, p]) => id === comp && p === pin))!.id;
  // What engine-test-04 reports at a coincident capture: `qb` latched LOW on classic and HIGH on the
  // core, its net, its LED's resistor and LED, the part's output_sag on it, and the rail current. The
  // core's 595 sources the LED's 4.005 mA from the rail where classic's sinks, so the supply delivers
  // exactly that much more (the net current leaving the rail into the cone).
  const capture = (c: SimCircuit, supplyDelta = 0.004005, t = 0.625): Mismatch[] => [
    mm("digitalState", "u1/qb", 0, 1, t),
    mm("netV", netOf(c, "u1", "qb"), 2.944, 4.7997, t),
    mm("elementI", "r_qb", 0.00033, 0.004005, t),
    mm("elementI", "led_qb", 0.00033, 0.004005, t),
    mm("failures", "output_sag\x00u1\x00qb", "absent", "present", t),
    mm("elementI", "vcc", -0.18533, -0.18533 - supplyDelta, t),
  ];
  const classify = (c: SimCircuit, ms: Mismatch[], r = run) => classifyMismatches(c, ms, registry, r);
  const unexplained = (c: SimCircuit, ms: Mismatch[], r = run) => classify(c, ms, r).unexplained.map((x) => `${x.category}:${x.id}`);

  it("is registered", () => {
    expect(registry).toHaveLength(1);
  });

  it("excuses the latched outputs, their loads, the output_sag on them and the rail current the loads draw", () => {
    const c = hc595Board();
    const result = classify(c, capture(c));
    expect(result.unexplained).toEqual([]);
    expect(result.known).toHaveLength(6);
  });

  it("does not excuse a 595 whose RCLK edges miss its SRCLK edges", () => {
    const c = hc595Board({ rclk: { kind: "pulse", per: 0.5, td: 0.001 } });
    expect(classify(c, capture(c)).known).toEqual([]);
  });

  it("does not excuse tied clocks (classic and the core already agree there)", () => {
    const c = hc595Board({ tied: true });
    expect(classify(c, capture(c)).known).toEqual([]);
  });

  it("still fails a failure elsewhere on the board, and the current it draws", () => {
    const c = hc595Board();
    const elsewhere = [mm("failures", "resistor_overload\x00r_other\x00", "absent", "present"), mm("elementI", "r_other", 0.05, 0.06)];
    expect(unexplained(c, [...capture(c), ...elsewhere])).toEqual(["failures:resistor_overload\x00r_other\x00", "elementI:r_other"]);
  });

  it("does not excuse a rail-current difference at an instant no excused load differs", () => {
    const c = hc595Board();
    expect(unexplained(c, [...capture(c), mm("elementI", "vcc", -0.18533, -0.189335, 1.25)])).toEqual(["elementI:vcc"]);
  });

  // H5bV finding 1: the allowance is the net current leaving the rail into the cone, signed and counted once.
  it("counts a series resistor and LED once: a rail difference near twice their current is not excused", () => {
    const c = hc595Board();
    expect(unexplained(c, capture(c, 0.006))).toEqual(["elementI:vcc"]);
  });

  it("does not excuse a rail difference of the opposite sign to the loads'", () => {
    const c = hc595Board();
    expect(unexplained(c, capture(c, -0.004005))).toEqual(["elementI:vcc"]);
  });

  it("shares the allowance across parallel supplies on the rail: two full shares are not excused", () => {
    const c = hc595Board({ parallelSupply: true });
    const both = [...capture(c), mm("elementI", "vcc2", -0.1, -0.1 - 0.004005)];
    expect(unexplained(c, both).sort()).toEqual(["elementI:vcc", "elementI:vcc2"]);
    const shared = [...capture(c, 0.002), mm("elementI", "vcc2", -0.1, -0.1 - 0.002005)];
    expect(unexplained(c, shared)).toEqual([]);
  });

  // Finding 2: a battery_pack or bench_psu rail is a fixed net the cone does not cross.
  it("does not reach across a battery_pack rail", () => {
    const c = hc595Board({ batteryRail: true });
    // The pull-up's own current changes with `qb` (it draws from the rail through its second pin), and the rail
    // delivers the LED's change less that: both are the cone's. The far load is not.
    const pullUp = mm("elementI", "r_pu", -0.0002056, -0.00002);
    expect(unexplained(c, [...capture(c, 0.004005 - 0.0001856), pullUp, mm("elementI", "r_far", 0.002, 0.0021)])).toEqual(["elementI:r_far"]);
  });

  // Finding 3: only a coincidence inside the run being compared seeds.
  it("does not seed on a coincidence past the end of the run", () => {
    const c = hc595Board({ rclk: { kind: "pulse", per: 1, td: 56 } });
    expect(classify(c, capture(c), { endS: 2.5 }).known).toEqual([]);
    expect(classify(c, capture(c, 0.004005, 56.1), { endS: 60 }).unexplained).toEqual([]);
  });

  it("does not seed without a run length", () => {
    const c = hc595Board();
    expect(classifyMismatches(c, capture(c), registry).known).toEqual([]);
  });

  // Finding 4: the seed follows what clocks the part.
  it("does not seed on an RCLK clock with duty 1 (it never falls, so it rises once)", () => {
    const c = hc595Board({ srclk: { kind: "pulse", per: 0.0625, td: 0 }, rclk: { kind: "clock", frequency: 2, duty: 1 } });
    expect(classify(c, capture(c)).known).toEqual([]);
  });

  it("does not seed on a pulse that never reaches Vih", () => {
    const c = hc595Board({ rclk: { kind: "pulse", per: 0.5, td: 0, v2: 3 } });
    expect(classify(c, capture(c)).known).toEqual([]);
  });

  it("compares the Vih crossings of resolved ramps, not their midpoints", () => {
    const ramp = (per: number, v2: number): ClockDrive => ({ kind: "pulse", per, td: 0, tr: 0.01, tf: 0.01, pw: 0.03, v2 });
    const same = hc595Board({ srclk: ramp(0.0625, 5), rclk: ramp(0.5, 5) });
    expect(classify(same, capture(same)).unexplained).toEqual([]);
    const steeper = hc595Board({ srclk: ramp(0.0625, 5), rclk: ramp(0.5, 10) });
    expect(classify(steeper, capture(steeper)).known).toEqual([]);
  });

  it("compares instants exactly, not to 1e-12", () => {
    const c = hc595Board({ rclk: { kind: "pulse", per: 0.5, td: 1e-15 } });
    expect(classify(c, capture(c)).known).toEqual([]);
  });

  it("does not seed a net another driver also drives (a 555's OUT)", () => {
    const c = hc595Board({ ne555OnSrclk: true });
    expect(classify(c, capture(c)).known).toEqual([]);
  });

  // H5bV re-check 1: a clock whose period is 0 or NaN cannot clock the part (and must not hang the seed).
  for (const frequency of [Infinity, Number.NaN]) {
    it(`does not seed on a clock of frequency ${String(frequency)}, and returns`, () => {
      const c = hc595Board({ srclk: { kind: "clock", frequency, duty: 0.5 }, rclk: { kind: "pulse", per: 0.5, td: 0 } });
      expect(classify(c, capture(c)).known).toEqual([]);
    });
  }

  // H5bV re-check 2: each parallel supply's change must go the cone's way, not only their sum.
  it("does not excuse parallel supplies whose changes have opposite signs, even when they sum to the cone's draw", () => {
    const c = hc595Board({ parallelSupply: true });
    const opposite = [...capture(c, 0.100005), mm("elementI", "vcc2", -0.1, -0.1 + 0.096)];
    expect(unexplained(c, opposite).sort()).toEqual(["elementI:vcc", "elementI:vcc2"]);
  });

  // H5bV re-check 3: the first coincidence is found from the two schedules, not by walking every edge.
  it("finds no coincidence between two 10 MHz clocks 1 ns apart over 1000 s in well under a second", () => {
    const c = hc595Board({ srclk: { kind: "clock", frequency: 1e7, duty: 0.5 }, rclk: { kind: "clock", frequency: 1e7, duty: 0.5, delay: 1e-9 } });
    const t0 = performance.now();
    expect(classify(c, capture(c), { endS: 1000 }).known).toEqual([]);
    expect(performance.now() - t0).toBeLessThan(1000);
  });

  it("finds a late coincidence of two fast schedules without walking to it", () => {
    // 2^23 Hz and 2^22 Hz clocks, the second delayed 512 s: their rises first meet at 512 s (every instant is an
    // exact binary fraction), four billion SRCLK edges in.
    const c = hc595Board({ srclk: { kind: "clock", frequency: 8_388_608, duty: 0.5 }, rclk: { kind: "clock", frequency: 4_194_304, duty: 0.5, delay: 512 } });
    const t0 = performance.now();
    expect(classify(c, capture(c, 0.004005, 600), { endS: 1000 }).unexplained).toEqual([]);
    expect(classify(c, capture(c, 0.004005, 600), { endS: 400 }).known).toEqual([]);
    expect(performance.now() - t0).toBeLessThan(1000);
  });

  // H5bV re-check of d8273fa, 1: the core compares computed float instants, so edges whose real values differ but
  // round to the same double coincide (0.04 + 0.1 and 0.08 + 6 * 0.01 are both 0.14000000000000001).
  const repros: Array<{ name: string; srclk: ClockDrive; rclk: ClockDrive; endS: number }> = [
    { name: "f=10 delay 0.04 against f=100 delay 0.08", srclk: { kind: "clock", frequency: 10, duty: 0.5, delay: 0.04 }, rclk: { kind: "clock", frequency: 100, duty: 0.5, delay: 0.08 }, endS: 3 },
    { name: "f=25 delay 0.08 against f=2 delay 0.1", srclk: { kind: "clock", frequency: 25, duty: 0.5, delay: 0.08 }, rclk: { kind: "clock", frequency: 2, duty: 0.5, delay: 0.1 }, endS: 7 },
    { name: "a pulse of period 1/3 and td 0.48 against period 0.16 and td 0", srclk: { kind: "pulse", per: 1 / 3, td: 0.48 }, rclk: { kind: "pulse", per: 0.16, td: 0 }, endS: 3 },
    { name: "f=50 delay 0.16 against f=3 delay 0", srclk: { kind: "clock", frequency: 50, duty: 0.5, delay: 0.16 }, rclk: { kind: "clock", frequency: 3, duty: 0.5, delay: 0 }, endS: 7 },
  ];
  for (const r of repros) {
    it(`seeds on edges that meet as computed floats though their real values differ: ${r.name}`, () => {
      const c = hc595Board({ srclk: r.srclk, rclk: r.rclk });
      expect(classify(c, capture(c, 0.004005, 1), { endS: r.endS }).unexplained).toEqual([]);
    });
  }

  it("agrees with a walk over every edge on 600 random periodic pairs", () => {
    // A seeded generator (mulberry32) over decimal periods and delays, where real-value misses and float
    // coincidences are common. The reference walks both schedules edge by edge, comparing doubles.
    let seed = 0x5eed61;
    const rand = () => {
      seed = (seed + 0x6d2b79f5) | 0;
      let t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
    const pick = <T,>(xs: readonly T[]): T => xs[Math.floor(rand() * xs.length)]!;
    const schedule = (): RiseSchedule => {
      const per = rand() < 0.5 ? 1 / pick([2, 3, 5, 6, 7, 10, 12, 20, 25, 30, 50, 60, 100]) : pick([1, 2, 3, 4, 5, 6, 8, 12, 16, 25]) * pick([0.01, 0.02, 0.04, 0.1]);
      const base = pick([0, 1, 2, 3, 4, 6, 8, 12, 16]) * pick([0.01, 0.02, 0.04, 0.1]);
      return { base, per, periodic: true, offsets: rand() < 0.2 ? [pick([0.001, 0.003, 0.005])] : [] };
    };
    const walk = (a: RiseSchedule, b: RiseSchedule, endS: number): boolean => {
      let i = 0, j = 0;
      for (;;) {
        const ta = riseAt(a, i), tb = riseAt(b, j);
        if (!(ta <= endS) || !(tb <= endS)) return false;
        if (ta === tb) return true;
        if (ta < tb) i++;
        else j++;
      }
    };
    let met = 0;
    const disagree: string[] = [];
    for (let k = 0; k < 600; k++) {
      const a = schedule(), b = schedule(), endS = pick([1, 3, 7, 10]);
      const want = walk(a, b, endS);
      if (want) met++;
      if (schedulesCoincide(a, b, endS) !== want) disagree.push(JSON.stringify({ a, b, endS, want }));
    }
    expect(disagree).toEqual([]);
    // The sample must exercise both answers, float-only coincidences among them.
    expect(met).toBeGreaterThan(50);
    expect(600 - met).toBeGreaterThan(50);
  });

  // H5bV re-check of d8273fa, 2: a supply whose change is exactly 0 goes no way, so it is compatible with any sign.
  it("excuses the rail when a parallel supply's change is exactly 0", () => {
    const c = hc595Board({ parallelSupply: true });
    expect(unexplained(c, [...capture(c), mm("elementI", "vcc2", -0.1, -0.1)])).toEqual([]);
  });

  // Finding 5: failures only at excused instants, and only the kinds the coincidence causes.
  it("does not excuse an output_sag at an instant nothing else was excused", () => {
    const c = hc595Board();
    expect(unexplained(c, [...capture(c), mm("failures", "output_sag\x00u1\x00qb", "absent", "present", 1.25)])).toEqual([
      "failures:output_sag\x00u1\x00qb",
    ]);
  });

  it("does not excuse a load's failure of a kind the coincidence cannot cause", () => {
    const c = hc595Board();
    expect(unexplained(c, [...capture(c), mm("failures", "output_sag\x00r_qb\x00a", "absent", "present")])).toEqual([
      "failures:output_sag\x00r_qb\x00a",
    ]);
  });
});
