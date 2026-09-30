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
