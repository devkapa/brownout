/**
 * S15.5 Wave 2.4 — 7-segment display electrical load tests.
 *
 * Each segment is a real Shockley diode (Vf ~2 V, if_max 20 mA).
 * Tests cover:
 *   (a) seg7_cc conducting ~9 mA through 330 ohm series resistor, segment lit
 *   (b) undriven segment: ~0 current, unlit
 *   (c) seg7_ca mirrored polarity: common anode to 5 V, segment sunk to GND
 *   (d) 74LS47 (open-collector) + seg7_ca + 330 ohm series: mA-scale, lit
 *   (e) CD4511 (push-pull) + seg7_cc + 330 ohm series: mA-scale, lit
 *   (f) Guard: the static led-no-resistor diagnostic does NOT fire critical for seg7
 */

import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";
import { buildNets } from "../../../src/sim/engine/graph.js";
import { breadboardToSimCircuit } from "../../helpers/circuit/breadboard.js";
import { runDiagnostics } from "../../helpers/diagnostics/index.js";
import rawCatalog from "../../helpers/catalog.js";
import type { Circuit, PartCatalog } from "../../../src/circuit/types.js";

const catalog = rawCatalog as unknown as PartCatalog;

// ─── Shared helpers ────────────────────────────────────────────────────────────

function enginePin(id: string): SimCircuit["components"][number]["pins"][number] {
  return { id };
}

function engineWire(
  fromComponent: string,
  fromPin: string,
  toComponent: string,
  toPin: string,
): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function vSource(id: string, voltage: number): SimCircuit["components"][number] {
  return {
    id, kind: "voltage_source",
    position: { x: 0, y: 0 }, rotation: 0,
    pins: [enginePin("pos"), enginePin("neg")],
    params: { voltage },
  };
}

function resistor(id: string, resistance: number): SimCircuit["components"][number] {
  return {
    id, kind: "resistor",
    position: { x: 0, y: 0 }, rotation: 0,
    pins: [enginePin("a"), enginePin("b")],
    params: { resistance },
  };
}

function seg7_cc(id: string): SimCircuit["components"][number] {
  return {
    id, kind: "seg7_cc",
    position: { x: 0, y: 0 }, rotation: 0,
    pins: [
      enginePin("g"), enginePin("f"), enginePin("com"),
      enginePin("a"), enginePin("b"),
      enginePin("e"), enginePin("d"), enginePin("com2"),
      enginePin("c"), enginePin("dp"),
    ],
    params: {},
  };
}

function seg7_ca(id: string): SimCircuit["components"][number] {
  return {
    id, kind: "seg7_ca",
    position: { x: 0, y: 0 }, rotation: 0,
    pins: [
      enginePin("g"), enginePin("f"), enginePin("com"),
      enginePin("a"), enginePin("b"),
      enginePin("e"), enginePin("d"), enginePin("com2"),
      enginePin("c"), enginePin("dp"),
    ],
    params: {},
  };
}

function ls47Ic(id: string): SimCircuit["components"][number] {
  return {
    id, kind: "74ls47",
    position: { x: 0, y: 0 }, rotation: 0,
    pins: [
      enginePin("b"), enginePin("c"),
      enginePin("lt_n"), enginePin("rbo_n"), enginePin("rbi_n"),
      enginePin("d"), enginePin("a"),
      enginePin("gnd"),
      enginePin("e_out"), enginePin("d_out"), enginePin("c_out"),
      enginePin("b_out"), enginePin("a_out"),
      enginePin("g_out"), enginePin("f_out"),
      enginePin("vcc"),
    ],
    params: {},
  };
}

function cd4511Ic(id: string): SimCircuit["components"][number] {
  return {
    id, kind: "cd4511",
    position: { x: 0, y: 0 }, rotation: 0,
    pins: [
      enginePin("b"), enginePin("c"), enginePin("lt_n"), enginePin("bl_n"),
      enginePin("le"), enginePin("d"), enginePin("a"),
      enginePin("gnd"),
      enginePin("e_out"), enginePin("d_out"), enginePin("c_out"),
      enginePin("b_out"), enginePin("a_out"), enginePin("g_out"), enginePin("f_out"),
      enginePin("vcc"),
    ],
    params: {},
  };
}

function runEngine(circuit: SimCircuit, steps = 5): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(1e-4);
  return engine;
}

describe("7-segment package common connectivity", () => {
  it.each(["seg7_cc", "seg7_ca"] as const)(
    "%s rejects conflicting voltages on its duplicate COM leads",
    (kind) => {
      const circuit: SimCircuit = {
        components: [
          kind === "seg7_cc" ? seg7_cc("disp") : seg7_ca("disp"),
          vSource("vcc", 5),
        ],
        wires: [
          engineWire("vcc", "pos", "disp", "com"),
          engineWire("vcc", "neg", "disp", "com2"),
        ],
      };
      const engine = new SimEngine();

      engine.load(circuit);

      // COM and COM2 are duplicate leads bonded to one internal node. An ideal
      // source across them is a conflicting constraint, never an ignored pin.
      expect(engine.lastConverged).toBe(false);
    },
  );
});

// ─── (a) seg7_cc: 5 V → 330 Ω → segment a → common → GND ───────────────────

describe("seg7_cc electrical load — series resistor", () => {
  /**
   * Circuit: 5 V supply → 330 Ω → segment a pin → [internal diode] → com → GND.
   * Expected: I = (5 - 2) / 330 ≈ 9.1 mA (ideal).  Shockley model gives ~8–10 mA.
   * Segment a must be lit (current >= 1 mA threshold).
   */
  it("segment a conducts ~9 mA and is reported lit via getElementChannelI", () => {
    const circuit: SimCircuit = {
      components: [
        seg7_cc("disp"),
        vSource("vcc", 5),
        resistor("r_a", 330),
      ],
      wires: [
        // Supply
        engineWire("vcc", "pos", "r_a", "a"),
        engineWire("r_a", "b",   "disp", "a"),
        // Common cathode to ground
        engineWire("vcc", "neg", "disp", "com"),
      ],
    };
    const engine = runEngine(circuit, 8);
    const ch = engine.getElementChannelI()["disp"];
    expect(ch).toBeDefined();
    // Index 0 = segment a (canonical order [a,b,c,d,e,f,g,dp])
    const iA = ch![0]!;
    // ~(5-2)/330 ≈ 9.1 mA; Shockley model lands between 7 and 11 mA
    expect(iA).toBeGreaterThan(0.007);
    expect(iA).toBeLessThan(0.011);
    // Segment is lit above the 1 mA threshold
    expect(iA).toBeGreaterThan(1e-3);
  });

  it("total element current equals the single lit segment current", () => {
    const circuit: SimCircuit = {
      components: [
        seg7_cc("disp"),
        vSource("vcc", 5),
        resistor("r_a", 330),
      ],
      wires: [
        engineWire("vcc", "pos", "r_a", "a"),
        engineWire("r_a", "b",   "disp", "a"),
        engineWire("vcc", "neg", "disp", "com"),
      ],
    };
    const engine = runEngine(circuit, 8);
    const ch = engine.getElementChannelI()["disp"]!;
    const totalI = engine.getElementI()["disp"] ?? 0;
    // Only segment a is connected; total should match channel a within noise
    expect(Math.abs(totalI - ch[0]!)).toBeLessThan(1e-6);
  });
});

// ─── (b) Undriven segment: ~0 current, unlit ─────────────────────────────────

describe("seg7_cc — undriven segment", () => {
  /**
   * Only segment a is wired; segments b-dp are not connected.
   * Channel current for unconnected segments must be ~0.
   */
  it("unconnected segments report ~0 current", () => {
    const circuit: SimCircuit = {
      components: [
        seg7_cc("disp"),
        vSource("vcc", 5),
        resistor("r_a", 330),
      ],
      wires: [
        engineWire("vcc", "pos", "r_a", "a"),
        engineWire("r_a", "b",   "disp", "a"),
        engineWire("vcc", "neg", "disp", "com"),
        // segments b-dp intentionally not wired
      ],
    };
    const engine = runEngine(circuit, 8);
    const ch = engine.getElementChannelI()["disp"]!;
    // Indices 1-7: b, c, d, e, f, g, dp — all unconnected
    for (let idx = 1; idx <= 7; idx++) {
      expect(ch[idx] ?? 0).toBeLessThan(1e-6);
    }
  });

  it("a seg7_cc with no pins wired has zero channel current", () => {
    const circuit: SimCircuit = {
      components: [
        seg7_cc("disp"),
        vSource("vcc", 5),
      ],
      wires: [],
    };
    const engine = runEngine(circuit, 3);
    const ch = engine.getElementChannelI()["disp"];
    // No com pin wired — channel array may be absent or all zeros
    if (ch !== undefined) {
      for (const I of ch) expect(I).toBeLessThan(1e-9);
    }
  });
});

// ─── (c) seg7_ca: mirrored polarity ──────────────────────────────────────────

describe("seg7_ca electrical load — mirrored polarity", () => {
  /**
   * Circuit: common anode (com) to 5 V; segment a pin → 330 Ω → GND (sinking).
   * Current flows from com through the internal diode to segment a, then through R to GND.
   * Expected: same ~9 mA as the CC case (same diode, same V/R).
   */
  it("segment a conducts ~9 mA with common-anode polarity", () => {
    const circuit: SimCircuit = {
      components: [
        seg7_ca("disp"),
        vSource("vcc", 5),
        resistor("r_a", 330),
      ],
      wires: [
        // Common anode to VCC
        engineWire("vcc", "pos", "disp", "com"),
        // Segment a cathode sinks through resistor to GND
        engineWire("disp", "a", "r_a", "a"),
        engineWire("r_a",  "b", "vcc", "neg"),
      ],
    };
    const engine = runEngine(circuit, 8);
    const ch = engine.getElementChannelI()["disp"];
    expect(ch).toBeDefined();
    const iA = ch![0]!; // index 0 = segment a
    expect(iA).toBeGreaterThan(0.007);
    expect(iA).toBeLessThan(0.011);
    expect(iA).toBeGreaterThan(1e-3);
  });

  it("reverse polarity (CC wired as CA) produces near-zero current", () => {
    // Wire a seg7_cc with common-anode topology — diodes reverse-biased, no current
    const circuit: SimCircuit = {
      components: [
        seg7_cc("disp"),
        vSource("vcc", 5),
        resistor("r_a", 330),
      ],
      wires: [
        // Wrong polarity: common to VCC, segment to GND via resistor
        engineWire("vcc", "pos", "disp", "com"),
        engineWire("disp", "a", "r_a", "a"),
        engineWire("r_a",  "b", "vcc", "neg"),
      ],
    };
    const engine = runEngine(circuit, 8);
    const ch = engine.getElementChannelI()["disp"];
    // Reverse-biased diode: current should be essentially zero
    if (ch !== undefined) {
      expect(ch[0] ?? 0).toBeLessThan(1e-6);
    }
  });
});

// ─── (d) 74LS47 + seg7_ca + 330 ohm: open-collector sinking ─────────────────

describe("74LS47 + seg7_ca — digit 1 (segments b and c lit)", () => {
  /**
   * 74LS47 is an open-collector active-LOW BCD decoder for common-anode displays.
   * Digit 1: BCD = 0001, segments b and c active (outputs pulled LOW by transistor).
   *
   * Topology:
   *   VCC → COM (seg7_ca)
   *   seg_b_pin → 330 Ω → 74LS47 b_out   (transistor sinks to GND when ON)
   *   seg_a_pin → 330 Ω → 74LS47 a_out   (transistor OFF = released, pulls up via R)
   *
   * For segment b (lit): I ≈ (5 - 2) / 330 ≈ 9 mA, channelI[1] >= 1 mA
   * For segment a (unlit): output released, net pulled high, reverse-biased → ~0 mA
   */
  it("lit segment (b) draws mA-scale current via 74LS47 open-collector sink", () => {
    const circuit: SimCircuit = {
      components: [
        seg7_ca("disp"),
        ls47Ic("u1"),
        vSource("vcc", 5),
        // BCD = 0001 (digit 1): A=1, B=0, C=0, D=0
        vSource("va", 5),
        vSource("vb", 0),
        vSource("vc", 0),
        vSource("vd", 0),
        resistor("r_b", 330),  // series resistor for segment b
        resistor("r_a", 330),  // series resistor for segment a (unlit)
      ],
      wires: [
        // Power the 74LS47
        engineWire("vcc", "pos", "u1", "vcc"),
        engineWire("vcc", "neg", "u1", "gnd"),
        // BCD inputs
        engineWire("va", "pos", "u1", "a"),   engineWire("va", "neg", "vcc", "neg"),
        engineWire("vb", "pos", "u1", "b"),   engineWire("vb", "neg", "vcc", "neg"),
        engineWire("vc", "pos", "u1", "c"),   engineWire("vc", "neg", "vcc", "neg"),
        engineWire("vd", "pos", "u1", "d"),   engineWire("vd", "neg", "vcc", "neg"),
        // /LT and /RBI high (inactive)
        engineWire("vcc", "pos", "u1", "lt_n"),
        engineWire("vcc", "pos", "u1", "rbi_n"),
        // Common anode to VCC
        engineWire("vcc", "pos", "disp", "com"),
        // Segment b: disp.b → r_b → u1.b_out (transistor sinks LOW for digit 1)
        engineWire("disp", "b", "r_b", "a"),
        engineWire("r_b", "b", "u1", "b_out"),
        // Segment a: disp.a → r_a → u1.a_out (transistor OFF = released for digit 1)
        engineWire("disp", "a", "r_a", "a"),
        engineWire("r_a", "b", "u1", "a_out"),
      ],
    };
    const engine = runEngine(circuit, 8);
    const ch = engine.getElementChannelI()["disp"];
    expect(ch).toBeDefined();

    // Segment b (index 1): 74LS47 sinks LOW → current flows → lit
    const iB = ch![1]!;
    expect(iB).toBeGreaterThan(1e-3); // >= 1 mA = lit

    // Segment a (index 0): 74LS47 transistor off → high impedance → no current
    const iA = ch![0]!;
    expect(iA).toBeLessThan(1e-4); // < 0.1 mA = unlit
  });
});

// ─── (e) CD4511 + seg7_cc + 330 ohm: push-pull sourcing ─────────────────────

describe("CD4511 + seg7_cc — digit 1 (segments b and c lit)", () => {
  /**
   * CD4511 is a push-pull active-HIGH BCD decoder for common-cathode displays.
   * Digit 1: BCD = 0001, segments b and c HIGH (pushed to ~VCC).
   *
   * Topology:
   *   COM → GND (common cathode)
   *   u1.b_out → 330 Ω → disp.b   (push-pull drives HIGH when segment is ON)
   *   u1.a_out → 330 Ω → disp.a   (push-pull drives LOW when segment is OFF)
   *
   * For segment b (lit): I ≈ (5 - 2) / 330 ≈ 9 mA, channelI[1] >= 1 mA
   * For segment a (unlit): output LOW → no forward bias → ~0 mA
   */
  it("lit segment (b) draws mA-scale current via CD4511 push-pull source", () => {
    const circuit: SimCircuit = {
      components: [
        seg7_cc("disp"),
        cd4511Ic("u1"),
        vSource("vcc", 5),
        // BCD = 0001 (digit 1): A=1, B=0, C=0, D=0
        vSource("va", 5),
        vSource("vb", 0),
        vSource("vc", 0),
        vSource("vd", 0),
        vSource("vle", 0),   // LE LOW = transparent (latch passthrough)
        resistor("r_b", 330),
        resistor("r_a", 330),
      ],
      wires: [
        engineWire("vcc", "pos", "u1", "vcc"),
        engineWire("vcc", "neg", "u1", "gnd"),
        engineWire("va", "pos", "u1", "a"),   engineWire("va", "neg", "vcc", "neg"),
        engineWire("vb", "pos", "u1", "b"),   engineWire("vb", "neg", "vcc", "neg"),
        engineWire("vc", "pos", "u1", "c"),   engineWire("vc", "neg", "vcc", "neg"),
        engineWire("vd", "pos", "u1", "d"),   engineWire("vd", "neg", "vcc", "neg"),
        engineWire("vle", "pos", "u1", "le"), engineWire("vle", "neg", "vcc", "neg"),
        // /LT and /BL left open — resolved HIGH (inactive) by IC_OPEN_HIGH_PINS
        // Common cathode to GND
        engineWire("vcc", "neg", "disp", "com"),
        // Segment b: u1.b_out → r_b → disp.b
        engineWire("u1", "b_out", "r_b", "a"),
        engineWire("r_b", "b", "disp", "b"),
        // Segment a: u1.a_out → r_a → disp.a
        engineWire("u1", "a_out", "r_a", "a"),
        engineWire("r_a", "b", "disp", "a"),
      ],
    };
    const engine = runEngine(circuit, 8);
    const ch = engine.getElementChannelI()["disp"];
    expect(ch).toBeDefined();

    // Segment b (index 1): CD4511 drives HIGH → forward biased → mA-scale current
    const iB = ch![1]!;
    expect(iB).toBeGreaterThan(1e-3);

    // Segment a (index 0): CD4511 drives LOW → no forward bias → near-zero
    const iA = ch![0]!;
    expect(iA).toBeLessThan(1e-4);
  });
});

// ─── (f) Static diagnostic guard: led-no-resistor is WARNING, not CRITICAL ──

describe("static diagnostic guard — seg7 without series resistor", () => {
  /**
   * Guard: the led-no-resistor finding is severity "warning", not "critical".
   * The fixture-03 known-good test already verifies zero criticals; this focused
   * unit test confirms that a seg7 wired without a resistor fires only a warning
   * so it cannot break the zero-criticals assertion for official fixtures.
   */

  function seg7NakedCircuit(): Circuit {
    // Minimal Circuit (breadboard-style) with seg7_cc wired direct — no resistor.
    return {
      components: [
        {
          id: "disp",
          kind: "seg7_cc",
          position: { x: 0, y: 0 },
          rotation: 0,
          pins: [
            { id: "g", label: "g", offset: { x: 0, y: 0 } },
            { id: "f", label: "f", offset: { x: 19, y: 0 } },
            { id: "com", label: "COM", offset: { x: 38, y: 0 } },
            { id: "a", label: "a", offset: { x: 57, y: 0 } },
            { id: "b", label: "b", offset: { x: 76, y: 0 } },
            { id: "e", label: "e", offset: { x: 0, y: 131 } },
            { id: "d", label: "d", offset: { x: 19, y: 131 } },
            { id: "com2", label: "COM", offset: { x: 38, y: 131 } },
            { id: "c", label: "c", offset: { x: 57, y: 131 } },
            { id: "dp", label: "dp", offset: { x: 76, y: 131 } },
          ],
          params: {},
        },
        {
          id: "bat",
          kind: "battery_pack",
          position: { x: 0, y: 0 },
          rotation: 0,
          pins: [
            { id: "pos", label: "+", offset: { x: 0, y: 0 } },
            { id: "neg", label: "-", offset: { x: 0, y: 19 } },
          ],
          params: { voltage: 5, cells: 3 },
        },
      ],
      wires: [
        {
          id: "w1",
          from_component: "bat",
          from_pin: "pos",
          to_component: "disp",
          to_pin: "a",
          resistance: 0,
        },
        {
          id: "w2",
          from_component: "bat",
          from_pin: "neg",
          to_component: "disp",
          to_pin: "com",
          resistance: 0,
        },
      ],
    } as Circuit;
  }

  it("seg7 without series resistor fires led-no-resistor as WARNING not CRITICAL", () => {
    const circuit = seg7NakedCircuit();
    const simCircuit = breadboardToSimCircuit(circuit);
    const nets = buildNets(simCircuit);
    const findings = runDiagnostics({ circuit: simCircuit, nets, catalog });
    const noResistor = findings.filter((f) => f.id === "led-no-resistor");
    // Must fire because no resistor is present
    expect(noResistor.length).toBeGreaterThan(0);
    // Must be warning severity — NOT critical
    for (const f of noResistor) {
      expect(f.severity).toBe("warning");
      expect(f.severity).not.toBe("critical");
    }
  });

  it("seg7 with series resistor does NOT fire led-no-resistor", () => {
    const circuit = seg7NakedCircuit();
    // Add a resistor in series
    (circuit.components as Circuit["components"]).push({
      id: "r1",
      kind: "resistor",
      position: { x: 0, y: 0 },
      rotation: 0,
      pins: [
        { id: "a", label: "a", offset: { x: 0, y: 0 } },
        { id: "b", label: "b", offset: { x: 19, y: 0 } },
      ],
      params: { resistance: 330 },
    });
    // Re-route bat.pos → r1.a → r1.b → disp.a  (instead of direct bat → disp.a)
    const wires = circuit.wires as Circuit["wires"];
    wires[0] = { ...wires[0]!, to_component: "r1", to_pin: "a" };
    wires.push({
      id: "w3",
      from_component: "r1",
      from_pin: "b",
      to_component: "disp",
      to_pin: "a",
      resistance: 0,
    });
    const simCircuit = breadboardToSimCircuit(circuit);
    const nets = buildNets(simCircuit);
    const findings = runDiagnostics({ circuit: simCircuit, nets, catalog });
    const noResistor = findings.filter((f) => f.id === "led-no-resistor");
    expect(noResistor).toHaveLength(0);
  });
});
