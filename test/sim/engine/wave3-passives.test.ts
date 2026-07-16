/**
 * Wave 3 passive-parts tests: ferrite bead, resistor array, capacitor variants,
 * and electrolytic reverse-polarity diagnostic.
 *
 * All numeric expected values are derived from first-principles physics (Ohm's
 * law) or measured from the engine during test authoring.  No constant is copied
 * from the implementation source.
 *
 * Sizing reasoning for ferrite bead test:
 *   Supply = 5 V, bead rDc = 0.5 Ω, load = 220 Ω.
 *   Total R = 220.5 Ω.  I = 5 / 220.5 ≈ 22.68 mA.
 *   V_drop_bead = 0.5 × 22.68e-3 ≈ 11.3 mV.
 *   P_load = 22.68e-3² × 220 ≈ 0.113 W < 0.25 W catalog p_max — safe.
 *
 * Sizing reasoning for resistor array tests:
 *   Each channel: 5 V supply through 10 kΩ → I = 0.5 mA.
 *   Isolation: two channels driven from different supplies — no cross-conduction.
 */

import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";
import { buildNets } from "../../../src/sim/engine/graph.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { analyzeLive } from "../../helpers/diagnostics/live.js";
import type { DiagnosticsInput, LiveReadings } from "../../helpers/diagnostics/index.js";

const catalog = rawCatalog as unknown as PartCatalog;

// ─── Helpers ──────────────────────────────────────────────────────────────────

function runEngine(circuit: SimCircuit, steps = 50, dt = 1e-4): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(dt);
  return engine;
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) =>
    n.pins.some(([cid, pid]) => cid === componentId && pid === pinId),
  );
  if (!net) return 0;
  return engine.getNetV()[net.id] ?? 0;
}

function liveFromEngine(engine: SimEngine): LiveReadings {
  return {
    netV: engine.getNetV(),
    elementI: engine.getElementI(),
    elementChannelI: engine.getElementChannelI(),
    digitalState: engine.digitalState,
    simTime: engine.simTime,
    converged: engine.lastConverged,
    failures: engine.getFailures(),
  };
}

function makeInput(circuit: SimCircuit, live: LiveReadings): DiagnosticsInput {
  return {
    circuit: circuit as unknown as DiagnosticsInput["circuit"],
    nets: buildNets(circuit),
    catalog,
    live,
  };
}

// ─── Part 1: Ferrite bead ─────────────────────────────────────────────────────

describe("ferrite_bead — DC series resistance model", () => {
  /**
   * Circuit: 5 V → ferrite bead (rDc=0.5 Ω) → 220 Ω load → GND.
   *
   * Expected:
   *   I = 5 / (0.5 + 220) = 5 / 220.5 ≈ 22.68 mA
   *   V_bead = 0.5 × 22.68e-3 ≈ 11.3 mV
   *   V_load = 220 × 22.68e-3 ≈ 4.989 V
   *
   * P_load = 22.68e-3² × 220 ≈ 0.113 W — below the 0.25 W catalog p_max for
   * the 220 Ω resistor, so no failure-framework interference.
   */
  it("conducts I ≈ 22–23.5 mA through a 5 V / 220 Ω circuit", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "fb1", kind: "ferrite_bead", pins: [{ id: "a" }, { id: "b" }], params: { rDc: 0.5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "fb1", to_pin: "a" },
        { from_component: "fb1", from_pin: "b", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    const i_mA = (eng.getElementI()["fb1"] ?? 0) * 1000;
    // I = 5 / 220.5 ≈ 22.68 mA
    expect(i_mA).toBeGreaterThan(22);
    expect(i_mA).toBeLessThan(23.5);
  });

  it("bead voltage drop ≈ rDc × I (Ohm's law verified)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "fb1", kind: "ferrite_bead", pins: [{ id: "a" }, { id: "b" }], params: { rDc: 0.5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "fb1", to_pin: "a" },
        { from_component: "fb1", from_pin: "b", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    const vA = netVoltage(eng, "fb1", "a");  // ≈ 5 V
    const vB = netVoltage(eng, "fb1", "b");  // ≈ 5 - 0.01134 V
    const vDrop_mV = (vA - vB) * 1000;
    // V_drop = 0.5 × 22.68e-3 ≈ 11.34 mV
    expect(vDrop_mV).toBeGreaterThan(9);
    expect(vDrop_mV).toBeLessThan(14);
  });

  it("catalog entry exists and is kind ferrite_bead", () => {
    const part = catalog.parts.find((p) => p.uid === "ferrite-bead");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("ferrite_bead");
    expect(part?.bom).toBeDefined();
    expect(part?.default_params.rDc).toBe(0.5);
  });
});

// ─── Part 2: Resistor array ───────────────────────────────────────────────────

describe("resistor_array — four independent channels", () => {
  /**
   * Circuit A: 5 V source → resistor array channel 1 (a1–b1, 10 kΩ) → GND.
   * Expected I_ch1 = 5 / 10000 = 0.5 mA.  Channels 2–4 are open (no load).
   */
  it("channel 1 conducts 0.5 mA with 5V / 10 kΩ", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "ra1", kind: "resistor_array", pins: [
          { id: "a1" }, { id: "b1" }, { id: "a2" }, { id: "b2" },
          { id: "a3" }, { id: "b3" }, { id: "a4" }, { id: "b4" },
        ], params: { resistance: 10000 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "ra1", to_pin: "a1" },
        { from_component: "ra1", from_pin: "b1", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    const channels = eng.getElementChannelI()["ra1"];
    expect(channels).toBeDefined();
    expect(channels).toHaveLength(4);
    const i_ch1_mA = Math.abs((channels?.[0] ?? 0) * 1000);
    // I = 5 / 10000 = 0.5 mA
    expect(i_ch1_mA).toBeGreaterThan(0.45);
    expect(i_ch1_mA).toBeLessThan(0.55);
  });

  it("unconnected channels carry ≈ 0 mA (isolation)", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "ra1", kind: "resistor_array", pins: [
          { id: "a1" }, { id: "b1" }, { id: "a2" }, { id: "b2" },
          { id: "a3" }, { id: "b3" }, { id: "a4" }, { id: "b4" },
        ], params: { resistance: 10000 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "ra1", to_pin: "a1" },
        { from_component: "ra1", from_pin: "b1", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    const channels = eng.getElementChannelI()["ra1"] ?? [];
    // Channels 2–4 have no voltage across them — current is 0 (open pins → GND net on both sides)
    for (let i = 1; i < 4; i++) {
      expect(Math.abs((channels[i] ?? 0) * 1000)).toBeLessThan(0.01);
    }
  });

  it("all four channels conduct independently with individual supplies", () => {
    /**
     * Four separate 5V sources, each driving one channel.  Expected: each channel
     * carries 5/10000 = 0.5 mA independently.  No cross-conduction between channels.
     *
     * Each supply has its own neg pin wired to GND (v1..v4 share one GND wire).
     * This is equivalent to four separate resistors.
     */
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "v2", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "v3", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "v4", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "ra1", kind: "resistor_array", pins: [
          { id: "a1" }, { id: "b1" }, { id: "a2" }, { id: "b2" },
          { id: "a3" }, { id: "b3" }, { id: "a4" }, { id: "b4" },
        ], params: { resistance: 10000 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "ra1", to_pin: "a1" },
        { from_component: "ra1", from_pin: "b1", to_component: "v1", to_pin: "neg" },
        { from_component: "v2", from_pin: "pos", to_component: "ra1", to_pin: "a2" },
        { from_component: "ra1", from_pin: "b2", to_component: "v2", to_pin: "neg" },
        { from_component: "v3", from_pin: "pos", to_component: "ra1", to_pin: "a3" },
        { from_component: "ra1", from_pin: "b3", to_component: "v3", to_pin: "neg" },
        { from_component: "v4", from_pin: "pos", to_component: "ra1", to_pin: "a4" },
        { from_component: "ra1", from_pin: "b4", to_component: "v4", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    const channels = eng.getElementChannelI()["ra1"] ?? [];
    for (let i = 0; i < 4; i++) {
      const i_mA = Math.abs((channels[i] ?? 0) * 1000);
      // Each: 5 V / 10 kΩ = 0.5 mA
      expect(i_mA, `channel ${i + 1} current`).toBeGreaterThan(0.45);
      expect(i_mA, `channel ${i + 1} current`).toBeLessThan(0.55);
    }
  });

  it("no cross-conduction between channels at different potentials", () => {
    /**
     * Channel 1: 5 V supply. Channel 2: 3.3 V supply. Both return to GND.
     * If the channels were connected, channel 2's b2 would see 5 V from channel 1's
     * b1 (shared net if they were shorted). They are NOT — so b2 stays at 3.3V level.
     *
     * We verify the node voltages are independent by measuring that:
     *   I_ch1 ≈ 5 / 10000 = 0.5 mA (not influenced by ch2's 3.3 V)
     *   I_ch2 ≈ 3.3 / 10000 = 0.33 mA (not influenced by ch1's 5 V)
     */
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "v2", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 3.3 } },
        { id: "ra1", kind: "resistor_array", pins: [
          { id: "a1" }, { id: "b1" }, { id: "a2" }, { id: "b2" },
          { id: "a3" }, { id: "b3" }, { id: "a4" }, { id: "b4" },
        ], params: { resistance: 10000 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "ra1", to_pin: "a1" },
        { from_component: "ra1", from_pin: "b1", to_component: "v1", to_pin: "neg" },
        { from_component: "v2", from_pin: "pos", to_component: "ra1", to_pin: "a2" },
        { from_component: "ra1", from_pin: "b2", to_component: "v2", to_pin: "neg" },
        // Share ground between the two supplies
        { from_component: "v1", from_pin: "neg", to_component: "v2", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit);
    const channels = eng.getElementChannelI()["ra1"] ?? [];
    const i_ch1_mA = Math.abs((channels[0] ?? 0) * 1000);
    const i_ch2_mA = Math.abs((channels[1] ?? 0) * 1000);
    // Channel 1: 5 / 10000 = 0.5 mA
    expect(i_ch1_mA).toBeGreaterThan(0.45);
    expect(i_ch1_mA).toBeLessThan(0.55);
    // Channel 2: 3.3 / 10000 = 0.33 mA
    expect(i_ch2_mA).toBeGreaterThan(0.28);
    expect(i_ch2_mA).toBeLessThan(0.38);
  });

  it("catalog entry exists with correct kind and 8-pin layout", () => {
    const part = catalog.parts.find((p) => p.uid === "resistor-array");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("resistor_array");
    expect(part?.pin_layout).toHaveLength(8);
    expect(part?.default_params.resistance).toBe(10000);
    expect(part?.electrical_specs?.value_tol).toBe(0.05);
    expect(part?.bom).toBeDefined();
  });

  it("value_tol is 0.05 — Monte Carlo runner will perturb resistance by ±5%", () => {
    // The Monte Carlo runner perturbs params[key] for any param whose name matches
    // a catalog value_tol-tagged field.  For resistor_array the tagged param is
    // "resistance".  The tolerance machinery in the analysis runner reads
    // electrical_specs.value_tol and applies a uniform delta to params.resistance
    // — verified by inspecting the Monte Carlo runner source (analysis worker picks
    // up value_tol from the catalog entry whose kind matches the component).
    //
    // NOTE: The Monte Carlo runner selects the catalog entry by kind, and
    // buildCatalogByKind returns the FIRST entry for a given kind.  For
    // resistor_array the only entry is "resistor-array", so this is unambiguous.
    const part = catalog.parts.find((p) => p.kind === "resistor_array");
    expect(part?.electrical_specs?.value_tol).toBe(0.05);
  });
});

// ─── Part 3: Capacitor variants ───────────────────────────────────────────────

describe("capacitor variants — catalog parsing", () => {
  it("cap-ceramic entry parses with style ceramic and 100 nF default", () => {
    const part = catalog.parts.find((p) => p.uid === "cap-ceramic");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("capacitor");
    expect(part?.default_params.style).toBe("ceramic");
    expect(part?.default_params.capacitance).toBeCloseTo(1e-7, 10);
    expect(part?.bom).toBeDefined();
    expect(part?.electrical_specs?.value_tol).toBe(0.1);
  });

  it("cap-electrolytic entry parses with style electrolytic and 100 µF default", () => {
    const part = catalog.parts.find((p) => p.uid === "cap-electrolytic");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("capacitor");
    expect(part?.default_params.style).toBe("electrolytic");
    expect(part?.default_params.capacitance).toBeCloseTo(1e-4, 10);
    expect(part?.bom).toBeDefined();
    expect(part?.bom?.polarityNotes).toBeDefined();
    expect(part?.electrical_specs?.value_tol).toBe(0.1);
  });

  it("cap-film entry parses with style film and 1 µF default", () => {
    const part = catalog.parts.find((p) => p.uid === "cap-film");
    expect(part).toBeDefined();
    expect(part?.kind).toBe("capacitor");
    expect(part?.default_params.style).toBe("film");
    expect(part?.default_params.capacitance).toBeCloseTo(1e-6, 10);
    expect(part?.bom).toBeDefined();
    expect(part?.electrical_specs?.value_tol).toBe(0.1);
  });

  it("all three new cap entries and the generic entry are kind 'capacitor'", () => {
    const capParts = catalog.parts.filter((p) => p.kind === "capacitor");
    // Generic + ceramic + electrolytic + film = 4 entries
    expect(capParts.length).toBeGreaterThanOrEqual(4);
    const uids = capParts.map((p) => p.uid);
    expect(uids).toContain("capacitor");
    expect(uids).toContain("cap-ceramic");
    expect(uids).toContain("cap-electrolytic");
    expect(uids).toContain("cap-film");
  });
});

describe("capacitor variants — RC charging identical across styles", () => {
  /**
   * RC charging circuit: 5 V → 10 kΩ → capacitor (any style, 10 µF) → GND.
   *
   * τ = RC = 10000 × 10e-6 = 0.1 s.  After t >> τ, the capacitor charges to ~5 V.
   * After 10 τ = 1 s, V_cap ≈ 5 × (1 - e^(-10)) ≈ 4.9998 V.
   *
   * All three styles share the same engine — the RC curve must be identical.
   * We run 500 ms and check the cap voltage is in the range [4.8, 5.0] V.
   */
  function makeRcCircuit(style: string): SimCircuit {
    return {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10000 } },
        { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 10e-6, style } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "a" },
        { from_component: "c1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
  }

  for (const style of ["ceramic", "electrolytic", "film"]) {
    it(`${style} capacitor charges to near-supply voltage after 5τ`, () => {
      const circuit = makeRcCircuit(style);
      // 500 steps × 1 ms = 500 ms > 5τ = 500 ms (generous τ = 100 ms)
      const eng = runEngine(circuit, 500, 1e-3);
      const vCap = netVoltage(eng, "c1", "a");
      // After 5τ: V ≈ 5 × (1 - e^-5) ≈ 4.966 V
      expect(vCap).toBeGreaterThan(4.8);
      expect(vCap).toBeLessThan(5.01);
    });
  }
});

// ─── Part 4: Electrolytic reverse-polarity warning ───────────────────────────

describe("diagnostics: electrolytic-reverse-polarity", () => {
  /**
   * Reversed electrolytic: 5 V source with + connected to pin b (negative terminal)
   * and − connected to pin a (positive terminal) → vA - vB ≈ -5 V < -1 V → fires.
   */
  it("does NOT fire for a style-less capacitor (abstract engine-level cap)", () => {
    // Programmatic circuits (engine tests, generated fixtures) construct bare
    // { capacitance } params with no style. Those model an abstract capacitance
    // and must not inherit electrolytic polarity semantics — strict equality
    // in the finding guard, no default fallback.
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        // Reversed orientation, but NO style param at all.
        { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 100e-6 } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "b" },
        { from_component: "c1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 1200, 1e-4);
    const findings = analyzeLive(makeInput(circuit, liveFromEngine(eng)));
    expect(findings.find((f) => f.id === "electrolytic-reverse-polarity")).toBeUndefined();
  });

  it("fires when electrolytic sees sustained reverse voltage > 1 V", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        // Reversed: source + → b (the − terminal), source − → a (the + terminal)
        { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 100e-6, style: "electrolytic" } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "b" },
        { from_component: "c1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    // Run past the 50 ms simTime guard (1200 steps × 0.1 ms = 120 ms)
    const eng = runEngine(circuit, 1200, 1e-4);
    const live = liveFromEngine(eng);
    const findings = analyzeLive(makeInput(circuit, live));
    const warning = findings.find((f) => f.id === "electrolytic-reverse-polarity");
    expect(warning).toBeDefined();
    expect(warning?.severity).toBe("warning");
    expect(warning?.componentIds).toContain("c1");
  });

  it("does NOT fire when electrolytic has correct polarity", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        // Correct: source + → a (the + terminal), source − → b (the − terminal)
        { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 100e-6, style: "electrolytic" } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "a" },
        { from_component: "c1", from_pin: "b", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 1200, 1e-4);
    const live = liveFromEngine(eng);
    const findings = analyzeLive(makeInput(circuit, live));
    const warning = findings.find((f) => f.id === "electrolytic-reverse-polarity");
    expect(warning).toBeUndefined();
  });

  it("does NOT fire for ceramic capacitor regardless of polarity", () => {
    // Ceramic is non-polarised — even with what would be "reverse" orientation,
    // the diagnostic must not fire.
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        // Pin b toward + supply — "reversed" orientation, but ceramic has no polarity.
        { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 100e-9, style: "ceramic" } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "b" },
        { from_component: "c1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 1200, 1e-4);
    const live = liveFromEngine(eng);
    const findings = analyzeLive(makeInput(circuit, live));
    const warning = findings.find((f) => f.id === "electrolytic-reverse-polarity");
    expect(warning).toBeUndefined();
  });

  it("does NOT fire for film capacitor regardless of polarity", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6, style: "film" } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "b" },
        { from_component: "c1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 1200, 1e-4);
    const live = liveFromEngine(eng);
    const findings = analyzeLive(makeInput(circuit, live));
    const warning = findings.find((f) => f.id === "electrolytic-reverse-polarity");
    expect(warning).toBeUndefined();
  });

  it("does NOT fire when simTime is below the 50 ms guard", () => {
    // Supply the reversed circuit but override simTime to be below 50 ms.
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 100e-6, style: "electrolytic" } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "b" },
        { from_component: "c1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 100, 1e-4); // 10 ms — below the 50 ms guard
    const live = liveFromEngine(eng);
    // simTime < 0.05 → analyzeLive returns [] immediately
    expect(live.simTime).toBeLessThan(0.05);
    const findings = analyzeLive(makeInput(circuit, live));
    const warning = findings.find((f) => f.id === "electrolytic-reverse-polarity");
    expect(warning).toBeUndefined();
  });

  it("generic capacitor placed from the palette (explicit default style) fires when reversed", () => {
    // Palette placement copies default_params, so a generic "Capacitor" carries
    // style: "electrolytic" EXPLICITLY. The finding requires that explicit style
    // (strict equality, no fallback) — bare { capacitance } params are abstract
    // engine-level caps and stay silent (separate test above).
    const circuit: SimCircuit = {
      components: [
        { id: "v1", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
        { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
        { id: "c1", kind: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6, style: "electrolytic" } },
      ],
      wires: [
        { from_component: "v1", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "c1", to_pin: "b" },
        { from_component: "c1", from_pin: "a", to_component: "v1", to_pin: "neg" },
      ],
    };
    const eng = runEngine(circuit, 1200, 1e-4);
    const live = liveFromEngine(eng);
    const findings = analyzeLive(makeInput(circuit, live));
    // Default style is "electrolytic" for the generic entry — should fire.
    const warning = findings.find((f) => f.id === "electrolytic-reverse-polarity");
    expect(warning).toBeDefined();
    expect(warning?.severity).toBe("warning");
  });
});
