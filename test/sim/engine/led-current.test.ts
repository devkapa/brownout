import { describe, expect, it } from "vitest";
import fs from "node:fs";
import path from "node:path";
import { breadboardToSimCircuit } from "../../helpers/circuit/breadboard.js";
import type { Circuit } from "../../../src/circuit/types.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

// B2 migration: loads the pre-converted electrical corpus (see
// test/fixtures/circuits). The directory mirrors docs/test-circuits one to
// one, so the overcurrent sweep below still audits exactly that corpus.
function loadFixture(name: string): Circuit {
  const root = path.resolve(__dirname, "../../fixtures/circuits");
  const file = name.endsWith(".sim.json") ? name : name.replace(/\.json$/, ".sim.json");
  return JSON.parse(fs.readFileSync(path.join(root, file), "utf8")) as Circuit;
}

function runFixture(name: string, steps = 80): SimEngine {
  const engine = new SimEngine();
  engine.load(breadboardToSimCircuit(loadFixture(name)));
  for (let i = 0; i < steps; i++) engine.step(1e-4);
  return engine;
}

function runSimCircuit(circuit: SimCircuit, steps = 80, dt = 1e-4): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < steps; i++) engine.step(dt);
  return engine;
}

// ─── Basic overcurrent guard ─────────────────────────────────────────────────

describe("LED current through breadboard fixtures", () => {
  it("keeps generated LED probes below burnout current when a resistor is in series", () => {
    const fixtureRoot = path.resolve(__dirname, "../../fixtures/circuits");
    const fixtureNames = fs.readdirSync(fixtureRoot).filter((name) => name.endsWith(".json")).sort();
    const overcurrent: Array<[string, string, number]> = [];

    for (const fixtureName of fixtureNames) {
      const engine = runFixture(fixtureName);
      const currents = engine.getElementI();
      for (const [id, current] of Object.entries(currents)) {
        if (id.includes("_led") || id.startsWith("led_")) {
          const abs = Math.abs(current);
          if (abs > 0.03) overcurrent.push([fixtureName, id, abs]);
        }
      }
    }

    expect(overcurrent).toEqual([]);
  });
});

// ─── Per-channel currents: rgb_led ──────────────────────────────────────────

describe("getElementChannelI — rgb_led", () => {
  function rgbCircuit(r: number, g: number, b: number, vcc = 5): SimCircuit {
    // Drive each channel through a separate resistor from a shared voltage source.
    return {
      components: [
        {
          id: "vcc",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: vcc },
        },
        {
          id: "r_r",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: r },
        },
        {
          id: "r_g",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: g },
        },
        {
          id: "r_b",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: b },
        },
        {
          id: "led1",
          kind: "rgb_led",
          pins: [{ id: "r_a" }, { id: "g_a" }, { id: "b_a" }, { id: "com_k" }],
          params: {},
        },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "r_r", to_pin: "a" },
        { from_component: "vcc", from_pin: "pos", to_component: "r_g", to_pin: "a" },
        { from_component: "vcc", from_pin: "pos", to_component: "r_b", to_pin: "a" },
        { from_component: "r_r", from_pin: "b", to_component: "led1", to_pin: "r_a" },
        { from_component: "r_g", from_pin: "b", to_component: "led1", to_pin: "g_a" },
        { from_component: "r_b", from_pin: "b", to_component: "led1", to_pin: "b_a" },
        { from_component: "led1", from_pin: "com_k", to_component: "vcc", to_pin: "neg" },
      ],
    };
  }

  it("returns three channels for rgb_led", () => {
    const engine = runSimCircuit(rgbCircuit(150, 150, 150));
    const channels = engine.getElementChannelI();
    expect(channels["led1"]).toHaveLength(3);
  });

  it("all channels conduct when all three are driven", () => {
    const engine = runSimCircuit(rgbCircuit(150, 150, 150));
    const [ir, ig, ib] = engine.getElementChannelI()["led1"];
    expect(ir).toBeGreaterThan(1e-4);
    expect(ig).toBeGreaterThan(1e-4);
    expect(ib).toBeGreaterThan(1e-4);
  });

  it("kills one channel when its resistor is open (very high R)", () => {
    // Blue resistor is astronomically large → blue channel effectively off.
    const engine = runSimCircuit(rgbCircuit(150, 150, 1e9));
    const [ir, ig, ib] = engine.getElementChannelI()["led1"];
    expect(ir).toBeGreaterThan(1e-4);
    expect(ig).toBeGreaterThan(1e-4);
    expect(ib).toBeLessThan(1e-6);
  });

  it("channels are independent — changing one resistor does not affect others", () => {
    const base = runSimCircuit(rgbCircuit(150, 150, 150));
    const varied = runSimCircuit(rgbCircuit(1000, 150, 150));
    const [irBase] = base.getElementChannelI()["led1"];
    const [irLow, igLow, ibLow] = varied.getElementChannelI()["led1"];
    // Red is lower in varied circuit.
    expect(irLow).toBeLessThan(irBase);
    // Green and blue are unchanged (same resistors).
    const [, igBase, ibBase] = base.getElementChannelI()["led1"];
    expect(igLow).toBeCloseTo(igBase, 4);
    expect(ibLow).toBeCloseTo(ibBase, 4);
  });
});

// ─── Per-channel currents: bicolor_led ──────────────────────────────────────

describe("getElementChannelI — bicolor_led", () => {
  function bicolorCircuit(r1: number, r2: number, vcc = 5): SimCircuit {
    return {
      components: [
        {
          id: "vcc",
          kind: "voltage_source",
          pins: [{ id: "pos" }, { id: "neg" }],
          params: { voltage: vcc },
        },
        {
          id: "r1",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: r1 },
        },
        {
          id: "r2",
          kind: "resistor",
          pins: [{ id: "a" }, { id: "b" }],
          params: { resistance: r2 },
        },
        {
          id: "led1",
          kind: "bicolor_led",
          pins: [{ id: "a1" }, { id: "k" }, { id: "a2" }],
          params: {},
        },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "r1", to_pin: "a" },
        { from_component: "vcc", from_pin: "pos", to_component: "r2", to_pin: "a" },
        { from_component: "r1", from_pin: "b", to_component: "led1", to_pin: "a1" },
        { from_component: "r2", from_pin: "b", to_component: "led1", to_pin: "a2" },
        { from_component: "led1", from_pin: "k", to_component: "vcc", to_pin: "neg" },
      ],
    };
  }

  it("returns two channels for bicolor_led", () => {
    const engine = runSimCircuit(bicolorCircuit(150, 150));
    const channels = engine.getElementChannelI();
    expect(channels["led1"]).toHaveLength(2);
  });

  it("both channels conduct when both anodes are driven", () => {
    const engine = runSimCircuit(bicolorCircuit(150, 150));
    const [i1, i2] = engine.getElementChannelI()["led1"];
    expect(i1).toBeGreaterThan(1e-4);
    expect(i2).toBeGreaterThan(1e-4);
  });

  it("channel 2 is off when its anode resistor is open", () => {
    const engine = runSimCircuit(bicolorCircuit(150, 1e9));
    const [i1, i2] = engine.getElementChannelI()["led1"];
    expect(i1).toBeGreaterThan(1e-4);
    expect(i2).toBeLessThan(1e-6);
  });
});
