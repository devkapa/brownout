/**
 * 74LS153 dual 4:1 data selector/multiplexer.
 *
 * Datasheet behaviour (SN74LS153 function table): select inputs A (LSB) and B are shared by
 * both sections; each section routes C[B*2+A] to its Y. A section whose
 * active-low strobe /G is HIGH drives its Y LOW (totem-pole, not tri-state:
 * the tri-state part is the 74LS253).
 */
import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../../src/sim/engine/sim-engine.js";
import defaultParts from "../../../src/parts/default-parts.json" with { type: "json" };

type Comp = SimCircuit["components"][number];
type Wire = SimCircuit["wires"][number];

// DIP-16 order, pin 1 to pin 16.
const LS153_PINS = [
  "/1g", "b", "1c3", "1c2", "1c1", "1c0", "1y", "gnd",
  "2y", "2c0", "2c1", "2c2", "2c3", "a", "/2g", "vcc",
] as const;

function wire(fc: string, fp: string, tc: string, tp: string): Wire {
  return { from_component: fc, from_pin: fp, to_component: tc, to_pin: tp };
}

function netVoltage(engine: SimEngine, componentId: string, pinId: string): number {
  const net = engine.nets.find((n) =>
    n.pins.some(([cid, pid]) => cid === componentId && pid === pinId),
  );
  if (!net) throw new Error(`No net found for ${componentId}.${pinId}`);
  return engine.getNetV()[net.id] ?? 0;
}

/**
 * Tie every logic input of a powered 74LS153 to VCC or GND, and load each
 * output with a 10k resistor to `outputPull` so an undriven output reads as
 * that rail rather than as the level we are testing for.
 */
function circuit(levels: Record<string, 0 | 1>, outputPull: "vcc" | "gnd"): SimCircuit {
  const components: Comp[] = [
    {
      id: "u1",
      kind: "74ls153",
      position: { x: 0, y: 0 },
      rotation: 0,
      pins: LS153_PINS.map((id) => ({ id })),
      params: {},
    },
    {
      id: "vcc",
      kind: "voltage_source",
      position: { x: 0, y: 0 },
      rotation: 0,
      pins: [{ id: "pos" }, { id: "neg" }],
      params: { voltage: 5 },
    },
  ];
  const wires: Wire[] = [wire("vcc", "pos", "u1", "vcc"), wire("vcc", "neg", "u1", "gnd")];
  for (const [pin, level] of Object.entries(levels)) {
    wires.push(wire("vcc", level ? "pos" : "neg", "u1", pin));
  }
  for (const y of ["1y", "2y"]) {
    components.push({
      id: `r_${y}`,
      kind: "resistor",
      position: { x: 0, y: 0 },
      rotation: 0,
      pins: [{ id: "a" }, { id: "b" }],
      params: { resistance: 10_000 },
    });
    wires.push(wire(`r_${y}`, "a", "u1", y));
    wires.push(wire(`r_${y}`, "b", "vcc", outputPull === "vcc" ? "pos" : "neg"));
  }
  return { components, wires };
}

function run(c: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.load(c);
  for (let i = 0; i < 3; i++) engine.step(1e-4);
  return engine;
}

/** Data inputs with only channel `hot` of each section HIGH. */
function oneHot(hot: number): Record<string, 0 | 1> {
  const levels: Record<string, 0 | 1> = {};
  for (let i = 0; i < 4; i++) {
    levels[`1c${i}`] = i === hot ? 1 : 0;
    levels[`2c${i}`] = i === hot ? 1 : 0;
  }
  return levels;
}

describe("74LS153 dual 4:1 mux", () => {
  for (let sel = 0; sel < 4; sel++) {
    const a = (sel & 1) as 0 | 1;
    const b = ((sel >> 1) & 1) as 0 | 1;

    it(`B=${b} A=${a} routes C${sel} of both sections to their Y`, () => {
      // Only the selected channel is HIGH: Y must go HIGH against a pull-down.
      const hi = run(circuit({ ...oneHot(sel), a, b, "/1g": 0, "/2g": 0 }, "gnd"));
      expect(netVoltage(hi, "u1", "1y")).toBeGreaterThan(2.7);
      expect(netVoltage(hi, "u1", "2y")).toBeGreaterThan(2.7);

      // Every channel but the selected one is HIGH: Y must go LOW against a pull-up.
      const inverted = Object.fromEntries(
        Object.entries(oneHot(sel)).map(([pin, v]) => [pin, v ? 0 : 1]),
      ) as Record<string, 0 | 1>;
      const lo = run(circuit({ ...inverted, a, b, "/1g": 0, "/2g": 0 }, "vcc"));
      expect(netVoltage(lo, "u1", "1y")).toBeLessThan(0.5);
      expect(netVoltage(lo, "u1", "2y")).toBeLessThan(0.5);
    });
  }

  it("a HIGH strobe drives only its own section's Y LOW", () => {
    // All data HIGH, so an enabled section outputs HIGH whatever the select.
    const allHigh = Object.fromEntries(
      Object.keys(oneHot(0)).map((pin) => [pin, 1]),
    ) as Record<string, 0 | 1>;
    const engine = run(circuit({ ...allHigh, a: 1, b: 0, "/1g": 1, "/2g": 0 }, "vcc"));
    // Section 1 is strobed off: driven LOW, not left floating at the pull-up.
    expect(netVoltage(engine, "u1", "1y")).toBeLessThan(0.5);
    expect(netVoltage(engine, "u1", "2y")).toBeGreaterThan(2.7);
  });

  it("follows the datasheet DIP-16 pinout", () => {
    const part = (defaultParts as { parts: Array<{ kind: string; pin_layout: Array<{ id: string; offset: { x: number; y: number } }> }> })
      .parts.find((p) => p.kind === "74ls153");
    expect(part).toBeDefined();
    // Pins 1-8 run down the left edge, 9-16 back up the right edge.
    const ordered = [...part!.pin_layout].sort((p, q) =>
      p.offset.x !== q.offset.x
        ? p.offset.x - q.offset.x
        : p.offset.x === 0
          ? p.offset.y - q.offset.y
          : q.offset.y - p.offset.y,
    );
    expect(ordered.map((p) => p.id)).toEqual([...LS153_PINS]);
  });
});
