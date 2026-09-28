import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const POT_KINDS = ["potentiometer", "trimmer"] as const;

// CW to +5 V, CCW to ground, the wiper optionally driving 220 Ω into a red
// LED: the textbook "turn the knob, dim the LED" board.
function divider(
  kind: (typeof POT_KINDS)[number],
  params: Record<string, unknown>,
  load: "none" | "led" | "short" = "none",
): SimCircuit {
  const components: SimCircuit["components"] = [
    { id: "psu", kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5, iLimit: 1 } },
    { id: "pot", kind, pins: [{ id: "cw" }, { id: "wiper" }, { id: "ccw" }], params: { rTotal: 10_000, ...params } },
  ];
  const wires: SimCircuit["wires"] = [
    { from_component: "psu", from_pin: "pos", to_component: "pot", to_pin: "cw" },
    { from_component: "pot", from_pin: "ccw", to_component: "psu", to_pin: "neg" },
  ];
  if (load === "led") {
    components.push(
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 220 } },
      { id: "d1", kind: "led", pins: [{ id: "a" }, { id: "k" }], params: { color: "red", vf: 1.8 } },
    );
    wires.push(
      { from_component: "pot", from_pin: "wiper", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "d1", to_pin: "a" },
      { from_component: "d1", from_pin: "k", to_component: "psu", to_pin: "neg" },
    );
  } else if (load === "short") {
    wires.push({ from_component: "pot", from_pin: "wiper", to_component: "psu", to_pin: "neg" });
  }
  return { components, wires };
}

function run(circuit: SimCircuit, seconds: number, dt = 1e-3): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < Math.ceil(seconds / dt); i++) engine.step(dt);
  return engine;
}

function wiperVolts(engine: SimEngine): number {
  const net = engine.getNetIdForPin("pot", "wiper");
  if (!net) throw new Error("wiper has no net");
  return engine.getNetV()[net] ?? Number.NaN;
}

describe.each(POT_KINDS)("%s", (kind) => {
  it("puts the wiper at CW when turned fully clockwise (position 1)", () => {
    const at = (position: number) => wiperVolts(run(divider(kind, { position }), 0.01));

    expect(at(1)).toBeCloseTo(5, 2);
    expect(at(0.75)).toBeCloseTo(3.75, 3);
    expect(at(0.25)).toBeCloseTo(1.25, 3);
    expect(at(0)).toBeCloseTo(0, 2);
  });

  it("carries the wiper's LED current at every position, both stops included", () => {
    const positions = [0, 0.01, 0.02, 0.05, 0.1, 0.5, 0.9, 0.95, 0.98, 0.99, 1];
    for (const position of positions) {
      // Two seconds: a segment 1.5x over its share latches inside that, and
      // before the rating floor these ran 3-100x over.
      const engine = run(divider(kind, { position }, "led"), 2, 2e-3);
      expect({ position, failures: engine.getFailures() }).toEqual({ position, failures: {} });
    }
    // The LED is brightest with the wiper at the CW (+5 V) stop.
    const atStop = run(divider(kind, { position: 1 }, "led"), 0.01).getElementI().d1 ?? 0;
    expect(atStop).toBeGreaterThan(0.013);
  });

  it("still fails open with its wiper shorted across the supply at a stop", () => {
    const engine = run(divider(kind, { position: 1 }, "short"), 0.2);
    const failures = Object.values(engine.getFailures());

    expect(failures).toHaveLength(1);
    expect(failures[0]?.kind).toBe("resistor_overload");
    // The message states a real power against a non-zero share.
    expect(failures[0]?.limit).toBeGreaterThan(0.01);
    expect(failures[0]?.message).not.toMatch(/0\.000 W/);
  });
});

describe("potentiometer audio (log) taper", () => {
  it("rises slowly from CCW, reaching a tenth of the supply at mid-rotation", () => {
    const at = (position: number) =>
      wiperVolts(run(divider("potentiometer", { position, taper: "log" }), 0.01));

    expect(at(0.5)).toBeCloseTo(0.5, 3);
    expect(at(1)).toBeCloseTo(5, 2);
    const sweep = [0, 0.25, 0.5, 0.75, 1].map(at);
    for (let i = 1; i < sweep.length; i++) expect(sweep[i]).toBeGreaterThan(sweep[i - 1]!);
  });
});
