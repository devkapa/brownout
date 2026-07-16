import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

function batteryCircuit(options: {
  catalogUid?: string;
  voltage?: number;
  rInternal?: number;
  capacityAh?: number;
  charge?: number;
  loadOhm?: number;
  temperatureC?: number;
} = {}): SimCircuit {
  return {
    components: [
      {
        id: "cell",
        kind: "battery_pack",
        ...(options.catalogUid ? { catalogUid: options.catalogUid } : {}),
        pins: [{ id: "pos" }, { id: "neg" }],
        params: {
          voltage: options.voltage ?? 3,
          ...(options.rInternal !== undefined ? { rInternal: options.rInternal } : {}),
          ...(options.capacityAh !== undefined ? { capacityAh: options.capacityAh } : {}),
          charge: options.charge ?? 1,
        },
      },
      {
        id: "load",
        kind: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: options.loadOhm ?? 100 },
      },
    ],
    wires: [
      { from_component: "cell", from_pin: "pos", to_component: "load", to_pin: "a" },
      { from_component: "load", from_pin: "b", to_component: "cell", to_pin: "neg" },
    ],
    environment: { temperatureC: options.temperatureC ?? 21, lux: 100 },
  };
}

function terminalVoltage(engine: SimEngine): number {
  const positive = engine.nets.find((net) =>
    net.pins.some(([id, pin]) => id === "cell" && pin === "pos"),
  )!;
  const negative = engine.nets.find((net) =>
    net.pins.some(([id, pin]) => id === "cell" && pin === "neg"),
  )!;
  return (engine.getNetV()[positive.id] ?? Number.NaN)
    - (engine.getNetV()[negative.id] ?? Number.NaN);
}

describe("battery chemistry integration", () => {
  it("uses CR2032 OCV and fresh resistance for terminal sag", () => {
    const engine = new SimEngine();
    engine.coldLoad(batteryCircuit({
      catalogUid: "battery-coin-cr2032",
      voltage: 3,
      rInternal: 10,
      loadOhm: 100,
    }));
    engine.step(1e-4);

    expect(engine.getBatteryState("cell")).toMatchObject({
      profileId: "lithium-cr2032",
      soc: expect.any(Number),
      openCircuitVoltageV: expect.closeTo(3.2, 7),
      internalResistanceOhm: expect.closeTo(10, 8),
    });
    expect(terminalVoltage(engine)).toBeCloseTo(3.2 * 100 / 110, 6);
  });

  it("counts delivered coulombs, lowers SoC, and rolls back exactly", () => {
    const engine = new SimEngine();
    engine.coldLoad(batteryCircuit({
      catalogUid: "battery-9v",
      voltage: 9,
      rInternal: 1.5,
      capacityAh: 1e-4,
      loadOhm: 20,
    }));
    const before = engine.getBatteryState("cell")!;
    const snapshot = engine.saveState();
    engine.step(0.2);
    const after = engine.getBatteryState("cell")!;

    expect(after.soc).toBeLessThan(before.soc);
    expect(after.dischargedCoulombs).toBeGreaterThan(before.dischargedCoulombs);

    engine.restoreState(snapshot);
    expect(engine.getBatteryState("cell")!.soc).toBeCloseTo(before.soc, 14);
    expect(engine.getBatteryState("cell")!.dischargedCoulombs)
      .toBeCloseTo(before.dischargedCoulombs, 14);
  });

  it("preserves runtime depletion across warm loads and resets on authored charge edit", () => {
    const circuit = batteryCircuit({
      catalogUid: "battery-9v",
      voltage: 9,
      capacityAh: 1e-4,
      loadOhm: 20,
    });
    const engine = new SimEngine();
    engine.coldLoad(circuit);
    engine.step(0.2);
    const depleted = engine.getBatteryState("cell")!.soc;

    engine.load(circuit);
    expect(engine.getBatteryState("cell")!.soc).toBeCloseTo(depleted, 14);

    circuit.components[0]!.params.charge = 0.8;
    engine.load(circuit);
    expect(engine.getBatteryState("cell")!.soc).toBeCloseTo(0.8, 12);
  });

  it("raises CR2032 resistance and sag at the profile's cold bound", () => {
    const warm = new SimEngine();
    warm.coldLoad(batteryCircuit({
      catalogUid: "battery-coin-cr2032",
      rInternal: 10,
      loadOhm: 100,
      temperatureC: 21,
    }));
    const cold = new SimEngine();
    cold.coldLoad(batteryCircuit({
      catalogUid: "battery-coin-cr2032",
      rInternal: 10,
      loadOhm: 100,
      temperatureC: -30,
    }));

    expect(warm.getBatteryState("cell")!.internalResistanceOhm).toBeCloseTo(10, 8);
    expect(cold.getBatteryState("cell")!.internalResistanceOhm).toBeCloseTo(50, 8);
    expect(terminalVoltage(cold)).toBeLessThan(terminalVoltage(warm));
  });

  it("keeps identity-less legacy batteries on the explicit linear fallback", () => {
    const engine = new SimEngine();
    engine.coldLoad(batteryCircuit({ voltage: 5, rInternal: 0, charge: 0.5, loadOhm: 1_000 }));
    expect(engine.getBatteryState("cell")!.profileId).toBe("generic-primary");
    expect(terminalVoltage(engine)).toBeCloseTo(2.5, 6);
  });
});
