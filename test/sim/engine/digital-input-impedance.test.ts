import { describe, expect, it } from "vitest";
import rawCatalog from "../../helpers/catalog.js";
import type { PartCatalog } from "../../../src/circuit/types.js";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

const catalog = rawCatalog as unknown as PartCatalog;

type DeviceCase = {
  name: string;
  uid: string;
  kind: string;
  pins: string[];
  inputPin: string;
  vccPin: string;
  gndPin: string;
};

const CASES: DeviceCase[] = [
  {
    name: "hobby servo signal",
    uid: "servo-sg90",
    kind: "servo",
    pins: ["sig", "vplus", "gnd"],
    inputPin: "sig",
    vccPin: "vplus",
    gndPin: "gnd",
  },
  {
    name: "MAX7219 DIN",
    uid: "max7219-matrix",
    kind: "max7219",
    pins: ["din", "clk", "cs", "vcc", "gnd"],
    inputPin: "din",
    vccPin: "vcc",
    gndPin: "gnd",
  },
  {
    name: "HD44780 RS",
    uid: "hd44780-lcd",
    kind: "hd44780",
    pins: ["vss", "vdd", "v0", "rs", "rw", "e", "d0", "d1", "d2", "d3", "d4", "d5", "d6", "d7", "a", "k"],
    inputPin: "rs",
    vccPin: "vdd",
    gndPin: "vss",
  },
];

function makeCircuit(device: DeviceCase): SimCircuit {
  return {
    components: [
      {
        id: "source",
        kind: "voltage_source",
        pins: [{ id: "pos" }, { id: "neg" }],
        params: { voltage: 5 },
      },
      {
        id: "probe-r",
        kind: "resistor",
        catalogUid: "resistor",
        pins: [{ id: "a" }, { id: "b" }],
        params: { resistance: 100e6 },
      },
      {
        id: "device",
        kind: device.kind,
        catalogUid: device.uid,
        pins: device.pins.map((id) => ({ id })),
        params: {},
      },
    ],
    wires: [
      { from_component: "source", from_pin: "pos", to_component: "device", to_pin: device.vccPin },
      { from_component: "source", from_pin: "neg", to_component: "device", to_pin: device.gndPin },
      { from_component: "source", from_pin: "pos", to_component: "probe-r", to_pin: "a" },
      { from_component: "probe-r", from_pin: "b", to_component: "device", to_pin: device.inputPin },
    ],
  };
}

describe("high-impedance behavioral inputs", () => {
  for (const device of CASES) {
    it(`${device.name} does not contain an invented 1 Mohm pull-down`, () => {
      const engine = new SimEngine();
      engine.load(makeCircuit(device));
      engine.step(1e-4);
      engine.step(1e-4);
      expect(engine.lastConverged).toBe(true);

      const net = engine.nets.find((candidate) =>
        candidate.pins.some(([componentId, pinId]) =>
          componentId === "device" && pinId === device.inputPin,
        ),
      );
      const voltage = net ? engine.getNetV()[net.id] : Number.NaN;
      // With only the disclosed 1 Tohm numerical shunt, 100 Mohm source
      // resistance gives 5 * 1T/(1T+100M) = 4.9995 V. A hidden 1 Mohm
      // pull-down would instead collapse this node to about 50 mV.
      expect(voltage).toBeGreaterThan(4.99);
    });
  }

  it("keeps MAX7219 and HD44780 catalog copy aligned with the high-Z stamps", () => {
    for (const uid of ["max7219-matrix", "hd44780-lcd"]) {
      const notes = catalog.parts.find((part) => part.uid === uid)?.electrical_specs?.notes ?? "";
      expect(notes, uid).not.toMatch(/1\s*M(?:Ohm|Ω).*pull(?:down|-down)/i);
      expect(notes, uid).toMatch(/high-impedance logic inputs/i);
      expect(notes, uid).toMatch(/no invented physical pull-down/i);
    }
  });
});
