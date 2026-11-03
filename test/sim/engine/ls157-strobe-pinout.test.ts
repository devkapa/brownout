/**
 * The 74LS157 is a quad 2:1 data selector with totem-pole outputs. Its strobe
 * (/G, pin 15, the catalog's "/oe") forces every Y output LOW while it is
 * HIGH; the outputs never float. The 3-state version of the same selector is
 * the 74LS257. (SN54LS157/SN74LS157 data sheet, function table: G = H gives
 * Y = L for any select or data input.)
 *
 * The package pinout is 1 S, 2 1A, 3 1B, 4 1Y, 5 2A, 6 2B, 7 2Y, 8 GND,
 * 9 3Y, 10 3B, 11 3A, 12 4Y, 13 4B, 14 4A, 15 /G, 16 VCC, and the catalog's
 * pin_layout order is the DIP pin number order de:volt lays parts out by.
 */
import fs from "node:fs";
import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type Comp = SimCircuit["components"][number];

const w = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });
const source = (id: string, voltage: number): Comp =>
  ({ id, kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage } }) as Comp;
const resistor = (id: string, resistance: number): Comp =>
  ({ id, kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance } }) as Comp;

const DATASHEET_ORDER = [
  "sel", "1a", "1b", "1y", "2a", "2b", "2y", "gnd",
  "3y", "3b", "3a", "4y", "4b", "4a", "/oe", "vcc",
];

/**
 * A powered 74LS157 with every data input HIGH, select LOW, and a 10 kohm
 * pull-up on each output, so an output that stops driving reads 5 V.
 */
function muxBoard(strobeHigh: boolean): SimEngine {
  const components: Comp[] = [
    source("vcc", 5),
    { id: "u1", kind: "74ls157", pins: DATASHEET_ORDER.map((id) => ({ id })), params: {} } as Comp,
  ];
  const wires = [
    w("vcc", "pos", "u1", "vcc"), w("vcc", "neg", "u1", "gnd"),
    w("vcc", "neg", "u1", "sel"), w("vcc", strobeHigh ? "pos" : "neg", "u1", "/oe"),
  ];
  for (let g = 1; g <= 4; g++) {
    wires.push(w("vcc", "pos", "u1", `${g}a`), w("vcc", "pos", "u1", `${g}b`));
    components.push(resistor(`pu${g}`, 10_000));
    wires.push(w("vcc", "pos", `pu${g}`, "a"), w(`pu${g}`, "b", "u1", `${g}y`));
  }
  const engine = new SimEngine();
  engine.load({ components, wires } as SimCircuit);
  for (let i = 0; i < 200; i++) engine.step(1e-7);
  return engine;
}

function outputVolts(engine: SimEngine, pin: string): number {
  const net = engine.getNetIdForPin("u1", pin);
  expect(net).toBeDefined();
  return engine.getNetV()[net!]!;
}

describe("74LS157 strobe", () => {
  it("drives every output LOW against a pull-up while /G is HIGH", () => {
    const engine = muxBoard(true);
    for (let g = 1; g <= 4; g++) {
      expect(outputVolts(engine, `${g}y`)).toBeLessThan(0.8);
      expect(engine.getIcState("u1")?.[`delay:${g}y`]).toBe(0);
    }
  });

  it("passes the selected input while /G is LOW", () => {
    const engine = muxBoard(false);
    for (let g = 1; g <= 4; g++) expect(outputVolts(engine, `${g}y`)).toBeGreaterThan(2.7);
  });
});

describe("74LS157 catalog pinout", () => {
  type Catalog = { parts: Array<{ kind: string; pin_layout: Array<{ id: string }> }> };
  const read = (rel: string): Catalog => JSON.parse(fs.readFileSync(new URL(rel, import.meta.url), "utf8"));
  const defaultParts = read("../../../src/parts/default-parts.json");
  const fixtureParts = read("../../fixtures/catalog/parts.json");
  const order = (catalog: Catalog) =>
    catalog.parts.find((p) => p.kind === "74ls157")?.pin_layout.map((p) => p.id);

  it("lists the pins in data-sheet pin-number order in the default catalog", () => {
    expect(order(defaultParts)).toEqual(DATASHEET_ORDER);
  });

  it("lists the pins in data-sheet pin-number order in the de:volt catalog fixture", () => {
    expect(order(fixtureParts)).toEqual(DATASHEET_ORDER);
  });
});
