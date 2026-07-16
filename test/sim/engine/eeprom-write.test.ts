import { describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";

type EepromKind = "28c16" | "28c256";

const EEPROM_ID = "rom";
const DATA_R = 10_000;
const DT = 50e-6;

const PINS_28C16 = [
  "a7", "a6", "a5", "a4", "a3", "a2", "a1", "a0", "/oe", "a10", "/ce", "gnd",
  "io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7", "a9", "a8", "/we", "vcc",
];

const PINS_28C256 = [
  "a14", "a12", "a7", "a6", "a5", "a4", "a3", "a2", "a1", "a0", "/oe", "a10", "/ce", "gnd",
  "io0", "io1", "io2", "io3", "io4", "io5", "io6", "io7", "a9", "a8", "a13", "a11", "/we", "vcc",
];

function addrBits(kind: EepromKind): number {
  return kind === "28c16" ? 11 : 15;
}

function dataSourceId(bit: number): string {
  return `data${bit}`;
}

function sourceId(pin: string): string {
  return `src_${pin.replace("/", "n")}`;
}

function makeEepromCircuit(kind: EepromKind, params: Record<string, number | string> = {}): SimCircuit {
  const pins = kind === "28c16" ? PINS_28C16 : PINS_28C256;
  // Ground is established via voltage_source neg pin aliasing (no standalone ground needed).
  const components: SimCircuit["components"] = [
    {
      id: EEPROM_ID,
      kind,
      pins: pins.map((id) => ({ id })),
      params: { vcc: 5, contents: "", ...params },
    },
    { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
  ];
  const wires: SimCircuit["wires"] = [
    { from_component: "vcc", from_pin: "pos", to_component: EEPROM_ID, to_pin: "vcc" },
    { from_component: "vcc", from_pin: "neg", to_component: EEPROM_ID, to_pin: "gnd" },
  ];

  for (let i = 0; i < addrBits(kind); i++) {
    const id = sourceId(`a${i}`);
    components.push({ id, kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } });
    wires.push(
      { from_component: id, from_pin: "pos", to_component: EEPROM_ID, to_pin: `a${i}` },
      { from_component: id, from_pin: "neg", to_component: "vcc", to_pin: "neg" },
    );
  }

  for (const pin of ["/ce", "/oe", "/we"]) {
    const id = sourceId(pin);
    components.push({ id, kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } });
    wires.push(
      { from_component: id, from_pin: "pos", to_component: EEPROM_ID, to_pin: pin },
      { from_component: id, from_pin: "neg", to_component: "vcc", to_pin: "neg" },
    );
  }

  for (let bit = 0; bit < 8; bit++) {
    const src = dataSourceId(bit);
    const r = `data_r${bit}`;
    components.push(
      { id: src, kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 0 } },
      { id: r, kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: DATA_R } },
    );
    wires.push(
      { from_component: src, from_pin: "pos", to_component: r, to_pin: "a" },
      { from_component: r, from_pin: "b", to_component: EEPROM_ID, to_pin: `io${bit}` },
      { from_component: src, from_pin: "neg", to_component: "vcc", to_pin: "neg" },
    );
  }

  return { components, wires };
}

function source(circuit: SimCircuit, id: string): SimCircuit["components"][number] {
  const comp = circuit.components.find((c) => c.id === id);
  if (!comp) throw new Error(`missing source ${id}`);
  return comp;
}

function driveAddr(circuit: SimCircuit, value: number): void {
  for (let i = 0; i < 15; i++) {
    const src = circuit.components.find((c) => c.id === sourceId(`a${i}`));
    if (src) src.params.voltage = (value >> i) & 1 ? 5 : 0;
  }
}

function driveData(circuit: SimCircuit, value: number): void {
  for (let bit = 0; bit < 8; bit++) {
    source(circuit, dataSourceId(bit)).params.voltage = (value >> bit) & 1 ? 5 : 0;
  }
}

function driveCe(circuit: SimCircuit, low: boolean): void {
  source(circuit, sourceId("/ce")).params.voltage = low ? 0 : 5;
}

function driveWe(circuit: SimCircuit, low: boolean): void {
  source(circuit, sourceId("/we")).params.voltage = low ? 0 : 5;
}

function driveOe(circuit: SimCircuit, low: boolean): void {
  source(circuit, sourceId("/oe")).params.voltage = low ? 0 : 5;
}

function pinNet(engine: SimEngine, compId: string, pinId: string): string {
  const net = engine.nets.find((candidate) =>
    candidate.pins.some(([c, p]) => c === compId && p === pinId),
  );
  if (!net) throw new Error(`missing net for ${compId}.${pinId}`);
  return net.id;
}

function readData(engine: SimEngine, circuit: SimCircuit, addr: number): number {
  driveData(circuit, 0);
  driveAddr(circuit, addr);
  driveCe(circuit, true);
  driveWe(circuit, false);
  driveOe(circuit, true);
  engine.step(DT);
  let value = 0;
  for (let bit = 0; bit < 8; bit++) {
    value |= (engine.digitalState[`${EEPROM_ID}/io${bit}`] ?? 0) << bit;
  }
  return value;
}

function readBusVoltageByte(engine: SimEngine): number {
  let value = 0;
  const netV = engine.getNetV();
  for (let bit = 0; bit < 8; bit++) {
    if ((netV[pinNet(engine, EEPROM_ID, `io${bit}`)] ?? 0) >= 2.5) value |= 1 << bit;
  }
  return value;
}

function stepFor(engine: SimEngine, sec: number): void {
  const target = engine.simTime + sec;
  while (engine.simTime < target - 1e-15) engine.step(Math.min(DT, target - engine.simTime));
}

function stepUntil(engine: SimEngine, sec: number): void {
  while (engine.simTime < sec - 1e-15) engine.step(Math.min(DT, sec - engine.simTime));
}

function pulseWeWrite(engine: SimEngine, circuit: SimCircuit, addr: number, data: number): void {
  driveAddr(circuit, addr);
  driveData(circuit, data);
  driveCe(circuit, true);
  driveOe(circuit, false);
  driveWe(circuit, true);
  stepFor(engine, 100e-6);
  driveWe(circuit, false);
  stepFor(engine, 100e-6);
}

function writeAndWait(engine: SimEngine, circuit: SimCircuit, addr: number, data: number): void {
  pulseWeWrite(engine, circuit, addr, data);
  stepFor(engine, 15e-3);
}

describe("EEPROM runtime writes", () => {
  it("writes a 28C16 byte after tWC", () => {
    const circuit = makeEepromCircuit("28c16");
    const engine = new SimEngine();
    engine.load(circuit);

    pulseWeWrite(engine, circuit, 0x100, 0x5a);
    stepUntil(engine, engine.simTime + 15e-3);

    const updates = engine.takeEepromUpdates();
    expect(updates[EEPROM_ID]?.[0x100]).toBe(0x5a);
    expect(readData(engine, circuit, 0x100)).toBe(0x5a);
  });

  it("does not write while /CE is HIGH", () => {
    const circuit = makeEepromCircuit("28c16");
    const engine = new SimEngine();
    engine.load(circuit);

    driveAddr(circuit, 0x100);
    driveData(circuit, 0x5a);
    driveCe(circuit, false);
    driveOe(circuit, false);
    driveWe(circuit, true);
    stepFor(engine, 100e-6);
    driveWe(circuit, false);
    stepFor(engine, 15e-3);

    expect(engine.takeEepromUpdates()[EEPROM_ID]).toBeUndefined();
    expect(readData(engine, circuit, 0x100)).toBe(0xff);
  });

  it("writes different bytes to different addresses", () => {
    const circuit = makeEepromCircuit("28c16");
    const engine = new SimEngine();
    engine.load(circuit);

    writeAndWait(engine, circuit, 0x000, 0xaa);
    writeAndWait(engine, circuit, 0x001, 0xbb);

    expect(readData(engine, circuit, 0x000)).toBe(0xaa);
    expect(readData(engine, circuit, 0x001)).toBe(0xbb);
  });

  it("supports CE-controlled write cycles", () => {
    const circuit = makeEepromCircuit("28c16");
    const engine = new SimEngine();
    engine.load(circuit);

    driveAddr(circuit, 0x042);
    driveData(circuit, 0x3c);
    driveOe(circuit, false);
    driveWe(circuit, true);
    driveCe(circuit, true);
    stepFor(engine, 100e-6);
    driveCe(circuit, false);
    stepFor(engine, 100e-6);
    stepFor(engine, 15e-3);

    expect(readData(engine, circuit, 0x042)).toBe(0x3c);
  });

  it("inverts O7 during the busy window", () => {
    const circuit = makeEepromCircuit("28c16");
    const engine = new SimEngine();
    engine.load(circuit);

    pulseWeWrite(engine, circuit, 0x000, 0x80);
    driveData(circuit, 0);
    driveAddr(circuit, 0x000);
    driveCe(circuit, true);
    driveOe(circuit, true);
    engine.step(DT);
    expect((readBusVoltageByte(engine) >> 7) & 1).toBe(0);
    stepFor(engine, 15e-3);
    expect(readData(engine, circuit, 0x000) >> 7).toBe(1);
  });

  it("toggles O6 during the busy window", () => {
    const circuit = makeEepromCircuit("28c16");
    const engine = new SimEngine();
    engine.load(circuit);

    pulseWeWrite(engine, circuit, 0x000, 0x00);
    driveData(circuit, 0);
    driveAddr(circuit, 0x000);
    driveCe(circuit, true);
    driveOe(circuit, true);
    const samples: number[] = [];
    for (let i = 0; i < 6; i++) {
      engine.step(DT);
      samples.push((readBusVoltageByte(engine) >> 6) & 1);
    }

    expect(new Set(samples).size).toBe(2);
  });

  it("reads unrelated addresses normally during the busy window", () => {
    const bytes = new Uint8Array(2048).fill(0xff);
    bytes[0x100] = 0x33;
    const circuit = makeEepromCircuit("28c16", { contents: btoa(String.fromCharCode(...bytes)) });
    const engine = new SimEngine();
    engine.load(circuit);

    pulseWeWrite(engine, circuit, 0x000, 0x80);
    driveData(circuit, 0);
    driveAddr(circuit, 0x100);
    driveCe(circuit, true);
    driveOe(circuit, true);
    engine.step(DT);

    expect(readBusVoltageByte(engine)).toBe(0x33);
  });

  it("drops unsequenced 28C256 writes while SDP is enabled", () => {
    const circuit = makeEepromCircuit("28c256");
    const engine = new SimEngine();
    engine.load(circuit);

    writeAndWait(engine, circuit, 0x100, 0x5a);

    expect(engine.takeEepromUpdates()[EEPROM_ID]).toBeUndefined();
    expect(readData(engine, circuit, 0x100)).toBe(0xff);
  });

  it("unlocks 28C256 writes after the SDP sequence", () => {
    const circuit = makeEepromCircuit("28c256");
    const engine = new SimEngine();
    engine.load(circuit);

    writeAndWait(engine, circuit, 0x5555, 0xaa);
    writeAndWait(engine, circuit, 0x2aaa, 0x55);
    writeAndWait(engine, circuit, 0x5555, 0xa0);
    writeAndWait(engine, circuit, 0x100, 0x5a);

    expect(engine.takeEepromUpdates()[EEPROM_ID]?.[0x100]).toBe(0x5a);
    expect(readData(engine, circuit, 0x100)).toBe(0x5a);
  });

  it("resets the SDP sequence on a bad byte", () => {
    const circuit = makeEepromCircuit("28c256");
    const engine = new SimEngine();
    engine.load(circuit);

    writeAndWait(engine, circuit, 0x5555, 0xaa);
    writeAndWait(engine, circuit, 0x2aaa, 0x55);
    writeAndWait(engine, circuit, 0x5555, 0xff);
    writeAndWait(engine, circuit, 0x100, 0x5a);

    expect(readData(engine, circuit, 0x100)).toBe(0xff);
  });

  it("bypasses 28C256 SDP when sdpBypass is set", () => {
    const circuit = makeEepromCircuit("28c256", { sdpBypass: 1 });
    const engine = new SimEngine();
    engine.load(circuit);

    writeAndWait(engine, circuit, 0x100, 0x5a);

    expect(readData(engine, circuit, 0x100)).toBe(0x5a);
  });

  it("restores pending writes across snapshots", () => {
    const circuit = makeEepromCircuit("28c16");
    const engine = new SimEngine();
    engine.load(circuit);

    pulseWeWrite(engine, circuit, 0x100, 0x5a);
    const snap = engine.saveState();
    stepFor(engine, 15e-3);
    expect(readData(engine, circuit, 0x100)).toBe(0x5a);

    engine.restoreState(snap);
    stepFor(engine, 15e-3);
    expect(readData(engine, circuit, 0x100)).toBe(0x5a);
  });

  it("carries EEPROM bytes across reloads with unchanged contents", () => {
    const circuit = makeEepromCircuit("28c16");
    const engine = new SimEngine();
    engine.load(circuit);
    writeAndWait(engine, circuit, 0x100, 0x5a);

    engine.load(circuit);

    expect(readData(engine, circuit, 0x100)).toBe(0x5a);
  });

  it("reseeds EEPROM bytes when contents changes", () => {
    const circuit = makeEepromCircuit("28c16");
    const engine = new SimEngine();
    engine.load(circuit);
    writeAndWait(engine, circuit, 0x100, 0x5a);

    const next = new Uint8Array(2048).fill(0xff);
    next[0x100] = 0x22;
    circuit.components.find((c) => c.id === EEPROM_ID)!.params.contents = btoa(String.fromCharCode(...next));
    engine.load(circuit);

    expect(readData(engine, circuit, 0x100)).toBe(0x22);
  });

  it("emits EEPROM updates once per committed write", () => {
    const circuit = makeEepromCircuit("28c16");
    const engine = new SimEngine();
    engine.load(circuit);

    writeAndWait(engine, circuit, 0x100, 0x5a);
    const first = engine.takeEepromUpdates();
    const second = engine.takeEepromUpdates();

    expect(first[EEPROM_ID]?.[0x100]).toBe(0x5a);
    expect(second).toEqual({});
  });

});
