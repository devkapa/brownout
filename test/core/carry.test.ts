/**
 * W2c: the carry map (src/core/carry.ts; core/README.md, "Carry"). The core
 * takes per-device records off the old engine; the map moves each onto the
 * new program's device by classic's own reload rules (SimEngine.load,
 * SE:2807-3200): by component id, a capacitor also by its two nets'
 * fingerprint (its sign flipped with its orientation), each kind-gated so a
 * same-id kind swap starts from the new kind's defaults.
 */
import fs from "node:fs";
import path from "node:path";
import { describe, expect, it } from "vitest";
import { CARRY_FAMILY, CARRY_VERSION, decodeCarry, encodeCarry, mapCarry, type CarryRecord, type CarrySource } from "../../src/core/carry.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Component = SimCircuit["components"][number];

function part(id: string, kind: string, pins: [string, string], params: Record<string, number | string> = {}, extra: Partial<Component> = {}): Component {
  return { id, kind, pins: pins.map((p) => ({ id: p })), params, ...extra } as Component;
}

function wire(a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] {
  return { from_component: a, from_pin: ap, to_component: b, to_pin: bp };
}

/** 5 V into r1 to node x, c1 from x to ground, r2 from x to ground; `extra` parts are added at the front. */
function rc(options: { capId?: string; reversed?: boolean; extra?: Component[] } = {}): SimCircuit {
  const cap = options.capId ?? "c1";
  const [p0, p1] = options.reversed ? ["b", "a"] : ["a", "b"];
  return {
    components: [
      ...(options.extra ?? []),
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
      part(cap, "capacitor", ["a", "b"], { capacitance: 1e-6 }),
      part("r2", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    wires: [
      wire("vs", "pos", "r1", "a"),
      wire("r1", "b", cap, p0),
      wire(cap, p1, "vs", "neg"),
      wire("r2", "a", "r1", "b"),
      wire("r2", "b", "vs", "neg"),
    ],
  } as SimCircuit;
}

function source(circuit: SimCircuit): CarrySource {
  const result = compileCircuit(circuit);
  if (!result.ok) throw new Error(`compile: ${JSON.stringify(result.reasons)}`);
  return { circuit, compiled: result.compiled };
}

function deviceOf(s: CarrySource, id: string): number {
  const i = s.compiled.devices.findIndex((d) => d.componentId === id);
  if (i < 0) throw new Error(`no device for ${id}`);
  return i;
}

function rec(device: number, family: number, values: number[]): CarryRecord {
  return { device, family, version: CARRY_VERSION, values };
}

describe("the carry buffer", () => {
  it("encodes and decodes [device, family, version, len, values] records, and refuses a cut buffer", () => {
    const records = [rec(3, CARRY_FAMILY.capacitor, [1.5]), rec(0, CARRY_FAMILY.ic, [50, 0, 2, 1, 0])];
    const buf = encodeCarry(records);
    expect(buf).toEqual([3, 1, 1, 1, 1.5, 0, 2, 1, 5, 50, 0, 2, 1, 0]);
    expect(decodeCarry(Float64Array.from(buf))).toEqual(records);
    expect(() => decodeCarry(buf.slice(0, -1))).toThrow(/carry/);
  });
});

describe("mapCarry: classic's reload rules", () => {
  it("carries a capacitor by id onto the new program's device index", () => {
    const from = source(rc());
    const to = source(rc({ extra: [part("r0", "resistor", ["a", "b"], { resistance: 10 })] }));
    const out = mapCarry([rec(deviceOf(from, "c1"), CARRY_FAMILY.capacitor, [1.5])], from, to);
    expect(out).toEqual([rec(deviceOf(to, "c1"), CARRY_FAMILY.capacitor, [1.5])]);
    expect(deviceOf(to, "c1")).not.toBe(deviceOf(from, "c1"));
  });

  it("carries a renamed capacitor on the same two nets by fingerprint, its sign flipped when its pins are", () => {
    const from = source(rc());
    const vc = [rec(deviceOf(from, "c1"), CARRY_FAMILY.capacitor, [1.5])];
    const renamed = source(rc({ capId: "c9" }));
    expect(mapCarry(vc, from, renamed)).toEqual([rec(deviceOf(renamed, "c9"), CARRY_FAMILY.capacitor, [1.5])]);
    const flipped = source(rc({ capId: "c9", reversed: true }));
    expect(mapCarry(vc, from, flipped)).toEqual([rec(deviceOf(flipped, "c9"), CARRY_FAMILY.capacitor, [-1.5])]);
  });

  it("drops what has no match, an unknown family and an unknown version", () => {
    const from = source(rc());
    const to = source({ ...rc(), components: rc().components.filter((c) => c.id !== "c1"), wires: rc().wires.filter((w) => w.from_component !== "c1" && w.to_component !== "c1") });
    const c1 = deviceOf(from, "c1");
    expect(mapCarry([rec(c1, CARRY_FAMILY.capacitor, [1.5])], from, to)).toEqual([]);
    expect(mapCarry([rec(c1, 99, [1])], from, source(rc()))).toEqual([]);
    expect(mapCarry([{ ...rec(c1, CARRY_FAMILY.capacitor, [1.5]), version: CARRY_VERSION + 1 }], from, source(rc()))).toEqual([]);
  });

  it("gates an IC's state, its slow physics and its latches by kind: a same-id kind swap starts afresh", () => {
    const fixture = JSON.parse(fs.readFileSync(path.resolve(__dirname, "../fixtures/core/16-74ls00-open-input.sim.json"), "utf8")) as SimCircuit;
    const from = source(fixture);
    const u1 = deviceOf(from, "u1");
    const records = [rec(u1, CARRY_FAMILY.ic, [50, 0, 4, 0, 0, 0, 0]), rec(u1, CARRY_FAMILY.slow, [0, 0.1, NaN, NaN, NaN, NaN, NaN]), rec(u1, CARRY_FAMILY.latch, [4, 3, 0.01, 0.2, 0.1, 1])];
    const same = source(fixture);
    expect(mapCarry(records, from, same)).toEqual(records.map((r) => ({ ...r, device: deviceOf(same, "u1") })));
    const swapped = JSON.parse(JSON.stringify(fixture)) as SimCircuit;
    swapped.components.find((c) => c.id === "u1")!.kind = "74ls08";
    expect(mapCarry(records, from, source(swapped))).toEqual([]);
  });

  it("carries a battery's state only onto the same authored charge and catalog part", () => {
    const pack = (charge: number, catalogUid = "battery-9v") =>
      ({
        components: [
          part("b1", "battery_pack", ["pos", "neg"], { voltage: 9, charge, internalResistance: 1 }, { catalogUid } as Partial<Component>),
          part("r1", "resistor", ["a", "b"], { resistance: 100 }),
        ],
        wires: [wire("b1", "pos", "r1", "a"), wire("r1", "b", "b1", "neg")],
      }) as SimCircuit;
    const from = source(pack(1));
    const state = [rec(deviceOf(from, "b1"), CARRY_FAMILY.battery, [0.8, 8.9, 1, 1440, 360, 0])];
    const same = source(pack(1));
    expect(mapCarry(state, from, same)).toEqual([{ ...state[0]!, device: deviceOf(same, "b1") }]);
    expect(mapCarry(state, from, source(pack(0.5))), "another authored charge").toEqual([]);
    expect(mapCarry(state, from, source(pack(1, "battery-aa"))), "another catalog part").toEqual([]);
  });
});
