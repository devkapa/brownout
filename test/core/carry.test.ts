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
import { afterEach, beforeAll, describe, expect, it } from "vitest";
import { CARRY_FAMILY, CARRY_VERSION, decodeCarry, encodeCarry, mapCarry, type CarryRecord, type CarrySource } from "../../src/core/carry.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS } from "../../src/parts/part-library.js";
import { CoreEngine } from "../../src/core/engine.js";
import { decodeProgram, withCarryBlock } from "../../src/core/format.js";
import { coreCall, initCore, verifyCoreExports, __installCoreForTest, __resetCoreForTest, type CoreExports } from "../../src/core/wasm.js";
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
    const records = [rec(u1, CARRY_FAMILY.ic, [50, 0, 4, 0, 0, 0, 0]), rec(u1, CARRY_FAMILY.slow, [0, 0.1, NaN, NaN, NaN, NaN, NaN, NaN]), rec(u1, CARRY_FAMILY.latch, [4, 3, 0.01, 0.2, 0.1, 1])];
    const same = source(fixture);
    expect(mapCarry(records, from, same)).toEqual(records.map((r) => ({ ...r, device: deviceOf(same, "u1") })));
    const swapped = JSON.parse(JSON.stringify(fixture)) as SimCircuit;
    swapped.components.find((c) => c.id === "u1")!.kind = "74ls08";
    expect(mapCarry(records, from, source(swapped))).toEqual([]);
  });

  it("drops a thermal model's temperature, shutdown and held temperature onto another part", () => {
    const withUid = (catalogUid?: string): SimCircuit => {
      const c = rc();
      if (catalogUid) Object.assign(c.components.find((k) => k.id === "r1")!, { catalogUid });
      return c;
    };
    const from = source(withUid("resistor"));
    expect(from.compiled.thermalDevices.has("r1")).toBe(true);
    const r1 = deviceOf(from, "r1");
    const slow = rec(r1, CARRY_FAMILY.slow, [0, 0.1, 40, 0, NaN, NaN, NaN, 38]);
    expect(mapCarry([slow], from, source(withUid("resistor")))).toEqual([slow]);
    expect(mapCarry([slow], from, source(withUid()))).toEqual([rec(r1, CARRY_FAMILY.slow, [0, 0.1, NaN, NaN, NaN, NaN, NaN, NaN])]);
  });

  it("carries an EEPROM's state only onto the same authored contents, as classic's contents hash", () => {
    const eeprom = (contents: string): SimCircuit => {
      const c = rc();
      const def = BUNDLED_DEFAULT_PARTS.find((d) => d.kind === "28c16")!;
      c.components.push({ id: "u1", kind: "28c16", catalogUid: def.uid, pins: def.pin_layout.map((p) => ({ id: p.id })), params: { contents } } as Component);
      return c;
    };
    const kinds = new Set([...CORE_IMPLEMENTED_KINDS, "28c16"]);
    const src = (c: SimCircuit): CarrySource => {
      const result = compileCircuit(c, kinds);
      if (!result.ok) throw new Error(`compile: ${JSON.stringify(result.reasons)}`);
      return { circuit: c, compiled: result.compiled };
    };
    const from = src(eeprom(""));
    const u1 = deviceOf(from, "u1");
    const r = rec(u1, CARRY_FAMILY.ic, [81, 3, 1, 2048, 1, 1, NaN, NaN, NaN, 0, 0, 0, 0, 0, -1, 8, ...Array(8).fill(NaN), 8, ...Array(24).fill(0)]);
    expect(mapCarry([r], from, src(eeprom("")))).toHaveLength(1);
    expect(mapCarry([r], from, src(eeprom("EQ==")))).toEqual([]);
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

// ---- ruling R100 (a): the ABI stays 2, the program goes to v4 ---------------

/** Loads `program` on a fresh handle through the raw exports: the load's code, and node voltages after `advance` to `t`. */
function rawRun(program: ArrayLike<number>, t: number): { code: number; voltages: number[] } {
  return coreCall((x) => {
    const h = x.bo_engine_create();
    const ptr = x.bo_alloc(program.length);
    try {
      new Float64Array(x.memory.buffer, ptr, program.length).set(Array.from(program));
      const code = x.bo_engine_load(h, ptr, program.length);
      if (code !== 0) return { code, voltages: [] };
      if (t > 0) expect(x.bo_engine_advance(h, t)).toBe(0);
      const n = x.bo_engine_node_count(h);
      return { code, voltages: Array.from(new Float64Array(x.memory.buffer, x.bo_engine_node_voltages(h), n)) };
    } finally {
      x.bo_free(ptr, program.length);
      x.bo_engine_destroy(h);
    }
  });
}

/** `v3` as program v4: the version, then the CARRY block before the trailer. */
function asV4(v3: Float64Array, carry: readonly number[]): number[] {
  const out = Array.from(v3);
  out[1] = 4;
  const trailer = out.pop()!;
  out.push(carry.length, ...carry, trailer);
  return out;
}

describe("program v4's CARRY block (src/core/format.ts)", () => {
  it("withCarryBlock writes v4 with the block before the trailer, and decodeProgram reads it back", () => {
    const v3 = source(rc()).compiled.program;
    const v4 = withCarryBlock(v3, [2, 1, 1, 1, 4.5]);
    expect(v4[1]).toBe(4);
    expect(v4.length).toBe(v3.length + 6);
    const decoded = decodeProgram(v4);
    expect(decoded.version).toBe(4);
    expect(Array.from(decoded.carry)).toEqual([2, 1, 1, 1, 4.5]);
    expect(decoded.devices).toEqual(decodeProgram(v3).devices);
    expect(Array.from(decodeProgram(v3).carry)).toEqual([]);
  });
});

describe("CoreEngine: a reload carries, coldLoad does not (classic's load and coldLoad)", () => {
  beforeAll(async () => {
    await initCore();
  });

  const vc = (core: CoreEngine) => core.getNetV()[core.getNetIdForPin("c1", "a")!]!;

  it("load() carries the capacitor's charge onto the edited circuit; coldLoad() starts it at 0 and the clock at 0", () => {
    const core = new CoreEngine();
    core.load(rc());
    core.advance(1e-3);
    const before = vc(core);
    expect(before).toBeCloseTo(2.5 * (1 - Math.exp(-2)), 6);
    const edited = rc();
    edited.components.find((c) => c.id === "r2")!.params.resistance = 2000;
    core.load(edited);
    expect(core.simTime).toBe(1e-3);
    expect(vc(core)).toBeCloseTo(before, 12);
    // And on from there: 5 V through 1k into 2k || 1 uF from `before`.
    core.advance(2e-3);
    const tau = (1000 * 2000) / 3000 * 1e-6;
    expect(vc(core)).toBeCloseTo(10 / 3 + (before - 10 / 3) * Math.exp(-1e-3 / tau), 4);

    core.coldLoad(edited);
    expect(core.simTime).toBe(0);
    expect(vc(core)).toBe(0);
    core.dispose();
  });

  it("a load onto a fresh engine carries nothing", () => {
    const core = new CoreEngine();
    core.load(rc(), undefined, { simTime: 1e-3 });
    expect(vc(core)).toBe(0);
    core.dispose();
  });
});

describe("R100 (a): program v4 on ABI 2", () => {
  beforeAll(async () => {
    await initCore();
  });

  afterEach(async () => {
    __resetCoreForTest();
    await initCore();
  });

  it("a v3 program, as this package's encoder writes it, loads and runs on the new core", () => {
    const s = source(rc());
    expect(s.compiled.program[1]).toBe(3);
    const run = rawRun(s.compiled.program, 1e-3);
    expect(run.code).toBe(0);
    const x = decodeProgram(s.compiled.program).devices[deviceOf(s, "c1")]!.pins[0]!.node;
    // 5 V through 1k into 1k || 1 uF: tau 0.5 ms, 2.5 (1 - e^-2) at 1 ms.
    expect(run.voltages[x]).toBeCloseTo(2.5 * (1 - Math.exp(-2)), 6);
  });

  it("a v4 program's CARRY block seeds the load: the capacitor starts at its carried charge", () => {
    const s = source(rc());
    const c1 = deviceOf(s, "c1");
    const x = decodeProgram(s.compiled.program).devices[c1]!.pins[0]!.node;
    const cold = rawRun(asV4(s.compiled.program, []), 0);
    expect(cold.code).toBe(0);
    expect(cold.voltages[x]).toBe(0);
    const carried = rawRun(asV4(s.compiled.program, [c1, CARRY_FAMILY.capacitor, CARRY_VERSION, 1, 4]), 0);
    expect(carried.code).toBe(0);
    expect(carried.voltages[x]).toBe(4);
    // A record on the wrong kind is a malformed program.
    expect(rawRun(asV4(s.compiled.program, [deviceOf(s, "r1"), CARRY_FAMILY.capacitor, CARRY_VERSION, 1, 4]), 0).code).toBe(-1);
  });

  it("a host whose core has no carry exports still loads and runs", () => {
    const real = coreCall((x) => x);
    const { bo_engine_carry_take: _take, bo_engine_carry_ptr: _ptr, ...rest } = real as CoreExports & Record<string, unknown>;
    expect(typeof _take).toBe("function");
    const without = verifyCoreExports({ ...rest, memory: real.memory } as unknown as Record<string, unknown>);
    expect("bo_engine_carry_take" in without).toBe(false);
    __installCoreForTest(without);
    const core = new CoreEngine();
    core.load(rc());
    core.advance(1e-3);
    expect(core.simTime).toBeCloseTo(1e-3, 12);
    expect(Object.keys(core.getNetV()).length).toBeGreaterThan(0);
    // A reload on it is a cold one at the same time: nothing to take a carry with.
    core.load(rc());
    expect(core.simTime).toBe(1e-3);
    expect(core.getNetV()[core.getNetIdForPin("c1", "a")!]).toBe(0);
    core.advance(2e-3);
    core.dispose();
  });
});
