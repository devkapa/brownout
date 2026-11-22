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
import { CoreUnsupportedError } from "../../src/core/errors.js";
import { decodeProgram, withCarryBlock } from "../../src/core/format.js";
import { coreCall, initCore, verifyCoreExports, __installCoreForTest, __resetCoreForTest, type CoreExports } from "../../src/core/wasm.js";
import { encodeRamContents } from "../../src/circuit/types.js";
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
    const records = [rec(u1, CARRY_FAMILY.ic, [50, 0, 4, 0, 0, 0, 0]), rec(u1, CARRY_FAMILY.slow, [0, 0.1, NaN, NaN, NaN, NaN, NaN, NaN]), rec(u1, CARRY_FAMILY.latch, [4, 3, 0.01, 0.2, 0.1, 1]), rec(u1, CARRY_FAMILY.sag, [4, 2, 1, NaN, 0, NaN, NaN, 0, NaN, NaN, 0, NaN, NaN])];
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

  it("carries a RAM's state only onto the same preload, as an EEPROM's onto the same contents", () => {
    const ram = (contents?: string): SimCircuit => {
      const c = rc();
      const def = BUNDLED_DEFAULT_PARTS.find((d) => d.kind === "74ls189")!;
      c.components.push({ id: "u1", kind: "74ls189", catalogUid: def.uid, pins: def.pin_layout.map((p) => ({ id: p.id })), params: contents === undefined ? {} : { contents } } as Component);
      return c;
    };
    const kinds = new Set([...CORE_IMPLEMENTED_KINDS, "74ls189"]);
    const src = (c: SimCircuit): CarrySource => {
      const result = compileCircuit(c, kinds);
      if (!result.ok) throw new Error(`compile: ${JSON.stringify(result.reasons)}`);
      return { circuit: c, compiled: result.compiled };
    };
    const countUp = encodeRamContents("74ls189", Array.from({ length: 16 }, (_, i) => i));
    const countDown = encodeRamContents("74ls189", Array.from({ length: 16 }, (_, i) => 15 - i));
    const r = (from: CarrySource) => rec(deviceOf(from, "u1"), CARRY_FAMILY.ic, [80, 2, ...Array(16).fill(9), 1]);
    // No contents before and after: the plain carry by id and kind, as before the field existed.
    const blank = src(ram());
    expect(mapCarry([r(blank)], blank, src(ram()))).toHaveLength(1);
    expect(mapCarry([r(blank)], blank, src(ram("")))).toHaveLength(1);
    // The same preload: the live words carry.
    const up = src(ram(countUp));
    expect(mapCarry([r(up)], up, src(ram(countUp)))).toHaveLength(1);
    // Another preload, a preload added, a preload removed: the chip starts from its new contents.
    expect(mapCarry([r(up)], up, src(ram(countDown)))).toEqual([]);
    expect(mapCarry([r(blank)], blank, src(ram(countUp)))).toEqual([]);
    expect(mapCarry([r(up)], up, src(ram()))).toEqual([]);
    // Two spellings of one preload (base64 with and without a line break) are the same contents.
    expect(mapCarry([r(up)], up, src(ram(`${countUp.slice(0, 8)}\n${countUp.slice(8)}`)))).toHaveLength(1);
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

  it("a carry take that fails is the host's to see: the reload throws as a load-time solve failure would, never a silent cold load", () => {
    // The take brings a copy of the engine to its time (the clamp); that can
    // fail as any advance can (-4, -7). Simulated here: the export answers
    // -4 with a message, as the core does.
    const real = coreCall((x) => x);
    let failNext = false;
    const failing = verifyCoreExports({
      ...(real as unknown as Record<string, unknown>),
      memory: real.memory,
      bo_engine_carry_take: (h: number) => (failNext ? -4 : real.bo_engine_carry_take!(h)),
    });
    __installCoreForTest(failing);
    const core = new CoreEngine();
    core.load(rc());
    core.advance(1e-3);
    failNext = true;
    let thrown: unknown;
    try {
      core.load(rc());
    } catch (err) {
      thrown = err;
    }
    expect(thrown).toBeInstanceOf(CoreUnsupportedError);
    expect((thrown as CoreUnsupportedError).reasons[0]).toMatch(/^carry:solve-failed:/);
    // As after any failed load: no program, the clock kept, and a cold load
    // is the host's call.
    expect(() => core.advance(2e-3)).toThrow();
    failNext = false;
    core.coldLoad(rc());
    core.dispose();
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

describe("P0-MAG: windings and the crystal carry as classic's load() carries `inds` (SE:3077-3131)", () => {
  beforeAll(async () => {
    await initCore();
  });

  /** 5 V into r1 to node x, an inductor `id` from x to ground (pins swapped when `reversed`), a coupled inductor and a crystal beside. */
  function rl(options: { indId?: string; reversed?: boolean; crystalC0?: number } = {}): SimCircuit {
    const ind = options.indId ?? "l1";
    const [p0, p1] = options.reversed ? ["b", "a"] : ["a", "b"];
    return {
      components: [
        part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r1", "resistor", ["a", "b"], { resistance: 100 }),
        part(ind, "inductor", ["a", "b"], { inductance: 10e-3 }),
        { id: "t1", kind: "coupled_inductor", pins: ["a1", "b1", "a2", "b2"].map((id) => ({ id })), params: { l1: 1e-3, l2: 4e-3, k: 0.95 } } as Component,
        part("r3", "resistor", ["a", "b"], { resistance: 200 }),
        part("rl", "resistor", ["a", "b"], { resistance: 1000 }),
        part("r4", "resistor", ["a", "b"], { resistance: 1000 }),
        part("xt", "crystal", ["a", "b"], { fSeries: 1e6, q: 1000, rs: 50, c0: options.crystalC0 ?? 5 }),
      ],
      wires: [
        wire("vs", "pos", "r1", "a"),
        wire("r1", "b", ind, p0),
        wire(ind, p1, "vs", "neg"),
        wire("vs", "pos", "r3", "a"),
        wire("r3", "b", "t1", "a1"),
        wire("t1", "b1", "vs", "neg"),
        wire("t1", "a2", "rl", "a"),
        wire("rl", "b", "t1", "b2"),
        wire("t1", "b2", "vs", "neg"),
        wire("vs", "pos", "r4", "a"),
        wire("r4", "b", "xt", "a"),
        wire("xt", "b", "vs", "neg"),
      ],
    } as SimCircuit;
  }

  it("maps an inductor by id, else by its net pair with its sign flipped by orientation; a coupled pair and a crystal by id only", () => {
    const from = source(rl());
    const xt = from.compiled.devices.flatMap((d, i) => (d.componentId === "xt" ? [i] : []));
    expect(xt.length).toBe(3);
    const records = [
      rec(deviceOf(from, "l1"), CARRY_FAMILY.inductor, [0.02]),
      rec(deviceOf(from, "t1"), CARRY_FAMILY.coupled, [0.004, -0.001]),
      rec(xt[0]!, CARRY_FAMILY.capacitor, [0.5]),
      rec(xt[1]!, CARRY_FAMILY.inductor, [1e-4]),
      rec(xt[2]!, CARRY_FAMILY.capacitor, [4.5]),
    ];
    const same = source(rl());
    const kept = mapCarry(records, from, same);
    const sameXt = same.compiled.devices.flatMap((d, i) => (d.componentId === "xt" ? [i] : []));
    expect(kept).toEqual(
      [
        rec(deviceOf(same, "l1"), CARRY_FAMILY.inductor, [0.02]),
        rec(deviceOf(same, "t1"), CARRY_FAMILY.coupled, [0.004, -0.001]),
        rec(sameXt[0]!, CARRY_FAMILY.capacitor, [0.5]),
        rec(sameXt[1]!, CARRY_FAMILY.inductor, [1e-4]),
        rec(sameXt[2]!, CARRY_FAMILY.capacitor, [4.5]),
      ].sort((a, b) => a.device - b.device || a.family - b.family),
    );
    // A renamed, reversed inductor on the same nets: the fingerprint, sign flipped.
    const renamed = source(rl({ indId: "l9", reversed: true }));
    expect(mapCarry(records, from, renamed).find((r) => r.device === deviceOf(renamed, "l9"))).toEqual(rec(deviceOf(renamed, "l9"), CARRY_FAMILY.inductor, [-0.02]));
    // A crystal whose record count changed (C0 0: no C0 record) starts cold.
    const noC0 = source(rl({ crystalC0: 0 }));
    const noC0Xt = new Set(noC0.compiled.devices.flatMap((d, i) => (d.componentId === "xt" ? [i] : [])));
    expect(mapCarry(records, from, noC0).filter((r) => noC0Xt.has(r.device))).toEqual([]);
  });

  it("load() carries the winding currents onto the edited circuit; coldLoad() starts them at 0", () => {
    const core = new CoreEngine();
    const gate = new Set([...CORE_IMPLEMENTED_KINDS]);
    core.load(rl(), gate);
    core.advance(5e-5);
    const before = core.getElementI();
    expect(Math.abs(before.l1!)).toBeGreaterThan(1e-3);
    expect(Math.abs(before.t1!)).toBeGreaterThan(1e-4);
    const edited = rl();
    edited.components.find((c) => c.id === "r4")!.params.resistance = 2000;
    core.load(edited, gate);
    expect(core.getElementI().l1).toBe(before.l1);
    expect(core.getElementI().t1).toBe(before.t1);
    core.coldLoad(edited, gate);
    expect(core.getElementI().l1).toBe(0);
    core.dispose();
  });
});

describe("P0-MAG: a thermistor's body temperature carries as classic's load() carries `thermalTemps` (SE:3274-3278)", () => {
  beforeAll(async () => {
    await initCore();
  });

  /** 5 V into 100 ohm to node x, a thermistor `id` (100 ohm, a 1 s body) from x to ground. */
  function heater(id = "th", kind = "thermistor"): SimCircuit {
    return {
      components: [
        part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r1", "resistor", ["a", "b"], { resistance: 100 }),
        part(id, kind, ["a", "b"], kind === "thermistor" ? { rNominal: 100, beta: 3950, thermalTau: 1 } : { resistance: 100 }),
      ],
      wires: [wire("vs", "pos", "r1", "a"), wire("r1", "b", id, "a"), wire(id, "b", "vs", "neg")],
    } as SimCircuit;
  }

  it("maps a thermistor's record by id onto a thermistor only", () => {
    const from = source(heater());
    const records = [rec(deviceOf(from, "th"), CARRY_FAMILY.thermistor, [41.5])];
    const same = source(heater());
    expect(mapCarry(records, from, same)).toEqual([rec(deviceOf(same, "th"), CARRY_FAMILY.thermistor, [41.5])]);
    // Renamed: classic's thermalTemps is keyed by id, so the body starts at its ambient.
    expect(mapCarry(records, from, source(heater("th2")))).toEqual([]);
    // The same id as a resistor takes nothing.
    expect(mapCarry(records, from, source(heater("th", "resistor")))).toEqual([]);
  });

  it("load() carries the body onto the edited circuit; coldLoad() starts it at the ambient", () => {
    const core = new CoreEngine();
    const gate = new Set([...CORE_IMPLEMENTED_KINDS]);
    core.load(heater(), gate);
    core.advance(0.5);
    const before = core.getPartTemperature("th")!;
    expect(before).toBeGreaterThan(26);
    const edited = heater();
    edited.components.find((c) => c.id === "r1")!.params.resistance = 120;
    core.load(edited, gate);
    expect(core.getPartTemperature("th")).toBe(before);
    core.coldLoad(edited, gate);
    expect(core.getPartTemperature("th")).toBe(25);
    core.dispose();
  });
});
