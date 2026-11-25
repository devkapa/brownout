/**
 * End-to-end cd4511 on the real wasm core (W1h item H2c): the decode, the latch, lamp test and blanking, the datasheet
 * delays (Data 660/520 ns, BL 400/350 ns, LT 150/250 ns, `le` on the package delay) and parity with classic's settled
 * segment levels. Every circuit is a static partition: switches drive the controls, 100k pull-downs load the segments.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { initCore } from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary } from "../../src/parts/part-library.js";

type SimComponent = SimCircuit["components"][number];

const part = (id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}): SimComponent => ({
  id,
  kind,
  pins: pinIds.map((pinId) => ({ id: pinId })),
  params,
});
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

const SEGMENTS = ["a_out", "b_out", "c_out", "d_out", "e_out", "f_out", "g_out"] as const;
/** a b c d e f g lit per digit, from the TI function table (classic's CD4511_SEGMENTS). */
const GLYPH: Record<number, string> = { 0: "abcdef", 1: "bc", 2: "abdeg", 3: "abcdg", 4: "bcfg", 5: "acdfg", 6: "cdefg", 7: "abc", 8: "abcdefg", 9: "abcfg" };

interface Controls {
  bcd: number;
  /** false = LOW. `undefined` leaves the pin open (lt_n and bl_n read HIGH by themselves). */
  lt?: boolean;
  bl?: boolean;
  le: boolean;
}

const switchPins = ["com", "b", "c"];

function build(ctl: Controls): SimCircuit {
  const cat = BUNDLED_DEFAULT_PARTS.find((p) => p.kind === "cd4511")!;
  const ic: SimComponent = { id: "u1", kind: "cd4511", catalogUid: cat.uid, pins: cat.pin_layout.map((p) => ({ id: p.id })), params: {} };
  const components: SimComponent[] = [part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }), ic];
  const wires: SimCircuit["wires"] = [wire("vsrc", "pos", "u1", "vcc"), wire("vsrc", "neg", "u1", "gnd")];
  const drive = (id: string, pin: string, high: boolean | undefined) => {
    if (high === undefined) return;
    components.push(part(`sw_${pin}`, "spdt_switch", switchPins, { position: high ? 1 : 0 }));
    wires.push(wire("vsrc", "neg", `sw_${pin}`, "b"), wire("vsrc", "pos", `sw_${pin}`, "c"), wire(`sw_${pin}`, "com", id, pin));
  };
  (["a", "b", "c", "d"] as const).forEach((pin, bit) => drive("u1", pin, (ctl.bcd >> bit & 1) === 1));
  drive("u1", "lt_n", ctl.lt);
  drive("u1", "bl_n", ctl.bl);
  drive("u1", "le", ctl.le);
  for (const seg of SEGMENTS) {
    components.push(part(`r_${seg}`, "resistor", ["a", "b"], { resistance: 100000 }));
    wires.push(wire("u1", seg, `r_${seg}`, "a"), wire(`r_${seg}`, "b", "vsrc", "neg"));
  }
  return { components, wires };
}

/** The switch parameters of `ctl` as `setParams` entries (positions only: the topology is the same). */
function positions(ctl: Controls) {
  const out: { componentId: string; params: Record<string, number> }[] = [];
  (["a", "b", "c", "d"] as const).forEach((pin, bit) => out.push({ componentId: `sw_${pin}`, params: { position: ctl.bcd >> bit & 1 } }));
  if (ctl.lt !== undefined) out.push({ componentId: "sw_lt_n", params: { position: ctl.lt ? 1 : 0 } });
  if (ctl.bl !== undefined) out.push({ componentId: "sw_bl_n", params: { position: ctl.bl ? 1 : 0 } });
  out.push({ componentId: "sw_le", params: { position: ctl.le ? 1 : 0 } });
  return out;
}

const WITH = new Set<string>([...CORE_IMPLEMENTED_KINDS, "cd4511"]);

function volts(engine: CoreEngine | SimEngine, seg: string): number {
  const net = engine.getNetIdForPin("u1", seg)!;
  return engine.getNetV()[net]!;
}
const lit = (engine: CoreEngine | SimEngine, seg: string) => volts(engine, seg) > 2.5;

function loadCore(ctl: Controls): CoreEngine {
  const e = new CoreEngine();
  e.load(build(ctl), WITH);
  return e;
}

beforeAll(async () => {
  await initCore();
});
afterAll(() => resetPartLibrary());

const T = 1e-6;

describe("cd4511 on the core: decode", () => {
  it("shows the datasheet glyph for every digit and blanks 10 to 15, matching classic's settled levels", () => {
    for (let code = 0; code < 16; code++) {
      const ctl: Controls = { bcd: code, le: false };
      const core = loadCore(ctl);
      core.advance(5 * T);
      const classic = new SimEngine();
      classic.load(build(ctl));
      for (let i = 0; i < 5; i++) classic.step(1e-6);
      const want = GLYPH[code] ?? "";
      SEGMENTS.forEach((seg, i) => {
        expect(lit(core, seg), `code ${code} ${seg}`).toBe(want.includes("abcdefg"[i]!));
        expect(lit(classic, seg), `classic code ${code} ${seg}`).toBe(want.includes("abcdefg"[i]!));
      });
      for (const seg of SEGMENTS) {
        expect(core.digitalState[`u1/${seg}`], `slot ${code} ${seg}`).toBe(want.includes("abcdefg"[SEGMENTS.indexOf(seg)]!) ? 1 : 0);
        expect(core.digitalState[`u1/${seg}`]).toBe(classic.digitalState[`u1/${seg}`]);
      }
    }
  });

  it("an open lt_n and bl_n read HIGH, so the display is neither lamp-tested nor blanked", () => {
    const core = loadCore({ bcd: 8, le: false });
    core.advance(2 * T);
    expect(SEGMENTS.every((s) => lit(core, s))).toBe(true);
  });
});

describe("cd4511 on the core: the latch", () => {
  it("holds the captured digit while le is high and follows the data again when it falls", () => {
    const core = loadCore({ bcd: 5, le: false });
    core.advance(2 * T);
    core.setParams(positions({ bcd: 5, le: true }));
    core.advance(4 * T);
    core.setParams(positions({ bcd: 1, le: true }));
    core.advance(8 * T);
    expect(SEGMENTS.filter((s) => lit(core, s)).map((s) => s[0])).toEqual([..."acdfg"]);
    core.setParams(positions({ bcd: 1, le: false }));
    core.advance(12 * T);
    expect(SEGMENTS.filter((s) => lit(core, s)).map((s) => s[0])).toEqual([..."bc"]);
  });
});

describe("cd4511 on the core: datasheet delays", () => {
  it("a data change: segments rise at +660 ns and fall at +520 ns", () => {
    const core = loadCore({ bcd: 1, le: false });
    core.advance(2 * T);
    expect(lit(core, "c_out") && !lit(core, "a_out")).toBe(true);
    core.setParams(positions({ bcd: 2, le: false })); // 1 (bc) -> 2 (abdeg): a rises, c falls
    core.advance(2 * T + 500e-9);
    expect(lit(core, "c_out"), "c still lit at +500 ns").toBe(true);
    expect(lit(core, "a_out"), "a not yet lit at +500 ns").toBe(false);
    core.advance(2 * T + 540e-9);
    expect(lit(core, "c_out"), "c fell at +520 ns").toBe(false);
    expect(lit(core, "a_out"), "a still dark at +540 ns").toBe(false);
    core.advance(2 * T + 640e-9);
    expect(lit(core, "a_out"), "a still dark at +640 ns").toBe(false);
    core.advance(2 * T + 680e-9);
    expect(lit(core, "a_out"), "a rose at +660 ns").toBe(true);
  });

  it("lamp test: everything lights 150 ns after /LT falls and darkens 250 ns after it rises", () => {
    const core = loadCore({ bcd: 1, lt: true, bl: true, le: false });
    core.advance(2 * T);
    core.setParams(positions({ bcd: 1, lt: false, bl: true, le: false }));
    core.advance(2 * T + 140e-9);
    expect(lit(core, "a_out"), "a at +140 ns").toBe(false);
    core.advance(2 * T + 160e-9);
    expect(SEGMENTS.every((s) => lit(core, s)), "all lit at +160 ns").toBe(true);
    core.advance(4 * T);
    core.setParams(positions({ bcd: 1, lt: true, bl: true, le: false }));
    core.advance(4 * T + 240e-9);
    expect(lit(core, "a_out"), "a still lit at +240 ns").toBe(true);
    core.advance(4 * T + 260e-9);
    expect(lit(core, "a_out"), "a dark at +260 ns").toBe(false);
    expect(lit(core, "b_out") && lit(core, "c_out")).toBe(true);
  });

  it("blanking: lit segments fall 350 ns after /BL falls and rise 400 ns after it rises", () => {
    const core = loadCore({ bcd: 8, lt: true, bl: true, le: false });
    core.advance(2 * T);
    core.setParams(positions({ bcd: 8, lt: true, bl: false, le: false }));
    core.advance(2 * T + 330e-9);
    expect(lit(core, "g_out"), "g at +330 ns").toBe(true);
    core.advance(2 * T + 370e-9);
    expect(SEGMENTS.every((s) => !lit(core, s)), "blank at +370 ns").toBe(true);
    core.advance(4 * T);
    core.setParams(positions({ bcd: 8, lt: true, bl: true, le: false }));
    core.advance(4 * T + 380e-9);
    expect(lit(core, "g_out"), "g still dark at +380 ns").toBe(false);
    core.advance(4 * T + 420e-9);
    expect(SEGMENTS.every((s) => lit(core, s)), "lit at +420 ns").toBe(true);
  });

  it("le releasing a held latch moves the segments on the package delay (250 ns), not a datasheet figure", () => {
    const core = loadCore({ bcd: 1, le: true });
    core.advance(2 * T);
    core.setParams(positions({ bcd: 2, le: true })); // data moves behind the held latch: nothing shows
    core.advance(4 * T);
    expect(lit(core, "a_out")).toBe(false);
    core.setParams(positions({ bcd: 2, le: false }));
    core.advance(4 * T + 240e-9);
    expect(lit(core, "a_out"), "a at +240 ns").toBe(false);
    core.advance(4 * T + 260e-9);
    expect(lit(core, "a_out"), "a at +260 ns").toBe(true);
  });
});
