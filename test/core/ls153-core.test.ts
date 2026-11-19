/**
 * The 74LS153 dual 4-to-1 multiplexer on the core (P0-IC, code 62): the compiled record, then the function table on
 * both engines through the headless runner. Expected values come from the TI datasheet (SDLS055A p.1, p.5), not from
 * either engine.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { HeadlessRunner } from "../../src/host/headless.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import { CORE_IC_DELAY_PAIRS, CORE_IMPLEMENTED_KINDS, CORE_SUPPORTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Comp = SimCircuit["components"][number];
type Wire = SimCircuit["wires"][number];

/** DIP-16 order, pin 1 to pin 16 (SDLS055A p.1). */
const PINS = ["/1g", "b", "1c3", "1c2", "1c1", "1c0", "1y", "gnd", "2y", "2c0", "2c1", "2c2", "2c3", "a", "/2g", "vcc"] as const;

const wire = (fc: string, fp: string, tc: string, tp: string): Wire => ({ from_component: fc, from_pin: fp, to_component: tc, to_pin: tp });

/** A powered 153 with every input tied to a rail and each output loaded by 10 k to the rail `pulls` names. */
function circuit(levels: Record<string, 0 | 1>, pulls: { "1y": "vcc" | "gnd"; "2y": "vcc" | "gnd" }): SimCircuit {
  const components: Comp[] = [
    { id: "u1", kind: "74ls153", pins: PINS.map((id) => ({ id })), params: {} },
    { id: "vs", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5 } },
  ];
  const wires: Wire[] = [wire("vs", "pos", "u1", "vcc"), wire("vs", "neg", "u1", "gnd")];
  for (const [pin, level] of Object.entries(levels)) wires.push(wire("vs", level ? "pos" : "neg", "u1", pin));
  for (const y of ["1y", "2y"] as const) {
    components.push({ id: `r${y}`, kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10_000 } });
    wires.push(wire(`r${y}`, "a", "u1", y), wire(`r${y}`, "b", "vs", pulls[y] === "vcc" ? "pos" : "neg"));
  }
  return { components, wires };
}

beforeAll(async () => {
  await initCore();
});

describe("74ls153 compiled record", () => {
  it("is code 62: 16 pins, 13 IC params and one delay pair (strobe, 16 / 21 ns), two slots 1y and 2y", () => {
    const result = compileCircuit(circuit({ a: 0, b: 0, "/1g": 0, "/2g": 0 }, { "1y": "gnd", "2y": "gnd" }), CORE_IMPLEMENTED_KINDS);
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    const dev = decodeProgram(result.compiled.program).devices.find((d) => d.kindCode === CORE_KIND_CODE["74ls153"])!;
    expect(CORE_KIND_CODE["74ls153"]).toBe(62);
    expect(CORE_IC_DELAY_PAIRS["74ls153"]).toEqual(["strobe"]);
    expect(dev.pins).toHaveLength(16);
    expect(dev.params).toHaveLength(15);
    expect(dev.params.slice(13)).toEqual([16 * 1e-9, 21 * 1e-9]);
    expect(dev.blob).toHaveLength(15);
    expect(result.compiled.digitalSlots).toEqual(["u1/1y", "u1/2y"]);
  });

  it("is implemented but not supported until a verifier has been over it", () => {
    expect(CORE_IMPLEMENTED_KINDS.has("74ls153")).toBe(true);
    expect(CORE_SUPPORTED_KINDS.has("74ls153")).toBe(false);
  });
});

describe.each(["classic", "core"] as const)("74ls153 function table on %s", (engine) => {
  function levelsOf(c: SimCircuit): { y1: number; y2: number } {
    const runner = new HeadlessRunner({ engine, coreSupportedKinds: CORE_IMPLEMENTED_KINDS });
    runner.load(c);
    expect(runner.activeEngine).toBe(engine);
    runner.run({ durationS: 1e-3 });
    const v = runner.snapshot().netV;
    return { y1: v[runner.netIdFor("u1", "1y")!]!, y2: v[runner.netIdFor("u1", "2y")!]! };
  }

  it("routes C[2B + A] to each section: one hot channel per section, opposite patterns, every select", () => {
    for (let sel = 0; sel < 4; sel++) {
      for (let hot = 0; hot < 4; hot++) {
        const lv: Record<string, 0 | 1> = { a: (sel & 1) as 0 | 1, b: ((sel >> 1) & 1) as 0 | 1, "/1g": 0, "/2g": 0 };
        for (let i = 0; i < 4; i++) {
          lv[`1c${i}`] = i === hot ? 1 : 0;
          lv[`2c${i}`] = i === hot ? 0 : 1;
        }
        // Pull each output away from the level under test so only a driven Y reads right.
        const y1 = sel === hot ? 1 : 0;
        const y2 = sel === hot ? 0 : 1;
        const { y1: v1, y2: v2 } = levelsOf(circuit(lv, { "1y": y1 ? "gnd" : "vcc", "2y": y2 ? "gnd" : "vcc" }));
        expect(y1 ? v1 > 2.7 : v1 < 0.5, `sel ${sel} hot ${hot}: 1y = ${v1}`).toBe(true);
        expect(y2 ? v2 > 2.7 : v2 < 0.5, `sel ${sel} hot ${hot}: 2y = ${v2}`).toBe(true);
      }
    }
  });

  it("a HIGH strobe drives only its own section's Y LOW against a pull-up (totem-pole, not 3-state)", () => {
    for (const off of [1, 2] as const) {
      const on = off === 1 ? 2 : 1;
      const lv: Record<string, 0 | 1> = { a: 1, b: 0, "/1g": 0, "/2g": 0, [`/${off}g`]: 1 };
      for (let i = 0; i < 4; i++) {
        lv[`1c${i}`] = 1;
        lv[`2c${i}`] = 1;
      }
      const v = levelsOf(circuit(lv, { "1y": "vcc", "2y": "vcc" }));
      expect(v[`y${off}`], `strobed section ${off}`).toBeLessThan(0.5);
      expect(v[`y${on}`], `open section ${on}`).toBeGreaterThan(2.7);
    }
  });
});
