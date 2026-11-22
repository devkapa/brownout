/**
 * P0-MAGV V7: buildElementI's plan is cached per compiled circuit (src/core/readout.ts). No code writes
 * `compiled.devices` or `elementIKeys` after compileCircuit returns (the fields are readonly; the only pushes are
 * inside compile/index.ts before the object exists). These tests hold the plan to Ohm's law on live engines
 * through a reload, a carried reload that adds a part, one that removes a part, a cold load, and two engines of the
 * same circuit advanced in turn, and hold two readouts of one compiled circuit independent of each other.
 */
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../../src/core/engine.js";
import { compileCircuit } from "../../../src/core/compile/index.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { buildElementI } from "../../../src/core/readout.js";
import { initCore } from "../../../src/core/wasm.js";
import { Net } from "./boards.js";

beforeAll(async () => {
  await initCore();
});
const engines: CoreEngine[] = [];
afterAll(() => engines.forEach((e) => e.dispose()));

const gate = new Set<string>([...CORE_IMPLEMENTED_KINDS]);

function divider(extra: boolean, withCrystal: boolean) {
  const n = new Net();
  n.add("vs", "voltage_source", { voltage: 10 }, "in", "0");
  n.add("r1", "resistor", { resistance: 1000 }, "in", "a");
  n.add("r2", "resistor", { resistance: 3000 }, "a", "0");
  if (extra) n.add("r3", "resistor", { resistance: 500 }, "a", "0");
  if (withCrystal) {
    n.add("xt", "crystal", { fSeries: 1e6, q: 1000, rs: 50, c0: 5 }, "a", "b");
    n.add("r4", "resistor", { resistance: 2000 }, "b", "0");
  }
  return n.circuit();
}

function ohm(e: CoreEngine, id: string, r: number, a: string, b: string): void {
  const v = e.getNetV();
  const net = (pin: string) => v[e.getNetIdForPin(id, pin)!] ?? 0;
  void a;
  void b;
  expect(e.getElementI()[id]!).toBeCloseTo((net("a") - net("b")) / r, 9);
}

describe("P0-MAGV V7: the readout plan", () => {
  it("is right on a first read, after a carried reload that adds a part, one that removes parts, and a cold load", () => {
    const e = new CoreEngine();
    e.load(divider(false, false));
    engines.push(e);
    e.advance(1e-3);
    expect(Object.keys(e.getElementI()).sort()).toEqual(["r1", "r2", "vs"]);
    ohm(e, "r2", 3000, "a", "b");
    e.load(divider(true, false));
    e.advance(2e-3);
    expect(Object.keys(e.getElementI()).sort()).toEqual(["r1", "r2", "r3", "vs"]);
    ohm(e, "r3", 500, "a", "b");
    ohm(e, "r2", 3000, "a", "b");
    e.load(divider(true, true));
    e.advance(3e-3);
    expect(Object.keys(e.getElementI()).sort()).toEqual(["r1", "r2", "r3", "r4", "vs", "xt"]);
    ohm(e, "r4", 2000, "a", "b");
    // the crystal's current is the series current into the part: KCL at node a (r1 in, r2 + r3 + xt out)
    const i = e.getElementI();
    expect(i.r1!).toBeCloseTo(i.r2! + i.r3! + i.xt!, 9);
    e.load(divider(false, false));
    e.advance(4e-3);
    expect(Object.keys(e.getElementI()).sort()).toEqual(["r1", "r2", "vs"]);
    ohm(e, "r2", 3000, "a", "b");
    e.coldLoad(divider(true, false));
    expect(Object.keys(e.getElementI()).sort()).toEqual(["r1", "r2", "r3", "vs"]);
    e.advance(1e-3);
    ohm(e, "r3", 500, "a", "b");
  });

  it("two engines of one circuit, advanced alternately, read their own currents", () => {
    const a = new CoreEngine();
    const b = new CoreEngine();
    a.load(divider(true, true));
    b.load(divider(true, true));
    engines.push(a, b);
    b.setParams([{ componentId: "vs", params: { voltage: 4 } }]);
    for (let k = 1; k <= 5; k++) {
      a.advance(k * 1e-4);
      const ia = { ...a.getElementI() };
      b.advance(k * 1e-4);
      expect(a.getElementI()).toEqual(ia);
      expect(b.getElementI().r1!).toBeCloseTo(0.4 * a.getElementI().r1!, 6);
    }
  });

  it("two readouts of one compiled circuit do not share their result", () => {
    const r = compileCircuit(divider(true, true), gate);
    if (!r.ok) throw new Error("compile");
    const n = r.compiled.devices.length;
    const first = buildElementI(r.compiled, Array.from({ length: n }, (_, k) => k + 1));
    const copy = { ...first };
    const second = buildElementI(r.compiled, Array.from({ length: n }, (_, k) => -(k + 1)));
    expect(first).toEqual(copy);
    expect(second).not.toEqual(first);
    // a result handed out is the caller's: mutating it does not reach the next readout
    first.r1 = 12345;
    expect(buildElementI(r.compiled, Array.from({ length: n }, (_, k) => k + 1)).r1).toBe(copy.r1);
  });
});
