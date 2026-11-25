/**
 * P0-SG2: a carried reload across the kinds that hold no slow state of their own (ldr, push_dpdt, current_source) and
 * the one that holds a latch (ferrite_bead). A host edit of a non-live parameter reloads the circuit; what a reload must
 * keep is every capacitor's charge, the live parameter the host moved earlier (the circuit it reloads with carries
 * the new value), and for a bead the overload stress and its latch, as the run that never reloaded has them.
 *
 * Each case runs the repository's own fixture (test/fixtures/core, 83 to 86) twice: once straight through, once
 * reloaded in the middle of a transient and again at an instant that is not on a step. The reloaded engine reads what
 * the other does at the same instant, within W0 (the reload re-plans the partition's steps, so not bit for bit).
 */
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";
import { BUNDLED_DEFAULT_PARTS } from "../../src/parts/part-library.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

const DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../fixtures/core");
const fixture = (name: string): SimCircuit => JSON.parse(fs.readFileSync(path.join(DIR, `${name}.sim.json`), "utf8")) as SimCircuit;

/** W0: 100 uV + 1e-3 relative; 1 nA + 1e-3 relative. */
const voltTol = (v: number): number => 1e-4 + 1e-3 * Math.abs(v);
const ampTol = (i: number): number => 1e-9 + 1e-3 * Math.abs(i);

function withParams(circuit: SimCircuit, id: string, params: Record<string, number>): SimCircuit {
  const copy = JSON.parse(JSON.stringify(circuit)) as SimCircuit;
  Object.assign(copy.components.find((c) => c.id === id)!.params, params);
  return copy;
}

beforeAll(async () => {
  await initCore();
});

describe("P0-SG2: a carried reload keeps what ldr, push_dpdt, current_source and ferrite_bead depend on", () => {
  // [fixture, the component the host edits live, the edit, a later instant for the reload, the end]
  const CASES: [string, string, Record<string, number>][] = [
    ["83-ldr-divider-rc", "ldr", { lux: 30 }],
    ["85-push-dpdt-press", "sw", { position: 1 }],
    ["86-current-source-rc", "is", { current: 2e-4 }],
  ];
  for (const [name, id, edit] of CASES) {
    it(`${name}: reloaded mid-transient after a live edit, the capacitors and the edited value carry`, () => {
      const base = fixture(name);
      const edited = withParams(base, id, edit);
      const straight = new CoreEngine();
      const reloaded = new CoreEngine();
      straight.load(base, CORE_IMPLEMENTED_KINDS);
      reloaded.load(base, CORE_IMPLEMENTED_KINDS);
      for (const e of [straight, reloaded]) {
        e.advance(0.03);
        e.setParams([{ componentId: id, params: edit }]);
        e.advance(0.047);
      }
      // The reload is at 47 ms, off any classic step; tau here is 47 to 57 ms, so the capacitors are far from settled.
      reloaded.load(edited, CORE_IMPLEMENTED_KINDS);
      expect(reloaded.simTime).toBeCloseTo(0.047, 12);
      for (const e of [straight, reloaded]) e.advance(0.1);
      const [a, b] = [straight.getNetV(), reloaded.getNetV()];
      expect(Object.keys(b).sort()).toEqual(Object.keys(a).sort());
      const charged = Object.values(a).filter((v) => Math.abs(v) > 0.05).length;
      expect(charged, "something is charged, so a lost carry would show").toBeGreaterThan(0);
      for (const net of Object.keys(a)) expect(Math.abs(b[net]! - a[net]!), `net ${net}`).toBeLessThanOrEqual(voltTol(a[net]!));
      const [ia, ib] = [straight.getElementI(), reloaded.getElementI()];
      for (const key of Object.keys(ia)) expect(Math.abs(ib[key]! - ia[key]!), `elementI ${key}`).toBeLessThanOrEqual(ampTol(ia[key]!));
      straight.dispose();
      reloaded.dispose();
    });
  }

  it("ferrite_bead: the overload stress and the latch carry, and a latched bead stays open through a reload", () => {
    const part = BUNDLED_DEFAULT_PARTS.find((p) => p.uid === "ferrite-bead")!;
    const rDc = Number(part.default_params.rDc);
    const pMax = part.electrical_specs!.p_max as number;
    // 4.5 x p_max across the bead alone: by hand the damage reaches 1 at 1 / 3.5 s = 285.7 ms.
    const volts = Math.sqrt(4.5 * pMax * rDc);
    const circuit: SimCircuit = {
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: volts } },
        { id: "fb1", kind: "ferrite_bead", pins: [{ id: "a" }, { id: "b" }], params: {}, catalogUid: "ferrite-bead" },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "fb1", to_pin: "a" },
        { from_component: "fb1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      ],
    };
    const straight = new CoreEngine();
    const reloaded = new CoreEngine();
    straight.load(circuit, CORE_IMPLEMENTED_KINDS);
    reloaded.load(circuit, CORE_IMPLEMENTED_KINDS);
    for (const e of [straight, reloaded]) e.advance(0.15);
    expect(Object.keys(straight.getFailures()), "not yet latched at 150 ms").toEqual([]);
    reloaded.load(circuit, CORE_IMPLEMENTED_KINDS);
    for (const e of [straight, reloaded]) e.advance(0.4);
    const [want, got] = [Object.values(straight.getFailures()), Object.values(reloaded.getFailures())];
    expect(want, "the straight run latched the bead").toHaveLength(1);
    expect(got, "the reloaded run latched it too, from the stress it carried").toHaveLength(1);
    expect(got[0]!.since).toBeCloseTo(want[0]!.since, 9);
    expect(want[0]!.since).toBeCloseTo(1 / 3.5, 9);
    // Once open, a reload carries it open.
    reloaded.load(circuit, CORE_IMPLEMENTED_KINDS);
    expect(reloaded.getElementI().fb1).toBe(0);
    expect(Object.values(reloaded.getFailures())).toHaveLength(1);
    reloaded.advance(0.6);
    expect(reloaded.getElementI().fb1).toBe(0);
    straight.dispose();
    reloaded.dispose();
  });

  it("ferrite_bead: a snapshot holds the stress and the latch, and a restore brings the run back to before the latch", () => {
    const part = BUNDLED_DEFAULT_PARTS.find((p) => p.uid === "ferrite-bead")!;
    const rDc = Number(part.default_params.rDc);
    const volts = Math.sqrt(4.5 * (part.electrical_specs!.p_max as number) * rDc);
    const circuit: SimCircuit = {
      components: [
        { id: "vcc", kind: "voltage_source", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: volts } },
        { id: "fb1", kind: "ferrite_bead", pins: [{ id: "a" }, { id: "b" }], params: {}, catalogUid: "ferrite-bead" },
      ],
      wires: [
        { from_component: "vcc", from_pin: "pos", to_component: "fb1", to_pin: "a" },
        { from_component: "fb1", from_pin: "b", to_component: "vcc", to_pin: "neg" },
      ],
    };
    const e = new CoreEngine();
    e.load(circuit, CORE_IMPLEMENTED_KINDS);
    e.advance(0.15);
    const snap = e.saveState();
    e.advance(0.4);
    const first = Object.values(e.getFailures());
    expect(first).toHaveLength(1);
    e.restoreState(snap);
    expect(e.simTime).toBeCloseTo(0.15, 12);
    expect(Object.keys(e.getFailures()), "back to before the latch").toEqual([]);
    expect(e.getElementI().fb1, "and conducting again").not.toBe(0);
    e.advance(0.4);
    const second = Object.values(e.getFailures());
    expect(second).toHaveLength(1);
    expect(second[0]!.since).toBe(first[0]!.since);
    e.dispose();
  });

  it("ferrite_bead (fixture 84): the charge of its RC carries", () => {
    const base = fixture("84-ferrite-bead-rc");
    const straight = new CoreEngine();
    const reloaded = new CoreEngine();
    straight.load(base, CORE_IMPLEMENTED_KINDS);
    reloaded.load(base, CORE_IMPLEMENTED_KINDS);
    for (const e of [straight, reloaded]) e.advance(0.047);
    reloaded.load(base, CORE_IMPLEMENTED_KINDS);
    for (const e of [straight, reloaded]) e.advance(0.12);
    const [a, b] = [straight.getNetV(), reloaded.getNetV()];
    expect(Math.max(...Object.values(a).map(Math.abs))).toBeGreaterThan(1);
    for (const net of Object.keys(a)) expect(Math.abs(b[net]! - a[net]!), `net ${net}`).toBeLessThanOrEqual(voltTol(a[net]!));
    straight.dispose();
    reloaded.dispose();
  });
});
