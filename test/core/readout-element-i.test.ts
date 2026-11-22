/**
 * `buildElementI` is called on every advance, so it works from a plan built once per compiled circuit. This checks the
 * plan against the straightforward Map-per-frame loop it replaced, key for key and bit for bit (`Object.is`: a `-0`
 * must stay a `-0`, a NaN a NaN), over hand-made and random device lists with the crystal's several records, ids that
 * are not published, published ids with no record, a repeated key, and short or undefined current arrays.
 */
import { describe, expect, it } from "vitest";

import type { CompiledCircuit } from "../../src/core/compile/index.js";
import { buildElementI } from "../../src/core/readout.js";

type Dev = { componentId: string; kind: string };

function reference(devices: Dev[], keys: string[], deviceCurrents: ArrayLike<number>): Record<string, number> {
  const currentOf = new Map<string, number>();
  const recordsSeen = new Map<string, number>();
  devices.forEach((device, i) => {
    const seen = recordsSeen.get(device.componentId) ?? 0;
    recordsSeen.set(device.componentId, seen + 1);
    if (device.kind === "crystal" && seen === 1) return;
    const current = deviceCurrents[i] ?? 0;
    currentOf.set(device.componentId, seen === 0 ? current : (currentOf.get(device.componentId) ?? 0) + current);
  });
  const elementI: Record<string, number> = {};
  for (const id of keys) elementI[id] = currentOf.get(id) ?? 0;
  return elementI;
}

function same(a: Record<string, number>, b: Record<string, number>): void {
  expect(Object.keys(a)).toEqual(Object.keys(b));
  for (const k of Object.keys(a)) expect(Object.is(a[k], b[k]), `${k}: ${String(a[k])} vs ${String(b[k])}`).toBe(true);
}

const compiledOf = (devices: Dev[], keys: string[]) => ({ devices, elementIKeys: keys }) as unknown as CompiledCircuit;

describe("buildElementI plan", () => {
  it("matches the reference on a crystal, a negative zero, an unpublished id and a component with no record", () => {
    const devices: Dev[] = [
      { componentId: "r1", kind: "resistor" },
      { componentId: "x1", kind: "crystal" },
      { componentId: "x1", kind: "crystal" },
      { componentId: "x1", kind: "crystal" },
      { componentId: "hidden", kind: "resistor" },
      { componentId: "z", kind: "capacitor" },
    ];
    const keys = ["x1", "r1", "norecord", "r1", "z"];
    const compiled = compiledOf(devices, keys);
    for (const currents of [[-0, 1.5, 7, 0.25, 3, -0], [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], [NaN, 1, 2, 3, 4, 5], [1, 2]]) {
      same(buildElementI(compiled, currents), reference(devices, keys, currents));
    }
    // The same compiled circuit again: the cached plan answers as the first call did.
    same(buildElementI(compiled, [2, 3, 4, 5, 6, 7]), reference(devices, keys, [2, 3, 4, 5, 6, 7]));
  });

  it("matches the reference on random device lists", () => {
    let seed = 12345;
    const rnd = () => ((seed = (Math.imul(seed, 1103515245) + 12345) >>> 0) / 2 ** 32);
    for (let round = 0; round < 200; round++) {
      const ids = Array.from({ length: 1 + Math.floor(rnd() * 8) }, (_, k) => `c${String(k)}`);
      const devices: Dev[] = Array.from({ length: Math.floor(rnd() * 30) }, () => {
        const id = ids[Math.floor(rnd() * ids.length)]!;
        return { componentId: id, kind: rnd() < 0.4 ? "crystal" : "resistor" };
      });
      const keys = Array.from({ length: Math.floor(rnd() * 12) }, () => (rnd() < 0.2 ? "other" : ids[Math.floor(rnd() * ids.length)]!));
      const pick = [0, -0, 1e-300, -2.5, 1e12, NaN];
      const currents = Array.from({ length: devices.length }, () => (rnd() < 0.3 ? pick[Math.floor(rnd() * pick.length)]! : rnd() - 0.5));
      if (rnd() < 0.2) currents.length = Math.floor(currents.length / 2);
      const compiled = compiledOf(devices, keys);
      same(buildElementI(compiled, currents), reference(devices, keys, currents));
      same(buildElementI(compiled, currents), reference(devices, keys, currents));
    }
  });
});
