/**
 * `liveParamDiff` (src/core/edit.ts, W2 D11): the edit-delivery decision. An
 * edit that changes only live params is delivered as `setParams` (state kept
 * exactly); anything else is `null`, which the host turns into a reload with
 * carry. A wrong "live" answer would silently skip a topology or parameter
 * change, so the cases here lean on the null side.
 */

import { beforeAll, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { CoreEngine } from "../../src/core/engine.js";
import { initCore } from "../../src/core/wasm.js";
import { LIVE_PARAM_SPECS } from "../../src/core/compile/index.js";
import { liveParamDiff, VOLTAGE_SOURCE_V_LIVE } from "../../src/core/edit.js";

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(fc: string, fp: string, tc: string, tp: string): SimCircuit["wires"][number] {
  return { from_component: fc, from_pin: fp, to_component: tc, to_pin: tp };
}

/** v1 -> s1 -> r1 -> back to v1, plus a floating led pair so floating-pin net numbering is exercised. */
function base(): SimCircuit {
  return {
    components: [
      part("v1", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("s1", "switch", ["a", "b"], { closed: 0 }),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
      part("d1", "dip_switch", ["a1", "b1", "a2", "b2", "a3", "b3", "a4", "b4"], { positions: 4, sw1: 0, sw2: 0, sw3: 0, sw4: 0 }),
      part("p1", "spdt_switch", ["com", "b", "c"], { position: 0 }),
      part("b1", "push_button", ["a", "b"], { closed: 0 }),
      part("c1", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
    ],
    wires: [
      wire("v1", "pos", "s1", "a"),
      wire("s1", "b", "r1", "a"),
      wire("r1", "b", "v1", "neg"),
      wire("p1", "com", "b1", "a"),
    ],
  };
}

function clone(c: SimCircuit): SimCircuit {
  return JSON.parse(JSON.stringify(c)) as SimCircuit;
}

function comp(c: SimCircuit, id: string): SimComponent {
  const found = c.components.find((x) => x.id === id);
  if (!found) throw new Error(`no component ${id}`);
  return found;
}

describe("liveParamDiff: no change", () => {
  it("an identical circuit is an empty list, not null", () => {
    expect(liveParamDiff(base(), clone(base()))).toEqual([]);
  });

  it("a position/rotation/circuit-id edit is not a change to the program", () => {
    const next = clone(base());
    next.id = "other";
    comp(next, "r1").position = { x: 10, y: 20 };
    comp(next, "r1").rotation = 90;
    expect(liveParamDiff(base(), next)).toEqual([]);
  });

  it("a live value that resolves to the same wire number is no change (closed 1 -> 2, absent -> 0)", () => {
    const a = clone(base());
    comp(a, "s1").params.closed = 1;
    const b = clone(a);
    comp(b, "s1").params.closed = 2;
    expect(liveParamDiff(a, b)).toEqual([]);
    const c = clone(base());
    delete comp(c, "s1").params.closed;
    expect(liveParamDiff(base(), c)).toEqual([]);
  });

  it("any change in component order is null, even when every net id stays put (device and driver ids follow the order)", () => {
    const next = clone(base());
    const r1 = next.components.splice(2, 1)[0]!;
    next.components.splice(1, 0, r1); // r1 is fully wired: no net id moves
    expect(liveParamDiff(base(), next)).toBeNull();
    expect(liveParamDiff(next, base())).toBeNull();
  });

  it("a reorder that also renumbers a floating pin's net id is null", () => {
    const next = clone(base());
    const c1 = next.components.pop()!;
    next.components.splice(0, 0, c1);
    expect(liveParamDiff(base(), next)).toBeNull();
  });

  it("a reorder alongside a live change is still null", () => {
    const next = clone(base());
    comp(next, "s1").params.closed = 1;
    next.components.reverse();
    expect(liveParamDiff(base(), next)).toBeNull();
  });
});

describe("liveParamDiff: switch family", () => {
  it("switch closed 0 -> 1", () => {
    const next = clone(base());
    comp(next, "s1").params.closed = 1;
    expect(liveParamDiff(base(), next)).toEqual([{ componentId: "s1", params: { closed: 1 } }]);
  });

  it("push_button closed 0 -> 1 and the string \"0\" counting as closed", () => {
    const next = clone(base());
    comp(next, "b1").params.closed = 1;
    expect(liveParamDiff(base(), next)).toEqual([{ componentId: "b1", params: { closed: 1 } }]);
    const strNext = clone(base());
    comp(strNext, "b1").params.closed = "0";
    expect(liveParamDiff(base(), strNext)).toEqual([{ componentId: "b1", params: { closed: "0" } }]);
  });

  it("spdt_switch position 0 -> 1 (and 1 -> 0)", () => {
    const next = clone(base());
    comp(next, "p1").params.position = 1;
    expect(liveParamDiff(base(), next)).toEqual([{ componentId: "p1", params: { position: 1 } }]);
    expect(liveParamDiff(next, base())).toEqual([{ componentId: "p1", params: { position: 0 } }]);
  });

  it("dip_switch: one sw, several sw, and two components at once, each as one change per component", () => {
    const one = clone(base());
    comp(one, "d1").params.sw2 = 1;
    expect(liveParamDiff(base(), one)).toEqual([{ componentId: "d1", params: { sw2: 1 } }]);

    const many = clone(base());
    comp(many, "d1").params.sw1 = 1;
    comp(many, "d1").params.sw4 = 1;
    comp(many, "s1").params.closed = 1;
    const diff = liveParamDiff(base(), many);
    expect(diff).toHaveLength(2);
    // Order follows the next circuit's component list.
    expect(diff).toEqual([
      { componentId: "s1", params: { closed: 1 } },
      { componentId: "d1", params: { sw1: 1, sw4: 1 } },
    ]);
  });

  it("dip_switch positions is NOT live: changing it is null", () => {
    const next = clone(base());
    comp(next, "d1").params.positions = 3;
    expect(liveParamDiff(base(), next)).toBeNull();
  });

  it("a live param the new circuit dropped hands over its resolved number (closed 1 -> absent is closed 0)", () => {
    const prev = clone(base());
    comp(prev, "s1").params.closed = 1;
    expect(liveParamDiff(prev, base())).toEqual([{ componentId: "s1", params: { closed: 0 } }]);
    const noPos = clone(base());
    comp(noPos, "p1").params.position = 1;
    const dropped = clone(base());
    delete comp(dropped, "p1").params.position;
    expect(liveParamDiff(noPos, dropped)).toEqual([{ componentId: "p1", params: { position: 0 } }]);
  });

  it("a live param newly present with a value that resolves differently (absent -> 1)", () => {
    const prev = clone(base());
    delete comp(prev, "p1").params.position;
    const next = clone(base());
    comp(next, "p1").params.position = 1;
    expect(liveParamDiff(prev, next)).toEqual([{ componentId: "p1", params: { position: 1 } }]);
  });
});

describe("liveParamDiff: voltage_source V (code 2 param 0), pending the Rust side", () => {
  function vChanged(): SimCircuit {
    const next = clone(base());
    comp(next, "v1").params.voltage = 3.3;
    return next;
  }

  it("the flag ships off: the wired state of Rust decides, and a V change reloads", () => {
    expect(VOLTAGE_SOURCE_V_LIVE).toBe(false);
    expect(liveParamDiff(base(), vChanged())).toBeNull();
  });

  it("with the flag on a V change is a live change; an unchanged V is not", () => {
    expect(liveParamDiff(base(), vChanged(), { voltageSourceLive: true })).toEqual([
      { componentId: "v1", params: { voltage: 3.3 } },
    ]);
    expect(liveParamDiff(base(), clone(base()), { voltageSourceLive: true })).toEqual([]);
  });

  it("with the flag on, V resolves as the compiler does (absent is 5), and a non-finite V reloads", () => {
    const prev = clone(base());
    delete comp(prev, "v1").params.voltage;
    expect(liveParamDiff(prev, base(), { voltageSourceLive: true })).toEqual([]);
    const bad = clone(base());
    comp(bad, "v1").params.voltage = "banana";
    expect(liveParamDiff(base(), bad, { voltageSourceLive: true })).toBeNull();
  });

  it("with the flag on, a V change plus a non-live change is still null", () => {
    const next = vChanged();
    comp(next, "r1").params.resistance = 2000;
    expect(liveParamDiff(base(), next, { voltageSourceLive: true })).toBeNull();
  });

  it("once Rust wires V, LIVE_PARAM_SPECS takes over and the flag no longer matters", () => {
    // Guard for the W2c hand-off: the day voltage_source has a spec the generic
    // path must be the one answering, so a stale flag cannot fork the behaviour.
    if (LIVE_PARAM_SPECS.has("voltage_source")) {
      expect(liveParamDiff(base(), vChanged())).toEqual([{ componentId: "v1", params: { voltage: 3.3 } }]);
    } else {
      expect(liveParamDiff(base(), vChanged())).toBeNull();
    }
  });
});

describe("liveParamDiff: anything else is null (reload)", () => {
  it("a wire added", () => {
    const next = clone(base());
    next.wires.push(wire("c1", "a", "r1", "a"));
    expect(liveParamDiff(base(), next)).toBeNull();
  });

  it("a wire removed", () => {
    const next = clone(base());
    next.wires.pop();
    expect(liveParamDiff(base(), next)).toBeNull();
  });

  it("a wire rewired to another pin", () => {
    const next = clone(base());
    next.wires[3] = wire("p1", "com", "b1", "b");
    expect(liveParamDiff(base(), next)).toBeNull();
  });

  it("a duplicate wire added or removed is conservatively null (the net list is unchanged but the wire list is not)", () => {
    const next = clone(base());
    next.wires.push({ ...next.wires[0]! });
    expect(liveParamDiff(base(), next)).toBeNull();
    expect(liveParamDiff(next, base())).toBeNull();
  });

  it("a wire written the other way round is the same wire", () => {
    const next = clone(base());
    const w = next.wires[1]!;
    next.wires[1] = wire(w.to_component, w.to_pin, w.from_component, w.from_pin);
    // Union order of the groups is unchanged, so the nets are too.
    expect(liveParamDiff(base(), next)).toEqual([]);
  });

  it("a non-live param changed (resistance), alone and alongside a live change", () => {
    const alone = clone(base());
    comp(alone, "r1").params.resistance = 2200;
    expect(liveParamDiff(base(), alone)).toBeNull();
    const both = clone(alone);
    comp(both, "s1").params.closed = 1;
    expect(liveParamDiff(base(), both)).toBeNull();
  });

  it("a non-live param added, removed, or retyped", () => {
    const added = clone(base());
    comp(added, "r1").params.tolerance = 0.05;
    expect(liveParamDiff(base(), added)).toBeNull();
    expect(liveParamDiff(added, base())).toBeNull();
    const retyped = clone(base());
    comp(retyped, "r1").params.resistance = "1000";
    expect(liveParamDiff(base(), retyped)).toBeNull();
  });

  it("a component added, removed, or its kind, catalogUid or pin list changed", () => {
    const added = clone(base());
    added.components.push(part("r2", "resistor", ["a", "b"], { resistance: 10 }));
    expect(liveParamDiff(base(), added)).toBeNull();
    expect(liveParamDiff(added, base())).toBeNull();

    const kind = clone(base());
    comp(kind, "c1").kind = "resistor";
    expect(liveParamDiff(base(), kind)).toBeNull();

    const uid = clone(base());
    comp(uid, "r1").catalogUid = "res-1k";
    expect(liveParamDiff(base(), uid)).toBeNull();

    const pins = clone(base());
    comp(pins, "r1").pins = [{ id: "b" }, { id: "a" }];
    expect(liveParamDiff(base(), pins)).toBeNull();
  });

  it("a component replaced under the same id with another kind that also has live params", () => {
    const next = clone(base());
    comp(next, "s1").kind = "push_button";
    expect(liveParamDiff(base(), next)).toBeNull();
  });

  it("the environment changed (ambient temperature is a compile input)", () => {
    const next = clone(base());
    next.environment = { temperatureC: 60 };
    expect(liveParamDiff(base(), next)).toBeNull();
    const lux = clone(base());
    lux.environment = { lux: 500 };
    expect(liveParamDiff(base(), lux)).toBeNull();
  });

  it("a live change on a kind with no live params at all (resistor closed) is a non-live change", () => {
    const next = clone(base());
    comp(next, "r1").params.closed = 1;
    expect(liveParamDiff(base(), next)).toBeNull();
  });

  it("does not mutate either circuit", () => {
    const prev = base();
    const next = clone(base());
    comp(next, "s1").params.closed = 1;
    const prevJson = JSON.stringify(prev);
    const nextJson = JSON.stringify(next);
    liveParamDiff(prev, next);
    expect(JSON.stringify(prev)).toBe(prevJson);
    expect(JSON.stringify(next)).toBe(nextJson);
  });
});

describe("liveParamDiff: what it returns is what setParams accepts, and agrees with a fresh load", () => {
  beforeAll(async () => {
    await initCore();
  });

  /** 5 V -> switch -> 1k -> 1k to ground: n1 reads 0 V open, 2.5 V closed. */
  function divider(closed: number | string | undefined): SimCircuit {
    const switchParams: Record<string, number | string> = {};
    if (closed !== undefined) switchParams.closed = closed;
    return {
      components: [
        part("v1", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("s1", "switch", ["a", "b"], switchParams),
        part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
        part("r2", "resistor", ["a", "b"], { resistance: 1000 }),
      ],
      wires: [
        wire("v1", "pos", "s1", "a"),
        wire("s1", "b", "r1", "a"),
        wire("r1", "b", "r2", "a"),
        wire("r2", "b", "v1", "neg"),
      ],
    };
  }

  it.each([
    ["closed 0 -> 1", 0, 1],
    ["closed 1 -> 0", 1, 0],
    ["closed 1 -> absent", 1, undefined],
    ["closed \"1\" -> \"\"", "1", ""],
  ])("%s: setParams of the diff reads the same nets as loading the next circuit", (_name, from, to) => {
    const prev = divider(from);
    const next = divider(to);
    const diff = liveParamDiff(prev, next);
    expect(diff).not.toBeNull();
    expect(diff!.length).toBeGreaterThan(0);

    const live = new CoreEngine();
    live.load(prev);
    expect(live.setParams(diff!)).toBe(true);
    const fresh = new CoreEngine();
    fresh.load(next);
    const a = live.getNetV();
    const b = fresh.getNetV();
    expect(Object.keys(a).sort()).toEqual(Object.keys(b).sort());
    for (const key of Object.keys(b)) expect(a[key]!).toBeCloseTo(b[key]!, 9);
  });
});
