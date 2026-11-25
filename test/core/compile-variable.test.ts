/**
 * compile/variable.ts (W1h item H1a): `potentiometer`, `trimmer`, `resistor_array`, `ldr`, `push_dpdt`,
 * `current_source` and `ferrite_bead` compiled to v2 records, their live-param specs and their declines. The
 * thermistor stays a decline (H4c).
 *
 * H1b admitted the ldr, push_dpdt, current_source and ferrite_bead to `CORE_IMPLEMENTED_KINDS`, H1c the pot, trimmer
 * and array with their overloads (a rated part carries its `p_max` in its record). The record tests use `UNRATED`, a
 * part whose specs carry no `p_max`, where the rating is not the point. Expected values are classic's own:
 * `potentiometerSegments`, `ldrResistance`, `specsForComponent`, and the numbers in spec 7.6, 7.7, 7.9, 7.14,
 * 8.13 (pot p_max 0.2 W, trimmer 0.1 W, bead 0.056 W at 3.5 mohm).
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { ldrResistance, SimEngine, specsForComponent, type SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { potentiometerSegments } from "../../src/sim/engine/devices/passives.js";
import { compileCircuit, LIVE_PARAM_SPECS } from "../../src/core/compile/index.js";
import { compileSlowRecords, checkSlowRecords } from "../../src/core/compile/thermal.js";
import { ldrLiveResistance, potLiveSegments } from "../../src/core/compile/variable.js";
import { CoreEngine } from "../../src/core/engine.js";
import { decodeProgram } from "../../src/core/format.js";
import { CORE_IMPLEMENTED_KINDS, CORE_KIND_CODES, CORE_RECORD_SHAPES } from "../../src/core/kinds.js";
import { openPinHashPrefix, deadBandHashPrefix } from "../../src/core/compile/topology.js";
import { __resetCoreForTest, coreCall, initCore, readCoreErrorMessage } from "../../src/core/wasm.js";

type SimComponent = SimCircuit["components"][number];

function part(id: string, kind: string, pinIds: string[], params: Record<string, number | string> = {}, catalogUid?: string): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params, ...(catalogUid ? { catalogUid } : {}) };
}

function wire(fromComponent: string, fromPin: string, toComponent: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

const H1A_KINDS = ["potentiometer", "trimmer", "resistor_array", "ldr", "push_dpdt", "current_source", "ferrite_bead"] as const;
const GATE: ReadonlySet<string> = new Set([...CORE_IMPLEMENTED_KINDS, ...H1A_KINDS]);

/** A catalog uid whose specs carry no `p_max`, so a pot or array on it is unrated and compiles. */
const UNRATED = "reg-7805";

const POT_PINS = ["cw", "wiper", "ccw"];
const ARRAY_PINS = ["a1", "b1", "a2", "b2", "a3", "b3", "a4", "b4"];
const DPDT_PINS = ["l1", "c1", "r1", "l2", "c2", "r2"];

/** A one-part circuit (a source across the first two pins keeps every net named). */
function compileOne(component: SimComponent, environment?: SimCircuit["environment"], gate: ReadonlySet<string> = GATE) {
  const circuit: SimCircuit = { components: [component], wires: [], ...(environment ? { environment } : {}) };
  return compileCircuit(circuit, gate);
}

/** The program's records without the section anchors (code 26), in record order: components, then ratings. */
function recordsOf(program: Float64Array) {
  return decodeProgram(program).devices.filter((device) => device.kindCode !== CORE_KIND_CODES.section_anchor);
}

function compiled(component: SimComponent, environment?: SimCircuit["environment"]) {
  const result = compileOne(component, environment);
  if (!result.ok) throw new Error(`expected ${component.kind} to compile, got ${result.reasons.join(", ")}`);
  return { result: result.compiled, devices: recordsOf(result.compiled.program) };
}

function reasonsOf(component: SimComponent, environment?: SimCircuit["environment"]): string[] {
  const result = compileOne(component, environment);
  if (result.ok) throw new Error(`expected ${component.kind} to decline`);
  return result.reasons;
}

describe("the default gate admits the H1b and H1c kinds and the thermistor", () => {
  it("admits every one of them", () => {
    for (const kind of H1A_KINDS) {
      expect(CORE_IMPLEMENTED_KINDS.has(kind), kind).toBe(true);
    }
    // A rated catalog pot compiles through the default gate, its rating in its record.
    const pot = compileCircuit({ components: [part("p1", "potentiometer", POT_PINS, { rTotal: 10000 }, "potentiometer")], wires: [] });
    expect(pot.ok).toBe(true);
  });

  it("compiles the thermistor (H4c) as [rNominal, beta, dissipationFactor, thermalTau, ambientC]", () => {
    const thermistor = part("t1", "thermistor", ["a", "b"], { rNominal: 4700, beta: 3435, dissipationFactor: 0, thermalTau: 30 });
    const { devices } = compiled(thermistor, { temperatureC: 40 });
    expect(devices[0]!.kindCode).toBe(CORE_KIND_CODES.thermistor);
    // The dissipation factor floors at classic's 1e-6; the ambient is the environment's over params.tempC.
    expect(devices[0]!.params).toEqual([4700, 3435, 1e-6, 30, 40]);
    expect(compiled(part("t1", "thermistor", ["a", "b"], { tempC: 5 })).devices[0]!.params).toEqual([10000, 3950, 0.007, 15, 5]);
    expect(reasonsOf(part("t1", "thermistor", ["a", "b"], { beta: "x" }))).toEqual(["malformed:t1:non-finite-thermistor-params"]);
    expect(reasonsOf(part("t1", "thermistor", ["a", "b"], { rNominal: 0 }))).toEqual(["malformed:t1:non-positive-resistance"]);
    expect(CORE_IMPLEMENTED_KINDS.has("thermistor")).toBe(true);
  });
});

describe("potentiometer and trimmer (R56 1; spec 7.7)", () => {
  it("emit [rCw, rCcw, pMax] from classic's potentiometerSegments, pins cw/wiper/ccw", () => {
    const pot = part("p1", "potentiometer", POT_PINS, { rTotal: 10000, position: 0.25 }, UNRATED);
    const { devices } = compiled(pot);
    expect(devices).toHaveLength(1);
    expect(devices[0]!.kindCode).toBe(CORE_KIND_CODES.potentiometer);
    // position runs CCW(0) to CW(1): at 0.25 the wiper is a quarter from CCW.
    expect(devices[0]!.params).toEqual([7500, 2500, 0]);
    expect(devices[0]!.params.slice(0, 2)).toEqual(Object.values(potentiometerSegments(pot)));
    expect(devices[0]!.pins).toHaveLength(3);

    const trim = compiled(part("t1", "trimmer", POT_PINS, { rTotal: 5000, position: 0.5 }, UNRATED)).devices[0]!;
    expect(trim.kindCode).toBe(CORE_KIND_CODES.trimmer);
    expect(trim.params).toEqual([2500, 2500, 0]);
  });

  it("carry a rated part's p_max (every catalog pot and trimmer, 0.2 W and 0.1 W) for H1c's two-segment overload", () => {
    expect(compiled(part("p1", "potentiometer", POT_PINS, { rTotal: 10000 }, "potentiometer")).devices[0]!.params).toEqual([5000, 5000, 0.2]);
    expect(compiled(part("t1", "trimmer", POT_PINS, {}, "trimmer")).devices[0]!.params).toEqual([5000, 5000, 0.1]);
    // No catalog uid resolves to the kind's default part, which is rated too.
    expect(compiled(part("p1", "potentiometer", POT_PINS)).devices[0]!.params[2]).toBe(0.2);
  });

  it("applies classic's defaults, the log taper and the 1 ohm floor at the stops", () => {
    expect(compiled(part("p1", "potentiometer", POT_PINS, {}, UNRATED)).devices[0]!.params.slice(0, 2)).toEqual([5000, 5000]);
    // A log pot at position 0 still has rCcw = 0.01 rTotal (spec 7.7).
    const log0 = compiled(part("p1", "potentiometer", POT_PINS, { rTotal: 10000, position: 0, taper: "log" }, UNRATED)).devices[0]!;
    expect(log0.params[0]).toBeCloseTo(9900, 9);
    expect(log0.params[1]).toBeCloseTo(100, 9);
    const cw = compiled(part("p1", "potentiometer", POT_PINS, { rTotal: 10000, position: 1 }, UNRATED)).devices[0]!;
    expect(cw.params.slice(0, 2)).toEqual([1, 10000]);
    // An rTotal below 1 is floored to 1 ohm first, then each segment to 1.
    expect(compiled(part("p1", "potentiometer", POT_PINS, { rTotal: 0.2, position: 0.5 }, UNRATED)).devices[0]!.params.slice(0, 2)).toEqual([1, 1]);
  });

  it("writes pMax 0 for a part with no rating, and for a non-positive one", () => {
    expect(compiled(part("p1", "potentiometer", POT_PINS, { rTotal: 100 }, "reg-7805")).devices[0]!.params[2]).toBe(0);
  });

  it("resolves the pins by id whatever the declared order, and reads an absent id as node 0", () => {
    const hashes = (id: string, ids: readonly string[]) => ids.map((pin) => [openPinHashPrefix(id, pin), deadBandHashPrefix(id, pin)]);
    const shuffled = compiled(part("p1", "potentiometer", ["ccw", "cw", "wiper"], { rTotal: 100 }, UNRATED)).devices[0]!.pins;
    expect(shuffled.map((pin) => [pin.openHash, pin.deadHash])).toEqual(hashes("p1", POT_PINS));
  });

  it("declines fewer than 3 pins (classic stamps nothing) and a non-finite segment", () => {
    expect(reasonsOf(part("p1", "potentiometer", ["cw", "wiper"]))).toEqual(["malformed:p1:too-few-pins"]);
    expect(reasonsOf(part("p1", "trimmer", POT_PINS, { rTotal: "abc" }))).toEqual(["malformed:p1:non-finite-resistance"]);
    expect(reasonsOf(part("p1", "potentiometer", POT_PINS, { rTotal: Infinity }))).toEqual(["malformed:p1:non-finite-resistance"]);
  });

  it("exposes rCw and rCcw as live params, flooring at 1 ohm, and refuses a non-finite value", () => {
    for (const kind of ["potentiometer", "trimmer"]) {
      const spec = LIVE_PARAM_SPECS.get(kind)!;
      expect(spec.paramNames).toEqual(["rCw", "rCcw", "pMax"]);
      expect(CORE_RECORD_SHAPES.get(CORE_KIND_CODES[kind as "potentiometer"])!.liveParams).toEqual([0, 1]);
      expect(spec.resolveLiveValue("rCw", 2500)).toBe(2500);
      expect(spec.resolveLiveValue("rCcw", "3300")).toBe(3300);
      expect(spec.resolveLiveValue("rCw", 0.25)).toBe(1);
      expect(() => spec.resolveLiveValue("rCw", Number.NaN)).toThrow(RangeError);
      expect(() => spec.resolveLiveValue("rCcw", Infinity)).toThrow(RangeError);
    }
  });

  it("gives the facade one helper that maps a knob move to the two wire values", () => {
    const pot = part("p1", "potentiometer", POT_PINS, { rTotal: 10000, position: 0.5, taper: "linear" });
    expect(potLiveSegments(pot, { position: 0.1 })).toEqual({ rCw: 9000, rCcw: 1000 });
    expect(potLiveSegments(pot, { rTotal: 2000 })).toEqual({ rCw: 1000, rCcw: 1000 });
    const log = potLiveSegments(pot, { position: 0.5, taper: "log" });
    expect(log.rCcw).toBeCloseTo(1000, 9);
    expect(log.rCw).toBeCloseTo(9000, 9);
    // The component is not mutated.
    expect(pot.params.position).toBe(0.5);
  });
});

describe("resistor_array (passives.ts:924-1010; spec 10.10)", () => {
  it("carries a rated array's per-channel p_max (every catalog one, 0.125 W) for H1c's per-channel overload", () => {
    const array = part("rn1", "resistor_array", ARRAY_PINS, { resistance: 4700 }, "resistor-array");
    expect(specsForComponent(array)!.p_max).toBe(0.125);
    expect(compiled(array).devices[0]!.params).toEqual([4700, 0.125]);
    expect(compiled(part("rn1", "resistor_array", ARRAY_PINS)).devices[0]!.params).toEqual([10000, 0.125]);
  });

  it("emits [R, pMaxPerChannel] on 8 by-id pins with the channel base as its blob", () => {
    const array = part("rn1", "resistor_array", ARRAY_PINS, { resistance: 4700 }, UNRATED);
    const { result, devices } = compiled(array);
    expect(devices[0]!.kindCode).toBe(CORE_KIND_CODES.resistor_array);
    expect(devices[0]!.pins).toHaveLength(8);
    expect(devices[0]!.params).toEqual([4700, 0]);
    expect(devices[0]!.blob).toEqual([0]);
    expect(result.channelSlots).toEqual([["rn1", 0], ["rn1", 1], ["rn1", 2], ["rn1", 3]]);
  });

  it("claims consecutive channel slots per array, in component order", () => {
    const circuit: SimCircuit = {
      components: [part("rn1", "resistor_array", ARRAY_PINS, {}, UNRATED), part("rn2", "resistor_array", ARRAY_PINS, { resistance: 330 }, UNRATED)],
      wires: [],
    };
    const result = compileCircuit(circuit, GATE);
    if (!result.ok) throw new Error(result.reasons.join());
    const devices = recordsOf(result.compiled.program);
    expect(devices.map((d) => d.blob)).toEqual([[0], [4]]);
    expect(result.compiled.channelSlots).toHaveLength(8);
    expect(result.compiled.channelSlots[5]).toEqual(["rn2", 1]);
  });

  it("defaults to 10 kohm, floors at 1 ohm and takes no rating from an unidentified part", () => {
    expect(compiled(part("rn1", "resistor_array", ARRAY_PINS, {}, UNRATED)).devices[0]!.params[0]).toBe(10000);
    expect(compiled(part("rn1", "resistor_array", ARRAY_PINS, { resistance: 0.01 }, UNRATED)).devices[0]!.params[0]).toBe(1);
    expect(compiled(part("rn1", "resistor_array", ARRAY_PINS, {}, "reg-7805")).devices[0]!.params[1]).toBe(0);
  });

  it("declines a non-finite resistance and, like classic, has no pin-count guard", () => {
    expect(reasonsOf(part("rn1", "resistor_array", ARRAY_PINS, { resistance: "x" }))).toEqual(["malformed:rn1:non-finite-resistance"]);
    expect(reasonsOf(part("rn1", "resistor_array", ARRAY_PINS, { resistance: Infinity }))).toEqual(["malformed:rn1:non-finite-resistance"]);
    // Four pins only: the absent by-id pins read node 0, as classic's missing pins do.
    expect(compileOne(part("rn1", "resistor_array", ["a1", "b1", "a2", "b2"], {}, UNRATED)).ok).toBe(true);
  });

  it("has no live params", () => {
    expect(LIVE_PARAM_SPECS.has("resistor_array")).toBe(false);
  });

  it("claims no slot when it declines", () => {
    const circuit: SimCircuit = {
      components: [part("rn0", "resistor_array", ARRAY_PINS, { resistance: "x" }), part("rn1", "resistor_array", ARRAY_PINS, {}, UNRATED)],
      wires: [],
    };
    const result = compileCircuit(circuit, GATE);
    expect(result.ok).toBe(false);
  });
});

describe("ldr (R56 4; spec 7.9)", () => {
  const ldr = (params: Record<string, number | string> = {}) => part("l1", "ldr", ["a", "b"], params);

  it("emits the resistance classic's ldrResistance gives for the lux in force, environment first", () => {
    // 100 lx default, no environment.
    expect(compiled(ldr()).devices[0]!.params).toEqual([ldrResistance(1_000_000, 500, 100, 15_000, 0.7)]);
    expect(compiled(ldr()).devices[0]!.kindCode).toBe(CORE_KIND_CODES.ldr);
    // params.lux is the fallback for a circuit with no environment lux.
    expect(compiled(ldr({ lux: 10 })).devices[0]!.params).toEqual([15_000]);
    // environment.lux wins over params.lux.
    expect(compiled(ldr({ lux: 10 }), { lux: 1000 }).devices[0]!.params).toEqual([ldrResistance(1_000_000, 500, 1000, 15_000, 0.7)]);
    // Dark: rDark. A negative lux is clamped to 0 first.
    expect(compiled(ldr(), { lux: 0 }).devices[0]!.params).toEqual([1_000_000]);
    expect(compiled(ldr(), { lux: -5 }).devices[0]!.params).toEqual([1_000_000]);
  });

  it("clamps to [rLight, rDark] and honours custom r10 and gamma", () => {
    expect(compiled(ldr({ rLight: 2000 }), { lux: 100000 }).devices[0]!.params).toEqual([2000]);
    expect(compiled(ldr({ rDark: 20000 }), { lux: 0.001 }).devices[0]!.params).toEqual([20000]);
    expect(compiled(ldr({ r10: 8000, gamma: 1 }), { lux: 20 }).devices[0]!.params).toEqual([ldrResistance(1_000_000, 500, 20, 8000, 1)]);
  });

  it("declines a non-finite or non-positive resistance and fewer than 2 pins", () => {
    expect(reasonsOf(ldr({ rDark: "dark" }), { lux: 0 })).toEqual(["malformed:l1:non-finite-resistance"]);
    expect(reasonsOf(ldr({ lux: "bright" }))).toEqual(["malformed:l1:non-finite-resistance"]);
    expect(reasonsOf(ldr({ rDark: 0 }), { lux: 0 })).toEqual(["malformed:l1:non-positive-resistance"]);
    expect(reasonsOf(part("l1", "ldr", ["a"]))).toEqual(["malformed:l1:too-few-pins"]);
  });

  it("exposes R as the one live param and gives the facade a helper for the environment move", () => {
    const spec = LIVE_PARAM_SPECS.get("ldr")!;
    expect(spec.paramNames).toEqual(["R"]);
    expect(spec.resolveLiveValue("R", 1234)).toBe(1234);
    expect(() => spec.resolveLiveValue("R", 0)).toThrow(RangeError);
    expect(() => spec.resolveLiveValue("R", Number.NaN)).toThrow(RangeError);
    expect(ldrLiveResistance(ldr(), 10)).toBe(15_000);
    expect(ldrLiveResistance(ldr(), -3)).toBe(1_000_000);
    expect(ldrLiveResistance(ldr({ rLight: 700 }), 1e9)).toBe(700);
  });
});

describe("push_dpdt (spec 7.6)", () => {
  it("emits [position] on 6 by-id pins l1 c1 r1 l2 c2 r2", () => {
    const { devices } = compiled(part("sw1", "push_dpdt", DPDT_PINS, { position: 0 }));
    expect(devices[0]!.kindCode).toBe(CORE_KIND_CODES.push_dpdt);
    expect(devices[0]!.params).toEqual([0]);
    expect(devices[0]!.pins).toHaveLength(6);
    const shuffled = compiled(part("sw1", "push_dpdt", ["r2", "c2", "l2", "r1", "c1", "l1"])).devices[0]!.pins;
    const want = DPDT_PINS.map((pin) => [openPinHashPrefix("sw1", pin), deadBandHashPrefix("sw1", pin)]);
    expect(shuffled.map((pin) => [pin.openHash, pin.deadHash])).toEqual(want);
    expect(devices[0]!.pins.map((pin) => [pin.openHash, pin.deadHash])).toEqual(want);
  });

  it("selects the right throws for any position classic does not read as 0, NaN included", () => {
    const position = (value: number | string | undefined) =>
      compiled(part("sw1", "push_dpdt", DPDT_PINS, value === undefined ? {} : { position: value })).devices[0]!.params[0];
    expect(position(undefined)).toBe(0);
    expect(position(1)).toBe(1);
    expect(position(2)).toBe(1);
    expect(position("abc")).toBe(1);
  });

  it("declines fewer than 6 pins and exposes position as the live param", () => {
    expect(reasonsOf(part("sw1", "push_dpdt", ["l1", "c1", "r1", "l2", "c2"]))).toEqual(["malformed:sw1:too-few-pins"]);
    const spec = LIVE_PARAM_SPECS.get("push_dpdt")!;
    expect(spec.paramNames).toEqual(["position"]);
    expect(spec.resolveLiveValue("position", 0)).toBe(0);
    expect(spec.resolveLiveValue("position", 7)).toBe(1);
  });
});

describe("current_source (R56 9; spec 8.13)", () => {
  const source = (params: Record<string, number | string> = {}) => part("i1", "current_source", ["pos", "neg"], params);

  it("emits [current, rParallel] on positional pins, defaulting both to 0", () => {
    const { devices } = compiled(source({ current: 0.002, rParallel: 1000 }));
    expect(devices[0]!.kindCode).toBe(CORE_KIND_CODES.current_source);
    expect(devices[0]!.params).toEqual([0.002, 1000]);
    expect(compiled(source()).devices[0]!.params).toEqual([0, 0]);
    // Either sign of current is legitimate.
    expect(compiled(source({ current: -5e-3 })).devices[0]!.params).toEqual([-5e-3, 0]);
  });

  it("encodes classic's 'shunt only when finite and > 0' as 0 for everything else", () => {
    for (const rParallel of [0, -10, Infinity, "abc", Number.NaN]) {
      expect(compiled(source({ current: 1e-3, rParallel })).devices[0]!.params, String(rParallel)).toEqual([1e-3, 0]);
    }
  });

  it("declines a non-finite current and fewer than 2 pins", () => {
    expect(reasonsOf(source({ current: "much" }))).toEqual(["malformed:i1:non-finite-current"]);
    expect(reasonsOf(source({ current: Infinity }))).toEqual(["malformed:i1:non-finite-current"]);
    expect(reasonsOf(part("i1", "current_source", ["pos"]))).toEqual(["malformed:i1:too-few-pins"]);
  });

  it("exposes both params as live", () => {
    const spec = LIVE_PARAM_SPECS.get("current_source")!;
    expect(spec.paramNames).toEqual(["current", "rParallel"]);
    expect(CORE_RECORD_SHAPES.get(CORE_KIND_CODES.current_source)!.liveParams).toEqual([0, 1]);
    expect(spec.resolveLiveValue("current", "0.5")).toBe(0.5);
    expect(spec.resolveLiveValue("rParallel", 470)).toBe(470);
    expect(spec.resolveLiveValue("rParallel", -1)).toBe(0);
    expect(spec.resolveLiveValue("rParallel", Infinity)).toBe(0);
    expect(() => spec.resolveLiveValue("current", Number.NaN)).toThrow(RangeError);
  });
});

describe("ferrite_bead (R56 3; spec 7.14): a plain resistor record plus a rating record", () => {
  const bead = (params: Record<string, number | string> = {}, uid?: string) => part("fb1", "ferrite_bead", ["a", "b"], params, uid);

  it("is a resistor record with R = max(1e-3, rDc ?? 0.0035)", () => {
    for (const [params, want] of [[{}, 0.0035], [{ rDc: 0.5 }, 0.5], [{ rDc: 0 }, 1e-3], [{ rDc: -2 }, 1e-3]] as const) {
      const { devices } = compiled(bead(params));
      expect(devices[0]!.kindCode, JSON.stringify(params)).toBe(CORE_KIND_CODES.resistor);
      expect(devices[0]!.params).toEqual([want]);
      expect(devices[0]!.pins).toHaveLength(2);
    }
  });

  it("carries one rating record, device = the bead, pMax by kind inference (0.056 W), rPower = the stamped R", () => {
    const { devices, result } = compiled(bead({ rDc: 0.01 }));
    expect(devices).toHaveLength(2);
    expect(devices[1]!.kindCode).toBe(CORE_KIND_CODES.rating);
    expect(devices[1]!.params).toEqual([0, 0.056, 0.01]);
    // The same by exact uid; and the bead has no thermal profile, so the uid declines nothing.
    expect(compiled(bead({ rDc: 0.01 }, "ferrite-bead")).devices[1]!.params).toEqual([0, 0.056, 0.01]);
    // The bead's device entry keeps its own kind, so setParams and the failure text can tell it from a resistor.
    expect(result.devices.map((d) => d.kind)).toEqual(["ferrite_bead", "section_anchor", "rating"]);
  });

  it("rates the floored default resistance, not the authored one", () => {
    expect(compiled(bead({})).devices[1]!.params).toEqual([0, 0.056, 0.0035]);
    expect(compiled(bead({ rDc: 0 })).devices[1]!.params).toEqual([0, 0.056, 1e-3]);
  });

  it("declines a non-finite rDc and fewer than 2 pins, and has no live params", () => {
    expect(reasonsOf(bead({ rDc: "x" }))).toEqual(["malformed:fb1:non-finite-resistance"]);
    expect(reasonsOf(bead({ rDc: Infinity }))).toEqual(["malformed:fb1:non-finite-resistance"]);
    expect(reasonsOf(part("fb1", "ferrite_bead", ["a"]))).toEqual(["malformed:fb1:too-few-pins"]);
    expect(LIVE_PARAM_SPECS.has("ferrite_bead")).toBe(false);
  });

  it("sits beside ordinary resistors: every resistor-coded record gets exactly one rating record", () => {
    const circuit: SimCircuit = { components: [part("r1", "resistor", ["a", "b"], { resistance: 100 }), bead({ rDc: 0.02 })], wires: [] };
    const result = compileCircuit(circuit, GATE);
    if (!result.ok) throw new Error(result.reasons.join());
    const devices = recordsOf(result.compiled.program);
    expect(devices.map((d) => d.kindCode)).toEqual([CORE_KIND_CODES.resistor, CORE_KIND_CODES.resistor, CORE_KIND_CODES.rating, CORE_KIND_CODES.rating]);
    expect(devices[2]!.params).toEqual([0, 0.25, 100]);
    expect(devices[3]!.params).toEqual([1, 0.056, 0.02]);
  });
});

describe("rating records: pot and array keep pMax in their own record, so they take none", () => {
  it("compileSlowRecords emits a rating record for a bead and for no pot, trimmer or array", () => {
    const circuit: SimCircuit = {
      components: [
        part("p1", "potentiometer", POT_PINS, {}, "potentiometer"),
        part("t1", "trimmer", POT_PINS, {}, "trimmer"),
        part("rn1", "resistor_array", ARRAY_PINS, {}, "resistor-array"),
        part("fb1", "ferrite_bead", ["a", "b"], { rDc: 0.05 }),
        part("l1", "ldr", ["a", "b"]),
      ],
      wires: [],
    };
    const devices = circuit.components.map((c) => ({ componentId: c.id, kind: c.kind }));
    const records = compileSlowRecords(circuit, devices);
    expect(records.map((r) => [r.componentId, r.kind, r.input.params])).toEqual([["fb1", "rating", [3, 0.056, 0.05]]]);
  });

  it("checkSlowRecords accepts a bead's record and refuses one naming a pot", () => {
    const rating = { kindCode: CORE_KIND_CODES.rating, componentIndex: 0, pins: [], params: [0, 0.056, 0.0035] };
    expect(() => checkSlowRecords(["ferrite_bead"], [rating])).not.toThrow();
    expect(() => checkSlowRecords(["potentiometer"], [rating])).toThrow(/not a resistor/);
    // A bead without its record is a compiler bug, as a bare resistor without one is.
    expect(() => checkSlowRecords(["ferrite_bead"], [])).toThrow(/has no rating record/);
  });
});

describe("the compiled programs against the real Rust reader", () => {
  beforeAll(async () => {
    await initCore();
  });
  afterAll(() => {
    __resetCoreForTest();
  });

  function load(program: Float64Array): number {
    return coreCall((exports) => {
      const handle = exports.bo_engine_create();
      const ptr = exports.bo_alloc(program.length) >>> 0;
      try {
        new Float64Array(exports.memory.buffer, ptr, program.length).set(program);
        return exports.bo_engine_load(handle, ptr, program.length);
      } finally {
        exports.bo_free(ptr, program.length);
        exports.bo_engine_destroy(handle);
      }
    });
  }

  it("accepts a source, a bead, a pot, an array, an ldr, a dpdt and a current source on one net", () => {
    // Every pin of every part on one net with a source: the reader checks shapes and values (unrated parts, since a
    // rated pot or array declines until H1c).
    const components = [
      part("v1", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("fb1", "ferrite_bead", ["a", "b"]),
      part("p1", "potentiometer", POT_PINS, { rTotal: 1000, position: 0.3 }, UNRATED),
      part("t1", "trimmer", POT_PINS, { rTotal: 500, position: 0 }, UNRATED),
      part("rn1", "resistor_array", ARRAY_PINS, { resistance: 330 }, UNRATED),
      part("l1", "ldr", ["a", "b"]),
      part("sw1", "push_dpdt", DPDT_PINS, { position: 1 }),
      part("i1", "current_source", ["pos", "neg"], { current: 1e-3, rParallel: 1e6 }),
    ];
    // First pin of each part on the source's + rail, the rest on its - rail.
    const wires = components.slice(1).flatMap((component) =>
      component.pins.map((p, i) => wire("v1", i === 0 ? "pos" : "neg", component.id, p.id)),
    );
    const result = compileCircuit({ components, wires }, GATE);
    if (!result.ok) throw new Error(result.reasons.join());
    const code = load(result.compiled.program);
    expect(code, code === 0 ? "" : coreCall((exports) => readCoreErrorMessage(exports))).toBe(0);
  });

  it("runs a ferrite bead on the core exactly as classic does (it is a resistor record to the solver)", () => {
    // 5 V -> bead (rDc 0.5) -> 99.5 ohm -> back: 50 mA through the bead, 25 mV across it.
    const circuit: SimCircuit = {
      components: [
        part("v1", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("fb1", "ferrite_bead", ["a", "b"], { rDc: 0.5 }),
        part("r1", "resistor", ["a", "b"], { resistance: 99.5 }),
      ],
      wires: [wire("v1", "pos", "fb1", "a"), wire("fb1", "b", "r1", "a"), wire("r1", "b", "v1", "neg")],
    };
    const classic = new SimEngine();
    classic.load(circuit);
    const core = new CoreEngine();
    core.load(circuit, GATE);
    const close = (got: number, want: number) => expect(Math.abs(got - want)).toBeLessThanOrEqual(1e-12 + 1e-9 * Math.abs(want));
    for (const [net, v] of Object.entries(classic.getNetV())) close(core.getNetV()[net]!, v);
    for (const [id, i] of Object.entries(classic.getElementI())) close(core.getElementI()[id]!, i);
    close(core.getElementI().fb1!, 0.05);
    core.dispose();
  });
});
