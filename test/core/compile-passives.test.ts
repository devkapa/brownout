/**
 * Unit coverage for compile/passives.ts: the W1b switch family (`switch`,
 * `push_button`, `spdt_switch`, `dip_switch`) compiled to v2 program records,
 * and the capacitor param resolver prepared (but not yet emitted) in the same
 * file. Every resolution rule is checked against classic's own runtime
 * behaviour — either directly (the formula matches devices/passives.ts's
 * code, reproduced in this file's helpers) or, for the trickiest edge cases
 * (the string `"0"` counting as closed, NaN position/positions handling),
 * against a REAL classic `SimEngine` load, so a silent drift in classic's own
 * formula would fail this suite too, not just prove this file self-
 * consistent.
 *
 * `switch`/`push_button`/`spdt_switch`/`dip_switch` are not yet in
 * `CORE_IMPLEMENTED_KINDS` (kinds.ts is the orchestrator's table) — every
 * `compileCircuit` call here passes a widened `implementedKinds` set, exactly
 * the documented escape hatch `compileCircuit`'s own doc comment describes
 * ("the parameter exists so a caller (chiefly tests) can compile against a
 * narrower [or here, wider] set").
 */

import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import {
  compileSpdtSwitch,
  resolveCapacitorParams,
  resolveDipSwitchClosed,
  resolveDipSwitchPositions,
  resolveSpdtSel,
  resolveSwitchClosed,
} from "../../src/core/compile/passives.js";
import { buildCoreNodes, computeOpenPinKeys } from "../../src/core/compile/topology.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";

// ---- small circuit-building helpers, local to this file (mirrors
// compile.test.ts's own conventions; this worker owns no shared test-helper
// files) ---------------------------------------------------------------

type SimComponent = SimCircuit["components"][number];

function part(
  id: string,
  kind: string,
  pinIds: string[],
  params: Record<string, number | string> = {},
): SimComponent {
  return { id, kind, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(fromComponent: string, fromPin: string, toComponent: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function circuit(components: SimComponent[], wires: SimCircuit["wires"] = []): SimCircuit {
  return { components, wires };
}

const WITH_PASSIVES = new Set([...CORE_IMPLEMENTED_KINDS, "switch", "push_button", "spdt_switch", "dip_switch", "capacitor"]);

function compileOk(c: SimCircuit) {
  const result = compileCircuit(c, WITH_PASSIVES);
  if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
  return result.compiled;
}

function runClassic(c: SimCircuit, steps = 20, h = 1e-4): SimEngine {
  const engine = new SimEngine();
  engine.load(c);
  for (let i = 0; i < steps; i++) engine.step(h);
  return engine;
}

// ---- switch / push_button --------------------------------------------------

describe("compileSwitchOrPushButton — value resolution", () => {
  it("Boolean(closed): numeric 0/1", () => {
    expect(resolveSwitchClosed(0)).toBe(0);
    expect(resolveSwitchClosed(1)).toBe(0 + 1);
  });

  it("the string \"0\" counts as CLOSED (Boolean(\"0\") is true) — spec §7.6", () => {
    expect(resolveSwitchClosed("0")).toBe(1);
    expect(resolveSwitchClosed("")).toBe(0); // the one string Boolean() treats as false
  });

  it("an absent closed param defaults to open (Boolean(undefined) is false)", () => {
    expect(resolveSwitchClosed(undefined)).toBe(0);
  });

  for (const kind of ["switch", "push_button"] as const) {
    it(`${kind}: compiles positional pins and the closed param, matching CORE_KIND_CODE`, () => {
      const c = circuit([
        part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("sw", kind, ["a", "b"], { closed: 1 }),
        part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
      ], [
        wire("vs", "pos", "sw", "a"),
        wire("sw", "b", "r1", "a"),
        wire("r1", "b", "vs", "neg"),
      ]);
      const compiled = compileOk(c);
      const decoded = decodeProgram(compiled.program);
      const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE[kind])!;
      expect(dev.params).toEqual([1]);
      expect(dev.pins).toHaveLength(2);
      // Positional: pins[0] is "a", pins[1] is "b" — same nodes buildCoreNodes assigns those pins directly.
      const built = buildCoreNodes(c);
      expect(dev.pins[0]!.node).toBe(built.pinNode.get("sw\u0000a"));
      expect(dev.pins[1]!.node).toBe(built.pinNode.get("sw\u0000b"));
    });

    it(`${kind}: too few pins is malformed, not silently accepted`, () => {
      const c = circuit([part("sw", kind, ["a"], { closed: 1 })]);
      const result = compileCircuit(c, WITH_PASSIVES);
      expect(result.ok).toBe(false);
      if (!result.ok) expect(result.reasons).toEqual([`malformed:sw:too-few-pins`]);
    });
  }

  it("the string \"0\" makes a real classic circuit CONDUCT — cross-checked against a live SimEngine run, not just the formula in isolation", () => {
    const c = circuit([
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("sw", "switch", ["a", "b"], { closed: "0" }),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
    ], [
      wire("vs", "pos", "sw", "a"),
      wire("sw", "b", "r1", "a"),
      wire("r1", "b", "vs", "neg"),
    ]);

    const engine = runClassic(c);
    expect(Math.abs(engine.getElementI().sw ?? 0)).toBeGreaterThan(4e-3); // ~5mA: closed, conducting

    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.switch)!;
    expect(dev.params).toEqual([1]); // compiled agrees: closed
  });
});

// ---- spdt_switch ------------------------------------------------------------

describe("compileSpdtSwitch — value resolution", () => {
  it("position 0 selects b (sel 0); any other finite value selects c (sel 1)", () => {
    expect(resolveSpdtSel(0)).toBe(0);
    expect(resolveSpdtSel(1)).toBe(1);
    expect(resolveSpdtSel(2)).toBe(1);
    expect(resolveSpdtSel(-1)).toBe(1);
  });

  it("an absent position defaults to 0 (selects b), matching devices/passives.ts's `?? 0`", () => {
    expect(resolveSpdtSel(undefined)).toBe(0);
  });

  it("NaN (a garbage string) selects c — spec §7.6's own wording", () => {
    expect(resolveSpdtSel("not-a-number")).toBe(1);
    expect(resolveSpdtSel(Number.NaN)).toBe(1);
  });

  it("compiles com/b/c by id, in that fixed order, regardless of the component's own pin array order", () => {
    const c = circuit([part("sw", "spdt_switch", ["c", "com", "b"], { position: 1 })]);
    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.spdt_switch)!;
    expect(dev.pins).toHaveLength(3);
    expect(dev.params).toEqual([1]);
    const built = buildCoreNodes(c);
    expect(dev.pins[0]!.node).toBe(built.pinNode.get("sw\u0000com"));
    expect(dev.pins[1]!.node).toBe(built.pinNode.get("sw\u0000b"));
    expect(dev.pins[2]!.node).toBe(built.pinNode.get("sw\u0000c"));
  });

  it("fewer than 3 declared pins is malformed, not silently compiled — classic's spdtSwitchModel.stamp/acStamp stamp NOTHING below 3 pins (a pure declared-pin-count check, unlike dip_switch's by-id-only rule)", () => {
    // Only "com" is declared; "b" and "c" do not exist on this component at
    // all. Before this guard, compileSpdtSwitch still emitted a full 3-pin
    // by-id record with b/c defaulting to node 0, so the core would close
    // com to ground through the (fabricated) "c" contact — a circuit classic
    // leaves completely open.
    const c = circuit([part("sw", "spdt_switch", ["com"], { position: 0 })]);
    const result = compileCircuit(c, WITH_PASSIVES);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual([`malformed:sw:too-few-pins`]);
  });

  it("a by-id pin the component lacks (but still declares >=3 pins total) maps to node 0 and is reported open — never a compile error", () => {
    // 3 pins declared (satisfies the >=3-pin guard), but the component's
    // actual ids are com/b/spare -- "c" itself is genuinely missing.
    const c = circuit([part("sw", "spdt_switch", ["com", "b", "spare"], { position: 0 })]);
    const result = compileCircuit(c, WITH_PASSIVES);
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    const decoded = decodeProgram(result.compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.spdt_switch)!;
    // pins[2] = c, absent by id -> node 0, open. pins[1] = b resolves normally.
    expect(dev.pins[2]!.node).toBe(0);
    expect(dev.pins[2]!.open).toBe(true);
    const built = buildCoreNodes(c);
    expect(dev.pins[1]!.node).toBe(built.pinNode.get("sw\u0000b"));
  });

  it("NaN position makes a real classic circuit stamp com->c (not com->b) — cross-checked against a live SimEngine run", () => {
    const c = circuit([
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("sw", "spdt_switch", ["com", "b", "c"], { position: "not-a-number" }),
      part("rb", "resistor", ["a", "b"], { resistance: 1000 }),
      part("rc", "resistor", ["a", "b"], { resistance: 1000 }),
    ], [
      wire("vs", "pos", "sw", "com"),
      wire("sw", "b", "rb", "a"),
      wire("rb", "b", "vs", "neg"),
      wire("sw", "c", "rc", "a"),
      wire("rc", "b", "vs", "neg"),
    ]);

    const engine = runClassic(c);
    // com->c conducts (~5mA through rc); com->b does not (rb sees ~0).
    expect(Math.abs(engine.getElementI().rc ?? 0)).toBeGreaterThan(4e-3);
    expect(Math.abs(engine.getElementI().rb ?? 0)).toBeLessThan(1e-4);

    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.spdt_switch)!;
    expect(dev.params).toEqual([1]); // compiled agrees: sel selects c
  });
});

// ---- dip_switch ---------------------------------------------------------

describe("compileDipSwitch / resolveDipSwitchPositions — k clamping", () => {
  it("clamps and rounds positions into [2, 8], matching classic exactly", () => {
    expect(resolveDipSwitchPositions(1)).toBe(2); // below floor
    expect(resolveDipSwitchPositions(2)).toBe(2);
    expect(resolveDipSwitchPositions(4.4)).toBe(4); // rounds
    expect(resolveDipSwitchPositions(4.6)).toBe(5); // rounds
    expect(resolveDipSwitchPositions(8)).toBe(8);
    expect(resolveDipSwitchPositions(20)).toBe(8); // above ceiling
  });

  it("defaults to 4 when positions is absent, matching the catalog default", () => {
    expect(resolveDipSwitchPositions(undefined)).toBe(4);
  });

  it("propagates NaN instead of silently clamping — Math.min/Math.max do not pick a side for NaN", () => {
    expect(Number.isNaN(resolveDipSwitchPositions("not-a-number"))).toBe(true);
    expect(Number.isNaN(resolveDipSwitchPositions(Number.NaN))).toBe(true);
  });

  it("a non-finite positions value is refused at compile time (malformed), not encoded as NaN on the wire", () => {
    const c = circuit([part("sw", "dip_switch", ["a1", "b1", "a2", "b2"], { positions: "garbage" })]);
    const result = compileCircuit(c, WITH_PASSIVES);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["malformed:sw:non-finite-positions"]);
  });
});

describe("resolveDipSwitchClosed", () => {
  it("Number(...) !== 0 counts as closed, including a NaN raw value (classic's own quirk)", () => {
    expect(resolveDipSwitchClosed(0)).toBe(0);
    expect(resolveDipSwitchClosed(1)).toBe(1);
    expect(resolveDipSwitchClosed(undefined)).toBe(0);
    expect(resolveDipSwitchClosed("not-a-number")).toBe(1); // NaN !== 0 is true
  });
});

describe("compileDipSwitch — pins and params", () => {
  it("always emits exactly 9 params ([k, sw1..sw8]), with extra sw_i beyond k still resolved (inert on the Rust side, per spec §7.6)", () => {
    const c = circuit([
      part("sw", "dip_switch", ["a1", "b1", "a2", "b2", "a3", "b3", "a4", "b4"], {
        positions: 4,
        sw1: 1,
        sw2: 0,
        sw5: 1, // beyond k=4 — inert, but still occupies a param slot
        sw8: 1,
      }),
    ]);
    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.dip_switch)!;
    expect(dev.params).toEqual([4, 1, 0, 0, 0, 1, 0, 0, 1]);
    expect(dev.pins).toHaveLength(8); // 2*k = 8
  });

  it("a pin id past k, or missing entirely, maps to node 0 and is reported open", () => {
    // Only a1/b1/a2/b2 declared; positions=4 means a3/b3/a4/b4 do not exist on this component.
    const c = circuit([part("sw", "dip_switch", ["a1", "b1", "a2", "b2"], { positions: 4, sw3: 1 })]);
    const result = compileCircuit(c, WITH_PASSIVES);
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    const decoded = decodeProgram(result.compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.dip_switch)!;
    expect(dev.pins).toHaveLength(8); // still 2*k = 8, k=4
    // a3/b3 are pins[4]/pins[5] (0-indexed pairs: [a1,b1,a2,b2,a3,b3,a4,b4]).
    expect(dev.pins[4]!.node).toBe(0);
    expect(dev.pins[4]!.open).toBe(true);
    expect(dev.pins[5]!.node).toBe(0);
    expect(dev.pins[5]!.open).toBe(true);
    // sw3 was still resolved (closed) even though a3/b3 don't physically exist.
    expect(dev.params[3]).toBe(1); // params = [k, sw1, sw2, sw3, ...]
  });

  it("k=2 minimum produces exactly 4 pins; k=8 maximum produces exactly 16", () => {
    const two = compileOk(circuit([part("sw", "dip_switch", ["a1", "b1", "a2", "b2"], { positions: 2 })]));
    const twoDecoded = decodeProgram(two.program).devices.find((d) => d.kindCode === CORE_KIND_CODE.dip_switch)!;
    expect(twoDecoded.pins).toHaveLength(4);

    const eightPins = Array.from({ length: 16 }, (_, k) => (k % 2 === 0 ? `a${k / 2 + 1}` : `b${(k - 1) / 2 + 1}`));
    const eight = compileOk(circuit([part("sw", "dip_switch", eightPins, { positions: 8 })]));
    const eightDecoded = decodeProgram(eight.program).devices.find((d) => d.kindCode === CORE_KIND_CODE.dip_switch)!;
    expect(eightDecoded.pins).toHaveLength(16);
  });

  it("cross-checked against a live classic run: sw3 closed on a 4-position switch conducts through a3/b3, and extra sw params beyond positions are inert", () => {
    const c = circuit([
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("sw", "dip_switch", ["a1", "b1", "a2", "b2", "a3", "b3", "a4", "b4"], {
        positions: 4,
        sw3: 1,
        sw5: 1, // inert: beyond positions
      }),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
    ], [
      wire("vs", "pos", "r1", "a"),
      wire("r1", "b", "sw", "a3"),
      wire("sw", "b3", "vs", "neg"),
    ]);

    const engine = runClassic(c);
    expect(Math.abs(engine.getElementI().sw ?? 0)).toBeGreaterThan(1e-3); // closed at position 3

    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.dip_switch)!;
    expect(dev.params).toEqual([4, 0, 0, 1, 0, 1, 0, 0, 0]); // sw5 resolved but has no pins to act on
  });
});

// ---- capacitor param resolution (prepared, not yet emitted) ----------------

describe("resolveCapacitorParams — classic's modelParam exact-uid rule (spec §7.2.6)", () => {
  it("capacitance defaults to 1e-6 and floors at 1e-15, matching devices/passives.ts's capacitorModel", () => {
    expect(resolveCapacitorParams({ kind: "capacitor", params: {} }).capacitance).toBe(1e-6);
    expect(resolveCapacitorParams({ kind: "capacitor", params: { capacitance: 0 } }).capacitance).toBe(1e-15);
    expect(resolveCapacitorParams({ kind: "capacitor", params: { capacitance: 4.7e-6 } }).capacitance).toBe(4.7e-6);
  });

  it("no catalogUid: ideal (esr 0, leakage 0 meaning classic's infinite parallel resistance)", () => {
    const resolved = resolveCapacitorParams({ kind: "capacitor", params: { capacitance: 1e-6 } });
    expect(resolved.esr).toBe(0);
    expect(resolved.leakage).toBe(0);
  });

  it("an authored esr/leakageResistance param always wins over the catalog default", () => {
    const resolved = resolveCapacitorParams({
      kind: "capacitor",
      catalogUid: "capacitor",
      params: { capacitance: 1e-6, esr: 3, leakageResistance: 42 },
    });
    expect(resolved.esr).toBe(3);
    expect(resolved.leakage).toBe(42);
  });

  it("exact catalog identity resolves esr/leakage per the README's own table", () => {
    const cases: Array<[string, number, number]> = [
      ["capacitor", 1, 1e7],
      ["cap-ceramic", 0.05, 1e9],
      ["cap-electrolytic", 0.2, 1e6],
      ["cap-film", 0.1, 1e9],
    ];
    for (const [catalogUid, esr, leakage] of cases) {
      const resolved = resolveCapacitorParams({ kind: "capacitor", catalogUid, params: { capacitance: 1e-6 } });
      expect(resolved.esr).toBeCloseTo(esr, 12);
      expect(resolved.leakage).toBeCloseTo(leakage, 3);
    }
  });

  it("a stale/kind-mismatched catalogUid falls back to ideal, not the wrong part's defaults", () => {
    const resolved = resolveCapacitorParams({
      kind: "capacitor",
      catalogUid: "not-a-real-uid",
      params: { capacitance: 1e-6 },
    });
    expect(resolved.esr).toBe(0);
    expect(resolved.leakage).toBe(0);
  });

  it("cross-checked against a live classic run on the exact catalogUid:\"capacitor\" fixture from passive-parasitics.test.ts", () => {
    const c = circuit([
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      { id: "c", kind: "capacitor", catalogUid: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1 } },
    ], [
      wire("vs", "pos", "c", "a"),
      wire("c", "b", "vs", "neg"),
    ]);
    const engine = new SimEngine();
    engine.load(c);
    engine.step(0.5);
    // Generic identified capacitor defaults: ESR=1 ohm, leakage=10 Mohm (passive-parasitics.test.ts).
    const expectedSeries = 5 / (1 + 0.5 / 1);
    expect(engine.getElementI().c).toBeCloseTo(expectedSeries + 5 / 10_000_000, 7);

    const resolved = resolveCapacitorParams(c.components[1]!);
    expect(resolved.esr).toBe(1);
    expect(resolved.leakage).toBe(10_000_000);
  });
});

// ---- capacitor: wire emission (compileCircuit / compileCapacitor) ----------
//
// The block above proves resolveCapacitorParams itself against classic's
// modelParam; this one proves the ACTUAL program record compileCircuit emits
// for "capacitor" — pins, and [C, esr, leakage] in the wire's param order —
// on the same passive-parasitics.test.ts catalogUid cases the README's own
// kind-codes table cites (catalogUid "capacitor", "cap-ceramic",
// "cap-electrolytic", "cap-film", and no catalogUid at all).

describe("compileCapacitor / compileCircuit — capacitor wire emission", () => {
  it("capacitor IS implemented in the default CORE_IMPLEMENTED_KINDS (kinds.ts; the orchestrator added it once this worker's compiler emission and R2's Rust device/solver both landed)", () => {
    const c = circuit([
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("c1", "capacitor", ["a", "b"], { capacitance: 1e-6 }),
    ]);
    const result = compileCircuit(c); // default implementedKinds, no override
    expect(result.ok).toBe(true);
  });

  it("emits positional pins and [C, esr, leakage] for an ideal (no catalogUid) capacitor", () => {
    const c = circuit([
      part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
      part("c1", "capacitor", ["a", "b"], { capacitance: 4.7e-6 }),
    ], [
      wire("vs", "pos", "r1", "a"),
      wire("r1", "b", "c1", "a"),
      wire("c1", "b", "vs", "neg"),
    ]);
    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.capacitor)!;
    expect(dev.params).toEqual([4.7e-6, 0, 0]); // ideal: esr 0, leakage 0 (meaning none)
    expect(dev.pins).toHaveLength(2);
    const built = buildCoreNodes(c);
    expect(dev.pins[0]!.node).toBe(built.pinNode.get("c1\u0000a"));
    expect(dev.pins[1]!.node).toBe(built.pinNode.get("c1\u0000b"));
  });

  it.each([
    ["capacitor", 1, 1e7],
    ["cap-ceramic", 0.05, 1e9],
    ["cap-electrolytic", 0.2, 1e6],
    ["cap-film", 0.1, 1e9],
  ] as const)("catalogUid %s emits [C, esr, leakage] = [C, %f, %f], per the README's own table", (catalogUid, esr, leakage) => {
    const c = circuit([
      { id: "c1", kind: "capacitor", catalogUid, pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 2.2e-6 } },
    ]);
    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.capacitor)!;
    expect(dev.params[0]).toBeCloseTo(2.2e-6, 15);
    expect(dev.params[1]).toBeCloseTo(esr, 12);
    expect(dev.params[2]).toBeCloseTo(leakage, 3);
  });

  it("an authored esr/leakageResistance param always wins over the catalog default, in the emitted wire record too", () => {
    const c = circuit([
      { id: "c1", kind: "capacitor", catalogUid: "capacitor", pins: [{ id: "a" }, { id: "b" }], params: { capacitance: 1e-6, esr: 3, leakageResistance: 42 } },
    ]);
    const compiled = compileOk(c);
    const decoded = decodeProgram(compiled.program);
    const dev = decoded.devices.find((d) => d.kindCode === CORE_KIND_CODE.capacitor)!;
    expect(dev.params).toEqual([1e-6, 3, 42]);
  });

  it("too few pins is malformed, not silently accepted (matches resistor/voltage_source's own rule)", () => {
    const c = circuit([part("c1", "capacitor", ["a"], { capacitance: 1e-6 })]);
    const result = compileCircuit(c, WITH_PASSIVES);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["malformed:c1:too-few-pins"]);
  });

  it("a non-finite capacitance (garbage string) is refused at compile time, routing to classic", () => {
    const c = circuit([part("c1", "capacitor", ["a", "b"], { capacitance: "not-a-number" })]);
    const result = compileCircuit(c, WITH_PASSIVES);
    expect(result.ok).toBe(false);
    if (!result.ok) expect(result.reasons).toEqual(["malformed:c1:non-finite-capacitance"]);
  });

  it("capacitance floors at 1e-15 and defaults to 1e-6, matching devices/passives.ts's capacitorModel, in the emitted wire record", () => {
    const zero = compileOk(circuit([part("c1", "capacitor", ["a", "b"], { capacitance: 0 })]));
    expect(decodeProgram(zero.program).devices[0]!.params[0]).toBe(1e-15);

    const absent = compileOk(circuit([part("c1", "capacitor", ["a", "b"], {})]));
    expect(decodeProgram(absent.program).devices[0]!.params[0]).toBe(1e-6);
  });
});

// ---- computeOpenPinKeys sanity for by-id switch pins ------------------------

describe("compile/passives.ts pin helpers agree with computeOpenPinKeys for a declared-but-unwired pin", () => {
  it("a declared spdt \"c\" pin with no wire at all is open, matching classic's _isOpenPin", () => {
    const c = circuit([part("sw", "spdt_switch", ["com", "b", "c"], { position: 0 })]);
    const built = buildCoreNodes(c);
    const openKeys = computeOpenPinKeys(c, built.nets);
    const result = compileSpdtSwitch(built, openKeys, "sw", c.components[0]!.pins, { position: 0 });
    expect(result.ok).toBe(true);
    if (result.ok) {
      // pins = [com, b, c]; "c" is declared but never wired to anything.
      expect(result.pins[2]!.open).toBe(true);
    }
  });
});
