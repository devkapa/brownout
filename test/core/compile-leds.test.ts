/**
 * Unit coverage for compile/leds.ts: the W1e LED and display family (`led`,
 * `rgb_led`, `bicolor_led`, `seg7_cc`, `seg7_ca` — core/README.md's "Kind
 * codes and records", codes 30-34) compiled to v3 program records.
 *
 * Every expected value comes from classic's own exported helpers
 * (`specsForComponent`, `thermalProfileForComponent`) or classic's own
 * runtime behaviour reached through a test-only cast (`_isOpenPin` —
 * compile.test.ts's own established convention), never re-derived. Catalog
 * identity is exercised against src/parts/default-parts.json's real `led`/
 * `rgb_led`/`bicolor_led`/`seg7_cc`/`seg7_ca` parts, plus injected catalogs
 * (compile-ics.test.ts's own `setPartLibrary`/`resetPartLibrary` convention)
 * where the bundled catalog can't show a precedence split on its own.
 *
 * These five kinds are not yet in CORE_IMPLEMENTED_KINDS (kinds.ts is the
 * orchestrator's table) — every `compileCircuit` call here passes a widened
 * `implementedKinds` set, exactly compile-passives.test.ts's own convention.
 */

import { afterEach, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { SimEngine } from "../../src/sim/engine/sim-engine.js";
import { compileCircuit } from "../../src/core/compile/index.js";
import {
  compileBicolorLed,
  compileLed,
  compileRgbLed,
  compileSeg7,
} from "../../src/core/compile/leds.js";
import { buildCoreNodes, computeOpenPinKeys } from "../../src/core/compile/topology.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import {
  CORE_BICOLOR_LED_PARAM,
  CORE_IMPLEMENTED_KINDS,
  CORE_LED_PARAM,
  CORE_RGB_LED_PARAM,
  CORE_SEG7_PARAM,
} from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS, resetPartLibrary, setPartLibrary } from "../../src/parts/part-library.js";
import type { PartDefinition } from "../../src/circuit/types.js";

// ---- small circuit-building helpers, local to this file (mirrors
// compile.test.ts / compile-ics.test.ts's own conventions — this worker owns
// no shared test-helper files) -----------------------------------------

type SimComponent = SimCircuit["components"][number];

function part(
  id: string,
  kind: string,
  pinIds: string[],
  params: Record<string, number | string> = {},
  catalogUid?: string,
): SimComponent {
  return { id, kind, catalogUid, pins: pinIds.map((pinId) => ({ id: pinId })), params };
}

function wire(fromComponent: string, fromPin: string, toComponent: string, toPin: string): SimCircuit["wires"][number] {
  return { from_component: fromComponent, from_pin: fromPin, to_component: toComponent, to_pin: toPin };
}

function circuit(components: SimComponent[], wires: SimCircuit["wires"] = []): SimCircuit {
  return { components, wires };
}

const LED_KINDS = ["led", "rgb_led", "bicolor_led", "seg7_cc", "seg7_ca"] as const;
const WITH_LEDS = new Set<string>([...CORE_IMPLEMENTED_KINDS, ...LED_KINDS]);

function compileOk(c: SimCircuit) {
  const result = compileCircuit(c, WITH_LEDS);
  if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
  return result.compiled;
}

function compileDeclined(c: SimCircuit): string[] {
  const result = compileCircuit(c, WITH_LEDS);
  if (result.ok) throw new Error("expected ok:false");
  return result.reasons;
}

function built(c: SimCircuit) {
  const b = buildCoreNodes(c);
  const openKeys = computeOpenPinKeys(c, b.nets);
  return { b, openKeys };
}

function classicIsOpenPin(engine: SimEngine, componentId: string, pinId: string): boolean {
  return (engine as unknown as { _isOpenPin(compId: string, pinId: string): boolean })._isOpenPin(componentId, pinId);
}

afterEach(() => resetPartLibrary());

// ─────────────────────────────────────────────────────────────────────────
// `led` (code 30): precedence
// ─────────────────────────────────────────────────────────────────────────

describe("compileLed — param precedence (core/README.md's LED and display records)", () => {
  it("vf: params over specs over the literal 1.8", () => {
    // No catalogUid (so the W1e thermal decline never fires): params.color
    // alone uniquely infers `led-blue` (vf 3) through classic's own legacy-
    // params catalog inference (catalog-resolver.ts) — the same specs
    // `specsForComponent` reads at runtime.
    const specsOnly = part("led1", "led", ["a", "k"], { color: "blue" });
    const { b, openKeys } = built(circuit([specsOnly]));
    const bySpecs = compileLed(b, openKeys, specsOnly);
    expect(bySpecs).toMatchObject({ ok: true, params: [3, 2, 0.02, 0, 0, 0] });

    const byParam = part("led2", "led", ["a", "k"], { color: "blue", vf: 2.5 });
    const b2 = built(circuit([byParam]));
    const withParam = compileLed(b2.b, b2.openKeys, byParam);
    expect(withParam).toMatchObject({ ok: true, params: [2.5, 2, 0.02, 0, 0, 0] });

    const literalOnly = part("led3", "led", ["a", "k"]);
    const b3 = built(circuit([literalOnly]));
    const literal = compileLed(b3.b, b3.openKeys, literalOnly);
    expect(literal).toMatchObject({ ok: true, params: [1.8, 2, 0.02, 0, 0, 0] });
  });

  it("iRated: params over specs' if_max over the literal 0.02", () => {
    // led-white's catalog if_max is 0.02 (same as the literal), so use an
    // injected catalog whose if_max actually differs to prove specs are
    // read at all, then prove an authored iRated still wins over it.
    const customLed: PartDefinition = {
      uid: "test-led-custom",
      kind: "led",
      default_params: {},
      pin_layout: [
        { id: "a", offset: { x: 0, y: 0 }, function: "anode" },
        { id: "k", offset: { x: 0, y: 0 }, function: "cathode" },
      ],
      electrical_specs: { if_max: 0.05 },
    };
    setPartLibrary([customLed]);

    const bySpecs = part("led1", "led", ["a", "k"]); // only-candidate inference, no catalogUid needed
    const b1 = built(circuit([bySpecs]));
    expect(compileLed(b1.b, b1.openKeys, bySpecs)).toMatchObject({ params: [1.8, 2, 0.05, 0, 0, 0] });

    const byParam = part("led2", "led", ["a", "k"], { iRated: 0.1 });
    const b2 = built(circuit([byParam]));
    expect(compileLed(b2.b, b2.openKeys, byParam)).toMatchObject({ params: [1.8, 2, 0.1, 0, 0, 0] });
  });

  it("n: defaults to 2, overridden by params.n", () => {
    const defaulted = part("led1", "led", ["a", "k"]);
    const b1 = built(circuit([defaulted]));
    expect(compileLed(b1.b, b1.openKeys, defaulted)).toMatchObject({ params: [1.8, 2, 0.02, 0, 0, 0] });

    const overridden = part("led2", "led", ["a", "k"], { n: 1 });
    const b2 = built(circuit([overridden]));
    expect(compileLed(b2.b, b2.openKeys, overridden)).toMatchObject({ params: [1.8, 1, 0.02, 0, 0, 0] });
  });

  it("Is/isSet: absent gives isSet 0, is 0; Is: 0 gives isSet 1, is 0; an authored Is is carried", () => {
    const absent = part("led1", "led", ["a", "k"]);
    const b1 = built(circuit([absent]));
    expect(compileLed(b1.b, b1.openKeys, absent)).toMatchObject({ params: [1.8, 2, 0.02, 0, 0, 0] });

    const zero = part("led2", "led", ["a", "k"], { Is: 0 });
    const b2 = built(circuit([zero]));
    expect(compileLed(b2.b, b2.openKeys, zero)).toMatchObject({ params: [1.8, 2, 0.02, 1, 0, 0] });

    const authored = part("led3", "led", ["a", "k"], { Is: 5e-9 });
    const b3 = built(circuit([authored]));
    expect(compileLed(b3.b, b3.openKeys, authored)).toMatchObject({ params: [1.8, 2, 0.02, 1, 5e-9, 0] });
  });

  it("burnt: JS truthiness — 1, true and the string \"0\" are burnt; 0 and absent are not", () => {
    for (const [value, expected] of [
      [1, 1],
      [true as unknown as number, 1],
      ["0" as unknown as number, 1],
      [0, 0],
    ] as const) {
      const comp = part("led1", "led", ["a", "k"], { burnt: value });
      const b = built(circuit([comp]));
      expect(compileLed(b.b, b.openKeys, comp)).toMatchObject({ params: [1.8, 2, 0.02, 0, 0, expected] });
    }
    const absent = part("led2", "led", ["a", "k"]);
    const b = built(circuit([absent]));
    expect(compileLed(b.b, b.openKeys, absent)).toMatchObject({ params: [1.8, 2, 0.02, 0, 0, 0] });
  });
});

// ─────────────────────────────────────────────────────────────────────────
// `led` (code 30): declines
// ─────────────────────────────────────────────────────────────────────────

describe("compileLed — declines", () => {
  it("too-few-pins: a 1-pin (or 0-pin) led", () => {
    for (const pins of [["a"], []]) {
      const comp = part("led1", "led", pins);
      const b = built(circuit([comp]));
      expect(compileLed(b.b, b.openKeys, comp)).toEqual({ ok: false, reason: "malformed:led1:too-few-pins" });
    }
  });

  it("non-finite-vf: a garbage vf string", () => {
    const comp = part("led1", "led", ["a", "k"], { vf: "banana" });
    const b = built(circuit([comp]));
    expect(compileLed(b.b, b.openKeys, comp)).toEqual({ ok: false, reason: "malformed:led1:non-finite-vf" });
  });

  it("non-positive-n: n <= 0", () => {
    for (const n of [0, -5]) {
      const comp = part("led1", "led", ["a", "k"], { n });
      const b = built(circuit([comp]));
      expect(compileLed(b.b, b.openKeys, comp)).toEqual({ ok: false, reason: "malformed:led1:non-positive-n" });
    }
  });

  it("non-finite-is: an authored Is that resolves to NaN", () => {
    const comp = part("led1", "led", ["a", "k"], { Is: "banana" });
    const b = built(circuit([comp]));
    expect(compileLed(b.b, b.openKeys, comp)).toEqual({ ok: false, reason: "malformed:led1:non-finite-is" });
  });

  it("unsupported-feature:led-thermal — a led whose catalogUid classic's own _thermalProfile accepts; a control with no uid still compiles", () => {
    const identified = part("led1", "led", ["a", "k"], {}, "led-red");
    const b1 = built(circuit([identified]));
    expect(compileLed(b1.b, b1.openKeys, identified)).toEqual({
      ok: false,
      reason: "unsupported-feature:led-thermal:led1",
    });

    // The control: an otherwise identical led with NO catalogUid compiles
    // fine — the decline is on the component's own EXPLICIT catalogUid
    // (thermalProfileForComponent's own gate), not on whether specs happen
    // to resolve.
    const control = part("led2", "led", ["a", "k"]);
    const b2 = built(circuit([control]));
    expect(compileLed(b2.b, b2.openKeys, control)).toMatchObject({ ok: true });
  });

  it("every catalog led part (default-parts.json) is declined once its catalogUid is authored explicitly", () => {
    for (const p of BUNDLED_DEFAULT_PARTS.filter((p) => p.kind === "led")) {
      const comp = part("led1", "led", ["a", "k"], {}, p.uid);
      const b = built(circuit([comp]));
      expect(compileLed(b.b, b.openKeys, comp)).toEqual({
        ok: false,
        reason: "unsupported-feature:led-thermal:led1",
      });
    }
  });
});

// ─────────────────────────────────────────────────────────────────────────
// `rgb_led` / `bicolor_led` (codes 31-32): the specs-blind/specs-aware split
// ─────────────────────────────────────────────────────────────────────────

describe("compileRgbLed / compileBicolorLed — vf/iRated ignore specs, ifMax does not (Q-O24)", () => {
  it("rgb_led: vf_r/vf_g/vf_b/iRated ignore an injected catalog; ifMax reads its if_max", () => {
    const customRgb: PartDefinition = {
      uid: "test-rgb-custom",
      kind: "rgb_led",
      default_params: {},
      pin_layout: [
        { id: "r_a", offset: { x: 0, y: 0 } },
        { id: "g_a", offset: { x: 0, y: 0 } },
        { id: "b_a", offset: { x: 0, y: 0 } },
        { id: "com_k", offset: { x: 0, y: 0 } },
      ],
      electrical_specs: { if_max: 0.03 },
    };
    setPartLibrary([customRgb]);

    const comp = part("u1", "rgb_led", ["r_a", "g_a", "b_a", "com_k"]);
    const b = built(circuit([comp]));
    const result = compileRgbLed(b.b, b.openKeys, comp);
    expect(result).toMatchObject({ ok: true, params: [1.8, 2.0, 3.0, 0.02, 0.03, 0] });
    // The split the README calls out explicitly: iRated (specs-blind, 0.02)
    // differs from ifMax (specs-aware, 0.03) whenever iRated is unauthored.
    if (result.ok) {
      expect(result.params[CORE_RGB_LED_PARAM.iRated]).not.toBe(result.params[CORE_RGB_LED_PARAM.ifMax]);
    }

    // An authored iRated moves BOTH iRated and ifMax together (the same
    // expression), so the damage limit tracks a raised current rating.
    const withIRated = part("u2", "rgb_led", ["r_a", "g_a", "b_a", "com_k"], { iRated: 0.05 });
    const b2 = built(circuit([withIRated]));
    expect(compileRgbLed(b2.b, b2.openKeys, withIRated)).toMatchObject({
      params: [1.8, 2.0, 3.0, 0.05, 0.05, 0],
    });
  });

  it("bicolor_led: vf1/vf2/iRated ignore an injected catalog; ifMax reads its if_max", () => {
    const customBicolor: PartDefinition = {
      uid: "test-bicolor-custom",
      kind: "bicolor_led",
      default_params: {},
      pin_layout: [
        { id: "a1", offset: { x: 0, y: 0 } },
        { id: "a2", offset: { x: 0, y: 0 } },
        { id: "k", offset: { x: 0, y: 0 } },
      ],
      electrical_specs: { if_max: 0.03 },
    };
    setPartLibrary([customBicolor]);

    const comp = part("u1", "bicolor_led", ["a1", "a2", "k"]);
    const b = built(circuit([comp]));
    const result = compileBicolorLed(b.b, b.openKeys, comp);
    expect(result).toMatchObject({ ok: true, params: [1.8, 2.0, 0.02, 0.03, 0] });
    if (result.ok) {
      expect(result.params[CORE_BICOLOR_LED_PARAM.iRated]).not.toBe(result.params[CORE_BICOLOR_LED_PARAM.ifMax]);
    }
  });

  it("rgb_led/bicolor_led have no thermal decline: an explicit catalogUid still compiles (LEDs point 3: multi-junction kinds are ambient-only forever)", () => {
    const rgb = part("u1", "rgb_led", ["r_a", "g_a", "b_a", "com_k"], {}, "led-rgb");
    const b1 = built(circuit([rgb]));
    expect(compileRgbLed(b1.b, b1.openKeys, rgb)).toMatchObject({ ok: true });

    const bicolor = part("u2", "bicolor_led", ["a1", "a2", "k"], {}, "led-bicolor");
    const b2 = built(circuit([bicolor]));
    expect(compileBicolorLed(b2.b, b2.openKeys, bicolor)).toMatchObject({ ok: true });
  });

  it("too-few-pins: rgb_led needs 4 declared pins, bicolor_led needs 3", () => {
    const rgb = part("u1", "rgb_led", ["r_a", "g_a", "b_a"]);
    const b1 = built(circuit([rgb]));
    expect(compileRgbLed(b1.b, b1.openKeys, rgb)).toEqual({ ok: false, reason: "malformed:u1:too-few-pins" });

    const bicolor = part("u2", "bicolor_led", ["a1", "a2"]);
    const b2 = built(circuit([bicolor]));
    expect(compileBicolorLed(b2.b, b2.openKeys, bicolor)).toEqual({ ok: false, reason: "malformed:u2:too-few-pins" });
  });

  it("burnt: JS truthiness, same rule as led", () => {
    const rgb = part("u1", "rgb_led", ["r_a", "g_a", "b_a", "com_k"], { burnt: "0" });
    const b1 = built(circuit([rgb]));
    const result = compileRgbLed(b1.b, b1.openKeys, rgb);
    expect(result).toMatchObject({ ok: true });
    if (result.ok) expect(result.params[CORE_RGB_LED_PARAM.burnt]).toBe(1);
  });
});

// ─────────────────────────────────────────────────────────────────────────
// `seg7_cc` / `seg7_ca` (codes 33-34): specs-aware, no burnt/ifMax, no pin gate
// ─────────────────────────────────────────────────────────────────────────

describe("compileSeg7 — vf/iRated from specs, no pin-count gate", () => {
  it("reads vf/iRated from an injected catalog when params lack them", () => {
    const customSeg7: PartDefinition = {
      uid: "test-seg7-custom",
      kind: "seg7_cc",
      default_params: {},
      pin_layout: ["a", "b", "c", "d", "e", "f", "g", "dp", "com", "com2"].map((id) => ({
        id,
        offset: { x: 0, y: 0 },
      })),
      electrical_specs: { vf: 2.5, if_max: 0.03 },
    };
    setPartLibrary([customSeg7]);

    const comp = part("u1", "seg7_cc", ["a", "b", "c", "d", "e", "f", "g", "dp", "com", "com2"]);
    const b = built(circuit([comp]));
    expect(compileSeg7(b.b, b.openKeys, comp)).toMatchObject({ ok: true, params: [2.5, 0.03] });
  });

  it("falls back to the literal 2.0/0.02 with no specs and no params", () => {
    const comp = part("u1", "seg7_ca", ["a", "b", "c", "d", "e", "f", "g", "dp", "com", "com2"]);
    const b = built(circuit([comp]));
    expect(compileSeg7(b.b, b.openKeys, comp)).toMatchObject({ ok: true, params: [2.0, 0.02] });
  });

  it("params still win over specs", () => {
    const customSeg7: PartDefinition = {
      uid: "test-seg7-custom2",
      kind: "seg7_cc",
      default_params: {},
      pin_layout: ["a", "b", "c", "d", "e", "f", "g", "dp", "com", "com2"].map((id) => ({
        id,
        offset: { x: 0, y: 0 },
      })),
      electrical_specs: { vf: 2.5, if_max: 0.03 },
    };
    setPartLibrary([customSeg7]);
    const comp = part("u1", "seg7_cc", ["a", "b", "c", "d", "e", "f", "g", "dp", "com", "com2"], {
      vf: 1.9,
      iRated: 0.04,
    });
    const b = built(circuit([comp]));
    expect(compileSeg7(b.b, b.openKeys, comp)).toMatchObject({ ok: true, params: [1.9, 0.04] });
  });

  it("has no pin-count gate: even with zero declared pins it compiles (every pin defaults to node 0, OPEN)", () => {
    const comp = part("u1", "seg7_cc", []);
    const b = built(circuit([comp]));
    const result = compileSeg7(b.b, b.openKeys, comp);
    expect(result).toMatchObject({ ok: true });
    if (result.ok) {
      expect(result.pins).toHaveLength(10);
      for (const p of result.pins) {
        expect(p.node).toBe(0);
        expect(p.open).toBe(true);
      }
    }
  });
});

// ─────────────────────────────────────────────────────────────────────────
// Pins: absent-by-id defaults, the seg7 com/com2 tie, OPEN vs classic
// ─────────────────────────────────────────────────────────────────────────

describe("pins", () => {
  it("bicolor_led: an absent k maps to node 0, OPEN", () => {
    // 3 declared pins (clears the too-few-pins gate), but none of them is
    // actually named "k" — the by-id lookup for "k" then falls back to node
    // 0, OPEN, exactly as the README's "Pins by id" rule requires.
    const comp = part("u1", "bicolor_led", ["a1", "a2", "bogus"]);
    const b = built(circuit([comp]));
    const result = compileBicolorLed(b.b, b.openKeys, comp);
    expect(result).toMatchObject({ ok: true });
    if (result.ok) {
      const kPin = result.pins[2]!; // a1, a2, k
      expect(kPin.node).toBe(0);
      expect(kPin.open).toBe(true);
    }
  });

  it("seg7_cc: com and com2 share one node and one OPEN status, matching classic's own net union", () => {
    const comp = part("u1", "seg7_cc", ["a", "b", "c", "d", "e", "f", "g", "dp", "com", "com2"]);
    const vsrc = part("gnd_src", "voltage_source", ["pos", "neg"], { voltage: 5 });
    const c = circuit([vsrc, comp], [wire("gnd_src", "neg", "u1", "com")]); // only "com" wired
    const { b, openKeys } = built(c);
    const result = compileSeg7(b, openKeys, comp);
    expect(result).toMatchObject({ ok: true });
    if (!result.ok) return;
    const comPin = result.pins[8]!; // a b c d e f g dp com com2
    const com2Pin = result.pins[9]!;
    expect(comPin.node).toBe(com2Pin.node);
    expect(comPin.open).toBe(false); // wired
    expect(com2Pin.open).toBe(false); // wiring EITHER lead makes both non-open (G8)

    const engine = new SimEngine();
    engine.load(c);
    expect(comPin.open).toBe(classicIsOpenPin(engine, "u1", "com"));
    expect(com2Pin.open).toBe(classicIsOpenPin(engine, "u1", "com2"));
  });

  it("OPEN flags on every rgb_led pin match classic's own _isOpenPin", () => {
    const rSrc = part("r1", "resistor", ["a", "b"], { resistance: 150 });
    const vsrc = part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 });
    const rgb = part("u1", "rgb_led", ["r_a", "g_a", "b_a", "com_k"]); // g_a, b_a left floating
    const c = circuit(
      [vsrc, rSrc, rgb],
      [wire("vsrc", "pos", "r1", "a"), wire("r1", "b", "u1", "r_a"), wire("u1", "com_k", "vsrc", "neg")],
    );
    const { b, openKeys } = built(c);
    const result = compileRgbLed(b, openKeys, rgb);
    expect(result).toMatchObject({ ok: true });
    if (!result.ok) return;

    const engine = new SimEngine();
    engine.load(c);
    const pinIds = ["r_a", "g_a", "b_a", "com_k"];
    result.pins.forEach((pinRecord, i) => {
      expect(pinRecord.open).toBe(classicIsOpenPin(engine, "u1", pinIds[i]!));
    });
    // Sanity: the floating channels really are open, the wired ones are not.
    expect(result.pins[0]!.open).toBe(false); // r_a, wired
    expect(result.pins[1]!.open).toBe(true); // g_a, floating
    expect(result.pins[2]!.open).toBe(true); // b_a, floating
    expect(result.pins[3]!.open).toBe(false); // com_k, wired
  });
});

// ─────────────────────────────────────────────────────────────────────────
// Channel slot table (compile/index.ts's allocation, through compileCircuit)
// ─────────────────────────────────────────────────────────────────────────

describe("channel slot table", () => {
  it("assigns consecutive channel slots in component order: rgb_led (3), bicolor_led (2), seg7_cc (8)", () => {
    const c = circuit([
      part("vsrc", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("u1", "rgb_led", ["r_a", "g_a", "b_a", "com_k"]),
      part("u2", "bicolor_led", ["a1", "a2", "k"]),
      part("u3", "seg7_cc", ["a", "b", "c", "d", "e", "f", "g", "dp", "com", "com2"]),
    ]);
    const compiled = compileOk(c);
    expect(compiled.channelSlots).toEqual([
      ["u1", 0], ["u1", 1], ["u1", 2],
      ["u2", 0], ["u2", 1],
      ["u3", 0], ["u3", 1], ["u3", 2], ["u3", 3], ["u3", 4], ["u3", 5], ["u3", 6], ["u3", 7],
    ]);

    const decoded = decodeProgram(compiled.program);
    expect(decoded.channelSlotCount).toBe(13);
    const u1 = decoded.devices.find((d) => d.componentIndex === 1)!;
    const u2 = decoded.devices.find((d) => d.componentIndex === 2)!;
    const u3 = decoded.devices.find((d) => d.componentIndex === 3)!;
    expect(u1.blob).toEqual([0]);
    expect(u2.blob).toEqual([3]);
    expect(u3.blob).toEqual([5]);
  });

  it("led claims no channel slots (an empty blob)", () => {
    const c = circuit([part("u1", "led", ["a", "k"])]);
    const compiled = compileOk(c);
    expect(compiled.channelSlots).toEqual([]);
    const decoded = decodeProgram(compiled.program);
    expect(decoded.devices[0]!.blob).toEqual([]);
  });
});

// ─────────────────────────────────────────────────────────────────────────
// The program round trip through decodeProgram, every new kind
// ─────────────────────────────────────────────────────────────────────────

describe("program round trip through decodeProgram", () => {
  it("led: 2 pins, 6 params, empty blob", () => {
    const c = circuit([part("u1", "led", ["a", "k"])]);
    const compiled = compileOk(c);
    expect(() => decodeProgram(compiled.program)).not.toThrow();
    const dev = decodeProgram(compiled.program).devices[0]!;
    expect(dev.kindCode).toBe(CORE_KIND_CODE.led);
    expect(dev.pins).toHaveLength(2);
    expect(dev.params).toHaveLength(6);
    expect(dev.blob).toHaveLength(0);
  });

  it("rgb_led: 4 pins, 6 params, blob [channelBase]", () => {
    const c = circuit([part("u1", "rgb_led", ["r_a", "g_a", "b_a", "com_k"])]);
    const compiled = compileOk(c);
    expect(() => decodeProgram(compiled.program)).not.toThrow();
    const dev = decodeProgram(compiled.program).devices[0]!;
    expect(dev.kindCode).toBe(CORE_KIND_CODE.rgb_led);
    expect(dev.pins).toHaveLength(4);
    expect(dev.params).toHaveLength(6);
    expect(dev.blob).toEqual([0]);
  });

  it("bicolor_led: 3 pins, 5 params, blob [channelBase]", () => {
    const c = circuit([part("u1", "bicolor_led", ["a1", "a2", "k"])]);
    const compiled = compileOk(c);
    expect(() => decodeProgram(compiled.program)).not.toThrow();
    const dev = decodeProgram(compiled.program).devices[0]!;
    expect(dev.kindCode).toBe(CORE_KIND_CODE.bicolor_led);
    expect(dev.pins).toHaveLength(3);
    expect(dev.params).toHaveLength(5);
    expect(dev.blob).toEqual([0]);
  });

  it("seg7_cc / seg7_ca: 10 pins, 2 params, blob [channelBase]", () => {
    for (const kind of ["seg7_cc", "seg7_ca"] as const) {
      const c = circuit([part("u1", kind, ["a", "b", "c", "d", "e", "f", "g", "dp", "com", "com2"])]);
      const compiled = compileOk(c);
      expect(() => decodeProgram(compiled.program)).not.toThrow();
      const dev = decodeProgram(compiled.program).devices[0]!;
      expect(dev.kindCode).toBe(CORE_KIND_CODE[kind]);
      expect(dev.pins).toHaveLength(10);
      expect(dev.params).toHaveLength(2);
      expect(dev.blob).toEqual([0]);
    }
  });
});

// ---- CORE_LED_PARAM/CORE_RGB_LED_PARAM/CORE_BICOLOR_LED_PARAM/CORE_SEG7_PARAM
// index sanity: these tests index params by name via the shared kinds.ts
// tables, so a drift between the table and this file's own literal arrays
// would silently mis-check the wrong index. -------------------------------

describe("CORE_LED_PARAM et al. — index sanity", () => {
  it("matches core/README.md's per-kind param tables", () => {
    expect(CORE_LED_PARAM).toEqual({ vf: 0, n: 1, iRated: 2, isSet: 3, is: 4, burnt: 5 });
    expect(CORE_RGB_LED_PARAM).toEqual({ vfR: 0, vfG: 1, vfB: 2, iRated: 3, ifMax: 4, burnt: 5 });
    expect(CORE_BICOLOR_LED_PARAM).toEqual({ vf1: 0, vf2: 1, iRated: 2, ifMax: 3, burnt: 4 });
    expect(CORE_SEG7_PARAM).toEqual({ vf: 0, iRated: 1 });
  });
});
