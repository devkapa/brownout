/**
 * Unit coverage for compile/sources_w1h.ts: the `signal_gen` record builder (code 27, W1h item H3a).
 *
 * The kind is NOT in CORE_IMPLEMENTED_KINDS (nothing drives it until H3b), so every compile here widens the gate
 * by hand. Expected values come from classic's own `parseSignalGenParams` / `signalGenEnabled`, never restated:
 * the record is "classic's parse, clamps applied" (core/spec §8.12.1), so a hand-written default here would be a
 * second copy that could drift from the first.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { parseSignalGenParams, signalGenEnabled } from "../../src/sim/engine/waveform.js";
import { compileCircuit, LIVE_PARAM_SPECS } from "../../src/core/compile/index.js";
import { encodeSignalGen, SIGNAL_GEN_LIVE_PARAM_SPECS } from "../../src/core/compile/sources_w1h.js";
import { CORE_KIND_CODE, decodeProgram } from "../../src/core/format.js";
import { CORE_IMPLEMENTED_KINDS, CORE_SIGNAL_GEN_PARAM, CORE_SIGNAL_GEN_SHAPE } from "../../src/core/kinds.js";
import { __resetCoreForTest, coreCall, initCore } from "../../src/core/wasm.js";

type Params = Record<string, number | string>;

const WITH_SIGNAL_GEN = new Set<string>([...CORE_IMPLEMENTED_KINDS, "signal_gen"]);

function circuitOf(params: Params, pins: string[] = ["pos", "neg"]): SimCircuit {
  return {
    components: [
      { id: "g1", kind: "signal_gen", pins: pins.map((id) => ({ id })), params },
      { id: "r1", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 1000 } },
      { id: "gnd", kind: "ground", pins: [{ id: "gnd" }], params: {} },
    ],
    wires: [
      { from_component: "g1", from_pin: "pos", to_component: "r1", to_pin: "a" },
      { from_component: "r1", from_pin: "b", to_component: "gnd", to_pin: "gnd" },
      { from_component: "g1", from_pin: "neg", to_component: "gnd", to_pin: "gnd" },
    ],
  };
}

function compileRecord(params: Params) {
  const result = compileCircuit(circuitOf(params), WITH_SIGNAL_GEN);
  if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
  const dev = decodeProgram(result.compiled.program).devices.find((d) => d.kindCode === CORE_KIND_CODE.signal_gen);
  if (!dev) throw new Error("no signal_gen record");
  return dev;
}

function reasonsOf(params: Params, pins?: string[]): string[] {
  const result = compileCircuit(circuitOf(params, pins), WITH_SIGNAL_GEN);
  if (result.ok) throw new Error("expected a decline");
  return [...result.reasons];
}

/** The expected param vector, built from classic's parse alone. */
function expectedParams(raw: Params): number[] {
  const p = parseSignalGenParams(raw);
  return [
    CORE_SIGNAL_GEN_SHAPE[p.waveform],
    p.amplitude,
    p.offset,
    p.frequency,
    p.phaseDeg,
    p.duty,
    p.delay,
    p.tr,
    p.tf,
    p.pw,
    p.seed,
    signalGenEnabled(raw) ? 1 : 0,
    Number(raw.rSource ?? 50),
  ];
}

describe("signal_gen record", () => {
  it("is one 2-pin record of code 27 with 13 params in the kinds.ts order", () => {
    const dev = compileRecord({});
    expect(dev.pins).toHaveLength(2);
    expect(dev.params).toHaveLength(13);
    expect(Object.keys(CORE_SIGNAL_GEN_PARAM)).toHaveLength(13);
  });

  it("with no params carries classic's defaults (sine, 2.5 V, 2.5 V offset, 1 kHz, 50 ohm source)", () => {
    const dev = compileRecord({});
    expect(dev.params).toEqual(expectedParams({}));
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.waveform]).toBe(CORE_SIGNAL_GEN_SHAPE.sine);
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.rSource]).toBe(50);
    expect(dev.blob).toEqual([0]);
  });

  it.each(Object.keys(CORE_SIGNAL_GEN_SHAPE))("encodes %s by classic's name, with the clamps applied", (waveform) => {
    const raw: Params = {
      waveform,
      amplitude: -3,
      offset: -1.5,
      frequency: 0,
      phaseDeg: -450,
      duty: 2,
      delay: -1,
      tr: 0,
      tf: -5,
      pw: 0,
      seed: 2.5,
      rSource: 75,
    };
    const dev = compileRecord(raw);
    expect(dev.params).toEqual(expectedParams(raw));
    // The clamps are visible in the record: amplitude 0, frequency 1e-6, duty 0.99, delay 0, floors 1e-9, seed 3.
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.amplitude]).toBe(0);
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.frequency]).toBe(1e-6);
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.duty]).toBe(0.99);
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.delay]).toBe(0);
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.tr]).toBe(1e-9);
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.tf]).toBe(1e-9);
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.pw]).toBe(1e-9);
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.seed]).toBe(3);
  });

  it("plays an unknown or non-string waveform as a sine, as classic's parse does", () => {
    for (const waveform of ["SINE", "triangel", "", 3]) {
      const dev = compileRecord({ waveform });
      expect(dev.params[CORE_SIGNAL_GEN_PARAM.waveform], String(waveform)).toBe(CORE_SIGNAL_GEN_SHAPE.sine);
    }
  });

  it("encodes enabled as classic reads it: only an explicit 0 disables, NaN counts as enabled", () => {
    expect(compileRecord({ enabled: 0 }).params[CORE_SIGNAL_GEN_PARAM.enabled]).toBe(0);
    expect(compileRecord({ enabled: 1 }).params[CORE_SIGNAL_GEN_PARAM.enabled]).toBe(1);
    expect(compileRecord({ enabled: 7 }).params[CORE_SIGNAL_GEN_PARAM.enabled]).toBe(1);
    expect(compileRecord({ enabled: Number.NaN }).params[CORE_SIGNAL_GEN_PARAM.enabled]).toBe(1);
  });

  it("keeps rSource raw (a negative value is the ideal form, read at each hook)", () => {
    expect(compileRecord({ rSource: 0 }).params[CORE_SIGNAL_GEN_PARAM.rSource]).toBe(0);
    expect(compileRecord({ rSource: -4 }).params[CORE_SIGNAL_GEN_PARAM.rSource]).toBe(-4);
    expect(compileRecord({ rSource: "12" }).params[CORE_SIGNAL_GEN_PARAM.rSource]).toBe(12);
  });
});

describe("signal_gen PWL blob", () => {
  it("is classic's parsed table: parseFloat pairs, sorted by time (stably), [n, t0, v0, ...]", () => {
    const pwl = "2e-3:1, 1e-3:5, 1e-3:3, junk, 4e-3:abc, 3e-3:2.5xyz";
    const dev = compileRecord({ waveform: "pwl", pwl });
    const pts = parseSignalGenParams({ waveform: "pwl", pwl }).pwlPoints;
    expect(pts.length).toBe(4);
    expect(dev.blob).toEqual([pts.length, ...pts.flatMap((p) => [p.t, p.v])]);
    // Equal times keep their authored order (5 V before 3 V at 1 ms).
    expect(dev.blob.slice(0, 5)).toEqual([4, 1e-3, 5, 1e-3, 3]);
  });

  it("keeps the first 64 points after sorting", () => {
    const pwl = Array.from({ length: 70 }, (_, i) => `${(69 - i) * 1e-3}:${i}`).join(",");
    const dev = compileRecord({ waveform: "pwl", pwl });
    expect(dev.blob[0]).toBe(64);
    expect(dev.blob).toHaveLength(1 + 2 * 64);
    expect(dev.blob[1]).toBe(0);
    expect(dev.blob[dev.blob.length - 2]).toBe(63e-3);
  });

  it("is an empty table for an unparseable string, and [0] for every shape but pwl", () => {
    expect(compileRecord({ waveform: "pwl", pwl: "nonsense" }).blob).toEqual([0]);
    expect(compileRecord({ waveform: "pwl" }).blob).toEqual([0]);
    expect(compileRecord({ waveform: "square", pwl: "0:1,1:2" }).blob).toEqual([0]);
  });
});

describe("signal_gen declines", () => {
  const names = Object.keys(CORE_SIGNAL_GEN_PARAM).filter((n) => n !== "waveform" && n !== "enabled");

  it.each(names)("declines a NaN %s as non-finite-%s", (name) => {
    expect(reasonsOf({ [name]: Number.NaN })).toEqual([`malformed:g1:non-finite-${name}`]);
  });

  it.each(["amplitude", "offset", "frequency", "phaseDeg", "delay", "tr", "pw", "rSource", "seed"])(
    "declines an infinite %s as non-finite-%s",
    (name) => {
      expect(reasonsOf({ [name]: Number.POSITIVE_INFINITY })).toEqual([`malformed:g1:non-finite-${name}`]);
    },
  );

  it("declines a non-numeric string like classic's Number() reads it (NaN)", () => {
    expect(reasonsOf({ amplitude: "loud" })).toEqual(["malformed:g1:non-finite-amplitude"]);
  });

  it("declines a seed too large for String() to print in decimal", () => {
    expect(reasonsOf({ waveform: "noise", seed: 1e21 })).toEqual(["malformed:g1:seed-out-of-range"]);
    expect(compileRecord({ waveform: "noise", seed: 1e20 }).params[CORE_SIGNAL_GEN_PARAM.seed]).toBe(1e20);
  });

  it("declines a pulse cycle cut short (tr + pw + tf > 1/f), Q-O72", () => {
    // 1 kHz, pw 2 ms: classic never reaches the fall (probe P1c).
    expect(reasonsOf({ waveform: "pulse", frequency: 1000, pw: 2e-3 })).toEqual(["malformed:g1:truncated-pulse"]);
    expect(reasonsOf({ waveform: "pulse", frequency: 1000, tr: 4e-4, pw: 4e-4, tf: 4e-4 })).toEqual([
      "malformed:g1:truncated-pulse",
    ]);
  });

  it("allows a cycle that exactly fills the period, summed (tr + pw) + tf as classic does", () => {
    // 0.1 + 0.2 + 0.3 is 0.6000000000000001 left to right and 0.6 the other way; a 0.6 s period must decline.
    expect(reasonsOf({ waveform: "pulse", frequency: 1 / 0.6, tr: 0.1, pw: 0.2, tf: 0.3 })).toEqual([
      "malformed:g1:truncated-pulse",
    ]);
    const dev = compileRecord({ waveform: "pulse", frequency: 1000, tr: 2.5e-4, pw: 5e-4, tf: 2.5e-4 });
    expect(dev.params[CORE_SIGNAL_GEN_PARAM.waveform]).toBe(CORE_SIGNAL_GEN_SHAPE.pulse);
  });

  it("applies the truncation rule to a pulse only", () => {
    for (const waveform of ["dc", "sine", "square", "triangle", "ramp", "noise"]) {
      const dev = compileRecord({ waveform, frequency: 1000, pw: 2e-3 });
      expect(dev.params[CORE_SIGNAL_GEN_PARAM.waveform], waveform).toBe(CORE_SIGNAL_GEN_SHAPE[waveform as keyof typeof CORE_SIGNAL_GEN_SHAPE]);
    }
  });

  it("declines fewer than two pins", () => {
    expect(reasonsOf({}, ["pos"])).toEqual(["malformed:g1:too-few-pins"]);
  });

  it("encodeSignalGen returns the same decline for a pure caller", () => {
    expect(encodeSignalGen("g1", { amplitude: Number.NaN })).toEqual({ ok: false, reason: "malformed:g1:non-finite-amplitude" });
    const ok = encodeSignalGen("g1", {});
    expect(ok.ok).toBe(true);
  });
});

describe("signal_gen declines the records the reader would refuse (verifier fixes)", () => {
  it("declines a delay whose ulp nears the period, for the shapes with a schedule", () => {
    // ulp(delay) <= period * 2^-20: a 1 kHz generator may start about 4e6 s in; noise (1e-4 s buckets) about 5e5 s.
    for (const waveform of ["square", "triangle", "ramp", "pulse"]) {
      expect(reasonsOf({ waveform, delay: 1e13 }), waveform).toEqual(["malformed:g1:delay-too-late"]);
      expect(compileRecord({ waveform, delay: 1e6 }).params[CORE_SIGNAL_GEN_PARAM.delay], waveform).toBe(1e6);
    }
    expect(reasonsOf({ waveform: "noise", delay: 1e6 })).toEqual(["malformed:g1:delay-too-late"]);
    expect(compileRecord({ waveform: "noise", delay: 3e5 }).params[CORE_SIGNAL_GEN_PARAM.delay]).toBe(3e5);
    // A slow generator tolerates a later one; a sine, dc and a table have no lattice to order.
    expect(compileRecord({ waveform: "square", frequency: 1e-3, delay: 1e12 }).params[CORE_SIGNAL_GEN_PARAM.delay]).toBe(1e12);
    for (const waveform of ["dc", "sine", "pwl"]) expect(compileRecord({ waveform, delay: 1e13 }).params[CORE_SIGNAL_GEN_PARAM.delay]).toBe(1e13);
  });

  it("declines levels the schedule's lines and sums could not hold finite", () => {
    for (const waveform of ["sine", "square", "triangle", "ramp", "pulse", "noise"]) {
      expect(reasonsOf({ waveform, offset: 1e308, amplitude: 1e308 }), waveform).toEqual(["malformed:g1:level-out-of-range"]);
      expect(reasonsOf({ waveform, amplitude: 1e150 }), waveform).toEqual(["malformed:g1:level-out-of-range"]);
    }
    expect(reasonsOf({ waveform: "dc", offset: 1e308 })).toEqual(["malformed:g1:level-out-of-range"]);
    // dc ignores the amplitude and a table the offset and amplitude, as classic does.
    expect(compileRecord({ waveform: "dc", amplitude: 1e308 }).params[CORE_SIGNAL_GEN_PARAM.amplitude]).toBe(1e308);
    expect(compileRecord({ waveform: "pwl", pwl: "0:1e90,1:-1e90", amplitude: 1e308 }).blob).toEqual([2, 0, 1e90, 1, -1e90]);
    expect(reasonsOf({ waveform: "pwl", pwl: "0:1e308,1:-1e308" })).toEqual(["malformed:g1:level-out-of-range"]);
    expect(reasonsOf({ waveform: "pwl", pwl: "-1e308:0,1e308:1" })).toEqual(["malformed:g1:level-out-of-range"]);
  });
});

describe("signal_gen in the default gate", () => {
  it("is held out of the default gate pending P0-SG's live-params decision, and adds no live param", () => {
    expect(CORE_IMPLEMENTED_KINDS.has("signal_gen")).toBe(false);
    expect(compileCircuit(circuitOf({})).ok).toBe(false);
    expect(compileCircuit(circuitOf({}), WITH_SIGNAL_GEN).ok).toBe(true);
    expect(SIGNAL_GEN_LIVE_PARAM_SPECS.size).toBe(0);
    expect(LIVE_PARAM_SPECS.has("signal_gen")).toBe(false);
  });
});

describe("signal_gen records against the real Rust reader", () => {
  beforeAll(async () => {
    await initCore();
  });
  afterAll(() => {
    __resetCoreForTest();
  });

  /** `bo_engine_load`'s return code for a compiled program on a fresh engine: 0, or the reader's negative code. */
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

  it.each(Object.keys(CORE_SIGNAL_GEN_SHAPE))("accepts a compiled %s generator", (waveform) => {
    const raw: Params = { waveform, pwl: "0:0,1e-3:5,2e-3:1", seed: 7, delay: 1e-4, phaseDeg: 30 };
    const result = compileCircuit(circuitOf(raw), WITH_SIGNAL_GEN);
    if (!result.ok) throw new Error(result.reasons.join(", "));
    expect(load(result.compiled.program)).toBe(0);
  });

  it("accepts the ideal and the disabled form too", () => {
    const forms: Params[] = [{ rSource: 0, waveform: "square" }, { enabled: 0 }, { rSource: -1, waveform: "square" }];
    for (const raw of forms) {
      const result = compileCircuit(circuitOf(raw), WITH_SIGNAL_GEN);
      if (!result.ok) throw new Error(result.reasons.join(", "));
      expect(load(result.compiled.program), JSON.stringify(raw)).toBe(0);
    }
  });
});
