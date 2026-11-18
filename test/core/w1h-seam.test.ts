/**
 * The W1h contract seam (item H0): kind codes, record shapes, delay pairs and
 * the compiler dispatch, pinned in one place so that the items which fill the
 * seam in (H1a, H2a-c, H3a, H4a) can never move it by accident. Nothing here
 * runs a W1h kind: none is in CORE_IMPLEMENTED_KINDS and every per-item module
 * is a stub that declines. The Rust half is core/src/kinds.rs's own tests and
 * the reader tests in core/src/program.rs; the wasm-load test at the end ties
 * the two together (every TS shape is accepted by the real Rust reader).
 */
import { afterAll, beforeAll, describe, expect, it } from "vitest";
import {
  CORE_DEVICE_STATE_LAYOUT,
  CORE_IC_DELAY_PAIRS,
  CORE_IC_PARAM_COUNT,
  CORE_IMPLEMENTED_KINDS,
  CORE_KIND_CODES,
  CORE_RECORD_SHAPES,
  CORE_SUPPORTED_KINDS,
  CORE_W1H_IC_OUTPUTS,
  CORE_W1H_IC_PINS,
} from "../../src/core/kinds.js";
import { CORE_DELAY_PATHS, resolveDelayTail } from "../../src/core/compile/delays.js";
import { compileCircuit, LIVE_PARAM_SPECS } from "../../src/core/compile/index.js";
import { SHIFT_KINDS } from "../../src/core/compile/shift.js";
import { COUNTER_KINDS } from "../../src/core/compile/counters.js";
import { DECODER_KINDS } from "../../src/core/compile/decoder.js";
import { MAGNETICS_KINDS } from "../../src/core/compile/magnetics.js";
import { SIGNAL_GEN_KINDS } from "../../src/core/compile/sources_w1h.js";
import { VARIABLE_KINDS } from "../../src/core/compile/variable.js";
import { CORE_OPTION_KEYS, decodeProgram, ProgramWriter } from "../../src/core/format.js";
import { __resetCoreForTest, coreCall, initCore } from "../../src/core/wasm.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import type { ElectricalSpecs } from "../../src/circuit/types.js";

/** The kinds W1h adds a code or a shape for, with the module that compiles each. */
const W1H_KINDS = [
  "potentiometer",
  "trimmer",
  "ldr",
  "thermistor",
  "inductor",
  "resistor_array",
  "coupled_inductor",
  "push_dpdt",
  "signal_gen",
  "current_source",
  "cd4511",
  "74hc595",
  "74hc165",
  "cd4017",
  "cd4060",
] as const;

/** W1h kinds whose item has landed: in CORE_IMPLEMENTED_KINDS, compiled by their module instead of a declining stub. */
const LANDED: ReadonlySet<string> = new Set(["ldr", "push_dpdt", "current_source", "ferrite_bead", "potentiometer", "trimmer", "resistor_array", "signal_gen"]);


describe("W1h kind codes", () => {
  it("assigns the ruled codes and keeps the reserved ones", () => {
    expect(CORE_KIND_CODES).toMatchObject({
      potentiometer: 8,
      trimmer: 9,
      ldr: 11,
      thermistor: 12,
      inductor: 15,
      resistor_array: 16,
      coupled_inductor: 17,
      push_dpdt: 18,
      signal_gen: 27,
      current_source: 28,
      cd4511: 60,
      "74hc595": 73,
      "74hc165": 74,
      cd4017: 75,
      cd4060: 76,
      // The pre-W1h codes did not move.
      resistor: 1,
      fuse: 10,
      ptc_fuse: 13,
      section_anchor: 26,
      led: 30,
      "74ls245": 61,
      "74ls161": 70,
      rating: 90,
      thermal: 91,
    });
  });

  it("never assigns a code twice, and leaves 14 (ferrite_bead) and 19 (crystal) free (R56 3)", () => {
    const codes = Object.values(CORE_KIND_CODES);
    expect(new Set(codes).size).toBe(codes.length);
    expect(codes).not.toContain(14);
    expect(codes).not.toContain(19);
    expect(CORE_KIND_CODES).not.toHaveProperty("ferrite_bead");
    expect(CORE_KIND_CODES).not.toHaveProperty("crystal");
  });

  it("gives every code a record shape, and no W1h kind joins the supported set", () => {
    for (const code of Object.values(CORE_KIND_CODES)) {
      expect(CORE_RECORD_SHAPES.has(code), `code ${code}`).toBe(true);
    }
    for (const kind of W1H_KINDS) {
      // A kind joins CORE_IMPLEMENTED_KINDS when its item lands (its own tests cover its compiler), never
      // CORE_SUPPORTED_KINDS before W1z.
      expect(CORE_IMPLEMENTED_KINDS.has(kind), kind).toBe(LANDED.has(kind));
      expect(CORE_SUPPORTED_KINDS.has(kind), kind).toBe(false);
    }
    // H1b: a bead is a `resistor` record, in the set since its solver path (the resistor's) needs nothing new.
    expect(CORE_IMPLEMENTED_KINDS.has("ferrite_bead")).toBe(true);
    expect(CORE_IMPLEMENTED_KINDS.has("crystal")).toBe(false);
  });
});

describe("W1h record shapes", () => {
  it("are the contract: pins, params, blob, live params and slots", () => {
    const shape = (kind: (typeof W1H_KINDS)[number]) => CORE_RECORD_SHAPES.get(CORE_KIND_CODES[kind])!;
    const plain = (pins: number, params: number, liveParams: readonly number[] = []) => ({
      pins,
      pinsMax: pins,
      params,
      blob: { exact: 0 },
      liveParams,
    });
    expect(shape("potentiometer")).toEqual(plain(3, 3, [0, 1]));
    expect(shape("trimmer")).toEqual(plain(3, 3, [0, 1]));
    expect(shape("ldr")).toEqual(plain(2, 1, [0]));
    expect(shape("thermistor")).toEqual(plain(2, 5, [4]));
    expect(shape("inductor")).toEqual(plain(2, 3));
    expect(shape("coupled_inductor")).toEqual(plain(4, 5));
    expect(shape("push_dpdt")).toEqual(plain(6, 1, [0]));
    expect(shape("current_source")).toEqual(plain(2, 2, [0, 1]));
    expect(shape("resistor_array")).toEqual({ ...plain(8, 2), blob: { exact: 1 }, channelSlots: 4 });
    expect(shape("signal_gen")).toEqual({ ...plain(2, 13), blob: { atLeast: 1 } });
    const slots = { cd4511: 7, "74hc595": 8, "74hc165": 2, cd4017: 12, cd4060: 13 } as const;
    for (const [kind, digitalSlots] of Object.entries(slots)) {
      const pairs = CORE_IC_DELAY_PAIRS[kind]!.length;
      expect(shape(kind as keyof typeof slots), kind).toEqual({
        pins: 16,
        pinsMax: 16,
        params: CORE_IC_PARAM_COUNT + 2 * pairs,
        blob: { exact: 15 },
        liveParams: [],
        digitalSlots,
      });
    }
  });

  it("list each logic IC's pins and driven pins consistently with its slot count", () => {
    for (const kind of ["74hc595", "74hc165", "cd4017", "cd4060", "cd4511"]) {
      const pins = CORE_W1H_IC_PINS[kind]!;
      const outputs = CORE_W1H_IC_OUTPUTS[kind]!;
      expect(pins.length, kind).toBe(14);
      expect(new Set(pins).size, kind).toBe(14);
      const bareCountKey = kind === "cd4017" || kind === "cd4060" ? 1 : 0;
      expect(CORE_RECORD_SHAPES.get(CORE_KIND_CODES[kind as "cd4511"])!.digitalSlots, kind).toBe(outputs.length + bareCountKey);
    }
    // Q-O58: the 595's QH' is listed (it is a catalog pin) and never driven.
    const hc595 = CORE_W1H_IC_PINS["74hc595"]!;
    expect(CORE_W1H_IC_OUTPUTS["74hc595"]!.map((i) => hc595[i])).toEqual(["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh"]);
    // The 4060 follows the catalog's stage pins, not icPinMap's stale q3/q2/q1.
    expect(CORE_W1H_IC_PINS.cd4060).toContain("q9");
    expect(CORE_W1H_IC_PINS.cd4060).not.toContain("q1");
  });

  it("name the device-state layouts R56 8 rules", () => {
    expect(CORE_DEVICE_STATE_LAYOUT).toEqual({ inductor: ["i", "v"], coupled_inductor: ["i1", "i2", "v1", "v2"] });
  });
});

describe("W1h delay pairs (R56 10)", () => {
  it("extend CORE_IC_DELAY_PAIRS for the five kinds, each with a catalog path", () => {
    expect(CORE_IC_DELAY_PAIRS["74hc595"]).toEqual(["srclkQh", "rclkQ", "srclrQh", "oeEnable", "oeDisable"]);
    expect(CORE_IC_DELAY_PAIRS["74hc165"]).toEqual(["clkQ", "shldQ", "dataQ"]);
    expect(CORE_IC_DELAY_PAIRS.cd4017).toEqual(["clkQ", "clrQ", "clkCo"]);
    expect(CORE_IC_DELAY_PAIRS.cd4060).toEqual(["clkQ", "stageQ", "clrQ"]);
    expect(CORE_IC_DELAY_PAIRS.cd4511).toEqual(["dataSeg"]);
    for (const kind of Object.keys(CORE_IC_DELAY_PAIRS)) {
      for (const pair of CORE_IC_DELAY_PAIRS[kind]!) expect(CORE_DELAY_PATHS[pair], `${kind}/${pair}`).toBeDefined();
    }
  });

  it("resolve to w1-datasheet-delays.md's typical figures from the catalog blocks", async () => {
    const { BUNDLED_DEFAULT_PARTS } = await import("../../src/parts/part-library.js");
    const specsOf = (uid: string): ElectricalSpecs | undefined => BUNDLED_DEFAULT_PARTS.find((p) => p.uid === uid)?.electrical_specs ?? undefined;
    const ns = (kind: string, uid: string, fallbackNs: number) => resolveDelayTail(kind, specsOf(uid), fallbackNs * 1e-9).map((s) => Math.round(s * 1e12) / 1e3);
    // [dH, dL] per pair, in ns.
    expect(ns("74hc595", "ic-74hc595", 15)).toEqual([17, 17, 17, 17, 18, 18, 15, 15, 23, 23]);
    expect(ns("74hc165", "ic-74hc165", 15)).toEqual([15, 15, 20, 20, 15, 15]);
    expect(ns("cd4017", "ic-cd4017", 250)).toEqual([325, 325, 265, 265, 300, 300]);
    expect(ns("cd4060", "ic-cd4060", 550)).toEqual([370, 370, 100, 100, 180, 180]);
    // The Data row: 660 ns rising, 520 ns falling (correcting Q-O9's 250 ns, which is the LT row).
    expect(ns("cd4511", "ic-cd4511", 250)).toEqual([660, 520]);
  });

  it("fall back to the package delay when a catalog has no block (devolt's injected catalog until it gains one)", () => {
    expect(resolveDelayTail("74hc595", undefined, 15e-9)).toEqual(new Array(10).fill(15e-9));
    expect(resolveDelayTail("cd4511", undefined, 250e-9)).toEqual([250e-9, 250e-9]);
  });
});

describe("W1h compiler dispatch", () => {
  it("routes every W1h kind to exactly one per-item module", () => {
    const modules = [VARIABLE_KINDS, MAGNETICS_KINDS, SIGNAL_GEN_KINDS, SHIFT_KINDS, COUNTER_KINDS, DECODER_KINDS];
    const owned = modules.flatMap((set) => [...set]);
    expect(new Set(owned).size).toBe(owned.length);
    expect(new Set(owned)).toEqual(new Set([...W1H_KINDS, "ferrite_bead", "crystal"]));
    expect(VARIABLE_KINDS.has("ferrite_bead")).toBe(true);
    expect(MAGNETICS_KINDS.has("crystal")).toBe(true);
  });

  it("declines each W1h kind not yet landed as unsupported under the default gate, and through its stub when the gate is widened", () => {
    // A landed kind (in CORE_IMPLEMENTED_KINDS) has a real compiler and its own tests.
    for (const kind of [...W1H_KINDS, "ferrite_bead", "crystal"].filter((k) => !LANDED.has(k))) {
      const circuit: SimCircuit = {
        components: [{ id: "x1", kind, pins: [{ id: "a" }, { id: "b" }], params: {} }],
        wires: [],
      };
      const expected = { ok: false, reasons: [`unsupported-kind:${kind}`] };
      expect(compileCircuit(circuit), kind).toEqual(expected);
      // A caller that claims the kind reaches its module's stub, which declines the same way; before the
      // seam this threw the "no compiler case" drift error.
      expect(compileCircuit(circuit, new Set([kind])), kind).toEqual(expected);
    }
  });

  it("adds a live-param spec only for the kinds whose item has landed with live params", () => {
    const withSpec = new Set(["potentiometer", "trimmer", "ldr", "push_dpdt", "current_source"]);
    for (const kind of [...W1H_KINDS, "ferrite_bead", "crystal"]) expect(LIVE_PARAM_SPECS.has(kind), kind).toBe(withSpec.has(kind));
  });
});

describe("W1h shapes against the real Rust reader", () => {
  beforeAll(async () => {
    await initCore();
  });
  afterAll(() => {
    __resetCoreForTest();
  });

  const pin = (node: number) => ({ node, open: false, openHigh: false, openHash: 0, deadHash: 0 });
  const hcParams = (tail: number, schmitt = false) => {
    const params = [2, 6, 2, 0, 0, 0, 0, 0, 0, 0, 5, 0.025, 20e-9];
    if (schmitt) params.splice(7, 3, 1, 2.75, 2.25);
    return [...params, ...new Array(2 * tail).fill(30e-9)];
  };

  /** One well-formed record per W1h kind: `[kind, pins, params, blob]`. */
  const records: ReadonlyArray<readonly [(typeof W1H_KINDS)[number], number, number[], number[]]> = [
    ["potentiometer", 3, [5000, 5000, 0.2], []],
    ["trimmer", 3, [1, 9999, 0], []],
    ["ldr", 2, [15000], []],
    ["thermistor", 2, [10000, 3950, 0.007, 15, 25], []],
    ["inductor", 2, [1e-3, 5, 1e5], []],
    ["resistor_array", 8, [10000, 0.0625], [0]],
    ["coupled_inductor", 4, [1e-3, 1e-3, 0.98, 5, 5], []],
    ["push_dpdt", 6, [0], []],
    ["signal_gen", 2, [1, 2.5, 2.5, 1000, 0, 0.5, 0, 1e-6, 1e-6, 5e-4, 1, 1, 50], [0]],
    ["current_source", 2, [1e-3, 0], []],
    ["cd4511", 16, hcParams(1), [0, ...new Array(14).fill(0)]],
    ["74hc595", 16, hcParams(5), [7, ...new Array(14).fill(0)]],
    ["74hc165", 16, hcParams(3), [15, ...new Array(14).fill(0)]],
    ["cd4017", 16, hcParams(3), [17, ...new Array(14).fill(0)]],
    ["cd4060", 16, hcParams(3, true), [29, ...new Array(14).fill(0)]],
  ];

  /** `bo_engine_load`'s return code for `program` on a fresh engine: 0, or the reader's negative code. */
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

  function program(kind: (typeof W1H_KINDS)[number], pins: number, params: number[], blob: number[]): Float64Array {
    const writer = new ProgramWriter(2, [[CORE_OPTION_KEYS.AMBIENT_C, 25]]);
    writer.addDevice({
      kindCode: CORE_KIND_CODES[kind],
      componentIndex: 0,
      pins: Array.from({ length: pins }, (_, i) => pin((i + 1) % 2)),
      params,
      blob,
    });
    // S = 42: the five ICs' slot ranges laid end to end (7 + 8 + 2 + 12 + 13); C = 4 for the array.
    return writer.finish(42, 4);
  }

  it("accepts a well-formed record of every W1h kind, as the TS writer and decoder do", () => {
    for (const [kind, pins, params, blob] of records) {
      const built = program(kind, pins, params, blob);
      expect(decodeProgram(built).devices[0]!.kindCode, kind).toBe(CORE_KIND_CODES[kind]);
      expect(load(built), kind).toBe(0);
    }
  });

  it("refuses a bad value with the reader's -1, so the load above really parses the record", () => {
    // rCw = 0 (pot), L = 0 (inductor), k = 1 (coupled), waveform 9 (signal_gen), schmitt on a 595.
    expect(load(program("potentiometer", 3, [0, 5000, 0.2], []))).toBe(-1);
    expect(load(program("inductor", 2, [0, 5, 1e5], []))).toBe(-1);
    expect(load(program("coupled_inductor", 4, [1e-3, 1e-3, 1, 5, 5], []))).toBe(-1);
    expect(load(program("signal_gen", 2, [9, 2.5, 2.5, 1000, 0, 0.5, 0, 1e-6, 1e-6, 5e-4, 1, 1, 50], [0]))).toBe(-1);
    expect(load(program("74hc595", 16, hcParams(5, true), [7, ...new Array(14).fill(0)]))).toBe(-1);
  });

  it("refuses a record of the wrong param count in the TS writer, before the reader sees it", () => {
    for (const [kind, pins, params, blob] of records) {
      expect(() => program(kind, pins, params.slice(0, -1), blob), kind).toThrow();
    }
  });
});
