/**
 * Two different questions get asked about a device kind and the core, and
 * conflating them is the failure mode this file exists to prevent:
 *
 *   1. Does the TS compiler know how to turn this kind into a program
 *      record (or, for `ground`, into nothing), and does the W0 Rust kernel
 *      know that record's physics? -> CORE_IMPLEMENTED_KINDS.
 *   2. Has that implementation been PROVEN equivalent to the classic engine
 *      on real circuits, so it is safe to actually route a circuit's
 *      simulation through the core instead of the classic engine?
 *      -> CORE_SUPPORTED_KINDS.
 *
 * "Implemented" answers (1); "supported" answers (2). A kind can be
 * implemented for a long time before it is supported — that gap is
 * deliberate, not a bug: it is the space the equivalence work happens in.
 */

/**
 * Kinds compileCircuit (compile.ts) can turn into a program record, and the
 * W0 kernel's fixed-node/partition/solve logic (core/README.md, "W0 kernel
 * semantics") knows the physics of. `ground` is included even though it
 * compiles to no record at all: it is still a kind the compiler fully
 * understands, as opposed to one it has never heard of.
 *
 * This set is what a circuit is checked against to compile *at all* — it is
 * not, by itself, permission to run that circuit through the core instead
 * of the classic engine. See CORE_SUPPORTED_KINDS.
 */
export const CORE_IMPLEMENTED_KINDS: ReadonlySet<string> = new Set([
  "ground",
  "resistor",
  "voltage_source",
  "switch",
  "push_button",
  "spdt_switch",
  "dip_switch",
  "capacitor",
  // W1c: the 11 combinational ICs.
  "74ls00",
  "74ls04",
  "74ls08",
  "74ls32",
  "74ls86",
  "74ls157",
  "74ls283",
  "74hc138",
  "74hc14",
  "74ls47",
  "74ls245",
]);

/**
 * Kinds the engine-selection flag is allowed to route to the core for an
 * actual simulation. Empty in W0, on purpose: nothing has cleared the bar
 * yet, even though `resistor`/`voltage_source`/`ground` are fully
 * implemented above.
 *
 * A kind is added here only after it passes all three accuracy gates:
 *   1. brownout's own analytic-golden and first-principles suites, run
 *      against the core instead of the classic engine;
 *   2. the dual-engine equivalence harness (core vs. classic, same
 *      circuits, same results within tolerance);
 *   3. the devolt sim/learn/simcore suites, green against the core.
 *
 * The engine-selection flag routes a WHOLE circuit to the core only when
 * every kind that circuit uses is in this set. A circuit with even one
 * kind missing from CORE_SUPPORTED_KINDS falls back to the classic engine
 * in full — there is no per-device split between the two engines.
 */
export const CORE_SUPPORTED_KINDS: ReadonlySet<string> = new Set();

/**
 * Kind codes for program format v2, mirroring core/src/kinds.rs and the
 * README's "Kind codes and records" table. The orchestrator owns this table:
 * it gains a kind's record shape below, and the README row, before the
 * sub-wave that ports that kind is dispatched. A code without a shape is
 * reserved; the Rust reader rejects it as -2.
 */
export const CORE_KIND_CODES = {
  resistor: 1,
  voltage_source: 2,
  capacitor: 3,
  switch: 4,
  push_button: 5,
  spdt_switch: 6,
  dip_switch: 7,
  potentiometer: 8,
  trimmer: 9,
  fuse: 10,
  ldr: 11,
  thermistor: 12,
  battery_pack: 20,
  bench_psu: 21,
  clock: 22,
  clock_gen: 23,
  pulse_source: 24,
  pulse_gen: 25,
  led: 30,
  rgb_led: 31,
  bicolor_led: 32,
  seg7_cc: 33,
  seg7_ca: 34,
  ne555: 40,
  "74ls00": 50,
  "74ls04": 51,
  "74ls08": 52,
  "74ls32": 53,
  "74ls86": 54,
  "74ls157": 55,
  "74ls283": 56,
  "74hc138": 57,
  "74hc14": 58,
  "74ls47": 59,
  cd4511: 60,
  "74ls245": 61,
  "74ls161": 70,
  "74ls173": 71,
  "74hc74": 72,
  "74hc595": 73,
  "74hc165": 74,
  cd4017: 75,
  cd4060: 76,
  "74ls189": 80,
  "28c16": 81,
  "28c256": 82,
} as const;

export type CoreKindCodeName = keyof typeof CORE_KIND_CODES;

/** A kind's record shape, mirroring core/src/kinds.rs `KindShape`. */
export interface CoreRecordShape {
  /** Minimum pin count, in the order the README's record-layout table gives. */
  pins: number;
  /** Maximum pin count; equal to `pins` except where the count follows a param (dip_switch). */
  pinsMax: number;
  params: number;
  /** Exact blob length, or a floor for variable-length contents. */
  blob: { exact: number } | { atLeast: number };
  /** Param indices `bo_engine_set_params` may change after load (owner decision 1). */
  liveParams: readonly number[];
  /** Digital readout slots a record publishes, starting at `blob[0]` (core/README.md, "Slot indices"). 0 or absent: none. */
  digitalSlots?: number;
}

/** The params every W1c combinational IC carries, in order (core/README.md, "IC params"; core/src/kinds.rs `ic_param`). */
export const CORE_IC_PARAM = {
  vccMin: 0,
  vccMax: 1,
  famFlags: 2,
  vilFixed: 3,
  vil: 4,
  vihFixed: 5,
  vih: 6,
  schmitt: 7,
  vtPlus: 8,
  vtMinus: 9,
  nominal: 10,
  ioMax: 11,
  delay: 12,
} as const;
export const CORE_IC_PARAM_COUNT = 13;

/** `famFlags` bits and per-logic-pin blob flag bits (core/README.md, "IC params" and "IC blob"). */
export const CORE_IC_FAM_TTL = 1;
export const CORE_IC_FAM_CMOS_RATIO = 2;
export const CORE_IC_PIN_FLOATING_INPUT = 1;
export const CORE_IC_PIN_OPEN_COLLECTOR = 2;

/**
 * The delay tail's pairs per kind, in order (core/README.md, "IC delay tail";
 * core/src/kinds.rs `delay_pairs`). Pair `k` is params `13 + 2k` (dH) and
 * `14 + 2k` (dL). A kind absent here has no tail.
 */
export const CORE_IC_DELAY_PAIRS: Readonly<Record<string, readonly string[]>> = {
  "74ls157": ["strobe"],
  "74ls245": ["oeEnable", "oeDisable"],
  "74ls161": ["clkQ", "clrQ", "clkRco", "entRco"],
  "74ls173": ["clkQ", "clrQ", "mnEnable", "mnDisable"],
  "74hc74": ["clkQ", "preClrQ"],
  "74ls189": ["addrO", "csEnable", "csDisable", "weEnable", "weDisable"],
  "28c16": ["addrIo", "ceEnable", "ceDisable", "oeEnable", "oeDisable"],
  "28c256": ["addrIo", "ceEnable", "ceDisable", "oeEnable", "oeDisable"],
};

/** The EEPROMs' params after their delay tail (core/src/kinds.rs `eeprom_param`). */
export const CORE_EEPROM_PARAM = { tWC: 23, sdp: 24 } as const;
export const CORE_EEPROM_PARAM_COUNT = 25;

/** EEPROM image sizes in bytes; the blob carries `size / 4` little-endian words (core/README.md, "ROM image"). */
export const CORE_EEPROM_SIZE: Readonly<Record<string, number>> = { "28c16": 2048, "28c256": 32768 };

/** A sequential or memory IC's shape (codes 70-82): `vcc`, `gnd`, `logic` pins, `params` in all, `blob` values in all. */
function w1dShape(logic: number, params: number, blob: number, slots: number): CoreRecordShape {
  return { pins: 2 + logic, pinsMax: 2 + logic, params, blob: { exact: blob }, liveParams: [], digitalSlots: slots };
}

/**
 * A combinational IC's shape: `vcc`, `gnd`, then `logic` pins; one digital
 * slot per output-capable pin. `pairs` is the kind's own delay-tail pair
 * count (0 for a plain W1c kind; 74ls157: 1, 74ls245: 2 — `CORE_IC_DELAY_PAIRS`),
 * adding `2 * pairs` params on top of the 13 plain IC params.
 */
function icShape(logic: number, outputs: number, pairs = 0): CoreRecordShape {
  return { pins: 2 + logic, pinsMax: 2 + logic, params: CORE_IC_PARAM_COUNT + 2 * pairs, blob: { exact: 1 + logic }, liveParams: [], digitalSlots: outputs };
}

/** Shapes of the kinds ported so far. Grows per sub-wave, in step with core/src/kinds.rs. */
export const CORE_RECORD_SHAPES: ReadonlyMap<number, CoreRecordShape> = new Map<number, CoreRecordShape>([
  [CORE_KIND_CODES.resistor, { pins: 2, pinsMax: 2, params: 1, blob: { exact: 0 }, liveParams: [] }],
  [CORE_KIND_CODES.voltage_source, { pins: 2, pinsMax: 2, params: 1, blob: { exact: 0 }, liveParams: [] }],
  [CORE_KIND_CODES.capacitor, { pins: 2, pinsMax: 2, params: 3, blob: { exact: 0 }, liveParams: [] }],
  [CORE_KIND_CODES.switch, { pins: 2, pinsMax: 2, params: 1, blob: { exact: 0 }, liveParams: [0] }],
  [CORE_KIND_CODES.push_button, { pins: 2, pinsMax: 2, params: 1, blob: { exact: 0 }, liveParams: [0] }],
  [CORE_KIND_CODES.spdt_switch, { pins: 3, pinsMax: 3, params: 1, blob: { exact: 0 }, liveParams: [0] }],
  [CORE_KIND_CODES.dip_switch, { pins: 4, pinsMax: 16, params: 9, blob: { exact: 0 }, liveParams: [1, 2, 3, 4, 5, 6, 7, 8] }],
  [CORE_KIND_CODES["74ls00"], icShape(12, 4)],
  [CORE_KIND_CODES["74ls04"], icShape(12, 6)],
  [CORE_KIND_CODES["74ls08"], icShape(12, 4)],
  [CORE_KIND_CODES["74ls32"], icShape(12, 4)],
  [CORE_KIND_CODES["74ls86"], icShape(12, 4)],
  [CORE_KIND_CODES["74ls157"], icShape(14, 4, 1)],
  [CORE_KIND_CODES["74ls283"], icShape(14, 5)],
  [CORE_KIND_CODES["74hc138"], icShape(14, 8)],
  [CORE_KIND_CODES["74hc14"], icShape(12, 6)],
  [CORE_KIND_CODES["74ls47"], icShape(14, 7)],
  [CORE_KIND_CODES["74ls245"], icShape(18, 16, 2)],
  [CORE_KIND_CODES["74ls161"], w1dShape(14, 21, 1 + 14, 5)],
  [CORE_KIND_CODES["74ls173"], w1dShape(14, 21, 1 + 14, 4)],
  [CORE_KIND_CODES["74hc74"], w1dShape(12, 17, 1 + 12, 4)],
  [CORE_KIND_CODES["74ls189"], w1dShape(14, 23, 1 + 14, 4)],
  [CORE_KIND_CODES["28c16"], w1dShape(22, CORE_EEPROM_PARAM_COUNT, 1 + 22 + 2048 / 4, 8)],
  [CORE_KIND_CODES["28c256"], w1dShape(26, CORE_EEPROM_PARAM_COUNT, 1 + 26 + 32768 / 4, 8)],
]);
