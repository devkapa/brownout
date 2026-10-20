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
  "74ls161",
  "74ls173",
  "74hc74",
  "74ls189",
  "28c16",
  "28c256",
  // W1e: LEDs, the 7-segment displays and the NE555.
  "led",
  "rgb_led",
  "bicolor_led",
  "seg7_cc",
  "seg7_ca",
  "ne555",
  // W1f: the sources (codes 20-25). The compiler adds a `section_anchor`
  // record (code 26) for each isolated section on its own; it is never a
  // component kind.
  "battery_pack",
  "bench_psu",
  "clock",
  "clock_gen",
  "pulse_source",
  "pulse_gen",
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
  // Synthetic: emitted by the compiler for each isolated section, never a component kind (core/README.md, "Section anchor record").
  section_anchor: 26,
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
  /**
   * Exact blob length, a floor for variable-length contents, or a table:
   * `blob[0] = n` and the blob holds `1 + 2n` values (core/src/kinds.rs
   * `BlobShape::Table`, the battery's OCV table).
   */
  blob: { exact: number } | { atLeast: number } | { table: true };
  /** Param indices `bo_engine_set_params` may change after load (owner decision 1). */
  liveParams: readonly number[];
  /** Digital readout slots a record publishes, starting at `blob[0]` (core/README.md, "Slot indices"). 0 or absent: none. */
  digitalSlots?: number;
  /**
   * Channel readout slots a record publishes (core/README.md, "Slot indices"). Their base follows the
   * digital base in the blob, or is `blob[0]` when the kind has no digital slots. 0 or absent: none.
   */
  channelSlots?: number;
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

/**
 * LED and display params (core/README.md, "LED and display records"; core/src/kinds.rs `led_param`,
 * `rgb_param`, `bicolor_param`, `seg7_param`). `led` carries no `ifMax`: its damage limit resolves
 * exactly as `iRated` does. `seg7` has no `burnt` and no damage limit (Q-O22).
 */
export const CORE_LED_PARAM = { vf: 0, n: 1, iRated: 2, isSet: 3, is: 4, burnt: 5 } as const;
export const CORE_LED_PARAM_COUNT = 6;
export const CORE_RGB_LED_PARAM = { vfR: 0, vfG: 1, vfB: 2, iRated: 3, ifMax: 4, burnt: 5 } as const;
export const CORE_RGB_LED_PARAM_COUNT = 6;
export const CORE_BICOLOR_LED_PARAM = { vf1: 0, vf2: 1, iRated: 2, ifMax: 3, burnt: 4 } as const;
export const CORE_BICOLOR_LED_PARAM_COUNT = 5;
export const CORE_SEG7_PARAM = { vf: 0, iRated: 1 } as const;
export const CORE_SEG7_PARAM_COUNT = 2;

/**
 * The battery record's params, in order (core/README.md, "Battery record"; core/src/kinds.rs `battery_param`).
 * The blob is the OCV table `[n, soc_0, v_0, ..., soc_{n-1}, v_{n-1}]` (`BlobShape::Table`).
 */
export const CORE_BATTERY_PARAM = {
  ocvScale: 0,
  rBase: 1,
  tempFactor: 2,
  depA: 3,
  depP: 4,
  depK: 5,
  capacityAh: 6,
  soc0: 7,
  profileCode: 8,
} as const;
export const CORE_BATTERY_PARAM_COUNT = 9;

/** `profileCode` by classic's profile id (core/README.md, "Battery record" index 8). Rust ignores it; TS reads the id and label from the profile. */
export const CORE_BATTERY_PROFILE_CODE: Readonly<Record<string, number>> = {
  "generic-primary": 0,
  "alkaline-aa-4s": 1,
  "alkaline-pp3-9v": 2,
  "lithium-cr2032": 3,
};

/** The bench PSU's params (core/README.md, "Bench PSU record"; core/src/kinds.rs). `iLimit` 0 is an ideal source. */
export const CORE_BENCH_PSU_PARAM = { V: 0, iLimit: 1 } as const;
export const CORE_BENCH_PSU_PARAM_COUNT = 2;

/** The clock's params (core/README.md, "Waveform source records"). */
export const CORE_CLOCK_PARAM = { freq: 0, duty: 1, vHigh: 2, vLow: 3, delay: 4 } as const;
export const CORE_CLOCK_PARAM_COUNT = 5;

/** The pulse source's params (core/README.md, "Waveform source records"). `per` 0 is a one-shot. */
export const CORE_PULSE_PARAM = { v1: 0, v2: 1, td: 2, tr: 3, tf: 4, pw: 5, per: 6 } as const;
export const CORE_PULSE_PARAM_COUNT = 7;

/** An LED-family shape (codes 30-34): `pins` by the README's table, `params`, and `channels` slots whose base is the blob's only value. */
function ledShape(pins: number, params: number, channels: number): CoreRecordShape {
  return { pins, pinsMax: pins, params, blob: { exact: channels > 0 ? 1 : 0 }, liveParams: [], channelSlots: channels };
}

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
  // W1f sources (core/README.md, "Source records"): positional pins except the battery's `pos`/`neg`; no live params.
  [CORE_KIND_CODES.battery_pack, { pins: 2, pinsMax: 2, params: CORE_BATTERY_PARAM_COUNT, blob: { table: true }, liveParams: [] }],
  [CORE_KIND_CODES.bench_psu, { pins: 2, pinsMax: 2, params: CORE_BENCH_PSU_PARAM_COUNT, blob: { exact: 0 }, liveParams: [] }],
  [CORE_KIND_CODES.clock, { pins: 2, pinsMax: 2, params: CORE_CLOCK_PARAM_COUNT, blob: { exact: 1 }, liveParams: [], digitalSlots: 1 }],
  [CORE_KIND_CODES.clock_gen, { pins: 2, pinsMax: 2, params: CORE_CLOCK_PARAM_COUNT, blob: { exact: 1 }, liveParams: [], digitalSlots: 1 }],
  [CORE_KIND_CODES.pulse_source, { pins: 2, pinsMax: 2, params: CORE_PULSE_PARAM_COUNT, blob: { exact: 1 }, liveParams: [], digitalSlots: 1 }],
  [CORE_KIND_CODES.pulse_gen, { pins: 2, pinsMax: 2, params: CORE_PULSE_PARAM_COUNT, blob: { exact: 1 }, liveParams: [], digitalSlots: 1 }],
  [CORE_KIND_CODES.section_anchor, { pins: 1, pinsMax: 1, params: 0, blob: { exact: 0 }, liveParams: [] }],
  [CORE_KIND_CODES.led, ledShape(2, CORE_LED_PARAM_COUNT, 0)],
  [CORE_KIND_CODES.rgb_led, ledShape(4, CORE_RGB_LED_PARAM_COUNT, 3)],
  [CORE_KIND_CODES.bicolor_led, ledShape(3, CORE_BICOLOR_LED_PARAM_COUNT, 2)],
  [CORE_KIND_CODES.seg7_cc, ledShape(10, CORE_SEG7_PARAM_COUNT, 8)],
  [CORE_KIND_CODES.seg7_ca, ledShape(10, CORE_SEG7_PARAM_COUNT, 8)],
  // NE555: pins `8 1 2 3 4 5 6 7`, the 13 IC params with 7-9, 11 and 12 forced to 0, blob `[slotBase, 0 x 6]`.
  [CORE_KIND_CODES.ne555, { pins: 8, pinsMax: 8, params: CORE_IC_PARAM_COUNT, blob: { exact: 7 }, liveParams: [], digitalSlots: 1 }],
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
