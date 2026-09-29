/**
 * "Program format, version 3" — the wire format the TS compiler (compile/)
 * emits and the Rust W1a event kernel reads (core/README.md, "Program
 * format, version 3"). This file is the TS half of that contract: the
 * format constants, a small writer used by compile/index.ts, and a decoder
 * that performs every validation the contract requires of the Rust reader.
 *
 * The decoder exists for two reasons: a Float64Array is otherwise opaque to
 * a test, and it doubles as executable documentation of the exact layout —
 * the prose in core/README.md and the checks below must never drift, since
 * the Rust reader is written against the README's table, not against this
 * file.
 *
 * Every value in a program is an f64. Counts, indices and 32-bit hash states
 * are carried as exact doubles; every one of them this format ever produces
 * is far below 2^53, so there is no precision loss doing that.
 *
 * Layout (core/README.md, "Program format, version 3"):
 *   header   MAGIC, VERSION, nodeCount N, deviceCount D,
 *            optionCount O, (key, value) x O,
 *            digitalSlotCount S, channelSlotCount C
 *   devices  D records, in component order:
 *              kindCode, componentIndex, pinCount, pin record x pinCount,
 *              paramCount, param x paramCount, blobCount, blob x blobCount
 *   trailer  MAGIC
 *
 * A pin record is four values: `node, flags, openHash, deadHash` (the
 * contract's "pin record" bullet). Kind codes and per-kind record shapes
 * (pin/param counts, blob shape) are NOT owned here — they are the
 * orchestrator's table in kinds.ts (CORE_KIND_CODES / CORE_RECORD_SHAPES),
 * re-exported below under their v1 names too, since some call sites still
 * import them from this file.
 */

import { CORE_KIND_CODES, CORE_RECORD_SHAPES, type CoreKindCodeName } from "./kinds.js";

/** "BOC1" read as one big-endian u32. Stamped at both the start and the end of every program. */
export const CORE_FORMAT_MAGIC = 0x424f4331;

/**
 * The only version this package's TS and Rust halves understand. Both sides
 * ship in one npm package, so there is no cross-version compatibility to
 * preserve: a layout change bumps this constant on both sides in the same
 * commit, and decodeProgram rejects anything else outright rather than
 * trying to interpret an old or new layout. W1d bumped it from 2 to 3
 * because the 74ls157 and 74ls245 records gained a delay tail
 * (core/README.md, "Program format, version 3").
 */
export const CORE_FORMAT_VERSION = 3;

/**
 * v1 export names, kept as aliases now that the kind-code table moved to
 * kinds.ts (the orchestrator's table, shared with core/src/kinds.rs) — any
 * code still importing `CORE_KIND_CODE`/`CoreKindName` from format.ts (this
 * file's own v1-era tests, chiefly) keeps working unchanged.
 */
export const CORE_KIND_CODE = CORE_KIND_CODES;
export type CoreKindName = CoreKindCodeName;

/** Reverse lookup: kind code -> kind name. Used by decodeProgram's error messages and by debugging tools. */
export const CORE_KIND_NAME_BY_CODE: ReadonlyMap<number, CoreKindCodeName> = new Map(
  (Object.keys(CORE_KIND_CODES) as CoreKindCodeName[]).map((name) => [CORE_KIND_CODES[name], name]),
);

/**
 * Option keys, core/README.md "Program format, version 3" > the Options
 * table. Every key is optional in the wire format itself (each appears at
 * most once; an absent key means the reader applies the README's Default
 * column) — ProgramWriter/decodeProgram only validate the keys that ARE
 * present, they never invent a default value to write.
 */
export const CORE_OPTION_KEYS = {
  INTEGRATION: 1,
  NO_SOLUTION_CACHE: 2,
  NO_RAIL_SHORTCUT: 3,
  FORCE_STEPPED: 4,
  AMBIENT_C: 5,
  FLOATING_MODEL: 6,
  START_TIME: 7,
} as const;

export type CoreOptionKeyName = keyof typeof CORE_OPTION_KEYS;

const CORE_OPTION_KEY_VALUES: ReadonlySet<number> = new Set(Object.values(CORE_OPTION_KEYS));

const CORE_OPTION_NAME_BY_KEY: ReadonlyMap<number, CoreOptionKeyName> = new Map(
  (Object.keys(CORE_OPTION_KEYS) as CoreOptionKeyName[]).map((name) => [CORE_OPTION_KEYS[name], name]),
);

/**
 * Validates one option's VALUE against the README's Values column for its
 * key. Assumes the key itself is already known (CORE_OPTION_KEY_VALUES) —
 * callers check that separately, since "unknown key" and "bad value for a
 * known key" are different failure messages on both the writer and reader
 * side. Returns a human-readable reason, or null when the value is in range.
 */
function optionValueError(key: number, value: number): string | null {
  switch (key) {
    case CORE_OPTION_KEYS.INTEGRATION:
    case CORE_OPTION_KEYS.NO_SOLUTION_CACHE:
    case CORE_OPTION_KEYS.NO_RAIL_SHORTCUT:
    case CORE_OPTION_KEYS.FORCE_STEPPED:
      return value === 0 || value === 1 ? null : `must be 0 or 1, got ${value}`;
    case CORE_OPTION_KEYS.AMBIENT_C:
      return Number.isFinite(value) ? null : `must be a finite number, got ${value}`;
    case CORE_OPTION_KEYS.FLOATING_MODEL:
      return value === 0 ? null : `must be 0 (the only value in W1), got ${value}`;
    case CORE_OPTION_KEYS.START_TIME:
      return Number.isFinite(value) && value >= 0 ? null : `must be a finite number >= 0, got ${value}`;
    default:
      // Unreachable once the caller's "known key" check has already run.
      return null;
  }
}

/** Bit 0 of a pin record's `flags` word (core/README.md's pin-record bullet). */
const PIN_FLAG_OPEN = 1;
/** Bit 1. */
const PIN_FLAG_OPEN_HIGH = 2;
const PIN_FLAG_KNOWN_BITS = PIN_FLAG_OPEN | PIN_FLAG_OPEN_HIGH;

/** The largest value a 32-bit FNV-1a state (or its `>>> 0` unsigned reading) can take. */
const U32_MAX = 0xffffffff;

/**
 * One pin's wire-format record before it is flattened into four raw
 * Float64Array values (core/README.md's pin-record bullet). `openHash`/
 * `deadHash` are the FNV-1a-32 states after hashing
 * `${componentId}:${pinId}:` and `${componentId}:${pinId}:undefined:`
 * respectively (compile/topology.ts computes these; format.ts only carries
 * and validates the finished numbers).
 */
export interface ProgramPinInput {
  /** The pin's node index, `< nodeCount`. */
  node: number;
  /** Classic's `_isOpenPin` result for this pin (spec §1.6). */
  open: boolean;
  /** Whether this pin is in classic's `IC_OPEN_HIGH_PINS` (spec §1.5). */
  openHigh: boolean;
  openHash: number;
  deadHash: number;
}

/** One decoded pin record — decodeProgram's view of a ProgramPinInput after the round trip. */
export interface DecodedPin {
  node: number;
  open: boolean;
  openHigh: boolean;
  openHash: number;
  deadHash: number;
}

/** One device record's input to ProgramWriter.addDevice, before it is flattened into the Float64Array. */
export interface ProgramDeviceInput {
  kindCode: number;
  /** Index into the source circuit's `components` array (contract: "the index in circuit.components"). */
  componentIndex: number;
  /** Pin records, positional or by-id per the kind's "Pins (in order)" column (core/README.md, G7). */
  pins: readonly ProgramPinInput[];
  /** Resolved parameter values, positional per the kind's "Params" column. */
  params: readonly number[];
  /** ROM/RAM/pin-map payload. Empty for every W1a kind; later waves carry per-kind blob data. */
  blob?: readonly number[];
}

/** One `(key, value)` option pair, in the order it appears on the wire. */
export type ProgramOption = readonly [key: number, value: number];

/**
 * Builds one program: header (including the option block and readout slot
 * counts), then device records in the order addDevice is called (which must
 * be component order — the caller's job, not this class's), then trailer.
 *
 * `deviceCount`, `digitalSlotCount` and `channelSlotCount` are not known
 * until every device has been added (compile/index.ts allocates readout
 * slots WHILE compiling devices), so all three are written as zero
 * placeholders and patched by `finish()` — the only two-pass part of an
 * otherwise append-only writer.
 *
 * addDevice validates each record against kinds.ts's CORE_RECORD_SHAPES and
 * against this program's own nodeCount. This is a belt-and-braces check on
 * the producer side, not a substitute for decodeProgram (or the Rust
 * reader): the source of truth for "is this program well-formed" is the
 * reader, because that is what actually has to survive an untrusted or
 * hand-built Float64Array.
 */
export class ProgramWriter {
  private readonly values: number[];
  private readonly nodeCount: number;
  private readonly deviceCountIndex: number;
  private readonly slotCountsIndex: number;
  private deviceCount = 0;
  private finished = false;
  // Every device whose shape declares `digitalSlots` (W1c's combinational
  // ICs, so far): its blob[0] (slot base) and slot count, checked against
  // the final digitalSlotCount in finish() — the same `blob[0] + count <= S`
  // rule core/src/program.rs's reader enforces (core/README.md, "Slot
  // indices"). Deferred to finish() rather than checked in addDevice()
  // because S itself is a finish()-time placeholder until every device (and
  // therefore every slot allocation) has been added — see this class's own
  // doc comment on the two-pass deviceCount/slotCounts patch.
  private readonly pendingDigitalSlotChecks: { slotBase: number; count: number }[] = [];
  // The channel-slot twin of the above (W1e's `rgb_led`/`bicolor_led`/
  // `seg7_cc`/`seg7_ca`, codes 31-34, the first kinds with `channelSlots`):
  // `base + count <= C`, checked against the final channelSlotCount in
  // finish() for the same two-pass reason. A kind's blob carries digital
  // slots before channel slots (core/README.md, "Slot indices"), so the
  // channel base sits at `blob[1]` for a kind with both and `blob[0]` for one
  // with channel slots only — no W1e kind has both, but addDevice computes
  // the index generically from the shape rather than assuming `blob[0]`.
  private readonly pendingChannelSlotChecks: { slotBase: number; count: number }[] = [];

  constructor(nodeCount: number, options: readonly ProgramOption[] = []) {
    if (!Number.isInteger(nodeCount) || nodeCount < 1) {
      throw new Error(`ProgramWriter: nodeCount must be an integer >= 1, got ${nodeCount}`);
    }
    this.nodeCount = nodeCount;

    const seenKeys = new Set<number>();
    for (const [key, value] of options) {
      if (!CORE_OPTION_KEY_VALUES.has(key)) {
        throw new Error(`ProgramWriter: unknown option key ${key}`);
      }
      if (seenKeys.has(key)) {
        throw new Error(
          `ProgramWriter: option key ${key} (${CORE_OPTION_NAME_BY_KEY.get(key)}) is repeated`,
        );
      }
      seenKeys.add(key);
      const error = optionValueError(key, value);
      if (error) {
        throw new Error(`ProgramWriter: option ${CORE_OPTION_NAME_BY_KEY.get(key)} ${error}`);
      }
    }

    // Header up through the option block: MAGIC, VERSION, nodeCount,
    // deviceCount (placeholder, index 3), optionCount, then the pairs
    // themselves. The digitalSlotCount/channelSlotCount placeholders follow
    // immediately after (see below) — both patched by finish().
    this.values = [CORE_FORMAT_MAGIC, CORE_FORMAT_VERSION, nodeCount, 0, options.length];
    this.deviceCountIndex = 3;
    for (const [key, value] of options) this.values.push(key, value);
    this.slotCountsIndex = this.values.length;
    this.values.push(0, 0);
  }

  addDevice(device: ProgramDeviceInput): void {
    if (this.finished) {
      throw new Error("ProgramWriter.addDevice: called after finish()");
    }
    const shape = CORE_RECORD_SHAPES.get(device.kindCode);
    if (!shape) {
      throw new Error(`ProgramWriter.addDevice: unknown or unshaped kind code ${device.kindCode}`);
    }
    if (device.pins.length < shape.pins || device.pins.length > shape.pinsMax) {
      throw new Error(
        `ProgramWriter.addDevice: kind code ${device.kindCode} takes ${shape.pins === shape.pinsMax ? shape.pins : `${shape.pins}..${shape.pinsMax}`} pin(s), got ${device.pins.length}`,
      );
    }
    if (device.params.length !== shape.params) {
      throw new Error(
        `ProgramWriter.addDevice: kind code ${device.kindCode} takes ${shape.params} param(s), got ${device.params.length}`,
      );
    }
    const blob = device.blob ?? [];
    const blobOk = "exact" in shape.blob ? blob.length === shape.blob.exact : blob.length >= shape.blob.atLeast;
    if (!blobOk) {
      const expected = "exact" in shape.blob ? `exactly ${shape.blob.exact}` : `at least ${shape.blob.atLeast}`;
      throw new Error(
        `ProgramWriter.addDevice: kind code ${device.kindCode} takes a blob of ${expected} value(s), got ${blob.length}`,
      );
    }
    if (shape.digitalSlots) {
      // blob[0] itself must be finite (checked below with every other blob
      // value); the integer/non-negative shape is specific to it being a
      // slot index, so it's checked here rather than folded into that
      // generic finite loop.
      const slotBase = blob[0]!;
      if (!Number.isInteger(slotBase) || slotBase < 0) {
        throw new Error(
          `ProgramWriter.addDevice: kind code ${device.kindCode}'s blob[0] (digital slot base) must be a non-negative integer, got ${slotBase}`,
        );
      }
      this.pendingDigitalSlotChecks.push({ slotBase, count: shape.digitalSlots });
    }
    if (shape.channelSlots) {
      // Channel slots follow digital slots in the blob (core/README.md,
      // "Slot indices"): blob[1] when this kind also has digital slots,
      // blob[0] otherwise. No W1e kind has both, but the index is derived
      // from the shape rather than hardcoded.
      const channelBlobIndex = shape.digitalSlots ? 1 : 0;
      const channelBase = blob[channelBlobIndex]!;
      if (!Number.isInteger(channelBase) || channelBase < 0) {
        throw new Error(
          `ProgramWriter.addDevice: kind code ${device.kindCode}'s blob[${channelBlobIndex}] (channel slot base) must be a non-negative integer, got ${channelBase}`,
        );
      }
      this.pendingChannelSlotChecks.push({ slotBase: channelBase, count: shape.channelSlots });
    }
    if (!Number.isInteger(device.componentIndex) || device.componentIndex < 0) {
      throw new Error(
        `ProgramWriter.addDevice: componentIndex must be a non-negative integer, got ${device.componentIndex}`,
      );
    }

    const pinValues: number[] = [];
    for (const pin of device.pins) {
      if (!Number.isInteger(pin.node) || pin.node < 0 || pin.node >= this.nodeCount) {
        throw new Error(
          `ProgramWriter.addDevice: pin node ${pin.node} out of range for nodeCount ${this.nodeCount}`,
        );
      }
      for (const [label, hash] of [
        ["openHash", pin.openHash],
        ["deadHash", pin.deadHash],
      ] as const) {
        if (!Number.isInteger(hash) || hash < 0 || hash > U32_MAX) {
          throw new Error(`ProgramWriter.addDevice: ${label} ${hash} must be an integer in [0, 2^32)`);
        }
      }
      const flags = (pin.open ? PIN_FLAG_OPEN : 0) | (pin.openHigh ? PIN_FLAG_OPEN_HIGH : 0);
      pinValues.push(pin.node, flags, pin.openHash, pin.deadHash);
    }

    this.values.push(
      device.kindCode,
      device.componentIndex,
      device.pins.length,
      ...pinValues,
      device.params.length,
      ...device.params,
      blob.length,
      ...blob,
    );
    this.deviceCount++;
  }

  /**
   * Patches deviceCount and the readout slot counts, appends the trailer
   * magic, and returns the finished program. `digitalSlotCount`/
   * `channelSlotCount` are the compiler's own slot-table lengths (compile/
   * index.ts's SlotAllocator) — this class never allocates slots itself.
   */
  finish(digitalSlotCount: number, channelSlotCount: number): Float64Array {
    if (this.finished) {
      throw new Error("ProgramWriter.finish: already finished");
    }
    for (const [label, count] of [
      ["digitalSlotCount", digitalSlotCount],
      ["channelSlotCount", channelSlotCount],
    ] as const) {
      if (!Number.isInteger(count) || count < 0) {
        throw new Error(`ProgramWriter.finish: ${label} must be a non-negative integer, got ${count}`);
      }
    }
    // Now that S is final, check every digital-slot-owning device added so
    // far against it (core/README.md's "Slot indices": `blob[0] + count <=
    // S`) — the belt-and-braces mirror of core/src/program.rs's reader
    // check, on the producer side.
    for (const { slotBase, count } of this.pendingDigitalSlotChecks) {
      if (slotBase + count > digitalSlotCount) {
        throw new Error(
          `ProgramWriter.finish: a device's digital slots [${slotBase}, ${slotBase + count}) exceed digitalSlotCount ${digitalSlotCount}`,
        );
      }
    }
    // The channel-slot twin of the loop above (core/README.md, "Slot
    // indices": `base + count <= C`).
    for (const { slotBase, count } of this.pendingChannelSlotChecks) {
      if (slotBase + count > channelSlotCount) {
        throw new Error(
          `ProgramWriter.finish: a device's channel slots [${slotBase}, ${slotBase + count}) exceed channelSlotCount ${channelSlotCount}`,
        );
      }
    }

    this.finished = true;
    this.values[this.deviceCountIndex] = this.deviceCount;
    this.values[this.slotCountsIndex] = digitalSlotCount;
    this.values[this.slotCountsIndex + 1] = channelSlotCount;
    this.values.push(CORE_FORMAT_MAGIC);
    return Float64Array.from(this.values);
  }
}

/** One decoded device record — decodeProgram's view of a ProgramDeviceInput after the round trip. */
export interface DecodedDevice {
  kindCode: number;
  componentIndex: number;
  pins: DecodedPin[];
  params: number[];
  blob: number[];
}

export interface DecodedProgram {
  version: number;
  nodeCount: number;
  options: readonly ProgramOption[];
  digitalSlotCount: number;
  channelSlotCount: number;
  devices: DecodedDevice[];
}

/**
 * Reads and fully validates a program, mirroring every check the contract
 * requires of the Rust reader (core/README.md's "Validation" bullet: magic,
 * version, counts, each option key known/not repeated/in range, every node
 * index `< N`, flags with no unknown bit, hash states integral and in
 * range, known kind codes, the exact pin/param/blob shape per kind, every
 * param and blob value finite, every slot index in range, the trailer, and
 * no trailing data). This function IS that sentence, spelled out.
 *
 * There is no error-code return here the way there is over the C ABI
 * (core/README.md's `-1` et al. on the Rust side): on the TS side a
 * malformed program throws a plain Error whose message names the exact
 * field and offset that failed, since this function's job is to be a
 * precise diagnostic for tests and tooling, not a hot solver path.
 */
export function decodeProgram(program: Float64Array): DecodedProgram {
  let offset = 0;

  const readValue = (what: string): number => {
    if (offset >= program.length) {
      throw new Error(
        `decodeProgram: truncated program while reading ${what} (offset ${offset}, length ${program.length})`,
      );
    }
    return program[offset++]!;
  };

  const readCount = (what: string): number => {
    const v = readValue(what);
    if (!Number.isInteger(v) || v < 0) {
      throw new Error(`decodeProgram: ${what} must be a non-negative integer, got ${v}`);
    }
    return v;
  };

  const magic = readValue("header magic");
  if (magic !== CORE_FORMAT_MAGIC) {
    throw new Error(
      `decodeProgram: bad header magic ${magic}, expected ${CORE_FORMAT_MAGIC} (0x${CORE_FORMAT_MAGIC.toString(16)})`,
    );
  }

  const version = readCount("version");
  if (version !== CORE_FORMAT_VERSION) {
    throw new Error(`decodeProgram: unsupported version ${version}, expected ${CORE_FORMAT_VERSION}`);
  }

  const nodeCount = readCount("nodeCount");
  if (nodeCount < 1) {
    throw new Error(`decodeProgram: nodeCount must be >= 1, got ${nodeCount}`);
  }

  const deviceCount = readCount("deviceCount");

  const optionCount = readCount("optionCount");
  const options: ProgramOption[] = [];
  const seenOptionKeys = new Set<number>();
  for (let i = 0; i < optionCount; i++) {
    const key = readCount(`option[${i}].key`);
    if (!CORE_OPTION_KEY_VALUES.has(key)) {
      throw new Error(`decodeProgram: option[${i}] has unknown key ${key}`);
    }
    if (seenOptionKeys.has(key)) {
      throw new Error(
        `decodeProgram: option key ${key} (${CORE_OPTION_NAME_BY_KEY.get(key)}) is repeated`,
      );
    }
    seenOptionKeys.add(key);
    const value = readValue(`option[${i}].value`);
    const error = optionValueError(key, value);
    if (error) {
      throw new Error(`decodeProgram: option ${CORE_OPTION_NAME_BY_KEY.get(key)} ${error}`);
    }
    options.push([key, value]);
  }

  const digitalSlotCount = readCount("digitalSlotCount");
  const channelSlotCount = readCount("channelSlotCount");

  const devices: DecodedDevice[] = [];

  for (let d = 0; d < deviceCount; d++) {
    const kindCode = readCount(`device[${d}].kindCode`);
    const kindName = CORE_KIND_NAME_BY_CODE.get(kindCode);
    if (kindName === undefined) {
      throw new Error(`decodeProgram: device[${d}] has unknown kind code ${kindCode}`);
    }
    const shape = CORE_RECORD_SHAPES.get(kindCode);
    if (!shape) {
      // A code with no record shape yet is "well-formed but unknown" per the
      // contract (-2), which on the TS side is exactly the same failure as
      // an outright-unknown code: the reader has nothing to check it against.
      throw new Error(`decodeProgram: device[${d}] (${kindName}) has no known record shape (kind code ${kindCode})`);
    }

    const componentIndex = readCount(`device[${d}].componentIndex`);

    const pinCount = readCount(`device[${d}].pinCount`);
    if (pinCount < shape.pins || pinCount > shape.pinsMax) {
      throw new Error(
        `decodeProgram: device[${d}] (${kindName}) has pinCount ${pinCount}, expected ${shape.pins === shape.pinsMax ? shape.pins : `${shape.pins}..${shape.pinsMax}`}`,
      );
    }
    const pins: DecodedPin[] = [];
    for (let p = 0; p < pinCount; p++) {
      const node = readCount(`device[${d}].pins[${p}].node`);
      if (node >= nodeCount) {
        throw new Error(
          `decodeProgram: device[${d}].pins[${p}].node = ${node} is out of range for nodeCount ${nodeCount}`,
        );
      }
      // `flags > PIN_FLAG_KNOWN_BITS`, not a bitwise `& ~KNOWN_BITS` mask:
      // the mask form coerces through ToInt32 (mod 2^32), so a flags word at
      // or above 2^32 wraps around and silently passes (2^32 -> 0, "no
      // flags"; 2^32+1 -> 1, a spurious OPEN) instead of being rejected the
      // way the Rust reader rejects it (core/src/program.rs's bounded_index
      // compares the raw f64 directly, no modulo). `readCount` above already
      // guarantees `flags` is a non-negative integer, so a plain numeric
      // comparison is exact here with no wraparound at any magnitude.
      const flags = readCount(`device[${d}].pins[${p}].flags`);
      if (flags > PIN_FLAG_KNOWN_BITS) {
        throw new Error(
          `decodeProgram: device[${d}].pins[${p}].flags = ${flags} has an unknown bit set (known bits: 0x${PIN_FLAG_KNOWN_BITS.toString(16)})`,
        );
      }
      const openHash = readValue(`device[${d}].pins[${p}].openHash`);
      const deadHash = readValue(`device[${d}].pins[${p}].deadHash`);
      for (const [label, hash] of [
        ["openHash", openHash],
        ["deadHash", deadHash],
      ] as const) {
        if (!Number.isInteger(hash) || hash < 0 || hash > U32_MAX) {
          throw new Error(
            `decodeProgram: device[${d}].pins[${p}].${label} = ${hash} must be an integer in [0, 2^32)`,
          );
        }
      }
      pins.push({
        node,
        open: (flags & PIN_FLAG_OPEN) !== 0,
        openHigh: (flags & PIN_FLAG_OPEN_HIGH) !== 0,
        openHash,
        deadHash,
      });
    }

    const paramCount = readCount(`device[${d}].paramCount`);
    if (paramCount !== shape.params) {
      throw new Error(
        `decodeProgram: device[${d}] (${kindName}) has paramCount ${paramCount}, expected ${shape.params}`,
      );
    }
    const params: number[] = [];
    for (let p = 0; p < paramCount; p++) {
      const value = readValue(`device[${d}].params[${p}]`);
      if (!Number.isFinite(value)) {
        throw new Error(`decodeProgram: device[${d}].params[${p}] must be finite, got ${value}`);
      }
      params.push(value);
    }

    const blobCount = readCount(`device[${d}].blobCount`);
    const blobOk = "exact" in shape.blob ? blobCount === shape.blob.exact : blobCount >= shape.blob.atLeast;
    if (!blobOk) {
      const expected = "exact" in shape.blob ? `exactly ${shape.blob.exact}` : `at least ${shape.blob.atLeast}`;
      throw new Error(
        `decodeProgram: device[${d}] (${kindName}) has blobCount ${blobCount}, expected ${expected}`,
      );
    }
    const blob: number[] = [];
    for (let b = 0; b < blobCount; b++) {
      const value = readValue(`device[${d}].blob[${b}]`);
      if (!Number.isFinite(value)) {
        throw new Error(`decodeProgram: device[${d}].blob[${b}] must be finite, got ${value}`);
      }
      blob.push(value);
    }

    // A kind that publishes digital readouts (W1c's combinational ICs, so
    // far) carries its first slot in blob[0]; every slot it owns must exist
    // (core/README.md, "Slot indices" — mirrors core/src/program.rs's
    // reader check). `digitalSlotCount` (S) was already read above, before
    // this devices loop.
    if (shape.digitalSlots) {
      const slotBase = blob[0]!;
      if (!Number.isInteger(slotBase) || slotBase < 0) {
        throw new Error(
          `decodeProgram: device[${d}] (${kindName}) has blob[0] (digital slot base) ${slotBase}, expected a non-negative integer`,
        );
      }
      if (slotBase + shape.digitalSlots > digitalSlotCount) {
        throw new Error(
          `decodeProgram: device[${d}] (${kindName})'s digital slots [${slotBase}, ${slotBase + shape.digitalSlots}) exceed digitalSlotCount ${digitalSlotCount}`,
        );
      }
    }

    // The channel-slot twin of the digital check above (core/README.md,
    // "Slot indices"): W1e's `rgb_led`/`bicolor_led`/`seg7_cc`/`seg7_ca`
    // carry their channel base at blob[1] when the kind also has digital
    // slots, blob[0] otherwise (no W1e kind has both).
    if (shape.channelSlots) {
      const channelBlobIndex = shape.digitalSlots ? 1 : 0;
      const channelBase = blob[channelBlobIndex]!;
      if (!Number.isInteger(channelBase) || channelBase < 0) {
        throw new Error(
          `decodeProgram: device[${d}] (${kindName}) has blob[${channelBlobIndex}] (channel slot base) ${channelBase}, expected a non-negative integer`,
        );
      }
      if (channelBase + shape.channelSlots > channelSlotCount) {
        throw new Error(
          `decodeProgram: device[${d}] (${kindName})'s channel slots [${channelBase}, ${channelBase + shape.channelSlots}) exceed channelSlotCount ${channelSlotCount}`,
        );
      }
    }

    devices.push({ kindCode, componentIndex, pins, params, blob });
  }

  const trailer = readValue("trailer magic");
  if (trailer !== CORE_FORMAT_MAGIC) {
    throw new Error(
      `decodeProgram: bad trailer magic ${trailer}, expected ${CORE_FORMAT_MAGIC} (0x${CORE_FORMAT_MAGIC.toString(16)})`,
    );
  }

  if (offset !== program.length) {
    throw new Error(
      `decodeProgram: ${program.length - offset} trailing value(s) after the trailer at offset ${offset}`,
    );
  }

  return { version, nodeCount, options, digitalSlotCount, channelSlotCount, devices };
}
