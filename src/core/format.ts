/**
 * "Program format, version 1" — the wire format the TS compiler
 * (compile.ts) emits and the Rust W0 kernel reads (core/README.md, "Program
 * format, version 1"). This file is the TS half of that contract: the
 * format constants, a small writer used by compile.ts, and a decoder that
 * performs every validation the contract requires of the Rust reader.
 *
 * The decoder exists for two reasons: a Float64Array is otherwise opaque to
 * a test, and it doubles as executable documentation of the exact layout —
 * the prose in core/README.md and the checks below must never drift, since
 * the Rust reader is written against the README's table, not against this
 * file.
 *
 * Every value in a program is an f64. Counts and indices are integers
 * carried as exact doubles; every count/index this format ever produces is
 * far below 2^53, so there is no precision loss doing that.
 *
 * Layout (core/README.md):
 *   header   MAGIC, VERSION, nodeCount N, deviceCount D
 *   devices  D records, in component order:
 *              kindCode, componentIndex, pinCount, pin x pinCount,
 *              paramCount, param x paramCount, blobCount, blob x blobCount
 *   trailer  MAGIC
 */

/** "BOC1" read as one big-endian u32. Stamped at both the start and the end of every program. */
export const CORE_FORMAT_MAGIC = 0x424f4331;

/**
 * The only version this package's TS and Rust halves understand. Both sides
 * ship in one npm package, so there is no cross-version compatibility to
 * preserve: a layout change bumps this constant on both sides in the same
 * commit, and decodeProgram rejects anything else outright rather than
 * trying to interpret an old or new layout.
 */
export const CORE_FORMAT_VERSION = 1;

/**
 * Kind codes, "Kind codes (v1)" table in core/README.md. A code is part of
 * the wire format, not an implementation detail of either side: adding a
 * device kind to the format means adding a code here (and bumping
 * CORE_FORMAT_VERSION only if it changes an existing kind's record layout).
 */
export const CORE_KIND_CODE = {
  resistor: 1,
  voltage_source: 2,
} as const;

export type CoreKindName = keyof typeof CORE_KIND_CODE;

/** Reverse lookup: kind code -> kind name. Used by decodeProgram's error messages and by debugging tools. */
export const CORE_KIND_NAME_BY_CODE: ReadonlyMap<number, CoreKindName> = new Map(
  (Object.keys(CORE_KIND_CODE) as CoreKindName[]).map((name) => [CORE_KIND_CODE[name], name]),
);

/**
 * Pin and parameter counts fixed by the "Kind codes (v1)" table's Pins and
 * Params columns. Both are part of the wire format: a resistor record with
 * three pins is exactly as malformed as one with two params.
 */
const CORE_KIND_PIN_COUNT: ReadonlyMap<number, number> = new Map([
  [CORE_KIND_CODE.resistor, 2],
  [CORE_KIND_CODE.voltage_source, 2],
]);
const CORE_KIND_PARAM_COUNT: ReadonlyMap<number, number> = new Map([
  [CORE_KIND_CODE.resistor, 1],
  [CORE_KIND_CODE.voltage_source, 1],
]);

/** One device record's input to ProgramWriter.addDevice, before it is flattened into the Float64Array. */
export interface ProgramDeviceInput {
  kindCode: number;
  /** Index into the source circuit's `components` array (contract: "the index in circuit.components"). */
  componentIndex: number;
  /** Node indices, positional per the kind's "Pins (in order)" column. */
  pins: readonly number[];
  /** Resolved parameter values, positional per the kind's "Params" column. */
  params: readonly number[];
  /** ROM/RAM/pin-map payload. Always empty in W0; from W1 it carries per-kind blob data. */
  blob?: readonly number[];
}

/**
 * Builds one program: header, then device records in the order addDevice is
 * called (which must be component order — the caller's job, not this
 * class's), then trailer.
 *
 * deviceCount is not known until every device has been added, so it is
 * written as a zero placeholder by the constructor and patched by finish().
 * That is the only two-pass part of an otherwise append-only writer.
 *
 * addDevice validates each record against the fixed per-kind pin/param
 * counts and against this program's own nodeCount. This is a belt-and-
 * braces check on the producer side, not a substitute for decodeProgram (or
 * the Rust reader): the source of truth for "is this program well-formed"
 * is the reader, because that is what actually has to survive an untrusted
 * or hand-built Float64Array.
 */
export class ProgramWriter {
  private readonly values: number[];
  private readonly nodeCount: number;
  private deviceCount = 0;
  private finished = false;

  constructor(nodeCount: number) {
    if (!Number.isInteger(nodeCount) || nodeCount < 1) {
      throw new Error(`ProgramWriter: nodeCount must be an integer >= 1, got ${nodeCount}`);
    }
    this.nodeCount = nodeCount;
    // Header: MAGIC, VERSION, nodeCount, deviceCount. The deviceCount slot
    // (index 3) is a placeholder patched by finish().
    this.values = [CORE_FORMAT_MAGIC, CORE_FORMAT_VERSION, nodeCount, 0];
  }

  addDevice(device: ProgramDeviceInput): void {
    if (this.finished) {
      throw new Error("ProgramWriter.addDevice: called after finish()");
    }
    const expectedPins = CORE_KIND_PIN_COUNT.get(device.kindCode);
    const expectedParams = CORE_KIND_PARAM_COUNT.get(device.kindCode);
    if (expectedPins === undefined || expectedParams === undefined) {
      throw new Error(`ProgramWriter.addDevice: unknown kind code ${device.kindCode}`);
    }
    if (device.pins.length !== expectedPins) {
      throw new Error(
        `ProgramWriter.addDevice: kind code ${device.kindCode} takes ${expectedPins} pin(s), got ${device.pins.length}`,
      );
    }
    if (device.params.length !== expectedParams) {
      throw new Error(
        `ProgramWriter.addDevice: kind code ${device.kindCode} takes ${expectedParams} param(s), got ${device.params.length}`,
      );
    }
    for (const node of device.pins) {
      if (!Number.isInteger(node) || node < 0 || node >= this.nodeCount) {
        throw new Error(
          `ProgramWriter.addDevice: pin node ${node} out of range for nodeCount ${this.nodeCount}`,
        );
      }
    }
    if (!Number.isInteger(device.componentIndex) || device.componentIndex < 0) {
      throw new Error(
        `ProgramWriter.addDevice: componentIndex must be a non-negative integer, got ${device.componentIndex}`,
      );
    }
    const blob = device.blob ?? [];
    this.values.push(
      device.kindCode,
      device.componentIndex,
      device.pins.length,
      ...device.pins,
      device.params.length,
      ...device.params,
      blob.length,
      ...blob,
    );
    this.deviceCount++;
  }

  /** Patches the header's deviceCount, appends the trailer magic, and returns the finished program. */
  finish(): Float64Array {
    if (this.finished) {
      throw new Error("ProgramWriter.finish: already finished");
    }
    this.finished = true;
    this.values[3] = this.deviceCount;
    this.values.push(CORE_FORMAT_MAGIC);
    return Float64Array.from(this.values);
  }
}

/** One decoded device record — decodeProgram's view of a ProgramDeviceInput after the round trip. */
export interface DecodedDevice {
  kindCode: number;
  componentIndex: number;
  pins: number[];
  params: number[];
  blob: number[];
}

export interface DecodedProgram {
  version: number;
  nodeCount: number;
  devices: DecodedDevice[];
}

/**
 * Reads and fully validates a program, mirroring every check the contract
 * requires of the Rust reader (core/README.md: "The reader validates
 * everything: magic, version, counts, every node index < N, known kind
 * codes, the exact pin and param counts per kind, every param finite, the
 * trailer, and no trailing data."). This function IS that sentence, spelled
 * out.
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
  const devices: DecodedDevice[] = [];

  for (let d = 0; d < deviceCount; d++) {
    const kindCode = readCount(`device[${d}].kindCode`);
    const kindName = CORE_KIND_NAME_BY_CODE.get(kindCode);
    if (kindName === undefined) {
      throw new Error(`decodeProgram: device[${d}] has unknown kind code ${kindCode}`);
    }

    const componentIndex = readCount(`device[${d}].componentIndex`);

    const pinCount = readCount(`device[${d}].pinCount`);
    const expectedPinCount = CORE_KIND_PIN_COUNT.get(kindCode)!;
    if (pinCount !== expectedPinCount) {
      throw new Error(
        `decodeProgram: device[${d}] (${kindName}) has pinCount ${pinCount}, expected ${expectedPinCount}`,
      );
    }
    const pins: number[] = [];
    for (let p = 0; p < pinCount; p++) {
      const node = readCount(`device[${d}].pins[${p}]`);
      if (node >= nodeCount) {
        throw new Error(
          `decodeProgram: device[${d}].pins[${p}] = ${node} is out of range for nodeCount ${nodeCount}`,
        );
      }
      pins.push(node);
    }

    const paramCount = readCount(`device[${d}].paramCount`);
    const expectedParamCount = CORE_KIND_PARAM_COUNT.get(kindCode)!;
    if (paramCount !== expectedParamCount) {
      throw new Error(
        `decodeProgram: device[${d}] (${kindName}) has paramCount ${paramCount}, expected ${expectedParamCount}`,
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
    const blob: number[] = [];
    for (let b = 0; b < blobCount; b++) {
      blob.push(readValue(`device[${d}].blob[${b}]`));
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

  return { version, nodeCount, devices };
}
