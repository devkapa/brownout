/**
 * The W1g facade's pure half (core/README.md, "Readouts", W1g): decoding
 * `bo_engine_records` buffers into classic's `SimFailure` objects and keys,
 * the message builders, the PTC set, the thermal readout and the static
 * warnings with their announce queue. No wasm call happens here: engine.ts
 * collects the buffers and the device state and hands them over.
 *
 * Every string is classic's. The message builders copy classic's template
 * literals character for character (G10), because classic builds them
 * inline in its device models; the thermal readout and the static warnings
 * call classic's own exported functions.
 */

import type { Net } from "../sim/engine/graph.js";
import {
  missingFlybackWarning,
  type FlybackTopology,
  type SimCircuit,
  type SimFailure,
  type SimFailureKind,
  type SimWarning,
  type SimWarningCode,
  type ThermalRuntimeReadout,
} from "../sim/engine/sim-engine.js";
import { stepThermalDevice, type ThermalDeviceProfile } from "../sim/thermal-physics.js";

// ─── Records ────────────────────────────────────────────────────────────────

/** f64 values per record: `device, code, pin, since, value, limit, aux` (C ABI, `bo_engine_records`). */
export const CORE_RECORD_STRIDE = 7;

/** `bo_engine_records` channels. */
export const CORE_RECORD_CHANNEL = { failures: 0, warnings: 1, ptcTripped: 2, latchLog: 3 } as const;

/**
 * The failure kinds the core records. Classic's other kinds stay off this
 * list: `supply_overvoltage` (0.6.8) latches on an MCU board's supply, and
 * MCU boards never compile to the core (unsupported-kind), so no core record
 * names one. Keeping the type narrow keeps `failureFromRecord`'s switch
 * exhaustive when classic adds a kind.
 */
export type CoreFailureKind = Extract<SimFailureKind, "resistor_overload" | "fuse_tripped" | "led_failed" | "output_sag">;

/** Failure codes on channels 0 and 3. */
export const CORE_FAILURE_KIND_BY_CODE: ReadonlyMap<number, CoreFailureKind> = new Map([
  [1, "resistor_overload"],
  [2, "fuse_tripped"],
  [3, "led_failed"],
  [4, "output_sag"],
]);

/** Warning codes on channel 1. `missing_flyback` is reserved: the facade derives it (W1g point 11). */
export const CORE_WARNING_CODE_BY_CODE: ReadonlyMap<number, SimWarningCode> = new Map([
  [1, "transistor_overcurrent"],
  [2, "missing_flyback"],
]);

/** The one PTC code on channel 2. */
export const CORE_PTC_TRIPPED_CODE = 1;

/** One decoded record. `pin` is `-1` for none; `aux` is the `output_sag` direction (1 HIGH, 0 LOW). */
export interface CoreSlowRecord {
  device: number;
  code: number;
  pin: number;
  since: number;
  value: number;
  limit: number;
  aux: number;
}

/** Splits a collected buffer into records. A length off the stride is an ABI drift, so it throws. */
export function decodeRecords(buffer: ArrayLike<number>): CoreSlowRecord[] {
  if (buffer.length % CORE_RECORD_STRIDE !== 0) {
    throw new Error(`decodeRecords: buffer length ${buffer.length} is not a multiple of ${CORE_RECORD_STRIDE}`);
  }
  const out: CoreSlowRecord[] = [];
  for (let i = 0; i < buffer.length; i += CORE_RECORD_STRIDE) {
    out.push({
      device: buffer[i]!,
      code: buffer[i + 1]!,
      pin: buffer[i + 2]!,
      since: buffer[i + 3]!,
      value: buffer[i + 4]!,
      limit: buffer[i + 5]!,
      aux: buffer[i + 6]!,
    });
  }
  return out;
}

/**
 * What a record's `device` and `pin` indices name: per device record, its
 * component id and its pin ids in pin-record order (for an IC, `vcc` and
 * `gnd` first). The compiler keeps both.
 */
export interface RecordNames {
  readonly devices: ReadonlyArray<{ readonly componentId: string; readonly pinIds?: readonly string[] }>;
}

function deviceIdOf(names: RecordNames, record: CoreSlowRecord): string {
  const device = names.devices[record.device];
  if (!device) throw new Error(`records: device ${record.device} is not a record of this program`);
  return device.componentId;
}

function pinIdOf(names: RecordNames, record: CoreSlowRecord): string {
  const pinId = names.devices[record.device]?.pinIds?.[record.pin];
  if (pinId === undefined) {
    throw new Error(`records: device ${record.device} has no pin ${record.pin}`);
  }
  return pinId;
}

// ─── Keys ───────────────────────────────────────────────────────────────────

/** Classic's `failureKey` (SE:947): `${kind}\0${componentId}\0${pinId}`, `pinId` `""` when absent. */
export function failureKey(kind: SimFailureKind, componentId: string, pinId = ""): string {
  return `${kind}\x00${componentId}\x00${pinId}`;
}

/** Classic's `warningKey` (SE:975), the same shape as `failureKey`. */
export function warningKey(code: SimWarningCode, componentId: string, pinId = ""): string {
  return `${code}\x00${componentId}\x00${pinId}`;
}

// ─── Messages (classic's templates, character for character) ──────────────

/** `resistor_overload` for a `resistor` (PA:155; `commitResistiveOverloadStress`, devices/passives.ts). */
export function resistorOverloadMessage(componentId: string, value: number, limit: number): string {
  return `${componentId} dissipated ${value.toFixed(3)} W through a ${limit.toFixed(3)} W rating long enough to fail open.`;
}

/** `fuse_tripped` (PA:731; `fuseModel.updateFailures`, devices/passives.ts). */
export function fuseTrippedMessage(componentId: string, value: number, limit: number): string {
  return `${componentId} carried ${(value * 1000).toFixed(1)} mA through a ${(limit * 1000).toFixed(1)} mA fuse rating and tripped open.`;
}

/** `led_failed` (SC:508; `commitLedOvercurrentStress`, devices/semiconductors.ts). */
export function ledFailedMessage(componentId: string, value: number, limit: number): string {
  return `${componentId} exceeded its ${(limit * 1000).toFixed(1)} mA LED rating with ${(value * 1000).toFixed(1)} mA long enough to fail open.`;
}

/** `output_sag` (SE:6333; the failure pass's sag scan, sim-engine.ts). `high` is the commanded direction. */
export function outputSagMessage(componentId: string, pinId: string, high: boolean, actual: number): string {
  return `${componentId}.${pinId} is commanded ${high ? "HIGH" : "LOW"} but sits at ${actual.toFixed(2)} V under the present load. The finite-strength output is outside its own package family's ${high ? "HIGH" : "LOW"} input-guarantee band; connected receivers may have different requirements.`;
}

// ─── Failures ───────────────────────────────────────────────────────────────

/**
 * One failure record as classic's `SimFailure`, in classic's key order:
 * `componentId, kind, since, value, limit, message`, with `pinId` after
 * `kind` on `output_sag` only (classic sets it nowhere else, SE:6328).
 */
export function failureFromRecord(record: CoreSlowRecord, names: RecordNames): { key: string; failure: SimFailure } {
  const kind = CORE_FAILURE_KIND_BY_CODE.get(record.code);
  if (kind === undefined) throw new Error(`records: unknown failure code ${record.code}`);
  const componentId = deviceIdOf(names, record);
  const { since, value, limit } = record;
  switch (kind) {
    case "resistor_overload":
      return {
        key: failureKey(kind, componentId),
        failure: { componentId, kind, since, value, limit, message: resistorOverloadMessage(componentId, value, limit) },
      };
    case "fuse_tripped":
      return {
        key: failureKey(kind, componentId),
        failure: { componentId, kind, since, value, limit, message: fuseTrippedMessage(componentId, value, limit) },
      };
    case "led_failed":
      return {
        key: failureKey(kind, componentId),
        failure: { componentId, kind, since, value, limit, message: ledFailedMessage(componentId, value, limit) },
      };
    case "output_sag": {
      const pinId = pinIdOf(names, record);
      const high = record.aux === 1;
      return {
        key: failureKey(kind, componentId, pinId),
        failure: {
          componentId,
          kind,
          pinId,
          since,
          value,
          limit,
          message: outputSagMessage(componentId, pinId, high, value),
        },
      };
    }
  }
}

/** Channel 0 as classic's `getFailures()`: a fresh object in record (latch) order. */
export function failuresFromRecords(records: readonly CoreSlowRecord[], names: RecordNames): Record<string, SimFailure> {
  const out: Record<string, SimFailure> = {};
  for (const record of records) {
    const { key, failure } = failureFromRecord(record, names);
    out[key] = failure;
  }
  return out;
}

/** Channel 3, the latch log, as a list of failures, each with its latch's `since`. */
export function failureListFromRecords(records: readonly CoreSlowRecord[], names: RecordNames): SimFailure[] {
  return records.map((record) => failureFromRecord(record, names).failure);
}

/** Channel 2 as classic's `getPtcTripped()`: a fresh set of component ids. */
export function ptcTrippedFromRecords(records: readonly CoreSlowRecord[], names: RecordNames): Set<string> {
  const out = new Set<string>();
  for (const record of records) {
    if (record.code !== CORE_PTC_TRIPPED_CODE) throw new Error(`records: unknown PTC code ${record.code}`);
    out.add(deviceIdOf(names, record));
  }
  return out;
}

// ─── Thermal readout ────────────────────────────────────────────────────────

/** What `getThermalState` reads from `bo_engine_device_state` for a device with a code 91 record. */
export interface ThermalDeviceReading {
  temperatureC: number;
  dissipatedPowerW: number;
  thermalShutdown: boolean;
}

/**
 * Classic's `ThermalRuntimeReadout` for one heated device, composed through
 * classic's own `stepThermalDevice` at `dt = 0` (core/README.md, Readouts,
 * `getThermalState`): with no elapsed time it returns `T` unchanged and
 * computes the target, the derating, the per-state warnings and
 * `withinContinuousLimits` exactly as classic's step does, so every string
 * is classic's. The key order is classic's stored state (SE:6394-6403) plus
 * `label` (`getThermalState`, SE:2514).
 */
export function thermalReadout(
  profile: ThermalDeviceProfile,
  catalogUid: string,
  reading: ThermalDeviceReading,
  ambientC: number,
): ThermalRuntimeReadout {
  const stepped = stepThermalDevice(
    profile,
    { temperatureC: reading.temperatureC, thermalShutdown: reading.thermalShutdown },
    { ambientTemperatureC: ambientC, dissipatedPowerW: reading.dissipatedPowerW, dtSeconds: 0 },
  );
  return {
    temperatureC: reading.temperatureC,
    thermalShutdown: reading.thermalShutdown,
    catalogUid,
    profileId: profile.id,
    dissipatedPowerW: reading.dissipatedPowerW,
    targetTemperatureC: stepped.targetTemperatureC,
    allowedPowerW: stepped.derating.allowedPowerW,
    withinContinuousLimits: stepped.withinContinuousLimits,
    warnings: [...new Set([...profile.warnings, ...stepped.warnings])],
    label: profile.label,
  };
}

// ─── Warnings ───────────────────────────────────────────────────────────────

/** The loaded graph as classic's `_evaluateStaticWarnings` builds it, from `buildNets`'s nets. */
export function flybackTopology(circuit: SimCircuit, nets: readonly Net[]): FlybackTopology {
  // Built the way classic builds `_componentById` and `_pinToNetIndex`
  // (SE:4968, 5045), so a later duplicate wins in both, as there.
  const kindById = new Map(circuit.components.map((component) => [component.id, component.kind]));
  const netByPin = new Map<string, string>();
  for (const net of nets) {
    for (const [componentId, pinId] of net.pins) netByPin.set(`${componentId}\0${pinId}`, net.id);
  }
  const pinsByNet = new Map(nets.map((net) => [net.id, net.pins]));
  return {
    netIdForPin: (componentId, pinId) => netByPin.get(`${componentId}\0${pinId}`) ?? null,
    pinsOnNet: (netId) => pinsByNet.get(netId) ?? [],
    kindOf: (componentId) => kindById.get(componentId) ?? "",
  };
}

/**
 * The facade's warnings and their announce queue, with classic's semantics
 * (`_latchWarning`, `_pruneUnannouncedWarnings`, `takeNewWarnings`,
 * `_evaluateStaticWarnings`; SE:5719-5726, 2488, 5809): a warning is queued
 * once when it latches, a still-true one keeps its `since` and is not queued
 * again, and a key that is gone leaves the queue with it.
 */
export class WarningLedger {
  private readonly warnings = new Map<string, SimWarning>();
  private unannounced: string[] = [];

  /** `_latchWarning`: a no-op for a key already latched. */
  latch(key: string, warning: SimWarning): void {
    if (this.warnings.has(key)) return;
    this.warnings.set(key, warning);
    this.unannounced.push(key);
  }

  /**
   * `_evaluateStaticWarnings`: re-derives `missing_flyback` for every
   * component, latching a new condition with `since`, dropping one that no
   * longer holds, then pruning the queue to the map.
   */
  evaluateStatic(circuit: SimCircuit, nets: readonly Net[], since: number): void {
    const topology = flybackTopology(circuit, nets);
    for (const component of circuit.components) {
      const key = warningKey("missing_flyback", component.id);
      const warning = missingFlybackWarning(component, since, topology);
      if (warning) {
        this.latch(key, warning);
      } else {
        this.warnings.delete(key);
      }
    }
    this.prune();
  }

  /** `_pruneUnannouncedWarnings`. */
  prune(): void {
    this.unannounced = this.unannounced.filter((key) => this.warnings.has(key));
  }

  /** `resetFailures`'s first half: every warning and the queue go. The caller re-derives the static ones. */
  clear(): void {
    this.warnings.clear();
    this.unannounced = [];
  }

  /** `getWarnings()`: a fresh object in latch order. */
  all(): Record<string, SimWarning> {
    return Object.fromEntries([...this.warnings].map(([key, warning]) => [key, { ...warning }]));
  }

  /** A deep copy, for `CoreEngine.saveState` (W2a): the warnings and the announce queue as they stand. */
  copy(): WarningLedger {
    const out = new WarningLedger();
    for (const [key, warning] of this.warnings) out.warnings.set(key, { ...warning });
    out.unannounced = [...this.unannounced];
    return out;
  }

  /** `takeNewWarnings()`: the queued warnings in latch order, then an empty queue. */
  takeNew(): SimWarning[] {
    const out: SimWarning[] = [];
    for (const key of this.unannounced) {
      const warning = this.warnings.get(key);
      if (warning) out.push({ ...warning });
    }
    this.unannounced = [];
    return out;
  }
}
