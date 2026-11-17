/**
 * The W1g synthetic records (core/README.md, "Rating record" and "Thermal
 * record"): code 90 `rating` and code 91 `thermal`, and the two declines
 * that keep the core's slow physics to what classic does.
 *
 * The records come after every component record and after W1f's code 26
 * anchors, so `compileSlowRecords` takes the finished device list and runs
 * after the anchor pass. The declines are per component and run in the
 * component pass with the other reasons (`slowPhysicsDeclines`).
 *
 * Every value is classic's own: `specsForComponent` is what classic's failure
 * pass reads through `ctx.electricalSpecs`, and `thermalProfileForComponent`
 * is `_thermalProfile` without its cache. Nothing here re-derives a catalog
 * or a thermal identity.
 */

import type { SimCircuit } from "../../sim/engine/sim-engine.js";
import { specsForComponent, thermalProfileForComponent } from "../../sim/engine/sim-engine.js";
import { thermalTimeConstantS, type ThermalDeviceProfile } from "../../sim/thermal-physics.js";
import type { ProgramDeviceInput } from "../format.js";
import { CORE_KIND_CODES, CORE_RATING_PARAM, CORE_THERMAL_PARAM } from "../kinds.js";

type SimComponent = SimCircuit["components"][number];

/** The kinds the core heats in W1 (core/README.md, "Thermal record"). Every other profile is W3's. */
export const CORE_THERMAL_KINDS: ReadonlySet<string> = new Set(["resistor", "led"]);

/** Kinds that take a code 90 record: one per resistor, identity or not (Q-O47). */
const RATED_KINDS: ReadonlySet<string> = new Set(["resistor"]);

/**
 * IC records are codes 40-82, the block the README lays out with `vcc`,
 * `gnd` and the IC params; classic scans exactly the components whose specs
 * carry `vcc_range` (SE:6296).
 */
function isIcKind(kind: string): boolean {
  const code = (CORE_KIND_CODES as Readonly<Record<string, number>>)[kind];
  return code !== undefined && code >= 40 && code < 90;
}

/**
 * `rPower`: the resistance classic multiplies `I²` by, `Number(params.resistance
 * ?? 1000)` unfloored (PA:107, Q-O27), or 0 when it is not `> 0`, because
 * classic's power is `resistance > 0 ? I*I*resistance : 0` (R18). A `-Infinity`
 * resistance therefore rates 0 W rather than leave the record non-finite.
 */
export function ratingResistance(component: SimComponent): number {
  const resistance = Number(component.params.resistance ?? 1000);
  return resistance > 0 ? resistance : 0;
}

/** `pMax`: the resolved specs' `p_max` when it is a finite number, else 0, meaning no limit (PA:138). */
export function ratingPowerLimit(component: SimComponent): number {
  const pMax = specsForComponent(component)?.p_max;
  return typeof pMax === "number" && Number.isFinite(pMax) ? pMax : 0;
}

/**
 * The reasons `component` cannot run on the core's slow physics, in the
 * compiler's reason format:
 * - a thermal identity on a kind the core does not heat
 *   (`unsupported-feature:thermal:<id>`);
 * - an IC whose resolved specs lack `vcc_range`
 *   (`unsupported-feature:no-vcc-range:<id>`), which classic never scans for
 *   `output_sag` while the core scans every IC record;
 * - a resistor whose `rPower` is not finite (`malformed:<id>:non-finite-resistance`),
 *   the reason the resistor's own case gives for the same value: the rating
 *   record must be finite whatever order the checks run in.
 */
export function slowPhysicsDeclines(component: SimComponent): string[] {
  const reasons: string[] = [];
  if (!CORE_THERMAL_KINDS.has(component.kind) && thermalProfileForComponent(component) !== undefined) {
    reasons.push(`unsupported-feature:thermal:${component.id}`);
  }
  if (isIcKind(component.kind) && !specsForComponent(component)?.vcc_range) {
    reasons.push(`unsupported-feature:no-vcc-range:${component.id}`);
  }
  if (RATED_KINDS.has(component.kind) && !Number.isFinite(ratingResistance(component))) {
    reasons.push(`malformed:${component.id}:non-finite-resistance`);
  }
  return reasons;
}

/** One synthetic record, with the `devices` entry the compiler records for it. */
export interface CompiledSlowRecord {
  kind: "rating" | "thermal";
  /** The rated or heated device's component id. */
  componentId: string;
  /** The rated or heated device's record index (the record's `device` param). */
  device: number;
  input: ProgramDeviceInput;
  /** A `thermal` record's profile: `getThermalState` composes the readout from it (core/README.md, Readouts). */
  profile?: ThermalDeviceProfile;
}

/**
 * The code 90 and 91 records for `devices`, the compiler's record list so far
 * (components, then anchors), in the reader's order: ascending `device`, 90
 * before 91 for one device. Checked with `checkSlowRecords` before it returns.
 */
export function compileSlowRecords(
  circuit: SimCircuit,
  devices: ReadonlyArray<{ componentId: string; kind: string }>,
): CompiledSlowRecord[] {
  const indexById = new Map<string, number>();
  circuit.components.forEach((component, index) => indexById.set(component.id, index));

  const out: CompiledSlowRecord[] = [];
  devices.forEach((device, deviceIndex) => {
    if (!RATED_KINDS.has(device.kind) && !CORE_THERMAL_KINDS.has(device.kind)) return;
    const componentIndex = indexById.get(device.componentId);
    if (componentIndex === undefined) {
      throw new Error(`compileSlowRecords: device ${deviceIndex} names no component "${device.componentId}"`);
    }
    const component = circuit.components[componentIndex]!;

    if (RATED_KINDS.has(device.kind)) {
      const params = new Array<number>(3);
      params[CORE_RATING_PARAM.device] = deviceIndex;
      params[CORE_RATING_PARAM.pMax] = ratingPowerLimit(component);
      params[CORE_RATING_PARAM.rPower] = ratingResistance(component);
      out.push({
        kind: "rating",
        componentId: device.componentId,
        device: deviceIndex,
        input: { kindCode: CORE_KIND_CODES.rating, componentIndex, pins: [], params },
      });
    }

    const profile = CORE_THERMAL_KINDS.has(device.kind) ? thermalProfileForComponent(component) : undefined;
    if (profile) {
      const shutdown = profile.shutdown;
      const params = new Array<number>(6);
      params[CORE_THERMAL_PARAM.device] = deviceIndex;
      params[CORE_THERMAL_PARAM.rThetaJA] = profile.network.rThetaJA_CPerW;
      params[CORE_THERMAL_PARAM.tauS] = thermalTimeConstantS(profile.network);
      params[CORE_THERMAL_PARAM.hasShutdown] = shutdown ? 1 : 0;
      params[CORE_THERMAL_PARAM.tripC] = shutdown ? shutdown.tripTemperatureC : 0;
      params[CORE_THERMAL_PARAM.restartC] = shutdown ? shutdown.restartTemperatureC : 0;
      out.push({
        kind: "thermal",
        componentId: device.componentId,
        device: deviceIndex,
        input: { kindCode: CORE_KIND_CODES.thermal, componentIndex, pins: [], params },
        profile,
      });
    }
  });

  checkSlowRecords(
    devices.map((device) => device.kind),
    out.map((record) => record.input),
  );
  return out;
}

/**
 * The Rust reader's rules for codes 90 and 91 (core/README.md, "Reader rules
 * for codes 90 and 91"), as thrown errors: `deviceKinds` is the kind of every
 * record before them, `records` the synthetic records in program order. A
 * failure here is a compiler bug, never a fact about the user's circuit.
 */
export function checkSlowRecords(deviceKinds: readonly string[], records: readonly ProgramDeviceInput[]): void {
  const fail = (index: number, why: string): never => {
    throw new Error(`checkSlowRecords: synthetic record ${index}: ${why}`);
  };
  let previous: { device: number; code: number } | null = null;
  const rated = new Set<number>();

  records.forEach((record, index) => {
    const code = record.kindCode;
    if (code !== CORE_KIND_CODES.rating && code !== CORE_KIND_CODES.thermal) {
      fail(index, `kind code ${code} is not 90 or 91`);
    }
    if (record.pins.length !== 0) fail(index, `takes no pins, got ${record.pins.length}`);
    if ((record.blob?.length ?? 0) !== 0) fail(index, `takes no blob, got ${record.blob!.length}`);
    const expectedParams = code === CORE_KIND_CODES.rating ? 3 : 6;
    if (record.params.length !== expectedParams) {
      fail(index, `code ${code} takes ${expectedParams} params, got ${record.params.length}`);
    }
    record.params.forEach((value, p) => {
      if (!Number.isFinite(value)) fail(index, `param ${p} must be finite, got ${value}`);
    });

    const device = record.params[0]!;
    if (!Number.isInteger(device) || device < 0 || device >= deviceKinds.length) {
      fail(index, `device ${device} names no earlier record`);
    }
    const kind = deviceKinds[device]!;
    if (kind === "section_anchor" || kind === "rating" || kind === "thermal") {
      fail(index, `device ${device} is a synthetic record (${kind})`);
    }
    if (code === CORE_KIND_CODES.rating && !RATED_KINDS.has(kind)) {
      fail(index, `a rating record names device ${device}, a ${kind}, not a resistor`);
    }
    if (code === CORE_KIND_CODES.thermal && !CORE_THERMAL_KINDS.has(kind)) {
      fail(index, `a thermal record names device ${device}, a ${kind}, not a resistor or led`);
    }

    // Ascending device, 90 before 91: the pair (device, code) strictly
    // increases, which also allows at most one of each code per device.
    if (previous && (device < previous.device || (device === previous.device && code <= previous.code))) {
      fail(index, `(device ${device}, code ${code}) follows (device ${previous.device}, code ${previous.code})`);
    }
    previous = { device, code };

    if (code === CORE_KIND_CODES.rating) {
      rated.add(device);
    } else {
      const rThetaJA = record.params[CORE_THERMAL_PARAM.rThetaJA]!;
      const tauS = record.params[CORE_THERMAL_PARAM.tauS]!;
      const hasShutdown = record.params[CORE_THERMAL_PARAM.hasShutdown]!;
      const tripC = record.params[CORE_THERMAL_PARAM.tripC]!;
      const restartC = record.params[CORE_THERMAL_PARAM.restartC]!;
      if (!(rThetaJA > 0)) fail(index, `rThetaJA must be > 0, got ${rThetaJA}`);
      if (!(tauS > 0)) fail(index, `tauS must be > 0, got ${tauS}`);
      if (hasShutdown !== 0 && hasShutdown !== 1) fail(index, `hasShutdown must be 0 or 1, got ${hasShutdown}`);
      if (hasShutdown === 0 && (tripC !== 0 || restartC !== 0)) {
        fail(index, `tripC and restartC must be 0 without a shutdown, got ${tripC} and ${restartC}`);
      }
      if (hasShutdown === 1 && !(restartC < tripC)) {
        fail(index, `restartC ${restartC} must be below tripC ${tripC}`);
      }
    }
  });

  deviceKinds.forEach((kind, device) => {
    if (RATED_KINDS.has(kind) && !rated.has(device)) {
      throw new Error(`checkSlowRecords: resistor device ${device} has no rating record`);
    }
  });
}
