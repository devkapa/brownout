/**
 * Unit coverage for compile/sources.ts: the W1f source kinds (`battery_pack`,
 * `bench_psu`, `clock`, `clock_gen`, `pulse_source`, `pulse_gen`, codes 20-25)
 * and the synthetic `section_anchor` records (code 26), compiled to v3
 * program records per core/README.md's "Source records" and the sections it
 * points to.
 *
 * Every expected value comes from classic: its exported battery-physics
 * functions and model (`batteryModelForComponent`), the values a loaded
 * `SimEngine` publishes (`getBatteryState`, `getNetV`, `digitalState`,
 * `getIcState`, `isolatedSectionAnchorRows`), or a classic probe the README
 * cites (P3, Q4). The record is checked for being sufficient: the README's
 * formulas, evaluated from the compiled params and blob in the operation
 * order Rust uses, must reproduce classic's own numbers bit for bit.
 *
 * These kinds are in CORE_IMPLEMENTED_KINDS since W1f's Rust side landed
 * (F1 round 1). The calls here still pass `WITH_SOURCES`, which now equals
 * CORE_IMPLEMENTED_KINDS, so a compile here and through the default set
 * give the same records.
 */

import { describe, expect, it } from "vitest";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { SimEngine, batteryModelForComponent } from "../../src/sim/engine/sim-engine.js";
import { getDeviceModel } from "../../src/sim/engine/device-registry.js";
import { clockVoltage, pulseVoltage } from "../../src/sim/engine/digital.js";
import {
  BATTERY_PROFILES_BY_CATALOG_UID,
  batteryOpenCircuitVoltageV,
  batteryOperatingPoint,
  type BatteryPhysicsProfile,
} from "../../src/sim/battery-physics.js";
import { compileCircuit, type CompiledCircuit } from "../../src/core/compile/index.js";
import { findSectionAnchors } from "../../src/core/compile/sources.js";
import { buildCoreNodes, corePinKey } from "../../src/core/compile/topology.js";
import { CORE_KIND_CODE, ProgramWriter, decodeProgram, type DecodedDevice } from "../../src/core/format.js";
import {
  CORE_BATTERY_PARAM,
  CORE_BATTERY_PROFILE_CODE,
  CORE_BENCH_PSU_PARAM,
  CORE_CLOCK_PARAM,
  CORE_IMPLEMENTED_KINDS,
  CORE_PULSE_PARAM,
  CORE_RECORD_SHAPES,
} from "../../src/core/kinds.js";

// ---- small circuit-building helpers, local to this file --------------------

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

function circuit(
  components: SimComponent[],
  wires: SimCircuit["wires"] = [],
  environment?: SimCircuit["environment"],
): SimCircuit {
  return { components, wires, ...(environment ? { environment } : {}) };
}

const SOURCE_KINDS = ["battery_pack", "bench_psu", "clock", "clock_gen", "pulse_source", "pulse_gen"] as const;
const WITH_SOURCES = new Set<string>([...CORE_IMPLEMENTED_KINDS, ...SOURCE_KINDS]);

function compileOk(c: SimCircuit): CompiledCircuit {
  const result = compileCircuit(c, WITH_SOURCES);
  if (!result.ok) throw new Error(`expected ok:true, got reasons: ${result.reasons.join(", ")}`);
  return result.compiled;
}

function compileDeclined(c: SimCircuit): string[] {
  const result = compileCircuit(c, WITH_SOURCES);
  if (result.ok) throw new Error("expected ok:false");
  return result.reasons;
}

/** A source `id` (pins `pins`, the first two positional) into a 1 kOhm load and back, the usual one-supply loop. */
function loop(source: SimComponent, environment?: SimCircuit["environment"]): SimCircuit {
  const [plus, minus] = source.pins.map((pin) => pin.id) as [string, string];
  return circuit(
    [source, part("r1", "resistor", ["a", "b"], { resistance: 1000 })],
    [wire(source.id, plus, "r1", "a"), wire("r1", "b", source.id, minus)],
    environment,
  );
}

/** The one record of `kind` in the compiled program, decoded. */
function recordOf(compiled: CompiledCircuit, kind: keyof typeof CORE_KIND_CODE): DecodedDevice {
  const found = decodeProgram(compiled.program).devices.filter((device) => device.kindCode === CORE_KIND_CODE[kind]);
  expect(found).toHaveLength(1);
  return found[0]!;
}

// ---- classic reference helpers ---------------------------------------------

/** What `SimEngine._batteryOperatingPoint` returns beyond the operating point itself (reached through a test-only cast, compile.test.ts's own convention). */
type ClassicBattery = {
  profile: BatteryPhysicsProfile;
  nominalVoltageScale: number;
  referenceInternalResistanceOhm: number | undefined;
};

function loadClassic(c: SimCircuit): SimEngine {
  const engine = new SimEngine();
  engine.load(c);
  return engine;
}

/**
 * The README's rail formulas for a battery record, in the operation order
 * Rust uses: `R = (rBase * dep) * tempFactor` with `dep = 1 + depA *
 * (1 - soc)^depP / (soc + depK)`, and `OCV = table(soc) * ocvScale`, the table
 * read by the README's rule (`soc <= soc_0` gives `v_0`, the first point with
 * `soc <= soc_i` bounds the segment, a zero-width segment gives `v_i`) and
 * interpolated with `batteryOpenCircuitVoltageV`'s own expression,
 * `left.v + (right.v - left.v) * t` with `t = (soc - left.soc) / width`, not
 * a regrouping of it: a different association rounds differently.
 */
function recordResistance(params: readonly number[], soc: number): number {
  const dep = 1 + (params[CORE_BATTERY_PARAM.depA]! * Math.pow(1 - soc, params[CORE_BATTERY_PARAM.depP]!)) / (soc + params[CORE_BATTERY_PARAM.depK]!);
  return params[CORE_BATTERY_PARAM.rBase]! * dep * params[CORE_BATTERY_PARAM.tempFactor]!;
}

function recordOcv(blob: readonly number[], soc: number): number {
  const n = blob[0]!;
  const pts = Array.from({ length: n }, (_, i) => ({ soc: blob[1 + 2 * i]!, v: blob[2 + 2 * i]! }));
  if (soc <= pts[0]!.soc) return pts[0]!.v;
  for (let i = 1; i < n; i++) {
    const right = pts[i]!;
    if (soc > right.soc) continue;
    const left = pts[i - 1]!;
    const width = right.soc - left.soc;
    if (width <= 0) return right.v;
    const t = (soc - left.soc) / width;
    return left.v + (right.v - left.v) * t;
  }
  return pts[n - 1]!.v;
}

// ─────────────────────────────────────────────────────────────────────────
// battery_pack (code 20)
// ─────────────────────────────────────────────────────────────────────────

const SOC_SWEEP = [0, 0.01, 0.02, 0.05, 0.1, 0.3, 0.5, 0.9, 0.99, 1];

describe("battery_pack record: the compiled model against classic's own", () => {
  const cases: Array<{
    name: string;
    component: SimComponent;
    environment?: SimCircuit["environment"];
    profileId: string;
    profileCode: number;
  }> = [
    { name: "identity-less, no rInternal", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }), profileId: "generic-primary", profileCode: 0 },
    { name: "identity-less with rInternal", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 0.9 }), profileId: "generic-primary", profileCode: 0 },
    { name: "supply-5v at 25 C", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }, "supply-5v"), profileId: "alkaline-aa-4s", profileCode: 1 },
    { name: "supply-5v at 21 C", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }, "supply-5v"), environment: { temperatureC: 21 }, profileId: "alkaline-aa-4s", profileCode: 1 },
    { name: "supply-5v with catalog defaults only", component: part("bat", "battery_pack", ["pos", "neg"], {}, "supply-5v"), profileId: "alkaline-aa-4s", profileCode: 1 },
    { name: "battery-9v", component: part("bat", "battery_pack", ["pos", "neg"], {}, "battery-9v"), profileId: "alkaline-pp3-9v", profileCode: 2 },
    { name: "CR2032 at -30 C", component: part("bat", "battery_pack", ["pos", "neg"], {}, "battery-coin-cr2032"), environment: { temperatureC: -30 }, profileId: "lithium-cr2032", profileCode: 3 },
    { name: "CR2032 at -40 C (clamped)", component: part("bat", "battery_pack", ["pos", "neg"], {}, "battery-coin-cr2032"), environment: { temperatureC: -40 }, profileId: "lithium-cr2032", profileCode: 3 },
    { name: "a stale uid: generic, no catalog defaults", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }, "supply-5v-retired"), profileId: "generic-primary", profileCode: 0 },
    { name: "a stale uid with no voltage: still no catalog default", component: part("bat", "battery_pack", ["pos", "neg"], {}, "supply-5v-retired"), profileId: "generic-primary", profileCode: 0 },
    { name: "charge that is not a number (soc0 0)", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, charge: "x" }), profileId: "generic-primary", profileCode: 0 },
    { name: "charge above 1 (clamped)", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, charge: 3 }), profileId: "generic-primary", profileCode: 0 },
    { name: "charge 0.4", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, charge: 0.4 }, "supply-5v"), profileId: "alkaline-aa-4s", profileCode: 1 },
    { name: "capacityAh 0 (floored at 1e-12)", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, capacityAh: 0 }), profileId: "generic-primary", profileCode: 0 },
    { name: "capacityAh overriding an exact profile", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, capacityAh: 0.1 }, "supply-5v"), profileId: "alkaline-aa-4s", profileCode: 1 },
    { name: "voltage 0 (a dead cell)", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 0, rInternal: 1 }), profileId: "generic-primary", profileCode: 0 },
    { name: "rInternal that is not a number (classic falls back to 0.25)", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: "x" }), profileId: "generic-primary", profileCode: 0 },
    { name: "a negative rInternal (clamped to 0)", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: -2 }), profileId: "generic-primary", profileCode: 0 },
    { name: "charge 0 (a dead cell, not the default 1)", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, charge: 0 }), profileId: "generic-primary", profileCode: 0 },
    { name: "supply-5v with an authored rInternal of 2", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 2 }, "supply-5v"), profileId: "alkaline-aa-4s", profileCode: 1 },
    { name: "supply-5v with an authored rInternal of 2 at 21 C", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 2 }, "supply-5v"), environment: { temperatureC: 21 }, profileId: "alkaline-aa-4s", profileCode: 1 },
    { name: "supply-5v with an rInternal that is not a number (the catalog's 0.9 stands)", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: "x" }, "supply-5v"), profileId: "alkaline-aa-4s", profileCode: 1 },
    { name: "supply-5v with a negative rInternal (clamped to 0, an ideal source)", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: -2 }, "supply-5v"), profileId: "alkaline-aa-4s", profileCode: 1 },
    { name: "supply-5v with an rInternal of 0", component: part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 0 }, "supply-5v"), profileId: "alkaline-aa-4s", profileCode: 1 },
    { name: "battery-9v with an authored rInternal of 3", component: part("bat", "battery_pack", ["pos", "neg"], { rInternal: 3 }, "battery-9v"), profileId: "alkaline-pp3-9v", profileCode: 2 },
    { name: "CR2032 at -30 C with an authored rInternal of 80", component: part("bat", "battery_pack", ["pos", "neg"], { rInternal: 80 }, "battery-coin-cr2032"), environment: { temperatureC: -30 }, profileId: "lithium-cr2032", profileCode: 3 },
  ];

  for (const { name, component, environment, profileId, profileCode } of cases) {
    it(`${name}: params, table and statics reproduce classic`, () => {
      const c = loop(component, environment);
      const compiled = compileOk(c);
      const record = recordOf(compiled, "battery_pack");
      const engine = loadClassic(c);
      const classic = (engine as unknown as { _batteryOperatingPoint(comp: SimComponent): ClassicBattery })._batteryOperatingPoint(component);
      const ambient = environment?.temperatureC ?? 25;

      expect(record.params).toHaveLength(9);
      expect(record.params[CORE_BATTERY_PARAM.profileCode]).toBe(profileCode);
      expect(classic.profile.id).toBe(profileId);

      // The load-time point, then a sweep: the README's formulas on the record
      // must give classic's numbers exactly, at every soc, not just soc0.
      const soc0 = record.params[CORE_BATTERY_PARAM.soc0]!;
      for (const soc of [soc0, ...SOC_SWEEP]) {
        const truth = batteryOperatingPoint(classic.profile, {
          soc,
          ambientTemperatureC: ambient,
          referenceInternalResistanceOhm: classic.referenceInternalResistanceOhm,
        });
        expect(recordResistance(record.params, soc), `R(${soc})`).toBe(truth.internalResistanceOhm);
        expect(recordOcv(record.blob, soc) * record.params[CORE_BATTERY_PARAM.ocvScale]!, `OCV(${soc})`).toBe(
          batteryOpenCircuitVoltageV(classic.profile, soc) * classic.nominalVoltageScale,
        );
      }

      // Capacity and the starting SoC are what classic's load-time state holds.
      expect(record.params[CORE_BATTERY_PARAM.capacityAh]).toBe(classic.profile.referenceCapacityAh);
      const readout = engine.getBatteryState("bat")!;
      // The record's formulas at the SoC classic's own state holds must give the
      // two numbers classic publishes for it, bit for bit: the resistance and
      // the scaled open-circuit voltage its stamp uses.
      expect(recordResistance(record.params, readout.soc), "R at classic's own soc").toBe(readout.internalResistanceOhm);
      expect(recordOcv(record.blob, readout.soc) * record.params[CORE_BATTERY_PARAM.ocvScale]!, "OCV at classic's own soc").toBe(
        readout.openCircuitVoltageV,
      );
      // Classic's load seeds a 1 ps commit (README, Rails point 6, "Load"), which
      // moves SoC by I * 1e-12 / (3600 * capacity): nothing at a real capacity,
      // visible only at the 1e-12 Ah floor, where the seed is not the record.
      if (record.params[CORE_BATTERY_PARAM.capacityAh]! >= 1e-6) {
        expect(readout.soc).toBeCloseTo(soc0, 6);
        expect(readout.remainingCapacityCoulombs).toBeCloseTo(record.params[CORE_BATTERY_PARAM.capacityAh]! * 3600 * soc0, 6);
      }

      // The facade's statics are classic's readout fields.
      expect(compiled.batteries.get("bat")).toEqual({
        profileId: readout.profileId,
        label: readout.label,
        modelTemperatureC: readout.modelTemperatureC,
        temperatureWasClamped: readout.temperatureWasClamped,
      });
      expect(readout.profileId).toBe(profileId);
    });
  }

  it("the OCV table is the profile's curve before ocvScale, generic being [2, 0, 0, 1, nominal]", () => {
    const generic = compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 })));
    expect(recordOf(generic, "battery_pack").blob).toEqual([2, 0, 0, 1, 5]);

    const aa = compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }, "supply-5v")));
    const aaRecord = recordOf(aa, "battery_pack");
    const aaProfile = batteryModelForComponent(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }, "supply-5v")).profile;
    expect(aaRecord.blob[0]).toBe(8);
    expect(aaRecord.blob).toEqual([8, ...aaProfile.ocvCurve.flatMap((p) => [p.soc, p.voltageV])]);

    const cr = compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], {}, "battery-coin-cr2032")));
    expect(recordOf(cr, "battery_pack").blob[0]).toBe(9);
  });

  it("the table blob reads as classic's interpolation at 2,001 socs per profile, not a regrouping of it", () => {
    // The README's rule is one expression; a different association of the same
    // terms differs in the last bit at dozens of these socs (53 of 2,001 for AA_4S).
    for (const [uid, profile] of Object.entries(BATTERY_PROFILES_BY_CATALOG_UID)) {
      const compiled = compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], {}, uid)));
      const { blob } = recordOf(compiled, "battery_pack");
      for (let k = 0; k <= 2000; k++) {
        const soc = k / 2000;
        expect(recordOcv(blob, soc), `${uid} at soc ${soc}`).toBe(batteryOpenCircuitVoltageV(profile, soc));
      }
    }
  });

  it("an authored rInternal on an exact profile replaces the catalog's fresh resistance, unless it is not a finite non-negative number", () => {
    // Classic's resolved model for each: `referenceInternalResistanceOhm` is
    // Math.max(0, Number(rInternal)), and the base resistance is that when
    // finite and non-negative, else the profile's own 0.9 ohm.
    const rBaseOf = (rInternal: number | string) => {
      const component = part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal }, "supply-5v");
      const model = batteryModelForComponent(component);
      expect(model.profile.id).toBe("alkaline-aa-4s");
      expect(model.profile.freshInternalResistanceOhm).toBe(0.9);
      const record = recordOf(compileOk(loop(component)), "battery_pack");
      return { rBase: record.params[CORE_BATTERY_PARAM.rBase], reference: model.referenceInternalResistanceOhm };
    };
    expect(rBaseOf(2)).toEqual({ rBase: 2, reference: 2 });
    expect(rBaseOf("x")).toEqual({ rBase: 0.9, reference: NaN });
    expect(rBaseOf(-2)).toEqual({ rBase: 0, reference: 0 });
    expect(rBaseOf(0)).toEqual({ rBase: 0, reference: 0 });
    // Not authored at all: the catalog's own, from the exact profile's fresh value.
    const unauthored = recordOf(compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }, "supply-5v"))), "battery_pack");
    expect(unauthored.params[CORE_BATTERY_PARAM.rBase]).toBe(0.9);
  });

  it("a supply-5v with {voltage: 5} reads AA_4S at 5/6, 0.9 ohm: 0.8041 ohm at 25 C, 0.9 at 21 C (probe P3)", () => {
    const at = (temperatureC: number) =>
      recordOf(compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }, "supply-5v"), { temperatureC })), "battery_pack");
    const r25 = at(25);
    expect(r25.params[CORE_BATTERY_PARAM.ocvScale]).toBe(5 / 6);
    expect(r25.params[CORE_BATTERY_PARAM.rBase]).toBe(0.9);
    expect(recordResistance(r25.params, 1)).toBe(0.8040943233289921);
    expect(recordResistance(at(21).params, 1)).toBe(0.9);
  });

  it("a CR2032 at -30 C reads 50 ohm (gate 1's battery-engine-physics, classic's own test)", () => {
    const record = recordOf(
      compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], {}, "battery-coin-cr2032"), { temperatureC: -30 })),
      "battery_pack",
    );
    expect(recordResistance(record.params, 1)).toBe(50);
  });

  it("never infers an identity: {voltage: 5} without a catalogUid is generic (no legacy alias, Q-O38)", () => {
    const compiled = compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 })));
    const record = recordOf(compiled, "battery_pack");
    expect(record.params[CORE_BATTERY_PARAM.profileCode]).toBe(0);
    expect(record.params[CORE_BATTERY_PARAM.rBase]).toBe(0);
    expect(compiled.batteries.get("bat")).toMatchObject({ profileId: "generic-primary", label: "Generic primary battery (fallback)" });
  });

  it("identity-less with rInternal is the generic shape (A 0.5, K 0.05), rBase = rInternal (probe Q1)", () => {
    const record = recordOf(compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, rInternal: 0.9 }))), "battery_pack");
    expect(record.params[CORE_BATTERY_PARAM.rBase]).toBe(0.9);
    expect(record.params[CORE_BATTERY_PARAM.depA]).toBe(0.5);
    expect(record.params[CORE_BATTERY_PARAM.depP]).toBe(2);
    expect(record.params[CORE_BATTERY_PARAM.depK]).toBe(0.05);
    expect(record.params[CORE_BATTERY_PARAM.tempFactor]).toBe(1);
  });

  it("the statics: generic reads 25 C unclamped, a CR2032 at -40 C reads -30 C clamped", () => {
    const generic = compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 })));
    expect(generic.batteries.get("bat")).toEqual({
      profileId: "generic-primary",
      label: "Generic primary battery (fallback)",
      modelTemperatureC: 25,
      temperatureWasClamped: false,
    });
    const cold = compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], {}, "battery-coin-cr2032"), { temperatureC: -40 }));
    expect(cold.batteries.get("bat")).toMatchObject({ profileId: "lithium-cr2032", modelTemperatureC: -30, temperatureWasClamped: true });
  });

  it("soc0 is NaN-safe: charge 'x' gives 0, an absent charge gives 1", () => {
    const bad = recordOf(compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, charge: "x" }))), "battery_pack");
    expect(bad.params[CORE_BATTERY_PARAM.soc0]).toBe(0);
    const absent = recordOf(compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }))), "battery_pack");
    expect(absent.params[CORE_BATTERY_PARAM.soc0]).toBe(1);
  });

  it("capacityAh 0 floors at 1e-12, as classic does", () => {
    const record = recordOf(compileOk(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, capacityAh: 0 }))), "battery_pack");
    expect(record.params[CORE_BATTERY_PARAM.capacityAh]).toBe(1e-12);
  });

  it("carries pos then neg, by id, whatever order the pins are declared in", () => {
    const c = circuit(
      [part("bat", "battery_pack", ["neg", "pos"], { voltage: 5 }), part("r1", "resistor", ["a", "b"])],
      [wire("bat", "pos", "r1", "a"), wire("r1", "b", "bat", "neg")],
    );
    const compiled = compileOk(c);
    const record = recordOf(compiled, "battery_pack");
    expect(record.pins.map((p) => p.node)).toEqual([
      compiled.pinNode.get(corePinKey("bat", "pos")),
      compiled.pinNode.get(corePinKey("bat", "neg")),
    ]);
    expect(compiled.pinNode.get(corePinKey("bat", "neg"))).toBe(0);
  });

  it("declines a non-finite resolved value, named by the record's param", () => {
    expect(compileDeclined(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: "x" })))).toEqual([
      "malformed:bat:non-finite-ocvScale",
    ]);
    expect(compileDeclined(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, capacityAh: "x" })))).toEqual([
      "malformed:bat:non-finite-capacityAh",
    ]);
    expect(compileDeclined(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5, capacityAh: Infinity })))).toEqual([
      "malformed:bat:non-finite-capacityAh",
    ]);
    // An exact profile scales its own nominal, so a non-numeric voltage is caught there too.
    expect(compileDeclined(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: "x" }, "supply-5v")))).toEqual([
      "malformed:bat:non-finite-ocvScale",
    ]);
  });

  it("declines fewer than 2 pins as too-few-pins, and a missing pos or neg as missing-pin", () => {
    expect(compileDeclined(circuit([part("bat", "battery_pack", ["pos"], { voltage: 5 })]))).toEqual([
      "malformed:bat:too-few-pins",
    ]);
    expect(compileDeclined(circuit([part("bat", "battery_pack", [], { voltage: 5 })]))).toEqual([
      "malformed:bat:too-few-pins",
    ]);
    expect(compileDeclined(circuit([part("bat", "battery_pack", ["pos", "x"], { voltage: 5 })]))).toEqual([
      "malformed:bat:missing-pin",
    ]);
    expect(compileDeclined(circuit([part("bat", "battery_pack", ["p", "neg"], { voltage: 5 })]))).toEqual([
      "malformed:bat:missing-pin",
    ]);
    expect(compileDeclined(circuit([part("bat", "battery_pack", ["a", "b"], { voltage: 5 })]))).toEqual([
      "malformed:bat:missing-pin",
    ]);
  });
});

describe("battery profiles the record cannot name", () => {
  it("every chemistry classic ships has its own profileCode, so no catalog battery is declined", () => {
    const ids = ["generic-primary", ...Object.values(BATTERY_PROFILES_BY_CATALOG_UID).map((profile) => profile.id)];
    for (const id of ids) expect(CORE_BATTERY_PROFILE_CODE[id], id).toBeTypeOf("number");
    expect(new Set(ids.map((id) => CORE_BATTERY_PROFILE_CODE[id])).size).toBe(ids.length);
    expect(Object.keys(CORE_BATTERY_PROFILE_CODE).sort()).toEqual([...ids].sort());
  });

  it("a profile with no profileCode declines unsupported-feature:battery-profile:<profileId> (a chemistry classic gains later)", () => {
    // A fifth chemistry added to classic's table, for the length of this test.
    const table = BATTERY_PROFILES_BY_CATALOG_UID as Record<string, BatteryPhysicsProfile>;
    const uid = "test-chemistry-uid";
    table[uid] = { ...table["supply-5v"]!, id: "test-chemistry" as BatteryPhysicsProfile["id"], catalogUid: uid };
    try {
      const component = part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }, uid);
      expect(batteryModelForComponent(component).profile.id).toBe("test-chemistry");
      expect(compileDeclined(loop(component))).toEqual(["unsupported-feature:battery-profile:test-chemistry"]);
    } finally {
      delete table[uid];
    }
    // Gone again: the same uid is a stale one, generic, and compiles.
    const stale = part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }, uid);
    expect(recordOf(compileOk(loop(stale)), "battery_pack").params[CORE_BATTERY_PARAM.profileCode]).toBe(0);
  });

  it("a profile whose OCV table holds a non-finite entry declines malformed:<id>:non-finite-ocvTable, and only for that", () => {
    // classic's shipped tables are all finite, so the only way to reach the
    // check is a profile added to its table, as above. This copy of supply-5v
    // keeps that profile's id (so it has a profileCode) and every scalar the
    // record carries finite: the table is the one thing wrong, which is why
    // the params check ahead of it does not catch this.
    const table = BATTERY_PROFILES_BY_CATALOG_UID as Record<string, BatteryPhysicsProfile>;
    const uid = "test-bad-ocv-uid";
    const base = table["supply-5v"]!;
    const withPoint = (point: { soc: number; voltageV: number }): BatteryPhysicsProfile => ({
      ...base,
      catalogUid: uid,
      ocvCurve: [base.ocvCurve[0]!, point, ...base.ocvCurve.slice(1)],
    });
    const compileWith = (profile: BatteryPhysicsProfile) => {
      table[uid] = profile;
      try {
        const component = part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }, uid);
        expect(batteryModelForComponent(component).profile.ocvCurve).toBe(profile.ocvCurve);
        return compileCircuit(loop(component), WITH_SOURCES);
      } finally {
        delete table[uid];
      }
    };

    for (const [what, point] of [
      ["a NaN voltage", { soc: 0.5, voltageV: NaN }],
      ["an infinite voltage", { soc: 0.5, voltageV: Infinity }],
      ["a NaN soc", { soc: NaN, voltageV: 3 }],
      ["a negative-infinite soc", { soc: -Infinity, voltageV: 3 }],
    ] as const) {
      expect(compileWith(withPoint(point)), what).toEqual({ ok: false, reasons: ["malformed:bat:non-finite-ocvTable"] });
    }

    // The control: the same injection with a finite point compiles, so the
    // decline above is the entry and nothing else about the profile.
    const control = compileWith(withPoint({ soc: 0.5, voltageV: 3 }));
    expect(control.ok).toBe(true);
    if (control.ok) {
      const blob = recordOf(control.compiled, "battery_pack").blob;
      expect(blob[0]).toBe(base.ocvCurve.length + 1);
      expect(blob.slice(3, 5)).toEqual([0.5, 3]);
    }
    // And the table is gone again: the uid is stale, generic, and compiles.
    expect(table[uid]).toBeUndefined();
  });
});

// ─────────────────────────────────────────────────────────────────────────
// bench_psu (code 21)
// ─────────────────────────────────────────────────────────────────────────

describe("bench_psu record", () => {
  function psu(params: Record<string, number | string>, catalogUid?: string): SimComponent {
    return part("psu", "bench_psu", ["pos", "neg"], params, catalogUid);
  }

  it("V and iLimit come from params only: a supply-var with no iLimit is an ideal source (probe P6)", () => {
    const c = loop(psu({}, "supply-var"));
    const record = recordOf(compileOk(c), "bench_psu");
    expect(record.params).toEqual([5, 0]);
    // Classic: 5 V, and no icState, so no regime logic ever runs.
    const engine = loadClassic(c);
    expect(engine.getNetV()["n0"]).toBeCloseTo(5, 9);
    expect(engine.getIcState("psu")).toBeUndefined();
  });

  it("an iLimit above 0 is carried, and classic then holds a regime state", () => {
    const c = loop(psu({ voltage: 12, iLimit: 0.25 }));
    const compiled = compileOk(c);
    const record = recordOf(compiled, "bench_psu");
    expect(record.params[CORE_BENCH_PSU_PARAM.V]).toBe(12);
    expect(record.params[CORE_BENCH_PSU_PARAM.iLimit]).toBe(0.25);
    // The facade picks the PSU's device-state layout from this, so it is the record's own value.
    expect(compiled.benchPsuLimits.get("psu")).toBe(0.25);
    expect(loadClassic(c).getIcState("psu")).toBeDefined();
  });

  it("an iLimit that is not above 0 is emitted as 0, exactly where classic keeps no regime state", () => {
    for (const iLimit of [0, -1, -1e-9, "x", NaN, -Infinity, ""]) {
      const c = loop(psu({ voltage: 5, iLimit }));
      const compiled = compileOk(c);
      const record = recordOf(compiled, "bench_psu");
      expect(record.params[CORE_BENCH_PSU_PARAM.iLimit], `iLimit ${String(iLimit)}`).toBe(0);
      expect(compiled.benchPsuLimits.get("psu"), `iLimit ${String(iLimit)}`).toBe(0);
      expect(loadClassic(c).getIcState("psu"), `iLimit ${String(iLimit)}`).toBeUndefined();
    }
    // A string that parses is the limit, as Number() reads it.
    const parsed = loop(psu({ voltage: 5, iLimit: "0.5" }));
    expect(recordOf(compileOk(parsed), "bench_psu").params[CORE_BENCH_PSU_PARAM.iLimit]).toBe(0.5);
    expect(loadClassic(parsed).getIcState("psu")).toBeDefined();
  });

  it("a voltage of 0 is 0, not the default: `??` as classic reads it, where `||` would give 5", () => {
    for (const iLimit of [undefined, 0.5]) {
      const params: Record<string, number | string> = iLimit === undefined ? { voltage: 0 } : { voltage: 0, iLimit };
      const c = loop(psu(params));
      const record = recordOf(compileOk(c), "bench_psu");
      expect(record.params[CORE_BENCH_PSU_PARAM.V], `iLimit ${String(iLimit)}`).toBe(0);
      // Classic's rail with the setpoint at 0: no volts, whatever the limit.
      const engine = loadClassic(c);
      engine.step(1e-4);
      expect(engine.getNetV()["n0"], `rail, iLimit ${String(iLimit)}`).toBeCloseTo(0, 9);
    }
    // And the empty string is not nullish either: Number("") is 0.
    expect(recordOf(compileOk(loop(psu({ voltage: "" }))), "bench_psu").params[CORE_BENCH_PSU_PARAM.V]).toBe(0);
  });

  it("a negative setpoint is carried as authored", () => {
    expect(recordOf(compileOk(loop(psu({ voltage: -5, iLimit: 0.1 }))), "bench_psu").params).toEqual([-5, 0.1]);
  });

  it("declines a non-finite voltage, and an infinite limit, which is above 0 in classic but no finite limit", () => {
    expect(compileDeclined(loop(psu({ voltage: "x" })))).toEqual(["malformed:psu:non-finite-voltage"]);
    expect(compileDeclined(loop(psu({ voltage: Infinity, iLimit: 1 })))).toEqual(["malformed:psu:non-finite-voltage"]);
    expect(compileDeclined(loop(psu({ voltage: 5, iLimit: Infinity })))).toEqual(["malformed:psu:non-finite-iLimit"]);
    expect(compileDeclined(loop(psu({ voltage: 5, iLimit: "Infinity" })))).toEqual(["malformed:psu:non-finite-iLimit"]);
  });

  it("declines fewer than 2 pins", () => {
    expect(compileDeclined(circuit([part("psu", "bench_psu", ["pos"], { voltage: 5 })]))).toEqual([
      "malformed:psu:too-few-pins",
    ]);
  });

  it("reads its pins by position, not by id, and owns no slot, no blob", () => {
    const c = circuit(
      [part("psu", "bench_psu", ["out", "ret"], { voltage: 5, iLimit: 1 }), part("r1", "resistor", ["a", "b"])],
      [wire("psu", "out", "r1", "a"), wire("r1", "b", "psu", "ret")],
    );
    const compiled = compileOk(c);
    const record = recordOf(compiled, "bench_psu");
    expect(record.pins.map((p) => p.node)).toEqual([
      compiled.pinNode.get(corePinKey("psu", "out")),
      compiled.pinNode.get(corePinKey("psu", "ret")),
    ]);
    expect(record.blob).toEqual([]);
    expect(compiled.digitalSlots).toEqual([]);
  });
});

// ─────────────────────────────────────────────────────────────────────────
// clock and clock_gen (codes 22, 23)
// ─────────────────────────────────────────────────────────────────────────

/**
 * Steps classic and checks each published level against `level(t)` built
 * from the COMPILED params: if the record carries what classic resolved, the
 * two agree at every sample, for both the node voltage and the digital slot.
 */
function expectMatchesClassic(
  c: SimCircuit,
  sourceId: string,
  netOfPlus: string,
  level: (t: number) => number,
  threshold: number,
  stepS: number,
  steps: number,
): void {
  const engine = loadClassic(c);
  for (let i = 0; i < steps; i++) {
    engine.step(stepS);
    const t = engine.simTime;
    expect(engine.getNetV()[netOfPlus], `v at ${t}`).toBeCloseTo(level(t), 9);
    expect(engine.digitalState[sourceId], `digital at ${t}`).toBe(level(t) > threshold ? 1 : 0);
  }
}

describe("clock and clock_gen records", () => {
  function clk(kind: "clock" | "clock_gen", params: Record<string, number | string> = {}): SimComponent {
    return part("clk", kind, ["out", "gnd"], params);
  }

  it("defaults are classic's inline ones, with no catalog default read: [1000, 0.5, 5, 0, 0]", () => {
    for (const kind of ["clock", "clock_gen"] as const) {
      const compiled = compileOk(loop(clk(kind)));
      const record = recordOf(compiled, kind);
      expect(record.kindCode).toBe(kind === "clock" ? 22 : 23);
      expect(record.params).toEqual([1000, 0.5, 5, 0, 0]);
    }
  });

  it("carries authored values, floors frequency at 1e-6 and reads strings as Number() does", () => {
    const record = recordOf(
      compileOk(loop(clk("clock", { frequency: "250", duty: 0.3, v_high: 3.3, v_low: -1, delay: 4e-4 }))),
      "clock",
    );
    expect(record.params).toEqual([250, 0.3, 3.3, -1, 4e-4]);
    for (const frequency of [0, -50]) {
      expect(recordOf(compileOk(loop(clk("clock", { frequency }))), "clock").params[CORE_CLOCK_PARAM.freq]).toBe(1e-6);
    }
  });

  it("the compiled params reproduce classic's clock at every sample (node voltage and digital slot)", () => {
    const cases: Record<string, number | string>[] = [
      {},
      { frequency: 1000, duty: 0.3, v_high: 3.3, v_low: -1, delay: 4e-4 },
      { frequency: 2500, duty: 0.5, v_high: 5, v_low: 0, delay: -1e-4 },
      { frequency: 800, duty: 1, v_high: 5, v_low: 0 },
      { frequency: 800, duty: 0, v_high: 5, v_low: 1 },
      { frequency: 0, v_high: 5 },
    ];
    for (const params of cases) {
      const c = loop(clk("clock", params));
      const record = recordOf(compileOk(c), "clock");
      const [freq, duty, vHigh, vLow, delay] = record.params as [number, number, number, number, number];
      expectMatchesClassic(
        c,
        "clk",
        "n0",
        (t) => clockVoltage(t, 1 / freq, duty, vHigh, vLow, delay),
        (vHigh + vLow) / 2,
        params.frequency === 0 ? 1e5 : 1e-4,
        params.frequency === 0 ? 5 : 40,
      );
    }
  });

  it("claims one digital slot keyed by the bare component id, in component order", () => {
    const c = circuit(
      [
        part("clk1", "clock", ["out", "gnd"]),
        part("r1", "resistor", ["a", "b"]),
        part("clk2", "clock_gen", ["out", "gnd"]),
      ],
      [wire("clk1", "out", "r1", "a"), wire("r1", "b", "clk2", "out")],
    );
    const compiled = compileOk(c);
    expect(compiled.digitalSlots).toEqual(["clk1", "clk2"]);
    const clocks = decodeProgram(compiled.program).devices.filter((d) => d.kindCode === 22 || d.kindCode === 23);
    expect(clocks.map((d) => d.blob)).toEqual([[0], [1]]);
  });

  it("a v_high of 0 is 0, not the default: `??` as classic reads it, where `||` would give 5", () => {
    for (const kind of ["clock", "clock_gen"] as const) {
      // v_low -1 makes the high level 0 and the midpoint threshold -0.5, so the
      // two readings of the param could not agree by accident.
      const c = loop(clk(kind, { v_high: 0, v_low: -1, duty: 0.5, frequency: 1000 }));
      const record = recordOf(compileOk(c), kind);
      expect(record.params[CORE_CLOCK_PARAM.vHigh], kind).toBe(0);
      // Classic's node, through one high phase and one low phase of the period.
      const engine = loadClassic(c);
      const seen = new Set<number>();
      for (let i = 0; i < 20; i++) {
        engine.step(1e-4);
        seen.add(Math.round(engine.getNetV()["n0"]! * 1e6) / 1e6);
      }
      expect([...seen].sort((a, b) => a - b), kind).toEqual([-1, 0]);
    }
  });

  it("declines any non-finite value, naming the record's param (freq, duty, vHigh, vLow, delay), for both kinds", () => {
    for (const kind of ["clock", "clock_gen"] as const) {
      const reason = (params: Record<string, number | string>) => compileDeclined(loop(clk(kind, params)));
      expect(reason({ frequency: "x" })).toEqual(["malformed:clk:non-finite-freq"]);
      expect(reason({ frequency: Infinity })).toEqual(["malformed:clk:non-finite-freq"]);
      expect(reason({ duty: "x" })).toEqual(["malformed:clk:non-finite-duty"]);
      expect(reason({ v_high: Infinity })).toEqual(["malformed:clk:non-finite-vHigh"]);
      expect(reason({ v_low: -Infinity })).toEqual(["malformed:clk:non-finite-vLow"]);
      expect(reason({ delay: "x" })).toEqual(["malformed:clk:non-finite-delay"]);
      // Several at once: the first in record order is the one named.
      expect(reason({ frequency: "x", v_high: "x", v_low: "x" })).toEqual(["malformed:clk:non-finite-freq"]);
    }
  });

  it("declines fewer than 2 pins, and reads its pins by position", () => {
    expect(compileDeclined(circuit([part("clk", "clock", ["out"])]))).toEqual(["malformed:clk:too-few-pins"]);
    expect(compileDeclined(circuit([part("clk", "clock_gen", [])]))).toEqual(["malformed:clk:too-few-pins"]);
    const c = circuit([part("clk", "clock", ["a", "b"]), part("r1", "resistor", ["a", "b"])], [wire("clk", "a", "r1", "a"), wire("r1", "b", "clk", "b")]);
    const compiled = compileOk(c);
    expect(recordOf(compiled, "clock").pins.map((p) => p.node)).toEqual([
      compiled.pinNode.get(corePinKey("clk", "a")),
      compiled.pinNode.get(corePinKey("clk", "b")),
    ]);
  });
});

// ─────────────────────────────────────────────────────────────────────────
// pulse_source and pulse_gen (codes 24, 25)
// ─────────────────────────────────────────────────────────────────────────

describe("pulse_source and pulse_gen records", () => {
  function pls(kind: "pulse_source" | "pulse_gen", params: Record<string, number | string> = {}): SimComponent {
    return part("p1", kind, ["pos", "neg"], params);
  }

  it("defaults are classic's inline ones: [0, 5, 0, 1e-6, 1e-6, 5e-4, 1e-3]", () => {
    for (const kind of ["pulse_source", "pulse_gen"] as const) {
      const record = recordOf(compileOk(loop(pls(kind))), kind);
      expect(record.kindCode).toBe(kind === "pulse_source" ? 24 : 25);
      expect(record.params).toEqual([0, 5, 0, 1e-6, 1e-6, 5e-4, 1e-3]);
    }
  });

  it("the compiled params reproduce classic's pulse at every sample, one-shot forms included", () => {
    const cases: Record<string, number | string>[] = [
      {},
      { v1: 1, v2: 4, td: 2e-4, tr: 1e-4, tf: 2e-4, pw: 3e-4, per: 1e-3 },
      { v1: 0, v2: 5, td: 1e-4, tr: 5e-5, tf: 5e-5, pw: 2e-4, per: 0 },
      { v1: 0, v2: 5, td: 0, tr: 0, tf: 0, pw: 2e-4, per: -1 },
      { v1: 0, v2: 5, td: 0, tr: 1e-4, tf: 1e-4, pw: 2e-4, per: "x" },
      { v1: 0, v2: 5, td: 0, tr: 1e-4, tf: 1e-4, pw: 2e-4, per: Infinity },
      { v1: 0, v2: 5, td: 0, tr: 1e-4, tf: 1e-4, pw: 0, per: 1e-3 },
      { v1: 3, v2: 3, td: 0, tr: 1e-4, tf: 1e-4, pw: 2e-4, per: 1e-3 },
    ];
    for (const params of cases) {
      const c = loop(pls("pulse_source", params));
      const record = recordOf(compileOk(c), "pulse_source");
      const [v1, v2, td, tr, tf, pw, per] = record.params as [number, number, number, number, number, number, number];
      expectMatchesClassic(c, "p1", "n0", (t) => pulseVoltage(t, v1, v2, td, tr, tf, pw, per), (v1 + v2) / 2, 5e-5, 60);
    }
  });

  it("a v2 of 0 is 0, not the default: `??` as classic reads it, where `||` would give 5", () => {
    for (const kind of ["pulse_source", "pulse_gen"] as const) {
      // v1 1 and v2 0: the flat top reads 0 V and the base reads 1 V, so a v2
      // read as the default 5 would show on the node at once.
      const c = loop(pls(kind, { v1: 1, v2: 0, td: 0, tr: 1e-6, tf: 1e-6, pw: 5e-4, per: 1e-3 }));
      const record = recordOf(compileOk(c), kind);
      expect(record.params[CORE_PULSE_PARAM.v2], kind).toBe(0);
      const engine = loadClassic(c);
      engine.step(2.5e-4); // mid flat top
      expect(engine.getNetV()["n0"], `${kind} flat top`).toBeCloseTo(0, 9);
      engine.step(5e-4); // t = 7.5e-4, past the fall
      expect(engine.getNetV()["n0"], `${kind} base`).toBeCloseTo(1, 9);
    }
  });

  it("a period that is not finite and positive is emitted as 0 (a one-shot); a positive one is carried", () => {
    const per = (value: number | string) =>
      recordOf(compileOk(loop(pls("pulse_source", { per: value }))), "pulse_source").params[CORE_PULSE_PARAM.per];
    expect(per(0)).toBe(0);
    expect(per(-1e-3)).toBe(0);
    expect(per("x")).toBe(0);
    expect(per(NaN)).toBe(0);
    expect(per(Infinity)).toBe(0);
    expect(per(-Infinity)).toBe(0);
    expect(per(2e-3)).toBe(2e-3);
  });

  it("declines a truncated cycle, and only a truncated one: a period equal to tr + pw + tf is whole", () => {
    // Binary-exact lengths, so `tr + pw + tf` is exactly 1 with no rounding to argue about.
    const whole = { tr: 0.25, pw: 0.5, tf: 0.25, per: 1 };
    expect(recordOf(compileOk(loop(pls("pulse_source", whole))), "pulse_source").params[CORE_PULSE_PARAM.per]).toBe(1);
    expect(compileDeclined(loop(pls("pulse_source", { ...whole, per: 0.9 })))).toEqual(["malformed:p1:truncated-pulse"]);
    expect(compileDeclined(loop(pls("pulse_gen", { tr: 4e-4, tf: 4e-4, pw: 4e-4, per: 1e-3 })))).toEqual(["malformed:p1:truncated-pulse"]);
    // The catalog-shaped default cycle is whole, and a one-shot has no cycle to cut
    // (the source, its load, and the load's W1g rating record).
    expect(compileOk(loop(pls("pulse_source", {}))).devices).toHaveLength(3);
    expect(compileOk(loop(pls("pulse_source", { tr: 1, pw: 1, tf: 1, per: 0 }))).devices).toHaveLength(3);
  });

  it("the truncation test sums as classic's pulseVoltage does, (tr + pw) + tf: a period between the two groupings declines", () => {
    const tr = 0.1;
    const pw = 0.2;
    const tf = 0.3;
    const fallEnd = tr + pw + tf; // 0.6000000000000001: classic's `fall + tf`
    const regrouped = tr + (pw + tf); // 0.6: one ulp lower
    expect(regrouped).toBeLessThan(fallEnd);

    // Classic's own function, as a one-shot so nothing wraps: at 0.6 the fall
    // is still running (a period of 0.6 cuts it short), and it ends exactly at
    // the left-to-right sum.
    expect(pulseVoltage(regrouped, 0, 5, 0, tr, tf, pw, 0)).toBeGreaterThan(0);
    expect(pulseVoltage(fallEnd, 0, 5, 0, tr, tf, pw, 0)).toBe(0);

    const lengths = { v1: 0, v2: 5, td: 0, tr, tf, pw };
    expect(compileDeclined(loop(pls("pulse_source", { ...lengths, per: regrouped })))).toEqual(["malformed:p1:truncated-pulse"]);
    expect(compileDeclined(loop(pls("pulse_gen", { ...lengths, per: regrouped })))).toEqual(["malformed:p1:truncated-pulse"]);
    // The period that is exactly the sum is whole.
    const whole = recordOf(compileOk(loop(pls("pulse_source", { ...lengths, per: fallEnd }))), "pulse_source");
    expect(whole.params[CORE_PULSE_PARAM.per]).toBe(fallEnd);
  });

  it("declines a negative tr, tf or pw, and any non-finite value except per", () => {
    const reason = (params: Record<string, number | string>) => compileDeclined(loop(pls("pulse_source", params)));
    expect(reason({ tr: -1 })).toEqual(["malformed:p1:negative-tr"]);
    expect(reason({ tf: -1e-9 })).toEqual(["malformed:p1:negative-tf"]);
    expect(reason({ pw: -1 })).toEqual(["malformed:p1:negative-pw"]);
    expect(reason({ v1: "x" })).toEqual(["malformed:p1:non-finite-v1"]);
    expect(reason({ v2: Infinity })).toEqual(["malformed:p1:non-finite-v2"]);
    expect(reason({ td: NaN })).toEqual(["malformed:p1:non-finite-td"]);
    expect(reason({ tr: "x" })).toEqual(["malformed:p1:non-finite-tr"]);
    expect(reason({ tf: Infinity })).toEqual(["malformed:p1:non-finite-tf"]);
    expect(reason({ pw: -Infinity })).toEqual(["malformed:p1:non-finite-pw"]);
  });

  it("accepts a zero-width flat top and a negative delay, and reads a negative td as authored", () => {
    expect(recordOf(compileOk(loop(pls("pulse_source", { pw: 0, tr: 0, tf: 0 }))), "pulse_source").params).toEqual([
      0, 5, 0, 0, 0, 0, 1e-3,
    ]);
    expect(recordOf(compileOk(loop(pls("pulse_source", { td: -1e-3 }))), "pulse_source").params[CORE_PULSE_PARAM.td]).toBe(-1e-3);
  });

  it("claims one digital slot keyed by the bare component id; slots are consecutive across kinds", () => {
    const c = circuit(
      [
        part("clk1", "clock", ["out", "gnd"]),
        part("pa", "pulse_source", ["pos", "neg"]),
        part("pb", "pulse_gen", ["pos", "neg"]),
        part("r1", "resistor", ["a", "b"]),
      ],
      [wire("clk1", "out", "r1", "a"), wire("r1", "b", "pa", "pos"), wire("pa", "neg", "pb", "pos")],
    );
    const compiled = compileOk(c);
    expect(compiled.digitalSlots).toEqual(["clk1", "pa", "pb"]);
    const sources = decodeProgram(compiled.program).devices.filter((d) => d.kindCode >= 22 && d.kindCode <= 25);
    expect(sources.map((d) => d.blob)).toEqual([[0], [1], [2]]);
  });

  it("declines fewer than 2 pins", () => {
    expect(compileDeclined(circuit([part("p1", "pulse_source", ["pos"])]))).toEqual(["malformed:p1:too-few-pins"]);
    expect(compileDeclined(circuit([part("p1", "pulse_gen", [])]))).toEqual(["malformed:p1:too-few-pins"]);
  });
});

// ─────────────────────────────────────────────────────────────────────────
// section_anchor (code 26)
// ─────────────────────────────────────────────────────────────────────────

/** Anchors against classic's own isolatedSectionAnchorRows(): class row `r` is core node `r + 1`. */
function expectAnchorsMatchClassic(c: SimCircuit): { compiled: CompiledCircuit; anchors: DecodedDevice[] } {
  const compiled = compileOk(c);
  const decoded = decodeProgram(compiled.program);
  const anchors = decoded.devices.filter((d) => d.kindCode === CORE_KIND_CODE.section_anchor);
  const rows = loadClassic(c).isolatedSectionAnchorRows();
  expect(anchors.map((a) => a.pins[0]!.node)).toEqual(rows.map((row) => row + 1));
  return { compiled, anchors };
}

/** Two batteries and their own loads, joined only through the compiler's reference choice (probe Q4's shape). */
function secondBatteryBoard(): SimCircuit {
  return circuit(
    [
      part("bat1", "battery_pack", ["pos", "neg"], { voltage: 5 }),
      part("r1", "resistor", ["a", "b"], { resistance: 100 }),
      part("bat2", "battery_pack", ["pos", "neg"], { voltage: 9 }),
      part("r2", "resistor", ["a", "b"], { resistance: 1000 }),
    ],
    [
      wire("bat1", "pos", "r1", "a"),
      wire("r1", "b", "bat1", "neg"),
      wire("bat2", "pos", "r2", "a"),
      wire("r2", "b", "bat2", "neg"),
    ],
  );
}

describe("section_anchor records", () => {
  it("a second battery's section gets one record, on that battery's neg, owned by it (probe Q4)", () => {
    const c = secondBatteryBoard();
    const { compiled, anchors } = expectAnchorsMatchClassic(c);
    expect(anchors).toHaveLength(1);
    const anchor = anchors[0]!;
    expect(anchor.pins).toHaveLength(1);
    expect(anchor.pins[0]!.node).toBe(compiled.pinNode.get(corePinKey("bat2", "neg")));
    expect(anchor.componentIndex).toBe(2); // bat2
    expect(anchor.params).toEqual([]);
    expect(anchor.blob).toEqual([]);
  });

  it("emits the anchors after every component record, and lists them in devices with a synthetic id", () => {
    const compiled = compileOk(secondBatteryBoard());
    // The W1g rating records follow the anchors (compile-thermal.test.ts).
    expect(compiled.devices.map((d) => d.kind)).toEqual([
      "battery_pack",
      "resistor",
      "battery_pack",
      "resistor",
      "section_anchor",
      "rating",
      "rating",
    ]);
    const anchorNode = compiled.pinNode.get(corePinKey("bat2", "neg"))!;
    expect(compiled.devices[4]).toEqual({ componentId: `section_anchor:${anchorNode}`, kind: "section_anchor" });
    // No component id can collide with it, so an id-keyed readout never sees the anchor as bat2.
    expect(compiled.elementIKeys).toEqual(["bat1", "r1", "bat2", "r2"]);
  });

  it("the anchor's pin is bit for bit the owner's own pin record (open flag and hashes included)", () => {
    const compiled = compileOk(secondBatteryBoard());
    const decoded = decodeProgram(compiled.program);
    const bat2 = decoded.devices[2]!;
    const anchor = decoded.devices[4]!;
    expect(anchor.pins[0]).toEqual(bat2.pins[1]); // bat2.neg is pins[1]
  });

  it("a circuit whose every component is on the datum's section has none", () => {
    const { anchors } = expectAnchorsMatchClassic(loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 })));
    expect(anchors).toEqual([]);
  });

  it("finds the same sections as classic, in ascending anchor node, mixing return-pin and lowest-node anchors", () => {
    // Circuit order puts the fallback section (two resistors, no source) before
    // the clock's, so a wrong ordering or a wrong owner would show.
    const c = circuit(
      [
        part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }),
        part("r1", "resistor", ["a", "b"]),
        part("rz", "resistor", ["a", "b"]),
        part("ry", "resistor", ["a", "b"]),
        part("clk", "clock", ["out", "gnd"]),
        part("r2", "resistor", ["a", "b"]),
      ],
      [
        wire("bat", "pos", "r1", "a"),
        wire("r1", "b", "bat", "neg"),
        wire("ry", "b", "rz", "a"),
        wire("clk", "out", "r2", "a"),
        wire("r2", "b", "clk", "gnd"),
      ],
    );
    const { compiled, anchors } = expectAnchorsMatchClassic(c);
    expect(anchors).toHaveLength(2);
    const nodes = anchors.map((a) => a.pins[0]!.node);
    expect(nodes).toEqual([...nodes].sort((a, b) => a - b));

    const byOwner = new Map(anchors.map((a) => [a.componentIndex, a]));
    // The clock's section: its return pin (`gnd`), owned by the clock (index 4).
    expect(byOwner.get(4)!.pins[0]!.node).toBe(compiled.pinNode.get(corePinKey("clk", "gnd")));
    // The resistor pair's section has no source: the lowest node in it, owned by
    // the first component in circuit order on that node's net.
    const fallback = anchors.find((a) => a.componentIndex !== 4)!;
    const sectionNodes = ["rz", "ry"].flatMap((id) => ["a", "b"].map((pin) => compiled.pinNode.get(corePinKey(id, pin))!));
    expect(fallback.pins[0]!.node).toBe(Math.min(...sectionNodes));
    const ownerIndex = fallback.componentIndex;
    expect(["rz", "ry"]).toContain(c.components[ownerIndex]!.id);
    const netOfAnchor = compiled.nets.find((net) => compiled.nodeOfNet.get(net.id) === fallback.pins[0]!.node)!;
    const owners = netOfAnchor.pins.map(([id]) => c.components.findIndex((comp) => comp.id === id));
    expect(ownerIndex).toBe(Math.min(...owners));
  });

  it("a circuit with no datum net at all anchors every section, as classic does", () => {
    const c = circuit([part("r1", "resistor", ["a", "b"]), part("r2", "resistor", ["a", "b"])]);
    const { anchors } = expectAnchorsMatchClassic(c);
    expect(anchors).toHaveLength(2);
    // Node 0 is the synthetic datum: never an anchor.
    expect(anchors.every((a) => a.pins[0]!.node > 0)).toBe(true);
  });

  it("an isolated ideal source is anchored on its neg too (Q-O39 stands only for a source joined through resistors)", () => {
    const c = circuit(
      [
        part("vs1", "voltage_source", ["pos", "neg"], { voltage: 9 }),
        part("r1", "resistor", ["a", "b"]),
        part("vs2", "voltage_source", ["pos", "neg"], { voltage: 3 }),
        part("r2", "resistor", ["a", "b"]),
      ],
      [
        wire("vs1", "pos", "r1", "a"),
        wire("r1", "b", "vs1", "neg"),
        wire("vs2", "pos", "r2", "a"),
        wire("r2", "b", "vs2", "neg"),
      ],
    );
    const { compiled, anchors } = expectAnchorsMatchClassic(c);
    expect(anchors).toHaveLength(1);
    expect(anchors[0]!.pins[0]!.node).toBe(compiled.pinNode.get(corePinKey("vs2", "neg")));
    expect(anchors[0]!.componentIndex).toBe(2);
  });

  it("a source joined to the datum through a resistor is not isolated: no anchor", () => {
    const c = circuit(
      [
        part("vs1", "voltage_source", ["pos", "neg"], { voltage: 9 }),
        part("r1", "resistor", ["a", "b"]),
        part("vs2", "voltage_source", ["pos", "neg"], { voltage: 3 }),
      ],
      [wire("vs1", "pos", "r1", "a"), wire("r1", "b", "vs1", "neg"), wire("vs2", "neg", "r1", "a")],
    );
    expect(expectAnchorsMatchClassic(c).anchors).toEqual([]);
  });

  describe("two sources in one isolated section: the anchor is the return pin of the FIRST in circuit order", () => {
    interface SourceSpec {
      kind: string;
      plus: string;
      minus: string;
    }

    /**
     * A datum loop (bat0, r0) and, apart from it, one section holding sources
     * `a` and `b` in a ring through a load, with `a`'s return pin and `b`'s on
     * different nets. The wires are fixed; only the components' order changes,
     * so the net numbering is identical in both orders and only "first" moves.
     * The section's lowest node is the load's, which neither return pin is.
     */
    function ring(a: SourceSpec, b: SourceSpec, order: "a-first" | "b-first"): SimCircuit {
      const srcA = part("sa", a.kind, [a.plus, a.minus]);
      const srcB = part("sb", b.kind, [b.plus, b.minus]);
      return circuit(
        [
          part("bat0", "battery_pack", ["pos", "neg"], { voltage: 5 }),
          part("r0", "resistor", ["a", "b"]),
          ...(order === "a-first" ? [srcA, srcB] : [srcB, srcA]),
          part("rl", "resistor", ["a", "b"]),
        ],
        [
          wire("bat0", "pos", "r0", "a"),
          wire("r0", "b", "bat0", "neg"),
          wire("sb", b.plus, "rl", "a"),
          wire("rl", "b", "sa", a.minus),
          wire("sa", a.plus, "sb", b.minus),
        ],
      );
    }

    const VS: SourceSpec = { kind: "voltage_source", plus: "pos", minus: "neg" };
    const BAT: SourceSpec = { kind: "battery_pack", plus: "pos", minus: "neg" };
    const CLK: SourceSpec = { kind: "clock", plus: "out", minus: "gnd" };
    const pairs: Array<[string, SourceSpec, SourceSpec]> = [
      ["two voltage sources", VS, { ...VS }],
      ["a battery and a voltage source", BAT, VS],
      ["a clock (return pin gnd) and a battery (return pin neg)", CLK, BAT],
    ];

    for (const [name, a, b] of pairs) {
      it(`${name}: classic's anchor in both orders, neither the section's lowest node`, () => {
        const nodes: Record<string, number> = {};
        for (const order of ["a-first", "b-first"] as const) {
          const c = ring(a, b, order);
          const { compiled, anchors } = expectAnchorsMatchClassic(c); // the node classic anchors
          expect(anchors, order).toHaveLength(1);
          const first = order === "a-first" ? { id: "sa", spec: a } : { id: "sb", spec: b };
          const anchor = anchors[0]!;
          // The first source's return pin, owned by the first source.
          expect(anchor.pins[0]!.node, order).toBe(compiled.pinNode.get(corePinKey(first.id, first.spec.minus)));
          expect(c.components[anchor.componentIndex]!.id, order).toBe(first.id);
          // And not the fallback: the section's lowest node is rl.a's.
          expect(anchor.pins[0]!.node, order).toBeGreaterThan(compiled.pinNode.get(corePinKey("rl", "a"))!);
          nodes[order] = anchor.pins[0]!.node;
        }
        // Moving the sources moved the anchor.
        expect(nodes["a-first"]).not.toBe(nodes["b-first"]);
      });
    }

    it("a source with no return pin is skipped: the first one that has one anchors", () => {
      // `sx` has pins `p` and `q`, so classic reads no return pin for it.
      const c = circuit(
        [
          part("bat0", "battery_pack", ["pos", "neg"], { voltage: 5 }),
          part("r0", "resistor", ["a", "b"]),
          part("sx", "voltage_source", ["p", "q"]),
          part("sa", "battery_pack", ["pos", "neg"]),
          part("rl", "resistor", ["a", "b"]),
        ],
        [
          wire("bat0", "pos", "r0", "a"),
          wire("r0", "b", "bat0", "neg"),
          wire("sa", "pos", "rl", "a"),
          wire("rl", "b", "sx", "p"),
          wire("sx", "q", "sa", "neg"),
        ],
      );
      const { compiled, anchors } = expectAnchorsMatchClassic(c);
      expect(anchors).toHaveLength(1);
      expect(anchors[0]!.pins[0]!.node).toBe(compiled.pinNode.get(corePinKey("sa", "neg")));
      expect(c.components[anchors[0]!.componentIndex]!.id).toBe("sa");
    });
  });

  describe("a wire naming a component the circuit does not hold", () => {
    const board = () => loop(part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }));
    const withWires = (c: SimCircuit, ...wires: SimCircuit["wires"]): SimCircuit => ({ ...c, wires: [...c.wires, ...wires] });

    it("an ownerless net declines unknown-component for each missing component, and classic keeps the circuit", () => {
      const c = withWires(board(), wire("ghost1", "x", "ghost2", "y"));
      expect(compileDeclined(c)).toEqual(["malformed:ghost1:unknown-component", "malformed:ghost2:unknown-component"]);
      // Classic loads it, runs the real loop at 5 V, and anchors the ghost net
      // as one more isolated section (row 1: n1, which holds only ghost pins).
      const engine = loadClassic(c);
      engine.step(1e-4);
      expect(engine.getNetV()["n0"]).toBeCloseTo(5, 9);
      expect(engine.getNetV()["n1"]).toBeCloseTo(0, 9);
      expect(engine.isolatedSectionAnchorRows()).toEqual([1]);
    });

    it("a circuit of nothing but wires to absent components declines, and never throws", () => {
      const c = circuit([], [wire("ghost1", "x", "ghost2", "y")]);
      expect(compileDeclined(c)).toEqual(["malformed:ghost1:unknown-component", "malformed:ghost2:unknown-component"]);
      expect(() => loadClassic(c)).not.toThrow();
    });

    it("a ghost pin on a net a real component also holds has an owner: it compiles, as before, and classic anchors nothing", () => {
      const c = withWires(board(), wire("ghost1", "x", "r1", "a"));
      const { anchors } = expectAnchorsMatchClassic(c);
      expect(anchors).toEqual([]);
    });

    it("a wire to a pin the real component does not declare compiles too", () => {
      const c = withWires(board(), wire("r1", "nope", "r1", "a"));
      expect(expectAnchorsMatchClassic(c).anchors).toEqual([]);
    });

    it("never throws on random circuits wired to absent components, undeclared pins and one-pin parts", () => {
      // A fixed-seed LCG: the same 600 circuits every run.
      let seed = 20260930;
      const next = () => {
        seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
        return seed / 2 ** 32;
      };
      const pick = <T,>(items: readonly T[]): T => items[Math.floor(next() * items.length)]!;
      const kinds: Array<[string, string[]]> = [
        ["battery_pack", ["pos", "neg"]],
        ["bench_psu", ["pos", "neg"]],
        ["clock", ["out", "gnd"]],
        ["pulse_source", ["pos", "neg"]],
        ["resistor", ["a", "b"]],
        ["voltage_source", ["pos", "neg"]],
        ["led", ["anode", "cathode"]],
      ];
      const pinNames = ["a", "b", "pos", "neg", "out", "gnd", "anode", "cathode", "undeclared"];
      for (let i = 0; i < 600; i++) {
        const components = Array.from({ length: 1 + Math.floor(next() * 5) }, (_, index) => {
          const [kind, pins] = pick(kinds);
          return part(`c${index}`, kind, next() < 0.1 ? pins.slice(0, 1) : pins);
        });
        const ids = [...components.map((component) => component.id), "ghostA", "ghostB"];
        const wires = Array.from({ length: Math.floor(next() * 6) }, () => wire(pick(ids), pick(pinNames), pick(ids), pick(pinNames)));
        const result = compileCircuit(circuit(components, wires), WITH_SOURCES);
        if (!result.ok) {
          for (const reason of result.reasons) expect(reason).toMatch(/^malformed:[^:]+:(too-few-pins|unknown-component)$/);
        }
      }
    });
  });

  it("findSectionAnchors returns the same list the program carries", () => {
    const c = secondBatteryBoard();
    const found = findSectionAnchors(c, buildCoreNodes(c));
    expect(found).toHaveLength(1);
    expect(found[0]).toMatchObject({ componentId: "bat2", kind: "battery_pack", pinId: "neg", componentIndex: 2 });
  });

  it("a declined circuit carries no anchors: reasons and nothing else, though without the declined part it would have them", () => {
    const board = secondBatteryBoard();
    // A third battery on no wire at all: an isolated section of its own.
    const third = (pins: string[]) => part("bat3", "battery_pack", pins, { voltage: 3 });
    const declined = circuit([...board.components, third(["pos", "x"])], board.wires);
    // The result is exactly the reasons: no `compiled`, so no program and no anchor records.
    expect(compileCircuit(declined, WITH_SOURCES)).toEqual({ ok: false, reasons: ["malformed:bat3:missing-pin"] });
    // The control: the same circuit with a valid third battery compiles, and each
    // of its two isolated sections (bat2's and bat3's) gets an anchor.
    const control = circuit([...board.components, third(["pos", "neg"])], board.wires);
    expect(expectAnchorsMatchClassic(control).anchors).toHaveLength(2);
  });

  it("no kind the compiler emits declares galvanicGroups, which is what makes each component one group", () => {
    for (const kind of WITH_SOURCES) {
      expect(getDeviceModel(kind)?.galvanicGroups, kind).toBeUndefined();
    }
  });
});

// ─────────────────────────────────────────────────────────────────────────
// Duplicate component ids
// ─────────────────────────────────────────────────────────────────────────

describe("a component id two components share", () => {
  /** The usual one-supply loop with a second component of the same id wired across the same pins (they are the same pins: a pin is keyed by component id and pin id). */
  function withTwin(source: SimComponent): SimCircuit {
    const base = loop(source);
    return { ...base, components: [base.components[0]!, source, base.components[1]!] };
  }

  const SOURCES: Array<[string, string[], Record<string, number | string>]> = [
    ["battery_pack", ["pos", "neg"], { voltage: 5 }],
    ["bench_psu", ["pos", "neg"], { voltage: 5 }],
    ["clock", ["out", "gnd"], {}],
    ["clock_gen", ["out", "gnd"], {}],
    ["pulse_source", ["pos", "neg"], {}],
    ["pulse_gen", ["pos", "neg"], {}],
    ["voltage_source", ["pos", "neg"], { voltage: 5 }],
  ];

  it("classic cannot solve a duplicated source id: the second claim on vsrcIdx leaves the first one's branch row unstamped", () => {
    // Probed per kind. `vsrcIdx.set(id, row)` is last-wins, so the first
    // source's row is allocated and nothing stamps it: singular, and
    // `lastConverged` stays false. The unique-id loop is the control.
    for (const [kind, pins, params] of SOURCES) {
      const control = loadClassic(loop(part("s", kind, pins, params)));
      control.step(1e-4);
      expect(control.lastConverged, `${kind}: one source`).toBe(true);

      const twin = loadClassic(withTwin(part("s", kind, pins, params)));
      twin.step(1e-4);
      expect(twin.lastConverged, `${kind}: two sources with one id`).toBe(false);
    }
  });

  it("the compiler declines it, once per id, whatever the kind and however many components carry it", () => {
    for (const [kind, pins, params] of SOURCES) {
      expect(compileDeclined(withTwin(part("s", kind, pins, params))), kind).toEqual(["malformed:s:duplicate-id"]);
    }
    // Three of one id is one reason; two ids give two, sorted.
    const three = circuit([
      part("r1", "resistor", ["a", "b"]),
      part("r1", "resistor", ["a", "b"]),
      part("r1", "resistor", ["a", "b"]),
      part("r0", "resistor", ["a", "b"]),
      part("r0", "resistor", ["a", "b"]),
    ]);
    expect(compileDeclined(three)).toEqual(["malformed:r0:duplicate-id", "malformed:r1:duplicate-id"]);
    // Two kinds sharing an id (distinct pin names, so the pins do not even alias) decline too.
    const mixed = circuit([part("x", "battery_pack", ["pos", "neg"], { voltage: 5 }), part("x", "resistor", ["a", "b"])]);
    expect(compileDeclined(mixed)).toEqual(["malformed:x:duplicate-id"]);
    // Ids that differ only by case or by a trailing character are distinct components.
    const distinct = circuit([part("R1", "resistor", ["a", "b"]), part("r1", "resistor", ["a", "b"]), part("r1 ", "resistor", ["a", "b"])]);
    expect(compileCircuit(distinct, WITH_SOURCES).ok).toBe(true);
  });

  it("a duplicated passive id is one classic does solve (the pins alias, the stamps add), so the decline is the core's own policy, not classic's", () => {
    // The core's tables are id-keyed (pinNode, digital slots, batteries,
    // elementIKeys), so it declines every duplicate rather than only the
    // source kinds classic cannot solve; the circuit then runs on classic.
    const c = circuit(
      [
        part("v1", "voltage_source", ["pos", "neg"], { voltage: 5 }),
        part("r1", "resistor", ["a", "b"], { resistance: 1000 }),
        part("r1", "resistor", ["a", "b"], { resistance: 2000 }),
      ],
      [wire("v1", "pos", "r1", "a"), wire("r1", "b", "v1", "neg")],
    );
    const engine = loadClassic(c);
    engine.step(1e-4);
    expect(engine.lastConverged).toBe(true);
    expect(engine.getNetV()["n0"]).toBeCloseTo(5, 9);
    expect(compileDeclined(c)).toEqual(["malformed:r1:duplicate-id"]);
  });

  it("findSectionAnchors, reached directly, owns a lowest-node anchor by the FIRST component of its id in circuit order", () => {
    // Two resistors both named "r" with the same pins: one section with no
    // source and no datum, so the fallback anchor. Classic keys the load by id
    // and anchors row 1 (the section is the one ghost-free pair). The README's
    // owner is the first in circuit order, index 2 here, not the last (3): a
    // `new Map(entries)` index would say 3.
    const c = circuit(
      [
        part("bat", "battery_pack", ["pos", "neg"], { voltage: 5 }),
        part("rl", "resistor", ["a", "b"], { resistance: 1000 }),
        part("r", "resistor", ["a", "b"], { resistance: 1000 }),
        part("r", "resistor", ["a", "b"], { resistance: 2000 }),
      ],
      [wire("bat", "pos", "rl", "a"), wire("rl", "b", "bat", "neg")],
    );
    expect(loadClassic(c).isolatedSectionAnchorRows()).toEqual([1]);
    const found = findSectionAnchors(c, buildCoreNodes(c));
    expect(found).toHaveLength(1);
    expect(found[0]).toMatchObject({ node: 2, componentId: "r", kind: "resistor", componentIndex: 2 });
    // compileCircuit never gets that far: the id is declined first.
    expect(compileDeclined(c)).toEqual(["malformed:r:duplicate-id"]);
  });
});

// ─────────────────────────────────────────────────────────────────────────
// Shapes and the table blob
// ─────────────────────────────────────────────────────────────────────────

describe("kinds.ts shapes for codes 20-26 mirror the README", () => {
  it("has the pin, param and blob shape of every source row, and no live params", () => {
    const shape = (code: number) => CORE_RECORD_SHAPES.get(code)!;
    expect(shape(20)).toMatchObject({ pins: 2, pinsMax: 2, params: 9, blob: { table: true }, liveParams: [] });
    expect(shape(21)).toMatchObject({ pins: 2, pinsMax: 2, params: 2, blob: { exact: 0 }, liveParams: [] });
    for (const code of [22, 23]) {
      expect(shape(code)).toMatchObject({ pins: 2, pinsMax: 2, params: 5, blob: { exact: 1 }, liveParams: [], digitalSlots: 1 });
    }
    for (const code of [24, 25]) {
      expect(shape(code)).toMatchObject({ pins: 2, pinsMax: 2, params: 7, blob: { exact: 1 }, liveParams: [], digitalSlots: 1 });
    }
    expect(shape(26)).toMatchObject({ pins: 1, pinsMax: 1, params: 0, blob: { exact: 0 }, liveParams: [] });
    for (const code of [20, 21, 22, 23, 24, 25, 26]) expect(shape(code).channelSlots ?? 0).toBe(0);
    expect(CORE_KIND_CODE.section_anchor).toBe(26);
  });

  it("the writer and the decoder reject a table blob whose n and length disagree", () => {
    const pin = { node: 0, open: false, openHigh: false, openHash: 0, deadHash: 0 };
    const battery = (blob: number[]) => {
      const writer = new ProgramWriter(1);
      writer.addDevice({ kindCode: 20, componentIndex: 0, pins: [pin, pin], params: new Array(9).fill(1), blob });
    };
    expect(() => battery([2, 0, 0, 1, 5])).not.toThrow();
    expect(() => battery([])).toThrow(/blob of 1 \+ 2n/);
    expect(() => battery([2, 0, 0, 1])).toThrow(/1 \+ 2n/);
    expect(() => battery([3, 0, 0, 1, 5])).toThrow(/n\) is 3/);
    expect(() => battery([1.5, 0, 0])).toThrow(/n\) is 1.5/);

    // A program tampered after writing: n no longer matches the blob length.
    const writer = new ProgramWriter(1);
    writer.addDevice({ kindCode: 20, componentIndex: 0, pins: [pin, pin], params: new Array(9).fill(1), blob: [2, 0, 0, 1, 5] });
    const program = writer.finish(0, 0);
    const at = program.indexOf(2, program.length - 8);
    const tampered = program.slice();
    tampered[at] = 3;
    expect(() => decodeProgram(tampered)).toThrow(/n\) is 3/);
  });
});
