import { describe, expect, it } from "vitest";
import rawCatalog from "../helpers/catalog.js";
import type { PartCatalog } from "../../src/circuit/types.js";
import {
  advancePrimaryBatteryState,
  batteryInternalResistanceOhm,
  batteryOpenCircuitVoltageV,
  batteryOperatingPoint,
  batteryTerminalVoltageV,
  createBatteryState,
  resolveBatteryProfile,
} from "../../src/sim/battery-physics.js";

const catalog = rawCatalog as unknown as PartCatalog;

function exact(uid: string) {
  const resolution = resolveBatteryProfile({ catalogUid: uid });
  expect(resolution.source).toBe("exact-catalog-uid");
  expect(resolution.warning).toBeNull();
  return resolution.profile;
}

describe("battery profile resolution", () => {
  it.each([
    ["supply-5v", "alkaline-aa-4s", "alkaline-zn-mno2", 4],
    ["battery-9v", "alkaline-pp3-9v", "alkaline-zn-mno2", 6],
    ["battery-coin-cr2032", "lithium-cr2032", "lithium-mno2", 1],
  ] as const)("resolves exact catalog UID %s", (uid, id, chemistry, cellCount) => {
    expect(exact(uid)).toMatchObject({
      id,
      catalogUid: uid,
      chemistry,
      cellCount,
      rechargeable: false,
    });
  });

  it("uses an explicit, caller-parameterised generic fallback for missing identity", () => {
    const resolution = resolveBatteryProfile({
      fallbackNominalVoltageV: 12,
      fallbackCapacityAh: 7,
      fallbackFreshInternalResistanceOhm: 0.08,
    });

    expect(resolution).toMatchObject({
      source: "generic-fallback",
      requestedCatalogUid: null,
      profile: {
        id: "generic-primary",
        chemistry: "generic-primary",
        nominalVoltageV: 12,
        referenceCapacityAh: 7,
        freshInternalResistanceOhm: 0.08,
      },
    });
    expect(resolution.warning).toMatch(/no catalogUid/i);
    expect(resolution.profile.assumptions[0]).toMatch(/No chemistry was inferred/);
  });

  it("does not infer chemistry from an unknown or non-battery UID", () => {
    const resolution = resolveBatteryProfile({ catalogUid: "led-red" });
    expect(resolution.source).toBe("generic-fallback");
    expect(resolution.requestedCatalogUid).toBe("led-red");
    expect(resolution.profile.chemistry).toBe("generic-primary");
    expect(resolution.warning).toContain("led-red");
  });

  it("keeps the AA catalog voltage envelope aligned with the exact profile", () => {
    const aaPart = catalog.parts.find((part) => part.uid === "supply-5v")!;
    const profile = exact("supply-5v");
    expect(aaPart.default_params.voltage).toBe(profile.nominalVoltageV);
    expect(aaPart.electrical_specs?.vcc_range).toEqual({
      min: 4.8,
      nominal: profile.nominalVoltageV,
      max: batteryOpenCircuitVoltageV(profile, 1),
    });
  });
});

describe("chemistry-shaped OCV curves", () => {
  it("fits the 4×AA alkaline envelope and interpolates analytically", () => {
    const aa = exact("supply-5v");
    expect(batteryOpenCircuitVoltageV(aa, 1)).toBeCloseTo(6.4, 12);
    expect(batteryOpenCircuitVoltageV(aa, 0.5)).toBeCloseTo(5.12, 12);
    expect(batteryOpenCircuitVoltageV(aa, 0.1)).toBeCloseTo(4.08, 12);
    // S=0.6 is halfway between the 0.5/5.12 V and 0.7/5.52 V anchors.
    expect(batteryOpenCircuitVoltageV(aa, 0.6)).toBeCloseTo(5.32, 12);
    expect(batteryOpenCircuitVoltageV(aa, 0)).toBe(0);
  });

  it("fits the PP3 six-cell alkaline envelope down to its 4.8 V cutoff", () => {
    const pp3 = exact("battery-9v");
    expect(batteryOpenCircuitVoltageV(pp3, 1)).toBeCloseTo(9.6, 12);
    expect(batteryOpenCircuitVoltageV(pp3, 0.5)).toBeCloseTo(7.68, 12);
    expect(batteryOpenCircuitVoltageV(pp3, 0.02)).toBeCloseTo(4.8, 12);
    expect(batteryOpenCircuitVoltageV(pp3, 0)).toBe(0);
  });

  it("keeps CR2032 voltage on a lithium plateau before the end-of-life knee", () => {
    const coin = exact("battery-coin-cr2032");
    expect(batteryOpenCircuitVoltageV(coin, 1)).toBeCloseTo(3.2, 12);
    expect(batteryOpenCircuitVoltageV(coin, 0.5)).toBeCloseTo(2.98, 12);
    expect(batteryOpenCircuitVoltageV(coin, 0.1)).toBeCloseTo(2.75, 12);
    expect(batteryOpenCircuitVoltageV(coin, 0.02)).toBeCloseTo(2, 12);
    expect(3.2 - batteryOpenCircuitVoltageV(coin, 0.5)).toBeLessThan(0.25);
  });

  it("keeps the generic fallback honestly linear", () => {
    const generic = resolveBatteryProfile({ fallbackNominalVoltageV: 5 }).profile;
    expect(batteryOpenCircuitVoltageV(generic, 0.5)).toBe(2.5);
  });
});

describe("depletion- and temperature-dependent internal resistance", () => {
  it("starts at the fresh reference resistance at full SoC and reference temperature", () => {
    expect(batteryInternalResistanceOhm(exact("supply-5v"), 1, 21)).toBeCloseTo(0.9, 12);
    expect(batteryInternalResistanceOhm(exact("battery-9v"), 1, 21)).toBeCloseTo(1.5, 12);
    expect(batteryInternalResistanceOhm(exact("battery-coin-cr2032"), 1, 21)).toBeCloseTo(10, 12);
  });

  it("matches the documented AA depletion equation at 20% SoC", () => {
    const aa = exact("supply-5v");
    // R = 0.9 * [1 + 0.7*(1-0.2)^2/(0.2+0.05)] = 2.5128 Ω.
    expect(batteryInternalResistanceOhm(aa, 0.2, 21)).toBeCloseTo(2.5128, 10);
  });

  it("rises monotonically toward depletion for every exact chemistry", () => {
    for (const uid of ["supply-5v", "battery-9v", "battery-coin-cr2032"]) {
      const profile = exact(uid);
      const full = batteryInternalResistanceOhm(profile, 1, 21);
      const half = batteryInternalResistanceOhm(profile, 0.5, 21);
      const empty = batteryInternalResistanceOhm(profile, 0, 21);
      expect(half).toBeGreaterThan(full);
      expect(empty).toBeGreaterThan(half);
      expect(Number.isFinite(empty)).toBe(true);
    }
  });

  it("applies the calibrated cold resistance multipliers and clamps outside rated temperature", () => {
    const aa = exact("supply-5v");
    const pp3 = exact("battery-9v");
    const coin = exact("battery-coin-cr2032");
    expect(batteryInternalResistanceOhm(aa, 1, -18)).toBeCloseTo(2.7, 10);
    expect(batteryInternalResistanceOhm(pp3, 1, -18)).toBeCloseTo(6, 10);
    expect(batteryInternalResistanceOhm(coin, 1, -30)).toBeCloseTo(50, 10);
    expect(batteryInternalResistanceOhm(coin, 1, -100)).toBeCloseTo(50, 10);
  });

  it("honours an explicit reference-resistance override without changing chemistry", () => {
    const coin = exact("battery-coin-cr2032");
    expect(batteryInternalResistanceOhm(coin, 1, 21, 15)).toBeCloseTo(15, 12);
  });
});

describe("primary-cell coulomb counting", () => {
  it.each([
    ["supply-5v", 2.5],
    ["battery-9v", 0.55],
    ["battery-coin-cr2032", 0.235],
  ] as const)("one capacity-Amp-hour for one hour depletes %s", (uid, currentA) => {
    const result = advancePrimaryBatteryState(exact(uid), createBatteryState(1), {
      dischargeCurrentA: currentA,
      dtSeconds: 3600,
    });
    expect(result.state.soc).toBe(0);
    expect(result.depleted).toBe(true);
    expect(result.acceptedDischargeCoulombs).toBeCloseTo(currentA * 3600, 10);
  });

  it("integrates partial discharge from an arbitrary initial SoC", () => {
    const aa = exact("supply-5v");
    // 2.5 Ah = 9000 C. A 1 A load for 900 s removes 900 C = 10% SoC.
    const result = advancePrimaryBatteryState(aa, createBatteryState(0.75), {
      dischargeCurrentA: 1,
      dtSeconds: 900,
    });
    expect(result.state.soc).toBeCloseTo(0.65, 12);
    expect(result.state.dischargedCoulombs).toBe(900);
  });

  it("clips accepted discharge to the remaining charge", () => {
    const coin = exact("battery-coin-cr2032");
    const result = advancePrimaryBatteryState(coin, createBatteryState(0.01), {
      dischargeCurrentA: 1,
      dtSeconds: 100,
    });
    // Remaining charge = 0.235 Ah * 3600 * 0.01 = 8.46 C.
    expect(result.requestedDischargeCoulombs).toBe(100);
    expect(result.acceptedDischargeCoulombs).toBeCloseTo(8.46, 12);
    expect(result.state.soc).toBe(0);
  });

  it("rejects reverse current without increasing SoC", () => {
    const pp3 = exact("battery-9v");
    const result = advancePrimaryBatteryState(pp3, createBatteryState(0.4), {
      dischargeCurrentA: -0.1,
      dtSeconds: 10,
    });
    expect(result.state.soc).toBe(0.4);
    expect(result.acceptedDischargeCoulombs).toBe(0);
    expect(result.rejectedRechargeCoulombs).toBe(1);
    expect(result.state.rejectedRechargeCoulombs).toBe(1);
  });

  it("treats a negative or non-finite timestep as no elapsed time", () => {
    const aa = exact("supply-5v");
    expect(advancePrimaryBatteryState(aa, createBatteryState(0.5), {
      dischargeCurrentA: 1,
      dtSeconds: -1,
    }).state.soc).toBe(0.5);
    expect(advancePrimaryBatteryState(aa, createBatteryState(0.5), {
      dischargeCurrentA: 1,
      dtSeconds: Number.NaN,
    }).state.soc).toBe(0.5);
  });
});

describe("Thevenin operating point", () => {
  it("combines AA OCV and resistance into an independently derived loaded voltage", () => {
    const aa = exact("supply-5v");
    const point = batteryOperatingPoint(aa, { soc: 0.5, ambientTemperatureC: 21 });
    // R(0.5,21C) = 0.9 * [1 + 0.7*0.5^2/0.55] = 1.186363636... Ω.
    // Vterm at 0.5 A = 5.12 - 0.5*1.186363636 = 4.526818182 V.
    expect(point.openCircuitVoltageV).toBeCloseTo(5.12, 12);
    expect(point.internalResistanceOhm).toBeCloseTo(1.1863636363636363, 12);
    expect(batteryTerminalVoltageV(point, 0.5)).toBeCloseTo(4.526818181818182, 12);
  });

  it("reports when ambient temperature is outside the profile envelope", () => {
    const coin = exact("battery-coin-cr2032");
    expect(batteryOperatingPoint(coin, { soc: 1, ambientTemperatureC: -40 })).toMatchObject({
      ambientTemperatureC: -40,
      modelTemperatureC: -30,
      temperatureWasClamped: true,
    });
  });
});
