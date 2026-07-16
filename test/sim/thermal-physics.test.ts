import { describe, expect, it } from "vitest";
import rawCatalog from "../helpers/catalog.js";
import {
  createThermalState,
  resolveThermalProfile,
  stepThermalDevice,
  thermalPowerDerating,
  thermalTimeConstantS,
  type ThermalDeviceProfile,
} from "../../src/sim/thermal-physics.js";

function exact(uid: string): ThermalDeviceProfile {
  const resolution = resolveThermalProfile({ catalogUid: uid });
  expect(resolution.source).toBe("exact-catalog-uid");
  expect(resolution.warning).toBeNull();
  return resolution.profile;
}

describe("thermal profile resolution", () => {
  it.each([
    ["reg-7805", "lm7805-to220", "linear-regulator"],
    ["reg-ams1117-33", "ams1117-sot223-module", "linear-regulator"],
    ["reg-ams1117-50", "ams1117-sot223-module", "linear-regulator"],
    ["lm317", "lm317-to220-module", "linear-regulator"],
    ["lm358", "lm358-pdip8", "analog-ic"],
    ["mcp6002", "mcp6002-pdip8", "analog-ic"],
    ["lm386", "lm386-pdip8", "analog-ic"],
    ["resistor", "axial-resistor-quarter-watt", "resistor"],
    ["led-red", "led-5mm-through-hole", "led"],
    ["led-blue", "led-5mm-through-hole", "led"],
    ["led-white", "led-5mm-through-hole", "led"],
  ] as const)("resolves exact package identity %s", (uid, id, deviceClass) => {
    expect(exact(uid)).toMatchObject({ id, catalogUid: uid, deviceClass });
  });

  it("registers every catalog LED as the exact shared package profile", () => {
    const ledUids = rawCatalog.parts
      .filter((part) => part.kind === "led")
      .map((part) => part.uid);
    expect(ledUids.length).toBeGreaterThan(0);
    for (const uid of ledUids) {
      expect(exact(uid)).toMatchObject({
        id: "led-5mm-through-hole",
        catalogUid: uid,
        deviceClass: "led",
      });
    }
  });

  it("keeps an unknown UID explicit and caller-parameterised", () => {
    const resolution = resolveThermalProfile({
      catalogUid: "mystery-power-part",
      fallbackRThetaJA_CPerW: 40,
      fallbackThermalCapacitanceJPerC: 2,
      fallbackRatedPowerW: 3,
      fallbackMaximumContinuousJunctionC: 140,
      fallbackShutdown: { tripTemperatureC: 160, restartTemperatureC: 145 },
    });

    expect(resolution).toMatchObject({
      source: "generic-fallback",
      requestedCatalogUid: "mystery-power-part",
      profile: {
        id: "generic-thermal-device",
        catalogUid: null,
        network: { rThetaJA_CPerW: 40, thermalCapacitanceJPerC: 2 },
        ratedPowerW: 3,
        shutdown: { tripTemperatureC: 160, restartTemperatureC: 145 },
      },
    });
    expect(resolution.warning).toContain("mystery-power-part");
    expect(resolution.profile.warnings[0]).toMatch(/No package was identified/);
  });

  it("does not invent invalid shutdown hysteresis for a generic fallback", () => {
    const profile = resolveThermalProfile({
      fallbackShutdown: { tripTemperatureC: 140, restartTemperatureC: 150 },
    }).profile;
    expect(profile.shutdown).toBeNull();
  });
});

describe("thermal network time constant", () => {
  it("derives tau from RθJA × Cth", () => {
    expect(thermalTimeConstantS({
      rThetaJA_CPerW: 10,
      thermalCapacitanceJPerC: 2,
    })).toBe(20);
  });

  it("uses an explicit tau in preference to Cth", () => {
    expect(thermalTimeConstantS({
      rThetaJA_CPerW: 10,
      thermalCapacitanceJPerC: 2,
      timeConstantS: 7,
    })).toBe(7);
  });
});

describe("first-order heating and cooling", () => {
  const generic = resolveThermalProfile({
    fallbackRThetaJA_CPerW: 10,
    fallbackTimeConstantS: 10,
    fallbackMaximumContinuousJunctionC: 150,
    fallbackRatedPowerW: 10,
  }).profile;

  it("matches the analytical one-time-constant heating solution", () => {
    const result = stepThermalDevice(generic, createThermalState(25), {
      ambientTemperatureC: 25,
      dissipatedPowerW: 2,
      dtSeconds: 10,
    });
    // Tinf=25+2*10=45°C. T(10s)=45+(25-45)e^-1=37.642411...°C.
    expect(result.targetTemperatureC).toBe(45);
    expect(result.state.temperatureC).toBeCloseTo(37.64241117657115, 12);
    expect(result.timeConstantS).toBe(10);
  });

  it("approaches ambient plus P×Rθ at steady state", () => {
    const result = stepThermalDevice(generic, createThermalState(25), {
      ambientTemperatureC: 25,
      dissipatedPowerW: 2,
      dtSeconds: 100,
    });
    expect(result.state.temperatureC).toBeCloseTo(44.99909200140475, 10);
  });

  it("cools exponentially when dissipation is removed", () => {
    const result = stepThermalDevice(generic, createThermalState(45), {
      ambientTemperatureC: 25,
      dissipatedPowerW: 0,
      dtSeconds: 10,
    });
    // 25 + (45-25)e^-1 = 32.3575888°C.
    expect(result.state.temperatureC).toBeCloseTo(32.35758882342885, 12);
  });

  it("does not advance on a negative timestep", () => {
    const result = stepThermalDevice(generic, createThermalState(40), {
      ambientTemperatureC: 25,
      dissipatedPowerW: 2,
      dtSeconds: -1,
    });
    expect(result.state.temperatureC).toBe(40);
  });
});

describe("reversible thermal shutdown hysteresis", () => {
  it("enters LM7805 shutdown when the integrated junction crosses 150°C", () => {
    const regulator = exact("reg-7805");
    const result = stepThermalDevice(regulator, createThermalState(149), {
      ambientTemperatureC: 25,
      dissipatedPowerW: 10,
      dtSeconds: 1,
    });
    expect(result.state.temperatureC).toBeGreaterThan(150);
    expect(result.state.thermalShutdown).toBe(true);
    expect(result.shutdownTransition).toBe("entered");
  });

  it("holds shutdown inside the hysteresis band", () => {
    const regulator = exact("reg-7805");
    const result = stepThermalDevice(
      regulator,
      { temperatureC: 140, thermalShutdown: true },
      { ambientTemperatureC: 25, dissipatedPowerW: 0, dtSeconds: 0 },
    );
    expect(result.state.thermalShutdown).toBe(true);
    expect(result.shutdownTransition).toBeNull();
  });

  it("restarts only after cooling to the restart threshold", () => {
    const regulator = exact("reg-7805");
    const result = stepThermalDevice(
      regulator,
      { temperatureC: 135, thermalShutdown: true },
      { ambientTemperatureC: 25, dissipatedPowerW: 0, dtSeconds: 0 },
    );
    expect(result.state.thermalShutdown).toBe(false);
    expect(result.shutdownTransition).toBe("exited");
  });

  it("uses the higher AMS1117 protection threshold", () => {
    const ams = exact("reg-ams1117-33");
    expect(ams.shutdown).toEqual({ tripTemperatureC: 165, restartTemperatureC: 145 });
  });

  it("does not invent shutdown for an ordinary resistor", () => {
    const resistor = exact("resistor");
    const result = stepThermalDevice(
      resistor,
      { temperatureC: 500, thermalShutdown: true },
      { ambientTemperatureC: 25, dissipatedPowerW: 0, dtSeconds: 0 },
    );
    expect(result.state.thermalShutdown).toBe(false);
    expect(result.shutdownTransition).toBeNull();
  });
});

describe("ambient/package power derating", () => {
  it("reproduces the IEC quarter-watt resistor P70 curve", () => {
    const resistor = exact("resistor");
    expect(thermalPowerDerating(resistor, 70)).toMatchObject({
      thermalLimitPowerW: 0.25,
      allowedPowerW: 0.25,
      deratingFactor: 1,
    });
    expect(thermalPowerDerating(resistor, 112.5)).toMatchObject({
      thermalLimitPowerW: 0.125,
      allowedPowerW: 0.125,
      deratingFactor: 0.5,
    });
    expect(thermalPowerDerating(resistor, 155).allowedPowerW).toBe(0);
  });

  it("derates a headline 15 W 7805 to its installed TO-220 thermal path", () => {
    const regulator = exact("reg-7805");
    const result = thermalPowerDerating(regulator, 25);
    // (125-25)/23.9 = 4.184100418 W; the 15 W nameplate does not override RθJA.
    expect(result.allowedPowerW).toBeCloseTo(4.184100418410042, 12);
    expect(result.deratingFactor).toBeCloseTo(0.2789400278940028, 12);
  });

  it("makes a 1 W AMS1117 module ambient-limited above 35°C", () => {
    const ams = exact("reg-ams1117-50");
    expect(thermalPowerDerating(ams, 25).allowedPowerW).toBe(1);
    expect(thermalPowerDerating(ams, 80)).toMatchObject({
      allowedPowerW: 0.5,
      deratingFactor: 0.5,
    });
  });

  it("applies the 5 mm LED package envelope at its 85°C ambient limit", () => {
    const led = exact("led-red");
    const result = thermalPowerDerating(led, 85);
    expect(result.allowedPowerW).toBeCloseTo(40 / 560, 12);
    expect(result.deratingFactor).toBeCloseTo((40 / 560) / 0.075, 12);
  });

  it("returns dynamic warnings and invalidity when limits are exceeded", () => {
    const resistor = exact("resistor");
    const result = stepThermalDevice(resistor, createThermalState(25), {
      ambientTemperatureC: 200,
      dissipatedPowerW: 1,
      dtSeconds: 100,
    });
    expect(result.withinContinuousLimits).toBe(false);
    expect(result.warnings.some((warning) => warning.includes("outside"))).toBe(true);
    expect(result.warnings.some((warning) => warning.includes("exceeds"))).toBe(true);
  });
});
