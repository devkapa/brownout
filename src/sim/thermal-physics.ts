/**
 * Pure first-order package thermal physics.
 *
 * The lumped junction/body model is:
 *
 *   T_inf = T_ambient + P_dissipated * R_theta_JA
 *   T_next = T_inf + (T_previous - T_inf) * exp(-dt / tau)
 *   tau = R_theta_JA * C_thermal
 *
 * A caller may provide either `timeConstantS` directly or thermal capacitance
 * `thermalCapacitanceJPerC`; an explicit time constant takes precedence. This
 * is a one-pole teaching model, not a replacement for a package's multi-pole
 * transient thermal-impedance graph.
 *
 * Continuous power derating follows the standard thermal constraint:
 *
 *   P_thermal_max(Ta) = max(0, (Tj_continuous_max - Ta) / R_theta_JA)
 *   P_allowed = min(P_rated, P_thermal_max)
 *
 * Sources for package limits and typical RθJA values:
 * - TI LM7805/LM340: https://www.ti.com/lit/ds/symlink/lm7800.pdf
 * - TI LM317: https://www.ti.com/lit/ds/symlink/lm317.pdf
 * - TI LM358: https://www.ti.com/lit/ds/symlink/lm358.pdf
 * - TI LM386: https://www.ti.com/lit/ds/symlink/lm386.pdf
 * - Microchip MCP6002: https://ww1.microchip.com/downloads/aemDocuments/documents/MSLD/ProductDocuments/DataSheets/MCP6001-1R-1U-2-4-1-MHz-Low-Power-Op-Amp-DS20001733L.pdf
 * - AMS1117 family: https://www.advanced-monolithic.com/products/voltreg.html
 * - Kingbright 5 mm LED example: https://www.kingbrightusa.com/images/catalog/spec/wp7113id.pdf
 * - Vishay/IEC resistor derating explanation: https://www.vishay.com/en/landingpage/rifaq/index.html
 *
 * Datasheets rarely specify a single package time constant, shutdown restart
 * threshold, or breadboard/module RθJA. Those values are explicitly marked as
 * assumptions in each profile and surfaced as warnings.
 */

import { getPartLibrary, getPartLibraryVersion } from "../parts/part-library.js";

export type ThermalProfileId =
  | "lm7805-to220"
  | "ams1117-sot223-module"
  | "lm317-to220-module"
  | "lm358-pdip8"
  | "mcp6002-pdip8"
  | "lm386-pdip8"
  | "axial-resistor-quarter-watt"
  | "led-5mm-through-hole"
  | "generic-thermal-device";

export type ThermalDeviceClass =
  | "linear-regulator"
  | "analog-ic"
  | "resistor"
  | "led"
  | "generic";

export interface ThermalNetwork {
  /** Junction/body-to-ambient thermal resistance. */
  rThetaJA_CPerW: number;
  /** Optional lumped heat capacity. Used when timeConstantS is absent. */
  thermalCapacitanceJPerC?: number;
  /** Optional first-order time constant. Takes precedence over Cth. */
  timeConstantS?: number;
}

export interface ThermalShutdownThresholds {
  tripTemperatureC: number;
  restartTemperatureC: number;
}

export interface ThermalValidityBounds {
  recommendedAmbientTemperatureC: { min: number; max: number };
  maximumContinuousJunctionTemperatureC: number;
  absoluteMaximumJunctionTemperatureC: number;
  rThetaJACondition: string;
}

export interface ThermalDeviceProfile {
  id: ThermalProfileId;
  catalogUid: string | null;
  label: string;
  deviceClass: ThermalDeviceClass;
  network: ThermalNetwork;
  /** Nameplate/package power rating before ambient/package derating. */
  ratedPowerW: number | null;
  shutdown: ThermalShutdownThresholds | null;
  validity: ThermalValidityBounds;
  warnings: readonly string[];
}

export interface ResolveThermalProfileInput {
  catalogUid?: string | null;
  /** Generic fallback assumptions only. */
  fallbackRThetaJA_CPerW?: number;
  fallbackThermalCapacitanceJPerC?: number;
  fallbackTimeConstantS?: number;
  fallbackRatedPowerW?: number;
  fallbackMaximumContinuousJunctionC?: number;
  fallbackAbsoluteMaximumJunctionC?: number;
  fallbackAmbientTemperatureC?: { min: number; max: number };
  fallbackShutdown?: ThermalShutdownThresholds | null;
}

export interface ThermalProfileResolution {
  profile: ThermalDeviceProfile;
  source: "exact-catalog-uid" | "generic-fallback";
  requestedCatalogUid: string | null;
  warning: string | null;
}

export interface ThermalDeviceState {
  temperatureC: number;
  thermalShutdown: boolean;
}

export interface ThermalPowerDerating {
  ambientTemperatureC: number;
  thermalLimitPowerW: number;
  allowedPowerW: number;
  deratingFactor: number | null;
  ambientWithinRecommendedRange: boolean;
  warnings: string[];
}

export interface ThermalStepInput {
  ambientTemperatureC: number;
  dissipatedPowerW: number;
  dtSeconds: number;
  /** Optional installed-package/heatsink override. */
  network?: ThermalNetwork;
}

export interface ThermalStepResult {
  state: ThermalDeviceState;
  targetTemperatureC: number;
  timeConstantS: number;
  shutdownTransition: "entered" | "exited" | null;
  derating: ThermalPowerDerating;
  withinContinuousLimits: boolean;
  warnings: string[];
}

const FIRST_ORDER_WARNING =
  "The package time constant is a one-pole teaching assumption; real transient thermal impedance is multi-pole.";

function finitePositive(value: number | undefined, fallback: number): number {
  return value != null && Number.isFinite(value) && value > 0 ? value : fallback;
}

function finiteNonNegative(value: number | undefined, fallback: number): number {
  return value != null && Number.isFinite(value) && value >= 0 ? value : fallback;
}

function finiteTemperature(value: number, fallback = 25): number {
  return Number.isFinite(value) ? value : fallback;
}

function exactProfile(
  profile: Omit<ThermalDeviceProfile, "catalogUid">,
  catalogUid: string,
): ThermalDeviceProfile {
  return { ...profile, catalogUid };
}

const LM7805_BASE: Omit<ThermalDeviceProfile, "catalogUid"> = {
  id: "lm7805-to220",
  label: "LM7805 / LM340 TO-220",
  deviceClass: "linear-regulator",
  // TI NDE TO-220 typical RθJA = 23.9 °C/W. Tau is a compact package/board fit.
  network: { rThetaJA_CPerW: 23.9, timeConstantS: 15 },
  ratedPowerW: 15,
  shutdown: { tripTemperatureC: 150, restartTemperatureC: 135 },
  validity: {
    recommendedAmbientTemperatureC: { min: 0, max: 125 },
    maximumContinuousJunctionTemperatureC: 125,
    absoluteMaximumJunctionTemperatureC: 150,
    rThetaJACondition: "TI NDE TO-220 JEDEC board metric; mounting and heatsinking can dominate.",
  },
  warnings: [
    FIRST_ORDER_WARNING,
    "The 15 W headline rating requires suitable heatsinking; default-package derating is authoritative here.",
    "150→135 °C shutdown hysteresis uses the specified trip point and an assumed restart point.",
  ],
};

const AMS1117_BASE: Omit<ThermalDeviceProfile, "catalogUid"> = {
  id: "ams1117-sot223-module",
  label: "AMS1117 SOT-223 breakout module",
  deviceClass: "linear-regulator",
  // The original AMS1117 sheet gives about 90 °C/W for minimally mounted
  // SOT-223; copper-rich modules can be materially lower.
  network: { rThetaJA_CPerW: 90, timeConstantS: 8 },
  ratedPowerW: 1,
  shutdown: { tripTemperatureC: 165, restartTemperatureC: 145 },
  validity: {
    recommendedAmbientTemperatureC: { min: 0, max: 125 },
    maximumContinuousJunctionTemperatureC: 125,
    absoluteMaximumJunctionTemperatureC: 150,
    rThetaJACondition: "Assumes a small labeled SOT-223 carrier; PCB copper may move RθJA roughly 55–90 °C/W.",
  },
  warnings: [
    FIRST_ORDER_WARNING,
    "SOT-223 RθJA is strongly dependent on carrier copper area and airflow.",
    "The 165 °C protection point is above the 150 °C absolute rating; shutdown is protection, not an operating target.",
    "The 145 °C restart point is a documented compact-model assumption.",
  ],
};

const LM317_BASE: Omit<ThermalDeviceProfile, "catalogUid"> = {
  id: "lm317-to220-module",
  label: "LM317 TO-220 breakout module",
  deviceClass: "linear-regulator",
  // TI KCT TO-220 metric; variants/boards span about 23.5–66.8 °C/W.
  network: { rThetaJA_CPerW: 37.9, timeConstantS: 15 },
  ratedPowerW: 20,
  shutdown: { tripTemperatureC: 150, restartTemperatureC: 135 },
  validity: {
    recommendedAmbientTemperatureC: { min: 0, max: 125 },
    maximumContinuousJunctionTemperatureC: 125,
    absoluteMaximumJunctionTemperatureC: 150,
    rThetaJACondition: "Representative TI TO-220 metric on a board; breakout and heatsink construction are unspecified.",
  },
  warnings: [
    FIRST_ORDER_WARNING,
    "The catalog's 20 W rating is heatsink-dependent and is derated by the installed thermal network.",
    "TI specifies internal thermal shutdown but not one universal restart threshold; 150→135 °C is assumed.",
  ],
};

const LM358_BASE: Omit<ThermalDeviceProfile, "catalogUid"> = {
  id: "lm358-pdip8",
  label: "LM358 PDIP-8",
  deviceClass: "analog-ic",
  network: { rThetaJA_CPerW: 80.9, timeConstantS: 12 },
  ratedPowerW: 0.83,
  shutdown: null,
  validity: {
    recommendedAmbientTemperatureC: { min: 0, max: 70 },
    maximumContinuousJunctionTemperatureC: 125,
    absoluteMaximumJunctionTemperatureC: 150,
    rThetaJACondition: "TI P-package PDIP-8 JEDEC thermal metric.",
  },
  warnings: [FIRST_ORDER_WARNING, "No reversible thermal shutdown is claimed for this generic LM358 profile."],
};

const MCP6002_BASE: Omit<ThermalDeviceProfile, "catalogUid"> = {
  id: "mcp6002-pdip8",
  label: "MCP6002 PDIP-8",
  deviceClass: "analog-ic",
  network: { rThetaJA_CPerW: 85, timeConstantS: 12 },
  // Derived from 125 °C continuous-junction target at 25 °C and 85 °C/W.
  ratedPowerW: 1.18,
  shutdown: null,
  validity: {
    recommendedAmbientTemperatureC: { min: -40, max: 125 },
    maximumContinuousJunctionTemperatureC: 125,
    absoluteMaximumJunctionTemperatureC: 150,
    rThetaJACondition: "Microchip 8-lead PDIP package thermal resistance.",
  },
  warnings: [
    FIRST_ORDER_WARNING,
    "1.18 W is a thermal-envelope derivation, not a guaranteed functional output-power rating.",
    "No reversible thermal shutdown is claimed by the MCP6002 profile.",
  ],
};

const LM386_BASE: Omit<ThermalDeviceProfile, "catalogUid"> = {
  id: "lm386-pdip8",
  label: "LM386 PDIP-8",
  deviceClass: "analog-ic",
  network: { rThetaJA_CPerW: 53.4, timeConstantS: 12 },
  ratedPowerW: 1.25,
  shutdown: null,
  validity: {
    recommendedAmbientTemperatureC: { min: 0, max: 70 },
    maximumContinuousJunctionTemperatureC: 125,
    absoluteMaximumJunctionTemperatureC: 150,
    rThetaJACondition: "TI P-package PDIP-8 JEDEC thermal metric.",
  },
  warnings: [
    FIRST_ORDER_WARNING,
    "1.25 W package dissipation is a conservative teaching assumption; audio output power is not heat one-for-one.",
    "No reversible thermal shutdown is claimed by the generic LM386 profile.",
  ],
};

const RESISTOR_BASE: Omit<ThermalDeviceProfile, "catalogUid"> = {
  id: "axial-resistor-quarter-watt",
  label: "¼ W axial film resistor",
  deviceClass: "resistor",
  // IEC/Vishay: P70=0.25 W and film limit 155 °C. Rθ=(155-70)/0.25=340 °C/W.
  network: { rThetaJA_CPerW: 340, timeConstantS: 8 },
  ratedPowerW: 0.25,
  shutdown: null,
  validity: {
    recommendedAmbientTemperatureC: { min: -55, max: 155 },
    maximumContinuousJunctionTemperatureC: 155,
    absoluteMaximumJunctionTemperatureC: 155,
    rThetaJACondition: "Derived from IEC P70 quarter-watt derating to a 155 °C film hot spot.",
  },
  warnings: [
    FIRST_ORDER_WARNING,
    "RθJA is inferred from the standard derating curve; lead length, PCB copper, resistor technology, and airflow vary it.",
  ],
};

const LED_BASE: Omit<ThermalDeviceProfile, "catalogUid"> = {
  id: "led-5mm-through-hole",
  label: "5 mm through-hole indicator LED",
  deviceClass: "led",
  // Kingbright WP7113ID: 560 °C/W, 75 mW, Tj max 125 °C on its stated FR4 pads.
  network: { rThetaJA_CPerW: 560, timeConstantS: 2 },
  ratedPowerW: 0.075,
  shutdown: null,
  validity: {
    recommendedAmbientTemperatureC: { min: -40, max: 85 },
    maximumContinuousJunctionTemperatureC: 125,
    absoluteMaximumJunctionTemperatureC: 125,
    rThetaJACondition: "Kingbright 5 mm lamp mounted on FR4 with at least 16 mm² pad per lead.",
  },
  warnings: [
    FIRST_ORDER_WARNING,
    "All catalog LED colours share this package fit; die material changes exact thermal and optical behaviour.",
    "No reversible shutdown exists; exceeding the envelope represents degradation or failure risk.",
  ],
};

const staticExactProfiles: Record<string, ThermalDeviceProfile> = {
  "reg-7805": exactProfile(LM7805_BASE, "reg-7805"),
  "reg-ams1117-33": exactProfile(AMS1117_BASE, "reg-ams1117-33"),
  "reg-ams1117-50": exactProfile(AMS1117_BASE, "reg-ams1117-50"),
  lm317: exactProfile(LM317_BASE, "lm317"),
  lm358: exactProfile(LM358_BASE, "lm358"),
  mcp6002: exactProfile(MCP6002_BASE, "mcp6002"),
  lm386: exactProfile(LM386_BASE, "lm386"),
  resistor: exactProfile(RESISTOR_BASE, "resistor"),
};

// LED package profiles fan out over every kind==="led" catalog entry, so they
// depend on the INJECTED part library, not on a bundled catalog. The map is
// rebuilt lazily and keyed on the library version (injection-reactive rather
// than one-shot lazy) so a host that calls setPartLibrary() after this module
// was imported — or even after profiles were first resolved — still gets LED
// profiles for its own catalog uids on the next lookup.
let cachedProfiles: {
  version: number;
  map: Record<string, ThermalDeviceProfile>;
} | null = null;

function exactProfilesForActiveLibrary(): Record<string, ThermalDeviceProfile> {
  const version = getPartLibraryVersion();
  if (!cachedProfiles || cachedProfiles.version !== version) {
    const map: Record<string, ThermalDeviceProfile> = { ...staticExactProfiles };
    for (const part of getPartLibrary()) {
      if (part.kind === "led") map[part.uid] = exactProfile(LED_BASE, part.uid);
    }
    cachedProfiles = { version, map };
  }
  return cachedProfiles.map;
}

/**
 * Snapshot of the exact-uid profile map for the ACTIVE part library.
 * Replaces simcore's module-init THERMAL_PROFILES_BY_CATALOG_UID constant:
 * a constant cannot be injection-reactive, so the map is now behind a call.
 */
export function thermalProfilesByCatalogUid(): Readonly<Record<string, ThermalDeviceProfile>> {
  return exactProfilesForActiveLibrary();
}

function validShutdown(
  value: ThermalShutdownThresholds | null | undefined,
): ThermalShutdownThresholds | null {
  if (!value) return null;
  if (
    !Number.isFinite(value.tripTemperatureC) ||
    !Number.isFinite(value.restartTemperatureC) ||
    value.restartTemperatureC >= value.tripTemperatureC
  ) return null;
  return { ...value };
}

function genericProfile(input: ResolveThermalProfileInput): ThermalDeviceProfile {
  const rTheta = finitePositive(input.fallbackRThetaJA_CPerW, 100);
  const tau = input.fallbackTimeConstantS != null && input.fallbackTimeConstantS > 0
    ? input.fallbackTimeConstantS
    : undefined;
  const cth = input.fallbackThermalCapacitanceJPerC != null && input.fallbackThermalCapacitanceJPerC > 0
    ? input.fallbackThermalCapacitanceJPerC
    : undefined;
  const maxContinuous = finiteTemperature(input.fallbackMaximumContinuousJunctionC ?? 125, 125);
  const absoluteMax = Math.max(
    maxContinuous,
    finiteTemperature(input.fallbackAbsoluteMaximumJunctionC ?? 150, 150),
  );
  const ambient = input.fallbackAmbientTemperatureC ?? { min: -40, max: 85 };
  const ambientMin = finiteTemperature(ambient.min, -40);
  const ambientMax = Math.max(ambientMin, finiteTemperature(ambient.max, 85));

  return {
    id: "generic-thermal-device",
    catalogUid: null,
    label: "Generic thermal device (fallback)",
    deviceClass: "generic",
    network: {
      rThetaJA_CPerW: rTheta,
      ...(tau !== undefined ? { timeConstantS: tau } : {}),
      ...(cth !== undefined ? { thermalCapacitanceJPerC: cth } : {}),
      ...(tau === undefined && cth === undefined ? { timeConstantS: 10 } : {}),
    },
    ratedPowerW: input.fallbackRatedPowerW != null && input.fallbackRatedPowerW > 0
      ? input.fallbackRatedPowerW
      : null,
    shutdown: validShutdown(input.fallbackShutdown),
    validity: {
      recommendedAmbientTemperatureC: { min: ambientMin, max: ambientMax },
      maximumContinuousJunctionTemperatureC: maxContinuous,
      absoluteMaximumJunctionTemperatureC: absoluteMax,
      rThetaJACondition: "Caller/default generic assumption; no package identity is claimed.",
    },
    warnings: [
      "No package was identified; every thermal number is a caller/default generic assumption.",
      FIRST_ORDER_WARNING,
    ],
  };
}

export function resolveThermalProfile(
  input: ResolveThermalProfileInput = {},
): ThermalProfileResolution {
  const requestedCatalogUid = input.catalogUid?.trim() || null;
  const exact = requestedCatalogUid
    ? exactProfilesForActiveLibrary()[requestedCatalogUid]
    : undefined;
  if (exact) {
    return {
      profile: exact,
      source: "exact-catalog-uid",
      requestedCatalogUid,
      warning: null,
    };
  }

  return {
    profile: genericProfile(input),
    source: "generic-fallback",
    requestedCatalogUid,
    warning: requestedCatalogUid
      ? `No thermal package profile exists for catalogUid "${requestedCatalogUid}"; using an explicit generic fallback.`
      : "Component has no catalogUid; using an explicit generic thermal fallback.",
  };
}

export function thermalTimeConstantS(network: ThermalNetwork): number {
  if (network.timeConstantS != null && Number.isFinite(network.timeConstantS) && network.timeConstantS > 0) {
    return network.timeConstantS;
  }
  const rTheta = finitePositive(network.rThetaJA_CPerW, 1);
  const cth = finitePositive(network.thermalCapacitanceJPerC, 1);
  return rTheta * cth;
}

export function createThermalState(
  ambientTemperatureC = 25,
  thermalShutdown = false,
): ThermalDeviceState {
  return {
    temperatureC: finiteTemperature(ambientTemperatureC),
    thermalShutdown,
  };
}

export function thermalPowerDerating(
  profile: ThermalDeviceProfile,
  ambientTemperatureC: number,
  network: ThermalNetwork = profile.network,
): ThermalPowerDerating {
  const ambient = finiteTemperature(ambientTemperatureC);
  const rTheta = finitePositive(network.rThetaJA_CPerW, profile.network.rThetaJA_CPerW);
  const thermalLimitPowerW = Math.max(
    0,
    (profile.validity.maximumContinuousJunctionTemperatureC - ambient) / rTheta,
  );
  const allowedPowerW = profile.ratedPowerW == null
    ? thermalLimitPowerW
    : Math.min(profile.ratedPowerW, thermalLimitPowerW);
  const recommended = profile.validity.recommendedAmbientTemperatureC;
  const ambientWithinRecommendedRange = ambient >= recommended.min && ambient <= recommended.max;
  const warnings: string[] = [];
  if (!ambientWithinRecommendedRange) {
    warnings.push(
      `Ambient ${ambient.toFixed(1)} °C is outside the profile's recommended ${recommended.min}..${recommended.max} °C range.`,
    );
  }

  return {
    ambientTemperatureC: ambient,
    thermalLimitPowerW,
    allowedPowerW,
    deratingFactor: profile.ratedPowerW != null && profile.ratedPowerW > 0
      ? Math.max(0, Math.min(1, allowedPowerW / profile.ratedPowerW))
      : null,
    ambientWithinRecommendedRange,
    warnings,
  };
}

export function stepThermalDevice(
  profile: ThermalDeviceProfile,
  previous: ThermalDeviceState,
  input: ThermalStepInput,
): ThermalStepResult {
  const ambientTemperatureC = finiteTemperature(input.ambientTemperatureC);
  const dissipatedPowerW = finiteNonNegative(input.dissipatedPowerW, 0);
  const dtSeconds = finiteNonNegative(input.dtSeconds, 0);
  const network = input.network ?? profile.network;
  const rTheta = finitePositive(network.rThetaJA_CPerW, profile.network.rThetaJA_CPerW);
  const timeConstantS = thermalTimeConstantS(network);
  const previousTemperatureC = Number.isFinite(previous.temperatureC)
    ? previous.temperatureC
    : ambientTemperatureC;
  const targetTemperatureC = ambientTemperatureC + dissipatedPowerW * rTheta;
  const temperatureC = dtSeconds > 0
    ? targetTemperatureC +
      (previousTemperatureC - targetTemperatureC) * Math.exp(-dtSeconds / timeConstantS)
    : previousTemperatureC;

  let thermalShutdown = previous.thermalShutdown;
  let shutdownTransition: ThermalStepResult["shutdownTransition"] = null;
  if (profile.shutdown) {
    if (!thermalShutdown && temperatureC >= profile.shutdown.tripTemperatureC) {
      thermalShutdown = true;
      shutdownTransition = "entered";
    } else if (thermalShutdown && temperatureC <= profile.shutdown.restartTemperatureC) {
      thermalShutdown = false;
      shutdownTransition = "exited";
    }
  } else {
    thermalShutdown = false;
  }

  const derating = thermalPowerDerating(profile, ambientTemperatureC, network);
  const warnings = [...derating.warnings];
  if (dissipatedPowerW > derating.allowedPowerW) {
    warnings.push(
      `Dissipation ${dissipatedPowerW.toFixed(3)} W exceeds the ambient/package allowance ${derating.allowedPowerW.toFixed(3)} W.`,
    );
  }
  if (temperatureC > profile.validity.maximumContinuousJunctionTemperatureC) {
    warnings.push(
      `Temperature ${temperatureC.toFixed(1)} °C exceeds the ${profile.validity.maximumContinuousJunctionTemperatureC} °C continuous-junction limit.`,
    );
  }
  if (temperatureC > profile.validity.absoluteMaximumJunctionTemperatureC) {
    warnings.push(
      `Temperature ${temperatureC.toFixed(1)} °C exceeds the ${profile.validity.absoluteMaximumJunctionTemperatureC} °C absolute maximum.`,
    );
  }

  const withinContinuousLimits =
    derating.ambientWithinRecommendedRange &&
    dissipatedPowerW <= derating.allowedPowerW &&
    temperatureC <= profile.validity.maximumContinuousJunctionTemperatureC;

  return {
    state: { temperatureC, thermalShutdown },
    targetTemperatureC,
    timeConstantS,
    shutdownTransition,
    derating,
    withinContinuousLimits,
    warnings,
  };
}
