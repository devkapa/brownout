/**
 * Pure primary-battery physics used by the simulator.
 *
 * Scope and evidence
 * ------------------
 * The three exact profiles are compact teaching fits to manufacturer data:
 *
 * - Energizer E91 AA alkaline: https://data.energizer.com/pdfs/e91.pdf
 * - Energizer 522 PP3/6LR61: https://data.energizer.com/pdfs/522.pdf
 * - Energizer CR2032: https://data.energizer.com/pdfs/cr2032.pdf
 * - Alkaline resistance behaviour:
 *   https://data.energizer.com/pdfs/alkaline_appman.pdf
 *
 * Those sheets publish typical loaded discharge envelopes rather than a full
 * electrochemical equivalent circuit. We therefore use a deliberately small
 * Thevenin model:
 *
 *   V_terminal = V_oc(S) - I_discharge * R_internal(S, T)
 *
 * V_oc(S) is linearly interpolated through chemistry-shaped voltage/SoC
 * anchors. The final 2% of usable SoC collapses from the manufacturer's cutoff
 * voltage to zero so a clamped-empty state cannot remain an infinite source.
 *
 * Internal resistance is:
 *
 *   R(S,T) = R_fresh * [1 + A(1-S)^p/(S+K)]
 *                      * max(F_min, exp(alpha * (T_ref - T_clamped)))
 *
 * This captures the two effects supported by the manufacturer guidance:
 * resistance rises non-linearly with depletion and as ambient temperature
 * falls. `alpha` is calibrated to a documented profile assumption at the
 * lowest rated temperature; it is not presented as a manufacturer-guaranteed
 * coefficient.
 *
 * SoC uses coulomb counting:
 *
 *   S_next = clamp01(S - max(I_discharge, 0) * dt / (3600 * capacity_Ah))
 *
 * All supported cells are primary. Reverse current is counted as rejected
 * charge but never increases SoC. This module intentionally omits recovery,
 * rate-capacity/Peukert effects, self-discharge, ageing, and self-heating. Rated
 * capacity is therefore a reference-drain approximation, not a runtime promise.
 */

export type BatteryChemistry =
  | "alkaline-zn-mno2"
  | "lithium-mno2"
  | "generic-primary";

export type BatteryProfileId =
  | "alkaline-aa-4s"
  | "alkaline-pp3-9v"
  | "lithium-cr2032"
  | "generic-primary";

export interface BatteryOcvPoint {
  /** Usable state of charge, 0..1. */
  soc: number;
  /** Pack open-circuit source voltage at this SoC. */
  voltageV: number;
}

export interface BatteryPhysicsProfile {
  id: BatteryProfileId;
  catalogUid: string | null;
  label: string;
  chemistry: BatteryChemistry;
  rechargeable: false;
  cellCount: number;
  nominalVoltageV: number;
  referenceCapacityAh: number;
  freshInternalResistanceOhm: number;
  referenceTemperatureC: number;
  operatingTemperatureC: { min: number; max: number };
  ocvCurve: readonly BatteryOcvPoint[];
  resistance: {
    /** A in the depletion multiplier equation. */
    depletionScale: number;
    /** p in the depletion multiplier equation. */
    depletionExponent: number;
    /** K in the depletion multiplier equation; keeps R finite at S=0. */
    kneeSoc: number;
    /** alpha in exp(alpha * (T_ref - T)). */
    temperatureCoefficientPerC: number;
    /** Prevents optimistic near-zero resistance at high temperature. */
    minimumTemperatureFactor: number;
  };
  assumptions: readonly string[];
}

export interface ResolveBatteryProfileInput {
  catalogUid?: string | null;
  /** Used only by the generic fallback. */
  fallbackNominalVoltageV?: number;
  /** Used only by the generic fallback. */
  fallbackCapacityAh?: number;
  /** Used only by the generic fallback. Zero is allowed for legacy ideal-source compatibility. */
  fallbackFreshInternalResistanceOhm?: number;
}

export interface BatteryProfileResolution {
  profile: BatteryPhysicsProfile;
  source: "exact-catalog-uid" | "generic-fallback";
  requestedCatalogUid: string | null;
  warning: string | null;
}

export interface BatteryDynamicState {
  soc: number;
  /** Accepted discharge since this state was initialised. */
  dischargedCoulombs: number;
  /** Reverse-current charge deliberately not accepted by a primary cell. */
  rejectedRechargeCoulombs: number;
}

export interface BatteryStateStepInput {
  /** Positive means current delivered out of the positive terminal. */
  dischargeCurrentA: number;
  dtSeconds: number;
}

export interface BatteryStateStepResult {
  state: BatteryDynamicState;
  requestedDischargeCoulombs: number;
  acceptedDischargeCoulombs: number;
  rejectedRechargeCoulombs: number;
  depleted: boolean;
}

export interface BatteryOperatingPointInput {
  soc: number;
  ambientTemperatureC?: number;
  /** Optional explicit part parameter; otherwise the chemistry profile value is used. */
  referenceInternalResistanceOhm?: number;
}

export interface BatteryOperatingPoint {
  soc: number;
  openCircuitVoltageV: number;
  internalResistanceOhm: number;
  remainingCapacityCoulombs: number;
  ambientTemperatureC: number;
  modelTemperatureC: number;
  temperatureWasClamped: boolean;
}

const ALKALINE_CELL_OCV: readonly BatteryOcvPoint[] = [
  // SoC=0 is an explicit energy-conservation collapse. At 2% the cell reaches
  // the 0.8 V cutoff used by the E91 sheet and the 522's 4.8 V / six-cell cutoff.
  { soc: 0, voltageV: 0 },
  { soc: 0.02, voltageV: 0.8 },
  { soc: 0.1, voltageV: 1.02 },
  { soc: 0.3, voltageV: 1.18 },
  { soc: 0.5, voltageV: 1.28 },
  { soc: 0.7, voltageV: 1.38 },
  { soc: 0.9, voltageV: 1.48 },
  { soc: 1, voltageV: 1.6 },
];

function scaleCurve(curve: readonly BatteryOcvPoint[], scale: number): BatteryOcvPoint[] {
  return curve.map((point) => ({ soc: point.soc, voltageV: point.voltageV * scale }));
}

function coefficientForColdMultiplier(
  referenceTemperatureC: number,
  minimumTemperatureC: number,
  multiplierAtMinimum: number,
): number {
  return Math.log(multiplierAtMinimum) / (referenceTemperatureC - minimumTemperatureC);
}

const AA_REFERENCE_TEMPERATURE_C = 21;
const AA_MINIMUM_TEMPERATURE_C = -18;
const CR_REFERENCE_TEMPERATURE_C = 21;
const CR_MINIMUM_TEMPERATURE_C = -30;

const AA_4S: BatteryPhysicsProfile = {
  id: "alkaline-aa-4s",
  catalogUid: "supply-5v",
  label: "4×AA alkaline pack",
  chemistry: "alkaline-zn-mno2",
  rechargeable: false,
  cellCount: 4,
  nominalVoltageV: 6,
  // E91 is roughly 2.5 Ah around a 100 mA continuous reference drain. Four
  // series cells raise voltage; they do not multiply amp-hour capacity.
  referenceCapacityAh: 2.5,
  // E91 publishes 150–300 mΩ fresh per cell. Midpoint × four series cells.
  freshInternalResistanceOhm: 0.9,
  referenceTemperatureC: AA_REFERENCE_TEMPERATURE_C,
  operatingTemperatureC: { min: AA_MINIMUM_TEMPERATURE_C, max: 55 },
  ocvCurve: scaleCurve(ALKALINE_CELL_OCV, 4),
  resistance: {
    depletionScale: 0.7,
    depletionExponent: 2,
    kneeSoc: 0.05,
    // Compact fit assumption: a fresh pack has 3× its 21°C resistance at -18°C.
    temperatureCoefficientPerC: coefficientForColdMultiplier(
      AA_REFERENCE_TEMPERATURE_C,
      AA_MINIMUM_TEMPERATURE_C,
      3,
    ),
    minimumTemperatureFactor: 0.5,
  },
  assumptions: [
    "2.5 Ah reference capacity represents a moderate drain; real alkaline capacity falls at high drain.",
    "Fresh resistance is four times the E91 datasheet midpoint because the cells are in series.",
    "Temperature changes resistance only; capacity and OCV temperature shifts are deferred.",
  ],
};

const PP3_9V: BatteryPhysicsProfile = {
  id: "alkaline-pp3-9v",
  catalogUid: "battery-9v",
  label: "PP3 / 6LR61 9 V alkaline",
  chemistry: "alkaline-zn-mno2",
  rechargeable: false,
  cellCount: 6,
  nominalVoltageV: 9,
  // The 522 sheet spans roughly 500–600 mAh at its low reference drains.
  referenceCapacityAh: 0.55,
  freshInternalResistanceOhm: 1.5,
  referenceTemperatureC: AA_REFERENCE_TEMPERATURE_C,
  operatingTemperatureC: { min: AA_MINIMUM_TEMPERATURE_C, max: 55 },
  ocvCurve: scaleCurve(ALKALINE_CELL_OCV, 6),
  resistance: {
    depletionScale: 0.55,
    depletionExponent: 2,
    kneeSoc: 0.05,
    // Smaller internal cells are more cold-sensitive: 4× at -18°C is the
    // explicit compact-model assumption.
    temperatureCoefficientPerC: coefficientForColdMultiplier(
      AA_REFERENCE_TEMPERATURE_C,
      AA_MINIMUM_TEMPERATURE_C,
      4,
    ),
    minimumTemperatureFactor: 0.5,
  },
  assumptions: [
    "0.55 Ah is a low-drain reference; a PP3 is not a high-current 9 V source.",
    "The six-cell alkaline curve uses the 522 sheet's 4.8 V cutoff.",
    "Fresh resistance uses the catalog's 1.5 Ω nominal and rises toward end of life.",
  ],
};

const CR2032: BatteryPhysicsProfile = {
  id: "lithium-cr2032",
  catalogUid: "battery-coin-cr2032",
  label: "CR2032 lithium coin cell",
  chemistry: "lithium-mno2",
  rechargeable: false,
  cellCount: 1,
  nominalVoltageV: 3,
  // Energizer rates 235 mAh to 2.0 V at 15 kΩ / 21°C.
  referenceCapacityAh: 0.235,
  freshInternalResistanceOhm: 10,
  referenceTemperatureC: CR_REFERENCE_TEMPERATURE_C,
  operatingTemperatureC: { min: CR_MINIMUM_TEMPERATURE_C, max: 60 },
  ocvCurve: [
    { soc: 0, voltageV: 0 },
    { soc: 0.02, voltageV: 2 },
    { soc: 0.05, voltageV: 2.55 },
    { soc: 0.1, voltageV: 2.75 },
    { soc: 0.2, voltageV: 2.88 },
    { soc: 0.5, voltageV: 2.98 },
    { soc: 0.8, voltageV: 3.02 },
    { soc: 0.95, voltageV: 3.08 },
    { soc: 1, voltageV: 3.2 },
  ],
  resistance: {
    depletionScale: 0.75,
    depletionExponent: 2,
    kneeSoc: 0.05,
    // Compact fit assumption: pulse resistance is 5× the 21°C value at -30°C.
    temperatureCoefficientPerC: coefficientForColdMultiplier(
      CR_REFERENCE_TEMPERATURE_C,
      CR_MINIMUM_TEMPERATURE_C,
      5,
    ),
    minimumTemperatureFactor: 0.5,
  },
  assumptions: [
    "235 mAh applies near the 0.19 mA datasheet drain; high pulse loads yield less usable capacity.",
    "10 Ω is the fresh reference; the datasheet pulse-resistance curve rises strongly with discharge.",
    "Reverse current is rejected; the datasheet permits only about 1 µA maximum reverse charge.",
  ],
};

export const BATTERY_PROFILES_BY_CATALOG_UID: Readonly<Record<string, BatteryPhysicsProfile>> = {
  "supply-5v": AA_4S,
  "battery-9v": PP3_9V,
  "battery-coin-cr2032": CR2032,
};

function finitePositive(value: number | undefined, fallback: number): number {
  return value != null && Number.isFinite(value) && value > 0 ? value : fallback;
}

function finiteNonNegative(value: number | undefined, fallback: number): number {
  return value != null && Number.isFinite(value) && value >= 0 ? value : fallback;
}

function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

function clampSoc(value: number): number {
  return Number.isFinite(value) ? clamp(value, 0, 1) : 0;
}

function genericProfile(input: ResolveBatteryProfileInput): BatteryPhysicsProfile {
  const nominalVoltageV = finitePositive(input.fallbackNominalVoltageV, 5);
  const capacityAh = finitePositive(input.fallbackCapacityAh, 1);
  const freshInternalResistanceOhm = finiteNonNegative(
    input.fallbackFreshInternalResistanceOhm,
    0.25,
  );
  const referenceTemperatureC = 25;
  const minimumTemperatureC = -20;

  return {
    id: "generic-primary",
    catalogUid: null,
    label: "Generic primary battery (fallback)",
    chemistry: "generic-primary",
    rechargeable: false,
    cellCount: 1,
    nominalVoltageV,
    referenceCapacityAh: capacityAh,
    freshInternalResistanceOhm,
    referenceTemperatureC,
    operatingTemperatureC: { min: minimumTemperatureC, max: 60 },
    // A linear fallback is intentionally not called alkaline or lithium. It
    // retains legacy charge×voltage behaviour while making that uncertainty explicit.
    ocvCurve: [
      { soc: 0, voltageV: 0 },
      { soc: 1, voltageV: nominalVoltageV },
    ],
    resistance: {
      depletionScale: 0.5,
      depletionExponent: 2,
      kneeSoc: 0.05,
      temperatureCoefficientPerC: coefficientForColdMultiplier(
        referenceTemperatureC,
        minimumTemperatureC,
        2.5,
      ),
      minimumTemperatureFactor: 0.5,
    },
    assumptions: [
      "No chemistry was inferred: voltage falls linearly with usable SoC.",
      `Reference capacity is the caller/default assumption (${capacityAh} Ah).`,
      `Fresh internal resistance is the caller/default assumption (${freshInternalResistanceOhm} Ω).`,
    ],
  };
}

export function resolveBatteryProfile(
  input: ResolveBatteryProfileInput = {},
): BatteryProfileResolution {
  const requestedCatalogUid = input.catalogUid?.trim() || null;
  const exact = requestedCatalogUid
    ? BATTERY_PROFILES_BY_CATALOG_UID[requestedCatalogUid]
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
      ? `No battery chemistry profile exists for catalogUid "${requestedCatalogUid}"; using an explicit generic-primary fallback.`
      : "Battery has no catalogUid; using an explicit generic-primary fallback.",
  };
}

export function batteryOpenCircuitVoltageV(
  profile: BatteryPhysicsProfile,
  socInput: number,
): number {
  const soc = clampSoc(socInput);
  const curve = profile.ocvCurve;
  if (curve.length === 0) return 0;
  if (soc <= curve[0].soc) return curve[0].voltageV;

  for (let i = 1; i < curve.length; i += 1) {
    const right = curve[i];
    if (soc > right.soc) continue;
    const left = curve[i - 1];
    const width = right.soc - left.soc;
    if (width <= 0) return right.voltageV;
    const t = (soc - left.soc) / width;
    return left.voltageV + (right.voltageV - left.voltageV) * t;
  }
  return curve[curve.length - 1].voltageV;
}

export function batteryInternalResistanceOhm(
  profile: BatteryPhysicsProfile,
  socInput: number,
  ambientTemperatureC = profile.referenceTemperatureC,
  referenceInternalResistanceOhm = profile.freshInternalResistanceOhm,
): number {
  const soc = clampSoc(socInput);
  const baseResistance = finiteNonNegative(
    referenceInternalResistanceOhm,
    profile.freshInternalResistanceOhm,
  );
  const { depletionScale, depletionExponent, kneeSoc } = profile.resistance;
  const depletionFactor =
    1 + depletionScale * Math.pow(1 - soc, depletionExponent) / (soc + kneeSoc);

  const temperature = Number.isFinite(ambientTemperatureC)
    ? clamp(
        ambientTemperatureC,
        profile.operatingTemperatureC.min,
        profile.operatingTemperatureC.max,
      )
    : profile.referenceTemperatureC;
  const rawTemperatureFactor = Math.exp(
    profile.resistance.temperatureCoefficientPerC *
      (profile.referenceTemperatureC - temperature),
  );
  const temperatureFactor = Math.max(
    profile.resistance.minimumTemperatureFactor,
    rawTemperatureFactor,
  );

  return baseResistance * depletionFactor * temperatureFactor;
}

export function createBatteryState(initialSoc = 1): BatteryDynamicState {
  return {
    soc: clampSoc(initialSoc),
    dischargedCoulombs: 0,
    rejectedRechargeCoulombs: 0,
  };
}

export function advancePrimaryBatteryState(
  profile: BatteryPhysicsProfile,
  previous: BatteryDynamicState,
  input: BatteryStateStepInput,
): BatteryStateStepResult {
  const soc = clampSoc(previous.soc);
  const dtSeconds = Number.isFinite(input.dtSeconds) ? Math.max(0, input.dtSeconds) : 0;
  const currentA = Number.isFinite(input.dischargeCurrentA) ? input.dischargeCurrentA : 0;
  const requestedDischargeCoulombs = Math.max(0, currentA) * dtSeconds;
  const rejectedRechargeCoulombs = Math.max(0, -currentA) * dtSeconds;
  const capacityCoulombs = profile.referenceCapacityAh * 3600;
  const remainingCoulombs = capacityCoulombs * soc;
  const acceptedDischargeCoulombs = Math.min(
    requestedDischargeCoulombs,
    remainingCoulombs,
  );
  const nextSoc = capacityCoulombs > 0
    ? clampSoc(soc - acceptedDischargeCoulombs / capacityCoulombs)
    : 0;

  const state: BatteryDynamicState = {
    soc: nextSoc,
    dischargedCoulombs:
      finiteNonNegative(previous.dischargedCoulombs, 0) + acceptedDischargeCoulombs,
    rejectedRechargeCoulombs:
      finiteNonNegative(previous.rejectedRechargeCoulombs, 0) + rejectedRechargeCoulombs,
  };

  return {
    state,
    requestedDischargeCoulombs,
    acceptedDischargeCoulombs,
    rejectedRechargeCoulombs,
    depleted: nextSoc <= 0,
  };
}

export function batteryOperatingPoint(
  profile: BatteryPhysicsProfile,
  input: BatteryOperatingPointInput,
): BatteryOperatingPoint {
  const soc = clampSoc(input.soc);
  const ambientTemperatureC = Number.isFinite(input.ambientTemperatureC)
    ? input.ambientTemperatureC!
    : profile.referenceTemperatureC;
  const modelTemperatureC = clamp(
    ambientTemperatureC,
    profile.operatingTemperatureC.min,
    profile.operatingTemperatureC.max,
  );

  return {
    soc,
    openCircuitVoltageV: batteryOpenCircuitVoltageV(profile, soc),
    internalResistanceOhm: batteryInternalResistanceOhm(
      profile,
      soc,
      ambientTemperatureC,
      input.referenceInternalResistanceOhm,
    ),
    remainingCapacityCoulombs: profile.referenceCapacityAh * 3600 * soc,
    ambientTemperatureC,
    modelTemperatureC,
    temperatureWasClamped: modelTemperatureC !== ambientTemperatureC,
  };
}

export function batteryTerminalVoltageV(
  point: Pick<BatteryOperatingPoint, "openCircuitVoltageV" | "internalResistanceOhm">,
  dischargeCurrentA: number,
): number {
  const currentA = Number.isFinite(dischargeCurrentA) ? dischargeCurrentA : 0;
  return point.openCircuitVoltageV - currentA * point.internalResistanceOhm;
}
