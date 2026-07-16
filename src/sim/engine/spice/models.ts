/**
 * SPICE .model card mapping (Wave A7) — the documented translation from
 * SPICE model parameters onto the engine's device params.
 *
 * DESIGN RULES
 * - One documented entry per SUPPORTED parameter, in the tables below. A
 *   .model parameter outside its device's table produces a collected WARNING
 *   (never a fatal error): the caller still gets a runnable circuit, plus an
 *   explicit record of every parameter the engine physics cannot honor.
 *   Contrast with the netlist grammar itself, where unknown CARDS are hard
 *   errors — an ignored card silently changes topology, while an ignored
 *   second-order model parameter only shifts accuracy, and the warning says
 *   exactly by how much trust to discount.
 * - Pure module: no filesystem/DOM access, no engine imports beyond the
 *   shared pure element helpers. The netlist converter is the only caller.
 *
 * PER-DEVICE FIDELITY NOTES
 *
 * Diode (.model D): the engine diode (devices/semiconductors.ts,
 * stampShockleyJunction) reads params.Is DIRECTLY when present — the
 * vf/iRated anchor is only its fallback derivation for catalog parts — so
 * SPICE IS passes through with no inversion step and no rounding detour
 * through shockleyIsFromVf. Exactness caveat, documented rather than hidden:
 * the engine treats an authored Is as the 25 C saturation current and maps
 * it through saturationCurrentAtTemperature(Is, junctionTempC, n), which is
 * the IDENTITY at the engine's default 25 C ambient (t == tNom makes both
 * the power-law and exponential factors 1). SPICE's default temperature is
 * 27 C with TNOM 27, so a netlist that never sets .temp runs the same Is at
 * a 2 C cooler junction: thermal voltage 25.69 mV here vs 25.86 mV in
 * ngspice, worth about 0.7% on a forward drop. The subset accepts that
 * documented delta instead of resampling Is behind the author's back.
 * .temp INTERACTION (the trap in that anchor difference): for any non-25 C
 * .temp the engine resamples Is FROM ITS 25 C ANCHOR while ngspice
 * resamples from TNOM (default 27) — so adding ".temp 27" to match
 * ngspice's default temperature makes agreement WORSE, not better (the
 * engine rescales Is by about 1.36x, ngspice not at all). Cross-engine
 * comparisons must pin ".temp 25" in the deck plus TNOM=25 on the ngspice
 * side, which is exactly what the crossval suite does.
 *
 * BJT (.model NPN/PNP): IS/BF/BR/NF/NR/VAF map one-to-one onto the engine's
 * transport Ebers-Moll (Is/betaF/betaR/nF/nR/earlyVoltage) — the same model
 * class SPICE reduces to when its charge-storage and resistance parameters
 * are absent. The same Is-at-25C temperature note as the diode applies.
 * VAF caveat: the PARAMETER transfers one-to-one but the Early EQUATION
 * does not — the engine scales forward transport by (1 + vCE/VA)
 * (elements.ts bjtCurrents) while SPICE's Gummel-Poon divides by its qb
 * charge factor (approximately 1 - vBC/VAF with VAR absent), a
 * vBE/VAF-order divergence worth about 1% of Ic at VAF=80. The crossval
 * suite pins its BJT deck with VAF absent for exactly this reason.
 *
 * MOSFET (.model NMOS/PMOS): VTO/KP/LAMBDA map onto the engine's level-1
 * Shichman-Hodges (vto/k/lambda) with k = (KP/2)*(W/L) folded from the
 * element line's W/L (SPICE default 100 um each, ratio 1). The engine's vto
 * is a positive magnitude for enhancement devices of BOTH polarities (the
 * kind carries the sign), so PMOS negates SPICE's negative VTO; NMOS passes
 * VTO through signed, which preserves depletion (negative-VTO) NMOS devices.
 *
 * JFET (.model NJF/PJF): the engine parameterizes the channel by idss
 * (drain current at vGS = 0), not BETA, so BETA converts via the Shichman-
 * Hodges identity idss = BETA*VTO^2. The engine then rebuilds beta =
 * idss/vto^2 internally (devices/model-families.ts jfetParams) — an exact
 * round trip apart from one f64 multiply/divide pair, PROVIDED the values
 * stay above jfetParams' floors: it clamps |vto| up to 0.05 V and idss up
 * to 1e-9 A, and below either floor the rebuilt beta silently differs from
 * the authored one, so the mapper warns there. VTO passes through signed;
 * jfetParams coerces it onto the depletion convention (-|vto|) either way.
 * LAMBDA maps directly.
 */

/** Model types in the documented subset, as lowercased card tokens. */
export type SpiceModelType = "d" | "npn" | "pnp" | "nmos" | "pmos" | "njf" | "pjf";

export const SPICE_MODEL_TYPES: ReadonlySet<string> = new Set([
  "d", "npn", "pnp", "nmos", "pmos", "njf", "pjf",
]);

/** One parsed .model card (name/type lowercased by the tokenizer). */
export interface SpiceModelCard {
  name: string;
  type: SpiceModelType;
  /** Lowercased SPICE parameter name -> numeric value. */
  params: Map<string, number>;
  /** 1-based netlist line of the .model card (for diagnostics). */
  line: number;
  /** Original card text (for diagnostics). */
  card: string;
}

/**
 * Supported-parameter tables: SPICE parameter -> engine param key. Anything
 * absent warns. LEVEL is accepted on MOS models as a pseudo-parameter and
 * validated separately (only level 1 exists here).
 */
const DIODE_PARAM_MAP: ReadonlyMap<string, string> = new Map([
  ["is", "Is"],
  ["n", "n"],
]);

const BJT_PARAM_MAP: ReadonlyMap<string, string> = new Map([
  ["is", "Is"],
  ["bf", "betaF"],
  ["br", "betaR"],
  ["nf", "nF"],
  ["nr", "nR"],
  ["vaf", "earlyVoltage"],
]);

const MOS_PARAM_MAP: ReadonlyMap<string, string> = new Map([
  ["vto", "vto"],
  ["kp", "kp"],
  ["lambda", "lambda"],
]);

const JFET_PARAM_MAP: ReadonlyMap<string, string> = new Map([
  ["vto", "vto"],
  ["beta", "beta"],
  ["lambda", "lambda"],
]);

function tableFor(type: SpiceModelType): ReadonlyMap<string, string> {
  switch (type) {
    case "d": return DIODE_PARAM_MAP;
    case "npn":
    case "pnp": return BJT_PARAM_MAP;
    case "nmos":
    case "pmos": return MOS_PARAM_MAP;
    case "njf":
    case "pjf": return JFET_PARAM_MAP;
  }
}

/**
 * Extract the supported parameters of a model card into a plain lowercased
 * SPICE-name record, warning once per unsupported parameter. Shared by every
 * per-device mapper below so the warning wording stays uniform.
 */
function supportedParams(
  model: SpiceModelCard,
  warnings: string[],
): Map<string, number> {
  const table = tableFor(model.type);
  const out = new Map<string, number>();
  for (const [key, value] of model.params) {
    if (key === "level" && (model.type === "nmos" || model.type === "pmos")) {
      // LEVEL selects the SPICE model equations; only level 1 matches the
      // engine's Shichman-Hodges stamp.
      if (value !== 1) {
        warnings.push(
          `line ${model.line}: .model ${model.name}: LEVEL=${value} is not ` +
          "supported (engine implements level 1 Shichman-Hodges); proceeding with level-1 equations",
        );
      }
      continue;
    }
    const engineKey = table.get(key);
    if (engineKey === undefined) {
      warnings.push(
        `line ${model.line}: .model ${model.name} (${model.type.toUpperCase()}): ` +
        `parameter ${key.toUpperCase()}=${value} has no engine mapping; ignored`,
      );
      continue;
    }
    out.set(key, value);
  }
  return out;
}

/**
 * .model D -> engine diode params. IS passes through as params.Is (read
 * directly by the engine diode — see the header fidelity note), N as
 * params.n. Engine defaults when absent mirror SPICE's: IS = 1e-14, N = 1.
 */
export function diodeParamsFromModel(
  model: SpiceModelCard,
  warnings: string[],
): Record<string, number> {
  const p = supportedParams(model, warnings);
  return {
    Is: p.get("is") ?? 1e-14,
    n: p.get("n") ?? 1,
  };
}

/**
 * .model NPN/PNP -> engine BJT params. Defaults are SPICE's (IS = 1e-16,
 * BF = 100, BR = 1, NF = NR = 1, VAF = infinity — expressed as the engine's
 * 0 = "no Early effect" sentinel).
 */
export function bjtParamsFromModel(
  model: SpiceModelCard,
  warnings: string[],
): Record<string, number> {
  const p = supportedParams(model, warnings);
  return {
    Is: p.get("is") ?? 1e-16,
    betaF: p.get("bf") ?? 100,
    betaR: p.get("br") ?? 1,
    nF: p.get("nf") ?? 1,
    nR: p.get("nr") ?? 1,
    earlyVoltage: p.get("vaf") ?? 0,
  };
}

/**
 * .model NMOS/PMOS plus the element line's W/L -> engine MOSFET params.
 *
 *   k = (KP/2) * (W/L)
 *
 * exactly the engine's own K definition (elements.ts stampMOSFET header).
 * SPICE defaults: VTO = 0 (a zero-threshold device — passed through, the
 * engine handles it), KP = 2e-5, W = L = 100 um. PMOS VTO sign is flipped
 * to the engine's positive-magnitude convention (header note); NMOS keeps
 * the sign so depletion devices survive.
 *
 * W/L arrive pre-validated (> 0) from the element-card parser: this module
 * stays throw-free so its only outputs are mapped params and warnings.
 */
export function mosfetParamsFromModel(
  model: SpiceModelCard,
  widthMeters: number | undefined,
  lengthMeters: number | undefined,
  warnings: string[],
): Record<string, number> {
  const p = supportedParams(model, warnings);
  const w = widthMeters ?? 100e-6;
  const l = lengthMeters ?? 100e-6;
  const vtoSpice = p.get("vto") ?? 0;
  const kp = p.get("kp") ?? 2e-5;
  return {
    vto: model.type === "pmos" ? -vtoSpice : vtoSpice,
    k: (kp / 2) * (w / l),
    lambda: p.get("lambda") ?? 0,
  };
}

/**
 * .model NJF/PJF -> engine JFET params via idss = BETA*VTO^2 (header note).
 * SPICE defaults: VTO = -2, BETA = 1e-4, LAMBDA = 0.
 */
export function jfetParamsFromModel(
  model: SpiceModelCard,
  warnings: string[],
): Record<string, number> {
  const p = supportedParams(model, warnings);
  const vto = p.get("vto") ?? -2;
  const beta = p.get("beta") ?? 1e-4;
  const idss = beta * vto * vto;
  // jfetParams (devices/model-families.ts) silently clamps |vto| up to
  // 0.05 V and idss up to 1e-9 A before rebuilding beta = idss/vto^2, so
  // below either floor the documented exact round trip no longer holds:
  // the device runs, but with a different beta than authored. Disclose it
  // (same warning philosophy as unmapped parameters — accuracy shifts,
  // topology does not).
  if (Math.abs(vto) < 0.05) {
    warnings.push(
      `line ${model.line}: .model ${model.name}: |VTO|=${Math.abs(vto)} V is below the engine's `
      + "0.05 V floor; the clamped device rebuilds a different BETA than authored",
    );
  }
  if (idss < 1e-9) {
    warnings.push(
      `line ${model.line}: .model ${model.name}: BETA*VTO^2=${idss} A is below the engine's `
      + "1e-9 A idss floor; the clamped device rebuilds a different BETA than authored",
    );
  }
  return {
    vto,
    idss,
    lambda: p.get("lambda") ?? 0,
  };
}
