/**
 * Deterministic device-model registration (Wave A4).
 *
 * This is the ONLY module that calls registerDeviceModel. Cohort modules
 * export plain model objects with no side effects, and sim-engine.ts calls
 * registerBuiltinDeviceModels() exactly once at module scope, so the
 * registered set and its order are a function of this explicit list — never
 * of incidental import-graph ordering. Keep registrations grouped by cohort,
 * in the order the engine's _stampAll switch originally declared the kinds,
 * so a reviewer can diff this list against the shrinking switch.
 */

import { registerDeviceModel } from "../device-registry.js";
import {
  capacitorModel,
  dipSwitchModel,
  ferriteBeadModel,
  fuseModel,
  inductorModel,
  ldrModel,
  potentiometerModel,
  ptcFuseModel,
  pushDpdtModel,
  resistorArrayModel,
  resistorModel,
  spdtSwitchModel,
  switchModel,
  thermistorModel,
} from "./passives.js";
import {
  batteryPackModel,
  benchPsuModel,
  clockModel,
  currentSourceModel,
  pulseSourceModel,
  signalGenModel,
  voltageSourceModel,
} from "./sources.js";
import {
  bicolorLedModel,
  bjtModel,
  diodeModel,
  ledModel,
  mosfetModel,
  rgbLedModel,
  tl431Model,
  tvsDiodeModel,
  zenerDiodeModel,
} from "./semiconductors.js";
import {
  dcdcConverterModel,
  dualOpAmpModel,
  linearRegModel,
  lm317Model,
  lm386Model,
  lm393Model,
} from "./amps-regulators.js";
import {
  cd4017Model,
  cd4060Model,
  cd4511Model,
  combinationalIcModel,
  eepromModel,
  hc165Model,
  hc595Model,
  hc74Model,
  hd44780Model,
  ls161Model,
  ls173Model,
  ls189Model,
  max7219Model,
  ne555Model,
  seg7Model,
} from "./digital-ics.js";
import {
  buzzerModel,
  dcMotorModel,
  hcsr04Model,
  l293dModel,
  relayModel,
  servoModel,
  speakerModel,
  stepperModel,
  tb6612Model,
  ulnModel,
} from "./electromech-audio.js";
import {
  analogSwitchModel,
  coupledInductorModel,
  crystalModel,
  jfetModel,
  optoNpnModel,
  scrModel,
  triacModel,
} from "./model-families.js";

// Registration is an explicit call, not an import side effect: the package
// ships sideEffects:false so bundlers may prune bare `import "..."` edges,
// which would silently drop every builtin registration. sim-engine.ts calls
// this at module scope (a retained top-level call survives tree shaking),
// preserving the old evaluate-before-engine-body ordering guarantee.
// Idempotent so multiple entry points (or tests) can call it safely without
// tripping registerDeviceModel's deliberate duplicate-kind throw.
let builtinModelsRegistered = false;

export function registerBuiltinDeviceModels(): void {
  if (builtinModelsRegistered) return;
  builtinModelsRegistered = true;

  // ── PASSIVES cohort ─────────────────────────────────────────────────────────
  registerDeviceModel(resistorModel);
  registerDeviceModel(capacitorModel);
  registerDeviceModel(inductorModel);
  registerDeviceModel(switchModel);
  registerDeviceModel(spdtSwitchModel);
  registerDeviceModel(pushDpdtModel);
  registerDeviceModel(dipSwitchModel);
  registerDeviceModel(potentiometerModel);
  registerDeviceModel(fuseModel);
  registerDeviceModel(ptcFuseModel);
  registerDeviceModel(ferriteBeadModel);
  registerDeviceModel(resistorArrayModel);
  registerDeviceModel(ldrModel);
  registerDeviceModel(thermistorModel);

  // ── SOURCES cohort ──────────────────────────────────────────────────────────
  registerDeviceModel(voltageSourceModel);
  registerDeviceModel(batteryPackModel);
  registerDeviceModel(benchPsuModel);
  registerDeviceModel(clockModel);
  registerDeviceModel(pulseSourceModel);
  registerDeviceModel(signalGenModel);

  // ── SEMICONDUCTORS cohort ───────────────────────────────────────────────────
  // diodeModel/ledModel split one shared engine case body (schottky/led/diode)
  // so the updateFailures hook exists only on "led" — hook presence drives
  // failure-bucket membership, and plain diodes were never in that pass.
  registerDeviceModel(zenerDiodeModel);
  registerDeviceModel(tvsDiodeModel);
  registerDeviceModel(diodeModel);
  registerDeviceModel(ledModel);
  registerDeviceModel(bicolorLedModel);
  registerDeviceModel(rgbLedModel);
  registerDeviceModel(bjtModel);
  registerDeviceModel(mosfetModel);
  registerDeviceModel(tl431Model);

  // ── AMPS-REGULATORS cohort ──────────────────────────────────────────────────
  // dcdc_converter leads because its _stampAll case preceded the W4.1 op-amp
  // block in the engine switch this list mirrors.
  registerDeviceModel(dcdcConverterModel);
  registerDeviceModel(dualOpAmpModel);
  registerDeviceModel(lm386Model);
  registerDeviceModel(lm393Model);
  registerDeviceModel(linearRegModel);
  registerDeviceModel(lm317Model);

  // ── DIGITAL-ICS cohort ──────────────────────────────────────────────────────
  registerDeviceModel(ne555Model);
  registerDeviceModel(combinationalIcModel);
  registerDeviceModel(ls161Model);
  registerDeviceModel(ls173Model);
  registerDeviceModel(ls189Model);
  registerDeviceModel(hc595Model);
  registerDeviceModel(hc165Model);
  registerDeviceModel(hc74Model);
  registerDeviceModel(cd4017Model);
  registerDeviceModel(cd4511Model);
  registerDeviceModel(cd4060Model);
  registerDeviceModel(eepromModel);
  registerDeviceModel(seg7Model);
  registerDeviceModel(hd44780Model);
  registerDeviceModel(max7219Model);

  // ── ELECTROMECH-AUDIO cohort ────────────────────────────────────────────────
  // ulnModel covers uln2003 and uln2803 (their _stampAll cases shared a body).
  registerDeviceModel(relayModel);
  registerDeviceModel(dcMotorModel);
  registerDeviceModel(servoModel);
  registerDeviceModel(stepperModel);
  registerDeviceModel(hcsr04Model);
  registerDeviceModel(ulnModel);
  registerDeviceModel(buzzerModel);
  registerDeviceModel(speakerModel);
  registerDeviceModel(l293dModel);
  registerDeviceModel(tb6612Model);

  // ── MODEL-FAMILIES cohort (Wave A6) ─────────────────────────────────────────
  // Appended at the end on purpose: these kinds never existed in the engine
  // switch, so end-of-list registration keeps every pre-A6 registration order
  // (and any order-sensitive audit diffing this list) untouched.
  registerDeviceModel(coupledInductorModel);
  registerDeviceModel(jfetModel);
  registerDeviceModel(scrModel);
  registerDeviceModel(triacModel);
  registerDeviceModel(optoNpnModel);
  registerDeviceModel(crystalModel);
  registerDeviceModel(analogSwitchModel);

  // ── Wave A7 (SPICE interop) ─────────────────────────────────────────────────
  // current_source lives in the SOURCES module but registers here at the end,
  // for the same order-preservation reason as the A6 block above.
  registerDeviceModel(currentSourceModel);
}
