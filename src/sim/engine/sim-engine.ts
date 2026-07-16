/**
 * SimEngine — continuous Modified Nodal Analysis simulator.
 *
 * Accepts a circuit, builds a net list, and advances state one dt at a time.
 * State (capacitor voltages, inductor currents) persists across steps so
 * the simulation behaves like physical hardware: connect a cap and watch
 * it charge; disconnect power and the charge stays.
 *
 * Topology changes (load()) try to preserve element state by:
 *   1. Component-ID match (primary): same component ID → same state.
 *   2. Net-pair fingerprint match (fallback): if a component's two pins
 *      connect to the same pair of nets as an old component of the same
 *      kind (identified by the full sorted pin-set of those nets), carry
 *      the old state. Handles re-wires that preserve connectivity but
 *      generate new component IDs.
 *   3. Zero-state initialization: truly new capacitors and inductors start
 *      unenergized. Oscillator start-up must come from the oscillator model's
 *      powered state transition, never hidden energy in unrelated passives.
 *
 * R2 — non-linear elements (diode, LED): each step runs Newton-Raphson
 * to convergence. Linear stamps (R, V, C, L, switch) are re-applied
 * each iteration (cheap) alongside the non-linear companion stamps
 * linearised about the last-iter voltage guess.
 *
 * R3 — adaptive step size and state snapshots: saveState()/restoreState()
 * let the worker roll back a rejected step; coldLoad() resets all element
 * state for a hard restart.
 */

import { buildNets, type Net } from "./graph.js";
import { createLinearSystem, type LinearSystem } from "./linear-system.js";
import {
  parallelLossCurrent,
  shockleyDiodeCurrent,
  shockleyIsFromVf,
  stampDiodeShockley,
  stampResistor,
  stampVSource,
} from "./elements.js";
import { solveNonlinear } from "./newton.js";
import { clockVoltage, evalCD4511, evalCombinationalIC, evalSeg7, IC_OPEN_HIGH_PINS, icPinMap, pulseVoltage } from "./digital.js";
import { type PinEvent } from "./arduino.js";
import { type MicrocontrollerCore, mcuFactory } from "./mcu.js";
import { isArduinoBoardKind, isMcuBoardKind } from "../../circuit/arduino.js";
import { resolveCatalogPart } from "../../circuit/catalog-resolver.js";
import { getPartLibraryVersion } from "../../parts/part-library.js";
import {
  batteryOperatingPoint,
  createBatteryState,
  resolveBatteryProfile,
  type BatteryDynamicState,
  type BatteryPhysicsProfile,
} from "../battery-physics.js";
import {
  createThermalState,
  resolveThermalProfile,
  stepThermalDevice,
  type ThermalDeviceProfile,
  type ThermalDeviceState,
} from "../thermal-physics.js";
import type { DisplayInfo, Hcsr04EchoEvent } from "../messages.js";
import type { ComponentKind, ElectricalSpecs, PartDefinition } from "../../circuit/types.js";
import { getDeviceModel, type AcDeviceContext, type DeviceContext, type DeviceModel } from "./device-registry.js";
// Deterministic device-model registration (Wave A4). This call is the single
// point where registered kinds enter the engine: devices/index registers
// every model in a fixed explicit order before this module's body evaluates,
// so dispatch never depends on incidental import ordering. It is an explicit
// top-level call rather than a bare side-effect import because the package
// declares sideEffects:false, which licenses bundlers to prune import-only
// edges — pruning here would silently unregister every builtin device.
import { registerBuiltinDeviceModels } from "./devices/index.js";
registerBuiltinDeviceModels();

export type { Net };

function nowMs(): number {
  return typeof performance !== "undefined" ? performance.now() : Date.now();
}

// Minimum conductance stamped onto every node to prevent singular matrices
// when a node has no explicit DC path to ground (same trick as SPICE rshunt).
// Exported for the small-signal AC driver (Wave A5), which seeds its complex
// system with the identical node shunt so both systems are regular under the
// same topologies.
export const NODE_RSHUNT_G = 1e-12;

// Typical internal GPIO pulls. ATmega328P is specified at 20–50 kΩ; RP2040
// at 50–80 kΩ. Mid-band values give realistic button-divider behaviour while
// preserving the intentionally weak nature of the pulls.
const AVR_PULLUP_R = 35_000;
const RP2040_PULL_R = 65_000;

// Shockley defaults. `n` is the emission coefficient: 1 for ideal Si
// junctions, ~2 for LEDs (non-radiative recombination dominates at low
// current). Junction temperature is resolved from the circuit environment.
const K_B_OVER_Q = 8.617_333_262_145e-5; // Boltzmann/electron charge, V/K
const NOMINAL_TEMP_C = 25;
const DEFAULT_VF_TEMPCO = -0.002; // V/°C, representative silicon/LED value
// Exported for migrated device modules (devices/*.ts); also still consumed
// by the engine's own clamp-diode, seg7, and telemetry paths.
export const N_DIODE = 1.0;
export const N_LED = 2.0;
export const IRATED_LED = 0.02; // 20 mA rated forward current — used to derive Is from Vf

// Exported for migrated device modules (devices/*.ts); values are part of
// the damage-model contract the bitwise oracle suite pins.
export const RESISTOR_DAMAGE_THRESHOLD = 1;
export const FUSE_I2T_THRESHOLD = 0.3;
export const LED_DAMAGE_THRESHOLD = 24;

// Consecutive committed steps an output must stay outside its guaranteed logic
// band before a sag warning is published. A finite-strength output commanded
// LOW sits at the HIGH rail for exactly the one step where the flip-flop
// commits LOW while the deferred output stamp is still driving HIGH (e.g. a 555
// one-shot's trailing edge). That single-step switching transient is normal and
// must not raise a warning; a genuine sustained sag persists past it. Two steps
// filters the artifact while surfacing real sags within an imperceptible delay.
const OUTPUT_SAG_DEBOUNCE_STEPS = 2;

/** A missing, invalid, or negative series loss keeps the historical ideal model. */
export function seriesLossResistance(value: unknown): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed > 0 ? parsed : 0;
}

/** A missing, invalid, or non-positive parallel loss disables that branch. */
export function parallelLossResistance(value: unknown): number {
  const parsed = Number(value);
  return Number.isFinite(parsed) && parsed > 0
    ? parsed
    : Number.POSITIVE_INFINITY;
}

// SPICE pnjlim critical voltage: V_crit = VtN · ln(VtN / (Is·√2)). The
// guard on Is avoids log(±∞) for pathologically small saturation currents.
// Exported for the SEMICONDUCTORS device cohort's uncached-BJT fallback;
// the per-load cache build below remains the primary consumer.
export function vCritFor(Is: number, VtN: number): number {
  return VtN * Math.log(VtN / (Math.max(1e-30, Is) * Math.SQRT2));
}

/** Physical junction thermal voltage kT/q. */
export function thermalVoltage(tempC: number): number {
  const kelvin = Math.max(1, Number(tempC) + 273.15);
  return K_B_OVER_Q * kelvin;
}

/** Empirical rated-current forward drop around the 25 °C catalog value. */
export function junctionForwardVoltage(
  vfAt25C: number,
  tempC: number,
  tempco = DEFAULT_VF_TEMPCO,
): number {
  return Math.max(0.05, vfAt25C + tempco * (tempC - NOMINAL_TEMP_C));
}

/**
 * SPICE-style silicon saturation-current temperature scaling. This is needed
 * in addition to Vt(T): holding Is constant makes a BJT's Vbe rise with heat,
 * the opposite of real junction behaviour.
 */
export function saturationCurrentAtTemperature(
  isAt25C: number,
  tempC: number,
  emission = 1,
): number {
  const tNom = NOMINAL_TEMP_C + 273.15;
  const t = Math.max(1, tempC + 273.15);
  const n = Math.max(0.1, emission);
  const siliconBandgapEv = 1.11;
  const xti = 3;
  const exponent = Math.max(
    -80,
    Math.min(80, (siliconBandgapEv / (n * K_B_OVER_Q)) * (1 / tNom - 1 / t)),
  );
  return Math.max(
    1e-30,
    Math.min(1, isAt25C * Math.pow(t / tNom, xti / n) * Math.exp(exponent)),
  );
}

export interface SimCircuit {
  id?: string;
  components: Array<{
    id: string;
    kind: string;
    /** Exact catalog/model variant; optional only for legacy/imported circuits. */
    catalogUid?: string;
    position?: { x: number; y: number };
    rotation?: number;
    pins: Array<{ id: string }>;
    params: Record<string, number | string>;
  }>;
  wires: Array<{
    from_component: string;
    from_pin: string;
    to_component: string;
    to_pin: string;
  }>;
  environment?: { temperatureC?: number; lux?: number };
}

type SimComponent = SimCircuit["components"][number];

// Exported as the DeviceStateMaps.ne555s value type; the map itself (and the
// crossing-split machinery that snapshots it around each step) stays here.
export interface NE555EngineState { outHigh: boolean; }

/**
 * W6.1 — relay persistent state, carried across load() by component ID.
 *
 * iCoil: committed coil current from the previous timestep (A). The RL companion
 *   uses i_prev to inject the history current source that carries inductive memory.
 *   On the first step i_prev=0 (no prior current) — acceptable, the coil charges
 *   up in a few time constants.
 *
 * energized: committed contact state from the previous timestep. Stamped at the
 *   start of each Newton inner loop (committed-state pattern — never re-decided
 *   from xGuess to prevent chattering). Updated post-solve using hysteresis on
 *   the coil voltage |v_ab|.
 */
export interface RelayEngineState {
  iCoil: number;
  energized: boolean;
}

/**
 * W7.1 — DC motor persistent state, carried across load() by component ID.
 *
 * iWinding: committed winding current from the previous timestep (A).
 *   The RL+back-EMF companion uses i_prev to inject the history current source
 *   that carries inductive memory. Zero on first step (motor at rest).
 *
 * omega: committed rotor angular velocity from the previous timestep (rad/s).
 *   Used only as a back-EMF driver (Vbemf = Ke * omega) and as a readout.
 *   Decided/updated POST-SOLVE (committed-state pattern) and held constant
 *   through the Newton inner loop — never re-read from within-iteration xGuess
 *   to prevent back-EMF chattering during convergence.
 *
 * NOTE: omega is a readout and back-EMF driver ONLY — no rotor-position physics,
 * no torque coupling to external mechanical loads, no inertia-to-circuit feedback.
 * The motor is an electrical load with a speed-dependent back-EMF.
 */
export interface MotorEngineState {
  iWinding: number;  // committed winding current (A), i_prev for companion
  omega: number;     // committed rotor speed (rad/s), drives back-EMF next step
}

/**
 * W7.2 — Hobby servo persistent state, carried across load() by component ID.
 *
 * PWM decode: a directly connected Arduino/Pico output uses its ordered,
 * cycle-timestamped PinEvents so pulse width is independent of the outer
 * electrical step. Other drivers retain post-solve threshold sampling at
 * HIGH ≥ 2.0 V relative to the gnd pin:
 *   - Rising edge (0→1): record riseT = edge time.
 *   - Falling edge (1→0): compute pulseMs = (edge time − riseT) × 1000, then
 *       targetAngle = minAngle +
 *         (pulseMs − minPulseMs) / (maxPulseMs − minPulseMs)
 *         × (maxAngle − minAngle).
 *     The target is clamped to [minAngle, maxAngle] via the pulseMs clamp.
 *
 * The shaft then travels toward the last decoded target at a finite no-load
 * speed scaled by the solved supply voltage. The target holds between pulses;
 * loss of signal therefore does not teleport or reset the shaft.
 *
 * lastSig: committed signal level from the previous step (0 or 1).
 * riseT:   simTime of the last observed rising edge (s).
 * pulseMs: last measured HIGH-time in ms (updated on each falling edge).
 * targetAngle: last commanded angle (deg). Holds between complete pulses.
 * angle:   actual rate-limited shaft angle (deg).
 * velocity: last accepted shaft velocity (deg/s).
 * powered/supplyVoltage: committed supply state used by telemetry and load.
 */
export interface ServoEngineState {
  lastSig: number;  // 0 or 1: committed signal level from previous step
  riseT: number;    // simTime (s) of last rising edge; NaN until first rising edge
  pulseMs: number;  // last measured HIGH pulse duration (ms); NaN until first pulse
  targetAngle: number;
  angle: number;
  velocity: number;
  moving: boolean;
  powered: boolean;
  supplyVoltage: number;
  /** Supply resistance that was physically stamped for the accepted step. */
  loadResistance: number;
}

// Exported for devices/electromech-audio.ts (the servo commit handler);
// stays here because load()'s state carry-forward seeds new servos too.
export function defaultServoState(): ServoEngineState {
  return {
    lastSig: 0,
    riseT: Number.NaN,
    pulseMs: Number.NaN,
    targetAngle: 90,
    angle: 90,
    velocity: 0,
    moving: false,
    powered: false,
    supplyVoltage: 0,
    loadResistance: Number.NaN,
  };
}

/**
 * W7.2 — Bipolar stepper motor persistent state, carried across load() by component ID.
 *
 * TWO RL Norton companions (one per coil) use iPrev values to carry inductive memory:
 *   iA: committed current in coil A (a1→a2), A. Used as i_prev in the Norton companion.
 *   iB: committed current in coil B (b1→b2), A. Used as i_prev in the Norton companion.
 *
 * Step detection (committed post-solve):
 *   signA = sign(iA) with ±1 mA deadband → +1 / −1 / 0.
 *   signB = sign(iB) with ±1 mA deadband → +1 / −1 / 0.
 *   Phase lookup (signA, signB) → 0..3 per the full-step Gray-code table:
 *     phase 0: (+1, +1)
 *     phase 1: (−1, +1)
 *     phase 2: (−1, −1)
 *     phase 3: (+1, −1)
 *   Sequential advance +1 (mod 4) → position++; −1 (mod 4) → position--.
 *   No change or jump > 1 → hold (non-sequential drive = half-step or fault; not counted).
 *
 * phase: current (committed) phase index (0-3), or -1 when coils are unenergised / deadband.
 * position: signed step count. angle = position × 360 / stepsPerRev (mod 360).
 */
export interface StepperEngineState {
  iA: number;            // committed coil A current (a1→a2), A; i_prev for companion
  iB: number;            // committed coil B current (b1→b2), A; i_prev for companion
  phase: number;         // committed phase index 0-3; -1 = unenergised / deadband
  lastKnownPhase: number; // last phase that was not -1; used for step counting across deadband
  position: number;      // signed step count (incremented/decremented by the phase detector)
}

/**
 * S18b — HC-SR04 ultrasonic distance sensor persistent state, carried across
 * load() by component ID (mirrors ServoEngineState exactly — see its comment
 * above for the general pattern this follows).
 *
 * TRIG detection is committed-state edge detection, same philosophy as the
 * servo PWM decoder: `trigLevel` is the last-committed TRIG level. Edges are
 * only consulted while `phase === "idle"` — a rising edge (0→1) arms the
 * sensor; the matching falling edge (1→0) schedules the echo. Re-triggering
 * while `phase` is "pending"/"active" is ignored (real module behaviour):
 * `trigLevel` still tracks the net so a genuine post-echo edge is seen
 * cleanly, but phase transitions are gated on `phase === "idle"`.
 *
 * TRIG edge timestamps come from one of two paths (see the hcsr04
 * commitState handler in devices/electromech-audio.ts):
 *   (a) an MCU pin drives the TRIG net directly — cycle-timestamped PinEvents
 *       give the exact edge time regardless of the outer step h (a 10 µs
 *       digitalWrite pulse would otherwise vanish between once-per-step
 *       samples).
 *   (b) otherwise — committed net-voltage threshold sampling (2.0 V), timed
 *       at `this.simTime` (the step-start value, servo precedent): acceptable
 *       because non-MCU TRIG sources hold the level for at least one full step.
 *
 * Schedule (armed on TRIG rise, computed on TRIG fall):
 *   echoRiseAt = tFall + 250 µs
 *   echoFallAt = echoRiseAt + (58.0 µs/cm × distanceCm), or +38 ms if noEcho
 *     (out-of-range timeout).
 * `nextScheduledEventTime()` returns the earliest of these across all
 * hcsr04s so sim.worker.ts can clamp the step size to land exactly on them
 * (event-aligned stepping — see that method's comment).
 *
 * vccV / vccOk: VCC-GND committed from the LAST step (never the live Newton
 * guess — the ECHO push-pull Norton stamp and the whole TRIG/echo state
 * machine read this so the drive decision cannot flicker mid-iteration).
 * Below 3.0 V the part is inert: the stamp skips the ECHO drive entirely
 * (real dropout behaviour — a dead chip cannot actively drive its output)
 * and the commit handler force-resets phase/schedule to idle every step.
 * echoOut: committed ECHO drive level (0/1) — a real HC-SR04 drives ECHO
 * push-pull continuously while powered (0 when idle, not hi-Z).
 *
 * pendingEchoEvent: a queued micro:bit ECHO-bridge event, set the instant an
 * echo bridged to a micro:bit edge pin is scheduled (see messages.ts
 * Hcsr04EchoEvent) and drained by takeHcsr04EchoEvents(). Deliberately part
 * of THIS map entry (not a free-standing queue): it must live inside the
 * saveState()/restoreState() snapshot so a rolled-back Newton step (failed
 * convergence) correctly un-queues a spurious event too.
 */
export interface Hcsr04EngineState {
  trigLevel: 0 | 1;
  phase: "idle" | "armed" | "pending" | "active";
  echoRiseAt: number; // sim-time (s) of scheduled ECHO rise; NaN unless phase === "pending"
  echoFallAt: number; // sim-time (s) of scheduled ECHO fall; NaN unless phase is "pending"/"active"
  echoOut: 0 | 1;
  vccV: number; // committed VCC-GND (V) from the previous step
  pendingEchoEvent: Hcsr04EchoEvent | null;
}

// Exported for devices/electromech-audio.ts (the hcsr04 commit handler);
// stays here because load()'s state carry-forward seeds new sensors too.
export function defaultHcsr04State(): Hcsr04EngineState {
  return {
    trigLevel: 0, phase: "idle", echoRiseAt: NaN, echoFallAt: NaN, echoOut: 0, vccV: 0,
    pendingEchoEvent: null,
  };
}

/**
 * W8.2 — HD44780 16x2 character LCD persistent state, carried across load() by component ID.
 *
 * DDRAM holds the character codes for both lines (length 0x68 = 104 bytes).
 * Address map: 0x00-0x27 = line 1 (40 chars, only first 16 visible), 0x40-0x67 = line 2.
 *
 * 4-bit vs 8-bit mode is detected via the function-set command DL bit (DL=0 → 4-bit).
 * In 4-bit mode a full byte takes two E-strobe pulses (high nibble first via D4-D7).
 *
 * RW pin is sampled but read-back is not supported — tie RW to GND in all real circuits.
 * CGRAM custom character data (codes 0x00-0x07) is not rendered in v1; writes go to the
 * address counter but are discarded. Cursor/display-shift command (0x10-0x1F) is accepted
 * but not rendered in v1.
 */
export interface Hd44780State {
  ddram: number[];       // length 0x68 (104); index = DDRAM address. 0x00-0x27 = line 1, 0x40-0x67 = line 2.
  addr: number;          // address counter (DDRAM address, wraps within 0x00-0x27 / 0x40-0x67)
  fourBit: boolean;      // false = 8-bit mode (power-on default), true = 4-bit mode
  nibblePhase: 0 | 1;    // 4-bit only: 0 = awaiting high nibble, 1 = awaiting low nibble
  hiNib: number;         // captured high nibble (D7-D4) awaiting the low nibble in 4-bit mode
  displayOn: boolean;    // display on/off control D bit from last 0x08-0x0F command
  increment: boolean;    // entry mode I/D bit: true = address increments after each write (default)
  cgram: boolean;        // true when address counter points at CGRAM (data writes ignored in v1)
  twoLine: boolean;      // function-set N bit: true = 2-line mode
  lastE: 0 | 1;          // committed E level at end of previous step (edge detection seed)
  lastRS: 0 | 1;         // committed RS level at end of previous step
  lastData: number[];    // length 8: committed D0-D7 levels at end of previous step
}

// Exported for devices/digital-ics.ts (the hd44780 decode handler); stays
// here because load()'s lcds carry-forward deep-copies through it too.
export function defaultHd44780State(): Hd44780State {
  return {
    // DDRAM initialised to 0x20 (space) per HD44780 power-on spec.
    ddram: new Array(0x68).fill(0x20),
    addr: 0,
    fourBit: false,
    nibblePhase: 0,
    hiNib: 0,
    displayOn: false,
    increment: true,
    cgram: false,
    twoLine: false,
    lastE: 0,
    lastRS: 0,
    lastData: new Array(8).fill(0),
  };
}

// The delay prefix is exported for the small-signal AC driver (Wave A5): a
// committed `delay:<pinId>` icState slot is the canonical record of a driven
// digital output stage, so the driver's generic Thevenin-to-rail AC default
// keys off it instead of duplicating per-kind output tables.
export const DIGITAL_DELAY_PREFIX = "delay:";
const DIGITAL_PENDING_PREFIX = "pending:";
const DIGITAL_DUE_PREFIX = "due:";

// Exported as the DeviceStateMaps.eeproms value type; the map, its load()
// initialisation (contents/SDP seeding), and snapshot/rollback stay here.
export interface EepromState {
  bytes: Uint8Array;
  contentsHash: string;
  lastWeHigh: boolean;
  lastCeHigh: boolean;
  writeCompletesAt: number | null;
  pendingAddr: number | null;
  pendingByte: number | null;
  pollReads: number;
  sdpEnabled: boolean;
  sdpUnlocked: boolean;
  sdpStep: 0 | 1 | 2 | 3;
  dirty: boolean;
}

/**
 * PTC fuse session state.
 *
 * The PTC uses a stress accumulator that is NOT part of the latched failure
 * framework: it does not latch permanently. Instead:
 *   tripped:      whether the device is currently in high-R state.
 *   tripStress:   accumulated excess (I² − I_hold²)·h above hold current.
 *   recoveryTime: accumulated sim-seconds at low current while tripped.
 *
 * State persists across load() by component ID, mirroring ne555s.
 * resetFailures() clears trip state so the toolbar "Clear failures" button
 * resets everything the user expects to reset — both the latched failure
 * framework AND the PTC.
 */
export interface PtcState {
  tripped: boolean;
  tripStress: number;
  recoveryTime: number;
}

export interface BatteryRuntimeState extends BatteryDynamicState {
  /** Authored initial SoC used to distinguish a user reset from runtime drain. */
  authoredCharge: number;
  /** Changing exact battery identity resets incompatible chemistry state. */
  catalogUid: string | null;
}

export interface BatteryRuntimeReadout {
  profileId: string;
  label: string;
  soc: number;
  openCircuitVoltageV: number;
  internalResistanceOhm: number;
  remainingCapacityCoulombs: number;
  dischargedCoulombs: number;
  rejectedRechargeCoulombs: number;
  modelTemperatureC: number;
  temperatureWasClamped: boolean;
}

export interface ThermalRuntimeState extends ThermalDeviceState {
  catalogUid: string;
  profileId: string;
  dissipatedPowerW: number;
  targetTemperatureC: number;
  allowedPowerW: number;
  withinContinuousLimits: boolean;
  warnings: string[];
}

export interface ThermalRuntimeReadout extends ThermalRuntimeState {
  label: string;
}

export interface MosfetGateState {
  /** Previous accepted physical gate-to-source voltage. */
  vgs: number;
  /** Previous accepted physical gate-to-drain voltage. */
  vgd: number;
}

interface ElementState {
  caps: Map<string, number>;  // compId → internal ideal-C voltage (pin0 - pin1)
  capCurrents: Map<string, number>; // compId → total terminal current (pin0 → pin1)
  // Trapezoidal history (Wave A2): last ACCEPTED series-branch current per cap
  // and last ACCEPTED winding terminal voltage per inductor. Maintained,
  // snapshotted, and restored only while integrationMethod is "trap" — in BE
  // mode the maps hold inert load-time values, and every path into trap mode
  // (load, setIntegrationMethod, resetFailures) arms the one-step BE anchor
  // that rewrites them before any trap stamp reads them.
  capsI: Map<string, number>; // compId → series-branch current (pin0 → pin1)
  indsV: Map<string, number>; // compId → winding terminal voltage (pin0 - pin1)
  inds: Map<string, number>;  // compId → winding current (pin0 → pin1)
  mosfetGates: Map<string, MosfetGateState>; // compId → Cgs/Cgd history voltages
  batteries: Map<string, BatteryRuntimeState>; // compId → runtime primary-cell SoC
  ne555s: Map<string, NE555EngineState>;
  relays: Map<string, RelayEngineState>; // compId → relay coil current + energized flag
  motors: Map<string, MotorEngineState>; // compId → dc_motor winding current + rotor speed
  servos: Map<string, ServoEngineState>;   // W7.2 compId → servo PWM decode state + angle
  steppers: Map<string, StepperEngineState>; // W7.2 compId → stepper coil currents + position
  lcds: Map<string, Hd44780State>;          // W8.2 compId → HD44780 DDRAM + protocol state
  hcsr04: Map<string, Hcsr04EngineState>;   // S18b compId → HC-SR04 TRIG/ECHO state machine
  failureStress: Map<string, number>; // failure key → accumulated normalized damage/energy
  failures: Map<string, SimFailure>; // failure key → latched runtime failure
  icState: Map<string, Record<string, number>>; // compId → named state slots for IC timing/sequential storage
  arduinos: Map<string, MicrocontrollerCore>; // compId → running MCU core (any board)
  eeproms: Map<string, EepromState>;
  ptcs: Map<string, PtcState>; // compId → PTC trip state
  thermalTemps: Map<string, number>; // compId → committed body temperature (°C)
  thermalDevices: Map<string, ThermalRuntimeState>; // exact catalog package thermal state
}

export type SimFailureKind =
  | "resistor_overload"
  | "fuse_tripped"
  | "led_failed"
  | "output_sag";

export interface SimFailure {
  componentId: string;
  kind: SimFailureKind;
  /** Simulation time, in seconds, when this failure latched. */
  since: number;
  message: string;
  pinId?: string;
  value?: number;
  limit?: number;
}

export interface SolverDiagnosticsSnapshot {
  lastIters: number;
  lastConverged: boolean;
  lastSolveUs: number;
  lastMatrixSize: number;
  lastMatrixSingular: boolean;
  lastMatrixIllConditioned: boolean;
  lastRelativeResidual: number;
}

// ─── DC operating point (Wave A3) ──────────────────────────────────────────

/**
 * Companion interval used by every operating-point and seed-rescue solve.
 * It matches load()'s historical 1 ps seed on purpose: time-dependent
 * sources sample the present instant (no digital event time advances), and
 * companion-based device state outside the dc-stamped set (relay coils,
 * motor windings, op-amp poles) holds instead of integrating physical time.
 */
const DC_OP_COMPANION_H = 1e-12;

/**
 * Gmin homotopy ladder (stage b). Every rung must converge before moving
 * down; the terminal 0 removes the overlay entirely so the accepted point
 * is a genuine solution of the unmodified system, not a gmin-loaded one.
 */
const DC_OP_GMIN_LADDER = [
  1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10, 1e-11, 1e-12, 0,
] as const;

// Source stepping (stage c) keeps a mild gmin load pinned while the sources
// ramp: 1e-9 S is strong enough to tame exponential devices at partial drive
// yet weak enough that the final full-drive rung is a short hop to gmin 0.
const DC_OP_SOURCE_STEP_GMIN_G = 1e-9;

// Pseudo-transient (stage d) envelope: backward-Euler relaxation steps grow
// from 1 ns to 1 ms (a decade at a time, a few steps per decade), and the
// point is accepted when one full-size step moves no net voltage by more
// than 1 uV. The budget bounds total work on truly stiff failures.
const DC_OP_PTA_H_MIN_S = 1e-9;
const DC_OP_PTA_H_MAX_S = 1e-3;
const DC_OP_PTA_STEPS_PER_H = 5;
const DC_OP_PTA_STEP_BUDGET = 500;
const DC_OP_PTA_SETTLE_V = 1e-6;

// Newton budget for operating-point and seed-rescue solves — the ITL1/ITL4
// split SPICE makes for the same reason. Per-frame transient solves keep the
// historical 50-iteration cap, but a one-shot OP solve may have to walk a
// junction voltage down from a far-off warm start, and the clamped-exponential
// device stamps descend by only ~n*Vt per iteration from deep forward bias
// (e.g. a base pinned volts above the emitter needs (V - vf)/0.026 steps).
// 1000 covers a 24 V rail's worth of walk with margin and stays a bounded
// one-shot cost.
const DC_OP_NEWTON_MAX_ITER = 1000;

/**
 * Stability screen for accepted operating-point candidates. Every root of
 * the DC equations is a fixed point of the ladder, but only points that are
 * stable under parasitic dynamics are physical resting states — a bistable
 * latch also has a metastable saddle no real circuit can sit at. The screen
 * uses the determinant pencil p(g) = det(J + g·P), P the identity on node
 * rows: p is the characteristic polynomial of the linearised circuit loaded
 * with a unit parasitic capacitance on every node, and its real positive
 * roots are exactly the growing real modes. Because sign(p(0)) and the sign
 * past the largest real root are both invariant under any positive diagonal
 * re-weighting of P, a sign difference proves an odd number of growing
 * modes for EVERY node-to-ground parasitic assignment — the screen can flag
 * only genuine saddles and can never reject a stable point. (Even counts of
 * growing modes pass undetected; the screen is a sufficient test, matching
 * its role as a rescue trigger rather than a proof of stability.)
 *
 * The dense O(n^3) determinant evaluations are a bounded one-shot cost at
 * OP time; systems larger than the cap skip the screen and keep today's
 * accept-on-convergence behavior.
 */
const DC_OP_STABILITY_MAX_SIZE = 320;

// Escape kicks off a rejected saddle: displacement magnitudes as fractions
// of the accepted point's own voltage span, tried along both orientations
// of the growing eigenmode. Two decades of magnitude cover both shallow
// unfoldings (weak mismatch) and hard-switching basins.
const DC_OP_ESCAPE_KICK_SCALES = [0.25, 1] as const;

// Component kinds whose state-commit handler integrates PHYSICAL time —
// h-scaled mechanics (motor inertia, servo travel, stepper phase), heat
// (thermistor self-heating), charge (battery depletion), or echo timing
// (hcsr04) — rather than electrical companion state. Operating-point solves
// advance no physical time, so these hold at their entry values for the
// whole ladder: stage d's pseudo-steps (h up to DC_OP_PTA_H_MAX_S, budget
// DC_OP_PTA_STEP_BUDGET) would otherwise integrate up to ~0.5 s of spin-up,
// self-heating, and drain into the committed point while simTime stands
// still. Capacitors/inductors/MOSFET gates stay OUT of this set: their
// companion updates are exactly how pseudo-transient relaxation moves.
const DC_OP_PHYSICAL_TIME_HOLD_KINDS: ReadonlySet<string> = new Set([
  "battery_pack",
  "thermistor",
  "dc_motor",
  "servo",
  "stepper",
  "hcsr04",
]);

/** Aggregate Newton/regime work counters threaded through the OP ladder. */
interface DcOpCounters {
  iterations: number;
  regimeIterations: number;
}

export interface DcOperatingPointResult {
  converged: boolean;
  /** Ladder stage that produced the accepted point (last stage tried on failure). */
  method: "direct" | "gmin" | "source" | "pseudo-transient";
  /** Total Newton iterations consumed across every stage attempted. */
  iterations: number;
  /** Committed-regime re-solves performed (stage a plus post-homotopy settles). */
  regimeIterations: number;
  /** Node voltages at the accepted operating point (pre-call values on failure). */
  netV: Record<string, number>;
}

/** Snapshot for save/restore during adaptive stepping in the worker. */
export interface StateSnapshot {
  caps: Map<string, number>;
  /** Optional for compatibility with snapshots created before capacitor current tracking. */
  capCurrents?: Map<string, number>;
  /** Optional for compatibility with snapshots created before trapezoidal integration. */
  capsI?: Map<string, number>;
  /** Optional for compatibility with snapshots created before trapezoidal integration. */
  indsV?: Map<string, number>;
  /** Pending force-backward-Euler flag; must survive rollback so replayed trials stamp the same method. */
  integrationBeNextStep?: boolean;
  inds: Map<string, number>;
  /** Optional for compatibility with snapshots created before MOS gate charge. */
  mosfetGates?: Map<string, MosfetGateState>;
  /** Optional for compatibility with snapshots created before battery depletion. */
  batteries?: Map<string, BatteryRuntimeState>;
  ne555s: Map<string, NE555EngineState>;
  relays: Map<string, RelayEngineState>;
  motors: Map<string, MotorEngineState>; // W7.1 — dc_motor rotor speed + winding current
  servos: Map<string, ServoEngineState>;     // W7.2 — servo PWM state + angle
  steppers: Map<string, StepperEngineState>; // W7.2 — stepper coil currents + position
  lcds: Map<string, Hd44780State>;           // W8.2 — HD44780 DDRAM + protocol state
  hcsr04: Map<string, Hcsr04EngineState>;    // S18b — HC-SR04 TRIG/ECHO state machine
  failureStress: Map<string, number>;
  failures: Map<string, SimFailure>;
  icState: Map<string, Record<string, number>>;
  eeproms: Map<string, EepromState>;
  ptcs: Map<string, PtcState>;
  thermalTemps: Map<string, number>;
  /** Optional for compatibility with snapshots created before package thermal coupling. */
  thermalDevices?: Map<string, ThermalRuntimeState>;
  netV: Record<string, number>;
  elementI: Record<string, number>;
  digitalState: Record<string, number>;
  /** Full converged MNA vector used to derive multi-channel component currents. */
  lastSolution?: Float64Array | null;
  /** Derived display output must roll back with the protocol state that produced it. */
  displayState?: Record<string, DisplayInfo>;
  /** Public solver diagnostics from the trusted state, not a rejected trial. */
  solverDiagnostics?: SolverDiagnosticsSnapshot;
  simTime: number;
}

/**
 * Minimal immutable view used by the worker's coarse/refined error estimate.
 * It intentionally excludes EEPROM bytes, LCD buffers, display/current
 * readouts, the MNA vector, and diagnostics because none participate in the
 * local truncation-error decision.
 */
export type ErrorStateSnapshot = Pick<
  StateSnapshot,
  | "netV"
  | "caps"
  | "capsI"
  | "indsV"
  | "inds"
  | "mosfetGates"
  | "batteries"
  | "ne555s"
  | "relays"
  | "motors"
  | "servos"
  | "steppers"
  | "hcsr04"
  | "failureStress"
  | "failures"
  | "icState"
  | "ptcs"
  | "thermalTemps"
  | "thermalDevices"
>;

function cloneDisplayState(
  state: Readonly<Record<string, DisplayInfo>>,
): Record<string, DisplayInfo> {
  return Object.fromEntries(
    Object.entries(state).map(([id, display]) => {
      switch (display.kind) {
        case "max7219":
          return [id, { ...display, rows: [...display.rows] }];
        case "hd44780":
          return [id, { ...display, lines: [...display.lines] }];
        case "microbit":
          return [id, { ...display, pixels: [...display.pixels] }];
      }
    }),
  );
}

// ─── Net-pair fingerprinting for cross-topology state carry ───────────────

/**
 * A net's "fingerprint" is the sorted list of all its (compId, pinId) pairs,
 * excluding the component whose state we're carrying (since that component
 * may have a new ID). Stable as long as the other pins on that net don't change.
 */
function netFingerprint(net: Net, excludeCompId: string): string {
  return net.pins
    .filter(([c]) => c !== excludeCompId)
    .map(([c, p]) => `${c}.${p}`)
    .sort()
    .join(",");
}

/**
 * Canonical key for the net-pair connected to a two-terminal element.
 * Normalised so swapping net0/net1 gives the same key; `flipped` tells
 * the caller whether to negate the stored value (for polarity consistency).
 */
function netPairKey(
  net0: Net,
  net1: Net,
  excludeCompId: string,
): { key: string; flipped: boolean } {
  const f0 = netFingerprint(net0, excludeCompId);
  const f1 = netFingerprint(net1, excludeCompId);
  if (f0 <= f1) {
    return { key: `${f0}||${f1}`, flipped: false };
  }
  return { key: `${f1}||${f0}`, flipped: true };
}

function failureKey(kind: SimFailureKind, componentId: string, pinId = ""): string {
  return `${kind}\x00${componentId}\x00${pinId}`;
}

function failureKeyComponentId(key: string): string | null {
  return key.split("\x00")[1] ?? null;
}

// Gate output resistance: 50 Ω minimum → stiff enough for typical LED/resistor
// loads while avoiding ill-conditioned MNA rows. Wave D raises this per family
// when catalog io_max says the chip cannot source/sink much current.
// Exported for devices/electromech-audio.ts (the hcsr04 ECHO drive); stays
// here because _icPowerInfo, _stampDigitalOutput, and the MCU board stamps
// share it.
export const GATE_R_OUT = 50;

const REGULATOR_QUIESCENT_CURRENT_A: Readonly<Record<string, number>> = {
  "reg-7805": 0.0065,
  "reg-ams1117-33": 0.005,
  "reg-ams1117-50": 0.005,
  // Approximates the LM317 adjustment-pin current omitted by the ideal pass
  // branch; the separate minimum-load requirement remains outside the model.
  lm317: 0.00005,
};

// Exported for devices/amps-regulators.ts (the linear_reg/lm317 stamps);
// stays here because the engine's _thermalDissipationW linear-regulator
// branch computes the same quiescent loss for package heating.
export function regulatorQuiescentCurrent(comp: SimComponent): number {
  const explicit = Number(comp.params.quiescentCurrent);
  if (Number.isFinite(explicit) && explicit >= 0) return explicit;
  if (!comp.catalogUid) return 0;
  const exactPart = partFor(comp);
  if (!exactPart || exactPart.uid !== comp.catalogUid) return 0;
  return REGULATOR_QUIESCENT_CURRENT_A[exactPart.uid] ?? 0;
}

const MCU_BOARD_IDLE_CURRENT_A: Readonly<Record<
  "arduino_uno" | "arduino_nano" | "raspberry_pi_pico" | "microbit",
  number
>> = {
  arduino_uno: 0.045,
  arduino_nano: 0.020,
  raspberry_pi_pico: 0.025,
  microbit: 0.015,
};

// S18b — see hcsr04MaxStepH()'s doc comment: a hard cap on outer step size,
// comfortably under the fixed 250 µs TRIG-fall-to-ECHO-rise delay, applied
// whenever the circuit contains an hcsr04.
const HCSR04_MAX_STEP_H = 100e-6;

// S18b — micro:bit devolt pin id ("p0".."p16", Feature 3 breakout set) -> the
// fork's own sim pin id ("pin0".."pin16"), scoped down to what the hcsr04
// ECHO bridge needs. Mirrors the eventual MicrobitHost EDGE_PINS map (Feature
// 3, a later agent's scope) but is intentionally its own small table here
// rather than importing/depending on MicrobitHost.tsx (out of this wave's
// scope) — keep in sync if that map's naming ever changes.
const MICROBIT_EDGE_PIN_TO_SIM: Record<string, string> = {
  p0: "pin0", p1: "pin1", p2: "pin2",
  p8: "pin8", p12: "pin12", p13: "pin13", p14: "pin14", p15: "pin15", p16: "pin16",
};

// Sequential IC kinds that carry internal state across load().
const COMBINATIONAL_IC_KINDS = new Set<string>([
  "74ls00", "74ls04", "74ls08", "74ls32", "74ls86", "74ls157", "74ls283", "74ls245",
  "74hc14", "74hc138", "74ls47",
]);

const IC_STATE_KINDS = new Set<string>([
  ...COMBINATIONAL_IC_KINDS,
  "74ls161", "74ls173", "74ls189", "74hc595", "74hc165",
  // 74HC14 icState carries per-input Schmitt hysteresis level in addition to delay keys.
  // W2.2 sequential ICs:
  "74hc74",  // dual D flip-flop — edge clock + async pre/clr state per FF
  "cd4017",  // decade counter — count + lastClk
  "cd4511",  // BCD latch/decoder — latchedBcd + lastLE
  "cd4060",  // 14-stage ripple counter with oscillator — count + lastClk (oscillator uses cap companion)
  // W4.1 op-amp ICs: icState stores the dominant-pole voltage, last input
  // differential, signed output current, and committed current-limit regime.
  // Keeping all analogue memory here makes save/restore and load-by-ID exact.
  "lm358",
  "mcp6002",
  "lm386",
  // W4.2 — LM393 comparator: icState stores committed output state per unit.
  // out1/out2: 1 = released (hi-Z), 0 = sinking (low). NaN → released (first step).
  "lm393",
  // W5.1 — linear regulators: icState stores committed regime (0=REG, 1=DROPOUT, 2=CC).
  // NaN on first step → initial regime is derived from headroom in regulatorRegime().
  "linear_reg",
  "lm317",
  // W6.2 — ULN2003/ULN2803 Darlington sink arrays: icState stores the committed
  // logic state for each input channel (in1..in7 for 2003, in1..in8 for 2803).
  // 1 = input HIGH (output sinks), 0 = input LOW (output hi-Z). NaN → LOW first step.
  "uln2003",
  "uln2803",
  // W6.2 — buzzer (active/passive): icState tracks last-sign and crossing-timestamp
  // for the passive frequency estimator. Active buzzer needs only sounding threshold.
  // Speaker tracks peak-to-peak variation over a short window.
  "buzzer",
  "speaker",
  // W6.3 — H-bridge motor drivers: icState stores committed output drive state
  // (1=HIGH, 0=LOW, -1=HIZ) for each output pin and the committed VM voltage.
  // First-step sentinel: all outputs absent from icState → stamp reads -1 (HIZ).
  "l293d",
  "tb6612",
  // W8.1 — MAX7219: icState stores the 16-bit shift register, bit counter,
  // all 8 digit/row registers, decode-mode, intensity, scan-limit, shutdown,
  // display-test, and the last committed CLK/DIN/CS levels for edge detection.
  "max7219",
  // Wave A6 — committed-regime device families (devices/model-families.ts):
  // scr/triac latch their conduction regime, opto_npn carries the committed
  // LED current its lagged photo drive reads, analog_switch latches the
  // committed control level. All are stamp-consulted regime state, so they
  // live in icState for snapshot/rollback and load-by-ID carry.
  "scr",
  "triac",
  "opto_npn",
  "analog_switch",
]);

/** Component kinds that can mutate accepted analogue/device state post-solve. */
const STATE_UPDATE_KINDS = new Set<string>([
  "battery_pack", "bench_psu", "buzzer", "capacitor", "dc_motor",
  "dcdc_converter", "hcsr04", "inductor", "l293d", "linear_reg", "lm317",
  "lm358", "lm386", "lm393", "mcp6002", "nmos", "pmos", "relay", "servo",
  "speaker", "stepper", "tb6612", "thermistor", "uln2003", "uln2803",
]);

/** Component kinds whose switch cases can publish or mutate digital state. */
const DIGITAL_UPDATE_KINDS = new Set<string>([
  "28c16", "28c256", "74hc138", "74hc14", "74hc165", "74hc595", "74hc74",
  "74ls00", "74ls04", "74ls08", "74ls157", "74ls161", "74ls173", "74ls189",
  "74ls245", "74ls283", "74ls32", "74ls47", "74ls86", "cd4017", "cd4060",
  "cd4511", "clock", "clock_gen", "hd44780", "l293d", "lm393", "max7219",
  "ne555", "pulse_gen", "pulse_source", "tb6612", "uln2003", "uln2803",
]);

/** Components that participate in the latched failure/damage pass. */
const PASSIVE_FAILURE_UPDATE_KINDS = new Set<string>([
  "28c16", "bicolor_led", "fuse", "led", "potentiometer", "ptc_fuse",
  "resistor", "resistor_array", "rgb_led", "trimmer", "uln2003",
]);

/**
 * Entire stamps that are invariant between load/failure-topology changes.
 * Source waveforms, companion models, nonlinear devices, thermal resistances,
 * and digitally switched output stages remain dynamic overlays.
 */
const STATIC_IDEAL_SOURCE_KINDS = new Set<string>([
  "voltage_source", "clock", "clock_gen", "pulse_source", "pulse_gen",
]);

const FULL_STATIC_STAMP_KINDS = new Set<string>([
  "resistor", "switch", "push_button", "spdt_switch", "push_dpdt",
  "dip_switch", "potentiometer", "trimmer", "ldr", "resistor_array",
  "ferrite_bead",
]);

// Explicit identity resolution depends only on (kind, catalogUid), so it is
// safe to share across engines and mutable authored params. Identity-less
// legacy resolution still runs live because its result can depend on params.
// The cache additionally keys on the injected part-library generation:
// setPartLibrary() changes what a (kind, catalogUid) pair resolves to, so a
// stale generation must drop every memoized identity or a host injecting
// after first use would keep resolving against the bundled defaults.
const explicitCatalogPartCache = new Map<string, PartDefinition | undefined>();
let explicitCatalogPartCacheVersion = getPartLibraryVersion();

function partFor(comp: SimComponent): PartDefinition | undefined {
  const libraryVersion = getPartLibraryVersion();
  if (libraryVersion !== explicitCatalogPartCacheVersion) {
    explicitCatalogPartCache.clear();
    explicitCatalogPartCacheVersion = libraryVersion;
  }
  const cacheKey = comp.catalogUid ? `${comp.kind}\0${comp.catalogUid}` : null;
  if (cacheKey && explicitCatalogPartCache.has(cacheKey)) {
    return explicitCatalogPartCache.get(cacheKey);
  }
  const part = resolveCatalogPart({
    kind: comp.kind as ComponentKind,
    params: comp.params,
    catalogUid: comp.catalogUid,
  }) ?? undefined;
  if (cacheKey) explicitCatalogPartCache.set(cacheKey, part);
  return part;
}

/**
 * Apply exact catalog defaults without overwriting authored instance values.
 * This keeps older identified saves on the current declared model when a new
 * optional physics parameter (ESR, DCR, Early voltage, gate capacitance) did
 * not exist in their serialized params yet. Stale/kind-mismatched identities
 * resolve to no part and therefore retain the explicit generic fallback.
 */
function modelParam(
  comp: SimComponent,
  key: string,
  fallback: number | string,
): number | string {
  if (comp.params[key] !== undefined) return comp.params[key]!;
  if (!comp.catalogUid) return fallback;
  const exactPart = partFor(comp);
  if (!exactPart || exactPart.uid !== comp.catalogUid) return fallback;
  return exactPart.default_params?.[key] ?? fallback;
}

const FLOATING_INPUT_FUNCTIONS = new Set<string>([
  "input", "clock", "reset", "enable", "load", "io", "tri_state",
]);

export interface LogicThresholds {
  vil: number;
  vih: number;
}

// Exported as the DeviceContext.icPowerInfo return type; the helper itself
// stays private because the unmigrated digital IC switch cases share it.
export interface IcPowerInfo {
  powered: boolean;
  vccPin: string | null;
  gndPin: string | null;
  vcc: number;
  gnd: number;
  vSupply: number;
  specs?: ElectricalSpecs;
  thresholds: LogicThresholds;
  outputResistance: number;
  propagationDelay: number;
}

// Exported for migrated device modules that fall back to per-kind defaults
// (devices/amps-regulators.ts); the table stays whole here because load()'s
// icState carry-forward and the unmigrated digital IC cases read it too.
export function defaultIcState(kind: string): Record<string, number> {
  switch (kind) {
    case "74ls161": return { count: 0, lastClk: 0 };
    case "74ls173": return { q1: 0, q2: 0, q3: 0, q4: 0, lastClk: 0 };
    case "74ls189": return { ...Object.fromEntries(Array.from({ length: 16 }, (_, i) => [`mem${i}`, 0xf])), lastWE: 1 };
    case "74hc595": return { shift: 0, latch: 0, lastSRCLK: 0, lastRCLK: 0 };
    case "74hc165": return { shift: 0, lastClk: 0, lastLoad: 0 };
    // W2.2 sequential ICs
    case "74hc74":
      // Per flip-flop: q/qn (committed outputs) and lastClk for edge detection.
      // q1n/q2n stored explicitly so the both-PRE-and-CLR-low case (Q=Q_n=HIGH)
      // is represented correctly rather than computed as !q.
      return { q1: 0, q1n: 1, lastClk1: 0, q2: 0, q2n: 1, lastClk2: 0 };
    case "cd4017":
      return { count: 0, lastClk: 0 };
    case "cd4511":
      // latchedBcd: the BCD value held by the latch register.
      // lastLE: previous latch-enable level for edge detection.
      return { latchedBcd: 0, lastLE: 0 };
    case "cd4060":
      // count: 14-bit ripple counter value.
      // lastClk: committed level of CLK/RS (pin 11) — used for falling-edge detection.
      // inv1Out: committed output of inverter stage 1; drives RTC (pin 10) = NOT(RS).
      // inv2Out: committed output of inverter stage 2; drives CTC (pin 9) = in-phase with RS.
      // Internal stages with no external pins: Q1-Q3, Q11.
      return { count: 0, lastClk: 0, inv1Out: 0, inv2Out: 0 };
    // W4.1 op-amps: committed output voltage for each unit, used by the stamp to
    // decide the saturation regime without consulting xGuess (which oscillates).
    // Op-amp saturation regime, committed once per step (see opAmpRegime).
    // uN is the dominant-pole internal voltage, dN the accepted input
    // differential, limitN the output regime (-1 source / 0 voltage / +1 sink),
    // and iN the signed MNA branch current. All live in icState so adaptive-step
    // rollback and load-by-ID state carry include the complete analogue memory.
    case "lm358":
    case "mcp6002":
      return {
        reg1: 0, d1: NaN, u1: NaN, limit1: 0, i1: 0,
        reg2: 0, d2: NaN, u2: NaN, limit2: 0, i2: 0,
        supplyV: 0,
      };
    case "lm386":
      return { reg1: 0, d1: NaN, u1: NaN, limit1: 0, i1: 0, supplyV: 0 };
    // W4.2 — LM393 comparator: committed output state per unit.
    // NaN means "uninitialized"; treated as released (hi-Z) until first commit.
    case "lm393":
      return { out1: NaN, out2: NaN };
    // W5.1 — linear regulators: committed regime 0=REG 1=DROPOUT 2=CC.
    // NaN is the initial sentinel; regulatorRegime() picks the starting regime
    // from input conditions on the first step.
    case "linear_reg":
    case "lm317":
      return { reg: NaN };
    // W6.2 — ULN2003 (7-channel): committed input state per channel.
    // in1..in7: 1 = sinking (input HIGH), 0 = hi-Z (input LOW). NaN → 0 first step.
    // No GND-valid flag in icState; checked live from _isOpenPin(comp.id, "gnd").
    case "uln2003":
      return { in1: NaN, in2: NaN, in3: NaN, in4: NaN, in5: NaN, in6: NaN, in7: NaN };
    // W6.2 — ULN2803 (8-channel): same pattern, one extra channel.
    case "uln2803":
      return { in1: NaN, in2: NaN, in3: NaN, in4: NaN, in5: NaN, in6: NaN, in7: NaN, in8: NaN };
    // W6.2 — buzzer: lastSign tracks zero-crossing state for passive frequency detection.
    //   lastSign: sign of last terminal voltage sample (1 or -1; NaN = uninitialised).
    //   lastCrossT: simTime of last zero-crossing (for period measurement).
    //   detectedHz: estimated frequency from last period (0 = DC or silent).
    // Active buzzers do not use lastSign/lastCrossT but share the same state slot.
    case "buzzer":
      return { lastSign: NaN, lastCrossT: NaN, detectedHz: 0 };
    // W6.2 — speaker: tracks peak and trough of terminal voltage over a short window.
    //   vPeak: running maximum terminal voltage in the window.
    //   vTrough: running minimum terminal voltage in the window.
    //   windowStart: simTime when the current peak-to-peak window started.
    //   peakToPeak: last committed peak-to-peak amplitude (V).
    case "speaker":
      return { vPeak: NaN, vTrough: NaN, windowStart: NaN, peakToPeak: 0 };
    // W6.3 — L293D: committed drive state per output (1=HIGH, 0=LOW, -1=HIZ).
    // All outputs start as HIZ (safe: no current flows on first step before
    // enable/input pins are resolved). vm is the committed motor rail voltage.
    case "l293d":
      return { out1: -1, out2: -1, out3: -1, out4: -1, vm: 0 };
    // W6.3 — TB6612: same committed-state pattern for 4 outputs (ao1/ao2/bo1/bo2).
    case "tb6612":
      return { ao1: -1, ao2: -1, bo1: -1, bo2: -1, vm: 0 };
    // W8.1 — MAX7219: power-on defaults from datasheet (all registers reset to 0).
    // CS idles HIGH, so lastCS=1. shutdown=0 means display is off at power-on
    // (chip is in shutdown mode until the user sends register 0x0C = 0x01).
    case "max7219":
      return {
        shift: 0, bits: 0,
        dig0: 0, dig1: 0, dig2: 0, dig3: 0, dig4: 0, dig5: 0, dig6: 0, dig7: 0,
        decodeMode: 0, intensity: 0, scanLimit: 0, shutdown: 0, displayTest: 0,
        lastCLK: 0, lastDIN: 0, lastCS: 1,
      };
    // Wave A6 — SCR: committed conduction regime. 0 = blocking (leak only),
    // 1 = conducting (vtm + ron branch). Gate triggering and holding-current
    // dropout are decided post-solve in the device's commitState.
    case "scr":
      return { on: 0 };
    // Wave A6 — triac: same latch plus the committed conduction polarity
    // (pol, +1/-1 in the mt1->mt2 current sense), frozen at trigger time so
    // the stamped on-state branch cannot flip inside a Newton solve.
    case "triac":
      return { on: 0, pol: 1 };
    // Wave A6 — optocoupler: committed LED forward current from the last
    // accepted solve; the output-side photo drive reads it one step lagged.
    // The slot is named i1 deliberately: adaptive-step's icStateTolerance
    // maps i1 to the branch-current tolerance, so coarse/refined trials
    // compare it as the continuous ampere quantity it is instead of the
    // exact-match regime rule (which would reject every checked step).
    case "opto_npn":
      return { i1: 0 };
    // Wave A6 — analog switch (transmission gate): committed control logic
    // level. 1 = on (ron), 0 = off (roff); hysteresis decided post-solve.
    case "analog_switch":
      return { on: 0 };
    default: return {};
  }
}

function specsForComponent(comp: SimComponent): ElectricalSpecs | undefined {
  return partFor(comp)?.electrical_specs ?? undefined;
}

// Exported for devices/amps-regulators.ts (the op-amp stamps); stays here
// because the engine's _thermalDissipationW analog-ic branch reads the same
// catalog quiescent draw for package heating.
export function analogIcQuiescentCurrent(comp: SimComponent): number {
  const value = Number(specsForComponent(comp)?.quiescent_current_a ?? 0);
  return Number.isFinite(value) ? Math.max(0, value) : 0;
}

function supplyPinsForComponent(comp: SimComponent): { vccPin: string | null; gndPin: string | null } {
  const part = partFor(comp);
  const vccPin = part?.pin_layout.find((p) => p.function === "vcc" || p.id === "vcc")?.id
    ?? (comp.kind === "ne555" ? "8" : null);
  const gndPin = part?.pin_layout.find((p) => p.function === "gnd" || p.id === "gnd")?.id
    ?? (comp.kind === "ne555" ? "1" : null);
  return { vccPin, gndPin };
}

export function thresholdsFor(specs: ElectricalSpecs | undefined, vSupply: number): LogicThresholds {
  const family = specs?.logic_family ?? "";
  // HC-CMOS and CMOS-4000 both use 30%/70% supply-ratioed thresholds.
  // CMOS-4000 parts accept Vcc 3-15 V; the same formula applies across the
  // full range because the datasheet specifies thresholds as Vcc fractions.
  if (family.includes("HC-CMOS") || family.includes("CMOS-4000")) {
    return { vil: Math.max(0.2, 0.3 * vSupply), vih: Math.max(0.4, 0.7 * vSupply) };
  }
  if (specs?.v_input) {
    return {
      vil: specs.v_input.v_il_max ?? Math.max(0.8, 0.3 * vSupply),
      vih: specs.v_input.v_ih_min ?? Math.max(2.0, 0.6 * vSupply),
    };
  }
  return { vil: Math.max(0.8, 0.3 * vSupply), vih: Math.max(2.0, 0.6 * vSupply) };
}

function propagationDelayForComponent(comp: SimComponent): number {
  // Prefer the catalog-declared value so individual parts can override the
  // family default without touching engine code.  Fall back to prefix map
  // for parts that carry no prop_delay_ns (all legacy parts as of this wave).
  const catalogDelay = partFor(comp)?.electrical_specs?.prop_delay_ns;
  if (catalogDelay != null && catalogDelay > 0) return catalogDelay * 1e-9;
  if (comp.kind.startsWith("74ls")) return 20e-9;
  if (comp.kind.startsWith("74hc")) return 15e-9;
  return 0;
}

function hash32(input: string): number {
  let h = 0x811c9dc5;
  for (let i = 0; i < input.length; i++) {
    h ^= input.charCodeAt(i);
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h >>> 0;
}

/**
 * LDR (photoresistor) resistance as a function of ambient light.
 * Pure-extracted (zero behavior change) from the "ldr" stamp/current
 * handlers (now in devices/passives.ts) so apps/sim/lib/inspector/
 * sensor-live.ts can reuse the exact same formula for the Inspector's live
 * resistance readout without duplicating it.
 */
export function ldrResistance(
  rDark: number,
  rLight: number,
  lux: number,
  r10 = 15_000,
  gamma = 0.7,
): number {
  if (lux <= 0) return rDark;
  const resistance = r10 * Math.pow(Math.max(lux, 1e-6) / 10, -Math.max(0.01, gamma));
  return Math.max(rLight, Math.min(rDark, resistance));
}

/**
 * NTC thermistor resistance (beta equation) as a function of ambient
 * temperature. Pure-extracted (zero behavior change) from the "thermistor"
 * stamp/current handlers (now in devices/passives.ts) — see ldrResistance
 * above for why.
 */
export function thermistorResistance(rNominal: number, beta: number, tempC: number): number {
  const T = tempC + 273.15;
  const T0 = 298.15;  // 25 C reference, Kelvin
  return Math.max(1, rNominal * Math.exp(beta * (1 / T - 1 / T0)));
}

export class SimEngine {
  nets: Net[] = [];
  simTime = 0;
  netV: Record<string, number> = {};
  /** Last-step current through each component, keyed by compId (A, pin0→pin1). */
  elementI: Record<string, number> = {};
  /** Digital output level (0 or 1) for clock, gate, DFF, and 555 outputs. */
  digitalState: Record<string, number> = {};
  /** Per-component display state for driven display components (e.g. MAX7219, HD44780). */
  private _displayState: Record<string, DisplayInfo> = {};
  /** Newton diagnostics from the last step — surfaced to the UI for debugging. */
  lastIters = 0;
  lastConverged = true;
  lastSolveUs = 0;
  lastMatrixSize = 0;
  lastMatrixSingular = false;
  lastMatrixIllConditioned = false;
  lastRelativeResidual = 0;
  /** Monotonic diagnostic counter for optimization benchmarks. */
  electricalSolveCount = 0;

  get matrixFactorizationCount(): number {
    return this.mna?.factorizationCount ?? 0;
  }

  get matrixFactorizationReuseCount(): number {
    return this.mna?.factorizationReuseCount ?? 0;
  }

  private nodeIdx!: Map<string, number>; // netId → matrix row (-1 = gnd)
  private vsrcIdx!: Map<string, number>; // compId → extra-var row
  // Wave A6 — `${compId}:${name}` → internal solver-node row. Allocated by
  // _buildMatrix from DeviceModel.internalNodes, inside [0, nodeCount) so
  // node-row machinery (RSHUNT seed, gmin overlay, _vAt, AC seed) covers
  // them, but absent from nodeIdx so no netV/probe path can ever see one.
  private internalNodeIdx: Map<string, number> = new Map();
  private nodeCount = 0;
  // Lazily built per _updateDigitalState call; cleared at its entry so every
  // step rebuilds it fresh from the current solved topology.
  private _dispDrivers: Map<number, { compId: string; pin: string }> | null = null;
  private mna!: LinearSystem;
  private _lastX: Float64Array | null = null;
  // ─── Integration method (Wave A2) ────────────────────────────────────────
  // "be" (default) keeps the historical backward-Euler companions and must
  // remain byte-identical to the pre-A2 engine. "trap" opts capacitors and
  // inductors into trapezoidal companions for second-order accuracy.
  private _integrationMethod: "be" | "trap" = "be";
  // Force the NEXT solve(s) to stamp backward Euler even in trap mode.
  // Trapezoidal integration rings across discontinuities because its history
  // carries the pre-jump derivative; one BE step re-anchors the history.
  // Set by load()/coldLoad()/resetFailures(), by the NE555 crossing split
  // (before its post-crossing sub-step), and by markDiscontinuity(). Cleared
  // only after an ACCEPTED step so failed/rejected trials retry with the same
  // method, and snapshotted so worker rollback replays identically.
  private _beNextStep = true;

  /** Select the transient integration method for capacitors and inductors. */
  setIntegrationMethod(method: "be" | "trap"): void {
    if (method === this._integrationMethod) return;
    this._integrationMethod = method;
    // A method switch changes the difference equation mid-trajectory; treat it
    // as a discontinuity so trap never starts from foreign history.
    this._beNextStep = true;
  }

  /**
   * The caller aligned the previous accepted step onto an event boundary
   * (source waveform edge, scheduled ECHO edge). The next step crosses or
   * departs that discontinuity, so it must integrate with backward Euler.
   */
  markDiscontinuity(): void {
    this._beNextStep = true;
  }

  /** Trap companions apply only in trap mode and away from discontinuities. */
  private _useTrapThisSolve(): boolean {
    return this._integrationMethod === "trap" && !this._beNextStep;
  }

  // ─── DC operating point stamping controls (Wave A3) ─────────────────────
  // All of these are inert at their defaults and are mutated ONLY inside
  // dcOperatingPoint()/_rescueSeedOperatingPoint(), always restored in a
  // finally block. The stamp-side controls are consulted exclusively at
  // stamp time so the static-base capture and the exact-values LU reuse stay
  // coherent: each homotopy rung genuinely changes the stamped matrix/RHS,
  // while the captured base and its signature never see the overlay at all.
  //
  // True-OP stamping: capacitors stamp as opens (declared leakage shunt
  // only; MOSFET gate charge dropped) and inductors as their winding
  // resistance, turning the companion transient solve into a DC solve.
  private _dcSolveMode = false;
  // Gmin homotopy overlay: when > 0, _stampAll adds this conductance from
  // every non-ground node to ground on top of the static RSHUNT base.
  private _homotopyGminG = 0;
  // Source homotopy: when < 1, every INDEPENDENT source magnitude is scaled
  // at stamp time (see _independentSourceMagnitude for the canonical list).
  private _homotopySourceScale = 1;
  // Newton budget for the solve in flight: null means the historical
  // transient cap (50). Raised to DC_OP_NEWTON_MAX_ITER only while the
  // operating-point ladder runs, since `?? 50` is exact at the default.
  private _newtonMaxIterOverride: number | null = null;
  // Operating-point physical-time hold: OP-owned solves advance no physical
  // time, so while this is set (a) waveform sources sample the OP instant
  // regardless of the solve's companion/pseudo interval (see
  // _waveformSampleTime) and (b) the h-scaled physical integrators — package
  // thermal, failure stress, and the DC_OP_PHYSICAL_TIME_HOLD_KINDS branches
  // of _updateState — hold at their entry values instead of stepping.
  private _dcPhysicalTimeHold = false;
  // Digital hold for homotopy rung solves: rung voltages are physically
  // meaningless waypoints (gmin-loaded nodes, partially driven sources), so
  // _updateDigitalState must not read them — a strong gmin rung sags a HIGH
  // clock line below VIL and the next rung releases it, which would commit a
  // phantom rising edge into counters/latches with simTime frozen. Set only
  // around _homotopyRungConverged's solve; settle and pseudo-transient
  // solves keep digital updates so the accepted point's regime/latch picture
  // reconciles against true-system voltages.
  private _dcDigitalHold = false;
  // Sparse-backend stamp-pattern watermark captured when load() finishes;
  // undefined on backends without pattern state (dense). See load() and
  // dcOperatingPoint for the bit-identity contract it protects.
  private _mnaLoadPatternWatermark: number | undefined;

  private state: ElementState = {
    caps: new Map(),
    capCurrents: new Map(),
    capsI: new Map(),
    indsV: new Map(),
    inds: new Map(),
    mosfetGates: new Map(),
    batteries: new Map(),
    ne555s: new Map(),
    relays: new Map(),
    motors: new Map(),   // W7.1
    servos: new Map(),   // W7.2
    steppers: new Map(), // W7.2
    lcds: new Map(),     // W8.2
    hcsr04: new Map(),   // S18b
    failureStress: new Map(),
    failures: new Map(),
    icState: new Map(),
    arduinos: new Map(),
    eeproms: new Map(),
    ptcs: new Map(),
    thermalTemps: new Map(),
    thermalDevices: new Map(),
  };
  private circuit: SimCircuit | null = null;
  /** Immutable for one loaded topology; hot stamping/state paths only read it. */
  private _componentById = new Map<string, SimComponent>();
  private _pinToNetIndex = new Map<string, string>();
  private _pinNodeIndex = new Map<string, number>();
  private _openPinKeys = new Set<string>();
  private _thermalProfileCache = new Map<string, ThermalDeviceProfile | undefined>();
  private _batteryModelCache = new Map<string, {
    profile: BatteryPhysicsProfile;
    nominalVoltageScale: number;
    referenceInternalResistanceOhm: number | undefined;
  }>();
  private _hasNe555 = false;
  private _hasHcsr04 = false;
  private _ne555Components: SimComponent[] = [];
  private _mcuComponents: SimComponent[] = [];
  private _stateUpdateComponents: SimComponent[] = [];
  private _digitalUpdateComponents: SimComponent[] = [];
  private _failureUpdateComponents: SimComponent[] = [];
  private _thermalComponents: SimComponent[] = [];
  private _staticIdealSourceIds = new Set<string>();
  private _staticStampComponents: SimComponent[] = [];
  private _dynamicStampComponents: SimComponent[] = [];
  // Wave A4: device-model pointers aligned index-for-index with the bucket
  // (or full component) list each pass iterates. Registration is immutable
  // after devices/index.ts evaluates (duplicates throw) and bucket
  // membership is already frozen per load, so resolving the model per
  // component per pass visit was pure hot-path overhead — the A4 perf
  // review measured the repeated registry lookups on every Newton
  // iteration. Rebuilt in _compileRuntimeMetadata alongside the buckets.
  private _stateUpdateModels: (DeviceModel | undefined)[] = [];
  private _digitalUpdateModels: (DeviceModel | undefined)[] = [];
  private _failureUpdateModels: (DeviceModel | undefined)[] = [];
  private _staticStampModels: (DeviceModel | undefined)[] = [];
  private _dynamicStampModels: (DeviceModel | undefined)[] = [];
  private _elementCurrentModels: (DeviceModel | undefined)[] = [];
  private _staticStampSignature = "";
  private _combinationalEvalCache = new Map<string, {
    signature: string;
    outputs: Record<string, boolean>;
  }>();
  private _nextDigitalDueTimeCache: number | null = null;
  private _digitalDueCacheDirty = true;
  digitalEvaluationCount = 0;
  digitalEvaluationReuseCount = 0;

  // pnjlim limiter state for BJT junctions only. `_vPrevBJT` holds each
  // junction's voltage from the previous Newton iteration — pnjlim's
  // `vOld`. Cleared at the top of every _solve() so iteration 1 falls back
  // to the raw guess (which equals the previous time-step's converged
  // value via xInit), satisfying SPICE's "vOld = previous converged step
  // on iter 1" convention implicitly.
  //
  // Plain diodes/LEDs are intentionally NOT pnjlim-damped: the standalone
  // diode model in this engine is current-driven (typically through a
  // series resistor) and is already well-conditioned. Applying pnjlim to
  // them creates oscillation where Newton would otherwise converge.
  private _vPrevBJT: Map<string, { vBE: number; vBC: number }> = new Map();
  // True when the most recent stamp pass clamped at least one junction via
  // pnjlim. solveNonlinear consults this through limitedThisIteration so a
  // bit-still iterate cannot be accepted while the limiter is still walking
  // a junction's evaluation voltage (the delta-x test alone would report
  // false convergence with, e.g., a base parked volts into forward bias).
  // Reset at the top of every stamp closure in _solve.
  private _junctionLimitedThisIteration = false;
  private _bjtCache: Map<string, { IsT: number; VtNF: number; VtNR: number; vCritBE: number; vCritBC: number }> = new Map();

  // Current-limit regimes are accepted-step state, but limit entry and voltage
  // compliance are algebraic constraints. A switched hard load must not get one
  // accepted ideal-CV overcurrent frame, and a committed CC source whose load
  // is removed must not spend one physical step as an unconstrained ideal
  // current source (I_limit into RSHUNT would be teravolts). During each Newton
  // solve these sets monotonically select CC or its voltage-compliance branch
  // from provisional candidates before any electrical/thermal state is accepted.
  // It is cleared before every solve, so rejected/adaptive trial steps cannot
  // leak active-set choices into a later attempt.
  private _currentLimitComplianceClamps = new Set<string>();
  private _currentLimitCurrentBranchTried = new Set<string>();
  private _currentLimitEntryClamps = new Set<string>();
  private _regulatorHeadroomBranchTried = new Set<string>();
  private _regulatorHeadroomSelections = new Map<string, 0 | 1 | 3>();

  private _regulatorHeadroomRegime(
    key: string,
    acceptedRegime: number,
    headroomGuess: number,
    regulatedVoltage: number,
    dropoutVoltage: number,
  ): 0 | 1 | 3 {
    const accepted: 0 | 1 | 3 = acceptedRegime === 1
      ? 1
      : acceptedRegime === 3
        ? 3
        : 0;
    // Let the accepted branch establish the present topology/rail candidate
    // once before interpreting headroom. This avoids treating the all-zero
    // cold-start vector or a newly-renumbered net as a real brownout.
    if (!this._regulatorHeadroomBranchTried.has(key)) {
      this._regulatorHeadroomBranchTried.add(key);
      return accepted;
    }

    const required: 0 | 1 | 3 = headroomGuess <= Math.max(0, dropoutVoltage)
      ? 3
      : headroomGuess < regulatedVoltage + dropoutVoltage
        ? 1
        : 0;
    const selected = this._regulatorHeadroomSelections.get(key);
    if (selected === undefined) {
      this._regulatorHeadroomSelections.set(key, required);
      return required;
    }

    // After the first present-solve selection, capability can only decrease:
    // REG -> DROPOUT -> OFF. A restoration candidate may initially select REG,
    // then conservatively fall back if loading makes its own input sag.
    const capability = (regime: 0 | 1 | 3): number => regime === 0 ? 2 : regime === 1 ? 1 : 0;
    if (capability(required) < capability(selected)) {
      this._regulatorHeadroomSelections.set(key, required);
      return required;
    }
    return selected;
  }

  private _useCurrentLimitEntryClamp(
    key: string,
    branchCurrentGuess: number,
    currentLimit: number,
  ): boolean {
    if (this._currentLimitEntryClamps.has(key)) return true;
    if (!Number.isFinite(branchCurrentGuess) || !(currentLimit > 0)) return false;
    const tolerance = 1e-12 + 1e-6 * currentLimit;
    if (Math.abs(branchCurrentGuess) > currentLimit + tolerance) {
      this._currentLimitEntryClamps.add(key);
      return true;
    }
    return false;
  }

  private _useCurrentLimitComplianceClamp(
    key: string,
    outputMagnitudeGuess: number,
    complianceMagnitude: number,
  ): boolean {
    if (this._currentLimitComplianceClamps.has(key)) return true;
    // A newly-entered CC regime inherits the preceding CV solution as xInit.
    // Require one actual current-branch candidate before treating a setpoint
    // voltage as recovery; otherwise every overload would immediately bounce
    // back to CV without ever enforcing its current limit.
    if (!this._currentLimitCurrentBranchTried.has(key)) {
      this._currentLimitCurrentBranchTried.add(key);
      return false;
    }
    if (!Number.isFinite(outputMagnitudeGuess) || !Number.isFinite(complianceMagnitude)) {
      return false;
    }
    const boundedCompliance = Math.max(0, complianceMagnitude);
    const tolerance = 1e-9 + 1e-6 * Math.max(1, boundedCompliance);
    if (outputMagnitudeGuess >= boundedCompliance - tolerance) {
      this._currentLimitComplianceClamps.add(key);
      return true;
    }
    return false;
  }

  // Resolve ambient lux for an LDR component. The fallback chain is:
  //   circuit.environment (v2+ saves) → comp.params.lux (v1 saves and test
  //   fixtures that feed unmigrated JSON straight into the engine, e.g.
  //   apps/sim/lib/sim/engine/fixture-audit.test.ts) → historical default 100.
  private _envLux(comp: SimComponent): number {
    return Math.max(
      0,
      Number(this.circuit?.environment?.lux ?? comp.params.lux ?? 100),
    );
  }

  // Resolve ambient temperature for a thermistor component. Same fallback
  // chain as _envLux — see comment there.
  private _envTempC(comp: SimComponent): number {
    return Number(this.circuit?.environment?.temperatureC ?? comp.params.tempC ?? 25);
  }

  private _ambientTempC(): number {
    return Number(this.circuit?.environment?.temperatureC ?? NOMINAL_TEMP_C);
  }

  private _junctionVt(): number {
    return thermalVoltage(this._ambientTempC());
  }

  private _junctionVf(vfAt25C: number): number {
    return junctionForwardVoltage(vfAt25C, this._ambientTempC());
  }

  private _thermalProfile(comp: SimComponent): ThermalDeviceProfile | undefined {
    if (this._thermalProfileCache.has(comp.id)) {
      return this._thermalProfileCache.get(comp.id);
    }
    if (!comp.catalogUid) {
      this._thermalProfileCache.set(comp.id, undefined);
      return undefined;
    }
    // Thermal identity must pass the same UID/kind validation as electrical
    // identity. A stale or kind-mismatched UID must not lend another package's
    // temperature/shutdown behavior to a custom component.
    if (!partFor(comp)) {
      this._thermalProfileCache.set(comp.id, undefined);
      return undefined;
    }
    const resolved = resolveThermalProfile({ catalogUid: comp.catalogUid });
    const profile = resolved.source === "exact-catalog-uid" ? resolved.profile : undefined;
    this._thermalProfileCache.set(comp.id, profile);
    return profile;
  }

  private _batteryModel(comp: SimComponent): {
    profile: BatteryPhysicsProfile;
    nominalVoltageScale: number;
    referenceInternalResistanceOhm: number | undefined;
  } {
    const cached = this._batteryModelCache.get(comp.id);
    if (cached) return cached;
    // Only an explicit identity may contribute catalog defaults. Identity-less
    // legacy/custom sources stay generic and preserve their authored ideality.
    const defaults = comp.catalogUid ? partFor(comp)?.default_params : undefined;
    const configuredVoltage = Math.max(
      0,
      Number(comp.params.voltage ?? defaults?.voltage ?? 5),
    );
    const rawResistance = comp.params.rInternal ?? defaults?.rInternal;
    const configuredResistance = rawResistance === undefined
      ? undefined
      : Math.max(0, Number(rawResistance));
    const resolution = resolveBatteryProfile({
      // Missing identity intentionally stays generic; never infer chemistry
      // from kind or catalog ordering for a legacy/custom source.
      catalogUid: comp.catalogUid,
      fallbackNominalVoltageV: configuredVoltage,
      fallbackCapacityAh: Number(comp.params.capacityAh ?? 1),
      fallbackFreshInternalResistanceOhm: configuredResistance ?? 0,
    });
    const capacityAh = Math.max(
      1e-12,
      Number(comp.params.capacityAh ?? resolution.profile.referenceCapacityAh),
    );
    const profile = capacityAh === resolution.profile.referenceCapacityAh
      ? resolution.profile
      : { ...resolution.profile, referenceCapacityAh: capacityAh };
    const model = {
      profile,
      nominalVoltageScale: profile.nominalVoltageV > 0
        ? configuredVoltage / profile.nominalVoltageV
        : 1,
      referenceInternalResistanceOhm: configuredResistance,
    };
    this._batteryModelCache.set(comp.id, model);
    return model;
  }

  private _batteryOperatingPoint(comp: SimComponent) {
    const authoredCharge = Math.max(0, Math.min(1, Number(comp.params.charge ?? 1)));
    const state = this.state.batteries.get(comp.id)
      ?? {
        ...createBatteryState(authoredCharge),
        authoredCharge,
        catalogUid: comp.catalogUid ?? null,
      };
    const model = this._batteryModel(comp);
    const point = batteryOperatingPoint(model.profile, {
      soc: state.soc,
      ambientTemperatureC: this._ambientTempC(),
      referenceInternalResistanceOhm: model.referenceInternalResistanceOhm,
    });
    return { state, ...model, point };
  }

  // ─── Device-model dispatch (Wave A4) ────────────────────────────────────
  // The registry facade handed to migrated device handlers. Every member is
  // a thin alias of the exact private expression the old switch cases used
  // (see device-registry.ts, decision 2), so dispatching through it cannot
  // change a single float. Getter members track load()-time replacement of
  // this.mna and this.state; closures defer all reads to call time, so the
  // field initializer order is irrelevant.
  private _elementIOut: Record<string, number> | null = null;
  // Pass-scoped digitalState/display sinks for _updateDigitalState: device
  // updateDigital handlers publish through ctx.setDigitalState and
  // ctx.setDisplayInfo into the same per-step maps the switch cases write.
  private _digitalStateOut: Record<string, number> | null = null;
  private _displayStateOut: Record<string, DisplayInfo> | null = null;
  // Pass-scoped pre-commit MOSFET gate snapshot for _updateElementI: the
  // step commit captures state.mosfetGates before _updateState overwrites
  // it, and Cgd displacement current must difference against the history
  // the stamp actually used. Devices reach it via ctx.previousMosfetGate.
  private _previousMosfetGates: ReadonlyMap<string, MosfetGateState> | null = null;
  // Pass-scoped lazy MCU pin-driver map for _updateState device handlers
  // (servo PWM decode, hcsr04 TRIG edges), reached via
  // ctx.mcuEventDriverForNode: built with includePico=true on first use,
  // reset at _updateState entry. Deliberately SEPARATE from `_dispDrivers`
  // above — that cache is built includePico=false and the display decoders
  // must keep their exact prior uno/nano-only behaviour.
  private _mcuEventDrivers: Map<number, { compId: string; pin: string }> | null = null;
  // Pass-scoped lazy channel-current readout for _updateFailureStates,
  // reached via ctx.elementChannelCurrents: the old pass computed
  // getElementChannelI() at most once and shared it across every
  // multi-junction LED's stress commit; the box restores that compute-once
  // lifetime. Non-null (armed) only inside the pass — `value` stays null
  // until the first consumer computes it, exactly HEAD's `??=` pass local —
  // so callers outside the pass can never observe a stale readout.
  private _failureChannelCurrents: { value: Record<string, number[]> | null } | null = null;
  // Visit-scoped specs memo for _updateFailureStates: the pass resolves
  // specs for its generic vcc_range scan and the dispatched updateFailures
  // handler re-reads them via ctx.electricalSpecs. Identity-less catalog
  // resolution runs live (it may depend on in-place param mutations), so
  // without the memo every failure-bucket component paid that resolution
  // twice per step where the old inline blocks shared the loop's one local.
  // Valid only for the component currently under failure dispatch; every
  // other ctx.electricalSpecs call resolves fresh, exactly like the old
  // switch bodies did.
  private _failureSpecsComp: SimComponent | null = null;
  private _failureSpecsValue: ElectricalSpecs | undefined = undefined;
  private readonly _deviceCtx: DeviceContext = this._buildDeviceContext();

  private _buildDeviceContext(): DeviceContext {
    const engine = this;
    return {
      pinNode: (compId, pinId) => engine._pinNode(compId, pinId),
      isOpenPin: (compId, pinId) => engine._isOpenPin(compId, pinId),
      vsrcRow: (key) => engine.vsrcIdx.get(key),
      internalNode: (compId, name) =>
        engine.internalNodeIdx.get(`${compId}:${name}`) ?? -1,
      vAt: (x, row) => engine._vAt(x, row),
      get mna() {
        return engine.mna;
      },
      simTime: () => engine.simTime,
      modelParam: (comp, key, fallback) => modelParam(comp, key, fallback),
      electricalSpecs: (comp) =>
        // Reuse the failure pass's per-visit resolution (see the field's
        // comment); outside that visit this is the plain live resolution.
        comp === engine._failureSpecsComp
          ? engine._failureSpecsValue
          : specsForComponent(comp),
      catalogPart: (comp) => partFor(comp),
      envLux: (comp) => engine._envLux(comp),
      envTempC: (comp) => engine._envTempC(comp),
      ambientTempC: () => engine._ambientTempC(),
      junctionVt: () => engine._junctionVt(),
      junctionVf: (vfAt25C) => engine._junctionVf(vfAt25C),
      integrationMethod: () => engine._integrationMethod,
      useTrapThisSolve: () => engine._useTrapThisSolve(),
      dcSolveMode: () => engine._dcSolveMode,
      independentSourceMagnitude: (value) => engine._independentSourceMagnitude(value),
      waveformSampleTime: (h) => engine._waveformSampleTime(h),
      stampIndependentVoltageSource: (comp, branch, voltage) =>
        engine._stampIndependentVoltageSource(comp, branch, voltage),
      batteryOperatingPoint: (comp) => engine._batteryOperatingPoint(comp),
      useCurrentLimitEntryClamp: (key, branchCurrentGuess, currentLimit) =>
        engine._useCurrentLimitEntryClamp(key, branchCurrentGuess, currentLimit),
      useCurrentLimitComplianceClamp: (key, outputMagnitudeGuess, complianceMagnitude) =>
        engine._useCurrentLimitComplianceClamp(key, outputMagnitudeGuess, complianceMagnitude),
      currentLimitEntryClampActive: (key) => engine._currentLimitEntryClamps.has(key),
      currentLimitComplianceClampActive: (key) => engine._currentLimitComplianceClamps.has(key),
      bjtJunctionCache: (compId) => engine._bjtCache.get(compId),
      bjtPrevJunction: (compId) => engine._vPrevBJT.get(compId),
      setBjtPrevJunction: (compId, prev) => {
        engine._vPrevBJT.set(compId, prev);
      },
      markJunctionLimitedThisIteration: () => {
        engine._junctionLimitedThisIteration = true;
      },
      previousMosfetGate: (compId) => {
        // Pass-scoped on purpose (mirrors setElementCurrent): outside the
        // element-current pass there is no meaningful pre-commit snapshot,
        // so a read there would silently alias the wrong step's history.
        if (!engine._previousMosfetGates) {
          throw new Error("previousMosfetGate is only valid during the element-current pass");
        }
        return engine._previousMosfetGates.get(compId);
      },
      elementChannelCurrents: () => {
        // Compute-once inside the failure pass (see the field's comment);
        // outside it the cache is disarmed and every call computes fresh.
        const cache = engine._failureChannelCurrents;
        if (!cache) return engine.getElementChannelI();
        return cache.value ??= engine.getElementChannelI();
      },
      icPowerInfo: (comp, x) => engine._icPowerInfo(comp, x),
      stampDigitalOutput: (comp, pinId, high, power) =>
        engine._stampDigitalOutput(comp, pinId, high, power),
      regulatorHeadroomRegime: (key, acceptedRegime, headroomGuess, regulatedVoltage, dropoutVoltage) =>
        engine._regulatorHeadroomRegime(
          key,
          acceptedRegime,
          headroomGuess,
          regulatedVoltage,
          dropoutVoltage,
        ),
      logicHigh: (comp, pinId, x, power) => engine._logicHigh(comp, pinId, x, power),
      logicHighH: (comp, pinId, x, power) => engine._logicHighH(comp, pinId, x, power),
      schmittLogicHigh: (comp, pinId, x, power, commit) =>
        engine._schmittLogicHigh(comp, pinId, x, power, commit),
      stampFloatingDigitalInputs: (comp, x, power) =>
        engine._stampFloatingDigitalInputs(comp, x, power),
      combinationalOutputs: (comp, xGuess, power, commit) =>
        // An omitted commit reaches the engine helper as undefined, which
        // its `commit = false` default resolves exactly like the old
        // three-argument call sites did.
        engine._combinationalOutputs(comp, xGuess, power, commit),
      delayedOutputLevel: (comp, pinId, immediate) =>
        engine._delayedOutputLevel(comp, pinId, immediate),
      updateDelayedOutputs: (comp, immediate, h, power) =>
        engine._updateDelayedOutputs(comp, immediate, h, power),
      stampRailReferencedOutput: (node, lowRail, highRail, fraction, outputResistance) =>
        engine._stampRailReferencedOutput(node, lowRail, highRail, fraction, outputResistance),
      readAddr4: (comp, x, power) => engine._readAddr4(comp, x, power),
      mergedDisplayEvents: (comp, pinNames) => {
        // Same lazy shared driver map the old max7219/hd44780 cases built:
        // the first display visited in the pass builds it (uno/nano only —
        // includePico stays false here), later displays reuse it.
        engine._dispDrivers ??= engine._buildArduinoPinDrivers();
        return engine._mergedDisplayEvents(comp, pinNames, engine._dispDrivers);
      },
      setDigitalState: (key, value) => {
        // Pass-scoped on purpose (mirrors setElementCurrent): digital state
        // is committed once per accepted step in _updateDigitalState.
        if (!engine._digitalStateOut) {
          throw new Error("setDigitalState is only valid during the digital-state pass");
        }
        engine._digitalStateOut[key] = value;
      },
      setDisplayInfo: (compId, info) => {
        if (!engine._displayStateOut) {
          throw new Error("setDisplayInfo is only valid during the digital-state pass");
        }
        engine._displayStateOut[compId] = info;
      },
      componentById: (compId) => engine._componentById.get(compId),
      mcuEventDriverForNode: (node) => {
        // Same lazy shared driver map the old servo/hcsr04 _updateState
        // blocks built once per pass (includePico=true): the first handler
        // consulting it builds it, later ones reuse it; _updateState resets
        // it at entry so every step sees the freshly loaded topology.
        engine._mcuEventDrivers ??= engine._buildArduinoPinDrivers(true);
        return engine._mcuEventDrivers.get(node);
      },
      mcuPowered: (comp, x) => engine._mcuPowered(comp, x),
      mcuBootSimTime: (compId) => engine._mcuBootSimTime.get(compId),
      hcsr04MicrobitBridge: (comp) => engine._hcsr04MicrobitBridge(comp),
      state: {
        get caps() {
          return engine.state.caps;
        },
        get capCurrents() {
          return engine.state.capCurrents;
        },
        get capsI() {
          return engine.state.capsI;
        },
        get inds() {
          return engine.state.inds;
        },
        get indsV() {
          return engine.state.indsV;
        },
        get ptcs() {
          return engine.state.ptcs;
        },
        get thermalTemps() {
          return engine.state.thermalTemps;
        },
        get batteries() {
          return engine.state.batteries;
        },
        get icState() {
          return engine.state.icState;
        },
        get ne555s() {
          return engine.state.ne555s;
        },
        get eeproms() {
          return engine.state.eeproms;
        },
        get lcds() {
          return engine.state.lcds;
        },
        get mosfetGates() {
          return engine.state.mosfetGates;
        },
        get relays() {
          return engine.state.relays;
        },
        get motors() {
          return engine.state.motors;
        },
        get servos() {
          return engine.state.servos;
        },
        get steppers() {
          return engine.state.steppers;
        },
        get hcsr04() {
          return engine.state.hcsr04;
        },
        get arduinos() {
          return engine.state.arduinos;
        },
        get thermalDevices() {
          return engine.state.thermalDevices;
        },
      },
      hasFailure: (compId, kind, pinId = "") => engine._hasFailure(compId, kind, pinId),
      recordAccumulatedStress: (kind, componentId, pinId, stressRate, recoveryRate, h, threshold, makeFailure) =>
        engine._recordAccumulatedStress(
          failureKey(kind, componentId, pinId),
          stressRate,
          recoveryRate,
          h,
          threshold,
          makeFailure,
        ),
      elementCurrent: (compId) => engine.elementI[compId],
      setElementCurrent: (compId, amps) => {
        // Pass-scoped on purpose: element currents are committed once per
        // accepted solve in _updateElementI; a device writing outside that
        // pass would desynchronize elementI from the solution it describes.
        if (!engine._elementIOut) {
          throw new Error("setElementCurrent is only valid during the element-current pass");
        }
        engine._elementIOut[compId] = amps;
      },
    };
  }

  // ─── Snapshot API (used by the worker for adaptive-step rollback) ──────

  saveState(): StateSnapshot {
    const snapshot: StateSnapshot = {
      caps: new Map(this.state.caps),
      capCurrents: new Map(this.state.capCurrents),
      integrationBeNextStep: this._beNextStep,
      inds: new Map(this.state.inds),
      mosfetGates: new Map(
        [...this.state.mosfetGates].map(([k, v]) => [k, { ...v }]),
      ),
      batteries: new Map([...this.state.batteries].map(([k, v]) => [k, { ...v }])),
      ne555s: new Map(this.state.ne555s),
      relays: new Map([...this.state.relays].map(([k, v]) => [k, { ...v }])),
      motors: new Map([...this.state.motors].map(([k, v]) => [k, { ...v }])),   // W7.1
      servos: new Map([...this.state.servos].map(([k, v]) => [k, { ...v }])),   // W7.2
      steppers: new Map([...this.state.steppers].map(([k, v]) => [k, { ...v }])), // W7.2
      // W8.2 — deep-copy: ddram and lastData are arrays that must not share references.
      lcds: new Map([...this.state.lcds].map(([k, v]) => [k, { ...v, ddram: [...v.ddram], lastData: [...v.lastData] }])),
      hcsr04: new Map([...this.state.hcsr04].map(([k, v]) => [k, { ...v }])), // S18b
      failureStress: new Map(this.state.failureStress),
      failures: new Map([...this.state.failures].map(([k, v]) => [k, { ...v }])),
      icState: new Map([...this.state.icState].map(([k, v]) => [k, { ...v }])),
      eeproms: new Map(
        [...this.state.eeproms].map(([k, v]) => [k, {
          ...v,
          bytes: new Uint8Array(v.bytes),
        }]),
      ),
      ptcs: new Map([...this.state.ptcs].map(([k, v]) => [k, { ...v }])),
      thermalTemps: new Map(this.state.thermalTemps),
      thermalDevices: new Map(
        [...this.state.thermalDevices].map(([k, v]) => [k, { ...v, warnings: [...v.warnings] }]),
      ),
      netV: { ...this.netV },
      elementI: { ...this.elementI },
      digitalState: { ...this.digitalState },
      lastSolution: this._lastX ? new Float64Array(this._lastX) : null,
      displayState: cloneDisplayState(this._displayState),
      solverDiagnostics: {
        lastIters: this.lastIters,
        lastConverged: this.lastConverged,
        lastSolveUs: this.lastSolveUs,
        lastMatrixSize: this.lastMatrixSize,
        lastMatrixSingular: this.lastMatrixSingular,
        lastMatrixIllConditioned: this.lastMatrixIllConditioned,
        lastRelativeResidual: this.lastRelativeResidual,
      },
      simTime: this.simTime,
    };
    // Trap-mode-only: the histories are read only by trap stamps, so copying
    // them per snapshot would be pure overhead on the default checked-step
    // path (saveState runs before every coarse/refined replay). A BE-mode
    // rollback leaves the live maps untouched instead — they are write-free in
    // BE mode and a later switch to trap re-anchors via _beNextStep before any
    // stamp reads them.
    if (this._integrationMethod === "trap") {
      snapshot.capsI = new Map(this.state.capsI);
      snapshot.indsV = new Map(this.state.indsV);
    }
    return snapshot;
  }

  /** Capture only state inspected by estimateStepError(). */
  captureErrorState(): ErrorStateSnapshot {
    const snapshot: ErrorStateSnapshot = {
      netV: { ...this.netV },
      caps: new Map(this.state.caps),
      inds: new Map(this.state.inds),
      mosfetGates: new Map(
        [...this.state.mosfetGates].map(([key, value]) => [key, { ...value }]),
      ),
      batteries: new Map(
        [...this.state.batteries].map(([key, value]) => [key, { ...value }]),
      ),
      ne555s: new Map(
        [...this.state.ne555s].map(([key, value]) => [key, { ...value }]),
      ),
      relays: new Map(
        [...this.state.relays].map(([key, value]) => [key, { ...value }]),
      ),
      motors: new Map(
        [...this.state.motors].map(([key, value]) => [key, { ...value }]),
      ),
      servos: new Map(
        [...this.state.servos].map(([key, value]) => [key, { ...value }]),
      ),
      steppers: new Map(
        [...this.state.steppers].map(([key, value]) => [key, { ...value }]),
      ),
      hcsr04: new Map(
        [...this.state.hcsr04].map(([key, value]) => [key, { ...value }]),
      ),
      failureStress: new Map(this.state.failureStress),
      failures: new Map(
        [...this.state.failures].map(([key, value]) => [key, { ...value }]),
      ),
      icState: new Map(
        [...this.state.icState].map(([key, value]) => [key, { ...value }]),
      ),
      ptcs: new Map(
        [...this.state.ptcs].map(([key, value]) => [key, { ...value }]),
      ),
      thermalTemps: new Map(this.state.thermalTemps),
      thermalDevices: new Map(
        [...this.state.thermalDevices].map(([key, value]) => [
          key,
          { ...value, warnings: [...value.warnings] },
        ]),
      ),
    };
    // Trap-mode-only: the trap difference equation reads these histories, so
    // their coarse/refined disagreement is a real local-error term there. In
    // BE mode they are omitted entirely; comparing them would add new
    // rejection/step-factor inputs and break default-mode bit-identity.
    if (this._integrationMethod === "trap") {
      snapshot.capsI = new Map(this.state.capsI);
      snapshot.indsV = new Map(this.state.indsV);
    }
    return snapshot;
  }

  restoreState(snap: StateSnapshot): void {
    this.state.caps = new Map(snap.caps);
    this.state.capCurrents = new Map(snap.capCurrents ?? new Map());
    // Trap-mode snapshots always carry both histories (saveState pairs them
    // with the mode). A snapshot without them is a BE-mode or legacy one:
    // leave the live maps alone — nothing reads them until a mode switch,
    // and that switch re-anchors through _beNextStep first.
    if (snap.capsI) this.state.capsI = new Map(snap.capsI);
    if (snap.indsV) this.state.indsV = new Map(snap.indsV);
    // Restore the pending-BE flag exactly: a rolled-back trial consumed no
    // discontinuity, and the worker's coarse/refined replay must stamp the
    // same integration method in every leg.
    this._beNextStep = snap.integrationBeNextStep ?? this._beNextStep;
    this.state.inds = new Map(snap.inds);
    this.state.mosfetGates = new Map(
      [...(snap.mosfetGates ?? new Map())].map(([k, v]) => [k, { ...v }]),
    );
    this.state.batteries = new Map(
      [...(snap.batteries ?? new Map())].map(([k, v]) => [k, { ...v }]),
    );
    this.state.ne555s = new Map(snap.ne555s);
    this.state.relays = new Map([...(snap.relays ?? new Map())].map(([k, v]) => [k, { ...v }]));
    this.state.motors = new Map([...(snap.motors ?? new Map())].map(([k, v]) => [k, { ...v }]));   // W7.1
    this.state.servos = new Map([...(snap.servos ?? new Map())].map(([k, v]) => [k, { ...v }]));   // W7.2
    this.state.steppers = new Map([...(snap.steppers ?? new Map())].map(([k, v]) => [k, { ...v }])); // W7.2
    // W8.2 — deep-copy: ddram and lastData are arrays that must not share references.
    this.state.lcds = new Map([...(snap.lcds ?? new Map())].map(([k, v]) => [k, { ...v, ddram: [...v.ddram], lastData: [...v.lastData] }]));
    this.state.hcsr04 = new Map([...(snap.hcsr04 ?? new Map())].map(([k, v]) => [k, { ...v }])); // S18b
    this.state.failureStress = new Map(snap.failureStress ?? new Map());
    this.state.failures = new Map([...(snap.failures ?? new Map())].map(([k, v]) => [k, { ...v }]));
    this.state.icState = new Map([...(snap.icState ?? new Map())].map(([k, v]) => [k, { ...v }]));
    this.state.eeproms = new Map(
      [...(snap.eeproms ?? new Map())].map(([k, v]) => [k, {
        ...v,
        bytes: new Uint8Array(v.bytes),
      }]),
    );
    this.state.ptcs = new Map([...(snap.ptcs ?? new Map())].map(([k, v]) => [k, { ...v }]));
    this.state.thermalTemps = new Map(snap.thermalTemps ?? new Map());
    this.state.thermalDevices = new Map(
      [...(snap.thermalDevices ?? new Map())].map(([k, v]) => [k, { ...v, warnings: [...v.warnings] }]),
    );
    this.netV = { ...snap.netV };
    this.elementI = { ...snap.elementI };
    this.digitalState = { ...snap.digitalState };
    this._lastX = snap.lastSolution ? new Float64Array(snap.lastSolution) : null;
    this._displayState = cloneDisplayState(snap.displayState ?? {});
    if (snap.solverDiagnostics) {
      this.lastIters = snap.solverDiagnostics.lastIters;
      this.lastConverged = snap.solverDiagnostics.lastConverged;
      this.lastSolveUs = snap.solverDiagnostics.lastSolveUs;
      this.lastMatrixSize = snap.solverDiagnostics.lastMatrixSize;
      this.lastMatrixSingular = snap.solverDiagnostics.lastMatrixSingular;
      this.lastMatrixIllConditioned = snap.solverDiagnostics.lastMatrixIllConditioned;
      this.lastRelativeResidual = snap.solverDiagnostics.lastRelativeResidual;
    }
    this.simTime = snap.simTime;
    this._digitalDueCacheDirty = true;
  }

  getFailures(): Record<string, SimFailure> {
    return Object.fromEntries([...this.state.failures].map(([k, v]) => [k, { ...v }]));
  }

  /** Returns the set of component IDs whose PTC fuse is currently tripped. */
  getPtcTripped(): ReadonlySet<string> {
    const result = new Set<string>();
    for (const [id, st] of this.state.ptcs) {
      if (st.tripped) result.add(id);
    }
    return result;
  }

  /** Latest committed body/junction temperature for a thermally modelled component. */
  getPartTemperature(componentId: string): number | undefined {
    return this.state.thermalDevices.get(componentId)?.temperatureC
      ?? this.state.thermalTemps.get(componentId);
  }

  /** Exact package thermal state for structured telemetry and UI warnings. */
  getThermalState(componentId: string): ThermalRuntimeReadout | undefined {
    const state = this.state.thermalDevices.get(componentId);
    const comp = this._componentById.get(componentId);
    if (!state || !comp) return undefined;
    const profile = this._thermalProfile(comp);
    if (!profile || profile.id !== state.profileId) return undefined;
    return { ...state, warnings: [...state.warnings], label: profile.label };
  }

  resetFailures(): void {
    this.state.failures.clear();
    this.state.failureStress.clear();
    // PTC state is separate from the latched failure framework but still
    // controllable by the user via the "Clear failures" toolbar button —
    // a power-cycle (which the toolbar simulates) unlatches a tripped PTC.
    for (const ptc of this.state.ptcs.values()) {
      ptc.tripped = false;
      ptc.tripStress = 0;
      ptc.recoveryTime = 0;
    }

    // Failure state changes the electrical topology: a latched fuse/resistor
    // can move from effectively open back to its authored resistance, while a
    // reset PTC moves from its high-R trip state back to rNormal. Re-seed the
    // operating point at the current simulation time so public readings never
    // describe the pre-reset topology. This mirrors load()'s 1 ps seed without
    // advancing simTime or the MCU. If the seed is rejected, _solve keeps the
    // last trusted state internally and marks lastConverged=false; worker/UI
    // callers must then withhold those retained electrical values.
    if (this.circuit) {
      this._rebuildStaticStampBase();
      // A cleared failure re-opens/re-closes branches instantly; that is a
      // discontinuity for the trapezoidal history, so re-anchor with one BE
      // step (and keep this seed solve itself on BE).
      this._beNextStep = true;
      this._solve(1e-12);
      // The rebuild above re-captured the sparse base over the full pattern
      // discovered so far, which raises the backend's truncation floor above
      // the load-time watermark and would silently turn dcOperatingPoint's
      // pattern restore into a no-op. Re-capture at this boundary — mirroring
      // load()'s rebuild+seed+watermark sequence — so OP-ladder pattern
      // growth keeps being truncated relative to the current base.
      this._mnaLoadPatternWatermark = this.mna.patternCheckpoint?.();
    }
  }

  /**
   * Returns EEPROM byte arrays changed since the last call, then clears
   * their dirty flags. Empty object is the common no-op path.
   */
  takeEepromUpdates(): Record<string, Uint8Array> {
    const out: Record<string, Uint8Array> = {};
    for (const [compId, st] of this.state.eeproms) {
      if (st.dirty) {
        out[compId] = new Uint8Array(st.bytes);
        st.dirty = false;
      }
    }
    return out;
  }

  // ─── Circuit loading ───────────────────────────────────────────────────

  /**
   * Load a new circuit. Preserves element state for components whose ID
   * hasn't changed (ID match), then falls back to net-pair fingerprint
   * matching for components that were re-placed/renamed. Truly new
   * capacitors and inductors start with exactly zero stored energy.
   */
  load(circuit: SimCircuit): void {
    // A load boundary is a topology/parameter discontinuity: force the first
    // step (and the 1 ps seed solve below) onto backward Euler so trapezoidal
    // history never straddles an edit. Cleared after the first accepted step.
    this._beNextStep = true;

    // Snapshot the *previous* circuit and nets before overwriting them so
    // we can build the old net-pair → state index below.
    const oldCircuit = this.circuit;
    const oldNets = this.nets;

    this.circuit = circuit;
    this.nets = buildNets(circuit);
    this._buildMatrix();
    this._compileRuntimeMetadata(circuit);

    // Drop program-driven micro:bit pin drives for boards deleted from the
    // circuit; surviving boards keep theirs (their embedded sims are still
    // running and won't re-push unchanged pins after a mere topology edit).
    for (const id of [...this.microbitDrives.keys()]) {
      if (!circuit.components.some((c) => c.id === id)) {
        this.microbitDrives.delete(id);
      }
    }
    // S18b — same pruning for the hcsr04 cycle-event MCU boot baseline (see
    // _mcuBootSimTime declaration); harmless to leave stale entries around,
    // but freshly-constructed cores below always overwrite them anyway.
    for (const id of [...this._mcuBootSimTime.keys()]) {
      if (!circuit.components.some((c) => c.id === id)) {
        this._mcuBootSimTime.delete(id);
      }
    }

    const prevCaps = this.state.caps;
    const prevCapsI = this.state.capsI;
    const prevIndsV = this.state.indsV;
    const prevInds = this.state.inds;
    const prevMosfetGates = this.state.mosfetGates;
    const prevBatteries = this.state.batteries;
    const prevNe555s = this.state.ne555s;
    const prevRelays = this.state.relays;
    const prevMotors = this.state.motors;   // W7.1
    const prevServos = this.state.servos;   // W7.2
    const prevSteppers = this.state.steppers; // W7.2
    const prevLcds = this.state.lcds;         // W8.2
    const prevHcsr04 = this.state.hcsr04;     // S18b
    const prevFailureStress = this.state.failureStress;
    const prevFailures = this.state.failures;
    const prevIcState = this.state.icState;
    const prevArduinos = this.state.arduinos;
    const prevEeproms = this.state.eeproms;
    const prevPtcs = this.state.ptcs;
    const prevThermalTemps = this.state.thermalTemps;
    const prevThermalDevices = this.state.thermalDevices;
    const oldKindById = new Map((oldCircuit?.components ?? []).map((c) => [c.id, c.kind]));
    this.state = {
      caps: new Map(),
      capCurrents: new Map(),
      capsI: new Map(),
      indsV: new Map(),
      inds: new Map(),
      mosfetGates: new Map(),
      batteries: new Map(),
      ne555s: new Map(),
      relays: new Map(),
      motors: new Map(),   // W7.1
      servos: new Map(),   // W7.2
      steppers: new Map(), // W7.2
      lcds: new Map(),     // W8.2
      hcsr04: new Map(),   // S18b
      failureStress: new Map(),
      failures: new Map(),
      icState: new Map(),
      arduinos: new Map(),
      eeproms: new Map(),
      ptcs: new Map(),
      thermalTemps: new Map(),
      thermalDevices: new Map(),
    };

    // Build old net-pair → value indexes for fingerprint-based carry.
    const oldCapByNetPair = new Map<string, number>();
    const oldCapIByNetPair = new Map<string, number>();
    const oldIndByNetPair = new Map<string, number>();
    const oldIndVByNetPair = new Map<string, number>();

    if (oldCircuit) {
      for (const c of oldCircuit.components) {
        if (c.pins.length < 2) continue;
        const n0 = oldNets.find((n) =>
          n.pins.some(([cc, pp]) => cc === c.id && pp === c.pins[0].id),
        );
        const n1 = oldNets.find((n) =>
          n.pins.some(([cc, pp]) => cc === c.id && pp === c.pins[1].id),
        );
        if (!n0 || !n1) continue;

        if (c.kind === "capacitor") {
          const v = prevCaps.get(c.id);
          if (v !== undefined) {
            const { key, flipped } = netPairKey(n0, n1, c.id);
            oldCapByNetPair.set(key, flipped ? -v : v);
            // Trap history current is pin0→pin1, so it flips with orientation
            // exactly like the winding current carried for inductors.
            const iHist = prevCapsI.get(c.id);
            if (iHist !== undefined) {
              oldCapIByNetPair.set(key, flipped ? -iHist : iHist);
            }
          }
        }
        if (c.kind === "inductor") {
          const i = prevInds.get(c.id);
          if (i !== undefined) {
            const { key, flipped } = netPairKey(n0, n1, c.id);
            oldIndByNetPair.set(key, flipped ? -i : i);
            // Trap history voltage is pin0−pin1 and flips the same way.
            const vHist = prevIndsV.get(c.id);
            if (vHist !== undefined) {
              oldIndVByNetPair.set(key, flipped ? -vHist : vHist);
            }
          }
        }
      }
    }

    // State carry: ID match → net-pair match → exact zero state.
    for (const c of circuit.components) {
      if (c.kind === "capacitor" && c.pins.length >= 2) {
        // Recomputed from the first accepted solve. Do not carry this derived
        // value across topology edits because terminal orientation may change.
        this.state.capCurrents.set(c.id, 0);
        if (prevCaps.has(c.id)) {
          this.state.caps.set(c.id, prevCaps.get(c.id)!);
          // Same-ID carry keeps the trap history current alongside the
          // internal voltage (invariant #4: element state survives edits).
          this.state.capsI.set(c.id, prevCapsI.get(c.id) ?? 0);
        } else {
          const n0 = this.nets.find((n) =>
            n.pins.some(([cc, pp]) => cc === c.id && pp === c.pins[0].id),
          );
          const n1 = this.nets.find((n) =>
            n.pins.some(([cc, pp]) => cc === c.id && pp === c.pins[1].id),
          );
          let v = 0;
          let iHist = 0;
          if (n0 && n1) {
            const { key, flipped } = netPairKey(n0, n1, c.id);
            const carried = oldCapByNetPair.get(key);
            if (carried !== undefined) v = flipped ? -carried : carried;
            const carriedI = oldCapIByNetPair.get(key);
            if (carriedI !== undefined) iHist = flipped ? -carriedI : carriedI;
          }
          this.state.caps.set(c.id, v);
          this.state.capsI.set(c.id, iHist);
        }
      }

      if (c.kind === "inductor" && c.pins.length >= 2) {
        if (prevInds.has(c.id)) {
          this.state.inds.set(c.id, prevInds.get(c.id)!);
          // Same-ID carry for the trap history voltage, mirroring capsI.
          this.state.indsV.set(c.id, prevIndsV.get(c.id) ?? 0);
        } else {
          const n0 = this.nets.find((n) =>
            n.pins.some(([cc, pp]) => cc === c.id && pp === c.pins[0].id),
          );
          const n1 = this.nets.find((n) =>
            n.pins.some(([cc, pp]) => cc === c.id && pp === c.pins[1].id),
          );
          let i = 0;
          let vHist = 0;
          if (n0 && n1) {
            const { key, flipped } = netPairKey(n0, n1, c.id);
            const carried = oldIndByNetPair.get(key);
            if (carried !== undefined) i = flipped ? -carried : carried;
            const carriedV = oldIndVByNetPair.get(key);
            if (carriedV !== undefined) vHist = flipped ? -carriedV : carriedV;
          }
          this.state.inds.set(c.id, i);
          this.state.indsV.set(c.id, vHist);
        }
      }

      // Wave A6 — coupled_inductor: two winding currents (and their trap
      // terminal-voltage histories) under composite keys id:1/id:2. Carry is
      // same-ID ONLY: the net-pair fingerprint index above is built for one
      // two-terminal branch keyed on pins[0]/pins[1], while a multi-winding
      // part has two independent branches whose pairing and dot polarity
      // across an id change are ambiguous — carrying a guess would inject
      // phantom flux into the wrong winding, worse than a clean zero start.
      if (c.kind === "coupled_inductor" && c.pins.length >= 4) {
        for (const winding of ["1", "2"]) {
          const key = `${c.id}:${winding}`;
          this.state.inds.set(key, prevInds.get(key) ?? 0);
          this.state.indsV.set(key, prevIndsV.get(key) ?? 0);
        }
      }

      // Wave A6 — crystal: motional Ls/Cs branch state plus the C0 shunt,
      // under composite keys id:ls / id:cs / id:c0. Same-ID-only carry for
      // the same multi-branch reason as coupled_inductor: the fingerprint
      // index cannot describe a branch that ends on an internal solver node.
      if (c.kind === "crystal" && c.pins.length >= 2) {
        for (const capKey of [`${c.id}:cs`, `${c.id}:c0`]) {
          this.state.caps.set(capKey, prevCaps.get(capKey) ?? 0);
          this.state.capsI.set(capKey, prevCapsI.get(capKey) ?? 0);
          this.state.capCurrents.set(capKey, 0);
        }
        const lsKey = `${c.id}:ls`;
        this.state.inds.set(lsKey, prevInds.get(lsKey) ?? 0);
        this.state.indsV.set(lsKey, prevIndsV.get(lsKey) ?? 0);
      }

      if ((c.kind === "nmos" || c.kind === "pmos") && c.pins.length >= 3) {
        const previous = oldKindById.get(c.id) === c.kind
          ? prevMosfetGates.get(c.id)
          : undefined;
        this.state.mosfetGates.set(
          c.id,
          previous ? { ...previous } : { vgs: 0, vgd: 0 },
        );
      }

      if (c.kind === "battery_pack") {
        const authoredCharge = Math.max(0, Math.min(1, Number(c.params.charge ?? 1)));
        const catalogUid = c.catalogUid ?? null;
        const previous = prevBatteries.get(c.id);
        this.state.batteries.set(
          c.id,
          previous
            && previous.authoredCharge === authoredCharge
            && previous.catalogUid === catalogUid
            ? { ...previous }
            : { ...createBatteryState(authoredCharge), authoredCharge, catalogUid },
        );
      }

      const thermalProfile = this._thermalProfile(c);
      if (thermalProfile && c.catalogUid) {
        const previous = prevThermalDevices.get(c.id);
        if (previous?.catalogUid === c.catalogUid && previous.profileId === thermalProfile.id) {
          this.state.thermalDevices.set(c.id, { ...previous, warnings: [...previous.warnings] });
        } else {
          const ambientC = this._ambientTempC();
          const initial = stepThermalDevice(
            thermalProfile,
            createThermalState(ambientC),
            { ambientTemperatureC: ambientC, dissipatedPowerW: 0, dtSeconds: 0 },
          );
          this.state.thermalDevices.set(c.id, {
            ...initial.state,
            catalogUid: c.catalogUid,
            profileId: thermalProfile.id,
            dissipatedPowerW: 0,
            targetTemperatureC: initial.targetTemperatureC,
            allowedPowerW: initial.derating.allowedPowerW,
            withinContinuousLimits: initial.withinContinuousLimits,
            warnings: [...initial.warnings],
          });
        }
      }

      if (c.kind === "ne555") {
        this.state.ne555s.set(
          c.id,
          prevNe555s.get(c.id) ?? { outHigh: true },
        );
      }
      // W6.1 — relay state carries across load() by component ID, mirroring ne555s.
      // A new relay starts de-energised with zero coil current.
      if (c.kind === "relay") {
        this.state.relays.set(
          c.id,
          prevRelays.get(c.id) ?? { iCoil: 0, energized: false },
        );
      }
      // W7.1 — dc_motor state carries across load() by component ID.
      // A new motor starts at rest: zero winding current, zero rotor speed.
      if (c.kind === "dc_motor") {
        this.state.motors.set(
          c.id,
          prevMotors.get(c.id) ?? { iWinding: 0, omega: 0 },
        );
      }
      // W7.2 — servo state carries across load() by component ID.
      // A new servo starts at neutral (90°) with no prior edge; angle persists
      // so the shaft does not snap on circuit edits when sim is running.
      if (c.kind === "servo") {
        const previous = prevServos.get(c.id);
        const fallback = defaultServoState();
        this.state.servos.set(
          c.id,
          previous
            ? {
                ...fallback,
                ...previous,
                targetAngle: Number.isFinite(previous.targetAngle)
                  ? previous.targetAngle
                  : previous.angle,
                velocity: Number.isFinite(previous.velocity) ? previous.velocity : 0,
                moving: previous.moving ?? false,
                powered: previous.powered ?? false,
                supplyVoltage: Number.isFinite(previous.supplyVoltage)
                  ? previous.supplyVoltage
                  : 0,
                loadResistance: Number.isFinite(previous.loadResistance)
                  ? previous.loadResistance
                  : Number.NaN,
              }
            : fallback,
        );
      }
      // W7.2 — stepper state carries across load() by component ID.
      // A new stepper starts at position 0, unenergised (phase -1).
      if (c.kind === "stepper") {
        this.state.steppers.set(
          c.id,
          prevSteppers.get(c.id) ?? { iA: 0, iB: 0, phase: -1, lastKnownPhase: -1, position: 0 },
        );
      }
      // W8.2 — hd44780 state carries across load() by component ID.
      // A new LCD starts with blank DDRAM (0x20) and all registers at power-on defaults.
      // Existing LCDs keep their DDRAM contents so circuit edits don't clear the display.
      if (c.kind === "hd44780") {
        const prev = prevLcds.get(c.id);
        this.state.lcds.set(
          c.id,
          // Deep-copy the previous state to avoid shared array references across load().
          prev ? { ...prev, ddram: [...prev.ddram], lastData: [...prev.lastData] } : defaultHd44780State(),
        );
      }
      // S18b — hcsr04 state carries across load() by component ID, so an
      // in-flight echo survives a live distance-drag reload (params are only
      // consulted when a FRESH echo is scheduled on the next TRIG fall, never
      // retroactively applied to one already pending/active).
      if (c.kind === "hcsr04") {
        this.state.hcsr04.set(c.id, prevHcsr04.get(c.id) ?? defaultHcsr04State());
      }
      // PTC state carries across load() by component ID, same as ne555s.
      // A new PTC starts un-tripped; an existing one keeps whatever state
      // it had (may still be in trip or recovery).
      if (c.kind === "ptc_fuse") {
        this.state.ptcs.set(
          c.id,
          prevPtcs.get(c.id) ?? { tripped: false, tripStress: 0, recoveryTime: 0 },
        );
      }
      if (c.kind === "thermistor") {
        this.state.thermalTemps.set(
          c.id,
          prevThermalTemps.get(c.id) ?? this._envTempC(c),
        );
      }
      if (oldKindById.get(c.id) === c.kind) {
        for (const [key, failure] of prevFailures) {
          if (failure.componentId === c.id) this.state.failures.set(key, { ...failure });
        }
        for (const [key, stress] of prevFailureStress) {
          if (failureKeyComponentId(key) === c.id) this.state.failureStress.set(key, stress);
        }
      }
      if (IC_STATE_KINDS.has(c.kind)) {
        // Kind-gated like the failures/mosfetGates carries above: a same-ID
        // kind swap must not read another kind's slots as its own (an scr's
        // committed `on` latch would seed a replacement triac's regime); the
        // fresh kind starts from its own defaults instead.
        const carried = oldKindById.get(c.id) === c.kind
          ? prevIcState.get(c.id)
          : undefined;
        this.state.icState.set(c.id, carried ?? defaultIcState(c.kind));
      }
      // W5.1 — bench_psu iLimit: initialise icState only when limiting is active.
      // When iLimit is absent or <= 0 the bench_psu behaves as an ideal voltage
      // source and no icState is needed (preserves byte-identical behaviour for
      // existing saves that never set iLimit).
      if (c.kind === "bench_psu" && Number(c.params.iLimit ?? 0) > 0) {
        this.state.icState.set(c.id, prevIcState.get(c.id) ?? { reg: 0 });
      }
      // W5.2 — dcdc_converter always needs committed regime/current state.
      // Start OFF: the nonlinear solve promotes it to REG only after the input
      // is proven inside the declared operating range.
      if (c.kind === "dcdc_converter") {
        this.state.icState.set(c.id, prevIcState.get(c.id) ?? { reg: 3, iin: 0 });
      }
      if (isArduinoBoardKind(c.kind)) {
        const hexText = String(c.params.hex ?? "");
        const existing = prevArduinos.get(c.id);
        if (existing) {
          // Keep the running MCU; if the hex changed, reset with new program.
          const prevHex = (oldCircuit?.components.find((oc) => oc.id === c.id)?.params.hex) ?? "";
          if (hexText !== String(prevHex)) {
            // New hex uploaded — cold-reset this MCU (empty hex keeps the old core).
            const rebuilt = hexText ? mcuFactory(c.kind, { hex: hexText }) : existing;
            if (rebuilt) {
              this.state.arduinos.set(c.id, rebuilt);
              // S18b — a genuinely fresh core boots its cycle counter at 0;
              // record the engine simTime this happened at so hcsr04's TRIG
              // cycle-event path can convert an absolute PinEvent.cycle into
              // absolute engine simTime (this.simTime hasn't advanced yet at
              // this point in load() — see _mcuBootSimTime declaration).
              if (hexText) this._mcuBootSimTime.set(c.id, this.simTime);
            }
          } else {
            this.state.arduinos.set(c.id, existing);
          }
        } else if (hexText) {
          const built = mcuFactory(c.kind, { hex: hexText });
          if (built) {
            this.state.arduinos.set(c.id, built);
            this._mcuBootSimTime.set(c.id, this.simTime);
          }
        }
      }
      if (c.kind === "raspberry_pi_pico") {
        // MicroPython boards carry their program in `params.script`. A changed
        // script re-instantiates the core (fresh boot + re-fed program), mirroring
        // the Arduino hex-change path. mcuFactory returns null until the worker has
        // registered the firmware image — the board stays inert until then.
        const script = String(c.params.script ?? "");
        const existing = prevArduinos.get(c.id);
        if (existing) {
          const prevScript = (oldCircuit?.components.find((oc) => oc.id === c.id)?.params.script) ?? "";
          if (script !== String(prevScript)) {
            const rebuiltPico = mcuFactory(c.kind, { script });
            this.state.arduinos.set(c.id, rebuiltPico ?? existing);
            if (rebuiltPico) this._mcuBootSimTime.set(c.id, this.simTime); // S18b — see comment above
          } else {
            this.state.arduinos.set(c.id, existing);
          }
        } else {
          const built = mcuFactory(c.kind, { script });
          if (built) {
            this.state.arduinos.set(c.id, built);
            this._mcuBootSimTime.set(c.id, this.simTime); // S18b — see comment above
          }
        }
      }
      if (c.kind === "28c16" || c.kind === "28c256") {
        const addrBits = c.kind === "28c16" ? 11 : 15;
        const size = 1 << addrBits;
        const contentsB64 = String(c.params.contents ?? "");
        const hash = `${size}:${contentsB64.length}:${contentsB64}`;
        const prev = prevEeproms.get(c.id);
        const sdpEnabled = c.kind === "28c256" && Number(c.params.sdpBypass ?? 0) === 0;
        if (prev && prev.contentsHash === hash && prev.bytes.length === size) {
          prev.sdpEnabled = sdpEnabled;
          this.state.eeproms.set(c.id, prev);
        } else {
          const bytes = new Uint8Array(size).fill(0xff);
          if (contentsB64) {
            try {
              const raw = Uint8Array.from(atob(contentsB64), (ch) => ch.charCodeAt(0));
              bytes.set(raw.subarray(0, size));
            } catch { /* invalid base64 -> keep 0xff fill */ }
          }
          this.state.eeproms.set(c.id, {
            bytes,
            contentsHash: hash,
            lastWeHigh: true,
            lastCeHigh: true,
            writeCompletesAt: null,
            pendingAddr: null,
            pendingByte: null,
            pollReads: 0,
            sdpEnabled,
            sdpUnlocked: false,
            sdpStep: 0,
            dirty: false,
          });
        }
      }
    }

    this.digitalState = {};
    this.elementI = {};
    this.netV = { gnd: 0 };

    this._rebuildJunctionCache(circuit);
    this._rebuildStaticStampBase();

    // Seed netV with an initial OP using a 1 ps companion interval. This is an
    // algebraic initialization at the current simTime, not elapsed physical
    // time: preserve the exact capacitor/inductor initial conditions that were
    // stamped. Active device state may still resolve from the powered OP (which
    // is how compact oscillators start without energizing unrelated passives).
    const initializedCaps = new Map(this.state.caps);
    const initializedInds = new Map(this.state.inds);
    // The 1 ps seed's post-solve update would replace the carried trap
    // histories with meaningless 1 ps derivative estimates (i = C·dv/1e-12).
    // Preserve them alongside caps/inds; the first real step is forced BE by
    // _beNextStep anyway, and its accepted update refreshes both maps.
    const initializedCapsI = new Map(this.state.capsI);
    const initializedIndsV = new Map(this.state.indsV);
    this._solve(1e-12);
    // Wave A3 rescue, gated on the single lastConverged check. Limiter-free
    // circuits take a bit-identical path through this seed; the one
    // deliberate exception is the pnjlim acceptance guard inside the Newton
    // loop itself (newton.ts limitedThisIteration), which keeps iterating
    // where the pre-A3 delta-x-only test published a false-converged frame
    // with a junction parked volts into forward bias — a correctness fix
    // locked by dc-homotopy-pathological, and the only ordinary-path
    // numeric change Wave A3 ships. Running the rescue before the map
    // restores below keeps the authored initial conditions exact whether or
    // not the ladder had to touch them.
    if (!this.lastConverged) this._rescueSeedOperatingPoint();
    this.state.caps = initializedCaps;
    this.state.inds = initializedInds;
    this.state.capsI = initializedCapsI;
    this.state.indsV = initializedIndsV;
    // Sparse-backend pattern watermark at the load boundary. Any engine that
    // loads this circuit walks the identical seed/rescue code path and
    // discovers the identical slot sequence, so this watermark is the one
    // pattern state a snapshot-restored twin is guaranteed to share. The
    // dcOperatingPoint ladder truncates back to it (see there for why).
    this._mnaLoadPatternWatermark = this.mna.patternCheckpoint?.();
  }

  /**
   * Pre-compute `vCrit` (and the per-junction `n·Vt`) for every PN-junction
   * element, so pnjlim doesn't take a log per stamp call. Rebuilt on every
   * `load()`/`coldLoad()` since params can change at the load boundary but
   * not within a step.
   */
  private _rebuildJunctionCache(circuit: SimCircuit): void {
    this._bjtCache.clear();
    this._vPrevBJT.clear();
    const tempC = this._ambientTempC();
    const vt = thermalVoltage(tempC);

    for (const c of circuit.components) {
      if (c.kind === "bjt_npn" || c.kind === "bjt_pnp") {
        const IsT = saturationCurrentAtTemperature(
          Number(c.params.Is ?? 1e-16),
          tempC,
          Number(c.params.nF ?? 1),
        );
        const nF = Number(c.params.nF ?? 1);
        const nR = Number(c.params.nR ?? 1);
        const VtNF = nF * vt;
        const VtNR = nR * vt;
        this._bjtCache.set(c.id, {
          IsT, VtNF, VtNR,
          vCritBE: vCritFor(IsT, VtNF),
          vCritBC: vCritFor(IsT, VtNR),
        });
      }
    }
  }

  /**
   * Cold-load: zero all element state then call load().
   * Used by the "Cold reset" button — caps/inductors start from exact zero
   * stored voltage/current before the initial operating-point solve.
   */
  coldLoad(circuit: SimCircuit): void {
    this.state = {
      caps: new Map(),
      capCurrents: new Map(),
      capsI: new Map(),
      indsV: new Map(),
      inds: new Map(),
      mosfetGates: new Map(),
      batteries: new Map(),
      ne555s: new Map(),
      relays: new Map(),
      motors: new Map(),   // W7.1
      servos: new Map(),   // W7.2
      steppers: new Map(), // W7.2
      lcds: new Map(),     // W8.2
      hcsr04: new Map(),   // S18b
      failureStress: new Map(),
      failures: new Map(),
      icState: new Map(),
      arduinos: new Map(),
      eeproms: new Map(),
      ptcs: new Map(),
      thermalTemps: new Map(),
      thermalDevices: new Map(),
    };
    this.simTime = 0;
    this.netV = { gnd: 0 };
    this.elementI = {};
    this.digitalState = {};
    this.load(circuit);
  }

  // ─── DC operating point (Wave A3) ──────────────────────────────────────

  /**
   * Robust DC operating point: true-OP stamping (capacitors open, inductors
   * shorted through their winding resistance) plus a homotopy rescue ladder
   * — direct solve at the committed regimes, gmin stepping, source stepping,
   * then pseudo-transient relaxation. Each stage warm-starts from the
   * previous iterate because a failed _solve is a rejected trial that keeps
   * the last trusted netV.
   *
   * Never advances simTime, MCU firmware, digital event time, or any other
   * physical clock: every dc solve uses the same 1 ps companion interval as
   * load()'s seed (stage d grows its BE pseudo-interval but pins waveform
   * sampling to the same instant — see _waveformSampleTime), _solve never
   * touches MCU cores, and the h-scaled physical integrators (thermal,
   * failure stress, battery charge, mechanics) hold at their entry values
   * for the whole ladder via _dcPhysicalTimeHold.
   *
   * On success the point is committed like an accepted solve, with the
   * dynamic-element state normalised to its DC meaning (see
   * _commitDcOperatingPointState). On total failure the engine is restored
   * to the state it had before the call.
   */
  dcOperatingPoint(options?: { maxOuterRegimeIters?: number }): DcOperatingPointResult {
    const counters: DcOpCounters = { iterations: 0, regimeIterations: 0 };
    if (!this.circuit) {
      return {
        converged: false,
        method: "direct",
        iterations: 0,
        regimeIterations: 0,
        netV: { ...this.netV },
      };
    }
    // Non-finite budgets must not pass the sanitizer: Math.round(NaN) is NaN,
    // `outer < NaN` never runs a settle solve, and the stability screen would
    // then bless the stale pre-call frame as a "converged" operating point;
    // Infinity would let a never-stabilising regime picture hang the settle
    // loop. Anything unusable falls back to the default budget.
    const requestedOuterIters = options?.maxOuterRegimeIters;
    const maxOuterRegimeIters =
      requestedOuterIters !== undefined && Number.isFinite(requestedOuterIters)
        ? Math.max(1, Math.round(requestedOuterIters))
        : 8;
    const entry = this.saveState();
    // Sparse-backend pattern watermark at OP entry. The failure contract is
    // "the caller keeps the exact engine it had", and on the sparse backend
    // that includes the discovered stamp pattern: membership fixes the fill
    // ordering and therefore the exact rounding of every later
    // factorization, so truncating any deeper (or shallower) than this on
    // failure would make the very next step drift by ULPs from a no-OP run.
    const entryPatternCheckpoint = this.mna.patternCheckpoint?.();
    let method: DcOperatingPointResult["method"] | null = null;
    try {
      this._dcSolveMode = true;
      this._newtonMaxIterOverride = DC_OP_NEWTON_MAX_ITER;
      this._dcPhysicalTimeHold = true;
      // Every stage acceptance passes the stability screen: a converged
      // point that is a saddle of the parasitic dynamics is not a physical
      // resting state, so it is either escaped along its growing eigenmode
      // (accepted within the same stage) or handed to the next stage.
      if (this._dcRegimeSettle(maxOuterRegimeIters, counters)
        && this._dcAcceptStableOrEscape(maxOuterRegimeIters, counters)) {
        method = "direct";
      }
      // Each homotopy stage ends at gmin 0 / scale 1, so a stage success is
      // already a true-system solution; the settle pass afterwards only
      // re-solves until the regime/latch picture it committed stops moving.
      if (method === null
        && this._homotopyGminLadder(counters)
        && this._dcRegimeSettle(maxOuterRegimeIters, counters)
        && this._dcAcceptStableOrEscape(maxOuterRegimeIters, counters)) {
        method = "gmin";
      }
      if (method === null
        && this._homotopySourceLadder(counters)
        && this._dcRegimeSettle(maxOuterRegimeIters, counters)
        && this._dcAcceptStableOrEscape(maxOuterRegimeIters, counters)) {
        method = "source";
      }
      if (method === null) {
        // Stage d abandons dc stamping: physical BE companions relax the
        // system along a pseudo-trajectory until it parks at a fixed point.
        this._dcSolveMode = false;
        this._homotopyGminG = 0;
        this._homotopySourceScale = 1;
        if (this._dcPseudoTransient(counters)
          && this._dcAcceptStableOrEscape(maxOuterRegimeIters, counters)) {
          method = "pseudo-transient";
        }
      }
    } catch (error) {
      // A throw mid-ladder (a poisoned stamp, a backend fault) is a total
      // failure that skips the method===null branch below, so it must do the
      // same restore here: without it the caller would keep the engine
      // abandoned at a homotopy anchor (e.g. the source ladder's scale-0
      // commit with every rail at ~0 V). The finally clause still resets the
      // stamping controls after this block.
      this.restoreState(entry);
      if (entryPatternCheckpoint !== undefined) {
        this.mna.restorePatternCheckpoint?.(entryPatternCheckpoint);
      }
      this.mna.invalidateFactorization?.();
      throw error;
    } finally {
      // Reentrant-safe: the stamping controls must never leak into ordinary
      // stepping, even if a stamp or post-solve update throws mid-ladder.
      this._dcSolveMode = false;
      this._homotopyGminG = 0;
      this._homotopySourceScale = 1;
      this._newtonMaxIterOverride = null;
      this._dcPhysicalTimeHold = false;
    }
    if (method === null) {
      // Total failure is diagnostic-only: the caller keeps the exact engine
      // it had, including the pre-call solver diagnostics in the snapshot.
      // (Monotonic work telemetry — electricalSolveCount and the backend
      // factorization counters — intentionally keeps the ladder's work
      // visible; callers read deltas, not absolutes.) The pattern restore
      // uses the ENTRY checkpoint, not the load watermark: the failure
      // contract is bitwise equivalence with an engine that never ran the
      // ladder, and that engine still carries every slot ordinary stepping
      // discovered since load(). The factorization invalidation covers what
      // the checkpoint cannot: when the ladder grew no new slots the
      // truncation no-ops, but the backend's stored pivot order still
      // describes a ladder system — replaying it on the next ordinary step
      // would eliminate in a different order than the no-ladder engine and
      // drift by ULPs (verified on the relay-pulse repro).
      this.restoreState(entry);
      if (entryPatternCheckpoint !== undefined) {
        this.mna.restorePatternCheckpoint?.(entryPatternCheckpoint);
      }
      this.mna.invalidateFactorization?.();
      return {
        converged: false,
        method: "pseudo-transient",
        iterations: counters.iterations,
        regimeIterations: counters.regimeIterations,
        netV: { ...this.netV },
      };
    }
    // Success: truncate the sparse backend's discovered pattern to the
    // load-time watermark. The ladder (and the transient history before it)
    // stamp positions a snapshot-restored twin engine never sees, and
    // pattern membership fixes the fill ordering and therefore the exact
    // rounding of every later factorization. The load watermark — not the
    // OP entry watermark — is the one pattern state a twin reconstructs
    // exactly (identical load code path), so truncating to it keeps post-OP
    // stepping bitwise equal to a fresh engine handed the same state.
    if (this._mnaLoadPatternWatermark !== undefined) {
      this.mna.restorePatternCheckpoint?.(this._mnaLoadPatternWatermark);
    }
    // If a mid-run static-base rebuild raised the backend's truncation floor
    // above the watermark, the call above was a silent no-op; falling back
    // to the entry checkpoint still strips the ladder-only pattern growth
    // (twin equality is unattainable after such a rebuild, but OP calls must
    // never permanently enlarge the pattern either way). After a successful
    // watermark truncation this second call is a no-op by construction.
    if (entryPatternCheckpoint !== undefined) {
      this.mna.restorePatternCheckpoint?.(entryPatternCheckpoint);
    }
    this._commitDcOperatingPointState(method !== "pseudo-transient");
    return {
      converged: true,
      method,
      iterations: counters.iterations,
      regimeIterations: counters.regimeIterations,
      netV: { ...this.netV },
    };
  }

  /**
   * Stage a — direct dc solve at the committed regimes. Regime and latch
   * state machines (op-amp saturation, regulator CC/dropout, bench_psu CC,
   * 555 SR, sequential logic) commit post-solve, so one solve can invalidate
   * the active set it was stamped with; re-solve until the committed picture
   * stops changing. An astable latch has no DC fixed point at all, so
   * hitting the cap still accepts the last converged solve (its regimes are
   * self-consistent for one of the two alternating phases) instead of
   * failing the stage.
   */
  private _dcRegimeSettle(maxOuterRegimeIters: number, counters: DcOpCounters): boolean {
    this._homotopyGminG = 0;
    this._homotopySourceScale = 1;
    for (let outer = 0; outer < maxOuterRegimeIters; outer++) {
      const before = this._dcCommittedRegimeSignature();
      this._solve(DC_OP_COMPANION_H);
      counters.iterations += this.lastIters;
      counters.regimeIterations += 1;
      if (!this.lastConverged) return false;
      if (this._dcCommittedRegimeSignature() === before) return true;
    }
    return true;
  }

  /**
   * Fingerprint of the committed regime/latch state consulted by _stampAll.
   * Continuous icState slots (branch currents, settled pole voltages) rest
   * across repeated 1 ps solves at a fixed instant; quantising to 1e-9 keeps
   * the fingerprint stable there while any genuine discrete transition
   * (regime enums, latch bits, counters) moves by >= 1. One deliberate
   * exception: a mid-slew op-amp pole moves ~slewRate * 1 ps per solve —
   * hundreds of quanta — so circuits carrying such a pole exhaust
   * maxOuterRegimeIters and take _dcRegimeSettle's accept-last-converged
   * exit. That is the documented DC_OP_COMPANION_H hold semantics (the OP is
   * the resting point GIVEN the carried device state, poles included), paid
   * for as a bounded settle budget rather than a wrong answer.
   */
  private _dcCommittedRegimeSignature(): string {
    const q = (value: number): string =>
      Number.isFinite(value) ? String(Math.round(value * 1e9)) : String(value);
    const parts: string[] = [];
    for (const key of Object.keys(this.digitalState).sort()) {
      parts.push(key, q(this.digitalState[key] ?? 0));
    }
    for (const id of [...this.state.ne555s.keys()].sort()) {
      parts.push(id, this.state.ne555s.get(id)?.outHigh ? "1" : "0");
    }
    for (const id of [...this.state.relays.keys()].sort()) {
      parts.push(id, this.state.relays.get(id)?.energized ? "1" : "0");
    }
    for (const id of [...this.state.icState.keys()].sort()) {
      parts.push(id);
      const slots = this.state.icState.get(id) ?? {};
      for (const slot of Object.keys(slots).sort()) {
        parts.push(slot, q(slots[slot] ?? 0));
      }
    }
    return parts.join("\0");
  }

  /** One homotopy rung: a converged _solve at the given overlay strength. */
  private _homotopyRungConverged(
    gminG: number,
    sourceScale: number,
    counters: DcOpCounters,
  ): boolean {
    this._homotopyGminG = gminG;
    this._homotopySourceScale = sourceScale;
    // Rung voltages are physically meaningless waypoints of the overlaid
    // system, so sequential logic must not observe them: a 1e-2 S gmin rung
    // sags every driven-HIGH line toward ground and the next rung releases
    // it — a phantom clock edge that would advance counters/latch bits with
    // simTime frozen. Digital state holds across every rung (this is the
    // single chokepoint for gmin, source, bisect, and rescue rungs); the
    // settle pass after ladder success — or the seed re-solve in the rescue
    // path — reconciles it against true-system voltages exactly the way a
    // directly converged solve would have.
    const priorDigitalHold = this._dcDigitalHold;
    this._dcDigitalHold = true;
    try {
      this._solve(DC_OP_COMPANION_H);
    } finally {
      this._dcDigitalHold = priorDigitalHold;
    }
    counters.iterations += this.lastIters;
    // A failed rung is a rejected trial: netV keeps the last successful
    // rung, which is exactly the warm start the retreat/bisect path wants.
    return this.lastConverged;
  }

  /**
   * Stage b — gmin stepping. A strong every-node-to-ground conductance makes
   * any circuit nearly linear; walking it down a decade at a time drags the
   * iterate along a continuous solution path to the unloaded system. On a
   * rung failure, retreat to the half-decade point between the last success
   * and the failure (geometric mean), then retry the failed rung once; a
   * second failure fails the stage.
   */
  private _homotopyGminLadder(counters: DcOpCounters): boolean {
    // Virtual predecessor one decade above the first rung so a first-rung
    // failure still has a well-defined half-decade retreat.
    let lastSuccessG = 1e-1;
    for (const g of DC_OP_GMIN_LADDER) {
      if (!this._homotopyRungConverged(g, 1, counters)) {
        // The terminal 0 rung has no geometric midpoint; bisect as if it
        // were the next decade down, preserving the half-decade spacing.
        const bisectG = Math.sqrt(lastSuccessG * (g > 0 ? g : lastSuccessG * 0.1));
        if (!this._homotopyRungConverged(bisectG, 1, counters)) return false;
        if (!this._homotopyRungConverged(g, 1, counters)) return false;
      }
      if (g > 0) lastSuccessG = g;
    }
    return true;
  }

  /**
   * Stage c — source stepping under a pinned mild gmin load. Anchors at
   * scale 0 (every independent source dead — the most docile system there
   * is), then ramps all drives together in adaptive increments: halve on
   * failure, double on success, floored at 1/1024 of full drive.
   */
  private _homotopySourceLadder(counters: DcOpCounters): boolean {
    if (!this._homotopyRungConverged(DC_OP_SOURCE_STEP_GMIN_G, 0, counters)) return false;
    let scale = 0;
    let increment = 0.1;
    while (scale < 1) {
      const candidate = Math.min(1, scale + increment);
      if (this._homotopyRungConverged(DC_OP_SOURCE_STEP_GMIN_G, candidate, counters)) {
        scale = candidate;
        increment *= 2;
      } else {
        increment /= 2;
        // Below 1/1024 of full drive the continuation is stuck on a turning
        // point that source ramping cannot round; hand over to the next stage.
        if (increment < 1 / 1024) return false;
      }
    }
    // The full-drive point still carries the pinned gmin load, so it is not
    // yet a solution of the true system; finish with the overlay removed.
    return this._homotopyRungConverged(0, 1, counters);
  }

  /**
   * Stage d — pseudo-transient continuation with physical companion
   * stamping. Backward Euler is L-stable, so growing pseudo-steps damp
   * oscillatory modes instead of amplifying them and the trajectory parks at
   * an operating point without the trajectory itself needing to be accurate.
   * simTime never advances: pseudo-time is not physical time. That includes
   * the sources — _waveformSampleTime pins clock/pulse/signal-gen sampling
   * to the OP instant while _dcPhysicalTimeHold is set, so a growing h never
   * stamps (or lets _updateDigitalState read) a source phase up to 1 ms past
   * the instant the caller asked about. Digital/regime state otherwise keeps
   * co-settling here: the relaxation needs gate outputs and latches to
   * follow the trajectory to reach a mixed-signal fixed point.
   */
  private _dcPseudoTransient(counters: DcOpCounters): boolean {
    let h = DC_OP_PTA_H_MIN_S;
    let stepsAtThisH = 0;
    let previousNetV: Record<string, number> = { ...this.netV };
    for (let attempts = 0; attempts < DC_OP_PTA_STEP_BUDGET; attempts++) {
      // Every pseudo-step is a discontinuity-grade BE step; trap history
      // must never form across a non-physical trajectory.
      this._beNextStep = true;
      this._solve(h);
      counters.iterations += this.lastIters;
      if (!this.lastConverged) {
        // Rejected trial keeps the trusted state; retry a decade gentler.
        h = Math.max(1e-12, h / 10);
        stepsAtThisH = 0;
        continue;
      }
      let movement = 0;
      for (const [netId, v] of Object.entries(this.netV)) {
        const delta = Math.abs(v - (previousNetV[netId] ?? 0));
        if (delta > movement) movement = delta;
      }
      previousNetV = { ...this.netV };
      // Settlement is only meaningful at full pseudo-step size: far from the
      // point a 1 ns step also moves almost nothing.
      if (h >= DC_OP_PTA_H_MAX_S && movement < DC_OP_PTA_SETTLE_V) return true;
      stepsAtThisH += 1;
      if (h < DC_OP_PTA_H_MAX_S && stepsAtThisH >= DC_OP_PTA_STEPS_PER_H) {
        h = Math.min(DC_OP_PTA_H_MAX_S, h * 10);
        stepsAtThisH = 0;
      }
    }
    return false;
  }

  /**
   * Assemble the dc-mode system matrix (the Newton Jacobian) at iterate x
   * into a plain dense array by replaying the stamp pass through a
   * recording facade. This is deliberately backend-independent: dense and
   * sparse runs replay the identical stamp order into the same flat array,
   * so every stability decision below is bit-identical across backends.
   * dc stamping is forced for the duration so the screen always measures
   * the true DC Jacobian — the pseudo-transient stage's candidate was
   * produced under BE companion stamping, where a capacitor's 1 ps
   * companion conductance would mask any resistive instability.
   */
  private _dcExtractSystemMatrix(x: Float64Array): Float64Array | null {
    const n = this.mna.size;
    const matrix = new Float64Array(n * n);
    const recorder: LinearSystem = {
      size: n,
      lastSolveInfo: {
        singular: false,
        illConditioned: false,
        rank: n,
        minScaledPivot: 0,
        relativeResidual: 0,
        nonFinite: false,
      },
      factorizationCount: 0,
      factorizationReuseCount: 0,
      clear: () => { /* recording only */ },
      add: (row, col, val) => { matrix[row * n + col] += val; },
      addB: () => { /* the screen needs the matrix, not the RHS */ },
      captureBase: () => { /* recording only */ },
      resetToBase: () => { /* recording only */ },
      solve: () => new Float64Array(n),
      solveRhs: () => new Float64Array(n),
    };
    const realMna = this.mna;
    const priorDcMode = this._dcSolveMode;
    const priorGmin = this._homotopyGminG;
    const priorScale = this._homotopySourceScale;
    try {
      this.mna = recorder;
      this._dcSolveMode = true;
      this._homotopyGminG = 0;
      this._homotopySourceScale = 1;
      // Replay of _rebuildStaticStampBase: RSHUNT plus ideal-source
      // incidence, then the static components, then the dynamic pass.
      for (let row = 0; row < this.nodeCount; row++) {
        recorder.add(row, row, NODE_RSHUNT_G);
      }
      for (const componentId of this._staticIdealSourceIds) {
        const component = this._componentById.get(componentId);
        const branch = this.vsrcIdx.get(componentId);
        if (!component || branch === undefined) continue;
        stampVSource(
          recorder,
          this._pinNode(component.id, component.pins[0]!.id),
          this._pinNode(component.id, component.pins[1]!.id),
          branch,
          0,
        );
      }
      this._stampAll(DC_OP_COMPANION_H, x, this._staticStampComponents);
      this._stampAll(DC_OP_COMPANION_H, x);
      return matrix;
    } catch {
      // A stamp failure here must never poison the accepted solve; the
      // screen simply reports itself inapplicable.
      return null;
    } finally {
      this.mna = realMna;
      this._dcSolveMode = priorDcMode;
      this._homotopyGminG = priorGmin;
      this._homotopySourceScale = priorScale;
    }
  }

  /**
   * Sign of det(A + g·P) with P the identity on node rows, zero on source
   * branch rows. Row equilibration (a positive scaling of det) plus
   * partial-pivot elimination tracking swaps keeps the computation
   * overflow-free while preserving exactly the sign. Returns 0 when the
   * shifted matrix is numerically singular — callers treat that as
   * "screen inconclusive", never as instability.
   */
  private _dcShiftedDetSign(matrix: Float64Array, gShift: number): number {
    const n = this.mna.size;
    const lu = new Float64Array(matrix);
    if (gShift !== 0) {
      for (let row = 0; row < this.nodeCount; row++) {
        lu[row * n + row] += gShift;
      }
    }
    let sign = 1;
    for (let row = 0; row < n; row++) {
      let max = 0;
      for (let col = 0; col < n; col++) {
        max = Math.max(max, Math.abs(lu[row * n + col]));
      }
      if (!(max > 0) || !Number.isFinite(max)) return 0;
      const scale = 1 / max;
      for (let col = 0; col < n; col++) {
        lu[row * n + col] *= scale;
      }
    }
    for (let col = 0; col < n; col++) {
      let pivotRow = col;
      let maxVal = Math.abs(lu[col * n + col]);
      for (let row = col + 1; row < n; row++) {
        const value = Math.abs(lu[row * n + col]);
        if (value > maxVal) {
          maxVal = value;
          pivotRow = row;
        }
      }
      // Relative to the equilibrated (order-1) rows this is far below any
      // healthy pivot; a smaller one means "too close to singular to trust
      // a sign", which the screen maps to inconclusive.
      if (!(maxVal > 1e-12) || !Number.isFinite(maxVal)) return 0;
      if (pivotRow !== col) {
        sign = -sign;
        for (let k = col; k < n; k++) {
          const tmp = lu[col * n + k];
          lu[col * n + k] = lu[pivotRow * n + k];
          lu[pivotRow * n + k] = tmp;
        }
      }
      const pivot = lu[col * n + col];
      if (pivot < 0) sign = -sign;
      for (let row = col + 1; row < n; row++) {
        const multiplier = lu[row * n + col] / pivot;
        if (multiplier === 0) continue;
        for (let k = col + 1; k < n; k++) {
          lu[row * n + k] -= multiplier * lu[col * n + k];
        }
      }
    }
    return sign;
  }

  /**
   * Run the determinant pencil screen (see DC_OP_STABILITY_MAX_SIZE) on the
   * committed iterate. "unknown" means the screen is inapplicable (system
   * too large, no committed iterate, or a numerically singular pencil
   * endpoint) and is treated as acceptance by callers — the screen only
   * ever downgrades a provably unstable point.
   */
  private _dcCandidateStability(): {
    verdict: "stable" | "unstable" | "unknown";
    matrix: Float64Array | null;
    gBig: number;
  } {
    const n = this.mna.size;
    if (n === 0 || n > DC_OP_STABILITY_MAX_SIZE || this.nodeCount === 0) {
      return { verdict: "unknown", matrix: null, gBig: 0 };
    }
    const x = this._lastX;
    if (!x || x.length !== n) return { verdict: "unknown", matrix: null, gBig: 0 };
    const matrix = this._dcExtractSystemMatrix(x);
    if (!matrix) return { verdict: "unknown", matrix: null, gBig: 0 };
    let maxAbs = 0;
    for (const value of matrix) {
      const magnitude = Math.abs(value);
      if (magnitude > maxAbs) maxAbs = magnitude;
    }
    if (!(maxAbs > 0) || !Number.isFinite(maxAbs)) {
      return { verdict: "unknown", matrix: null, gBig: 0 };
    }
    // Beyond every finite pencil eigenvalue (Gershgorin-style bound with a
    // wide safety factor). p(g) has constant sign past its largest real
    // root, so overshooting costs nothing.
    const gBig = 1e6 * n * maxAbs;
    const signAtZero = this._dcShiftedDetSign(matrix, 0);
    const signAtInfinity = this._dcShiftedDetSign(matrix, gBig);
    if (signAtZero === 0 || signAtInfinity === 0) {
      return { verdict: "unknown", matrix, gBig };
    }
    return {
      verdict: signAtZero === signAtInfinity ? "stable" : "unstable",
      matrix,
      gBig,
    };
  }

  /**
   * Dense equilibrated partial-pivot LU over a shifted copy of the screen
   * matrix, packaged as a solve closure for the inverse iteration below.
   * Local on purpose: it must be bit-identical across linear backends, and
   * the pencil shift g·P is not expressible through the stamp interface.
   */
  private _dcDenseSolveFactory(
    matrix: Float64Array,
    gShift: number,
  ): ((rhs: Float64Array) => Float64Array | null) | null {
    const n = this.mna.size;
    const lu = new Float64Array(matrix);
    if (gShift !== 0) {
      for (let row = 0; row < this.nodeCount; row++) {
        lu[row * n + row] += gShift;
      }
    }
    const rowScale = new Float64Array(n);
    for (let row = 0; row < n; row++) {
      let max = 0;
      for (let col = 0; col < n; col++) {
        max = Math.max(max, Math.abs(lu[row * n + col]));
      }
      if (!(max > 0) || !Number.isFinite(max)) return null;
      const scale = 1 / max;
      rowScale[row] = scale;
      for (let col = 0; col < n; col++) {
        lu[row * n + col] *= scale;
      }
    }
    const pivots = new Int32Array(n);
    for (let col = 0; col < n; col++) {
      let pivotRow = col;
      let maxVal = Math.abs(lu[col * n + col]);
      for (let row = col + 1; row < n; row++) {
        const value = Math.abs(lu[row * n + col]);
        if (value > maxVal) {
          maxVal = value;
          pivotRow = row;
        }
      }
      // Unlike the det-sign pass, a tiny (but nonzero) pivot is fine here:
      // inverse iteration WANTS a nearly singular shift — the solve blows
      // up along precisely the eigenvector being sought, and the caller
      // renormalizes. Only exact zero/non-finite pivots are fatal.
      if (maxVal === 0 || !Number.isFinite(maxVal)) return null;
      pivots[col] = pivotRow;
      if (pivotRow !== col) {
        for (let k = 0; k < n; k++) {
          const tmp = lu[col * n + k];
          lu[col * n + k] = lu[pivotRow * n + k];
          lu[pivotRow * n + k] = tmp;
        }
      }
      const pivot = lu[col * n + col];
      for (let row = col + 1; row < n; row++) {
        const multiplier = lu[row * n + col] / pivot;
        lu[row * n + col] = multiplier;
        if (multiplier === 0) continue;
        for (let k = col + 1; k < n; k++) {
          lu[row * n + k] -= multiplier * lu[col * n + k];
        }
      }
    }
    return (rhs: Float64Array): Float64Array | null => {
      const work = new Float64Array(n);
      for (let row = 0; row < n; row++) {
        work[row] = rhs[row] * rowScale[row];
      }
      for (let col = 0; col < n; col++) {
        const pivotRow = pivots[col];
        if (pivotRow !== col) {
          const tmp = work[col];
          work[col] = work[pivotRow];
          work[pivotRow] = tmp;
        }
      }
      for (let row = 0; row < n; row++) {
        let sum = work[row];
        for (let col = 0; col < row; col++) {
          sum -= lu[row * n + col] * work[col];
        }
        work[row] = sum;
      }
      for (let row = n - 1; row >= 0; row--) {
        let sum = work[row];
        for (let col = row + 1; col < n; col++) {
          sum -= lu[row * n + col] * work[col];
        }
        work[row] = sum / lu[row * n + row];
        if (!Number.isFinite(work[row])) return null;
      }
      return work;
    };
  }

  /**
   * Approximate the growing real eigenmode of a rejected saddle. The
   * det-sign change of p(g) on (0, gBig] is bisected down to the unstable
   * pencil eigenvalue, then a few steps of inverse iteration with that
   * shift pull out its eigenvector. Returned max-norm-1 on node rows with
   * a canonical orientation (largest-magnitude entry positive) so escape
   * kicks are bit-reproducible run to run.
   */
  private _dcUnstableDirection(
    matrix: Float64Array,
    gBig: number,
  ): Float64Array | null {
    const n = this.mna.size;
    const signAtZero = this._dcShiftedDetSign(matrix, 0);
    if (signAtZero === 0) return null;
    let low = 0;
    let high = gBig;
    for (let iter = 0; iter < 60; iter++) {
      const mid = 0.5 * (low + high);
      if (!(mid > low) || !(mid < high)) break;
      const signAtMid = this._dcShiftedDetSign(matrix, mid);
      // A singular midpoint IS the eigenvalue; tighten onto it from above.
      if (signAtMid === 0 || signAtMid !== signAtZero) {
        high = mid;
      } else {
        low = mid;
      }
    }
    // The bracket midpoint sits close enough to the eigenvalue for inverse
    // iteration to converge in a handful of steps, while staying far
    // enough from exact singularity for the factorization to complete.
    const shift = 0.5 * (low + high);
    const solve = this._dcDenseSolveFactory(matrix, shift);
    if (!solve) return null;
    let direction = new Float64Array(n);
    for (let row = 0; row < this.nodeCount; row++) direction[row] = 1;
    for (let step = 0; step < 8; step++) {
      const rhs = new Float64Array(n);
      for (let row = 0; row < this.nodeCount; row++) {
        rhs[row] = direction[row];
      }
      const next = solve(rhs);
      if (!next) return null;
      let maxComponent = 0;
      for (let row = 0; row < this.nodeCount; row++) {
        const magnitude = Math.abs(next[row]);
        if (magnitude > maxComponent) maxComponent = magnitude;
      }
      if (!(maxComponent > 0) || !Number.isFinite(maxComponent)) return null;
      const inv = 1 / maxComponent;
      direction = new Float64Array(n);
      for (let row = 0; row < this.nodeCount; row++) {
        direction[row] = next[row] * inv;
      }
    }
    let anchorRow = 0;
    let anchorAbs = 0;
    for (let row = 0; row < this.nodeCount; row++) {
      const magnitude = Math.abs(direction[row]);
      if (magnitude > anchorAbs) {
        anchorAbs = magnitude;
        anchorRow = row;
      }
    }
    if (!(anchorAbs > 0)) return null;
    if (direction[anchorRow] < 0) {
      for (let row = 0; row < this.nodeCount; row++) {
        direction[row] = -direction[row];
      }
    }
    return direction;
  }

  /**
   * Gate applied to every ladder-stage acceptance: pass the stability
   * screen, or escape the saddle by kicking the iterate along its growing
   * eigenmode and re-settling. A successful escape is accepted within the
   * same stage (the re-settled point is a true-system solution found by
   * that stage's iterate). Failure restores the saddle iterate so the next
   * ladder stage starts from the same trusted state it would have today.
   *
   * Known stage-d limitation: this gate runs with whatever stamping mode the
   * stage used, and stage d arrives with _dcSolveMode off, so the kick
   * re-settle solves stamp capacitors as their 1 ps companions (~C/1e-12
   * siemens holds pinned at the saddle voltages) rather than dc opens. A
   * saddle held by a commutating capacitor bridging the switching nodes can
   * therefore absorb every kick and re-converge onto itself, failing the
   * whole OP instead of escaping. That outcome is fail-safe (the saddle is
   * rejected, never committed); re-settling in dc mode here would make the
   * escape effective for those circuits but requires the dc-mode inductor
   * commit as well — a deliberate tradeoff, revisit if a real circuit hits
   * it.
   */
  private _dcAcceptStableOrEscape(
    maxOuterRegimeIters: number,
    counters: DcOpCounters,
  ): boolean {
    const screen = this._dcCandidateStability();
    if (screen.verdict !== "unstable") return true;
    if (!screen.matrix) return false;
    const direction = this._dcUnstableDirection(screen.matrix, screen.gBig);
    if (!direction) return false;
    // Kick magnitudes scale with the point's own voltage span so a 3.3 V
    // logic latch and a 24 V industrial one get proportionate shoves.
    let span = 1;
    for (const value of Object.values(this.netV)) {
      if (Number.isFinite(value)) span = Math.max(span, Math.abs(value));
    }
    const saddleNetV = { ...this.netV };
    for (const scale of DC_OP_ESCAPE_KICK_SCALES) {
      for (const orientation of [1, -1] as const) {
        const attempt = this.saveState();
        const kicked: Record<string, number> = { gnd: 0 };
        for (const [netId, row] of this.nodeIdx) {
          const base = saddleNetV[netId] ?? 0;
          const component =
            row >= 0 && row < this.nodeCount ? direction[row] : 0;
          kicked[netId] = base + orientation * scale * span * component;
        }
        this.netV = kicked;
        if (
          this._dcRegimeSettle(maxOuterRegimeIters, counters)
          && this._dcCandidateStability().verdict !== "unstable"
        ) {
          return true;
        }
        this.restoreState(attempt);
      }
    }
    return false;
  }

  /**
   * Commit an accepted operating point the way an accepted solve would.
   * _solve already committed netV/elementI and the digital/regime state from
   * the final converged rung; what remains is the capacitor/inductor and
   * electromechanical-winding state that dc stamping deliberately did not
   * integrate, plus the histories the A2 trapezoidal method reads.
   */
  private _commitDcOperatingPointState(viaDcStamps: boolean): void {
    if (!this.circuit) return;
    const x = this._lastX;
    for (const comp of this.circuit.components) {
      if (comp.pins.length < 2) continue;
      // Electromechanical windings (relay coil, dc_motor, stepper coils) get
      // the same DC normalisation as plain inductors: their dc-mode stamps
      // were R-only shorts, so the solved terminal voltages map 1:1 onto DC
      // winding currents. Their own commitState paths cannot produce these —
      // dc_motor/stepper are physical-time-held during the ladder and the
      // relay's companion formula holds the entry current at the 1 ps
      // companion interval — and pseudo-transient reached the point through
      // real BE companions, so (exactly like inductors below) the settled
      // currents are already correct there and must not be recomputed.
      if (viaDcStamps && x && comp.kind === "relay") {
        const vCoil = this._vAt(x, this._pinNode(comp.id, "coil_a"))
          - this._vAt(x, this._pinNode(comp.id, "coil_b"));
        const coilR = Math.max(1, Number(comp.params.coilR ?? 70));
        const prev = this.state.relays.get(comp.id) ?? { iCoil: 0, energized: false };
        const iCoil = vCoil / coilR;
        this.state.relays.set(comp.id, { ...prev, iCoil });
        // The element-current pass ran before this normalisation and read
        // the held map entry; republish so the OP readout matches (same
        // treatment as the inductor branch below).
        this.elementI[comp.id] = iCoil;
        continue;
      }
      if (viaDcStamps && x && comp.kind === "dc_motor") {
        const vWinding = this._vAt(x, this._pinNode(comp.id, "m1"))
          - this._vAt(x, this._pinNode(comp.id, "m2"));
        const windingR = Math.max(0.001, Number(comp.params.windingR ?? 5));
        const Ke = Number(comp.params.Ke ?? 0.01);
        const prev = this.state.motors.get(comp.id) ?? { iWinding: 0, omega: 0 };
        // Rotor speed is held physical-time state; only the electrical
        // current is normalised to the DC meaning of the R + back-EMF stamp.
        const iWinding = (vWinding - Ke * prev.omega) / windingR;
        this.state.motors.set(comp.id, { ...prev, iWinding });
        this.elementI[comp.id] = iWinding;
        continue;
      }
      if (viaDcStamps && x && comp.kind === "stepper") {
        const coilR = Math.max(0.001, Number(comp.params.coilR ?? 10));
        const vA = this._vAt(x, this._pinNode(comp.id, "a1"))
          - this._vAt(x, this._pinNode(comp.id, "a2"));
        const vB = this._vAt(x, this._pinNode(comp.id, "b1"))
          - this._vAt(x, this._pinNode(comp.id, "b2"));
        const prev = this.state.steppers.get(comp.id)
          ?? { iA: 0, iB: 0, phase: -1, lastKnownPhase: -1, position: 0 };
        // Phase/position stay held: they are step-counting mechanics, and an
        // OP is a bias solve, not a motion interval.
        const iA = vA / coilR;
        const iB = vB / coilR;
        this.state.steppers.set(comp.id, { ...prev, iA, iB });
        this.elementI[comp.id] = Math.abs(iA) + Math.abs(iB);
        continue;
      }
      if (comp.kind !== "capacitor" && comp.kind !== "inductor") continue;
      const a = this._pinNode(comp.id, comp.pins[0].id);
      const b = this._pinNode(comp.id, comp.pins[1].id);
      const terminalVoltage = x ? this._vAt(x, a) - this._vAt(x, b) : 0;
      if (comp.kind === "capacitor") {
        // Series current is zero at DC, so the ESR drop vanishes and the
        // internal capacitor voltage equals the terminal voltage. Only the
        // declared leakage shunt carries current at the point.
        this.state.caps.set(comp.id, terminalVoltage);
        this.state.capsI.set(comp.id, 0);
        const leakageCurrent = parallelLossCurrent(
          terminalVoltage,
          parallelLossResistance(modelParam(comp, "leakageResistance", 0)),
        );
        this.state.capCurrents.set(comp.id, leakageCurrent);
        this.elementI[comp.id] = leakageCurrent;
      } else {
        if (viaDcStamps) {
          // The dc stamp was exactly R = max(dcr, 1e-6), so the solved
          // terminal voltage maps 1:1 onto the DC winding current.
          const dcr = seriesLossResistance(modelParam(comp, "dcr", 0));
          const windingCurrent = terminalVoltage / Math.max(dcr, 1e-6);
          this.state.inds.set(comp.id, windingCurrent);
          this.elementI[comp.id] = windingCurrent + parallelLossCurrent(
            terminalVoltage,
            parallelLossResistance(modelParam(comp, "coreLossResistance", 0)),
          );
        }
        // Pseudo-transient reached the point through real BE companions, so
        // state.inds already holds the settled winding current there —
        // recomputing it from the ~zero terminal drop of an ideal coil would
        // destroy it. The A2 terminal-voltage history is refreshed either way.
        this.state.indsV.set(comp.id, terminalVoltage);
      }
    }
    // An OP commit is a discontinuity for the trapezoidal history: whatever
    // came before, the next accepted step must re-anchor on backward Euler.
    this._beNextStep = true;
  }

  /**
   * Wave A3 seed rescue: when load()'s 1 ps seed fails, walk the same gmin
   * and source-stepping ladders dcOperatingPoint uses — but at the seed's
   * own companion stamping (NOT dc mode), preserving its hold-the-initial-
   * conditions semantics. Success ends with one plain warm-started re-solve
   * of the unmodified seed system, so a rescued load commits the same single
   * full post-solve update pass (digital state included) a first-try
   * converged seed would have; total failure restores the failed-seed state,
   * i.e. today's quiet give-up. Reached only when this.lastConverged is
   * false, so circuits whose seed converges never enter this path
   * (bit-identity guarantee).
   */
  private _rescueSeedOperatingPoint(): void {
    const entry = this.saveState();
    // Rescue rungs can stamp positions ordinary seeding never discovers
    // (regime flips at partial drive); on give-up the sparse pattern must
    // roll back with the state, or the quiet failure would still leave a
    // permanently enlarged pattern behind (see dcOperatingPoint for why
    // pattern membership is numerics, not bookkeeping).
    const entryPatternCheckpoint = this.mna.patternCheckpoint?.();
    const counters: DcOpCounters = { iterations: 0, regimeIterations: 0 };
    let rescued = false;
    try {
      this._newtonMaxIterOverride = DC_OP_NEWTON_MAX_ITER;
      // Rungs hold physical time exactly like dcOperatingPoint's: a rescue
      // is still a solve of the present instant, not a physical interval.
      this._dcPhysicalTimeHold = true;
      rescued = this._homotopyGminLadder(counters);
      if (!rescued) rescued = this._homotopySourceLadder(counters);
    } catch (error) {
      // Mirror dcOperatingPoint's throw contract: never let an exception
      // escape load() with the engine abandoned at a homotopy anchor.
      this.restoreState(entry);
      if (entryPatternCheckpoint !== undefined) {
        this.mna.restorePatternCheckpoint?.(entryPatternCheckpoint);
      }
      this.mna.invalidateFactorization?.();
      throw error;
    } finally {
      this._homotopyGminG = 0;
      this._homotopySourceScale = 1;
      this._newtonMaxIterOverride = null;
      this._dcPhysicalTimeHold = false;
    }
    if (rescued) {
      // The rungs held digital and physical-time state, but a genuinely
      // converged seed commits one full post-solve update pass. Re-solving
      // the unmodified seed system from its own solution (controls are back
      // at their defaults, so this IS the plain seed solve, warm-started)
      // performs exactly that pass — without it, a rescued load would carry
      // pre-seed digital levels against post-rescue voltages.
      this._solve(DC_OP_COMPANION_H);
      rescued = this.lastConverged;
    }
    // The snapshot carries the failed-seed diagnostics, so restoring it
    // reproduces the pre-rescue engine exactly, lastConverged=false included
    // (the factorization invalidation drops rung pivot memory the pattern
    // checkpoint cannot express — see dcOperatingPoint's failure path).
    if (!rescued) {
      this.restoreState(entry);
      if (entryPatternCheckpoint !== undefined) {
        this.mna.restorePatternCheckpoint?.(entryPatternCheckpoint);
      }
      this.mna.invalidateFactorization?.();
    }
  }

  // ─── Simulation stepping ───────────────────────────────────────────────

  /**
   * Advance simulation by h seconds.
   *
   * When a NE555 is present: takes a trial full step, checks whether the
   * THR or TRIG node crossed the 1/3·VCC or 2/3·VCC threshold during the
   * step, and if so rolls back and re-solves in two sub-steps split at the
   * estimated crossing time. This keeps the 555 SR-flip output transition
   * aligned with the actual physical crossing rather than lagging by a
   * full step.
   */
  step(h: number): void {
    if (!this.circuit) return;

    const has555 = this._hasNe555;

    if (!has555) {
      this._solve(h);
      if (!this.lastConverged) return;
      this.simTime += h;
      this._advanceMcusAfterAcceptedSolve(h);
      // One accepted step re-anchors the trap history; failed solves above
      // return with the flag intact so the retry stamps the same method.
      this._beNextStep = false;
      return;
    }

    // Snapshot pre-step voltages and 555 state for crossing detection.
    const preNetV = { ...this.netV };
    const preNe555s = new Map(this.state.ne555s);
    const snap = this.saveState();

    this._solve(h);

    // A failed electrical trial is not a physical interval. In particular, do
    // not sample inputs or advance an external CPU that saveState cannot rewind.
    if (!this.lastConverged) {
      this._restoreFailedSplitTrial(snap);
      return;
    }

    const frac = this._find555CrossingFrac(preNetV, preNe555s);
    if (frac === null) {
      this.simTime = snap.simTime + h;
      this._advanceMcusAfterAcceptedSolve(h);
      this._beNextStep = false;
      return;
    }

    // Roll back, split at the threshold crossing.
    this.restoreState(snap);
    this.simTime = snap.simTime;

    this._solve(h * frac);
    if (!this.lastConverged) {
      this._restoreFailedSplitTrial(snap);
      return;
    }
    this.simTime = snap.simTime + h * frac;

    // The 555 output flips at the sub-step boundary just solved, so the
    // post-crossing sub-step stamps a step discontinuity: integrate it with
    // backward Euler in trap mode. A failed sub-solve restores the pre-step
    // flag via the snapshot inside _restoreFailedSplitTrial.
    this._beNextStep = true;

    this._solve(h * (1 - frac));
    if (!this.lastConverged) {
      this._restoreFailedSplitTrial(snap);
      return;
    }
    this.simTime = snap.simTime + h;
    this._advanceMcusAfterAcceptedSolve(h);
    this._beNextStep = false;
  }

  /**
   * Commit the external CPU side of one converged electrical interval.
   *
   * The accepted endpoint is sampled first, then each powered core advances
   * exactly once. Its resulting GPIO state and ordered PinEvents are retained
   * for the next electrical interval. This explicit ordering keeps an MCU out
   * of Newton/NE555 trial solves and makes a worker retry firmware-neutral.
   */
  private _advanceMcusAfterAcceptedSolve(h: number): void {
    this._sampleArduinoInputs();
    for (const [id, mcu] of this.state.arduinos) {
      const comp = this._componentById.get(id);
      if (comp && this._mcuPowered(comp)) mcu.step(h);
    }
  }

  /** Restore the trusted pre-trial state while preserving failed-solve diagnostics. */
  private _restoreFailedSplitTrial(snap: StateSnapshot): void {
    const diagnostics: SolverDiagnosticsSnapshot = {
      lastIters: this.lastIters,
      lastConverged: false,
      lastSolveUs: this.lastSolveUs,
      lastMatrixSize: this.lastMatrixSize,
      lastMatrixSingular: this.lastMatrixSingular,
      lastMatrixIllConditioned: this.lastMatrixIllConditioned,
      lastRelativeResidual: this.lastRelativeResidual,
    };
    this.restoreState(snap);
    this.lastIters = diagnostics.lastIters;
    this.lastConverged = diagnostics.lastConverged;
    this.lastSolveUs = diagnostics.lastSolveUs;
    this.lastMatrixSize = diagnostics.lastMatrixSize;
    this.lastMatrixSingular = diagnostics.lastMatrixSingular;
    this.lastMatrixIllConditioned = diagnostics.lastMatrixIllConditioned;
    this.lastRelativeResidual = diagnostics.lastRelativeResidual;
  }

  getNetV(): Record<string, number> {
    return this.netV;
  }

  /** Resolve a component pin to its current deterministic net id. */
  getNetIdForPin(componentId: string, pinId: string): string | undefined {
    return this._pinToNetIndex.get(`${componentId}\0${pinId}`);
  }

  // ─── Small-signal AC exposure (Wave A5) ─────────────────────────────────
  // Read-only seams for the runSmallSignalAc driver (ac-analysis.ts). The
  // driver lives outside the engine, so the loaded topology layout and the
  // held operating-point solution are published here as pure reads — no new
  // mutable surface, and none of these are consulted by any transient pass.

  /** Matrix layout of the loaded topology. `size` includes branch rows;
   *  node rows occupy [0, nodeCount). Zeros before the first load(). */
  matrixDimensions(): { size: number; nodeCount: number } {
    return { size: this.mna?.size ?? 0, nodeCount: this.nodeCount };
  }

  /** MNA row for a net id: -1 for gnd, undefined for an unknown net. */
  netRow(netId: string): number | undefined {
    return this.nodeIdx?.get(netId);
  }

  /** Loaded components in circuit order (empty before the first load()). */
  loadedComponents(): readonly SimComponent[] {
    return this.circuit?.components ?? [];
  }

  /**
   * Build the read-only device facade for one small-signal AC analysis. A
   * fresh object per call — unlike the cached _deviceCtx — because the AC
   * input designation is per-analysis state that must not leak between
   * runs. Every member aliases the exact engine expression the transient
   * stamps read (see AcDeviceContext in device-registry.ts); the solution
   * accessors read the engine's last committed solve, which after a
   * converged dcOperatingPoint() is precisely the held operating point.
   */
  acDeviceContext(designation: { inputId: string | null }): AcDeviceContext {
    const engine = this;
    return {
      pinNode: (compId, pinId) => engine._pinNode(compId, pinId),
      isOpenPin: (compId, pinId) => engine._isOpenPin(compId, pinId),
      vsrcRow: (key) => engine.vsrcIdx.get(key),
      internalNode: (compId, name) =>
        engine.internalNodeIdx.get(`${compId}:${name}`) ?? -1,
      opVoltage: (row) =>
        engine._lastX ? engine._vAt(engine._lastX, row) : 0,
      opPinVoltage: (compId, pinId) =>
        engine._lastX
          ? engine._vAt(engine._lastX, engine._pinNode(compId, pinId))
          : 0,
      modelParam: (comp, key, fallback) => modelParam(comp, key, fallback),
      electricalSpecs: (comp) => specsForComponent(comp),
      catalogPart: (comp) => partFor(comp),
      envLux: (comp) => engine._envLux(comp),
      envTempC: (comp) => engine._envTempC(comp),
      ambientTempC: () => engine._ambientTempC(),
      junctionVt: () => engine._junctionVt(),
      junctionVf: (vfAt25C) => engine._junctionVf(vfAt25C),
      state: {
        get ptcs() {
          return engine.state.ptcs;
        },
        get thermalTemps() {
          return engine.state.thermalTemps;
        },
        get icState() {
          return engine.state.icState;
        },
        get ne555s() {
          return engine.state.ne555s;
        },
        get eeproms() {
          return engine.state.eeproms;
        },
        get relays() {
          return engine.state.relays;
        },
        get servos() {
          return engine.state.servos;
        },
        get hcsr04() {
          return engine.state.hcsr04;
        },
        get thermalDevices() {
          return engine.state.thermalDevices;
        },
      },
      hasFailure: (compId, kind, pinId = "") => engine._hasFailure(compId, kind, pinId),
      batteryOperatingPoint: (comp) => engine._batteryOperatingPoint(comp),
      icPowerInfoAtOp: (comp) =>
        engine._icPowerInfo(
          comp,
          // A missing solution (never solved) degrades to the all-zero
          // vector: every package then reads unpowered, which is the honest
          // small-signal picture of an engine with no committed OP.
          engine._lastX ?? new Float64Array(engine.mna?.size ?? 0),
        ),
      logicHighAtOp: (comp, pinId, power) =>
        engine._logicHigh(comp, pinId, engine._acOpVector(), power),
      logicHighHAtOp: (comp, pinId, power) =>
        engine._logicHighH(comp, pinId, engine._acOpVector(), power),
      readAddr4AtOp: (comp, power) =>
        engine._readAddr4(comp, engine._acOpVector(), power),
      simTime: () => engine.simTime,
      acInputMagnitude: (compId) => (compId === designation.inputId ? 1 : 0),
    };
  }

  /** Held OP solution for the AC facade's logic reads, with the same
   *  all-zero degradation icPowerInfoAtOp uses before any solve. */
  private _acOpVector(): Float64Array {
    return this._lastX ?? new Float64Array(this.mna?.size ?? 0);
  }

  /** Reset (restart) the simulated MCU for the given component id. */
  resetArduino(componentId: string): void {
    const mcu = this.state.arduinos.get(componentId);
    if (mcu) {
      mcu.reset();
      // S18b — reset() rebuilds the core with its cycle counter back at 0
      // (both ArduinoMcu and RP2040Mcu), so the boot baseline moves too.
      this._mcuBootSimTime.set(componentId, this.simTime);
    }
  }

  /**
   * S18b — engine simTime at which each MCU core currently in `state.arduinos`
   * (re)booted (cycle counter 0). PinEvent.cycle (arduino.ts / rp2040.ts) is a
   * cumulative-since-boot counter, NOT step-relative, at each core's native
   * clock (16 MHz AVR / 125 MHz RP2040 — neither exposed on the shared
   * MicrocontrollerCore contract, so cycle-accurate peripheral decoders need
   * this baseline instead of assuming boot coincided with simTime 0: a hex/
   * script can be (re)flashed mid-simulation, and resetArduino() rebuilds the
   * core independently of any circuit reload). Captured wherever `load()`
   * actually constructs a NEW core object, and on resetArduino(); pruned
   * alongside microbitDrives above.
   */
  private _mcuBootSimTime = new Map<string, number>();

  /** Program-driven micro:bit edge-pin states (P0/P1/P2), pushed from the client
   *  embedded sim. Keyed by component id → { pinId → { mode, value } }. Runtime
   *  only; PRUNED to live boards on load() (surviving boards keep their drives —
   *  the embedded sim keeps running across circuit edits and only re-pushes on
   *  change; the host clears a board's entry on flash/reset/power-off). */
  private microbitDrives = new Map<string, Record<string, { mode: "digital" | "analog"; value: number }>>();

  setMicrobitDrive(
    componentId: string,
    drives: Record<string, { mode: "digital" | "analog"; value: number }>,
  ): void {
    this.microbitDrives.set(componentId, drives);
  }

  getElementI(): Record<string, number> {
    return this.elementI;
  }

  /** Runtime chemistry/SoC state for battery telemetry and user readouts. */
  getBatteryState(compId: string): BatteryRuntimeReadout | undefined {
    const candidate = this._componentById.get(compId);
    const comp = candidate?.kind === "battery_pack" ? candidate : undefined;
    const state = this.state.batteries.get(compId);
    if (!comp || !state) return undefined;
    const battery = this._batteryOperatingPoint(comp);
    return {
      profileId: battery.profile.id,
      label: battery.profile.label,
      soc: state.soc,
      openCircuitVoltageV: battery.point.openCircuitVoltageV * battery.nominalVoltageScale,
      internalResistanceOhm: battery.point.internalResistanceOhm,
      remainingCapacityCoulombs: battery.point.remainingCapacityCoulombs,
      dischargedCoulombs: state.dischargedCoulombs,
      rejectedRechargeCoulombs: state.rejectedRechargeCoulombs,
      modelTemperatureC: battery.point.modelTemperatureC,
      temperatureWasClamped: battery.point.temperatureWasClamped,
    };
  }

  /**
   * W7.1 — Returns the committed motor state (iWinding + omega) for the given
   * component ID, or undefined if no motor with that ID exists.
   *
   * Used by tests and future UI indicators to read rotor speed (omega, rad/s)
   * and winding current without re-running the companion formula.
   * RPM = omega * 60 / (2π).
   */
  getMotorState(compId: string): MotorEngineState | undefined {
    return this.state.motors.get(compId);
  }

  /**
   * W7.2 — Returns the committed servo state for the given component ID, or
   * undefined if the component is not known.
   *
   * Used by tests and future UI to read the decoded shaft angle and PWM timing.
   */
  getServoState(compId: string): ServoEngineState | undefined {
    return this.state.servos.get(compId);
  }

  /**
   * W7.2 — Returns the committed stepper state for the given component ID, or
   * undefined if the component is not known.
   *
   * Used by tests and future UI to read coil currents, phase, and position.
   */
  getStepperState(compId: string): StepperEngineState | undefined {
    return this.state.steppers.get(compId);
  }

  /**
   * S18b — Returns the committed HC-SR04 state (TRIG/ECHO state machine) for
   * the given component ID, or undefined if the component is not known.
   * Used by tests and future UI to read phase/schedule without violating the
   * private-state encapsulation in production paths.
   */
  getHcsr04State(compId: string): Hcsr04EngineState | undefined {
    return this.state.hcsr04.get(compId);
  }

  /**
   * Earliest pending physical/discrete event across sensors and delayed logic,
   * or null if none are scheduled. sim.worker.ts clamps each step's h so it
   * lands exactly on this instant
   * (guarded to h >= H_MIN) instead of letting an adaptive step grow past a
   * µs-scale edge — the NE555 sub-step crossing split (see `step()` above) is
   * the in-engine precedent for aligning a step boundary to a physical edge
   * rather than sampling it at whatever granularity the adaptive h happens
   * to be at. Unlike the 555 case (always exactly one step ahead), an hcsr04
   * echo can be scheduled tens of milliseconds out, so the clamp lives in the
   * OUTER batch loop rather than inside step() itself.
   */
  nextScheduledEventTime(): number | null {
    let next: number | null = null;
    const consider = (value: number): void => {
      if (Number.isFinite(value) && value > this.simTime && (next === null || value < next)) next = value;
    };
    for (const st of this.state.hcsr04.values()) {
      consider(st.echoRiseAt);
      consider(st.echoFallAt);
    }
    if (this._nextDigitalDueTimeCache !== null && this._nextDigitalDueTimeCache <= this.simTime) {
      this._digitalDueCacheDirty = true;
    }
    if (this._digitalDueCacheDirty) {
      this._nextDigitalDueTimeCache = null;
      for (const state of this.state.icState.values()) {
        for (const [key, value] of Object.entries(state)) {
          if (
            key.startsWith(DIGITAL_DUE_PREFIX)
            && Number.isFinite(value)
            && value > this.simTime
            && (this._nextDigitalDueTimeCache === null || value < this._nextDigitalDueTimeCache)
          ) {
            this._nextDigitalDueTimeCache = value;
          }
        }
      }
      this._digitalDueCacheDirty = false;
    }
    if (this._nextDigitalDueTimeCache !== null) consider(this._nextDigitalDueTimeCache);
    return next;
  }

  /**
   * S18b — hard cap on step size (comfortably under the fixed 250 µs
   * TRIG-fall-to-ECHO-rise delay) whenever the circuit contains any hcsr04,
   * or null otherwise. sim.worker.ts's runBatch applies this ALONGSIDE the
   * nextScheduledEventTime() clamp, and for a different reason: that clamp
   * only protects a step that begins AFTER a schedule is already known. The
   * very step in which a TRIG fall is detected and the schedule is BORN has
   * no such advance warning — nextScheduledEventTime() was null when that
   * step's h was chosen. Without this cap, a step larger than ~250 µs could
   * ALSO already be "due" for the freshly-created echoRiseAt by the time
   * the state pass computes it (this.simTime + h already past tFall + 250 µs),
   * silently skipping the rise straight to "active" a full step late and
   * corrupting the ECHO pulse width as measured by a real pulseIn/
   * time_pulse_us caller. Keeping every step under the 250 µs delay
   * guarantees echoRiseAt (>= stepStart + 250 µs, since tFall >= stepStart)
   * always exceeds this.simTime + h (<= stepStart + h < stepStart + 250 µs),
   * so the schedule is always seen "not yet due" in its birth step and the
   * outer nextScheduledEventTime() clamp gets a fair chance to react on a
   * later, appropriately-shrunk step.
   */
  hcsr04MaxStepH(): number | null {
    if (!this.circuit) return null;
    return this._hasHcsr04 ? HCSR04_MAX_STEP_H : null;
  }

  /**
   * S18b — drains HC-SR04 -> micro:bit ECHO bridge events queued since the
   * last call. See Hcsr04EngineState.pendingEchoEvent for why this lives
   * inside the per-component state map rather than a free-standing queue.
   */
  takeHcsr04EchoEvents(): Hcsr04EchoEvent[] {
    const out: Hcsr04EchoEvent[] = [];
    for (const st of this.state.hcsr04.values()) {
      if (st.pendingEchoEvent) {
        out.push(st.pendingEchoEvent);
        st.pendingEchoEvent = null;
      }
    }
    return out;
  }

  /**
   * Returns the committed IC state record for the given component ID, or
   * undefined if the component has no IC state.  Used primarily by tests to
   * inspect the state of ULN, buzzer, speaker, and sequential IC components
   * without violating the private-state encapsulation in production paths.
   */
  getIcState(compId: string): Record<string, number> | undefined {
    return this.state.icState.get(compId);
  }

  /** Per-board runtime state for onboard LED/button art. Only boards with an MCU are included. */
  getArduinoState(): Record<string, import("../messages.js").ArduinoState> {
    if (!this.circuit) return {};
    const result: Record<string, import("../messages.js").ArduinoState> = {};
    for (const comp of this._mcuComponents) {
      const mcu = this.state.arduinos.get(comp.id);
      const powered = this._mcuPowered(comp);
      // Onboard LED: Arduino = D13, Pico = GP25 (both surfaced via the d13High flag).
      const ledPin = comp.kind === "raspberry_pi_pico" ? "gp25" : "d13";
      const ledDrive = mcu ? mcu.pinDriveState(ledPin) : "input";
      result[comp.id] = {
        powered,
        d13High: ledDrive === "out-high",
        resetActive: false,
        txActivity: false,
        rxActivity: false,
      };
    }
    return result;
  }

  /**
   * Per-channel currents (A) for multi-junction LED parts.
   *
   * rgb_led:      [r, g, b]                   — indices 0-2
   * bicolor_led:  [a1, a2]                    — indices 0-1
   * seg7_cc/ca:   [a, b, c, d, e, f, g, dp]   — indices 0-7
   *
   * The renderer uses these to decide lit/brightness state instead of reading
   * raw net voltages, so the display reflects real diode-level current.
   */
  getElementChannelI(): Record<string, number[]> {
    if (!this.circuit) return {};
    const result: Record<string, number[]> = {};
    const x = this._lastX;
    if (!x) return result;
    const vt = this._junctionVt();
    const VtN = N_LED * vt;
    for (const comp of this.circuit.components) {
      if (comp.kind === "rgb_led" && !comp.params.burnt && !this._hasFailure(comp.id, "led_failed")) {
        const comKNode = this._pinNode(comp.id, "com_k");
        const channels: [string, string, number][] = [
          ["r_a", "vf_r", 1.8],
          ["g_a", "vf_g", 2.0],
          ["b_a", "vf_b", 3.0],
        ];
        const currents: number[] = [];
        const iRated = Number(comp.params.iRated ?? IRATED_LED);
        for (const [aPin, vfParam, vfDefault] of channels) {
          const Vf = this._junctionVf(Number(comp.params[vfParam] ?? vfDefault));
          const Is = shockleyIsFromVf(Vf, iRated, N_LED, vt);
          const vSat = Math.min(Math.max(40 * VtN, Vf + 5 * VtN), 80 * VtN);
          const vdCh = this._vAt(x, this._pinNode(comp.id, aPin)) - this._vAt(x, comKNode);
          currents.push(Math.max(0, shockleyDiodeCurrent(vdCh, Is, N_LED, vt, vSat)));
        }
        result[comp.id] = currents;
      } else if (comp.kind === "bicolor_led" && !comp.params.burnt && !this._hasFailure(comp.id, "led_failed")) {
        const kNode = this._pinNode(comp.id, "k");
        const iRated = Number(comp.params.iRated ?? IRATED_LED);
        const channels: [string, string, number][] = [
          ["a1", "vf1", 1.8],
          ["a2", "vf2", 2.0],
        ];
        const currents: number[] = [];
        for (const [aPin, vfParam, vfDefault] of channels) {
          const Vf = this._junctionVf(Number(comp.params[vfParam] ?? vfDefault));
          const Is = shockleyIsFromVf(Vf, iRated, N_LED, vt);
          const vSat = Math.min(Math.max(40 * VtN, Vf + 5 * VtN), 80 * VtN);
          const vd = this._vAt(x, this._pinNode(comp.id, aPin)) - this._vAt(x, kNode);
          currents.push(Math.max(0, shockleyDiodeCurrent(vd, Is, N_LED, vt, vSat)));
        }
        result[comp.id] = currents;
      } else if ((comp.kind === "seg7_cc" || comp.kind === "seg7_ca")) {
        // Per-segment diode currents in canonical order [a,b,c,d,e,f,g,dp].
        // The renderer indexes this array positionally to decide lit/brightness.
        const SEG7_CH = ["a", "b", "c", "d", "e", "f", "g", "dp"] as const;
        const commonAnode = comp.kind === "seg7_ca";
        const specs = specsForComponent(comp);
        const Vf = this._junctionVf(Number(comp.params.vf ?? specs?.vf ?? 2.0));
        const iRated = Number(comp.params.iRated ?? specs?.if_max ?? 0.02);
        const Is = shockleyIsFromVf(Vf, iRated, N_LED, vt);
        const vSat = Math.min(Math.max(40 * VtN, Vf + 5 * VtN), 80 * VtN);
        // buildNets bonds COM and COM2 to one package node. Prefer the canonical
        // COM label; the COM2 fallback supports malformed/legacy pin lists.
        let comN: number;
        if (!this._isOpenPin(comp.id, "com")) {
          comN = this._pinNode(comp.id, "com");
        } else if (!this._isOpenPin(comp.id, "com2")) {
          comN = this._pinNode(comp.id, "com2");
        } else {
          comN = -2; // sentinel: neither COM pin is wired — skip all segments
        }
        const currents: number[] = [];
        for (const seg of SEG7_CH) {
          if (comN === -2) {
            currents.push(0);
            continue;
          }
          const segN = this._pinNode(comp.id, seg);
          if (this._isOpenPin(comp.id, seg)) {
            currents.push(0);
            continue;
          }
          const [ai, ci] = commonAnode ? [comN, segN] : [segN, comN];
          const vd = this._vAt(x, ai) - this._vAt(x, ci);
          currents.push(Math.max(0, shockleyDiodeCurrent(vd, Is, N_LED, vt, vSat)));
        }
        result[comp.id] = currents;
      } else if (comp.kind === "resistor_array") {
        // Per-channel currents [ch1, ch2, ch3, ch4] — each channel is ai<->bi.
        // The renderer can use these to display individual channel loads.
        const rArrCh = Math.max(1, Number(comp.params.resistance ?? 10000));
        const chCurrents: number[] = [];
        for (let i = 1; i <= 4; i++) {
          if (this._hasFailure(comp.id, "resistor_overload", `ch${i}`)) {
            chCurrents.push(0);
            continue;
          }
          const va = this._vAt(x, this._pinNode(comp.id, `a${i}`));
          const vb = this._vAt(x, this._pinNode(comp.id, `b${i}`));
          chCurrents.push((va - vb) / rArrCh);
        }
        result[comp.id] = chCurrents;
      }
    }
    return result;
  }

  /** Per-component display state committed at the end of the last step. */
  getDisplayState(): Record<string, DisplayInfo> { return this._displayState; }

  // ─── Internal ─────────────────────────────────────────────────────────

  /** Net ID for the net that contains (compId, pinId), or null. */
  private _netIdForPin(compId: string, pinId: string): string | null {
    return this._pinToNetIndex.get(`${compId}\0${pinId}`) ?? null;
  }

  /**
   * Compare pre-step and post-step 555 states to find the earliest
   * threshold-crossing fraction within (0,1). Returns null if no 555 flipped.
   *
   * Derivation: voltage moves approximately linearly across the step, so
   *   frac = (V_target − V_pre) / (V_post − V_pre)
   * The frac is clamped to [0.02, 0.98] to keep both sub-steps non-trivial.
   */
  private _find555CrossingFrac(
    preNetV: Record<string, number>,
    preNe555s: Map<string, NE555EngineState>,
  ): number | null {
    let minFrac: number | null = null;

    for (const comp of this._ne555Components) {

      const prev = preNe555s.get(comp.id) ?? { outHigh: true };
      const next = this.state.ne555s.get(comp.id) ?? { outHigh: true };
      if (prev.outHigh === next.outHigh) continue;

      // Estimate VCC from pre-step value (assumed stable over one step).
      const vccId = this._netIdForPin(comp.id, "8");
      const vVcc = vccId ? (preNetV[vccId] ?? 5) : 5;

      let frac: number | null = null;

      if (prev.outHigh && !next.outHigh) {
        // RESET: THR (pin 6) crossed above 2/3·VCC.
        const thrId = this._netIdForPin(comp.id, "6");
        if (thrId) {
          const vPre = preNetV[thrId] ?? 0;
          const vPost = this.netV[thrId] ?? 0;
          const target = (2 / 3) * vVcc;
          const dv = vPost - vPre;
          if (Math.abs(dv) > 1e-10) frac = (target - vPre) / dv;
        }
      } else {
        // SET: TRIG (pin 2) crossed below 1/3·VCC.
        const trigId = this._netIdForPin(comp.id, "2");
        if (trigId) {
          const vPre = preNetV[trigId] ?? 0;
          const vPost = this.netV[trigId] ?? 0;
          const target = (1 / 3) * vVcc;
          const dv = vPost - vPre;
          if (Math.abs(dv) > 1e-10) frac = (target - vPre) / dv;
        }
      }

      if (frac !== null) {
        const clamped = Math.max(0.02, Math.min(0.98, frac));
        if (minFrac === null || clamped < minFrac) minFrac = clamped;
      }
    }

    return minFrac;
  }

  private _buildMatrix(): void {
    this.nodeIdx = new Map();
    let idx = 0;
    for (const n of this.nets) {
      this.nodeIdx.set(n.id, n.id === "gnd" ? -1 : idx++);
    }

    // Wave A6: internal solver nodes, allocated after every net node and
    // included in nodeCount so the node-row machinery (RSHUNT seed, gmin
    // overlay, _vAt bounds, the AC system's node loop) covers them without
    // any per-pass special case. They stay out of nodeIdx, so the netV
    // extraction — which iterates nets only — can never publish one.
    // Circuits without internal-node kinds allocate nothing here, keeping
    // every existing row index (and therefore every solve) bit-identical.
    this.internalNodeIdx = new Map();
    for (const c of this.circuit!.components) {
      const model = getDeviceModel(c.kind);
      if (!model?.internalNodes) continue;
      for (const name of model.internalNodes(c)) {
        this.internalNodeIdx.set(`${c.id}:${name}`, idx++);
      }
    }
    this.nodeCount = idx;

    this.vsrcIdx = new Map();
    let vsrcCount = 0;
    for (const c of this.circuit!.components) {
      // Wave A4: a registered kind owns its branch-row allocation. Rows are
      // claimed in the model's returned order at this component's position,
      // so allocation order — and therefore every row index — is unchanged
      // for kinds still handled by the chain below.
      const deviceModel = getDeviceModel(c.kind);
      if (deviceModel) {
        if (deviceModel.branchRows) {
          for (const rowKey of deviceModel.branchRows(c, this._deviceCtx)) {
            this.vsrcIdx.set(rowKey, this.nodeCount + vsrcCount++);
          }
        }
        continue;
      }
      if (isMcuBoardKind(c.kind) && Number(c.params.usb_power ?? 1) !== 0) {
        this.vsrcIdx.set(`${c.id}:usb`, this.nodeCount + vsrcCount++);
      } else if (c.kind === "microbit" && Number(c.params.power ?? 1) !== 0) {
        this.vsrcIdx.set(`${c.id}:usb`, this.nodeCount + vsrcCount++);
      }
    }

    this.mna = createLinearSystem(this.nodeCount + vsrcCount);
  }

  /**
   * Compile topology facts that are immutable until the next load(). The old
   * implementation rediscovered these facts by scanning every net (and, for
   * open pins, every wire plus a freshly allocated Set) on each stamp/read.
   * Keeping the exact legacy rules here makes the hot paths O(1) without
   * changing what counts as an externally connected package pin.
   */
  private _compileRuntimeMetadata(circuit: SimCircuit): void {
    this._componentById = new Map(circuit.components.map((component) => [component.id, component]));
    this._pinToNetIndex = new Map();
    this._pinNodeIndex = new Map();
    this._openPinKeys = new Set();
    this._thermalProfileCache.clear();
    this._batteryModelCache.clear();
    this._combinationalEvalCache.clear();
    this._nextDigitalDueTimeCache = null;
    this._digitalDueCacheDirty = true;
    this._ne555Components = circuit.components.filter((component) => component.kind === "ne555");
    this._hasNe555 = this._ne555Components.length > 0;
    this._hasHcsr04 = circuit.components.some((component) => component.kind === "hcsr04");
    this._mcuComponents = circuit.components.filter((component) => isMcuBoardKind(component.kind));
    // Wave A4: bucket membership is (engine const set) OR (registry hook
    // present). Migrated kinds remain listed in the sets, so the union is an
    // exact no-op for them; only a third-party registered kind can extend a
    // bucket. Order is still the circuit's component order in every case.
    this._stateUpdateComponents = circuit.components.filter((component) =>
      STATE_UPDATE_KINDS.has(component.kind)
      || getDeviceModel(component.kind)?.commitState !== undefined,
    );
    this._digitalUpdateComponents = circuit.components.filter((component) =>
      DIGITAL_UPDATE_KINDS.has(component.kind)
      || getDeviceModel(component.kind)?.updateDigital !== undefined,
    );
    this._failureUpdateComponents = circuit.components.filter((component) =>
      PASSIVE_FAILURE_UPDATE_KINDS.has(component.kind)
      || getDeviceModel(component.kind)?.updateFailures !== undefined
      || Boolean(specsForComponent(component)?.vcc_range),
    );
    this._thermalComponents = circuit.components.filter((component) => Boolean(this._thermalProfile(component)));
    this._staticIdealSourceIds = new Set(
      circuit.components
        .filter((component) =>
          STATIC_IDEAL_SOURCE_KINDS.has(component.kind)
          && component.pins.length >= 2
          && this.vsrcIdx.has(component.id),
        )
        .map((component) => component.id),
    );
    // Wave A4: registry staticStamp unions with the engine set the same way
    // (no-op for migrated kinds, extension point for third-party kinds).
    const isFullStaticStampKind = (kind: string): boolean =>
      FULL_STATIC_STAMP_KINDS.has(kind)
      || getDeviceModel(kind)?.staticStamp === true;
    this._staticStampComponents = circuit.components.filter((component) =>
      isFullStaticStampKind(component.kind),
    );
    this._dynamicStampComponents = circuit.components.filter((component) =>
      !isFullStaticStampKind(component.kind),
    );
    // Resolve each bucket's device-model pointers once per load (see the
    // field block's comment). Same registry lookup the passes used to do
    // per visit, so dispatch targets — and therefore float order — are
    // untouched; a kind registered after this load already could not join
    // the hook-derived buckets above, so caching adds no new staleness.
    const modelFor = (component: SimComponent): DeviceModel | undefined =>
      getDeviceModel(component.kind);
    this._stateUpdateModels = this._stateUpdateComponents.map(modelFor);
    this._digitalUpdateModels = this._digitalUpdateComponents.map(modelFor);
    this._failureUpdateModels = this._failureUpdateComponents.map(modelFor);
    this._staticStampModels = this._staticStampComponents.map(modelFor);
    this._dynamicStampModels = this._dynamicStampComponents.map(modelFor);
    this._elementCurrentModels = circuit.components.map(modelFor);

    for (const net of this.nets) {
      const row = this.nodeIdx.get(net.id) ?? -1;
      const componentIds = new Set(net.pins.map(([componentId]) => componentId));
      const pinsByComponent = new Map<string, Set<string>>();

      for (const [componentId, pinId] of net.pins) {
        const key = `${componentId}\0${pinId}`;
        this._pinToNetIndex.set(key, net.id);
        this._pinNodeIndex.set(key, row);
        let pinIds = pinsByComponent.get(componentId);
        if (!pinIds) {
          pinIds = new Set();
          pinsByComponent.set(componentId, pinIds);
        }
        pinIds.add(pinId);
      }

      // A multi-component net is externally connected for every pin on it.
      if (componentIds.size > 1) continue;

      const componentId = net.pins[0]?.[0];
      if (!componentId) continue;
      const pinIds = pinsByComponent.get(componentId)!;
      const hasExplicitPackageWire = circuit.wires.some((wire) =>
        (wire.from_component === componentId && pinIds.has(wire.from_pin))
        || (wire.to_component === componentId && pinIds.has(wire.to_pin)),
      );
      if (!hasExplicitPackageWire) {
        for (const pinId of pinIds) this._openPinKeys.add(`${componentId}\0${pinId}`);
      }
    }
  }

  /** Compile RSHUNT plus topology-only ideal-source incidence once per load. */
  private _rebuildStaticStampBase(): void {
    if (!this.circuit || !this.mna) return;
    this.mna.clear();
    for (let row = 0; row < this.nodeCount; row++) {
      this.mna.add(row, row, NODE_RSHUNT_G);
    }
    for (const componentId of this._staticIdealSourceIds) {
      const component = this._componentById.get(componentId);
      const branch = this.vsrcIdx.get(componentId);
      if (!component || branch === undefined) continue;
      stampVSource(
        this.mna,
        this._pinNode(component.id, component.pins[0]!.id),
        this._pinNode(component.id, component.pins[1]!.id),
        branch,
        0,
      );
    }
    this._stampAll(
      1e-12,
      new Float64Array(this.mna.size),
      this._staticStampComponents,
    );
    this.mna.captureBase();
    this._staticStampSignature = this._currentStaticStampSignature();
  }

  /**
   * Direct callers may mutate a loaded component's params object before the
   * next step (tests and analysis tools rely on this). Track only values used
   * by compiled full stamps so that path remains exact without re-stamping the
   * common unchanged case.
   */
  private _currentStaticStampSignature(): string {
    const values: string[] = [];
    for (let i = 0; i < this._staticStampComponents.length; i++) {
      const component = this._staticStampComponents[i]!;
      values.push(component.id, component.kind);
      // Wave A4: every static-stamp kind's per-kind signature values live on
      // its registered model (registration enforces staticStamp implies
      // staticSignature, so a static component can only lack values here if
      // it entered the bucket via FULL_STATIC_STAMP_KINDS without a model —
      // impossible once the PASSIVES cohort, which covers that whole set,
      // is registered). The hook pushes straight into the accumulator (out-
      // parameter contract) so this once-per-step scan allocates nothing
      // per component, exactly like the old per-kind switch; value order is
      // still that switch's order.
      const deviceModel = this._staticStampModels[i];
      if (deviceModel?.staticSignature) {
        deviceModel.staticSignature(this._deviceCtx, component, values);
      }
    }
    return values.join("\0");
  }

  private _refreshStaticStampBaseIfNeeded(): void {
    if (this._currentStaticStampSignature() !== this._staticStampSignature) {
      this._rebuildStaticStampBase();
    }
  }

  private _stampIndependentVoltageSource(
    component: SimComponent,
    branch: number,
    voltage: number,
  ): void {
    if (this._staticIdealSourceIds.has(component.id)) {
      this.mna.addB(branch, voltage);
      return;
    }
    stampVSource(
      this.mna,
      this._pinNode(component.id, component.pins[0]!.id),
      this._pinNode(component.id, component.pins[1]!.id),
      branch,
      voltage,
    );
  }

  /**
   * Source-stepping homotopy scale (Wave A3), applied at stamp time to every
   * INDEPENDENT source magnitude. Canonical list of scaled kinds:
   *   - voltage_source / clock / clock_gen / pulse_source / pulse_gen /
   *     signal_gen waveform voltages
   *   - current_source programmed current (Wave A7 — devices/sources.ts
   *     routes both its stamp and its published element current through
   *     this helper so I sources relax on the same ladder as V sources)
   *   - battery_pack open-circuit voltage (internal resistance stays
   *     unscaled — it is an impedance, not a drive)
   *   - bench_psu voltage setpoint (iLimit stays unscaled — it is a
   *     compliance bound enforced by the committed CC regime, not an
   *     independent drive; the regime follows the scaled setpoint naturally)
   *   - MCU/micro:bit USB rail sources (arduino_uno, arduino_nano,
   *     raspberry_pi_pico, microbit vcc branches)
   * Deliberately NOT scaled:
   *   - dependent/behavioral outputs (op-amp, comparator, linear_reg, lm317,
   *     dcdc_converter, TL431, logic and MCU pin Thevenin stages, 555 output)
   *     — they are referenced to their package rails or gated by input
   *     UVLO/power checks, so they collapse with their scaled supplies;
   *   - load-shaped stamps (board idle-draw resistors, IC quiescent current
   *     sinks, companion history sources) — scaling a load or a state memory
   *     would distort the system instead of relaxing its drives.
   */
  private _independentSourceMagnitude(value: number): number {
    // Exact identity (not a multiply by 1) when the homotopy is inert, so the
    // default stamp path stays bit-identical to the pre-A3 engine.
    return this._homotopySourceScale === 1 ? value : value * this._homotopySourceScale;
  }

  /**
   * Waveform sample instant for the solve in flight (Wave A3). Ordinary
   * stepping samples the interval endpoint, simTime + h — the identical
   * float expression as before, so the default path stays bit-exact. While
   * _dcPhysicalTimeHold is set, sampling is pinned to the OP instant
   * (simTime + DC_OP_COMPANION_H) no matter what companion/pseudo interval
   * the solve uses: stage d's backward-Euler pseudo-steps grow h to 1 ms of
   * NON-physical time, and sampling there would stamp — and let
   * _updateDigitalState latch — a source phase up to 1 ms past the instant
   * the operating point is being computed for (e.g. a 500 Hz clock read in
   * its opposite half-cycle). Every clock/pulse/signal-gen evaluation,
   * stamp-side and post-solve, must route through this one helper so the
   * stamped source and its readback never disagree.
   */
  private _waveformSampleTime(h: number): number {
    return this.simTime + (this._dcPhysicalTimeHold ? DC_OP_COMPANION_H : h);
  }

  private _pinNode(compId: string, pinId: string): number {
    return this._pinNodeIndex.get(`${compId}\0${pinId}`) ?? -1;
  }

  private _isOpenPin(compId: string, pinId: string): boolean {
    const key = `${compId}\0${pinId}`;
    return !this._pinNodeIndex.has(key) || this._openPinKeys.has(key);
  }

  /**
   * S18b — if `comp`'s ECHO pin shares a net with a micro:bit edge pin, return
   * the bridge target (board component id + SIM-form pin id). Used the instant
   * an echo is scheduled (TRIG falls) to queue the micro:bit worker event —
   * see Hcsr04EngineState.pendingEchoEvent.
   */
  private _hcsr04MicrobitBridge(comp: SimComponent): { boardComponentId: string; pinId: string } | null {
    if (!this.circuit) return null;
    const net = this.nets.find((n) => n.pins.some(([c, p]) => c === comp.id && p === "echo"));
    if (!net) return null;
    for (const [c, p] of net.pins) {
      if (c === comp.id) continue;
      const other = this._componentById.get(c);
      if (other?.kind === "microbit" && MICROBIT_EDGE_PIN_TO_SIM[p]) {
        return { boardComponentId: c, pinId: MICROBIT_EDGE_PIN_TO_SIM[p] };
      }
    }
    return null;
  }

  private _arduinoPowered(comp: SimComponent, xGuess?: Float64Array): boolean {
    const vccNode = this._pinNode(comp.id, "5v");
    const gndNode = this._pinNode(comp.id, "gnd");
    const vcc = xGuess ? this._vAt(xGuess, vccNode) : this._netVoltageForPin(comp.id, "5v");
    const gnd = xGuess ? this._vAt(xGuess, gndNode) : this._netVoltageForPin(comp.id, "gnd");
    return vcc - gnd > 3.0;
  }

  /** Whether an MCU board is powered enough to run. The Pico is USB(VBUS)-powered
   *  (its 3V3 pin is an output, not a supply input), so use the USB switch. */
  private _mcuPowered(comp: SimComponent, xGuess?: Float64Array): boolean {
    if (comp.kind === "raspberry_pi_pico") {
      return Number(comp.params.usb_power ?? 1) !== 0;
    }
    return this._arduinoPowered(comp, xGuess);
  }

  private _icPowerInfo(comp: SimComponent, x: Float64Array): IcPowerInfo {
    const specs = specsForComponent(comp);
    const { vccPin, gndPin } = supplyPinsForComponent(comp);
    const fallbackV = Number(comp.params.vcc ?? specs?.vcc_range?.nominal ?? 5);

    if (!vccPin || !gndPin) {
      return {
        powered: true,
        vccPin,
        gndPin,
        vcc: fallbackV,
        gnd: 0,
        vSupply: fallbackV,
        specs,
        thresholds: thresholdsFor(specs, fallbackV),
        outputResistance: GATE_R_OUT,
        propagationDelay: propagationDelayForComponent(comp),
      };
    }

    const vccNode = this._pinNode(comp.id, vccPin);
    const gndNode = this._pinNode(comp.id, gndPin);
    const vcc = this._vAt(x, vccNode);
    const gnd = this._vAt(x, gndNode);
    const vSupply = vcc - gnd;
    const minV = specs?.vcc_range?.min ?? 2.0;
    const maxV = specs?.vcc_range?.max ?? 18.0;
    const suppliesConnected = !this._isOpenPin(comp.id, vccPin) && !this._isOpenPin(comp.id, gndPin);
    const powered =
      suppliesConnected &&
      vSupply >= minV * 0.9 &&
      vSupply <= maxV * 1.1;
    const ioMax = specs?.io_max;
    const outputResistance =
      ioMax && ioMax > 0 && vSupply > 0
        ? Math.max(GATE_R_OUT, Math.min(1000, vSupply / (ioMax * 4)))
        : GATE_R_OUT;

    return {
      powered,
      vccPin,
      gndPin,
      vcc,
      gnd,
      vSupply: Math.max(0, vSupply),
      specs,
      thresholds: thresholdsFor(specs, Math.max(0, vSupply)),
      outputResistance,
      propagationDelay: propagationDelayForComponent(comp),
    };
  }

  private _floatingLogicHigh(comp: SimComponent, pinId: string, power: IcPowerInfo): boolean {
    const bucket = Math.floor((this.simTime + 1e-12) / 0.01);
    const n = hash32(`${comp.id}:${pinId}:${bucket}`) / 0xffffffff;
    const family = power.specs?.logic_family ?? "";
    const pHigh = family.includes("TTL") ? 0.7 : 0.5;
    return n < pHigh;
  }

  private _logicHigh(comp: SimComponent, pinId: string, x: Float64Array, power: IcPowerInfo): boolean {
    if (this._isOpenPin(comp.id, pinId)) return this._floatingLogicHigh(comp, pinId, power);
    const v = this._vAt(x, this._pinNode(comp.id, pinId)) - power.gnd;
    if (v >= power.thresholds.vih) return true;
    if (v <= power.thresholds.vil) return false;
    return this._floatingLogicHigh(comp, `${pinId}:undefined`, power);
  }

  /**
   * Like _logicHigh but respects IC_OPEN_HIGH_PINS: pins listed there resolve
   * deterministically HIGH when unconnected, instead of using the floating model.
   * Use for direct pin reads on ICs whose control inputs have on-chip pull-ups.
   */
  private _logicHighH(comp: SimComponent, pinId: string, x: Float64Array, power: IcPowerInfo): boolean {
    const openHighPins = IC_OPEN_HIGH_PINS[comp.kind];
    if (openHighPins?.has(pinId) && this._isOpenPin(comp.id, pinId)) return true;
    return this._logicHigh(comp, pinId, x, power);
  }

  /**
   * Schmitt-trigger hysteresis for parts whose catalog spec carries v_t_plus /
   * v_t_minus (e.g. 74HC14).  The thresholds are defined at vcc_range.nominal
   * and scale linearly with the actual supply; state (last stable level) is
   * persisted in icState under "schmitt_<pinId>".
   *
   * commit = false (default): read prevHigh for the dead band, compute nextHigh,
   *   but do NOT write back to icState. This is the correct behaviour during
   *   Newton-iteration stamping: intermediate non-physical xGuess values crossing
   *   a threshold must not latch wrong state (worst on the canonical 74HC14 RC
   *   relaxation oscillator).
   * commit = true: persist nextHigh only when it differs from prevHigh. The
   *   post-solve _updateDigitalState path (converged x) is the only caller that
   *   passes commit = true.
   *
   * Only called for pins that belong to a part with Schmitt thresholds —
   * all other parts use the unmodified _logicHigh path.
   */
  private _schmittLogicHigh(
    comp: SimComponent,
    pinId: string,
    x: Float64Array,
    power: IcPowerInfo,
    commit: boolean,
  ): boolean {
    if (this._isOpenPin(comp.id, pinId)) return this._floatingLogicHigh(comp, pinId, power);

    const v = this._vAt(x, this._pinNode(comp.id, pinId)) - power.gnd;
    const vi = power.specs?.v_input;
    const nominal = power.specs?.vcc_range?.nominal ?? 5;

    // Scaling factor: thresholds in catalog are at nominal; scale to actual supply.
    const scale = nominal > 0 ? power.vSupply / nominal : 1;
    const vtPlus  = (vi?.v_t_plus  ?? 2.7) * scale;
    const vtMinus = (vi?.v_t_minus ?? 1.6) * scale;

    const stKey = `schmitt_${pinId}`;
    const st = this.state.icState.get(comp.id) ?? {};
    // Default initial state: low (0).
    const prevHigh = (st[stKey] ?? 0) >= 0.5;

    let nextHigh: boolean;
    if (v >= vtPlus) {
      nextHigh = true;
    } else if (v <= vtMinus) {
      nextHigh = false;
    } else {
      // Dead band: hold previous state (that is the hysteresis).
      nextHigh = prevHigh;
    }

    // Only persist to icState when commit is true (post-solve, converged x).
    // During Newton iterations commit is false — reading prevHigh for the dead
    // band is still correct but we must not latch intermediate xGuess crossings.
    if (commit && nextHigh !== prevHigh) {
      st[stKey] = nextHigh ? 1 : 0;
      this.state.icState.set(comp.id, st);
    }

    return nextHigh;
  }

  private _logicPinMap(
    comp: SimComponent,
    x: Float64Array,
    power: IcPowerInfo,
    commit = false,
  ): Record<string, number> {
    const useSchmitt = power.specs?.v_input?.v_t_plus != null;
    const openHighPins = IC_OPEN_HIGH_PINS[comp.kind];
    const pinV: Record<string, number> = {};
    for (const p of comp.pins) {
      let high: boolean;
      if (openHighPins?.has(p.id) && this._isOpenPin(comp.id, p.id)) {
        // Pin has an internal pull-up on the physical chip: resolve HIGH
        // deterministically when left unconnected rather than using the
        // seeded TTL floating model which can randomly assert active-low controls.
        high = true;
      } else if (useSchmitt) {
        high = this._schmittLogicHigh(comp, p.id, x, power, commit);
      } else {
        high = this._logicHigh(comp, p.id, x, power);
      }
      pinV[p.id] = high ? 1 : 0;
    }
    return pinV;
  }

  private _shouldTreatOpenPinAsDigitalInput(comp: SimComponent, pinId: string): boolean {
    const fn = partFor(comp)?.pin_layout.find((p) => p.id === pinId)?.function;
    if (fn && FLOATING_INPUT_FUNCTIONS.has(fn)) return true;
    return icPinMap(comp.kind).some((pin) =>
      pin.id === pinId && (pin.role === "input" || pin.role === "clock" || pin.role === "control")
    );
  }

  private _floatingInputResistance(power: IcPowerInfo): number {
    const family = power.specs?.logic_family ?? "";
    // TTL inputs present a low impedance due to input transistor base current;
    // CMOS (HC-CMOS and CMOS-4000) inputs are gate-oxide high-Z.
    return family.includes("TTL") ? 1_000_000 : 10_000_000;
  }

  private _stampFloatingDigitalInputs(
    comp: SimComponent,
    _x: Float64Array,
    power: IcPowerInfo,
  ): void {
    const r = this._floatingInputResistance(power);
    const g = 1 / r;
    const openHighPins = IC_OPEN_HIGH_PINS[comp.kind];
    for (const pin of comp.pins) {
      if (!this._isOpenPin(comp.id, pin.id)) continue;
      if (!this._shouldTreatOpenPinAsDigitalInput(comp, pin.id)) continue;
      // Pins with on-chip pull-ups (e.g. 74LS47 /LT, /RBI, /RBO) resolve HIGH
      // deterministically. Skip the seeded weak-pull stamp — nothing else reads
      // these open nets and seeding them LOW can assert an active-low control.
      if (openHighPins?.has(pin.id)) continue;
      const node = this._pinNode(comp.id, pin.id);
      if (node < 0) continue;
      const high = this._floatingLogicHigh(comp, pin.id, power);
      const target = power.gnd + (high ? power.vSupply : 0);
      this.mna.add(node, node, g);
      this.mna.addB(node, target * g);
    }
  }

  private _combinationalOutputs(
    comp: SimComponent,
    xGuess: Float64Array,
    power: IcPowerInfo,
    commit = false,
  ): Record<string, boolean> {
    const pinValues = this._logicPinMap(comp, xGuess, power, commit);
    let signature = "";
    for (const pin of comp.pins) signature += pinValues[pin.id] >= 0.5 ? "1" : "0";
    const cached = this._combinationalEvalCache.get(comp.id);
    if (cached?.signature === signature) {
      this.digitalEvaluationReuseCount += 1;
      return cached.outputs;
    }
    const outputs =
      (comp.kind === "74ls157" || comp.kind === "74ls245") && pinValues["/oe"] >= 0.5
        ? {}
        : evalCombinationalIC(comp.kind, pinValues, 0.5);
    this.digitalEvaluationCount += 1;
    this._combinationalEvalCache.set(comp.id, { signature, outputs });
    return outputs;
  }

  private _stampDigitalOutput(
    comp: SimComponent,
    pinId: string,
    high: boolean,
    power: IcPowerInfo,
  ): void {
    const node = this._pinNode(comp.id, pinId);
    if (node < 0) return;
    const outputResistance = power.outputResistance;
    const railPin = high ? power.vccPin : power.gndPin;
    if (railPin && !this._isOpenPin(comp.id, railPin)) {
      // A physical output transistor connects the pin to its package rail.
      // Stamping a resistor to that rail, rather than a hidden absolute Norton
      // source, makes load current appear on VCC/GND and follows rail sag.
      stampResistor(this.mna, node, this._pinNode(comp.id, railPin), outputResistance);
      return;
    }

    // Supply-less behavioural parts retain a local-reference fallback. ICs
    // with explicit rails never reach this path while powered.
    const target = power.gnd + (high ? power.vSupply : 0);
    const g = 1 / outputResistance;
    this.mna.add(node, node, g);
    this.mna.addB(node, target * g);
  }

  /**
   * Thevenin output referenced to real low/high rails. A fractional target is
   * represented by complementary conductances whose sum is 1/R, conserving
   * current at both rails while producing Vlow + fraction*(Vhigh-Vlow).
   */
  private _stampRailReferencedOutput(
    node: number,
    lowRail: number,
    highRail: number,
    fraction: number,
    outputResistance: number,
  ): void {
    if (node < 0) return;
    const f = Math.max(0, Math.min(1, fraction));
    if (f > 1e-12) stampResistor(this.mna, node, highRail, outputResistance / f);
    if (f < 1 - 1e-12) stampResistor(this.mna, node, lowRail, outputResistance / (1 - f));
  }

  /**
   * Board-level regulator/MCU/indicator load.  A nominal-current resistor keeps
   * the stamp linear and naturally reduces draw during brown-out. Users can
   * override `idleCurrent` for a particular firmware/peripheral configuration.
   */
  private _stampBoardIdleLoad(
    comp: SimComponent,
    vccPin: string,
    gndPin: string,
    nominalV: number,
    defaultCurrentA: number,
  ): void {
    const currentA = Math.max(0, Number(comp.params.idleCurrent ?? defaultCurrentA));
    if (currentA <= 0) return;
    const resistance = Math.max(0.01, nominalV / currentA);
    stampResistor(
      this.mna,
      this._pinNode(comp.id, vccPin),
      this._pinNode(comp.id, gndPin),
      resistance,
    );
  }

  private _delayedOutputLevel(comp: SimComponent, pinId: string, immediate: boolean): boolean {
    const st = this.state.icState.get(comp.id);
    const stored = st?.[`${DIGITAL_DELAY_PREFIX}${pinId}`];
    return stored === undefined ? immediate : stored >= 0.5;
  }

  private _updateDelayedOutputs(
    comp: SimComponent,
    immediate: Record<string, boolean>,
    h: number,
    power: IcPowerInfo,
  ): Record<string, boolean> {
    const st = this.state.icState.get(comp.id) ?? {};
    const now = this.simTime + h;
    const visible: Record<string, boolean> = {};
    let dueChanged = false;

    for (const [pinId, immediateLevel] of Object.entries(immediate)) {
      const valueKey = `${DIGITAL_DELAY_PREFIX}${pinId}`;
      const pendingKey = `${DIGITAL_PENDING_PREFIX}${pinId}`;
      const dueKey = `${DIGITAL_DUE_PREFIX}${pinId}`;
      const dueBefore = st[dueKey];

      if (st[valueKey] === undefined) {
        st[valueKey] = immediateLevel ? 1 : 0;
      }

      if (st[dueKey] !== undefined && now >= st[dueKey]) {
        st[valueKey] = st[pendingKey] ?? st[valueKey] ?? 0;
        delete st[pendingKey];
        delete st[dueKey];
      }

      const committed = (st[valueKey] ?? 0) >= 0.5;
      if (committed !== immediateLevel) {
        if (power.propagationDelay <= 0) {
          st[valueKey] = immediateLevel ? 1 : 0;
          delete st[pendingKey];
          delete st[dueKey];
        } else if (st[pendingKey] === undefined || (st[pendingKey] >= 0.5) !== immediateLevel) {
          st[pendingKey] = immediateLevel ? 1 : 0;
          st[dueKey] = now + power.propagationDelay;
        }
      } else {
        delete st[pendingKey];
        delete st[dueKey];
      }

      visible[pinId] = (st[valueKey] ?? 0) >= 0.5;
      if (!Object.is(dueBefore, st[dueKey])) dueChanged = true;
    }

    // A `delay:` slot is the canonical record of a DRIVEN output stage (the
    // small-signal AC default and the pin readouts key off it), so a pin
    // that left `immediate` — a 74LS245/157 whose /OE released the bus — must
    // drop its value/pending/due slots rather than advertise a stage that the
    // transient stamp no longer writes. Re-enabling later re-seeds from the
    // immediate level, exactly like the first driven step.
    for (const key of Object.keys(st)) {
      let pinId: string | null = null;
      if (key.startsWith(DIGITAL_DELAY_PREFIX)) {
        pinId = key.slice(DIGITAL_DELAY_PREFIX.length);
      } else if (key.startsWith(DIGITAL_PENDING_PREFIX)) {
        pinId = key.slice(DIGITAL_PENDING_PREFIX.length);
      } else if (key.startsWith(DIGITAL_DUE_PREFIX)) {
        pinId = key.slice(DIGITAL_DUE_PREFIX.length);
      }
      if (pinId === null || pinId in immediate) continue;
      if (key.startsWith(DIGITAL_DUE_PREFIX)) dueChanged = true;
      delete st[key];
    }

    this.state.icState.set(comp.id, st);
    if (dueChanged) this._digitalDueCacheDirty = true;
    return visible;
  }

  private _netVoltageForPin(compId: string, pinId: string): number {
    const netId = this._netIdForPin(compId, pinId);
    return netId ? (this.netV[netId] ?? 0) : 0;
  }

  /** Voltage at a matrix row (or 0 for ground / unmapped). */
  private _vAt(x: Float64Array, row: number): number {
    if (row < 0 || row >= this.nodeCount) return 0;
    return x[row] ?? 0;
  }

  private _hasFailure(compId: string, kind: SimFailureKind, pinId = ""): boolean {
    return this.state.failures.has(failureKey(kind, compId, pinId));
  }

  private _componentFailed(compId: string, kind?: SimFailureKind): boolean {
    for (const failure of this.state.failures.values()) {
      if (failure.componentId === compId && (kind === undefined || failure.kind === kind)) return true;
    }
    return false;
  }

  private _recordAccumulatedStress(
    key: string,
    stressRate: number,
    recoveryRate: number,
    h: number,
    threshold: number,
    makeFailure: () => SimFailure,
  ): void {
    if (this.state.failures.has(key)) return;
    const previous = this.state.failureStress.get(key) ?? 0;
    const stress = Math.max(
      0,
      previous + (stressRate > 0 ? stressRate : -Math.max(0, recoveryRate)) * h,
    );
    if (stress >= threshold) {
      this.state.failures.set(key, makeFailure());
      this.state.failureStress.delete(key);
    } else if (stress > 0) {
      this.state.failureStress.set(key, stress);
    } else {
      this.state.failureStress.delete(key);
    }
  }

  private _solve(h: number): void {
    this._refreshStaticStampBaseIfNeeded();
    this.electricalSolveCount += 1;
    const started = nowMs();
    if (this.mna.size === 0) {
      this.netV = { gnd: 0 };
      this.elementI = {};
      this.lastIters = 0;
      this.lastConverged = true;
      this.lastSolveUs = 0;
      this.lastMatrixSize = 0;
      this.lastMatrixSingular = false;
      this.lastMatrixIllConditioned = false;
      this.lastRelativeResidual = 0;
      return;
    }
    this.lastMatrixSize = this.mna.size;

    // Reset pnjlim per-junction history so Newton iter 1 sees an empty map
    // → falls back to `vRaw` (the previous time-step's converged value via
    // xInit) → pnjlim is a no-op on iter 1, then damps iter 2+ against the
    // limited iter-1 value. This is the standard SPICE behaviour.
    this._vPrevBJT.clear();
    this._currentLimitComplianceClamps.clear();
    this._currentLimitCurrentBranchTried.clear();
    this._currentLimitEntryClamps.clear();
    this._regulatorHeadroomBranchTried.clear();
    this._regulatorHeadroomSelections.clear();

    // Initial guess: last-step node voltages (zero vector on first solve).
    const xInit = new Float64Array(this.mna.size);
    for (const [netId, row] of this.nodeIdx) {
      if (row >= 0 && row < this.nodeCount) {
        xInit[row] = this.netV[netId] ?? 0;
      }
    }
    // Wave A6: internal solver rows warm-start from the previous solution
    // the same way net rows warm-start from netV — netV cannot carry them
    // (they are not nets), and a cold zero guess would re-walk the pnjlim
    // ladder every step for models with internal junction nodes (the
    // optocoupler base). _lastX is committed state (snapshot/restore carry
    // it), so rollback replays see the identical guess; the size guard
    // covers the load() boundary, where _lastX may still have the previous
    // topology's dimension. Empty map = existing circuits = no-op.
    if (this.internalNodeIdx.size > 0 && this._lastX && this._lastX.length === this.mna.size) {
      for (const row of this.internalNodeIdx.values()) {
        xInit[row] = this._lastX[row] ?? 0;
      }
    }

    const result = solveNonlinear({
      size: this.mna.size,
      xInit,
      // Cold/low-temperature junctions can require more than the generic
      // 25 pnjlim iterations when starting from an all-zero operating point.
      // Give the physical solve enough runway instead of relying on a rejected
      // iterate to leak through as the next step's warm start. Operating-point
      // ladder solves raise the cap (see DC_OP_NEWTON_MAX_ITER); the null
      // default keeps every transient solve at the historical 50 exactly.
      maxIter: this._newtonMaxIterOverride ?? 50,
      clear: () => this.mna.resetToBase(),
      stamp: (xGuess) => {
        this._junctionLimitedThisIteration = false;
        this._stampAll(h, xGuess);
      },
      solve: () => this.mna.solve(),
      limitedThisIteration: () => this._junctionLimitedThisIteration,
    });

    this.lastIters = result.iters;
    const solveInfo = this.mna.lastSolveInfo;
    this.lastMatrixSingular = solveInfo.singular;
    this.lastMatrixIllConditioned = solveInfo.illConditioned;
    this.lastRelativeResidual = solveInfo.relativeResidual;
    const linearSolveHealthy =
      !solveInfo.singular &&
      !solveInfo.nonFinite &&
      solveInfo.relativeResidual <= 1e-8;
    this.lastConverged = result.converged && linearSolveHealthy;
    this.lastSolveUs = Math.max(0, (nowMs() - started) * 1000);

    // A Newton or linear failure is a rejected trial, not a physical interval.
    // Keep the last trusted electrical, dynamic, thermal, failure, and display
    // state intact for direct callers as well as the worker's retry path.
    if (!this.lastConverged) return;

    const x = result.x;

    // Extract net voltages from solution
    const netV: Record<string, number> = { gnd: 0 };
    for (const [netId, row] of this.nodeIdx) {
      netV[netId] = row >= 0 && row < this.nodeCount ? (x[row] ?? 0) : 0;
    }
    this.netV = netV;

    const previousMosfetGates = new Map(this.state.mosfetGates);
    this._lastX = x;
    this._updateState(h, x);
    // Homotopy rung solves hold digital state: their voltages are overlay
    // artifacts, and a committed phantom clock edge would survive ladder
    // success (only total failure restores the entry snapshot). See
    // _homotopyRungConverged for the full rationale.
    if (!this._dcDigitalHold) this._updateDigitalState(h, x);
    this._updateElementI(h, x, previousMosfetGates);
    // Operating-point solves advance no physical time, so the h-scaled
    // physical integrators hold: without this, stage d's pseudo-steps feed
    // up to ~0.5 s of pseudo-time into package heating and stress
    // accumulators — enough to latch a permanent component failure — while
    // simTime stands still. Held state also keeps the stamped topology
    // fixed across the ladder (a failure latching mid-ladder would change
    // the system under the continuation's feet).
    if (!this._dcPhysicalTimeHold) {
      this._updatePackageThermalState(h, x);
      this._updateFailureStates(h, x);
    }
  }

  /**
   * Stamp every element into the (already-cleared + RSHUNT-seeded) MNA
   * using `xGuess` as the linearisation point for non-linear elements.
   * Linear elements ignore `xGuess` and stamp the same values every
   * iteration.
   */
  private _stampAll(
    h: number,
    xGuess: Float64Array,
    components: readonly SimComponent[] = this._dynamicStampComponents,
  ): void {
    // Wave A3 gmin homotopy overlay — dynamic pass only. The static-base
    // rebuild routes through this method too (with _staticStampComponents);
    // stamping the overlay there would bake it into the captured base, where
    // it would silently outlive the homotopy ladder. Keying on the dynamic
    // component list keeps the overlay per-solve, mirroring the RSHUNT loop.
    if (this._homotopyGminG > 0 && components === this._dynamicStampComponents) {
      for (let row = 0; row < this.nodeCount; row++) {
        this.mna.add(row, row, this._homotopyGminG);
      }
    }
    // Only the two compiled bucket lists are ever passed here; their model
    // pointers were resolved at load. The registry fallback keeps any other
    // (hypothetical) list correct rather than silently misdispatching.
    const models = components === this._dynamicStampComponents
      ? this._dynamicStampModels
      : components === this._staticStampComponents
        ? this._staticStampModels
        : null;
    for (let i = 0; i < components.length; i++) {
      const comp = components[i]!;
      // Wave A4: registered kinds stamp through their device model at this
      // component's exact position in the pass — same list, same order, same
      // floats — and unmigrated kinds fall through to the switch below.
      const deviceModel = models ? models[i] : getDeviceModel(comp.kind);
      if (deviceModel?.stamp) {
        deviceModel.stamp(this._deviceCtx, comp, xGuess, h);
        continue;
      }
      switch (comp.kind) {
        case "arduino_uno":
        case "arduino_nano": {
          const usbK = this.vsrcIdx.get(`${comp.id}:usb`);
          if (usbK !== undefined && Number(comp.params.usb_power ?? 1) !== 0) {
            stampVSource(
              this.mna,
              this._pinNode(comp.id, "5v"),
              this._pinNode(comp.id, "gnd"),
              usbK,
              this._independentSourceMagnitude(Number(comp.params.vcc ?? 5)),
            );
          }
          this._stampBoardIdleLoad(
            comp,
            "5v",
            "gnd",
            Number(comp.params.vcc ?? 5),
            MCU_BOARD_IDLE_CURRENT_A[comp.kind],
          );
          const mcu = this.state.arduinos.get(comp.id);
          if (!mcu) break;
          if (!this._arduinoPowered(comp, xGuess)) break;
          const vccNode = this._pinNode(comp.id, "5v");
          const gndNode = this._pinNode(comp.id, "gnd");
          for (const pinId of mcu.ioPins) {
            const drive = mcu.pinDriveState(pinId);
            const node = this._pinNode(comp.id, pinId);
            if (drive === "input") continue;
            if (drive === "input-pullup") {
              // Pull to the actual board supply node, not an ideal fixed
              // voltage, so external-supply sag and power-off remain physical.
              stampResistor(this.mna, node, this._pinNode(comp.id, "5v"), AVR_PULLUP_R);
              continue;
            }
            if (drive === "input-pulldown") {
              stampResistor(this.mna, node, this._pinNode(comp.id, "gnd"), AVR_PULLUP_R);
              continue;
            }
            if (node < 0) continue;
            this._stampRailReferencedOutput(
              node,
              gndNode,
              vccNode,
              drive === "out-high" ? 1 : 0,
              GATE_R_OUT,
            );
          }
          break;
        }

        case "raspberry_pi_pico": {
          // 3V3(OUT) sources 3.3 V when USB-powered — stamp it so a wired supply
          // rail works. (Allocation of this branch row is gated on usb_power too.)
          const v3K = this.vsrcIdx.get(`${comp.id}:usb`);
          if (v3K !== undefined && Number(comp.params.usb_power ?? 1) !== 0) {
            stampVSource(
              this.mna,
              this._pinNode(comp.id, "3v3"),
              this._pinNode(comp.id, "gnd"),
              v3K,
              this._independentSourceMagnitude(Number(comp.params.vcc ?? 3.3)),
            );
          }
          this._stampBoardIdleLoad(
            comp,
            "3v3",
            "gnd",
            Number(comp.params.vcc ?? 3.3),
            MCU_BOARD_IDLE_CURRENT_A.raspberry_pi_pico,
          );
          const mcu = this.state.arduinos.get(comp.id);
          if (!mcu) break;
          if (!this._mcuPowered(comp, xGuess)) break;
          const vccNode = this._pinNode(comp.id, "3v3");
          const gndNode = this._pinNode(comp.id, "gnd");
          for (const pinId of mcu.ioPins) {
            const drive = mcu.pinDriveState(pinId);
            const node = this._pinNode(comp.id, pinId);
            if (drive === "input") continue;
            if (drive === "input-pullup") {
              stampResistor(this.mna, node, this._pinNode(comp.id, "3v3"), RP2040_PULL_R);
              continue;
            }
            if (drive === "input-pulldown") {
              stampResistor(this.mna, node, this._pinNode(comp.id, "gnd"), RP2040_PULL_R);
              continue;
            }
            if (node < 0) continue;
            this._stampRailReferencedOutput(
              node,
              gndNode,
              vccNode,
              drive === "out-high" ? 1 : 0,
              GATE_R_OUT,
            );
          }
          break;
        }

        case "microbit": {
          // The micro:bit runs in the client-side embedded sim, not the worker.
          // The `power` param (default on, toggled by the connector/inspector)
          // gates the whole board: unpowered => 3V pad dead and no pin drives,
          // like a real board with the USB unplugged.
          if (Number(comp.params.power ?? 1) === 0) break;
          const vcc = Number(comp.params.vcc ?? 3.3);
          const node3v = this._pinNode(comp.id, "3v");
          const gndNode = this._pinNode(comp.id, "gnd");
          const usbK = this.vsrcIdx.get(`${comp.id}:usb`);
          if (usbK !== undefined) {
            // The rail source scales during source stepping; the idle-load
            // resistor below keeps its authored nominal (it is a load).
            stampVSource(
              this.mna,
              node3v,
              gndNode,
              usbK,
              this._independentSourceMagnitude(vcc),
            );
          }
          this._stampBoardIdleLoad(
            comp,
            "3v",
            "gnd",
            vcc,
            MCU_BOARD_IDLE_CURRENT_A.microbit,
          );
          // Program-driven edge pins pushed in via setMicrobitDrive — P0/P1/P2 on
          // the bare board, plus P8/P12/P13/P14/P15/P16 when the breakout dock is
          // on (S18b Feature 3). No per-pin allowlist needed: _pinNode looks the
          // id up in the component's CURRENT pin set and returns -1 if absent, so
          // a drive for a pin the breakout doesn't currently expose is a no-op —
          // exactly the "stamp only pins present on the component" contract.
          const drives = this.microbitDrives.get(comp.id);
          if (drives) {
            for (const [pinId, d] of Object.entries(drives)) {
              const node = this._pinNode(comp.id, pinId);
              if (node < 0) continue;
              const fraction = d.mode === "digital"
                ? (d.value ? 1 : 0)
                : Math.max(0, Math.min(1023, d.value)) / 1023;
              this._stampRailReferencedOutput(node, gndNode, node3v, fraction, GATE_R_OUT);
            }
          }
          break;
        }

        // breadboard, ic: structural only — unknown kinds silently no-op
      }
    }
  }

  /** Read a 4-bit address from pins a0..a3 of a component. */
  private _readAddr4(comp: SimComponent, xGuess: Float64Array, power?: IcPowerInfo): number {
    const p = power ?? this._icPowerInfo(comp, xGuess);
    let addr = 0;
    for (let i = 0; i < 4; i++) {
      if (this._logicHigh(comp, `a${i}`, xGuess, p)) addr |= (1 << i);
    }
    return addr;
  }

  /** Post-solve: update capacitor voltages, inductor currents, and op-amp committed outputs. */
  private _thermalDissipationW(
    comp: SimComponent,
    x: Float64Array,
    profile: ThermalDeviceProfile,
  ): number {
    let powerW = 0;

    if (profile.deviceClass === "resistor" && comp.pins.length >= 2) {
      if (this._hasFailure(comp.id, "resistor_overload")) return 0;
      const a = this._pinNode(comp.id, comp.pins[0].id);
      const b = this._pinNode(comp.id, comp.pins[1].id);
      const resistance = Math.max(1e-3, Number(comp.params.resistance ?? 1_000));
      const voltage = this._vAt(x, a) - this._vAt(x, b);
      powerW = voltage * voltage / resistance;
    } else if (profile.deviceClass === "led" && comp.kind === "led") {
      if (comp.params.burnt || this._hasFailure(comp.id, "led_failed")) {
        // The electrical model is an open failure. Do not recompute the intact
        // Shockley law across the resulting full supply voltage and turn that
        // numerical open-circuit voltage into fictitious junction heating.
        return 0;
      }
      const anode = this._pinNode(comp.id, comp.pins[0]?.id ?? "a");
      const cathode = this._pinNode(comp.id, comp.pins[1]?.id ?? "k");
      const voltage = this._vAt(x, anode) - this._vAt(x, cathode);
      const specs = specsForComponent(comp);
      const tempC = this.state.thermalDevices.get(comp.id)?.temperatureC
        ?? this._ambientTempC();
      const vt = thermalVoltage(tempC);
      const vf = junctionForwardVoltage(
        Number(comp.params.vf ?? specs?.vf ?? 1.8),
        tempC,
      );
      const emission = Number(comp.params.n ?? N_LED);
      const ratedCurrent = Number(comp.params.iRated ?? specs?.if_max ?? IRATED_LED);
      const saturation = comp.params.Is !== undefined
        ? saturationCurrentAtTemperature(Number(comp.params.Is), tempC, emission)
        : shockleyIsFromVf(vf, ratedCurrent, emission, vt);
      const vtN = emission * vt;
      const vSat = Math.min(Math.max(40 * vtN, vf + 5 * vtN), 80 * vtN);
      const current = shockleyDiodeCurrent(voltage, saturation, emission, vt, vSat);
      powerW = Math.max(0, voltage * current);
    } else if (profile.deviceClass === "linear-regulator") {
      if (this.state.thermalDevices.get(comp.id)?.thermalShutdown) return 0;
      const k = this.vsrcIdx.get(comp.id);
      if (k !== undefined) {
        const inPin = "in";
        const outPin = "out";
        const refPin = comp.kind === "lm317" ? "adj" : "gnd";
        const vin = this._vAt(x, this._pinNode(comp.id, inPin));
        const vout = this._vAt(x, this._pinNode(comp.id, outPin));
        const vref = this._vAt(x, this._pinNode(comp.id, refPin));
        const deliveredCurrent = Math.abs(x[k] ?? 0);
        const passLoss = Math.max(0, vin - vout) * deliveredCurrent;
        const iqLoss = Math.max(0, vin - vref) * regulatorQuiescentCurrent(comp);
        powerW = passLoss + iqLoss;
      }
    } else if (profile.deviceClass === "analog-ic") {
      const power = this._icPowerInfo(comp, x);
      if (power.powered) {
        powerW = Math.max(0, power.vSupply) * analogIcQuiescentCurrent(comp);
        const outputPins = partFor(comp)?.pin_layout.filter((pin) => pin.function === "analog_out") ?? [];
        for (const [unitIndex, outputPin] of outputPins.entries()) {
          const branchId = `${comp.id}:op${unitIndex}`;
          const k = this.vsrcIdx.get(branchId);
          if (k === undefined) continue;
          const current = x[k] ?? 0;
          const vout = this._vAt(x, this._pinNode(comp.id, outputPin.id));
          powerW += current < 0
            ? (-current) * Math.max(0, power.vcc - vout)
            : current * Math.max(0, vout - power.gnd);
        }
      }
    }

    return Number.isFinite(powerW) ? Math.max(0, powerW) : 0;
  }

  /**
   * Advance exact package thermal states after electrical currents have been
   * reconstructed from this solution. Snapshot ownership makes this one update
   * rollback-safe under full-step/half-step trials and rejected adaptive steps.
   */
  private _updatePackageThermalState(h: number, x: Float64Array): void {
    for (const comp of this._thermalComponents) {
      const thermalProfile = this._thermalProfile(comp);
      const previousThermal = this.state.thermalDevices.get(comp.id);
      if (!thermalProfile || !previousThermal || !comp.catalogUid) continue;
      const dissipatedPowerW = this._thermalDissipationW(comp, x, thermalProfile);
      const stepped = stepThermalDevice(
        thermalProfile,
        previousThermal,
        {
          ambientTemperatureC: this._ambientTempC(),
          dissipatedPowerW,
          dtSeconds: h,
        },
      );
      this.state.thermalDevices.set(comp.id, {
        ...stepped.state,
        catalogUid: comp.catalogUid,
        profileId: thermalProfile.id,
        dissipatedPowerW,
        targetTemperatureC: stepped.targetTemperatureC,
        allowedPowerW: stepped.derating.allowedPowerW,
        withinContinuousLimits: stepped.withinContinuousLimits,
        warnings: [...new Set([...thermalProfile.warnings, ...stepped.warnings])],
      });
    }
  }

  private _updateState(h: number, x: Float64Array): void {
    // Reset the lazy MCU event-driver map (see the field's comment) so the
    // servo/hcsr04 commit handlers rebuild it fresh each pass via
    // ctx.mcuEventDriverForNode — pins may have changed between load()
    // calls, mirroring the _dispDrivers reset in _updateDigitalState.
    this._mcuEventDrivers = null;

    for (let i = 0; i < this._stateUpdateComponents.length; i++) {
      const comp = this._stateUpdateComponents[i]!;
      // Operating-point solves advance no physical time: the h-scaled
      // physical integrators (mechanics, heat, charge, echo timing) hold at
      // their entry values while the electrical companions below them keep
      // settling — the post-solve half of the DC_OP_COMPANION_H hold
      // promise. See DC_OP_PHYSICAL_TIME_HOLD_KINDS for the set's rationale.
      if (this._dcPhysicalTimeHold && DC_OP_PHYSICAL_TIME_HOLD_KINDS.has(comp.kind)) {
        continue;
      }
      // Wave A4: registered kinds commit through their device model at this
      // exact position (after the OP physical-time hold). Every kind the
      // engine ever placed in STATE_UPDATE_KINDS is migrated, so there is
      // no legacy if-chain left below the dispatch; a bucket member without
      // a commitState hook is a no-op here, exactly like falling through
      // the old kind-exclusive chain.
      const stateModel = this._stateUpdateModels[i];
      if (stateModel?.commitState) {
        stateModel.commitState(this._deviceCtx, comp, x, h);
        continue;
      }
    }
  }

  // Build a map from MNA node index to the Arduino component + pin that
  // drives that net.  Used by display decoders to look up which Arduino
  // pin corresponds to a given display-component pin.  Last-writer wins when
  // two Arduino pins share a node — acceptable for v1.
  //
  // `includePico` widens this to also cover raspberry_pi_pico for servo and
  // HC-SR04 cycle-event detection. Defaults false so the existing
  // max7219/hd44780 display-decode call sites (which share the `_dispDrivers`
  // cache below and were never audited against Pico sub-step timing) keep
  // their exact prior uno/nano-only behaviour.
  private _buildArduinoPinDrivers(includePico = false): Map<number, { compId: string; pin: string }> {
    const map = new Map<number, { compId: string; pin: string }>();
    if (!this.circuit) return map;
    for (const comp of this._mcuComponents) {
      const isUnoNano = comp.kind === "arduino_uno" || comp.kind === "arduino_nano";
      const isPico = includePico && comp.kind === "raspberry_pi_pico";
      if (!isUnoNano && !isPico) continue;
      const mcu = this.state.arduinos.get(comp.id);
      if (!mcu) continue;
      for (const pinId of mcu.ioPins) {
        const node = this._pinNode(comp.id, pinId);
        if (node >= 0) map.set(node, { compId: comp.id, pin: pinId });
      }
    }
    return map;
  }

  // Given a display component and a list of its protocol pin names, collect
  // the ordered sub-step PinEvents contributed by the single Arduino that
  // drives all those pins.  If the pins span multiple Arduinos (multi-master),
  // or no Arduino drives any of them, returns an empty event list so callers
  // can fall back to sampled-level decoding.
  private _mergedDisplayEvents(
    comp: SimComponent,
    pinNames: string[],
    drivers: Map<number, { compId: string; pin: string }>,
  ): { arduinoCompId: string | null; events: Array<{ displayPin: string; level: 0 | 1 | null; cycle: number }> } {
    const arduinoPinToDisplayPin: Record<string, string> = {};
    const driverCompIds = new Set<string>();

    for (const displayPin of pinNames) {
      const node = this._pinNode(comp.id, displayPin);
      if (node < 0) continue;
      const driver = drivers.get(node);
      if (!driver) continue;
      arduinoPinToDisplayPin[driver.pin] = displayPin;
      driverCompIds.add(driver.compId);
    }

    if (driverCompIds.size !== 1) {
      // Zero drivers (no Arduino connected) or multi-master (unsupported v1).
      return { arduinoCompId: null, events: [] };
    }

    const thatCompId = [...driverCompIds][0];
    const mcu = this.state.arduinos.get(thatCompId);
    if (!mcu) return { arduinoCompId: null, events: [] };

    const mapped = mcu
      .getStepPinEvents()
      .filter((e: PinEvent) => e.pin in arduinoPinToDisplayPin)
      .map((e: PinEvent) => ({
        displayPin: arduinoPinToDisplayPin[e.pin],
        level: e.level,
        cycle: e.cycle,
      }))
      .sort((a, b) => a.cycle - b.cycle);

    return { arduinoCompId: thatCompId, events: mapped };
  }

  /**
   * Post-solve: update digital element states (DFF edge detection, 555 SR
   * flip-flop) and compute the public `digitalState` map (0/1 per component).
   */
  private _updateDigitalState(h: number, x: Float64Array): void {
    const ds: Record<string, number> = {};
    const disp: Record<string, DisplayInfo> = {};
    // Reset the lazy Arduino-pin-driver map so it is rebuilt fresh each step
    // from the converged topology (pins may have changed between load() calls).
    this._dispDrivers = null;
    // Wave A4: device updateDigital handlers publish into these pass-scoped
    // sinks through ctx.setDigitalState / ctx.setDisplayInfo — the same
    // per-step maps the remaining switch cases write directly.
    this._digitalStateOut = ds;
    this._displayStateOut = disp;

    for (let i = 0; i < this._digitalUpdateComponents.length; i++) {
      const comp = this._digitalUpdateComponents[i]!;
      // Wave A4: registered kinds commit digital state through their device
      // model at this component's exact position in the pass — same list,
      // same order — and unmigrated kinds fall through to the switch below.
      const deviceModel = this._digitalUpdateModels[i];
      if (deviceModel?.updateDigital) {
        deviceModel.updateDigital(this._deviceCtx, comp, x, h);
        continue;
      }

      switch (comp.kind) {
        case "clock_gen":
        case "clock": {
          const freq = Math.max(1e-6, Number(comp.params.frequency ?? 1000));
          const duty = Number(comp.params.duty ?? 0.5);
          const vHi  = Number(comp.params.v_high ?? 5);
          const vLo  = Number(comp.params.v_low  ?? 0);
          const delay = Number(comp.params.delay ?? 0);
          const v = clockVoltage(this._waveformSampleTime(h), 1 / freq, duty, vHi, vLo, delay);
          ds[comp.id] = v > (vHi + vLo) / 2 ? 1 : 0;
          break;
        }

        case "pulse_gen":
        case "pulse_source": {
          const v1  = Number(comp.params.v1  ?? 0);
          const v2  = Number(comp.params.v2  ?? 5);
          const td  = Number(comp.params.td  ?? 0);
          const tr  = Number(comp.params.tr  ?? 1e-6);
          const tf  = Number(comp.params.tf  ?? 1e-6);
          const pw  = Number(comp.params.pw  ?? 5e-4);
          const per = Number(comp.params.per ?? 1e-3);
          const v = pulseVoltage(this._waveformSampleTime(h), v1, v2, td, tr, tf, pw, per);
          ds[comp.id] = v > (v1 + v2) / 2 ? 1 : 0;
          break;
        }
      }
    }

    this._digitalStateOut = null;
    this._displayStateOut = null;
    this.digitalState = ds;
    this._displayState = disp;
  }

  /**
   * Post-solve: compute and cache per-component current (pin0 → pin1).
   * The renderer and Inspector read these for LED brightness / probes.
   */
  private _updateElementI(
    h: number,
    x: Float64Array,
    previousMosfetGates: ReadonlyMap<string, { vgs: number; vgd: number }>,
  ): void {
    const out: Record<string, number> = {};
    // Wave A4: device updateCurrent handlers publish through
    // ctx.setElementCurrent, which writes into this pass-scoped sink, and
    // read the pre-commit gate history through ctx.previousMosfetGate.
    this._elementIOut = out;
    this._previousMosfetGates = previousMosfetGates;
    const components = this.circuit!.components;
    for (let i = 0; i < components.length; i++) {
      const comp = components[i]!;
      if (comp.pins.length < 2) continue;

      // Wave A4: registered kinds publish their element current through the
      // device model at this exact position; dispatched components skip the
      // switch (including its default), exactly like their old cases did.
      // The old shared a/b/va/vb/vd preamble lives in the handlers now (each
      // recomputes the same pure reads), and no remaining switch case below
      // reads it — computing it here too would double the _pinNode work for
      // every dispatched component (A4 perf review).
      const deviceModel = this._elementCurrentModels[i];
      if (deviceModel?.updateCurrent) {
        deviceModel.updateCurrent(this._deviceCtx, comp, x, h);
        continue;
      }

      switch (comp.kind) {
        case "arduino_uno":
        case "arduino_nano":
        case "raspberry_pi_pico":
        case "microbit": {
          // USB/on-board regulator branch current now includes rail loading
          // from every GPIO drive instead of reporting an impossible zero.
          const k = this.vsrcIdx.get(`${comp.id}:usb`);
          out[comp.id] = k !== undefined ? (x[k] ?? 0) : 0;
          break;
        }
        default:
          out[comp.id] = 0;
      }
    }
    this._elementIOut = null;
    this._previousMosfetGates = null;
    this.elementI = out;
  }

  private _updateFailureStates(h: number, x: Float64Array): void {
    if (!this.circuit) return;
    // Arm the pass-scoped channel-current cache (see the field's comment).
    this._failureChannelCurrents = { value: null };

    for (let i = 0; i < this._failureUpdateComponents.length; i++) {
      const comp = this._failureUpdateComponents[i]!;
      const specs = specsForComponent(comp);
      // Share this resolution with the dispatched handler's
      // ctx.electricalSpecs read (see _failureSpecsComp).
      this._failureSpecsComp = comp;
      this._failureSpecsValue = specs;

      // Wave A4: registered kinds integrate their damage/trip state through
      // the device model. The old kind-gated blocks here were mutually
      // exclusive, so dispatch replaces exactly one of them per component;
      // the generic supply-range/output-sag scan below is NOT part of the
      // hook and still runs afterward for anything with vcc_range specs.
      const deviceModel = this._failureUpdateModels[i];
      if (deviceModel?.updateFailures) {
        deviceModel.updateFailures(this._deviceCtx, comp, x, h);
      }

      if (specs?.vcc_range) {
        const power = this._icPowerInfo(comp, x);
        if (power.powered) {
          const targets = this._drivenOutputTargets(comp, x, power);
          const targetPins = new Set(targets.map((target) => target.pinId));
          for (const target of targets) {
            const pinNode = this._pinNode(comp.id, target.pinId);
            const actual = this._vAt(x, pinNode) - power.gnd;
            const limit = target.high ? power.thresholds.vih : power.thresholds.vil;
            const sagging = target.high ? actual < power.thresholds.vih : actual > power.thresholds.vil;
            const keySag = failureKey("output_sag", comp.id, target.pinId);
            if (sagging) {
              // Output sag is a reversible load-line observation, not invented
              // permanent silicon damage. Finite output resistance already
              // determines the voltage; never mutate that resistance because a
              // logic threshold was missed. A real damage model would need
              // package current/power/thermal ratings and accumulated stress.
              //
              // Debounce single-step switching transients before publishing:
              // failureStress counts consecutive sagging steps under this key.
              // It is snapshotted, so an adaptive-step rollback + retry at the
              // same edge does not double-count. Only sags that outlast the
              // switching edge (>= OUTPUT_SAG_DEBOUNCE_STEPS) surface as a
              // warning; the count is capped so a sustained sag stays published
              // without growing unbounded.
              const seen = (this.state.failureStress.get(keySag) ?? 0) + 1;
              if (seen >= OUTPUT_SAG_DEBOUNCE_STEPS) {
                this.state.failureStress.set(keySag, OUTPUT_SAG_DEBOUNCE_STEPS);
                this.state.failures.set(keySag, {
                  componentId: comp.id,
                  kind: "output_sag",
                  pinId: target.pinId,
                  since: this.simTime + h,
                  value: actual,
                  limit,
                  message:
                    `${comp.id}.${target.pinId} is commanded ${target.high ? "HIGH" : "LOW"} but sits at ${actual.toFixed(2)} V under the present load. The finite-strength output is outside its own package family's ${target.high ? "HIGH" : "LOW"} input-guarantee band; connected receivers may have different requirements.`,
                });
              } else {
                // Still within the debounce window — record progress but do not
                // surface the warning yet.
                this.state.failureStress.set(keySag, seen);
                this.state.failures.delete(keySag);
              }
            } else {
              this.state.failures.delete(keySag);
              this.state.failureStress.delete(keySag);
            }
          }

          const part = partFor(comp);
          for (const pin of part?.pin_layout ?? []) {
            if (pin.function !== "output" && pin.function !== "io" && pin.function !== "tri_state") continue;
            if (!targetPins.has(pin.id)) {
              const keySag = failureKey("output_sag", comp.id, pin.id);
              this.state.failures.delete(keySag);
              this.state.failureStress.delete(keySag);
            }
          }
        } else {
          this._clearOutputSagWarnings(comp);
        }
      }
    }

    this._failureSpecsComp = null;
    this._failureSpecsValue = undefined;
    this._failureChannelCurrents = null;
  }

  private _clearOutputSagWarnings(comp: SimComponent): void {
    for (const pin of comp.pins) {
      const key = failureKey("output_sag", comp.id, pin.id);
      this.state.failures.delete(key);
      this.state.failureStress.delete(key);
    }
  }

  private _drivenOutputTargets(
    comp: SimComponent,
    x: Float64Array,
    power: IcPowerInfo,
  ): Array<{ pinId: string; high: boolean }> {
    if (!power.powered) return [];

    switch (comp.kind) {
      case "ne555": {
        const { outHigh } = this.state.ne555s.get(comp.id) ?? { outHigh: true };
        return [{ pinId: "3", high: outHigh }];
      }
      case "74ls00":
      case "74ls04":
      case "74ls08":
      case "74ls32":
      case "74ls86":
      case "74ls157":
      case "74ls283":
      case "74ls245":
      case "74hc14":
      case "74hc138": {
        return Object.entries(this._combinationalOutputs(comp, x, power)).map(([pinId, level]) => ({
          pinId,
          high: this._delayedOutputLevel(comp, pinId, level),
        }));
      }
      case "74ls47": {
        // Open-collector outputs operate in electrical convention (true=HIGH=released,
        // false=LOW=sinking). Report only the SINKING outputs for output-sag tracking:
        // filter to entries whose delayed level is LOW (false = transistor on).
        return Object.entries(this._combinationalOutputs(comp, x, power))
          .filter(([pinId, level]) => !this._delayedOutputLevel(comp, pinId, level))
          .map(([pinId]) => ({ pinId, high: false }));
      }
      case "lm393": {
        // Open-collector outputs: report only the sinking (LOW) outputs for sag
        // tracking.  A released output is hi-Z — the pull-up determines the voltage,
        // not the LM393 driver, so it cannot sag in the output-driver sense.
        const stLm393 = this.state.icState.get(comp.id) ?? defaultIcState("lm393");
        const targets: Array<{ pinId: string; high: boolean }> = [];
        if (stLm393.out1 === 0) targets.push({ pinId: "1", high: false });
        if (stLm393.out2 === 0) targets.push({ pinId: "7", high: false });
        return targets;
      }
      case "74ls161": {
        const st = this.state.icState.get(comp.id) ?? defaultIcState("74ls161");
        const count = st.count ?? 0;
        const entHigh = this._logicHigh(comp, "ent", x, power);
        return [
          { pinId: "qa", high: (count & 1) !== 0 },
          { pinId: "qb", high: (count & 2) !== 0 },
          { pinId: "qc", high: (count & 4) !== 0 },
          { pinId: "qd", high: (count & 8) !== 0 },
          { pinId: "rco", high: count === 15 && entHigh },
        ];
      }
      case "74ls173": {
        const mHigh = this._logicHigh(comp, "m", x, power);
        const nHigh = this._logicHigh(comp, "n", x, power);
        if (mHigh || nHigh) return [];
        const st = this.state.icState.get(comp.id) ?? defaultIcState("74ls173");
        return [1, 2, 3, 4].map((n) => ({ pinId: `q${n}`, high: (st[`q${n}`] ?? 0) === 1 }));
      }
      case "74ls189": {
        const csLow = !this._logicHigh(comp, "/cs", x, power);
        const weHigh = this._logicHigh(comp, "/we", x, power);
        if (!csLow || !weHigh) return [];
        const st = this.state.icState.get(comp.id) ?? defaultIcState("74ls189");
        const data = st[`mem${this._readAddr4(comp, x, power)}`] ?? 0xf;
        return [0, 1, 2, 3].map((bit) => ({ pinId: `/o${bit + 1}`, high: ((data >> bit) & 1) === 0 }));
      }
      case "74hc595": {
        if (this._logicHigh(comp, "/oe", x, power)) return [];
        const latch = this.state.icState.get(comp.id)?.latch ?? 0;
        return ["qa", "qb", "qc", "qd", "qe", "qf", "qg", "qh"].map((pinId, bit) => ({
          pinId,
          high: ((latch >> bit) & 1) === 1,
        }));
      }
      case "74hc165": {
        const shift = this.state.icState.get(comp.id)?.shift ?? 0;
        const q7 = ((shift >> 7) & 1) === 1;
        return [{ pinId: "q7", high: q7 }, { pinId: "/q7", high: !q7 }];
      }
      // W2.2 sequential ICs
      case "74hc74": {
        const st = this.state.icState.get(comp.id) ?? defaultIcState("74hc74");
        const q1  = (st.q1  ?? 0) >= 0.5;
        const q1n = (st.q1n ?? 1) >= 0.5;
        const q2  = (st.q2  ?? 0) >= 0.5;
        const q2n = (st.q2n ?? 1) >= 0.5;
        return [
          { pinId: "q1",   high: q1  },
          { pinId: "q1_n", high: q1n },
          { pinId: "q2",   high: q2  },
          { pinId: "q2_n", high: q2n },
        ];
      }
      case "cd4017": {
        const count = this.state.icState.get(comp.id)?.count ?? 0;
        const targets: Array<{ pinId: string; high: boolean }> = [];
        for (let i = 0; i <= 9; i++) {
          targets.push({ pinId: `q${i}`, high: count === i });
        }
        targets.push({ pinId: "co", high: count < 5 });
        return targets;
      }
      case "cd4511": {
        const st = this.state.icState.get(comp.id) ?? defaultIcState("cd4511");
        const ltLow = !this._logicHighH(comp, "lt_n", x, power);
        const blLow = !this._logicHighH(comp, "bl_n", x, power);
        const segs = evalCD4511(st.latchedBcd ?? 0, ltLow, blLow);
        return Object.entries(segs).map(([pinId, high]) => ({ pinId, high }));
      }
      case "cd4060": {
        const st = this.state.icState.get(comp.id) ?? defaultIcState("cd4060");
        const resetHigh = this._logicHigh(comp, "reset", x, power);
        const count = resetHigh ? 0 : (st.count ?? 0);
        const targets: Array<{ pinId: string; high: boolean }> = [];
        // Only pinned stages — Q1-Q3 and Q11 are internal (no output pin)
        for (const [pinId, stage] of [
          ["q4", 4], ["q5", 5], ["q6", 6], ["q7", 7],
          ["q8", 8], ["q9", 9], ["q10", 10],
          ["q12", 12], ["q13", 13], ["q14", 14],
        ] as [string, number][]) {
          targets.push({ pinId, high: ((count >> (stage - 1)) & 1) === 1 });
        }
        // Always stamp inv outputs — both LOW during reset, non-floating on release.
        // inv1Out drives rtc (pin 10); inv2Out drives ctc (pin 9).
        targets.push({ pinId: "rtc", high: (st.inv1Out ?? 0) >= 0.5 });
        targets.push({ pinId: "ctc", high: (st.inv2Out ?? 0) >= 0.5 });
        return targets;
      }
      case "28c16":
      case "28c256": {
        const st = this.state.eeproms.get(comp.id);
        if (!st) return [];
        const oeLow = !this._logicHigh(comp, "/oe", x, power);
        const ceLow = !this._logicHigh(comp, "/ce", x, power);
        if (!oeLow || !ceLow) return [];
        const addrBits = comp.kind === "28c16" ? 11 : 15;
        let addr = 0;
        for (let i = 0; i < addrBits; i++) {
          if (this._logicHigh(comp, `a${i}`, x, power)) addr |= (1 << i);
        }
        const byte = st.bytes[addr] ?? 0xff;
        return Array.from({ length: 8 }, (_, bit) => ({
          pinId: `io${bit}`,
          high: ((byte >> bit) & 1) === 1,
        }));
      }
      // W6.2 — ULN2003/2803: report sinking outputs (low level) for sag monitoring.
      // Open-collector outputs that are actively sinking are the only driver side;
      // released (hi-Z) outputs cannot sag — the pull-up determines the voltage.
      case "uln2003":
      case "uln2803": {
        const channels_D = comp.kind === "uln2003" ? 7 : 8;
        const stUlnD = this.state.icState.get(comp.id) ?? defaultIcState(comp.kind);
        const targets: Array<{ pinId: string; high: boolean }> = [];
        for (let ch = 1; ch <= channels_D; ch++) {
          const committedInD = stUlnD[`in${ch}`] as number;
          // Sinking: report as LOW output (for output-sag monitoring).
          if (!Number.isNaN(committedInD) && committedInD >= 0.5) {
            targets.push({ pinId: `out${ch}`, high: false });
          }
        }
        return targets;
      }
      // W6.3 — L293D / TB6612: report driven output pins for output-sag monitoring.
      // Both HIGH (sourcing from VM) and LOW (sinking to GND) outputs are reported;
      // only HIZ outputs are excluded (they cannot sag since they are not driven).
      case "l293d": {
        const stL293D = this.state.icState.get(comp.id) ?? defaultIcState("l293d");
        const targets293: Array<{ pinId: string; high: boolean }> = [];
        const l293dOuts = [
          { pinId: "out1", state: stL293D.out1 as number },
          { pinId: "out2", state: stL293D.out2 as number },
          { pinId: "out3", state: stL293D.out3 as number },
          { pinId: "out4", state: stL293D.out4 as number },
        ];
        for (const { pinId, state } of l293dOuts) {
          if (state === 1) targets293.push({ pinId, high: true });
          else if (state === 0) targets293.push({ pinId, high: false });
          // HIZ (-1): not driven, skip.
        }
        return targets293;
      }
      case "tb6612": {
        const stTb6D = this.state.icState.get(comp.id) ?? defaultIcState("tb6612");
        const targetsTb: Array<{ pinId: string; high: boolean }> = [];
        const tb6Outs = [
          { pinId: "ao1", state: stTb6D.ao1 as number },
          { pinId: "ao2", state: stTb6D.ao2 as number },
          { pinId: "bo1", state: stTb6D.bo1 as number },
          { pinId: "bo2", state: stTb6D.bo2 as number },
        ];
        for (const { pinId, state } of tb6Outs) {
          if (state === 1) targetsTb.push({ pinId, high: true });
          else if (state === 0) targetsTb.push({ pinId, high: false });
        }
        return targetsTb;
      }
      default:
        return [];
    }
  }

  /**
   * After analog solve converges: read each Arduino input pin's net voltage
   * and push it back into the MCU so the next step sees correct PINX registers.
   */
  private _sampleArduinoInputs(): void {
    if (!this.circuit) return;
    for (const comp of this._mcuComponents) {
      const mcu = this.state.arduinos.get(comp.id);
      if (!mcu) continue;
      if (!this._mcuPowered(comp)) continue;
      const isPico = comp.kind === "raspberry_pi_pico";
      const supplyPin = isPico ? "3v3" : "5v";
      const vcc = Math.max(0, this._netVoltageForPin(comp.id, supplyPin) - this._netVoltageForPin(comp.id, "gnd")) || Number(comp.params.vcc ?? (isPico ? 3.3 : 5));
      const vth = vcc * 0.6; // ~0.6·Vcc logic threshold (3 V @ 5 V, 1.98 V @ 3.3 V)
      for (const pinId of mcu.ioPins) {
        const drive = mcu.pinDriveState(pinId);
        if (drive === "out-high" || drive === "out-low") continue;
        const netId = this._netIdForPin(comp.id, pinId);
        if (!netId) continue;
        const v = this.netV[netId] ?? 0;
        mcu.setInputBit(pinId, v >= vth ? 1 : 0);
      }
      // ADC sampling: push real net voltages into the AVRADC channels so analogRead()
      // returns a true 10-bit value, not the binary threshold from the digital path above.
      // Uno-kind boards skip A6/A7 because PC6/PC7 aren't bonded out on that footprint.
      const isUno = comp.kind === "arduino_uno";
      for (const pinId of mcu.analogPins) {
        if (isUno && (pinId === "a6" || pinId === "a7")) continue;
        const netId = this._netIdForPin(comp.id, pinId);
        if (!netId) continue;
        const v = this.netV[netId] ?? 0;
        mcu.setAnalogVolts(pinId, v, vcc);
      }
    }
  }
}
