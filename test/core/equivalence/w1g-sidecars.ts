/**
 * W1g's gate 2c sidecars (core/README.md, "Kernel semantics: thermal,
 * failures and warnings (W1g)" point 19.1, and the W1g Equivalence harness
 * rules): a fixture's slow events held to the model's own answer.
 *
 * THE GENERATOR (`slowEvents`) reads the circuit and classic's catalog and
 * thermal profiles, and computes from the model's own equations, never from
 * core code:
 * - `resistor-overload`, `fuse`, `led-overdrive`: the dwell integrator's
 *   latch at `threshold / rate` after the start or the last reset (points
 *   5-8), with the drive (`value`) and the rating (`limit`);
 * - `ptc`: the trip at `3·iHold·iHold·0.1 / (I² − iHold²)`, and, while the
 *   tripped current is cool, the recovery 2.0 s after it (point 10);
 * - `output-sag`: a gate output commanded HIGH into a load to the datum
 *   latches `OUTPUT_SAG_WINDOW` after the start or a reset (point 9), at
 *   `vS·R/(R + R_out)` against `vih`, with classic's `R_out` and thresholds;
 * - `resistor-heat`: the one-pole `Ta + P·Rθ·(1 − e^(−t/τ))` (point 3);
 * - `led-heat`: the coupled rise of an identified LED, which has no closed
 *   form: the model's own ODE `T' = (Ta + P(T)·Rθ − T)/τ`, `P(T)` the
 *   junction law at `T` (LEDs point 2) solved through its series resistor,
 *   integrated by RK4 at `h` and `h/2`, refused unless the two agree.
 *
 * THE RUNNER (`runSlowChecks`) drives both engines over the same horizon,
 * with `resetFailures()` at the declared instants:
 * - the core is read at every advance end through its latch log
 *   (`takeLatchedFailures`), so every latch's own `since` is seen; a PTC
 *   transition is bracketed by readings `1e-9 s + 1e-9` relative either side
 *   of the analytic instant;
 * - classic is polled at every accepted step (as S1 criterion 5 polls it),
 *   so its first latch time is the end of the step that latched.
 * Gated: the core's event set and times (within `1e-9 s + 1e-9` relative),
 * its `value`/`limit` against the analytic ones (decision 6), its messages
 * against classic's template on its own numbers, its temperatures within
 * `1e-3` of the rise plus `1e-3 °C`, its currents within decision 6; that
 * classic fires every event; and that classic still fires each at the time
 * the fixture records (`classicS`). Classic's times and temperatures against
 * the analytic answer are recorded, never gated (Q-O31).
 */

import { CoreEngine } from "../../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../../src/core/kinds.js";
import { buildCoreNodes, corePinKey, type CoreNodeBuild } from "../../../src/core/compile/topology.js";
import { HeadlessRunner } from "../../../src/host/headless.js";
import {
  authoredBatteryCharge,
  batteryModelForComponent,
  batteryOperatingPointFor,
  specsForComponent,
  thermalProfileForComponent,
  thresholdsFor,
} from "../../../src/sim/engine/sim-engine.js";
import type { SimCircuit, SimFailure, SimFailureKind } from "../../../src/sim/engine/sim-engine.js";
import { thermalTimeConstantS } from "../../../src/sim/thermal-physics.js";
import { ledLaw, ledSeriesNode, NODE_SHUNT_SIEMENS, type LedLaw } from "./analytic.js";
import { pieceValueAt, pulseModelPieces, pulseParamsOf, pulsePieces, psuRailGate } from "./w1f-sidecars.js";
import { explainCommittedTarget, W1G_SAG_COMMITTED_TARGET, type PendingDelayWindow } from "./known-divergences.js";
import { AMP_ABS_TOL, AMP_REL_TOL, withinTolerance } from "./tolerance.js";
import type { KnownDivergenceHit, Mismatch, SlowCheck, SlowSidecarModel } from "./types.js";
import { classicFailureMessage, failureTolerance } from "./w1g-failures.js";

type SimComponent = SimCircuit["components"][number];

/** core/README.md W1g constants (points 5-10). */
export const RESISTOR_DAMAGE_THRESHOLD = 1;
export const FUSE_I2T_THRESHOLD = 0.3;
export const LED_DAMAGE_THRESHOLD = 24;
export const OUTPUT_SAG_WINDOW_S = 1e-8;
export const PTC_RECOVERY_S = 2.0;
/** Classic's gate output resistance floor and ceiling (sim-engine.ts `GATE_R_OUT`, `_icPowerInfo`). */
const GATE_R_OUT = 50;
const GATE_R_OUT_MAX = 1000;

/** The integrated latch time tolerance (Q-O31, point 19.1): `1e-9 s + 1e-9` relative. */
export function slowTimeTolerance(t: number): number {
  return 1e-9 + 1e-9 * Math.abs(t);
}

/** Temperatures (point 19.1): within `1e-3` of the rise above ambient plus `1e-3 °C`. */
export function temperatureTolerance(expectedC: number, ambientC: number): number {
  return 1e-3 * Math.abs(expectedC - ambientC) + 1e-3;
}

export interface SlowEvent {
  readonly label: string;
  /** `sag-clear`: a latched `output_sag` (`kind`, `componentId`, `pinId`) stops being reported. */
  readonly type: "latch" | "ptc-trip" | "ptc-recover" | "sag-clear";
  readonly componentId: string;
  readonly kind?: SimFailureKind;
  readonly pinId?: string;
  readonly t: number;
  readonly value?: number;
  readonly limit?: number;
}

export interface SlowReading {
  readonly label: string;
  readonly what: "temperature" | "current";
  readonly componentId: string;
  readonly t: number;
  readonly expected: number;
  readonly ambientC: number;
}

export interface SlowExpectation {
  /** In time order: what `classicS` lists, one entry per event. */
  readonly events: readonly SlowEvent[];
  readonly readings: readonly SlowReading[];
}

// ─── circuit reading ──────────────────────────────────────────────────────────

function componentOf(circuit: SimCircuit, id: string, kinds: readonly string[]): SimComponent {
  const comp = circuit.components.find((c) => c.id === id);
  if (!comp) throw new Error(`slow sidecar: no component "${id}"`);
  if (!kinds.includes(comp.kind)) throw new Error(`slow sidecar: "${id}" is a ${comp.kind}, expected ${kinds.join(" or ")}`);
  return comp;
}

function param(comp: SimComponent, key: string): number {
  const v = comp.params[key];
  const n = v === undefined ? Number.NaN : Number(v);
  if (!Number.isFinite(n)) throw new Error(`slow sidecar: ${comp.id} must declare a finite "${key}" (the generator does not guess a catalog default)`);
  return n;
}

function nodeOf(built: CoreNodeBuild, componentId: string, pinId: string): number {
  const node = built.pinNode.get(corePinKey(componentId, pinId));
  if (node === undefined) throw new Error(`slow sidecar: pin ${componentId}.${pinId} is on no node`);
  return node;
}

/** Node volts held by ideal `voltage_source`s whose `neg` is on the datum (the only fixed nodes these fixtures use). */
function fixedVolts(circuit: SimCircuit, built: CoreNodeBuild): Map<number, number> {
  const fixed = new Map<number, number>([[0, 0]]);
  for (const c of circuit.components) {
    if (c.kind !== "voltage_source") continue;
    if (nodeOf(built, c.id, c.pins[1]!.id) !== 0) continue;
    fixed.set(nodeOf(built, c.id, c.pins[0]!.id), Number(c.params.voltage ?? 5));
  }
  return fixed;
}

/** `V(pin0) − V(pin1)` of a two-pin part straight across ideal sources. */
function voltsAcross(circuit: SimCircuit, built: CoreNodeBuild, comp: SimComponent): number {
  const fixed = fixedVolts(circuit, built);
  const a = fixed.get(nodeOf(built, comp.id, comp.pins[0]!.id));
  const b = fixed.get(nodeOf(built, comp.id, comp.pins[1]!.id));
  if (a === undefined || b === undefined) throw new Error(`slow sidecar: ${comp.id} is not straight across ideal sources`);
  return a - b;
}

function ambientOf(circuit: SimCircuit): number {
  return Number(circuit.environment?.temperatureC ?? 25);
}

// ─── the generator ─────────────────────────────────────────────────────────────

/** Latch times of a dwell integrator at a constant `rate > 0`, restarted from 0 at every reset (point 13). */
function dwellLatches(rate: number, threshold: number, horizonS: number, resetsS: readonly number[]): number[] {
  if (!(rate > 0)) return [];
  const out: number[] = [];
  const starts = [0, ...resetsS];
  starts.forEach((start, i) => {
    const end = i + 1 < starts.length ? starts[i + 1]! : horizonS;
    const t = start + threshold / rate;
    if (t <= end) out.push(t);
  });
  return out;
}

function ledSeries(circuit: SimCircuit, check: Extract<SlowCheck, { check: "led-overdrive" | "led-heat" }>): {
  led: SimComponent;
  vs: number;
  rOhms: number;
  lawAt: (tempC: number) => LedLaw;
} {
  const built = buildCoreNodes(circuit);
  const led = componentOf(circuit, check.led, ["led"]);
  const res = componentOf(circuit, check.resistor, ["resistor"]);
  const src = componentOf(circuit, check.source, ["voltage_source"]);
  const vs = Number(src.params.voltage ?? 5);
  const anode = nodeOf(built, led.id, led.pins[0]!.id);
  const ends = [nodeOf(built, res.id, res.pins[0]!.id), nodeOf(built, res.id, res.pins[1]!.id)];
  if (
    nodeOf(built, led.id, led.pins[1]!.id) !== 0 ||
    nodeOf(built, src.id, src.pins[1]!.id) !== 0 ||
    !ends.includes(anode) ||
    !ends.includes(nodeOf(built, src.id, src.pins[0]!.id))
  ) {
    throw new Error("slow sidecar: expected source - resistor - LED anode, the cathode and the source's neg on the datum");
  }
  const vf25 = param(led, "vf");
  const iRated = param(led, "iRated");
  const n = param(led, "n");
  return { led, vs, rOhms: param(res, "resistance"), lawAt: (tempC) => ledLaw({ vf25, iRated, n, tempC }) };
}

/**
 * The coupled LED rise (point 19.1): RK4 on `T' = (Ta + P(T)·Rθ − T)/τ`,
 * returned as `T(t)`; refused unless `h` and `h/2` agree within 1e-9 °C.
 */
function ledHeatCurve(circuit: SimCircuit, check: Extract<SlowCheck, { check: "led-heat" }>, horizonS: number): (t: number) => { tempC: number; amps: number } {
  const { led, vs, rOhms, lawAt } = ledSeries(circuit, check);
  const profile = thermalProfileForComponent(led);
  if (!profile) throw new Error(`slow sidecar: ${led.id} has no exact thermal identity`);
  const ta = ambientOf(circuit);
  const rTheta = profile.network.rThetaJA_CPerW;
  const tau = thermalTimeConstantS(profile.network);
  const op = (tempC: number) => ledSeriesNode(lawAt(tempC), vs, rOhms, "anode-node");
  const power = (tempC: number): number => {
    const p = op(tempC);
    return Math.max(0, p.nodeV * p.ledAmps);
  };
  const f = (tempC: number): number => (ta + power(tempC) * rTheta - tempC) / tau;
  const integrate = (h: number): Float64Array => {
    const n = Math.ceil(horizonS / h);
    const out = new Float64Array(n + 1);
    let x = ta;
    out[0] = x;
    for (let i = 0; i < n; i++) {
      const k1 = f(x);
      const k2 = f(x + (h / 2) * k1);
      const k3 = f(x + (h / 2) * k2);
      const k4 = f(x + h * k3);
      x += (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
      out[i + 1] = x;
    }
    return out;
  };
  const h = 1e-3;
  const coarse = integrate(h);
  const fine = integrate(h / 2);
  coarse.forEach((x, i) => {
    if (Math.abs(x - fine[2 * i]!) > 1e-9) throw new Error(`led-heat: RK4 at h and h/2 disagree at t=${i * h} (${x} against ${fine[2 * i]})`);
  });
  const fineH = h / 2;
  return (t: number) => {
    const i = Math.min(Math.floor(t / fineH), fine.length - 1);
    let x = fine[i]!;
    const dt = t - i * fineH;
    if (dt > 0) {
      const k1 = f(x);
      const k2 = f(x + (dt / 2) * k1);
      const k3 = f(x + (dt / 2) * k2);
      const k4 = f(x + dt * k3);
      x += (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
    }
    return { tempC: x, amps: op(x).ledAmps };
  };
}

/**
 * The rail of a `rail-sag` check and its crossings of the power window's
 * lower edge (`vccMin·0.9`, W1c point 1):
 * - `psu`: W1f's `psu-rail-gate` sidecar (Rails points 4 and 5), whose
 *   premise holds: the IC's LOW output sinks the pull-up's current to the
 *   datum, not from the rail;
 * - `battery`: a cell with no rail capacitor into a resistor to a pulse
 *   source's node, so the rail is static at each instant,
 *   `u = (E/r + s/R + 0)/(1/r + 1/R + g)`, with `E` and `r` from classic's own
 *   model at the authored charge (the drain over a fixture's milliseconds
 *   moves `E` by microvolts), and the pulse as the model's pieces, whose
 *   fast ramps are midpoint steps (Sources point 3).
 */
function railSagRail(
  circuit: SimCircuit,
  built: CoreNodeBuild,
  check: Extract<SlowCheck, { check: "rail-sag" }>,
  horizonS: number,
  gateLow: number,
): { railAt: (t: number) => number; flips: Array<{ t: number; powered: boolean }>; initiallyPowered: boolean } {
  if (check.rail.kind === "psu") {
    const gate = psuRailGate(
      circuit,
      { kind: "psu-rail-gate", psu: check.rail.psu, capacitor: check.rail.capacitor, load: check.load, source: check.source, ic: check.ic, outputKey: `${check.ic}/${check.pin}`, traceDurationS: horizonS },
      horizonS,
    );
    return { railAt: gate.railAt, flips: [...gate.flips], initiallyPowered: false };
  }
  const bat = componentOf(circuit, check.rail.battery, ["battery_pack"]);
  const load = componentOf(circuit, check.load, ["resistor"]);
  const src = componentOf(circuit, check.source, ["pulse_source", "pulse_gen"]);
  const pos = nodeOf(built, bat.id, bat.pins[0]!.id);
  const ends = [nodeOf(built, load.id, load.pins[0]!.id), nodeOf(built, load.id, load.pins[1]!.id)];
  if (nodeOf(built, bat.id, bat.pins[1]!.id) !== 0 || nodeOf(built, src.id, src.pins[1]!.id) !== 0 || !ends.includes(pos) || !ends.includes(nodeOf(built, src.id, src.pins[0]!.id)) || nodeOf(built, check.ic, "vcc") !== pos) {
    throw new Error("slow sidecar: rail-sag (battery) expects the IC's vcc and the load on the cell's pos, the load's far end on the source, every return on the datum");
  }
  const model = batteryModelForComponent(bat);
  const point = batteryOperatingPointFor(model, Math.max(0, Math.min(1, authoredBatteryCharge(bat))), ambientOf(circuit));
  const E = point.openCircuitVoltageV * model.nominalVoltageScale;
  const r = point.internalResistanceOhm;
  const R = param(load, "resistance");
  const pieces = pulseModelPieces(pulseParamsOf(src), horizonS);
  const railOf = (sv: number): number => (r > 0 ? (E / r + sv / R) / (1 / r + 1 / R + NODE_SHUNT_SIEMENS) : E);
  const railAt = (t: number): number => {
    const piece = pieces.find((pc) => t >= pc.t0 && t < pc.t1) ?? pieces[pieces.length - 1]!;
    return railOf(pieceValueAt(piece, Math.min(t, piece.t1)));
  };
  const flips: Array<{ t: number; powered: boolean }> = [];
  const initiallyPowered = railAt(0) >= gateLow;
  let powered = initiallyPowered;
  for (const piece of pieces) {
    const u0 = railOf(piece.v0);
    const u1 = railOf(piece.v1);
    // A jump between two pieces is a crossing at the jump; a ramp crosses where its line meets the edge.
    if ((u0 >= gateLow) !== powered) {
      powered = u0 >= gateLow;
      flips.push({ t: piece.t0, powered });
    }
    if ((u1 >= gateLow) !== powered && piece.v0 !== piece.v1) {
      const f = (gateLow - u0) / (u1 - u0);
      powered = u1 >= gateLow;
      flips.push({ t: piece.t0 + f * (piece.t1 - piece.t0), powered });
    }
  }
  return { railAt, flips: flips.filter((f) => f.t <= horizonS), initiallyPowered };
}

/** The model's events and readings for a fixture (the module header). Throws on a circuit a check cannot model. */
export function slowEvents(circuit: SimCircuit, model: SlowSidecarModel): SlowExpectation {
  const built = buildCoreNodes(circuit);
  const resets = model.resetsS ?? [];
  const events: SlowEvent[] = [];
  const readings: SlowReading[] = [];
  const ambient = ambientOf(circuit);

  for (const check of model.checks) {
    switch (check.check) {
      case "resistor-overload": {
        const r = componentOf(circuit, check.resistor, ["resistor"]);
        const pMax = specsForComponent(r)?.p_max;
        if (typeof pMax !== "number" || !(pMax > 0)) throw new Error(`slow sidecar: ${r.id} has no positive p_max`);
        const resistance = Number(r.params.resistance ?? 1000);
        const R = Math.max(1e-3, resistance);
        const i = voltsAcross(circuit, built, r) / R;
        const p = resistance > 0 ? i * i * resistance : 0;
        dwellLatches(p / pMax - 1, RESISTOR_DAMAGE_THRESHOLD, model.horizonS, resets).forEach((t, k) =>
          events.push({ label: `${r.id} resistor_overload #${k}`, type: "latch", componentId: r.id, kind: "resistor_overload", t, value: p, limit: pMax }),
        );
        break;
      }
      case "fuse": {
        const fz = componentOf(circuit, check.fuse, ["fuse"]);
        const rNormal = param(fz, "rNormal");
        const iRating = param(fz, "iRating");
        const i = Math.abs(voltsAcross(circuit, built, fz)) / rNormal;
        dwellLatches((i / iRating) ** 2 - 1, FUSE_I2T_THRESHOLD, model.horizonS, resets).forEach((t, k) =>
          events.push({ label: `${fz.id} fuse_tripped #${k}`, type: "latch", componentId: fz.id, kind: "fuse_tripped", t, value: i, limit: iRating }),
        );
        break;
      }
      case "led-overdrive": {
        const { led, vs, rOhms, lawAt } = ledSeries(circuit, check);
        if (thermalProfileForComponent(led)) throw new Error(`slow sidecar: ${led.id} has a thermal identity; led-overdrive models an ambient junction`);
        const amps = ledSeriesNode(lawAt(ambient), vs, rOhms, "anode-node").ledAmps;
        const ifMax = param(led, "iRated");
        dwellLatches((amps / ifMax) ** 2 - 1, LED_DAMAGE_THRESHOLD, model.horizonS, resets).forEach((t, k) =>
          events.push({ label: `${led.id} led_failed #${k}`, type: "latch", componentId: led.id, kind: "led_failed", t, value: amps, limit: ifMax }),
        );
        break;
      }
      case "ptc": {
        const ptc = componentOf(circuit, check.ptc, ["ptc_fuse"]);
        const rNormal = param(ptc, "rNormal");
        const iHold = param(ptc, "iHold");
        const v = Math.abs(voltsAcross(circuit, built, ptc));
        const iCold = v / rNormal;
        const iHot = v / (rNormal * 200);
        const cool = iHot < iHold / 2 && iHot * iHot * rNormal * 200 < iHold * iHold * rNormal * 0.1;
        const rate = iCold >= iHold ? Math.max(0, iCold * iCold - iHold * iHold) : 0;
        const threshold = 3 * iHold * iHold * 0.1;
        const starts = [0, ...resets];
        starts.forEach((start, s) => {
          const end = s + 1 < starts.length ? starts[s + 1]! : model.horizonS;
          let t = start;
          for (let k = 0; rate > 0; k++) {
            const trip = t + threshold / rate;
            if (trip > end) break;
            events.push({ label: `${ptc.id} trip #${k} after ${start}`, type: "ptc-trip", componentId: ptc.id, t: trip });
            if (!cool) break;
            const recover = trip + PTC_RECOVERY_S;
            if (recover > end) break;
            events.push({ label: `${ptc.id} recovery #${k} after ${start}`, type: "ptc-recover", componentId: ptc.id, t: recover });
            t = recover;
          }
        });
        break;
      }
      case "output-sag": {
        const ic = circuit.components.find((c) => c.id === check.ic);
        if (!ic) throw new Error(`slow sidecar: no component "${check.ic}"`);
        const load = componentOf(circuit, check.load, ["resistor"]);
        const fixed = fixedVolts(circuit, built);
        const vS = (fixed.get(nodeOf(built, ic.id, "vcc")) ?? Number.NaN) - (fixed.get(nodeOf(built, ic.id, "gnd")) ?? Number.NaN);
        if (!(vS > 0)) throw new Error(`slow sidecar: ${ic.id}'s supply is not on ideal sources`);
        const pinNode = nodeOf(built, ic.id, check.pin);
        const ends = [nodeOf(built, load.id, load.pins[0]!.id), nodeOf(built, load.id, load.pins[1]!.id)];
        if (!ends.includes(pinNode) || !ends.includes(0)) throw new Error("slow sidecar: output-sag expects the load from the pin to the datum");
        const specs = specsForComponent(ic);
        const ioMax = specs?.io_max;
        const rOut = ioMax && ioMax > 0 ? Math.max(GATE_R_OUT, Math.min(GATE_R_OUT_MAX, vS / (ioMax * 4))) : GATE_R_OUT;
        const rLoad = param(load, "resistance");
        const value = (vS * rLoad) / (rLoad + rOut);
        const { vih } = thresholdsFor(specs, vS);
        if (!(value < vih)) throw new Error("slow sidecar: the loaded HIGH output does not sag");
        [0, ...resets].forEach((start, k) => {
          const t = start + OUTPUT_SAG_WINDOW_S;
          if (t <= model.horizonS) {
            events.push({ label: `${ic.id}.${check.pin} output_sag #${k}`, type: "latch", componentId: ic.id, kind: "output_sag", pinId: check.pin, t, value, limit: vih });
          }
        });
        break;
      }
      case "resistor-heat": {
        const r = componentOf(circuit, check.resistor, ["resistor"]);
        const profile = thermalProfileForComponent(r);
        if (!profile) throw new Error(`slow sidecar: ${r.id} has no exact thermal identity`);
        const resistance = param(r, "resistance");
        const i = voltsAcross(circuit, built, r) / Math.max(1e-3, resistance);
        const p = i * i * resistance;
        const rise = p * profile.network.rThetaJA_CPerW;
        const tau = thermalTimeConstantS(profile.network);
        for (const t of check.atS) {
          readings.push({ label: `${r.id} T(${t})`, what: "temperature", componentId: r.id, t, expected: ambient + rise * -Math.expm1(-t / tau), ambientC: ambient });
        }
        break;
      }
      case "enable-sag": {
        const ic = circuit.components.find((c) => c.id === check.ic);
        if (!ic) throw new Error(`slow sidecar: no component "${check.ic}"`);
        const load = componentOf(circuit, check.load, ["resistor"]);
        const enable = componentOf(circuit, check.enable, ["pulse_source", "pulse_gen"]);
        const fixed = fixedVolts(circuit, built);
        const vS = (fixed.get(nodeOf(built, ic.id, "vcc")) ?? Number.NaN) - (fixed.get(nodeOf(built, ic.id, "gnd")) ?? Number.NaN);
        if (!(vS > 0)) throw new Error(`slow sidecar: ${ic.id}'s supply is not on ideal sources`);
        const ends = [nodeOf(built, load.id, load.pins[0]!.id), nodeOf(built, load.id, load.pins[1]!.id)];
        if (!ends.includes(nodeOf(built, ic.id, check.pin)) || !ends.includes(0)) throw new Error("slow sidecar: enable-sag expects the load from the pin to the datum");
        if (nodeOf(built, enable.id, enable.pins[0]!.id) !== nodeOf(built, ic.id, "/oe")) throw new Error("slow sidecar: enable-sag expects the pulse on the IC's /oe");
        const p = pulseParamsOf(enable);
        if (!(p.v1 > p.v2)) throw new Error("slow sidecar: enable-sag expects /oe to fall from v1 (disabled) to v2 (enabled)");
        // The enable is read at the modelled pulse's step (Sources point 3), which needs a ramp within RAMP_STEP_RATIO of the flat stretch.
        const step = pulseModelPieces(p, model.horizonS).find((pc) => pc.t0 > 0 && pc.v0 === p.v2 && pc.v1 === p.v2);
        if (!step || step.t0 !== p.td + p.tr / 2) throw new Error("slow sidecar: enable-sag needs the enable's fall to be a modelled step");
        const specs = specsForComponent(ic);
        const tpzh = specs?.switching_ns?.oe?.tpzh;
        if (typeof tpzh !== "number" || !(tpzh >= 0)) throw new Error(`slow sidecar: ${ic.id} has no catalog tPZH on its oe path`);
        const ioMax = specs?.io_max;
        const rOut = ioMax && ioMax > 0 ? Math.max(GATE_R_OUT, Math.min(GATE_R_OUT_MAX, vS / (ioMax * 4))) : GATE_R_OUT;
        const rLoad = param(load, "resistance");
        const value = (vS * rLoad) / (rLoad + rOut);
        const { vih } = thresholdsFor(specs, vS);
        if (!(value < vih)) throw new Error("slow sidecar: the enabled HIGH output does not sag");
        const t = step.t0 + tpzh * 1e-9 + OUTPUT_SAG_WINDOW_S;
        if (t <= model.horizonS) events.push({ label: `${ic.id}.${check.pin} output_sag after the enable`, type: "latch", componentId: ic.id, kind: "output_sag", pinId: check.pin, t, value, limit: vih });
        break;
      }
      case "sag-edges":
      case "committed-target":
        // Each judged by its own runner (`runSagEdges`, `runCommittedTarget`), not as timed events.
        break;
      case "rail-sag": {
        const ic = circuit.components.find((c) => c.id === check.ic);
        if (!ic) throw new Error(`slow sidecar: no component "${check.ic}"`);
        const pullup = componentOf(circuit, check.pullup, ["resistor"]);
        const pull = componentOf(circuit, check.pullSource, ["voltage_source"]);
        const vPull = Number(pull.params.voltage ?? 5);
        const pinNode = nodeOf(built, ic.id, check.pin);
        const pullEnds = [nodeOf(built, pullup.id, pullup.pins[0]!.id), nodeOf(built, pullup.id, pullup.pins[1]!.id)];
        if (!pullEnds.includes(pinNode) || !pullEnds.includes(nodeOf(built, pull.id, pull.pins[0]!.id)) || nodeOf(built, pull.id, pull.pins[1]!.id) !== 0) {
          throw new Error("slow sidecar: rail-sag expects the pull-up from the pin to the pull source's pos, its neg on the datum");
        }
        const rPull = param(pullup, "resistance");
        const specs = specsForComponent(ic);
        const ioMax = specs?.io_max;
        const vccMin = specs?.vcc_range?.min ?? 2.0;
        const gateLow = vccMin * 0.9;
        const { railAt, flips, initiallyPowered } = railSagRail(circuit, built, check, model.horizonS, gateLow);
        const sagAt = (t: number): { value: number; limit: number } => {
          const u = railAt(t);
          const rOut = ioMax && ioMax > 0 ? Math.max(GATE_R_OUT, Math.min(GATE_R_OUT_MAX, u / (ioMax * 4))) : GATE_R_OUT;
          return { value: (vPull * rOut) / (rOut + rPull), limit: thresholdsFor(specs, u).vil };
        };
        const latchAt = (t: number, k: number): void => {
          const { value, limit } = sagAt(t);
          if (!(value > limit)) throw new Error(`slow sidecar: the pulled-up LOW output does not sag at t=${t}`);
          if (t <= model.horizonS) events.push({ label: `${ic.id}.${check.pin} output_sag #${k}`, type: "latch", componentId: ic.id, kind: "output_sag", pinId: check.pin, t, value, limit });
        };
        let k = 0;
        if (initiallyPowered) latchAt(OUTPUT_SAG_WINDOW_S, k++);
        for (const flip of flips) {
          if (flip.powered) latchAt(flip.t + OUTPUT_SAG_WINDOW_S, k++);
          else events.push({ label: `${ic.id}.${check.pin} output_sag clears at ${flip.t}`, type: "sag-clear", componentId: ic.id, kind: "output_sag", pinId: check.pin, t: flip.t });
        }
        break;
      }
      case "led-heat": {
        const curve = ledHeatCurve(circuit, check, Math.max(...check.atS, ...check.currentAtS));
        for (const t of check.atS) readings.push({ label: `${check.led} T(${t})`, what: "temperature", componentId: check.led, t, expected: curve(t).tempC, ambientC: ambient });
        for (const t of check.currentAtS) readings.push({ label: `${check.led} I(${t})`, what: "current", componentId: check.led, t, expected: curve(t).amps, ambientC: ambient });
        break;
      }
    }
  }
  events.sort((a, b) => a.t - b.t);
  readings.sort((a, b) => a.t - b.t);
  return { events, readings };
}

// ─── the runner ─────────────────────────────────────────────────────────────────

/** Recorded, never gated (Q-O31): classic's own event times and readings against the analytic answer. */
export const CLASSIC_SLOW_TIME_ID = "classic-slow-event-step-time";
const CLASSIC_SLOW_TIME_REF =
  "core/README.md W1g point 5 and 19.1 (Q-O31): classic's integrators are the exact solution sampled per step, so its " +
  "latch falls on the end of the step that crosses; the core is held to the analytic time and classic's is recorded. " +
  "An output_sag classic latches late (its two-step debounce on growing steps, after an enable or a reset) is a classic " +
  "accuracy shortfall recorded the same way (ruling R35 (a)), never an excused divergence.";

/** Classic's time against the fixture's record of it (`classicS`): deterministic, so anything past rounding means the record is stale. */
const CLASSIC_RECORD_REL = 1e-9;

function sidecarMismatch(t: number, id: string, want: unknown, got: unknown, detail: string): Mismatch {
  return { sampleTime: t, category: "sidecar", id, classicValue: want, coreValue: got, detail };
}

function failureKeyOf(f: Pick<SimFailure, "kind" | "componentId" | "pinId">): string {
  return `${f.kind}/${f.componentId}/${f.pinId ?? ""}`;
}

export interface SlowRunOutcome {
  readonly mismatches: Mismatch[];
  readonly recorded: KnownDivergenceHit[];
  /** Classic's measured time per event, in `expectation.events` order (`NaN`: never fired): what `classicS` records. */
  readonly classicS: number[];
}

interface CoreObservation {
  readonly latches: SimFailure[];
  /** Tripped PTC ids at each stop. */
  readonly ptcAt: Map<number, ReadonlySet<string>>;
  /** Failure keys (`failureKeyOf`) reported at each stop. */
  readonly failuresAt: Map<number, ReadonlySet<string>>;
  readonly readingAt: Map<string, number>;
}

function observeCore(circuit: SimCircuit, model: SlowSidecarModel, exp: SlowExpectation): CoreObservation {
  const resets = new Set(model.resetsS ?? []);
  const stops = new Set<number>([model.horizonS, ...resets]);
  for (const e of exp.events) {
    if (e.type !== "latch") {
      const tol = slowTimeTolerance(e.t);
      stops.add(e.t - tol);
      stops.add(e.t + tol);
    }
  }
  for (const r of exp.readings) stops.add(r.t);
  const ordered = [...stops].filter((t) => t > 0 && t <= model.horizonS).sort((a, b) => a - b);

  const engine = new CoreEngine();
  const latches: SimFailure[] = [];
  const ptcAt = new Map<number, ReadonlySet<string>>();
  const failuresAt = new Map<number, ReadonlySet<string>>();
  const readingAt = new Map<string, number>();
  try {
    engine.load(circuit, CORE_IMPLEMENTED_KINDS);
    latches.push(...engine.takeLatchedFailures());
    for (const t of ordered) {
      engine.advance(t);
      if (!engine.lastConverged) throw new Error(`the core did not converge at t=${engine.simTime}`);
      latches.push(...engine.takeLatchedFailures());
      ptcAt.set(t, engine.getPtcTripped());
      failuresAt.set(t, new Set(Object.values(engine.getFailures()).map(failureKeyOf)));
      for (const r of exp.readings) {
        if (r.t !== t) continue;
        const value = r.what === "temperature" ? engine.getThermalState(r.componentId)?.temperatureC : engine.getElementI()[r.componentId];
        readingAt.set(r.label, value ?? Number.NaN);
      }
      if (resets.has(t)) {
        engine.resetFailures();
        latches.push(...engine.takeLatchedFailures());
      }
    }
  } finally {
    engine.dispose();
  }
  return { latches, ptcAt, failuresAt, readingAt };
}

interface ClassicObservation {
  /** Every failure key's appearances, in order, each the end of the accepted step it was first seen at. */
  readonly latches: Array<{ key: string; t: number; failure: SimFailure }>;
  readonly ptc: Array<{ id: string; tripped: boolean; t: number }>;
  /** Every failure key's disappearances, in order. */
  readonly clears: Array<{ key: string; t: number }>;
  readonly readingAt: Map<string, number>;
}

function observeClassic(circuit: SimCircuit, model: SlowSidecarModel, exp: SlowExpectation): ClassicObservation {
  const resets = new Set(model.resetsS ?? []);
  const stops = [...new Set([model.horizonS, ...resets, ...exp.readings.map((r) => r.t)])].filter((t) => t > 0 && t <= model.horizonS).sort((a, b) => a - b);
  const runner = new HeadlessRunner();
  runner.load(circuit);
  const engine = runner.engine;
  const latches: ClassicObservation["latches"] = [];
  const ptc: ClassicObservation["ptc"] = [];
  const clears: ClassicObservation["clears"] = [];
  const readingAt = new Map<string, number>();
  let present = new Set<string>();
  let tripped = new Set<string>();
  const poll = (t: number): void => {
    const now = new Map(Object.values(engine.getFailures()).map((f) => [failureKeyOf(f), f] as const));
    for (const [key, failure] of now) if (!present.has(key)) latches.push({ key, t, failure: { ...failure } });
    for (const key of present) if (!now.has(key)) clears.push({ key, t });
    present = new Set(now.keys());
    const nowTripped = new Set(engine.getPtcTripped());
    for (const id of nowTripped) if (!tripped.has(id)) ptc.push({ id, tripped: true, t });
    for (const id of tripped) if (!nowTripped.has(id)) ptc.push({ id, tripped: false, t });
    tripped = nowTripped;
  };
  poll(engine.simTime);
  for (const t of stops) {
    const result = runner.run({ durationS: t - engine.simTime, onSample: (s) => poll(s.simTime) });
    if (result.hitMinStep || result.hitStepCap) throw new Error(`classic stopped short of t=${t}`);
    for (const r of exp.readings) {
      if (r.t !== t) continue;
      const value = r.what === "temperature" ? engine.getThermalState(r.componentId)?.temperatureC : engine.getElementI()[r.componentId];
      readingAt.set(r.label, value ?? Number.NaN);
    }
    if (resets.has(t)) {
      engine.resetFailures();
      poll(engine.simTime);
    }
  }
  return { latches, ptc, clears, readingAt };
}

/** The gate 2c judgement for one fixture (the module header). */
export function judgeSlow(model: SlowSidecarModel, exp: SlowExpectation, core: CoreObservation, classic: ClassicObservation): SlowRunOutcome {
  const mismatches: Mismatch[] = [];
  const recorded: KnownDivergenceHit[] = [];
  const classicS: number[] = [];

  // The core's latch log against the analytic latches, one for one.
  const wantLatches = exp.events.filter((e) => e.type === "latch");
  if (core.latches.length !== wantLatches.length) {
    mismatches.push(
      sidecarMismatch(0, "core latches", wantLatches.map((e) => e.label), core.latches.map((f) => `${failureKeyOf(f)}@${f.since}`), "the core's latch log does not hold exactly the model's latches"),
    );
  }
  wantLatches.forEach((want, i) => {
    const got = core.latches[i];
    if (!got) return;
    const tol = slowTimeTolerance(want.t);
    if (failureKeyOf(got) !== failureKeyOf({ kind: want.kind!, componentId: want.componentId, pinId: want.pinId })) {
      mismatches.push(sidecarMismatch(want.t, want.label, failureKeyOf({ kind: want.kind!, componentId: want.componentId, pinId: want.pinId }), failureKeyOf(got), "the core latched a different (kind, componentId, pinId)"));
      return;
    }
    if (!(Math.abs(got.since - want.t) <= tol)) {
      mismatches.push(sidecarMismatch(want.t, `${want.label} since`, want.t, got.since, `the core latched ${(got.since - want.t).toExponential(3)} s off the analytic time, beyond ${tol.toExponential(2)} s`));
    }
    const [absTol, relTol] = failureTolerance(want.kind!);
    for (const field of ["value", "limit"] as const) {
      if (!withinTolerance(want[field]!, got[field] ?? Number.NaN, absTol, relTol)) {
        mismatches.push(sidecarMismatch(want.t, `${want.label} ${field}`, want[field], got[field], `the core's ${field} differs from the model's beyond decision 6's tolerance`));
      }
    }
    const message = classicFailureMessage(got);
    if (message !== got.message) mismatches.push(sidecarMismatch(want.t, `${want.label} message`, message, got.message, "the core's message is not classic's template on its own numbers"));
  });

  // Sag clears, bracketed: reported just before the analytic instant, gone just after.
  for (const e of exp.events) {
    if (e.type !== "sag-clear") continue;
    const tol = slowTimeTolerance(e.t);
    const key = failureKeyOf({ kind: e.kind!, componentId: e.componentId, pinId: e.pinId });
    const before = core.failuresAt.get(e.t - tol)?.has(key);
    const after = core.failuresAt.get(e.t + tol)?.has(key);
    if (before !== true || after !== false) {
      mismatches.push(sidecarMismatch(e.t, e.label, "true then false", `${before} then ${after}`, `the core's sag must clear within ${tol.toExponential(2)} s of the rail leaving the power window`));
    }
  }

  // PTC transitions, bracketed.
  for (const e of exp.events) {
    if (e.type !== "ptc-trip" && e.type !== "ptc-recover") continue;
    const tol = slowTimeTolerance(e.t);
    const before = core.ptcAt.get(e.t - tol)?.has(e.componentId);
    const after = core.ptcAt.get(e.t + tol)?.has(e.componentId);
    const trip = e.type === "ptc-trip";
    if (before !== !trip || after !== trip) {
      mismatches.push(sidecarMismatch(e.t, e.label, `${!trip} then ${trip}`, `${before} then ${after}`, `the core's tripped flag must change within ${tol.toExponential(2)} s of the analytic instant`));
    }
  }

  // Readings.
  for (const r of exp.readings) {
    const got = core.readingAt.get(r.label) ?? Number.NaN;
    const ok =
      r.what === "temperature"
        ? Math.abs(got - r.expected) <= temperatureTolerance(r.expected, r.ambientC)
        : withinTolerance(r.expected, got, AMP_ABS_TOL, AMP_REL_TOL);
    if (!ok) mismatches.push(sidecarMismatch(r.t, r.label, r.expected, got, `the core's ${r.what} is off the model's answer beyond its tolerance`));
    const classicValue = classic.readingAt.get(r.label) ?? Number.NaN;
    recorded.push({
      mismatch: sidecarMismatch(r.t, `classic ${r.label}`, classicValue, r.expected, `classic ${classicValue} against the model's ${r.expected}`),
      divergenceId: CLASSIC_SLOW_TIME_ID,
      decisionRef: CLASSIC_SLOW_TIME_REF,
      reason: "recorded, not gated",
    });
  }

  // Classic fires every event; its times are recorded against the analytic ones.
  const seenOfKey = new Map<string, number>();
  exp.events.forEach((e) => {
    let classicT: number | undefined;
    if (e.type === "latch") {
      const key = failureKeyOf({ kind: e.kind!, componentId: e.componentId, pinId: e.pinId });
      const k = seenOfKey.get(key) ?? 0;
      seenOfKey.set(key, k + 1);
      classicT = classic.latches.filter((l) => l.key === key)[k]?.t;
    } else if (e.type === "sag-clear") {
      const key = failureKeyOf({ kind: e.kind!, componentId: e.componentId, pinId: e.pinId });
      const k = seenOfKey.get(`clear:${key}`) ?? 0;
      seenOfKey.set(`clear:${key}`, k + 1);
      classicT = classic.clears.filter((c) => c.key === key)[k]?.t;
    } else {
      const key = `${e.componentId}/${e.type}`;
      const k = seenOfKey.get(key) ?? 0;
      seenOfKey.set(key, k + 1);
      classicT = classic.ptc.filter((p) => p.id === e.componentId && p.tripped === (e.type === "ptc-trip"))[k]?.t;
    }
    classicS.push(classicT ?? Number.NaN);
    if (classicT === undefined) {
      mismatches.push(sidecarMismatch(e.t, `classic ${e.label}`, "fires", "never", "classic must fire every event the fixture holds the core to (the classic-fires proof)"));
      return;
    }
    recorded.push({
      mismatch: sidecarMismatch(e.t, `classic ${e.label}`, classicT, e.t, `classic at ${classicT} s, the model at ${e.t} s (${(classicT - e.t).toExponential(3)} s)`),
      divergenceId: CLASSIC_SLOW_TIME_ID,
      decisionRef: CLASSIC_SLOW_TIME_REF,
      reason: "recorded, not gated",
    });
  });
  if (classic.latches.length !== wantLatches.length) {
    mismatches.push(sidecarMismatch(0, "classic latches", wantLatches.length, classic.latches.map((l) => `${l.key}@${l.t}`), "classic latched a different number of failures than the model"));
  }
  return { mismatches, recorded, classicS };
}

/**
 * The model's latch counts for a `sag-edges` check. An edge that commands
 * LOW discharges the capacitor through `R_out` from `vS` and leaves the LOW
 * band after `τ·ln(vS/vil)`; one that commands HIGH charges it from 0 and
 * enters the HIGH band after `τ·ln(vS/(vS − vih))`, `τ = R_out·C` (the node
 * shunt moves these by 1e-12 relative). An edge latches when that exceeds
 * `OUTPUT_SAG_WINDOW` (point 9). The load counts as a HIGH edge when the
 * output starts HIGH on the uncharged capacitor (the fixtures start it LOW
 * instead: classic's 1 ps load seed charges the capacitor a little where the
 * core's load holds it, which the plain t = 0 sample would report). An edge
 * must also end before the horizon.
 */
export function sagEdgeCounts(
  circuit: SimCircuit,
  check: Extract<SlowCheck, { check: "sag-edges" }>,
  horizonS: number,
): { low: number; high: number; edges: number; tauS: number } {
  const built = buildCoreNodes(circuit);
  const ic = circuit.components.find((c) => c.id === check.ic);
  if (!ic) throw new Error(`slow sidecar: no component "${check.ic}"`);
  const cap = componentOf(circuit, check.capacitor, ["capacitor"]);
  const src = componentOf(circuit, check.source, ["pulse_source", "pulse_gen"]);
  const pinNode = nodeOf(built, ic.id, check.pin);
  const capEnds = [nodeOf(built, cap.id, cap.pins[0]!.id), nodeOf(built, cap.id, cap.pins[1]!.id)];
  if (!capEnds.includes(pinNode) || !capEnds.includes(0)) throw new Error("slow sidecar: sag-edges expects the capacitor from the pin to the datum");
  if (Number(cap.params.esr ?? 0) !== 0) throw new Error("slow sidecar: sag-edges models an ideal capacitor");
  const fixed = fixedVolts(circuit, built);
  const vS = (fixed.get(nodeOf(built, ic.id, "vcc")) ?? Number.NaN) - (fixed.get(nodeOf(built, ic.id, "gnd")) ?? Number.NaN);
  const specs = specsForComponent(ic);
  const ioMax = specs?.io_max;
  const rOut = ioMax && ioMax > 0 ? Math.max(GATE_R_OUT, Math.min(GATE_R_OUT_MAX, vS / (ioMax * 4))) : GATE_R_OUT;
  const tau = rOut * param(cap, "capacitance");
  const { vil, vih } = thresholdsFor(specs, vS);
  const lowLatches = tau * Math.log(vS / vil) > OUTPUT_SAG_WINDOW_S;
  const highLatches = tau * Math.log(vS / (vS - vih)) > OUTPUT_SAG_WINDOW_S;
  // An inverter: the input's rising edge commands LOW, its falling edge HIGH. Each cycle's first corner takes the
  // input from v1 to v2, its second back to v1.
  const p = pulseParamsOf(src);
  if (p.v1 === p.v2) throw new Error("slow sidecar: sag-edges needs a pulse that moves");
  let firsts = 0;
  let seconds = 0;
  for (let c = 0; ; c++) {
    const base = p.td + c * p.per;
    if (base >= horizonS) break;
    if (base + p.tr + 1e-6 <= horizonS) firsts++;
    if (base + p.tr + p.pw + p.tf + 1e-6 <= horizonS) seconds++;
    if (p.per === 0) break;
  }
  const rises = p.v2 > p.v1 ? firsts : seconds;
  const falls = p.v2 > p.v1 ? seconds : firsts;
  const startsHigh = p.v1 <= vil;
  return { low: lowLatches ? rises : 0, high: highLatches ? falls + (startsHigh ? 1 : 0) : 0, edges: rises + falls, tauS: tau };
}

const direction = (f: Pick<SimFailure, "value" | "limit">): "HIGH" | "LOW" => ((f.value ?? 0) < (f.limit ?? 0) ? "HIGH" : "LOW");

/** A `sag-edges` fixture: both engines' latch counts per direction against the model's; classic's latch times are the record. */
function runSagEdges(circuit: SimCircuit, model: SlowSidecarModel, check: Extract<SlowCheck, { check: "sag-edges" }>): SlowRunOutcome {
  const want = sagEdgeCounts(circuit, check, model.horizonS);
  const mismatches: Mismatch[] = [];
  const recorded: KnownDivergenceHit[] = [];
  const key = failureKeyOf({ kind: "output_sag", componentId: check.ic, pinId: check.pin });

  const coreDirs: string[] = [];
  const engine = new CoreEngine();
  try {
    engine.load(circuit, CORE_IMPLEMENTED_KINDS);
    const chunk = 1e-4;
    for (let k = 1; k * chunk <= model.horizonS + 1e-12; k++) {
      engine.advance(Math.min(k * chunk, model.horizonS));
      for (const f of engine.takeLatchedFailures()) {
        if (failureKeyOf(f) !== key) mismatches.push(sidecarMismatch(f.since, failureKeyOf(f), "none", f.message, "the core latched a failure the fixture does not model"));
        else coreDirs.push(direction(f));
      }
    }
  } finally {
    engine.dispose();
  }

  const classicLatches: Array<{ t: number; dir: string }> = [];
  const runner = new HeadlessRunner();
  runner.load(circuit);
  let present = false;
  runner.run({
    durationS: model.horizonS,
    onSample: (s) => {
      const f = Object.values(runner.engine.getFailures()).find((x) => failureKeyOf(x) === key);
      if (f && !present) classicLatches.push({ t: s.simTime, dir: direction(f) });
      present = f !== undefined;
    },
  });

  const count = (dirs: readonly string[], d: string) => dirs.filter((x) => x === d).length;
  for (const d of ["LOW", "HIGH"] as const) {
    const expected = d === "LOW" ? want.low : want.high;
    const core = count(coreDirs, d);
    const classic = count(classicLatches.map((l) => l.dir), d);
    if (core !== expected) mismatches.push(sidecarMismatch(0, `core ${d} latches`, expected, core, `the core must latch ${expected} ${d} edge(s) (tau ${want.tauS.toExponential(3)} s against the ${OUTPUT_SAG_WINDOW_S} s window)`));
    if (classic !== expected) mismatches.push(sidecarMismatch(0, `classic ${d} latches`, expected, classic, `classic must latch the same ${expected} ${d} edge(s)`));
  }
  // No latch at all is a legitimate answer (an edge that ends inside the window); no edge at all is not a check.
  if (want.edges === 0) mismatches.push(sidecarMismatch(0, "sag-edges", "at least one edge", 0, "the source has no edge inside the horizon: the check would pass vacuously"));
  recorded.push({
    mismatch: sidecarMismatch(0, "classic sag-edges", classicLatches.length, coreDirs.length, `classic latched ${classicLatches.length}, the core ${coreDirs.length}`),
    divergenceId: CLASSIC_SLOW_TIME_ID,
    decisionRef: CLASSIC_SLOW_TIME_REF,
    reason: "recorded, not gated",
  });
  return { mismatches, recorded, classicS: classicLatches.map((l) => l.t) };
}

/**
 * The pending delays of `check`'s pin in a core trace: each change of the
 * pin's key ends one, which began at the enable's last ramp start before it
 * (the instant the enable started to move; the part reads it somewhere on
 * that ramp). The change at load is not a delay.
 */
export function pendingDelayWindows(
  circuit: SimCircuit,
  check: Extract<SlowCheck, { check: "committed-target" }>,
  trace: readonly { t: number; key: string }[],
  horizonS: number,
): PendingDelayWindow[] {
  const enable = componentOf(circuit, check.enable, ["pulse_source", "pulse_gen"]);
  const rampStarts = pulsePieces(pulseParamsOf(enable), horizonS)
    .filter((pc) => pc.v0 !== pc.v1)
    .map((pc) => pc.t0);
  const windows: PendingDelayWindow[] = [];
  for (const e of trace) {
    if (e.key !== `${check.ic}/${check.pin}` || !(e.t > 0)) continue;
    const from = rampStarts.filter((t) => t <= e.t).pop();
    if (from !== undefined) windows.push({ componentId: check.ic, pinId: check.pin, fromS: from, untilS: e.t });
  }
  return windows;
}

/** A `committed-target` fixture (W1g Q12): the sets at every classic accepted step, differences excused only inside a pending delay. */
function runCommittedTarget(circuit: SimCircuit, model: SlowSidecarModel, check: Extract<SlowCheck, { check: "committed-target" }>): SlowRunOutcome {
  const mismatches: Mismatch[] = [];
  const recorded: KnownDivergenceHit[] = [];
  const key = failureKeyOf({ kind: "output_sag", componentId: check.ic, pinId: check.pin });
  const mismatchId = `output_sag\x00${check.ic}\x00${check.pin}`;

  const classicAt: Array<{ t: number; has: boolean }> = [];
  const runner = new HeadlessRunner();
  runner.load(circuit);
  runner.run({
    durationS: model.horizonS,
    onSample: (s) => classicAt.push({ t: s.simTime, has: Object.values(runner.engine.getFailures()).some((f) => failureKeyOf(f) === key) }),
  });

  const coreAt: boolean[] = [];
  let trace: Array<{ t: number; key: string }> = [];
  const engine = new CoreEngine();
  try {
    engine.load(circuit, CORE_IMPLEMENTED_KINDS);
    engine.enableTrace(true);
    for (const { t } of classicAt) {
      if (t > engine.simTime) engine.advance(t);
      coreAt.push(Object.values(engine.getFailures()).some((f) => failureKeyOf(f) === key));
    }
    trace = engine.readTrace();
  } finally {
    engine.dispose();
  }

  const windows = pendingDelayWindows(circuit, check, trace, model.horizonS);
  let excused = 0;
  classicAt.forEach(({ t, has }, i) => {
    if (has === coreAt[i]) return;
    const mismatch: Mismatch = { sampleTime: t, category: "failures", id: mismatchId, classicValue: has, coreValue: coreAt[i], detail: "the output_sag sets differ" };
    if (explainCommittedTarget(mismatch, windows)) {
      excused++;
      recorded.push({ mismatch, divergenceId: W1G_SAG_COMMITTED_TARGET.id, decisionRef: W1G_SAG_COMMITTED_TARGET.decisionRef, reason: W1G_SAG_COMMITTED_TARGET.reason });
    } else {
      mismatches.push({ ...mismatch, detail: `the output_sag sets differ outside every pending delay (${JSON.stringify(windows.map((w) => [w.fromS, w.untilS]))})` });
    }
  });
  if (windows.length === 0) mismatches.push(sidecarMismatch(0, "pending delays", "at least one", 0, "the core never committed a change of the pin: no delay was exercised"));
  if (excused === 0) mismatches.push(sidecarMismatch(0, "w1g-sag-committed-target", "at least one difference inside a delay", 0, "the sets never differed: the divergence was not exercised"));
  const classicLatches: number[] = [];
  classicAt.forEach(({ t, has }, i) => {
    if (has && !(i > 0 && classicAt[i - 1]!.has)) classicLatches.push(t);
  });
  if (classicLatches.length === 0 || !coreAt.some(Boolean)) mismatches.push(sidecarMismatch(0, "latches", "both latch", `${classicLatches.length} / ${coreAt.filter(Boolean).length}`, "both engines must latch the sag"));
  return { mismatches, recorded, classicS: classicLatches };
}

/** Runs one fixture's slow checks on both engines and judges them, including the fixture's record of classic's times. */
export function runSlowChecks(circuit: SimCircuit, model: SlowSidecarModel): SlowRunOutcome {
  const alone = model.checks.find((c) => c.check === "sag-edges" || c.check === "committed-target");
  if (alone && model.checks.length !== 1) throw new Error(`slow sidecar: a ${alone.check} check stands alone in its fixture`);
  if (alone?.check === "sag-edges") return withRecord(model, runSagEdges(circuit, model, alone), (i) => `classic latch ${i}`);
  if (alone?.check === "committed-target") return withRecord(model, runCommittedTarget(circuit, model, alone), (i) => `classic latch ${i}`);
  return runTimedChecks(circuit, model);
}

/** The fixture's record of classic's times (`classicS`) against what classic does now. */
function withRecord(model: SlowSidecarModel, outcome: SlowRunOutcome, label: (i: number) => string): SlowRunOutcome {
  if (model.classicS.length !== outcome.classicS.length) {
    outcome.mismatches.push(sidecarMismatch(0, "classicS", model.classicS.length, outcome.classicS.length, "the fixture records a different number of classic event times than classic produced"));
    return outcome;
  }
  outcome.classicS.forEach((t, i) => {
    const want = model.classicS[i]!;
    if (!(Math.abs(t - want) <= CLASSIC_RECORD_REL * Math.abs(want) + 1e-15)) {
      outcome.mismatches.push(sidecarMismatch(want, `classicS[${i}] ${label(i)}`, want, t, "classic no longer fires at the time the fixture records: re-measure and update the record"));
    }
  });
  return outcome;
}

function runTimedChecks(circuit: SimCircuit, model: SlowSidecarModel): SlowRunOutcome {
  const exp = slowEvents(circuit, model);
  if (exp.events.length + exp.readings.length === 0) {
    return { mismatches: [sidecarMismatch(0, "slow", "at least one event", 0, "the model has no event or reading: the check would pass vacuously")], recorded: [], classicS: [] };
  }
  const core = observeCore(circuit, model, exp);
  const classic = observeClassic(circuit, model, exp);
  return withRecord(model, judgeSlow(model, exp, core, classic), (i) => exp.events[i]!.label);
}
