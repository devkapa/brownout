/**
 * True linearized small-signal AC analysis (Wave A5) — the SPICE-style
 * counterpart of the large-signal "ac-sweep" runner in
 * analysis/run-ac-sweep.ts. That runner drives the real nonlinear circuit
 * with a sine and DFTs the response, so its results include distortion and
 * drive-amplitude effects; THIS module linearizes every device about a
 * committed DC operating point and solves the complex admittance system
 * exactly, so its results are the ideal small-signal transfer function. Both
 * modes stay available on purpose: they answer different questions.
 *
 * Flow per analysis:
 *   1. engine.dcOperatingPoint() — the Wave A3 ladder — must converge; the
 *      committed point (node voltages, regimes, latches, trip states) is the
 *      linearization bias and stays held for the whole sweep.
 *   2. One AcSystem (bordered 2n real system, see ac-system.ts) is built for
 *      the loaded matrix layout and reused across every frequency point:
 *      clear() keeps the sparse backend's discovered pattern, so the sweep
 *      pays the symbolic analysis once and each point is a value-only
 *      restamp + factorization.
 *   3. Per frequency, every registered device with an acStamp hook stamps
 *      its linearized contribution at the committed OP; devices without the
 *      hook are documented high-Z (no small-signal contribution), except
 *      committed digital output stages, which get the generic
 *      Thevenin-to-rail default below. The designated input source injects
 *      a unit 1 + j0 drive; every other independent source is AC-zeroed
 *      (voltage branches become AC shorts, current sources become opens).
 *
 * MCU boards are rejected outright: firmware has no small-signal model, and
 * silently treating a running core's outputs as bias-frozen would present a
 * confident wrong answer. Circuits without MCUs cover every registered kind
 * via acStamp hooks or the defaults here.
 */

import { AcSystem, stampAcAdmittance } from "./ac-system.js";
import {
  getDeviceModel,
  type AcDeviceContext,
  type DeviceComponent,
  type DeviceModel,
} from "./device-registry.js";
import { isMcuBoardKind } from "../../circuit/arduino.js";
import { signalGenEnabled } from "./waveform.js";
import {
  DIGITAL_DELAY_PREFIX,
  NODE_RSHUNT_G,
  type DcOperatingPointResult,
  type SimEngine,
} from "./sim-engine.js";

/**
 * Source kinds that can carry the AC input designation. Each one's acStamp
 * consults ctx.acInputMagnitude, so designating anything else would silently
 * inject nothing; rejecting up front turns that mistake into a clear error.
 */
const AC_INPUT_KINDS = new Set([
  "voltage_source",
  "battery_pack",
  "bench_psu",
  "clock",
  "clock_gen",
  "pulse_source",
  "pulse_gen",
  "signal_gen",
  // Wave A7: unit 1 A Norton injection (SPICE I-element AC convention);
  // reported node magnitudes are then transfer impedances in ohms.
  "current_source",
]);

// Sanity window for analysis frequencies. Values outside it are not
// physically meaningful for this engine and their reactive admittances
// over/underflow double arithmetic (omega*C collapsing to 0 or Infinity),
// which would surface as a misleading "singular system" diagnosis blaming
// circuit topology. Rejecting up front names the real problem.
const AC_MIN_FREQUENCY_HZ = 1e-9;
const AC_MAX_FREQUENCY_HZ = 1e15;

export interface SmallSignalAcOptions {
  /** Component id of the independent source carrying the unit AC drive. */
  inputId: string;
  /** Deterministic net ids to report (e.g. "n3"; "gnd" reads as 0). */
  outputNetIds: readonly string[];
  /** Analysis frequencies in hertz; every entry must be finite, > 0 (the
   *  operating point itself is the DC answer), and inside the documented
   *  [1e-9, 1e15] Hz sanity window. */
  frequenciesHz: readonly number[];
}

export interface SmallSignalAcNetOutput {
  netId: string;
  /** 20*log10(|V|) per point — gain in dB relative to the 1 V unit drive.
   *  Exactly-zero responses (e.g. probing "gnd") read -Infinity. */
  magnitudeDb: number[];
  /** Phase in degrees, atan2(im, re), in (-180, 180]. */
  phaseDeg: number[];
  /** Raw complex response per point (volts per volt of drive). */
  re: number[];
  im: number[];
}

export interface SmallSignalAcResult {
  frequenciesHz: number[];
  outputs: SmallSignalAcNetOutput[];
  /** Description of the injected reference drive. */
  reference: { inputId: string; kind: string; injection: string };
  /** Which dcOperatingPoint stage produced the linearization bias. */
  opMethod: DcOperatingPointResult["method"];
  /**
   * Frequencies whose bordered solve reported an ill-conditioned (but not
   * singular) factorization. Results at these points are returned — the
   * regularized system is solvable by construction, e.g. an ideal lossless
   * LC probed exactly at resonance answers through the disclosed 1e-12 node
   * shunt — but the backend's health flag is surfaced instead of silently
   * discarded so callers can see which points sit on a near-singularity.
   */
  illConditionedFrequenciesHz: number[];
}

/**
 * Generic small-signal default for committed digital output stages (kinds
 * without an acStamp hook). A committed `delay:<pinId>` icState slot is the
 * engine's canonical record that this pin carries a driven finite-strength
 * output stage, and its level says which package rail the stage ties to —
 * the identical machinery _stampDigitalOutput used to stamp the transient
 * stage. Its linearization is the stage's Thevenin output resistance to that
 * rail node (an AC ground through the package supply), implemented ONCE here
 * for the combinational-IC cohort — the only kinds that record their stages
 * as `delay:` slots (_updateDelayedOutputs drops the slots of tri-stated
 * pins, so a slot present at the OP is always a live stage). Kinds that
 * stamp output conductances from any other state encoding carry their own
 * acStamp hooks (see the coverage census). Everything else about these kinds
 * (inputs, protocol state) is high-Z by design.
 */
function stampCommittedDigitalOutputs(
  ctx: AcDeviceContext,
  comp: DeviceComponent,
  ac: AcSystem,
): void {
  const st = ctx.state.icState.get(comp.id);
  if (!st) return;
  // Resolved lazily so components with no committed output stages never pay
  // the package supply readout.
  let power: ReturnType<AcDeviceContext["icPowerInfoAtOp"]> | null = null;
  for (const key of Object.keys(st)) {
    if (!key.startsWith(DIGITAL_DELAY_PREFIX)) continue;
    const pinId = key.slice(DIGITAL_DELAY_PREFIX.length);
    const node = ctx.pinNode(comp.id, pinId);
    if (node < 0) continue;
    power ??= ctx.icPowerInfoAtOp(comp);
    // An unpowered package drives nothing — its outputs are high-Z in the
    // transient system too.
    if (!power.powered) return;
    const high = (st[key] ?? 0) >= 0.5;
    // A committed-HIGH open-collector pin is a RELEASED transistor: the
    // transient stamp writes nothing for it (external pull-ups own the
    // net), so the AC system must not invent a pull to VCC. Same
    // pin_layout gate as stampCombinationalIC.
    if (high) {
      const pinFunction = ctx.catalogPart(comp)
        ?.pin_layout.find((p) => p.id === pinId)?.function;
      if (pinFunction === "open_collector") continue;
    }
    const railPin = high ? power.vccPin : power.gndPin;
    const g = 1 / power.outputResistance;
    if (railPin && !ctx.isOpenPin(comp.id, railPin)) {
      stampAcAdmittance(ac, node, ctx.pinNode(comp.id, railPin), g, 0);
    } else {
      // Supply-less behavioural fallback: the transient stage is a Norton
      // to a fixed local target, whose constant differentiates away and
      // leaves only the self conductance (mirrors _stampDigitalOutput).
      ac.addAc(node, node, g, 0);
    }
  }
}

/**
 * Well-posedness guard for branch rows owned by a model without an acStamp
 * (possible for third-party registered kinds): an allocated row nothing
 * stamps is all-zero and would make the AC system singular. Pinning the
 * branch current to zero (dI = 0) is exactly the "no small-signal
 * contribution" default expressed on a branch instead of a node. branchRows
 * is pure over comp/catalog (DeviceCatalogContext contract), so re-querying
 * it here is safe; AcDeviceContext satisfies that parameter structurally.
 */
function pinUnstampedBranchRows(
  ctx: AcDeviceContext,
  comp: DeviceComponent,
  model: DeviceModel,
  ac: AcSystem,
): void {
  if (!model.branchRows) return;
  for (const rowKey of model.branchRows(comp, ctx)) {
    const row = ctx.vsrcRow(rowKey);
    if (row !== undefined) ac.addAc(row, row, 1, 0);
  }
}

/**
 * Run a linearized small-signal AC analysis over a loaded engine.
 *
 * The engine must have a circuit loaded; its DC operating point is computed
 * (and committed) here, so the engine's public readouts afterwards describe
 * the bias point the analysis linearized about. Throws with a descriptive
 * message on every unusable configuration instead of returning NaNs: a
 * missing/invalid input, an MCU-bearing circuit, a non-converged OP, or a
 * singular AC system.
 */
export function runSmallSignalAc(
  engine: SimEngine,
  options: SmallSignalAcOptions,
): SmallSignalAcResult {
  const components = engine.loadedComponents();
  if (components.length === 0) {
    throw new Error("runSmallSignalAc: engine has no loaded circuit");
  }

  const mcu = components.find(
    (comp) => isMcuBoardKind(comp.kind) || comp.kind === "microbit",
  );
  if (mcu) {
    throw new Error(
      `runSmallSignalAc: circuit contains MCU board "${mcu.id}" (${mcu.kind}) — `
      + "running firmware has no small-signal model, so linearized AC analysis "
      + "is not defined for this circuit. Use the large-signal ac-sweep instead.",
    );
  }

  const input = components.find((comp) => comp.id === options.inputId);
  if (!input) {
    throw new Error(
      `runSmallSignalAc: input component "${options.inputId}" is not in the loaded circuit`,
    );
  }
  if (!AC_INPUT_KINDS.has(input.kind)) {
    throw new Error(
      `runSmallSignalAc: input "${options.inputId}" has kind "${input.kind}", `
      + `which is not an independent source (expected one of: ${[...AC_INPUT_KINDS].join(", ")})`,
    );
  }
  if (input.kind === "signal_gen" && !signalGenEnabled(input.params)) {
    throw new Error(
      `runSmallSignalAc: input signal_gen "${options.inputId}" is disabled — `
      + "a hi-Z source cannot inject the AC drive",
    );
  }

  for (const f of options.frequenciesHz) {
    if (!Number.isFinite(f) || f <= 0) {
      throw new Error(
        `runSmallSignalAc: frequency ${String(f)} Hz is not a finite positive number `
        + "(the DC answer is the operating point itself)",
      );
    }
    if (f < AC_MIN_FREQUENCY_HZ || f > AC_MAX_FREQUENCY_HZ) {
      throw new Error(
        `runSmallSignalAc: frequency ${String(f)} Hz is outside the supported `
        + `[${String(AC_MIN_FREQUENCY_HZ)}, ${String(AC_MAX_FREQUENCY_HZ)}] Hz window — `
        + "reactive admittances over/underflow double precision there and would "
        + "fail with a misleading singular-system diagnosis",
      );
    }
  }

  const op = engine.dcOperatingPoint();
  if (!op.converged) {
    throw new Error(
      "runSmallSignalAc: DC operating point did not converge "
      + `(method "${op.method}", ${String(op.iterations)} Newton iterations) — `
      + "there is no bias point to linearize about",
    );
  }

  const { size, nodeCount } = engine.matrixDimensions();
  if (size === 0) {
    throw new Error("runSmallSignalAc: loaded circuit produced an empty MNA system");
  }

  const outputRows = options.outputNetIds.map((netId) => {
    const row = engine.netRow(netId);
    if (row === undefined) {
      throw new Error(`runSmallSignalAc: unknown output net "${netId}"`);
    }
    return row;
  });

  const ctx = engine.acDeviceContext({ inputId: options.inputId });

  // A designated bench_psu whose committed OP regime is constant-current
  // pins its branch current, so the unit voltage drive cannot inject — the
  // analysis would silently report a dead circuit.
  if (input.kind === "bench_psu" && Number(input.params.iLimit ?? 0) > 0) {
    const st = ctx.state.icState.get(input.id);
    if (st && (st.reg ?? 0) === 2) {
      throw new Error(
        `runSmallSignalAc: input bench_psu "${options.inputId}" is in its committed `
        + "constant-current regime at the operating point; its branch current is "
        + "pinned and cannot carry an AC voltage drive",
      );
    }
  }

  const ac = new AcSystem(size);

  const frequenciesHz = [...options.frequenciesHz];
  const outputs: SmallSignalAcNetOutput[] = options.outputNetIds.map((netId) => ({
    netId,
    magnitudeDb: [],
    phaseDeg: [],
    re: [],
    im: [],
  }));
  const illConditionedFrequenciesHz: number[] = [];

  for (const f of frequenciesHz) {
    const omega = 2 * Math.PI * f;
    ac.clear();
    // The transient system's universal node shunt, real-valued here: nodes
    // reached only by high-Z defaults stay solvable (and read ~0) exactly as
    // they do in the large-signal system.
    for (let row = 0; row < nodeCount; row++) {
      ac.addAc(row, row, NODE_RSHUNT_G, 0);
    }
    for (const comp of components) {
      const model = getDeviceModel(comp.kind);
      if (!model) continue; // unregistered kinds are MCU boards, rejected above
      if (model.acStamp) {
        model.acStamp(ctx, comp, ac, omega);
        continue;
      }
      // Documented high-Z default, minus the two generic obligations: a
      // committed digital output stage still loads the circuit, and an
      // allocated branch row must stay well-posed.
      stampCommittedDigitalOutputs(ctx, comp, ac);
      pinUnstampedBranchRows(ctx, comp, model, ac);
    }

    const solved = ac.solveAc();
    if (solved.info.singular || solved.info.nonFinite) {
      throw new Error(
        `runSmallSignalAc: AC system is ${solved.info.singular ? "singular" : "non-finite"} `
        + `at ${String(f)} Hz — check for components left floating at the operating point`,
      );
    }
    // Solvable but near-singular points (e.g. an ideal lossless LC probed at
    // exact resonance, held up only by the disclosed node shunt) are
    // reported, not hidden — see SmallSignalAcResult.
    if (solved.info.illConditioned) illConditionedFrequenciesHz.push(f);

    for (let j = 0; j < outputRows.length; j++) {
      const row = outputRows[j]!;
      const re = row >= 0 ? (solved.re[row] ?? 0) : 0;
      const im = row >= 0 ? (solved.im[row] ?? 0) : 0;
      const out = outputs[j]!;
      out.re.push(re);
      out.im.push(im);
      out.magnitudeDb.push(20 * Math.log10(Math.hypot(re, im)));
      out.phaseDeg.push((Math.atan2(im, re) * 180) / Math.PI);
    }
  }

  const rSource = Number(input.params.rSource ?? 50);
  // Wave A7: a designated current_source injects current, not a branch
  // voltage, so the human-readable reference must not claim a 1 V drive.
  const injection = input.kind === "current_source"
    ? "unit 1 A AC Norton injection (out of pos, into neg); all other independent sources AC-zeroed"
    : input.kind === "signal_gen" && rSource > 0
      ? `unit 1 V AC Thevenin drive behind rSource = ${String(rSource)} ohm `
        + "(injected as its Norton equivalent); all other independent sources AC-zeroed"
      : "unit 1 V AC drive on the source branch; all other independent sources AC-zeroed";

  return {
    frequenciesHz,
    outputs,
    reference: { inputId: options.inputId, kind: input.kind, injection },
    opMethod: op.method,
    illConditionedFrequenciesHz,
  };
}
