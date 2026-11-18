/**
 * G-0 item 2: one small circuit and one dynamic stimulus for every part kind
 * the core does not run yet (plan 1.0.0 section 1.1: 53 registered kinds plus
 * the four engine-owned boards), registered with the both-engine harness so a
 * kind is picked up the day it lands.
 *
 * What a skeleton is: a circuit in the style of test/fixtures/core (an ideal
 * source, a ground, the kind under test, a load or two) taken from the kind's
 * own classic test (`cite` names the file and the case it mirrors), plus
 * `phases` (stimuli.ts): scripted parameter changes of a live source, switch,
 * or the part itself. runCorpusEntry replays them on classic first; the core
 * then declines the circuit for the missing kind, and the entry reports
 * `core-stimuli-pending` ("declined: unsupported-kind:<kind>"). When the kind
 * is in CORE_IMPLEMENTED_KINDS the same entry must come out "pass": whoever
 * lands the kind gets a failing both-engine comparison to fix, with the
 * circuit and stimulus already written.
 *
 * Pins come from the part catalog fixture (test/fixtures/catalog/parts.json,
 * first part of the kind, whose default_params also fill the part's params)
 * or, for the ten kinds the catalog does not carry, from EXTRA_PINS below
 * (each read from the classic test that builds the kind).
 */
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import rawCatalog from "../../helpers/catalog.js";
import { kindsOf } from "./kinds-of.js";
import { detectOpenInput } from "./open-input.js";
import type { StimulusPhase } from "./stimuli.js";
import type { CorpusEntry } from "./types.js";

/** Pin ids of the kinds the catalog fixture has no part for, copied from the classic tests that build them. */
export const EXTRA_PINS: Readonly<Record<string, readonly string[]>> = {
  "74ls153": ["/1g", "b", "1c3", "1c2", "1c1", "1c0", "1y", "gnd", "2y", "2c0", "2c1", "2c2", "2c3", "a", "/2g", "vcc"], // ls153-mux.test.ts, DIP-16 order
  coupled_inductor: ["a1", "b1", "a2", "b2"], // magnetics-crystal.test.ts
  crystal: ["a", "b"], // magnetics-crystal.test.ts
  njfet: ["d", "g", "s"], // extended-semis.test.ts
  pjfet: ["d", "g", "s"], // extended-integration.test.ts
  scr: ["a", "k", "g"], // extended-semis.test.ts
  triac: ["mt1", "mt2", "g"], // extended-semis.test.ts
  opto_npn: ["led_a", "led_k", "c", "e"], // extended-semis.test.ts
  analog_switch: ["a", "b", "ctrl"], // extended-semis.test.ts
  current_source: ["pos", "neg"], // current-source-polarity.test.ts
  ground: ["gnd"],
  voltage_source: ["pos", "neg"],
  pulse_source: ["pos", "neg"], // hc595-tied-clocks.test.ts
};

/** One part: [id, kind, params overriding the catalog defaults]. */
type Part = readonly [id: string, kind: string, params?: Record<string, number | string>];

export interface SkeletonSpec {
  /** The kind this skeleton stands for. The circuit may hold other kinds the core has not got (a driver chip needs a transistor), and then it waits for those too. */
  readonly kind: string;
  /** The package of plan-1.0.0.md that brings the kind. */
  readonly pkg: string;
  /** The classic test file and case the circuit is taken from. */
  readonly cite: string;
  readonly parts: readonly Part[];
  /** Groups of "id.pin" that are one net; the token "gnd" is the ground. */
  readonly nets: readonly (readonly string[])[];
  readonly phases: readonly StimulusPhase[];
}

const V = (id: string, voltage: number): Part => [id, "voltage_source", { voltage }];
const R = (id: string, resistance: number): Part => [id, "resistor", { resistance }];
const PULSE = (id: string, params: Record<string, number>): Part => [id, "pulse_source", { v1: 0, v2: 5, tr: 1e-6, tf: 1e-6, ...params }];
const settle = (durationS: number): StimulusPhase => ({ durationS });
const set = (id: string, params: Record<string, number | string>, durationS: number): StimulusPhase => ({ set: [{ id, params }], durationS });

/** Supply rails: a source `vcc` of `volts` with its minus on ground. */
const RAIL = (volts: number): { parts: Part[]; nets: string[][] } => ({ parts: [V("vcc", volts)], nets: [["vcc.neg", "gnd"]] });

/** A digital chip on a 5 V rail, ground wired; `pins` names the supply pins. */
function chip(kind: string, id: string, vccPin: string, gndPin: string, extraParts: Part[], nets: string[][], phases: StimulusPhase[], pkg: string, cite: string): SkeletonSpec {
  const rail = RAIL(5);
  return {
    kind,
    pkg,
    cite,
    parts: [...rail.parts, [id, kind], ...extraParts],
    nets: [...rail.nets, ["vcc.pos", `${id}.${vccPin}`], ["gnd", `${id}.${gndPin}`], ...nets],
    phases,
  };
}

/** An input driven by a 0/5 V source (toggled by `phases`), one source per name. */
const DRIVE = (id: string, v: number): Part => V(id, v);

const LS = "test/sim/engine/";

export const SKELETONS: readonly SkeletonSpec[] = [
  // ---- P0-SG ----------------------------------------------------------------------------------
  {
    kind: "potentiometer", pkg: "P0-SG", cite: `${LS}potentiometer.test.ts: divider, "turn the knob, dim the LED"`,
    parts: [["psu", "bench_psu", { voltage: 5, iLimit: 1 }], ["pot", "potentiometer", { rTotal: 10000, position: 0.5 }], R("r1", 220), ["d1", "led", { vf: 1.8 }]],
    nets: [["psu.pos", "pot.cw"], ["pot.ccw", "psu.neg", "gnd"], ["pot.wiper", "r1.a"], ["r1.b", "d1.a"], ["d1.k", "gnd"]],
    phases: [settle(0.01), set("pot", { position: 1 }, 0.01), set("pot", { position: 0.1 }, 0.01), set("pot", { position: 0 }, 0.01)],
  },
  {
    kind: "trimmer", pkg: "P0-SG", cite: `${LS}potentiometer.test.ts: the same divider with the trimmer (POT_KINDS)`,
    parts: [["psu", "bench_psu", { voltage: 5, iLimit: 1 }], ["pot", "trimmer", { rTotal: 10000, position: 0.5 }], R("r1", 220), ["d1", "led", { vf: 1.8 }]],
    nets: [["psu.pos", "pot.cw"], ["pot.ccw", "psu.neg", "gnd"], ["pot.wiper", "r1.a"], ["r1.b", "d1.a"], ["d1.k", "gnd"]],
    phases: [settle(0.01), set("pot", { position: 0.9 }, 0.01), set("pot", { position: 0.2 }, 0.01)],
  },
  {
    kind: "resistor_array", pkg: "P0-SG", cite: `${LS}wave3-passives.test.ts: resistor_array — four independent channels`,
    parts: [V("vcc", 5), ["ra", "resistor_array", { resistance: 10000 }], R("rl", 4700)],
    nets: [["vcc.pos", "ra.a1", "ra.a2", "ra.a3"], ["ra.b1", "gnd", "vcc.neg"], ["ra.b2", "rl.a"], ["rl.b", "gnd"], ["ra.b3", "ra.a4"], ["ra.b4", "gnd"]],
    phases: [settle(0.01), set("vcc", { voltage: 3.3 }, 0.01), set("vcc", { voltage: 5 }, 0.01)],
  },
  {
    kind: "ldr", pkg: "P0-SG", cite: `${LS}environment.test.ts: LDR voltage divider (params.lux, the per-part stimulus)`,
    parts: [V("vcc", 5), R("rf", 10000), ["ldr1", "ldr", { rDark: 1000000, rLight: 500, lux: 10 }]],
    nets: [["vcc.pos", "rf.a"], ["rf.b", "ldr1.a"], ["ldr1.b", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("ldr1", { lux: 500 }, 0.01), set("ldr1", { lux: 0 }, 0.01), set("ldr1", { lux: 50 }, 0.01)],
  },
  {
    kind: "ferrite_bead", pkg: "P0-SG", cite: `${LS}ferrite-bead-overload.test.ts: a 5 V / 220 ohm circuit through the bead (the DC series resistance model)`,
    parts: [V("vcc", 5), ["fb1", "ferrite_bead", { rDc: 0.0035 }], R("r1", 220)],
    nets: [["vcc.pos", "fb1.a"], ["fb1.b", "r1.a"], ["r1.b", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("vcc", { voltage: 2 }, 0.01), set("vcc", { voltage: 5 }, 0.01)],
  },
  {
    kind: "push_dpdt", pkg: "P0-SG", cite: `${LS}push-dpdt.test.ts: dpdtRig, both commons from +5 V onto four 1 k resistors`,
    parts: [V("vcc", 5), ["sw", "push_dpdt", { position: 0 }], R("rL1", 1000), R("rR1", 1000), R("rL2", 1000), R("rR2", 1000)],
    nets: [["vcc.pos", "sw.c1", "sw.c2"], ["sw.l1", "rL1.a"], ["sw.r1", "rR1.a"], ["sw.l2", "rL2.a"], ["sw.r2", "rR2.a"], ["rL1.b", "rR1.b", "rL2.b", "rR2.b", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("sw", { position: 1 }, 0.01), set("sw", { position: 0 }, 0.01)],
  },
  {
    kind: "current_source", pkg: "P0-SG", cite: `${LS}current-source-polarity.test.ts: isrcIntoResistor, 1 mA into 220 ohm to ground`,
    parts: [["i1", "current_source", { current: 0.001, rParallel: 0 }], R("r1", 220)],
    nets: [["i1.pos", "r1.a"], ["r1.b", "i1.neg", "gnd"]],
    phases: [settle(0.01), set("i1", { current: 0.01 }, 0.01), set("i1", { current: 0 }, 0.01)],
  },
  {
    kind: "signal_gen", pkg: "P0-SG", cite: `${LS}signal-gen.test.ts: a sine into an RC load (330 Hz here: the harness steps at 1 ms, so a multiple of 1 kHz would alias to one phase) (the waveform and rSource model)`,
    parts: [["sg", "signal_gen", { waveform: "sine", amplitude: 2.5, offset: 2.5, frequency: 330, rSource: 50, enabled: 1 }], R("r1", 1000), ["c1", "capacitor", { capacitance: 1e-7 }]],
    nets: [["sg.pos", "r1.a"], ["r1.b", "c1.a"], ["c1.b", "sg.neg", "gnd"]],
    phases: [settle(0.01), set("sg", { waveform: "square", frequency: 170 }, 0.01), set("sg", { enabled: 0 }, 0.01), set("sg", { enabled: 1, waveform: "sine" }, 0.01)],
  },

  // ---- P0-IC ----------------------------------------------------------------------------------
  chip(
    "74hc595", "u1", "vcc", "gnd",
    [PULSE("ck", { per: 2e-3, pw: 1e-3, td: 5e-4 }), PULSE("lat", { per: 2e-3, pw: 5e-4, td: 1.5e-3 }), DRIVE("ser", 5), R("rq", 10000)],
    [["u1./oe", "gnd"], ["u1./srclr", "vcc.pos"], ["ck.pos", "u1.srclk"], ["ck.neg", "gnd"], ["lat.pos", "u1.rclk"], ["lat.neg", "gnd"], ["ser.pos", "u1.ser"], ["ser.neg", "gnd"], ["u1.qa", "rq.a"], ["rq.b", "gnd"]],
    [settle(0.01), set("ser", { voltage: 0 }, 0.01), set("ser", { voltage: 5 }, 0.01)],
    "P0-IC", `${LS}hc595-tied-clocks.test.ts: board(), SER high, separate SRCLK and RCLK pulse sources`,
  ),
  chip(
    "74hc165", "u1", "vcc", "gnd",
    [DRIVE("d0", 5), DRIVE("d1", 0), PULSE("ck", { per: 2e-3, pw: 1e-3 }), PULSE("pl", { per: 20e-3, pw: 18e-3, td: 0 }), R("rq", 10000)],
    [["u1.d0", "d0.pos"], ["u1.d1", "d1.pos"], ["d0.neg", "gnd"], ["d1.neg", "gnd"], ["u1.cp", "ck.pos"], ["ck.neg", "gnd"], ["u1./pl", "pl.pos"], ["pl.neg", "gnd"], ["u1.cp_inh", "gnd"], ["u1.ds", "gnd"], ["u1.q7", "rq.a"], ["rq.b", "gnd"]],
    [settle(0.01), set("d1", { voltage: 5 }, 0.01), set("d0", { voltage: 0 }, 0.01)],
    "P0-IC", `${LS}mcu-165-readback.test.ts: parallel load then shift, Q7 read through a load`,
  ),
  chip(
    "cd4017", "u1", "vcc", "gnd",
    [PULSE("ck", { per: 1e-3, pw: 5e-4 }), DRIVE("rst", 0), R("rq", 10000)],
    [["u1.clk", "ck.pos"], ["ck.neg", "gnd"], ["u1.reset", "rst.pos"], ["rst.neg", "gnd"], ["u1.clkinh", "gnd"], ["u1.q3", "rq.a"], ["rq.b", "gnd"]],
    [settle(0.01), set("rst", { voltage: 5 }, 0.005), set("rst", { voltage: 0 }, 0.01)],
    "P0-IC", `${LS}wave2-sequential.test.ts: CD4017 decade counter, clocked with RESET toggled`,
  ),
  chip(
    "cd4060", "u1", "vcc", "gnd",
    [DRIVE("rst", 5), PULSE("ck", { per: 1e-4, pw: 5e-5 }), R("rq", 10000)],
    [["u1.clk_in", "ck.pos"], ["ck.neg", "gnd"], ["u1.reset", "rst.pos"], ["rst.neg", "gnd"], ["u1.q4", "rq.a"], ["rq.b", "gnd"]],
    [settle(0.005), set("rst", { voltage: 0 }, 0.02), set("rst", { voltage: 5 }, 0.005)],
    "P0-IC", `${LS}wave2-sequential.test.ts: CD4060 14-stage counter, RESET released then re-asserted`,
  ),
  chip(
    "cd4511", "u1", "vcc", "gnd",
    [DRIVE("a", 5), DRIVE("b", 0), DRIVE("c", 0), DRIVE("d", 0), R("rseg", 1000)],
    [["u1.a", "a.pos"], ["u1.b", "b.pos"], ["u1.c", "c.pos"], ["u1.d", "d.pos"], ["a.neg", "gnd"], ["b.neg", "gnd"], ["c.neg", "gnd"], ["d.neg", "gnd"], ["u1.lt_n", "vcc.pos"], ["u1.bl_n", "vcc.pos"], ["u1.le", "gnd"], ["u1.a_out", "rseg.a"], ["rseg.b", "gnd"]],
    [settle(0.01), set("b", { voltage: 5 }, 0.01), set("a", { voltage: 0 }, 0.01)],
    "P0-IC", `${LS}wave2-sequential.test.ts: CD4511 BCD to 7-segment decode, input code stepped`,
  ),
  chip(
    "74ls153", "u1", "vcc", "gnd",
    [DRIVE("sa", 0), DRIVE("sb", 0), R("r1y", 10000), R("r2y", 10000)],
    [["u1.a", "sa.pos"], ["u1.b", "sb.pos"], ["sa.neg", "gnd"], ["sb.neg", "gnd"], ["u1./1g", "gnd"], ["u1./2g", "gnd"], ["u1.1c0", "vcc.pos"], ["u1.1c1", "gnd"], ["u1.1c2", "gnd"], ["u1.1c3", "gnd"], ["u1.2c0", "gnd"], ["u1.2c1", "vcc.pos"], ["u1.2c2", "gnd"], ["u1.2c3", "gnd"], ["u1.1y", "r1y.a"], ["r1y.b", "gnd"], ["u1.2y", "r2y.a"], ["r2y.b", "gnd"]],
    [settle(0.01), set("sa", { voltage: 5 }, 0.01), set("sb", { voltage: 5 }, 0.01)],
    "P0-IC", `${LS}ls153-mux.test.ts: circuit(), each section routes C[B*2+A], outputs loaded by 10 k`,
  ),

  // ---- P0-MAG ---------------------------------------------------------------------------------
  {
    kind: "inductor", pkg: "P0-MAG", cite: `${LS}magnetics-crystal.test.ts and test/sim/engine/passive-parasitics.test.ts: an RL step`,
    parts: [PULSE("vs", { v2: 5, per: 4e-3, pw: 2e-3 }), R("r1", 100), ["l1", "inductor", { inductance: 0.01, dcr: 5 }], ["sx", "switch", { closed: 0 }], R("rx", 88.679)],
    nets: [["vs.pos", "r1.a", "sx.a"], ["r1.b", "l1.a", "rx.b"], ["sx.b", "rx.a"], ["l1.b", "vs.neg", "gnd"]],
    phases: [settle(0.01), set("sx", { closed: 1 }, 0.01), set("sx", { closed: 0 }, 0.01)], // rx across r1 takes it from 100 to 47 ohm
  },
  {
    kind: "coupled_inductor", pkg: "P0-MAG", cite: `${LS}magnetics-crystal.test.ts: coupled inductor, transformer turns ratio (a 50 Hz square here: the harness steps at 1 ms)`,
    parts: [["sg", "pulse_source", { v1: -1, v2: 1, per: 0.02, pw: 0.01, tr: 1e-4, tf: 1e-4 }], R("rs", 100), ["tx", "coupled_inductor", { l1: 0.01, l2: 0.04, k: 0.9 }], R("rl", 1000), ["sx", "switch", { closed: 1 }], R("rx", 111.11)],
    nets: [["sg.pos", "rs.a"], ["rs.b", "tx.a1"], ["tx.b1", "sg.neg", "gnd"], ["tx.a2", "rl.a", "sx.a"], ["rl.b", "tx.b2", "rx.b", "gnd"], ["sx.b", "rx.a"]],
    phases: [settle(0.01), set("sx", { closed: 0 }, 0.01), set("sx", { closed: 1 }, 0.01)], // rx across rl: 100 ohm closed, 1 k open
  },
  {
    kind: "crystal", pkg: "P0-MAG", cite: `${LS}magnetics-crystal.test.ts: crystal xt (fSeries, q, rs, c0) driven through a series resistor (a slow square: the harness steps at 1 ms, far above the 1 MHz series resonance, so this exercises the c0 and motional branch transients, not the resonance)`,
    parts: [["sg", "pulse_source", { v1: 0, v2: 1, per: 0.02, pw: 0.01, tr: 1e-5, tf: 1e-5 }], R("rs", 100), ["xt", "crystal", { fSeries: 1e6, q: 50000, rs: 50, c0: 5 }], R("rl", 1000), ["sx", "switch", { closed: 0 }], R("rx", 111.11)],
    nets: [["sg.pos", "rs.a"], ["rs.b", "xt.a"], ["xt.b", "rl.a", "sx.a"], ["rl.b", "sg.neg", "rx.b", "gnd"], ["sx.b", "rx.a"]],
    phases: [settle(1e-3), set("sx", { closed: 1 }, 1e-3), set("sx", { closed: 0 }, 1e-3)], // rx across rl: 1 k open, 100 ohm closed
  },
  {
    kind: "thermistor", pkg: "P0-MAG", cite: `${LS}environment.test.ts: thermistor voltage divider (params.tempC, the per-part stimulus)`,
    parts: [V("vcc", 5), R("rf", 10000), ["th1", "thermistor", { rNominal: 10000, beta: 3950, tempC: 25, dissipationFactor: 0.001, thermalTau: 0.003 }]],
    nets: [["vcc.pos", "rf.a"], ["rf.b", "th1.a"], ["th1.b", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("th1", { tempC: 60 }, 0.01), set("th1", { tempC: -10 }, 0.01), set("th1", { tempC: 25 }, 0.01)],
  },

  // ---- W3: diodes -----------------------------------------------------------------------------
  {
    kind: "diode", pkg: "W3-D1", cite: `${LS}extended-semis.test.ts and physics-reference-benchmarks.test.ts: half-wave rectifier into R and C, forward and reverse`,
    parts: [["sg", "pulse_source", { v1: -5, v2: 5, per: 4e-3, pw: 2e-3, tr: 1e-4, tf: 1e-4 }], ["d1", "diode", { vf: 0.7 }], R("r1", 1000), ["c1", "capacitor", { capacitance: 1e-6 }], ["sx", "switch", { closed: 0 }], R("rx", 282.05)],
    nets: [["sg.pos", "d1.a"], ["d1.k", "r1.a", "c1.a", "sx.a"], ["r1.b", "c1.b", "sg.neg", "rx.b", "gnd"], ["sx.b", "rx.a"]],
    phases: [settle(0.01), set("sx", { closed: 1 }, 0.01), set("sx", { closed: 0 }, 0.01)], // rx across r1: 1 k open, 220 ohm closed
  },
  {
    kind: "schottky_diode", pkg: "W3-D1", cite: `${LS}extended-semis.test.ts: the diode circuit with the Schottky's lower forward drop`,
    parts: [V("vcc", 5), R("r1", 1000), ["d1", "schottky_diode", { vf: 0.3 }]],
    nets: [["vcc.pos", "r1.a"], ["r1.b", "d1.a"], ["d1.k", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("vcc", { voltage: -3 }, 0.01), set("vcc", { voltage: 1 }, 0.01)],
  },
  {
    kind: "zener_diode", pkg: "W3-D2", cite: `${LS}wave3-protection.test.ts: zener_diode regulation at vz (5 V... 12 V through 1 k)`,
    parts: [V("vcc", 12), R("r1", 1000), ["dz", "zener_diode", { vf: 0.7, vz: 5.1 }]],
    nets: [["vcc.pos", "r1.a"], ["r1.b", "dz.k"], ["dz.a", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("vcc", { voltage: 3 }, 0.01), set("vcc", { voltage: 12 }, 0.01)],
  },
  {
    kind: "tvs_diode", pkg: "W3-D2", cite: `${LS}wave3-protection.test.ts: tvs_diode unidirectional clamping at vbr (12 V source clamps at 6.8 V)`,
    parts: [V("vcc", 12), R("r1", 220), ["dt", "tvs_diode", { vbr: 6.8, bidirectional: 0 }]],
    nets: [["vcc.pos", "r1.a"], ["r1.b", "dt.k"], ["dt.a", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("vcc", { voltage: 5 }, 0.01), set("vcc", { voltage: 12 }, 0.01)],
  },

  // ---- W3: transistors ------------------------------------------------------------------------
  {
    kind: "bjt_npn", pkg: "W3-T", cite: `${LS}semiconductor-parasitics.test.ts and physics-reference-benchmarks.test.ts: NPN DC point, common emitter with a base resistor`,
    parts: [V("vcc", 5), V("vin", 2), R("rb", 100000), R("rc", 1000), ["q1", "bjt_npn", { Is: 1e-14, betaF: 100, earlyVoltage: 75 }]],
    nets: [["vcc.pos", "rc.a"], ["rc.b", "q1.c"], ["vin.pos", "rb.a"], ["rb.b", "q1.b"], ["q1.e", "vcc.neg", "vin.neg", "gnd"]],
    phases: [settle(0.01), set("vin", { voltage: 0 }, 0.01), set("vin", { voltage: 5 }, 0.01)],
  },
  {
    kind: "bjt_pnp", pkg: "W3-T", cite: `${LS}semiconductor-parasitics.test.ts: PNP symmetry of the Early slope, emitter on the rail`,
    parts: [V("vcc", 5), V("vin", 3), R("rb", 100000), R("rc", 1000), ["q1", "bjt_pnp", { Is: 1e-14, betaF: 100, earlyVoltage: 100 }]],
    nets: [["vcc.pos", "q1.e"], ["q1.c", "rc.a"], ["rc.b", "vcc.neg", "gnd"], ["vin.pos", "rb.a"], ["rb.b", "q1.b"], ["vin.neg", "gnd"]],
    phases: [settle(0.01), set("vin", { voltage: 5 }, 0.01), set("vin", { voltage: 0 }, 0.01)],
  },
  {
    kind: "nmos", pkg: "W3-T", cite: `${LS}semiconductor-parasitics.test.ts and physics-reference-benchmarks.test.ts: NMOS DC points, gate stepped`,
    parts: [V("vdd", 5), V("vg", 3), R("rd", 1000), ["m1", "nmos", { vto: 0.7, k: 0.02, lambda: 0.02 }]],
    nets: [["vdd.pos", "rd.a"], ["rd.b", "m1.d"], ["vg.pos", "m1.g"], ["m1.s", "vdd.neg", "vg.neg", "gnd"]],
    phases: [settle(0.01), set("vg", { voltage: 0 }, 0.01), set("vg", { voltage: 5 }, 0.01)],
  },
  {
    kind: "pmos", pkg: "W3-T", cite: `${LS}extended-integration.test.ts and semiconductor-parasitics.test.ts: PMOS high-side switch, gate pulled to the rail`,
    parts: [V("vdd", 5), V("vg", 5), R("rl", 1000), ["m1", "pmos", { vto: 0.7, k: 0.02, lambda: 0.02 }]],
    nets: [["vdd.pos", "m1.s"], ["m1.d", "rl.a"], ["rl.b", "vdd.neg", "gnd"], ["vg.pos", "m1.g"], ["vg.neg", "gnd"]],
    phases: [settle(0.01), set("vg", { voltage: 0 }, 0.01), set("vg", { voltage: 5 }, 0.01)],
  },

  // ---- W3: amplifiers and comparators ---------------------------------------------------------
  {
    kind: "lm358", pkg: "W3-O", cite: `${LS}wave4-opamps.test.ts: LM358 unit 1 non-inverting amplifier, gain 2`,
    parts: [V("v1", 5), V("vin", 0.5), R("rf", 10000), R("rg", 10000), ["oa1", "lm358"]],
    nets: [["v1.pos", "oa1.8"], ["v1.neg", "oa1.4", "gnd"], ["vin.pos", "oa1.3"], ["vin.neg", "gnd"], ["oa1.1", "rf.a"], ["rf.b", "oa1.2", "rg.a"], ["rg.b", "gnd"]],
    phases: [settle(0.01), set("vin", { voltage: 1.5 }, 0.01), set("vin", { voltage: 0.2 }, 0.01)],
  },
  {
    kind: "mcp6002", pkg: "W3-O", cite: `${LS}wave4-opamps.test.ts: MCP6002 unit 1 voltage follower on a single supply`,
    parts: [V("v1", 3.3), V("vin", 1), ["oa1", "mcp6002"]],
    nets: [["v1.pos", "oa1.8"], ["v1.neg", "oa1.4", "gnd"], ["vin.pos", "oa1.3"], ["vin.neg", "gnd"], ["oa1.1", "oa1.2"]],
    phases: [settle(0.01), set("vin", { voltage: 2.5 }, 0.01), set("vin", { voltage: 0.1 }, 0.01)],
  },
  {
    kind: "lm386", pkg: "W3-O", cite: `${LS}wave4-opamps.test.ts: LM386 power amplifier, gain 20, input at its pin 3 with the speaker load`,
    parts: [V("v1", 9), V("vin", 0.05), R("rl", 8), ["a1", "lm386", { gain: 20 }]],
    nets: [["v1.pos", "a1.6"], ["v1.neg", "a1.4", "gnd"], ["vin.pos", "a1.3"], ["vin.neg", "gnd"], ["a1.2", "gnd"], ["a1.5", "rl.a"], ["rl.b", "gnd"]],
    phases: [settle(0.01), set("vin", { voltage: 0.2 }, 0.01), set("vin", { voltage: 0 }, 0.01)],
  },
  {
    kind: "lm393", pkg: "W3-C", cite: `${LS}wave4-comparator-shunt.test.ts: LM393 comparator 1, open-collector output with a pull-up`,
    parts: [V("v1", 5), V("vin", 3), V("vref", 2.5), R("rp", 10000), ["u1", "lm393"]],
    nets: [["v1.pos", "u1.8", "rp.a"], ["v1.neg", "u1.4", "gnd"], ["vin.pos", "u1.3"], ["vin.neg", "gnd"], ["vref.pos", "u1.2"], ["vref.neg", "gnd"], ["u1.1", "rp.b"]],
    phases: [settle(0.01), set("vin", { voltage: 2 }, 0.01), set("vin", { voltage: 4 }, 0.01)],
  },
  {
    kind: "tl431", pkg: "W3-C", cite: `${LS}wave4-comparator-shunt.test.ts: TL431 shunt regulator, reference tied to the cathode, 5 V through 1 k`,
    parts: [V("vcc", 5), R("r1", 1000), ["u1", "tl431"]],
    nets: [["vcc.pos", "r1.a"], ["r1.b", "u1.cathode", "u1.ref"], ["u1.anode", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("vcc", { voltage: 3 }, 0.01), set("vcc", { voltage: 9 }, 0.01)],
  },

  // ---- W3: the six the plan never names -------------------------------------------------------
  {
    kind: "njfet", pkg: "W3-F", cite: `${LS}extended-semis.test.ts: jfetBiasCircuit, njfet njfet Shichman-Hodges channel physics (10 V, gate stepped)`,
    parts: [V("vdd", 10), V("vg", 0), ["j1", "njfet", { vto: -2, idss: 0.01, lambda: 0.01 }]],
    nets: [["vdd.pos", "j1.d"], ["j1.s", "vdd.neg", "gnd"], ["vg.pos", "j1.g"], ["vg.neg", "gnd"]],
    phases: [settle(0.01), set("vg", { voltage: -1 }, 0.01), set("vg", { voltage: -3 }, 0.01)],
  },
  {
    kind: "pjfet", pkg: "W3-F", cite: `${LS}extended-integration.test.ts: q_pj pjfet with its drain, gate and source resistors (vto -2, idss 10 mA)`,
    parts: [V("vdd", 10), ["q", "pjfet", { vto: -2, idss: 0.01 }], R("rd", 1000), R("rg", 100000), R("rs", 330)],
    nets: [["vdd.pos", "rs.a"], ["rs.b", "q.s"], ["q.d", "rd.a"], ["rd.b", "gnd", "vdd.neg"], ["q.g", "rg.a"], ["rg.b", "gnd"]],
    phases: [settle(0.01), set("vdd", { voltage: 6 }, 0.01), set("vdd", { voltage: 12 }, 0.01)],
  },
  {
    kind: "opto_npn", pkg: "W3-F", cite: `${LS}extended-semis.test.ts: optoCircuit, LED side 383.6 ohm, output side 5 V through 200 ohm`,
    parts: [V("vin", 5), R("rled", 383.6), ["oc1", "opto_npn", { ctr: 1, betaF: 100, vf: 1.2, iRated: 0.02 }], V("vcc", 5), R("rc", 200)],
    nets: [["vin.pos", "rled.a"], ["rled.b", "oc1.led_a"], ["oc1.led_k", "vin.neg", "gnd"], ["vcc.pos", "rc.a"], ["rc.b", "oc1.c"], ["oc1.e", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("vin", { voltage: 0 }, 0.01), set("vin", { voltage: 5 }, 0.01)],
  },
  {
    kind: "scr", pkg: "W3-F", cite: `${LS}extended-semis.test.ts: scrLifecycleCircuit, anode supply and a gate pulse through 220 ohm`,
    parts: [V("vsup", 12), R("rl", 100), ["scr1", "scr", { vgt: 0.7, igt: 0.005, ih: 0.005, vtm: 1.2, ron: 0.1 }], V("vgate", 0), R("rg", 220)],
    nets: [["vsup.pos", "rl.a"], ["rl.b", "scr1.a"], ["scr1.k", "vsup.neg", "gnd"], ["vgate.pos", "rg.a"], ["rg.b", "scr1.g"], ["vgate.neg", "gnd"]],
    phases: [settle(0.01), set("vgate", { voltage: 5 }, 0.005), set("vgate", { voltage: 0 }, 0.01), set("vsup", { voltage: 0 }, 0.01), set("vsup", { voltage: 12 }, 0.01)],
  },
  {
    kind: "triac", pkg: "W3-F", cite: `${LS}extended-semis.test.ts: triacCircuit, a 50 Hz sine across the load with the gate driven through 220 ohm`,
    parts: [["sg", "pulse_source", { v1: -10, v2: 10, per: 0.02, pw: 0.01, tr: 1e-4, tf: 1e-4 }], R("rl", 100), ["tr1", "triac", { vgt: 0.7, igt: 0.005, ih: 0.005, vtm: 1.2, ron: 0.1 }], V("gate", 0), R("rg", 220)],
    nets: [["sg.pos", "rl.a"], ["rl.b", "tr1.mt2"], ["tr1.mt1", "sg.neg", "gnd"], ["gate.pos", "rg.a"], ["rg.b", "tr1.g"], ["gate.neg", "gnd"]],
    phases: [settle(0.02), set("gate", { voltage: 5 }, 0.04), set("gate", { voltage: 0 }, 0.04)],
  },
  {
    kind: "analog_switch", pkg: "W3-F", cite: `${LS}extended-semis.test.ts: analogSwitchCircuit, a 5 V source through 1 k, the switch, and 1 k to ground, control stepped`,
    parts: [V("vsrc", 5), R("r1", 1000), ["sw1", "analog_switch", { ron: 50, roff: 1e9 }], R("r2", 1000), V("ctrl", 5)],
    nets: [["vsrc.pos", "r1.a"], ["r1.b", "sw1.a"], ["sw1.b", "r2.a"], ["r2.b", "vsrc.neg", "gnd"], ["ctrl.pos", "sw1.ctrl"], ["ctrl.neg", "gnd"]],
    phases: [settle(0.01), set("ctrl", { voltage: 0 }, 0.01), set("ctrl", { voltage: 5 }, 0.01)],
  },

  // ---- W3: regulators and the converter -------------------------------------------------------
  {
    kind: "linear_reg", pkg: "W3-R", cite: `${LS}wave5-regulators.test.ts and regulator-cc-exit.test.ts: a 7805-style regulator on a 9 V supply into a load`,
    parts: [V("vin", 9), ["u1", "linear_reg", { vout: 5, vdropout: 2, iLimit: 1 }], R("rl", 100), ["sx", "switch", { closed: 0 }], R("rx", 4.1667)],
    nets: [["vin.pos", "u1.in"], ["u1.gnd", "vin.neg", "gnd"], ["u1.out", "rl.a", "sx.a"], ["rl.b", "rx.b", "gnd"], ["sx.b", "rx.a"]],
    phases: [settle(0.01), set("vin", { voltage: 6 }, 0.01), set("vin", { voltage: 12 }, 0.01), set("sx", { closed: 1 }, 0.01)], // rx across rl: 100 ohm open, 4 ohm closed (past the 1 A limit)
  },
  {
    kind: "lm317", pkg: "W3-R", cite: `${LS}wave5-regulators.test.ts: LM317 set by its two resistors (1.25 V reference, ADJ divider)`,
    parts: [V("vin", 12), ["u1", "lm317", { vref: 1.25, vdropout: 2, iLimit: 1.5 }], R("r1", 240), R("r2", 720), R("rl", 100)],
    nets: [["vin.pos", "u1.in"], ["u1.out", "r1.a", "rl.a"], ["r1.b", "u1.adj", "r2.a"], ["r2.b", "gnd", "vin.neg"], ["rl.b", "gnd"]],
    phases: [settle(0.01), set("vin", { voltage: 5 }, 0.01), set("vin", { voltage: 15 }, 0.01)],
  },
  {
    kind: "dcdc_converter", pkg: "W3-X", cite: `${LS}wave5-converters-batteries.test.ts: dcdc_converter buck, regulated output at the input of a load`,
    parts: [V("vin", 12), ["u1", "dcdc_converter", { vout: 5, eta: 0.85, iLimit: 2, vdropout: 1.5 }], R("rl", 25), ["sx", "switch", { closed: 0 }], R("rx", 6.25)],
    nets: [["vin.pos", "u1.in_pos"], ["vin.neg", "u1.in_neg", "u1.out_neg", "gnd"], ["u1.out_pos", "rl.a", "sx.a"], ["rl.b", "rx.b", "gnd"], ["sx.b", "rx.a"]],
    phases: [settle(0.01), set("vin", { voltage: 8 }, 0.01), set("vin", { voltage: 14 }, 0.01), set("sx", { closed: 1 }, 0.01)], // rx across rl: 25 ohm open, 5 ohm closed
  },

  // ---- W4: the rest ---------------------------------------------------------------------------
  {
    kind: "relay", pkg: "W4-E", cite: `${LS}wave6-relay.test.ts: relay-spdt-5v, coil on a 5 V source, contact switching a load`,
    parts: [V("vcoil", 5), ["k1", "relay", { coilR: 70, coilL: 0.05, vPull: 3.5, vDrop: 1.5 }], V("vload", 5), R("rl", 1000), ["dfw", "diode", { vf: 0.7 }]],
    nets: [["vcoil.pos", "k1.coil_a", "dfw.k"], ["k1.coil_b", "vcoil.neg", "dfw.a", "gnd"], ["vload.pos", "k1.com"], ["k1.no", "rl.a"], ["rl.b", "vload.neg", "gnd"]],
    phases: [settle(0.02), set("vcoil", { voltage: 0 }, 0.02), set("vcoil", { voltage: 5 }, 0.02)],
  },
  {
    kind: "dc_motor", pkg: "W4-E", cite: `${LS}wave7-dcmotor.test.ts: dc-motor on a 5 V supply, then loaded`,
    parts: [V("vm", 5), ["m1", "dc_motor", { windingR: 5, windingL: 0.002, Ke: 0.01, Kt: 0.01, inertia: 1e-5, friction: 1e-5, loadTorque: 0 }]],
    nets: [["vm.pos", "m1.m1"], ["m1.m2", "vm.neg", "gnd"]],
    phases: [settle(0.05), set("m1", { loadTorque: 0.002 }, 0.05), set("vm", { voltage: 0 }, 0.05)],
  },
  {
    kind: "servo", pkg: "W4-E", cite: `${LS}wave7-servo-stepper.test.ts and servo-engine-physics.test.ts: servo-sg90 on a 5 V rail with a pulse train on its signal pin`,
    parts: [V("vcc", 5), PULSE("sigA", { per: 0.02, pw: 0.001, tr: 1e-7, tf: 1e-7 }), PULSE("sigB", { per: 0.02, pw: 0.002, tr: 1e-7, tf: 1e-7 }), ["swA", "switch", { closed: 1 }], ["swB", "switch", { closed: 0 }], ["s1", "servo"]],
    nets: [["vcc.pos", "s1.vplus"], ["vcc.neg", "s1.gnd", "sigA.neg", "sigB.neg", "gnd"], ["sigA.pos", "swA.a"], ["sigB.pos", "swB.a"], ["swA.b", "swB.b", "s1.sig"]],
    phases: [
      settle(0.05),
      { set: [{ id: "swA", params: { closed: 0 } }, { id: "swB", params: { closed: 1 } }], durationS: 0.05 },
      { set: [{ id: "swB", params: { closed: 0 } }, { id: "swA", params: { closed: 1 } }], durationS: 0.05 },
    ], // 1 ms then 2 ms then 1 ms command pulses, picked by two switches (a pulse source has no live params on the core)
  },
  {
    kind: "stepper", pkg: "W4-E", cite: `${LS}wave7-servo-stepper.test.ts: stepper-bipolar-nema17, coil A energised, then coil B`,
    parts: [V("va", 5), V("vb", 0), ["st1", "stepper", { coilR: 10, coilL: 0.01, stepsPerRev: 200 }]],
    nets: [["va.pos", "st1.a1"], ["va.neg", "st1.a2", "gnd"], ["vb.pos", "st1.b1"], ["vb.neg", "st1.b2", "gnd"]],
    phases: [settle(0.02), set("va", { voltage: 0 }, 0.02), set("vb", { voltage: 5 }, 0.02)],
  },
  {
    kind: "buzzer", pkg: "W4-A", cite: `${LS}wave6-drivers-audio.test.ts: an active buzzer on a square-wave drive (the tone from its edges)`,
    parts: [PULSE("drv", { per: 1 / 440, pw: 0.5 / 440 }), ["bz", "buzzer", { type: "active", resistance: 32 }], ["sx", "switch", { closed: 1 }]],
    nets: [["drv.pos", "sx.a"], ["sx.b", "bz.p1"], ["bz.p2", "drv.neg", "gnd"]],
    phases: [settle(0.02), set("sx", { closed: 0 }, 0.02), set("sx", { closed: 1 }, 0.02)], // the drive gated by a switch: a pulse source has no live params on the core
  },
  {
    kind: "speaker", pkg: "W4-A", cite: `${LS}wave6-drivers-audio.test.ts: speaker-8ohm driven by a pulse source`,
    parts: [PULSE("drv", { per: 1e-3, pw: 5e-4 }), ["sp", "speaker", { resistance: 8 }], ["sx", "switch", { closed: 1 }]],
    nets: [["drv.pos", "sx.a"], ["sx.b", "sp.p1"], ["sp.p2", "drv.neg", "gnd"]],
    phases: [settle(0.02), set("sx", { closed: 0 }, 0.02), set("sx", { closed: 1 }, 0.02)], // the drive gated by a switch: a pulse source has no live params on the core
  },
  {
    kind: "hcsr04", pkg: "W4-A", cite: `test/sim/engine/hcsr04.test.ts: hcsr04 powered, TRIG pulsed, echo read through a load`,
    parts: [V("vcc", 5), PULSE("trig", { per: 0.06, pw: 1e-5, td: 0.001 }), ["u1", "hcsr04", { distanceCm: 50, noEcho: 0 }], R("rq", 10000)],
    nets: [["vcc.pos", "u1.vcc"], ["vcc.neg", "u1.gnd", "trig.neg", "gnd"], ["trig.pos", "u1.trig"], ["u1.echo", "rq.a"], ["rq.b", "gnd"]],
    phases: [settle(0.07), set("u1", { distanceCm: 120 }, 0.07), set("u1", { noEcho: 1 }, 0.07)],
  },
  {
    kind: "uln2003", pkg: "W4-D", cite: `${LS}wave6-drivers-audio.test.ts: ULN2003A channel 1 sinking a 5 V relay-coil load`,
    parts: [V("vcc", 5), V("in1", 5), R("rl", 100), ["u1", "uln2003"]],
    nets: [["vcc.pos", "u1.com", "rl.a"], ["rl.b", "u1.out1"], ["in1.pos", "u1.in1"], ["in1.neg", "u1.gnd", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("in1", { voltage: 0 }, 0.01), set("in1", { voltage: 5 }, 0.01)],
  },
  {
    kind: "uln2803", pkg: "W4-D", cite: `${LS}wave6-drivers-audio.test.ts: ULN2803A channel 8 sinking a load`,
    parts: [V("vcc", 5), V("in8", 5), R("rl", 100), ["u1", "uln2803"]],
    nets: [["vcc.pos", "u1.com", "rl.a"], ["rl.b", "u1.out8"], ["in8.pos", "u1.in8"], ["in8.neg", "u1.gnd", "vcc.neg", "gnd"]],
    phases: [settle(0.01), set("in8", { voltage: 0 }, 0.01), set("in8", { voltage: 5 }, 0.01)],
  },
  {
    kind: "l293d", pkg: "W4-D", cite: `${LS}wave6-hbridge.test.ts: L293D forward drive (IN1 HIGH, IN2 LOW) across a motor-sized load`,
    parts: [V("vcc", 5), V("in1", 5), V("in2", 0), R("rm", 20), ["u1", "l293d"]],
    nets: [["vcc.pos", "u1.vcc1", "u1.vcc2", "u1.en12"], ["vcc.neg", "u1.gnd1", "u1.gnd2", "u1.gnd3", "u1.gnd4", "gnd"], ["in1.pos", "u1.in1"], ["in2.pos", "u1.in2"], ["in1.neg", "gnd"], ["in2.neg", "gnd"], ["u1.out1", "rm.a"], ["rm.b", "u1.out2"]],
    phases: [settle(0.01), set("in1", { voltage: 0 }, 0.01), set("in2", { voltage: 5 }, 0.01), set("in2", { voltage: 0 }, 0.01)],
  },
  {
    kind: "tb6612", pkg: "W4-D", cite: `${LS}wave6-hbridge.test.ts: TB6612FNG channel A forward, STBY high`,
    parts: [V("vm", 6), V("vcc", 5), V("ain1", 5), V("ain2", 0), R("rm", 20), ["u1", "tb6612"]],
    nets: [["vm.pos", "u1.vm"], ["vcc.pos", "u1.vcc", "u1.stby", "u1.pwma"], ["vm.neg", "vcc.neg", "u1.gnd", "u1.gnd2", "u1.gnd3", "gnd"], ["ain1.pos", "u1.ain1"], ["ain2.pos", "u1.ain2"], ["ain1.neg", "gnd"], ["ain2.neg", "gnd"], ["u1.ao1", "rm.a"], ["rm.b", "u1.ao2"]],
    phases: [settle(0.01), set("ain1", { voltage: 0 }, 0.01), set("ain2", { voltage: 5 }, 0.01), set("ain2", { voltage: 0 }, 0.01)],
  },
  {
    kind: "hd44780", pkg: "W4-P", cite: `${LS}wave8-hd44780.test.ts: HD44780 powered with E strobed (the decoder's write-strobe path)`,
    parts: [V("vcc", 5), PULSE("e", { per: 1e-3, pw: 5e-4 }), DRIVE("rs", 0), ["u1", "hd44780"]],
    nets: [["vcc.pos", "u1.vdd", "u1.a"], ["vcc.neg", "u1.vss", "u1.v0", "u1.rw", "u1.k", "e.neg", "rs.neg", "gnd"], ["e.pos", "u1.e"], ["rs.pos", "u1.rs"]],
    phases: [settle(0.01), set("rs", { voltage: 5 }, 0.01), set("rs", { voltage: 0 }, 0.01)],
  },
  {
    kind: "max7219", pkg: "W4-P", cite: `${LS}wave8-max7219.test.ts: MAX7219 powered with CLK strobed and CS held`,
    parts: [V("vcc", 5), PULSE("clk", { per: 1e-4, pw: 5e-5 }), DRIVE("din", 5), DRIVE("cs", 5), ["u1", "max7219"]],
    nets: [["vcc.pos", "u1.vcc"], ["vcc.neg", "u1.gnd", "clk.neg", "din.neg", "cs.neg", "gnd"], ["clk.pos", "u1.clk"], ["din.pos", "u1.din"], ["cs.pos", "u1.cs"]],
    phases: [settle(0.01), set("din", { voltage: 0 }, 0.01), set("cs", { voltage: 0 }, 0.01), set("cs", { voltage: 5 }, 0.01)],
  },
  {
    kind: "arduino_uno", pkg: "W4-S", cite: `${LS}mcu-usb-power-path.test.ts: boardWithPsu, a bench supply on the Uno's 5V pin with USB power off`,
    parts: [["u", "arduino_uno", { vcc: 5, usb_power: 0 }], V("p", 5)],
    nets: [["p.pos", "u.5v"], ["p.neg", "u.gnd", "gnd"]],
    phases: [settle(0.01), set("p", { voltage: 4.5 }, 0.01), set("p", { voltage: 5 }, 0.01)],
  },
  {
    kind: "arduino_nano", pkg: "W4-S", cite: `${LS}mcu-usb-power-path.test.ts: boardWithPsu, a bench supply on the Nano's 5V pin with USB power off`,
    parts: [["u", "arduino_nano", { vcc: 5, usb_power: 0 }], V("p", 5)],
    nets: [["p.pos", "u.5v"], ["p.neg", "u.gnd", "gnd"]],
    phases: [settle(0.01), set("p", { voltage: 4.5 }, 0.01), set("p", { voltage: 5 }, 0.01)],
  },
  {
    kind: "raspberry_pi_pico", pkg: "W7-R", cite: `${LS}mcu-usb-power-path.test.ts: boardWithPsu, a bench supply on the Pico's 3V3 pin with USB power off`,
    parts: [["u", "raspberry_pi_pico", { vcc: 3.3, usb_power: 0 }], V("p", 3.3)],
    nets: [["p.pos", "u.3v3"], ["p.neg", "u.gnd", "gnd"]],
    phases: [settle(0.01), set("p", { voltage: 3 }, 0.01), set("p", { voltage: 3.3 }, 0.01)],
  },
  {
    kind: "microbit", pkg: "W4-S", cite: `${LS}microbit-3v-path.test.ts: microbitWithPsu, a bench supply on the 3V pad with USB power off`,
    parts: [["mb", "microbit", { script: "", power: 0 }], V("p", 3.3)],
    nets: [["p.pos", "mb.3v"], ["p.neg", "mb.gnd", "gnd"]],
    phases: [settle(0.01), set("p", { voltage: 3 }, 0.01), set("p", { voltage: 3.3 }, 0.01)],
  },
];

/** The four boards are engine-owned, so the device registry does not list them. */
export const ENGINE_OWNED_BOARD_KINDS: readonly string[] = ["arduino_uno", "arduino_nano", "raspberry_pi_pico", "microbit"];

/** The pin ids of `kind`: the catalog fixture's first part of that kind, else EXTRA_PINS. */
export function pinsOfKind(kind: string): readonly string[] {
  const extra = EXTRA_PINS[kind];
  if (extra) return extra;
  const part = rawCatalog.parts.find((p) => p.kind === kind);
  if (!part) throw new Error(`skeleton-fixtures: no pin layout for kind "${kind}" (catalog fixture and EXTRA_PINS)`);
  return part.pin_layout.map((pin) => pin.id);
}

function defaultParamsOfKind(kind: string): Record<string, number | string> {
  if (EXTRA_PINS[kind]) return {};
  const part = rawCatalog.parts.find((p) => p.kind === kind);
  const out: Record<string, number | string> = {};
  for (const [key, value] of Object.entries(part?.default_params ?? {})) {
    if (typeof value === "number" || typeof value === "string") out[key] = value;
  }
  return out;
}

/** A skeleton as the SimCircuit the harness loads: ground added, params over the catalog defaults, one wire per consecutive pair of a net. */
export function buildSkeletonCircuit(spec: SkeletonSpec): SimCircuit {
  const components: SimCircuit["components"] = [{ id: "gnd", kind: "ground", pins: [{ id: "gnd" }], params: {} }];
  const pinsById = new Map<string, readonly string[]>();
  for (const [id, kind, params] of spec.parts) {
    const pins = pinsOfKind(kind);
    pinsById.set(id, pins);
    components.push({ id, kind, pins: pins.map((p) => ({ id: p })), params: { ...defaultParamsOfKind(kind), ...(params ?? {}) } });
  }
  const wires: SimCircuit["wires"] = [];
  const end = (token: string): { component: string; pin: string } => {
    if (token === "gnd") return { component: "gnd", pin: "gnd" };
    const dot = token.indexOf(".");
    const component = token.slice(0, dot);
    const pin = token.slice(dot + 1);
    if (!pinsById.get(component)?.includes(pin)) throw new Error(`skeleton ${spec.kind}: "${token}" is not a pin of a part of the circuit`);
    return { component, pin };
  };
  for (const net of spec.nets) {
    for (let i = 1; i < net.length; i++) {
      const a = end(net[0]!);
      const b = end(net[i]!);
      wires.push({ from_component: a.component, from_pin: a.pin, to_component: b.component, to_pin: b.pin });
    }
  }
  return { components, wires };
}

export interface LoadedSkeleton {
  entry: CorpusEntry;
  circuit: SimCircuit;
  spec: SkeletonSpec;
}

export function loadSkeletonCorpus(): LoadedSkeleton[] {
  return SKELETONS.map((spec) => {
    const circuit = buildSkeletonCircuit(spec);
    const entry: CorpusEntry = {
      id: `skeleton/${spec.kind}`,
      origin: "skeleton",
      name: spec.kind,
      durationS: spec.phases.reduce((sum, phase) => sum + phase.durationS, 0),
      kinds: kindsOf(circuit),
      stimulus: null,
      file: `skeleton-fixtures.ts:${spec.kind}`,
      phases: spec.phases,
      openInput: detectOpenInput(circuit),
    };
    return { entry, circuit, spec };
  });
}
