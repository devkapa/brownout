/**
 * W1h H3b: `signal_gen` on the core (spec 8.12; Q-O67 to Q-O74, D34) against classic driven as the equivalence
 * harness drives it (`HeadlessRunner`'s adaptive controller, which lands its steps on the generator's events: the
 * primary oracle, R56 12 and plan decision 12). Step 1 runs the piecewise-constant shapes (`dc`, `square`, `noise`
 * and a `pulse` whose ramps are midpoint steps) in all three forms (ideal, Norton, disabled), into a resistor (a
 * static partition), an RC (closed form) and an RC ladder (stepped). Every reading is held to W0 (100 uV + 1e-3
 * relative on a voltage, 1 nA + 1e-3 relative on a current), sampled away from the event instants, where classic's
 * side of an edge is a float accident (D34). `elementI` keeps the form's sign (Q-O67): the ideal form's branch
 * current is negative while delivering, the Norton form's sourced current positive.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { type SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { HeadlessRunner } from "../../src/host/headless.js";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";

type Component = SimCircuit["components"][number];
type Params = Record<string, number | string>;

/** Step 1 runs before the kind joins the default set (after step 2's gate). */
const WITH_SIGNAL_GEN = new Set<string>([...CORE_IMPLEMENTED_KINDS, "signal_gen"]);

function part(id: string, kind: string, pins: string[], params: Params = {}): Component {
  return { id, kind, pins: pins.map((p) => ({ id: p })), params };
}
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

function within(got: number, want: number, kind: "V" | "I"): boolean {
  return Math.abs(got - want) <= (kind === "V" ? 1e-4 : 1e-9) + 1e-3 * Math.max(Math.abs(got), Math.abs(want));
}

/** A reading the core matched only against the fine run: the adaptive oracle's own error (Q-O74, decision 12). */
interface Divergence {
  what: string;
  core: number;
  adaptive: number;
  fine: number;
}
const divergences: Divergence[] = [];
/**
 * Readings that both classic runs miss on a Norton generator driving a stepped partition, where the core reads what
 * it reads with the generator replaced by its ideal form behind a 50 ohm resistor (the control): the drive channel
 * stamps exactly the resistor, and the gap is the stepped step sequence after an edge, which the same control with a
 * W1f clock shows too (the R78 question: floor-step growth x2 against classic's x1.5).
 */
const controllerDivergences: Divergence[] = [];
/**
 * P0-SG: node readings on the Norton ladder that both classic runs miss (outside W0 of the fine run), each the
 * control's own reading bit for bit. On the 1.0.0 line the stepped step sequence after an edge leaves the second
 * capacitor's node 1.35 W0 from the fine run at 0.52 ms; whether the control's rule (W1h, elementI only) extends to
 * node voltages is a decision the P0-SG report raises, so the rows are pinned: any change fails here.
 */
const netControlDivergences: Divergence[] = [];

/**
 * Every classic reading, held to W0 against the adaptive run (the primary oracle). A reading outside it must be
 * within W0 of the fine run (the check), and is recorded; a reading outside both fails.
 */
function sameReadings(core: CoreEngine, adaptive: HeadlessRunner, fine: HeadlessRunner | undefined, what: string, opts: { elementI: boolean; control?: CoreEngine }): void {
  const snap = adaptive.snapshot();
  const fineSnap = fine?.snapshot();
  expect(snap.converged, `${what}: classic converged`).toBe(true);
  const check = (key: string, got: number | undefined, want: number, fineWant: number | undefined, kind: "V" | "I"): void => {
    expect(got, `${what} ${key}: present`).toBeTypeOf("number");
    if (within(got!, want, kind)) return;
    expect(fineWant, `${what} ${key}: core ${got} vs classic ${want}, and no fine run to check against`).toBeTypeOf("number");
    if (within(got!, fineWant!, kind)) {
      divergences.push({ what: `${what} ${key}`, core: got!, adaptive: want, fine: fineWant! });
      return;
    }
    if (key.startsWith("net ") && opts.control) {
      // P0-SG: a node reading both classic runs miss, on the Norton ladder, is held to its control's matching node
      // (the control has one more net, rs's far end, so its ids sit one higher) bit for bit and pinned below.
      const mapped = `n${Number(key.slice(5)) + 1}`;
      expect(got, `${what} ${key}: core ${got} vs classic ${want} (fine ${fineWant}) vs control`).toBe(opts.control.getNetV()[mapped]);
      netControlDivergences.push({ what: `${what} ${key}`, core: got!, adaptive: want, fine: fineWant! });
      return;
    }
    const id = key.startsWith("elementI ") ? key.slice("elementI ".length) : undefined;
    // The Norton current out of `pos` is the control resistor's `a -> b` current, `(Vt - V(pos)) / rSource`.
    const controlled = id === undefined ? undefined : opts.control?.getElementI()[id === "g" ? "rs" : id];
    expect(controlled, `${what} ${key}: core ${got} vs classic ${want} (fine ${fineWant})`).toBeTypeOf("number");
    expect(Math.abs(got! - controlled!), `${what} ${key}: core ${got} vs its control ${controlled}`).toBeLessThanOrEqual(1e-15 + 1e-9 * Math.abs(controlled!));
    controllerDivergences.push({ what: `${what} ${key}`, core: got!, adaptive: want, fine: fineWant! });
  };
  for (const [net, v] of Object.entries(snap.netV)) check(`net ${net}`, core.getNetV()[net], v, fineSnap?.netV[net], "V");
  if (!opts.elementI) return;
  for (const [id, i] of Object.entries(snap.elementI)) check(`elementI ${id}`, core.getElementI()[id], i, fineSnap?.elementI[id], "I");
}

/** The fine run's step: BE at 10 ns, about 1/5000 of the RC's 47.6 us, well inside W0 of the exact response. */
const FINE_STEP_S = 1e-8;

function runTo(adaptive: HeadlessRunner, fine: HeadlessRunner | undefined, core: CoreEngine, t: number): void {
  const result = adaptive.run({ durationS: t - adaptive.snapshot().simTime });
  expect(result.hitMinStep || result.hitStepCap, `classic reached ${t}`).toBe(false);
  if (fine) fine.run({ durationS: t - fine.snapshot().simTime, adaptive: false, fixedStepS: FINE_STEP_S, maxSteps: 1e7 });
  core.advance(t);
}

const SHAPES: Record<string, Params> = {
  dc: { waveform: "dc", offset: 3.3, amplitude: 9 },
  // Edges at 0.075 + n ms (rise) and 0.375 + n ms (fall).
  square: { waveform: "square", amplitude: 2.5, offset: 2.5, frequency: 1000, duty: 0.3, phaseDeg: 45, delay: 2e-4 },
  // Buckets every 0.1 ms from the delay.
  noise: { waveform: "noise", amplitude: 0.5, offset: 2, seed: 7, delay: 1e-4 },
  // 1 ns ramps: midpoint steps (W1f Q5). High on about [0.1, 0.3) ms of each cycle.
  pulse: { waveform: "pulse", amplitude: 2, offset: 1, frequency: 1000, tr: 1e-9, tf: 1e-9, pw: 2e-4, delay: 1e-4 },
};
const FORMS: Record<string, Params> = { ideal: { rSource: 0 }, norton: { rSource: 50 }, disabled: { enabled: 0 } };

/** Sample instants, none on an event of the shapes above. */
const SAMPLES = [0.05e-3, 0.13e-3, 0.31e-3, 0.52e-3, 0.77e-3, 1.21e-3, 1.55e-3, 2.33e-3];

/**
 * The generator into `load`: `r` 1 kohm (static), `rc` 1 kohm || 1 uF (closed form), `ladder` two 1 kohm / 1 uF
 * sections (stepped). A disabled generator gets a 5 V supply through 1 kohm on its `pos` so its node reads something.
 */
function circuit(shape: Params, form: Params, load: "r" | "rc" | "ladder", asControl = false): SimCircuit {
  // The control: the ideal form behind the Norton form's own resistance, as a resistor.
  const components: Component[] = [part("g", "signal_gen", ["pos", "neg"], { ...shape, ...form, ...(asControl ? { rSource: 0 } : {}) })];
  const wires: SimCircuit["wires"] = [];
  const [d, dp] = asControl ? ["rs", "b"] : ["g", "pos"];
  if (asControl) {
    components.push(part("rs", "resistor", ["a", "b"], { resistance: Number(form.rSource) }));
    wires.push(wire("g", "pos", "rs", "a"));
  }
  components.push(part("r1", "resistor", ["a", "b"], { resistance: 1000 }));
  wires.push(wire(d, dp, "r1", "a"), wire("r1", "b", "g", "neg"));
  if (load === "rc") {
    components.push(part("c1", "capacitor", ["a", "b"], { capacitance: 1e-6 }));
    wires.push(wire(d, dp, "c1", "a"), wire("c1", "b", "g", "neg"));
  }
  if (load === "ladder") {
    components.push(part("r2", "resistor", ["a", "b"], { resistance: 1000 }), part("c1", "capacitor", ["a", "b"], { capacitance: 1e-6 }), part("c2", "capacitor", ["a", "b"], { capacitance: 1e-6 }));
    wires.push(wire(d, dp, "c1", "a"), wire("c1", "b", "g", "neg"), wire(d, dp, "r2", "a"), wire("r2", "b", "c2", "a"), wire("c2", "b", "g", "neg"));
  }
  if (form.enabled === 0) {
    components.push(part("vcc", "voltage_source", ["pos", "neg"], { voltage: 5 }), part("rp", "resistor", ["a", "b"], { resistance: 1000 }));
    wires.push(wire("vcc", "pos", "rp", "a"), wire("rp", "b", "g", "pos"), wire("vcc", "neg", "g", "neg"));
  }
  return { components, wires };
}

beforeAll(async () => {
  await initCore();
});
const engines: CoreEngine[] = [];
afterAll(() => engines.forEach((e) => e.dispose()));

function pair(c: SimCircuit, withFine: boolean): { classic: HeadlessRunner; fine: HeadlessRunner | undefined; core: CoreEngine } {
  const classic = new HeadlessRunner();
  classic.load(c);
  let fine: HeadlessRunner | undefined;
  if (withFine) {
    fine = new HeadlessRunner();
    fine.load(c);
  }
  const core = new CoreEngine();
  core.load(c, WITH_SIGNAL_GEN);
  engines.push(core);
  return { classic, fine, core };
}

describe("signal_gen step 1 on the core against classic", () => {
  afterAll(() => {
    // The record decision 12 asks for: every reading the adaptive oracle missed, with the numbers.
    const rows = divergences.map((d) => `${d.what}: core ${d.core.toPrecision(7)}, adaptive ${d.adaptive.toPrecision(7)}, fine ${d.fine.toPrecision(7)}`);
    console.log(`signal_gen step 1: ${divergences.length} readings matched the fine run only\n${rows.join("\n")}`);
    const stepped = controllerDivergences.map((d) => `${d.what}: core ${d.core.toPrecision(7)}, adaptive ${d.adaptive.toPrecision(7)}, fine ${d.fine.toPrecision(7)}`);
    console.log(`signal_gen step 1: ${controllerDivergences.length} readings matched the control only\n${stepped.join("\n")}`);
    const nets = netControlDivergences.map((d) => `${d.what}: core ${d.core.toPrecision(7)}, adaptive ${d.adaptive.toPrecision(7)}, fine ${d.fine.toPrecision(7)}`);
    console.log(`signal_gen step 1: ${netControlDivergences.length} node readings matched the control only\n${nets.join("\n")}`);
  });

  for (const [shapeName, shape] of Object.entries(SHAPES)) {
    for (const [formName, form] of Object.entries(FORMS)) {
      for (const load of ["r", "rc", "ladder"] as const) {
        it(`${shapeName}, ${formName}, into ${load}`, () => {
          const c = circuit(shape, form, load);
          const { classic, fine, core } = pair(c, load !== "r");
          let control: CoreEngine | undefined;
          if (formName === "norton" && load === "ladder") {
            control = new CoreEngine();
            control.load(circuit(shape, form, load, true), WITH_SIGNAL_GEN);
            engines.push(control);
          }
          // Classic's 1 ps load seed charges a capacitor straight across an ideal source in one step (millions of
          // amps of elementI), a voltage_source's as well (the control below); the core's rule 2a capacitor has no
          // seed. Voltages agree at load; currents from the first sample on.
          sameReadings(core, classic, fine, `${shapeName} ${formName} ${load} at load`, { elementI: !(formName === "ideal" && load !== "r") });
          for (const t of SAMPLES) {
            runTo(classic, fine, core, t);
            control?.advance(t);
            sameReadings(core, classic, fine, `${shapeName} ${formName} ${load} at ${t}`, { elementI: true, control });
          }
        }, 60_000);
      }
    }
  }

  it("pins the node readings that match only the control (P0-SG: a decision raised, not a tolerance)", () => {
    expect(netControlDivergences.map((d) => d.what)).toEqual(["square norton ladder at 0.00052 net n0", "square norton ladder at 0.00155 net n0"]);
  });

  it("the load-seed current of a capacitor across an ideal source is classic's own, as a voltage_source shows", () => {
    const g = circuit(SHAPES.dc!, FORMS.ideal!, "rc");
    const v = circuit(SHAPES.dc!, FORMS.ideal!, "rc");
    v.components[0] = part("g", "voltage_source", ["pos", "neg"], { voltage: 3.3 });
    const seed = (c: SimCircuit): number => {
      const classic = new HeadlessRunner();
      classic.load(c);
      return classic.snapshot().elementI.g!;
    };
    expect(seed(g)).toBe(seed(v));
    expect(Math.abs(seed(g))).toBeGreaterThan(1e6);
  });

  it("keeps elementI's sign by form (Q-O67)", () => {
    const at = (form: Params): number => {
      const { classic, core } = pair(circuit(SHAPES.dc!, form, "r"), false);
      runTo(classic, undefined, core, 1e-3);
      sameReadings(core, classic, undefined, `rSource ${String(form.rSource)}`, { elementI: true });
      return core.getElementI().g!;
    };
    // 3.3 V into 1 kohm: the ideal branch current enters pos, the Norton current leaves it.
    expect(at({ rSource: 0 })).toBeLessThan(0);
    expect(at({ rSource: 50 })).toBeGreaterThan(0);
  });

  it("declines a generator on a hub rail, and step 2's shapes for now", () => {
    const decline = (c: SimCircuit): void => {
      const core = new CoreEngine();
      engines.push(core);
      expect(() => core.load(c, WITH_SIGNAL_GEN)).toThrow(/unsupported|-3|hub rail|not ported/i);
    };
    const rail: SimCircuit = {
      components: [part("psu", "bench_psu", ["pos", "neg"], { voltage: 10, iLimit: 1 }), part("r1", "resistor", ["a", "b"], { resistance: 1000 }), part("g", "signal_gen", ["pos", "neg"], { ...SHAPES.square, rSource: 50 })],
      wires: [wire("psu", "pos", "r1", "a"), wire("r1", "b", "psu", "neg"), wire("psu", "pos", "g", "pos"), wire("g", "neg", "psu", "neg")],
    };
    decline(rail);
    for (const waveform of ["sine", "triangle", "ramp"]) decline(circuit({ waveform, frequency: 1000 }, FORMS.norton!, "r"));
  });
});
