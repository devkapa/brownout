/**
 * W1h H1b: the LDR, push_dpdt, current source, ferrite bead, and the (unrated) pot, trimmer and resistor array run
 * on the core, against classic `SimEngine` on the same circuit and against the spec's formula (spec 7.6, 7.7, 7.9,
 * 7.14, 8.13, 10.10). Each circuit runs as a static partition, with a capacitor that makes it a closed-form or (with
 * `forceStepped`) a stepped partition, and on a bench PSU rail (a hub partition). The live paths (`setParams` with a
 * pot knob, a dpdt position, a source current; `setEnvironment` with a new lux) are held against classic reloaded
 * with the same change, which is what a host does on classic.
 *
 * The pot, trimmer and array are not in `CORE_IMPLEMENTED_KINDS` (their catalog parts are rated and their overloads
 * are H1c's), so the gate is widened here and those parts sit on `UNRATED`, a catalog uid with no `p_max`.
 */

import { afterAll, beforeAll, describe, expect, it } from "vitest";
import { SimEngine, type SimCircuit } from "../../src/sim/engine/sim-engine.js";
import { CoreEngine } from "../../src/core/engine.js";
import { CORE_IMPLEMENTED_KINDS } from "../../src/core/kinds.js";
import { initCore } from "../../src/core/wasm.js";

type Component = SimCircuit["components"][number];

const GATE: ReadonlySet<string> = new Set([...CORE_IMPLEMENTED_KINDS, "potentiometer", "trimmer", "resistor_array"]);
const UNRATED = "reg-7805";

function part(id: string, kind: string, pins: string[], params: Record<string, number | string> = {}, catalogUid?: string): Component {
  return { id, kind, pins: pins.map((p) => ({ id: p })), params, ...(catalogUid ? { catalogUid } : {}) };
}
const wire = (a: string, ap: string, b: string, bp: string): SimCircuit["wires"][number] => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });

type Variant = "static" | "closed-form" | "stepped" | "hub";
const VARIANTS: readonly Variant[] = ["static", "closed-form", "stepped", "hub"];

/**
 * The supply `vs` (5 V across pos/neg), as an ideal source or, for "hub", a 5 V bench PSU with a 1 A limit (CV
 * throughout), plus for the dynamic variants a 100 nF capacitor from `probe` to the supply's negative pin.
 */
function withSupply(variant: Variant, components: Component[], wires: SimCircuit["wires"], probe: [string, string]): SimCircuit {
  const supply = variant === "hub" ? part("vs", "bench_psu", ["pos", "neg"], { voltage: 5, iLimit: 1 }) : part("vs", "voltage_source", ["pos", "neg"], { voltage: 5 });
  const out: SimCircuit = { components: [supply, ...components], wires: [...wires] };
  if (variant === "closed-form" || variant === "stepped") {
    out.components.push(part("cprobe", "capacitor", ["a", "b"], { capacitance: 1e-7 }));
    out.wires.push(wire(probe[0], probe[1], "cprobe", "a"), wire("cprobe", "b", "vs", "neg"));
  }
  return out;
}

/** Long enough for the probe capacitor to settle many times over behind kilohm circuits. */
const RUN_S = 0.05;
const H = 1e-4;

function runClassic(circuit: SimCircuit, seconds = RUN_S): SimEngine {
  const engine = new SimEngine();
  engine.load(circuit);
  for (let i = 0; i < Math.round(seconds / H); i++) engine.step(H);
  return engine;
}

function runCore(circuit: SimCircuit, variant: Variant, seconds = RUN_S): CoreEngine {
  const engine = new CoreEngine();
  engine.load(circuit, GATE, { forceStepped: variant === "stepped" });
  engine.advance(seconds);
  return engine;
}

/** W0's tolerance: 100 uV + 1e-3 relative for a voltage, 1 nA + 1e-3 relative for a current. */
function close(got: number | undefined, want: number | undefined, what: string, kind: "V" | "I"): void {
  expect(got, `${what}: present`).toBeTypeOf("number");
  const abs = kind === "V" ? 1e-4 : 1e-9;
  const tolerance = abs + 1e-3 * Math.abs(want!);
  expect(Math.abs(got! - want!), `${what}: core ${got} vs ${want}`).toBeLessThanOrEqual(tolerance);
}

/** Every classic net voltage, element current and channel current, matched on the core. */
function sameReadings(core: CoreEngine, classic: SimEngine, what: string): void {
  for (const [net, v] of Object.entries(classic.getNetV())) close(core.getNetV()[net], v, `${what} net ${net}`, "V");
  for (const [id, i] of Object.entries(classic.getElementI())) close(core.getElementI()[id], i, `${what} elementI ${id}`, "I");
  for (const [id, channels] of Object.entries(classic.getElementChannelI())) {
    channels.forEach((i, ch) => close(core.getElementChannelI()[id]?.[ch], i, `${what} ${id} channel ${ch}`, "I"));
  }
}

function pinV(engine: CoreEngine | SimEngine, id: string, pin: string): number {
  return engine.getNetV()[engine.getNetIdForPin(id, pin)!]!;
}

beforeAll(async () => {
  await initCore();
});

const engines: CoreEngine[] = [];
afterAll(() => engines.forEach((e) => e.dispose()));
function core(circuit: SimCircuit, variant: Variant, seconds?: number): CoreEngine {
  const e = runCore(circuit, variant, seconds);
  engines.push(e);
  return e;
}

// ─── ldr (spec 7.9) ────────────────────────────────────────────────────────

function ldrDivider(variant: Variant, lux: number): SimCircuit {
  const c = withSupply(
    variant,
    [part("ldr1", "ldr", ["a", "b"]), part("r1", "resistor", ["a", "b"], { resistance: 10_000 })],
    [wire("vs", "pos", "ldr1", "a"), wire("ldr1", "b", "r1", "a"), wire("r1", "b", "vs", "neg")],
    ["r1", "a"],
  );
  c.environment = { lux };
  return c;
}

describe("ldr on the core", () => {
  it("matches classic and the GL5528 power law in every partition kind", () => {
    for (const variant of VARIANTS) {
      const circuit = ldrDivider(variant, 50);
      const e = core(circuit, variant);
      sameReadings(e, runClassic(circuit), `ldr ${variant}`);
      // R = r10 * (10 / lux)^gamma at the catalog defaults (spec 7.9): 15k * 0.2^-0.7.
      const r = 15_000 * Math.pow(10 / 50, 0.7);
      close(pinV(e, "r1", "a"), (5 * 10_000) / (r + 10_000), `ldr ${variant} divider`, "V");
    }
  });

  it("follows setEnvironment as classic follows a new environment", () => {
    for (const variant of VARIANTS) {
      const e = core(ldrDivider(variant, 50), variant);
      e.setEnvironment({ lux: 2000 });
      e.advance(2 * RUN_S);
      sameReadings(e, runClassic(ldrDivider(variant, 2000), 2 * RUN_S), `ldr ${variant} after setEnvironment`);
      // No part of this divider reads the ambient (H4c: only an LED or a thermal record pins it at load).
      expect(() => e.setEnvironment({ temperatureC: 60 })).not.toThrow();
    }
  });
});

// ─── push_dpdt (spec 7.6) ──────────────────────────────────────────────────

function dpdtRig(variant: Variant, position: number): SimCircuit {
  const loads = ["rL1", "rR1", "rL2", "rR2"].map((id, i) => part(id, "resistor", ["a", "b"], { resistance: 1000 * (i + 1) }));
  return withSupply(
    variant,
    [part("sw", "push_dpdt", ["l1", "c1", "r1", "l2", "c2", "r2"], { position }), part("rf", "resistor", ["a", "b"], { resistance: 470 }), ...loads],
    [
      wire("vs", "pos", "sw", "c1"),
      wire("vs", "pos", "rf", "a"),
      wire("rf", "b", "sw", "c2"),
      wire("sw", "l1", "rL1", "a"),
      wire("sw", "r1", "rR1", "a"),
      wire("sw", "l2", "rL2", "a"),
      wire("sw", "r2", "rR2", "a"),
      ...["rL1", "rR1", "rL2", "rR2"].map((id) => wire(id, "b", "vs", "neg")),
    ],
    ["rL2", "a"],
  );
}

describe("push_dpdt on the core", () => {
  it("matches classic at both positions in every partition kind, elementI pole 1 to its active throw", () => {
    for (const variant of VARIANTS) {
      for (const position of [0, 1]) {
        const circuit = dpdtRig(variant, position);
        const e = core(circuit, variant);
        sameReadings(e, runClassic(circuit), `dpdt ${variant} position ${position}`);
        const throwLoad = position === 0 ? 1000 : 2000;
        close(e.getElementI().sw, 5 / (throwLoad + 0.001), `dpdt ${variant} elementI`, "I");
      }
    }
  });

  it("switches live with setParams as classic does on a reload", () => {
    for (const variant of VARIANTS) {
      const e = core(dpdtRig(variant, 0), variant);
      e.setParams([{ componentId: "sw", params: { position: 1 } }]);
      e.advance(2 * RUN_S);
      sameReadings(e, runClassic(dpdtRig(variant, 1), 2 * RUN_S), `dpdt ${variant} after the toggle`);
    }
  });
});

// ─── current_source (spec 8.13) ────────────────────────────────────────────

/** 1 mA out of `pos` through 2.2 kohm into `neg` on the supply's negative rail; a second source with a 1 kohm shunt draws 0.5 mA off the supply through 470 ohm. */
function currentBoard(variant: Variant, current = 1e-3): SimCircuit {
  return withSupply(
    variant,
    [
      part("i1", "current_source", ["pos", "neg"], { current }),
      part("r1", "resistor", ["a", "b"], { resistance: 2200 }),
      part("i2", "current_source", ["pos", "neg"], { current: 0.5e-3, rParallel: 1000 }),
      part("r2", "resistor", ["a", "b"], { resistance: 470 }),
    ],
    [
      wire("i1", "pos", "r1", "a"),
      wire("r1", "b", "i1", "neg"),
      wire("i1", "neg", "vs", "neg"),
      wire("vs", "pos", "r2", "a"),
      wire("r2", "b", "i2", "neg"),
      wire("i2", "pos", "vs", "neg"),
    ],
    ["r1", "a"],
  );
}

describe("current_source on the core", () => {
  it("drives its current out of pos and publishes -I plus its shunt, matching classic in every partition kind", () => {
    for (const variant of VARIANTS) {
      const circuit = currentBoard(variant);
      const e = core(circuit, variant);
      sameReadings(e, runClassic(circuit), `current ${variant}`);
      close(pinV(e, "r1", "a") - pinV(e, "r1", "b"), 1e-3 * 2200, `current ${variant} I*R`, "V");
      close(e.getElementI().i1, -1e-3, `current ${variant} elementI`, "I");
      // i2 pushes 0.5 mA out of pos into the supply's negative rail, pulling its neg end down: its shunt and r2 in
      // series from 5 V, so V(pos - neg) = (5 + 0.5 mA * 470) * 1000 / 1470 ... solved by KCL at the neg node.
      const vNeg = (5 / 470 - 0.5e-3) / (1 / 470 + 1 / 1000);
      close(e.getElementI().i2, -0.5e-3 + (0 - vNeg) / 1000, `current ${variant} shunted elementI`, "I");
    }
  });

  it("takes a live current write as classic takes a reload", () => {
    for (const variant of VARIANTS) {
      const e = core(currentBoard(variant), variant);
      e.setParams([{ componentId: "i1", params: { current: 2.5e-3 } }]);
      e.advance(2 * RUN_S);
      sameReadings(e, runClassic(currentBoard(variant, 2.5e-3), 2 * RUN_S), `current ${variant} after the write`);
    }
  });
});

// ─── ferrite_bead (spec 7.14) ──────────────────────────────────────────────

describe("ferrite_bead on the core", () => {
  it("is a resistor at max(1e-3, rDc) in every partition kind, as classic", () => {
    for (const variant of VARIANTS) {
      for (const rDc of [0.5, 0]) {
        const circuit = withSupply(
          variant,
          [part("fb1", "ferrite_bead", ["a", "b"], { rDc }), part("r1", "resistor", ["a", "b"], { resistance: 220 })],
          [wire("vs", "pos", "fb1", "a"), wire("fb1", "b", "r1", "a"), wire("r1", "b", "vs", "neg")],
          ["r1", "a"],
        );
        const e = core(circuit, variant);
        sameReadings(e, runClassic(circuit), `bead ${variant} rDc ${rDc}`);
        close(e.getElementI().fb1, 5 / (220 + Math.max(1e-3, rDc)), `bead ${variant} current`, "I");
      }
    }
  });
});

// ─── potentiometer / trimmer, unrated (spec 7.7) ───────────────────────────

function potBoard(variant: Variant, kind: string, params: Record<string, number | string>): SimCircuit {
  return withSupply(
    variant,
    [part("p1", kind, ["cw", "wiper", "ccw"], { rTotal: 10_000, ...params }, UNRATED), part("rl", "resistor", ["a", "b"], { resistance: 4700 })],
    [wire("vs", "pos", "p1", "cw"), wire("p1", "ccw", "vs", "neg"), wire("p1", "wiper", "rl", "a"), wire("rl", "b", "vs", "neg")],
    ["rl", "a"],
  );
}

describe("unrated potentiometer and trimmer on the core", () => {
  it("match classic at the stops and between, in every partition kind, elementI cw to wiper", () => {
    for (const kind of ["potentiometer", "trimmer"]) {
      for (const variant of VARIANTS) {
        for (const position of [0, 0.3, 1]) {
          const circuit = potBoard(variant, kind, { position });
          const e = core(circuit, variant);
          sameReadings(e, runClassic(circuit), `${kind} ${variant} at ${position}`);
          const rCw = Math.max(1, 10_000 * (1 - position));
          close(e.getElementI().p1, (5 - pinV(e, "p1", "wiper")) / rCw, `${kind} ${variant} elementI`, "I");
        }
      }
    }
  });

  it("move both segments with one knob write, as classic does on a reload", () => {
    for (const variant of VARIANTS) {
      const e = core(potBoard(variant, "potentiometer", { position: 0.3 }), variant);
      e.setParams([{ componentId: "p1", params: { position: 0.8 } }]);
      e.advance(2 * RUN_S);
      sameReadings(e, runClassic(potBoard(variant, "potentiometer", { position: 0.8 }), 2 * RUN_S), `pot ${variant} after the knob`);
      // A later write of another knob name keeps the position it was given.
      e.setParams([{ componentId: "p1", params: { taper: "log" } }]);
      e.advance(3 * RUN_S);
      sameReadings(e, runClassic(potBoard(variant, "potentiometer", { position: 0.8, taper: "log" }), 3 * RUN_S), `pot ${variant} after the taper`);
    }
  });
});

// ─── resistor_array, unrated (spec 10.10) ──────────────────────────────────

describe("unrated resistor_array on the core", () => {
  it("publishes four isolated signed channels and their sum, matching classic in every partition kind", () => {
    for (const variant of VARIANTS) {
      const circuit = withSupply(
        variant,
        [
          part("rn1", "resistor_array", ["a1", "b1", "a2", "b2", "a3", "b3", "a4", "b4"], { resistance: 10_000 }, UNRATED),
          part("rf", "resistor", ["a", "b"], { resistance: 1000 }),
          part("rb", "resistor", ["a", "b"], { resistance: 4700 }),
        ],
        [
          // ch1: supply -1k- a1, b1 to ground. ch2 reversed: a2 on ground, b2 on the supply. ch3: a3 on the supply,
          // b3 through 4.7k to ground. ch4 unconnected.
          wire("vs", "pos", "rf", "a"),
          wire("rf", "b", "rn1", "a1"),
          wire("rn1", "b1", "vs", "neg"),
          wire("rn1", "a2", "vs", "neg"),
          wire("rn1", "b2", "vs", "pos"),
          wire("vs", "pos", "rn1", "a3"),
          wire("rn1", "b3", "rb", "a"),
          wire("rb", "b", "vs", "neg"),
        ],
        ["rb", "a"],
      );
      const e = core(circuit, variant);
      sameReadings(e, runClassic(circuit), `array ${variant}`);
      const channels = e.getElementChannelI().rn1!;
      close(channels[0], 5 / 11_000, `array ${variant} ch1`, "I");
      close(channels[1], -5 / 10_000, `array ${variant} ch2 (reversed)`, "I");
      close(channels[2], 5 / 14_700, `array ${variant} ch3`, "I");
      close(channels[3], 0, `array ${variant} ch4`, "I");
      close(e.getElementI().rn1, channels.reduce((s, i) => s + i, 0), `array ${variant} elementI`, "I");
    }
  });
});
