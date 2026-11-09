import { describe, expect, it } from "vitest";
import { readFileSync } from "node:fs";
import { join } from "node:path";
import { SimEngine, type SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { cloneArduinoUnoPins } from "../../../src/circuit/arduino.js";

// A linear bench supply's output stage is a series pass element: it sources
// current and does not sink it. Driven above its setpoint, it lets go of the
// output. The engine's bench_psu was an ideal source both ways, so it sank
// whatever an external source pushed back into it.

type Params = { voltage: number; iLimit?: number };

function psu(id: string, p: Params) {
  return { id, kind: "bench_psu", pins: [{ id: "pos" }, { id: "neg" }], params: { ...p } };
}

function run(circuit: SimCircuit, seconds = 0.05): { engine: SimEngine; allConverged: boolean } {
  const engine = new SimEngine();
  engine.load(circuit);
  let allConverged = engine.lastConverged;
  for (let i = 0; i < Math.round(seconds / 1e-3); i++) {
    engine.step(1e-3);
    allConverged &&= engine.lastConverged;
  }
  return { engine, allConverged };
}

function netOf(engine: SimEngine, comp: string, pin: string): number {
  const net = engine.nets.find((n) => n.pins.some(([c, p]) => c === comp && p === pin))!;
  return engine.getNetV()[net.id] ?? Number.NaN;
}

/** Positive = the supply delivers current (MNA branch current is into it). */
function delivered(engine: SimEngine, id: string): number {
  return -(engine.getElementI()[id] ?? Number.NaN);
}

function parallel(a: Params, b: Params, loadOhms = 100): SimCircuit {
  return {
    components: [
      psu("a", a),
      psu("b", b),
      { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: loadOhms } },
    ],
    wires: [
      { from_component: "a", from_pin: "pos", to_component: "b", to_pin: "pos" },
      { from_component: "a", from_pin: "neg", to_component: "b", to_pin: "neg" },
      { from_component: "a", from_pin: "pos", to_component: "r", to_pin: "a" },
      { from_component: "a", from_pin: "neg", to_component: "r", to_pin: "b" },
    ],
  };
}

describe("bench_psu does not sink current", () => {
  it("a 4.9 V PSU on a USB-powered Uno's 5V pin lets go instead of sinking 0.194 A", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "u", kind: "arduino_uno", pins: cloneArduinoUnoPins().map((p) => ({ id: p.id })), params: { vcc: 5, usb_power: 1 } },
        psu("p", { voltage: 4.9, iLimit: 1 }),
      ],
      wires: [
        { from_component: "p", from_pin: "pos", to_component: "u", to_pin: "5v" },
        { from_component: "p", from_pin: "neg", to_component: "u", to_pin: "gnd" },
      ],
    };
    const { engine, allConverged } = run(circuit);
    expect(allConverged).toBe(true);
    expect(Math.abs(delivered(engine, "p"))).toBeLessThan(1e-6);
    // The rail is the USB-fed one, as if the PSU were absent (see
    // mcu-usb-power-path.test.ts for 4.98117 V).
    expect(netOf(engine, "u", "5v")).toBeCloseTo(4.98117, 4);
  });

  for (const iLimit of [undefined, 1]) {
    const tag = iLimit === undefined ? "ideal" : "1 A limited";
    it(`two ${tag} PSUs in parallel at 5 V and 5.5 V: the higher one carries the load`, () => {
      const { engine, allConverged } = run(parallel({ voltage: 5, iLimit }, { voltage: 5.5, iLimit }));
      expect(allConverged).toBe(true);
      expect(netOf(engine, "r", "a") - netOf(engine, "r", "b")).toBeCloseTo(5.5, 4);
      expect(Math.abs(delivered(engine, "a"))).toBeLessThan(1e-6);
      expect(delivered(engine, "b")).toBeCloseTo(0.055, 6);
    });

    it(`two ${tag} PSUs in parallel at the same 5 V setpoint converge and share the load`, () => {
      const { engine, allConverged } = run(parallel({ voltage: 5, iLimit }, { voltage: 5, iLimit }));
      expect(allConverged).toBe(true);
      expect(netOf(engine, "r", "a") - netOf(engine, "r", "b")).toBeCloseTo(5, 4);
      expect(delivered(engine, "a") + delivered(engine, "b")).toBeCloseTo(0.05, 6);
      expect(delivered(engine, "a")).toBeGreaterThanOrEqual(-1e-6);
      expect(delivered(engine, "b")).toBeGreaterThanOrEqual(-1e-6);
    });
  }

  it("the lower PSU takes over once the higher one current-limits", () => {
    // 5.5 V limited to 30 mA into 100 ohm cannot hold 5.5 V (55 mA): it folds
    // to CC, the rail falls to 5 V, and the 5 V supply makes up the rest.
    const { engine, allConverged } = run(parallel({ voltage: 5, iLimit: 1 }, { voltage: 5.5, iLimit: 0.03 }));
    expect(allConverged).toBe(true);
    expect(netOf(engine, "r", "a") - netOf(engine, "r", "b")).toBeCloseTo(5, 4);
    expect(delivered(engine, "b")).toBeCloseTo(0.03, 6);
    expect(delivered(engine, "a")).toBeCloseTo(0.02, 6);
  });

  it("takes the rail back each time a pulsed source stops driving it", () => {
    // A 0/7 V pulse generator drives the 5 V rail through 10 ohm: while it is
    // at 7 V it holds the rail above 5 V and the PSU lets go; at 0 V it loads
    // the rail and the PSU must take it back at 5 V, on every period.
    const circuit: SimCircuit = {
      components: [
        psu("a", { voltage: 5, iLimit: 2 }),
        { id: "g", kind: "pulse_gen", pins: [{ id: "pos" }, { id: "neg" }], params: { v1: 0, v2: 7, td: 0, tr: 1e-6, tf: 1e-6, pw: 0.005, per: 0.01 } },
        { id: "rs", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
      ],
      wires: [
        { from_component: "g", from_pin: "pos", to_component: "rs", to_pin: "a" },
        { from_component: "rs", from_pin: "b", to_component: "a", to_pin: "pos" },
        { from_component: "a", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "a", from_pin: "neg", to_component: "r", to_pin: "b" },
        { from_component: "g", from_pin: "neg", to_component: "a", to_pin: "neg" },
      ],
    };
    const engine = new SimEngine();
    engine.load(circuit);
    const rail = () => netOf(engine, "r", "a") - netOf(engine, "r", "b");
    const high: number[] = [];
    const low: number[] = [];
    for (let i = 0; i < 300; i++) {
      engine.step(1e-4);
      expect(engine.lastConverged).toBe(true);
      const phase = (engine.simTime % 0.01) / 0.01;
      if (phase > 0.1 && phase < 0.45) high.push(rail());
      if (phase > 0.6 && phase < 0.95) low.push(rail());
    }
    // 7 V into 10 ohm || 100 ohm with the PSU let go: 7 * 100 / 110.
    for (const v of high) expect(v).toBeCloseTo(700 / 110, 3);
    for (const v of low) expect(v).toBeCloseTo(5, 4);
  });

  it("recovers to CV when the source driving it away goes", () => {
    const { engine } = run(parallel({ voltage: 5 }, { voltage: 5.5 }));
    const solo: SimCircuit = {
      components: [psu("a", { voltage: 5 }), { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } }],
      wires: [
        { from_component: "a", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "a", from_pin: "neg", to_component: "r", to_pin: "b" },
      ],
    };
    engine.load(solo);
    expect(netOf(engine, "r", "a") - netOf(engine, "r", "b")).toBeCloseTo(5, 6);
    engine.step(1e-3);
    expect(netOf(engine, "r", "a") - netOf(engine, "r", "b")).toBeCloseTo(5, 6);
    expect(delivered(engine, "a")).toBeCloseTo(0.05, 9);
  });
});

describe("bench_psu let-go in the dc operating point", () => {
  it("decides the regime on the converged point: two supplies at 5 V and 5.5 V", () => {
    const engine = new SimEngine();
    engine.load(parallel({ voltage: 5, iLimit: 1 }, { voltage: 5.5, iLimit: 1 }));
    const op = engine.dcOperatingPoint();
    expect(op.converged).toBe(true);
    expect(netOf(engine, "r", "a") - netOf(engine, "r", "b")).toBeCloseTo(5.5, 5);
    expect(Math.abs(delivered(engine, "a"))).toBeLessThan(1e-6);
  });

  it("keeps the supply on through the ladder of a 555 switching an inductive load", () => {
    // devolt's fixture 13 (555 driving an NPN into a 10 mH coil, no flyback
    // diode), as devolt's breadboardToSimCircuit emits it. A let-go decided
    // inside dc Newton iterates switched the supply off mid-ladder and the
    // operating point failed; decided on converged points it solves directly.
    const circuit = JSON.parse(readFileSync(join(__dirname, "__fixtures__/inductive-load-no-flyback.sim.json"), "utf8"));
    const engine = new SimEngine();
    engine.load(circuit);
    const op = engine.dcOperatingPoint();
    expect(op.converged).toBe(true);
    expect(op.method).toBe("direct");
    expect(netOf(engine, "psu1", "pos") - netOf(engine, "psu1", "neg")).toBeCloseTo(5, 6);
    expect(delivered(engine, "psu1")).toBeGreaterThan(0);
  });
});

describe("bench_psu let-go and take-back are crossings inside a step", () => {
  // A 0 -> 10 -> 0 V triangle (10 ms up, 10 ms down) drives a 5 V supply's
  // rail through 10 ohm into a 100 ohm load. The supply lets go once the
  // source alone holds the rail above 5 V (source above 5.5 V, at 5.5 ms)
  // and takes it back on the way down (14.5 ms), both inside 1 ms steps.
  // Each step's published endpoint must already be in the regime its
  // endpoint asks for: a switch deferred to the step boundary would publish
  // the 6 ms sample with the supply sinking.
  function triangle(): SimCircuit {
    return {
      components: [
        psu("a", { voltage: 5, iLimit: 2 }),
        { id: "g", kind: "pulse_gen", pins: [{ id: "pos" }, { id: "neg" }], params: { v1: 0, v2: 10, td: 0, tr: 0.01, tf: 0.01, pw: 0, per: 1 } },
        { id: "rs", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 10 } },
        { id: "r", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
      ],
      wires: [
        { from_component: "g", from_pin: "pos", to_component: "rs", to_pin: "a" },
        { from_component: "rs", from_pin: "b", to_component: "a", to_pin: "pos" },
        { from_component: "a", from_pin: "pos", to_component: "r", to_pin: "a" },
        { from_component: "a", from_pin: "neg", to_component: "r", to_pin: "b" },
        { from_component: "g", from_pin: "neg", to_component: "a", to_pin: "neg" },
      ],
    };
  }

  it("publishes every 1 ms endpoint in the regime the source asks for", () => {
    const engine = new SimEngine();
    engine.load(triangle());
    for (let ms = 1; ms <= 20; ms++) {
      engine.step(1e-3);
      expect(engine.lastConverged).toBe(true);
      const vg = ms <= 10 ? ms : 20 - ms;
      const off = vg * 100 / 110 > 5;
      const rail = netOf(engine, "r", "a") - netOf(engine, "r", "b");
      expect(rail, `rail at ${ms} ms`).toBeCloseTo(off ? vg * 100 / 110 : 5, 5);
      expect(delivered(engine, "a"), `supply at ${ms} ms`).toBeCloseTo(off ? 0 : 0.05 - (vg - 5) / 10, 6);
    }
  });
});
