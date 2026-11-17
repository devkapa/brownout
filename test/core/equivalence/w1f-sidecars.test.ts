/**
 * Unit coverage for w1f-sidecars.ts: each generator against an independent
 * reference (classic's `pulseVoltage`, a brute-force integration, or a hand
 * formula), and the harness detectors that read them shown to fail on a
 * seeded core value.
 */

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { beforeAll, describe, expect, it } from "vitest";
import { pulseVoltage } from "../../../src/sim/engine/digital.js";
import type { SimCircuit } from "../../../src/sim/engine/sim-engine.js";
import { initCore } from "../../../src/core/wasm.js";
import { CoreEngine } from "../../../src/core/engine.js";
import { explainAnalyticMismatches, judgeStimuliAnalyticCore, ne555Sidecars } from "./run-entry.js";
import type { StimulusStepSample } from "./stimuli.js";
import type { SyntheticExpectation } from "./synthetic-fixtures.js";
import type { AnalyticPsuRailModel, AnalyticPulseRcModel, BatteryDrainModel, CorpusEntry, Mismatch, PsuRailGateModel, RampCrossingsModel } from "./types.js";
import {
  extractBatteryDrain,
  extractPsuRail,
  extractPulseRc,
  pieceValueAt,
  psuRailGate,
  pulseModelPieces,
  pulsePieces,
  rampCrossings,
  type PulseParams,
} from "./w1f-sidecars.js";

const FIXTURES_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "..", "..", "fixtures", "core");
function fixture(name: string): { circuit: SimCircuit; expected: SyntheticExpectation } {
  return {
    circuit: JSON.parse(fs.readFileSync(path.join(FIXTURES_DIR, `${name}.sim.json`), "utf8")) as SimCircuit,
    expected: JSON.parse(fs.readFileSync(path.join(FIXTURES_DIR, `${name}.expected.json`), "utf8")) as SyntheticExpectation,
  };
}

function entryOf(name: string, expected: SyntheticExpectation): CorpusEntry {
  return {
    id: `synthetic/${name}`,
    origin: "synthetic",
    name,
    durationS: expected.durationS ?? null,
    kinds: [],
    stimulus: null,
    file: name,
    analytic: expected.analytic,
    phases: expected.phases,
    digital: expected.digital,
    sidecar: expected.sidecar,
  };
}

/** Classic RK4 on `y' = f(t, y)` from `t0` to `t1` in `n` steps. */
function rk4(f: (t: number, y: number) => number, y0: number, t0: number, t1: number, n: number): number {
  const h = (t1 - t0) / n;
  let y = y0;
  for (let i = 0; i < n; i++) {
    const t = t0 + i * h;
    const k1 = f(t, y);
    const k2 = f(t + h / 2, y + (h / 2) * k1);
    const k3 = f(t + h / 2, y + (h / 2) * k2);
    const k4 = f(t + h, y + h * k3);
    y += (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
  }
  return y;
}

describe("pulsePieces", () => {
  const cases: readonly PulseParams[] = [
    { v1: 0, v2: 5, td: 0, tr: 1e-6, tf: 1e-6, pw: 5e-4, per: 1e-3 },
    { v1: 0, v2: 5, td: 0, tr: 1e-3, tf: 1e-3, pw: 4e-3, per: 1e-2 },
    { v1: 5, v2: 0, td: 0.01, tr: 1e-6, tf: 1e-6, pw: 0.003, per: 0.01 },
    { v1: -1, v2: 2, td: 2e-4, tr: 0, tf: 3e-4, pw: 1e-4, per: 0 },
  ];
  it("matches classic's pulseVoltage away from the corners", () => {
    for (const p of cases) {
      const horizon = 0.035;
      const pieces = pulsePieces(p, horizon);
      for (let i = 1; i < 2000; i++) {
        const t = (horizon * i) / 2000 + 1.234567e-9;
        const piece = pieces.find((q) => t >= q.t0 && t < q.t1)!;
        expect(pieceValueAt(piece, t)).toBeCloseTo(pulseVoltage(t, p.v1, p.v2, p.td, p.tr, p.tf, p.pw, p.per), 6);
      }
    }
  });

  it("makes the catalog default's 1 us ramps midpoint steps and keeps a 1 ms ramp on a 4 ms segment (Sources point 3)", () => {
    const steps = pulseModelPieces(cases[0]!, 2e-3);
    expect(steps.every((q) => q.v0 === q.v1)).toBe(true);
    const want = [0, 0.5e-6, 5.015e-4, 1.0005e-3, 1.5015e-3];
    expect(steps).toHaveLength(want.length);
    steps.forEach((q, i) => expect(q.t0).toBeCloseTo(want[i]!, 15));
    const ramps = pulseModelPieces(cases[1]!, 0.01);
    expect(ramps.filter((q) => q.v0 !== q.v1)).toHaveLength(2);
  });
});

describe("extractPulseRc", () => {
  it("is the exact response of the model's waveform: brute-force RK4 agrees to 1e-9 V", () => {
    for (const name of ["41-pulse-161", "42-pulse-ramp-1ms"]) {
      const { circuit, expected } = fixture(name);
      const model = expected.analytic as AnalyticPulseRcModel;
      const rc = extractPulseRc(circuit, model, 0.03);
      const src = circuit.components.find((c) => c.id === model.source)!;
      const p = src.params as Record<string, number>;
      const pieces = pulseModelPieces({ v1: p.v1!, v2: p.v2!, td: p.td!, tr: p.tr!, tf: p.tf!, pw: p.pw!, per: p.per! }, 0.03);
      const R = 1000;
      const C = 1e-6;
      let x = 0;
      for (const piece of pieces) {
        const t1 = Math.min(piece.t1, 0.0125);
        if (piece.t0 >= t1) break;
        x = rk4((t, v) => ((pieceValueAt(piece, t) - v) / R - 1e-12 * v) / C, x, piece.t0, t1, 400);
        expect(Math.abs(rc.voltageAt(t1) - x)).toBeLessThan(1e-9);
      }
    }
  });
});

describe("extractPsuRail", () => {
  it("P9: 100 uF from 0 V at 0.1 A leaves CC at (V - esr iLimit) C / iLimit and relaxes with esr C", () => {
    const { circuit, expected } = fixture("46-psu-railcap-inrush-esr");
    const rail = extractPsuRail(circuit, expected.analytic as AnalyticPsuRailModel);
    expect(rail.exitsS).toHaveLength(1);
    expect(rail.exitsS[0]! / (((5 - 0.05 * 0.1) * 1e-4) / 0.1)).toBeCloseTo(1, 9);
    expect(rail.capacitorCurrentAt(rail.exitsS[0]! + 5e-6)).toBeCloseTo(0.1 * Math.exp(-1), 9);
    expect(rail.railAt(2e-3)).toBeCloseTo(2e-3 * 1000 + 0.05 * 0.1, 9);
    expect(rail.psuCurrentAt(2e-3)).toBe(-0.1);
    expect(rail.afterExit(4e-3)).toBe(false);
    expect(rail.afterExit(6e-3)).toBe(true);
  });

  it("a switched load: CC from 5 V toward 2 V with tau 20 ms, then out of CC rising toward 10 V with tau 100 ms", () => {
    const { circuit, expected } = fixture("44-psu-cc-load-step-railcap");
    const rail = extractPsuRail(circuit, expected.analytic as AnalyticPsuRailModel, expected.phases);
    // Load: C u' = 0.1 - u/100 from 0 reaches 5 V at 0.1 ln 2 (the node shunt moves it by 3e-12 s).
    expect(rail.exitsS[0]).toBeCloseTo(0.1 * Math.LN2, 10);
    // The switch closes at 80 ms (phase 1): the draw at 5 V is 0.25 A, so CC from 5 V.
    expect(rail.regimes.map((r) => r.cc)).toEqual([true, false, true, false]);
    expect(rail.regimes[2]!.t0).toBeCloseTo(0.08, 15);
    // The hand formulas leave out the node shunt, which moves these by about 1e-10, but not the switch's 1 mOhm
    // contact in series with rs: G is 0.01 + 1/25.001, not 0.05, which is 6e-5 V on uOpen (F1r10, D10).
    const gClosed = 0.01 + 1 / 25.001;
    const uOpen = 0.1 / gClosed + (5 - 0.1 / gClosed) * Math.exp((-gClosed / 1e-3) * 0.03);
    expect(Math.abs(uOpen - (2 + 3 * Math.exp(-1.5)))).toBeGreaterThan(5e-5);
    expect(rail.railAt(0.11, 1)).toBeCloseTo(uOpen, 9);
    // The phase-1 boundary sample still has the switch closed; phase 2's does not.
    expect(rail.loadCurrentAt.get("rs")!(0.11, 1)).toBeCloseTo(uOpen / 25.001, 9);
    expect(rail.loadCurrentAt.get("sw")!(0.11, 1)).toBeCloseTo(uOpen / 25.001, 9);
    expect(rail.loadCurrentAt.get("rs")!(0.1105, 2)).toBe(0);
    expect(rail.exitsS[1]).toBeCloseTo(0.11 + 0.1 * Math.log((10 - uOpen) / 5), 9);
  });
});

describe("extractPsuRail against the core inside the esr*C window (F1r10, D10)", () => {
  beforeAll(async () => {
    await initCore();
  });

  /**
   * The core over `circuit` with the entry's phases (each phase's `set` applied at its start, phase 0's at load as
   * stimuli.ts folds it), read at `times` (ascending, each tagged with its phase): the worst rail and capacitor
   * current against the sidecar, the current in W0 tolerances (`1 nA + 1e-3 |exact|`).
   */
  function worstAgainstCore(name: string, times: readonly { t: number; phase: number }[]): { volts: number; ampsTol: number } {
    const { circuit, expected } = fixture(name);
    const rail = extractPsuRail(circuit, expected.analytic as AnalyticPsuRailModel, expected.phases);
    const phases = expected.phases ?? [];
    const loaded: SimCircuit = {
      ...circuit,
      components: circuit.components.map((c) => {
        const set = phases[0]?.set?.find((s) => s.id === c.id);
        return set ? { ...c, params: { ...c.params, ...set.params } } : c;
      }),
    };
    const engine = new CoreEngine();
    let volts = 0;
    let ampsTol = 0;
    try {
      engine.load(loaded);
      let phase = 0;
      for (const { t, phase: p } of times) {
        while (phase < p) {
          phase++;
          const start = phases.slice(0, phase).reduce((sum, ph) => sum + ph.durationS, 0);
          engine.advance(start);
          const set = phases[phase]!.set ?? [];
          if (set.length > 0) engine.setParams(set.map((s) => ({ componentId: s.id, params: s.params as Record<string, number> })));
        }
        engine.advance(t);
        const u = engine.getNetV()[rail.rail]!;
        const i = engine.getElementI()[rail.capacitor]!;
        const exactI = rail.capacitorCurrentAt(t, p);
        volts = Math.max(volts, Math.abs(u - rail.railAt(t, p)));
        ampsTol = Math.max(ampsTol, Math.abs(i - exactI) / (1e-9 + 1e-3 * Math.abs(exactI)));
      }
    } finally {
      engine.dispose();
    }
    return { volts, ampsTol };
  }

  it("fixture 45: the switch's 1 mOhm contact puts the second exit 1.1 us before an ideal switch would", () => {
    const { circuit, expected } = fixture("45-psu-cc-load-step-railcap-esr");
    const rail = extractPsuRail(circuit, expected.analytic as AnalyticPsuRailModel, expected.phases);
    expect(rail.regimes.map((r) => r.cc)).toEqual([true, false, true, false]);
    // From 0 V toward 10 V at G / (C (1 + esr G)): vc reaches V (1 + esr G) - esr iLimit at 69.299 ms.
    const g = 0.01 + 1e-12;
    const k = g / (1e-3 * (1 + 0.05 * g));
    expect(Math.abs(rail.exitsS[0]! - -Math.log1p(-(5 * (1 + 0.05 * g) - 0.005) / (0.1 / g)) / k)).toBeLessThan(1e-15);
    // The exact coupled solution with the contact and node 2's shunt, solved independently (F1r10's Rust reference).
    expect(Math.abs(rail.exitsS[1]! - 0.14819583329670835)).toBeLessThan(1e-12);
    // An ideal switch (G = 0.05 in phase 1) put it at 0.148196949 s: F4's D10 reading.
    expect(0.14819694949781312 - rail.exitsS[1]!).toBeGreaterThan(1.1e-6);
  });

  it("fixture 45: the core is on the exact coupled rail every 100 us and every 1 us of both 50 us windows", () => {
    const { circuit, expected } = fixture("45-psu-cc-load-step-railcap-esr");
    const rail = extractPsuRail(circuit, expected.analytic as AnalyticPsuRailModel, expected.phases);
    const bounds = [0.08, 0.11, 0.17];
    const times: { t: number; phase: number }[] = [];
    for (let k = 1; k <= 1700; k++) times.push({ t: k * 1e-4, phase: 0 });
    for (const x of rail.exitsS) for (let k = 0; k < 60; k++) times.push({ t: x + k * 1e-6, phase: 0 });
    for (const s of times) s.phase = bounds.findIndex((b) => s.t <= b + 1e-15);
    times.sort((a, b) => a.t - b.t);
    const worst = worstAgainstCore("45-psu-cc-load-step-railcap-esr", times);
    // Against an ideal switch it was 8.2e-5 V and 2.2 % (22 tolerances) of the capacitor's current.
    expect(worst.volts).toBeLessThan(1e-10);
    expect(worst.ampsTol).toBeLessThan(1e-3);
  });

  // Fixture 51 (1 mF at esr 0.05 from 0 V, 100 ohm, {5 V, 0.1 A}) declares the CC stretch every 1 ms and its exit's
  // window from 0.6 to 50.6 us, all held to the exact rail by the always-on core check. Its durationS, 0.2772 s, puts
  // the W0 grid's D/4 sample 0.6 us past the exit, so classic is compared only from the exit on (P9): from one run to
  // 20 ms, classic's own CC ramp under a load is 0.35 % low.
  it("fixture 51: the core is on the exact rail at every instant it declares, through its exit's esr*C window", () => {
    const { expected } = fixture("51-psu-cc-exit-esr-window");
    const instants = (expected.analytic as AnalyticPsuRailModel).instantsS!;
    const worst = worstAgainstCore("51-psu-cc-exit-esr-window", instants.map((t) => ({ t, phase: 0 })));
    expect(worst.volts).toBeLessThan(1e-10);
    expect(worst.ampsTol).toBeLessThan(1e-3);
  });
});

describe("extractBatteryDrain", () => {
  it("with r = 0 the generic cell's ODE is one exponential, soc = e^(-V G t / Qcap); the RK4 table agrees to 1e-12", () => {
    const circuit: SimCircuit = {
      components: [
        { id: "gnd", kind: "ground", pins: [{ id: "gnd" }], params: {} },
        { id: "bat", kind: "battery_pack", pins: [{ id: "pos" }, { id: "neg" }], params: { voltage: 5, rInternal: 0, capacityAh: 1e-4 } },
        { id: "rl", kind: "resistor", pins: [{ id: "a" }, { id: "b" }], params: { resistance: 100 } },
      ],
      wires: [
        { from_component: "bat", from_pin: "neg", to_component: "gnd", to_pin: "gnd" },
        { from_component: "bat", from_pin: "pos", to_component: "rl", to_pin: "a" },
        { from_component: "rl", from_pin: "b", to_component: "gnd", to_pin: "gnd" },
      ],
    };
    const model: BatteryDrainModel = { kind: "battery-drain", battery: "bat", load: "rl", instantsS: [2] };
    const drain = extractBatteryDrain(circuit, model, 2);
    const G = 1 / 100 + 1e-12;
    const qCap = 1e-4 * 3600;
    for (const t of [0.3, 1, 1.7, 2]) {
      const soc = Math.exp((-5 * G * t) / qCap);
      expect(Math.abs(drain.at(t).soc - soc)).toBeLessThan(1e-12);
      expect(Math.abs(drain.at(t).dischargedCoulombs - (1 - soc) * qCap)).toBeLessThan(1e-12);
      expect(drain.at(t).openCircuitVoltageV).toBeCloseTo(5 * soc, 12);
    }
  });
});

describe("rampCrossings", () => {
  it("the 74HC14 on the 1 ms ramp: V_T+ 2.7 V at 0.54 ms and V_T- 1.6 V at 5.68 ms, each plus 15 ns", () => {
    const { circuit, expected } = fixture("42-pulse-ramp-1ms");
    const crossings = rampCrossings(circuit, expected.sidecar as RampCrossingsModel, 5, 0.012);
    expect(crossings.map((c) => c.value)).toEqual([0, 1, 0]);
    expect(crossings[0]!.t).toBeCloseTo(0.54e-3 + 15e-9, 15);
    expect(crossings[1]!.t).toBeCloseTo(5.68e-3 + 15e-9, 15);
    expect(crossings[2]!.t).toBeCloseTo(10.54e-3 + 15e-9, 15);
  });
});

describe("psuRailGate", () => {
  it("the first sag crosses 4.275 V where 2 + 3 e^(-dt / 2 ms) does, from the load step's midpoint", () => {
    const { circuit, expected } = fixture("50-psu-cc-rail-sag-74ls04");
    const gate = psuRailGate(circuit, expected.sidecar as PsuRailGateModel, 0.04);
    expect(gate.gateLowV).toBeCloseTo(4.275, 12);
    expect(gate.flips.map((f) => f.powered)).toEqual([true, false, true, false, true, false, true]);
    // With the node shunt left out of the hand formula the asymptote moves by 4e-11 V; the crossing by far less than 1 ns.
    const down = 0.01 + 0.5e-6 + 2e-3 * Math.log(3 / (4.275 - 2));
    expect(Math.abs(gate.flips[1]!.t - down)).toBeLessThan(1e-12);
  });

  it("agrees with a brute-force RK4 rail to the integration step", () => {
    const { circuit, expected } = fixture("50-psu-cc-rail-sag-74ls04");
    const gate = psuRailGate(circuit, expected.sidecar as PsuRailGateModel, 0.02);
    // Brute force: the same regimes, stepped at 10 ns, the load node from the model's pulse.
    const pieces = pulseModelPieces({ v1: 5, v2: 0, td: 0.01, tr: 1e-6, tf: 1e-6, pw: 0.003, per: 0.01 }, 0.02);
    const s = (t: number): number => pieceValueAt(pieces.find((q) => t >= q.t0 && t < q.t1) ?? pieces[pieces.length - 1]!, t);
    const h = 1e-8;
    let u = 0;
    let cc = true;
    let powered = false;
    const flips: number[] = [];
    for (let i = 0; i < 0.02 / h; i++) {
      const t = i * h;
      if (!cc && (5 - s(t)) / 20 + 1e-12 * 5 > 0.1) cc = true;
      if (cc) {
        u = rk4((tt, v) => (0.1 - (v - s(tt)) / 20 - 1e-12 * v) / 1e-4, u, t, t + h, 1);
        if (u >= 5) {
          u = 5;
          cc = false;
        }
      }
      if ((u >= 4.275) !== powered) {
        powered = !powered;
        flips.push(t + h);
      }
    }
    expect(flips).toHaveLength(gate.flips.length);
    // A flip is seen at the end of the step that crossed, and a step's state lags the source's jump by up to one step.
    flips.forEach((t, i) => expect(Math.abs(t - gate.flips[i]!.t)).toBeLessThanOrEqual(2 * h));
  });
});

describe("the W1f detectors fail on a seeded core value", () => {
  const { circuit, expected } = fixture("45-psu-cc-load-step-railcap-esr");
  const entry = entryOf("45-psu-cc-load-step-railcap-esr", expected);
  const rail = extractPsuRail(circuit, expected.analytic as AnalyticPsuRailModel, expected.phases);
  const exit = rail.exitsS[1]!;
  const mismatch = (t: number, phase: number, coreValue: number): Mismatch => ({
    sampleTime: t,
    phaseIndex: phase,
    category: "elementI",
    id: "c1",
    classicValue: 0.0123,
    coreValue,
    detail: "seeded",
  });

  it("P9: a post-exit sample is explained only when the core reads the exact value", () => {
    const t = exit + 2e-6;
    const exact = rail.capacitorCurrentAt(t, 2);
    expect(explainAnalyticMismatches(entry, circuit, [mismatch(t, 2, exact)]).explained).toHaveLength(1);
    // Off by ten tolerances: still a mismatch.
    const off = exact + 10 * (1e-9 + 1e-3 * Math.abs(exact));
    expect(explainAnalyticMismatches(entry, circuit, [mismatch(t, 2, off)]).remaining).toHaveLength(1);
  });

  it("P9: a sample before the exit stays classic-against-core even when the core is exact", () => {
    const t = exit - 1e-3;
    expect(rail.afterExit(t, 2)).toBe(false);
    const exact = rail.capacitorCurrentAt(t, 2);
    expect(explainAnalyticMismatches(entry, circuit, [mismatch(t, 2, exact)]).remaining).toHaveLength(1);
  });

  it("the stimuli path's always-on check flags a replayed core sample off the exact rail", () => {
    const t = exit + 1e-3;
    const exactI = rail.capacitorCurrentAt(t, 2);
    const sample = (c1: number): StimulusStepSample => ({
      phaseIndex: 2,
      simTime: t,
      netV: { [rail.rail]: rail.railAt(t, 2) },
      elementI: { c1, psu: rail.psuCurrentAt(t, 2), rl: rail.loadCurrentAt.get("rl")!(t, 2), rs: 0, sw: 0 },
      digitalState: {},
    } as unknown as StimulusStepSample);
    expect(judgeStimuliAnalyticCore(entry, circuit, [sample(exactI)])).toEqual([]);
    const flagged = judgeStimuliAnalyticCore(entry, circuit, [sample(exactI + 1e-6)]);
    expect(flagged.map((m) => m.id)).toEqual(["psu-rail.elementI.c1"]);
  });

  it("pulse-rc: a mismatch on the RC node is explained only within tolerance of the exact response", () => {
    const f = fixture("42-pulse-ramp-1ms");
    const e = entryOf("42-pulse-ramp-1ms", f.expected);
    const rc = extractPulseRc(f.circuit, f.expected.analytic as AnalyticPulseRcModel, 0.03);
    const t = 0.5e-3;
    const m = (core: number): Mismatch => ({ sampleTime: t, category: "netV", id: rc.node, classicValue: rc.voltageAt(t) + 0.02, coreValue: core, detail: "seeded" });
    expect(explainAnalyticMismatches(e, f.circuit, [m(rc.voltageAt(t))]).explained).toHaveLength(1);
    expect(explainAnalyticMismatches(e, f.circuit, [m(rc.voltageAt(t) + 2e-3)]).remaining).toHaveLength(1);
  });
});

describe("a 555 on a limited PSU's rail keeps its 1e-6 sidecar only while the PSU stays in CV (the moving-rail set)", () => {
  beforeAll(async () => {
    await initCore();
  });
  // Fixture 31's astable with its ideal 5 V source swapped for a bench_psu.
  const withPsu = (iLimit: number): SimCircuit => {
    const { circuit } = fixture("31-ne555-astable-led");
    return {
      ...circuit,
      components: circuit.components.map((c) => (c.id === "vcc" ? { ...c, kind: "bench_psu", params: { voltage: 5, iLimit } } : c)),
    };
  };
  const entry = entryOf("31-ne555-astable-led", { status: "pass", durationS: 0.02 } as SyntheticExpectation);

  it("a 1 A limit never limits the astable: the sidecar is extracted", () => {
    const { sidecars, unmodelled } = ne555Sidecars(entry, withPsu(1), 0.02);
    expect(sidecars.map((sc) => sc.spec.component)).toEqual(["u1"]);
    expect(unmodelled).toEqual([]);
  });

  it("a 1 mA limit puts the PSU in CC: the 555 is left to the classic comparison", () => {
    const { sidecars, unmodelled } = ne555Sidecars(entry, withPsu(1e-3), 0.02);
    expect(sidecars).toEqual([]);
    expect(unmodelled).toEqual(["u1"]);
  });
});
