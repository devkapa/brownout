/**
 * G-0 item 3's checks: the stored ngspice references match the decks, the
 * cases the core can run today (a resistive divider `.op`, an RC step
 * `.tran`) pass against them with the classic test's envelopes, the check can
 * fail (a 5 % wrong circuit is rejected), and every other case is still
 * declined by the core for exactly the packages it waits on.
 */
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../../src/core/wasm.js";
import {
  NGSPICE_CASES,
  OP_LINEAR,
  buildCoreCircuit,
  compareOp,
  compareTran,
  findNgspice,
  generateReference,
  loadReference,
  ngspiceDeckFor,
  runCaseOnCore,
  waitsFor,
  type NgspiceCase,
} from "./ngspice-deck.js";

// P0-MAG: the K-coupled transformer runs on the core (coupled_inductor) and passes the classic test's envelope.
const LIVE_TODAY = ["divider-op", "rc-lowpass-tran", "transformer-tran"];

function caseById(id: string): NgspiceCase {
  const found = NGSPICE_CASES.find((c) => c.id === id);
  if (!found) throw new Error(`no ngspice case ${id}`);
  return found;
}

beforeAll(async () => {
  await initCore();
});

describe("stored ngspice references", () => {
  it("exist for every case, hold their analysis's data, and carry the deck they were generated from", () => {
    for (const testCase of NGSPICE_CASES) {
      const reference = loadReference(testCase.id);
      expect(reference, `${testCase.id}: run pnpm run ngspice:reference`).not.toBeNull();
      expect(reference!.deck, `${testCase.id}: the case's deck drifted from its stored reference; regenerate it`).toBe(ngspiceDeckFor(testCase).deck);
      const a = testCase.analysis;
      if (a.kind === "op") expect(Object.keys(reference!.op ?? {}).sort()).toEqual([...a.nodes].sort());
      if (a.kind === "tran") {
        expect(Object.keys(reference!.tran ?? {}).sort()).toEqual([...a.nodes].sort());
        for (const node of a.nodes) expect(reference!.tran![node]!.t.length).toBeGreaterThan(50);
      }
      if (a.kind === "opac") {
        expect(Object.keys(reference!.op ?? {}).sort()).toEqual([...a.opNodes].sort());
        expect(reference!.ac?.length).toBe(10);
      }
    }
  });

  it("parse to a circuit whose kinds say what each case waits on", () => {
    const waits = Object.fromEntries(NGSPICE_CASES.map((c) => [c.id, waitsFor(c)]));
    expect(waits["divider-op"]).toEqual([]);
    expect(waits["rc-lowpass-tran"]).toEqual([]);
    expect(waits["transformer-tran"]).toEqual([]);
    for (const id of ["rectifier-tran", "bjt-ce-op-ac", "rlc-ringdown-tran", "jfet-bias-op"]) {
      expect(waits[id]!.length, `${id} waits on something`).toBeGreaterThan(0);
    }
  });
});

describe("the live cases run on the core against the stored reference", () => {
  for (const id of LIVE_TODAY) {
    it(`${id} passes with the classic test's envelope`, () => {
      const testCase = caseById(id);
      const result = runCaseOnCore(testCase, loadReference(id)!);
      expect(result.status, JSON.stringify(result)).toBe("pass");
    });
  }

  it("rejects a divider that is 5 % wrong and an RC that is 10 % wrong, because the envelope is exceeded (not because the run errored)", () => {
    const divider = runCaseOnCore(caseById("divider-op"), loadReference("divider-op")!, { perturbFirstResistor: 1.05 });
    expect(divider.status).toBe("fail");
    expect(JSON.stringify(divider)).toMatch(/\.op worst \|dV\| [\d.e+-]+ V at v\(2\) \(envelope 4\.00e-2 V\)/);
    const rc = runCaseOnCore(caseById("rc-lowpass-tran"), loadReference("rc-lowpass-tran")!, { perturbFirstResistor: 1.1 });
    expect(rc.status).toBe("fail");
    expect(JSON.stringify(rc)).toMatch(/v\(out\) RMS [\d.e+-]+ V = [\d.]+% of 4\.750 V swing \(envelope 2%\)/);
    // The classic test's 2 % RMS envelope is as wide as that: a 5 % error in the RC's resistor (1.45 %) still passes, so the probe is 10 %.
    expect(runCaseOnCore(caseById("rc-lowpass-tran"), loadReference("rc-lowpass-tran")!, { perturbFirstResistor: 1.05 }).status).toBe("pass");
  });

  it("compares with the classic helpers' arithmetic: envelope edges and interpolation", () => {
    expect(compareOp({ a: 4.004 }, { a: 4 }, OP_LINEAR).ok).toBe(true);
    expect(compareOp({ a: 4.06 }, { a: 4 }, OP_LINEAR).ok).toBe(false);
    expect(compareOp({}, { a: 4 }, OP_LINEAR).ok).toBe(false);
    const ref = { t: [0, 1, 2], v: [0, 10, 0] };
    expect(compareTran([0, 0.5, 1, 1.5, 2], [0, 5, 10, 5, 0], ref, 2).pct).toBeCloseTo(0, 9);
    expect(compareTran([0, 1, 2], [0, 12, 0], ref, 2).ok).toBe(false);
  });
});

describe("the waiting cases are still declined by the core", () => {
  it("lists what each waits for and why the core says no", () => {
    const rows: string[] = [];
    for (const testCase of NGSPICE_CASES) {
      if (LIVE_TODAY.includes(testCase.id)) continue;
      const result = runCaseOnCore(testCase, loadReference(testCase.id)!);
      expect(result.status, `${testCase.id} is ready: the core runs it now, so list it in LIVE_TODAY and drop its extraWaits`).toBe("waiting");
      if (result.status === "waiting") rows.push(`${testCase.id}: waits for ${result.waitsFor.join(", ")}; core says ${result.declined.join(" | ") || "(extra wait only)"}`);
    }
    expect(rows.length).toBe(NGSPICE_CASES.length - LIVE_TODAY.length);
    // The circuits that hold a kind the core lacks are refused for that kind.
    expect(buildCoreCircuit(caseById("rectifier-tran")).circuit.components.some((c) => c.kind === "diode")).toBe(true);
  });
});

// With an ngspice binary on the machine, the stored references of the live cases are re-derived and must still agree
// (the stored file is not stale against the installed ngspice).
const ngspice = findNgspice();
describe.skipIf(ngspice === null)("stored references still match a live ngspice", () => {
  it("re-runs the live cases and agrees with the stored traces inside the envelope", () => {
    for (const id of LIVE_TODAY) {
      const testCase = caseById(id);
      const fresh = generateReference(testCase, ngspice!);
      const stored = loadReference(id)!;
      if (fresh.op) expect(compareOp(fresh.op, stored.op!, OP_LINEAR).ok, id).toBe(true);
      if (fresh.tran) {
        for (const [node, trace] of Object.entries(fresh.tran)) {
          expect(compareTran(trace.t, trace.v, stored.tran![node]!, 0.5).ok, `${id} v(${node})`).toBe(true);
        }
      }
    }
  });
});
