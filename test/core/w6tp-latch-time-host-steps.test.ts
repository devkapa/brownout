/**
 * Core-side counterpart for physics-telemetry.test.ts, "preserves the engine's
 * optional value/limit metadata for a latched failure" (gate 1, W6-T, case 11).
 * Classic reads `since` as 0.2, the end of the 200 ms step that crossed the
 * threshold; the crossing is 1/9 s, which is what the core reports, at host steps
 * of 200 ms, 10 ms and 1 ms (core/README.md, "Integrated latch times (Q-O31)").
 */
import { beforeAll, describe, expect, it } from "vitest";
import { initCore } from "../../src/core/wasm.js";
import { CoreEngine } from "../../src/core/engine.js";
import type { SimCircuit } from "../../src/sim/engine/sim-engine.js";

type Comp = SimCircuit["components"][number];

const wire = (a: string, ap: string, b: string, bp: string) => ({ from_component: a, from_pin: ap, to_component: b, to_pin: bp });
const part = (id: string, kind: string, pins: string[], params: Record<string, number | string> = {}, catalogUid?: string): Comp =>
  ({ id, kind, pins: pins.map((p) => ({ id: p })), params, ...(catalogUid ? { catalogUid } : {}) }) as Comp;

/** The failure key of a resistor overload, `failureKey("resistor_overload", id)` (src/core/failures.ts). */
const overloadKey = (id: string): string => `resistor_overload\u0000${id}\u0000`;

beforeAll(async () => {
  await initCore();
});

// ─── Case 11: a 10 ohm resistor at 5 V, rated 0.25 W ──────────────────────────

describe("a 10 ohm resistor on 5 V (2.5 W against its 0.25 W rating) latches at 1/9 s", () => {
  // P = 25 / 10 = 2.5 W, ten times the rating, so the stress rate is
  // P / 0.25 - 1 = 9 per second and the threshold 1 is reached at 1/9 s. The
  // latch time is the crossing itself (core/README.md, "Integrated latch times
  // (Q-O31)": held to 1e-9 s + 1e-9 relative), whatever the host step; the host
  // sees the failure at the first step whose end is at or after it. Classic's
  // 0.2 is the end of its 200 ms step.
  const board = (resistance: number): SimCircuit => ({
    components: [
      part("src", "voltage_source", ["pos", "neg"], { voltage: 5 }),
      part("r", "resistor", ["a", "b"], { resistance }),
    ],
    wires: [wire("src", "pos", "r", "a"), wire("r", "b", "src", "neg")],
  });
  const tolerance = (analytic: number): number => 1e-9 + 1e-9 * analytic;

  it.each([0.2, 0.01, 0.001])("with host steps of %s s", (h) => {
    const analytic = 1 / 9;
    const core = new CoreEngine();
    core.load(board(10));
    let seenAt = Number.NaN;
    let before = 0;
    for (let k = 1; k <= Math.ceil(0.3 / h); k++) {
      core.advance(k * h);
      const failure = core.getFailures()[overloadKey("r")];
      if (failure === undefined) {
        before++;
        continue;
      }
      seenAt = k * h;
      expect(failure.kind).toBe("resistor_overload");
      expect(Math.abs(failure.since - analytic)).toBeLessThanOrEqual(tolerance(analytic));
      expect(failure.limit).toBe(0.25);
      expect(failure.value).toBeCloseTo(2.5, 12);
      break;
    }
    // The first host step that ends at or after 1/9 s sees it, no earlier.
    expect(seenAt).toBeGreaterThanOrEqual(analytic - 1e-12);
    expect(seenAt).toBeLessThan(analytic + h + 1e-12);
    expect(before).toBe(Math.round(seenAt / h) - 1);
    const latched = core.takeLatchedFailures();
    expect(latched).toHaveLength(1);
    expect(Math.abs(latched[0]!.since - analytic)).toBeLessThanOrEqual(tolerance(analytic));
    core.dispose();
  });

  it("a 25 ohm resistor (1 W, rate 3) latches at 1/3 s: the readout can tell the instants apart", () => {
    const core = new CoreEngine();
    core.load(board(25));
    core.advance(0.2);
    expect(core.getFailures()).toEqual({});
    core.advance(0.4);
    expect(Math.abs(core.getFailures()[overloadKey("r")]!.since - 1 / 3)).toBeLessThanOrEqual(tolerance(1 / 3));
    core.dispose();
  });
});
