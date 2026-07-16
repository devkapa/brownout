import { describe, expect, it } from "vitest";
import { solveNonlinear } from "../../../src/sim/engine/newton.js";

describe("solveNonlinear finite-value guard", () => {
  it("rejects a non-finite linear solve instead of declaring convergence", () => {
    const result = solveNonlinear({
      size: 1,
      xInit: new Float64Array([0]),
      clear: () => undefined,
      stamp: () => undefined,
      solve: () => new Float64Array([Number.NaN]),
    });

    expect(result.converged).toBe(false);
    expect(result.x[0]).toBe(0);
    expect(result.lastDelta).toBe(Number.POSITIVE_INFINITY);
  });
});
