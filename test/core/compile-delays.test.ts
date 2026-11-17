/**
 * The IC delay tail's resolution (core/README.md, "IC delay tail" and
 * "Resolution"; src/core/compile/delays.ts): catalog figures, the
 * one-direction mirror, and the package-delay fallback.
 */
import { describe, expect, it } from "vitest";
import type { ElectricalSpecs } from "../../src/circuit/types.js";
import { CORE_DELAY_PATHS, resolveDelayTail } from "../../src/core/compile/delays.js";
import { CORE_IC_DELAY_PAIRS } from "../../src/core/kinds.js";
import { BUNDLED_DEFAULT_PARTS } from "../../src/parts/part-library.js";

const NS = 1e-9;

function catalogSpecs(uid: string): ElectricalSpecs {
  const part = BUNDLED_DEFAULT_PARTS.find((candidate) => candidate.uid === uid);
  if (!part?.electrical_specs) throw new Error(`no electrical_specs for ${uid}`);
  return part.electrical_specs;
}

function expectTail(actual: number[], expectedNs: number[]): void {
  expect(actual).toHaveLength(expectedNs.length);
  actual.forEach((value, i) => expect(value).toBeCloseTo(expectedNs[i]! * NS, 18));
}

describe("delay tail — the bundled catalog's figures", () => {
  it("74ls161: clkQ 13/18, clrQ 20/20 (tPHL mirrored), clkRco 20/18, entRco 9/9", () => {
    expectTail(resolveDelayTail("74ls161", catalogSpecs("ic-74ls161"), 20 * NS), [13, 18, 20, 20, 20, 18, 9, 9]);
  });

  it("74ls173: clkQ 17/22, clrQ 26/26, mnEnable 15/18, mnDisable 11/11", () => {
    expectTail(resolveDelayTail("74ls173", catalogSpecs("ic-74ls173"), 20 * NS), [17, 22, 26, 26, 15, 18, 11, 11]);
  });

  it("74hc74: clkQ 20/20, preClrQ 20/20, never the catalog's 25 ns package delay", () => {
    expectTail(resolveDelayTail("74hc74", catalogSpecs("ic-74hc74"), 25 * NS), [20, 20, 20, 20]);
  });

  it("74ls189: addrO 37/37 and csEnable 10/10; the unpublished pairs take the 20 ns package delay", () => {
    expectTail(resolveDelayTail("74ls189", catalogSpecs("ic-74ls189"), 20 * NS), [37, 37, 10, 10, 20, 20, 20, 20, 20, 20]);
  });

  it("28c16 and 28c256: the datasheet maxima, addrIo 150, ceEnable 150, ceDisable 50, oeEnable 70, oeDisable 50", () => {
    for (const [kind, uid] of [["28c16", "ic-28c16"], ["28c256", "ic-28c256"]] as const) {
      expectTail(resolveDelayTail(kind, catalogSpecs(uid), 0), [150, 150, 150, 150, 50, 50, 70, 70, 50, 50]);
    }
  });
});

describe("delay tail — resolution rules", () => {
  it("a catalog without switching_ns gives every pair the package delay in both directions", () => {
    expectTail(resolveDelayTail("74ls173", { prop_delay_ns: 20 } as ElectricalSpecs, 20 * NS), [20, 20, 20, 20, 20, 20, 20, 20]);
    expectTail(resolveDelayTail("74ls161", undefined, 20 * NS), [20, 20, 20, 20, 20, 20, 20, 20]);
  });

  it("a path published in one direction only carries that figure both ways", () => {
    const specs = { switching_ns: { clk_q: { tplh: 7 } } } as unknown as ElectricalSpecs;
    expectTail(resolveDelayTail("74hc74", specs, 15 * NS), [7, 7, 15, 15]);
  });

  it("a negative, NaN or non-numeric figure counts as unpublished", () => {
    const specs = { switching_ns: { clk_q: { tplh: -1, tphl: Number.NaN }, pre_clr_q: { tplh: "9" } } } as unknown as ElectricalSpecs;
    expectTail(resolveDelayTail("74hc74", specs, 15 * NS), [15, 15, 15, 15]);
  });

  it("a kind with no pairs has an empty tail", () => {
    expect(resolveDelayTail("74ls00", catalogSpecs("ic-74ls157"), 20 * NS)).toEqual([]);
  });

  it("every pair any kind declares has a catalog path", () => {
    for (const pairs of Object.values(CORE_IC_DELAY_PAIRS)) {
      for (const pair of pairs) expect(CORE_DELAY_PATHS[pair], pair).toBeDefined();
    }
  });
});
