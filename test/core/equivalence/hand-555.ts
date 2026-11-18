/**
 * The hand model of a 555 on a battery's rail (ruling R122 (a), the
 * `handBound` marker; types.ts's `HandBoundSpec`). Shared by the harness's
 * hand-judged mode (hand-bound.ts) and by test/core/battery-555-corpus.test.ts.
 *
 * A 555's thresholds are 1/3 and 2/3 of its supply and the timing capacitor
 * charges toward the same supply, so a steady rail cancels: only R and C set
 * the times. What is left is the discharge pin, which pulls pin 7 to ground
 * through DIS_OHM while R_A still feeds it from the rail (core/README.md NE555
 * point 7), so the discharge runs through R_B + (DIS || R_A) toward
 * DIS / (DIS + R_A) of the rail instead of toward 0.
 */

export const LN2 = Math.LN2;
export const LN3 = Math.log(3);

export interface AstableHandModel {
  readonly kind: "astable";
  readonly raOhm: number;
  readonly rbOhm: number;
  readonly cF: number;
  /** The discharge pin's resistance (core/README.md NE555 point 7: 10 ohm). */
  readonly disOhm: number;
}

export interface MonostableHandModel {
  readonly kind: "monostable";
  readonly rOhm: number;
  readonly cF: number;
  /** When the trigger button is pressed (the fixture's phase boundary). */
  readonly pressS: number;
}

export type HandModel = AstableHandModel | MonostableHandModel;

/** The OUT edges a hand model puts inside `[0, endS]`, and what follows from them. */
export interface HandEdges {
  readonly up: number[];
  readonly down: number[];
  /** Astable: the period, and the HIGH fraction of it. */
  readonly periodS?: number;
  readonly duty?: number;
  /** Monostable: each pulse's width, in order (the power-on pulse, then the pressed one). */
  readonly pulsesS?: number[];
  /** With a counter on the 555's OUT that reads 1 from load and adds one at each rise. */
  readonly finalCount: number;
}

export function handEdges(model: HandModel, endS: number): HandEdges {
  if (model.kind === "monostable") {
    const width = LN3 * model.rOhm * model.cF;
    return { up: [model.pressS], down: [width, model.pressS + width], pulsesS: [width, width], finalCount: 2 };
  }
  const { raOhm: ra, rbOhm: rb, cF: c, disOhm: dis } = model;
  const tH = LN2 * (ra + rb) * c;
  const k = dis / (dis + ra);
  const tL = (rb + (dis * ra) / (dis + ra)) * c * Math.log((2 / 3 - k) / (1 / 3 - k));
  const up: number[] = [];
  const down: number[] = [];
  let t = LN3 * (ra + rb) * c; // from load the capacitor is empty and OUT high: 0 to 2/3
  down.push(t);
  for (;;) {
    t += tL;
    if (t > endS) break;
    up.push(t);
    t += tH;
    if (t > endS) break;
    down.push(t);
  }
  return { up, down, periodS: tH + tL, duty: tH / (tH + tL), finalCount: (1 + up.length) % 16 };
}

/** The same quantities read off measured edge lists. */
export function measuredQuantities(up: readonly number[], down: readonly number[], model: HandModel): { periodS?: number; duty?: number; pulsesS?: number[] } {
  if (model.kind === "monostable") {
    const pulses: number[] = [];
    if (down.length > 0) pulses.push(down[0]!);
    if (up.length > 0 && down.length > 1) pulses.push(down[1]! - up[0]!);
    return { pulsesS: pulses };
  }
  // A complete cycle is a rise, the fall after it and the next rise.
  let high = 0;
  let low = 0;
  let n = 0;
  for (let i = 0; i + 1 < up.length; i++) {
    const fall = down.find((d) => d > up[i]!);
    if (fall === undefined || fall > up[i + 1]!) continue;
    high += fall - up[i]!;
    low += up[i + 1]! - fall;
    n++;
  }
  if (n === 0) return {};
  return { periodS: (high + low) / n, duty: high / (high + low) };
}
