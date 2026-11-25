/**
 * The kinds the core runs (CORE_IMPLEMENTED_KINDS) but a host is not yet allowed to route to it
 * (CORE_SUPPORTED_KINDS), each with the reason and the ruling that keeps it there, and the both-engine
 * comparisons that fail for it by design ("named not-yet rows") with the numbers behind each.
 *
 * Two tests hold this file to the code, so neither can rot:
 *  - test/core/promoted-kinds-fixtures.test.ts asserts IMPLEMENTED minus SUPPORTED is EXACTLY the kinds named here
 *    (a fifth unpromoted kind, or a silent promotion, fails it);
 *  - test/core/equivalence/skeleton-fixtures.test.ts asserts a skeleton of an unpromoted kind either passes or has a
 *    row below, and that a row below still fails (a row that starts passing must be deleted, and the kind judged for
 *    promotion).
 *
 * Truth below is an independent computation outside both engines (ship-logs/p0-mag/5-truth: the matrix exponential of
 * the piecewise-linear ODE, or fixed-step RK4 for the thermistor, h and h/2 agreeing at 1e-15); errors are in W0
 * units (tolerance.ts: 100 uV + 1e-3 relative for volts, 1 nA + 1e-3 relative for amps, of the truth).
 */

export interface UnpromotedKind {
  readonly kind: string;
  /** The ruling that keeps the kind implemented but unpromoted. */
  readonly ruling: string;
  readonly reason: string;
}

export const UNPROMOTED_KINDS: readonly UnpromotedKind[] = [
  {
    kind: "inductor",
    ruling: "R147",
    reason:
      "Implemented by P0-MAG, not promoted. The skeleton fails against classic and against the truth: the core's winding current carries an offset of V * 0.1 us / L after every pulse-source edge (5.0e-5 A on the skeleton, 1 to 6 W0 for 400 us after an edge) and reads the 1 us ramp as a midpoint step (README Sources point 3).",
  },
  {
    kind: "coupled_inductor",
    ruling: "R147",
    reason:
      "Implemented by P0-MAG, not promoted. The core is right and classic's 1 ms backward Euler step is wrong by up to 87000 W0, but the core is 1.0 to 5.7 W0 off the truth on the secondary current for about 1 ms after each edge, and reads 24 nA at t = 0 where the truth is 0.",
  },
  {
    kind: "crystal",
    ruling: "R147",
    reason:
      "Implemented by P0-MAG, not promoted. The ring of the motional branch is 47 W0 off the truth after a quarter period and 349 W0 off after ten periods (fixture 102), and a 10 us pulse ramp is read as a step that excites the resonance the ramp would not.",
  },
  {
    kind: "thermistor",
    ruling: "R147",
    reason:
      "Implemented by P0-MAG, not promoted. The core is within 0.7 W0 of the RK4 truth; classic, which stamps the temperature committed a step earlier, is off by up to 415 W0 at its 1 ms step, so the plain comparison against classic fails by design. Promotion waits for a closed-form (truth) counterpart in the table, R134 (3) (a).",
  },
];

/** The kinds the unpromoted rows name, as a set. */
export const UNPROMOTED_KIND_NAMES: ReadonlySet<string> = new Set(UNPROMOTED_KINDS.map((k) => k.kind));

export interface NotYetRow {
  /** The corpus entry id, as the harness names it. */
  readonly entry: string;
  readonly kind: string;
  /** The numbers: truth, core, classic, first instant. */
  readonly numbers: string;
}

/** Both-engine comparisons that fail by design, one row per entry. */
export const NOT_YET_ROWS: readonly NotYetRow[] = [
  {
    entry: "skeleton/inductor",
    kind: "inductor",
    numbers:
      "0.99 us (inside the 1 us rise): truth i = 2.4418e-4 A and n0 = 4.95 V; core 2.9383e-4 A (+4.97e-5 A, 202 W0) and 5.0 V (a midpoint step, 9 W0); classic 5.3349e-4 A (backward Euler across the whole ramp, 1180 W0). After the edge the core keeps +4.97e-5 A (V * 0.1 us / L), decaying with L / R: 1.05 W0 at 2.002 ms. Classic's 400 us steps are 6 to 289000 W0 off the truth.",
  },
  {
    entry: "synthetic/100-inductor-rl",
    kind: "inductor",
    numbers:
      "5 V through 100 ohm into 10 mH (dcr 2 ohm) from no current. 250 us: closed form i = 4.5192e-2 A, n1 = 0.48079 V; core 4.5201e-2 A (0.2 W0) and n1 = 0.47986 V (1.6 W0, 9.3e-4 V); classic backward Euler 4.5055e-2 A (3 W0) and n1 = 0.49447 V (25 W0). 500 us: n1 truth 0.12793 V, core 0.12770 V (1.0 W0), classic 0.13096 V. Classic's trapezoid equals the core to the digit. 1 ms: core within 0.1 W0.",
  },
  {
    entry: "skeleton/coupled_inductor",
    kind: "coupled_inductor",
    numbers:
      "99.99 us (inside the 100 us rise): truth n1 = 0.43427 V; core 0.43430 V (0.1 W0); classic 0.26032 V (325 W0). Core worst 5.7 W0 on the secondary current rl at 11.2 ms (truth 4.8e-5 A scale), 11 sample-quantities over 1 W0 of 156.",
  },
  {
    entry: "skeleton/crystal",
    kind: "crystal",
    numbers:
      "9.99 us (inside the 10 us rise): truth n2 = 5.0015e-4 V and i = 5.0015e-7 A; core -2.504e-5 V and +2.50e-8 A (5.2 W0 and 350 W0: a midpoint step at 5 us rings the 1 MHz motional branch for 10 periods, the ramp is exactly 10 periods and does not); classic 5.0608e-4 V and 5.0608e-7 A (4.0 W0). The ring left by the step is still 11.3 W0 on the current at 1 ms.",
  },
  {
    entry: "skeleton/thermistor",
    kind: "thermistor",
    numbers:
      "11 ms (1 ms after tempC goes 25 to 60): truth n1 = 1.9453 V, core 1.9453 V (0.7 W0 worst over the run); classic 2.4666 V (it stamps the temperature committed before the step, so its divider lags a whole 1 ms step; 415 W0 worst). Classic and core both lag the ambient change with tau = 3 ms; neither jumps.",
  },
];
