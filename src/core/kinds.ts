/**
 * Two different questions get asked about a device kind and the core, and
 * conflating them is the failure mode this file exists to prevent:
 *
 *   1. Does the TS compiler know how to turn this kind into a program
 *      record (or, for `ground`, into nothing), and does the W0 Rust kernel
 *      know that record's physics? -> CORE_IMPLEMENTED_KINDS.
 *   2. Has that implementation been PROVEN equivalent to the classic engine
 *      on real circuits, so it is safe to actually route a circuit's
 *      simulation through the core instead of the classic engine?
 *      -> CORE_SUPPORTED_KINDS.
 *
 * "Implemented" answers (1); "supported" answers (2). A kind can be
 * implemented for a long time before it is supported — that gap is
 * deliberate, not a bug: it is the space the equivalence work happens in.
 */

/**
 * Kinds compileCircuit (compile.ts) can turn into a program record, and the
 * W0 kernel's fixed-node/partition/solve logic (core/README.md, "W0 kernel
 * semantics") knows the physics of. `ground` is included even though it
 * compiles to no record at all: it is still a kind the compiler fully
 * understands, as opposed to one it has never heard of.
 *
 * This set is what a circuit is checked against to compile *at all* — it is
 * not, by itself, permission to run that circuit through the core instead
 * of the classic engine. See CORE_SUPPORTED_KINDS.
 */
export const CORE_IMPLEMENTED_KINDS: ReadonlySet<string> = new Set([
  "ground",
  "resistor",
  "voltage_source",
]);

/**
 * Kinds the engine-selection flag is allowed to route to the core for an
 * actual simulation. Empty in W0, on purpose: nothing has cleared the bar
 * yet, even though `resistor`/`voltage_source`/`ground` are fully
 * implemented above.
 *
 * A kind is added here only after it passes all three accuracy gates:
 *   1. brownout's own analytic-golden and first-principles suites, run
 *      against the core instead of the classic engine;
 *   2. the dual-engine equivalence harness (core vs. classic, same
 *      circuits, same results within tolerance);
 *   3. the devolt sim/learn/simcore suites, green against the core.
 *
 * The engine-selection flag routes a WHOLE circuit to the core only when
 * every kind that circuit uses is in this set. A circuit with even one
 * kind missing from CORE_SUPPORTED_KINDS falls back to the classic engine
 * in full — there is no per-device split between the two engines.
 */
export const CORE_SUPPORTED_KINDS: ReadonlySet<string> = new Set();
