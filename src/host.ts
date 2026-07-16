/**
 * Host-adapter entry: the headless Node runner, plus the step-controller
 * contract every host adapter needs.
 *
 * WHY THIS IS A SEPARATE ENTRY AND NOT PART OF THE BARREL
 * The core is transport-free on purpose: "brownout" gives you a solver you
 * drive yourself, and it must stay importable somewhere with no loop, no
 * clock, and no host at all (a build step, a lambda, another engine's inner
 * loop). A host adapter is the opposite — it OWNS the loop and encodes a
 * policy about time. Folding one into the barrel would make every consumer
 * pay for a driving strategy most of them will replace, so HeadlessRunner is
 * deliberately NOT re-exported from src/index.ts. Reach it by subpath:
 *
 *   import { HeadlessRunner } from "brownout/host";
 *
 * This entry is for adapters that are cheap and dependency-free enough to
 * ship inside the engine package. The `@brownout/*` npm scope is reserved for
 * future STANDALONE adapters — anything that would drag a real dependency or
 * a platform assumption into this package (a browser worker adapter and its
 * transport protocol are the motivating case). Those ship as their own
 * packages against this entry's contract rather than widening brownout's
 * dependency surface; nothing in `@brownout/*` is published yet.
 *
 * WHAT THE CONTRACT IS
 * estimateStepError/nextStepFactor are exported here, not from the barrel,
 * because they are exactly what an out-of-tree adapter needs to reimplement
 * the loop and nothing a plain SimEngine consumer needs. HeadlessRunner is
 * their reference implementation: read src/host/headless.ts attemptStep() for
 * the coarse-vs-refined trial protocol they assume.
 *
 * Still unexported, pending the adapter that freezes their shape: the worker
 * protocol messages (sim/messages.ts) and the instrument DSP (meter, scope
 * channels/trigger, trace recorder/decimator). They describe a transport and
 * a UI cadence that no shipped adapter here has yet.
 */

export { HeadlessRunner } from "./host/headless.js";
export type {
  HeadlessRunOptions,
  HeadlessRunResult,
  HeadlessRunnerOptions,
  HeadlessSample,
  HeadlessSnapshot,
} from "./host/headless.js";

// The step-controller contract. See the header: the seam for out-of-tree
// adapters, not general-purpose engine API.
export { estimateStepError, nextStepFactor } from "./sim/adaptive-step.js";
export type { StepErrorEstimate } from "./sim/adaptive-step.js";
