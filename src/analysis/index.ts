// Analysis module — teaching-grade waveform measurements and operating-point.
// Wave C adds trace-csv and operating-point modules co-located here.
// Wave E1 adds sweep job types, the sweep runner, and worker message types.
//
// WHY analysis-worker-client is excluded from this barrel:
//   It constructs a `new Worker(new URL(...))` which is a DOM/browser API.
//   Importing it in a vitest test (Node) or a server-side context would throw.
//   The client is exported from its own package.json exports entry instead so
//   callers can import it directly only in browser contexts.
export * from "./measure.js";
export * from "./trace-csv.js";
export * from "./operating-point.js";
export * from "./jobs.js";
export * from "./run-sweep.js";
export * from "./run-ac-sweep.js";
export * from "./run-monte-carlo.js";
export * from "./job-messages.js";
export * from "./checked-step.js";
