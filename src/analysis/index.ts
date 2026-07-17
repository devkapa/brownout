// Analysis module — teaching-grade waveform measurements and operating-point.
// Wave C adds trace-csv and operating-point modules co-located here.
// Wave E1 adds sweep job types, the sweep runner, and worker message types.
//
// WHY analysis-worker-client is absent entirely:
//   It constructs a `new Worker(new URL(...))` which is a DOM/browser API, so
//   importing it in Node (vitest, SSR) would throw. That transport shell is
//   app-domain and was not extracted, so — unlike in simcore, where the client
//   shipped behind its own exports entry — brownout has no such module and no
//   such entry. A browser host adapter belongs behind "./host" if one lands.
export * from "./measure.js";
export * from "./trace-csv.js";
export * from "./operating-point.js";
export * from "./jobs.js";
export * from "./run-sweep.js";
export * from "./run-ac-sweep.js";
// Wave A5 follow-on: worker-hostable driver for the linearized small-signal AC
// analysis (the fast Bode path), with progress + cooperative-cancel hooks the
// bare runSmallSignalAc lacks. Composes it unchanged — see the file header.
export * from "./run-small-signal-ac-sweep.js";
export * from "./run-monte-carlo.js";
export * from "./job-messages.js";
export * from "./checked-step.js";
