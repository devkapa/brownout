/**
 * Offline analysis entry: the pure, Node-safe runners (DC/temperature sweep,
 * driven AC sweep, Monte Carlo, operating point), the job/message types they
 * consume, trace measurement, CSV export, and the checked-step transient
 * harness.
 *
 * The analysis/index.js barrel already curates this set. What is deliberately
 * absent: simcore's analysis-worker-client (it constructs a browser Worker —
 * that transport shell is app-domain and was not extracted; a browser host
 * adapter arrives with the "./host" entry in phase B3).
 */

export * from "./analysis/index.js";
