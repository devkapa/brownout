/**
 * Example: a SPICE netlist in, a Bode table out.
 *
 * An RC low-pass with its corner placed exactly at 1 kHz
 * (fc = 1/(2*pi*R*C), R = 1 kOhm, C = 159.155 nF), swept with `.ac dec`.
 * A single-pole response is the one filter whose Bode plot everyone knows by
 * heart — -3.01 dB and -45 deg at the corner, then -20 dB/decade — so the
 * printed table can carry the closed form beside the solver's answer and be
 * checked at a glance.
 *
 * Shows: runSpice() executing directives as written, the `AC 1` input
 * designation, and what comes back on SpiceAcResult.
 *
 * This is the OTHER way to drive the engine. brownout/host's HeadlessRunner
 * hands you the step loop; runSpice keeps SPICE's own semantics — the deck
 * declares the analysis and the runner owns the grid. Reach for this when you
 * already have a netlist, or when you want ngspice-comparable output.
 *
 * Run with: pnpm run example:bode   (needs pnpm run build first)
 */

import { runSpice } from "brownout/spice";

const R_OHM = 1000;
const C_FARAD = 159.155e-9;
const FC_HZ = 1 / (2 * Math.PI * R_OHM * C_FARAD); // 1000.0 Hz by construction

// `AC 1` designates the small-signal input: exactly one element per netlist
// may carry it, and .ac requires one. The DC value is 0 because the small-
// signal system is linearized about the operating point and driven separately.
const NETLIST = `RC low-pass — 1 kHz corner
V1 in 0 DC 0 AC 1
R1 in out 1k
C1 out 0 159.155n
.ac dec 4 10 100k
.end
`;

const result = runSpice(NETLIST);

// Warnings carry every disclosed approximation and every ignored parameter.
// An example that hid them would be teaching the wrong habit.
if (result.warnings.length > 0) {
  console.log("warnings:");
  for (const warning of result.warnings) console.log(`  - ${warning}`);
  console.log();
}

const ac = result.ac;
if (!ac) throw new Error("netlist declared no .ac analysis");

const out = ac.nodeResponses.out;
if (!out) throw new Error('no response for node "out"');

console.log(`${result.title}`);
console.log(`.ac dec 4 10 100k — ${ac.frequenciesHz.length} points, input ${ac.inputId} at AC ${ac.inputMagnitude}`);
console.log(`R=${R_OHM} Ohm, C=${C_FARAD * 1e9} nF  ->  fc = 1/(2*pi*R*C) = ${FC_HZ.toFixed(3)} Hz\n`);

// Nearest sweep point to the corner, not an exact match: 159.155 nF is a
// rounded catalog value, so fc lands at 999.99963 Hz while the dec lattice
// puts a point at exactly 1000 Hz.
const cornerIndex = ac.frequenciesHz.reduce(
  (best, f, i) => (Math.abs(f - FC_HZ) < Math.abs(ac.frequenciesHz[best]! - FC_HZ) ? i : best),
  0,
);

console.log("|      f (Hz) |   |H| (dB) |  exact (dB) |  phase (deg) | exact (deg) |");
console.log("| ----------: | ---------: | ----------: | -----------: | ----------: |");
for (let i = 0; i < ac.frequenciesHz.length; i++) {
  const f = ac.frequenciesHz[i]!;
  const ratio = f / FC_HZ;
  // Single pole: H(jw) = 1 / (1 + j*f/fc).
  const exactDb = -10 * Math.log10(1 + ratio ** 2);
  const exactDeg = (-Math.atan(ratio) * 180) / Math.PI;
  const corner = i === cornerIndex ? "  <- corner" : "";
  console.log(
    `| ${f.toFixed(2).padStart(11)} `
    + `| ${out.magnitudeDb[i]!.toFixed(4).padStart(10)} `
    + `| ${exactDb.toFixed(4).padStart(11)} `
    + `| ${out.phaseDeg[i]!.toFixed(3).padStart(12)} `
    + `| ${exactDeg.toFixed(3).padStart(11)} |${corner}`,
  );
}

// Worst deviation from the closed form across the whole sweep — the single
// number that says whether the sweep is trustworthy.
let worstDb = 0;
let worstDeg = 0;
for (let i = 0; i < ac.frequenciesHz.length; i++) {
  const ratio = ac.frequenciesHz[i]! / FC_HZ;
  worstDb = Math.max(worstDb, Math.abs(out.magnitudeDb[i]! - -10 * Math.log10(1 + ratio ** 2)));
  worstDeg = Math.max(worstDeg, Math.abs(out.phaseDeg[i]! - (-Math.atan(ratio) * 180) / Math.PI));
}

console.log(`
At the corner (${ac.frequenciesHz[cornerIndex]!.toFixed(1)} Hz): ${out.magnitudeDb[cornerIndex]!.toFixed(4)} dB, \
${out.phaseDeg[cornerIndex]!.toFixed(3)} deg   (textbook: -3.0103 dB, -45.000 deg)
Rolloff 10 kHz -> 100 kHz: \
${(out.magnitudeDb[ac.frequenciesHz.length - 1]! - out.magnitudeDb[ac.frequenciesHz.length - 5]!).toFixed(3)} dB/decade \
(textbook: -20 dB/decade)
Worst deviation across the sweep: ${worstDb.toExponential(2)} dB, ${worstDeg.toExponential(2)} deg`);
