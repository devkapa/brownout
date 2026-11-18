/** `pnpm run classification:totals`: the per-package table of gate 1's classification (test/core/suites/classification-totals.ts). */
import { computeTotals, readClassification, renderTotals } from "../test/core/suites/classification-totals.js";

const classification = readClassification();
const totals = computeTotals(classification);
if (process.argv.includes("--json")) process.stdout.write(`${JSON.stringify(totals, null, 2)}\n`);
else process.stdout.write(`${renderTotals(totals)}\n`);
