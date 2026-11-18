/**
 * Writes test/fixtures/ngspice/<id>.json: the ngspice reference of every case in
 * test/core/equivalence/ngspice-deck.ts, from a live ngspice binary (NGSPICE_BIN,
 * PATH, or the usual install paths). Run it when a deck changes; the references are
 * committed so the core's comparison needs no ngspice at test time.
 *
 *   pnpm run ngspice:reference
 */
import fs from "node:fs";
import { NGSPICE_CASES, NGSPICE_REFERENCE_DIR, findNgspice, generateReference, referencePath } from "../test/core/equivalence/ngspice-deck.js";

const ngspice = findNgspice();
if (!ngspice) {
  console.error("ngspice-reference: no ngspice binary found (set NGSPICE_BIN).");
  process.exit(1);
}
fs.mkdirSync(NGSPICE_REFERENCE_DIR, { recursive: true });
for (const testCase of NGSPICE_CASES) {
  const reference = generateReference(testCase, ngspice);
  fs.writeFileSync(referencePath(testCase.id), `${JSON.stringify(reference)}\n`);
  console.log(`wrote ${referencePath(testCase.id)}`);
}
