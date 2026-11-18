/**
 * The decline census (test/core/equivalence/census.ts): over brownout's fixtures,
 * external corpora and directories of `.sim.json` boards, count circuits per
 * decline reason and per missing kind. Text on stdout, JSON on request.
 *
 *   pnpm run census:declines [--json [path]] [--no-fixtures] [--corpus <dir>]... [<boards-dir>...]
 *
 * `--corpus` is a manifest.json directory (BROWNOUT_CORE_CORPUS is read too, ':'-separated).
 * A bare argument is a directory of `*.sim.json` boards. `--json` alone prints the JSON
 * instead of the text; `--json <path>` writes it there and still prints the text.
 */
import fs from "node:fs";
import { initCore } from "../src/core/wasm.js";
import { renderCensus, runCensusOn } from "../test/core/equivalence/census.js";
import { parseCorpusDirsEnv } from "../test/core/equivalence/external-corpus.js";

const argv = process.argv.slice(2);
const corpusDirs = parseCorpusDirsEnv(process.env.BROWNOUT_CORE_CORPUS);
const boardDirs: string[] = [];
let fixtures = true;
let jsonMode: "none" | "stdout" | "file" = "none";
let jsonPath = "";
for (let i = 0; i < argv.length; i++) {
  const arg = argv[i]!;
  if (arg === "--corpus") {
    const dir = argv[++i];
    if (!dir) throw new Error("decline-census: --corpus needs a directory");
    corpusDirs.push(dir);
  } else if (arg === "--no-fixtures") fixtures = false;
  else if (arg === "--json") {
    const next = argv[i + 1];
    if (next !== undefined && !next.startsWith("--") && next.endsWith(".json")) {
      jsonMode = "file";
      jsonPath = next;
      i++;
    } else jsonMode = "stdout";
  } else if (arg.startsWith("--")) throw new Error(`decline-census: unknown flag ${arg}`);
  else boardDirs.push(arg);
}

await initCore();
const result = runCensusOn({ fixtures, corpusDirs, boardDirs });
if (jsonMode === "stdout") process.stdout.write(`${JSON.stringify(result, null, 2)}\n`);
else {
  process.stdout.write(`${renderCensus(result)}\n`);
  if (jsonMode === "file") fs.writeFileSync(jsonPath, `${JSON.stringify(result, null, 2)}\n`);
}
