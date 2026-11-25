#!/usr/bin/env node
// Lock-step check of the Rust iCE40 fabric (core/src/mcu/fpga.rs, feature
// `fpga`) against the reference Ice40 (src/sim/engine/fpga.ts). Each netlist
// is built by the reference, handed to the core as `image()`, and both then
// take the same seeded levels on their inputs while configuring, then the
// same seeded script of pin levels (0, 1, undriven) and advances of
// varying length, some under an edge budget and a few that are not a
// finite time, which both must refuse alike; after every action both sides' drive on every pin, edge
// count, RGB sink currents and state hash (FNV over every net's level) must agree, and so must
// the moment a design begins to oscillate (the run of that netlist ends
// there). The first disagreement prints the action, the pins that differ and
// both hashes.
//
// Usage:
//   node scripts/fpga-lockstep.mjs --build              # build the feature wasm
//   node scripts/fpga-lockstep.mjs [--netlist name|path|all] [--actions N] [--seed S]
//   node scripts/fpga-lockstep.mjs --bench [--netlist name] [--seconds T]
// The feature wasm goes to core/target/fpga-lockstep/, apart from the shipped
// build. `all` is every netlist in test/sim/engine/__fixtures__/fpga; a
// `name.pcf` beside a netlist places its ports, and the board's 12 MHz
// oscillator drives gpio_20 as on the UPduino with its OSC jumper fitted.
// The reference is bundled from src with esbuild on each run, so it is never
// stale.
//
// Not shipped: scripts/ is outside the package `files` list.

import { spawnSync } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { buildSync } from "esbuild";

const here = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(here, "..");
const coreDir = path.join(repoRoot, "core");
const targetDir = path.join(coreDir, "target", "fpga-lockstep");
const defaultWasm = path.join(targetDir, "wasm32-unknown-unknown", "release", "brownout_core.wasm");
const fixtureDir = path.join(repoRoot, "test/sim/engine/__fixtures__/fpga");
const argv = process.argv.slice(2);
const argOf = (name, fallback) => { const i = argv.indexOf(`--${name}`); return i >= 0 && i + 1 < argv.length ? argv[i + 1] : fallback; };
const flag = (name) => argv.includes(`--${name}`);
if (flag("help")) { console.log(fs.readFileSync(fileURLToPath(import.meta.url), "utf8").split("\n").slice(1, 20).join("\n")); process.exit(0); }
if (flag("build")) {
  const r = spawnSync("cargo", ["build", "--release", "--target", "wasm32-unknown-unknown", "--features", "fpga", "--target-dir", targetDir, "-j", process.env.CARGO_BUILD_JOBS || "3"], { cwd: coreDir, stdio: "inherit" });
  if (r.status !== 0) { console.error("fpga-lockstep: cargo build failed"); process.exit(1); }
  if (argv.length === 1) process.exit(0);
}
const opt = {
  wasm: argOf("wasm", defaultWasm), actions: Number(argOf("actions", 400)), seed: Number(argOf("seed", 1)),
  bench: flag("bench"), seconds: Number(argOf("seconds", 0.05)),
};
const netlists = (() => {
  const all = fs.readdirSync(fixtureDir).filter((f) => f.endsWith(".json")).sort().map((f) => path.join(fixtureDir, f));
  const sel = argOf("netlist", "all");
  if (sel === "all") return all;
  return sel.split(",").map((n) => all.find((f) => path.basename(f, ".json") === n) ?? n);
})();
if (!fs.existsSync(opt.wasm)) { console.error(`fpga-lockstep: no feature wasm at ${path.relative(repoRoot, opt.wasm)}; run with --build`); process.exit(2); }

// ---- the reference -------------------------------------------------------------
const bundle = buildSync({ entryPoints: [path.join(repoRoot, "src/sim/engine/fpga.ts")], bundle: true, format: "esm", platform: "node", write: false });
const { Ice40, FpgaError, UPDUINO_PINS } = await import("data:text/javascript;base64," + Buffer.from(bundle.outputFiles[0].text).toString("base64"));

// ---- the core -------------------------------------------------------------------
const wasm = new WebAssembly.Instance(new WebAssembly.Module(fs.readFileSync(opt.wasm)), { env: { bo_host_now_ms: () => 0 } }).exports;
for (const f of ["fpga_create", "fpga_set_pin", "fpga_preset_pin", "fpga_sink_current", "fpga_pin_drive", "fpga_advance", "fpga_state_hash", "fpga_edge_count", "fpga_alloc"]) {
  if (typeof wasm[f] !== "function") { console.error(`${opt.wasm} has no ${f}: build it with --build`); process.exit(2); }
}
function coreFabric(image) {
  const ib = image.ints.length * 4, fb = image.floats.length * 8;
  const ptr = wasm.fpga_alloc(ib + fb + 8);
  const fptr = ptr + Math.ceil(ib / 8) * 8;
  new Int32Array(wasm.memory.buffer, ptr, image.ints.length).set(image.ints);
  new Float64Array(wasm.memory.buffer, fptr, image.floats.length).set(image.floats);
  const f = wasm.fpga_create(ptr, image.ints.length, fptr, image.floats.length);
  if (!f) throw new Error("the core says the design does not settle");
  return f;
}

// ---- the run ----------------------------------------------------------------------
function rng(seed) {
  let s = seed >>> 0 || 1;
  return () => { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s / 2 ** 32; };
}
const OSC = { pin: 20, hz: 12e6 };

function build(file) {
  const netlist = JSON.parse(fs.readFileSync(file, "utf8"));
  const pcfFile = file.replace(/\.json$/, ".pcf");
  const pcf = fs.existsSync(pcfFile) ? fs.readFileSync(pcfFile, "utf8") : undefined;
  return new Ice40(netlist, { pcf, boardClocks: [OSC] });
}

const pinDriveOf = (v) => (v === 2 ? null : v);
let failed = 0;
for (const file of netlists) {
  const name = path.basename(file, ".json");
  const ref = build(file);
  const core = coreFabric(ref.image());
  const pins = ref.pins;
  const inputs = pins.filter((p) => p !== OSC.pin);
  if (opt.bench) {
    let t = performance.now();
    ref.advance(opt.seconds);
    const tRef = performance.now() - t;
    t = performance.now();
    wasm.fpga_advance(core, opt.seconds, Infinity);
    const tCore = performance.now() - t;
    console.log(`${name}: ${opt.seconds}s, ${ref.edgeCount} edges: reference ${tRef.toFixed(0)} ms, core ${tCore.toFixed(0)} ms (${(tRef / tCore).toFixed(1)}x)`);
    continue;
  }
  const rand = rng(opt.seed);
  const compare = (what) => {
    const diff = [];
    for (const p of pins) {
      const a = ref.pinDrive(p), b = pinDriveOf(wasm.fpga_pin_drive(core, p));
      if (a !== b) diff.push(`pin ${p}: reference ${a}, core ${b}`);
      const sa = ref.sinkCurrent(p), sb = wasm.fpga_sink_current(core, p);
      if (sa !== sb) diff.push(`pin ${p} sinks: reference ${sa} A, core ${sb} A`);
    }
    // A pin no design can use reads as undriven on both sides.
    for (const p of UPDUINO_PINS) if (!pins.includes(p) && wasm.fpga_pin_drive(core, p) !== 2) diff.push(`unused pin ${p} driven by the core`);
    const hr = ref.stateHash(), hc = wasm.fpga_state_hash(core) >>> 0;
    const er = ref.edgeCount, ec = wasm.fpga_edge_count(core);
    if (hr !== hc) diff.push(`state hash: reference ${hr.toString(16)}, core ${hc.toString(16)}`);
    if (er !== ec) diff.push(`edges: reference ${er}, core ${ec}`);
    if (diff.length) {
      console.log(`${name}: MISMATCH after ${what}\n  ${diff.join("\n  ")}`);
      return false;
    }
    return true;
  };
  // Runs one action on both sides: true when both settled, false when both
  // found the design oscillating, and a mismatch when only one did.
  const both = (refAction, coreAction) => {
    let refOsc = false;
    try {
      refAction();
    } catch (err) {
      if (!(err instanceof FpgaError)) throw err;
      refOsc = true;
    }
    const coreOsc = coreAction() !== 0;
    if (refOsc !== coreOsc) return `the ${refOsc ? "reference" : "core"} oscillated and the ${refOsc ? "core" : "reference"} did not`;
    return !refOsc;
  };
  // An advance on both sides: true when both ran (to the end or to the budget alike), false when both found the
  // design oscillating, and a mismatch otherwise. The core says 0 done, 1 oscillates, 2 bad time, 3 budget.
  const advanceBoth = (seconds, budget) => {
    let refOut;
    try {
      refOut = ref.advance(seconds, budget) ? 0 : 3;
    } catch (err) {
      if (err instanceof FpgaError) refOut = 1;
      else if (err instanceof RangeError) refOut = 2;
      else throw err;
    }
    const coreOut = wasm.fpga_advance(core, seconds, budget);
    if (refOut === 3) budgetStops++;
    if (refOut !== coreOut) return `advance returned ${refOut} on the reference and ${coreOut} on the core`;
    return refOut !== 1;
  };
  let budgetStops = 0;
  let ok = compare("power-up");
  // The levels the circuit holds while the device configures, as the board takes them.
  for (const pin of inputs) {
    if (!ok) break;
    const r = rand();
    const level = r < 0.4 ? 0 : r < 0.8 ? 1 : null;
    const ran = both(() => ref.presetPin(pin, level), () => wasm.fpga_preset_pin(core, pin, level ?? 2));
    if (ran !== true) {
      console.log(`${name}: MISMATCH presetting pin ${pin} = ${level}: ${ran || "both oscillated"}`);
      ok = false;
    } else ok = compare(`preset pin ${pin} = ${level}`);
  }
  let oscillated = null;
  for (let a = 0; ok && a < opt.actions; a++) {
    let what, ran;
    if (inputs.length && rand() < 0.5) {
      const pin = inputs[Math.floor(rand() * inputs.length)];
      const r = rand();
      const level = r < 0.4 ? 0 : r < 0.8 ? 1 : null;
      what = `action ${a}: pin ${pin} = ${level}`;
      ran = both(() => ref.setPin(pin, level), () => wasm.fpga_set_pin(core, pin, level ?? 2));
    } else {
      // Mostly short steps that land between edges, and some long ones that cross thousands; some stop at an edge
      // budget, and a few are not a time at all.
      const r = rand();
      const seconds = r < 0.02 ? [Number.NaN, Infinity, -1e-6][Math.floor(rand() * 3)] : r < 0.8 ? rand() * 2e-7 : rand() * 2e-4;
      const budget = rand() < 0.2 ? Math.floor(rand() * 50) : Infinity;
      what = `action ${a}: advance ${seconds}${budget === Infinity ? "" : ` within ${budget} edges`}`;
      ran = advanceBoth(seconds, budget);
    }
    if (typeof ran === "string") {
      console.log(`${name}: MISMATCH after ${what}\n  ${ran}`);
      ok = false;
    } else if (!ran) {
      oscillated = what;
      break;
    } else ok = compare(what);
  }
  if (ok && oscillated) console.log(`${name}: OK (both oscillate after ${oscillated}, ${ref.edgeCount} edges, ${pins.length} pins)`);
  else if (ok) console.log(`${name}: OK (${opt.actions} actions, ${ref.edgeCount} edges, ${budgetStops} stopped at a budget, ${pins.length} pins)`);
  else failed++;
  wasm.fpga_free(core);
}
process.exit(failed ? 1 : 0);
