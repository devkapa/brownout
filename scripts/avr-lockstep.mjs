#!/usr/bin/env node
// Lock-step check of the Rust AVR port (core/src/mcu/avr, feature `avr`)
// against avr8js (plan-1.0.0 section 4.3). The same hex image goes into both;
// both advance by the same instruction count; every K instructions a state
// record is compared (PC, SREG, SP, cycle count, r0-r31, a hash of the whole
// data space, the pending-interrupt state); on a mismatch the run is replayed
// from the start to the last agreeing checkpoint and stepped one instruction
// at a time to the first differing instruction, and both decodes and both
// states are printed.
//
// Usage (same shape as scripts/rp2040-lockstep.mjs):
//   node scripts/avr-lockstep.mjs --build              # build the feature wasm
//   node scripts/avr-lockstep.mjs [--image name|path|all] [--steps N] [--k K]
//        [--mode trap|plain|both] [--no-irq] [--no-decode-sweep]
//   node scripts/avr-lockstep.mjs --bench [--image name] [--steps N]
// The feature wasm goes to core/target/avr-lockstep/, apart from the shipped
// build, which this script never touches. `all` is every hex image in the
// repositories (brownout's test fixtures and devolt's apps/sim/lib/firmware);
// a name is matched against those, anything else is read as a path.
//
// Modes: `trap` stops both sides after the first access to a peripheral
// register (data address 0x20-0xff outside RAMPZ/EIND/SP/SREG), the expected
// end of a CPU-only run; `plain` runs on with the peripheral registers as
// plain memory, which is exactly what a bare avr8js CPU does. Inputs (the
// recorded script) are interrupt flags raised at fixed cycle counts on both
// sides; --no-irq drops them.
//
// Not shipped: scripts/ is outside the package `files` list.

import { spawnSync } from "node:child_process";
import fs from "node:fs";
import path from "node:path";
import { createRequire } from "node:module";
import { fileURLToPath } from "node:url";

const here = path.dirname(fileURLToPath(import.meta.url));
const require = createRequire(import.meta.url);

// ---- arguments ------------------------------------------------------------
const repoRoot = path.resolve(here, "..");
const coreDir = path.join(repoRoot, "core");
const targetDir = path.join(coreDir, "target", "avr-lockstep");
const defaultWasm = path.join(targetDir, "wasm32-unknown-unknown", "release", "brownout_core.wasm");
const DEVOLT_FIRMWARE = "/Users/dev/Documents/DEVOLTProject/devolt/apps/sim/lib/firmware";
const argv = process.argv.slice(2);
const argOf = (name, fallback) => { const i = argv.indexOf(`--${name}`); return i >= 0 && i + 1 < argv.length ? argv[i + 1] : fallback; };
const flag = (name) => argv.includes(`--${name}`);
if (flag("help")) { console.log(fs.readFileSync(fileURLToPath(import.meta.url), "utf8").split("\n").slice(1, 24).join("\n")); process.exit(0); }
if (flag("build")) {
  const r = spawnSync("cargo", ["build", "--release", "--target", "wasm32-unknown-unknown", "--features", "avr", "--target-dir", targetDir, "-j", process.env.CARGO_BUILD_JOBS || "3"], { cwd: coreDir, stdio: "inherit" });
  if (r.status !== 0) { console.error("avr-lockstep: cargo build failed"); process.exit(1); }
  if (argv.length === 1) process.exit(0);
}
function allImages() {
  const out = [];
  const add = (dir, tag) => { if (fs.existsSync(dir)) for (const f of fs.readdirSync(dir).sort()) if (f.endsWith(".hex")) out.push({ name: `${tag}/${f}`, file: path.join(dir, f) }); };
  add(path.join(repoRoot, "test/sim/engine/__fixtures__"), "brownout");
  add(DEVOLT_FIRMWARE, "devolt");
  return out;
}
const opt = {
  wasm: argOf("wasm", defaultWasm), k: Number(argOf("k", 256)), steps: Number(argOf("steps", 3_000_000)),
  irq: !flag("no-irq"), mode: argOf("mode", "both"), sweep: !flag("no-decode-sweep"),
  bench: flag("bench") ? Number(argOf("steps", 20_000_000)) : 0, images: [],
};
{
  const sel = argOf("image", "all"), all = allImages();
  if (sel === "all") opt.images = all.map((i) => i.file);
  else opt.images = sel.split(",").map((n) => all.find((i) => i.name === n || path.basename(i.file) === n || i.name.endsWith("/" + n))?.file ?? n);
}
if (!fs.existsSync(opt.wasm)) { console.error(`avr-lockstep: no feature wasm at ${path.relative(repoRoot, opt.wasm)}; run with --build`); process.exit(2); }

// ---- avr8js ------------------------------------------------------------------
function loadAvr8js() {
  const tries = [process.env.AVR8JS, "avr8js"].filter(Boolean);
  for (const t of tries) { try { return { mod: require(t), from: t }; } catch { /* next */ } }
  // devolt's pnpm copy (the version brownout's wrapper runs against)
  const p = "/Users/dev/Documents/DEVOLTProject/devolt/node_modules/.pnpm/avr8js@0.21.0/node_modules/avr8js";
  return { mod: createRequire(p + "/package.json")("avr8js"), from: p };
}
const { mod: avr, from: avrFrom } = loadAvr8js();
const avrDir = path.dirname(createRequire(import.meta.url).resolve(avrFrom, { paths: [here] }).replace(/\/dist\/.*$/, "/package.json"));
const avrVersion = JSON.parse(fs.readFileSync(path.join(avrDir, "package.json"), "utf8")).version;

// ---- Intel HEX, as brownout's parseIntelHex does it ----------------------------
function parseIntelHex(hex) {
  const flash = new Uint8Array(0x8000 * 2);
  let base = 0;
  for (const raw of hex.split(/\r?\n/)) {
    const line = raw.trim();
    if (!line.startsWith(":")) continue;
    const bytes = new Uint8Array((line.length - 1) / 2);
    for (let i = 0; i < bytes.length; i++) bytes[i] = parseInt(line.slice(1 + i * 2, 3 + i * 2), 16);
    const n = bytes[0], addr = (bytes[1] << 8) | bytes[2], type = bytes[3];
    if (type === 0) { for (let i = 0; i < n; i++) if (base + addr + i < flash.length) flash[base + addr + i] = bytes[4 + i]; }
    else if (type === 2) base = ((bytes[4] << 8) | bytes[5]) << 4;
    else if (type === 4) base = ((bytes[4] << 8) | bytes[5]) << 16;
  }
  return flash;
}

// ---- the JS decode: the reference's own if/else chain, evaluated ---------------
function buildJsDecoder() {
  const file = path.join(avrDir, "dist/esm/cpu/instruction.js");
  const src = fs.readFileSync(file, "utf8");
  // top-level branches only: one level of indentation inside avrInstruction
  const re = /\n    (?:if|else if) \(((?:(?!\n {4}\S)[\s\S])*?)\) \{\n {8}\/\* (\S+?),/g;
  const chain = [];
  let m;
  while ((m = re.exec(src))) chain.push([new Function("opcode", "return " + m[1]), m[2]]);
  return (op) => { for (const [f, name] of chain) if (f(op)) return name; return "?"; };
}
const jsDecode = buildJsDecoder();

// ---- the wasm ----------------------------------------------------------------------
const wasmBytes = fs.readFileSync(opt.wasm);
const wasm = new WebAssembly.Instance(new WebAssembly.Module(wasmBytes), { env: { bo_host_now_ms: () => 0 } }).exports;
for (const f of ["avr_create", "avr_run", "avr_state", "avr_decode_name"]) {
  if (typeof wasm[f] !== "function") { console.error(`${opt.wasm} has no ${f}: build it with --features avr`); process.exit(2); }
}
const STATE_WORDS = wasm.avr_state_words();
const rustDecode = (op) => {
  const ptr = wasm.avr_alloc(32);
  const n = wasm.avr_decode_name(op, ptr, 32);
  const s = Buffer.from(new Uint8Array(wasm.memory.buffer, ptr, n)).toString();
  wasm.avr_free(ptr, 32);
  return s;
};

// ---- the recorded input script: interrupt flags at fixed cycle counts ---------------
// TIMER0_OVF (vector 16, word 0x20, TIFR0 0x35 bit 0, TIMSK0 0x6e bit 0) every
// 1024 cycles from cycle 2000 and TIMER2_OVF (vector 9, word 0x12, TIFR2 0x37,
// TIMSK2 0x70) every 6000 from 3500: what the two overflow timers would raise,
// so the sketches' millis ISR runs and two sources are sometimes pending at once.
const IRQ_SOURCES = [
  { address: 0x20, flagRegister: 0x35, flagMask: 1, enableRegister: 0x6e, enableMask: 1, constant: false, inverseFlag: false, first: 2000, every: 1024 },
  { address: 0x12, flagRegister: 0x37, flagMask: 1, enableRegister: 0x70, enableMask: 1, constant: false, inverseFlag: false, first: 3500, every: 6000 },
];
// Lazily generated, in cycle order, identical on both sides.
class IrqScript {
  constructor() { this.next = IRQ_SOURCES.map((s) => s.first); }
  due(cycles) {
    const out = [];
    for (;;) {
      let best = -1;
      for (let i = 0; i < this.next.length; i++) if (this.next[i] <= cycles && (best < 0 || this.next[i] < this.next[best])) best = i;
      if (best < 0) return out;
      out.push(IRQ_SOURCES[best]);
      this.next[best] += IRQ_SOURCES[best].every;
    }
  }
}

function fnv1a(bytes) {
  let h = 0x811c9dc5;
  for (let i = 0; i < bytes.length; i++) { h ^= bytes[i]; h = Math.imul(h, 0x01000193); }
  return h >>> 0;
}

// ---- the two machines ----------------------------------------------------------------
class JsSide {
  constructor(flash, trap, irq) {
    this.cpu = new avr.CPU(new Uint16Array(flash.buffer.slice(0)));
    this.trap = trap; this.hit = null; this.steps = 0; this.wd = 0; this.irqTaken = 0;
    this.script = irq ? new IrqScript() : null;
    this.cpu.onWatchdogReset = () => { this.wd++; };
    if (trap) {
      for (let a = 0x20; a < 0x100; a++) {
        if (a >= 0x5b && a <= 0x5f) continue;
        this.cpu.readHooks[a] = (addr) => { if (!this.hit) this.hit = { address: addr, write: false }; return this.cpu.data[addr]; };
        this.cpu.writeHooks[a] = (_v, _old, addr) => { if (!this.hit) this.hit = { address: addr, write: true }; return false; };
      }
    }
  }
  run(n) {
    const cpu = this.cpu, instr = avr.avrInstruction;
    let done = 0;
    while (done < n && !this.hit) {
      if (this.script) for (const s of this.script.due(cpu.cycles)) cpu.setInterruptFlag(s);
      instr(cpu);
      const c1 = cpu.cycles;
      cpu.tick();
      if (cpu.cycles - c1 === 2) this.irqTaken++; // avrInterrupt costs exactly two cycles
      done++; this.steps++;
    }
    return done;
  }
  state() {
    const c = this.cpu, s = new Float64Array(STATE_WORDS);
    s[0] = c.pc; s[1] = c.data[95]; s[2] = c.SP; s[3] = c.cycles; s[4] = c.nextInterrupt;
    s[5] = this.hit ? this.hit.address : -1; s[6] = this.hit && this.hit.write ? 1 : 0;
    s[7] = fnv1a(c.data);
    let n = 0; for (let e = c.nextClockEvent; e; e = e.next) n++;
    s[8] = n; s[9] = this.wd;
    for (let i = 0; i < 32; i++) s[10 + i] = c.data[i];
    return s;
  }
  data() { return this.cpu.data; }
}

class RustSide {
  constructor(flash, trap, irq) {
    const ptr = wasm.avr_alloc(flash.length);
    new Uint8Array(wasm.memory.buffer, ptr, flash.length).set(flash);
    this.m = wasm.avr_create(ptr, flash.length);
    wasm.avr_free(ptr, flash.length);
    wasm.avr_set_io_trap(this.m, trap ? 1 : 0);
    this.script = irq ? new IrqScript() : null;
    this.steps = 0;
    this.out = wasm.avr_alloc(STATE_WORDS * 8);
    this.irqOn = irq;
    // The Rust machine applies scheduled interrupts itself, so the whole script
    // up to a horizon is scheduled up front; see run().
    this.horizon = 0;
  }
  schedule(upToCycle) {
    if (!this.irqOn) return;
    for (let i = 0; i < IRQ_SOURCES.length; i++) {
      const s = IRQ_SOURCES[i];
      this.next ??= IRQ_SOURCES.map((x) => x.first);
      while (this.next[i] <= upToCycle) {
        wasm.avr_schedule_irq(this.m, this.next[i], s.address, s.flagRegister, s.flagMask, s.enableRegister, s.enableMask, s.constant ? 1 : 0, s.inverseFlag ? 1 : 0);
        this.next[i] += s.every;
      }
    }
  }
  run(n) {
    // An instruction takes at most 5 cycles, so scheduling 6 cycles per
    // instruction ahead of the current count always covers the run; the Rust
    // side applies each input before the first instruction at or after it, and
    // a source that falls due in the same cycle order is applied in the order
    // scheduled (due-time order, as IrqScript yields them).
    this.schedule(this.cycles() + 6 * n + 8);
    const done = wasm.avr_run(this.m, n);
    this.steps += done;
    return done;
  }
  cycles() { wasm.avr_state(this.m, this.out); return new Float64Array(wasm.memory.buffer, this.out, STATE_WORDS)[3]; }
  state() { wasm.avr_state(this.m, this.out); return Float64Array.from(new Float64Array(wasm.memory.buffer, this.out, STATE_WORDS)); }
  data() { return new Uint8Array(wasm.memory.buffer, wasm.avr_data_ptr(this.m), wasm.avr_data_len(this.m)); }
  free() { wasm.avr_destroy(this.m); wasm.avr_free(this.out, STATE_WORDS * 8); }
}

const FIELDS = ["pc", "sreg", "sp", "cycles", "nextInterrupt", "ioAddr", "ioWrite", "dataHash", "clockEvents", "wdr", ...Array.from({ length: 32 }, (_, i) => `r${i}`)];
function diffStates(a, b) {
  const d = [];
  for (let i = 0; i < STATE_WORDS; i++) if (a[i] !== b[i]) d.push(`${FIELDS[i]}: js=${a[i]} rust=${b[i]}`);
  return d;
}
const hex = (n, w = 4) => "0x" + (n >>> 0).toString(16).padStart(w, "0");

function make(flash, trap) { return [new JsSide(flash, trap, opt.irq), new RustSide(flash, trap, opt.irq)]; }

// Replays both sides from a fresh start to `upTo`, then steps single
// instructions to the first difference.
function bisect(flash, trap, upTo, k) {
  const [js, rs] = make(flash, trap);
  js.run(upTo); rs.run(upTo);
  for (let i = 0; i < k + 1; i++) {
    const preJs = js.state(), preRs = rs.state();
    const pcBefore = preJs[0];
    const word = new Uint16Array(flash.buffer.slice(0))[pcBefore] ?? 0;
    js.run(1); rs.run(1);
    const sj = js.state(), sr = rs.state();
    const d = diffStates(sj, sr);
    if (d.length) {
      console.log(`  first divergence at instruction ${upTo + i + 1} (pc ${hex(pcBefore)}, opcode ${hex(word)})`);
      console.log(`  decode: avr8js=${jsDecode(word)}  rust=${rustDecode(word)}`);
      console.log(`  state before: ${diffStates(preJs, preRs).length ? "ALREADY DIFFERED " : "equal: "}pc=${hex(pcBefore)} sreg=${hex(preJs[1], 2)} sp=${hex(preJs[2])} cycles=${preJs[3]}`);
      for (const line of d) console.log("    " + line);
      const dj = js.data(), dr = rs.data();
      let shown = 0;
      for (let a = 0; a < Math.min(dj.length, dr.length) && shown < 12; a++) if (dj[a] !== dr[a]) { console.log(`    data[${hex(a)}]: js=${dj[a]} rust=${dr[a]}`); shown++; }
      rs.free();
      return;
    }
    if (js.hit) break;
  }
  console.log("  (bisect did not reproduce the divergence: nondeterminism?)");
  rs.free();
}

function runImage(file, flash, trap) {
  const label = `${path.basename(file)} [${trap ? "trap" : "plain"}${opt.irq ? "+irq" : ""}]`;
  const [js, rs] = make(flash, trap);
  const target = trap ? 5_000_000 : opt.steps;
  let done = 0, ok = true, checks = 0;
  const t0 = Date.now();
  while (done < target) {
    const n = Math.min(opt.k, target - done);
    const a = js.run(n), b = rs.run(n);
    done += a;
    checks++;
    const d = a !== b ? [`instructions run: js=${a} rust=${b}`] : diffStates(js.state(), rs.state());
    if (d.length) {
      ok = false;
      console.log(`DIVERGED ${label} within instructions ${done - a + 1}..${done}`);
      for (const line of d.slice(0, 8)) console.log("    " + line);
      bisect(flash, trap, done - a, a);
      break;
    }
    if (js.hit) break;
  }
  const s = js.state();
  const ms = Date.now() - t0;
  if (ok) {
    const stop = js.hit
      ? `stopped after instruction ${done}: first ${js.hit.write ? "write to" : "read of"} peripheral register ${hex(js.hit.address, 2)} at pc ${hex(s[0])} (same on both sides)`
      : `ran ${done} instructions, ${s[3]} cycles, ${js.irqTaken} interrupts taken, to pc ${hex(s[0])}`;
    console.log(`OK       ${label}: ${stop}; ${checks} checkpoints (K=${opt.k}), ${ms} ms`);
  }
  rs.free();
  return { ok, done, hit: js.hit, pc: s[0], cycles: s[3] };
}

// ---- the decode sweep: all 65536 opcode words -------------------------------------------
let failed = false;
console.log(`avr8js ${avrVersion} (${avrDir}); wasm ${opt.wasm} (${wasmBytes.length} bytes)`);
if (opt.sweep) {
  let bad = 0;
  for (let op = 0; op < 0x10000; op++) {
    const j = jsDecode(op), r = rustDecode(op);
    if (j !== r) { if (bad++ < 10) console.log(`  decode mismatch ${hex(op)}: avr8js=${j} rust=${r}`); }
  }
  console.log(bad ? `DECODE DIVERGED on ${bad} of 65536 opcode words` : "OK       decode: all 65536 opcode words name the same instruction on both sides");
  if (bad) failed = true;
}
if (opt.bench) {
  // Instructions per second, each side alone, on the first image (plain memory, no interrupts).
  const flash = parseIntelHex(fs.readFileSync(opt.images[0], "utf8"));
  const js = new JsSide(flash, false, false), rs = new RustSide(flash, false, false);
  let t = process.hrtime.bigint(); js.run(opt.bench);
  const tj = Number(process.hrtime.bigint() - t) / 1e9;
  t = process.hrtime.bigint(); rs.run(opt.bench);
  const tr = Number(process.hrtime.bigint() - t) / 1e9;
  console.log(`bench ${path.basename(opt.images[0])}: ${opt.bench} instructions; avr8js ${(opt.bench / tj / 1e6).toFixed(1)} M/s, rust wasm ${(opt.bench / tr / 1e6).toFixed(1)} M/s`);
}
for (const file of opt.bench ? [] : opt.images) {
  const flash = parseIntelHex(fs.readFileSync(file, "utf8"));
  if (opt.mode === "trap" || opt.mode === "both") { if (!runImage(file, flash, true).ok) failed = true; }
  if (opt.mode === "plain" || opt.mode === "both") { if (!runImage(file, flash, false).ok) failed = true; }
}
process.exit(failed ? 1 : 0);
