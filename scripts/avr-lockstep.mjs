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
//        [--mode cpu|trap|full|serial|both] [--no-irq] [--no-decode-sweep]
//   node scripts/avr-lockstep.mjs --bench [--image name] [--steps N]
// The feature wasm goes to core/target/avr-lockstep/, apart from the shipped
// build, which this script never touches. `all` is every hex image in the
// repositories (brownout's test fixtures and devolt's apps/sim/lib/firmware);
// a name is matched against those, anything else is read as a path.
//
// Modes (`both` runs all four): `cpu` is a bare CPU that stops both sides
// after the first access to a peripheral register (data address 0x20-0xff
// outside RAMPZ/EIND/SP/SREG); `trap` adds the peripherals ArduinoMcu builds
// and stops at the first register none of them models; `full` runs on with
// such registers as plain memory; `serial` is `full` with typed serial bytes
// as its only input, so each sketch runs as written (the other inputs enable
// interrupts most sketches have no handler for, which restarts them) and its
// own Serial, Wire, SPI and EEPROM use is compared. Outside cpu mode the
// avr8js side is the engine's own ArduinoMcu (dist/sim/engine/arduino.js,
// rebuilt when src is newer), and each checkpoint also compares the serial,
// I2C and SPI event streams. Inputs (the recorded script below) are
// interrupt flags in cpu mode, and pin levels, ADC volts, register writes and
// typed serial bytes otherwise, applied at fixed cycle counts on both sides;
// --no-irq drops them.
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
const DEVOLT_FIRMWARE = process.env.DEVOLT_FIRMWARE || "/Users/dev/Documents/DEVOLTProject/devolt/apps/sim/lib/firmware";
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
// The engine's ArduinoMcu, from the built package, so the harness runs the engine's own peripherals (and its
// departures from avr8js, src/sim/engine/avr-bus-timing.ts) rather than a copy of how it builds them. Rebuilt first
// when any source file is newer than the build, so a stale dist can never stand in for the code under test.
const arduinoJs = path.join(repoRoot, "dist/sim/engine/arduino.js");
{
  // Every src/**/*.ts whose dist/**/*.js is older than it (tsc rewrites only the outputs that changed, so each file is
  // checked against its own output).
  const stale = (dir = path.join(repoRoot, "src")) => {
    const out = [];
    for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
      const f = path.join(dir, e.name);
      if (e.isDirectory()) out.push(...stale(f));
      else if (e.name.endsWith(".ts") && !e.name.endsWith(".d.ts")) {
        const js = path.join(repoRoot, "dist", path.relative(path.join(repoRoot, "src"), f)).replace(/\.ts$/, ".js");
        if (fs.existsSync(js) && fs.statSync(js).mtimeMs < fs.statSync(f).mtimeMs) out.push(path.relative(repoRoot, f));
      }
    }
    return out;
  };
  if (!fs.existsSync(arduinoJs) || stale().length > 0) {
    // tsc decides what to re-emit from file contents, so a source touched but unchanged keeps its older output: a
    // clean build is the check, not the timestamps afterwards.
    console.log("avr-lockstep: dist is older than src; building it (npx tsc -p tsconfig.build.json)");
    const r = spawnSync("npx", ["tsc", "-p", "tsconfig.build.json"], { cwd: repoRoot, stdio: "inherit" });
    if (r.status !== 0 || !fs.existsSync(arduinoJs)) {
      console.error("avr-lockstep: the build failed (above); fix it, or delete dist and .tsbuildinfo, then rerun");
      process.exit(2);
    }
  }
}
const engine = await import(arduinoJs);
const avrDir = path.dirname(createRequire(import.meta.url).resolve(avrFrom, { paths: [here] }).replace(/\/dist\/.*$/, "/package.json"));
const avrVersion = JSON.parse(fs.readFileSync(path.join(avrDir, "package.json"), "utf8")).version;
engine.setAvr8Module(avr);

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

// ---- the recorded input scripts -----------------------------------------------------
// Both sides consume the same sorted event list, each event applied before the
// first instruction that starts at or after its cycle.
//  cpu mode (no peripherals): interrupt flags, as before: TIMER0_OVF (vector
//   word 0x20, TIFR0 0x35 bit 0, TIMSK0 0x6e bit 0) every 1024 cycles from
//   cycle 2000 and TIMER2_OVF (0x12, TIFR2 0x37, TIMSK2 0x70) every 6000 from 3500.
//  peripheral modes: pin levels (INT0 on PD2, PB0, PC3, INT1/OC2B on PD3, the
//   timer 0 clock pin PD4), ADC channel volts, and register writes that put
//   the timers and interrupt controllers through modes no sketch reaches
//   (phase-correct and fast PWM, CTC, toggle outputs, force compare, INT0/INT1
//   and pin-change enables, ADC conversions, USART framing and baud changes
//   with the receive interrupt on, SPI transfers and write collisions, a TWI
//   start/address/stop, EEPROM writes (EEMPE and EEPE in one write, so each
//   starts whatever the instruction timing), a write while one is in progress,
//   a write-only one and a read), and bytes typed into the serial port, so the
//   lock-step covers more of the peripherals than the sketches do.
const IRQ_SOURCES = [
  { kind: "irq", address: 0x20, flagRegister: 0x35, flagMask: 1, enableRegister: 0x6e, enableMask: 1, first: 2000, every: 1024 },
  { kind: "irq", address: 0x12, flagRegister: 0x37, flagMask: 1, enableRegister: 0x70, enableMask: 1, first: 3500, every: 6000 },
];
const PERIPH_SOURCES = [
  { kind: "pin", port: 2, pin: 2, first: 3000, every: 4001 },
  { kind: "pin", port: 0, pin: 0, first: 5000, every: 7919 },
  { kind: "pin", port: 1, pin: 3, first: 6100, every: 9973 },
  { kind: "pin", port: 2, pin: 3, first: 9000, every: 12011 },
  { kind: "pin", port: 2, pin: 4, first: 2500, every: 997 },
  { kind: "adc", channel: 0, first: 20000, every: 30011 },
  { kind: "adc", channel: 1, first: 25000, every: 41011 },
  { kind: "adc", channel: 3, first: 31000, every: 50021 },
  { kind: "serial", first: 1_650_000, every: 2111 },
];
// `serial` mode: a byte every 1361 cycles from cycle 400000 (after setup()), one cycle over a 115200-baud frame,
// so a sketch at 115200 takes them as they come and one at 9600 builds a backlog for the feed to work through.
const SERIAL_SOURCES = [{ kind: "serial", first: 400_000, every: 1361 }];
// [cycle, address, value]: written in this order, each once.
const POKES = [
  [50_000, 0x69, 0x07], [50_100, 0x3d, 0x03], [60_000, 0x68, 0x07], [60_100, 0x6d, 0xff], [60_200, 0x6b, 0xff], [60_300, 0x6c, 0x0f],
  [100_000, 0xb0, 0x21], [100_050, 0xb4, 0x40], [100_100, 0xb1, 0x03], [100_200, 0x70, 0x07],
  [200_000, 0x80, 0x81], [200_050, 0x89, 0x00], [200_060, 0x88, 0x80], [200_100, 0x81, 0x01], [200_200, 0x6f, 0x07],
  [300_000, 0x44, 0x42], [300_050, 0x47, 0x7f], [300_100, 0x45, 0x03], [300_200, 0x6e, 0x07],
  [400_000, 0x80, 0x00], [400_050, 0x81, 0x0b], [400_100, 0x89, 0x10], [400_110, 0x88, 0x00], [400_200, 0x6f, 0x02],
  [500_000, 0x7c, 0x40], [500_100, 0x7a, 0xcf], [520_000, 0x7a, 0xc7], [540_000, 0x7c, 0xc3], [540_100, 0x7a, 0xc7],
  [600_000, 0x44, 0xa3], [600_050, 0x45, 0x01], [600_100, 0x47, 0x40], [600_110, 0x48, 0x90], [600_200, 0x81, 0x1a],
  [700_000, 0x44, 0x10], [700_050, 0x45, 0x41], [700_100, 0x45, 0x05],
  [800_000, 0x80, 0x23], [800_010, 0x81, 0x19], [800_020, 0x87, 0x01], [800_030, 0x86, 0x00], [800_040, 0x89, 0x00], [800_050, 0x88, 0x40],
  [900_000, 0xb0, 0x13], [900_050, 0xb1, 0x0d], [900_100, 0xb3, 0x80], [900_200, 0xb4, 0x20],
  [1_000_000, 0x45, 0x06], [1_000_100, 0x45, 0x07], [1_100_000, 0x81, 0x06], [1_100_100, 0x81, 0x01],
  [1_200_000, 0x84, 0x10], [1_200_010, 0x85, 0x01], [1_300_000, 0x46, 0xfa], [1_400_000, 0xb2, 0xf0],
  [1_500_000, 0x7a, 0x00], [1_500_100, 0x7a, 0x87], [1_500_200, 0x7a, 0xc7],
  [1_600_000, 0xc4, 0x10], [1_600_010, 0xc0, 0x02], [1_600_020, 0xc1, 0x98], [1_600_100, 0xc6, 0x41], [1_600_110, 0xc6, 0x42],
  [1_600_400, 0xc0, 0x40], [1_600_500, 0xc2, 0x0e], [1_600_600, 0xc2, 0x06],
  [1_700_000, 0x4c, 0x50], [1_700_010, 0x4e, 0xa5], [1_700_012, 0x4e, 0x5a], [1_700_200, 0x4d, 0x81], [1_700_210, 0x4d, 0x01],
  [1_700_300, 0x4c, 0x53], [1_700_310, 0x4e, 0x3c],
  [1_800_000, 0xb8, 0x48], [1_800_010, 0xbc, 0xa4], [1_800_100, 0xbb, 0x50], [1_800_110, 0xbc, 0x84], [1_800_200, 0xbc, 0x94], [1_800_300, 0xbc, 0x04],
  [1_900_000, 0x40, 0x3c], [1_900_005, 0x41, 0x07], [1_900_010, 0x42, 0x00], [1_900_020, 0x3f, 0x06], [1_900_100, 0x3f, 0x06],
  [1_950_000, 0x3f, 0x01], [1_960_000, 0x3f, 0x02], [1_970_000, 0x3f, 0x26],
  [2_000_000, 0x3d, 0x00], [2_000_100, 0x69, 0x00], [2_000_200, 0x3d, 0x03],
];
// Lazily generated in cycle order (ties: sources in listed order, then pokes).
class InputScript {
  constructor(mode) {
    this.sources = mode === "cpu" ? IRQ_SOURCES : mode === "serial" ? SERIAL_SOURCES : PERIPH_SOURCES;
    this.next = this.sources.map((s) => s.first);
    this.k = this.sources.map(() => 0);
    this.pokes = mode === "cpu" || mode === "serial" ? [] : POKES.slice();
  }
  due(cycles) {
    const out = [];
    for (;;) {
      let best = -1, bestAt = Infinity;
      for (let i = 0; i < this.next.length; i++) if (this.next[i] <= cycles && this.next[i] < bestAt) { best = i; bestAt = this.next[i]; }
      const poke = this.pokes.length && this.pokes[0][0] <= cycles && this.pokes[0][0] < bestAt ? this.pokes[0] : null;
      if (poke) { this.pokes.shift(); out.push({ at: poke[0], kind: "write", addr: poke[1], value: poke[2] }); continue; }
      if (best < 0) return out;
      const s = this.sources[best], k = this.k[best]++;
      this.next[best] += s.every;
      if (s.kind === "pin") out.push({ at: bestAt, kind: "pin", port: s.port, pin: s.pin, level: k % 2 === 0 });
      else if (s.kind === "serial") out.push({ at: bestAt, kind: "serial", byte: (k * 37 + 65) & 0x7f });
      else if (s.kind === "adc") out.push({ at: bestAt, kind: "adc", channel: s.channel, volts: Math.min(5, Math.max(0, ((k * 0.737 + s.channel) % 5.4) )) });
      else out.push({ at: bestAt, kind: "irq", ...s });
    }
  }
}

function fnv1a(bytes) {
  let h = 0x811c9dc5;
  for (let i = 0; i < bytes.length; i++) { h ^= bytes[i]; h = Math.imul(h, 0x01000193); }
  return h >>> 0;
}

// ---- the two machines ----------------------------------------------------------------
// `cpu`: a bare CPU, as in AVR1. `trap` / `full`: `new ArduinoMcu(hex)` itself (ports B, C, D, timers 0-2, the ADC,
// USART0, TWI, SPI and the EEPROM, its serial feed and bus log included); `trap` stops after the first access to a
// register no ported peripheral models, `full` runs on with those as plain memory.
const FIELDS = ["pc", "sreg", "sp", "cycles", "nextInterrupt", "ioAddr", "ioWrite", "dataHash", "clockEvents", "wdr",
  ...Array.from({ length: 32 }, (_, i) => `r${i}`), "pend0", "pend1", "pend2", "pend3", "clockEventHash",
  ...[0, 1, 2].flatMap((t) => ["tcnt", "lastCycle", "ocrA", "ocrB", "icr", "divider", "flags", "nextOcrA", "nextOcrB", "tcntNext"].map((n) => `t${t}.${n}`)),
  ...[0, 1, 2].flatMap((p) => ["pinValue", "lastValue", "lastDdr", "lastPin", "overrideMask", "overrideValue"].map((n) => `port${"BCD"[p]}.${n}`)),
  "adc.converting", "adc.conversionCycles", "events", "commsBusy", "usart.rxByte", "usart.txCount", "usart.txHash",
  "serialQueued", "eeprom.writeEnabledCycles", "eeprom.writeCompleteCycles", "eeprom.hash",
  "usart.txShifting", "usart.txPending", "twi.steps", "twi.stepHash", "usart.rxHeld", "usart.rxOverrun"];
// Bus events as both sides encode them (the core's avr_drain_bus): a code (UART sent 0, received 1; I2C start 2,
// restart 3, stop 4, address 5, write 6, read 7; SPI 8), start cycle, length, then the event's own fields.
const I2C_CODES = { start: 2, restart: 3, stop: 4, address: 5, write: 6, read: 7 };
function encodeBusEvent(e) {
  if (e.bus === "uart") return [e.dir === "tx" ? 0 : 1, e.cycle, e.cycles, e.value, 0, 0];
  if (e.bus === "spi") return [8, e.cycle, e.cycles, e.mosi, e.miso, 0];
  const code = I2C_CODES[e.kind];
  if (e.kind === "address") return [code, e.cycle, e.cycles, e.address, e.read ? 1 : 0, e.ack ? 1 : 0];
  if (e.kind === "write" || e.kind === "read") return [code, e.cycle, e.cycles, e.value, e.ack ? 1 : 0, 0];
  return [code, e.cycle, e.cycles, 0, 0, 0];
}

class JsSide {
  constructor(flash, hexText, mode, withInputs) {
    this.mode = mode; this.hit = null; this.steps = 0; this.wd = 0; this.irqTaken = 0; this.events = []; this.bus = [];
    this.script = withInputs ? new InputScript(mode) : null;
    if (mode === "cpu") this.cpu = new avr.CPU(new Uint16Array(flash.buffer.slice(0)));
    else {
      // The board as the engine builds it: ports B, C, D, timers 0-2, the ADC, USART0, TWI, SPI and the EEPROM, with
      // its serial feed, bus log and datasheet departures. Its fields are private in the types, plain at run time.
      const m = (this.mcu = new engine.ArduinoMcu(hexText));
      this.cpu = m.cpu;
      this.ports = [m.portB, m.portC, m.portD];
      this.timers = [m.timer0, m.timer1, m.timer2];
      Object.assign(this, { adc: m.adc, usart: m.usart, usart328: m.usart328, twi: m.twi, spi: m.spi, eeprom: m.eeprom, eepromBackend: m.eepromBackend });
      this.txCount = 0; this.txHash = 0x811c9dc5; this.twiSteps = 0; this.twiHash = 0x811c9dc5;
    }
    const cpu = this.cpu;
    cpu.onWatchdogReset = () => { this.wd++; };
    if (mode !== "cpu") {
      this.ports.forEach((port, pi) => port.addListener((value, prev) => {
        let packed = 0;
        for (let b = 0; b < 8; b++) packed += port.pinState(b) * 4 ** b;
        this.events.push([pi, cpu.cycles, value, prev, packed]);
      }));
    }
    if (mode === "cpu" || mode === "trap") {
      for (let a = 0x20; a < 0x100; a++) {
        if (a >= 0x5b && a <= 0x5f) continue;
        if (mode === "trap" && wasm.avr_is_ported(a)) continue;
        cpu.readHooks[a] = (addr) => { if (!this.hit) this.hit = { address: addr, write: false }; return cpu.data[addr]; };
        cpu.writeHooks[a] = (_v, _old, addr) => { if (!this.hit) this.hit = { address: addr, write: true }; return false; };
      }
    }
  }
  // The board's bus log since the last call, encoded, with the sent-byte count and hash and the I2C step count and
  // hash the core keeps, accumulated from it.
  pullBus() {
    if (!this.mcu) return;
    for (const e of this.mcu.drainBusEvents().events) {
      const row = encodeBusEvent(e);
      this.bus.push(row);
      if (row[0] === 0) { this.txCount++; this.txHash = Math.imul(this.txHash ^ e.value, 0x01000193) >>> 0; }
      if (row[0] >= 2 && row[0] <= 7) {
        this.twiSteps++;
        for (const word of row) this.twiHash = Math.imul(this.twiHash ^ word, 0x01000193) >>> 0;
      }
    }
  }
  apply(ev) {
    const cpu = this.cpu;
    if (ev.kind === "irq") cpu.setInterruptFlag({ address: ev.address, flagRegister: ev.flagRegister, flagMask: ev.flagMask, enableRegister: ev.enableRegister, enableMask: ev.enableMask, constant: false, inverseFlag: false });
    else if (ev.kind === "pin") this.ports[ev.port].setPin(ev.pin, ev.level);
    else if (ev.kind === "adc") this.adc.channelValues[ev.channel] = ev.volts;
    else if (ev.kind === "serial") this.mcu.writeSerialInput(Uint8Array.of(ev.byte));
    else cpu.writeData(ev.addr, ev.value);
  }
  run(n) {
    const cpu = this.cpu, instr = avr.avrInstruction;
    let done = 0;
    while (done < n && !this.hit) {
      if (this.script) for (const ev of this.script.due(cpu.cycles)) this.apply(ev);
      instr(cpu);
      const c1 = cpu.cycles;
      cpu.tick();
      if (cpu.cycles - c1 === 2) this.irqTaken++; // avrInterrupt costs exactly two cycles
      done++; this.steps++;
      // The board's bus log is a 4096-event ring and the counts above it are built from it; a long run between
      // checkpoints (bisect replays millions of instructions) must not let it drop events the core still counts.
      if ((done & 1023) === 0) this.pullBus();
    }
    this.pullBus();
    return done;
  }
  state() {
    this.pullBus();
    const c = this.cpu, s = new Float64Array(STATE_WORDS);
    s[0] = c.pc; s[1] = c.data[95]; s[2] = c.SP; s[3] = c.cycles; s[4] = c.nextInterrupt;
    s[5] = this.hit ? this.hit.address : -1; s[6] = this.hit && this.hit.write ? 1 : 0;
    s[7] = fnv1a(c.data);
    let n = 0, h = 0x811c9dc5;
    for (let e = c.nextClockEvent; e; e = e.next) { n++; h ^= e.cycles >>> 0; h = Math.imul(h, 0x01000193); }
    s[8] = n; s[9] = this.wd; s[46] = h >>> 0;
    for (let i = 0; i < 32; i++) s[10 + i] = c.data[i];
    for (let i = 0; i < 128; i++) if (c.pendingInterrupts[i]) s[42 + (i >> 5)] += 2 ** (i & 31);
    if (this.timers) {
      this.timers.forEach((t, i) => {
        const b = 47 + i * 10;
        s[b] = t.tcnt; s[b + 1] = t.lastCycle; s[b + 2] = t.ocrA; s[b + 3] = t.ocrB; s[b + 4] = t.icr; s[b + 5] = t.divider;
        s[b + 6] = (t.countingUp ? 1 : 0) | (t.tcntUpdated ? 2 : 0) | (t.updateDivider ? 4 : 0);
        s[b + 7] = t.nextOcrA; s[b + 8] = t.nextOcrB; s[b + 9] = t.tcntNext;
      });
      this.ports.forEach((p, i) => {
        const b = 77 + i * 6;
        s[b] = p.pinValue; s[b + 1] = p.lastValue; s[b + 2] = p.lastDdr; s[b + 3] = p.lastPin; s[b + 4] = p.overrideMask; s[b + 5] = p.overrideValue;
      });
      s[95] = this.adc.converting ? 1 : 0; s[96] = this.adc.conversionCycles;
      const u = this.usart328;
      s[98] = (u.receiving ? 1 : 0) | (this.spi.transmissionActive ? 2 : 0) | (this.twi.busy ? 4 : 0);
      // The receive FIFO, packed as the core packs it: length, then each frame's value and DOR flag.
      s[99] = u.fifo.reduce((acc, f, i) => acc + (f.value + 256 * (f.dor ? 1 : 0)) * 4 * 512 ** i, u.fifo.length);
      s[100] = this.txCount; s[101] = this.txHash; s[102] = u.input.length;
      s[103] = this.eeprom.writeEnabledCycles; s[104] = this.eeprom.writeCompleteCycles; s[105] = fnv1a(this.eepromBackend.memory);
      s[106] = u.shifting ? 1 : 0; s[107] = u.pending; s[108] = this.twiSteps; s[109] = this.twiHash;
      s[110] = u.held; s[111] = u.overrun ? 1 : 0;
    }
    s[97] = this.events.length;
    return s;
  }
  drainEvents() { const e = this.events; this.events = []; return e; }
  drainBus() { this.pullBus(); const b = this.bus; this.bus = []; return b; }
  data() { return this.cpu.data; }
}

class RustSide {
  constructor(flash, mode, withInputs) {
    const ptr = wasm.avr_alloc(flash.length);
    new Uint8Array(wasm.memory.buffer, ptr, flash.length).set(flash);
    this.m = wasm.avr_create(ptr, flash.length);
    wasm.avr_free(ptr, flash.length);
    if (mode === "cpu" || mode === "trap") wasm.avr_set_io_trap(this.m, 1);
    if (mode !== "cpu") wasm.avr_attach(this.m);
    this.script = withInputs ? new InputScript(mode) : null;
    this.steps = 0;
    this.out = wasm.avr_alloc(STATE_WORDS * 8);
    this.evBuf = wasm.avr_alloc(4096 * 6 * 8);
    this.horizon = -1;
  }
  schedule(upToCycle) {
    if (!this.script) return;
    for (const ev of this.script.due(upToCycle)) {
      if (ev.kind === "irq") wasm.avr_schedule_irq(this.m, ev.at, ev.address, ev.flagRegister, ev.flagMask, ev.enableRegister, ev.enableMask, 0, 0);
      else if (ev.kind === "pin") wasm.avr_schedule_pin(this.m, ev.at, ev.port, ev.pin, ev.level ? 1 : 0);
      else if (ev.kind === "adc") wasm.avr_schedule_adc(this.m, ev.at, ev.channel, ev.volts);
      else if (ev.kind === "serial") wasm.avr_schedule_serial(this.m, ev.at, ev.byte);
      else wasm.avr_schedule_write(this.m, ev.at, ev.addr, ev.value);
    }
  }
  run(n) {
    // An instruction takes at most 5 cycles (an interrupt adds 2), so scheduling 8
    // cycles per instruction ahead of the current count always covers the run.
    this.schedule(this.cycles() + 8 * n + 16);
    const done = wasm.avr_run(this.m, n);
    this.steps += done;
    return done;
  }
  cycles() { wasm.avr_state(this.m, this.out); return new Float64Array(wasm.memory.buffer, this.out, STATE_WORDS)[3]; }
  state() { wasm.avr_state(this.m, this.out); return Float64Array.from(new Float64Array(wasm.memory.buffer, this.out, STATE_WORDS)); }
  drainEvents() {
    const all = [];
    for (;;) {
      const n = wasm.avr_drain_events(this.m, this.evBuf, 4096);
      const v = new Float64Array(wasm.memory.buffer, this.evBuf, n * 5);
      for (let i = 0; i < n; i++) all.push(Array.from(v.subarray(i * 5, i * 5 + 5)));
      if (n < 4096) return all;
    }
  }
  drainBus() {
    const all = [];
    for (;;) {
      const n = wasm.avr_drain_bus(this.m, this.evBuf, 4096);
      const v = new Float64Array(wasm.memory.buffer, this.evBuf, n * 6);
      for (let i = 0; i < n; i++) all.push(Array.from(v.subarray(i * 6, i * 6 + 6)));
      if (n < 4096) return all;
    }
  }
  data() { return new Uint8Array(wasm.memory.buffer, wasm.avr_data_ptr(this.m), wasm.avr_data_len(this.m)); }
  free() { wasm.avr_destroy(this.m); wasm.avr_free(this.out, STATE_WORDS * 8); wasm.avr_free(this.evBuf, 4096 * 6 * 8); }
}

function diffStates(a, b) {
  const d = [];
  for (let i = 0; i < STATE_WORDS; i++) if (a[i] !== b[i] && FIELDS[i] !== "events") d.push(`${FIELDS[i]}: js=${a[i]} rust=${b[i]}`);
  return d;
}
function diffEvents(a, b) {
  if (a.length !== b.length) return [`GPIO listener calls: js=${a.length} rust=${b.length}`];
  for (let i = 0; i < a.length; i++) if (a[i].some((x, j) => x !== b[i][j])) return [`GPIO listener call ${i}: js=[${a[i]}] rust=[${b[i]}] (port, cycle, value, prev, pin states)`];
  return [];
}
const hex = (n, w = 4) => "0x" + (n >>> 0).toString(16).padStart(w, "0");

function diffBus(a, b) {
  const names = ["sent", "received", "I2C start", "I2C restart", "I2C stop", "I2C address", "I2C write", "I2C read", "SPI"];
  const show = (e) => `${names[e[0]]} at ${e[1]} for ${e[2]} [${e.slice(3)}]`;
  for (let i = 0; i < Math.max(a.length, b.length); i++) {
    if (!a[i] || !b[i] || a[i].some((x, j) => x !== b[i][j])) return [`bus event ${i}: js=${a[i] ? show(a[i]) : "none"} rust=${b[i] ? show(b[i]) : "none"}`];
  }
  return [];
}

function make(flash, hexText, mode) { return [new JsSide(flash, hexText, mode, opt.irq), new RustSide(flash, mode, opt.irq)]; }

// Compares GPIO listener calls, the serial, I2C and SPI event streams (cpu mode has none), and the state records.
function compare(js, rs) {
  const ej = js.drainEvents(), er = rs.drainEvents();
  const sj = js.state(), sr = rs.state();
  const bj = js.drainBus(), br = js.mode === "cpu" ? [] : rs.drainBus();
  sj[97] = 0; sr[97] = 0; // both drained
  return [...diffEvents(ej, er), ...diffBus(bj, br), ...diffStates(sj, sr)];
}

// Replays both sides from a fresh start to `upTo`, then steps single
// instructions to the first difference.
function bisect(flash, hexText, mode, upTo, k) {
  const [js, rs] = make(flash, hexText, mode);
  js.run(upTo); rs.run(upTo);
  js.drainEvents(); rs.drainEvents();
  if (mode !== "cpu") { js.drainBus(); rs.drainBus(); }
  const words = new Uint16Array(flash.buffer.slice(0));
  for (let i = 0; i < k + 1; i++) {
    const preJs = js.state();
    const pcBefore = preJs[0];
    const word = words[pcBefore] ?? 0;
    js.run(1); rs.run(1);
    const d = compare(js, rs);
    if (d.length) {
      console.log(`  first divergence at instruction ${upTo + i + 1} (pc ${hex(pcBefore)}, opcode ${hex(word)})`);
      console.log(`  decode: avr8js=${jsDecode(word)}  rust=${rustDecode(word)}`);
      console.log(`  state before: pc=${hex(pcBefore)} sreg=${hex(preJs[1], 2)} sp=${hex(preJs[2])} cycles=${preJs[3]}`);
      for (const line of d.slice(0, 12)) console.log("    " + line);
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

function runImage(file, flash, hexText, mode) {
  const label = `${path.basename(file)} [${mode}${opt.irq ? "+inputs" : ""}]`;
  const [js, rs] = make(flash, hexText, mode);
  const target = opt.steps;
  let done = 0, ok = true, checks = 0, eventsSeen = 0, pins = new Set();
  const t0 = Date.now();
  while (done < target) {
    const n = Math.min(opt.k, target - done);
    const a = js.run(n), b = rs.run(n);
    done += a;
    checks++;
    const ejs = js.events.length;
    eventsSeen += ejs;
    for (const e of js.events) pins.add(e[0]);
    const d = a !== b ? [`instructions run: js=${a} rust=${b}`] : compare(js, rs);
    if (d.length) {
      ok = false;
      console.log(`DIVERGED ${label} within instructions ${done - a + 1}..${done}`);
      for (const line of d.slice(0, 8)) console.log("    " + line);
      bisect(flash, hexText, mode, done - a, a);
      break;
    }
    if (js.hit) break;
  }
  const s = js.state();
  const ms = Date.now() - t0;
  if (ok) {
    const stop = js.hit
      ? `stopped after instruction ${done}: first ${js.hit.write ? "write to" : "read of"} ${mode === "cpu" ? "peripheral" : "unported"} register ${hex(js.hit.address, 2)} at pc ${hex(s[0])} (same on both sides)`
      : `ran ${done} instructions, ${s[3]} cycles, ${js.irqTaken} interrupts taken, to pc ${hex(s[0])}`;
    console.log(`OK       ${label}: ${stop}; ${checks} checkpoints (K=${opt.k})${mode === "cpu" ? "" : `, ${eventsSeen} GPIO listener calls compared, ${s[100]} serial bytes sent, ${s[102]} typed bytes waiting, EEPROM ${s[104] ? "written" : "never written"}`}, ${ms} ms`);
  }
  rs.free();
  return { ok, done, hit: js.hit, pc: s[0], cycles: s[3], events: eventsSeen };
}

// ---- the decode sweep: all 65536 opcode words -------------------------------------------
let failed = false;
console.log(`avr8js ${avrVersion} (${avrDir}); wasm ${opt.wasm} (${wasmBytes.length} bytes)`);
if (opt.sweep && !opt.bench) {
  let bad = 0;
  for (let op = 0; op < 0x10000; op++) {
    const j = jsDecode(op), r = rustDecode(op);
    if (j !== r) { if (bad++ < 10) console.log(`  decode mismatch ${hex(op)}: avr8js=${j} rust=${r}`); }
  }
  console.log(bad ? `DECODE DIVERGED on ${bad} of 65536 opcode words` : "OK       decode: all 65536 opcode words name the same instruction on both sides");
  if (bad) failed = true;
}
if (opt.bench) {
  // Instructions per second, each side alone, on the first image, built as ArduinoMcu builds it
  // (peripherals attached, no scripted inputs).
  const hexText = fs.readFileSync(opt.images[0], "utf8"), flash = parseIntelHex(hexText);
  for (const mode of ["full"]) {
    const js = new JsSide(flash, hexText, mode, false), rs = new RustSide(flash, mode, false);
    let t = process.hrtime.bigint(); js.run(opt.bench);
    const tj = Number(process.hrtime.bigint() - t) / 1e9;
    t = process.hrtime.bigint(); rs.run(opt.bench);
    const tr = Number(process.hrtime.bigint() - t) / 1e9;
    console.log(`bench ${path.basename(opt.images[0])} [${mode}]: ${opt.bench} instructions; avr8js ${(opt.bench / tj / 1e6).toFixed(1)} M/s, rust wasm ${(opt.bench / tr / 1e6).toFixed(1)} M/s`);
  }
}
const modes = opt.mode === "both" ? ["cpu", "trap", "full", "serial"] : [opt.mode];
for (const file of opt.bench ? [] : opt.images) {
  const hexText = fs.readFileSync(file, "utf8"), flash = parseIntelHex(hexText);
  for (const mode of modes) if (!runImage(file, flash, hexText, mode).ok) failed = true;
}
process.exit(failed ? 1 : 0);
