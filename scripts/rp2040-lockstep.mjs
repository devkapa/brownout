#!/usr/bin/env node
// Lock-step harness for the RP2040 port (W7, plan-1.0.0 section 4.3).
//
// Loads the same image into rp2040js 1.3.3 (the reference, from
// node_modules) and into the Rust port (the core's wasm built with the
// `rp2040` cargo feature, whose test-only `rp_*` exports this script drives),
// advances both by the same number of steps, and every K steps compares a
// state record (registers, xPSR, both stack pointers, PRIMASK/CONTROL, cycle
// count, clock, NVIC state, SIO and SysTick state, an interpolator digest),
// a hash of SRAM and the GPIO pin-event stream. On a mismatch it replays both
// from the start to the last matching checkpoint, single-steps to the first
// differing step and prints both decodes and the differing fields.
//
// A step is one iteration of rp2040js's Simulator.execute loop: a waiting
// core fast-forwards the clock to its next alarm, otherwise one instruction
// runs and the clock advances 8 ns per cycle. Inputs (GPIO levels) come from
// a recorded script applied at fixed cycle counts (clock nanos / 8), before
// the first step that starts at or after each, on both sides.
//
// Where the port stops (the first access to a peripheral R1 does not port,
// a break, or an idle core), both sides are compared up to the last step
// the port completed, and the stop is reported.
//
// Usage:
//   node scripts/rp2040-lockstep.mjs --build              # build the feature wasm
//   node scripts/rp2040-lockstep.mjs [--image name|all] [--steps N] [--k K]
//   node scripts/rp2040-lockstep.mjs --bench [--image name] [--steps N]
//
// The feature wasm goes to core/target/rp2040-lockstep/, apart from the
// shipped build, which this script never touches.

import { spawnSync } from "node:child_process";
import { fileURLToPath } from "node:url";
import fs from "node:fs";
import path from "node:path";
import { Simulator, ConsoleLogger, LogLevel } from "rp2040js";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(scriptDir, "..");
const coreDir = path.join(repoRoot, "core");
const targetDir = path.join(coreDir, "target", "rp2040-lockstep");
const wasmPath = path.join(targetDir, "wasm32-unknown-unknown", "release", "brownout_core.wasm");
const uf2Path = path.join(repoRoot, "test", "fixtures", "firmware", "pico-micropython.uf2");
const bootromPath = path.join(repoRoot, "src", "sim", "engine", "rp2040-bootrom.ts");

// The reference reads the RTC through a local-time `Date`; the port computes
// UTC, so the reference runs in UTC.
process.env.TZ = "UTC";

const STATE_WORDS = 90;
const STATE_NAMES = [
  ...Array.from({ length: 13 }, (_, i) => `r${i}`), "sp", "lr", "pc",
  "xPSR", "MSP", "PSP", "PRIMASK", "CONTROL", "cycles", "nanos", "waiting", "eventRegistered",
  "pendingInterrupts", "enabledInterrupts", "prio0", "prio1", "prio2", "prio3", "pendingNMI|PendSV|SVC|SysTick",
  "interruptsUpdated", "VTOR", "SHPR2", "SHPR3", "handlerMode", "sio.gpioOut", "sio.gpioOE", "sio.qspiOut",
  "sio.qspiOE", "sio.spinlocks", "sio.divCSR", "sio.quotient", "sio.remainder", "systick.flags",
  "systick.counter", "interp.digest",
  "timer.armed", "timer.alarm0", "timer.alarm1", "timer.alarm2", "timer.alarm3", "timer.intr", "timer.inte", "timer.intf",
  "timer.latchedHigh", "timer.paused", "clock.alarmCount", "clock.alarmSum", "resets.reset", "resets.wdsel",
  "clocks.sysCtrl", "clocks.refCtrl", "clocks.digest", "xosc.ctrl", "xosc.status", "xosc.dormant", "xosc.startup",
  "xosc.count", "psm.frceOn", "psm.frceOff", "psm.wdsel", "watchdog.ctrl", "watchdog.reason", "watchdog.scratch",
  "watchdog.tick", "watchdog.triggers", "rtc.ctrl", "rtc.setup0", "rtc.setup1", "rtc.rtc1", "rtc.rtc0", "syscfg.nmiMask",
  "busctrl.digest", "pads.bank0", "pads.qspi", "io.digest", "watchdog.enables", "pads.voltage",
];
const STOP_KINDS = ["running", "peripheral", "usb-dpram", "unaligned-read", "out-of-range", "break", "idle"];

// ---- args -------------------------------------------------------------------

const args = process.argv.slice(2);
function arg(name, fallback) {
  const i = args.indexOf(`--${name}`);
  return i >= 0 && i + 1 < args.length ? args[i + 1] : fallback;
}
const flag = (name) => args.includes(`--${name}`);

// ---- build ------------------------------------------------------------------

function build() {
  const jobs = process.env.CARGO_BUILD_JOBS || "3";
  const r = spawnSync(
    "cargo",
    ["build", "--release", "--target", "wasm32-unknown-unknown", "--features", "rp2040", "--target-dir", targetDir, "-j", jobs],
    { cwd: coreDir, stdio: "inherit" },
  );
  if (r.status !== 0) {
    console.error("rp2040-lockstep: cargo build failed");
    process.exit(1);
  }
}

// ---- inputs -----------------------------------------------------------------

function loadBootrom() {
  const text = fs.readFileSync(bootromPath, "utf8");
  const body = text.slice(text.indexOf("new Uint32Array(["));
  const words = body.match(/0x[0-9a-fA-F]{8}/g).map((h) => parseInt(h, 16) >>> 0);
  return new Uint32Array(words);
}

/** UF2 blocks as [flashOffset, bytes], as RP2040Mcu's loadUF2IntoFlash applies them. */
function uf2Blocks(uf2) {
  const view = new DataView(uf2.buffer, uf2.byteOffset, uf2.byteLength);
  const blocks = [];
  for (let off = 0; off + 512 <= uf2.byteLength; off += 512) {
    if (view.getUint32(off, true) !== 0x0a324655 || view.getUint32(off + 4, true) !== 0x9e5d5157) continue;
    const target = view.getUint32(off + 12, true);
    const size = view.getUint32(off + 16, true);
    const dest = target - 0x10000000;
    if (dest < 0 || dest + size > 16 * 1024 * 1024) continue;
    blocks.push([dest, uf2.subarray(off + 32, off + 32 + size)]);
  }
  return blocks;
}

// ---- a small Thumb assembler for the synthetic images -------------------------

class Asm {
  constructor(base) {
    this.base = base;
    this.h = [];
    this.labels = new Map();
    this.fixups = [];
    this.pool = [];
  }
  get pc() {
    return this.base + 2 * this.h.length;
  }
  op(...hs) {
    for (const x of hs) this.h.push(x & 0xffff);
    return this;
  }
  label(name) {
    this.labels.set(name, this.pc);
    return this;
  }
  /** LDR Rt, =value (literal pool after the code). */
  ldr(rt, value) {
    this.fixups.push({ kind: "lit", at: this.h.length, rt, value: value >>> 0 });
    return this.op(0);
  }
  /** LDR Rt, =address-of-label. */
  adr(rt, label) {
    this.fixups.push({ kind: "litlabel", at: this.h.length, rt, label });
    return this.op(0);
  }
  b(label, cond = 14) {
    this.fixups.push({ kind: cond === 14 ? "b" : "bcond", at: this.h.length, label, cond });
    return this.op(0);
  }
  bl(label) {
    this.fixups.push({ kind: "bl", at: this.h.length, label });
    return this.op(0, 0);
  }
  finish() {
    if (this.h.length % 2) this.op(0xbf00);
    const poolBase = this.pc;
    const words = [];
    const lit = (v) => {
      let i = words.indexOf(v);
      if (i < 0) {
        i = words.length;
        words.push(v);
      }
      return poolBase + 4 * i;
    };
    for (const f of this.fixups) {
      const at = this.base + 2 * f.at;
      if (f.kind === "lit" || f.kind === "litlabel") {
        const v = f.kind === "lit" ? f.value : this.labels.get(f.label) | 1;
        const addr = lit(v >>> 0);
        const off = addr - ((at + 4) & ~3);
        if (off < 0 || off > 1020 || off % 4) throw new Error(`literal out of range at ${at.toString(16)}`);
        this.h[f.at] = 0x4800 | (f.rt << 8) | (off >> 2);
      } else {
        const target = this.labels.get(f.label);
        if (target === undefined) throw new Error(`no label ${f.label}`);
        const off = target - (at + 4);
        const range = f.kind === "b" ? 2048 : f.kind === "bcond" ? 256 : 1 << 24;
        if (off < -range || off >= range) throw new Error(`branch to ${f.label} out of range at ${at.toString(16)}`);
        if (f.kind === "b") this.h[f.at] = 0xe000 | ((off >> 1) & 0x7ff);
        else if (f.kind === "bcond") this.h[f.at] = 0xd000 | (f.cond << 8) | ((off >> 1) & 0xff);
        else {
          const s = (off >> 24) & 1, i1 = (off >> 23) & 1, i2 = (off >> 22) & 1;
          const j1 = ~(i1 ^ s) & 1, j2 = ~(i2 ^ s) & 1;
          this.h[f.at] = 0xf000 | (s << 10) | ((off >> 12) & 0x3ff);
          this.h[f.at + 1] = 0xd000 | (j1 << 13) | (j2 << 11) | ((off >> 1) & 0x7ff);
        }
      }
    }
    const bytes = new Uint8Array(2 * this.h.length + 4 * words.length);
    const dv = new DataView(bytes.buffer);
    this.h.forEach((x, i) => dv.setUint16(2 * i, x, true));
    words.forEach((w, i) => dv.setUint32(2 * this.h.length + 4 * i, w, true));
    return bytes;
  }
}

// Encoders (ARMv6-M, A6.7).
const T = {
  movs: (rd, imm) => 0x2000 | (rd << 8) | imm,
  adds8: (rd, imm) => 0x3000 | (rd << 8) | imm,
  subs8: (rd, imm) => 0x3800 | (rd << 8) | imm,
  ldrImm: (rt, rn, imm5) => 0x6800 | (imm5 << 6) | (rn << 3) | rt,
  strImm: (rt, rn, imm5) => 0x6000 | (imm5 << 6) | (rn << 3) | rt,
  ldrbImm: (rt, rn, imm5) => 0x7800 | (imm5 << 6) | (rn << 3) | rt,
  strbImm: (rt, rn, imm5) => 0x7000 | (imm5 << 6) | (rn << 3) | rt,
  ldrhImm: (rt, rn, imm5) => 0x8800 | (imm5 << 6) | (rn << 3) | rt,
  strhImm: (rt, rn, imm5) => 0x8000 | (imm5 << 6) | (rn << 3) | rt,
  reg: (op, rt, rn, rm) => (op << 9) | (rm << 6) | (rn << 3) | rt,
  dp: (op, rm, rdn) => 0x4000 | (op << 6) | (rm << 3) | rdn,
  movHi: (rd, rm) => 0x4600 | ((rd & 8) << 4) | (rm << 3) | (rd & 7),
  push: (list, lr) => 0xb400 | (lr ? 0x100 : 0) | list,
  pop: (list, pc) => 0xbc00 | (pc ? 0x100 : 0) | list,
  stmia: (rn, list) => 0xc000 | (rn << 8) | list,
  ldmia: (rn, list) => 0xc800 | (rn << 8) | list,
  lsls: (rd, rm, imm) => (imm << 6) | (rm << 3) | rd,
  uxtb: (rd, rm) => 0xb2c0 | (rm << 3) | rd,
  bx: (rm) => 0x4700 | (rm << 3),
  svc: (imm) => 0xdf00 | imm,
  bkpt: (imm) => 0xbe00 | imm,
  msr: (sysm, rn) => [0xf380 | rn, 0x8800 | sysm],
  mrs: (rd, sysm) => [0xf3ef, 0x8000 | (rd << 8) | sysm],
  nop: 0xbf00,
  wfe: 0xbf20,
  wfi: 0xbf30,
  sev: 0xbf40,
  yield: 0xbf10,
  cpsid: 0xb672,
  cpsie: 0xb662,
  dmb: [0xf3bf, 0x8f5f],
  dsb: [0xf3bf, 0x8f4f],
  isb: [0xf3bf, 0x8f6f],
};

/** Deterministic PRNG (mulberry32). */
function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
const pick = (r, n) => Math.floor(r() * n);

/** A random 16-bit data-processing instruction whose destination is not in `avoid`. */
function aluOp(r, avoid) {
  for (;;) {
    const cls = pick(r, 7);
    let h, rd;
    if (cls === 0) {
      h = pick(r, 0x2000); // shifts by immediate, ADDS/SUBS register and imm3
      rd = h & 7;
    } else if (cls === 1) {
      h = 0x2000 | pick(r, 0x2000); // MOVS/CMP/ADDS/SUBS imm8
      rd = (h >> 8) & 7;
      if ((h >> 11) === 0b00101) rd = -1; // CMP writes nothing
    } else if (cls === 2) {
      h = 0x4000 | pick(r, 0x400); // the sixteen data-processing ops
      rd = h & 7;
      const op = (h >> 6) & 0xf;
      if (op === 0x8 || op === 0xa || op === 0xb) rd = -1; // TST CMP CMN
    } else if (cls === 3) {
      // ADD/CMP/MOV with high registers; never write SP or PC
      const kind = pick(r, 3);
      const rm = pick(r, 16);
      rd = pick(r, 13);
      if (kind === 0) h = 0x4400 | ((rd & 8) << 4) | (rm << 3) | (rd & 7);
      else if (kind === 1) {
        h = 0x4500 | ((rd & 8) << 4) | (rm << 3) | (rd & 7);
        rd = -1;
      } else h = 0x4600 | ((rd & 8) << 4) | (rm << 3) | (rd & 7);
    } else if (cls === 4) {
      h = 0xb200 | pick(r, 0x100); // extends
      rd = h & 7;
    } else if (cls === 5) {
      h = 0xba00 | pick(r, 0x100); // REV, REV16, REVSH and the unallocated 0xba80 row
      rd = h & 7;
    } else {
      h = [T.nop, T.yield, T.sev][pick(r, 3)];
      rd = -1;
    }
    if (rd >= 0 && avoid.includes(rd)) continue;
    return h;
  }
}

/** Fills `asm` with a random block; returns nothing. Branches only skip 16-bit ops. */
function aluBlock(asm, r, n, avoid) {
  for (let i = 0; i < n; i++) {
    const c = r();
    if (c < 0.06) {
      const skip = pick(r, 3);
      asm.op(0xd000 | (pick(r, 14) << 8) | skip);
      for (let j = 0; j <= skip; j++) asm.op(aluOp(r, avoid));
    } else if (c < 0.08) {
      // MRS from a random special register, MSR APSR (flags) from a random one
      const rd = [0, 1, 2, 3, 4, 5].filter((x) => !avoid.includes(x))[0];
      asm.op(...T.mrs(rd, [0, 3, 5, 8, 9, 16, 20][pick(r, 7)]));
      asm.op(...T.msr(0, pick(r, 6)));
    } else if (c < 0.09) {
      asm.op(...[T.dmb, T.dsb, T.isb][pick(r, 3)]);
    } else {
      asm.op(aluOp(r, avoid));
    }
  }
}


// ---- R2 image helpers ---------------------------------------------------------

/** VTOR = RAM, ISER = timer IRQs 0..3 and the IO bank (IRQ 13 only in r2-seq). */
function r2Prologue(asm, ioIrq) {
  asm.ldr(0, 0xe000ed08).ldr(1, RAM).op(T.strImm(1, 0, 0));
  asm.ldr(0, 0xe000e100).ldr(1, ioIrq ? 0x200f : 0xf).op(T.strImm(1, 0, 0));
}

/** The one handler: clear the timer's INTR and INTF, the IO bank's INTE, INTF and edges. */
function r2Handler(asm) {
  asm.ldr(0, 0x40054000).op(T.ldrImm(1, 0, 0x34 / 4), T.strImm(1, 0, 0x34 / 4), T.movs(1, 0), T.strImm(1, 0, 0x3c / 4));
  asm.ldr(0, 0x40014100).op(T.movs(1, 0));
  for (let i = 0; i < 8; i++) asm.op(T.strImm(1, 0, i));
  asm.ldr(0, 0x400140f0).ldr(1, 0xcccccccc).op(T.strImm(1, 0, 0), T.strImm(1, 0, 1), T.strImm(1, 0, 2), T.strImm(1, 0, 3));
  asm.op(T.bx(14));
}

function r2Vectors(side, asm) {
  const vt = new Uint8Array(4 * 48);
  const dv = new DataView(vt.buffer);
  const handler = asm.labels.get("handler") | 1;
  for (let n = 0; n < 48; n++) dv.setUint32(4 * n, handler, true);
  side.sram(0, vt);
}

/** rb = 0x40000000 + (n << 14) + (alias << 12) + (k << 7), built from small immediates. */
function r2SetBase(asm, rb, n, alias, k) {
  asm.op(T.movs(4, n), T.lsls(4, 4, 14), T.movs(rb, 0x40), T.lsls(rb, rb, 24), T.reg(0b0001100, rb, rb, 4));
  if (alias) asm.op(T.movs(4, alias), T.lsls(4, 4, 12), T.reg(0b0001100, rb, rb, 4));
  if (k) asm.op(T.movs(4, k), T.lsls(4, 4, 7), T.reg(0b0001100, rb, rb, 4));
}

// ---- images -------------------------------------------------------------------

const RAM = 0x20000000;

function seedRegs(r, set, keep) {
  for (let i = 0; i < 13; i++) if (!keep.includes(i)) set(i, Math.floor(r() * 4294967296) >>> 0);
}

/** Every image: { name, setup(side) } where side has bootrom/flash/sram/reg/pc/input setters. */
const IMAGES = {
  // RP2040Mcu.boot(): boot ROM, then MicroPython's UF2 in flash, PC at boot2.
  "mpy-wrapper": {
    k: 2000,
    steps: 2_000_000,
    setup(s) {
      s.bootrom(loadBootrom());
      for (const [off, bytes] of uf2Blocks(fs.readFileSync(uf2Path))) s.flash(off, bytes);
      s.reg(15, 0x10000000);
    },
  },
  // A real reset: the boot ROM's reset vector, with MicroPython in flash.
  "mpy-reset": {
    k: 2000,
    steps: 2_000_000,
    setup(s) {
      s.bootrom(loadBootrom());
      for (const [off, bytes] of uf2Blocks(fs.readFileSync(uf2Path))) s.flash(off, bytes);
    },
  },
  // MicroPython's application entry (its vector table after boot2 at
  // 0x10000100), past the boot ROM and boot2: pico-sdk's crt0.
  "mpy-app": {
    k: 2000,
    steps: 20_000_000,
    setup(s) {
      s.bootrom(loadBootrom());
      const blocks = uf2Blocks(fs.readFileSync(uf2Path));
      for (const [off, bytes] of blocks) s.flash(off, bytes);
      const flash = new Uint8Array(0x200);
      for (const [off, bytes] of blocks) if (off < 0x200) flash.set(bytes.subarray(0, Math.min(bytes.length, 0x200 - off)), off);
      const dv = new DataView(flash.buffer);
      s.reg(13, dv.getUint32(0x100, true));
      s.reg(15, dv.getUint32(0x104, true) & ~1);
    },
  },
  // Random data-processing streams (every 16-bit ALU, shift, extend and
  // reverse encoding, forward conditional branches, MRS/MSR, barriers) in a
  // loop, from random registers.
  "fuzz-alu": {
    k: 500,
    steps: 400_000,
    setup(s) {
      const r = rng(0xa1);
      const asm = new Asm(RAM + 0x1000);
      asm.label("top");
      aluBlock(asm, r, 3000, []);
      asm.bl("top"); // the loop is longer than B reaches
      s.sram(0x1000, asm.finish());
      seedRegs(r, s.reg, []);
      s.reg(13, RAM + 0x40000);
      s.reg(15, RAM + 0x1000);
    },
  },
  // Random loads and stores (word, halfword, byte, signed, immediate and
  // register offsets, LDM/STM, PUSH/POP) into an SRAM window, mixed with ALU
  // ops, in a loop.
  "fuzz-mem": {
    k: 500,
    steps: 400_000,
    setup(s) {
      const r = rng(0xb2);
      const asm = new Asm(RAM + 0x1000);
      asm.label("top");
      const avoid = [6, 7, 8];
      for (let i = 0; i < 1500; i++) {
        const c = pick(r, 9);
        const rt = pick(r, 6);
        if (c === 0) asm.op(T.ldrImm(rt, 7, pick(r, 32)), T.strImm(pick(r, 6), 7, pick(r, 32)));
        else if (c === 1) asm.op(T.ldrbImm(rt, 7, pick(r, 32)), T.strbImm(pick(r, 6), 7, pick(r, 32)));
        else if (c === 2) asm.op(T.ldrhImm(rt, 7, pick(r, 32)), T.strhImm(pick(r, 6), 7, pick(r, 32)));
        else if (c === 3) {
          // register offset: Rm = (Rm & 0xff) << scale
          const rm = pick(r, 6);
          const op = [0b0101100, 0b0101000, 0b0101101, 0b0101001, 0b0101110, 0b0101010, 0b0101011, 0b0101111][pick(r, 8)];
          const scale = op === 0b0101100 || op === 0b0101000 ? 2 : op === 0b0101110 || op === 0b0101010 || op === 0b0101011 ? 0 : 1;
          asm.op(T.uxtb(rm, rm), T.lsls(rm, rm, scale), T.reg(op, rt === rm ? (rm + 1) % 6 : rt, 7, rm));
        } else if (c === 4) {
          asm.op(T.movHi(6, 8), T.stmia(6, 1 + pick(r, 63)), T.movHi(6, 8), T.ldmia(6, 1 + pick(r, 63)));
        } else if (c === 5) {
          const list = 1 + pick(r, 31); // r0..r4: the pop takes one more
          const n = list.toString(2).replace(/0/g, "").length;
          let list2 = 0;
          while (list2.toString(2).replace(/0/g, "").length !== n + 1) list2 = pick(r, 64);
          asm.op(T.push(list, true), T.pop(list2, false)); // LR counts: pop one more low register
        } else if (c === 6) {
          asm.op(T.strImm(rt, 6, pick(r, 32)), T.ldrImm(pick(r, 6), 6, pick(r, 32)));
        } else aluBlock(asm, r, 2, avoid);
      }
      asm.bl("top"); // the loop is longer than B reaches
      s.sram(0x1000, asm.finish());
      seedRegs(r, s.reg, []);
      s.reg(6, RAM + 0x20000);
      s.reg(7, RAM + 0x30000);
      s.reg(8, RAM + 0x20000);
      s.reg(13, RAM + 0x40000);
      s.reg(15, RAM + 0x1000);
    },
  },
  // Random SIO traffic: GPIO set/clear/xor, the divider, spinlocks and both
  // interpolators, including byte and halfword stores (read-modify-write).
  "fuzz-sio": {
    k: 500,
    steps: 300_000,
    setup(s) {
      const r = rng(0xc3);
      const asm = new Asm(RAM + 0x1000);
      asm.label("top");
      const avoid = [5, 6, 7];
      for (let i = 0; i < 1500; i++) {
        const base = 5 + pick(r, 3);
        const rt = pick(r, 4);
        const c = pick(r, 8);
        if (c < 3) asm.op(T.strImm(rt, base, pick(r, 32)));
        else if (c < 5) asm.op(T.ldrImm(rt, base, pick(r, 32)));
        else if (c === 5) asm.op([T.strbImm, T.strhImm][pick(r, 2)](rt, base, pick(r, 32) & ~1));
        else if (c === 6) asm.op([T.ldrbImm, T.ldrhImm][pick(r, 2)](rt, base, pick(r, 32) & ~1));
        else aluBlock(asm, r, 2, avoid);
      }
      asm.bl("top"); // the loop is longer than B reaches
      s.sram(0x1000, asm.finish());
      seedRegs(r, s.reg, []);
      s.reg(5, 0xd0000000);
      s.reg(6, 0xd0000080);
      s.reg(7, 0xd0000100);
      s.reg(13, RAM + 0x40000);
      s.reg(15, RAM + 0x1000);
    },
    inputs: [
      { atCycle: 100, pin: 3, level: 1 },
      { atCycle: 5000, pin: 3, level: 0 },
      { atCycle: 9000, pin: 25, level: 1 },
    ],
  },
  // Exceptions: SVC, PendSV, SysTick and NVIC software IRQs with priorities
  // (nesting, masking by PRIMASK), the process stack, WFE/SEV, then a BKPT.
  sys: {
    k: 200,
    steps: 400_000,
    setup(s) {
      const asm = new Asm(RAM + 0x200);
      const COUNT = RAM + 0x100; // [0] handler entries, [1] last IPSR, [2] IPSR history xor
      asm.label("main");
      asm.ldr(0, 0xe000ed08).ldr(1, RAM).op(T.strImm(1, 0, 0)); // VTOR = RAM
      asm.ldr(0, 0xe000ed20).ldr(1, (2 << 30) | (3 << 22)).op(T.strImm(1, 0, 0)); // SHPR3: SysTick 2, PendSV 3
      asm.ldr(0, 0xe000ed1c).ldr(1, 1 << 30).op(T.strImm(1, 0, 0)); // SHPR2: SVCall 1
      asm.ldr(0, 0xe000e100).ldr(1, 0xfc000000).op(T.strImm(1, 0, 0)); // ISER: IRQ 26..31
      asm.ldr(0, 0xe000e418).ldr(1, 0xc0804000).op(T.strImm(1, 0, 0)); // IPR6: 25=1 26=2 27=3
      asm.ldr(0, 0xe000e41c).ldr(1, 0x00c04080).op(T.strImm(1, 0, 0)); // IPR7: 28=2 29=1 30=3 31=0
      asm.op(T.svc(1), T.nop);
      asm.ldr(0, 0xe000ed04).ldr(1, 1 << 28).op(T.strImm(1, 0, 0), T.nop); // PendSV
      asm.ldr(0, 0xe000e200).ldr(1, 1 << 28).op(T.strImm(1, 0, 0), T.nop); // pend IRQ 28
      asm.ldr(0, 0xe000e010).ldr(1, 300).op(T.strImm(1, 0, 1)); // RVR
      asm.op(T.movs(1, 0), T.strImm(1, 0, 2)); // CVR
      asm.op(T.movs(1, 7), T.strImm(1, 0, 0)); // CSR: enable, tick interrupt
      asm.ldr(4, 3000).label("spin").op(T.subs8(4, 1)).b("spin", 1); // ~30 SysTicks
      asm.op(T.ldrImm(2, 0, 0)); // CSR read clears COUNTFLAG
      asm.op(T.cpsid).ldr(0, 0xe000e200).ldr(1, 1 << 29).op(T.strImm(1, 0, 0), T.nop, T.nop, T.cpsie, T.nop);
      asm.ldr(0, 0xe000e010).op(T.movs(1, 0), T.strImm(1, 0, 0)); // SysTick off
      asm.ldr(0, RAM + 0x30000).op(...T.msr(9, 0), T.movs(1, 2), ...T.msr(20, 1)); // PSP, CONTROL.SPSEL
      asm.op(T.svc(2), T.nop, ...T.mrs(3, 20), ...T.mrs(4, 9), ...T.mrs(5, 8));
      asm.ldr(0, 0xe000e200).ldr(1, 1 << 31).op(T.strImm(1, 0, 0), T.nop); // IRQ 31 from thread on PSP
      asm.op(T.movs(1, 0), ...T.msr(20, 1)); // back to MSP
      asm.op(T.sev, T.wfe); // the last exception return registered an event: WFE consumes it
      asm.ldr(0, 0xe000ed04).op(T.ldrImm(1, 0, 0)); // ICSR
      asm.op(T.bkpt(0x42));
      // Every handler: count, log IPSR, re-pend from PendSV, return.
      asm.label("handler");
      asm.ldr(0, COUNT).op(T.ldrImm(1, 0, 0), T.adds8(1, 1), T.strImm(1, 0, 0));
      asm.op(...T.mrs(2, 5), T.strImm(2, 0, 1), T.ldrImm(3, 0, 2), T.dp(0x1, 2, 3), T.lsls(3, 3, 1), T.strImm(3, 0, 2));
      asm.op(0x2a0e); // CMP r2, #14 (PendSV)
      asm.b("ret", 1);
      asm.ldr(0, 0xe000e200).ldr(1, (1 << 30) | (1 << 27)).op(T.strImm(1, 0, 0), T.nop);
      asm.label("ret").op(T.bx(14));
      const code = asm.finish();
      const vt = new Uint8Array(4 * 48);
      const handler = asm.labels.get("handler") | 1;
      const dv = new DataView(vt.buffer);
      for (let n = 0; n < 48; n++) dv.setUint32(4 * n, handler, true);
      s.sram(0, vt);
      s.sram(0x200, code);
      s.reg(13, RAM + 0x40000);
      s.reg(15, RAM + 0x200);
    },
  },
  // R2: random word, halfword and byte traffic (and the atomic aliases) over
  // the small blocks: clocks, resets, psm, xosc, busctrl, timer, rtc,
  // pads, IO bank, syscfg, the stubs. Timer alarms are armed a few
  // microseconds ahead so their interrupts fire into a handler. The watchdog
  // gets scratch traffic only: the reference re-fires its alarm for ever
  // when the counter lands exactly on zero (see the port's note), so no
  // image may enable it for long.
  "r2-fuzz": {
    k: 500,
    steps: 400_000,
    setup(s) {
      const r = rng(0xd4);
      const asm = new Asm(RAM + 0x200);
      asm.label("top");
      const BLOCKS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1b, 0x15, 0x17, 0x7, 0x5, 0x2];
      for (let i = 0; i < 700; i++) {
        const n = BLOCKS[pick(r, BLOCKS.length)];
        const wd = n === 0x16;
        const alias = pick(r, 3) === 0 ? 1 + pick(r, 3) : 0;
        const off = wd ? 0 : pick(r, 3) === 0 ? pick(r, 4) : 0;
        r2SetBase(asm, 5, n, alias, off);
        const count = 1 + pick(r, 4);
        for (let j = 0; j < count; j++) {
          const rt = pick(r, 4);
          const kind = pick(r, 8);
          if (kind === 7 && n !== 0x16 && alias === 0 && off === 0 && n === 0x15) {
            // arm alarm A at now + a few microseconds
            asm.op(T.ldrImm(rt, 5, 0x28 / 4), T.adds8(rt, 5 + pick(r, 60)), T.strImm(rt, 5, (0x10 + 4 * pick(r, 4)) / 4));
          } else if (kind < 3) {
            asm.op(T.strImm(rt, 5, wd ? 3 + pick(r, 8) : pick(r, 32)));
          } else if (kind < 5) {
            asm.op(T.ldrImm(rt, 5, pick(r, 32)));
          } else if (kind === 5) {
            asm.op([T.strhImm, T.strbImm][pick(r, 2)](rt, 5, wd ? 12 + pick(r, 30) : pick(r, 32)) );
          } else {
            asm.op([T.ldrhImm, T.ldrbImm][pick(r, 2)](rt, 5, pick(r, 32)));
          }
        }
        aluBlock(asm, r, 1 + pick(r, 3), [4, 5, 6, 7]);
      }
      asm.bl("top");
      // The setup and the handler sit beside the literal pool, which the
      // body is too long to reach.
      asm.label("start");
      r2Prologue(asm, false);
      asm.bl("top");
      asm.label("handler");
      r2Handler(asm);
      s.sram(0x200, asm.finish());
      r2Vectors(s, asm);
      seedRegs(r, s.reg, [4, 5, 6, 7]);
      s.reg(13, RAM + 0x40000);
      s.reg(15, asm.labels.get("start"));
    },
  },
  // R2, directed: interrupts and long sleeps. Timer alarms wake the core out
  // of WFI (the clock jumps to each alarm: tens of simulated minutes pass in a
  // few steps), the RTC rolls over a year end, the 64-bit timer passes 2^32 us,
  // the watchdog counts down and is forced, and pin edges raise the IO bank's
  // interrupt.
  "r2-seq": {
    k: 50,
    steps: 3000,
    inputs: [
      { atCycle: 400, pin: 3, level: 1 },
      { atCycle: 700, pin: 3, level: 0 },
      { atCycle: 2000, pin: 5, level: 1 },
      { atCycle: 2600, pin: 5, level: 0 },
      { atCycle: 1e12, pin: 3, level: 1 },
      { atCycle: 2e12, pin: 5, level: 1 },
    ],
    setup(s) {
      const asm = new Asm(RAM + 0x200);
      r2Prologue(asm, true);
      asm.ldr(0, 0x4001c000).op(T.movs(1, 0x40), T.strImm(1, 0, 4), T.strImm(1, 0, 6)); // IE on GPIO3, GPIO5
      asm.ldr(0, 0x40014100).ldr(1, (8 << 12) | (4 << 20)).op(T.strImm(1, 0, 0)); // INTE0: GPIO3 EDGE_HIGH, GPIO5 EDGE_LOW
      asm.ldr(4, 0x40054000).op(T.movs(1, 0xf), T.strImm(1, 4, 0x38 / 4)); // timer INTE
      asm.ldr(7, 0x40058000).ldr(1, 0xcafe0001).op(T.strImm(1, 7, 3), T.strImm(1, 7, 10)); // watchdog scratch
      asm.ldr(1, 0xffffff).op(T.strImm(1, 7, 1)); // LOAD
      asm.ldr(1, (1 << 30) | (7 << 24)).op(T.strImm(1, 7, 0)); // CTRL: enable
      asm.ldr(3, 300).label("s1").op(T.subs8(3, 1)).b("s1", 1);
      asm.op(T.ldrImm(2, 7, 0)); // CTRL.TIME while counting
      asm.ldr(1, 7 << 24).op(T.strImm(1, 7, 0)); // disable (see the note on zero)
      asm.op(T.ldrImm(2, 7, 0));
      asm.ldr(1, (1 << 31) | (7 << 24)).op(T.strImm(1, 7, 0), T.ldrImm(2, 7, 2)); // TRIGGER, then REASON
      asm.ldr(5, 0x4005c000).ldr(1, (2023 << 12) | (12 << 8) | 31).op(T.strImm(1, 5, 1)); // RTC SETUP0
      asm.ldr(1, (23 << 16) | (59 << 8) | 50).op(T.strImm(1, 5, 2)); // SETUP1
      asm.op(T.movs(1, 0x10), T.strImm(1, 5, 3), T.movs(1, 1), T.strImm(1, 5, 3)); // LOAD, then ENABLE
      asm.op(T.ldrImm(0, 4, 0x0c / 4), T.adds8(0, 100), T.strImm(0, 4, 0x10 / 4), T.wfi); // ALARM0 in 100 us
      asm.op(T.ldrImm(2, 5, 0x1c / 4), T.ldrImm(3, 5, 0x18 / 4));
      asm.ldr(1, 0xc0000000); // 3221 s of microseconds
      asm.label("loop");
      asm.op(T.ldrImm(0, 4, 0x0c / 4), T.reg(0b0001100, 0, 0, 1), T.strImm(0, 4, 0x14 / 4), T.wfi); // ALARM1
      asm.op(T.ldrImm(0, 4, 0x0c / 4), T.reg(0b0001100, 0, 0, 1), T.strImm(0, 4, 0x18 / 4), T.wfi); // ALARM2
      asm.op(T.ldrImm(2, 4, 0x08 / 4), T.ldrImm(2, 5, 0x1c / 4), T.ldrImm(3, 5, 0x18 / 4)); // TIMEHR, RTC
      asm.b("loop");
      asm.label("handler");
      r2Handler(asm);
      s.sram(0x200, asm.finish());
      r2Vectors(s, asm);
      s.reg(13, RAM + 0x40000);
      s.reg(15, RAM + 0x200);
    },
  },
};

// ---- the two sides ------------------------------------------------------------

let wasmModule = null;
function rustSide() {
  if (!wasmModule) {
    if (!fs.existsSync(wasmPath)) {
      console.error(`rp2040-lockstep: no feature wasm at ${path.relative(repoRoot, wasmPath)}; run with --build`);
      process.exit(1);
    }
    wasmModule = new WebAssembly.Module(fs.readFileSync(wasmPath));
  }
  const inst = new WebAssembly.Instance(wasmModule, { env: { bo_host_now_ms: () => performance.now() } });
  const x = inst.exports;
  const m = x.rp_create();
  const mem = () => new Uint8Array(x.memory.buffer);
  const withBytes = (bytes, fn) => {
    const p = x.rp_alloc(bytes.length);
    mem().set(bytes, p);
    const r = fn(p);
    x.rp_free(p, bytes.length);
    return r;
  };
  const stateBuf = x.rp_alloc(8 * STATE_WORDS);
  const nameBuf = x.rp_alloc(64);
  return {
    side: {
      bootrom: (words) => withBytes(new Uint8Array(words.buffer, words.byteOffset, words.byteLength), (p) => x.rp_load_bootrom(m, p, words.byteLength)),
      flash: (off, bytes) => withBytes(bytes, (p) => x.rp_flash_write(m, off, p, bytes.length)),
      sram: (off, bytes) => withBytes(bytes, (p) => x.rp_sram_write(m, off, p, bytes.length)),
      reg: (n, v) => x.rp_set_reg(m, n, v >>> 0),
      input: (atCycle, pin, level) => x.rp_schedule_input(m, atCycle, pin, level),
    },
    run: (n) => x.rp_run(m, n),
    steps: () => x.rp_steps(m),
    state() {
      x.rp_state(m, stateBuf);
      return Array.from(new Float64Array(x.memory.buffer, stateBuf, STATE_WORDS));
    },
    ramHash: () => x.rp_ram_hash(m) >>> 0,
    stop() {
      const kind = x.rp_stop_kind(m);
      const n = x.rp_stop_name(m, nameBuf, 64);
      const name = new TextDecoder().decode(new Uint8Array(x.memory.buffer, nameBuf, n));
      return { kind: STOP_KINDS[kind], address: x.rp_stop_address(m) >>> 0, write: !!x.rp_stop_write(m), name };
    },
    pinEvents() {
      const total = x.rp_pin_events(m, 0, 0);
      const p = x.rp_alloc(24 * total + 8);
      x.rp_pin_events(m, p, total);
      const f = new Float64Array(x.memory.buffer, p, 3 * total);
      const out = [];
      for (let i = 0; i < total; i++) out.push([f[3 * i], f[3 * i + 1], f[3 * i + 2]]);
      x.rp_free(p, 24 * total + 8);
      return out;
    },
    peek: (a) => x.rp_peek32(m, a) >>> 0,
    destroy: () => x.rp_destroy(m),
  };
}

function jsSide() {
  const sim = new Simulator();
  const rp = sim.rp2040;
  rp.logger = new ConsoleLogger(LogLevel.Error, true);
  let breakCode = null;
  let wdTriggers = 0;
  rp.peripherals[0x40058].onWatchdogTrigger = () => {
    wdTriggers++;
  };
  rp.onBreak = (code) => {
    breakCode = code;
  };
  const events = [];
  rp.gpio.forEach((pin, i) => pin.addListener((state) => events.push([i, state, sim.clock.nanos])));
  const inputs = [];
  let steps = 0;
  let stop = null;
  const core = rp.core;
  function step() {
    const nowCycle = sim.clock.nanos / 8;
    for (let i = 0; i < inputs.length; ) {
      if (inputs[i].atCycle <= nowCycle) {
        const inp = inputs.splice(i, 1)[0];
        rp.gpio[inp.pin].setInputValue(!!inp.level);
      } else i++;
    }
    if (core.waiting) {
      if (!sim.clock.nextAlarm) {
        stop = { kind: "idle" };
        return false; // nothing ran
      }
      sim.clock.tick(sim.clock.nanosToNextAlarm);
    } else {
      let cycles;
      try {
        cycles = core.executeInstruction();
      } catch (e) {
        stop = { kind: "threw", message: String(e.message || e) };
        return false;
      }
      sim.clock.tick(cycles * 8);
    }
    steps++;
    if (breakCode !== null) stop = { kind: "break", address: breakCode };
    return true; // a step completed (a breaking one included)
  }
  return {
    side: {
      bootrom: (words) => rp.loadBootrom(words),
      flash: (off, bytes) => rp.flash.set(bytes, off),
      sram: (off, bytes) => rp.sram.set(bytes, off),
      reg: (n, v) => {
        core.registers[n] = v >>> 0;
      },
      input: (atCycle, pin, level) => inputs.push({ atCycle, pin, level }),
    },
    run(n) {
      let done = 0;
      while (done < n && stop === null && step()) done++;
      // step() returns true for the breaking step, so it is counted above.
      return done;
    },
    steps: () => steps,
    stop: () => stop,
    state() {
      const s = new Array(STATE_WORDS).fill(0);
      for (let i = 0; i < 16; i++) s[i] = core.registers[i];
      s[16] = core.xPSR >>> 0;
      s[17] = core.SPmain >>> 0;
      s[18] = core.SPprocess >>> 0;
      s[19] = core.PM ? 1 : 0;
      s[20] = (core.SPSEL ? 2 : 0) | (core.nPRIV ? 1 : 0);
      s[21] = core.cycles;
      s[22] = sim.clock.nanos;
      s[23] = core.waiting ? 1 : 0;
      s[24] = core.eventRegistered ? 1 : 0;
      s[25] = core.pendingInterrupts >>> 0;
      s[26] = core.enabledInterrupts >>> 0;
      for (let p = 0; p < 4; p++) s[27 + p] = core.interruptPriorities[p] >>> 0;
      s[31] = (core.pendingNMI ? 1 : 0) | (core.pendingPendSV ? 2 : 0) | (core.pendingSVCall ? 4 : 0) | (core.pendingSystick ? 8 : 0);
      s[32] = core.interruptsUpdated ? 1 : 0;
      s[33] = core.VTOR >>> 0;
      s[34] = core.SHPR2 >>> 0;
      s[35] = core.SHPR3 >>> 0;
      s[36] = core.currentMode === 1 ? 1 : 0;
      const sio = rp.sio;
      s[37] = sio.gpioValue >>> 0;
      s[38] = sio.gpioOutputEnable >>> 0;
      s[39] = sio.qspiGpioValue >>> 0;
      s[40] = sio.qspiGpioOutputEnable >>> 0;
      s[41] = sio.spinLock >>> 0;
      s[42] = sio.divCSR >>> 0;
      s[43] = sio.divQuotient >>> 0;
      s[44] = sio.divRemainder >>> 0;
      const ppb = rp.ppb;
      s[45] = (ppb.systickCountFlag ? 1 : 0) | (ppb.systickClkSource ? 2 : 0) | (ppb.systickIntEnable ? 4 : 0) | (ppb.systickTimer.enable ? 8 : 0);
      s[46] = ppb.systickTimer.counter;
      let h = 0x811c9dc5;
      for (const it of [sio.interp0, sio.interp1]) {
        for (const v of [it.accum0, it.accum1, it.base0, it.base1, it.base2, it.ctrl0, it.ctrl1, it.result0, it.result1, it.result2, it.smresult0, it.smresult1]) {
          h = Math.imul(h ^ (v >>> 0), 0x01000193) >>> 0;
        }
      }
      s[47] = h;
      const u = (v) => v >>> 0;
      const fnv = (vals) => {
        let x = 0x811c9dc5;
        for (const v of vals) x = Math.imul(x ^ (v >>> 0), 0x01000193) >>> 0;
        return x;
      };
      const per = rp.peripherals;
      const timer = per[0x40054];
      s[48] = timer.alarms.reduce((m, a, i) => m | ((a.armed ? 1 : 0) << i), 0);
      for (let i = 0; i < 4; i++) s[49 + i] = u(timer.alarms[i].targetMicros);
      s[53] = u(timer.intRaw);
      s[54] = u(timer.intEnable);
      s[55] = u(timer.intForce);
      s[56] = u(timer.latchedTimeHigh);
      s[57] = timer.paused ? 1 : 0;
      let count = 0;
      let sum = 0;
      for (let a = sim.clock.nextAlarm; a; a = a.next) {
        count++;
        sum += a.nanos;
      }
      s[58] = count;
      s[59] = sum;
      const resets = per[0x4000c];
      s[60] = u(resets.reset);
      s[61] = u(resets.wdsel);
      const clk = per[0x40008];
      s[62] = u(clk.sysCtrl);
      s[63] = u(clk.refCtrl);
      s[64] = fnv([
        clk.gpout0Ctrl, clk.gpout0Div, clk.gpout1Ctrl, clk.gpout1Div, clk.gpout2Ctrl, clk.gpout2Div, clk.gpout3Ctrl, clk.gpout3Div,
        clk.refCtrl, clk.refDiv, clk.sysCtrl, clk.sysDiv, clk.periCtrl, clk.periDiv, clk.usbCtrl, clk.usbDiv, clk.adcCtrl,
        clk.adcDiv, clk.rtcCtrl, clk.rtcDiv,
      ]);
      const xosc = per[0x40024];
      s[65] = u(xosc.ctrl);
      s[66] = u(xosc.readUint32(4));
      s[67] = u(xosc.dormant);
      s[68] = u(xosc.startup);
      s[69] = u(xosc.count);
      const psm = per[0x40010];
      s[70] = u(psm.frceOn);
      s[71] = u(psm.frceOff);
      s[72] = u(psm.wdsel);
      const wd = per[0x40058];
      s[73] = u(wd.readUint32(0));
      s[74] = u(wd.reason);
      s[75] = fnv(wd.scratchData);
      s[76] = u(wd.readUint32(0x2c));
      s[77] = wdTriggers;
      const rtc = per[0x4005c];
      s[78] = u(rtc.ctrl);
      s[79] = u(rtc.setup0);
      s[80] = u(rtc.setup1);
      s[81] = u(rtc.readUint32(0x18));
      s[82] = u(rtc.readUint32(0x1c));
      s[83] = u(rp.core.interruptNMIMask);
      const bus = per[0x40030];
      s[84] = fnv([...bus.perfCtr, ...bus.perfSel]);
      s[85] = fnv(rp.gpio.map((p) => p.padValue));
      s[86] = fnv(rp.qspi.flatMap((p) => [p.padValue, p.ctrl, p.irqStatus]));
      s[87] = fnv(rp.gpio.flatMap((p) => [p.ctrl, p.irqStatus, p.irqEnableMask, p.irqForceMask]));
      s[88] = (wd.alarm.enable ? 1 : 0) | (wd.timer.enable ? 2 : 0);
      s[89] = u(per[0x4001c].voltageSelect) | (u(per[0x40020].voltageSelect) << 1);
      return s;
    },
    ramHash() {
      const w = new Uint32Array(rp.sram.buffer, rp.sram.byteOffset, rp.sram.byteLength / 4);
      let h = 0x811c9dc5;
      for (let i = 0; i < w.length; i++) h = Math.imul(h ^ w[i], 0x01000193) >>> 0;
      return h;
    },
    pinEvents: () => events,
    peek(a) {
      a = a & ~3;
      if (a < 0x4000) return rp.bootrom[a / 4] >>> 0;
      if (a >= 0x10000000 && a < 0x14000000) return rp.flashView.getUint32(a & 0xffffff, true);
      if (a >= RAM && a < RAM + rp.sram.length) return rp.sramView.getUint32(a - RAM, true);
      return 0xffffffff;
    },
    destroy() {},
  };
}

// ---- decode ---------------------------------------------------------------------

/** The reference's decoder arm for an opcode (its if-chain order). */
function decode(op, op2) {
  const arms = [
    [(o) => o >> 6 === 0b0100000101, "ADCS"], [(o) => o >> 11 === 0b10101, "ADD Rd, SP, #imm"],
    [(o) => o >> 7 === 0b101100000, "ADD SP, #imm"], [(o) => o >> 9 === 0b0001110, "ADDS imm3"],
    [(o) => o >> 11 === 0b00110, "ADDS imm8"], [(o) => o >> 9 === 0b0001100, "ADDS reg"],
    [(o) => o >> 8 === 0b01000100, "ADD hi"], [(o) => o >> 11 === 0b10100, "ADR"],
    [(o) => o >> 6 === 0b0100000000, "ANDS"], [(o) => o >> 11 === 0b00010, "ASRS imm"],
    [(o) => o >> 6 === 0b0100000100, "ASRS reg"], [(o) => o >> 12 === 0b1101 && ((o >> 9) & 7) !== 7, "B<cond>"],
    [(o) => o >> 11 === 0b11100, "B"], [(o) => o >> 6 === 0b0100001110, "BICS"], [(o) => o >> 8 === 0b10111110, "BKPT"],
    [(o, o2) => o >> 11 === 0b11110 && o2 >> 14 === 0b11 && ((o2 >> 12) & 1) === 1, "BL"],
    [(o) => o >> 7 === 0b010001111 && (o & 7) === 0, "BLX"], [(o) => o >> 7 === 0b010001110 && (o & 7) === 0, "BX"],
    [(o) => o >> 6 === 0b0100001011, "CMN"], [(o) => o >> 11 === 0b00101, "CMP imm"], [(o) => o >> 6 === 0b0100001010, "CMP reg"],
    [(o) => o >> 8 === 0b01000101, "CMP hi"], [(o) => o === 0xb672, "CPSID"], [(o) => o === 0xb662, "CPSIE"],
    [(o, o2) => o === 0xf3bf && (o2 & 0xfff0) === 0x8f50, "DMB"], [(o, o2) => o === 0xf3bf && (o2 & 0xfff0) === 0x8f40, "DSB"],
    [(o) => o >> 6 === 0b0100000001, "EORS"], [(o, o2) => o === 0xf3bf && (o2 & 0xfff0) === 0x8f60, "ISB"],
    [(o) => o >> 11 === 0b11001, "LDMIA"], [(o) => o >> 11 === 0b01101, "LDR imm"], [(o) => o >> 11 === 0b10011, "LDR sp"],
    [(o) => o >> 11 === 0b01001, "LDR lit"], [(o) => o >> 9 === 0b0101100, "LDR reg"], [(o) => o >> 11 === 0b01111, "LDRB imm"],
    [(o) => o >> 9 === 0b0101110, "LDRB reg"], [(o) => o >> 11 === 0b10001, "LDRH imm"], [(o) => o >> 9 === 0b0101101, "LDRH reg"],
    [(o) => o >> 9 === 0b0101011, "LDRSB"], [(o) => o >> 9 === 0b0101111, "LDRSH"], [(o) => o >> 11 === 0, "LSLS imm"],
    [(o) => o >> 6 === 0b0100000010, "LSLS reg"], [(o) => o >> 11 === 0b00001, "LSRS imm"], [(o) => o >> 6 === 0b0100000011, "LSRS reg"],
    [(o) => o >> 8 === 0b01000110, "MOV"], [(o) => o >> 11 === 0b00100, "MOVS"],
    [(o, o2) => o === 0xf3ef && o2 >> 12 === 0b1000, "MRS"], [(o, o2) => o >> 4 === 0b111100111000 && o2 >> 8 === 0b10001000, "MSR"],
    [(o) => o >> 6 === 0b0100001101, "MULS"], [(o) => o >> 6 === 0b0100001111, "MVNS"], [(o) => o >> 6 === 0b0100001100, "ORRS"],
    [(o) => o >> 9 === 0b1011110, "POP"], [(o) => o >> 9 === 0b1011010, "PUSH"], [(o) => o >> 6 === 0b1011101000, "REV"],
    [(o) => o >> 6 === 0b1011101001, "REV16"], [(o) => o >> 6 === 0b1011101011, "REVSH"], [(o) => o >> 6 === 0b0100000111, "RORS"],
    [(o) => o >> 6 === 0b0100001001, "RSBS"], [(o) => o === 0xbf00, "NOP"], [(o) => o >> 6 === 0b0100000110, "SBCS"],
    [(o) => o === 0xbf40, "SEV"], [(o) => o >> 11 === 0b11000, "STMIA"], [(o) => o >> 11 === 0b01100, "STR imm"],
    [(o) => o >> 11 === 0b10010, "STR sp"], [(o) => o >> 9 === 0b0101000, "STR reg"], [(o) => o >> 11 === 0b01110, "STRB imm"],
    [(o) => o >> 9 === 0b0101010, "STRB reg"], [(o) => o >> 11 === 0b10000, "STRH imm"], [(o) => o >> 9 === 0b0101001, "STRH reg"],
    [(o) => o >> 7 === 0b101100001, "SUB SP, #imm"], [(o) => o >> 9 === 0b0001111, "SUBS imm3"], [(o) => o >> 11 === 0b00111, "SUBS imm8"],
    [(o) => o >> 9 === 0b0001101, "SUBS reg"], [(o) => o >> 8 === 0b11011111, "SVC"], [(o) => o >> 6 === 0b1011001001, "SXTB"],
    [(o) => o >> 6 === 0b1011001000, "SXTH"], [(o) => o >> 6 === 0b0100001000, "TST"], [(o) => o >> 8 === 0b11011110, "UDF"],
    [(o, o2) => o >> 4 === 0b111101111111 && o2 >> 12 === 0b1010, "UDF.W"], [(o) => o >> 6 === 0b1011001011, "UXTB"],
    [(o) => o >> 6 === 0b1011001010, "UXTH"], [(o) => o === 0xbf20, "WFE"], [(o) => o === 0xbf30, "WFI"], [(o) => o === 0xbf10, "YIELD"],
  ];
  for (const [test, name] of arms) if (test(op, op2)) return name;
  return "unimplemented";
}

function decodeAt(side, pc) {
  const a = pc & ~1;
  const w0 = side.peek(a & ~3);
  const op = a & 2 ? w0 >>> 16 : w0 & 0xffff;
  const w1 = side.peek((a + 2) & ~3);
  const op2 = (a + 2) & 2 ? w1 >>> 16 : w1 & 0xffff;
  const wide = op >> 12 === 0b1111 || op >> 11 === 0b11101;
  return `${hex(a)}: ${hex(op, 4)}${wide ? " " + hex(op2, 4) : ""}  ${decode(op, op2)}`;
}

const hex = (v, w = 8) => "0x" + (v >>> 0).toString(16).padStart(w, "0");

// ---- the comparison ---------------------------------------------------------------

function make(image) {
  const rust = rustSide();
  const js = jsSide();
  for (const s of [rust, js]) {
    image.setup(s.side);
    for (const inp of image.inputs ?? []) s.side.input(inp.atCycle, inp.pin, inp.level);
  }
  return { rust, js };
}

function diffState(a, b) {
  const out = [];
  for (let i = 0; i < STATE_WORDS; i++) if (a[i] !== b[i]) out.push(`${STATE_NAMES[i]}: rust ${fmt(a[i])} js ${fmt(b[i])}`);
  return out;
}
const fmt = (v) => (Number.isInteger(v) && v >= 0 && v <= 0xffffffff ? hex(v) : String(v));

function sameEvents(a, b) {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) for (let j = 0; j < 3; j++) if (a[i][j] !== b[i][j]) return false;
  return true;
}

function compare(rust, js, withRam) {
  const d = diffState(rust.state(), js.state());
  if (rust.steps() !== js.steps()) d.unshift(`steps: rust ${rust.steps()} js ${js.steps()}`);
  if (withRam) {
    const hr = rust.ramHash(), hj = js.ramHash();
    if (hr !== hj) d.push(`sram hash: rust ${hex(hr)} js ${hex(hj)}`);
  }
  if (!sameEvents(rust.pinEvents(), js.pinEvents())) d.push(`pin events: rust ${JSON.stringify(rust.pinEvents())} js ${JSON.stringify(js.pinEvents())}`);
  return d;
}

/** Replays to `good` steps, then single-steps to the first differing step. */
function bisect(image, good) {
  const { rust, js } = make(image);
  rust.run(good);
  js.run(good);
  for (let i = good; ; i++) {
    const pc = rust.state()[15];
    const before = [decodeAt(rust, pc), decodeAt(js, js.state()[15])];
    const nr = rust.run(1);
    const nj = js.run(1);
    const d = compare(rust, js, true);
    if (nr !== nj && d.length === 0) d.push(`completed: rust ${nr} js ${nj}`);
    if (d.length) {
      console.log(`  first differing step: ${i + 1} (after ${i} matching steps)`);
      console.log(`    rust executed  ${before[0]}`);
      console.log(`    js executed    ${before[1]}`);
      for (const line of d) console.log(`    ${line}`);
      return;
    }
    if (nr === 0 && nj === 0) {
      console.log("  bisect: both stopped without a per-step difference");
      return;
    }
  }
}

function runImage(name, image, steps, k) {
  const t0 = performance.now();
  const { rust, js } = make(image);
  let good = 0;
  let total = 0;
  let checkpoints = 0;
  for (;;) {
    const want = Math.min(k, steps - total);
    if (want <= 0) break;
    const nr = rust.run(want);
    // The reference runs exactly what the port completed, so a port stop
    // leaves both at the same comparable step.
    const nj = js.run(nr);
    total += nr;
    checkpoints++;
    const d = compare(rust, js, true);
    if (nj !== nr && d.length === 0) d.push(`reference stopped early: ${JSON.stringify(js.stop())}`);
    if (d.length) {
      console.log(`${name}: DIVERGED between steps ${good} and ${total}`);
      for (const line of d.slice(0, 12)) console.log(`  ${line}`);
      bisect(image, good);
      return false;
    }
    good = total;
    if (nr < want) break;
  }
  const st = rust.stop();
  const pc = rust.state()[15];
  let where = `ran ${total} steps`;
  if (st.kind !== "running") {
    const what =
      st.kind === "peripheral"
        ? `${st.write ? "write to" : "read from"} ${st.name} at ${hex(st.address)}`
        : st.kind === "break"
          ? `break code ${hex(st.address, 2)}`
          : st.kind === "idle"
            ? "WFI/WFE with no alarm scheduled"
            : `${st.kind} at ${hex(st.address)}`;
    where += `; stopped: ${what}`;
    if (st.kind !== "idle") where += ` (instruction ${decodeAt(rust, pc)})`;
  }
  const state = rust.state();
  console.log(
    `${name}: ZERO divergence over ${total} steps, ${checkpoints} checkpoints of <= ${k}; ` +
      `${state[21]} cycles, clock ${state[22]} ns, ${rust.pinEvents().length} pin events; ${where} ` +
      `[${((performance.now() - t0) / 1000).toFixed(1)} s]`,
  );
  rust.destroy();
  return true;
}

function bench(name, image, steps) {
  const results = {};
  for (const which of ["rust", "js"]) {
    const s = which === "rust" ? rustSide() : jsSide();
    image.setup(s.side);
    s.run(1000); // warm-up
    const t0 = performance.now();
    const n = s.run(steps);
    const dt = (performance.now() - t0) / 1000;
    results[which] = { n, dt, ips: n / dt };
    s.destroy();
  }
  console.log(
    `${name} bench: port ${(results.rust.ips / 1e6).toFixed(2)} M steps/s (${results.rust.n} in ${results.rust.dt.toFixed(2)} s), ` +
      `reference ${(results.js.ips / 1e6).toFixed(2)} M steps/s (${results.js.n} in ${results.js.dt.toFixed(2)} s), ` +
      `ratio ${(results.rust.ips / results.js.ips).toFixed(2)}x`,
  );
}

// ---- main -------------------------------------------------------------------------

// The reference's SIO and peripheral stubs warn on every invalid access
// through console.warn; the fuzz images make many on purpose.
console.warn = () => {};

if (flag("build")) build();
const names = arg("image", "all") === "all" ? Object.keys(IMAGES) : arg("image").split(",");
if (flag("bench")) {
  for (const n of names) bench(n, IMAGES[n], Number(arg("steps", "5000000")));
} else if (!flag("build") || args.includes("--image")) {
  let ok = true;
  for (const n of names) {
    const image = IMAGES[n];
    if (!image) throw new Error(`no image ${n}; images: ${Object.keys(IMAGES).join(", ")}`);
    ok = runImage(n, image, Number(arg("steps", image.steps)), Number(arg("k", image.k))) && ok;
  }
  process.exit(ok ? 0 : 1);
}
