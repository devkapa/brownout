/**
 * An iCE40 UltraPlus (UP5K) running a synthesized design, as on the UPduino v3.1: the netlist Yosys writes for
 * `synth_ice40` (`write_json`), mapped to the board's package pins by port name (`gpio_N`) or a PCF, and simulated
 * cell by cell. The fabric's primitives are the iCE40 technology library's (Lattice "iCE Technology Library",
 * rev 3.0): 4-input LUTs, the carry chain, D flip-flops with clock enable and synchronous or asynchronous set and
 * reset on either clock edge, global buffers, I/O cells with output enable and pull-up, the 48 MHz and 10 kHz
 * oscillators and the RGB LED current-sink driver.
 *
 * Every flip-flop powers up 0, as the device's do; Yosys realizes other initial values with inverters around them.
 * Logic settles in zero time between clock edges (no routing delays), levelized so each LUT is evaluated once its
 * inputs are final; an edge on a net a flip-flop is clocked by samples every flip-flop on that edge together, and a
 * flip-flop's own output clocking another (a ripple counter) runs in the same instant, as the real chain does within
 * its few nanoseconds. Time advances from edge to edge of the design's clocks (the oscillators and the board's
 * 12 MHz on gpio_20), so a design costs time per edge it actually has, not per simulated second.
 */

/** Yosys `write_json` output, the parts this reads. */
export interface YosysJson {
  modules: Record<string, YosysModule>;
}

export interface YosysModule {
  ports: Record<string, { direction: "input" | "output" | "inout"; bits: (number | string)[] }>;
  cells: Record<string, { type: string; parameters?: Record<string, string>; connections: Record<string, (number | string)[]> }>;
}

/** A design the fabric cannot run, with the reason in the words a user can act on. */
export class FpgaError extends Error {
  override name = "FpgaError";
}

/**
 * The package pins the UPduino v3.1 brings out: its 32 gpio_N headers (N the package pin), the SPI flash pins
 * (14 MISO, 15 SCK, 16 SS, 17 MOSI) and the on-board RGB LED's pads (39 green, 40 blue, 41 red).
 */
export const UPDUINO_PINS: readonly number[] = [
  2, 3, 4, 6, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 25, 26, 27, 28, 31, 32, 34, 35, 36, 37, 38, 39, 40,
  41, 42, 43, 44, 45, 46, 47, 48,
];
/** The pads SB_RGBA_DRV's RGB0, RGB1 and RGB2 sink (the UP5K's fixed RGB pins). */
export const RGB_PADS = [39, 40, 41] as const;
// Port names the UPduino's own examples use for fixed pins.
const NAMED_PINS: Record<string, number> = {
  led_green: 39, led_blue: 40, led_red: 41, spi_miso: 14, spi_sck: 15, spi_ssn: 16, spi_mosi: 17,
};

const HFOSC_HZ = 48e6;
const LFOSC_HZ = 10e3;
// Settling rounds within one instant before the design is taken to oscillate (a combinational loop through a
// flip-flop's asynchronous reset, say).
const MAX_ROUNDS = 64;

const enum Op {
  Lut,
  Carry,
  Buf,
  Tbuf,
}

interface Flop {
  d: number;
  q: number;
  clk: number;
  neg: boolean;
  en: number;
  /** Set or reset net, or -1. */
  sr: number;
  srAsync: boolean;
  /** The value the set or reset forces. */
  srValue: number;
}

/** How a package pin meets the fabric. */
interface PadBinding {
  /** The net the pad's level drives into the design, or -1. */
  input: number;
  /** The net the design drives the pad from, or -1. */
  output: number;
  /** The output enable: a net, or -1 for always (with an output), or -2 for never. */
  enable: number;
  pullup: boolean;
  /** Driven by SB_RGBA_DRV: sinks (low) while its net is 1, else high impedance. */
  rgb: boolean;
}

interface Clock {
  net: number;
  half: number;
  next: number;
}

function param(value: string | undefined, fallback: number): number {
  if (value === undefined) return fallback;
  const v = value.trim();
  if (/^0b[01]+$/i.test(v)) return parseInt(v.slice(2), 2);
  if (/^[01]+$/.test(v)) return parseInt(v, 2);
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

/** `set_io` lines of a PCF: port bit name (`led`, `bus[3]`) to package pin. */
export function parsePcf(text: string): Map<string, number> {
  const pins = new Map<string, number>();
  text.split(/\r?\n/).forEach((raw, i) => {
    const line = raw.replace(/#.*/, "").trim();
    if (!line) return;
    const words = line.split(/\s+/);
    if (words[0] !== "set_io") throw new FpgaError(`PCF line ${i + 1}: only set_io is understood, not "${words[0]}"`);
    const args = words.slice(1).filter((w, j, all) => !w.startsWith("-") && !(j > 0 && all[j - 1] === "-pullup"));
    if (args.length !== 2 || !/^\d+$/.test(args[1]!)) throw new FpgaError(`PCF line ${i + 1}: expected "set_io <port> <pin>"`);
    pins.set(args[0]!, Number(args[1]));
  });
  return pins;
}

export interface FpgaOptions {
  /** A PCF placing ports on package pins; ports it leaves out are placed by name (`gpio_N`). */
  pcf?: string;
  /** Oscillators on the board driving package pins (the UPduino's 12 MHz on gpio_20 with its OSC jumper fitted). */
  boardClocks?: { pin: number; hz: number }[];
}

export class Ice40 {
  /** The nets' levels: 0 and 1 are the constants. */
  readonly nets: Uint8Array;
  private readonly ops: Int32Array;
  private readonly opInit: Int32Array;
  private readonly opCount: number;
  private readonly opLevel: Int32Array;
  private readonly fanout: Int32Array[];
  private readonly levels: number;
  private readonly dirty: Uint8Array;
  private readonly buckets: number[][];
  private readonly flops: Flop[] = [];
  // The flip-flops as arrays, for the edge loop: D, Q, enable, set/reset net (-1 none) and its forced value, and
  // whether that is asynchronous.
  private fD = new Int32Array(0);
  private fQ = new Int32Array(0);
  private fEn = new Int32Array(0);
  private fSr = new Int32Array(0);
  private fSrValue = new Uint8Array(0);
  private fSrAsync = new Uint8Array(0);
  // Per net: the flip-flops it clocks on its rising and falling edges, and those it sets or resets asynchronously.
  private riseOf: (Int32Array | null)[] = [];
  private fallOf: (Int32Array | null)[] = [];
  private asyncOf: (Int32Array | null)[] = [];
  private readonly isClock: Uint8Array;
  private readonly isAsync: Uint8Array;
  // Clock and asynchronous-control nets that moved since the last settling round, reused between rounds.
  private edges: number[] = [];
  private asyncTouched: number[] = [];
  private updQ = new Int32Array(0);
  private updV = new Uint8Array(0);
  private readonly pads = new Map<number, PadBinding>();
  private readonly clocks: Clock[] = [];
  /** Simulated seconds since configuration. */
  time = 0;
  /** Clock edges run so far, for the host's cost accounting. */
  edgeCount = 0;

  constructor(netlist: YosysJson, options: FpgaOptions = {}) {
    const top = netlist.modules.top ?? Object.values(netlist.modules)[0];
    if (!top) throw new FpgaError("the netlist has no module");
    let maxNet = 1;
    const bit = (b: number | string | undefined): number => {
      if (typeof b === "number") {
        if (b > maxNet) maxNet = b;
        return b;
      }
      return b === "1" ? 1 : 0;
    };
    for (const p of Object.values(top.ports)) p.bits.forEach(bit);
    for (const c of Object.values(top.cells)) for (const bits of Object.values(c.connections)) bits.forEach(bit);
    this.nets = new Uint8Array(maxNet + 1);
    this.nets[1] = 1;
    this.isClock = new Uint8Array(maxNet + 1);
    this.isAsync = new Uint8Array(maxNet + 1);

    const ops: number[][] = [];
    const inits: number[] = [];
    const unsupported = new Set<string>();
    const rgbNets = new Set<number>();
    const ioCells: { pin: number; input: number; output: number; enable: number; pullup: boolean }[] = [];
    const ioPortBits = new Set<number>();
    const conn = (c: YosysModule["cells"][string], port: string, fallback = 0) => {
      const b = c.connections[port];
      return b && b.length ? bit(b[0]) : fallback;
    };
    for (const [name, c] of Object.entries(top.cells)) {
      const p = c.parameters ?? {};
      const t = c.type;
      if (t === "SB_LUT4") {
        ops.push([Op.Lut, conn(c, "O"), conn(c, "I0"), conn(c, "I1"), conn(c, "I2"), conn(c, "I3")]);
        inits.push(param(p.LUT_INIT, 0));
      } else if (t === "SB_CARRY") {
        ops.push([Op.Carry, conn(c, "CO"), conn(c, "I0"), conn(c, "I1"), conn(c, "CI"), 0]);
        inits.push(0);
      } else if (t === "SB_GB") {
        ops.push([Op.Buf, conn(c, "GLOBAL_BUFFER_OUTPUT"), conn(c, "USER_SIGNAL_TO_GLOBAL_BUFFER"), 0, 0, 0]);
        inits.push(0);
      } else if (t === "$_TBUF_") {
        ops.push([Op.Tbuf, conn(c, "Y"), conn(c, "A"), conn(c, "E"), 0, 0]);
        inits.push(0);
      } else if (/^SB_DFFN?E?(SR|R|SS|S)?$/.test(t)) {
        const m = /^SB_DFF(N?)(E?)(SR|R|SS|S)?$/.exec(t)!;
        const kind = m[3] ?? "";
        this.flops.push({
          d: conn(c, "D"),
          q: conn(c, "Q"),
          clk: conn(c, "C"),
          neg: m[1] === "N",
          en: m[2] ? conn(c, "E", 1) : 1,
          sr: kind ? conn(c, kind.startsWith("S") && kind !== "SR" ? "S" : "R") : -1,
          srAsync: kind === "R" || kind === "S",
          srValue: kind === "S" || kind === "SS" ? 1 : 0,
        });
      } else if (t === "SB_HFOSC" || t === "SB_LFOSC") {
        const hf = t === "SB_HFOSC";
        const out = conn(c, hf ? "CLKHF" : "CLKLF");
        const on = conn(c, hf ? "CLKHFEN" : "CLKLFEN", 1) === 1 && conn(c, hf ? "CLKHFPU" : "CLKLFPU", 1) === 1;
        if (on) {
          const hz = hf ? HFOSC_HZ / (1 << (param(p.CLKHF_DIV, 0) & 3)) : LFOSC_HZ;
          this.clocks.push({ net: out, half: 0.5 / hz, next: 0.5 / hz });
        }
      } else if (t === "SB_RGBA_DRV" || t === "SB_RGB_DRV") {
        const enabled = conn(c, "CURREN", 1) === 1 && conn(c, "RGBLEDEN", 1) === 1;
        ["RGB0", "RGB1", "RGB2"].forEach((pad, i) => {
          const out = c.connections[pad]?.[0];
          if (typeof out !== "number") return;
          rgbNets.add(out);
          ops.push([Op.Buf, out, enabled ? conn(c, `${pad}PWM`) : 0, 0, 0, 0]);
          inits.push(0);
          ioCells.push({ pin: RGB_PADS[i]!, input: -1, output: out, enable: -1, pullup: false });
        });
      } else if (t === "SB_IO") {
        const pkg = c.connections.PACKAGE_PIN?.[0];
        if (typeof pkg !== "number") throw new FpgaError(`SB_IO ${name} has no PACKAGE_PIN`);
        const type = param(p.PIN_TYPE, 0);
        const inMode = type & 3;
        const outData = (type >> 2) & 3;
        const outEnable = (type >> 4) & 3;
        if (inMode !== 1 && inMode !== 0) unsupported.add(`SB_IO ${name}: latched input (PIN_TYPE ${p.PIN_TYPE})`);
        if (inMode === 0 && c.connections.D_IN_0) unsupported.add(`SB_IO ${name}: registered input (PIN_TYPE ${p.PIN_TYPE})`);
        if (outEnable !== 0 && outData !== 2) unsupported.add(`SB_IO ${name}: registered or DDR output (PIN_TYPE ${p.PIN_TYPE})`);
        if (outEnable === 3) unsupported.add(`SB_IO ${name}: registered output enable (PIN_TYPE ${p.PIN_TYPE})`);
        ioPortBits.add(pkg);
        ioCells.push({
          pin: -pkg,
          input: c.connections.D_IN_0 ? conn(c, "D_IN_0") : -1,
          output: outEnable === 0 ? -1 : conn(c, "D_OUT_0"),
          enable: outEnable === 0 ? -2 : outEnable === 1 ? -1 : conn(c, "OUTPUT_ENABLE", 1),
          pullup: param(p.PULLUP, 0) === 1,
        });
      } else if (t === "SB_WARMBOOT" || t === "$scopeinfo") {
        // Nothing to simulate: a design that warm-boots is reconfigured by its host.
      } else {
        unsupported.add(t);
      }
    }
    if (unsupported.size) {
      throw new FpgaError(`not simulated yet: ${[...unsupported].join(", ")}`);
    }

    // Package pins: the PCF first, then names.
    const pcf = options.pcf ? parsePcf(options.pcf) : new Map<string, number>();
    const pinOf = (port: string, i: number, width: number): number | null => {
      const key = width === 1 ? port : `${port}[${i}]`;
      const placed = pcf.get(key) ?? (width === 1 ? pcf.get(`${port}[0]`) : undefined);
      if (placed !== undefined) return placed;
      if (width !== 1) return null;
      const m = /^gpio_(\d+)$/.exec(port);
      if (m) return Number(m[1]);
      return NAMED_PINS[port] ?? null;
    };
    const ioNetPin = new Map<number, number>();
    for (const [port, p] of Object.entries(top.ports)) {
      p.bits.forEach((b, i) => {
        if (typeof b !== "number") return;
        const label = p.bits.length === 1 ? port : `${port}[${i}]`;
        // An RGB driver's output is its fixed pad whatever the port is called.
        if (rgbNets.has(b)) return;
        const pin = pinOf(port, i, p.bits.length);
        if (pin === null) throw new FpgaError(`port ${label} has no pin: name it gpio_<pin> or place it in a PCF`);
        if (!UPDUINO_PINS.includes(pin)) throw new FpgaError(`port ${label} is on pin ${pin}, which the UPduino does not bring out`);
        if (ioPortBits.has(b)) {
          ioNetPin.set(b, pin);
          return;
        }
        if (this.pads.has(pin)) throw new FpgaError(`pin ${pin} is placed twice (port ${label})`);
        this.pads.set(pin, {
          input: p.direction === "output" ? -1 : b,
          output: p.direction === "input" ? -1 : b,
          enable: p.direction === "input" ? -2 : -1,
          pullup: false,
          rgb: false,
        });
      });
    }
    for (const io of ioCells) {
      const pin = io.pin < 0 ? ioNetPin.get(-io.pin) : io.pin;
      if (pin === undefined) throw new FpgaError("an SB_IO's PACKAGE_PIN is not a top-level port");
      if (this.pads.has(pin)) throw new FpgaError(`pin ${pin} is placed twice`);
      this.pads.set(pin, { input: io.input, output: io.output, enable: io.enable, pullup: io.pullup, rgb: io.pin > 0 });
    }
    for (const { pin, hz } of options.boardClocks ?? []) {
      const pad = this.pads.get(pin);
      if (pad && pad.input >= 0) this.clocks.push({ net: pad.input, half: 0.5 / hz, next: 0.5 / hz });
    }

    // Levelize the combinational cells: a cell's level is one past its deepest input's driver.
    this.opCount = ops.length;
    this.ops = new Int32Array(ops.length * 6);
    this.opInit = Int32Array.from(inits);
    ops.forEach((o, i) => this.ops.set(o, i * 6));
    const driverOf = new Int32Array(maxNet + 1).fill(-1);
    ops.forEach((o, i) => {
      if (driverOf[o[1]!] !== -1) throw new FpgaError(`net ${o[1]} has more than one driver`);
      driverOf[o[1]!] = i;
    });
    const inputsOf = (i: number) => {
      const o = ops[i]!;
      return o[0] === Op.Lut ? [o[2]!, o[3]!, o[4]!, o[5]!] : o[0] === Op.Buf ? [o[2]!] : [o[2]!, o[3]!, ...(o[0] === Op.Carry ? [o[4]!] : [])];
    };
    this.opLevel = new Int32Array(ops.length).fill(-1);
    const state = new Uint8Array(ops.length);
    const levelOf = (start: number): number => {
      // Iterative depth-first walk, so a long carry chain does not overflow the stack.
      const stack = [start];
      while (stack.length) {
        const i = stack[stack.length - 1]!;
        if (state[i] === 0) {
          state[i] = 1;
          for (const n of inputsOf(i)) {
            const d = driverOf[n]!;
            if (d < 0 || state[d] === 2) continue;
            if (state[d] === 1) throw new FpgaError("the design has a combinational loop (logic feeding back to itself without a flip-flop)");
            stack.push(d);
          }
        } else {
          stack.pop();
          if (state[i] === 2) continue;
          let level = 0;
          for (const n of inputsOf(i)) {
            const d = driverOf[n]!;
            if (d >= 0) level = Math.max(level, this.opLevel[d]! + 1);
          }
          this.opLevel[i] = level;
          state[i] = 2;
        }
      }
      return this.opLevel[start]!;
    };
    let levels = 0;
    for (let i = 0; i < ops.length; i++) levels = Math.max(levels, levelOf(i) + 1);
    this.levels = levels;
    const fan: number[][] = Array.from({ length: maxNet + 1 }, () => []);
    ops.forEach((_, i) => {
      for (const n of new Set(inputsOf(i))) if (n > 1) fan[n]!.push(i);
    });
    this.fanout = fan.map((f) => Int32Array.from(f));
    this.dirty = new Uint8Array(ops.length);
    this.buckets = Array.from({ length: levels }, () => []);

    const nf = this.flops.length;
    this.fD = Int32Array.from(this.flops, (f) => f.d);
    this.fQ = Int32Array.from(this.flops, (f) => f.q);
    this.fEn = Int32Array.from(this.flops, (f) => f.en);
    this.fSr = Int32Array.from(this.flops, (f) => f.sr);
    this.fSrValue = Uint8Array.from(this.flops, (f) => f.srValue);
    this.fSrAsync = Uint8Array.from(this.flops, (f) => (f.srAsync ? 1 : 0));
    const rise: number[][] = [], fall: number[][] = [], async: number[][] = [];
    this.flops.forEach((f, i) => {
      if (f.clk > 1) {
        ((f.neg ? fall : rise)[f.clk] ??= []).push(i);
        this.isClock[f.clk] = 1;
      }
      if (f.srAsync && f.sr > 1) {
        (async[f.sr] ??= []).push(i);
        this.isAsync[f.sr] = 1;
      }
    });
    const lists = (l: number[][]) => Array.from({ length: maxNet + 1 }, (_, n) => (l[n] ? Int32Array.from(l[n]!) : null));
    this.riseOf = lists(rise);
    this.fallOf = lists(fall);
    this.asyncOf = lists(async);
    this.updQ = new Int32Array(Math.max(16, nf * 4));
    this.updV = new Uint8Array(this.updQ.length);
    // Pads with a pull-up read high until the circuit says otherwise; then everything settles from power-up.
    for (const pad of this.pads.values()) if (pad.input >= 0 && pad.pullup) this.nets[pad.input] = 1;
    for (let i = 0; i < ops.length; i++) this.markDirty(i);
    this.asyncTouched = this.asyncOf.flatMap((l, n) => (l ? [n] : []));
    this.settle();
  }

  /**
   * The configured design as flat arrays, for the Rust core's port (core/src/mcu/fpga.rs), which builds the same
   * fabric from them. ints: net count, cell count, levels, flip-flop count, pad count, clock count; then per cell its
   * op and five nets; each cell's LUT_INIT; each cell's level; per flip-flop D, Q, C, falling-edge, E, set/reset,
   * asynchronous, forced value; per pad (in pin order) pin, input, output, enable, pull-up, RGB sink; each clock's
   * net. floats: each clock's half period, in seconds.
   */
  image(): { ints: Int32Array; floats: Float64Array } {
    const pads = [...this.pads].sort((a, b) => a[0] - b[0]);
    const ints: number[] = [this.nets.length, this.opCount, this.levels, this.flops.length, pads.length, this.clocks.length];
    ints.push(...this.ops, ...this.opInit, ...this.opLevel);
    for (const f of this.flops) ints.push(f.d, f.q, f.clk, f.neg ? 1 : 0, f.en, f.sr, f.srAsync ? 1 : 0, f.srValue);
    for (const [pin, p] of pads) ints.push(pin, p.input, p.output, p.enable, p.pullup ? 1 : 0, p.rgb ? 1 : 0);
    for (const c of this.clocks) ints.push(c.net);
    return { ints: Int32Array.from(ints), floats: Float64Array.from(this.clocks, (c) => c.half) };
  }

  /** FNV-1a over the nets' levels, with the edge count: what the lock-step harness compares. */
  stateHash(): number {
    let h = 0x811c9dc5;
    for (let i = 0; i < this.nets.length; i++) h = Math.imul(h ^ this.nets[i]!, 0x01000193) >>> 0;
    return Math.imul(h ^ (this.edgeCount >>> 0), 0x01000193) >>> 0;
  }

  /** The package pins the design uses. */
  get pins(): number[] {
    return [...this.pads.keys()].sort((a, b) => a - b);
  }

  /**
   * The circuit's level at a package pin (`null` when nothing drives it: the pin's pull-up, if enabled, holds it
   * high, else it reads 0). Logic and any flip-flops it clocks respond at once.
   */
  setPin(pin: number, level: 0 | 1 | null): void {
    const pad = this.pads.get(pin);
    if (!pad || pad.input < 0) return;
    const v = level ?? (pad.pullup ? 1 : 0);
    if (this.nets[pad.input] === v) return;
    this.setNet(pad.input, v);
    this.settle();
  }

  /** The design's drive on a package pin: 0 or 1, or `null` while its output is off (or it has none). */
  pinDrive(pin: number): 0 | 1 | null {
    const pad = this.pads.get(pin);
    if (!pad || pad.output < 0 || pad.enable === -2) return null;
    if (pad.enable >= 0 && this.nets[pad.enable] === 0) return null;
    const v = this.nets[pad.output]! as 0 | 1;
    // The RGB pads are current sinks: on pulls the pad low, off leaves it open.
    if (pad.rgb) return v ? 0 : null;
    return v;
  }

  /** Whether a pin has an enabled pull-up. */
  pullup(pin: number): boolean {
    return this.pads.get(pin)?.pullup ?? false;
  }

  /** Runs the design `seconds` further, edge by edge of its clocks. */
  advance(seconds: number): void {
    const end = this.time + seconds;
    const clocks = this.clocks;
    if (clocks.length === 0) {
      this.time = end;
      return;
    }
    for (;;) {
      let next = Infinity;
      for (const c of clocks) if (c.next < next) next = c.next;
      if (next > end) break;
      this.time = next;
      for (const c of clocks) {
        if (c.next !== next) continue;
        this.setNet(c.net, this.nets[c.net] ? 0 : 1);
        c.next += c.half;
      }
      this.edgeCount++;
      this.settle();
    }
    this.time = end;
  }

  // The span of levels holding dirty cells, so an edge that disturbs no logic costs nothing to settle.
  private dirtyLo = 0;
  private dirtyHi = -1;

  private markDirty(i: number): void {
    if (this.dirty[i]) return;
    this.dirty[i] = 1;
    const level = this.opLevel[i]!;
    this.buckets[level]!.push(i);
    if (this.dirtyHi < this.dirtyLo) {
      this.dirtyLo = level;
      this.dirtyHi = level;
    } else if (level < this.dirtyLo) this.dirtyLo = level;
    else if (level > this.dirtyHi) this.dirtyHi = level;
  }

  private setNet(n: number, v: number): void {
    if (this.nets[n] === v) return;
    this.nets[n] = v;
    const fan = this.fanout[n]!;
    for (let k = 0; k < fan.length; k++) this.markDirty(fan[k]!);
    if (this.isClock[n]) this.edges.push(n);
    if (this.isAsync[n]) this.asyncTouched.push(n);
  }

  private propagate(): void {
    const nets = this.nets;
    const ops = this.ops;
    // Cells only ever dirty later levels while a level runs, so the span grows only upward.
    for (let level = this.dirtyLo; level <= this.dirtyHi; level++) {
      const bucket = this.buckets[level]!;
      for (let k = 0; k < bucket.length; k++) {
        const i = bucket[k]!;
        this.dirty[i] = 0;
        const b = i * 6;
        let v: number;
        switch (ops[b]) {
          case Op.Lut:
            v = (this.opInit[i]! >> (nets[ops[b + 2]!]! | (nets[ops[b + 3]!]! << 1) | (nets[ops[b + 4]!]! << 2) | (nets[ops[b + 5]!]! << 3))) & 1;
            break;
          case Op.Carry: {
            const a = nets[ops[b + 2]!]!, c = nets[ops[b + 3]!]!, ci = nets[ops[b + 4]!]!;
            v = (a & c) | ((a | c) & ci);
            break;
          }
          case Op.Buf:
            v = nets[ops[b + 2]!]!;
            break;
          default:
            v = nets[ops[b + 3]!] ? nets[ops[b + 2]!]! : 0;
        }
        this.setNet(ops[b + 1]!, v);
      }
      bucket.length = 0;
    }
    this.dirtyLo = 0;
    this.dirtyHi = -1;
  }

  /** Settles logic, then runs the flip-flops whose clocks moved, until nothing changes. */
  private settle(): void {
    const nets = this.nets;
    const fD = this.fD, fQ = this.fQ, fEn = this.fEn, fSr = this.fSr, fSrValue = this.fSrValue, fSrAsync = this.fSrAsync;
    for (let round = 0; ; round++) {
      if (round > MAX_ROUNDS) throw new FpgaError("the design does not settle: it oscillates through its flip-flops' clocks or resets");
      this.propagate();
      const edges = this.edges;
      const touched = this.asyncTouched;
      if (edges.length === 0 && touched.length === 0) return;
      // Sample every flip-flop clocked in this instant before any of them changes.
      let count = 0;
      for (let e = 0; e < edges.length; e++) {
        const n = edges[e]!;
        const list = nets[n] ? this.riseOf[n] : this.fallOf[n];
        if (!list) continue;
        if (count + list.length > this.updQ.length) this.growUpdates(count + list.length);
        for (let k = 0; k < list.length; k++) {
          const f = list[k]!;
          const sr = fSr[f]!;
          if (sr >= 0 && fSrAsync[f] && nets[sr]) continue;
          if (!nets[fEn[f]!]) continue;
          this.updQ[count] = fQ[f]!;
          this.updV[count++] = sr >= 0 && !fSrAsync[f] && nets[sr] ? fSrValue[f]! : nets[fD[f]!]!;
        }
      }
      for (let e = 0; e < touched.length; e++) {
        const n = touched[e]!;
        const list = this.asyncOf[n];
        if (!list || !nets[n]) continue;
        if (count + list.length > this.updQ.length) this.growUpdates(count + list.length);
        for (let k = 0; k < list.length; k++) {
          this.updQ[count] = fQ[list[k]!]!;
          this.updV[count++] = fSrValue[list[k]!]!;
        }
      }
      edges.length = 0;
      touched.length = 0;
      for (let k = 0; k < count; k++) this.setNet(this.updQ[k]!, this.updV[k]!);
    }
  }

  private growUpdates(size: number): void {
    const q = new Int32Array(size * 2);
    const v = new Uint8Array(size * 2);
    q.set(this.updQ);
    v.set(this.updV);
    this.updQ = q;
    this.updV = v;
  }
}
