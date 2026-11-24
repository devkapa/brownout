/**
 * An iCE40 UltraPlus (UP5K) running a synthesized design, as on the UPduino v3.1: the netlist Yosys writes for
 * `synth_ice40` (`write_json`), mapped to the board's package pins by port name (`gpio_N`) or a PCF, and simulated
 * cell by cell. The fabric's primitives are the iCE40 technology library's (Lattice "iCE Technology Library",
 * rev 3.0): 4-input LUTs, the carry chain, D flip-flops with clock enable and synchronous or asynchronous set and
 * reset on either clock edge, global buffers, I/O cells with output enable and pull-up, the 48 MHz and 10 kHz
 * oscillators and the RGB LED current-sink driver.
 *
 * Every flip-flop powers up 0, as the device's do; Yosys realizes other initial values with inverters around them.
 * Power-up is not an edge: a clock net that comes up high from logic or a pulled-up pad has not risen.
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
  /** `top` is set (to a binary 1) on the module Yosys chose as the top of the hierarchy. */
  attributes?: Record<string, string>;
  /** A bus's bits, least significant first; `offset` is its lowest index and `upto` marks a `[lo:hi]` declaration. */
  ports: Record<string, { direction: "input" | "output" | "inout"; bits: (number | string)[]; offset?: number; upto?: number }>;
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
// Port names the UPduino's own examples use for fixed pins. The SPI names are the board's own PCF's
// (tinyvision-ai-inc/UPduino-v3.0, RTL/common/upduino.pcf: spi_miso 14, spi_sck 15, spi_ssn 16, spi_mosi 17, named
// for the FT232H's MPSSE engine as the bus master), and its pinout page gives the same; a design written against
// that PCF lands on the same pins here.
const NAMED_PINS: Record<string, number> = {
  led_green: 39, led_blue: 40, led_red: 41, spi_miso: 14, spi_sck: 15, spi_ssn: 16, spi_mosi: 17,
};

const HFOSC_HZ = 48e6;
const LFOSC_HZ = 10e3;
// Settling rounds within one instant, beyond one per flip-flop (a ripple chain through every flip-flop settles in
// that many), before the design is taken to oscillate (a combinational loop through a flip-flop's clock or
// asynchronous reset, say).
const EXTRA_ROUNDS = 64;

const enum Op {
  Lut,
  Carry,
  Buf,
  /** out = sel ? a : b, as [Mux, out, sel, a, b]: a bidirectional pad's input reads its own drive while enabled. */
  Mux,
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
  /**
   * The net the circuit's level at the pad sets, or -1. On a pad that can also drive, this is the pad's own net,
   * which the design reads through a Mux that gives its own drive while the output is enabled.
   */
  input: number;
  /** The net the design drives the pad from, or -1. */
  output: number;
  /** The output enable: a net, or -1 for always (with an output), or -2 for never. */
  enable: number;
  /** The pad's pull-up, in ohms, or 0 for none. */
  pullup: number;
  /**
   * Driven by SB_RGBA_DRV: the current it sinks while its net is 1, in microamps (else high impedance); 0 for any
   * other pad.
   */
  rgb: number;
}

// The RGB driver's sink current per step of RGBn_CURRENT (a thermometer code, 0b000001 to 0b111111): 4 mA in full
// current mode, 2 mA in half (Lattice TN1288, SB_RGBA_DRV attributes).
const RGB_STEP_UA = { full: 4000, half: 2000 };

interface Clock {
  net: number;
  /** While this net is 0 the clock holds low; its phase runs on, as the oscillator's does behind its output gate. */
  en: number;
  half: number;
  next: number;
}

/**
 * The iCE40's default (nominally 100 kOhm) I/O pull-up: iCE40 UltraPlus Family Data Sheet (FPGA-DS-02008 2.0),
 * Table 4.5, IPU, sources 11 to 128 uA into a pin at 0 V with VCCIO 3.3 V; modelled as the resistor that sources
 * their geometric mean, 37.5 uA.
 */
export const ICE40_WEAK_PULLUP_OHMS = 88_000;
// The strengths a UP5K pad's pull-up can be set to, by the names nextpnr's -pullup_resistor takes; 100K is the
// default one, characterised above.
const PULLUP_RESISTORS: Record<string, number> = { "3P3K": 3300, "6P8K": 6800, "10K": 10_000, "100K": ICE40_WEAK_PULLUP_OHMS };

function param(value: unknown, fallback: number): number {
  // write_json -compat-int writes small parameters and attributes as JSON numbers.
  if (typeof value === "number") return Number.isFinite(value) ? value : fallback;
  if (typeof value !== "string") return fallback;
  const v = value.trim();
  if (/^0b[01]+$/i.test(v)) return parseInt(v.slice(2), 2);
  if (/^[01]+$/.test(v)) return parseInt(v, 2);
  const n = Number(v);
  return Number.isFinite(n) ? n : fallback;
}

/**
 * A PCF as nextpnr reads it for the iCE40: `set_io [-nowarn] [-pullup yes|no] [-pullup_resistor R] <port> <pin>`
 * places a port bit (`led`, `bus[3]`) and can turn on its pad's pull-up, at one of the UP5K's strengths (3P3K, 6P8K,
 * 10K or the default 100K); `set_frequency` lines only steer timing.
 */
export function readPcf(text: string): {
  pins: Map<string, number>;
  pullups: Set<string>;
  /** The strength of each pull-up the PCF turns on. */
  pullupOhms: Map<string, number>;
  /** Each -pullup_resistor the PCF gives, whether or not it turns the pull-up on (the design's SB_IO may). */
  pullupStrength: Map<string, number>;
} {
  const pins = new Map<string, number>();
  const pullups = new Set<string>();
  const pullupOhms = new Map<string, number>();
  const pullupStrength = new Map<string, number>();
  text.split(/\r?\n/).forEach((raw, i) => {
    const line = raw.replace(/#.*/, "").trim();
    if (!line) return;
    const words = line.split(/\s+/);
    if (words[0] === "set_frequency") return;
    if (words[0] !== "set_io") throw new FpgaError(`PCF line ${i + 1}: only set_io and set_frequency are understood, not "${words[0]}"`);
    const args: string[] = [];
    let pullup = false;
    let ohms: number | undefined;
    for (let j = 1; j < words.length; j++) {
      const w = words[j]!;
      if (w === "-pullup") pullup = /^(yes|1|true)$/i.test(words[++j] ?? "");
      else if (w === "-pullup_resistor") {
        const r = (words[++j] ?? "").toUpperCase();
        const v = PULLUP_RESISTORS[r];
        if (v === undefined) throw new FpgaError(`PCF line ${i + 1}: the UP5K's pull-ups are 3P3K, 6P8K, 10K or 100K, not "${words[j] ?? ""}"`);
        ohms = v;
      } else if (!w.startsWith("-")) args.push(w);
    }
    if (args.length !== 2 || !/^\d+$/.test(args[1]!)) throw new FpgaError(`PCF line ${i + 1}: expected "set_io <port> <pin>"`);
    pins.set(args[0]!, Number(args[1]));
    if (ohms !== undefined) pullupStrength.set(args[0]!, ohms);
    if (pullup) {
      pullups.add(args[0]!);
      pullupOhms.set(args[0]!, ohms ?? ICE40_WEAK_PULLUP_OHMS);
    }
  });
  return { pins, pullups, pullupOhms, pullupStrength };
}

/** `set_io` lines of a PCF: port bit name to package pin. */
export function parsePcf(text: string): Map<string, number> {
  return readPcf(text).pins;
}

const DIRECTIONS = new Set(["input", "output", "inout"]);

/**
 * The module Yosys marked top (write_json also lists its cell library's models), checked for the shape read here,
 * with its nets renumbered 2, 3, ... in the order of their numbers, so the fabric's arrays are as long as the nets the
 * design uses whatever numbers a netlist gives them.
 */
function topModule(netlist: YosysJson | null | undefined): YosysModule {
  const modules = netlist && typeof netlist === "object" ? netlist.modules : undefined;
  if (!modules || typeof modules !== "object") throw new FpgaError("not a Yosys netlist: it has no modules (expected Yosys write_json output)");
  const all = Object.values(modules);
  const top = all.find((m) => m && typeof m === "object" && param(m.attributes?.top, 0) === 1) ?? modules.top ?? all[0];
  if (!top) throw new FpgaError("the netlist has no module");
  const bad = (what: string) => new FpgaError(`not a Yosys netlist: ${what}`);
  if (!top.ports || typeof top.ports !== "object") throw bad("its top module has no ports");
  if (!top.cells || typeof top.cells !== "object") throw bad("its top module has no cells");
  const validBits = (bits: unknown[]) =>
    bits.every((b) => (typeof b === "number" ? Number.isSafeInteger(b) && b >= 2 : b === "0" || b === "1" || b === "x" || b === "z"));
  for (const [name, p] of Object.entries(top.ports)) {
    if (!p || !DIRECTIONS.has(p.direction) || !Array.isArray(p.bits)) throw bad(`port ${name} has no direction or bits`);
    if (!validBits(p.bits)) throw bad(`port ${name} has a bit that is not a net number or a constant`);
  }
  for (const [name, c] of Object.entries(top.cells)) {
    if (!c || typeof c.type !== "string" || !c.connections || typeof c.connections !== "object") throw bad(`cell ${name} has no type or connections`);
    for (const bits of Object.values(c.connections)) {
      if (!Array.isArray(bits) || !validBits(bits)) throw bad(`cell ${name} has a connection that is not a list of net numbers and constants`);
    }
  }
  const used = new Set<number>();
  const note = (bits: (number | string)[]) => {
    for (const b of bits) if (typeof b === "number") used.add(b);
  };
  for (const p of Object.values(top.ports)) note(p.bits);
  for (const c of Object.values(top.cells)) for (const bits of Object.values(c.connections)) note(bits);
  const dense = new Map<number, number>();
  [...used].sort((a, b) => a - b).forEach((n, i) => dense.set(n, i + 2));
  const renumber = (bits: (number | string)[]) => bits.map((b) => (typeof b === "number" ? dense.get(b)! : b));
  return {
    ...top,
    ports: Object.fromEntries(Object.entries(top.ports).map(([name, p]) => [name, { ...p, bits: renumber(p.bits) }])),
    cells: Object.fromEntries(
      Object.entries(top.cells).map(([name, c]) => [
        name,
        { ...c, connections: Object.fromEntries(Object.entries(c.connections).map(([port, bits]) => [port, renumber(bits)])) },
      ]),
    ),
  };
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
  private readonly maxRounds: number;
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
    const top = topModule(netlist);
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
    // Nets the fabric adds: a bidirectional pad's own level, an oscillator's enable.
    let netCount = maxNet + 1;
    const fresh = () => netCount++;

    const ops: number[][] = [];
    const inits: number[] = [];
    const unsupported = new Set<string>();
    const rgbNets = new Set<number>();
    const tbufs = new Map<number, { a: number; e: number }>();
    const ioCells: { pin: number; input: number; output: number; enable: number; pullup: number; rgb?: number }[] = [];
    const ioPortBits = new Set<number>();
    const conn = (c: YosysModule["cells"][string], port: string, fallback = 0) => {
      const b = c.connections[port];
      return b && b.length ? bit(b[0]) : fallback;
    };
    // A cell's output: left unconnected (Yosys writes []), it drives a net of its own that nothing reads, never a
    // constant.
    const out = (c: YosysModule["cells"][string], port: string) => {
      const b = c.connections[port]?.[0];
      return typeof b === "number" ? bit(b) : fresh();
    };
    const connected = (c: YosysModule["cells"][string], port: string) => typeof c.connections[port]?.[0] === "number";
    for (const [name, c] of Object.entries(top.cells)) {
      const p = c.parameters ?? {};
      const t = c.type;
      if (t === "SB_LUT4") {
        ops.push([Op.Lut, out(c, "O"), conn(c, "I0"), conn(c, "I1"), conn(c, "I2"), conn(c, "I3")]);
        inits.push(param(p.LUT_INIT, 0));
      } else if (t === "SB_CARRY") {
        ops.push([Op.Carry, out(c, "CO"), conn(c, "I0"), conn(c, "I1"), conn(c, "CI"), 0]);
        inits.push(0);
      } else if (t === "SB_GB") {
        ops.push([Op.Buf, out(c, "GLOBAL_BUFFER_OUTPUT"), conn(c, "USER_SIGNAL_TO_GLOBAL_BUFFER"), 0, 0, 0]);
        inits.push(0);
      } else if (t === "$_TBUF_") {
        // A tristate on a port becomes that pad's output and enable; see the pads below.
        const y = c.connections.Y?.[0];
        if (typeof y !== "number") unsupported.add(`${t} ${name} driving a constant`);
        else tbufs.set(y, { a: conn(c, "A"), e: conn(c, "E") });
      } else if (/^SB_DFFN?E?(SR|R|SS|S)?$/.test(t)) {
        const m = /^SB_DFF(N?)(E?)(SR|R|SS|S)?$/.exec(t)!;
        const kind = m[3] ?? "";
        this.flops.push({
          d: conn(c, "D"),
          q: out(c, "Q"),
          clk: conn(c, "C"),
          neg: m[1] === "N",
          en: m[2] ? conn(c, "E", 1) : 1,
          sr: kind ? conn(c, kind.startsWith("S") && kind !== "SR" ? "S" : "R") : -1,
          srAsync: kind === "R" || kind === "S",
          srValue: kind === "S" || kind === "SS" ? 1 : 0,
        });
      } else if (t === "SB_HFOSC" || t === "SB_LFOSC") {
        const hf = t === "SB_HFOSC";
        const hz = hf ? HFOSC_HZ / (1 << (param(p.CLKHF_DIV, 0) & 3)) : LFOSC_HZ;
        // The output runs while both its power-up and its enable are high, either of which logic may drive.
        const en = fresh();
        ops.push([Op.Lut, en, conn(c, hf ? "CLKHFEN" : "CLKLFEN", 1), conn(c, hf ? "CLKHFPU" : "CLKLFPU", 1), 0, 0]);
        inits.push(0b1000);
        this.clocks.push({ net: out(c, hf ? "CLKHF" : "CLKLF"), en, half: 0.5 / hz, next: 0.5 / hz });
      } else if (t === "SB_RGBA_DRV" || t === "SB_RGB_DRV") {
        ["RGB0", "RGB1", "RGB2"].forEach((pad, i) => {
          const out = c.connections[pad]?.[0];
          if (typeof out !== "number") return;
          rgbNets.add(out);
          // A channel sinks its set current while its PWM, CURREN and RGBLEDEN are all high. With no current set
          // (the parameter's default, 0b000000) the driver leaves its pad alone, for an SB_IO_OD to use.
          let steps = param(p[`${pad}_CURRENT`], 0) & 0b111111;
          let ua = 0;
          for (; steps; steps >>= 1) ua += steps & 1 ? (param(p.CURRENT_MODE, 0) & 1 ? RGB_STEP_UA.half : RGB_STEP_UA.full) : 0;
          if (ua === 0) {
            ops.push([Op.Buf, out, 0, 0, 0, 0]);
            inits.push(0);
            return;
          }
          ops.push([Op.Lut, out, conn(c, `${pad}PWM`), conn(c, "CURREN", 1), conn(c, "RGBLEDEN", 1), 0]);
          inits.push(0b1000_0000);
          ioCells.push({ pin: RGB_PADS[i]!, input: -1, output: out, enable: -1, pullup: 0, rgb: ua });
        });
      } else if (t === "SB_IO") {
        const pkg = c.connections.PACKAGE_PIN?.[0];
        if (typeof pkg !== "number") throw new FpgaError(`SB_IO ${name} has no PACKAGE_PIN`);
        const type = param(p.PIN_TYPE, 0);
        const inMode = type & 3;
        const outData = (type >> 2) & 3;
        const outEnable = (type >> 4) & 3;
        if (inMode !== 1 && inMode !== 0) unsupported.add(`SB_IO ${name}: latched input (PIN_TYPE ${p.PIN_TYPE})`);
        if (inMode === 0 && connected(c, "D_IN_0")) unsupported.add(`SB_IO ${name}: registered input (PIN_TYPE ${p.PIN_TYPE})`);
        if (typeof c.connections.D_IN_1?.[0] === "number") unsupported.add(`SB_IO ${name}: DDR input (D_IN_1)`);
        if (String(p.IO_STANDARD ?? "").includes("LVDS")) unsupported.add(`SB_IO ${name}: LVDS input`);
        if (outEnable !== 0 && outData !== 2) unsupported.add(`SB_IO ${name}: registered or DDR output (PIN_TYPE ${p.PIN_TYPE})`);
        if (outEnable === 3) unsupported.add(`SB_IO ${name}: registered output enable (PIN_TYPE ${p.PIN_TYPE})`);
        ioPortBits.add(pkg);
        ioCells.push({
          pin: -pkg,
          input: connected(c, "D_IN_0") ? conn(c, "D_IN_0") : -1,
          output: outEnable === 0 ? -1 : conn(c, "D_OUT_0"),
          enable: outEnable === 0 ? -2 : outEnable === 1 ? -1 : conn(c, "OUTPUT_ENABLE", 1),
          pullup: param(p.PULLUP, 0) === 1 ? ICE40_WEAK_PULLUP_OHMS : 0,
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
    // What drives each net, so an inout port with no tristate is known for an output or an input: cells, the input
    // ports, and the SB_IO inputs.
    const driven = new Set<number>();
    for (const p of Object.values(top.ports)) if (p.direction === "input") for (const b of p.bits) if (typeof b === "number") driven.add(b);
    for (const io of ioCells) if (io.input >= 0) driven.add(io.input);
    for (const o of ops) driven.add(o[1]!);
    for (const f of this.flops) driven.add(f.q);
    for (const c of this.clocks) driven.add(c.net);
    for (const t of tbufs.keys()) driven.add(t);
    // A tristate net on several ports belongs to one pad (an inout before an output); the rest read it.
    const tbufOwner = new Map<number, { key: string; inout: boolean }>();
    for (const [port, p] of Object.entries(top.ports)) {
      p.bits.forEach((b, i) => {
        if (typeof b !== "number" || !tbufs.has(b)) return;
        const inout = p.direction === "inout";
        if (!tbufOwner.has(b) || (inout && !tbufOwner.get(b)!.inout)) tbufOwner.set(b, { key: `${port}[${i}]`, inout });
      });
    }
    // A bidirectional pad: the design reads its own drive while enabled and the circuit's level otherwise.
    const bidirectional = (read: number, enable: number, drive: number): number => {
      const ext = fresh();
      ops.push([Op.Mux, read, enable, drive, ext, 0]);
      inits.push(0);
      return ext;
    };

    // Package pins: the PCF first, then names.
    const pcf = options.pcf
      ? readPcf(options.pcf)
      : { pins: new Map<string, number>(), pullupOhms: new Map<string, number>(), pullupStrength: new Map<string, number>() };
    type Place = { pin: number; pullup: number; strength?: number };
    const placeOf = (port: string, index: number, width: number): Place | null => {
      const keys = width === 1 ? [port, `${port}[${index}]`] : [`${port}[${index}]`];
      for (const key of keys) {
        const pin = pcf.pins.get(key);
        if (pin !== undefined) return { pin, pullup: pcf.pullupOhms.get(key) ?? 0, strength: pcf.pullupStrength.get(key) };
      }
      if (width !== 1) return null;
      const m = /^gpio_(\d+)$/.exec(port);
      const pin = m ? Number(m[1]) : NAMED_PINS[port];
      return pin === undefined ? null : { pin, pullup: 0 };
    };
    const ioNetPin = new Map<number, Place>();
    for (const [port, p] of Object.entries(top.ports)) {
      p.bits.forEach((b, i) => {
        // An undriven (x) or released (z) output bit leaves its pin alone; a constant input bit reads nothing.
        if (typeof b !== "number" && (p.direction === "input" || (b !== "0" && b !== "1"))) return;
        // An RGB driver's output is its fixed pad whatever the port is called.
        if (typeof b === "number" && rgbNets.has(b)) return;
        // Bits are stored least significant first; their names are the declared indices.
        const index = (p.offset ?? 0) + (p.upto ? p.bits.length - 1 - i : i);
        const label = p.bits.length === 1 ? port : `${port}[${index}]`;
        const place = placeOf(port, index, p.bits.length);
        if (place === null) throw new FpgaError(`port ${label} has no pin: name it gpio_<pin> or place it in a PCF`);
        const { pin, pullup } = place;
        if (!UPDUINO_PINS.includes(pin)) throw new FpgaError(`port ${label} is on pin ${pin}, which the UPduino does not bring out`);
        if (typeof b === "number" && ioPortBits.has(b)) {
          ioNetPin.set(b, place);
          return;
        }
        if (this.pads.has(pin)) throw new FpgaError(`pin ${pin} is placed twice (port ${label})`);
        let pad: PadBinding;
        if (typeof b !== "number") {
          pad = { input: -1, output: b === "1" ? 1 : 0, enable: -1, pullup, rgb: 0 };
        } else if (tbufOwner.get(b)?.key === `${port}[${i}]`) {
          const t = tbufs.get(b)!;
          pad = { input: bidirectional(b, t.e, t.a), output: t.a, enable: t.e, pullup, rgb: 0 };
        } else if (p.direction === "input" || (p.direction === "inout" && !driven.has(b))) {
          pad = { input: b, output: -1, enable: -2, pullup, rgb: 0 };
        } else {
          pad = { input: -1, output: b, enable: -1, pullup, rgb: 0 };
        }
        // The RGB pads have no pull-up (TN1288).
        if ((RGB_PADS as readonly number[]).includes(pin)) pad.pullup = 0;
        this.pads.set(pin, pad);
      });
    }
    // A tristate inside the design, on no port, reads 0 while released.
    for (const [y, t] of tbufs) {
      if (tbufOwner.has(y)) continue;
      ops.push([Op.Mux, y, t.e, t.a, 0, 0]);
      inits.push(0);
    }
    for (const io of ioCells) {
      const placed: Place | undefined = io.pin < 0 ? ioNetPin.get(-io.pin) : { pin: io.pin, pullup: 0 };
      if (placed === undefined) throw new FpgaError("an SB_IO's PACKAGE_PIN is not a top-level port");
      const pin = placed.pin;
      if (this.pads.has(pin)) throw new FpgaError(`pin ${pin} is placed twice`);
      let input = io.input;
      if (input >= 0 && io.output >= 0 && io.enable !== -2) {
        if (io.enable === -1) {
          // Always driving: the input reads the pad's own drive.
          ops.push([Op.Buf, input, io.output, 0, 0, 0]);
          inits.push(0);
          input = -1;
        } else {
          input = bidirectional(input, io.enable, io.output);
        }
      }
      // The pull-up is on if the PCF or the SB_IO's PULLUP turns it on, at the PCF's strength if it gives one (the
      // SB_IO's own is the default); the RGB pads have none.
      const pullup = (RGB_PADS as readonly number[]).includes(pin) ? 0 : placed.pullup || (io.pullup && (placed.strength ?? io.pullup));
      this.pads.set(pin, { input, output: io.output, enable: io.enable, pullup, rgb: io.rgb ?? 0 });
    }
    for (const { pin, hz } of options.boardClocks ?? []) {
      const pad = this.pads.get(pin);
      if (!pad) continue;
      if (pad.output >= 0 && pad.enable !== -2) {
        throw new FpgaError(`the design drives pin ${pin}, which the board's ${hz / 1e6} MHz oscillator drives while the OSC jumper is fitted: remove the jumper or use another pin`);
      }
      if (pad.input >= 0) this.clocks.push({ net: pad.input, en: 1, half: 0.5 / hz, next: 0.5 / hz });
    }

    this.nets = new Uint8Array(netCount);
    this.nets[1] = 1;
    this.isClock = new Uint8Array(netCount);
    this.isAsync = new Uint8Array(netCount);

    // Levelize the combinational cells: a cell's level is one past its deepest input's driver.
    this.opCount = ops.length;
    this.ops = new Int32Array(ops.length * 6);
    this.opInit = Int32Array.from(inits);
    ops.forEach((o, i) => this.ops.set(o, i * 6));
    const driverOf = new Int32Array(netCount).fill(-1);
    ops.forEach((o, i) => {
      if (driverOf[o[1]!] !== -1) throw new FpgaError(`net ${o[1]} has more than one driver`);
      driverOf[o[1]!] = i;
    });
    const inputsOf = (i: number) => {
      const o = ops[i]!;
      return o[0] === Op.Lut ? [o[2]!, o[3]!, o[4]!, o[5]!] : o[0] === Op.Buf ? [o[2]!] : o[0] === Op.Carry || o[0] === Op.Mux ? [o[2]!, o[3]!, o[4]!] : [];
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
    const fan: number[][] = Array.from({ length: netCount }, () => []);
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
      // A set or reset tied high (net 1) holds its flip-flop from power-up.
      if (f.srAsync && f.sr >= 1) {
        (async[f.sr] ??= []).push(i);
        this.isAsync[f.sr] = 1;
      }
    });
    const lists = (l: number[][]) => Array.from({ length: netCount }, (_, n) => (l[n] ? Int32Array.from(l[n]!) : null));
    this.riseOf = lists(rise);
    this.fallOf = lists(fall);
    this.asyncOf = lists(async);
    this.updQ = new Int32Array(Math.max(16, nf * 4));
    this.updV = new Uint8Array(this.updQ.length);
    this.maxRounds = EXTRA_ROUNDS + nf;
    // Pads with a pull-up read high until the circuit says otherwise. Logic takes its power-up levels without any
    // flip-flop seeing an edge in them; asynchronous sets and resets then hold their flip-flops from the start.
    // A pad the board's oscillator drives starts low, its first edge rising, whatever its pull-up.
    const clockNets = new Set(this.clocks.map((c) => c.net));
    for (const pad of this.pads.values()) if (pad.input >= 0 && pad.pullup && !clockNets.has(pad.input)) this.nets[pad.input] = 1;
    for (let i = 0; i < ops.length; i++) this.markDirty(i);
    this.propagate();
    this.edges.length = 0;
    this.asyncTouched = this.asyncOf.flatMap((l, n) => (l ? [n] : []));
    this.settle();
  }

  /**
   * The configured design as flat arrays, for the Rust core's port (core/src/mcu/fpga.rs), which builds the same
   * fabric from them. ints: net count, cell count, levels, flip-flop count, pad count, clock count; then per cell its
   * op and five nets; each cell's LUT_INIT; each cell's level; per flip-flop D, Q, C, falling-edge, E, set/reset,
   * asynchronous, forced value; per pad (in pin order) pin, input, output, enable, pull-up in ohms (0: none), RGB sink current in microamps (0: not an
   * RGB driver pad); per clock its
   * net and enable net. floats: each clock's half period, in seconds. Ops are 0 LUT, 1 carry, 2 buffer, 3 mux.
   */
  image(): { ints: Int32Array; floats: Float64Array } {
    const pads = [...this.pads].sort((a, b) => a[0] - b[0]);
    const ints: number[] = [this.nets.length, this.opCount, this.levels, this.flops.length, pads.length, this.clocks.length];
    ints.push(...this.ops, ...this.opInit, ...this.opLevel);
    for (const f of this.flops) ints.push(f.d, f.q, f.clk, f.neg ? 1 : 0, f.en, f.sr, f.srAsync ? 1 : 0, f.srValue);
    for (const [pin, p] of pads) ints.push(pin, p.input, p.output, p.enable, p.pullup, p.rgb);
    for (const c of this.clocks) ints.push(c.net, c.en);
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
   * high, else it reads 0). Logic and any flip-flops it clocks respond at once. A pin the board's oscillator drives
   * keeps the oscillator's level.
   */
  setPin(pin: number, level: 0 | 1 | null): void {
    const pad = this.pads.get(pin);
    if (!pad || pad.input < 0) return;
    for (const c of this.clocks) if (c.net === pad.input) return;
    const v = level ?? (pad.pullup ? 1 : 0);
    if (this.nets[pad.input] === v) return;
    this.setNet(pad.input, v);
    this.settle();
  }

  /**
   * The circuit's level at a package pin while the device configures: logic takes it as its power-up level, so no
   * flip-flop sees an edge in it (an input already high when the design starts has not risen).
   */
  presetPin(pin: number, level: 0 | 1 | null): void {
    const pad = this.pads.get(pin);
    if (!pad || pad.input < 0) return;
    for (const c of this.clocks) if (c.net === pad.input) return;
    this.setNet(pad.input, level ?? (pad.pullup ? 1 : 0));
    this.propagate();
    this.edges.length = 0;
    this.settle();
  }

  /** The design's drive on a package pin: 0 or 1, or `null` while its output is off (or it has none). */
  pinDrive(pin: number): 0 | 1 | null {
    const pad = this.pads.get(pin);
    if (!pad || pad.output < 0 || pad.enable === -2) return null;
    if (pad.enable >= 0 && this.nets[pad.enable] === 0) return null;
    const v = this.nets[pad.output]! as 0 | 1;
    // The RGB pads are current sinks: on pulls the pad low, off leaves it open. Used as plain I/O they are open-drain.
    if (pad.rgb) return v ? 0 : null;
    if (v && (RGB_PADS as readonly number[]).includes(pin)) return null;
    return v;
  }

  /** Whether a pin has an enabled pull-up. */
  pullup(pin: number): boolean {
    return !!this.pads.get(pin)?.pullup;
  }

  /** A pin's enabled pull-up, in ohms, or `null` for none. */
  pullupOhms(pin: number): number | null {
    return this.pads.get(pin)?.pullup || null;
  }

  /** The current an RGB driver pad sinks now, in amps: its set current while lit, else 0 (as for every other pad). */
  sinkCurrent(pin: number): number {
    const pad = this.pads.get(pin);
    return pad && pad.rgb && this.pinDrive(pin) === 0 ? pad.rgb * 1e-6 : 0;
  }

  /**
   * Runs the design `seconds` further, edge by edge of its clocks, running at most `maxEdges` edges. A step that
   * reaches the budget skips the rest of its edges to end on time (each clock rejoins its phase there) and returns
   * false, so a host stepping a fast design in long steps stays responsive and can say the design fell behind.
   */
  advance(seconds: number, maxEdges = Infinity): boolean {
    // A NaN or infinite step would never reach its end.
    if (!Number.isFinite(seconds) || seconds < 0) throw new RangeError(`the fabric cannot advance ${seconds} s`);
    const end = this.time + seconds;
    const clocks = this.clocks;
    if (clocks.length === 0) {
      this.time = end;
      return true;
    }
    const nets = this.nets;
    for (let ran = 0; ; ran++) {
      let next = Infinity;
      for (const c of clocks) {
        // A gated clock held low makes no edges; once enabled again it rejoins its phase.
        if (!nets[c.en] && !nets[c.net]) continue;
        if (c.next <= this.time) {
          c.next += (Math.floor((this.time - c.next) / c.half) + 1) * c.half;
          while (c.next <= this.time) c.next += c.half;
        }
        if (c.next < next) next = c.next;
      }
      if (next > end) break;
      if (ran >= maxEdges) {
        this.skipTo(end);
        return false;
      }
      this.time = next;
      for (const c of clocks) {
        if (c.next !== next || (!nets[c.en] && !nets[c.net])) continue;
        this.setNet(c.net, nets[c.en] && !nets[c.net] ? 1 : 0);
        c.next += c.half;
      }
      this.edgeCount++;
      this.settle();
    }
    this.time = end;
    return true;
  }

  /**
   * Ends a step at `end` without running its remaining edges: each running clock takes the level those edges would
   * have left it at (an odd number of them toggles it; a disabled clock that was high falls on the first and holds),
   * as a level, not an edge, so the clocks keep their phase and polarity and no flip-flop samples.
   */
  private skipTo(end: number): void {
    const nets = this.nets;
    const levels: [number, number][] = [];
    for (const c of this.clocks) {
      if ((!nets[c.en] && !nets[c.net]) || c.next > end) continue;
      const skipped = Math.floor((end - c.next) / c.half) + 1;
      const v = nets[c.en] ? (skipped & 1 ? 1 - nets[c.net]! : nets[c.net]!) : 0;
      levels.push([c.net, v]);
    }
    this.time = end;
    for (const [net, v] of levels) this.setNet(net, v);
    this.propagate();
    this.edges.length = 0;
    this.settle();
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
            v = nets[ops[b + 2]!] ? nets[ops[b + 3]!]! : nets[ops[b + 4]!]!;
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
      if (round > this.maxRounds) throw new FpgaError("the design does not settle: it oscillates through its flip-flops' clocks or resets");
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
