/**
 * The Bosch BMP280 pressure and temperature sensor (datasheet BST-BMP280-DS001 rev 1.26) on its I2C interface, as
 * the breakout modules carry it. A sketch reads it as it reads the chip: chip id 0x58 at D0h, the factory trimming
 * parameters at 88h-9Fh, ctrl_meas (F4h) and config (F5h) to choose oversampling, mode, standby and the IIR filter,
 * and the 20-bit raw readings at F7h-FCh, which the sketch compensates with the trimming parameters (section 8.2).
 *
 * The part holds an ambient temperature and pressure its host sets. Each conversion turns them into the raw readings
 * the datasheet's own compensation maps back to them, so a library computes the ambient values to the resolution the
 * oversampling gives (16 bits at x1 to 20 at x16) after the IIR filter. Conversions take the datasheet's typical
 * measurement time; forced mode makes one and returns to sleep, normal mode repeats after each standby period. A
 * burst read sees one conversion's readings (the data shadowing of section 3.10).
 *
 * Time comes from the board that talks to it (`clock`), so a conversion lasts exactly as long as the sketch waits.
 */
import type { I2cDevice } from "./i2c-devices.js";

/** The datasheet's example trimming parameters (section 8.2, table 6 footnote), as real parts carry similar ones. */
export const BMP280_TRIMMING = {
  T1: 27504, T2: 26435, T3: -1000,
  P1: 36477, P2: -10685, P3: 3024, P4: 2855, P5: 140, P6: -7, P7: 15500, P8: -14600, P9: 6000,
} as const;

const CHIP_ID = 0x58;
const RESET_WORD = 0xb6;
// The value a skipped measurement reads (section 4.3.6/4.3.7).
const SKIPPED = 0x80000;
// Oversampling setting to sample count; 5, 6 and 7 are all x16.
const OVERSAMPLING = [0, 1, 2, 4, 8, 16, 16, 16];
// config t_sb (section 3.6.3), in seconds.
const STANDBY = [0.0005, 0.0625, 0.125, 0.25, 0.5, 1, 2, 4];
// config filter: the IIR coefficient (section 3.3.3).
const FILTER = [1, 2, 4, 8, 16, 16, 16, 16];

/** Temperature compensation (section 8.2, 32-bit): t_fine, and the temperature in 0.01 degC. */
export function bmp280Temperature(adcT: number): { tFine: number; centi: number } {
  const { T1, T2, T3 } = BMP280_TRIMMING;
  const var1 = ((((adcT >> 3) - (T1 << 1))) * T2) >> 11;
  const d = (adcT >> 4) - T1;
  const var2 = (((d * d) >> 12) * T3) >> 14;
  const tFine = var1 + var2;
  return { tFine, centi: (tFine * 5 + 128) >> 8 };
}

/** Pressure compensation (section 8.2, 64-bit as Adafruit_BMP280 uses): pressure in Pa x 256 (Q24.8). */
export function bmp280Pressure(adcP: number, tFine: number): bigint {
  const P = BMP280_TRIMMING;
  let var1 = BigInt(tFine) - 128000n;
  let var2 = var1 * var1 * BigInt(P.P6);
  var2 += (var1 * BigInt(P.P5)) << 17n;
  var2 += BigInt(P.P4) << 35n;
  var1 = ((var1 * var1 * BigInt(P.P3)) >> 8n) + ((var1 * BigInt(P.P2)) << 12n);
  var1 = (((1n << 47n) + var1) * BigInt(P.P1)) >> 33n;
  if (var1 === 0n) return 0n;
  let p = 1048576n - BigInt(adcP);
  p = (((p << 31n) - var2) * 3125n) / var1;
  var1 = (BigInt(P.P9) * (p >> 13n) * (p >> 13n)) >> 25n;
  var2 = (BigInt(P.P8) * p) >> 19n;
  return ((p + var1 + var2) >> 8n) + (BigInt(P.P7) << 4n);
}

/** The 20-bit raw temperature whose compensation is nearest `celsius`; compensation rises with it. */
export function bmp280RawTemperature(celsius: number): number {
  const target = Math.round(celsius * 100);
  let lo = 0;
  let hi = (1 << 20) - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (bmp280Temperature(mid).centi < target) lo = mid + 1;
    else hi = mid;
  }
  const below = Math.max(0, lo - 1);
  return Math.abs(bmp280Temperature(below).centi - target) < Math.abs(bmp280Temperature(lo).centi - target) ? below : lo;
}

/** The 20-bit raw pressure whose compensation at `tFine` is nearest `pascals`; compensation falls as it rises. */
export function bmp280RawPressure(pascals: number, tFine: number): number {
  const target = BigInt(Math.round(pascals * 256));
  let lo = 0;
  let hi = (1 << 20) - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (bmp280Pressure(mid, tFine) > target) lo = mid + 1;
    else hi = mid;
  }
  const below = Math.max(0, lo - 1);
  const err = (adc: number) => {
    const e = bmp280Pressure(adc, tFine) - target;
    return e < 0n ? -e : e;
  };
  return err(below) < err(lo) ? below : lo;
}

/** The typical measurement time (section 9.1) for the ctrl_meas oversampling settings, in seconds. */
export function bmp280MeasureSeconds(ctrlMeas: number): number {
  const t = OVERSAMPLING[(ctrlMeas >> 5) & 7]!;
  const p = OVERSAMPLING[(ctrlMeas >> 2) & 7]!;
  return (1 + 2 * t + (p > 0 ? 2 * p + 0.5 : 0)) / 1000;
}

function trimmingBytes(): Uint8Array {
  const T = BMP280_TRIMMING;
  const words = [T.T1, T.T2, T.T3, T.P1, T.P2, T.P3, T.P4, T.P5, T.P6, T.P7, T.P8, T.P9];
  const bytes = new Uint8Array(24);
  words.forEach((w, i) => {
    bytes[i * 2] = w & 0xff;
    bytes[i * 2 + 1] = (w >> 8) & 0xff;
  });
  return bytes;
}
const TRIMMING_BYTES = trimmingBytes();

export class Bmp280 implements I2cDevice {
  readonly address: number;
  powered = false;
  /** The board's time in seconds when it talks to the part; set by the board that addresses it. */
  clock: (() => number) | undefined = undefined;
  // Ambient temperature (degC) and pressure (Pa) the next conversion measures.
  private celsius = 25;
  private pascals = 101325;

  private ctrlMeas = 0;
  private config = 0;
  // The raw readings in the data registers, after the IIR filter.
  private adcT = SKIPPED;
  private adcP = SKIPPED;
  // The filter's state, unset until a conversion has run since reset or a filter change.
  private filteredT: number | null = null;
  private filteredP: number | null = null;
  // The conversion in progress or the next one in normal mode: when it ends.
  private conversionEnds: number | null = null;
  private lastNow = 0;
  // Bus state.
  private pointer = 0;
  private expectPointer = true;
  private pendingRegister = -1;
  private shadow: Uint8Array | null = null;

  constructor({ address = 0x77 }: { address?: number } = {}) {
    this.address = address;
    this.reset();
  }

  reset(): void {
    this.ctrlMeas = 0;
    this.config = 0;
    this.adcT = SKIPPED;
    this.adcP = SKIPPED;
    this.filteredT = null;
    this.filteredP = null;
    this.conversionEnds = null;
    this.pointer = 0;
    this.expectPointer = true;
    this.pendingRegister = -1;
    this.shadow = null;
  }

  /**
   * Sets the ambient values. Conversions run when the bus next touches the part, so the ones already due run first
   * with the values they measured.
   */
  setAmbient(celsius: number, pascals: number): void {
    if (celsius === this.celsius && pascals === this.pascals) return;
    this.advance();
    this.celsius = celsius;
    this.pascals = pascals;
  }

  /** The mode bits of ctrl_meas: 0 sleep, 1 or 2 forced, 3 normal. */
  get mode(): number {
    return this.ctrlMeas & 3;
  }

  start(): void {
    this.advance();
    this.expectPointer = true;
    this.pendingRegister = -1;
    // A burst read sees one conversion's readings, however long it takes.
    this.shadow = this.dataBytes();
  }

  write(value: number): boolean {
    this.advance();
    // A write is the register address, then register/data pairs (section 5.2.1).
    if (this.expectPointer) {
      this.pointer = value;
      this.pendingRegister = value;
      this.expectPointer = false;
      return true;
    }
    if (this.pendingRegister < 0) {
      this.pendingRegister = value;
      this.pointer = value;
      return true;
    }
    this.writeRegister(this.pendingRegister, value);
    this.pendingRegister = -1;
    return true;
  }

  read(): number {
    this.advance();
    const value = this.readRegister(this.pointer);
    this.pointer = Math.min(0xff, this.pointer + 1);
    return value;
  }

  stop(): void {
    this.advance();
    this.shadow = null;
    this.expectPointer = true;
    this.pendingRegister = -1;
  }

  private now(): number {
    const t = this.clock?.() ?? this.lastNow;
    // A board that reset runs its clock from zero again; a conversion it started has long finished by then.
    if (t < this.lastNow && this.conversionEnds !== null) this.conversionEnds = t;
    this.lastNow = t;
    return t;
  }

  /** Runs the conversions that have ended by now. */
  private advance(): void {
    const now = this.now();
    if (this.mode === 3 && this.conversionEnds !== null && this.conversionEnds <= now) {
      // A sketch that leaves the part in normal mode for minutes owes it thousands of conversions; past the last
      // thousand the IIR filter (at most 1/16 of the way a sample) has converged to well below one raw count.
      const period = STANDBY[(this.config >> 5) & 7]! + bmp280MeasureSeconds(this.ctrlMeas);
      const owed = Math.floor((now - this.conversionEnds) / period);
      if (owed > 1000) this.conversionEnds += (owed - 1000) * period;
    }
    while (this.conversionEnds !== null && this.conversionEnds <= now) {
      this.convert();
      if (this.mode === 3) {
        this.conversionEnds += STANDBY[(this.config >> 5) & 7]! + bmp280MeasureSeconds(this.ctrlMeas);
      } else {
        // Forced mode returns to sleep once its conversion ends (section 3.6.2).
        this.ctrlMeas &= ~3;
        this.conversionEnds = null;
      }
    }
  }

  // The raw readings for the ambient values they were last computed for: the searches run only when those change.
  private raw = { celsius: NaN, pascals: NaN, t: 0, p: 0 };

  private rawReadings(): { t: number; p: number } {
    if (this.raw.celsius !== this.celsius || this.raw.pascals !== this.pascals) {
      const t = bmp280RawTemperature(this.celsius);
      this.raw = { celsius: this.celsius, pascals: this.pascals, t, p: bmp280RawPressure(this.pascals, bmp280Temperature(t).tFine) };
    }
    return this.raw;
  }

  private convert(): void {
    const tSamples = OVERSAMPLING[(this.ctrlMeas >> 5) & 7]!;
    const pSamples = OVERSAMPLING[(this.ctrlMeas >> 2) & 7]!;
    const coefficient = FILTER[(this.config >> 2) & 7]!;
    const { t: rawT, p: rawP } = this.rawReadings();
    if (tSamples > 0) {
      this.filteredT = this.filtered(this.filteredT, rawT, coefficient);
      this.adcT = this.resolution(this.filteredT, tSamples);
    } else {
      this.adcT = SKIPPED;
    }
    if (pSamples > 0) {
      this.filteredP = this.filtered(this.filteredP, rawP, coefficient);
      this.adcP = this.resolution(this.filteredP, pSamples);
    } else {
      this.adcP = SKIPPED;
    }
  }

  private filtered(previous: number | null, raw: number, coefficient: number): number {
    // The IIR filter of section 3.4: each output moves 1/coefficient of the way to the new sample.
    return previous === null || coefficient === 1 ? raw : (previous * (coefficient - 1) + raw) / coefficient;
  }

  private resolution(value: number, samples: number): number {
    // x1 gives 16 bits and each doubling of the samples one more, to 20 bits at x16 (section 3.4, tables 4 and 5);
    // with the filter on, the output is 20 bits whatever the oversampling.
    const bits = (this.config >> 2) & 7 ? 20 : 16 + Math.log2(samples);
    const step = 1 << (20 - bits);
    return Math.min((1 << 20) - 1, Math.round(value / step) * step);
  }

  private writeRegister(register: number, value: number): void {
    if (register === 0xe0) {
      if (value === RESET_WORD) this.reset();
    } else if (register === 0xf4) {
      this.ctrlMeas = value;
      const mode = value & 3;
      // A write in sleep stops normal mode; forced mode starts one conversion; normal mode starts measuring now.
      this.conversionEnds = mode === 0 ? null : this.lastNow + bmp280MeasureSeconds(value);
    } else if (register === 0xf5) {
      if (((value >> 2) & 7) !== ((this.config >> 2) & 7)) {
        this.filteredT = null;
        this.filteredP = null;
      }
      this.config = value & 0xfd;
    }
  }

  private dataBytes(): Uint8Array {
    return Uint8Array.of(
      this.adcP >> 12, (this.adcP >> 4) & 0xff, (this.adcP & 0xf) << 4,
      this.adcT >> 12, (this.adcT >> 4) & 0xff, (this.adcT & 0xf) << 4,
    );
  }

  private readRegister(register: number): number {
    if (register >= 0x88 && register < 0x88 + TRIMMING_BYTES.length) return TRIMMING_BYTES[register - 0x88]!;
    if (register === 0xd0) return CHIP_ID;
    if (register === 0xe0) return 0;
    if (register === 0xf3) return this.conversionEnds !== null && this.measuring() ? 0x08 : 0;
    if (register === 0xf4) return this.ctrlMeas;
    if (register === 0xf5) return this.config;
    if (register >= 0xf7 && register <= 0xfc) return (this.shadow ?? this.dataBytes())[register - 0xf7]!;
    return 0;
  }

  private measuring(): boolean {
    // In normal mode the part is measuring only during the measurement, not the standby that follows it.
    if (this.conversionEnds === null) return false;
    return this.lastNow >= this.conversionEnds - bmp280MeasureSeconds(this.ctrlMeas);
  }
}
