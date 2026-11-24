/**
 * DHT22 (AM2302) and DHT11 humidity and temperature sensors on their single-wire data line, as the 3-pin modules
 * carry them (with the line's pull-up on board). The host pulls the line low to ask for a reading and releases it;
 * the sensor answers with an 80 us low and an 80 us high, then 40 bits, each a 50 us low followed by a 26-28 us high
 * for a 0 or a 70 us high for a 1, and a final 50 us low (AM2302 datasheet section 7; DHT11 datasheet section 5).
 * The 40 bits are humidity, temperature and a checksum: tenths in 16-bit words on the DHT22 (the temperature's top
 * bit its sign), whole and tenth bytes on the DHT11.
 *
 * The sensor sees the board's own drive edges at the cycle they happen and drives the board's input at the cycle it
 * changes it, so a library that times the bits by counting loop iterations reads them as from the part. As on the
 * part, a reading reports the measurement made after the previous one, the sensor ignores the line for its first
 * second of power, and a host start shorter than the datasheet's minimum gets no answer.
 */

/** The board's side of the line, as a pin target sees it. Cycles are the board's own clock. */
export interface PinLine {
  /** The board's clock rate, in Hz. */
  readonly hz: number;
  /** The board's cycle count now. */
  now(): number;
  /** Runs `fn` `cycles` from now on the board's clock. */
  after(cycles: number, fn: () => void): void;
  /** Pulls the line low (true) or lets it go back to its pull-up (false). */
  pull(low: boolean): void;
}

/** A part on one of a board's pins that talks to it in time, not through the circuit's voltages. */
export interface PinTarget {
  /** Supplied: an unpowered part neither answers nor holds the line. */
  powered: boolean;
  /** The board's drive on the line changed: 0 low, 1 high, null not driving (an input). */
  boardDrive(level: 0 | 1 | null, line: PinLine): void;
  /** Power-on reset. */
  reset(): void;
  /** The board reset: its clock starts over, so times the part kept on it mean nothing now. */
  boardReset?(): void;
}

export type DhtVariant = 11 | 22;

// Timings in microseconds, the datasheets' typical values.
const RESPONSE_DELAY_US = 30;
const RESPONSE_LOW_US = 80;
const RESPONSE_HIGH_US = 80;
const BIT_LOW_US = 50;
const ZERO_HIGH_US = 27;
const ONE_HIGH_US = 70;
const END_LOW_US = 50;
// The shortest host start each part answers (AM2302: 0.8 ms minimum, 1 ms typical; DHT11: at least 18 ms).
const START_MIN_US: Record<DhtVariant, number> = { 22: 800, 11: 18000 };
// "After power-on the sensor needs 1 s to pass its unstable state; send it nothing in that time" (AM2302 section 7).
const POWER_UP_S = 1;
// The shortest time between measurements (AM2302: "the minimum interval for reading the sensor is 2 s"; DHT11: 1 s).
// A reading sooner than that sends the previous measurement again and the part does not measure after it.
const MEASURE_INTERVAL_S: Record<DhtVariant, number> = { 22: 2, 11: 1 };

/** The five bytes a reading sends for `humidity` (%RH) and `celsius`, clamped to the part's range. */
export function dhtFrame(variant: DhtVariant, humidity: number, celsius: number): Uint8Array {
  const bytes = new Uint8Array(5);
  if (variant === 22) {
    // AM2302: 0-99.9 %RH, -40 to 80 degC, tenths.
    const rh = Math.round(Math.min(99.9, Math.max(0, humidity)) * 10);
    const t = Math.round(Math.min(80, Math.max(-40, celsius)) * 10);
    const tWord = t < 0 ? 0x8000 | -t : t;
    bytes.set([rh >> 8, rh & 0xff, tWord >> 8, tWord & 0xff]);
  } else {
    // DHT11: 20-90 %RH and 0-50 degC, whole units with the decimal byte (0 on the common parts' humidity).
    const rh = Math.round(Math.min(90, Math.max(20, humidity)));
    // Rounded to tenths before the split, so 23.96 carries to 24.0 rather than reading 23.0.
    const tenths = Math.round(Math.min(50, Math.max(0, celsius)) * 10);
    bytes.set([rh, 0, Math.floor(tenths / 10), tenths % 10]);
  }
  bytes[4] = (bytes[0]! + bytes[1]! + bytes[2]! + bytes[3]!) & 0xff;
  return bytes;
}

export class DhtSensor implements PinTarget {
  readonly variant: DhtVariant;
  private _powered = false;
  /** Seconds the part has been powered; its host keeps this current. */
  poweredFor = 0;
  private humidity = 50;
  private celsius = 25;
  // What the next reading sends: the measurement made after the previous reading, or the one at power-up (taken
  // from the first ambient values the host gives after a reset).
  private measured: Uint8Array | null = null;
  private startedLowAt: number | null = null;
  private answering = false;
  // The board cycles the reply spans, so a board that reset (its clock starting over, the reply's events gone)
  // does not leave the part answering forever.
  private answerSpan: [number, number] = [0, 0];
  // Bumped on reset so a reply in flight from before it stops.
  private generation = 0;
  // The board cycle of the last measurement after a reading, null before the first (or after a reset).
  private measuredAt: number | null = null;
  // The line this part is holding low, so losing power or resetting mid-reply lets it go: an unsupplied part's
  // output cannot sink the module's pull-up.
  private holding: PinLine | null = null;

  constructor({ variant = 22 }: { variant?: DhtVariant } = {}) {
    this.variant = variant;
  }

  setAmbient(humidity: number, celsius: number): void {
    this.humidity = humidity;
    this.celsius = celsius;
    this.measured ??= dhtFrame(this.variant, humidity, celsius);
  }

  get powered(): boolean {
    return this._powered;
  }

  set powered(on: boolean) {
    if (!on) this.release();
    this._powered = on;
  }

  private release(): void {
    const line = this.holding;
    this.holding = null;
    line?.pull(false);
  }

  boardReset(): void {
    this.measuredAt = null;
  }

  reset(): void {
    this.release();
    this.measuredAt = null;
    this.generation++;
    this.startedLowAt = null;
    this.answering = false;
    this.poweredFor = 0;
    this.measured = null;
  }

  boardDrive(level: 0 | 1 | null, line: PinLine): void {
    const now = line.now();
    if (this.answering && (now < this.answerSpan[0] || now > this.answerSpan[1])) this.answering = false;
    if (!this.powered || this.answering) return;
    if (level === 0) {
      this.startedLowAt = now;
      return;
    }
    const began = this.startedLowAt;
    this.startedLowAt = null;
    if (began === null) return;
    const lowUs = ((now - began) / line.hz) * 1e6;
    if (lowUs < START_MIN_US[this.variant]) return;
    if (this.poweredFor < POWER_UP_S) return;
    // The host released the line (or drove it high): the part answers once it sees the pull-up.
    this.answer(line);
  }

  private answer(line: PinLine): void {
    this.answering = true;
    const generation = this.generation;
    const frame = this.measured ?? dhtFrame(this.variant, this.humidity, this.celsius);
    // Edges as [microseconds from release, pull low?]; the bus idles high between them.
    const edges: [number, boolean][] = [];
    let t = RESPONSE_DELAY_US;
    edges.push([t, true]);
    t += RESPONSE_LOW_US;
    edges.push([t, false]);
    t += RESPONSE_HIGH_US;
    for (let i = 0; i < 40; i++) {
      const bit = (frame[i >> 3]! >> (7 - (i & 7))) & 1;
      edges.push([t, true]);
      t += BIT_LOW_US;
      edges.push([t, false]);
      t += bit ? ONE_HIGH_US : ZERO_HIGH_US;
    }
    edges.push([t, true]);
    t += END_LOW_US;
    edges.push([t, false]);
    const start = line.now();
    const cyclesAt = (us: number) => Math.round((us * line.hz) / 1e6);
    this.answerSpan = [start, start + cyclesAt(t)];
    edges.forEach(([us, low], i) => {
      line.after(start + cyclesAt(us) - line.now(), () => {
        if (generation !== this.generation || !this.powered) return;
        line.pull(low);
        this.holding = low ? line : null;
        if (i === edges.length - 1) {
          this.answering = false;
          // The part measures after a reading, the next reading reporting it, unless it measured too recently.
          // Timed from each reading's release, as a host spaces its readings, so the replies' lengths (which vary
          // with their bits) do not count. A board clock that started over (this reading before the last
          // measurement) has lost the time between: measure.
          const at = this.answerSpan[0];
          const last = this.measuredAt;
          if (last === null || at < last || (at - last) / line.hz >= MEASURE_INTERVAL_S[this.variant]) {
            this.measured = dhtFrame(this.variant, this.humidity, this.celsius);
            this.measuredAt = at;
          }
        }
      });
    });
  }
}
