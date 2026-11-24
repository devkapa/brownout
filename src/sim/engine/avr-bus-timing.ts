/**
 * Bus behaviour the ATmega328P has and avr8js 0.21.0 leaves out or gets wrong, shared by `ArduinoMcu` and the AVR
 * lock-step harness (scripts/avr-lockstep.mjs), so the classic engine and the Rust core (core/src/mcu/avr/twi.rs,
 * usart.rs) are held to the same model. Each departure from avr8js is toward the datasheet; where the datasheet gives
 * a figure the model uses it, and where it only bounds a time the model's reading is stated.
 *
 * TWI. avr8js finishes each bus step the instant the program starts it. Here a step takes the time the bus needs at
 * the SCL rate TWBR and TWSR's prescaler set, SCL = f / (16 + 2 x TWBR x 4^TWPS): a byte (address or data) is nine
 * SCL periods, eight bits and the acknowledge; a START from an idle bus is half a period (SDA falls, SCL follows
 * after the hold time), a repeated START one and a half (SCL released, set-up, SDA falls, hold) and a STOP one (SDA
 * low under SCL, SCL released, set-up, SDA released). The datasheet bounds those three by minimum set-up and hold
 * times rather than fixing them, so they are this model's reading of the waveform; the byte time is exact. A START
 * after a STOP also waits out the bus-free time, t_BUF in the datasheet's two-wire characteristics: 4.7 us at SCL
 * rates up to 100 kHz and 1.3 us above. TWINT (and, for a STOP, TWSTO clearing) comes when the step ends, so a
 * sketch that polls them waits as on hardware. Flags follow the datasheet where avr8js does not: TWINT is clear out
 * of reset (TWCR resets to 0), a STOP never sets it (TWSR reads 0xF8, "no relevant state information, TWINT = 0"),
 * and only writing a one clears it, so a write of zero leaves it as it was. TWSR's status bits are read-only.
 *
 * USART transmit. avr8js has no transmit buffer: UDRE stays clear for the whole frame, and a write to UDR while a
 * frame is going out starts a second, overlapping one. The 328P double-buffers: the first byte moves straight into
 * the shift register and UDRE stays set, a second waits in UDR (UDRE clear) and starts the moment the first frame
 * ends, so back-to-back bytes leave with no gap and the program runs on one frame earlier. A byte written while UDR
 * is full is ignored, as the datasheet says. TXC is set when a frame ends with nothing waiting and cleared only by
 * writing a one to it or by its interrupt, not when the next frame starts. Writing UCSRA no longer clears UDRE and
 * RXC (avr8js stores only MPCM and U2X), since those flags are read-only on the chip. A byte is reported to the host
 * when its frame starts on the wire, not when it is written to UDR.
 *
 * USART receive. avr8js holds one received byte, sets RXC when the whole frame has arrived, and lets the next frame
 * overwrite a byte nobody read. The 328P samples each bit at its middle (samples 8, 9 and 10 of 16, or 4, 5 and 6 of
 * 8 with U2X) and moves the frame into the receive buffer at the first stop bit's sample, so RXC rises half a bit
 * before the frame ends. The buffer is a two-level FIFO; a third frame waits in the shift register, and when a
 * fourth start bit arrives with that one still waiting, it is lost and the next frame read carries DOR. Reading UDR
 * takes the oldest frame, and RXC stays set while any remain. Disabling the receiver flushes the FIFO and drops a
 * frame on its way in. Bytes the host types queue here and go onto the wire one frame apart while RXEN is set.
 * Frames from the host are always well formed, so FE and UPE stay clear.
 *
 * SPI needs nothing here: avr8js already completes a transfer after eight SCK periods at the SPCR/SPI2X divider,
 * which is the hardware's timing, and the core matches it.
 *
 * The avr8js types appear only structurally, so this module's declarations never resolve the optional peer.
 */

interface InterruptLike {
  address: number;
  flagRegister: number;
  flagMask: number;
  enableRegister: number;
  enableMask: number;
  constant?: boolean;
  inverseFlag?: boolean;
}

interface CpuLike {
  cycles: number;
  data: Uint8Array;
  readHooks: { [addr: number]: (addr: number) => number };
  writeHooks: { [addr: number]: (value: number, oldValue: number, addr: number, mask: number) => boolean | void };
  addClockEvent(callback: () => void, cycles: number): unknown;
  setInterruptFlag(interrupt: InterruptLike): void;
  clearInterrupt(interrupt: InterruptLike, clearFlag?: boolean): void;
  clearInterruptByFlag(interrupt: InterruptLike, registerValue: number): void;
}

interface TwiLike {
  eventHandler: {
    start(repeated: boolean): void;
    stop(): void;
    connectToSlave(address: number, write: boolean): void;
    writeByte(value: number): void;
    readByte(ack: boolean): void;
  };
  completeStart(): void;
  completeConnect(ack: boolean): void;
  completeWrite(ack: boolean): void;
  completeRead(value: number): void;
}

interface UsartLike {
  onByteTransmit: ((value: number) => void) | null;
  onLineTransmit: ((line: string) => void) | null;
  onConfigurationChange: (() => void) | null;
}

const TWBR = 0xb8;
const TWSR = 0xb9;
const TWCR = 0xbc;
const TWSR_TWPS_MASK = 0x3;
const TWSR_TWS_MASK = 0xf8;
const TWCR_TWINT = 0x80;
const TWCR_TWSTO = 0x10;
const STATUS_TWI_IDLE = 0xf8;

const UCSRA = 0xc0;
const UCSRB = 0xc1;
const UDR = 0xc6;
const UCSRA_DOR = 0x8;
const UCSRA_U2X = 0x2;
const UCSRA_MPCM = 0x1;
const UCSRB_RXEN = 0x10;
// usart.js's UCSRA_CFG_MASK: the bits whose change is a configuration change.
const UCSRA_CFG_MASK = UCSRA_U2X;

/** One SCL period in CPU cycles: always even, so every step time below is a whole number of cycles. */
export function twiSclCycles(data: Uint8Array): number {
  const prescaler = 4 ** (data[TWSR]! & TWSR_TWPS_MASK);
  return 16 + 2 * data[TWBR]! * prescaler;
}

export type TwiStepKind = "start" | "restart" | "stop" | "byte";

/** How long a bus step holds the TWI busy, in CPU cycles (see the module comment for each). */
export function twiStepCycles(kind: TwiStepKind, scl: number): number {
  switch (kind) {
    case "start":
      return scl / 2;
    case "restart":
      return (scl * 3) / 2;
    case "stop":
      return scl;
    case "byte":
      return 9 * scl;
  }
}

/**
 * The bus-free time between a STOP and the next START (t_BUF), in whole CPU cycles, rounded up: 4.7 us for SCL up to
 * 100 kHz, 1.3 us above. Integer arithmetic, so the Rust core rounds the same way.
 */
export function twiBusFreeCycles(scl: number, clockHz: number): number {
  const tenthsOfUs = scl * 100_000 >= clockHz ? 47 : 13;
  return Math.ceil((clockHz * tenthsOfUs) / 10_000_000);
}

/** A bus step as it starts: what the program put on the bus, the answer it will get, and how long it takes. */
export type TimedTwiStep = { cycle: number; cycles: number } & (
  | { kind: "start" | "restart" | "stop" }
  | { kind: "address"; address: number; read: boolean; ack: boolean }
  | { kind: "write" | "read"; value: number; ack: boolean }
);

/**
 * Answers every step as an empty bus does (each address NACKed, a read 0xFF: avr8js's NoopTWIEventHandler), each
 * after the step's bus time, and puts TWINT, TWSTO and TWSR where the datasheet has them. `onStep` hears each step
 * as it starts on the bus.
 */
export function attachTimedTwi(cpu: CpuLike, twi: TwiLike, onStep?: (step: TimedTwiStep) => void): void {
  // twi.js keeps its interrupt descriptor and clock in fields the types call private.
  const fields = twi as unknown as { TWI: InterruptLike; freqHz: number; busy: boolean };
  const { TWI: interrupt, freqHz } = fields;
  // avr8js raises TWINT as it builds the TWI; TWCR is 0 out of reset.
  cpu.clearInterrupt(interrupt);
  // TWSR's status bits are read-only on the chip; avr8js stores a write whole, so a sketch that sets the prescaler
  // with a plain store would wipe the status (and the next START would read as a repeated one).
  cpu.writeHooks[TWSR] = (value: number, oldValue: number) => {
    cpu.data[TWSR] = (oldValue & ~TWSR_TWPS_MASK) | (value & TWSR_TWPS_MASK);
    return true;
  };
  // twi.js stores TWCR whole, so a write of zero to TWINT clears it, and a write the TWI does not act on stores a
  // one there. On the chip only writing a one clears it.
  const twcrHook = cpu.writeHooks[TWCR]!;
  cpu.writeHooks[TWCR] = (value: number, oldValue: number, addr: number, mask: number) => {
    twcrHook(value, oldValue, addr, mask);
    const twint = value & TWCR_TWINT ? 0 : oldValue & TWCR_TWINT;
    cpu.data[TWCR] = (value & ~TWCR_TWINT) | twint;
    if (twint) cpu.setInterruptFlag(interrupt);
    return true;
  };
  // When the last STOP left the bus, for the bus-free time before the next START.
  let stopEnded = Number.NEGATIVE_INFINITY;
  const later = (cycles: number, done: () => void) => cpu.addClockEvent(done, cycles);
  twi.eventHandler = {
    start: (repeated: boolean) => {
      const scl = twiSclCycles(cpu.data);
      const cycles = twiStepCycles(repeated ? "restart" : "start", scl);
      const wait = repeated ? 0 : Math.max(0, stopEnded + twiBusFreeCycles(scl, freqHz) - cpu.cycles);
      onStep?.({ kind: repeated ? "restart" : "start", cycle: cpu.cycles + wait, cycles });
      later(wait + cycles, () => twi.completeStart());
    },
    stop: () => {
      const cycles = twiStepCycles("stop", twiSclCycles(cpu.data));
      onStep?.({ kind: "stop", cycle: cpu.cycles, cycles });
      later(cycles, () => {
        // twi.js's completeStop, without the TWINT it raises there.
        fields.busy = false;
        cpu.data[TWCR] &= ~TWCR_TWSTO;
        cpu.data[TWSR] = (cpu.data[TWSR]! & ~TWSR_TWS_MASK) | STATUS_TWI_IDLE;
        stopEnded = cpu.cycles;
      });
    },
    connectToSlave: (address: number, write: boolean) => {
      const cycles = twiStepCycles("byte", twiSclCycles(cpu.data));
      onStep?.({ kind: "address", cycle: cpu.cycles, cycles, address, read: !write, ack: false });
      later(cycles, () => twi.completeConnect(false));
    },
    writeByte: (value: number) => {
      const cycles = twiStepCycles("byte", twiSclCycles(cpu.data));
      onStep?.({ kind: "write", cycle: cpu.cycles, cycles, value, ack: false });
      later(cycles, () => twi.completeWrite(false));
    },
    readByte: (ack: boolean) => {
      const cycles = twiStepCycles("byte", twiSclCycles(cpu.data));
      onStep?.({ kind: "read", cycle: cpu.cycles, cycles, value: 0xff, ack });
      later(cycles, () => twi.completeRead(0xff));
    },
  };
}

/** A received frame in the FIFO, with the overrun flag that travels with it. */
export interface RxFrame {
  value: number;
  dor: boolean;
}

/**
 * The 328P's USART0 on top of an avr8js AVRUSART: a double-buffered transmitter, and a receiver with its two-level
 * FIFO and the host's typed bytes. Replaces usart.js's UDR, UCSRA and UCSRB hooks (UCSRB's runs first);
 * `usart.onByteTransmit` (and `onLineTransmit`) now fire as each frame starts, and `onRxStart` as a typed byte's
 * frame starts arriving.
 */
export class Usart328 {
  /** A frame is in the transmit shift register. */
  shifting = false;
  /** The byte waiting in UDR for the shift register, or -1 when UDR is empty (UDRE set). */
  pending = -1;
  /** A frame is arriving on RXD. */
  receiving = false;
  /** Received frames in UDR's FIFO, oldest first, at most two. */
  readonly fifo: RxFrame[] = [];
  /** A complete frame waiting in the receive shift register while the FIFO is full, or -1. */
  held = -1;
  /** A frame was lost to an overrun; the next one into the FIFO carries DOR. */
  overrun = false;
  /** Bytes the host has typed that have not started arriving. */
  readonly input: number[] = [];
  /** A typed byte's frame has started arriving, lasting `cycles`. */
  onRxStart: ((value: number, cycles: number) => void) | null = null;
  private lineBuffer = "";
  // Bumped when the receiver is disabled, so a frame already on its way in is dropped when it lands.
  private rxGeneration = 0;
  private readonly udre: InterruptLike;
  private readonly txc: InterruptLike;
  private readonly rxc: InterruptLike;

  constructor(
    private readonly cpu: CpuLike,
    private readonly usart: UsartLike,
  ) {
    // usart.js keeps its interrupt descriptors private; they are plain fields at run time.
    const flags = usart as unknown as { UDRE: InterruptLike; TXC: InterruptLike; RXC: InterruptLike };
    this.udre = flags.UDRE;
    this.txc = flags.TXC;
    this.rxc = flags.RXC;
    // Not consumed, as usart.js's own hook: the value is also stored in UDR's data byte.
    cpu.writeHooks[UDR] = (value: number) => {
      this.write(value);
      return false;
    };
    cpu.readHooks[UDR] = () => this.read();
    cpu.writeHooks[UCSRA] = (value: number, oldValue: number) => {
      cpu.data[UCSRA] = (oldValue & ~(UCSRA_MPCM | UCSRA_U2X)) | (value & (UCSRA_MPCM | UCSRA_U2X));
      cpu.clearInterruptByFlag(this.txc, value);
      if ((value & UCSRA_CFG_MASK) !== (oldValue & UCSRA_CFG_MASK)) usart.onConfigurationChange?.();
      return true;
    };
    // usart.js's UCSRB hook sets the interrupt enables and stores the value, but also clears RXC on any write that
    // leaves RXEN set and raises UDRE whenever TXEN goes on, full UDR or not. Both flags are levels on the chip, so
    // they are put back from the buffers' state after it.
    const ucsrbHook = cpu.writeHooks[UCSRB]!;
    cpu.writeHooks[UCSRB] = (value: number, oldValue: number, addr: number, mask: number) => {
      ucsrbHook(value, oldValue, addr, mask);
      if (oldValue & UCSRB_RXEN && !(value & UCSRB_RXEN)) {
        this.fifo.length = 0;
        this.held = -1;
        this.overrun = false;
        this.rxGeneration++;
      }
      this.updateRx();
      if (this.pending < 0) cpu.setInterruptFlag(this.udre);
      else cpu.clearInterrupt(this.udre);
      if (value & UCSRB_RXEN && !(oldValue & UCSRB_RXEN)) this.feed();
      return true;
    };
  }

  /** usart.js's cyclesPerChar, read through the instance (it is a private getter in the types). */
  private get cyclesPerChar(): number {
    return (this.usart as unknown as { cyclesPerChar: number }).cyclesPerChar;
  }

  /** Queues bytes to arrive on RXD, one frame after another while the receiver is enabled. */
  queueInput(bytes: Iterable<number>): void {
    for (const b of bytes) this.input.push(b & 0xff);
    this.feed();
  }

  // UDRE is a level on the chip, set whenever UDR has room, and its interrupt keeps firing while it is; avr8js
  // clears the flag when it takes the interrupt. So each time UDR is left empty the flag is raised again, or an
  // interrupt-fed stream (HardwareSerial's) would stop after one byte.
  private write(value: number): void {
    if (this.pending >= 0) return; // UDRE clear: the datasheet has the transmitter ignore the write
    if (!this.shifting) {
      this.begin(value);
      this.cpu.setInterruptFlag(this.udre);
      return;
    }
    this.pending = value;
    this.cpu.clearInterrupt(this.udre);
  }

  private begin(value: number): void {
    this.shifting = true;
    this.usart.onByteTransmit?.(value);
    if (this.usart.onLineTransmit) {
      const ch = String.fromCharCode(value);
      if (ch === "\n") {
        this.usart.onLineTransmit(this.lineBuffer);
        this.lineBuffer = "";
      } else {
        this.lineBuffer += ch;
      }
    }
    this.cpu.addClockEvent(() => this.frameDone(), this.cyclesPerChar);
  }

  private frameDone(): void {
    if (this.pending >= 0) {
      const next = this.pending;
      this.pending = -1;
      this.begin(next);
      this.cpu.setInterruptFlag(this.udre);
      return;
    }
    this.shifting = false;
    this.cpu.setInterruptFlag(this.udre);
    this.cpu.setInterruptFlag(this.txc);
  }

  /** Cycles from a frame's start bit to the first stop bit's middle sample (the third of the three it votes on). */
  private get rxSampleCycles(): number {
    const u = this.usart as unknown as { UBRR: number; multiplier: number; bitsPerChar: number; parityEnabled: boolean };
    const stopBit = 1 + u.bitsPerChar + (u.parityEnabled ? 1 : 0);
    return (stopBit * u.multiplier + (u.multiplier === 16 ? 9 : 5)) * (u.UBRR + 1);
  }

  /** Puts the next typed byte on the wire, if the line is free and the receiver is listening. */
  private feed(): void {
    if (this.receiving || this.input.length === 0 || !(this.cpu.data[UCSRB]! & UCSRB_RXEN)) return;
    const value = this.input.shift()!;
    // A start bit with the FIFO full and a frame still waiting in the shift register: that frame is lost.
    if (this.held >= 0 && this.fifo.length === 2) {
      this.held = -1;
      this.overrun = true;
    }
    this.receiving = true;
    const frame = this.cyclesPerChar;
    const generation = this.rxGeneration;
    this.onRxStart?.(value, frame);
    this.cpu.addClockEvent(() => {
      if (generation === this.rxGeneration) this.arrive(value);
    }, this.rxSampleCycles);
    this.cpu.addClockEvent(() => {
      this.receiving = false;
      this.feed();
    }, frame);
  }

  private arrive(value: number): void {
    if (this.fifo.length < 2) {
      this.fifo.push({ value, dor: this.overrun });
      this.overrun = false;
    } else {
      this.held = value;
    }
    this.updateRx();
  }

  private read(): number {
    const head = this.fifo.shift();
    if (this.held >= 0) {
      this.fifo.push({ value: this.held, dor: this.overrun });
      this.overrun = false;
      this.held = -1;
    }
    this.updateRx();
    if (!head) return 0;
    const bits = (this.usart as unknown as { bitsPerChar: number }).bitsPerChar;
    return head.value & (bits >= 8 ? 0xff : (1 << bits) - 1);
  }

  /** RXC and DOR as the FIFO's head has them. */
  private updateRx(): void {
    const head = this.fifo[0];
    if (head?.dor) this.cpu.data[UCSRA] |= UCSRA_DOR;
    else this.cpu.data[UCSRA] &= ~UCSRA_DOR;
    if (head) this.cpu.setInterruptFlag(this.rxc);
    else this.cpu.clearInterrupt(this.rxc);
  }
}
