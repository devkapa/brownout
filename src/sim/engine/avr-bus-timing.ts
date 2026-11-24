/**
 * Bus timing the ATmega328P has and avr8js 0.21.0 leaves out, shared by `ArduinoMcu` and the AVR lock-step harness
 * (scripts/avr-lockstep.mjs), so the classic engine and the Rust core (core/src/mcu/avr/twi.rs, usart.rs) are held
 * to the same model. Two departures from avr8js, both toward the datasheet:
 *
 * TWI. avr8js finishes each bus step the instant the program starts it. Here a step takes the time the bus needs at
 * the SCL rate TWBR and TWSR's prescaler set, SCL = f / (16 + 2 x TWBR x 4^TWPS): a byte (address or data) is nine
 * SCL periods, eight bits and the acknowledge; a START from an idle bus is half a period (SDA falls, SCL follows
 * after the hold time), a repeated START one and a half (SCL released, set-up, SDA falls, hold) and a STOP one (SDA
 * low under SCL, SCL released, set-up, SDA released). The datasheet bounds those three by minimum set-up and hold
 * times rather than fixing them, so they are this model's reading of the waveform; the byte time is exact. TWINT
 * (and, for a STOP, TWSTO clearing) comes when the step ends, so a sketch that polls them waits as on hardware.
 * TWSR's status bits are read-only, as on the chip, where avr8js lets a store overwrite them.
 *
 * USART transmit. avr8js has no transmit buffer: UDRE stays clear for the whole frame, and a write to UDR while a
 * frame is going out starts a second, overlapping one. The 328P double-buffers: the first byte moves straight into
 * the shift register and UDRE stays set, a second waits in UDR (UDRE clear) and starts the moment the first frame
 * ends, so back-to-back bytes leave with no gap and the program runs on one frame earlier. A byte written while UDR
 * is already full replaces the waiting one, as the hardware overwrites it. Writing UCSRA no longer clears UDRE and
 * RXC (avr8js stores only MPCM and U2X), since those flags are read-only on the chip. A byte is reported to the
 * host when its frame starts on the wire, not when it is written to UDR.
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
  completeStop(): void;
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
const TWSR_TWPS_MASK = 0x3;

const UCSRA = 0xc0;
const UDR = 0xc6;
const UCSRA_U2X = 0x2;
const UCSRA_MPCM = 0x1;
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

/** A bus step as it starts: what the program put on the bus, the answer it will get, and how long it takes. */
export type TimedTwiStep = { cycle: number; cycles: number } & (
  | { kind: "start" | "restart" | "stop" }
  | { kind: "address"; address: number; read: boolean; ack: boolean }
  | { kind: "write" | "read"; value: number; ack: boolean }
);

/**
 * Answers every step as an empty bus does (each address NACKed, a read 0xFF: avr8js's NoopTWIEventHandler), each
 * after the step's bus time. `onStep` hears each step as it starts.
 */
export function attachTimedTwi(cpu: CpuLike, twi: TwiLike, onStep?: (step: TimedTwiStep) => void): void {
  // TWSR's status bits are read-only on the chip; avr8js stores a write whole, so a sketch that sets the prescaler
  // with a plain store would wipe the status (and the next START would read as a repeated one).
  cpu.writeHooks[TWSR] = (value: number, oldValue: number) => {
    cpu.data[TWSR] = (oldValue & ~TWSR_TWPS_MASK) | (value & TWSR_TWPS_MASK);
    return true;
  };
  const later = (cycles: number, done: () => void) => cpu.addClockEvent(done, cycles);
  twi.eventHandler = {
    start: (repeated: boolean) => {
      const cycles = twiStepCycles(repeated ? "restart" : "start", twiSclCycles(cpu.data));
      onStep?.({ kind: repeated ? "restart" : "start", cycle: cpu.cycles, cycles });
      later(cycles, () => twi.completeStart());
    },
    stop: () => {
      const cycles = twiStepCycles("stop", twiSclCycles(cpu.data));
      onStep?.({ kind: "stop", cycle: cpu.cycles, cycles });
      later(cycles, () => twi.completeStop());
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

/**
 * The 328P's double-buffered transmitter on top of an avr8js AVRUSART: replaces its UDR and UCSRA write hooks.
 * `usart.onByteTransmit` (and `onLineTransmit`) now fire as each frame starts.
 */
export class BufferedUsartTx {
  /** A frame is in the shift register. */
  shifting = false;
  /** The byte waiting in UDR for the shift register, or -1 when UDR is empty (UDRE set). */
  pending = -1;
  private lineBuffer = "";
  private readonly udre: InterruptLike;
  private readonly txc: InterruptLike;

  constructor(
    private readonly cpu: CpuLike,
    private readonly usart: UsartLike,
  ) {
    // usart.js keeps its interrupt descriptors private; they are plain fields at run time.
    const flags = usart as unknown as { UDRE: InterruptLike; TXC: InterruptLike };
    this.udre = flags.UDRE;
    this.txc = flags.TXC;
    // Not consumed, as usart.js's own hook: the value is also stored in UDR's data byte.
    cpu.writeHooks[UDR] = (value: number) => {
      this.write(value);
      return false;
    };
    cpu.writeHooks[UCSRA] = (value: number, oldValue: number) => {
      cpu.data[UCSRA] = (oldValue & ~(UCSRA_MPCM | UCSRA_U2X)) | (value & (UCSRA_MPCM | UCSRA_U2X));
      cpu.clearInterruptByFlag(this.txc, value);
      if ((value & UCSRA_CFG_MASK) !== (oldValue & UCSRA_CFG_MASK)) usart.onConfigurationChange?.();
      return true;
    };
  }

  /** usart.js's cyclesPerChar, read through the instance (it is a private getter in the types). */
  private get cyclesPerChar(): number {
    return (this.usart as unknown as { cyclesPerChar: number }).cyclesPerChar;
  }

  private write(value: number): void {
    if (!this.shifting) {
      this.begin(value);
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
    this.cpu.clearInterrupt(this.txc);
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
    this.cpu.setInterruptFlag(this.txc);
  }
}
