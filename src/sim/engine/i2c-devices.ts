/**
 * I2C parts a board's TWI can talk to. Each is a target on the bus: the TWI addresses it, then hands it the bytes of
 * a write or asks it for the bytes of a read, with a START before and a STOP after. The models work byte by byte, as
 * the parts' own interfaces do, so a sketch's Wire calls reach them exactly as they reach the chip.
 */

/** A target on an I2C bus. */
export interface I2cDevice {
  /** The 7-bit address it answers. */
  readonly address: number;
  /** Supplied: a target without power neither answers the bus nor keeps its state. Its host keeps this current. */
  powered: boolean;
  /** A START (or repeated START) addressed to it. */
  start(): void;
  /** A byte written to it; true when it acknowledges. */
  write(value: number): boolean;
  /** The next byte it sends for a read. */
  read(): number;
  /** The STOP that ends the transaction. */
  stop(): void;
  /** Power-on reset. */
  reset(): void;
  /**
   * The time on the clock of the board talking to it, in seconds, for a part whose behaviour takes time (a
   * conversion). The board sets it as it addresses the part.
   */
  clock?: (() => number) | undefined;
}

/** What an SSD1306 panel shows: 1 bit a pixel, rows packed MSB first (the leftmost pixel in bit 7). */
export interface Ssd1306Image {
  width: number;
  height: number;
  /** The panel is lit: display on (AFh) and its charge pump enabled (8Dh 14h). */
  on: boolean;
  /** The contrast setting (81h), 0-255. */
  contrast: number;
  pixels: Uint8Array;
}

const WIDTH = 128;
const PAGES = 8;

// How many argument bytes follow each multi-byte command (datasheet section 9, Table 9-1).
const ARGUMENTS: Record<number, number> = {
  0x20: 1, // memory addressing mode
  0x21: 2, // column address
  0x22: 2, // page address
  0x26: 6, // right horizontal scroll setup
  0x27: 6, // left horizontal scroll setup
  0x29: 5, // vertical and right horizontal scroll setup
  0x2a: 5, // vertical and left horizontal scroll setup
  0x81: 1, // contrast
  0x8d: 1, // charge pump
  0xa3: 2, // vertical scroll area
  0xa8: 1, // multiplex ratio
  0xd3: 1, // display offset
  0xd5: 1, // clock divide ratio and oscillator frequency
  0xd9: 1, // pre-charge period
  0xda: 1, // COM pins hardware configuration
  0xdb: 1, // VCOMH deselect level
};

/**
 * The SSD1306 OLED controller (Solomon Systech datasheet rev 1.1) on its I2C interface, as the 128x64 and 128x32
 * modules carry it. After the address, each byte pair or stream starts with a control byte: D/C (bit 6) says whether
 * what follows is commands or GDDRAM data, and Co (bit 7) set means only the next byte is, after which another control
 * byte comes. GDDRAM is 128 columns by 8 pages of 8 rows, a byte a column with its LSB on the page's top row, written
 * in page, horizontal or vertical addressing (section 10.1.3). The image applies the display start line, offset,
 * multiplex ratio, segment remap, COM scan direction, entire-display-on and inverse settings the way the controller
 * scans GDDRAM onto the rows; the COM pins configuration (DAh) is taken as the module's own wiring and not applied.
 * Scrolling commands are parsed so their arguments are never taken for commands, but the panel does not scroll.
 * Reads return the status byte (bit 6 set while the display is off).
 */
export class Ssd1306 implements I2cDevice {
  readonly address: number;
  readonly height: number;
  // Off until its host first finds it supplied, so it never answers a bus before its power is known.
  powered = false;
  private readonly ram = new Uint8Array(WIDTH * PAGES);
  // Control-byte state within a transaction.
  private expectControl = true;
  private single = false;
  private dataMode = false;
  // A multi-byte command waiting for its arguments.
  private command = -1;
  private args: number[] = [];
  // Registers, at their reset values in reset().
  private displayOn = false;
  private chargePump = false;
  private contrast = 0x7f;
  private mode = 2; // page addressing
  private colStart = 0;
  private colEnd = 127;
  private pageStart = 0;
  private pageEnd = 7;
  private col = 0;
  private page = 0;
  private pageModeCol = 0;
  private startLine = 0;
  private offset = 0;
  private mux = 63;
  private segRemap = false;
  private comRemap = false;
  private entireOn = false;
  private inverse = false;
  // The scanned-out image, kept until GDDRAM or a register changes: hosts ask every committed step, and rebuilding
  // 8192 pixels that often would cost more than the board driving them.
  private cached: Ssd1306Image | null = null;

  constructor({ address = 0x3c, height = 64 }: { address?: number; height?: 32 | 64 } = {}) {
    this.address = address;
    this.height = height;
    this.reset();
  }

  reset(): void {
    // GDDRAM is undefined at power-on; cleared here so a panel comes up dark and the same every run.
    this.ram.fill(0);
    this.cached = null;
    this.expectControl = true;
    this.command = -1;
    this.args = [];
    this.displayOn = false;
    this.chargePump = false;
    this.contrast = 0x7f;
    this.mode = 2;
    this.colStart = 0;
    this.colEnd = 127;
    this.pageStart = 0;
    this.pageEnd = 7;
    this.col = 0;
    this.page = 0;
    this.pageModeCol = 0;
    this.startLine = 0;
    this.offset = 0;
    this.mux = 63;
    this.segRemap = false;
    this.comRemap = false;
    this.entireOn = false;
    this.inverse = false;
  }

  start(): void {
    this.expectControl = true;
  }

  stop(): void {
    this.expectControl = true;
  }

  write(value: number): boolean {
    value &= 0xff;
    if (this.expectControl) {
      this.single = (value & 0x80) !== 0;
      this.dataMode = (value & 0x40) !== 0;
      this.expectControl = false;
      return true;
    }
    if (this.dataMode) this.writeData(value);
    else this.writeCommand(value);
    if (this.single) this.expectControl = true;
    return true;
  }

  read(): number {
    return this.displayOn ? 0x00 : 0x40;
  }

  /** The panel is lit: display on and its charge pump enabled. */
  get lit(): boolean {
    return this.displayOn && this.chargePump;
  }

  private writeData(value: number): void {
    this.ram[this.page * WIDTH + this.col] = value;
    this.cached = null;
    if (this.mode === 2) {
      // Page addressing: along the page, back to the start column at its end, on the same page.
      this.col = this.col >= 127 ? this.pageModeCol : this.col + 1;
    } else if (this.mode === 0) {
      if (this.col >= this.colEnd) {
        this.col = this.colStart;
        this.page = this.page >= this.pageEnd ? this.pageStart : this.page + 1;
      } else this.col++;
    } else {
      if (this.page >= this.pageEnd) {
        this.page = this.pageStart;
        this.col = this.col >= this.colEnd ? this.colStart : this.col + 1;
      } else this.page++;
    }
  }

  private writeCommand(value: number): void {
    if (this.command >= 0) {
      this.args.push(value);
      if (this.args.length === ARGUMENTS[this.command]) {
        this.apply(this.command, this.args);
        this.command = -1;
        this.args = [];
      }
      return;
    }
    if (ARGUMENTS[value] !== undefined) {
      this.command = value;
      this.args = [];
      return;
    }
    this.apply(value, []);
  }

  private apply(c: number, a: number[]): void {
    this.cached = null;
    if (c <= 0x0f) {
      this.pageModeCol = (this.pageModeCol & 0xf0) | c;
      if (this.mode === 2) this.col = this.pageModeCol;
    } else if (c <= 0x1f) {
      this.pageModeCol = ((c & 0x07) << 4) | (this.pageModeCol & 0x0f);
      if (this.mode === 2) this.col = this.pageModeCol;
    } else if (c === 0x20) {
      // 11b is invalid and leaves the mode as it was.
      if ((a[0]! & 3) !== 3) this.mode = a[0]! & 3;
    } else if (c === 0x21) {
      this.colStart = a[0]! & 0x7f;
      this.colEnd = a[1]! & 0x7f;
      this.col = this.colStart;
    } else if (c === 0x22) {
      this.pageStart = a[0]! & 7;
      this.pageEnd = a[1]! & 7;
      this.page = this.pageStart;
    } else if (c >= 0x40 && c <= 0x7f) this.startLine = c & 0x3f;
    else if (c === 0x81) this.contrast = a[0]!;
    else if (c === 0x8d) this.chargePump = (a[0]! & 0x04) !== 0;
    else if (c === 0xa0 || c === 0xa1) this.segRemap = c === 0xa1;
    else if (c === 0xa4 || c === 0xa5) this.entireOn = c === 0xa5;
    else if (c === 0xa6 || c === 0xa7) this.inverse = c === 0xa7;
    // Values below 15 are invalid and leave the ratio as it was.
    else if (c === 0xa8) {
      if ((a[0]! & 0x3f) >= 15) this.mux = a[0]! & 0x3f;
    } else if (c === 0xae || c === 0xaf) this.displayOn = c === 0xaf;
    else if (c >= 0xb0 && c <= 0xb7) {
      if (this.mode === 2) this.page = c & 7;
    } else if (c === 0xc0 || c === 0xc8) this.comRemap = c === 0xc8;
    else if (c === 0xd3) this.offset = a[0]! & 0x3f;
    // Everything else (timing, scrolling, NOP E3h) changes nothing the image shows.
  }

  image(): Ssd1306Image {
    if (this.cached) return this.cached;
    const rowBytes = WIDTH / 8;
    const pixels = new Uint8Array(rowBytes * this.height);
    const on = this.lit;
    if (on) {
      for (let y = 0; y < this.height; y++) {
        // Only COM0 to COM(mux) are driven; rows past the multiplex ratio stay dark. Modules wire the rows so the
        // remapped scan (C8h) puts COM0 on the top row; the reset scan (C0h) runs the other way, upside down.
        if (y > this.mux) continue;
        const com = this.comRemap ? y : this.mux - y;
        const ramRow = (com + this.startLine + this.offset) & 63;
        const base = (ramRow >> 3) * WIDTH;
        const bit = ramRow & 7;
        for (let x = 0; x < WIDTH; x++) {
          const ramCol = this.segRemap ? x : 127 - x;
          let lit = this.entireOn || ((this.ram[base + ramCol]! >> bit) & 1) === 1;
          if (this.inverse) lit = !lit;
          if (lit) pixels[y * rowBytes + (x >> 3)]! |= 0x80 >> (x & 7);
        }
      }
    }
    this.cached = { width: WIDTH, height: this.height, on, contrast: this.contrast, pixels };
    return this.cached;
  }
}
