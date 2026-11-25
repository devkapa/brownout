//! The SSD1306 OLED controller on its I2C interface, ported from brownout's
//! `src/sim/engine/i2c-devices.ts` (`Ssd1306`), which the lock-step harness
//! holds this to. After the address, each byte pair or stream starts with a
//! control byte: D/C (bit 6) says whether commands or GDDRAM data follow, and
//! Co (bit 7) set means only the next byte is, after which another control
//! byte comes (datasheet rev 1.1 section 8.1.5). GDDRAM is 128 columns by 8
//! pages, written in page, horizontal or vertical addressing (section
//! 10.1.3). The image scans GDDRAM onto the rows with the start line, offset,
//! multiplex ratio, segment remap, COM scan direction, entire-display-on and
//! inverse settings; scrolling is parsed but not drawn.

const WIDTH: usize = 128;
const PAGES: usize = 8;

/// Argument bytes after each multi-byte command (datasheet Table 9-1), 0
/// for a command that takes none.
fn arguments(c: u8) -> usize {
    match c {
        0x20 | 0x81 | 0x8d | 0xa8 | 0xd3 | 0xd5 | 0xd9 | 0xda | 0xdb => 1,
        0x21 | 0x22 | 0xa3 => 2,
        0x29 | 0x2a => 5,
        0x26 | 0x27 => 6,
        _ => 0,
    }
}

/// What the panel shows: 1 bit a pixel, rows packed MSB first.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Ssd1306Image {
    pub width: usize,
    pub height: usize,
    /// Display on (AFh) and charge pump enabled (8Dh 14h).
    pub on: bool,
    pub contrast: u8,
    pub pixels: Vec<u8>,
}

#[derive(Clone, Debug)]
pub struct Ssd1306 {
    pub address: u8,
    pub height: usize,
    /// Supplied; off until the host first finds it so.
    pub powered: bool,
    pub ram: [u8; WIDTH * PAGES],
    pub expect_control: bool,
    pub single: bool,
    pub data_mode: bool,
    /// A multi-byte command waiting for its arguments.
    pub command: Option<u8>,
    pub args: Vec<u8>,
    pub display_on: bool,
    pub charge_pump: bool,
    /// The panel's VCC is supplied from outside (SSD1306_EXTERNALVCC
    /// wiring), not by the charge pump the common modules rely on.
    pub external_vcc: bool,
    pub contrast: u8,
    pub mode: u8,
    pub col_start: u8,
    pub col_end: u8,
    pub page_start: u8,
    pub page_end: u8,
    pub col: u8,
    pub page: u8,
    pub page_mode_col: u8,
    pub start_line: u8,
    pub offset: u8,
    pub mux: u8,
    pub seg_remap: bool,
    pub com_remap: bool,
    pub entire_on: bool,
    pub inverse: bool,
}

impl Ssd1306 {
    pub fn new(address: u8, height: usize) -> Ssd1306 {
        let mut s = Ssd1306 {
            address,
            height,
            powered: false,
            ram: [0; WIDTH * PAGES],
            expect_control: true,
            single: false,
            data_mode: false,
            command: None,
            args: Vec::new(),
            display_on: false,
            charge_pump: false,
            external_vcc: false,
            contrast: 0x7f,
            mode: 2,
            col_start: 0,
            col_end: 127,
            page_start: 0,
            page_end: 7,
            col: 0,
            page: 0,
            page_mode_col: 0,
            start_line: 0,
            offset: 0,
            mux: 63,
            seg_remap: false,
            com_remap: false,
            entire_on: false,
            inverse: false,
        };
        s.reset();
        s
    }

    /// Power-on reset. `single` and `data_mode` keep their values, as the
    /// reference leaves them (the next control byte sets both).
    pub fn reset(&mut self) {
        // GDDRAM is undefined at power-on; cleared so a panel comes up dark.
        self.ram = [0; WIDTH * PAGES];
        self.expect_control = true;
        self.command = None;
        self.args.clear();
        self.display_on = false;
        self.charge_pump = false;
        self.contrast = 0x7f;
        self.mode = 2;
        self.col_start = 0;
        self.col_end = 127;
        self.page_start = 0;
        self.page_end = 7;
        self.col = 0;
        self.page = 0;
        self.page_mode_col = 0;
        self.start_line = 0;
        self.offset = 0;
        self.mux = 63;
        self.seg_remap = false;
        self.com_remap = false;
        self.entire_on = false;
        self.inverse = false;
    }

    pub fn start(&mut self) {
        self.expect_control = true;
    }

    pub fn stop(&mut self) {
        self.expect_control = true;
    }

    pub fn write(&mut self, value: u8) -> bool {
        if self.expect_control {
            self.single = value & 0x80 != 0;
            self.data_mode = value & 0x40 != 0;
            self.expect_control = false;
            return true;
        }
        if self.data_mode {
            self.write_data(value);
        } else {
            self.write_command(value);
        }
        if self.single {
            self.expect_control = true;
        }
        true
    }

    /// The status byte: bit 6 set while the display is off.
    pub fn read(&mut self) -> u8 {
        if self.display_on { 0x00 } else { 0x40 }
    }

    pub fn lit(&self) -> bool {
        self.display_on && (self.charge_pump || self.external_vcc)
    }

    fn write_data(&mut self, value: u8) {
        self.ram[self.page as usize * WIDTH + self.col as usize] = value;
        if self.mode == 2 {
            self.col = if self.col >= 127 { self.page_mode_col } else { self.col + 1 };
        } else if self.mode == 0 {
            if self.col >= self.col_end {
                self.col = self.col_start;
                self.page = if self.page >= self.page_end { self.page_start } else { self.page + 1 };
            } else {
                self.col += 1;
            }
        } else if self.page >= self.page_end {
            self.page = self.page_start;
            self.col = if self.col >= self.col_end { self.col_start } else { self.col + 1 };
        } else {
            self.page += 1;
        }
    }

    fn write_command(&mut self, value: u8) {
        if let Some(c) = self.command {
            self.args.push(value);
            if self.args.len() == arguments(c) {
                let a = std::mem::take(&mut self.args);
                self.apply(c, &a);
                self.command = None;
            }
            return;
        }
        if arguments(value) > 0 {
            self.command = Some(value);
            self.args.clear();
            return;
        }
        self.apply(value, &[]);
    }

    fn apply(&mut self, c: u8, a: &[u8]) {
        match c {
            0x00..=0x0f => {
                self.page_mode_col = (self.page_mode_col & 0xf0) | c;
                if self.mode == 2 {
                    self.col = self.page_mode_col;
                }
            }
            0x10..=0x1f => {
                self.page_mode_col = ((c & 0x07) << 4) | (self.page_mode_col & 0x0f);
                if self.mode == 2 {
                    self.col = self.page_mode_col;
                }
            }
            // 11b is invalid and leaves the mode as it was.
            0x20 => {
                if a[0] & 3 != 3 {
                    self.mode = a[0] & 3;
                }
            }
            0x21 => {
                self.col_start = a[0] & 0x7f;
                self.col_end = a[1] & 0x7f;
                self.col = self.col_start;
            }
            0x22 => {
                self.page_start = a[0] & 7;
                self.page_end = a[1] & 7;
                self.page = self.page_start;
            }
            0x40..=0x7f => self.start_line = c & 0x3f,
            0x81 => self.contrast = a[0],
            0x8d => self.charge_pump = a[0] & 0x04 != 0,
            0xa0 | 0xa1 => self.seg_remap = c == 0xa1,
            0xa4 | 0xa5 => self.entire_on = c == 0xa5,
            0xa6 | 0xa7 => self.inverse = c == 0xa7,
            // Below 15 is invalid and leaves the ratio as it was.
            0xa8 => {
                if a[0] & 0x3f >= 15 {
                    self.mux = a[0] & 0x3f;
                }
            }
            0xae | 0xaf => self.display_on = c == 0xaf,
            0xb0..=0xb7 => {
                if self.mode == 2 {
                    self.page = c & 7;
                }
            }
            0xc0 | 0xc8 => self.com_remap = c == 0xc8,
            0xd3 => self.offset = a[0] & 0x3f,
            // Timing, scrolling and NOP change nothing the image shows.
            _ => {}
        }
    }

    /// The scanned-out image.
    pub fn image(&self) -> Ssd1306Image {
        let row_bytes = WIDTH / 8;
        let mut pixels = vec![0u8; row_bytes * self.height];
        let on = self.lit();
        if on {
            for y in 0..self.height {
                // Rows past the multiplex ratio stay dark; the remapped scan
                // (C8h) puts COM0 on the top row, as modules wire it.
                if y > self.mux as usize {
                    continue;
                }
                let com = if self.com_remap { y } else { self.mux as usize - y };
                let ram_row = (com + self.start_line as usize + self.offset as usize) & 63;
                let base = (ram_row >> 3) * WIDTH;
                let bit = ram_row & 7;
                for x in 0..WIDTH {
                    let ram_col = if self.seg_remap { x } else { 127 - x };
                    let mut lit = self.entire_on || (self.ram[base + ram_col] >> bit) & 1 == 1;
                    if self.inverse {
                        lit = !lit;
                    }
                    if lit {
                        pixels[y * row_bytes + (x >> 3)] |= 0x80 >> (x & 7);
                    }
                }
            }
        }
        Ssd1306Image { width: WIDTH, height: self.height, on, contrast: self.contrast, pixels }
    }

    /// The state the lock-step harness compares, as words (its JS side
    /// builds the same list from the reference's fields).
    pub fn state_words(&self, out: &mut Vec<u32>) {
        out.push(super::harness::fnv1a(&self.ram));
        out.push(self.command.map_or(0, |c| c as u32 + 1));
        out.push(self.args.len() as u32);
        out.extend(self.args.iter().map(|&b| b as u32));
        for b in [
            self.expect_control, self.single, self.data_mode, self.display_on, self.charge_pump,
            self.seg_remap, self.com_remap, self.entire_on, self.inverse,
        ] {
            out.push(b as u32);
        }
        for v in [
            self.contrast, self.mode, self.col_start, self.col_end, self.page_start, self.page_end, self.col,
            self.page, self.page_mode_col, self.start_line, self.offset, self.mux,
        ] {
            out.push(v as u32);
        }
        out.push(self.external_vcc as u32);
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Ssd1306Image {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.width, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.height, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.on, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.contrast, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pixels, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Ssd1306 {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.address, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.height, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.powered, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ram, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.expect_control, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.single, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.data_mode, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.command, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.args, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.display_on, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.charge_pump, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.external_vcc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.contrast, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.mode, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.col_start, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.col_end, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.page_start, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.page_end, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.col, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.page, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.page_mode_col, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.start_line, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.offset, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.mux, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.seg_remap, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.com_remap, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.entire_on, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.inverse, w); }
        n
    }
}
