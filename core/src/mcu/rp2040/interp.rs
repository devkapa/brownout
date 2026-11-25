//! The SIO interpolators, ported from rp2040js 1.3.3 (MIT):
//! `src/interpolator.ts`.
//!
//! The reference sums on doubles and truncates to 32 bits only when it
//! stores a result; every such sum is truncated in the end, so it is done
//! here with wrapping u32 arithmetic, which leaves the same bits.

#[derive(Clone, Copy, Default)]
struct Config {
    shift: u32,
    mask_lsb: u32,
    mask_msb: u32,
    signed: bool,
    cross_input: bool,
    cross_result: bool,
    add_raw: bool,
    force_msb: u32,
    blend: bool,
    clamp: bool,
    overf0: bool,
    overf1: bool,
    overf: bool,
}

impl Config {
    fn from_u32(v: u32) -> Self {
        Config {
            shift: v & 0b11111,
            mask_lsb: (v >> 5) & 0b11111,
            mask_msb: (v >> 10) & 0b11111,
            signed: (v >> 15) & 1 != 0,
            cross_input: (v >> 16) & 1 != 0,
            cross_result: (v >> 17) & 1 != 0,
            add_raw: (v >> 18) & 1 != 0,
            force_msb: (v >> 19) & 0b11,
            blend: (v >> 21) & 1 != 0,
            clamp: (v >> 22) & 1 != 0,
            overf0: (v >> 23) & 1 != 0,
            overf1: (v >> 24) & 1 != 0,
            overf: (v >> 25) & 1 != 0,
        }
    }
    fn to_u32(self) -> u32 {
        self.shift
            | (self.mask_lsb << 5)
            | (self.mask_msb << 10)
            | ((self.signed as u32) << 15)
            | ((self.cross_input as u32) << 16)
            | ((self.cross_result as u32) << 17)
            | ((self.add_raw as u32) << 18)
            | (self.force_msb << 19)
            | ((self.blend as u32) << 21)
            | ((self.clamp as u32) << 22)
            | ((self.overf0 as u32) << 23)
            | ((self.overf1 as u32) << 24)
            | ((self.overf as u32) << 25)
    }
}

#[derive(Clone)]
pub struct Interpolator {
    pub index: u32,
    pub accum0: u32,
    pub accum1: u32,
    pub base0: u32,
    pub base1: u32,
    pub base2: u32,
    pub ctrl0: u32,
    pub ctrl1: u32,
    pub result0: u32,
    pub result1: u32,
    pub result2: u32,
    pub smresult0: u32,
    pub smresult1: u32,
}

fn msb_mask(msb: u32) -> u32 {
    if msb == 31 {
        0xffff_ffff
    } else {
        (1u32 << (msb + 1)).wrapping_sub(1)
    }
}

/// The JavaScript `Math.floor(alpha * (b1 - b0) / 256) | 0`.
fn blend_step(alpha: u32, b0: i64, b1: i64) -> u32 {
    ((alpha as i64) * (b1 - b0)).div_euclid(256) as u32
}

impl Interpolator {
    pub fn new(index: u32) -> Self {
        let mut i = Interpolator {
            index,
            accum0: 0,
            accum1: 0,
            base0: 0,
            base1: 0,
            base2: 0,
            ctrl0: 0,
            ctrl1: 0,
            result0: 0,
            result1: 0,
            result2: 0,
            smresult0: 0,
            smresult1: 0,
        };
        i.update();
        i
    }

    pub fn update(&mut self) {
        let n = self.index;
        let mut c0 = Config::from_u32(self.ctrl0);
        let mut c1 = Config::from_u32(self.ctrl1);
        let do_clamp = c0.clamp && n == 1;
        let do_blend = c0.blend && n == 0;
        c0.clamp = do_clamp;
        c0.blend = do_blend;
        c1.clamp = false;
        c1.blend = false;
        c1.overf0 = false;
        c1.overf1 = false;
        c1.overf = false;
        let input0 = if c0.cross_input { self.accum1 } else { self.accum0 };
        let input1 = if c1.cross_input { self.accum0 } else { self.accum1 };
        let msbmask0 = msb_mask(c0.mask_msb);
        let msbmask1 = msb_mask(c1.mask_msb);
        let mask0 = msbmask0 & !((1u32 << c0.mask_lsb).wrapping_sub(1));
        let mask1 = msbmask1 & !((1u32 << c1.mask_lsb).wrapping_sub(1));
        let uresult0 = (input0 >> c0.shift) & mask0;
        let uresult1 = (input1 >> c1.shift) & mask1;
        let overf0 = (input0 >> c0.shift) & !msbmask0 != 0;
        let overf1 = (input1 >> c1.shift) & !msbmask1 != 0;
        let overf = overf0 || overf1;
        let sext0 = if uresult0 & (1 << c0.mask_msb) != 0 { (-1i32 << c0.mask_msb) as u32 } else { 0 };
        let sext1 = if uresult1 & (1 << c1.mask_msb) != 0 { (-1i32 << c1.mask_msb) as u32 } else { 0 };
        let result0 = if c0.signed { uresult0 | sext0 } else { uresult0 };
        let result1 = if c1.signed { uresult1 | sext1 } else { uresult1 };
        let addresult0 = self.base0.wrapping_add(if c0.add_raw { input0 } else { result0 });
        let addresult1 = self.base1.wrapping_add(if c1.add_raw { input1 } else { result1 });
        let addresult2 = self.base2.wrapping_add(result0).wrapping_add(if do_blend { 0 } else { result1 });
        let uclamp0 = if result0 < self.base0 {
            self.base0
        } else if result0 > self.base1 {
            self.base1
        } else {
            result0
        };
        let sclamp0 = if (result0 as i32) < (self.base0 as i32) {
            self.base0
        } else if (result0 as i32) > (self.base1 as i32) {
            self.base1
        } else {
            result0
        };
        let clamp0 = if c0.signed { sclamp0 } else { uclamp0 };
        let alpha1 = result1 & 0xff;
        let ublend1 = self.base0.wrapping_add(blend_step(alpha1, self.base0 as i64, self.base1 as i64));
        let sblend1 =
            self.base0.wrapping_add(blend_step(alpha1, self.base0 as i32 as i64, self.base1 as i32 as i64));
        let blend1 = if c1.signed { sblend1 } else { ublend1 };
        self.smresult0 = result0;
        self.smresult1 = result1;
        // Reference quirk: lane 1 takes FORCE_MSB from CTRL_LANE0.
        self.result0 = if do_blend { alpha1 } else { (if do_clamp { clamp0 } else { addresult0 }) | (c0.force_msb << 28) };
        self.result1 = (if do_blend { blend1 } else { addresult1 }) | (c0.force_msb << 28);
        self.result2 = addresult2;
        c0.overf0 = overf0;
        c0.overf1 = overf1;
        c0.overf = overf;
        self.ctrl0 = c0.to_u32();
        self.ctrl1 = c1.to_u32();
    }

    pub fn writeback(&mut self) {
        let c0 = Config::from_u32(self.ctrl0);
        let c1 = Config::from_u32(self.ctrl1);
        self.accum0 = if c0.cross_result { self.result1 } else { self.result0 };
        self.accum1 = if c1.cross_result { self.result0 } else { self.result1 };
        self.update();
    }

    pub fn set_base01(&mut self, value: u32) {
        let c0 = Config::from_u32(self.ctrl0);
        let c1 = Config::from_u32(self.ctrl1);
        let do_blend = c0.blend && self.index == 0;
        let input0 = value & 0xffff;
        let input1 = (value >> 16) & 0xffff;
        let sext0 = if input0 & (1 << 15) != 0 { 0xffff_8000 } else { 0 };
        let sext1 = if input1 & (1 << 15) != 0 { 0xffff_8000 } else { 0 };
        let signed0 = if do_blend { c1.signed } else { c0.signed };
        self.base0 = if signed0 { input0 | sext0 } else { input0 };
        self.base1 = if c1.signed { input1 | sext1 } else { input1 };
        self.update();
    }

    /// A read at `reg` (offset within this interpolator's 0x40 block);
    /// `None` where the reference has no register.
    pub fn read(&mut self, reg: u32) -> Option<u32> {
        Some(match reg {
            0x00 => self.accum0,
            0x04 => self.accum1,
            0x08 => self.base0,
            0x0c => self.base1,
            0x10 => self.base2,
            0x14 => {
                let v = self.result0;
                self.writeback();
                v
            }
            0x18 => {
                let v = self.result1;
                self.writeback();
                v
            }
            0x1c => {
                let v = self.result2;
                self.writeback();
                v
            }
            0x20 => self.result0,
            0x24 => self.result1,
            0x28 => self.result2,
            0x2c => self.ctrl0,
            0x30 => self.ctrl1,
            0x34 => self.smresult0,
            0x38 => self.smresult1,
            _ => return None,
        })
    }

    /// A write at `reg`; false where the reference has no register.
    pub fn write(&mut self, reg: u32, value: u32) -> bool {
        match reg {
            0x00 => self.accum0 = value,
            0x04 => self.accum1 = value,
            0x08 => self.base0 = value,
            0x0c => self.base1 = value,
            0x10 => self.base2 = value,
            0x2c => self.ctrl0 = value,
            0x30 => self.ctrl1 = value,
            0x34 => self.accum0 = self.accum0.wrapping_add(value),
            0x38 => self.accum1 = self.accum1.wrapping_add(value),
            0x3c => {
                self.set_base01(value);
                return true;
            }
            _ => return false,
        }
        self.update();
        true
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for Config {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shift, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.mask_lsb, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.mask_msb, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.signed, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cross_input, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cross_result, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.add_raw, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.force_msb, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.blend, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.clamp, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.overf0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.overf1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.overf, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Interpolator {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.accum0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.accum1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.base0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.base1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.base2, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ctrl0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.ctrl1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.result0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.result1, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.result2, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.smresult0, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.smresult1, w); }
        n
    }
}
