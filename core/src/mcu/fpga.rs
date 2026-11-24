//! The iCE40 UltraPlus fabric, ported from brownout's `src/sim/engine/fpga.ts`
//! (`Ice40`), which the lock-step harness (scripts/fpga-lockstep.mjs) holds
//! this to. The reference parses the Yosys netlist, places its ports and
//! levelizes its logic; it hands the result over as flat arrays (`image()`),
//! and this builds the same fabric from them and runs it the same way: logic
//! settles level by level from the cells an edge disturbed, every flip-flop
//! an edge clocks samples before any changes, and time steps from clock edge
//! to clock edge in the same f64 arithmetic.

const OP_LUT: i32 = 0;
const OP_CARRY: i32 = 1;
const OP_BUF: i32 = 2;
/// out = sel ? a : b, as [3, out, sel, a, b]: a bidirectional pad's input.
const OP_MUX: i32 = 3;
// Settling rounds within one instant beyond one per flip-flop before the
// design is taken to oscillate.
const EXTRA_ROUNDS: usize = 64;

#[derive(Clone, Copy, Debug)]
struct Flop {
    d: usize,
    q: usize,
    en: usize,
    /// Set or reset net, if any.
    sr: Option<usize>,
    sr_async: bool,
    sr_value: u8,
}

#[derive(Clone, Copy, Debug)]
struct Pad {
    pin: i32,
    input: i32,
    output: i32,
    enable: i32,
    pullup: bool,
    rgb: bool,
}

#[derive(Clone, Copy, Debug)]
struct Clock {
    net: usize,
    /// While 0 the clock holds low; its phase runs on.
    en: usize,
    half: f64,
    next: f64,
}

/// A design that will not settle, as the reference's `FpgaError`.
#[derive(Debug, PartialEq, Eq)]
pub struct Oscillates;

pub struct Fabric {
    pub nets: Vec<u8>,
    ops: Vec<i32>,
    init: Vec<i32>,
    level: Vec<usize>,
    fanout: Vec<Vec<usize>>,
    dirty: Vec<bool>,
    buckets: Vec<Vec<usize>>,
    dirty_lo: usize,
    dirty_hi: Option<usize>,
    flops: Vec<Flop>,
    rise_of: Vec<Vec<usize>>,
    fall_of: Vec<Vec<usize>>,
    async_of: Vec<Vec<usize>>,
    is_clock: Vec<bool>,
    is_async: Vec<bool>,
    edges: Vec<usize>,
    async_touched: Vec<usize>,
    updates: Vec<(usize, u8)>,
    pads: Vec<Pad>,
    clocks: Vec<Clock>,
    max_rounds: usize,
    pub time: f64,
    pub edge_count: u64,
}

impl Fabric {
    /// Builds the fabric from the reference's `image()`, and settles it from
    /// power-up as the reference's constructor does.
    pub fn from_image(ints: &[i32], floats: &[f64]) -> Result<Fabric, Oscillates> {
        let mut it = ints.iter().copied();
        let mut next = || it.next().unwrap_or(0);
        let (net_count, op_count, levels, flop_count, pad_count, clock_count) =
            (next() as usize, next() as usize, next() as usize, next() as usize, next() as usize, next() as usize);
        let ops: Vec<i32> = (0..op_count * 6).map(|_| next()).collect();
        let init: Vec<i32> = (0..op_count).map(|_| next()).collect();
        let level: Vec<usize> = (0..op_count).map(|_| next() as usize).collect();
        let mut flops = Vec::with_capacity(flop_count);
        let mut clk = Vec::with_capacity(flop_count);
        for _ in 0..flop_count {
            let (d, q, c, neg, en, sr, sr_async, sr_value) = (next(), next(), next(), next(), next(), next(), next(), next());
            flops.push(Flop {
                d: d as usize,
                q: q as usize,
                en: en as usize,
                sr: (sr >= 0).then_some(sr as usize),
                sr_async: sr_async != 0,
                sr_value: sr_value as u8,
            });
            clk.push((c, neg != 0));
        }
        let pads: Vec<Pad> = (0..pad_count)
            .map(|_| {
                let (pin, input, output, enable, pullup, rgb) = (next(), next(), next(), next(), next(), next());
                Pad { pin, input, output, enable, pullup: pullup != 0, rgb: rgb != 0 }
            })
            .collect();
        let clocks: Vec<Clock> = (0..clock_count)
            .map(|i| {
                let half = floats.get(i).copied().unwrap_or(1.0);
                let (net, en) = (next() as usize, next() as usize);
                Clock { net, en, half, next: half }
            })
            .collect();

        // Fanout in cell order, each cell once per net (the reference's Set).
        let mut fanout = vec![Vec::new(); net_count];
        for i in 0..op_count {
            let o = &ops[i * 6..i * 6 + 6];
            let inputs: &[i32] = match o[0] {
                OP_LUT => &o[2..6],
                OP_BUF => &o[2..3],
                OP_CARRY | OP_MUX => &o[2..5],
                _ => &[],
            };
            let mut seen: Vec<i32> = Vec::new();
            for &n in inputs {
                if n > 1 && !seen.contains(&n) {
                    seen.push(n);
                    fanout[n as usize].push(i);
                }
            }
        }
        let mut rise_of = vec![Vec::new(); net_count];
        let mut fall_of = vec![Vec::new(); net_count];
        let mut async_of = vec![Vec::new(); net_count];
        let mut is_clock = vec![false; net_count];
        let mut is_async = vec![false; net_count];
        for (i, f) in flops.iter().enumerate() {
            let (c, neg) = clk[i];
            if c > 1 {
                let c = c as usize;
                let list = if neg { &mut fall_of } else { &mut rise_of };
                list[c].push(i);
                is_clock[c] = true;
            }
            if let (true, Some(sr)) = (f.sr_async, f.sr) {
                // Tied high (net 1), it holds the flip-flop from power-up.
                if sr >= 1 {
                    async_of[sr].push(i);
                    is_async[sr] = true;
                }
            }
        }
        let mut nets = vec![0u8; net_count];
        if net_count > 1 {
            nets[1] = 1;
        }
        for p in &pads {
            if p.input >= 0 && p.pullup {
                nets[p.input as usize] = 1;
            }
        }
        let mut f = Fabric {
            nets,
            ops,
            init,
            level,
            fanout,
            dirty: vec![false; op_count],
            buckets: vec![Vec::new(); levels],
            dirty_lo: 0,
            dirty_hi: None,
            flops,
            rise_of,
            fall_of,
            async_touched: (0..net_count).filter(|&n| !async_of[n].is_empty()).collect(),
            async_of,
            is_clock,
            is_async,
            edges: Vec::new(),
            updates: Vec::new(),
            pads,
            clocks,
            max_rounds: EXTRA_ROUNDS + flop_count,
            time: 0.0,
            edge_count: 0,
        };
        for i in 0..op_count {
            f.mark_dirty(i);
        }
        // Power-up levels are no edge; asynchronous sets and resets hold from
        // the start.
        f.propagate();
        f.edges.clear();
        f.async_touched = (0..net_count).filter(|&n| !f.async_of[n].is_empty()).collect();
        f.settle()?;
        Ok(f)
    }

    fn pad(&self, pin: i32) -> Option<&Pad> {
        self.pads.iter().find(|p| p.pin == pin)
    }

    /// The circuit's level at a package pin (`None`: undriven, the pull-up
    /// if enabled holds it high).
    pub fn set_pin(&mut self, pin: i32, level: Option<bool>) -> Result<(), Oscillates> {
        let Some(p) = self.pad(pin).copied() else { return Ok(()) };
        if p.input < 0 || self.clocks.iter().any(|c| c.net == p.input as usize) {
            return Ok(());
        }
        let v = match level {
            Some(l) => l as u8,
            None => p.pullup as u8,
        };
        if self.nets[p.input as usize] == v {
            return Ok(());
        }
        self.set_net(p.input as usize, v);
        self.settle()
    }

    /// The circuit's level at a pin while the device configures: taken as a
    /// power-up level, with no edge.
    pub fn preset_pin(&mut self, pin: i32, level: Option<bool>) -> Result<(), Oscillates> {
        let Some(p) = self.pad(pin).copied() else { return Ok(()) };
        if p.input < 0 || self.clocks.iter().any(|c| c.net == p.input as usize) {
            return Ok(());
        }
        let v = match level {
            Some(l) => l as u8,
            None => p.pullup as u8,
        };
        self.set_net(p.input as usize, v);
        self.propagate();
        self.edges.clear();
        self.settle()
    }

    /// The design's drive on a package pin, `None` while its output is off.
    pub fn pin_drive(&self, pin: i32) -> Option<bool> {
        let p = self.pad(pin)?;
        if p.output < 0 || p.enable == -2 {
            return None;
        }
        if p.enable >= 0 && self.nets[p.enable as usize] == 0 {
            return None;
        }
        let v = self.nets[p.output as usize] != 0;
        if p.rgb {
            return v.then_some(false);
        }
        // The UP5K's RGB pads (39-41) are open-drain as plain I/O.
        if v && (39..=41).contains(&pin) {
            return None;
        }
        Some(v)
    }

    pub fn advance(&mut self, seconds: f64) -> Result<(), Oscillates> {
        let end = self.time + seconds;
        if self.clocks.is_empty() {
            self.time = end;
            return Ok(());
        }
        loop {
            let mut next = f64::INFINITY;
            let time = self.time;
            for c in self.clocks.iter_mut() {
                if self.nets[c.en] == 0 && self.nets[c.net] == 0 {
                    continue;
                }
                if c.next <= time {
                    c.next += (((time - c.next) / c.half).floor() + 1.0) * c.half;
                    while c.next <= time {
                        c.next += c.half;
                    }
                }
                if c.next < next {
                    next = c.next;
                }
            }
            if next > end {
                break;
            }
            self.time = next;
            for i in 0..self.clocks.len() {
                let c = self.clocks[i];
                if c.next != next || (self.nets[c.en] == 0 && self.nets[c.net] == 0) {
                    continue;
                }
                let v = (self.nets[c.en] != 0 && self.nets[c.net] == 0) as u8;
                self.set_net(c.net, v);
                self.clocks[i].next += c.half;
            }
            self.edge_count += 1;
            self.settle()?;
        }
        self.time = end;
        Ok(())
    }

    fn mark_dirty(&mut self, i: usize) {
        if self.dirty[i] {
            return;
        }
        self.dirty[i] = true;
        let level = self.level[i];
        self.buckets[level].push(i);
        match self.dirty_hi {
            None => {
                self.dirty_lo = level;
                self.dirty_hi = Some(level);
            }
            Some(hi) => {
                if level < self.dirty_lo {
                    self.dirty_lo = level;
                } else if level > hi {
                    self.dirty_hi = Some(level);
                }
            }
        }
    }

    fn set_net(&mut self, n: usize, v: u8) {
        if self.nets[n] == v {
            return;
        }
        self.nets[n] = v;
        for k in 0..self.fanout[n].len() {
            let i = self.fanout[n][k];
            self.mark_dirty(i);
        }
        if self.is_clock[n] {
            self.edges.push(n);
        }
        if self.is_async[n] {
            self.async_touched.push(n);
        }
    }

    fn propagate(&mut self) {
        let mut level = self.dirty_lo;
        while let Some(hi) = self.dirty_hi {
            if level > hi {
                break;
            }
            let bucket = std::mem::take(&mut self.buckets[level]);
            for &i in &bucket {
                self.dirty[i] = false;
                let o = &self.ops[i * 6..i * 6 + 6];
                let n = |k: usize| self.nets[o[k] as usize];
                let v = match o[0] {
                    OP_LUT => {
                        let index = n(2) | n(3) << 1 | n(4) << 2 | n(5) << 3;
                        ((self.init[i] >> index) & 1) as u8
                    }
                    OP_CARRY => {
                        let (a, b, ci) = (n(2), n(3), n(4));
                        (a & b) | ((a | b) & ci)
                    }
                    OP_BUF => n(2),
                    _ => {
                        if n(2) != 0 {
                            n(3)
                        } else {
                            n(4)
                        }
                    }
                };
                let out = o[1] as usize;
                self.set_net(out, v);
            }
            let mut bucket = bucket;
            bucket.clear();
            self.buckets[level] = bucket;
            level += 1;
        }
        self.dirty_lo = 0;
        self.dirty_hi = None;
    }

    fn settle(&mut self) -> Result<(), Oscillates> {
        let mut round = 0;
        loop {
            if round > self.max_rounds {
                return Err(Oscillates);
            }
            round += 1;
            self.propagate();
            if self.edges.is_empty() && self.async_touched.is_empty() {
                return Ok(());
            }
            self.updates.clear();
            let edges = std::mem::take(&mut self.edges);
            for &n in &edges {
                let list = if self.nets[n] != 0 { &self.rise_of[n] } else { &self.fall_of[n] };
                for &fi in list {
                    let f = self.flops[fi];
                    if let (true, Some(sr)) = (f.sr_async, f.sr) {
                        if self.nets[sr] != 0 {
                            continue;
                        }
                    }
                    if self.nets[f.en] == 0 {
                        continue;
                    }
                    let v = match f.sr {
                        Some(sr) if !f.sr_async && self.nets[sr] != 0 => f.sr_value,
                        _ => self.nets[f.d],
                    };
                    self.updates.push((f.q, v));
                }
            }
            let touched = std::mem::take(&mut self.async_touched);
            for &n in &touched {
                if self.nets[n] == 0 {
                    continue;
                }
                for &fi in &self.async_of[n] {
                    let f = self.flops[fi];
                    self.updates.push((f.q, f.sr_value));
                }
            }
            let (mut edges, mut touched) = (edges, touched);
            edges.clear();
            touched.clear();
            self.edges = edges;
            self.async_touched = touched;
            for k in 0..self.updates.len() {
                let (q, v) = self.updates[k];
                self.set_net(q, v);
            }
        }
    }

    /// The reference's `stateHash()`.
    pub fn state_hash(&self) -> u32 {
        let mut h: u32 = 0x811c9dc5;
        for &b in &self.nets {
            h = (h ^ b as u32).wrapping_mul(0x01000193);
        }
        (h ^ self.edge_count as u32).wrapping_mul(0x01000193)
    }
}

// ---- lock-step exports ------------------------------------------------------

/// # Safety
/// `ints`/`floats` point at `ni`/`nf` values. Returns null for a design that
/// does not settle.
#[no_mangle]
pub unsafe extern "C" fn fpga_create(ints: *const i32, ni: u32, floats: *const f64, nf: u32) -> *mut Fabric {
    let ints = std::slice::from_raw_parts(ints, ni as usize);
    let floats = std::slice::from_raw_parts(floats, nf as usize);
    match Fabric::from_image(ints, floats) {
        Ok(f) => Box::into_raw(Box::new(f)),
        Err(_) => std::ptr::null_mut(),
    }
}

/// # Safety
/// `f` from `fpga_create`, not used afterwards.
#[no_mangle]
pub unsafe extern "C" fn fpga_free(f: *mut Fabric) {
    if !f.is_null() {
        drop(Box::from_raw(f));
    }
}

/// Level 0, 1, or 2 for undriven. Returns 0, or 1 if the design oscillated.
///
/// # Safety
/// `f` from `fpga_create`.
#[no_mangle]
pub unsafe extern "C" fn fpga_set_pin(f: *mut Fabric, pin: i32, level: u32) -> u32 {
    let level = match level {
        0 => Some(false),
        1 => Some(true),
        _ => None,
    };
    (*f).set_pin(pin, level).is_err() as u32
}

/// As `fpga_set_pin`, for a level taken during configuration.
///
/// # Safety
/// `f` from `fpga_create`.
#[no_mangle]
pub unsafe extern "C" fn fpga_preset_pin(f: *mut Fabric, pin: i32, level: u32) -> u32 {
    let level = match level {
        0 => Some(false),
        1 => Some(true),
        _ => None,
    };
    (*f).preset_pin(pin, level).is_err() as u32
}

/// 0, 1, or 2 for no drive.
///
/// # Safety
/// `f` from `fpga_create`.
#[no_mangle]
pub unsafe extern "C" fn fpga_pin_drive(f: *mut Fabric, pin: i32) -> u32 {
    match (*f).pin_drive(pin) {
        Some(v) => v as u32,
        None => 2,
    }
}

/// # Safety
/// `f` from `fpga_create`.
#[no_mangle]
pub unsafe extern "C" fn fpga_advance(f: *mut Fabric, seconds: f64) -> u32 {
    (*f).advance(seconds).is_err() as u32
}

/// # Safety
/// `f` from `fpga_create`.
#[no_mangle]
pub unsafe extern "C" fn fpga_state_hash(f: *mut Fabric) -> u32 {
    (*f).state_hash()
}

/// # Safety
/// `f` from `fpga_create`.
#[no_mangle]
pub unsafe extern "C" fn fpga_edge_count(f: *mut Fabric) -> f64 {
    (*f).edge_count as f64
}

/// Scratch memory for the harness to write an image into, 8-byte aligned
/// because `fpga_create` reads it as i32 and f64 slices. Never freed: the
/// harness takes one per fabric.
#[no_mangle]
pub extern "C" fn fpga_alloc(bytes: u32) -> *mut u8 {
    let mut v = vec![0u64; (bytes as usize).div_ceil(8)];
    let p = v.as_mut_ptr() as *mut u8;
    std::mem::forget(v);
    p
}

#[cfg(test)]
#[path = "fpga_tests.rs"]
mod fpga_tests;
