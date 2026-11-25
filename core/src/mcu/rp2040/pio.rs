//! The two PIO blocks, ported from rp2040js 1.3.3 (MIT):
//! `src/peripherals/pio.ts` (and the FIFO of `src/utils/fifo.ts`). Four state
//! machines per block, the full instruction set, 4-deep FIFOs, the IRQ flags,
//! pin mapping through SET, OUT, IN and side-set, and the two interrupt
//! lines.
//!
//! How rp2040js runs it, which the port keeps (each is a quirk with a test):
//!
//! - Time. `RPPIO.run()` executes 1000 steps at once and re-arms itself with
//!   `setTimeout(..., 0)`: a block advances by wall-clock event-loop turns,
//!   not by simulated time, and the clock dividers (`SMx_CLKDIV`) are stored
//!   and read back but never used. The port has the same shape. Enabling a
//!   machine from stopped runs the first 1000 steps inside the register write
//!   itself; each later batch of 1000 runs when the host calls
//!   [`Rp2040::pio_host_turn`] (the event-loop turn: the harness calls it every
//!   N steps on both sides, the wrapper once per host slice). Timers fire
//!   in the order they were created; one a turn creates waits for the next.
//! - Every state machine of a running block steps, enabled or not: `step()`
//!   never reads `enabled`.
//! - FIFO join (`FJOIN_RX` and `FJOIN_TX`) is not modelled and writes to
//!   SHIFTCTRL clear no FIFO.
//!
//! JavaScript numerics: the registers the reference keeps as numbers are held
//! here as 32-bit words, which is exact because every operation on them is
//! bitwise (the one that is not, `x - 1`, goes through `>>> 0`).

use super::dma::dreq;
use super::Rp2040;

// Generic registers.
const CTRL: u32 = 0x000;
const FSTAT: u32 = 0x004;
const FDEBUG: u32 = 0x008;
const FLEVEL: u32 = 0x00c;
const IRQ: u32 = 0x030;
const IRQ_FORCE: u32 = 0x034;
const INPUT_SYNC_BYPASS: u32 = 0x038;
const DBG_PADOUT: u32 = 0x03c;
const DBG_PADOE: u32 = 0x040;
const DBG_CFGINFO: u32 = 0x044;
const INSTR_MEM0: u32 = 0x48;
const INSTR_MEM31: u32 = 0x0c4;
const INTR: u32 = 0x128;
const IRQ0_INTE: u32 = 0x12c;
const IRQ0_INTF: u32 = 0x130;
const IRQ0_INTS: u32 = 0x134;
const IRQ1_INTE: u32 = 0x138;
const IRQ1_INTF: u32 = 0x13c;
const IRQ1_INTS: u32 = 0x140;

const TXF0: u32 = 0x010;
const TXF1: u32 = 0x014;
const TXF2: u32 = 0x018;
const TXF3: u32 = 0x01c;
const RXF0: u32 = 0x020;
const RXF1: u32 = 0x024;
const RXF2: u32 = 0x028;
const RXF3: u32 = 0x02c;
const SM_CLKDIV: [u32; 4] = [0x0c8, 0x0e0, 0x0f8, 0x110];
const SM_PINCTRL: [u32; 4] = [0x0dc, 0x0f4, 0x10c, 0x124];

// Register offsets within one state machine's window, from SM0_CLKDIV.
const SM_CLKDIV_R: u32 = 0x00;
const SM_EXECCTRL_R: u32 = 0x04;
const SM_SHIFTCTRL_R: u32 = 0x08;
const SM_ADDR_R: u32 = 0x0c;
const SM_INSTR_R: u32 = 0x10;
const SM_PINCTRL_R: u32 = 0x14;

const FSTAT_TXEMPTY: u32 = 1 << 24;
const FSTAT_TXFULL: u32 = 1 << 16;
const FSTAT_RXEMPTY: u32 = 1 << 8;
const FSTAT_RXFULL: u32 = 1 << 0;

const FDEBUG_TXSTALL: u32 = 1 << 24;
const FDEBUG_TXOVER: u32 = 1 << 16;
const FDEBUG_RXUNDER: u32 = 1 << 8;
const FDEBUG_RXSTALL: u32 = 1 << 0;

const SHIFTCTRL_AUTOPUSH: u32 = 1 << 16;
const SHIFTCTRL_AUTOPULL: u32 = 1 << 17;
const SHIFTCTRL_IN_SHIFTDIR: u32 = 1 << 18;
const SHIFTCTRL_OUT_SHIFTDIR: u32 = 1 << 19;

const EXECCTRL_STATUS_SEL: u32 = 1 << 4;
const EXECCTRL_SIDE_PINDIR: u32 = 1 << 29;
const EXECCTRL_SIDE_EN: u32 = 1 << 30;
const EXECCTRL_EXEC_STALLED: u32 = 1 << 31;

/// `IRQ.PIO0_IRQ0`; PIO1's is two higher. Each block's second line follows.
const IRQ_PIO0_IRQ0: u32 = 7;

/// `FUNCTION_PIO0` and `FUNCTION_PIO1` of the pin function select.
pub const FUNCTION_PIO0: u32 = 6;
pub const FUNCTION_PIO1: u32 = 7;

/// How many steps `run()` takes before it yields to the event loop.
const RUN_BATCH: u32 = 1000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WaitType {
    None = 0,
    Pin = 1,
    RxFifo = 2,
    TxFifo = 3,
    Irq = 4,
    Out = 5,
}

/// `FIFO(4)`.
#[derive(Clone, Copy, Default)]
pub struct Fifo {
    pub buf: [u32; 4],
    start: usize,
    used: usize,
}

impl Fifo {
    pub fn item_count(&self) -> u32 {
        self.used as u32
    }
    pub fn empty(&self) -> bool {
        self.used == 0
    }
    pub fn full(&self) -> bool {
        self.used == 4
    }
    /// Pushing into a full FIFO does nothing.
    pub fn push(&mut self, value: u32) {
        if self.used < 4 {
            self.buf[(self.start + self.used) % 4] = value;
            self.used += 1;
        }
    }
    /// Pulling from an empty FIFO answers 0.
    pub fn pull(&mut self) -> u32 {
        if self.used != 0 {
            let v = self.buf[self.start];
            self.start = (self.start + 1) % 4;
            self.used -= 1;
            v
        } else {
            0
        }
    }
    pub fn items(&self) -> impl Iterator<Item = u32> + '_ {
        (0..self.used).map(move |i| self.buf[(self.start + i) % 4])
    }
}

fn bit_reverse(mut x: u32) -> u32 {
    x = ((x & 0x5555_5555) << 1) | ((x & 0xaaaa_aaaa) >> 1);
    x = ((x & 0x3333_3333) << 2) | ((x & 0xcccc_cccc) >> 2);
    x = ((x & 0x0f0f_0f0f) << 4) | ((x & 0xf0f0_f0f0) >> 4);
    x = ((x & 0x00ff_00ff) << 8) | ((x & 0xff00_ff00) >> 8);
    x.rotate_left(16)
}

/// `irqIndex`: bit 4 makes the index relative to the machine.
fn irq_index(irq: u32, machine: usize) -> u32 {
    if irq & 0x10 != 0 {
        (irq & 0x4) | (((irq & 0x3) + machine as u32) & 0x3)
    } else {
        irq & 0x7
    }
}

/// `(1 << n) - 1` as JavaScript computes it before a bitwise operator
/// (n = 31 reaches `-2147483649`, whose ToInt32 is 0x7fffffff).
fn low_mask(n: u32) -> u32 {
    (1u32 << (n & 31)).wrapping_sub(1)
}

#[derive(Clone)]
pub struct StateMachine {
    pub enabled: bool,
    pub x: u32,
    pub y: u32,
    pub pc: u32,
    pub input_shift_reg: u32,
    pub input_shift_count: u32,
    pub output_shift_reg: u32,
    pub output_shift_count: u32,
    pub cycles: i64,
    pub exec_opcode: u32,
    pub exec_valid: bool,
    pub update_pc: bool,
    pub clock_div_int: u32,
    pub clock_div_frac: u32,
    pub exec_ctrl: u32,
    pub shift_ctrl: u32,
    pub pin_ctrl: u32,
    pub rx_fifo: Fifo,
    pub tx_fifo: Fifo,
    pub out_pin_values: u32,
    pub out_pin_direction: u32,
    pub waiting: bool,
    pub wait_type: WaitType,
    pub wait_index: u32,
    pub wait_polarity: bool,
    pub wait_delay: i64,
}

impl StateMachine {
    fn new() -> Self {
        StateMachine {
            enabled: false,
            x: 0,
            y: 0,
            pc: 0,
            input_shift_reg: 0,
            input_shift_count: 0,
            output_shift_reg: 0,
            output_shift_count: 0,
            cycles: 0,
            exec_opcode: 0,
            exec_valid: false,
            update_pc: true,
            clock_div_int: 1,
            clock_div_frac: 0,
            exec_ctrl: 0x1f << 12,
            shift_ctrl: 0b11 << 18,
            pin_ctrl: 0x5 << 26,
            rx_fifo: Fifo::default(),
            tx_fifo: Fifo::default(),
            out_pin_values: 0,
            out_pin_direction: 0,
            waiting: false,
            wait_type: WaitType::None,
            wait_index: 0,
            wait_polarity: false,
            wait_delay: -1,
        }
    }

    fn push_threshold(&self) -> u32 {
        let v = (self.shift_ctrl >> 20) & 0x1f;
        if v != 0 {
            v
        } else {
            32
        }
    }
    fn pull_threshold(&self) -> u32 {
        let v = (self.shift_ctrl >> 25) & 0x1f;
        if v != 0 {
            v
        } else {
            32
        }
    }
    fn sideset_count(&self) -> u32 {
        (self.pin_ctrl >> 29) & 0x7
    }
    fn set_count(&self) -> u32 {
        (self.pin_ctrl >> 26) & 0x7
    }
    fn out_count(&self) -> u32 {
        (self.pin_ctrl >> 20) & 0x3f
    }
    fn in_base(&self) -> u32 {
        (self.pin_ctrl >> 15) & 0x1f
    }
    fn sideset_base(&self) -> u32 {
        (self.pin_ctrl >> 10) & 0x1f
    }
    fn set_base(&self) -> u32 {
        (self.pin_ctrl >> 5) & 0x1f
    }
    fn out_base(&self) -> u32 {
        self.pin_ctrl & 0x1f
    }
    fn jmp_pin(&self) -> u32 {
        (self.exec_ctrl >> 24) & 0x1f
    }
    fn wrap_top(&self) -> u32 {
        (self.exec_ctrl >> 12) & 0x1f
    }
    fn wrap_bottom(&self) -> u32 {
        (self.exec_ctrl >> 7) & 0x1f
    }

    /// `status`: all ones when the selected FIFO holds fewer than N items.
    fn status(&self) -> u32 {
        let n = self.exec_ctrl & 0xf;
        let count = if self.exec_ctrl & EXECCTRL_STATUS_SEL != 0 {
            self.rx_fifo.item_count()
        } else {
            self.tx_fifo.item_count()
        };
        if count < n {
            0xffff_ffff
        } else {
            0
        }
    }

    fn fifo_stat(&self, index: usize) -> u32 {
        let r = (if self.tx_fifo.empty() { FSTAT_TXEMPTY } else { 0 })
            | (if self.tx_fifo.full() { FSTAT_TXFULL } else { 0 })
            | (if self.rx_fifo.empty() { FSTAT_RXEMPTY } else { 0 })
            | (if self.rx_fifo.full() { FSTAT_RXFULL } else { 0 });
        r << index
    }

    fn next_pc(&mut self) {
        if self.pc == self.wrap_top() {
            self.pc = self.wrap_bottom();
        } else {
            self.pc = (self.pc + 1) & 0x1f;
        }
    }

    /// `wait()`.
    fn wait(&mut self, ty: WaitType, polarity: bool, index: u32) {
        self.waiting = true;
        self.wait_type = ty;
        self.wait_polarity = polarity;
        self.wait_index = index;
        self.wait_delay = -1;
        self.update_pc = false;
    }

    /// `restart()`.
    fn restart(&mut self) {
        self.cycles = 0;
        self.input_shift_count = 0;
        self.output_shift_count = 32;
        self.input_shift_reg = 0;
        self.waiting = false;
    }
}

pub struct Pio {
    pub instructions: [u32; 32],
    pub machines: [StateMachine; 4],
    pub stopped: bool,
    pub fdebug: u32,
    pub tx_stall: u32,
    pub rx_stall: u32,
    pub input_sync_bypass: u32,
    pub irq: u32,
    pub pin_values: u32,
    pub pin_directions: u32,
    pub old_pin_values: u32,
    pub old_pin_directions: u32,
    pub irq0_int_enable: u32,
    pub irq0_int_force: u32,
    pub irq1_int_enable: u32,
    pub irq1_int_force: u32,
}

impl Pio {
    /// The block as constructed. Each machine's constructor also sets its two
    /// request lines (RX empty: low; TX not full: high), which
    /// `Periph::new` applies to the DMA block.
    pub fn new() -> Self {
        Pio {
            instructions: [0; 32],
            machines: std::array::from_fn(|_| StateMachine::new()),
            stopped: true,
            fdebug: 0,
            tx_stall: 0,
            rx_stall: 0,
            input_sync_bypass: 0,
            irq: 0,
            pin_values: 0,
            pin_directions: 0,
            old_pin_values: 0,
            old_pin_directions: 0,
            irq0_int_enable: 0,
            irq0_int_force: 0,
            irq1_int_enable: 0,
            irq1_int_force: 0,
        }
    }

    pub fn int_raw(&self) -> u32 {
        let m = &self.machines;
        ((self.irq & 0xf) << 8)
            | (if !m[3].tx_fifo.full() { 0x80 } else { 0 })
            | (if !m[2].tx_fifo.full() { 0x40 } else { 0 })
            | (if !m[1].tx_fifo.full() { 0x20 } else { 0 })
            | (if !m[0].tx_fifo.full() { 0x10 } else { 0 })
            | (if !m[3].rx_fifo.empty() { 0x08 } else { 0 })
            | (if !m[2].rx_fifo.empty() { 0x04 } else { 0 })
            | (if !m[1].rx_fifo.empty() { 0x02 } else { 0 })
            | (if !m[0].rx_fifo.empty() { 0x01 } else { 0 })
    }

    pub fn irq0_int_status(&self) -> u32 {
        (self.int_raw() & self.irq0_int_enable) | self.irq0_int_force
    }

    pub fn irq1_int_status(&self) -> u32 {
        (self.int_raw() & self.irq1_int_enable) | self.irq1_int_force
    }

    /// `pinValuesChanged` / `pinDirectionsChanged`: the new word for a pin
    /// register. A count over 31 selects all 32 bits (wrapping after pin 31
    /// is a TODO in the reference).
    fn merged(old: u32, value: u32, first_pin: u32, count: u32) -> u32 {
        let mask = if count > 31 { 0xffff_ffff } else { low_mask(count) << (first_pin & 31) };
        ((old & !mask) | ((value << (first_pin & 31)) & mask)) & 0x3fff_ffff
    }

    pub fn pin_values_changed(&mut self, value: u32, first_pin: u32, count: u32) {
        self.pin_values = Self::merged(self.pin_values, value, first_pin, count);
    }

    pub fn pin_directions_changed(&mut self, value: u32, first_pin: u32, count: u32) {
        self.pin_directions = Self::merged(self.pin_directions, value, first_pin, count);
    }

    /// FNV-1a over every machine's registers, FIFOs and wait state.
    pub fn machine_digest(&self) -> u32 {
        let mut h = 0x811c_9dc5u32;
        let mut mix = |v: u32| h = (h ^ v).wrapping_mul(0x0100_0193);
        for m in &self.machines {
            for v in [
                m.enabled as u32,
                m.x,
                m.y,
                m.pc,
                m.input_shift_reg,
                m.input_shift_count,
                m.output_shift_reg,
                m.output_shift_count,
                m.cycles as u32,
                m.exec_opcode,
                m.exec_valid as u32,
                m.update_pc as u32,
                m.clock_div_int,
                m.clock_div_frac,
                m.exec_ctrl,
                m.shift_ctrl,
                m.pin_ctrl,
                m.out_pin_values,
                m.out_pin_direction,
                m.waiting as u32,
                m.wait_type as u32,
                m.wait_index,
                m.wait_polarity as u32,
                m.wait_delay as u32,
            ] {
                mix(v);
            }
        }
        h
    }

    /// FNV-1a over the FIFOs' contents and fill levels.
    pub fn fifo_digest(&self) -> u32 {
        let mut h = 0x811c_9dc5u32;
        let mut mix = |v: u32| h = (h ^ v).wrapping_mul(0x0100_0193);
        for m in &self.machines {
            for f in [&m.tx_fifo, &m.rx_fifo] {
                mix(f.item_count());
                for v in f.items() {
                    mix(v);
                }
            }
        }
        h
    }

    /// FNV-1a over the block registers and the instruction memory.
    pub fn reg_digest(&self) -> u32 {
        let mut h = 0x811c_9dc5u32;
        let mut mix = |v: u32| h = (h ^ v).wrapping_mul(0x0100_0193);
        for v in [
            self.stopped as u32,
            self.fdebug,
            self.tx_stall,
            self.rx_stall,
            self.input_sync_bypass,
            self.irq,
            self.pin_values,
            self.pin_directions,
            self.old_pin_values,
            self.old_pin_directions,
            self.irq0_int_enable,
            self.irq0_int_force,
            self.irq1_int_enable,
            self.irq1_int_force,
        ]
        .into_iter()
        .chain(self.instructions)
        {
            mix(v);
        }
        h
    }
}

impl Default for Pio {
    fn default() -> Self {
        Self::new()
    }
}

impl Rp2040 {
    // ---- request lines -------------------------------------------------------

    fn pio_dreq_tx(p: usize, m: usize) -> u32 {
        dreq::PIO0_TX0 + (p as u32) * 8 + m as u32
    }
    fn pio_dreq_rx(p: usize, m: usize) -> u32 {
        dreq::PIO0_RX0 + (p as u32) * 8 + m as u32
    }

    fn pio_update_dma_tx(&mut self, p: usize, m: usize) {
        let line = Self::pio_dreq_tx(p, m);
        if self.periph.pio[p].machines[m].tx_fifo.full() {
            self.dma_clear_dreq(line);
        } else {
            self.dma_set_dreq(line);
        }
    }

    fn pio_update_dma_rx(&mut self, p: usize, m: usize) {
        let line = Self::pio_dreq_rx(p, m);
        if self.periph.pio[p].machines[m].rx_fifo.empty() {
            self.dma_clear_dreq(line);
        } else {
            self.dma_set_dreq(line);
        }
    }

    // ---- FIFOs ---------------------------------------------------------------

    fn pio_write_fifo(&mut self, p: usize, m: usize, value: u32) {
        let pio = &mut self.periph.pio[p];
        if pio.machines[m].tx_fifo.full() {
            pio.fdebug |= FDEBUG_TXOVER << m;
            return;
        }
        pio.machines[m].tx_fifo.push(value);
        pio.tx_stall &= !(FDEBUG_TXSTALL << m);
        self.pio_update_dma_tx(p, m);
        self.pio_check_wait(p, m);
        if self.periph.pio[p].machines[m].tx_fifo.full() {
            self.pio_check_interrupts(p);
        }
    }

    fn pio_read_fifo(&mut self, p: usize, m: usize) -> u32 {
        let pio = &mut self.periph.pio[p];
        if pio.machines[m].rx_fifo.empty() {
            pio.fdebug |= FDEBUG_RXUNDER << m;
            return 0;
        }
        let result = pio.machines[m].rx_fifo.pull();
        pio.rx_stall &= !(FDEBUG_RXSTALL << m);
        self.pio_update_dma_rx(p, m);
        self.pio_check_wait(p, m);
        if self.periph.pio[p].machines[m].rx_fifo.empty() {
            self.pio_check_interrupts(p);
        }
        result
    }

    // ---- interrupts ----------------------------------------------------------

    /// `checkInterrupts`.
    pub(crate) fn pio_check_interrupts(&mut self, p: usize) {
        let first = IRQ_PIO0_IRQ0 + 2 * p as u32;
        let s0 = self.periph.pio[p].irq0_int_status() != 0;
        let s1 = self.periph.pio[p].irq1_int_status() != 0;
        self.set_interrupt(first, s0);
        self.set_interrupt(first + 1, s1);
    }

    /// `irqUpdated`.
    fn pio_irq_updated(&mut self, p: usize) {
        for m in 0..4 {
            self.pio_check_wait(p, m);
        }
        self.pio_check_interrupts(p);
    }

    // ---- the state machine ---------------------------------------------------

    fn pio_jmp_condition(&mut self, p: usize, m: usize, condition: u32) -> bool {
        match condition {
            0b000 => true,
            0b001 => self.periph.pio[p].machines[m].x == 0,
            0b010 => {
                let sm = &mut self.periph.pio[p].machines[m];
                let old = sm.x;
                sm.x = sm.x.wrapping_sub(1);
                old != 0
            }
            0b011 => self.periph.pio[p].machines[m].y == 0,
            0b100 => {
                let sm = &mut self.periph.pio[p].machines[m];
                let old = sm.y;
                sm.y = sm.y.wrapping_sub(1);
                old != 0
            }
            0b101 => {
                let sm = &self.periph.pio[p].machines[m];
                sm.x != sm.y
            }
            0b110 => {
                let pin = self.periph.pio[p].machines[m].jmp_pin() as usize;
                pin < self.sio.gpio.len() && self.pin_input_value(pin)
            }
            _ => {
                let sm = &self.periph.pio[p].machines[m];
                sm.output_shift_count < sm.pull_threshold()
            }
        }
    }

    /// `inPins`: the GPIO inputs rotated right by IN_BASE.
    fn pio_in_pins(&self, p: usize, m: usize) -> u32 {
        let values = self.gpio_values();
        let base = self.periph.pio[p].machines[m].in_base();
        if base != 0 {
            values.rotate_right(base)
        } else {
            values
        }
    }

    fn pio_in_source_value(&self, p: usize, m: usize, source: u32) -> u32 {
        let sm = &self.periph.pio[p].machines[m];
        match source {
            0b000 => self.pio_in_pins(p, m),
            0b001 => sm.x,
            0b010 => sm.y,
            0b011 => 0,
            0b100 => 0,
            0b101 => sm.status(),
            0b110 => sm.input_shift_reg,
            _ => sm.output_shift_reg,
        }
    }

    fn pio_set_out_pins(&mut self, p: usize, m: usize, value: u32) {
        let pio = &mut self.periph.pio[p];
        let (base, count) = (pio.machines[m].out_base(), pio.machines[m].out_count());
        pio.machines[m].out_pin_values = value;
        pio.pin_values_changed(value, base, count);
    }

    fn pio_set_out_pin_dirs(&mut self, p: usize, m: usize, value: u32) {
        let pio = &mut self.periph.pio[p];
        let (base, count) = (pio.machines[m].out_base(), pio.machines[m].out_count());
        pio.machines[m].out_pin_direction = value;
        pio.pin_directions_changed(value, base, count);
    }

    fn pio_write_out_value(&mut self, p: usize, m: usize, destination: u32, value: u32, bit_count: u32) {
        match destination {
            0b000 => self.pio_set_out_pins(p, m, value),
            0b001 => self.periph.pio[p].machines[m].x = value,
            0b010 => self.periph.pio[p].machines[m].y = value,
            0b011 => {}
            0b100 => self.pio_set_out_pin_dirs(p, m, value),
            0b101 => {
                let sm = &mut self.periph.pio[p].machines[m];
                sm.pc = value & 0x1f;
                sm.update_pc = false;
            }
            0b110 => {
                let sm = &mut self.periph.pio[p].machines[m];
                sm.input_shift_reg = value;
                sm.input_shift_count = bit_count;
            }
            _ => {
                let sm = &mut self.periph.pio[p].machines[m];
                sm.exec_opcode = value;
                sm.exec_valid = true;
            }
        }
    }

    fn pio_out_instruction(&mut self, p: usize, m: usize, arg: u32) {
        let bit_count = arg & 0x1f;
        let destination = arg >> 5;
        if bit_count == 0 {
            let osr = self.periph.pio[p].machines[m].output_shift_reg;
            self.pio_write_out_value(p, m, destination, osr, 32);
            self.periph.pio[p].machines[m].output_shift_count = 32;
        } else {
            let sm = &mut self.periph.pio[p].machines[m];
            let value;
            if sm.shift_ctrl & SHIFTCTRL_OUT_SHIFTDIR != 0 {
                value = sm.output_shift_reg & low_mask(bit_count);
                sm.output_shift_reg >>= bit_count;
            } else {
                value = sm.output_shift_reg >> (32 - bit_count);
                sm.output_shift_reg <<= bit_count;
            }
            self.pio_write_out_value(p, m, destination, value, bit_count);
            let sm = &mut self.periph.pio[p].machines[m];
            sm.output_shift_count += bit_count;
            if sm.output_shift_count > 32 {
                sm.output_shift_count = 32;
            }
        }
    }

    fn pio_set_set_pins(&mut self, p: usize, m: usize, value: u32) {
        let pio = &mut self.periph.pio[p];
        let (base, count) = (pio.machines[m].set_base(), pio.machines[m].set_count());
        pio.pin_values_changed(value, base, count);
    }

    fn pio_set_set_pin_dirs(&mut self, p: usize, m: usize, value: u32) {
        let pio = &mut self.periph.pio[p];
        let (base, count) = (pio.machines[m].set_base(), pio.machines[m].set_count());
        pio.pin_directions_changed(value, base, count);
    }

    fn pio_set_sideset(&mut self, p: usize, m: usize, value: u32, count: u32) {
        let pio = &mut self.periph.pio[p];
        let base = pio.machines[m].sideset_base();
        if pio.machines[m].exec_ctrl & EXECCTRL_SIDE_PINDIR != 0 {
            pio.pin_directions_changed(value, base, count);
        } else {
            pio.pin_values_changed(value, base, count);
        }
    }

    fn pio_set_mov_destination(&mut self, p: usize, m: usize, destination: u32, value: u32) {
        match destination {
            0b000 => self.pio_set_out_pins(p, m, value),
            0b001 => self.periph.pio[p].machines[m].x = value,
            0b010 => self.periph.pio[p].machines[m].y = value,
            0b011 => {}
            0b100 => {
                let sm = &mut self.periph.pio[p].machines[m];
                sm.exec_opcode = value;
                sm.exec_valid = true;
            }
            0b101 => {
                let sm = &mut self.periph.pio[p].machines[m];
                sm.pc = value & 0x1f;
                sm.update_pc = false;
            }
            0b110 => {
                let sm = &mut self.periph.pio[p].machines[m];
                sm.input_shift_reg = value;
                sm.input_shift_count = 0;
            }
            _ => {
                let sm = &mut self.periph.pio[p].machines[m];
                sm.output_shift_reg = value;
                sm.output_shift_count = 0;
            }
        }
    }

    /// `executeInstruction`.
    pub(crate) fn pio_execute_instruction(&mut self, p: usize, m: usize, opcode: u32) {
        let arg = opcode & 0xff;
        match opcode >> 13 {
            // JMP
            0b000 => {
                if self.pio_jmp_condition(p, m, arg >> 5) {
                    let sm = &mut self.periph.pio[p].machines[m];
                    sm.pc = arg & 0x1f;
                    sm.update_pc = false;
                }
            }
            // WAIT
            0b001 => {
                let polarity = arg & 0x80 != 0;
                let source = (arg >> 5) & 0x3;
                let index = arg & 0x1f;
                let sm = &mut self.periph.pio[p].machines[m];
                match source {
                    0b00 => sm.wait(WaitType::Pin, polarity, index),
                    0b01 => {
                        let i = (index + sm.in_base()) % 32;
                        sm.wait(WaitType::Pin, polarity, i)
                    }
                    0b10 => sm.wait(WaitType::Irq, polarity, irq_index(index, m)),
                    _ => {}
                }
            }
            // IN
            0b010 => {
                let bit_count = arg & 0x1f;
                let mut source_value = self.pio_in_source_value(p, m, arg >> 5);
                let sm = &mut self.periph.pio[p].machines[m];
                if bit_count == 0 {
                    sm.input_shift_reg = source_value;
                    sm.input_shift_count = 32;
                } else {
                    source_value &= low_mask(bit_count);
                    if sm.shift_ctrl & SHIFTCTRL_IN_SHIFTDIR != 0 {
                        sm.input_shift_reg >>= bit_count;
                        sm.input_shift_reg |= source_value << (32 - bit_count);
                    } else {
                        sm.input_shift_reg <<= bit_count;
                        sm.input_shift_reg |= source_value;
                    }
                    sm.input_shift_count += bit_count;
                    if sm.input_shift_count > 32 {
                        sm.input_shift_count = 32;
                    }
                }
                if sm.shift_ctrl & SHIFTCTRL_AUTOPUSH != 0 && sm.input_shift_count >= sm.push_threshold() {
                    if !sm.rx_fifo.full() {
                        let v = sm.input_shift_reg;
                        sm.rx_fifo.push(v);
                        self.pio_update_dma_rx(p, m);
                        self.pio_check_interrupts(p);
                    } else {
                        let pio = &mut self.periph.pio[p];
                        pio.rx_stall |= FDEBUG_RXSTALL << m;
                        pio.fdebug |= pio.rx_stall;
                        let isr = pio.machines[m].input_shift_reg;
                        pio.machines[m].wait(WaitType::RxFifo, false, isr);
                    }
                    let sm = &mut self.periph.pio[p].machines[m];
                    sm.input_shift_count = 0;
                    sm.input_shift_reg = 0;
                }
            }
            // OUT
            0b011 => {
                let sm = &self.periph.pio[p].machines[m];
                if sm.shift_ctrl & SHIFTCTRL_AUTOPULL != 0 && sm.output_shift_count >= sm.pull_threshold() {
                    self.periph.pio[p].machines[m].output_shift_count = 0;
                    if !self.periph.pio[p].machines[m].tx_fifo.empty() {
                        let sm = &mut self.periph.pio[p].machines[m];
                        sm.output_shift_reg = sm.tx_fifo.pull();
                        self.pio_update_dma_tx(p, m);
                        self.pio_check_interrupts(p);
                    } else {
                        let pio = &mut self.periph.pio[p];
                        pio.tx_stall |= FDEBUG_TXSTALL << m;
                        pio.fdebug |= pio.tx_stall;
                        pio.machines[m].wait(WaitType::Out, false, arg);
                    }
                }
                if !self.periph.pio[p].machines[m].waiting {
                    self.pio_out_instruction(p, m, arg);
                }
            }
            // PUSH / PULL
            0b100 => {
                let block = arg & (1 << 5) != 0;
                let if_full_or_empty = arg & (1 << 6) != 0;
                if arg & 0x1f != 0 {
                    // Unknown instruction
                } else if arg & 0x80 != 0 {
                    // PULL
                    let sm = &self.periph.pio[p].machines[m];
                    if if_full_or_empty
                        && sm.shift_ctrl & SHIFTCTRL_AUTOPULL != 0
                        && sm.output_shift_count < sm.pull_threshold()
                    {
                        // The reference `break`s out of the whole instruction
                        // here, which still runs the trailing cycle count and
                        // side-set below.
                    } else {
                        if !self.periph.pio[p].machines[m].tx_fifo.empty() {
                            let sm = &mut self.periph.pio[p].machines[m];
                            sm.output_shift_reg = sm.tx_fifo.pull();
                            self.pio_update_dma_tx(p, m);
                            self.pio_check_interrupts(p);
                        } else {
                            let pio = &mut self.periph.pio[p];
                            pio.tx_stall |= FDEBUG_TXSTALL << m;
                            pio.fdebug |= pio.tx_stall;
                            if block {
                                pio.machines[m].wait(WaitType::TxFifo, false, 0);
                            } else {
                                let x = pio.machines[m].x;
                                pio.machines[m].output_shift_reg = x;
                            }
                        }
                        self.periph.pio[p].machines[m].output_shift_count = 0;
                    }
                } else {
                    // PUSH
                    let sm = &self.periph.pio[p].machines[m];
                    if if_full_or_empty
                        && sm.shift_ctrl & SHIFTCTRL_AUTOPUSH != 0
                        && sm.input_shift_count < sm.push_threshold()
                    {
                        // break, as above
                    } else {
                        if !sm.rx_fifo.full() {
                            let sm = &mut self.periph.pio[p].machines[m];
                            let v = sm.input_shift_reg;
                            sm.rx_fifo.push(v);
                            self.pio_update_dma_rx(p, m);
                            self.pio_check_interrupts(p);
                        } else {
                            let pio = &mut self.periph.pio[p];
                            pio.rx_stall |= FDEBUG_RXSTALL << m;
                            pio.fdebug |= pio.rx_stall;
                            if block {
                                let isr = pio.machines[m].input_shift_reg;
                                pio.machines[m].wait(WaitType::RxFifo, false, isr);
                            }
                        }
                        let sm = &mut self.periph.pio[p].machines[m];
                        sm.input_shift_reg = 0;
                        sm.input_shift_count = 0;
                    }
                }
            }
            // MOV
            0b101 => {
                let source = arg & 0x7;
                let op = (arg >> 3) & 0x3;
                let destination = (arg >> 5) & 0x7;
                let value = self.pio_in_source_value(p, m, source);
                let transformed = match op {
                    0b01 => !value,
                    0b10 => bit_reverse(value),
                    _ => value,
                };
                self.pio_set_mov_destination(p, m, destination, transformed);
            }
            // IRQ
            0b110 => {
                if arg & 0x80 == 0 {
                    let clear = arg & 0x40 != 0;
                    let wait = arg & 0x20 != 0;
                    let irq = irq_index(arg & 0x1f, m);
                    if clear {
                        self.periph.pio[p].irq &= !(1 << irq);
                        self.pio_irq_updated(p);
                    } else {
                        self.periph.pio[p].irq |= 1 << irq;
                        self.pio_irq_updated(p);
                        if wait {
                            self.periph.pio[p].machines[m].wait(WaitType::Irq, false, irq);
                        }
                    }
                }
            }
            // SET
            0b111 => {
                let data = arg & 0x1f;
                match arg >> 5 {
                    0b000 => self.pio_set_set_pins(p, m, data),
                    0b001 => self.periph.pio[p].machines[m].x = data,
                    0b010 => self.periph.pio[p].machines[m].y = data,
                    0b100 => self.pio_set_set_pin_dirs(p, m, data),
                    _ => {}
                }
            }
            // An opcode wider than 16 bits (OUT EXEC and MOV EXEC take 32)
            // matches no arm.
            _ => {}
        }

        let sm = &mut self.periph.pio[p].machines[m];
        sm.cycles += 1;
        let sideset_count = sm.sideset_count();
        let side_en = sm.exec_ctrl & EXECCTRL_SIDE_EN != 0;
        let delay_sideset = (opcode >> 8) & 0x1f;
        // `1 << (5 - count)` takes its count modulo 32 in JavaScript.
        let delay = delay_sideset & low_mask(5u32.wrapping_sub(sideset_count));
        if sideset_count != 0 && (!side_en || delay_sideset & 0x10 != 0) {
            let sideset = delay_sideset >> (5u32.wrapping_sub(sideset_count) & 31);
            let count = if side_en { sideset_count - 1 } else { sideset_count };
            self.pio_set_sideset(p, m, sideset, count);
        }

        let sm = &mut self.periph.pio[p].machines[m];
        if sm.exec_valid {
            sm.exec_valid = false;
            let op = sm.exec_opcode;
            self.pio_execute_instruction(p, m, op);
        } else if sm.waiting {
            if sm.wait_delay < 0 {
                sm.wait_delay = delay as i64;
            }
            self.pio_check_wait(p, m);
        } else {
            sm.cycles += delay as i64;
        }
    }

    /// `checkWait`.
    pub(crate) fn pio_check_wait(&mut self, p: usize, m: usize) {
        if !self.periph.pio[p].machines[m].waiting {
            return;
        }
        let (wait_type, wait_index, polarity) = {
            let sm = &self.periph.pio[p].machines[m];
            (sm.wait_type, sm.wait_index, sm.wait_polarity)
        };
        match wait_type {
            WaitType::Irq => {
                let pio = &mut self.periph.pio[p];
                let irq_value = pio.irq & (1u32 << (wait_index & 31)) != 0;
                if irq_value == polarity {
                    pio.machines[m].waiting = false;
                    if irq_value {
                        pio.irq &= !(1u32 << (wait_index & 31));
                    }
                }
            }
            WaitType::Pin => {
                if (wait_index as usize) < self.sio.gpio.len() && self.pin_input_value(wait_index as usize) == polarity
                {
                    self.periph.pio[p].machines[m].waiting = false;
                }
            }
            WaitType::RxFifo => {
                if !self.periph.pio[p].machines[m].rx_fifo.full() {
                    let sm = &mut self.periph.pio[p].machines[m];
                    sm.rx_fifo.push(wait_index);
                    sm.waiting = false;
                    self.pio_update_dma_rx(p, m);
                    self.pio_check_interrupts(p);
                }
            }
            WaitType::TxFifo => {
                if !self.periph.pio[p].machines[m].tx_fifo.empty() {
                    let sm = &mut self.periph.pio[p].machines[m];
                    sm.output_shift_reg = sm.tx_fifo.pull();
                    sm.waiting = false;
                    self.pio_update_dma_tx(p, m);
                    self.pio_check_interrupts(p);
                }
            }
            WaitType::Out => {
                if !self.periph.pio[p].machines[m].tx_fifo.empty() {
                    let sm = &mut self.periph.pio[p].machines[m];
                    sm.output_shift_reg = sm.tx_fifo.pull();
                    self.pio_out_instruction(p, m, wait_index);
                    self.periph.pio[p].machines[m].waiting = false;
                    self.pio_update_dma_tx(p, m);
                    self.pio_check_interrupts(p);
                }
            }
            WaitType::None => {}
        }
        let sm = &mut self.periph.pio[p].machines[m];
        if !sm.waiting {
            sm.next_pc();
            sm.cycles += sm.wait_delay;
            sm.exec_ctrl &= !EXECCTRL_EXEC_STALLED;
        }
    }

    /// `StateMachine.step`.
    fn pio_machine_step(&mut self, p: usize, m: usize) {
        if self.periph.pio[p].machines[m].waiting {
            self.pio_check_wait(p, m);
            if self.periph.pio[p].machines[m].waiting {
                return;
            }
        }
        let sm = &mut self.periph.pio[p].machines[m];
        sm.update_pc = true;
        let opcode = self.periph.pio[p].instructions[sm.pc as usize];
        self.pio_execute_instruction(p, m, opcode);
        let sm = &mut self.periph.pio[p].machines[m];
        if sm.update_pc {
            sm.next_pc();
        }
    }

    /// `checkChangedPins`.
    fn pio_check_changed_pins(&mut self, p: usize) {
        let pio = &mut self.periph.pio[p];
        let changed = (pio.old_pin_directions ^ pio.pin_directions) | (pio.old_pin_values ^ pio.pin_values);
        if changed != 0 {
            pio.old_pin_directions = pio.pin_directions;
            pio.old_pin_values = pio.pin_values;
            for i in 0..self.sio.gpio.len() {
                if changed & (1 << i) != 0 {
                    self.gpio_check_for_updates(i);
                }
            }
        }
    }

    /// `RPPIO.step`: every machine of the block steps, enabled or not.
    fn pio_step(&mut self, p: usize) {
        for m in 0..4 {
            self.pio_machine_step(p, m);
        }
        self.pio_check_changed_pins(p);
    }

    /// `RPPIO.run`: up to 1000 steps, then a timer for the next batch.
    fn pio_run(&mut self, p: usize) {
        for _ in 0..RUN_BATCH {
            if self.periph.pio[p].stopped {
                break;
            }
            self.pio_step(p);
        }
        if !self.periph.pio[p].stopped {
            self.periph.pio_timers.push(p as u8);
        }
    }

    /// One turn of the host's event loop: every `setTimeout` pending at its
    /// start runs, in creation order; the ones those create wait for the
    /// next turn.
    pub fn pio_host_turn(&mut self) {
        if self.periph.pio_timers.is_empty() {
            return;
        }
        let batch = std::mem::take(&mut self.periph.pio_timers);
        for p in batch {
            self.pio_run(p as usize);
        }
    }

    // ---- registers -----------------------------------------------------------

    fn pio_sm_read(&mut self, p: usize, m: usize, offset: u32) -> u32 {
        let sm = &self.periph.pio[p].machines[m];
        match offset {
            SM_CLKDIV_R => (sm.clock_div_int << 16) | (sm.clock_div_frac << 8),
            SM_EXECCTRL_R => sm.exec_ctrl,
            SM_SHIFTCTRL_R => sm.shift_ctrl,
            SM_ADDR_R => sm.pc,
            SM_INSTR_R => self.periph.pio[p].instructions[sm.pc as usize],
            SM_PINCTRL_R => sm.pin_ctrl,
            // The reference's `pio.error`, which its logger throws on; the
            // bus hands only word-aligned offsets, so no access reaches it.
            _ => 0,
        }
    }

    fn pio_sm_write(&mut self, p: usize, m: usize, offset: u32, value: u32) {
        match offset {
            SM_CLKDIV_R => {
                let sm = &mut self.periph.pio[p].machines[m];
                sm.clock_div_frac = (value >> 8) & 0xff;
                sm.clock_div_int = value >> 16;
            }
            SM_EXECCTRL_R => {
                let sm = &mut self.periph.pio[p].machines[m];
                sm.exec_ctrl = (value & 0x7fff_ffff) | (sm.exec_ctrl & 0x8000_0000);
            }
            SM_SHIFTCTRL_R => self.periph.pio[p].machines[m].shift_ctrl = value,
            SM_ADDR_R => {} // read-only
            SM_INSTR_R => {
                self.pio_execute_instruction(p, m, value & 0xffff);
                let sm = &mut self.periph.pio[p].machines[m];
                if sm.waiting {
                    sm.exec_ctrl |= EXECCTRL_EXEC_STALLED;
                }
            }
            SM_PINCTRL_R => self.periph.pio[p].machines[m].pin_ctrl = value,
            _ => {}
        }
    }

    pub(crate) fn pio_read(&mut self, p: usize, offset: u32) -> u32 {
        for m in 0..4 {
            if offset >= SM_CLKDIV[m] && offset <= SM_PINCTRL[m] {
                return self.pio_sm_read(p, m, offset - SM_CLKDIV[m]);
            }
        }
        let pio = &self.periph.pio[p];
        match offset {
            CTRL => (0..4).fold(0, |acc, i| acc | ((pio.machines[i].enabled as u32) << i)),
            FSTAT => (0..4).fold(0, |acc, i| acc | pio.machines[i].fifo_stat(i)),
            FDEBUG => pio.fdebug,
            FLEVEL => {
                let mut r = 0;
                for i in 0..4 {
                    r |= (pio.machines[i].tx_fifo.item_count() & 0xf) << (8 * i);
                    r |= (pio.machines[i].rx_fifo.item_count() & 0xf) << (8 * i + 4);
                }
                r
            }
            RXF0 => self.pio_read_fifo(p, 0),
            RXF1 => self.pio_read_fifo(p, 1),
            RXF2 => self.pio_read_fifo(p, 2),
            RXF3 => self.pio_read_fifo(p, 3),
            IRQ => pio.irq,
            IRQ_FORCE => 0,
            INPUT_SYNC_BYPASS => pio.input_sync_bypass,
            DBG_PADOUT => pio.pin_values,
            DBG_PADOE => pio.pin_directions,
            DBG_CFGINFO => 0x0020_0404,
            INTR => pio.int_raw(),
            IRQ0_INTE => pio.irq0_int_enable,
            IRQ0_INTF => pio.irq0_int_force,
            IRQ0_INTS => pio.irq0_int_status(),
            IRQ1_INTE => pio.irq1_int_enable,
            IRQ1_INTF => pio.irq1_int_force,
            IRQ1_INTS => pio.irq1_int_status(),
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
                0xffff_ffff
            }
        }
    }

    pub(crate) fn pio_write(&mut self, p: usize, offset: u32, value: u32, raw: u32) {
        if (INSTR_MEM0..=INSTR_MEM31).contains(&offset) {
            let index = ((offset - INSTR_MEM0) >> 2) as usize;
            self.periph.pio[p].instructions[index] = value & 0xffff;
            return;
        }
        for m in 0..4 {
            if offset >= SM_CLKDIV[m] && offset <= SM_PINCTRL[m] {
                self.pio_sm_write(p, m, offset - SM_CLKDIV[m], value);
                return;
            }
        }
        match offset {
            CTRL => {
                for index in 0..4usize {
                    self.periph.pio[p].machines[index].enabled = value & (1 << index) != 0;
                    if value & (1 << (4 + index)) != 0 {
                        self.periph.pio[p].machines[index].restart();
                    }
                    if value & (1 << (8 + index)) != 0 {
                        // `clkDivRestart` only warns: dividers do nothing.
                        self.warnings += 1;
                    }
                }
                let should_run = value & 0xf != 0;
                if self.periph.pio[p].stopped && should_run {
                    self.periph.pio[p].stopped = false;
                    self.pio_run(p);
                }
                if !should_run {
                    self.periph.pio[p].stopped = true;
                }
            }
            FDEBUG => {
                let pio = &mut self.periph.pio[p];
                pio.fdebug &= !raw;
                pio.fdebug |= pio.tx_stall | pio.rx_stall;
            }
            TXF0 => self.pio_write_fifo(p, 0, value),
            TXF1 => self.pio_write_fifo(p, 1, value),
            TXF2 => self.pio_write_fifo(p, 2, value),
            TXF3 => self.pio_write_fifo(p, 3, value),
            IRQ => {
                self.periph.pio[p].irq &= !raw;
                self.pio_irq_updated(p);
            }
            INPUT_SYNC_BYPASS => self.periph.pio[p].input_sync_bypass = value,
            IRQ_FORCE => {
                self.periph.pio[p].irq |= value;
                self.pio_irq_updated(p);
            }
            IRQ0_INTE => {
                self.periph.pio[p].irq0_int_enable = value & 0xfff;
                self.pio_check_interrupts(p);
            }
            IRQ0_INTF => {
                self.periph.pio[p].irq0_int_force = value & 0xfff;
                self.pio_check_interrupts(p);
            }
            IRQ1_INTE => {
                self.periph.pio[p].irq1_int_enable = value & 0xfff;
                self.pio_check_interrupts(p);
            }
            IRQ1_INTF => {
                self.periph.pio[p].irq1_int_force = value & 0xfff;
                self.pio_check_interrupts(p);
            }
            _ => {
                self.periph.unimplemented_accesses += 1;
                self.warnings += 1;
            }
        }
    }

    /// `GPIOPin.setInputValue`'s PIO hook: a machine waiting on this pin
    /// re-checks its wait. The condition is read afresh for each machine, as
    /// an earlier machine's check can change what a later one sees.
    pub(crate) fn pio_pin_input_changed(&mut self, pin: usize) {
        for p in 0..2 {
            for m in 0..4 {
                let sm = &self.periph.pio[p].machines[m];
                if sm.enabled && sm.waiting && sm.wait_type == WaitType::Pin && sm.wait_index as usize == pin {
                    self.pio_check_wait(p, m);
                }
            }
        }
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

impl crate::snapshot::HeapBytes for WaitType {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            WaitType::None => 0,
            WaitType::Pin => 0,
            WaitType::RxFifo => 0,
            WaitType::TxFifo => 0,
            WaitType::Irq => 0,
            WaitType::Out => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for Fifo {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.buf, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.start, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.used, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for StateMachine {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.enabled, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.x, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.y, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.input_shift_reg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.input_shift_count, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.output_shift_reg, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.output_shift_count, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.cycles, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.exec_opcode, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.exec_valid, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.update_pc, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.clock_div_int, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.clock_div_frac, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.exec_ctrl, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.shift_ctrl, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.pin_ctrl, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.rx_fifo, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.tx_fifo, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.out_pin_values, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.out_pin_direction, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.waiting, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.wait_type, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.wait_index, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.wait_polarity, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.wait_delay, w); }
        n
    }
}
