//! Unit tests for R4: the USB controller (device mode), the CDC host and the
//! REPL feeding of brownout's wrapper. Expected values come from the RP2040
//! datasheet (section 4.1, USB) and the USB 2.0 and CDC (PSTN) specifications;
//! the firmware side of each exchange is played by the test, through the
//! DPRAM and registers, as the pico-sdk's device stack would. Where
//! rp2040js 1.3.3 departs from the datasheet a `reference_quirk_*` test names
//! both.
//!
//! Upstream suites ported by hand, tag v1.3.3 (MIT; see NOTICE):
//! `src/usb/cdc.spec.ts` (`extractEndpointNumbers`, five tests) and
//! `src/utils/fifo.spec.ts` (three tests). rp2040js has no tests of
//! `usb.ts` itself.

use super::usb::*;
use super::*;

const USB: u32 = 0x5011_0000;
const DPRAM: u32 = 0x5010_0000;

const MAIN_CTRL: u32 = 0x40;
const SOF_RD: u32 = 0x48;
const SIE_CTRL: u32 = 0x4c;
const SIE_STATUS: u32 = 0x50;
const BUFF_STATUS: u32 = 0x58;
const EP_ABORT: u32 = 0x60;
const EP_ABORT_DONE: u32 = 0x64;
const USB_MUXING: u32 = 0x74;
const USB_PWR: u32 = 0x78;
const INTR: u32 = 0x8c;
const INTE: u32 = 0x90;
const INTF: u32 = 0x94;
const INTS: u32 = 0x98;

fn rd(m: &mut Rp2040, off: u32) -> u32 {
    m.read_u32(USB + off)
}
fn wr(m: &mut Rp2040, off: u32, v: u32) {
    m.write_u32(USB + off, v)
}
fn us(m: &mut Rp2040, micros: f64) {
    m.clock_tick(micros * 1000.0);
}
fn dpram_write(m: &mut Rp2040, off: u32, bytes: &[u8]) {
    m.dpram[off as usize..off as usize + bytes.len()].copy_from_slice(bytes);
}

// ---- src/usb/cdc.spec.ts -----------------------------------------------------------

fn flat(parts: &[&[u8]]) -> Vec<u8> {
    parts.iter().flat_map(|p| p.iter().copied()).collect()
}

#[test]
fn extract_endpoint_numbers_should_not_die_if_the_descriptors_are_invalid() {
    assert_eq!(extract_endpoint_numbers(&[0]), (-1, -1));
    assert_eq!(extract_endpoint_numbers(&[9]), (-1, -1));
}

#[test]
fn extract_endpoint_numbers_from_pi_pico_sdk_descriptors() {
    let d = flat(&[
        &[9, 2, 84, 0, 3, 1, 0, 128, 125],
        &[8, 11, 0, 2, 2, 2, 0, 0],
        &[9, 4, 0, 0, 1, 2, 2, 0, 4],
        &[5, 36, 0, 32, 1],
        &[5, 36, 1, 0, 1],
        &[4, 36, 2, 2],
        &[5, 36, 6, 0, 1],
        &[7, 5, 129, 3, 8, 0, 16],
        &[9, 4, 1, 0, 2, 10, 0, 0, 0],
        &[7, 5, 2, 2, 64, 0, 0],
        &[7, 5, 130, 2, 64, 0, 0],
        &[9, 4, 2, 0, 0, 255, 0, 1, 5],
    ]);
    assert_eq!(extract_endpoint_numbers(&d), (2, 2));
}

#[test]
fn extract_endpoint_numbers_from_micropython_descriptors() {
    let d = flat(&[
        &[9, 2, 75, 0, 2, 1, 0, 128, 125], // Configuration descriptor
        &[8, 11, 0, 2, 2, 2, 0, 0],
        &[9, 4, 0, 0, 1, 2, 2, 0, 4], // Interface descriptor
        &[5, 36, 0, 32, 1],
        &[5, 36, 1, 0, 1],
        &[4, 36, 2, 2],
        &[5, 36, 6, 0, 1],
        &[7, 5, 129, 3, 8, 0, 16], // Endpoint (interrupt)
        &[9, 4, 1, 0, 2, 10, 0, 0, 0], // interface (CDC-Data class)
        &[7, 5, 2, 2, 64, 0, 0], // Endpoint (bulk)
        &[7, 5, 130, 2, 64, 0, 0], // Endpoint (bulk)
    ]);
    assert_eq!(extract_endpoint_numbers(&d), (2, 2));
}

#[test]
fn extract_endpoint_numbers_from_circuitpython_descriptors() {
    let d = flat(&[
        &[9, 2, 218, 0, 6, 1, 0, 128, 50],
        &[8, 11, 0, 2, 2, 2, 0, 0],
        &[9, 4, 0, 0, 1, 2, 2, 0, 4],
        &[5, 36, 0, 16, 1],
        &[5, 36, 1, 1, 1],
        &[4, 36, 2, 2],
        &[5, 36, 6, 0, 1],
        &[7, 5, 129, 3, 64, 0, 16],
        &[9, 4, 1, 0, 2, 10, 0, 0, 5],
        &[7, 5, 2, 2, 64, 0, 0],
        &[7, 5, 130, 2, 64, 0, 0],
        &[9, 4, 2, 0, 2, 8, 6, 80, 6],
        &[7, 5, 131, 2, 64, 0, 0],
        &[7, 5, 3, 2, 64, 0, 0],
        &[9, 4, 3, 0, 2, 3, 0, 0, 7],
        &[9, 33, 17, 1, 0, 1, 34, 195, 0],
        &[7, 5, 132, 3, 64, 0, 8],
        &[7, 5, 4, 3, 64, 0, 8],
        &[9, 4, 4, 0, 0, 1, 1, 0, 11],
        &[9, 36, 1, 0, 1, 9, 0, 1, 5],
        &[9, 4, 5, 0, 2, 1, 3, 0, 10],
        &[7, 36, 1, 0, 1, 37, 0],
        &[6, 36, 2, 1, 1, 8],
        &[6, 36, 2, 2, 2, 0],
        &[9, 36, 3, 1, 3, 1, 2, 1, 9],
        &[9, 36, 3, 2, 4, 1, 1, 1, 0],
        &[7, 5, 5, 2, 64, 0, 0],
        &[5, 37, 1, 1, 1],
        &[7, 5, 133, 2, 64, 0, 0],
        &[5, 37, 1, 1, 3],
    ]);
    assert_eq!(extract_endpoint_numbers(&d), (2, 2));
}

#[test]
fn extract_endpoint_numbers_from_arduino_core_descriptors() {
    let d = flat(&[
        &[9, 2, 75, 0, 2, 1, 0, 192, 250],
        &[8, 11, 0, 2, 2, 2, 0, 0],
        &[9, 4, 0, 0, 1, 2, 2, 1, 0], // Interface
        &[5, 36, 0, 16, 1],
        &[5, 36, 1, 3, 1],
        &[4, 36, 2, 6],
        &[5, 36, 6, 0, 1],
        &[7, 5, 130, 3, 64, 0, 16], // Endpoint
        &[9, 4, 1, 0, 2, 10, 0, 0, 0], // Interface
        &[7, 5, 129, 2, 64, 0, 0], // Endpoint
        &[7, 5, 1, 2, 64, 0, 0], // Endpoint
    ]);
    assert_eq!(extract_endpoint_numbers(&d), (1, 1));
}

// ---- src/utils/fifo.spec.ts ------------------------------------------------------------

#[test]
fn fifo_should_successfully_push_and_pull_4_items() {
    let mut fifo = Fifo::new(3);
    assert!(fifo.is_empty());
    fifo.push(1);
    assert!(!fifo.is_empty());
    fifo.push(2);
    assert_eq!(fifo.item_count(), 2);
    assert!(!fifo.is_full());
    fifo.push(3);
    assert!(fifo.is_full());
    assert_eq!(fifo.pull(), 1);
    assert!(!fifo.is_full());
    fifo.push(4);
    assert!(fifo.is_full());
    assert_eq!(fifo.pull(), 2);
    assert_eq!(fifo.pull(), 3);
    assert!(!fifo.is_empty());
    assert_eq!(fifo.item_count(), 1);
    assert_eq!(fifo.pull(), 4);
    assert!(!fifo.is_full());
    assert!(fifo.is_empty());
}

#[test]
fn fifo_peek_returns_the_next_item_without_affecting_the_content() {
    let mut fifo = Fifo::new(3);
    fifo.push(10);
    fifo.push(20);
    assert_eq!(fifo.peek(), 10);
    assert_eq!(fifo.item_count(), 2);
    assert_eq!(fifo.pull(), 10);
}

#[test]
fn fifo_items_returns_all_the_content() {
    let mut fifo = Fifo::new(3);
    fifo.push(10);
    fifo.push(20);
    fifo.push(30);
    fifo.pull();
    assert_eq!(fifo.items(), vec![20, 30]);
}

#[test]
fn reference_quirk_fifo_drops_a_push_when_full_and_pulls_zero_when_empty() {
    let mut fifo = Fifo::new(2);
    fifo.push(1);
    fifo.push(2);
    fifo.push(3);
    assert_eq!(fifo.items(), vec![1, 2]);
    fifo.pull();
    fifo.pull();
    assert_eq!(fifo.pull(), 0);
}

// ---- USB 2.0 and CDC setup packets ---------------------------------------------------------

#[test]
fn setup_packets_follow_usb_2_0_section_9_3() {
    // bmRequestType: direction bit 7, type bits 6:5, recipient bits 4:0.
    assert_eq!(set_device_address_packet(1), [0x00, 0x05, 1, 0, 0, 0, 0, 0]);
    assert_eq!(get_descriptor_packet(2, 9, 0), [0x80, 0x06, 0x00, 0x02, 0, 0, 9, 0]);
    assert_eq!(get_descriptor_packet(2, 0x0114, 0), [0x80, 0x06, 0x00, 0x02, 0, 0, 0x14, 0x01]);
    assert_eq!(set_device_configuration_packet(1), [0x00, 0x09, 1, 0, 0, 0, 0, 0]);
    // CDC PSTN 1.2: SET_CONTROL_LINE_STATE (0x22), class request to the
    // interface, wValue bit 0 DTR and bit 1 RTS. The reference sends it to
    // the device recipient (bmRequestType 0x20) rather than the interface.
    assert_eq!(
        create_setup_packet(0, 1, 0, 0x22, 3, 0, 0),
        [0x20, 0x22, 3, 0, 0, 0, 0, 0]
    );
}

// ---- registers ----------------------------------------------------------------------------

#[test]
fn usb_registers_reset_to_zero() {
    let mut m = Rp2040::new();
    for off in [0x40u32, 0x4c, 0x50, 0x58, 0x8c, 0x90, 0x94, 0x98] {
        assert_eq!(rd(&mut m, off), 0, "register {off:#x}");
    }
}

#[test]
fn usb_interrupt_registers_are_20_bits_and_ints_combines_them() {
    // Datasheet 4.1.5: INTR bits 19:0; INTS = (INTR & INTE) | INTF.
    let mut m = Rp2040::new();
    wr(&mut m, INTE, 0xffff_ffff);
    wr(&mut m, INTF, 0xffff_ffff);
    assert_eq!(rd(&mut m, INTE), 0xf_ffff);
    assert_eq!(rd(&mut m, INTF), 0xf_ffff);
    wr(&mut m, INTF, 1 << 2);
    wr(&mut m, INTE, 0);
    assert_eq!(rd(&mut m, INTS), 1 << 2);
    assert_eq!(m.cpu.pending_interrupts & (1 << 5), 1 << 5, "USBCTRL is IRQ 5");
    wr(&mut m, INTF, 0);
    assert_eq!(m.cpu.pending_interrupts & (1 << 5), 0);
}

#[test]
fn usb_main_ctrl_keeps_sim_timing_host_ndevice_and_controller_en() {
    // Datasheet: MAIN_CTRL SIM_TIMING bit 31, HOST_NDEVICE bit 1, CONTROLLER_EN bit 0.
    let mut m = Rp2040::new();
    wr(&mut m, MAIN_CTRL, 0xffff_ffff);
    assert_eq!(rd(&mut m, MAIN_CTRL), (1 << 31) | 0b11);
    wr(&mut m, MAIN_CTRL, 0);
    assert_eq!(rd(&mut m, MAIN_CTRL), 0);
}

#[test]
fn usb_vbus_detect_override_drives_sie_status_bit_0() {
    // Datasheet: USB_PWR.VBUS_DETECT_OVERRIDE_EN bit 2, VBUS_DETECT bit 3;
    // SIE_STATUS.VBUS_DETECTED bit 0 and its INTR bit 11.
    let mut m = Rp2040::new();
    wr(&mut m, USB_PWR, (1 << 2) | (1 << 3));
    assert_eq!(rd(&mut m, SIE_STATUS) & 1, 1);
    wr(&mut m, USB_PWR, 1 << 2);
    assert_eq!(rd(&mut m, SIE_STATUS) & 1, 0);
    // The status change reaches INTR on the next SIE update (a write to it).
    wr(&mut m, USB_PWR, (1 << 2) | (1 << 3));
    wr(&mut m, SIE_STATUS, 0);
    assert_eq!(rd(&mut m, INTR) & (1 << 11), 1 << 11);
}

#[test]
fn usb_sie_status_write_one_to_clear_bits() {
    // Datasheet: BUS_RESET 19, SETUP_REC 17, CONNECTED 16, DATA_SEQ_ERROR 31,
    // ACK_REC 30, STALL_REC 29, NAK_REC 28, RX_TIMEOUT 27, RX_OVERFLOW 26,
    // BIT_STUFF_ERROR 25, CRC_ERROR 24, TRANS_COMPLETE 18, RESUME 11 are WC;
    // VBUS_DETECTED 0 and SUSPENDED 4 are read-only.
    let mut m = Rp2040::new();
    wr(&mut m, USB_PWR, (1 << 2) | (1 << 3));
    m.usb_send_setup_packet(&[0; 8]);
    assert_eq!(rd(&mut m, SIE_STATUS), (1 << 17) | 1);
    // (BUS_RESET is left out: clearing it makes the host send SET_ADDRESS.)
    wr(&mut m, SIE_STATUS, 0xffff_ffff & !(1 << 19));
    assert_eq!(rd(&mut m, SIE_STATUS), 1, "only VBUS_DETECTED stays");
}

#[test]
fn usb_buff_status_is_write_one_to_clear_and_drives_the_buffer_interrupt() {
    let mut m = Rp2040::new();
    m.write_u32(DPRAM + 0x80, (1 << 10) | (1 << 15)); // EP0 IN available, full, 0 bytes
    assert_eq!(rd(&mut m, BUFF_STATUS), 1);
    assert_eq!(rd(&mut m, INTR) & (1 << 4), 1 << 4);
    wr(&mut m, BUFF_STATUS, 1);
    assert_eq!(rd(&mut m, BUFF_STATUS), 0);
    assert_eq!(rd(&mut m, INTR) & (1 << 4), 0);
}

#[test]
fn reference_quirk_usb_ep_abort_completes_at_once_and_phy_registers_are_unmodelled() {
    // Datasheet: EP_ABORT_DONE sets when the controller has finished the
    // abort. rp2040js marks it done in the same write. USB_MUXING reads and
    // the USBPHY registers (0x7c..0x84) are not modelled at all.
    let mut m = Rp2040::new();
    wr(&mut m, EP_ABORT, 0b101);
    assert_eq!(rd(&mut m, EP_ABORT_DONE), 0b101);
    wr(&mut m, EP_ABORT_DONE, 0b001);
    assert_eq!(rd(&mut m, EP_ABORT_DONE), 0b100);
    assert_eq!(rd(&mut m, USB_MUXING), 0xffff_ffff);
    assert_eq!(rd(&mut m, 0x7c), 0xffff_ffff);
    assert_eq!(rd(&mut m, 0x84), 0xffff_ffff);
}

#[test]
fn usb_muxing_to_digital_pad_without_phy_reports_connected() {
    // pico-sdk's hw_enumeration_fix busy-waits on SIE_STATUS.CONNECTED after
    // muxing the pins to the digital pad; the reference sets it for that.
    let mut m = Rp2040::new();
    wr(&mut m, USB_MUXING, (1 << 3) | (1 << 2)); // SOFTCON | TO_DIGITAL_PAD
    assert_eq!(rd(&mut m, SIE_STATUS) & (1 << 16), 1 << 16);
    let mut m = Rp2040::new();
    wr(&mut m, USB_MUXING, (1 << 2) | 1); // TO_PHY as well: no
    assert_eq!(rd(&mut m, SIE_STATUS) & (1 << 16), 0);
}

#[test]
fn usb_host_mode_sof_and_transaction_timeout() {
    // Host mode: SOF_EN starts 1 ms frames (SOF_RD counts, INTR.HOST_SOF
    // bit 2); a transaction with no device attached times out
    // (SIE_STATUS.RX_TIMEOUT bit 27, INTR.ERROR_RX_TIMEOUT bit 6).
    let mut m = Rp2040::new();
    wr(&mut m, MAIN_CTRL, 0b11);
    wr(&mut m, SIE_CTRL, 1 << 9);
    us(&mut m, 999.0);
    assert_eq!(rd(&mut m, SOF_RD), 0);
    us(&mut m, 1.0);
    assert_eq!(rd(&mut m, SOF_RD), 1);
    assert_eq!(rd(&mut m, INTR) & (1 << 2), 1 << 2);
    us(&mut m, 2000.0);
    assert_eq!(rd(&mut m, SOF_RD), 3);
    wr(&mut m, SIE_CTRL, (1 << 9) | 1); // START_TRANS
    assert_eq!(rd(&mut m, SIE_STATUS) & (1 << 27), 1 << 27);
    assert_eq!(rd(&mut m, INTR) & (1 << 6), 1 << 6);
    wr(&mut m, SIE_CTRL, 0); // SOF off
    us(&mut m, 5000.0);
    // The reference leaves the pending SOF alarm linked: one more frame
    // lands, and it does not re-arm.
    assert_eq!(rd(&mut m, SOF_RD), 4);
}

// ---- enumeration, played from the firmware's side ---------------------------------------------

/// Configuration descriptor of a one-port CDC device: configuration, one
/// CDC data interface with a bulk OUT (0x02) and bulk IN (0x82) endpoint.
fn cdc_descriptors() -> Vec<u8> {
    flat(&[
        &[9, 2, 32, 0, 1, 1, 0, 0x80, 50],
        &[9, 4, 0, 0, 2, 10, 0, 0, 0],
        &[7, 5, 0x02, 2, 64, 0, 0],
        &[7, 5, 0x82, 2, 64, 0, 0],
    ])
}

/// Firmware sends `data` on EP0 IN: payload at the EP0 buffer (DPRAM 0x100),
/// then the buffer control word (AVAILABLE, FULL, LAST, length).
fn ep0_in(m: &mut Rp2040, data: &[u8]) {
    dpram_write(m, 0x100, data);
    m.write_u32(DPRAM + 0x80, (1 << 10) | (1 << 15) | (1 << 14) | data.len() as u32);
}

/// Runs the controller from `MAIN_CTRL.CONTROLLER_EN` through the whole CDC
/// enumeration, as MicroPython's TinyUSB would answer it.
fn enumerated(hold_reads: bool) -> Rp2040 {
    let mut m = Rp2040::new();
    m.periph.cdc.hold_reads = hold_reads;
    wr(&mut m, INTE, (1 << 16) | (1 << 12)); // SETUP_REQ, BUS_RESET
    wr(&mut m, MAIN_CTRL, 1);
    m.clock_tick(10_000_000.0 - 1.0);
    assert_eq!(rd(&mut m, SIE_STATUS) & (1 << 19), 0, "bus reset comes 10 ms after enable");
    m.clock_tick(1.0);
    assert_eq!(rd(&mut m, SIE_STATUS) & (1 << 19), 1 << 19);
    assert_eq!(rd(&mut m, INTR) & (1 << 12), 1 << 12);
    // Firmware acknowledges the reset: the host assigns address 1.
    wr(&mut m, SIE_STATUS, 1 << 19);
    assert_eq!(&m.dpram[..8], &[0x00, 0x05, 1, 0, 0, 0, 0, 0]);
    assert_eq!(rd(&mut m, SIE_STATUS) & (1 << 17), 1 << 17, "SETUP_REC");
    assert_eq!((rd(&mut m, SIE_STATUS) >> 2) & 3, 0b01, "line state J");
    assert_eq!(rd(&mut m, SIE_STATUS) & (1 << 16), 1 << 16, "CONNECTED");
    // Firmware completes the SET_ADDRESS status stage with a zero-length packet.
    wr(&mut m, SIE_STATUS, 1 << 17);
    ep0_in(&mut m, &[]);
    us(&mut m, 9.0);
    assert_eq!(m.periph.cdc.descriptors_size, None);
    us(&mut m, 1.0);
    assert_eq!(&m.dpram[..8], &[0x80, 0x06, 0x00, 0x02, 0, 0, 9, 0], "GET_DESCRIPTOR(configuration, 9)");
    // Firmware answers with the 9-byte header; the host asks for all of it.
    wr(&mut m, SIE_STATUS, 1 << 17);
    let d = cdc_descriptors();
    ep0_in(&mut m, &d[..9]);
    us(&mut m, 10.0);
    assert_eq!(m.periph.cdc.descriptors_size, Some(32));
    assert_eq!(&m.dpram[..8], &[0x80, 0x06, 0x00, 0x02, 0, 0, 32, 0]);
    wr(&mut m, SIE_STATUS, 1 << 17);
    ep0_in(&mut m, &d);
    us(&mut m, 10.0);
    assert_eq!((m.periph.cdc.in_endpoint, m.periph.cdc.out_endpoint), (2, 2));
    assert_eq!(&m.dpram[..8], &[0x00, 0x09, 1, 0, 0, 0, 0, 0], "SET_CONFIGURATION(1)");
    // Status stage of SET_CONFIGURATION: the host sets DTR and RTS and reports connected.
    wr(&mut m, SIE_STATUS, 1 << 17);
    assert!(!m.periph.cdc.connected);
    ep0_in(&mut m, &[]);
    us(&mut m, 10.0);
    assert_eq!(&m.dpram[..8], &[0x20, 0x22, 3, 0, 0, 0, 0, 0], "SET_CONTROL_LINE_STATE(DTR|RTS)");
    assert!(m.periph.cdc.connected);
    assert!(m.periph.cdc.initialized);
    wr(&mut m, SIE_STATUS, 1 << 17);
    m
}

#[test]
fn cdc_enumeration_completes_in_the_usb_spec_order_at_fixed_times() {
    let m = enumerated(false);
    // 10 ms to the bus reset, then four host steps 10 us after each firmware packet.
    assert_eq!(m.clock.nanos, 10_040_000.0);
}

#[test]
fn cdc_bulk_in_data_reaches_the_serial_output() {
    // Firmware sends "hi" on EP2 IN: endpoint control 0x10 holds the buffer
    // address (bits 15:6), the buffer control word is at 0x80 + 8 * 2.
    let mut m = enumerated(false);
    dpram_write(&mut m, 0x10, &(0x8000_0000u32 | (2 << 26) | 0x200).to_le_bytes());
    dpram_write(&mut m, 0x200, b"hi");
    m.write_u32(DPRAM + 0x90, (1 << 10) | (1 << 15) | 2);
    assert_eq!(rd(&mut m, BUFF_STATUS) & (1 << 4), 1 << 4, "EP2 IN buffer status bit 4");
    us(&mut m, 9.0);
    assert!(m.periph.cdc.serial_out.is_empty());
    us(&mut m, 1.0);
    assert_eq!(m.periph.cdc.serial_out, b"hi");
    // The controller took the buffer: AVAILABLE and FULL clear.
    let ctrl = u32::from_le_bytes([m.dpram[0x90], m.dpram[0x91], m.dpram[0x92], m.dpram[0x93]]);
    assert_eq!(ctrl & ((1 << 10) | (1 << 15)), 0);
}

#[test]
fn cdc_bulk_out_read_is_answered_with_what_the_host_queued() {
    // Plain USBCDC: an armed read gets what is in the FIFO, empty or not, 10 us later.
    let mut m = enumerated(false);
    dpram_write(&mut m, 0x14, &(0x8000_0000u32 | (2 << 26) | 0x300).to_le_bytes());
    m.cdc_send_serial_byte(b'x');
    m.cdc_send_serial_byte(b'y');
    m.write_u32(DPRAM + 0x94, (1 << 10) | 64); // EP2 OUT, up to 64 bytes
    us(&mut m, 9.0);
    assert_eq!(rd(&mut m, BUFF_STATUS) & (1 << 5), 0);
    us(&mut m, 1.0);
    assert_eq!(rd(&mut m, BUFF_STATUS) & (1 << 5), 1 << 5, "EP2 OUT buffer status bit 5");
    assert_eq!(&m.dpram[0x300..0x302], b"xy");
    let ctrl = u32::from_le_bytes([m.dpram[0x94], m.dpram[0x95], m.dpram[0x96], m.dpram[0x97]]);
    assert_eq!(ctrl & 0x3ff, 2, "length");
    assert_eq!(ctrl & (1 << 15), 1 << 15, "FULL");
    // With nothing queued the same read completes empty.
    m.write_u32(DPRAM + 0x94, (1 << 10) | 64);
    us(&mut m, 10.0);
    let ctrl = u32::from_le_bytes([m.dpram[0x94], m.dpram[0x95], m.dpram[0x96], m.dpram[0x97]]);
    assert_eq!(ctrl & 0x3ff, 0);
}

#[test]
fn wrapper_policy_holds_an_armed_read_until_repl_bytes_arrive() {
    // brownout's RP2040Mcu: an armed OUT read with nothing to send is held,
    // not answered empty; send() queues bytes and answers it.
    let mut m = enumerated(true);
    dpram_write(&mut m, 0x14, &(0x8000_0000u32 | (2 << 26) | 0x300).to_le_bytes());
    m.write_u32(DPRAM + 0x94, (1 << 10) | 64);
    us(&mut m, 1000.0);
    assert_eq!(rd(&mut m, BUFF_STATUS) & (1 << 5), 0, "no empty answer");
    assert_eq!(m.periph.cdc.held_reads, vec![(2, 64)]);
    m.cdc_send(b"print(1+1)\r\n");
    assert!(m.periph.cdc.held_reads.is_empty());
    us(&mut m, 10.0);
    assert_eq!(&m.dpram[0x300..0x30c], b"print(1+1)\r\n");
}

#[test]
fn wrapper_policy_tops_up_the_512_entry_fifo_from_the_queue() {
    let mut m = enumerated(true);
    dpram_write(&mut m, 0x14, &(0x8000_0000u32 | (2 << 26) | 0x300).to_le_bytes());
    let big: Vec<u8> = (0..700u32).map(|i| i as u8).collect();
    m.write_u32(DPRAM + 0x94, (1 << 10) | 64);
    m.cdc_send(&big);
    assert_eq!(m.periph.cdc.tx_fifo.item_count(), 512 - 64);
    assert_eq!(m.periph.cdc.tx_queue.len(), 700 - 512);
    us(&mut m, 10.0);
    assert_eq!(&m.dpram[0x300..0x340], &big[..64]);
}

#[test]
fn reference_quirk_every_later_ep0_write_resends_set_configuration() {
    // USBCDC tests `descriptorsSize === descriptors.length` after every EP0
    // packet of more than one byte, so once enumeration has the full
    // descriptor set, any further multi-byte EP0 write (say a string
    // descriptor reply) makes the host send SET_CONFIGURATION again.
    let mut m = enumerated(false);
    dpram_write(&mut m, 0x00, &[0xff; 8]);
    ep0_in(&mut m, &[1, 2, 3]);
    us(&mut m, 10.0);
    assert_eq!(&m.dpram[..8], &[0x00, 0x09, 1, 0, 0, 0, 0, 0]);
}

#[test]
fn reference_quirk_only_one_read_buffer_finishes_per_alarm() {
    // `endpointReadDone` links a buffer and (re)schedules the endpoint's one
    // alarm; its callback shifts a single buffer. Two reads queued before it
    // fires leave the second waiting for an alarm nobody schedules again.
    let mut m = Rp2040::new();
    m.write_u32(DPRAM + 0x84, 64); // EP0 OUT buffer control: 64 bytes asked
    m.usb_endpoint_read_done(0, vec![1, 2]);
    m.usb_endpoint_read_done(0, vec![3, 4, 5]);
    us(&mut m, 10.0);
    assert_eq!(&m.dpram[0x100..0x102], &[1, 2]);
    assert_eq!(m.periph.usb.read_buffers[0].len(), 1);
    us(&mut m, 1000.0);
    assert_eq!(m.periph.usb.read_buffers[0].len(), 1, "still waiting");
}

#[test]
fn usb_dpram_is_plain_memory_outside_buffer_control() {
    let mut m = Rp2040::new();
    m.write_u32(DPRAM + 0x200, 0x1234_5678);
    m.write_u8(DPRAM + 0x201, 0xab);
    assert_eq!(m.read_u32(DPRAM + 0x200), 0x1234_ab78);
    m.write_u16(DPRAM + 0x202, 0xcdef);
    assert_eq!(m.read_u32(DPRAM + 0x200), 0xcdef_ab78);
    assert!(m.stop.is_none());
    assert_eq!(rd(&mut m, BUFF_STATUS), 0);
}
