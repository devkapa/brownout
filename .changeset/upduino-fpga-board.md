---
"brownout": minor
---

A new board kind, `upduino`: the UPduino v3.1 and its iCE40UP5K FPGA, running a design synthesized by Yosys
(`synth_ice40 -top top -nobram; write_json`), given as the board's `netlist` parameter with an optional `pcf`. Its
fabric (`Ice40`, exported with `checkFpgaProgram`, `parsePcf` and the board's pin list) runs the cells such a design
uses: LUTs, the carry chain, every flip-flop variant (enable, synchronous and asynchronous set and reset, either
edge), SB_IO with its pull-up and output enable, global buffers, the 48 MHz and 10 kHz oscillators and the RGB LED
driver. Ports are placed by name (`gpio_N` for package pin N, `led_red`/`led_green`/`led_blue`, `spi_*`) or by the
PCF, and the board's 12 MHz oscillator reaches gpio_20 while its `osc_jumper` is fitted. On the breadboard the
board runs from its USB supply only, drives and reads its pins as a Pico does (an SB_IO pull-up is the iCE40's
weak pull-up, about 88 kOhm), reports its on-board RGB LED in the board state's new `rgb`, and reconfigures when its
design changes. A design it cannot run says why: an unsupported cell, a port with no pin, a pin the board does not
bring out, a combinational loop, or a design that oscillates.

The Rust core has the same fabric (feature `fpga`, not in the shipped WebAssembly module yet), built from the
reference's `Ice40.image()` and held to it by `scripts/fpga-lockstep.mjs`.
