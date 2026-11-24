---
"brownout": minor
---

A new board kind, `upduino`: the UPduino v3.1 and its iCE40UP5K FPGA, running a design synthesized by Yosys
(`synth_ice40 -top top -nobram; write_json`), given as the board's `netlist` parameter with an optional `pcf`. Its
fabric (`Ice40`, exported with `checkFpgaProgram`, `parsePcf`, `readPcf` and the board's pin list) runs the cells
such a design uses: LUTs, the carry chain, every flip-flop variant (enable, synchronous and asynchronous set and
reset, either edge), SB_IO with its pull-up and output enable (a bidirectional pad reads its own drive back while
enabled, as open-drain I2C needs), tristate ports, outputs tied to constants, global buffers, the 48 MHz and 10 kHz
oscillators with their enables and the RGB LED driver with its enables and per-channel current. It runs the module
Yosys marks top (also from `write_json -compat-int`), and neither power-up nor the levels the circuit holds while it
configures make a clock edge; the RGB pads used as plain outputs are open-drain, as on the UP5K. A PCF names a bus
bit by its declared index (`sw[3:1]`, `led[0:2]`). Ports are placed by name (`gpio_N` for package pin N,
`led_red`/`led_green`/`led_blue`, `spi_*`) or by the PCF (with nextpnr's `-pullup yes`; `set_frequency` is
ignored), and the board's 12 MHz oscillator drives gpio_20 while its `osc_jumper` is fitted. On the breadboard the
board runs from its USB supply only, drives and reads its pins as a Pico does (an SB_IO pull-up is the iCE40's weak
pull-up, about 88 kOhm), reports its on-board RGB LED in the board state's new `rgb`, and reconfigures when its design
changes or USB power returns. A design it cannot run says why: input that is not a Yosys netlist, an unsupported cell
or I/O mode (DDR, LVDS), a port with no pin, a pin the board does not bring out, an output on the oscillator's pin,
or a combinational loop. A design that begins to oscillate while running stops with its pins released, and the board
state's new `error` says why.

The Rust core has the same fabric (feature `fpga`, not in the shipped WebAssembly module yet), built from the
reference's `Ice40.image()` and held to it by `scripts/fpga-lockstep.mjs`.
