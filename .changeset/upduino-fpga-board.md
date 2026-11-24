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
configures make a clock edge; the RGB pads used as plain outputs are open-drain with no pull-up, as on the UP5K. A PCF names a bus
bit by its declared index (`sw[3:1]`, `led[0:2]`). Ports are placed by name (`gpio_N` for package pin N,
`led_red`/`led_green`/`led_blue`, `spi_*` as the UPduino's own PCF names them) or by the PCF (with nextpnr's
`-pullup yes` and `-pullup_resistor` 3P3K, 6P8K, 10K or 100K; any other strength is refused; `set_frequency` is
ignored), and the board's 12 MHz oscillator drives gpio_20 while its `osc_jumper` is fitted. On the breadboard the
board runs from its USB supply only, drives and reads its pins as a Pico does (a pull-up is the strength the PCF selects, or the iCE40's default
pull-up, about 88 kOhm), sinks an RGB pad's set current (4 mA per step of `RGBn_CURRENT`, 2 mA in half-current mode, with 0.5 V of
headroom; Lattice TN1288) rather than pulling it low, drives the board oscillator's 12 MHz onto gpio_20 with every
edge in each step reported as pin events (so a counter on that pin counts it), reports its on-board RGB LED in the
board state's new `rgb`, and reconfigures when its design
changes or USB power returns. A design it cannot run says why: input that is not a Yosys netlist, an unsupported cell
or I/O mode (DDR, LVDS), a port with no pin, a pin the board does not bring out, an output on the oscillator's pin,
or a combinational loop. A design that begins to oscillate while running stops with its pins released, and the board
state's new `error` says why. One step runs at most a million clock edges: a fast design stepped in long steps skips
the rest of the step and the board state's new `slowed` says its clocks fell behind. The fabric sizes its nets from the
ones a netlist uses, and `Ice40.advance` refuses a time that is not finite and non-negative (`RangeError`) and takes an
optional edge budget, returning false when it stops there.

The Rust core has the same fabric (feature `fpga`, not in the shipped WebAssembly module yet), built from the
reference's `Ice40.image()` and held to it by `scripts/fpga-lockstep.mjs`; CI now runs its tests
(`cargo test --features fpga`).
