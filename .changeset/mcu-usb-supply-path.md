---
"brownout": patch
---

A USB-powered Arduino Uno or Nano with a supply on its 5V pin, or a USB-powered Pico with a supply on its 3V3 pin, now converges. Before, the engine treated the board's USB rail as an ideal voltage source, so a bench PSU wired in parallel made the system singular and no step converged. Each board now models its real USB supply path:

- Uno R3: USB host (`vcc`, default 5 V) behind 0.2 ohm of host and cable, the F1 polyfuse (MF-MSMF050-2, 0.15 ohm, 0.5 A hold) and the T1 P-MOSFET (FDN340P, 0.07 ohm). A supply on the 5V pin shares the rail; one above the host voltage back-feeds through the path and trips the polyfuse. The polyfuse trips and recovers like the catalogue `ptc_fuse` part, and `getPtcTripped()` lists the board's id while it is tripped.
- Nano 3.x: USB host behind 0.27 ohm, then the D1 Schottky (MBR0520LT1, fitted to its datasheet forward curve). A supply on the 5V pin above the USB side reverse-biases D1.
- Pico: 3V3(OUT) is the RT6150B regulator's output, 3.3 V behind 10 mohm. It sources current but does not sink it, so a supply holding the pin above 3.3 V turns the regulator off until the pin falls back below the setpoint.

Readings that change, all on USB-powered boards (usb_power on); boards with usb_power off are unchanged, and so is every circuit with no Uno, Nano or Pico:

- Uno 5V rail, idle (45 mA): 5.000 V to 4.981 V. It drops about 0.42 V per amp of further load, e.g. 4.975 V with one LED at about 14 mA. Everything referenced to that rail moves with it: pull-up levels, GPIO HIGH levels, LED and load currents (about -0.8% for an LED from a pin), and the `vcc` the ADC uses, so `analogRead()` of a fixed voltage reads about 0.4% higher.
- Nano 5V rail, idle (20 mA): 5.000 V to 4.764 V, from the Schottky drop (-4.7%). The same knock-on effects follow at that size: `analogRead()` of a fixed voltage reads about 5% higher.
- Pico 3V3 rail, idle (25 mA): 3.300 V to 3.29975 V, a 10 mV/A load line.
- Board supply current telemetry for the Pico reads 0 while a supply on its 3V3 pin is back-feeding it.
