---
"brownout": patch
---

`bench_psu` no longer sinks current. A linear bench supply's output stage only sources, so a supply driven above its setpoint by another source now lets go of its output (it delivers 0 A) and takes the output back when it falls below the setpoint. In transient the switch is placed at the crossing inside a step (the step is split there), and in `dcOperatingPoint` it is decided on each converged point and settled, or the stage declines. A supply on its own, sourcing, reads exactly as before: every classic suite and fixture, and devolt's apps/sim and apps/learn suites, are unchanged.

Circuits that change class:

- **A supply back-driven by another source:** it used to sink whatever was pushed into it; now it lets go and the other source sets the rail.
  - A 4.9 V supply on a USB-powered Uno's 5V pin sank 0.194 A. Now it delivers 0 A and the rail is the board's own 4.981 V.
  - A 5 V supply across a 9 V battery (1.5 ohm) with a 100 ohm load sank 2.62 A and held the rail at 5 V. Now it delivers 0 A and the battery holds the rail at 8.87 V.
  - Charging, where the supply's setpoint is above the battery, is unchanged.
- **Two supplies in parallel, or a supply across an ideal `voltage_source`:** these never converged, because two ideal sources on one node pair are singular. They now converge.
  - The higher setpoint carries the load and the lower one delivers 0 A.
  - When the higher one current-limits, the rail falls to the lower setpoint and that supply makes up the rest.
  - At equal setpoints they share the load. A supply wired in parallel with another ideal source carries a 1 uohm output resistance to make the system solvable, which moves its reading by 1 uV per amp.
- **Current limit:** a sinking supply no longer enters its current-limit fold. `iLimit` counts delivered current only.
- **Readouts of a supply that has let go:** element current 0, and in small-signal AC an open branch.
