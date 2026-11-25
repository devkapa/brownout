---
"brownout": minor
---

Six more part kinds run on the Rust/WASM core and are cleared for automatic use (`CORE_SUPPORTED_KINDS`): `74hc595`, `74hc165`, `cd4017`, `cd4060`, `cd4511` and `74ls153`. A circuit made of these and the kinds the core already ran goes to the core with no change on the host's side; every other circuit still runs on the classic engine. Each was checked on both engines against repository fixtures (the per-kind table reads "pass"), its snapshot, rollback and carried-reload state held by tests, and its datasheet delays measured at the pin. A 595 now drives its QH' pin (`qh2`) as a ninth output, as classic does. The 46 kinds the core already ran read bit for bit as before.

Recorded divergences from classic: these chips follow their datasheet delays (the 74HC and 74LS figures at the supply they are rated for) where classic's edges are instant, and the 4000 series' delays (`cd4017`, `cd4060`, `cd4511`) are held at their 5 V figures at every supply, where the datasheets shorten them as VDD rises. A 74HC595 whose SRCLK and RCLK sit on separate nets and rise in the same instant stores the byte from before that instant's shift on the core (R61: every capture of an instant samples the pre-instant state, as on tied clocks), where classic's answer is the order its own evaluation takes. A 4511's LE has no datasheet figure and uses the package delay.
