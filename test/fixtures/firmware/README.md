# Bundled test firmware provenance

Copied from the devolt app (`apps/sim/public/firmware/`) during the B2 test-corpus
migration so the Pico integration suites stay runnable without the app checkout.
Bundled rather than skip-gated because the licence is permissive and the binary is
well under the 2 MB corpus budget.

## `pico-micropython.uf2`

- User-facing target: generic RP2040 MicroPython Board
- Upstream board target: `RPI_PICO`
- Upstream release: MicroPython v1.28.0, dated 2026-04-06
- Upstream source: <https://github.com/micropython/micropython/tree/v1.28.0>
- Upstream firmware: <https://micropython.org/resources/firmware/RPI_PICO-20260406-v1.28.0.uf2>
- SHA-256: `00aedb8431ebea83b470ef909509f16432c5b1b43e8f6eaf015f6f9560bc8152`
- Core licence: MIT, Copyright (c) 2013-2026 Damien P. George

MicroPython ports can contain third-party components under additional licences.
The RPI_PICO port includes pico-sdk components under BSD-3-Clause. Consult the
upstream v1.28.0 licence tree before updating or redistributing this binary.

When replacing the firmware, record the exact release, source URL, upstream
target, build date, licence information, and new SHA-256 here.
