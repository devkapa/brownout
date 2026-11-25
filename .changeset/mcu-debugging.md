---
"brownout": minor
---

Arduino boards can be debugged at the source level. `SimEngine.setMcuBreakpoints(id, addresses)` pauses a board
before the instruction at any of the given program byte addresses (as a DWARF line table gives them);
`stepMcu(id, addresses, minSp)` resumes it until the next of a set of addresses at or above a stack depth, which is
how a host steps one source line over or into calls; `pausedMcus()`, `resumeMcu(id)`, `mcuDebugSnapshot(id)` (PC,
SP, SREG, registers, cycles) and `readMcuData(id, address, length)` complete it. A paused board runs nothing while
the rest of the circuit carries on. With no breakpoints set, a board runs exactly as before.
