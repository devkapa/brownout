---
"brownout": patch
---

An LED's overcurrent damage limit now honours a user-set `iRated` the same way its I-V curve already does. `commitLedOvercurrentStress` (shared by `led`, `bicolor_led` and `rgb_led`) read the catalog's `if_max` before `params.iRated`, the reverse of the I-V stamp's precedence, which always resolves `iRated` first. Raising a part's `iRated` past its catalog rating therefore changed how brightly it drove without changing when it failed: it kept latching `led_failed` at the catalog's original limit (typically 20 mA), and lowering `iRated` below the catalog rating never latched at all until the part hit the untouched catalog ceiling. The damage limit now reads `params.iRated ?? specs.if_max ?? 0.02 A`, exactly like the stamp, so a de-rated or up-rated LED fails at the current the user actually asked for.
