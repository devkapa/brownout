---
"brownout": patch
---

A 74LS47 segment that turns off no longer keeps the `output_sag` it raised while it was lit. The segments are open-collector outputs, so only a segment that is sinking can sag; once the decoder releases it the load alone sets its voltage and the sag no longer applies. The scan stopped reporting the released segment, but the clean-up that drops the old record skipped open-collector pins, so a segment overloaded on one digit (a segment driven with too little series resistance, say) kept its sag on every later digit that leaves it dark, through reloads, until failures were cleared. The clean-up now covers open-collector pins, so the sag clears on the next step after the segment is released. An LM393's released output clears the same way.
