/**
 * Resettable PTC (polyfuse) trip model, shared by the catalogue's ptc_fuse
 * part and the polyfuse on the Arduino Uno's USB supply path, so both trip
 * and recover the same way. Pure functions over a PtcState-shaped record; no
 * engine imports, so either caller can use it without a module cycle.
 */

export interface PtcTripState {
  tripped: boolean;
  tripStress: number;
  recoveryTime: number;
}

/** Tripped resistance = 200x rNormal: high enough to limit a fault current,
 *  but not truly open (a PTC does not blow, it goes high-R). */
export const PTC_TRIPPED_R_MULTIPLIER = 200;

export function ptcResistance(rNormal: number, tripped: boolean): number {
  return tripped ? rNormal * PTC_TRIPPED_R_MULTIPLIER : rNormal;
}

/** Advance the trip / recovery state by one committed step of length h. */
export function advancePtcTrip(
  st: PtcTripState,
  current: number,
  iHold: number,
  rNormal: number,
  h: number,
): void {
  if (!st.tripped) {
    // Normalized excess I²t: 100 ms at exactly 2× hold current, faster
    // at larger faults. Stress bleeds below hold current instead of being
    // erased by every PWM off interval.
    if (current >= iHold) {
      st.tripStress += Math.max(0, current * current - iHold * iHold) * h;
      const tripThreshold = 3 * iHold * iHold * 0.1;
      if (st.tripStress >= tripThreshold) {
        st.tripped = true;
        st.tripStress = 0;
        st.recoveryTime = 0;
      }
    } else {
      // Bleed stress when current is below trip threshold so short spikes
      // don't permanently accumulate.
      st.tripStress = Math.max(0, st.tripStress - h * iHold * iHold * 0.5);
    }
  } else {
    // Recovery also requires low hot-state dissipation. A fault that is
    // merely current-limited by the tripped PTC keeps it hot and must not
    // unrealistically reset after a fixed timer.
    const trippedPower = current * current * rNormal * PTC_TRIPPED_R_MULTIPLIER;
    const coolPower = iHold * iHold * rNormal * 0.1;
    if (current < iHold / 2 && trippedPower < coolPower) {
      st.recoveryTime += h;
      if (st.recoveryTime >= 2.0) {
        st.tripped = false;
        st.tripStress = 0;
        st.recoveryTime = 0;
      }
    } else {
      // Current rose above recovery threshold — restart the recovery timer.
      st.recoveryTime = 0;
    }
  }
}
