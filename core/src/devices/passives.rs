//! The resistor (core/README.md's kind-code table; spec §7.3), moved from
//! `engine.rs`'s inline `ResistorStamp` handling with no behaviour change:
//! `stamp`/`current` below produce exactly the `g = 1/R` conductance and
//! `(Va-Vb)/R` current that used to be written directly into
//! `StaticPartitionSolver::build`/`recompute_currents`.

use crate::devices::Contribution;

/// `g = 1/R` between `pins[0]` and `pins[1]` (positional; core/README.md's
/// record-layout table). `kinds::resistor_check` already guarantees
/// `params[0] > 0` for anything that reaches here.
pub fn stamp(params: &[f64]) -> Vec<Contribution> {
    vec![Contribution::conductance(0, 1, 1.0 / params[0])]
}

/// `(Va-Vb)/R`, spec §7.3.
pub fn current(params: &[f64], pin_voltages: &[f64]) -> f64 {
    (pin_voltages[0] - pin_voltages[1]) / params[0]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stamps_the_reciprocal_conductance() {
        let s = stamp(&[2000.0]);
        assert_eq!(s.len(), 1);
        assert_eq!(s[0].pin_a, 0);
        assert_eq!(s[0].pin_b, 1);
        assert!((s[0].conductance - 0.0005).abs() < 1e-15);
        assert_eq!(s[0].injection, 0.0);
    }

    #[test]
    fn current_is_ohms_law() {
        let i = current(&[1000.0], &[5.0, 1.0]);
        assert!((i - 0.004).abs() < 1e-15);
    }
}
