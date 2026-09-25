//! Drivers: rail-referenced IC outputs and the inertial-delay rules
//! (core/README.md, "Kernel semantics: events", point 2). Owned by W1a
//! worker A1.
//!
//! A `Driver` is one IC output pin: a conductance to its own IC's hi/lo
//! supply nodes so load current debits the rails (contract point 2, spec
//! section 1.7), plus the inertial-cancellation bookkeeping of spec section
//! 2.4. The electrical fields (`node`, `hi_node`, `lo_node`, `conductance`,
//! `hi_level`, `lo_level`) are read by the partition solver when it stamps a
//! driven pin (pec3.mjs `_stamp`: a driver in HIGH mode contributes
//! `conductance * hi_level` toward `hi_node` and `conductance * (1 -
//! hi_level)` toward `lo_node`, symmetrically for LOW with `lo_level`; most
//! digital outputs use `hi_level = 1, lo_level = 0`, and a 555 OUT pin's
//! headroom-correct level is computed by its own device model and written
//! here). Stamping itself is W1b/W1c's job; this module does not solve
//! anything.
//!
//! The FSM in [`fsm`] is the part under test here: a pure function from
//! (committed state, in-flight transition, requested level, now, delay) to
//! (new in-flight transition, the `Commit` event to schedule, if any). It is
//! pure so every row of spec section 2.4 can be exercised as a direct
//! input/output table, with no heap, no partitions and no devices in scope.
//! `Driver` is a thin stateful wrapper around it for `kernel.rs` to hold in a
//! `Vec<Driver>` and call by id.

use crate::events::{Mode, Token};

/// An in-flight transition: the level a driver is committed to reaching, and
/// when. There is at most one of these per driver at a time (spec 2.4).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pending {
    pub target: Mode,
    pub due: f64,
}

/// The pure inertial-cancellation FSM (spec section 2.4) and the token
/// bookkeeping that lets a superseded `Commit` event be recognised and
/// dropped, lazily, when it is eventually popped (events.rs's contract).
///
/// None of these functions touch a heap or a `Driver`; they take and return
/// plain state so every FSM row is a value-in, value-out test.
pub mod fsm {
    use super::{Mode, Pending, Token};

    /// The result of a level request: the driver's new in-flight transition
    /// (or none), and — when this call must cause a new `Commit` event —
    /// the time and mode to schedule it with. The event's token is always
    /// the token this call returns; the caller (`Driver::request`) reads it
    /// off the updated state rather than out of this struct, so there is
    /// only one place a token is threaded through.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct Transition {
        pub pending: Option<Pending>,
        pub token: Token,
        pub schedule: Option<(f64, Mode)>,
    }

    /// One request for a driver to reach `requested`, evaluated at `now`
    /// against its device's `delay`. `committed` is `None` only for a driver
    /// that has never committed to anything yet; `pending`/`token` are the
    /// driver's current in-flight state (`None`/`0` for a fresh driver).
    ///
    /// This is deliberately not the literal transcription of classic's
    /// `_updateDelayedOutputs` (spec 2.4), which folds a due-fire and a new
    /// schedule into one call keyed off wall-clock `simTime + h`. The two-
    /// phase delta cycle (contract point 3, decision 4) already separates
    /// "apply due commits" from "devices re-evaluate and request levels"
    /// into different kernel phases, so a due-fire is always handled by
    /// [`apply_commit`] before any `request` call sees the result. What this
    /// function reproduces is the *rule*, row by row, not classic's
    /// single-function shape: pec2.mjs/pec3.mjs's `drive()` already factors
    /// it the same way, which is the mechanism the W1a brief says to port.
    pub fn request(
        committed: Option<Mode>,
        pending: Option<Pending>,
        token: Token,
        requested: Mode,
        now: f64,
        delay: f64,
    ) -> Transition {
        match committed {
            // Row: first observation. Nothing has ever committed on this
            // pin, so there is nothing for `requested` to differ from; it
            // schedules immediately regardless of `delay` (spec 2.4: "first
            // sample: zero delay"). This lands in the very next delta at
            // this same instant (contract point 3), not truly
            // instantaneously, because the kernel's two-phase cycle only
            // applies commits at the start of a delta.
            None => Transition {
                pending: Some(Pending { target: requested, due: now }),
                token: token + 1,
                schedule: Some((now, requested)),
            },
            // Row: requesting the already-committed level.
            Some(c) if c == requested => match pending {
                // Row: a glitch back cancels the in-flight transition. The
                // token bump is the whole cancellation: the `Commit` event
                // already sitting in the heap for the old token is left
                // exactly where it is (events.rs's lazy-deletion contract)
                // and is silently dropped on pop, so the glitch never takes
                // effect — "a glitch shorter than the delay is swallowed".
                Some(_) => Transition { pending: None, token: token + 1, schedule: None },
                // Nothing in flight and nothing to do.
                None => Transition { pending: None, token, schedule: None },
            },
            // committed is Some(_) and differs from `requested`.
            Some(_) => match pending {
                // Row: already pending to reach this exact target. No
                // reschedule, no fresh token: the original `due` stands.
                // This is what keeps a steady, unchanged request from
                // restarting its own delay on every re-evaluation.
                Some(p) if p.target == requested => {
                    Transition { pending: Some(p), token, schedule: None }
                }
                // Row: a new target — either nothing was pending (an
                // ordinary transition) or something else was (at most one
                // in-flight transition per pin: this supersedes it). Spec
                // 2.4: "a re-glitch gets a fresh due time" — always `now +
                // delay` from here, never the superseded transition's due.
                _ => {
                    let due = now + delay;
                    Transition {
                        pending: Some(Pending { target: requested, due }),
                        token: token + 1,
                        schedule: Some((due, requested)),
                    }
                }
            },
        }
    }

    /// The outcome of popping a `Commit` event against a driver's current
    /// token.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum Applied {
        /// `event_token` no longer matches: this commit was superseded or
        /// cancelled after it was scheduled. Dropped silently — this is the
        /// lazy-deletion half of the token contract, never an error.
        Stale,
        /// The token matched but the committed level was already
        /// `event_mode` (defensive parity with classic's own guard,
        /// `if (d.mode === ev.mode) return;`; not reachable through
        /// [`request`] alone, since a matching request never schedules a
        /// new event, but kept so callers do not have to assume it can't
        /// happen).
        Unchanged,
        /// The token matched and the committed level actually changed to
        /// this new mode; the caller must mark the driver's partition dirty.
        Changed(Mode),
    }

    /// Applies a popped `Commit { mode: event_mode, token: event_token }`
    /// against a driver whose committed level is `committed` and whose
    /// current token is `token`.
    pub fn apply_commit(committed: Option<Mode>, token: Token, event_mode: Mode, event_token: Token) -> Applied {
        if event_token != token {
            return Applied::Stale;
        }
        if committed == Some(event_mode) {
            return Applied::Unchanged;
        }
        Applied::Changed(event_mode)
    }

    /// Drops a driver's delay history entirely: the device did not drive
    /// this pin at all in the current evaluation (a tri-stated bus, a 74LS245
    /// direction flip — spec 2.4's cleanup pass, "drops the triad for any
    /// pin missing from `immediate`"). The next [`request`] on this pin is
    /// again a first observation with zero delay, exactly as if the driver
    /// had just been created. Returns the new token (bumped, so any
    /// `Commit` event already scheduled for this pin is stale on pop).
    pub fn reset(token: Token) -> Token {
        token + 1
    }
}

/// One IC output pin. See the module doc for the electrical fields; the FSM
/// state (`committed`/`pending`/`token`) is private, mutated only through
/// [`Driver::request`], [`Driver::apply_commit`] and [`Driver::reset`], each
/// a thin wrapper around the matching pure function in [`fsm`].
#[derive(Debug, Clone)]
pub struct Driver {
    /// The pin's own node.
    pub node: usize,
    /// The IC's hi supply node (its `vcc`/`vdd`).
    pub hi_node: usize,
    /// The IC's lo supply node (its `gnd`/`vss`).
    pub lo_node: usize,
    /// Output conductance before any rail-following adjustment (contract
    /// point 2; some kinds, such as a 555 OUT, recompute an effective
    /// conductance from live rail voltage instead of using this directly —
    /// device-specific, W1c/W1e).
    pub conductance: f64,
    /// Fraction of `conductance` directed toward `hi_node` while driving
    /// HIGH (the rest goes to `lo_node`). `1.0` for an ordinary logic
    /// output.
    pub hi_level: f64,
    /// Fraction of `conductance` directed toward `lo_node` while driving
    /// LOW. `0.0` for an ordinary logic output.
    pub lo_level: f64,
    committed: Option<Mode>,
    pending: Option<Pending>,
    token: Token,
}

impl Driver {
    pub fn new(node: usize, hi_node: usize, lo_node: usize, conductance: f64, hi_level: f64, lo_level: f64) -> Self {
        Driver {
            node,
            hi_node,
            lo_node,
            conductance,
            hi_level,
            lo_level,
            committed: None,
            pending: None,
            token: 0,
        }
    }

    pub fn committed(&self) -> Option<Mode> {
        self.committed
    }

    pub fn pending(&self) -> Option<Pending> {
        self.pending
    }

    pub fn token(&self) -> Token {
        self.token
    }

    /// Requests `level` at `now`, given this pin's device delay. Updates the
    /// driver's in-flight state and returns `(due, mode, token)` for the
    /// caller to push onto the event heap as `EventKind::Commit`, when this
    /// call causes a new one to be scheduled.
    pub fn request(&mut self, level: Mode, now: f64, delay: f64) -> Option<(f64, Mode, Token)> {
        let t = fsm::request(self.committed, self.pending, self.token, level, now, delay);
        self.pending = t.pending;
        self.token = t.token;
        t.schedule.map(|(due, mode)| (due, mode, self.token))
    }

    /// Applies a popped `Commit { mode, token }` event. Returns `true` only
    /// when the committed level actually changed, which is the caller's
    /// (`kernel.rs`'s) signal to mark this driver's partition dirty.
    pub fn apply_commit(&mut self, mode: Mode, token: Token) -> bool {
        match fsm::apply_commit(self.committed, self.token, mode, token) {
            fsm::Applied::Changed(m) => {
                self.committed = Some(m);
                self.pending = None;
                true
            }
            fsm::Applied::Unchanged => {
                self.pending = None;
                false
            }
            fsm::Applied::Stale => false,
        }
    }

    /// See [`fsm::reset`].
    pub fn reset(&mut self) {
        self.token = fsm::reset(self.token);
        self.committed = None;
        self.pending = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::events::Mode::{Hiz, High, Low};

    // ---- fsm::request, row by row (spec section 2.4) ----------------------

    #[test]
    fn row_first_observation_has_zero_delay() {
        // "first sample: zero delay" -- schedules at `now` regardless of the
        // device's own (nonzero) delay, and only because committed is None.
        let t = fsm::request(None, None, 0, High, 10.0, 20e-9);
        assert_eq!(t.schedule, Some((10.0, High)));
        assert_eq!(t.pending, Some(Pending { target: High, due: 10.0 }));
        assert_eq!(t.token, 1);
    }

    #[test]
    fn row_ordinary_transition_schedules_now_plus_delay() {
        let t = fsm::request(Some(Low), None, 5, High, 100.0, 20e-9);
        assert_eq!(t.schedule, Some((100.0 + 20e-9, High)));
        assert_eq!(t.pending, Some(Pending { target: High, due: 100.0 + 20e-9 }));
        assert_eq!(t.token, 6);
    }

    #[test]
    fn row_glitch_shorter_than_delay_is_swallowed() {
        // Committed LOW, a transition to HIGH is pending. Before it is due,
        // the request reverts to LOW (the committed level): the pending
        // transition is cancelled by a token bump, not by touching the heap.
        // Whatever `Commit` event is still sitting there for the old token
        // is now stale and drops silently on pop (events.rs's lazy
        // deletion), so the HIGH transition never took effect.
        let due = 100.0 + 20e-9;
        let pending = Some(Pending { target: High, due });
        let t = fsm::request(Some(Low), pending, 6, Low, 105.0, 20e-9);
        assert_eq!(t.pending, None);
        assert_eq!(t.schedule, None);
        assert_eq!(t.token, 7, "the cancellation must bump the token so the stale Commit is recognisable");
    }

    #[test]
    fn row_reglitch_gets_a_fresh_due_time() {
        // After the glitch-back above cancelled the pending HIGH, a later,
        // independent request for HIGH again must not resurrect the old due
        // time: it is a brand new transition, `now + delay` from here.
        let t = fsm::request(Some(Low), None, 7, High, 200.0, 20e-9);
        assert_eq!(t.schedule, Some((200.0 + 20e-9, High)));
        assert_eq!(t.pending, Some(Pending { target: High, due: 200.0 + 20e-9 }));
        assert_ne!(t.pending.unwrap().due, 100.0 + 20e-9, "must not reuse the superseded transition's due time");
    }

    #[test]
    fn row_repeated_request_for_the_same_pending_target_does_not_reschedule() {
        // A steady request for a target already pending must not restart its
        // own delay on every re-evaluation, or an output would never settle
        // under repeated per-delta re-evaluation.
        let due = 50.0 + 20e-9;
        let pending = Some(Pending { target: High, due });
        let t = fsm::request(Some(Low), pending, 9, High, 50.0 + 1e-9, 20e-9);
        assert_eq!(t.schedule, None, "no new Commit event; the original due stands");
        assert_eq!(t.pending, Some(Pending { target: High, due }));
        assert_eq!(t.token, 9, "no cancellation happened, so the token must not move");
    }

    #[test]
    fn row_at_most_one_transition_in_flight_a_redirect_supersedes_it() {
        // Committed HIZ, pending HIGH. A request for LOW (neither the
        // committed level nor the pending target) must replace the pending
        // transition outright, with a fresh due time, never queue a second
        // one alongside it.
        let pending = Some(Pending { target: High, due: 30.0 + 20e-9 });
        let t = fsm::request(Some(Hiz), pending, 3, Low, 31.0, 20e-9);
        assert_eq!(t.pending, Some(Pending { target: Low, due: 31.0 + 20e-9 }));
        assert_eq!(t.schedule, Some((31.0 + 20e-9, Low)));
        assert_eq!(t.token, 4, "redirecting a pending transition must bump the token so the superseded HIGH commit is stale");
    }

    #[test]
    fn row_requesting_the_committed_level_with_nothing_pending_is_inert() {
        let t = fsm::request(Some(High), None, 4, High, 60.0, 20e-9);
        assert_eq!(t, fsm::Transition { pending: None, token: 4, schedule: None });
    }

    #[test]
    fn zero_delay_device_still_goes_through_the_ordinary_path_after_first_observation() {
        // A live/zero-delay path (spec 2.5, e.g. 28C16 reads) is not a
        // special case of the FSM: the caller simply supplies delay = 0 on
        // every request after the first. This still produces a fresh Commit
        // event each time the level actually changes, at `now`.
        let t = fsm::request(Some(Low), None, 0, High, 77.0, 0.0);
        assert_eq!(t.schedule, Some((77.0, High)));
    }

    // ---- fsm::apply_commit -------------------------------------------------

    #[test]
    fn apply_commit_drops_a_stale_token_silently() {
        assert_eq!(fsm::apply_commit(Some(Low), 5, High, 4), fsm::Applied::Stale);
    }

    #[test]
    fn apply_commit_reports_changed_on_a_matching_token() {
        assert_eq!(fsm::apply_commit(Some(Low), 5, High, 5), fsm::Applied::Changed(High));
    }

    #[test]
    fn apply_commit_reports_unchanged_when_already_at_that_level() {
        assert_eq!(fsm::apply_commit(Some(High), 5, High, 5), fsm::Applied::Unchanged);
    }

    // ---- Driver: the stateful wrapper end to end ---------------------------

    #[test]
    fn driver_first_request_commits_with_zero_delay_after_one_apply() {
        let mut d = Driver::new(1, 2, 3, 1.0 / 50.0, 1.0, 0.0);
        assert_eq!(d.committed(), None);
        let (due, mode, token) = d.request(High, 0.0, 20e-9).expect("first observation always schedules");
        assert_eq!(due, 0.0);
        assert_eq!(mode, High);
        assert!(d.apply_commit(mode, token));
        assert_eq!(d.committed(), Some(High));
        assert_eq!(d.pending(), None);
    }

    #[test]
    fn driver_glitch_then_apply_of_the_stale_commit_is_a_no_op() {
        let mut d = Driver::new(1, 2, 3, 1.0 / 50.0, 1.0, 0.0);
        let first_token = d.request(Low, 0.0, 0.0).unwrap().2;
        d.apply_commit(Low, first_token);
        assert_eq!(d.committed(), Some(Low));

        // Schedule a transition to HIGH, then glitch back to LOW before it
        // is due -- this cancels it (token bump) without touching the
        // event that is still sitting in some caller's heap.
        let (due, mode, stale_token) = d.request(High, 10.0, 20e-9).unwrap();
        assert!(d.request(Low, 10.0 + 5e-9, 20e-9).is_none(), "the glitch-back cancels, it does not schedule");
        assert_eq!(d.committed(), Some(Low));

        // The caller pops that now-stale Commit{HIGH, stale_token} at `due`:
        // applying it must change nothing.
        assert!(!d.apply_commit(mode, stale_token));
        assert_eq!(d.committed(), Some(Low), "a stale commit must never move the committed level");
        let _ = due;
    }

    #[test]
    fn driver_reset_forces_the_next_request_to_be_a_first_observation_again() {
        let mut d = Driver::new(1, 2, 3, 1.0 / 50.0, 1.0, 0.0);
        let (_, mode, token) = d.request(High, 0.0, 20e-9).unwrap();
        d.apply_commit(mode, token);
        assert_eq!(d.committed(), Some(High));

        d.reset();
        assert_eq!(d.committed(), None);
        assert_eq!(d.pending(), None);

        let (due, ..) = d.request(Low, 5.0, 20e-9).unwrap();
        assert_eq!(due, 5.0, "post-reset, the next request is a first observation: zero delay");
    }

    #[test]
    fn driver_reset_makes_a_previously_scheduled_commit_stale() {
        let mut d = Driver::new(1, 2, 3, 1.0 / 50.0, 1.0, 0.0);
        let first_token = d.request(Low, 0.0, 0.0).unwrap().2;
        d.apply_commit(Low, first_token);
        let (_, mode, token) = d.request(High, 0.0, 20e-9).unwrap();
        d.reset();
        assert!(!d.apply_commit(mode, token), "the reset must invalidate the transition it interrupted");
        assert_eq!(d.committed(), None);
    }
}
