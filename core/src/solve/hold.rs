//! The power hold procedure (core/README.md, W1c point 1a; classic's H8,
//! `fix/power-off-stall`): an IC whose own draw pulls its supply out of its
//! power window, while its absence lets the supply recover, has no
//! consistent power state, and a solve that iterates over it alternates
//! until its cap. The failing solve's owner counts each IC's verdict flips
//! between its iterations (the rail loop's passes, the rail Newton's
//! iterates, `Rail` events in a row, an iterated static partition's Newton
//! iterates); once one has flipped twice it probes in place:
//!
//! 1. the flipping ICs held powered, every other gated IC of the hub or
//!    partition at its last reading;
//! 2. those reading unpowered there starve themselves (none: `-4`);
//! 3. probed again held unpowered, which gives the drop their own draw
//!    causes;
//! 4. solved with them held unpowered and the rest as they were: the
//!    instant's solution (an instant has no length, so a part whose supply
//!    reads low either way is held too: ruling D2 (a)).
//!
//! A probe or the held solve that fails is `-4` as before. A held IC is
//! retried once per host step (an `advance` call), as classic's H8 retries
//! it at each of its own (ruling R51 (b)), and at any re-solve once its
//! supply passes its release level: probed powered, it is free again once
//! it carries itself. On the rail the release level (ruling D1 (a): the window edge
//! inward by that drop and a margin) is the `Rail` event that brings a retry
//! when nothing else does (Rails point 4). An IC held and released more than
//! [`EPISODE_MAX`] times in one sliding episode, each hold shorter than a
//! host step, declines `-4` (the guard of rulings R51, R54 (a): [`Episode`]).
//!
//! This module holds the procedure's pieces both owners share and the rail's
//! own steps; the static partition's run inside
//! [`super::static_part::StaticPartitionSolver`], on its mirror of the table,
//! and hand their result to the owner ([`PartitionedSolver::take_static_holds`]).

use super::*;
use crate::digital::power::{HoldKind, PowerHold, Release};
use crate::rails;

#[cfg(test)]
thread_local! {
    /// Re-entries tried after a fallback (`reenter_slides`), for the tests.
    pub(super) static REENTRIES: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
}

#[cfg(test)]
thread_local! {
    static FAIL_PROBE_RESIDUAL: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Test seam (ruling R45 (a)): while the guard lives, the re-solve that
/// checks an accepted probe's residual reports its partitions unconverged,
/// the fault the acceptance must reject.
#[cfg(test)]
pub(super) struct FailProbeResidual;

#[cfg(test)]
impl FailProbeResidual {
    pub(super) fn arm() -> FailProbeResidual {
        FAIL_PROBE_RESIDUAL.with(|c| c.set(true));
        FailProbeResidual
    }
}

#[cfg(test)]
impl Drop for FailProbeResidual {
    fn drop(&mut self) {
        FAIL_PROBE_RESIDUAL.with(|c| c.set(false));
    }
}

#[cfg(test)]
thread_local! {
    static RETRY_SEAM: std::cell::Cell<RetrySeam> = const { std::cell::Cell::new(RetrySeam::Normal) };
}

/// Test seam for F5V2 finding 5: runs a board with no retry at all, or with
/// every retry probing but freeing nothing; the two must agree bit for bit.
#[cfg(test)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum RetrySeam {
    Normal,
    NoRetry,
    AsksOnly,
}

#[cfg(test)]
impl RetrySeam {
    pub(super) fn get() -> RetrySeam {
        RETRY_SEAM.with(|c| c.get())
    }

    /// Sets the seam until the guard drops.
    pub(super) fn arm(self) -> impl Drop {
        struct Reset;
        impl Drop for Reset {
            fn drop(&mut self) {
                RETRY_SEAM.with(|c| c.set(RetrySeam::Normal));
            }
        }
        RETRY_SEAM.with(|c| c.set(self));
        Reset
    }
}

#[cfg(test)]
thread_local! {
    static NO_SLIDE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Test seam: while the guard lives no chip slides (ruling R92), so a test
/// of the hold's own machinery (its retry, rollback, snapshot and episode
/// guard, all still the fallback of a slide that cannot be followed) runs
/// on boards whose chips would otherwise slide.
#[cfg(test)]
pub(super) struct NoSlide;

#[cfg(test)]
impl NoSlide {
    pub(super) fn arm() -> NoSlide {
        NO_SLIDE.with(|c| c.set(true));
        NoSlide
    }
}

#[cfg(test)]
impl Drop for NoSlide {
    fn drop(&mut self) {
        NO_SLIDE.with(|c| c.set(false));
    }
}

#[cfg(test)]
thread_local! {
    static FOLLOW_CAP: std::cell::Cell<Option<u32>> = const { std::cell::Cell::new(None) };
}

/// Test seam: while the guard lives the follow cap is `cap` re-solves a
/// host step, with no per-time allowance ([`slide_follow_cap`]), so a test
/// can give a slide up and watch its fallback hold.
#[cfg(test)]
pub(super) struct FollowCap;

#[cfg(test)]
impl FollowCap {
    pub(super) fn arm(cap: u32) -> FollowCap {
        FOLLOW_CAP.with(|c| c.set(Some(cap)));
        FollowCap
    }
}

#[cfg(test)]
impl Drop for FollowCap {
    fn drop(&mut self) {
        FOLLOW_CAP.with(|c| c.set(None));
    }
}


#[cfg(test)]
thread_local! {
    static TRIALS_TWICE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Test seam: while the guard lives each trial step of a slide's aim
/// (`SteppedSolver::trial_rail_draw`) is computed once more first and that
/// result thrown away, so a test can show the trials leave no trace.
#[cfg(test)]
pub(super) struct TrialsTwice;

#[cfg(test)]
impl TrialsTwice {
    pub(super) fn arm() -> TrialsTwice {
        TRIALS_TWICE.with(|c| c.set(true));
        TrialsTwice
    }
}

#[cfg(test)]
impl Drop for TrialsTwice {
    fn drop(&mut self) {
        TRIALS_TWICE.with(|c| c.set(false));
    }
}

#[inline]
fn sliding_allowed() -> bool {
    #[cfg(test)]
    if NO_SLIDE.with(|c| c.get()) {
        return false;
    }
    true
}

/// Ruling R97 (b): per IC (positions in `ics`), whether one of its outputs
/// reaches, through passive parts with no capacitor in the path, an input
/// that output is a function of, directly or through its other outputs
/// (`digital::output_inputs`; every input for a kind that does not say):
/// a loop from an output back into the logic that drives it. Such a chip's
/// slide reads its own averaged output at its inputs,
/// which sit between their thresholds and toggle it at every solve (cold 22
/// tpl 2: a 555 with OUT on its TRIG and THR rang at 1e9 events a second at
/// its duty, never finishing a run), so it never slides: it chatters to the
/// episode guard's decline as before ruling R92. The paths are through
/// resistive parts, switches (either state), fuses and LEDs, and stop at a
/// fixed node and at the chip's own supply pins (the loop through its supply
/// is the slide's own: cold 757 tpl 0's OUT reaches its TRIG only across its
/// rail, through a 0.64 ohm fuse). Only a series capacitor breaks a loop: a
/// shunt one on the shared node does not exempt it. 22 tpl 2's OUT, TRIG
/// and THR share a node with 18 nF to ground (the loop's time constant about
/// 0.1 us) and it rang all the same; 688 tpl 5's DIS shares node 3 with its
/// THR and CV (14 nF to ground, 0.32 and 3.9 uF to other nodes), and
/// allowed to slide its 555 carried itself after two holds and the rail
/// climbed to the setpoint, which the fine reference never does (it
/// chatters about the floor in a 70 us limit cycle and declines at 0.01184
/// s); held, it declines at 0.01210 s. An output that feeds another gate of
/// its own chip whose output goes nowhere back is no loop (R53V: a 74LS00's
/// 1Y on its own 2A and 2B, 2Y open, read as self-fed when every input
/// counted, and the 4.279 V cell's slide was lost).
pub(crate) fn self_fed(program: &crate::program::Program, ics: &[(usize, digital::IcRecord)], fixed: &[bool]) -> Vec<bool> {
    use crate::kinds;
    let n = fixed.len();
    let mut adjacent: Vec<Vec<usize>> = vec![Vec::new(); n];
    for device in &program.devices {
        let passive = matches!(
            device.kind,
            kinds::RESISTOR | kinds::SWITCH | kinds::PUSH_BUTTON | kinds::SPDT_SWITCH | kinds::DIP_SWITCH | kinds::POTENTIOMETER | kinds::TRIMMER | kinds::FUSE | kinds::LDR | kinds::THERMISTOR | kinds::PTC_FUSE | kinds::LED | kinds::RGB_LED | kinds::BICOLOR_LED | kinds::SEG7_CC | kinds::SEG7_CA
        );
        if !passive {
            continue;
        }
        let nodes: Vec<usize> = device.pins.iter().map(|p| p.node).filter(|&a| a < n).collect();
        for &a in &nodes {
            for &b in &nodes {
                if a != b {
                    adjacent[a].push(b);
                }
            }
        }
    }
    ics.iter()
        .map(|(_, ic)| {
            let outputs = digital::output_capable(ic.kind);
            let blocked = |node: usize| node >= n || fixed[node] || node == ic.vcc.node || node == ic.gnd.node;
            // The nodes each output reaches.
            let reach: Vec<Vec<bool>> = outputs
                .iter()
                .map(|&o| {
                    let mut seen = vec![false; n];
                    let mut stack: Vec<usize> = ic.logic.get(o).map(|p| p.node).filter(|&a| !blocked(a)).into_iter().collect();
                    while let Some(a) = stack.pop() {
                        if std::mem::replace(&mut seen[a], true) {
                            continue;
                        }
                        stack.extend(adjacent[a].iter().copied().filter(|&b| !blocked(b) && !seen[b]));
                    }
                    seen
                })
                .collect();
            // Output `j` feeds output `k` when it reaches an input `k` reads.
            let reads = |k: usize| -> Vec<usize> {
                match digital::output_inputs(ic.kind, outputs[k]) {
                    Some(inputs) => inputs.to_vec(),
                    None => (0..ic.logic.len()).filter(|i| !outputs.contains(i)).collect(),
                }
            };
            let feeds = |j: usize, k: usize| reads(k).iter().any(|&i| ic.logic.get(i).is_some_and(|p| p.node < n && reach[j][p.node]));
            // A loop among the outputs (one feeding itself included): a
            // walk from each output along `feeds` that comes back to it.
            (0..outputs.len()).any(|start| {
                let mut seen = vec![false; outputs.len()];
                let mut stack = vec![start];
                while let Some(j) = stack.pop() {
                    for k in 0..outputs.len() {
                        if feeds(j, k) {
                            if k == start {
                                return true;
                            }
                            if !std::mem::replace(&mut seen[k], true) {
                                stack.push(k);
                            }
                        }
                    }
                }
                false
            })
        })
        .collect()
}

/// An IC's raw supply difference over `v`.
#[inline]
pub(crate) fn supply(ic: &digital::IcRecord, v: &[f64]) -> f64 {
    v[ic.vcc.node] - v[ic.gnd.node]
}

/// Step 5's release level (ruling D1 (a)) for an IC whose supply read `v_on`
/// held powered and `v_off` held unpowered: the window edge `v_on` lies past,
/// moved inward by the drop its draw caused, `v_off - v_on`, and two reached
/// widths of that edge (Rails point 4's `eta`). `None` when `v_on` is inside
/// the window: that IC carries itself and is not held.
pub(crate) fn release_after(ic: &digital::IcRecord, v_on: f64, v_off: f64, now: f64) -> Option<Release> {
    let (lo, hi) = digital::levels::power_window(ic);
    let eta = |edge: f64| rails::REACHED_REL * edge.abs().max(1.0);
    if v_on < lo {
        Some(Release { level: lo + (v_off - v_on).max(0.0) + 2.0 * eta(lo), rising: true, margin: 2.0 * eta(lo), since: now })
    } else if v_on > hi {
        Some(Release { level: hi - (v_on - v_off).max(0.0) - 2.0 * eta(hi), rising: false, margin: 2.0 * eta(hi), since: now })
    } else {
        None
    }
}

/// Whether a held IC's supply difference `s` has passed its release.
#[inline]
pub(crate) fn passed(release: Release, s: f64) -> bool {
    if release.rising {
        s >= release.level
    } else {
        s <= release.level
    }
}

/// The hold a starving IC takes at step 4, from its two probes. One whose
/// supply read inside its window held powered starves only through the
/// segment's slope (ruling R44 (a)): its level is the nearer edge plus two
/// reached widths, and its draw threshold decides.
pub(crate) fn held_off(ic: &digital::IcRecord, v_on: f64, v_off: f64, now: f64) -> PowerHold {
    let release = release_after(ic, v_on, v_off, now).unwrap_or_else(|| {
        let (lo, hi) = digital::levels::power_window(ic);
        let eta = |edge: f64| rails::REACHED_REL * edge.abs().max(1.0);
        if v_on - lo <= hi - v_on {
            Release { level: lo + 2.0 * eta(lo), rising: true, margin: 2.0 * eta(lo), since: now }
        } else {
            Release { level: hi - 2.0 * eta(hi), rising: false, margin: 2.0 * eta(hi), since: now }
        }
    });
    PowerHold { kind: HoldKind::Off, release: Some(release) }
}

/// The hold of an IC still starving at a retry although its supply, held
/// unpowered, passed `old`'s level: `held_off` with the fresh drop and twice
/// `old`'s margin (up to the drop itself). Each probe solves the rail to its
/// own tolerance, so a supply that lands a few reached widths short of the
/// edge at each level two widths further on would never get there (cold
/// 111 tpl 2: 8.5e-12 V short at each, 3,900 releases in 6 ns); doubled, a
/// few retries clear it.
pub(crate) fn refreshed(ic: &digital::IcRecord, old: Release, v_on: f64, v_off: f64, now: f64) -> PowerHold {
    let mut hold = held_off(ic, v_on, v_off, now);
    if let Some(r) = hold.release.as_mut().filter(|r| r.rising == old.rising) {
        let margin = (2.0 * old.margin).min((v_off - v_on).abs()).max(r.margin);
        r.level += if r.rising { margin - r.margin } else { r.margin - margin };
        r.margin = margin;
    }
    hold
}

/// The duties `keep_sliding` may solve again in one host step before the
/// instant declines (`-4`, classic's fallback): a slide whose board moves
/// it past W0 that often is followed at a cost no host step can carry, and a
/// frozen tab is never acceptable. A slide that follows its board's own
/// pace (cold 175 tpl 2's 555, one every 60 us or so) takes about a hundred
/// a host step.
pub(crate) const SLIDE_SOLVES_MAX: u32 = 1000;

/// The duties `keep_sliding` solves again in one host step before it gives
/// a slide up as one it cannot follow (ruling R92): the chip is held as it
/// was before the ruling (point 1a's hold and its release), so a board that
/// ran under the hold still runs. A slide whose rail a dynamic partition
/// moves past W0 at each of its steps (cold 230 tpl 2: a stepped hub
/// partition on a CC rail, a solve every 210 ns) takes thousands a host
/// step; one that follows its board's pace takes a few (cold 175 tpl 2:
/// about one every 2 ms).
pub(crate) const SLIDE_FOLLOW_MAX: u32 = 100;

/// The simulated time per [`SLIDE_FOLLOW_MAX`] re-solves a long host step
/// is allowed besides its own 100 ([`slide_follow_cap`]). The cap was set
/// at the app's own cadence, a host step a frame (1/60 s): 100 re-solves in
/// 16.7 ms, one per 167 us. Counted only per host step it gave a slide up at
/// longer steps that it followed at frame steps (R53V: cold 766 tpl 0 under
/// the trapezoid, about 160 re-solves across a 50 ms host step, one every
/// 0.3 ms, given up there, kept at 1 ms stops), so the answer hung on the
/// host's cadence. Per 10 ms the allowance at any host step is never below
/// the frame cadence's pace (100 per 10 ms is one per 100 us).
pub(crate) const SLIDE_FOLLOW_SPAN: f64 = 10e-3;

/// The re-solves `keep_sliding` may make in a host step `host_step` long
/// before it gives a slide up: [`SLIDE_FOLLOW_MAX`] a host step, or as many
/// per [`SLIDE_FOLLOW_SPAN`] of it, whichever allows more; and its
/// backstop decline, [`SLIDE_SOLVES_MAX`], scaled alike, so it stays ten
/// times the cap.
pub(crate) fn slide_follow_cap(host_step: f64) -> (u32, u32) {
    let scale = if host_step.is_finite() { (host_step / SLIDE_FOLLOW_SPAN).max(1.0) } else { 1.0 };
    let cap = |n: u32| (n as f64 * scale).min(u32::MAX as f64 / 2.0) as u32;
    #[cfg(test)]
    if let Some(follow) = FOLLOW_CAP.with(|c| c.get()) {
        return (follow, SLIDE_SOLVES_MAX);
    }
    (cap(SLIDE_FOLLOW_MAX), cap(SLIDE_SOLVES_MAX))
}

/// How far a sliding IC's supply may drift from its edge before its duty is
/// solved again (ruling R92): W0's voltage tolerance at the edge, so every
/// reading the slide sets stays within W0 of the slide's own.
pub(crate) fn slide_drift(edge: f64) -> f64 {
    1e-4 + 1e-3 * edge.abs()
}

/// What a slide's solve probed (`slide_exits`): its residual at duty 0 and
/// 1 where it probed them, and how many duties it probed (none: the duty
/// was left as it was).
#[derive(Debug, Clone, Copy, Default)]
pub(super) struct SlideEnds {
    at: [Option<f64>; 2],
    evals: u32,
}

/// The aim of a slide on a CC rail capacitor's segment (ruling R92;
/// `slide_residual`): the draw at the edge past the limit, `dir` signed,
/// that brings the supply from `off` past the edge back to it over `span`.
/// The segment is `C u' = dir (I - D)`, and the draw moves with the rail by
/// its own conductance `g = dD/du`, so off the edge it is `C off' = dir (I -
/// w) - g off` (`w` the draw at the edge): off relaxes with `tau = C / g`
/// toward `dir (I - w) / g`, and reaches the edge at `span` for `w - I = dir
/// g off / (exp(g span / C) - 1)`. With `g` going to 0 that is the line
/// `dir C off / span`, the aim before the rail's conductance was counted,
/// also taken for `g <= 0` (no relaxation to lean on). Cold 2575 tpl 2: a
/// 555 at its 1.8 V floor on 11.9 nF with DIS on, `g` = 0.0575 S, `tau` =
/// 0.2 us; the line read as a current at the supply put the segment's
/// equilibrium at `off (1 - C / (g span))`, 0.92 of its W0 off, and it went
/// back past W0 every 2.7 us, past `SLIDE_FOLLOW_MAX`.
pub(crate) fn segment_aim(off: f64, span: f64, c: f64, g: f64, dir: f64) -> f64 {
    let r = g * span / c;
    if !(g > 0.0 && r.is_finite()) {
        return dir * c * off / span;
    }
    dir * g * off / r.exp_m1()
}

/// A probe's hold: the verdict fixed at `on`.
#[inline]
pub(crate) fn probe(on: bool) -> PowerHold {
    PowerHold { kind: HoldKind::Probe(on), release: None }
}

/// The hold-episode guard (rulings R51, R54 (a)): the short holds an IC
/// may take in one sliding episode before the instant declines (`-4`,
/// classic's fallback). An IC that slides along its power edge (powered,
/// its draw takes its supply out of its window; held off, the supply comes
/// back) is held, freed and held again at the pace of its own edge, femto-
/// to microseconds a hold, the run crawling or frozen; classic's H8 holds
/// such a chip one host step at a time, the core declines it.
pub(crate) const EPISODE_MAX: u32 = 50;

/// One IC's sliding episode (ruling R54 (a)): a hold counts toward it, at
/// its release, only if it lasted less than one host step (the `advance`
/// call it ends in) and its release was the chip's own, not one a foreign
/// driver change caused; either of those ends the episode instead. A hold
/// as long as a host step is classic's H8 cadence, and a release that
/// answers a waveform edge, another device's output or a switch answers
/// the circuit, not the chip's own draw (the evidence ruling R49 (d)'s
/// latch test uses), so a supply fed from a pulse that holds the chip at
/// every low phase starts afresh at each high one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Episode {
    /// The short holds counted so far.
    pub(crate) count: u32,
    /// When the hold in place was taken.
    held_at: f64,
    held: bool,
    /// The hold in place was taken by a stepped partition's seed or step
    /// (`stepped::hold_around`): `retry_stepped_holds` retries it.
    pub(crate) stepped: bool,
    /// While the IC slides (ruling R92): its residual's slope in its duty
    /// at the last solve, the next solve's first secant step (0: none yet).
    pub(crate) slope: f64,
    /// While the IC slides: when its duty was last solved (`-inf`: never),
    /// the span its segment is aimed back over (`slide_residual`).
    pub(crate) solved_at: f64,
    /// While the IC slides on a capacitor's segment: the end of the span
    /// its last solve aimed back over (`slide_span`; `-inf`: none), and
    /// the span its solve aims over, the rail's own settling
    /// (`rail_settling`; 0: none yet).
    pub(crate) aim_until: f64,
    /// While the IC slides on a capacitor's segment: the offset from its
    /// edge its duty aims the supply at, so the band's re-solves centre the
    /// supply's mean on the edge (`centred_aim`; 0 at entry).
    pub(crate) aim_off: f64,
    /// While the IC slides on a capacitor's segment: its supply's offset
    /// from its edge as its duty was last solved, where the drift that
    /// `centred_aim` measures starts.
    pub(crate) solved_off: f64,
    pub(crate) span: f64,
    /// While the IC slides off a capacitor's segment: the supply levels,
    /// lower then upper, past which one end of its duty's bracket stops
    /// bracketing, Filippov's exit (`slide_exits`; infinite: none).
    pub(crate) exits: (f64, f64),
    /// While the IC slides on a capacitor's segment: the same exit as draw
    /// thresholds, lower then upper, on its rail's draw at its edge, `dir`
    /// signed (`slide_exits`, `slide_draw_watches`; infinite: none).
    pub(crate) draw_exits: (f64, f64),
    /// Its slide could not be followed (`keep_sliding`'s fallback): held as
    /// before ruling R92 until the circuit changes, which drops every hold
    /// and episode.
    pub(crate) unfollowed: bool,
    /// The chatter the episode shows (ruling R95 (b)): the supply the last
    /// held solve left, when that hold was released and the supply then
    /// (`-inf`: no release yet). The next hold reads one whole cycle off
    /// them: held, then powered.
    s_held: f64,
    released_at: f64,
    s_released: f64,
    /// The slide was entered from that chatter (ruling R95 (b)): its duty is
    /// corrected across instants by the zero-drift condition at the edge
    /// (`keep_sliding`), not solved within one.
    pub(crate) chatter: bool,
    /// While a chatter slide runs: the bracket on its zero-drift duty, as
    /// (duty, drift rate) pairs: `held_side` a duty whose supply drifted as
    /// the chatter's held phase did, `powered_side` one that drifted as its
    /// powered phase did (at first the two phases themselves, at 0 and 1,
    /// `chatter_held` and `chatter_powered`: Filippov's drift is the duty's
    /// blend of them); `side` the end the last correction moved by
    /// interpolation; and the instant and supply its drift is measured from
    /// (the last correction).
    held_side: (f64, f64),
    powered_side: (f64, f64),
    chatter_held: (f64, f64),
    chatter_powered: (f64, f64),
    side: i8,
    pub(crate) drift_from: (f64, f64),
    /// While a chatter slide runs (read by `rail_power_edges`): its supply has crossed back over the
    /// edge since the last correction (which took it at W0's band), or its
    /// drift's measure started within the band.
    pub(crate) inside: bool,
}

impl Default for Episode {
    fn default() -> Episode {
        Episode {
            count: 0,
            held_at: 0.0,
            held: false,
            stepped: false,
            slope: 0.0,
            solved_at: 0.0,
            aim_until: f64::NEG_INFINITY,
            aim_off: 0.0,
            solved_off: 0.0,
            span: 0.0,
            exits: (f64::NEG_INFINITY, f64::INFINITY),
            draw_exits: (f64::NEG_INFINITY, f64::INFINITY),
            unfollowed: false,
            s_held: 0.0,
            released_at: f64::NEG_INFINITY,
            s_released: 0.0,
            chatter: false,
            held_side: (0.0, 0.0),
            powered_side: (1.0, 0.0),
            chatter_held: (0.0, 0.0),
            chatter_powered: (1.0, 0.0),
            side: 0,
            drift_from: (0.0, 0.0),
            inside: false,
        }
    }
}

impl Episode {
    /// After a hold taken at `now`; one already in place (the procedure
    /// re-running over a chip it holds, a retry moving its level) is the
    /// same hold.
    pub(crate) fn held(self, now: f64) -> Episode {
        if self.held {
            return self;
        }
        Episode { held_at: now, held: true, ..self }
    }

    /// Sets the chatter's measure as the hold procedure records it.
    #[cfg(test)]
    pub(crate) fn set_chatter_for_test(&mut self, s_held: f64, released_at: f64, s_released: f64) {
        (self.s_held, self.released_at, self.s_released) = (s_held, released_at, s_released);
    }

    /// After the release at `now`, in a host step `host_step` long, with
    /// `foreign` true when a foreign driver change in the instant caused it.
    pub(crate) fn released(self, now: f64, host_step: f64, foreign: bool) -> Episode {
        if !self.held {
            return self;
        }
        let short = now - self.held_at < host_step;
        Episode { count: if short && !foreign { self.count + 1 } else { 0 }, held_at: self.held_at, held: false, solved_at: f64::NEG_INFINITY, unfollowed: self.unfollowed, s_held: self.s_held, released_at: now, ..Episode::default() }
    }

    /// The duty to enter a slide at, from the chatter (ruling R95 (b)), at
    /// a hold taken at `now` with the supply at `s_now` powered: once the
    /// episode has counted two short self-released holds, the last one's
    /// cycle (held from `held_at` to `released_at`, then powered to `now`)
    /// gives the powered fraction, the seed, and the supply's rate in each
    /// phase, held then powered. `None`
    /// without a whole cycle, or where the two rates do not point at each
    /// other (no edge between them attracts).
    pub(crate) fn chatter_seed(self, now: f64, s_now: f64) -> Option<(f64, f64, f64)> {
        if self.count < 2 || self.held || !(self.held_at < self.released_at && self.released_at < now) {
            return None;
        }
        let (t_off, t_on) = (self.released_at - self.held_at, now - self.released_at);
        let r_off = (self.s_released - self.s_held) / t_off;
        let r_on = (s_now - self.s_released) / t_on;
        (r_off * r_on < 0.0 && (r_on - r_off).is_finite()).then(|| (t_on / (t_on + t_off), r_off, r_on))
    }

    /// A hold withdrawn at the instant it was taken (a landing's chip that
    /// carries itself with the others held, freed at once): no hold at all,
    /// so its release counts nothing.
    pub(crate) fn withdrawn(self) -> Episode {
        Episode { held: false, ..self }
    }

    /// `Err` with the decline's message past [`EPISODE_MAX`] short holds.
    pub(crate) fn check(self, device: usize) -> Result<(), String> {
        if self.count > EPISODE_MAX {
            Err(format!("power hold: IC (device {device}) held {} times in one sliding episode at its power edge, each shorter than a host step (W1c point 1a, rulings R51, R54)", self.count))
        } else {
            Ok(())
        }
    }
}

/// The foreign driver changes of the open instant (ruling R54 (a)), as the
/// latch evidence of ruling R49 (d) reads them: a driver commit, by the
/// device that owns it (`u32::MAX` for none), or a waveform source's edge.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) enum Changes {
    #[default]
    None,
    /// Commits by this one device only.
    One(u32),
    /// A source edge, or commits by more than one device.
    Many,
}

impl Changes {
    pub(crate) fn note(self, owner: u32) -> Changes {
        match self {
            Changes::None => Changes::One(owner),
            Changes::One(o) if o == owner => self,
            _ => Changes::Many,
        }
    }

    /// Whether a change other than IC `device`'s own was made.
    pub(crate) fn foreign_to(self, device: usize) -> bool {
        match self {
            Changes::None => false,
            Changes::One(o) => o as usize != device,
            Changes::Many => true,
        }
    }
}

/// Each IC's verdict flips across a solve's iterations, by position in the
/// owner's list.
#[derive(Debug, Clone, Default)]
pub(crate) struct Flips {
    last: Vec<Option<bool>>,
    count: Vec<u32>,
}

impl Flips {
    pub(crate) fn new(n: usize) -> Flips {
        Flips { last: vec![None; n], count: vec![0; n] }
    }

    /// Notes IC `k`'s verdict at the latest iteration; true when it flipped.
    pub(crate) fn note(&mut self, k: usize, verdict: bool) -> bool {
        let flipped = self.last[k].is_some_and(|v| v != verdict);
        if flipped {
            self.count[k] += 1;
        }
        self.last[k] = Some(verdict);
        flipped
    }

    /// Every flip counted so far.
    pub(crate) fn total(&self) -> u32 {
        self.count.iter().sum()
    }

    /// The last verdict noted for IC `k`.
    pub(crate) fn last(&self, k: usize) -> Option<bool> {
        self.last[k]
    }

    /// The ICs that flipped at least twice (classic's rule), by position.
    pub(crate) fn alternating(&self) -> Vec<usize> {
        self.count.iter().enumerate().filter(|&(_, &c)| c >= 2).map(|(k, _)| k).collect()
    }
}

/// The edge a landing's error names (`rails::cc_entry_rail`), when `err` is
/// one.
fn landing_edge(err: &str) -> Option<f64> {
    err.strip_prefix(rails::EDGE_LANDING)?.trim_start().strip_prefix("at ")?.trim().parse().ok()
}

/// What a rail retry puts back when it frees nothing ([`PartitionedSolver::retry_on_rail`]):
/// the published arrays, the dynamic snapshot and the hub's running sums.
pub(super) struct Checkpoint {
    published: (Vec<f64>, Vec<f64>),
    dynamic: Box<dyn std::any::Any>,
    sums: Option<(f64, f64)>,
}

/// How the rail loop's passes ended.
pub(super) enum Passes {
    /// The rail and its partitions settled; true when a pass solved the
    /// rail (something it sees changed: an event re-solve).
    Settled(bool),
    /// They did not, and these rail ICs (device indices) alternated: the
    /// procedure runs, and if it cannot, the instant fails with the message.
    Alternating(Vec<usize>, String),
}

impl PartitionedSolver {
    /// Steps 1-4 over the rail loop, for rail ICs `a` (device indices) whose
    /// verdict alternated; `err` is the loop's own failure, which every
    /// failing step returns. The other rail ICs keep their reading through
    /// the probes and their hold after them.
    pub(super) fn hold_on_rail(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, a: &[usize], err: String) -> Result<(), String> {
        let rail_ics: Vec<usize> = self.hub.as_ref().expect("a hub").rail_ics.iter().map(|&pos| self.ics[pos as usize].0).collect();
        let before: Vec<PowerHold> = rail_ics.iter().map(|&d| self.power_hold(d)).collect();
        // R74's landing (`rail_loop`) puts the state back if its powered
        // probe cannot settle (below).
        let start = std::mem::take(&mut self.landing_hold).then(|| Checkpoint { published: self.snapshot(), dynamic: self.dynamic_snapshot(), sums: self.hub.as_ref().map(|h| (h.state.a_run, h.state.g_run)) });
        // Step 1: the alternating ICs held powered, the rest at their reading
        // (one already held off stays so: a probe is no release).
        let mut seed = Vec::new();
        for (k, &d) in rail_ics.iter().enumerate() {
            if before[k].kind == HoldKind::Off && !a.contains(&d) {
                continue;
            }
            let on = a.contains(&d) || self.ic_powered(d);
            seed.extend(self.set_power_hold(d, probe(on)).into_iter().map(|p| p.0 as usize));
        }
        // R74's landing: a powered probe whose passes cannot settle (the
        // chips held powered past the limit, the rail loop alternating CV at
        // the setpoint and CC under it, cold 1986 tpl 0's 74LS00 after a
        // switch toggle) holds no operating point with them powered either:
        // they starve, read under the edge, from the state before the probe.
        let probe_failed = match (self.rail_settle(now, drivers, readers, seed, &err), start) {
            (Ok(()), _) => false,
            (Err(_), Some(start)) => {
                self.restore(start.published);
                self.dynamic_restore(start.dynamic);
                if let (Some(hub), Some((a, g))) = (self.hub.as_mut(), start.sums) {
                    (hub.state.a_run, hub.state.g_run) = (a, g);
                }
                true
            }
            (Err(e), None) => return Err(e),
        };
        let draw_on = self.segment_draw(now);
        // Ruling R74, a landing's own rule: held powered, the chips settle a
        // PSU in CV past its limit, which no current-limited supply holds
        // (the CC point lies at or below the landing's edge, where the walk
        // read the draw within the limit, and the chips are off there; the
        // probe cannot reach it because the rail's curve keeps their hub
        // partition draw linearised at powered: `rails::cc_entry_rail`). So
        // they starve themselves, their supply read two reached widths
        // under that edge, the CC point's side of it.
        let landing = landing_edge(&err).filter(|_| probe_failed || self.cv_past_limit(now));
        // Step 2: those reading unpowered there starve themselves; on a CC
        // rail capacitor's segment, so do they all when, powered, they turn
        // the segment down (ruling R44 (a): across instants the capacitor
        // holds the rail, and what their draw changes is its slope).
        let falling = draw_on.is_some_and(|draw| self.hub.as_ref().and_then(|h| h.psu_setpoint()).is_some_and(|(_, i_limit)| draw > i_limit));
        let mut starving: Vec<(usize, f64)> = a
            .iter()
            .filter_map(|&d| {
                let ic = &self.ics[self.ic_position[d] as usize].1;
                (falling || !digital::power::powered(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node])).then(|| (d, supply(ic, &self.voltages)))
            })
            .collect();
        let landed = starving.is_empty() && landing.is_some();
        if let (true, Some(edge)) = (starving.is_empty(), landing) {
            let rail = self.hub.as_ref().expect("a hub").node;
            let (v, _) = self.hub.as_ref().expect("a hub").psu_setpoint().expect("a PSU");
            let u_dn = edge - crate::devices::sources::direction(v) * 2.0 * rails::REACHED_REL * edge.abs().max(1.0);
            starving = a
                .iter()
                .map(|&d| {
                    let ic = &self.ics[self.ic_position[d] as usize].1;
                    let at = |node: usize| if node == rail { u_dn } else { self.voltages[node] };
                    (d, at(ic.vcc.node) - at(ic.gnd.node))
                })
                .collect();
        }
        if starving.is_empty() {
            // Every alternating IC carries itself in the probe. When every
            // rail IC's free verdict there is the one it was probed at, that
            // state is consistent and is the instant's solution: the free
            // passes alternated only on the way to it (a rail landed at the
            // power edge, re-solved a drift width under it: G2bv's bc_ic 97).
            // The holds go back as they were (checked below, ruling R45 (a))
            // and the rail's events are scheduled again with the power
            // windows the probe set aside.
            let consistent = rail_ics.iter().all(|&d| {
                let ic = &self.ics[self.ic_position[d] as usize].1;
                let free = digital::power::powered(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node]);
                self.power_hold(d).kind == HoldKind::Off || self.power_hold(d).kind == HoldKind::Probe(free)
            });
            if !consistent {
                return Err(err);
            }
            // Accepted only if that state passes the ordinary convergence
            // check (ruling R45 (a)): the partitions that read these ICs are
            // solved again from it with the holds as they were, and each
            // must settle within Newton's tolerance, as any converged solve
            // does. Anything less is rejected as before.
            let mut seed = Vec::new();
            for (k, &d) in rail_ics.iter().enumerate() {
                seed.extend(self.set_power_hold(d, before[k]).into_iter().map(|p| p.0 as usize));
            }
            for k in 0..rail_ics.len() {
                if a.contains(&rail_ics[k]) {
                    seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
                }
            }
            #[cfg(test)]
            let _fault = FAIL_PROBE_RESIDUAL.with(|c| c.get()).then(|| crate::solve::newton::hooks::force_unconverged(u32::MAX));
            if self.rail_settle(now, drivers, readers, seed, &err).is_err() {
                return Err(err);
            }
            readers.extend(a.iter().map(|&d| DeviceId(d as u32)));
            self.schedule_rail_event(now);
            return Ok(());
        }
        // Step 3: probed unpowered, for the drop.
        let mut seed = Vec::new();
        for &(d, _) in &starving {
            seed.extend(self.set_power_hold(d, probe(false)).into_iter().map(|p| p.0 as usize));
        }
        self.rail_settle(now, drivers, readers, seed, &err)?;
        // On a CC rail capacitor's segment the capacitors hold the rail through
        // both probes, so the drop is 0 there (esr aside): what the chips' draw
        // changes is the segment's slope. Their release then also waits for
        // the draw threshold (ruling R44 (a)), with the draw being powered adds.
        let added = draw_on.zip(self.segment_draw(now)).map(|(on, off)| on - off);
        {
            let state = &mut self.hub.as_mut().expect("a hub").state;
            state.held_draw.retain(|&(d, _)| !starving.iter().any(|&(s, _)| s == d));
            if let Some(added) = added {
                state.held_draw.extend(starving.iter().map(|&(d, _)| (d, added)));
            }
        }
        // A landing's chip that its own window turns off in the unpowered
        // probe is consistent free there (cold 360 tpl 3: a gate whose
        // ground pin a clock edge lifts, its supply 5.95 V over its 5.775 V
        // ceiling): it is freed, not held, as exact mode, solving the
        // partition at the instant, finds it.
        if landed {
            starving.retain(|&(d, _)| {
                let ic = &self.ics[self.ic_position[d] as usize].1;
                digital::power::powered(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node])
            });
        }
        // Ruling R92 (R53's sliding hold): one starving chip whose edge
        // attracts (held unpowered its supply comes back inside its window,
        // or on a CC rail capacitor's segment the segment rises; powered it
        // leaves, or the segment falls) has no hold that fits: it slides
        // along the edge, powered at the duty that keeps it there.
        if let [(d, v_on)] = starving[..] {
            let ic = &self.ics[self.ic_position[d] as usize].1;
            // A chip whose own output starves it only once it has committed
            // (a 555 in reset: its OUT sinks once it has turned on, after its
            // delay) shows nothing at the instant's powered probe; its edge
            // shows as a chatter instead, each hold its own: from its second,
            // the slide's own probes, which see its committed outputs, decide
            // (cold 757 tpl 0: on a 1.1 mF rail held just under its 1.8 V
            // floor, both probes there, freed as the rail rose past it,
            // starved once its OUT had sunk 82.8 mA past the 71.76 mA limit).
            let chattering = self.episode(d).count >= 1;
            let attracting = chattering
                || match added {
                    Some(_) => falling && self.segment_draw(now).zip(self.hub.as_ref().and_then(|h| h.psu_setpoint())).is_some_and(|(off, (_, i_limit))| off < i_limit),
                    None => !digital::power::powered(ic, v_on, 0.0) && digital::power::powered(ic, supply(ic, &self.voltages), 0.0),
                };
            let self_fed = self.self_fed[self.ic_position[d] as usize];
            if attracting && !self_fed && sliding_allowed() && !self.episode(d).unfollowed && self.slide_on_rail(now, drivers, readers, d, &err)? {
                let episode = self.episode(d).held(now);
                self.set_episode(d, episode)?;
                self.hub.as_mut().expect("a hub").state.held_draw.retain(|e| e.0 != d);
                return Ok(());
            }
        }
        // Ruling R95 (b): a chip whose edge shows only as a chatter (its
        // draw reaching the supply through a capacitor that carries it
        // within the instant, so neither end of the slide's own solve moves
        // it: cold 757 tpl 0's 555, its OUT sinking from behind a 0.64 ohm
        // fuse and 4.94 mF) slides at the duty the chatter shows, from the
        // second short hold it released itself on: seeded at the powered
        // fraction of the last cycle, then corrected across instants until
        // the supply stops drifting at the edge (`keep_sliding`).
        if let [(d, v_on)] = starving[..] {
            // Only a chatter at the edge: a supply held past W0 from it is
            // no slide's (cold 61 tpl 5's 74LS00 chattering 1.7 V over its
            // ceiling in its first microsecond, the rail's inrush).
            let near = (v_on - self.slide_edge_at(d, v_on)).abs() <= slide_drift(self.slide_edge_at(d, v_on));
            if let Some((x, r_off, r_on)) = self.episode(d).chatter_seed(now, v_on).filter(|_| near && sliding_allowed() && !self.episode(d).unfollowed && !self.self_fed[self.ic_position[d] as usize]) {
                if self.slide_from_chatter(now, drivers, readers, d, x, (r_off, r_on), &err)? {
                    return Ok(());
                }
            }
        }
        // Step 4: the held solve, each starving IC's episode counted first.
        for &(d, _) in &starving {
            let episode = self.episode(d).held(now);
            self.set_episode(d, episode)?;
        }
        let mut seed = Vec::new();
        for (k, &d) in rail_ics.iter().enumerate() {
            let hold = match starving.iter().find(|&&(s, _)| s == d) {
                Some(&(_, v_on)) => {
                    let ic = &self.ics[self.ic_position[d] as usize].1;
                    readers.push(DeviceId(d as u32));
                    held_off(ic, v_on, supply(ic, &self.voltages), now)
                }
                None => before[k],
            };
            // One held off as the procedure began and held off again is the
            // same hold, its release level refreshed: written uncounted (stat
            // 25 counts holds taken), as `retry_on_rail` puts a hold back.
            // Counted, cold 552 tpl 2's gate, probed again at the next flip
            // back once HG1's count is kept across a call that takes nothing,
            // read as held twice within one instant.
            let parts = if before[k].kind == HoldKind::Off && hold.kind == HoldKind::Off { self.write_power_hold(d, hold) } else { self.set_power_hold(d, hold) };
            seed.extend(parts.into_iter().map(|p| p.0 as usize));
        }
        self.rail_settle(now, drivers, readers, seed, &err)?;
        // Where the held solve left each supply: the chatter's measure of
        // its held phase (`Episode::chatter_seed`).
        for &(d, _) in &starving {
            let mut episode = self.episode(d);
            episode.s_held = supply(&self.ics[self.ic_position[d] as usize].1, &self.voltages);
            self.set_episode(d, episode)?;
        }
        Ok(())
    }

    /// Ruling R95 (b): rail IC `d` enters a slide at duty `x`, the powered
    /// fraction its chatter showed, `rates` its supply's drift rates held
    /// and powered. `false`, the state put back, when the passes cannot
    /// settle at that duty.
    #[allow(clippy::too_many_arguments)]
    fn slide_from_chatter(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, d: usize, x: f64, rates: (f64, f64), err: &str) -> Result<bool, String> {
        let cp = self.checkpoint();
        let k = self.hub.as_ref().expect("a hub").rail_ics.iter().position(|&pos| self.ics[pos as usize].0 == d).expect("a rail IC");
        let mut seed: Vec<usize> = self.set_power_hold(d, PowerHold::duty(x)).into_iter().map(|p| p.0 as usize).collect();
        seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
        let mut probe_readers = Vec::new();
        if self.rail_settle(now, drivers, &mut probe_readers, seed, err).is_err() {
            self.back_to(cp);
            return Ok(false);
        }
        readers.extend(probe_readers);
        let mut episode = self.episode(d).held(now);
        episode.chatter = true;
        (episode.chatter_held, episode.chatter_powered) = ((0.0, rates.0), (1.0, rates.1));
        (episode.held_side, episode.powered_side, episode.side) = (episode.chatter_held, episode.chatter_powered, 0);
        episode.drift_from = (now, supply(&self.ics[self.ic_position[d] as usize].1, &self.voltages));
        episode.inside = (episode.drift_from.1 - self.slide_edge(d)).abs() < slide_drift(self.slide_edge(d));
        episode.solved_at = now;
        self.set_episode(d, episode)?;
        self.hub.as_mut().expect("a hub").state.held_draw.retain(|e| e.0 != d);
        readers.push(DeviceId(d as u32));
        self.schedule_rail_event(now);
        Ok(true)
    }

    /// Ruling R95 (b): a chatter slide's duty corrected at `now`, its
    /// supply `off` past its edge. The drift rate since the last correction,
    /// measured, puts the duty on one side of its zero-drift duty, whose
    /// bracket it narrows; the duty is then the bracket's interpolation to
    /// the rate that brings the supply back to the edge over that span
    /// (Filippov's drift, linear in the duty), or its midpoint when the same
    /// end moved at the last correction too. A measured rate a bracket end
    /// contradicts (the board itself changed: cold 757 tpl 0's PTC fuse
    /// tripping) puts that end back at the chatter's own phase. Measured
    /// rates, not a fixed slope: 22 tpl 2's chatter shows its powered phase
    /// only as a sub-microsecond transient (-1300 V/s), where the slide
    /// drifts at a few volts a second; a secant on two corrections' duties
    /// read the board's own changes between them as the duty's. `false` when
    /// the chip, already fully powered (off), still drifts as held
    /// (powered): no duty keeps the supply at the edge.
    fn correct_chatter_slide(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, d: usize, off: f64) -> Result<bool, String> {
        let episode = self.episode(d);
        let (t0, s0) = episode.drift_from;
        let span = now - t0;
        let s = supply(&self.ics[self.ic_position[d] as usize].1, &self.voltages);
        let x = self.power_hold(d).kind.weight();
        let drift = (s - s0) / span;
        let aim = -off / span;
        // Which side of the bracket: the held phase's direction or not.
        let held_dir = episode.chatter_held.1.signum();
        let (mut a, mut b, mut side) = (episode.held_side, episode.powered_side, episode.side);
        let repeated = if drift * held_dir > 0.0 {
            if x >= 1.0 {
                return Ok(false);
            }
            if x >= b.0 {
                b = episode.chatter_powered;
            }
            let repeated = side == 1;
            (a, side) = ((x, drift), 1);
            repeated
        } else {
            if x <= 0.0 && drift != 0.0 {
                return Ok(false);
            }
            if x <= a.0 {
                a = episode.chatter_held;
            }
            let repeated = side == -1;
            (b, side) = ((x, drift), -1);
            repeated
        };
        // The same end moved twice running: the other end's rate is no
        // guide (22 tpl 2's powered phase, its whole fall a single esr
        // step over 0.1 us, -65,000 V/s against the slide's few V/s), so
        // the bracket is halved, as Dekker's safeguard does.
        let x_new = if repeated {
            side = 0;
            0.5 * (a.0 + b.0)
        } else if (aim - a.1) * held_dir >= 0.0 {
            a.0
        } else if (aim - b.1) * held_dir <= 0.0 {
            b.0
        } else {
            a.0 + (aim - a.1) * (b.0 - a.0) / (b.1 - a.1)
        };
        // The measure restarts here, its level the stepped partitions' edges
        // are cut at from the re-solve on (`rail_power_edges`); the supply
        // the re-solve leaves (the capacitors' esr steps with the draw)
        // starts it.
        let mut episode = self.episode(d);
        episode.inside = false;
        episode.drift_from = (now, s);
        (episode.held_side, episode.powered_side, episode.side) = (a, b, side);
        self.set_episode(d, episode)?;
        let k = self.hub.as_ref().expect("a hub").rail_ics.iter().position(|&pos| self.ics[pos as usize].0 == d).expect("a rail IC");
        let mut seed: Vec<usize> = self.set_power_hold(d, PowerHold::duty(x_new)).into_iter().map(|p| p.0 as usize).collect();
        seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
        self.rail_settle(now, drivers, readers, seed, &rails::loop_unsettled())?;
        let mut episode = self.episode(d);
        episode.drift_from = (now, supply(&self.ics[self.ic_position[d] as usize].1, &self.voltages));
        episode.solved_at = now;
        self.set_episode(d, episode)?;
        readers.push(DeviceId(d as u32));
        self.schedule_rail_event(now);
        Ok(true)
    }

    /// Ruling R51 (b), after rail passes that solved (an event re-solve) or
    /// at the horizon clamp: once per host step every rail IC held off, at
    /// any other re-solve one whose supply passed its release level, is
    /// probed powered, the rest as they are, as classic's H8 retries a held
    /// chip at its next host step. One that
    /// carries itself there (its free verdict powered), or whose supply
    /// reads unpowered both probed powered and held off (its own draw no
    /// longer decides), is free again; the rest go back to their holds, and
    /// the rail settles with that; one still starving past its release
    /// level (the `Rail` event that brought the retry) takes a further level
    /// ([`refreshed`]). A release that alternates again within the instant
    /// was none: its draw still decides, and it stays held as it was. A chip held on a CC rail capacitor's
    /// segment is retried only once ruling R44 (a)'s draw threshold passes
    /// (`Self::r44_draw_fits`): within an instant the capacitors carry it
    /// whatever its draw, and what its draw changes is the segment's slope.
    pub(super) fn retry_on_rail(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>) -> Result<(), String> {
        if self.holds_active == 0 {
            return Ok(());
        }
        #[cfg(test)]
        if RetrySeam::get() == RetrySeam::NoRetry {
            return Ok(());
        }
        if self.retry_armed {
            self.reenter_slides(now, drivers, readers)?;
        }
        let held: Vec<(usize, usize, PowerHold, f64)> = {
            let hub = self.hub.as_ref().expect("a hub");
            hub.rail_ics
                .iter()
                .enumerate()
                .filter_map(|(k, &pos)| {
                    let (d, ic) = &self.ics[pos as usize];
                    let hold = self.power_hold(*d);
                    // Not at the instant it was held: the next retry comes
                    // at a later one, as classic's at its next host step (the
                    // rail loop's own passes are no host steps; retried in
                    // them, a chip held at one pass and freed at the next
                    // cycled within the instant).
                    let later = hold.release.is_some_and(|r| r.since < now);
                    // Once per host step every one is retried, otherwise only
                    // one past its release level: a probe at each re-solve
                    // made cold 34 tpl 0's held 555 79,000 failed retries,
                    // 20 times its run's cost.
                    let s_off = supply(ic, &self.voltages);
                    let due = self.retry_armed || hold.release.is_some_and(|r| passed(r, s_off));
                    // A slide given up in this host step (`unfollowed`) is
                    // retried at its release level as any hold (its host
                    // step's retry is `reenter_slides`'): kept from it, the
                    // chip stayed off with its rail climbing 0.5 to 1.8 V
                    // past its floor for the rest of the host step (R53V,
                    // cold 766 tpl 0 under the trapezoid at 50 ms stops).
                    let passed_release = hold.release.is_some_and(|r| passed(r, s_off));
                    let retried = !self.episode(*d).unfollowed || passed_release;
                    (hold.kind == HoldKind::Off && later && due && retried && self.r44_draw_fits(now, *d) != Some(false)).then(|| (k, *d, hold, s_off))
                })
                .collect()
        };
        if held.is_empty() {
            return Ok(());
        }
        self.take_retry();
        // The probe only asks: a retry that frees nothing puts every piece of
        // state back as it was, bit for bit, rather than solving its way
        // back (a stepped partition re-seeded twice at each of its steps
        // restarted its step each time: bc_ic 178 at 20 ns steps).
        let checkpoint = Checkpoint { published: self.snapshot(), dynamic: self.dynamic_snapshot(), sums: self.hub.as_ref().map(|h| (h.state.a_run, h.state.g_run)) };
        let err = rails::loop_unsettled();
        let mut seed = Vec::new();
        for &(k, d, _, _) in &held {
            seed.extend(self.set_power_hold(d, probe(true)).into_iter().map(|p| p.0 as usize));
            seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
        }
        // A probe that settles a PSU in CV past its limit carries nothing
        // (ruling R74, as `hold_on_rail`'s landing rule reads it).
        let carried = self.rail_settle(now, drivers, readers, seed, &err).is_ok() && !self.cv_past_limit(now);
        // Each one's verdict and its hold after the retry.
        let verdicts: Vec<(bool, PowerHold)> = held
            .iter()
            .map(|&(_, d, hold, s_off)| {
                let ic = &self.ics[self.ic_position[d] as usize].1;
                let s_on = supply(ic, &self.voltages);
                let low = |s: f64| !digital::power::powered(ic, s, 0.0);
                let free = carried && (!low(s_on) || low(s_off) && low(s_on));
                // Still starving past its release level (the `Rail` event
                // that brought the retry): held again with the fresh drop
                // (D1 (a)), so the level, the rail's trigger, moves past
                // where it starves.
                let after = match hold.release {
                    Some(release) if carried && !free && passed(release, s_off) => refreshed(ic, release, s_on, s_off, now),
                    _ => hold,
                };
                (free, after)
            })
            .collect();
        #[cfg(test)]
        let verdicts: Vec<(bool, PowerHold)> = if RetrySeam::get() == RetrySeam::AsksOnly { held.iter().map(|&(_, _, hold, _)| (false, hold)).collect() } else { verdicts };
        if verdicts.iter().all(|&(free, _)| !free) {
            self.keep_held(now, checkpoint, &held, &verdicts);
            // Probed together, one chip's overload fails the probe for all
            // (REFV's probe T1b: an innocent 74LS00 driving 1 k, held beside
            // 471 tpl 2's culprit at the same edge, stayed off for good):
            // each is probed on its own as well.
            if held.len() > 1 {
                self.retry_each_on_rail(now, drivers, readers, &held)?;
            }
            return Ok(());
        }
        let counts = self.hold_counts;
        let mut seed = Vec::new();
        for (&(k, d, _, _), &(free, after)) in held.iter().zip(&verdicts) {
            // Back to its hold first, uncounted (stat 25 counts holds taken);
            // freeing it from there counts a release (stat 26).
            let mut parts = self.write_power_hold(d, after);
            if free {
                self.release_episode(d, now)?;
                readers.push(DeviceId(d as u32));
                self.hub.as_mut().expect("a hub").state.held_draw.retain(|e| e.0 != d);
                parts.extend(self.set_power_hold(d, PowerHold::FREE));
            }
            seed.extend(parts.into_iter().map(|p| p.0 as usize));
            seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
        }
        seed.sort_unstable();
        seed.dedup();
        match self.hold_passes(now, drivers, readers, seed)? {
            Passes::Settled(_) => Ok(()),
            // Freed, and at once alternating again: its draw still decides,
            // so the retry asked and found no release, and it stays held as
            // it was. Freed only because its supply read low both ways, FXH's
            // cold 830 tpl 3 (its PSU CC with the gate held, CV without), and
            // carrying itself in the probe but turning a CC rail capacitor's
            // segment down once free, cold 696 tpl 2 and 677 tpl 3 (ruling
            // R44 (a)), were released and held again at each retry, the holds
            // chaining one sliding episode a host step to the guard's `-4`.
            Passes::Alternating(a, _) if held.iter().zip(&verdicts).any(|(h, v)| v.0 && a.contains(&h.1)) => {
                self.hold_counts = counts;
                let kept: Vec<(bool, PowerHold)> = verdicts.iter().map(|&(_, after)| (false, after)).collect();
                self.keep_held(now, checkpoint, &held, &kept);
                Ok(())
            }
            Passes::Alternating(a, err) => self.hold_on_rail(now, drivers, readers, &a, err),
        }
    }

    /// Ruling R92 with R95's `SLIDE_FOLLOW_MAX`: a slide given up in a host
    /// step (`keep_sliding`'s fallback, `Episode::unfollowed`) is held for
    /// that host step only. The next host step's retry frees each such chip
    /// and runs the hold procedure again, which slides it where its edge
    /// attracts; held again instead (the drift still past following, or no
    /// edge that attracts), the solver goes back bit for bit, still given
    /// up, to be tried at the next host step: one extra solve a host step.
    /// Held until the circuit changed (piece 2), cold 2575 tpl 2's 555 sat
    /// unpowered with its rail in CV for 0.63 s after a source's 1.1 ms
    /// ramp had outrun its slide (switch 21 tying the source to OUT through
    /// 105.7 ohm: 114 A/s of draw against the rail's 0.04 S).
    fn reenter_slides(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>) -> Result<(), String> {
        // Not in the host step that gave it up (its re-solves still past
        // the cap, which the host step's end resets): retried there, it fell
        // back again at once, each hold a short one, to the episode guard.
        if self.slide_solves > slide_follow_cap(self.host_step).0 {
            return Ok(());
        }
        let given_up: Vec<(usize, usize)> = {
            let hub = self.hub.as_ref().expect("a hub");
            hub.rail_ics
                .iter()
                .enumerate()
                .map(|(k, &pos)| (k, self.ics[pos as usize].0))
                .filter(|&(_, d)| {
                    let hold = self.power_hold(d);
                    hold.kind == HoldKind::Off && hold.release.is_some_and(|r| r.since < now) && self.episode(d).unfollowed
                })
                .collect()
        };
        if !given_up.is_empty() {
            self.take_retry();
        }
        for (k, d) in given_up {
            #[cfg(test)]
            REENTRIES.with(|c| c.set(c.get() + 1));
            let checkpoint = self.checkpoint();
            let counts = self.hold_counts;
            let mut episode = self.episode(d);
            episode.unfollowed = false;
            self.set_episode(d, episode)?;
            self.release_episode(d, now)?;
            let mut moved = Vec::new();
            let mut seed: Vec<usize> = self.set_power_hold(d, PowerHold::FREE).into_iter().map(|p| p.0 as usize).collect();
            seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
            let kept = match self.hold_passes(now, drivers, &mut moved, seed) {
                Ok(Passes::Settled(_)) => true,
                Ok(Passes::Alternating(a, err)) => self.hold_on_rail(now, drivers, &mut moved, &a, err).is_ok() && self.power_hold(d).kind != HoldKind::Off,
                Err(_) => false,
            };
            if kept {
                self.hub.as_mut().expect("a hub").state.held_draw.retain(|e| e.0 != d);
                readers.extend(moved);
                readers.push(DeviceId(d as u32));
            } else {
                self.back_to(checkpoint);
                self.hold_counts = counts;
            }
        }
        Ok(())
    }

    /// Ruling R92 (R53, Q1): rail IC `d` slides along its power edge. Its
    /// duty `x` is solved, by the Illinois method on the rail loop's settled
    /// passes, so that its residual (`slide_residual`) is zero: on a CC rail
    /// capacitor's segment the draw meets the limit (the segment flat), else
    /// its supply sits at the edge. Both ends are probed first: `x = 0` must
    /// leave the residual negative (unpowered, the supply comes back) and
    /// `x = 1` positive (powered, it leaves), else there is no slide and
    /// `false` (the state is then whatever the last probe left; the caller
    /// re-solves with the hold it takes). `true` with the duty in the table
    /// and the passes settled at it.
    pub(super) fn slide_on_rail(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, d: usize, err: &str) -> Result<bool, String> {
        self.slide_from(now, drivers, readers, d, err, None)
    }

    /// [`Self::slide_on_rail`], from the duty `from` the passes settled at
    /// when there is one: the bracket is searched outward from it, so that
    /// keeping a slide probes its ends only when its duty has moved that far.
    fn slide_from(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, d: usize, err: &str, from: Option<f64>) -> Result<bool, String> {
        let off = supply(&self.ics[self.ic_position[d] as usize].1, &self.voltages) - self.slide_edge(d);
        // The rail's settling, read once as the slide enters and kept while
        // it lasts: read again at each re-solve, on cold 2575 tpl 2's source
        // ramp it came out 1.6 us at the entry and 4.8 ms 2.5 us later, and
        // the long one's trial put the residual under the limit at every
        // duty, so the slide left and entered again each 2.5 us to the
        // episode guard.
        if self.segment_draw(now).is_some() && !(self.episode(d).span > 0.0) {
            let mut episode = self.episode(d);
            episode.span = self.rail_settling(now, d);
            self.set_episode(d, episode)?;
        }
        let span = self.slide_span(now, d);
        let mut slid = self.slide_solve(now, drivers, readers, d, err, from)?;
        if !slid && self.segment_draw(now).is_some() {
            // No duty aims the segment back that fast, yet one holds it flat:
            // the duty at the end that turns it back fastest, the chip off
            // under the edge (or fully on over it), the unpowered (powered)
            // side's own motion back to the edge, where it is solved flat
            // (the watch on the edge itself). Exited only where no duty holds
            // it flat either (cold 766 tpl 0: re-solved 10 ns apart, the aim
            // over that span past what the limit can lift).
            self.slide_flat = true;
            let flat = self.slide_solve(now, drivers, readers, d, err, from);
            self.slide_flat = false;
            if flat? {
                let x = if off < 0.0 { 0.0 } else { 1.0 };
                let k = self.hub.as_ref().expect("a hub").rail_ics.iter().position(|&pos| self.ics[pos as usize].0 == d).expect("a rail IC");
                let mut seed: Vec<usize> = self.set_power_hold(d, PowerHold::duty(x)).into_iter().map(|p| p.0 as usize).collect();
                seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
                self.rail_settle(now, drivers, readers, seed, err)?;
                slid = true;
            }
        }
        if slid {
            let mut episode = self.episode(d);
            episode.solved_at = now;
            episode.solved_off = supply(&self.ics[self.ic_position[d] as usize].1, &self.voltages) - self.slide_edge(d);
            episode.aim_until = now + span;
            self.set_episode(d, episode)?;
        }
        Ok(slid)
    }

    /// [`Self::slide_from`]'s solve. Each probe of a duty starts from the
    /// state it was asked in: the rail's CC Newton lands on the root nearest
    /// where it starts, so a probe from where the last one left it gave the
    /// same duty another rail (cold 471 tpl 2: 4.277 V from the edge,
    /// -0.55 V after a probe one double further on had gone there). A probe
    /// whose passes fail answers "no slide", the state as it was asked.
    fn slide_solve(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, d: usize, err: &str, from: Option<f64>) -> Result<bool, String> {
        let mut cp = Some(self.checkpoint());
        let mut probe_readers = Vec::new();
        let mut ends = SlideEnds::default();
        let solved = match self.slide_solve_from(now, drivers, &mut probe_readers, d, err, from, &mut cp, &mut ends) {
            Ok(true) => self.slide_exits(now, drivers, &mut probe_readers, d, err, &mut cp, ends).map(|()| true),
            other => other,
        };
        match solved {
            Ok(slid) => {
                readers.extend(probe_readers);
                Ok(slid)
            }
            Err(_) => {
                self.back_to(cp.expect("a checkpoint"));
                Ok(false)
            }
        }
    }

    /// The solver's state as a retry or a slide's probe puts it back.
    fn checkpoint(&self) -> Checkpoint {
        Checkpoint { published: self.snapshot(), dynamic: self.dynamic_snapshot(), sums: self.hub.as_ref().map(|h| (h.state.a_run, h.state.g_run)) }
    }

    /// Back at `checkpoint`, bit for bit, the hub's running sums included.
    fn back_to(&mut self, checkpoint: Checkpoint) {
        self.restore(checkpoint.published);
        self.dynamic_restore(checkpoint.dynamic);
        if let (Some(hub), Some((a, g))) = (self.hub.as_mut(), checkpoint.sums) {
            (hub.state.a_run, hub.state.g_run) = (a, g);
        }
    }

    /// The residual at duty `x`, solved from the state the slide's solve
    /// was asked in (`cp`, kept for the next probe).
    #[allow(clippy::too_many_arguments)]
    fn slide_probe(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, d: usize, err: &str, cp: &mut Option<Checkpoint>, x: f64) -> Result<f64, String> {
        let k = self.hub.as_ref().expect("a hub").rail_ics.iter().position(|&pos| self.ics[pos as usize].0 == d).expect("a rail IC");
        self.back_to(cp.take().expect("a checkpoint"));
        *cp = Some(self.checkpoint());
        let mut seed: Vec<usize> = self.set_power_hold(d, digital::power::PowerHold::duty(x)).into_iter().map(|p| p.0 as usize).collect();
        seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
        self.rail_settle(now, drivers, readers, seed, err)?;
        Ok(self.slide_residual(now, d))
    }

    /// Filippov's exit (ruling R92). Off a capacitor's segment (class A):
    /// the supply levels past which one end of
    /// the duty's bracket stops bracketing, read from the probes at the ends
    /// themselves, duty 0 and 1, at the instant of the solve. Each end's
    /// residual is its own supply's distance past the edge, outward, so it
    /// falls by the volt as that supply moves inward; as the board moves at
    /// a fixed duty each end's supply moves with the sliding one, so an end
    /// stops bracketing once the supply has moved by its residual (the
    /// powered end's toward the inside, the unpowered end's outward). The
    /// `Rail` event lands two reached widths past either level and the duty
    /// is solved again there (`keep_sliding`), which decides: no bracket
    /// left, and the chip is freed. Not from the bracket's slope: on the
    /// cold 4.06 V cell the residual is far from linear in the duty (the
    /// Illinois bracket's ends gave 0.341 V per unit duty, the powered end
    /// 8e-11 V), and a watch so placed came 0.34 V out, past W0, so the chip
    /// sat at its duty of 2e-11 for 0.54 ms with its supply past its floor.
    /// An end the solve did not probe is probed here and the duty solved
    /// again after; one whose probe fails (or that does not bracket, a slide
    /// kept across a jump) is no exit. Kept as they were when the solve
    /// left the duty as it was. On a segment (class B), the same exit as
    /// draw thresholds (`draw_exits`, below): HG1's CC ramp, left to the W0
    /// re-solves 22 us apart, held its duty at 0.942 5.8 us past the exit,
    /// 1y 7.4 W0 under the fine reference.
    #[allow(clippy::too_many_arguments)]
    fn slide_exits(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, d: usize, err: &str, cp: &mut Option<Checkpoint>, ends: SlideEnds) -> Result<(), String> {
        if ends.evals == 0 {
            return Ok(());
        }
        let mut episode = self.episode(d);
        episode.exits = (f64::NEG_INFINITY, f64::INFINITY);
        episode.draw_exits = (f64::NEG_INFINITY, f64::INFINITY);
        if self.slide_flat {
            return self.set_episode(d, episode);
        }
        let x = self.power_hold(d).kind.weight();
        let mut r = ends.at;
        let mut probed = false;
        for (e, xe) in [(0usize, 0.0), (1, 1.0)] {
            if r[e].is_none() {
                r[e] = self.slide_probe(now, drivers, readers, d, err, cp, xe).ok();
                probed = true;
            }
        }
        if probed {
            self.slide_probe(now, drivers, readers, d, err, cp, x)?;
        }
        if let Some((_, i_limit)) = self.segment_draw(now).and(self.hub.as_ref().and_then(|h| h.psu_setpoint())) {
            // On a capacitor's segment the residual is the draw less the
            // limit and the aim, which the rail capacitor holds the same at
            // every probe (the rail cannot jump): an end's draw is the
            // duty's own plus the two residuals' difference, and the end
            // stops bracketing where that draw, at the edge, crosses the
            // limit (duty 1 no longer turning the segment down, duty 0 no
            // longer letting it rise). A watch on the duty's draw at the edge
            // as the board moves it (`slide_draw_watches`).
            // An end already past its exit there (the aim, not the board,
            // keeping it bracketing: the rail off the edge, aimed back) is
            // left to the W0 re-solves, as a watch already past would fire at
            // once and the re-solve keep it again (cold 766 tpl 0: held 51
            // times by 0.219 s).
            // The watched draw is the duty's own at the edge; the residual
            // also carries the stepped hub partitions' draw over the span in
            // place of their contributions (open item (c)), which the
            // thresholds take off as it stands at the solve.
            let r_x = self.slide_residual(now, d);
            let w_x = self.slide_draw_at_edge(now, d).unwrap_or(f64::NAN);
            let shift = self.slide_shift(now, d);
            if let Some(added) = r[1].map(|r1| r1 - r_x).filter(|&a| a > 0.0 && w_x > i_limit - a - shift) {
                episode.draw_exits.0 = i_limit - added - shift;
            }
            if let Some(added) = r[0].map(|r0| r0 - r_x).filter(|&a| a < 0.0 && w_x < i_limit - a - shift) {
                episode.draw_exits.1 = i_limit - added - shift;
            }
        } else {
            let ic = &self.ics[self.ic_position[d] as usize].1;
            let s = supply(ic, &self.voltages);
            let sigma = if self.slide_edge(d) == digital::levels::power_window(ic).0 { 1.0 } else { -1.0 };
            let levels = [r[0].filter(|&r0| r0 < 0.0), r[1].filter(|&r1| r1 > 0.0)];
            for level in levels.into_iter().flatten().map(|r| s + sigma * r) {
                if level < s {
                    episode.exits.0 = episode.exits.0.max(level);
                } else {
                    episode.exits.1 = episode.exits.1.min(level);
                }
            }
        }
        self.set_episode(d, episode)
    }

    /// A sliding IC `d`'s rail draw at its edge, `dir` signed, while the
    /// rail is on a capacitor's segment: what its `draw_exits` read.
    fn slide_draw_at_edge(&self, now: f64, d: usize) -> Option<f64> {
        self.segment_draw(now)?;
        let (v, _) = self.hub.as_ref().and_then(|h| h.psu_setpoint())?;
        let ic = &self.ics[self.ic_position[d] as usize].1;
        Some(crate::devices::sources::direction(v) * self.rail_draw_at(now, self.voltages[ic.gnd.node] + self.slide_edge(d)))
    }

    /// The draw-threshold watches of the chips sliding on a CC rail
    /// capacitor's segment (`slide_exits`): each one's draw at its edge,
    /// `dir` signed, reaching the levels where an end of its duty's bracket
    /// stops bracketing, its closed-form terms moving as the board moves it,
    /// as ruling R44 (a)'s held chips' are (`held_draw_watches`).
    pub(super) fn slide_draw_watches(&self, now: f64, v: f64, cf_motion: &dyn Fn(f64) -> Vec<rails::Motion>) -> Vec<rails::Watch> {
        let hub = self.hub.as_ref().expect("a hub");
        let dir = crate::devices::sources::direction(v);
        hub.rail_ics
            .iter()
            .map(|&pos| self.ics[pos as usize].0)
            .filter(|&d| matches!(self.power_hold(d).kind, HoldKind::Duty(_)) && !self.episode(d).chatter)
            .filter_map(|d| {
                let (lo, hi) = self.episode(d).draw_exits;
                let thresholds: Vec<f64> = [lo, hi].into_iter().filter(|t| t.is_finite()).collect();
                let value = self.slide_draw_at_edge(now, d)?;
                (!thresholds.is_empty()).then(|| rails::Watch { value, thresholds, cap: None, terms: cf_motion(dir) })
            })
            .collect()
    }

    #[allow(clippy::too_many_arguments)]
    fn slide_solve_from(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, d: usize, err: &str, from: Option<f64>, cp: &mut Option<Checkpoint>, ends: &mut SlideEnds) -> Result<bool, String> {
        let edge = self.slide_edge(d);
        let tol = self.slide_tolerance(now, d);
        let mut eval = |s: &mut Self, x: f64| -> Result<f64, String> {
            let r = s.slide_probe(now, drivers, readers, d, err, cp, x)?;
            ends.evals += 1;
            if x == 0.0 {
                ends.at[0] = Some(r);
            } else if x == 1.0 {
                ends.at[1] = Some(r);
            }
            Ok(r)
        };
        let (mut x0, mut r0, mut x1, mut r1): (f64, f64, f64, f64);
        match from {
            Some(xc) => {
                // The residual at the duty the passes settled at, then steps
                // doubling away from it, against its sign, to a sign change
                // or an end (where no change means no slide).
                let rc = self.slide_residual(now, d);
                if rc.abs() <= tol {
                    return Ok(true);
                }
                let down = rc > 0.0;
                let mut step = 1.0 / 64.0;
                let mut x_prev = xc;
                let mut r_prev = rc;
                // The slope the last solve found (the residual is near
                // linear in the duty): a secant step from it lands at or by
                // the root, the doubling steps after it only when the board
                // moved the slope too.
                let slope = self.episode(d).slope;
                if slope > 0.0 {
                    let xs = (xc - rc / slope).clamp(0.0, 1.0);
                    if xs != xc {
                        let rs = eval(self, xs)?;
                        if rs.abs() <= tol {
                            let mut episode = self.episode(d);
                            episode.slope = (rs - rc) / (xs - xc);
                            self.set_episode(d, episode)?;
                            readers.push(DeviceId(d as u32));
                            self.schedule_rail_event(now);
                            return Ok(true);
                        }
                        if (rs > 0.0) == down {
                            // Short of the root: on from there.
                            (x_prev, r_prev) = (xs, rs);
                            step = (xs - xc).abs().max(1.0 / 64.0);
                        } else {
                            // Past it: the bracket is closed.
                            (x_prev, r_prev, step) = (xs, rs, f64::NAN);
                        }
                    }
                }
                if step.is_nan() {
                    (x0, r0, x1, r1) = if down { (x_prev, r_prev, xc, rc) } else { (xc, rc, x_prev, r_prev) };
                } else { loop {
                    let xt = if down { (x_prev - step).max(0.0) } else { (x_prev + step).min(1.0) };
                    let rt = eval(self, xt)?;
                    if (rt > 0.0) != down || rt.abs() <= tol {
                        if rt.abs() <= tol {
                            readers.push(DeviceId(d as u32));
                            self.schedule_rail_event(now);
                            return Ok(true);
                        }
                        (x0, r0, x1, r1) = if down { (xt, rt, x_prev, r_prev) } else { (x_prev, r_prev, xt, rt) };
                        break;
                    }
                    if xt == 0.0 || xt == 1.0 {
                        return Ok(false);
                    }
                    (x_prev, r_prev) = (xt, rt);
                    step *= 2.0;
                } }
            }
            None => {
                (x0, r0) = (0.0, eval(self, 0.0)?);
                if r0 >= -tol {
                    return Ok(false);
                }
                (x1, r1) = (1.0, eval(self, 1.0)?);
                if r1 <= tol {
                    return Ok(false);
                }
            }
        }
        // The Illinois method: the residual is monotone in the duty, smooth
        // inside the bracket; the end kept twice in a row has its residual
        // halved. The bracket shrinks to the doubles next to the root at
        // worst, so the cap is never what ends a solve.
        let mut x = x1;
        let mut r = r1;
        let mut side = 0i8;
        for _ in 0..200 {
            x = (x0 * r1 - x1 * r0) / (r1 - r0);
            if !(x > x0 && x < x1) {
                x = 0.5 * (x0 + x1);
            }
            if x <= x0 || x >= x1 {
                break;
            }
            r = eval(self, x)?;
            if r.abs() <= tol {
                break;
            }
            if r < 0.0 {
                (x0, r0) = (x, r);
                if side == -1 {
                    r1 *= 0.5;
                }
                side = -1;
            } else {
                (x1, r1) = (x, r);
                if side == 1 {
                    r0 *= 0.5;
                }
                side = 1;
            }
        }
        if r.abs() > tol {
            // The bracket closed between two doubles with no root between:
            // the residual jumps there (cold 471 tpl 2: one duty further on,
            // the CC rail's root leaves the edge for one at -0.55 V, the
            // chip's partition draw still past the limit below its floor).
            // The nearer end is the slide when it holds the supply within W0
            // of the edge (a capacitor's segment, aimed by its residual, is
            // kept by `keep_sliding` in any case); else there is none.
            let (xe, re) = if r0.abs() <= r1.abs() { (x0, r0) } else { (x1, r1) };
            if self.segment_draw(now).is_none() && re.abs() > slide_drift(edge) {
                return Ok(false);
            }
            let _ = re;
            x = xe;
        }
        if self.power_hold(d) != digital::power::PowerHold::duty(x) {
            eval(self, x)?;
        }
        if x1 > x0 && r1 != r0 {
            let mut episode = self.episode(d);
            episode.slope = (r1 - r0) / (x1 - x0);
            self.set_episode(d, episode)?;
        }
        readers.push(DeviceId(d as u32));
        self.schedule_rail_event(now);
        Ok(true)
    }

    /// The residual a sliding rail IC `d`'s duty zeroes (ruling R92), rising
    /// with the duty: on a CC rail capacitor's segment the draw (`dir`
    /// signed) less the limit, less what aims the segment back at the edge
    /// over one host step from where the capacitors carried the supply (zero
    /// on the edge: there the segment is flat); else how far the IC's supply
    /// lies past the window edge nearer it, outward (under its floor, over
    /// its ceiling). The aim is what keeps a slide on a capacitor at its
    /// edge: the capacitors integrate any draw the duty leaves unbalanced
    /// as the board moves, and a flat segment wherever they stood when the
    /// supply had drifted W0 off left it there, solved again at each step
    /// after (cold 175 tpl 2, 1.8019 V on a 1.8 V edge, every 10 ns).
    fn slide_residual(&self, now: f64, d: usize) -> f64 {
        if let (Some(draw), Some((v, i_limit))) = (self.segment_draw(now), self.hub.as_ref().and_then(|h| h.psu_setpoint())) {
            let hub = self.hub.as_ref().expect("a hub");
            let ic = &self.ics[self.ic_position[d] as usize].1;
            // Aimed at the edge shifted by the centring offset (`centred_aim`).
            let off = supply(ic, &self.voltages) - self.slide_edge(d) - self.episode(d).aim_off;
            // Back over the time the supply took to drift off since the last
            // solve (a host step at most): the board's own pace, so the
            // segment reaches the edge before the drift that carried it off
            // carries it past the band again, where it is solved flat
            // (`keep_sliding`). Aimed back over a whole host step, cold 175
            // tpl 2's rail never left the band's edge, a solve every 60 us.
            let span = self.slide_span(now, d);
            if self.slide_flat {
                return draw + self.slide_shift(now, d) - i_limit;
            }
            // The draw at the edge, not at the supply: the rail's own
            // conductance `G = dD/du` (a sliding 555's DIS, x / 10 S) pulls
            // the segment toward where the draw meets the limit, and an aim
            // read at the supply as a current alone left that point W0 off
            // (`segment_aim`). Only near the edge, where the draw's line in
            // the rail holds: a supply far off it (cold 627 tpl 4, its ground
            // pin's step putting it 6.95 V under its floor) is read at the
            // supply, the line aim's, so no duty brackets it and it is
            // freed; read at the edge 6.95 V away a duty of 0.373 balanced
            // a draw the rail never had, and its watches fired a femtosecond
            // apart, 20 million events without progress.
            let dir = crate::devices::sources::direction(v);
            if off.abs() > 2.0 * slide_drift(self.slide_edge(d)) {
                return draw + self.slide_shift(now, d) - i_limit - dir * hub.c_rail * off / span;
            }
            // The draw's line at the supply carried to the edge (the
            // linearised segment's own `w`): read at the edge itself across
            // a kink of the draw (cold 766 tpl 0) it swung the duty between
            // its ends at each W0 crossing.
            let g = self.rail_draw_slope(hub.state.u);
            let mut at_edge = draw - dir * g * off;
            // Open item (c): a stepped hub partition's own capacitors carry
            // the rail over the span, which its contribution (its
            // companions at its own nanosecond step) does not show: its draw
            // there is a backward Euler step over the span with the rail at
            // the edge (`SteppedSolver::trial_rail_draw`), in place of its
            // contribution at the supply. Its pull read as a conductance of
            // the rail's own (2.66 uF) left cold 766 tpl 0's aim a thousand
            // times short of C17's 13.5 mF behind 2.82 ohm and 105 uF, the
            // rail running on past W0 by up to 2.2 W0 between re-solves.
            at_edge += self.slide_shift(now, d);
            return at_edge - i_limit - segment_aim(off, span, hub.c_rail, g, dir);
        }
        let ic = &self.ics[self.ic_position[d] as usize].1;
        let s = supply(ic, &self.voltages);
        let (lo, hi) = digital::levels::power_window(ic);
        if (s - lo).abs() <= (s - hi).abs() {
            lo - s
        } else {
            s - hi
        }
    }

    /// The span a slide on a capacitor's segment aims back over: its
    /// rail's own fastest settling (`Episode::span`, `rail_settling`, read
    /// as the slide enters), or what is left of the span the last solve aimed
    /// over, when longer. A solve again at an instant the slide's own
    /// commits changed, nanoseconds after the last, otherwise aimed the
    /// supply back over those nanoseconds, which no duty does once its pull
    /// on the rail runs through a capacitor (cold 757 tpl 0: 2 mV off, 10 ns
    /// after a solve, the residual at duty 0 224 A).
    fn slide_span(&self, now: f64, d: usize) -> f64 {
        let host = if self.host_step.is_finite() && self.host_step > 0.0 { self.host_step } else { 1.0 };
        let episode = self.episode(d);
        if !(episode.span > 0.0) {
            // No stepped hub partition carries the rail (`rail_settling`
            // none): the span since the last solve, a host step at most.
            let since = now - episode.solved_at;
            return if since > 0.0 { since.min(host) } else { host };
        }
        // Not bounded by the host step: the settling is the rail's own,
        // not the host's cadence. Bounded, cold 757 tpl 0 at 0.1 ms stops
        // aimed its relative-degree-2 rail back over 0.1 ms, which no duty
        // does: no slide at its hold, a chatter seed at duty 1 that left it
        // 2 W0 out, and the chip free from 0.1733 s to 0.8 s (4 s of wall);
        // unbounded it slides from 0.1726 s throughout (0.25 s of wall).
        let span = episode.span;
        let left = episode.aim_until - now;
        if left > span {
            left
        } else {
            span
        }
    }

    /// Open item (c): the span a slide on a capacitor's segment aims over,
    /// the rail's own fastest settling `C_eff / g_eff` at sliding IC `d`'s
    /// edge, from the rail's admittance over two spans, `T1` =
    /// `SLIDE_FOLLOW_SPAN` (10 ms) and a thousandth of it, `T2` = 10 us:
    /// `Y(T) = g + c_rail / T + dI/du` of each
    /// stepped hub partition's trial over `T` (`trial_rail_draw` a W0 apart),
    /// fitted to `g_eff + C_eff / T`. At least a step's floor
    /// (`stepped::H_MIN`: no step resolves a shorter one), and not bounded
    /// by the host step (`slide_span`; bounded at the entry, an
    /// event-stepped run's 30 ps host step held 2575 tpl 2's aim to 30 ps
    /// for the whole episode, its duty banging 0 to 1 every 20 ps). 0 (the
    /// span since the last solve, as without) when no stepped hub partition
    /// carries the rail or the fit is not positive. A duty
    /// promises the rail only over a span on which the fast and the slow
    /// paths from it agree: aimed over a host step (40 ms) on cold 766 tpl 0,
    /// the trial's end-of-span draw balanced the limit while the rail's
    /// direct draw, 175 mA against 113 mA, took the rail 5 mV down within a
    /// microsecond.
    fn rail_settling(&self, now: f64, d: usize) -> f64 {
        let floor = super::stepped::H_MIN;
        let hub = self.hub.as_ref().expect("a hub");
        let ic = &self.ics[self.ic_position[d] as usize].1;
        let edge = self.slide_edge(d);
        let off = supply(ic, &self.voltages) - edge;
        let u = hub.state.u;
        let u_edge = if ic.vcc.node == hub.node { u - off } else { u + off };
        let delta = slide_drift(edge);
        let g = self.rail_draw_slope(u);
        let parts: Vec<usize> = self.stepped.hub_partitions().into_iter().filter(|&p| hub.contrib.get(p).is_some_and(|c| c.active && c.b.is_none())).collect();
        if parts.is_empty() {
            return 0.0;
        }
        let admittance = |span: f64| -> Option<f64> {
            let mut y = g + hub.c_rail / span;
            for &p in &parts {
                let at = self.stepped.trial_rail_draw(p, now, span, u_edge, &self.driver_snapshot, &self.live_params, &self.voltages)?;
                let past = self.stepped.trial_rail_draw(p, now, span, u_edge + delta, &self.driver_snapshot, &self.live_params, &self.voltages)?;
                y += (past - at) / delta;
            }
            Some(y)
        };
        // Spans fixed in simulated time, not the host's: read over a host
        // step the settling moved with the host's cadence, and a run split
        // by budget yields left the run unsplit.
        let (t1, t2) = (SLIDE_FOLLOW_SPAN, 1e-3 * SLIDE_FOLLOW_SPAN);
        let (Some(y1), Some(y2)) = (admittance(t1), admittance(t2)) else { return 0.0 };
        let c_eff = (y2 - y1) / (1.0 / t2 - 1.0 / t1);
        let g_eff = y1 - c_eff / t1;
        if !(c_eff > 0.0 && g_eff > 0.0) {
            return 0.0;
        }
        (c_eff / g_eff).max(floor)
    }

    /// Open item (c): sliding IC `d`'s aim's shift, `dir` signed, of the
    /// stepped hub partitions' draw at the edge from their contributions to
    /// their draw over the span (`stepped_draw_over`); 0 with none.
    fn slide_shift(&self, now: f64, d: usize) -> f64 {
        let hub = self.hub.as_ref().expect("a hub");
        let Some((v, _)) = hub.psu_setpoint() else { return 0.0 };
        let off = supply(&self.ics[self.ic_position[d] as usize].1, &self.voltages) - self.slide_edge(d);
        let dir = crate::devices::sources::direction(v);
        self.stepped_draw_over(now, self.slide_span(now, d), hub.state.u, off, d).map_or(0.0, |shift| dir * shift)
    }

    /// Open item (c): what the stepped hub partitions draw at the end of a
    /// backward Euler step over `span` with the rail at sliding IC `d`'s
    /// edge (its supply `off` past it at the rail `u`), less what their
    /// contributions draw at `u` now. `None` when there is none, or a
    /// trial does not solve (the aim then reads the contributions, as
    /// before).
    fn stepped_draw_over(&self, now: f64, span: f64, u: f64, off: f64, d: usize) -> Option<f64> {
        let hub = self.hub.as_ref().expect("a hub");
        let ic = &self.ics[self.ic_position[d] as usize].1;
        // The rail at the edge: under the supply by `off` on the VCC side,
        // over it on the ground side.
        let u_edge = if ic.vcc.node == hub.node { u - off } else { u + off };
        let parts: Vec<usize> = self.stepped.hub_partitions().into_iter().filter(|&p| hub.contrib.get(p).is_some_and(|c| c.active && c.b.is_none())).collect();
        if parts.is_empty() {
            return None;
        }
        let mut shift = 0.0;
        for p in parts {
            let c = hub.contrib[p];
            #[cfg(test)]
            if TRIALS_TWICE.with(|t| t.get()) {
                let _ = self.stepped.trial_rail_draw(p, now, span, u_edge, &self.driver_snapshot, &self.live_params, &self.voltages);
            }
            let over = self.stepped.trial_rail_draw(p, now, span, u_edge, &self.driver_snapshot, &self.live_params, &self.voltages)?;
            shift += over - (c.i + c.g * (u - c.u_p));
        }
        Some(shift)
    }

    /// The window edge a sliding IC `d`'s supply sits at (the nearer one).
    pub(super) fn slide_edge(&self, d: usize) -> f64 {
        let ic = &self.ics[self.ic_position[d] as usize].1;
        self.slide_edge_at(d, supply(ic, &self.voltages))
    }

    /// IC `d`'s window edge nearer supply `s`.
    fn slide_edge_at(&self, d: usize, s: f64) -> f64 {
        let ic = &self.ics[self.ic_position[d] as usize].1;
        let (lo, hi) = digital::levels::power_window(ic);
        if (s - lo).abs() <= (s - hi).abs() {
            lo
        } else {
            hi
        }
    }

    /// The residual a slide is solved to: the reached width of the edge or
    /// of the limit (Rails point 4), as the regime rules land within.
    fn slide_tolerance(&self, now: f64, d: usize) -> f64 {
        if let (Some(_), Some((_, i_limit))) = (self.segment_draw(now), self.hub.as_ref().and_then(|h| h.psu_setpoint())) {
            return rails::REACHED_REL * i_limit.max(1.0);
        }
        let ic = &self.ics[self.ic_position[d] as usize].1;
        let s = supply(ic, &self.voltages);
        let (lo, hi) = digital::levels::power_window(ic);
        let edge = if (s - lo).abs() <= (s - hi).abs() { lo } else { hi };
        rails::REACHED_REL * edge.abs().max(1.0)
    }

    /// Ruling R92's user warning (R53, Dev's text; `slow::warnings`,
    /// `rail_ic_power_chatter`, classic R89's code): every chip still
    /// sliding along its power floor when a rail loop ends is latched on
    /// channel 1, once while it stays latched (classic's `_latchWarning`),
    /// with its floor and, when its rail is a PSU in CC, the PSU's limit
    /// (`NaN` otherwise: a cell, or a PSU in CV through a series part).
    /// Read where the loop has settled, so a slide a probe or a retry tried
    /// and gave back latches nothing, and through the slow state's journal,
    /// so a failed instant takes its latch with it. A slide at the ceiling
    /// latches nothing yet: its message is still to be worded.
    pub(super) fn latch_slide_warnings(&mut self, now: f64) {
        if self.holds_active == 0 {
            return;
        }
        let Some(hub) = self.hub.as_ref() else { return };
        let limit = match (hub.state.reg, hub.psu_setpoint()) {
            (crate::devices::sources::Regime::Cc, Some((_, i_limit))) => i_limit,
            _ => f64::NAN,
        };
        let mut fresh = Vec::new();
        for &pos in &hub.rail_ics {
            let (d, ref ic) = self.ics[pos as usize];
            if !matches!(self.power_hold(d).kind, HoldKind::Duty(_)) {
                continue;
            }
            let (floor, _) = digital::levels::power_window(ic);
            if self.slide_edge(d) != floor {
                continue;
            }
            let rec = crate::slow::LatchRecord { device: d as u32, code: crate::slow::warnings::WarningKind::RailIcPowerChatter as u32, pin: -1, since: now, value: floor, limit, aux: 0.0 };
            if !self.slow.warnings().iter().any(|w| w.key() == rec.key()) {
                fresh.push(rec);
            }
        }
        for rec in fresh {
            self.sw().latch_warning(rec);
        }
    }

    /// The orchestrator's ruling on R53-P8's Q1 (R103 (b)): a slide's goal
    /// is its supply's time-mean at the edge, not its value at each
    /// re-solve. Where the rail's own capacitor carries the segment (no
    /// settling span, `rail_settling`), a solve that leaves the segment flat
    /// zeroes the supply's velocity, and the draw then moves smoothly, so
    /// the supply leaves its solve level `a` quadratically and reaches the
    /// band W0 on the side it drifted to: its mean over that drift is `a +
    /// (W0 - a) / 3`, zero for `a = -W0 / 2`. A re-solve at the band whose
    /// supply drifted at least half of W0 since the last solve aims it there,
    /// half of W0 back past the edge (HG1's CC ramp: the RC's draw decays,
    /// the rail climbs from each flat solve, and aimed at the edge its mean
    /// sat 0.36 W0, 1.57 mV, up: 15.7 uA more through the 100 ohm, taken
    /// from the LED's 5.06 mA, the duty 0.0031 low and 1y, the duty times
    /// 3.599 V, 11 mV, 4.3 of its W0). A rail that a stepped partition
    /// carries over a settling span keeps moving after a flat solve
    /// (relative degree 2: so aimed, cold 757 tpl 0 ran 2.4 W0 past its aim
    /// and lost its slide): aimed at the edge, as is a re-solve a driver
    /// change or an exit takes.
    fn centred_aim(&mut self, now: f64, d: usize, off: f64) -> Result<(), String> {
        let mut episode = self.episode(d);
        let band = slide_drift(self.slide_edge(d));
        let delta = off - episode.solved_off;
        episode.aim_off = if !(episode.span > 0.0) && now > episode.solved_at && delta.abs() >= 0.5 * band {
            -0.5 * band * delta.signum()
        } else {
            0.0
        };
        self.set_episode(d, episode)
    }

    /// Ruling R92: after passes that settled, each rail IC sliding is kept
    /// on its edge: its duty solved again once its supply has drifted from
    /// the edge by more than W0 ([`slide_drift`]), as what it sees moves (a
    /// `Rail` event watches those two levels, `rail_power_edges`). Solved
    /// again at each re-solve that moved it at all, a duty that changed by
    /// ulps restarted the stepped partitions it feeds at their floor step,
    /// whose next 10 ns step moved it again (cold 175 tpl 2: a re-solve every
    /// 10 ns). One that no duty keeps there leaves the
    /// slide free (its draw carries it inside the window, or even unpowered
    /// its supply stays out), its episode released, and the passes run again
    /// with it free (alternating, the hold procedure takes it again).
    pub(super) fn keep_sliding(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>) -> Result<(), String> {
        if self.holds_active == 0 {
            return Ok(());
        }
        let sliding: Vec<usize> = self
            .hub
            .as_ref()
            .expect("a hub")
            .rail_ics
            .iter()
            .map(|&pos| self.ics[pos as usize].0)
            .filter(|&d| matches!(self.power_hold(d).kind, HoldKind::Duty(_)))
            .collect();
        for d in sliding {
            let ic = &self.ics[self.ic_position[d] as usize].1;
            let edge = self.slide_edge(d);
            let off = supply(ic, &self.voltages) - edge;
            // An instant that changed a driver or a source (a commit, the
            // sliding 555's own toggle among them) stepped the draw the duty
            // balances: solved again there, and in between only once the
            // supply has drifted past W0 (cold 175 tpl 2's 555, which runs on
            // as it slides: left to drift after each toggle it took a solve
            // every 30 us).
            let changed = self.changes != Changes::None;
            // A chatter slide (ruling R95 (b)) is corrected once its supply
            // has drifted out past W0 after crossing back over its edge since
            // the last correction, or a further W0 out from where that left
            // it (a `Rail` watch on that level): a rate is measured over a
            // drift as wide as W0, not over the settling at the band's level
            // a correction leaves (cold 757 tpl 0: 1.9 V/s read over 83 ns,
            // its duty taken to 1). Driver changes do not restart it: a chip
            // that toggles on its own as it slides (22 tpl 2's 555, its OUT
            // on its TRIG) drifts at its duty's average across its toggles.
            if self.episode(d).chatter {
                let mut episode = self.episode(d);
                let band = slide_drift(edge);
                let off0 = episode.drift_from.1 - edge;
                let inside = episode.inside || off * off0 <= 0.0;
                if off.abs() <= band || !(inside || off.abs() >= off0.abs() + band) {
                    if inside != episode.inside {
                        episode.inside = inside;
                        self.set_episode(d, episode)?;
                    }
                    continue;
                }
            }
            // Back at the edge on a capacitor's segment aimed there (the
            // watch on the edge itself): solved flat.
            let eta = rails::REACHED_REL * edge.abs().max(1.0);
            let chatter = self.episode(d).chatter;
            let aim_off = self.episode(d).aim_off;
            let at_edge = !chatter && (off - aim_off).abs() <= 2.0 * eta && self.segment_draw(now).zip(self.hub.as_ref().and_then(|h| h.psu_setpoint())).is_some_and(|(draw, (_, i_limit))| (draw + self.slide_shift(now, d) - i_limit).abs() > self.slide_tolerance(now, d));
            // Past a level where an end of its duty's bracket stops
            // bracketing (off a capacitor's segment, `slide_exits`, its
            // `Rail` watch): solved again, which decides.
            // On a capacitor's segment, past a draw where an end stops
            // bracketing (`slide_draw_watches`).
            let (exits, draw_exits) = (self.episode(d).exits, self.episode(d).draw_exits);
            let past_end = !chatter && (off + edge <= exits.0 || off + edge >= exits.1 || self.slide_draw_at_edge(now, d).is_some_and(|w| w <= draw_exits.0 || w >= draw_exits.1));
            if !changed && !at_edge && !past_end && off.abs() <= slide_drift(edge) {
                continue;
            }
            // On a capacitor's segment already aimed back at the edge: on its
            // way (the next solve comes inside the band, or at the far one).
            // Heading back both now (the draw against the limit) and over the
            // span (with the stepped hub partitions' draw over it, open item
            // (c)): over the span alone, cold 2575 tpl 2's 11.9 nF rail, solved
            // at -1.9 mV at a pulse edge, ran on to +2.3 mV 0.16 us later, its
            // draw still under the limit, and published 1.07 W0 off at 0.38146
            // s.
            if let (false, false, false, false, Some(draw), Some((_, i_limit))) = (chatter, changed, at_edge, past_end, self.segment_draw(now), self.hub.as_ref().and_then(|h| h.psu_setpoint())) {
                if (off > aim_off) == (draw > i_limit) && (off > aim_off) == (draw + self.slide_shift(now, d) > i_limit) {
                    continue;
                }
            }
            self.slide_solves += 1;
            let (follow_max, solves_max) = slide_follow_cap(self.host_step);
            if self.slide_solves > solves_max {
                return Err(format!("power slide: IC (device {d}) had its duty solved again {} times in one host step, its supply drifting past W0 from its edge each time (ruling R92)", self.slide_solves));
            }
            if self.slide_solves > follow_max {
                // Given up: free, and the passes find it alternating again,
                // where the hold procedure holds it as before the ruling.
                let mut episode = self.episode(d);
                episode.unfollowed = true;
                self.set_episode(d, episode)?;
                self.release_episode(d, now)?;
                readers.push(DeviceId(d as u32));
                let seed: Vec<usize> = self.set_power_hold(d, PowerHold::FREE).into_iter().map(|p| p.0 as usize).collect();
                match self.hold_passes(now, drivers, readers, seed)? {
                    Passes::Settled(_) => {}
                    Passes::Alternating(a, err) => self.hold_on_rail(now, drivers, readers, &a, err)?,
                }
                continue;
            }
            if self.episode(d).chatter {
                if self.correct_chatter_slide(now, drivers, readers, d, off)? {
                    continue;
                }
            } else {
                if !changed && !at_edge && !past_end && self.segment_draw(now).is_some() {
                    self.centred_aim(now, d, off)?;
                }
                let err = rails::loop_unsettled();
                let from = self.power_hold(d).kind.weight();
                if self.slide_from(now, drivers, readers, d, &err, Some(from))? {
                    continue;
                }
            }
            self.release_episode(d, now)?;
            readers.push(DeviceId(d as u32));
            let seed: Vec<usize> = self.set_power_hold(d, PowerHold::FREE).into_iter().map(|p| p.0 as usize).collect();
            match self.hold_passes(now, drivers, readers, seed)? {
                Passes::Settled(_) => {}
                Passes::Alternating(a, err) => self.hold_on_rail(now, drivers, readers, &a, err)?,
            }
        }
        Ok(())
    }

    /// `retry_on_rail`'s probe, one held chip at a time, the rest held: one
    /// that carries itself (its free verdict powered, the PSU not past its
    /// limit in CV) is freed, as a joint retry frees it; any other puts the
    /// solver back bit for bit.
    fn retry_each_on_rail(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, held: &[(usize, usize, PowerHold, f64)]) -> Result<(), String> {
        let err = rails::loop_unsettled();
        for &(k, d, hold, _) in held {
            let checkpoint = Checkpoint { published: self.snapshot(), dynamic: self.dynamic_snapshot(), sums: self.hub.as_ref().map(|h| (h.state.a_run, h.state.g_run)) };
            let counts = self.hold_counts;
            let mut seed: Vec<usize> = self.set_power_hold(d, probe(true)).into_iter().map(|p| p.0 as usize).collect();
            seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
            let mut probe_readers = Vec::new();
            let carried = self.rail_settle(now, drivers, &mut probe_readers, seed, &err).is_ok() && !self.cv_past_limit(now);
            let ic = &self.ics[self.ic_position[d] as usize].1;
            let free = carried && digital::power::powered(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node]);
            if free {
                let mut seed: Vec<usize> = self.write_power_hold(d, hold).into_iter().map(|p| p.0 as usize).collect();
                self.release_episode(d, now)?;
                self.hub.as_mut().expect("a hub").state.held_draw.retain(|e| e.0 != d);
                seed.extend(self.set_power_hold(d, PowerHold::FREE).into_iter().map(|p| p.0 as usize));
                seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
                seed.sort_unstable();
                seed.dedup();
                if let Ok(Passes::Settled(_)) = self.hold_passes(now, drivers, &mut probe_readers, seed) {
                    readers.extend(probe_readers);
                    readers.push(DeviceId(d as u32));
                    continue;
                }
            }
            self.hold_counts = counts;
            self.keep_held(now, checkpoint, &[], &[]);
        }
        Ok(())
    }

    /// A retry that frees nothing: the solver back at `checkpoint`, bit for
    /// bit, and each held IC at its hold, a further level where the retry
    /// moved it (`refreshed`).
    fn keep_held(&mut self, now: f64, checkpoint: Checkpoint, held: &[(usize, usize, PowerHold, f64)], verdicts: &[(bool, PowerHold)]) {
        self.restore(checkpoint.published);
        self.dynamic_restore(checkpoint.dynamic);
        // A whole restore re-sums the hub's running sums (Rails point 2), which
        // differ from the incremental ones by ulps; the probe only asked, so
        // they go back as they were.
        if let (Some(hub), Some((a, g))) = (self.hub.as_mut(), checkpoint.sums) {
            (hub.state.a_run, hub.state.g_run) = (a, g);
        }
        let mut moved = false;
        for (&(_, d, hold, _), &(_, after)) in held.iter().zip(verdicts) {
            if after != hold {
                // Off to Off at a new level: the stamps are the same, and the
                // hold the same one (its episode unchanged).
                self.write_power_hold(d, after);
                moved = true;
            }
        }
        if moved {
            self.schedule_rail_event(now);
        }
    }

    /// IC `d`'s episode after its release at `now` (ruling R54 (a)): counted
    /// when its hold was shorter than this host step and no foreign driver
    /// change in the instant caused it; `Err` past the guard.
    pub(super) fn release_episode(&mut self, d: usize, now: f64) -> Result<(), String> {
        let was = self.episode(d);
        let mut episode = was.released(now, self.host_step, self.changes.foreign_to(d));
        if episode != was {
            episode.s_released = supply(&self.ics[self.ic_position[d] as usize].1, &self.voltages);
            // A slide's end is no held phase: the chatter is measured again
            // from the next hold's release.
            if matches!(self.power_hold(d).kind, HoldKind::Duty(_)) {
                episode.released_at = f64::NEG_INFINITY;
            }
        }
        self.set_episode(d, episode)
    }

    /// The rail loop's passes with `seed` re-solved first, as a probe or the
    /// held solve runs them: anything but settling is `err`.
    fn rail_settle(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, mut seed: Vec<usize>, err: &str) -> Result<(), String> {
        seed.sort_unstable();
        seed.dedup();
        match self.hold_passes(now, drivers, readers, seed)? {
            Passes::Settled(_) => Ok(()),
            Passes::Alternating(..) => Err(err.to_string()),
        }
    }

    /// The rail loop's passes after a hold changed (a probe, a held solve, a
    /// release): the rail itself is solved again whatever partitions `seed`
    /// names. A rail IC's direct draw (a driver between the rail and a fixed
    /// node) reads its hold, but such a chip with every other pin on the
    /// rail or a fixed node dirties no partition, and a pass that finds the
    /// rail clean solves nothing: the probes then judged it at the rail as
    /// it was, and the held solve published a rail its hold no longer
    /// matched (the verifier's DC repro, a 74LS00 sinking 64 mA from a 45 mA
    /// PSU through outputs on the rail itself, left CV at 64 mA; on a clock,
    /// a CC rail of 2.25 V reported 45 mA with 55 mA fed in).
    pub(super) fn hold_passes(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, seed: Vec<usize>) -> Result<Passes, String> {
        self.hub.as_mut().expect("a hub").dirty = true;
        self.rail_passes(now, drivers, readers, seed)
    }

    /// Whether a bench PSU on the rail sits in CV delivering past its limit
    /// in its own direction (by more than the reached width the regime rules
    /// land within).
    fn cv_past_limit(&self, now: f64) -> bool {
        let hub = self.hub.as_ref().expect("a hub");
        let Some((v, i_limit)) = hub.psu_setpoint() else { return false };
        hub.state.reg == crate::devices::sources::Regime::Cv && crate::devices::sources::direction(v) * self.rail_draw_at(now, hub.state.u) > i_limit + rails::REACHED_REL * i_limit.max(1.0)
    }

    /// The rail's draw at its published `u`, `dir` signed, while a PSU with
    /// a rail capacitor is in CC (on a segment); `None` otherwise.
    fn segment_draw(&self, now: f64) -> Option<f64> {
        let hub = self.hub.as_ref().expect("a hub");
        let (v, _) = hub.psu_setpoint()?;
        if hub.state.reg != crate::devices::sources::Regime::Cc || !(hub.c_rail > 0.0) {
            return None;
        }
        Some(crate::devices::sources::direction(v) * self.rail_draw_at(now, hub.state.u))
    }

    /// Ruling R44 (a)'s draw threshold for rail IC `d` held off: on a CC
    /// rail capacitor's segment, with the draw powering adds recorded at its
    /// hold, whether the draw at its release level plus that fits the limit
    /// (the segment would not turn down with it powered); `None` for any
    /// other held chip, which the threshold does not concern.
    fn r44_draw_fits(&self, now: f64, d: usize) -> Option<bool> {
        let hub = self.hub.as_ref().expect("a hub");
        let (v, i_limit) = hub.psu_setpoint().filter(|_| hub.c_rail > 0.0)?;
        let &(_, added) = hub.state.held_draw.iter().find(|e| e.0 == d)?;
        let release = self.power_hold(d).release?;
        let ic = &self.ics[self.ic_position[d] as usize].1;
        // In CV the rail is the setpoint, and a chip whose powered draw
        // there is past the limit enters CC at once (ruling G, which R68
        // gives a PSU with a rail capacitor), the capacitors carrying the
        // rail back down to the edge it starved at: a release passed on the
        // way up is the same draw test at the setpoint (cold 696 tpl 2: a 555
        // held at its 1.8 V floor on 0.16 uF, freed at each crossing of its
        // release level, held 51 times within a host step).
        let at = match hub.state.reg {
            crate::devices::sources::Regime::Cc => self.voltages[ic.gnd.node] + release.level,
            crate::devices::sources::Regime::Cv => v,
        };
        Some(crate::devices::sources::direction(v) * self.rail_draw_at(now, at) + added <= i_limit)
    }

    /// The draw-threshold watches of the chips held on a CC rail capacitor's
    /// segment (ruling R44 (a)): the draw at each one's edge, `dir` signed,
    /// reaching the limit less what it adds, its closed-form terms moving as
    /// the CC exit's draw test reads them (Rails point 4).
    pub(super) fn held_draw_watches(&self, now: f64, v: f64, i_limit: f64, cf_motion: &dyn Fn(f64) -> Vec<rails::Motion>) -> Vec<rails::Watch> {
        let hub = self.hub.as_ref().expect("a hub");
        let dir = crate::devices::sources::direction(v);
        hub.state
            .held_draw
            .iter()
            .filter_map(|&(d, added)| {
                let hold = self.power_hold(d);
                let release = hold.release.filter(|_| hold.kind == HoldKind::Off)?;
                let ic = &self.ics[self.ic_position[d] as usize].1;
                let edge = self.voltages[ic.gnd.node] + release.level;
                Some(rails::Watch { value: dir * self.rail_draw_at(now, edge), thresholds: vec![i_limit - added], cap: None, terms: cf_motion(dir) })
            })
            .collect()
    }

    /// The holds the stepped solver's seeds and steps took (both modes,
    /// `stepped::hold_around`), entered in the table (journaled and counted
    /// as any hold, each one's episode counted as on the rail).
    pub(super) fn take_stepped_holds(&mut self) -> Result<(), String> {
        let requests = self.stepped.take_hold_requests();
        let mut result = Ok(());
        for (d, hold, now) in requests {
            let old = self.power_hold(d);
            let _ = self.set_power_hold(d, hold);
            let mut episode = self.episode(d);
            if old.kind != HoldKind::Off {
                episode = episode.held(now);
            }
            episode.stepped = true;
            result = result.and(self.set_episode(d, episode));
        }
        result
    }

    /// Default mode's retry of the chips a stepped partition's seed or step
    /// holds (`stepped::hold_around`), as the rail's and the static solver's
    /// are retried (ruling R51 (b)): once per host step at the clamp, and at
    /// any commit once the supply passes the hold's release level. The chip
    /// is freed and its stepped partition re-seeded, which holds it again if
    /// its own draw still takes the supply out of its window; each release
    /// of a hold shorter than the host step counts toward ruling R54 (a)'s
    /// guard. Cold 656 tpl 3: a 555 at its window's floor through 115 ohm
    /// slides along the edge (its OUT pair draws 22 mA where the feed gives
    /// 1.7 mA) and declines through that guard; held once and never retried
    /// it stayed unpowered at 2.1 V for the rest of the run.
    pub(super) fn retry_stepped_holds(&mut self, now: f64, clamp: bool, readers: &mut Vec<DeviceId>) -> Result<(), String> {
        use digital::power::{HoldKind, PowerHold};
        if self.exact.is_some() || self.holds_active == 0 {
            return Ok(());
        }
        let stepped_of = |n: usize| self.partition_of_node(n).map(|q| q.0 as usize).filter(|&q| self.partition_kinds[q] == PartitionKind::Stepped);
        let due: Vec<(usize, usize)> = self
            .ics
            .iter()
            .filter_map(|(d, ic)| {
                let q = stepped_of(ic.vcc.node).or_else(|| stepped_of(ic.gnd.node)).filter(|_| self.episode(*d).stepped)?;
                let was = self.power_hold(*d);
                let passed = was.release.is_some_and(|r| r.since < now && hold::passed(r, hold::supply(ic, &self.voltages)));
                (was.kind == HoldKind::Off && (clamp || passed)).then_some((*d, q))
            })
            .collect();
        if due.is_empty() {
            return Ok(());
        }
        let mut parts: Vec<usize> = Vec::new();
        for &(d, q) in &due {
            let _ = self.set_power_hold(d, PowerHold::FREE);
            self.release_episode(d, now)?;
            readers.push(DeviceId(d as u32));
            parts.push(q);
        }
        parts.sort_unstable();
        parts.dedup();
        self.begin_dynamic_writes_list(&parts);
        let pre = Rc::clone(&self.driver_snapshot);
        self.sync_stepped_cc(now);
        let stepped = self.stepped.resolve_dirty(&parts, now, &pre, &pre, &self.live_params, &mut self.voltages, &mut self.currents);
        self.stepped.cc = None;
        readers.extend(stepped?);
        self.end_dynamic_writes();
        self.take_stepped_holds()
    }

    /// The holds and releases the static solver took in its last resolve,
    /// entered in the table (journaled and counted as any hold).
    /// Each one's episode counts as on the rail; past the guard, `Err`.
    #[inline]
    pub(super) fn take_static_holds(&mut self) -> Result<(), String> {
        if !self.static_solver.has_hold_requests() {
            return Ok(());
        }
        let mut result = Ok(());
        for (d, hold, now) in self.static_solver.take_hold_requests() {
            let old = self.power_hold(d);
            let _ = self.set_power_hold(d, hold);
            match (old.kind, hold.kind) {
                (HoldKind::Off, HoldKind::Off) => {}
                (HoldKind::Off, _) => result = result.and(self.release_episode(d, now)),
                (_, HoldKind::Off) => {
                    let episode = self.episode(d).held(now);
                    result = result.and(self.set_episode(d, episode));
                }
                _ => {}
            }
        }
        result
    }
}

// ---- W2a: heap bytes (generated by core/scripts/gen_heap_bytes.py; regenerate, do not edit) ----

#[cfg(test)]
impl crate::snapshot::HeapBytes for RetrySeam {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            RetrySeam::Normal => 0,
            RetrySeam::NoRetry => 0,
            RetrySeam::AsksOnly => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for SlideEnds {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.at, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.evals, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Episode {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.count, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.held_at, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.held, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.stepped, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.slope, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.solved_at, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_until, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.aim_off, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.solved_off, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.span, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.exits, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.draw_exits, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.unfollowed, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s_held, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.released_at, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.s_released, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.chatter, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.held_side, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.powered_side, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.chatter_held, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.chatter_powered, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.side, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.drift_from, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.inside, w); }
        n
    }
}

impl crate::snapshot::HeapBytes for Changes {
    #[allow(unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        match self {
            Changes::None => 0,
            Changes::One(f0) => crate::snapshot::HeapBytes::heap_bytes(f0, w),
            Changes::Many => 0,
        }
    }
}

impl crate::snapshot::HeapBytes for Flips {
    #[allow(unused_mut, unused_variables)]
    fn heap_bytes(&self, w: &mut crate::snapshot::Walk) -> usize {
        let mut n = 0;
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.last, w); }
        { n += crate::snapshot::HeapBytes::heap_bytes(&self.count, w); }
        n
    }
}
