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
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Episode {
    /// The short holds counted so far.
    pub(crate) count: u32,
    /// When the hold in place was taken.
    held_at: f64,
    held: bool,
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

    /// After the release at `now`, in a host step `host_step` long, with
    /// `foreign` true when a foreign driver change in the instant caused it.
    pub(crate) fn released(self, now: f64, host_step: f64, foreign: bool) -> Episode {
        if !self.held {
            return self;
        }
        let short = now - self.held_at < host_step;
        Episode { count: if short && !foreign { self.count + 1 } else { 0 }, held_at: self.held_at, held: false }
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
        self.rail_settle(now, drivers, readers, seed, &err)?;
        let draw_on = self.segment_draw(now);
        // Step 2: those reading unpowered there starve themselves; on a CC
        // rail capacitor's segment, so do they all when, powered, they turn
        // the segment down (ruling R44 (a): across instants the capacitor
        // holds the rail, and what their draw changes is its slope).
        let falling = draw_on.is_some_and(|draw| self.hub.as_ref().and_then(|h| h.psu_setpoint()).is_some_and(|(_, i_limit)| draw > i_limit));
        let starving: Vec<(usize, f64)> = a
            .iter()
            .filter_map(|&d| {
                let ic = &self.ics[self.ic_position[d] as usize].1;
                (falling || !digital::power::powered(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node])).then(|| (d, supply(ic, &self.voltages)))
            })
            .collect();
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
            seed.extend(self.set_power_hold(d, hold).into_iter().map(|p| p.0 as usize));
        }
        self.rail_settle(now, drivers, readers, seed, &err)
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
                    (hold.kind == HoldKind::Off && later && due && self.r44_draw_fits(now, *d) != Some(false)).then(|| (k, *d, hold, s_off))
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
        let carried = self.rail_settle(now, drivers, readers, seed, &err).is_ok();
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
        let episode = self.episode(d).released(now, self.host_step, self.changes.foreign_to(d));
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
    fn hold_passes(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, seed: Vec<usize>) -> Result<Passes, String> {
        self.hub.as_mut().expect("a hub").dirty = true;
        self.rail_passes(now, drivers, readers, seed)
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
