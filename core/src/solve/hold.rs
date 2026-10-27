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
//! retried at every event re-solve, as classic's H8 retries it at each host
//! step (ruling R51 (b)): probed powered, it is free again once it carries
//! itself. On the rail the release level (ruling D1 (a): the window edge
//! inward by that drop and a margin) is the `Rail` event that brings a retry
//! when nothing else does (Rails point 4). An IC held more than
//! [`EPISODE_MAX`] times in one sliding episode declines `-4` (R51's guard).
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

/// Ruling R51's hold-episode guard: the holds an IC may take in one sliding
/// episode before the instant declines (`-4`, classic's fallback). An IC
/// that slides along its power edge (powered, its draw takes its supply out
/// of its window; held off, the supply comes back) is held, freed and held
/// again at the pace of its own edge: on the cold fuzz's 13 boards that
/// pace ran from femtoseconds to milliseconds and from 119 to 110,000
/// holds, the run crawling or frozen, where G2bv's 9,000 runs and the cold
/// fuzz's legitimate holds chain at most 8 (drafts/f5k-logs). Classic's H8
/// holds such a chip one host step at a time; the core declines it.
pub(crate) const EPISODE_MAX: u32 = 50;

/// An IC's next hold continues its episode when it comes within this many
/// times its last hold's length after that hold's release: the chip got no
/// further from its edge, powered, than it stayed held. A chip that runs
/// powered for longer (a relaxation at the edge, G2bv's fz_ic 487 hub 2:
/// 10 ns held, 150 us powered) starts a new episode at its next hold.
pub(crate) const EPISODE_RATIO: f64 = 100.0;

/// One IC's sliding episode: its holds in a row, each within
/// [`EPISODE_RATIO`] of the last one's length after its release; a retry that
/// still starves past its release level counts as one more.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub(crate) struct Episode {
    pub(crate) count: u32,
    held_at: f64,
    /// `None` while held.
    released_at: Option<f64>,
    held_for: f64,
}

impl Episode {
    /// After a hold taken at `now`.
    pub(crate) fn held(self, now: f64) -> Episode {
        // Held again with no release since (the procedure re-running over a
        // chip it already holds): the same hold.
        if self.count > 0 && self.released_at.is_none() {
            return self;
        }
        let chained = self.count > 0 && self.released_at.is_some_and(|r| now - r <= EPISODE_RATIO * self.held_for);
        Episode { count: if chained { self.count + 1 } else { 1 }, held_at: now, released_at: None, held_for: self.held_for }
    }

    /// After a retry that still starves past the release level.
    pub(crate) fn refreshed(self) -> Episode {
        Episode { count: self.count + 1, ..self }
    }

    /// After the release at `now`.
    pub(crate) fn released(self, now: f64) -> Episode {
        Episode { released_at: Some(now), held_for: now - self.held_at, ..self }
    }

    /// `Err` with the decline's message past [`EPISODE_MAX`] holds.
    pub(crate) fn check(self, device: usize) -> Result<(), String> {
        if self.count > EPISODE_MAX {
            Err(format!("power hold: IC (device {device}) held {} times in one sliding episode at its power edge (W1c point 1a, ruling R51)", self.count))
        } else {
            Ok(())
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

    /// Ruling R51 (b), after rail passes that solved (an event re-solve):
    /// every rail IC held off is probed powered, the rest as they are, as
    /// classic's H8 retries a held chip at its next host step. One that
    /// carries itself there (its free verdict powered), or whose supply
    /// reads unpowered both probed powered and held off (its own draw no
    /// longer decides), is free again; the rest go back to their holds, and
    /// the rail settles with that; one still starving past its release
    /// level (the `Rail` event that brought the retry) takes a further level
    /// ([`refreshed`]). A release that alternates again is held again by
    /// the procedure (a fresh drop). A chip held on a CC rail capacitor's
    /// segment is retried only once ruling R44 (a)'s draw threshold passes
    /// (`Self::r44_draw_fits`): within an instant the capacitors carry it
    /// whatever its draw, and what its draw changes is the segment's slope.
    pub(super) fn retry_on_rail(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>) -> Result<(), String> {
        if self.holds_active == 0 {
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
                    (hold.kind == HoldKind::Off && later && self.r44_draw_fits(now, *d) != Some(false)).then(|| (k, *d, hold, supply(ic, &self.voltages)))
                })
                .collect()
        };
        if held.is_empty() {
            return Ok(());
        }
        // The probe only asks: a retry that frees nothing puts every piece of
        // state back as it was, bit for bit, rather than solving its way
        // back (a stepped partition re-seeded twice at each of its steps
        // restarted its step each time: bc_ic 178 at 20 ns steps).
        let checkpoint = (self.snapshot(), self.dynamic_snapshot());
        let err = rails::loop_unsettled();
        let mut seed = Vec::new();
        for &(k, d, _, _) in &held {
            seed.extend(self.set_power_hold(d, probe(true)).into_iter().map(|p| p.0 as usize));
            seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
        }
        let carried = self.rail_settle(now, drivers, readers, seed, &err).is_ok();
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
        if verdicts.iter().all(|&(free, _)| !free) {
            let (snapshot, dynamic) = checkpoint;
            self.restore(snapshot);
            self.dynamic_restore(dynamic);
            let mut moved = false;
            for (&(_, d, hold, _), &(_, after)) in held.iter().zip(&verdicts) {
                if after != hold {
                    // Off to Off at a new level: the stamps are the same.
                    self.write_power_hold(d, after);
                    let episode = self.episode(d).refreshed();
                    self.set_episode(d, episode)?;
                    moved = true;
                }
            }
            if moved {
                self.schedule_rail_event(now);
            }
            return Ok(());
        }
        let mut seed = Vec::new();
        for (&(k, d, hold, _), &(free, after)) in held.iter().zip(&verdicts) {
            // Back to its hold first, uncounted (stat 25 counts holds taken);
            // freeing it from there counts a release (stat 26).
            let mut parts = self.write_power_hold(d, after);
            let episode = self.episode(d);
            if free {
                self.set_episode(d, episode.released(now))?;
            } else if after != hold {
                self.set_episode(d, episode.refreshed())?;
            }
            if free {
                readers.push(DeviceId(d as u32));
                self.hub.as_mut().expect("a hub").state.held_draw.retain(|e| e.0 != d);
                parts.extend(self.set_power_hold(d, PowerHold::FREE));
            }
            seed.extend(parts.into_iter().map(|p| p.0 as usize));
            seed.extend(&self.hub.as_ref().expect("a hub").ic_parts[k]);
        }
        seed.sort_unstable();
        seed.dedup();
        match self.rail_passes(now, drivers, readers, seed)? {
            Passes::Settled(_) => Ok(()),
            Passes::Alternating(a, err) => self.hold_on_rail(now, drivers, readers, &a, err),
        }
    }

    /// The rail loop's passes with `seed` re-solved first, as a probe or the
    /// held solve runs them: anything but settling is `err`.
    fn rail_settle(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, mut seed: Vec<usize>, err: &str) -> Result<(), String> {
        seed.sort_unstable();
        seed.dedup();
        match self.rail_passes(now, drivers, readers, seed)? {
            Passes::Settled(_) => Ok(()),
            Passes::Alternating(..) => Err(err.to_string()),
        }
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
        let (v, i_limit) = hub.psu_setpoint().filter(|_| hub.state.reg == crate::devices::sources::Regime::Cc && hub.c_rail > 0.0)?;
        let &(_, added) = hub.state.held_draw.iter().find(|e| e.0 == d)?;
        let release = self.power_hold(d).release?;
        let ic = &self.ics[self.ic_position[d] as usize].1;
        let edge = self.voltages[ic.gnd.node] + release.level;
        Some(crate::devices::sources::direction(v) * self.rail_draw_at(now, edge) + added <= i_limit)
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
            let (old, episode) = (self.power_hold(d), self.episode(d));
            let episode = match (old.kind, hold.kind) {
                (HoldKind::Off, HoldKind::Off) => episode.refreshed(),
                (HoldKind::Off, _) => episode.released(now),
                (_, HoldKind::Off) => episode.held(now),
                _ => episode,
            };
            let _ = self.set_power_hold(d, hold);
            if old.kind == HoldKind::Off || hold.kind == HoldKind::Off {
                result = result.and(self.set_episode(d, episode));
            }
        }
        result
    }
}
