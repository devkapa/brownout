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
//! A probe or the held solve that fails is `-4` as before. The hold ends
//! (ruling D1 (a)) when the supply, the IC still off, passes the window edge
//! inward by that drop plus two reached widths: on the rail a `Rail` event
//! threshold (Rails point 4), on a static partition a check at each re-solve.
//!
//! This module holds the procedure's pieces both owners share and the rail's
//! own steps; the static partition's run inside
//! [`super::static_part::StaticPartitionSolver`], on its mirror of the table,
//! and hand their result to the owner ([`PartitionedSolver::take_static_holds`]).

use super::*;
use crate::digital::power::{HoldKind, PowerHold, Release};
use crate::rails;

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
pub(crate) fn release_after(ic: &digital::IcRecord, v_on: f64, v_off: f64) -> Option<Release> {
    let (lo, hi) = digital::levels::power_window(ic);
    let eta = |edge: f64| rails::REACHED_REL * edge.abs().max(1.0);
    if v_on < lo {
        Some(Release { level: lo + (v_off - v_on).max(0.0) + 2.0 * eta(lo), rising: true })
    } else if v_on > hi {
        Some(Release { level: hi - (v_on - v_off).max(0.0) - 2.0 * eta(hi), rising: false })
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
pub(crate) fn held_off(ic: &digital::IcRecord, v_on: f64, v_off: f64) -> PowerHold {
    let release = release_after(ic, v_on, v_off).unwrap_or_else(|| {
        let (lo, hi) = digital::levels::power_window(ic);
        let eta = |edge: f64| rails::REACHED_REL * edge.abs().max(1.0);
        if v_on - lo <= hi - v_on {
            Release { level: lo + 2.0 * eta(lo), rising: true }
        } else {
            Release { level: hi - 2.0 * eta(hi), rising: false }
        }
    });
    PowerHold { kind: HoldKind::Off, release: Some(release) }
}

/// A probe's hold: the verdict fixed at `on`.
#[inline]
pub(crate) fn probe(on: bool) -> PowerHold {
    PowerHold { kind: HoldKind::Probe(on), release: None }
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
    /// The rail and its partitions settled.
    Settled,
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
            // The holds go back as they were; stamped as before, nothing
            // re-solves, but the rail's events are scheduled again with the
            // power windows the probe set aside.
            let consistent = rail_ics.iter().all(|&d| {
                let ic = &self.ics[self.ic_position[d] as usize].1;
                let free = digital::power::powered(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node]);
                self.power_hold(d).kind == HoldKind::Off || self.power_hold(d).kind == HoldKind::Probe(free)
            });
            if !consistent {
                return Err(err);
            }
            for (k, &d) in rail_ics.iter().enumerate() {
                let _ = self.set_power_hold(d, before[k]);
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
        // Step 4: the held solve.
        let mut seed = Vec::new();
        for (k, &d) in rail_ics.iter().enumerate() {
            let hold = match starving.iter().find(|&&(s, _)| s == d) {
                Some(&(_, v_on)) => {
                    let ic = &self.ics[self.ic_position[d] as usize].1;
                    readers.push(DeviceId(d as u32));
                    held_off(ic, v_on, supply(ic, &self.voltages))
                }
                None => before[k],
            };
            seed.extend(self.set_power_hold(d, hold).into_iter().map(|p| p.0 as usize));
        }
        self.rail_settle(now, drivers, readers, seed, &err)
    }

    /// The rail loop's passes with `seed` re-solved first, as a probe or the
    /// held solve runs them: anything but settling is `err`.
    fn rail_settle(&mut self, now: f64, drivers: &[Driver], readers: &mut Vec<DeviceId>, mut seed: Vec<usize>, err: &str) -> Result<(), String> {
        seed.sort_unstable();
        seed.dedup();
        match self.rail_passes(now, drivers, readers, seed)? {
            Passes::Settled => Ok(()),
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

    /// The held rail ICs whose release is due at the published voltages, by
    /// position in `hub.rail_ics`: the supply passed the release level (Rails
    /// point 4) and, for a chip held on a CC rail capacitor's segment while
    /// the rail is still on one, the draw at its edge plus what it adds is
    /// within the limit (ruling R44 (a)). Empty while nothing is held.
    pub(super) fn rail_releases_due(&self, now: f64) -> Vec<usize> {
        if self.holds_active == 0 {
            return Vec::new();
        }
        let hub = self.hub.as_ref().expect("a hub");
        let on_segment = hub.psu_setpoint().filter(|_| hub.state.reg == crate::devices::sources::Regime::Cc && hub.c_rail > 0.0);
        hub.rail_ics
            .iter()
            .enumerate()
            .filter(|&(_, &pos)| {
                let (d, ic) = &self.ics[pos as usize];
                let hold = self.power_hold(*d);
                let Some(release) = hold.release.filter(|_| hold.kind == HoldKind::Off) else { return false };
                if !passed(release, supply(ic, &self.voltages)) {
                    return false;
                }
                match (on_segment, hub.state.held_draw.iter().find(|e| e.0 == *d)) {
                    (Some((v, i_limit)), Some(&(_, added))) => {
                        let dir = crate::devices::sources::direction(v);
                        let edge = self.voltages[ic.gnd.node] + release.level;
                        dir * self.rail_draw_at(now, edge) + added <= i_limit
                    }
                    _ => true,
                }
            })
            .map(|(k, _)| k)
            .collect()
    }

    /// Frees every held rail IC whose release is due once the rail loop has
    /// settled, returning the partitions to re-solve with its stamps (the
    /// loop goes on; a flip it causes counts), or `None` when none is due.
    pub(super) fn release_settled(&mut self, now: f64, readers: &mut Vec<DeviceId>, flips: &mut Flips) -> Option<Vec<usize>> {
        let due = self.rail_releases_due(now);
        if due.is_empty() {
            return None;
        }
        let mut parts = Vec::new();
        for k in due {
            let (d, on) = {
                let (d, ic) = &self.ics[self.hub.as_ref().unwrap().rail_ics[k] as usize];
                (*d, digital::power::powered(ic, self.voltages[ic.vcc.node], self.voltages[ic.gnd.node]))
            };
            parts.extend(self.set_power_hold(d, PowerHold::FREE).into_iter().map(|p| p.0 as usize));
            parts.extend(&self.hub.as_ref().unwrap().ic_parts[k]);
            readers.push(DeviceId(d as u32));
            flips.note(k, on);
            self.hub.as_mut().unwrap().state.held_draw.retain(|e| e.0 != d);
        }
        self.hub.as_mut().unwrap().dirty = true;
        parts.sort_unstable();
        parts.dedup();
        Some(parts)
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
    #[inline]
    pub(super) fn take_static_holds(&mut self) {
        if !self.static_solver.has_hold_requests() {
            return;
        }
        for (d, hold) in self.static_solver.take_hold_requests() {
            let _ = self.set_power_hold(d, hold);
        }
    }
}
