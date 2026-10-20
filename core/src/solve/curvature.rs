//! A curved hub partition's re-solve band from its own curvature (Rails
//! point 3). Between rail solves a hub partition publishes the first-order
//! correction `x + s (u - u_p)`, `s` the exact derivative of its solution at
//! the solve (`rail_sensitivity`), so its error is the second-order
//! remainder, at most `sup |f''| du^2 / 2` over the band for each published
//! quantity `f`. The band is the widest `du`, up to `RAIL_RESOLVE_V`, at
//! which that bound stays inside W0's tolerance for every quantity the
//! partition publishes: its node voltages, its resistors' and switches'
//! currents, its junctions' currents and its draw from the rail.
//!
//! Two kinds of stamp curve the response: the junctions, and the drivers of
//! an IC with a supply on the rail, whose conductance follows the supply
//! (`R_out = vs / 4 io_max`, a 555's headroom split). Everything else
//! stamps a conductance, so differentiating `F(x(u), u) = 0` twice gives
//! `x'' = -J^-1 (sum_j e_j f_j''(v_j) v_j'^2 + sum_o e_o h_o)`, `h_o` the
//! drivers' own second derivative at node `o` (`IcGroup`). Seen from
//! junction `j`, the rest of the partition is a network with driving-point
//! resistance `R_j` and open-circuit sensitivity `a_j = dV_oc/du`, so `v_j'
//! = a_j / (1 + R_j g_j)` and `J^-1 e_j` is at most `R_j / (1 + R_j g_j)` at
//! any node (the maximum principle of a passive network); `J^-1 e_o` is at
//! most the driving-point resistance at `o`. With `f'' <= f' / vtn` for the
//! junction law (the exponential, and 0 on its tangent past `vsat`),
//! junction `j` adds at most
//! - `a_j^2 phi(R_j g_j) / vtn` to any node's or junction voltage's
//!   `|x''|`, `phi(y) = y / (1 + y)^3`, whose peak `4/27` at `y = 1/2` is
//!   the knee;
//! - `a_j^2 g_j / (vtn (1 + R_j g_j)^3)` to any branch current's or the
//!   draw's `|I''|`: the share of an injection across the junction that
//!   leaves through the rest of the network is `1 / (1 + R_j g_j)`, and no
//!   branch carries more than all of it;
//!
//! and a driver group `R_o |h_o|` and `|h_o|` (the draw also its own
//! branches' `h`).
//!
//! Each term is bounded over the band, not read at the solve, so the bound
//! holds through the knee from either side. With one junction and no
//! driver, `a` and `R` are constants of the linear rest; drivers move the
//! rest's resistances by at most their conductances' relative change over
//! the band (Rayleigh), and `a` by at most `b` times its slope, read from
//! the drivers' closed forms. With several junctions, `|a_j|` is at most the
//! span of the sensitivity network's terminals (1 without drivers, `Bound::
//! reach`) and `R_j` lies between its values with every other junction off
//! and at its steepest (Rayleigh). The junction's voltage over the band lies
//! in an interval, which bounds `g`; `phi` and `g / (1 + R g)^3` are
//! unimodal, so their suprema over an interval are closed forms.
//!
//! Where the bound cannot be formed, or holds at no band, the band is the
//! narrowest (`NARROWEST_BAND`): the partition solves again at every move
//! of the rail. A partition holding a free IC supply, whose stamps follow a
//! supply node and are no conductance network, takes a perturbation bound
//! instead (`supply_band`, below).

use crate::devices::led::JunctionLaw;
use crate::lu;
use crate::rails;
use crate::solve::{live_params_for, Bnd, DynShape, RailSens};
use std::collections::HashMap;

/// W0's equivalence tolerance (core/README.md, Rails point 3): a voltage
/// within `V_ABS + REL |v|`, a current within `A_ABS + REL |i|`.
pub(crate) const V_ABS: f64 = 1e-4;
pub(crate) const A_ABS: f64 = 1e-9;
pub(crate) const REL: f64 = 1e-3;

/// The band where no wider one is bounded: a smaller change of the rail is
/// not a move (Rails point 3), so this one re-solves at every move.
pub(crate) const NARROWEST_BAND: f64 = rails::RAIL_MOVE_EPS;

/// Bisection steps over `(0, RAIL_RESOLVE_V]`: about 1e-14 V of resolution,
/// far below any band the bound can ask for.
const BAND_STEPS: u32 = 40;

/// The junction law's `f'(v)`, GMIN excluded: the exponential's slope, held
/// at `vsat` on the tangent.
pub(crate) fn slope(law: &JunctionLaw, v: f64) -> f64 {
    law.is * (v.min(law.vsat) / law.vtn).exp() / law.vtn
}

/// `sup y / (1 + y)^3` over `[lo, hi]`, `0 <= lo <= hi` (either may be
/// infinite).
fn phi_sup(lo: f64, hi: f64) -> f64 {
    let phi = |y: f64| if y.is_finite() { y / ((1.0 + y) * (1.0 + y) * (1.0 + y)) } else { 0.0 };
    if hi < 0.5 {
        phi(hi)
    } else if lo > 0.5 {
        phi(lo)
    } else {
        4.0 / 27.0
    }
}

/// `sup g / (1 + r g)^3` over `g` in `[lo, hi]`, `r >= 0`.
fn psi_sup(lo: f64, hi: f64, r: f64) -> f64 {
    if !(r > 0.0) {
        return hi;
    }
    let psi = |g: f64| g / ((1.0 + r * g) * (1.0 + r * g) * (1.0 + r * g));
    let peak = 0.5 / r;
    if hi < peak {
        psi(hi)
    } else if lo > peak {
        psi(lo)
    } else {
        4.0 / (27.0 * r)
    }
}

/// An IC driver branch of a hub partition whose conductance follows the
/// rail: from local node `o` to the rail (`to_rail`) or to a fixed node at
/// `t`, `G(vs)` its `curve` in the supply difference `vs`, which moves as
/// `sigma u` (`1` for a `vcc` on the rail, `-1` for a `gnd`).
#[derive(Debug, Clone, Copy)]
pub(crate) struct RailDriver {
    pub o: usize,
    pub to_rail: bool,
    pub t: f64,
    pub sigma: f64,
    pub vs: f64,
    pub curve: crate::digital::power::SupplyCurve,
    /// The supply differences its IC is powered over (W1c point 1): a kink
    /// outside them is never reached while it stamps.
    pub window: (f64, f64),
}

impl RailDriver {
    /// How far the supply can move before the conductance kinks while the IC
    /// is powered, or infinity.
    fn kink_distance(&self) -> f64 {
        self.curve.kinks.iter().filter(|&&k| k >= self.window.0 && k <= self.window.1).map(|k| (k - self.vs).abs()).fold(f64::INFINITY, f64::min)
    }
}

/// A hub partition's IC stamps that follow the rail other than through a
/// conductance to it (`ic_rail_terms`): its rail drivers, and its floating
/// pulls `(pin, g)` whose target is the rail; and every local node an IC
/// has a pin on, which the band takes for one any IC may stamp.
#[derive(Debug, Clone, Default)]
pub(crate) struct IcRail {
    pub drivers: Vec<RailDriver>,
    pub pulls: Vec<(usize, f64)>,
    pub pinned: Vec<usize>,
    /// Whether any IC stamp in the partition ends on a node other than the
    /// rail: a driver or pull toward a fixed node, of any IC (the draw's
    /// rule, [`DrawRule`]).
    pub sinks: bool,
    /// Whether any IC branch in the partition ends on the rail (a driver
    /// whose conductance does not follow it included).
    pub touches_rail: bool,
}

/// What can carry the draw from the rail (V8's cost finding), from the
/// stamps the solve holds: the partition's devices, fixed at load, and its
/// IC drivers' committed branches, each change of which solves it again.
/// Never from a value: a draw that is merely small takes the general rule.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum DrawRule {
    /// A stamp conducts between the rail and the partition, and the
    /// current has somewhere to go: the draw is bounded as a branch current.
    General,
    /// No stamp touches the rail (a chip whose gnd is the rail, its output
    /// high toward a fixed vcc): the draw is 0 at every rail.
    Zero,
    /// Only IC branches to the rail touch the partition, and nothing else
    /// conducts in it (an output on the rail driving gate inputs): by KCL the
    /// draw is the node shunts' current, `NODE_RSHUNT_G` times the nodes'
    /// sum, and its error is theirs.
    ShuntOnly,
}

impl IcRail {
    /// Whether the partition's response to the rail is affine on the present
    /// pieces (no junction assumed): every group of drivers on one node and
    /// supply has a constant conductance and far-end sum (`IcGroup`'s `C`,
    /// `B` and `c_r` all zero: a 555's discharge transistor, not its OUT pair
    /// with vcc on the rail, whose split follows the supply), and none kinks
    /// while its IC is powered. Its first-order correction is then exact at
    /// any rail.
    pub(crate) fn affine(&self, u: f64) -> bool {
        let mut groups: Vec<(usize, f64, u64, f64, f64, f64)> = Vec::new();
        for d in &self.drivers {
            if d.kink_distance().is_finite() {
                return false;
            }
            let beta = d.curve.piece(d.vs).1;
            let far_b = if d.to_rail { beta * (u - d.sigma * d.vs) } else { beta * d.t };
            match groups.iter_mut().find(|g| g.0 == d.o && g.1 == d.sigma && g.2 == d.vs.to_bits()) {
                Some(g) => {
                    g.3 += beta;
                    g.4 += far_b;
                    if d.to_rail {
                        g.5 += beta;
                    }
                }
                None => groups.push((d.o, d.sigma, d.vs.to_bits(), beta, far_b, if d.to_rail { beta } else { 0.0 })),
            }
        }
        groups.iter().all(|g| g.3 == 0.0 && g.4 == 0.0 && g.5 == 0.0)
    }

    /// The part of `dF/du` at `z` (the partition's unknowns, then the rail)
    /// that the residual matrix's rail column does not hold: a driver
    /// current `G (v_o - v_far)` moves with `G' = sigma dG/dvs` too, and a
    /// pull's `g (v_pin - target)` with its target.
    pub(crate) fn explicit_u(&self, z: &[f64], n: usize) -> Vec<f64> {
        let mut out = vec![0.0; n + 1];
        for d in &self.drivers {
            let dg = d.sigma * d.curve.derivs(d.vs)[1];
            let far = if d.to_rail { z[n] } else { d.t };
            out[d.o] += dg * (z[d.o] - far);
            if d.to_rail {
                out[n] += dg * (far - z[d.o]);
            }
        }
        for &(pin, g) in &self.pulls {
            out[pin] -= g;
        }
        out
    }
}

/// One stamped junction of a hub partition at its solve.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Port {
    pub law: JunctionLaw,
    /// The junction voltage and its rail sensitivity at the solve.
    pub v: f64,
    pub dv: f64,
    /// Bounds over the band on the driving-point resistance of the rest of
    /// the partition seen across the junction, as it sets the junction's
    /// share of its own injection (`phi`'s argument is `R g`).
    pub r_min: f64,
    pub r_max: f64,
    /// A lower bound over the band on that resistance where it sets the
    /// share leaving through the rest (`psi`).
    pub r_psi: f64,
    /// The node bound's factor for a rest whose resistance moves with the
    /// rail (an IC driver's `R_out`): the most it grows over the band.
    pub node_scale: f64,
    /// A bound on `|dV_oc / du|` over the band (`a` below).
    pub alpha: f64,
    /// The only junction: the rest is linear in the partition's voltages,
    /// `R` is its value at the solve and `v` rises no faster than its slope
    /// there.
    pub single: bool,
}

/// What a port contributes over a band of half-width `b`: the voltage and
/// current curvature bounds, the largest slope its current takes and the
/// smallest magnitude it takes.
pub(crate) struct PortBound {
    pub k_v: f64,
    pub k_a: f64,
    pub g_hi: f64,
    pub i_min: f64,
}

impl Port {
    /// A port with a linear rest of resistance `r` at the solve, whose open
    /// port moves `alpha` per volt of rail.
    pub(crate) fn single(law: JunctionLaw, v: f64, dv: f64, r: f64, alpha: f64) -> Port {
        Port { law, v, dv, r_min: r, r_max: r, r_psi: r, node_scale: 1.0, alpha, single: true }
    }

    pub(crate) fn bound(&self, b: f64) -> PortBound {
        // One junction: `v' = a / (1 + R g)`. Below the solve `|v'| <= a`;
        // above it `g` is past its value at the solve, so `v` rises no faster
        // than `a / (1 + R g(v))` there. Several: `|v'| <= alpha`.
        let (lo, hi) = if self.single {
            (self.v - self.alpha * b, self.v + self.alpha * b / (1.0 + self.r_max * slope(&self.law, self.v)))
        } else {
            let w = self.dv.abs().max(self.alpha) * b;
            (self.v - w, self.v + w)
        };
        let (g_lo, g_hi) = (slope(&self.law, lo), slope(&self.law, hi));
        let a2 = self.alpha * self.alpha;
        let k_v = self.node_scale * a2 * phi_sup(self.r_min * g_lo, self.r_max * g_hi) / self.law.vtn;
        let k_a = a2 * psi_sup(g_lo, g_hi, self.r_psi) / self.law.vtn;
        let i_min = if lo >= 0.0 {
            self.law.current(lo)
        } else if hi <= 0.0 {
            -self.law.current(hi)
        } else {
            0.0
        };
        PortBound { k_v, k_a, g_hi, i_min }
    }
}

/// A published current that is linear in the partition's voltages: its
/// value and rail slope at the solve, its summed contact conductance and its
/// number of contacts.
#[derive(Debug, Clone, Copy)]
pub(crate) struct Branch {
    pub i: f64,
    pub di: f64,
    pub g: f64,
    pub contacts: f64,
    /// The junctions in series with it (`Bound::series`): a one-contact
    /// branch with an end on a node that holds nothing else but junctions
    /// (and the shunt).
    pub series: Option<usize>,
    /// The device publishing it (a capacitor's leakage: none), for the
    /// tests that hold a branch's true error against its bound.
    #[cfg_attr(not(test), allow(dead_code))]
    pub device: Option<usize>,
}

/// The rail drivers on one node `o` from ICs with one supply difference:
/// their summed conductance is `A + C / vs` and their summed `G * far end`
/// is affine in `u` plus `B / vs` (a far end on the rail is `c0 + sigma vs`,
/// one on a fixed node a constant), so the driver current into `o`, `Phi -
/// Gamma x_o`, curves the solution through
/// `h = Gamma'' x_o - Phi'' + 2 Gamma' x_o' = 2 (C x_o - B) / vs^3 - 2 sigma C
/// x_o' / vs^2`. Summing before bounding keeps what cancels (a 555's OUT
/// pair is a fixed 20 ohm to `u - 0.1 V`: `C = B = 0`). `c_r` is `C` of the
/// branches to the rail alone, whose own currents the draw holds.
#[derive(Debug, Clone)]
struct IcGroup {
    o: usize,
    sigma: f64,
    vs: f64,
    c: f64,
    b: f64,
    c_r: f64,
    /// `x_o`, `s_o` and `u` at the solve.
    x: f64,
    s: f64,
    u: f64,
    /// The rest's driving-point resistance at `o` at the solve, and its
    /// transfer resistance from each port to `o` (one junction only).
    z: f64,
    z_port: Vec<f64>,
}

/// One rail driver as the band reads it: its node, its far end's slope in
/// `u` (1 on the rail, 0 on a fixed node) and that end's offset `far - x_o`
/// at the solve.
#[derive(Debug, Clone, Copy)]
struct DriverAt {
    d: RailDriver,
    far_slope: f64,
    w: f64,
}

/// Everything the band reads, held so that any half-width can be checked
/// (and a test can hold a partition's true error against it).
#[derive(Debug, Clone)]
pub(crate) struct Bound {
    ports: Vec<Port>,
    /// For each port, `|a|` at the solve (one junction), which `alpha`
    /// grows from with the drivers.
    a_p: Vec<f64>,
    groups: Vec<IcGroup>,
    drivers: Vec<DriverAt>,
    /// The largest `|s|` at the solve.
    s_max: f64,
    nodes: Vec<(f64, f64)>,
    branches: Vec<Branch>,
    /// The junctions on each node a branch is in series with.
    series: Vec<Vec<usize>>,
    draw: (f64, f64),
    draw_rule: DrawRule,
    /// The widest half-width the precomputed parts hold for.
    b_max: f64,
}

/// The bound at one half-width `b`: the node (and junction-voltage),
/// branch-current and draw curvature bounds, and each port's own.
pub(crate) struct Eval {
    pub k_v: f64,
    pub k_a: f64,
    pub k_d: f64,
    pub ports: Vec<PortBound>,
}

impl Bound {
    /// A bound with no rail drivers.
    #[cfg(test)]
    pub(crate) fn plain(ports: &[Port], nodes: &[(f64, f64)], branches: &[Branch], draw: (f64, f64)) -> Bound {
        Bound { ports: ports.to_vec(), a_p: ports.iter().map(|p| p.alpha).collect(), groups: Vec::new(), drivers: Vec::new(), s_max: nodes.iter().map(|n| n.1.abs()).fold(0.0, f64::max), nodes: nodes.to_vec(), branches: branches.to_vec(), series: Vec::new(), draw, draw_rule: DrawRule::General, b_max: rails::RAIL_RESOLVE_V }
    }

    /// The curvature bounds over `|u - u_p| <= b`, or `None` where they do
    /// not hold (a driver's conductance kinks, or its supply reaches 0,
    /// inside the band; a resistance or sensitivity bound does not close).
    pub(crate) fn eval(&self, b: f64) -> Option<Eval> {
        if b > self.b_max {
            return None;
        }
        let (ranges, eps_lo, eps_hi) = self.driver_ranges(b)?;
        let (m_x, m_v) = self.reach(b, &ranges)?;
        // Each group's curvature injection `h` over the band: its value at
        // the solve and at most `b` times its slope's bound, `h0 + h1 K`
        // with `K` the node bound itself (`x_o''` is in `h'`), and the same
        // for its branches to the rail (`hr0 + hr1 K`). `x_o'` over the band
        // is at most `m_x`, and at most `|s_o| + b K`: each gives a bound
        // that holds, so the least of the two is taken (a node that barely
        // follows the rail is held by the second, one past its knee by the
        // first).
        let scale_z = 1.0 / (1.0 - eps_lo);
        let mut terms = Vec::with_capacity(self.groups.len());
        let mut c_a = vec![0.0; self.ports.len()];
        for gr in &self.groups {
            let vs_m = gr.vs - b;
            if !(vs_m > 0.0) {
                return None;
            }
            let (v2, v3) = (gr.vs * gr.vs, gr.vs * gr.vs * gr.vs);
            let (m2, m3, m4) = (vs_m * vs_m, vs_m * vs_m * vs_m, vs_m * vs_m * vs_m * vs_m);
            let (c, cr, bq) = (gr.c.abs(), gr.c_r.abs(), gr.b.abs());
            let h_p = (2.0 * (gr.c * gr.x - gr.b) / v3 - 2.0 * gr.sigma * gr.c * gr.s / v2).abs();
            let hr_p = (2.0 * gr.c_r * (gr.x - gr.u) / v3 - 2.0 * gr.sigma * gr.c_r * (gr.s - 1.0) / v2).abs();
            let x_max = gr.x.abs() + b * m_x;
            let w_r = (gr.x - gr.u).abs();
            // With `|x_o'| <= m_x`.
            let by_m = [
                h_p + b * (6.0 * c * m_x / m3 + 6.0 * (c * x_max + bq) / m4),
                b * 2.0 * c / m2,
                hr_p + b * (6.0 * cr * (m_x + 1.0) / m3 + 6.0 * cr * (w_r + b * (m_x + 1.0)) / m4),
                b * 2.0 * cr / m2,
            ];
            // With `|x_o'| <= |s_o| + b K`, `|x_o| <= |x_o| + b |s_o| + b^2 K / 2`.
            let s_o = gr.s.abs();
            let by_k = [
                h_p + b * (6.0 * c * s_o / m3 + 6.0 * (c * (gr.x.abs() + b * s_o) + bq) / m4),
                b * (2.0 * c / m2 + 6.0 * c * b / m3 + 3.0 * c * b * b / m4),
                hr_p + b * (6.0 * cr * (s_o + 1.0) / m3 + 6.0 * cr * (w_r + b * (s_o + 1.0)) / m4),
                b * (2.0 * cr / m2 + 6.0 * cr * b / m3 + 6.0 * cr * b * b / m4),
            ];
            terms.push((gr.z * scale_z, by_m, by_k));
            // How far the open port's sensitivity moves over the band.
            let ca = 2.0 * bq / m3 + 2.0 * c * x_max / m3 + 2.0 * c * m_x / m2;
            for (j, z) in gr.z_port.iter().enumerate() {
                c_a[j] += z.abs() * ca;
            }
        }
        let mut k_j_v = 0.0;
        let mut k_j_a = 0.0;
        let mut pbs = Vec::with_capacity(self.ports.len());
        for (j, p) in self.ports.iter().enumerate() {
            let port = if p.single {
                Port { alpha: self.a_p[j] + b * c_a[j], node_scale: p.node_scale * scale_z, r_psi: p.r_psi / (1.0 + eps_hi), ..*p }
            } else {
                Port { alpha: m_v, r_min: p.r_min / (1.0 + eps_hi), r_max: p.r_max * scale_z, r_psi: p.r_psi / (1.0 + eps_hi), ..*p }
            };
            let pb = port.bound(b);
            k_j_v += pb.k_v;
            k_j_a += pb.k_a;
            pbs.push(pb);
        }
        // `K <= k_j_v + sum z (h0 + h1 K)`, closed for each choice.
        let closed = |by_slope: bool| -> Option<(f64, f64, f64)> {
            let (mut num, mut den) = (k_j_v, 1.0);
            for (z, m, k) in &terms {
                let t = if by_slope { k } else { m };
                num += z * t[0];
                den -= z * t[1];
            }
            if !(den > 0.0) {
                return None;
            }
            let k_v = num / den;
            let (mut k_a, mut k_r) = (k_j_a, 0.0);
            for (_, m, k) in &terms {
                let t = if by_slope { k } else { m };
                k_a += t[0] + t[1] * k_v;
                k_r += t[2] + t[3] * k_v;
            }
            Some((k_v, k_a, k_a + k_r))
        };
        let (k_v, k_a, k_d) = match (closed(false), closed(true)) {
            (Some(a), Some(b)) => (a.0.min(b.0), a.1.min(b.1), a.2.min(b.2)),
            (Some(a), None) | (None, Some(a)) => a,
            (None, None) => return None,
        };
        (k_v.is_finite() && k_a.is_finite() && k_d.is_finite()).then_some(Eval { k_v, k_a, k_d, ports: pbs })
    }

    /// Each rail driver's conductance over `|u - u_p| <= b`, `(G_min,
    /// G_max, max |G'|, max |G''|)`, and the most any falls (`eps_lo`) or
    /// rises (`eps_hi`) relative to the solve. Each is monotone on its piece,
    /// so its range is its ends'; every resistance of the rest then moves
    /// inside `1 / (1 + eps_hi)` and `1 / (1 - eps_lo)` of its value at the
    /// solve (the rest with every conductance scaled by one factor bounds
    /// it, Rayleigh). `None` where a driver's conductance kinks, or its
    /// supply reaches 0, inside the band.
    #[allow(clippy::type_complexity)]
    fn driver_ranges(&self, b: f64) -> Option<(Vec<(f64, f64, f64, f64)>, f64, f64)> {
        let (mut eps_lo, mut eps_hi) = (0.0f64, 0.0f64);
        let mut ranges = Vec::with_capacity(self.drivers.len());
        for da in &self.drivers {
            let d = &da.d;
            let vs_lo = d.vs - b;
            if !(vs_lo > 0.0) || d.curve.kinks.iter().any(|&k| (k - d.vs).abs() <= b) {
                return None;
            }
            let g = d.curve.derivs(d.vs)[0];
            let (g_a, g_b) = (d.curve.derivs(vs_lo)[0], d.curve.derivs(d.vs + b)[0]);
            let (g_min, g_max) = (g_a.min(g_b), g_a.max(g_b));
            if !(g > 0.0 && g_min > 0.0) {
                return None;
            }
            eps_lo = eps_lo.max((g - g_min) / g);
            eps_hi = eps_hi.max((g_max - g) / g);
            let beta = d.curve.piece(d.vs).1.abs();
            ranges.push((g_min, g_max, beta / (vs_lo * vs_lo), 2.0 * beta / (vs_lo * vs_lo * vs_lo)));
        }
        (eps_lo < 1.0).then_some((ranges, eps_lo, eps_hi))
    }

    /// A bound on every node's sensitivity over `|u - u_p| <= b`, `(max
    /// |x'|, max |x_a' - x_b'|)`. The first derivative solves a passive
    /// network whose terminals sit at 1 (the rail), 0 (fixed nodes) and,
    /// through each driven node's driver conductance `Gamma`, at `rho =
    /// (Phi' - Gamma' x_o) / Gamma`; so every `x'` lies between the least
    /// and the greatest terminal. `rho` reads `x_o` over the band, which
    /// reads the bound: one assumed at `m0` that yields less than `m0`
    /// holds, since `|x'|` could not first reach `m0`. Without drivers,
    /// `(1, 1)`.
    fn reach(&self, b: f64, ranges: &[(f64, f64, f64, f64)]) -> Option<(f64, f64)> {
        let m0 = 2.0 * self.s_max.max(1.0);
        let (mut rho_lo, mut rho_hi) = (0.0f64, 1.0f64);
        let mut seen: Vec<usize> = self.drivers.iter().map(|d| d.d.o).collect();
        seen.sort_unstable();
        seen.dedup();
        for &o in &seen {
            let (mut gam_lo, mut gam_hi, mut num, mut dnum) = (0.0, 0.0, 0.0, 0.0);
            for (da, &(g_min, g_max, dg_max, d2g_max)) in self.drivers.iter().zip(ranges).filter(|(da, _)| da.d.o == o) {
                let d = &da.d;
                let [g, dg_dvs, ..] = d.curve.derivs(d.vs);
                let dg = d.sigma * dg_dvs;
                gam_lo += g_min;
                gam_hi += g_max;
                num += dg * da.w + g * da.far_slope;
                let w_max = da.w.abs() + b * (da.far_slope + m0);
                dnum += b * (d2g_max * w_max + dg.abs() * (da.far_slope + m0) + dg_max * da.far_slope);
            }
            let (n_lo, n_hi) = (num - dnum, num + dnum);
            rho_lo = rho_lo.min(if n_lo >= 0.0 { n_lo / gam_hi } else { n_lo / gam_lo });
            rho_hi = rho_hi.max(if n_hi >= 0.0 { n_hi / gam_lo } else { n_hi / gam_hi });
        }
        let m_x = rho_hi.max(-rho_lo);
        (m_x < m0).then_some((m_x, rho_hi - rho_lo))
    }

    /// The bound on junction `j`'s published current's error at `|u - u_p|
    /// = b`, from `eval(b)`. The published current is the law at the
    /// corrected voltage `v_p + v' du`, so its error is at most the slope
    /// over the band times the voltage's error; or, splitting it at the
    /// current's own first-order correction `I_p + g_p v' du`, the law's
    /// curvature over that step (`f'' <= f' / vtn`) plus the current's
    /// second-order remainder, which is a branch current's (`k_a`): seen from
    /// the junction, the rest of the partition carries the junction's own
    /// injection `f'' v'^2` scaled by `1 / (1 + R g)` and no more than all of
    /// any other injection (a passive network amplifies no current). Past
    /// the knee and driven through a resistance `R`, the second is `1 + R g`
    /// times the smaller: the voltage error the first multiplies by `g` is
    /// the driver's curvature through all of `R` (Rails point 3).
    fn junction_err(&self, ev: &Eval, j: usize, b: f64) -> f64 {
        let (pb, p) = (&ev.ports[j], &self.ports[j]);
        let e = 0.5 * b * b;
        let by_voltage = pb.g_hi * ev.k_v * e;
        let by_current = (pb.g_hi * p.dv * p.dv / p.law.vtn + ev.k_a) * e;
        by_voltage.min(by_current)
    }

    /// The bound on a linear branch's published current's error at `|u -
    /// u_p| = b`, from `eval(b)`: a branch current's curvature (`k_a`) at
    /// each contact, or its conductance times a node's. A branch in series
    /// with junctions, its end on a node that holds nothing else, carries
    /// by KCL there their currents and the node's shunt current, in the
    /// solution and (KCL being linear) in its first-order correction alike;
    /// so it errs by no more than they do: each junction current's
    /// first-order correction against the law at the corrected voltage
    /// (`f'' <= f' / vtn`), the law over the voltage's error, and GMIN and
    /// the shunt over it. A branch behind junctions that stay off over the
    /// band (a multiplexed display's segment line) carries nothing whatever
    /// the drivers elsewhere do, which `k_a`, the sum of every curvature
    /// injection in the partition, cannot see.
    fn branch_err(&self, ev: &Eval, br: &Branch, b: f64) -> f64 {
        let e = 0.5 * b * b;
        let mut err = (br.contacts * ev.k_a).min(br.g * ev.k_v) * e;
        if let Some(k) = br.series {
            let mut k_s = crate::solve::static_part::NODE_RSHUNT_G * ev.k_v;
            for &j in &self.series[k] {
                let (pb, p) = (&ev.ports[j], &self.ports[j]);
                k_s += pb.g_hi * p.dv * p.dv / p.law.vtn + (pb.g_hi + crate::devices::led::GMIN_D) * ev.k_v;
            }
            err = err.min(k_s * e);
        }
        err
    }

    /// Whether every published quantity's bound over `|u - u_p| <= b` is
    /// inside W0's tolerance at its smallest magnitude over the band.
    fn valid(&self, b: f64) -> bool {
        let Some(ev_b) = self.eval(b) else { return false };
        let e = 0.5 * b * b;
        let (ev, ed) = (ev_b.k_v * e, ev_b.k_d * e);
        let ok_v = self.nodes.iter().all(|&(x, s)| ev <= V_ABS + REL * (x.abs() - s.abs() * b - ev).max(0.0));
        let ok_j = ev_b.ports.iter().enumerate().all(|(j, pb)| self.junction_err(&ev_b, j, b) <= A_ABS + REL * pb.i_min);
        let ok_b = self.branches.iter().all(|br| {
            let err = self.branch_err(&ev_b, br, b);
            err <= A_ABS + REL * (br.i.abs() - br.di.abs() * b - err).max(0.0)
        });
        let ok_d = match self.draw_rule {
            DrawRule::Zero => true,
            DrawRule::ShuntOnly => crate::solve::static_part::NODE_RSHUNT_G * self.nodes.len() as f64 * ev <= A_ABS,
            DrawRule::General => ed <= A_ABS + REL * (self.draw.0.abs() - self.draw.1.abs() * b - ed).max(0.0),
        };
        ok_v && ok_j && ok_b && ok_d
    }

    /// The widest band, at most `RAIL_RESOLVE_V`, over which every published
    /// quantity's second-order bound stays inside W0's tolerance; where none
    /// does, the narrowest, which re-solves at every move of the rail.
    pub(crate) fn band(&self) -> f64 {
        let cap = rails::RAIL_RESOLVE_V;
        if self.valid(cap) {
            return cap;
        }
        let (mut lo, mut hi) = (0.0, cap);
        for _ in 0..BAND_STEPS {
            let mid = 0.5 * (lo + hi);
            if self.valid(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo.max(NARROWEST_BAND)
    }
}

/// The band of [`Bound::plain`] (no rail drivers): `nodes` are `(x, s)`,
/// `draw` is `(I_p, g_p)`.
#[cfg(test)]
pub(crate) fn band(ports: &[Port], nodes: &[(f64, f64)], branches: &[Branch], draw: (f64, f64)) -> f64 {
    Bound::plain(ports, nodes, branches, draw).band()
}

/// A curved hub partition's band at its solve (`x` at rail `u`, through
/// `sens`, whose `jac` is the Jacobian the residual stamps gave at the
/// solve, and its IC stamps that follow the rail `ic`): its junctions and
/// its rail-supplied drivers, either alone. `bnd` reads the boundary with
/// the rail stamped as unknown `n`, as the sensitivity's stamps did. Always
/// `Some`: where the bound cannot be formed (a rest that does not factor)
/// the band is the narrowest, never the flat one.
#[allow(clippy::too_many_arguments)]
pub(crate) fn hub_band(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, bnd: &Bnd, x: &[f64], u: f64, sens: &RailSens, ic: &IcRail) -> Option<f64> {
    let bound = hub_bound(shape, live_params, bnd, x, u, sens, ic);
    #[cfg(test)]
    hook::record(&shape.nodes, u, bound.clone());
    Some(bound.map_or(NARROWEST_BAND, |b| b.band()))
}

/// One end of a closed-form segment's capacitor range for
/// [`segment_band`]: the partition's node voltages `x` with the capacitor at
/// that voltage, their exact sensitivity and the draw's (`sens`, whose `jac`
/// stamps the capacitor as a conductance `g_cap` to its source), and the
/// capacitor's series current as a branch.
pub(crate) struct SegmentEnd {
    pub x: Vec<f64>,
    pub sens: RailSens,
    pub cap: Branch,
}

/// A curved closed-form hub partition's band (ruling R8 (a), Rails point 3).
/// The segment publishes `x0 + dx vc + (s0 + s1 vc) (u - u_seg)`, exact to
/// first order in the rail at every capacitor voltage, so its error is the
/// second-order remainder of `x(u, vc) = x0(u) + dx(u) vc`, which is affine
/// in `vc`: over the voltages the capacitor can take on the segment (`ends`)
/// it is at most the larger of the two ends' remainders. At each end the
/// capacitor holds its voltage, so the partition there is a static one with
/// the capacitor a source behind a conductance (`1 / esr`, or a stiff one at
/// `esr = 0`, which only overstates the rest's resistances, Rayleigh), and
/// [`hub_bound`] bounds it. Each end is checked against the least magnitude
/// each quantity takes over the range, so the band holds at every voltage
/// between them. Also returns whether the segment is affine in the rail,
/// whose band then holds at every `vc`, and may be wider than
/// `RAIL_RESOLVE_V`.
pub(crate) fn segment_band(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, bnd: &Bnd, u: f64, ends: &[SegmentEnd; 2], ic: &IcRail) -> (f64, bool) {
    let mut bounds = Vec::with_capacity(2);
    for end in ends {
        let Some(mut b) = hub_bound(shape, live_params, bnd, &end.x, u, &end.sens, ic) else {
            #[cfg(test)]
            hook::record(&shape.nodes, u, None);
            return (NARROWEST_BAND, false);
        };
        b.branches.push(end.cap);
        bounds.push(b);
    }
    // No junction, and every driver group's conductance and far-end sum
    // constant on its piece (a 555's discharge transistor, a driver on its
    // R_out clamp): the segment is affine in the rail at every `vc`, its
    // correction exact, until a driver's conductance kinks. A 555's OUT pair
    // with vcc on the rail is not: its split follows the supply (V9).
    if bounds.iter().all(|b| b.ports.is_empty() && b.groups.iter().all(|g| g.c == 0.0 && g.b == 0.0 && g.c_r == 0.0)) {
        let kink = ic.drivers.iter().map(|d| d.kink_distance()).fold(f64::INFINITY, f64::min);
        #[cfg(test)]
        hook::record(&shape.nodes, u, Some(bounds.swap_remove(0)));
        return (if kink.is_finite() { (kink * (1.0 - 1e-9)).max(NARROWEST_BAND) } else { f64::INFINITY }, true);
    }
    let (b0, b1) = bounds.split_at_mut(1);
    let (b0, b1) = (&mut b0[0], &mut b1[0]);
    // An affine quantity's least magnitude over the range: 0 where it
    // changes sign. Its slope in the rail is affine too: the greater end.
    let least = |p: f64, q: f64| if p * q <= 0.0 { 0.0 } else { p.abs().min(q.abs()) };
    for (n0, n1) in b0.nodes.iter_mut().zip(b1.nodes.iter_mut()) {
        let v = (least(n0.0, n1.0), n0.1.abs().max(n1.1.abs()));
        *n0 = v;
        *n1 = v;
    }
    for (r0, r1) in b0.branches.iter_mut().zip(b1.branches.iter_mut()) {
        let (i, di) = (least(r0.i, r1.i), r0.di.abs().max(r1.di.abs()));
        r0.i = i;
        r1.i = i;
        r0.di = di;
        r1.di = di;
    }
    let draw = (least(b0.draw.0, b1.draw.0), b0.draw.1.abs().max(b1.draw.1.abs()));
    b0.draw = draw;
    b1.draw = draw;
    let (w0, w1) = (b0.band(), b1.band());
    #[cfg(test)]
    hook::record(&shape.nodes, u, Some(if w0 <= w1 { b0.clone() } else { b1.clone() }));
    (w0.min(w1), false)
}

#[allow(clippy::too_many_arguments)]
fn hub_bound(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, bnd: &Bnd, x: &[f64], u: f64, sens: &RailSens, ic: &IcRail) -> Option<Bound> {
    let n = shape.n;
    if n == 0 {
        return None;
    }
    let size = n + 1;
    // A pin's voltage, rail sensitivity, and its row in the n x n system.
    let pin = |p: &crate::solve::DynPin| -> (f64, f64, Option<usize>) {
        match bnd.local(p) {
            Some(l) if l < n => (x[l], sens.s[l], Some(l)),
            Some(_) => (u, 1.0, None),
            None => (bnd.get(p.node), 0.0, None),
        }
    };
    let ends: Vec<(Option<usize>, Option<usize>, f64, f64)> = shape
        .junctions
        .iter()
        .map(|j| {
            let (va, sa, la) = pin(&j.anode);
            let (vk, sk, lk) = pin(&j.cathode);
            (la, lk, va - vk, sa - sk)
        })
        .collect();
    // The rest of the partition: the Jacobian less each junction's own
    // slope (its GMIN stays, a conductance like any other).
    let mut ext = vec![0.0; n * n];
    for r in 0..n {
        ext[r * n..r * n + n].copy_from_slice(&sens.jac[r * size..r * size + n]);
    }
    let add = |m: &mut [f64], la: Option<usize>, lk: Option<usize>, g: f64| {
        if let Some(a) = la {
            m[a * n + a] += g;
        }
        if let Some(k) = lk {
            m[k * n + k] += g;
        }
        if let (Some(a), Some(k)) = (la, lk) {
            m[a * n + k] -= g;
            m[k * n + a] -= g;
        }
    };
    for (j, &(la, lk, v, _)) in shape.junctions.iter().zip(&ends) {
        add(&mut ext, la, lk, -slope(&j.law, v));
    }
    // A network's response to a unit injection across a junction (`la`
    // in, `lk` out), and the driving-point resistance there.
    let response = |m: &lu::LuFactorization, la: Option<usize>, lk: Option<usize>| -> Option<(Vec<f64>, f64)> {
        let mut e = vec![0.0; n];
        if let Some(a) = la {
            e[a] += 1.0;
        }
        if let Some(k) = lk {
            e[k] -= 1.0;
        }
        m.solve(&mut e).ok()?;
        let r = la.map_or(0.0, |a| e[a]) - lk.map_or(0.0, |k| e[k]);
        (r.is_finite() && e.iter().all(|v| v.is_finite())).then_some((e, r.max(0.0)))
    };
    let rest = lu::lu_factor(ext.clone(), n).ok()?;
    let single = shape.junctions.len() == 1;
    let mut ports = Vec::with_capacity(ends.len());
    let mut z_ports: Vec<Vec<f64>> = Vec::new();
    let mut b_max = rails::RAIL_RESOLVE_V;
    if shape.junctions.is_empty() {
        // No port: the drivers alone curve it, through the rest, which is
        // the whole partition.
    } else if single {
        let (la, lk, v, dv) = ends[0];
        let law = shape.junctions[0].law;
        let (col, r) = response(&rest, la, lk)?;
        let alpha = dv.abs() * (1.0 + r * slope(&law, v));
        ports.push(Port::single(law, v, dv, r, alpha));
        z_ports.push(col);
    } else {
        // Over a band of half-width `reach` each junction's voltage moves no
        // more than `m_v reach` (`m_v` the partition's sensitivity bound
        // there), so its slope stays below its value that far up; with every
        // other one there, the resistance across a junction is at its least
        // (Rayleigh), and with none at its most. The widest band holds
        // unless a driver kinks inside it: then the bound is formed over the
        // widest half of it that does not.
        let probe = Bound { ports: Vec::new(), a_p: Vec::new(), groups: Vec::new(), drivers: rail_drivers(ic, x, u), s_max: sens.s.iter().fold(0.0f64, |m, s| m.max(s.abs())), nodes: Vec::new(), branches: Vec::new(), series: Vec::new(), draw: (0.0, 0.0), draw_rule: DrawRule::General, b_max: rails::RAIL_RESOLVE_V };
        let mut reach = rails::RAIL_RESOLVE_V;
        let m_v = loop {
            if let Some((ranges, _, _)) = probe.driver_ranges(reach) {
                if let Some((_, m_v)) = probe.reach(reach, &ranges) {
                    break m_v;
                }
            }
            reach *= 0.5;
            if reach < NARROWEST_BAND {
                return None;
            }
        };
        b_max = reach;
        let mut hi = ext;
        let g_hi: Vec<f64> = shape.junctions.iter().zip(&ends).map(|(j, &(_, _, v, dv))| slope(&j.law, v + dv.abs().max(m_v) * reach)).collect();
        for (&(la, lk, _, _), &g) in ends.iter().zip(&g_hi) {
            add(&mut hi, la, lk, g);
        }
        let all_on = lu::lu_factor(hi, n).ok()?;
        for (j, (&(la, lk, v, dv), &g)) in shape.junctions.iter().zip(ends.iter().zip(&g_hi)) {
            let r_max = response(&rest, la, lk)?.1;
            let rho = response(&all_on, la, lk)?.1;
            let den = 1.0 - rho * g;
            let r_min = if den > 0.0 { (rho / den).min(r_max) } else { 0.0 };
            ports.push(Port { law: j.law, v, dv, r_min, r_max, r_psi: r_min, node_scale: 1.0, alpha: 1.0, single: false });
        }
    }
    // The rail drivers, gathered by node and supply.
    let drivers = rail_drivers(ic, x, u);
    let mut groups: Vec<IcGroup> = Vec::new();
    for da in &drivers {
        let d = &da.d;
        let (_, beta) = d.curve.piece(d.vs);
        let c0 = u - d.sigma * d.vs;
        let far_b = if d.to_rail { beta * c0 } else { beta * d.t };
        let gr = match groups.iter_mut().find(|g| g.o == d.o && g.sigma == d.sigma && g.vs.to_bits() == d.vs.to_bits()) {
            Some(g) => g,
            None => {
                let mut e = vec![0.0; n];
                e[d.o] = 1.0;
                rest.solve(&mut e).ok()?;
                let z = e[d.o];
                if !(z.is_finite() && e.iter().all(|v| v.is_finite())) {
                    return None;
                }
                let z_port = z_ports.iter().map(|col| col[d.o]).collect();
                groups.push(IcGroup { o: d.o, sigma: d.sigma, vs: d.vs, c: 0.0, b: 0.0, c_r: 0.0, x: x[d.o], s: sens.s[d.o], u, z: z.max(0.0), z_port });
                groups.last_mut().unwrap()
            }
        };
        gr.c += beta;
        gr.b += far_b;
        if d.to_rail {
            gr.c_r += beta;
        }
    }
    let nodes: Vec<(f64, f64)> = (0..n).map(|l| (x[l], sens.s[l])).collect();
    // What each local node holds, for the branches in series with a
    // junction: contacts, junction ends, and anything else (a capacitor, an
    // IC pin) that rules a node out.
    let (mut contacts_at, mut junction_at, mut other_at) = (vec![0u32; n], vec![Vec::new(); n], vec![false; n]);
    for (j, &(la, lk, _, _)) in ends.iter().enumerate() {
        for l in [la, lk].into_iter().flatten() {
            junction_at[l].push(j);
        }
        // A junction across one node is no current through it.
        if la.is_some() && la == lk {
            other_at[la.unwrap()] = true;
        }
    }
    for cap in &shape.capacitors {
        for p in &cap.pins {
            if let (_, _, Some(l)) = pin(p) {
                other_at[l] = true;
            }
        }
    }
    for &l in &ic.pinned {
        other_at[l] = true;
    }
    let stamps: Vec<Vec<crate::devices::Contribution>> = shape.stamped.iter().map(|st| crate::devices::stamp(st.kind, st.pins.len(), live_params_for(st, live_params))).collect();
    for (st, cs) in shape.stamped.iter().zip(&stamps) {
        for c in cs {
            for p in [&st.pins[c.pin_a], &st.pins[c.pin_b]] {
                if let (_, _, Some(l)) = pin(p) {
                    contacts_at[l] += 1;
                }
            }
        }
    }
    let mut branches = Vec::new();
    let mut series: Vec<Vec<usize>> = Vec::new();
    for (st, cs) in shape.stamped.iter().zip(&stamps) {
        let (mut i, mut di, mut g, mut contacts) = (0.0, 0.0, 0.0, 0.0);
        let mut in_series = None;
        for c in cs {
            let (va, sa, la) = pin(&st.pins[c.pin_a]);
            let (vb, sb, lb) = pin(&st.pins[c.pin_b]);
            i += c.conductance * (va - vb);
            di += c.conductance * (sa - sb);
            g += c.conductance.abs();
            contacts += 1.0;
            if cs.len() == 1 && la != lb {
                if let Some(l) = [la, lb].into_iter().flatten().find(|&l| contacts_at[l] == 1 && !junction_at[l].is_empty() && !other_at[l]) {
                    in_series = Some(series.len());
                    series.push(junction_at[l].clone());
                }
            }
        }
        if contacts > 0.0 {
            branches.push(Branch { i, di, g, contacts, series: in_series, device: Some(st.device_index) });
        }
    }
    // A stepped capacitor publishes its committed series current plus its
    // leakage, so only the leakage follows the rail: its error is at most
    // `leak_g` times its voltage's, and its value is not read here (no
    // magnitude is assumed).
    for cap in &shape.capacitors {
        if cap.leak_g != 0.0 {
            let (_, sa, _) = pin(&cap.pins[0]);
            let (_, sb, _) = pin(&cap.pins[1]);
            branches.push(Branch { i: 0.0, di: cap.leak_g * (sa - sb), g: cap.leak_g, contacts: 1.0, series: None, device: None });
        }
    }
    let a_p = ports.iter().map(|p| p.alpha).collect();
    let s_max = sens.s.iter().fold(0.0f64, |m, s| m.max(s.abs()));
    // The draw's carriers (`DrawRule`): devices with a pin on the rail are
    // fixed at load; the IC branches are the committed ones.
    let on_rail = |p: &crate::solve::DynPin| bnd.local(p) == Some(n);
    let device_on_rail = shape.stamped.iter().zip(&stamps).any(|(st, cs)| cs.iter().any(|c| on_rail(&st.pins[c.pin_a]) || on_rail(&st.pins[c.pin_b])))
        || shape.junctions.iter().any(|j| on_rail(&j.anode) || on_rail(&j.cathode))
        || shape.capacitors.iter().any(|c| c.pins.iter().any(on_rail));
    let ic_on_rail = ic.touches_rail;
    let conducts = stamps.iter().any(|cs| !cs.is_empty()) || !shape.junctions.is_empty() || !shape.capacitors.is_empty();
    let draw_rule = if !device_on_rail && !ic_on_rail {
        DrawRule::Zero
    } else if !conducts && !ic.sinks {
        DrawRule::ShuntOnly
    } else {
        DrawRule::General
    };
    Some(Bound { ports, a_p, groups, drivers, s_max, nodes, branches, series, draw: (sens.i, sens.g), draw_rule, b_max })
}

/// Which partitions hold a logic pin of an IC with one supply on the rail
/// node `rail`: one its driver may stamp, whose conductance follows the
/// rail (Rails point 3), so the partition is curved in the rail whether or
/// not it holds a junction. A pin that only reads, or a driver that is off,
/// curves nothing; its partition's bound then has no driver term.
pub(crate) fn holds_rail_ic_pin(ics: &[(usize, crate::digital::IcRecord)], local_index: &[Option<(usize, usize)>], rail: usize, partitions: usize) -> Vec<bool> {
    let mut out = vec![false; partitions];
    for (_, ic) in ics {
        if (ic.vcc.node == rail) == (ic.gnd.node == rail) {
            continue;
        }
        for pin in &ic.logic {
            if let Some((p, _)) = local_index[pin.node] {
                out[p] = true;
            }
        }
    }
    out
}

/// `ic`'s drivers with their far ends read at the solve.
fn rail_drivers(ic: &IcRail, x: &[f64], u: f64) -> Vec<DriverAt> {
    ic.drivers
        .iter()
        .map(|&d| {
            let far = if d.to_rail { u } else { d.t };
            DriverAt { d, far_slope: if d.to_rail { 1.0 } else { 0.0 }, w: far - x[d.o] }
        })
        .collect()
}

// ---- A partition holding a free IC supply ------------------------------------
//
// An IC whose `vcc` or `gnd` is a node of the partition itself (supplied
// from the rail through a resistor, say) stamps a driver `G(vs) (v_a - v_f)`
// whose conductance follows that node: the partition is no conductance
// network, so neither the maximum principle nor Rayleigh's bounds above
// hold. Its band comes from a perturbation bound on the exact Jacobian
// instead, kept in the space of the curved elements so that what cancels
// across a junction stays cancelled.
//
// Each curved element `e` (a junction, a driver) injects into its two rows
// `P_e` and its Jacobian is `P_e (sum_t d_et chi_et^T)`: a junction's slope
// times its voltage form, a driver's `G` times `w = v_a - v_f` plus `G' w`
// times `vs`. Over the band each scalar `d_et(u)` moves from its value at
// the solve by `dd_et`, at most `delta_et`, read off the range each form
// spans. With `M = J0^-1` and `s = x'(u_p)`, differentiating `F(x(u), u) =
// 0` once and twice gives (`z' = (x', 1)`)
//   x'  = s - sum_e M P_e sum_t dd_et (chi_et . z'),
//   x'' = - sum_e M P_e (h_e + sum_t dd_et (chi_et . x'')),
// `h_e` the element's second-order injection (`f''(v) v'^2`, `G'' vs'^2 w +
// 2 G' vs' w'`). Reading each form through these, `y_f = |chi_f . z'|` and
// `q_f = |chi_f . x''|` satisfy `y <= |chi . s| + Q y` and `q <= q0 + Q q`,
// `Q_ff' = |chi_f M P_e(f')| delta_f'`, a system the size of the elements.
// One assumed `Y0` whose image `Y1 = |chi . s| + Q Y0` is below it holds
// over the band (`y` could not first reach it), and `Q Y0 < Y0` is the
// Collatz-Wielandt test that `Q`'s radius is under 1, so any `q >= q0 + Q q`
// bounds `q`. A published linear quantity `L x` then errs by `|L x''| <=
// sum_e |L M P_e| (eta_e + sum_t delta_et q_et)`. The bound holds on one
// smooth piece: a driver's kink inside the band rules the band out. Like
// the rail-supplied bound it holds within the present power state; a free
// supply crossing its power window's edge is no curvature (a re-solve
// forced at the edge only lands the iterated solve on the floor, where its
// power gate ping-pongs).

/// One end of an IC stamp: unknown `i` of `z` (the partition's own, then
/// the rail at `n`), or a fixed voltage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum End {
    Z(usize),
    Fixed(f64),
}

impl End {
    fn at(self, z: &[f64]) -> f64 {
        match self {
            End::Z(i) => z[i],
            End::Fixed(v) => v,
        }
    }
}

/// A driver branch carrying `G(vs) (v_a - v_f)` from `a` into `f`, `vs =
/// v_vcc - v_gnd`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SupplyDriver {
    pub a: End,
    pub f: End,
    pub vcc: End,
    pub gnd: End,
    pub curve: crate::digital::power::SupplyCurve,
}

/// A floating pull `g (v_pin - v_target)`, its target a supply pin.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SupplyPull {
    pub pin: usize,
    pub target: End,
    pub g: f64,
}

/// The IC stamps of a hub partition holding a free IC supply
/// (`ic_supply_terms`): every driver whose supply is not fixed, and every
/// pull whose target is not.
#[derive(Debug, Clone, Default)]
pub(crate) struct IcSupply {
    pub drivers: Vec<SupplyDriver>,
    pub pulls: Vec<SupplyPull>,
}

impl IcSupply {
    /// What the residual stamp's matrix at `z` misses of `dF/dz` (`(n +
    /// 1)^2`, the partition's unknowns then the rail, the rail's row its
    /// draw): a driver's current moves with its conductance, `G' (v_a -
    /// v_f)` per volt of supply, and a pull's target, stamped as a source,
    /// with its supply pin.
    pub(crate) fn extra_jacobian(&self, z: &[f64], n: usize) -> Vec<f64> {
        let size = n + 1;
        let mut out = vec![0.0; size * size];
        for d in &self.drivers {
            let dg = d.curve.derivs(d.vcc.at(z) - d.gnd.at(z))[1];
            let w = d.a.at(z) - d.f.at(z);
            for (row, sign) in [(d.a, 1.0), (d.f, -1.0)] {
                let End::Z(r) = row else { continue };
                for (col, coef) in [(d.vcc, 1.0), (d.gnd, -1.0)] {
                    if let End::Z(c) = col {
                        out[r * size + c] += sign * coef * dg * w;
                    }
                }
            }
        }
        for p in &self.pulls {
            if let End::Z(c) = p.target {
                out[p.pin * size + c] -= p.g;
            }
        }
        out
    }
}

/// A quantity affine in `z`: its value at the solve, its coefficients on
/// the partition's unknowns and on the rail, its rail slope at the solve
/// (`chi . (s, 1)`) and `chi_x . M P_e` for each element `e`.
#[derive(Debug, Clone, Default)]
struct Form {
    c0: f64,
    x: Vec<(usize, f64)>,
    slope: f64,
    t: Vec<f64>,
}

impl Form {
    fn of(ends: &[(End, f64)], z: &[f64], n: usize) -> Form {
        let mut f = Form::default();
        let mut u = 0.0;
        for &(e, coef) in ends {
            f.c0 += coef * e.at(z);
            match e {
                End::Z(i) if i < n => f.x.push((i, coef)),
                End::Z(_) => u += coef,
                End::Fixed(_) => {}
            }
        }
        f.slope = u;
        f
    }
}

#[derive(Debug, Clone)]
enum ElemKind {
    /// Its form `0` is the junction voltage.
    Junction(JunctionLaw),
    /// Its form `0` is `w = v_a - v_f`, form `1` the supply `vs`.
    Driver(crate::digital::power::SupplyCurve),
}

/// A curved element: its law, `M P_e` and its rail-row entry `r_e`, and its
/// forms' indices.
#[derive(Debug, Clone)]
struct Elem {
    kind: ElemKind,
    mp: Vec<f64>,
    r: f64,
    forms: Vec<usize>,
}

/// A published current linear in the partition's voltages: its value and
/// rail slope at the solve, and `L M P_e` for each element.
#[derive(Debug, Clone)]
struct SupplyBranch {
    i: f64,
    di: f64,
    t: Vec<f64>,
    #[cfg_attr(not(test), allow(dead_code))]
    device: Option<usize>,
}

/// The perturbation bound of a partition holding a free IC supply.
#[derive(Debug, Clone)]
pub(crate) struct SupplyBound {
    s: Vec<f64>,
    x: Vec<f64>,
    elems: Vec<Elem>,
    forms: Vec<Form>,
    /// The junctions' element indices, in the partition's junction order.
    junctions: Vec<usize>,
    branches: Vec<SupplyBranch>,
    /// The draw: `R_x M P_e + r_e`'s parts, and `(I_p, g_p)`.
    draw_t: Vec<f64>,
    draw: (f64, f64),
}

/// The bounds over one half-width: on each node's `|x''|`, each junction's
/// `|v''|` with its largest slope and smallest current, each branch's
/// `|I''|` and the draw's.
pub(crate) struct SupplyEval {
    k_node: Vec<f64>,
    k_junction: Vec<(f64, f64, f64)>,
    k_branch: Vec<f64>,
    k_draw: f64,
}

/// A driver conductance over `vs` in `[lo, hi]` inside one piece: its
/// derivatives at `vs0`, the most `G` and `G'` move from them, and the
/// largest `|G'|` and `|G''|` (`-beta / vs^2` and `2 beta / vs^3`, largest
/// at an end). `None` where a kink or `vs <= 0` falls inside.
fn curve_range(curve: &crate::digital::power::SupplyCurve, vs0: f64, lo: f64, hi: f64) -> Option<([f64; 4], f64, f64, f64, f64)> {
    if !(lo > 0.0) || curve.kinks.iter().any(|&k| k >= lo && k <= hi) {
        return None;
    }
    let (d0, dl, dh) = (curve.derivs(vs0), curve.derivs(lo), curve.derivs(hi));
    let dg = (dl[0] - d0[0]).abs().max((dh[0] - d0[0]).abs());
    let dg1 = (dl[1] - d0[1]).abs().max((dh[1] - d0[1]).abs());
    Some((d0, dg, dg1, dl[1].abs().max(dh[1].abs()), dl[2].abs().max(dh[2].abs())))
}

/// `q0 + Q q` for the element system: `Q_ff' = |t_f(e(f'))| delta_f'`.
fn apply_q(forms: &[Form], owner: &[usize], delta: &[f64], q0: &[f64], q: &[f64]) -> Vec<f64> {
    forms.iter().zip(q0).map(|(f, &a)| a + owner.iter().zip(delta).zip(q).map(|((&e, d), y)| f.t[e].abs() * d * y).sum::<f64>()).collect()
}

impl SupplyBound {
    /// Each form's `delta` (how far its coefficient in its element's
    /// Jacobian moves) over `|u - u_p| <= b` with `|chi . z'| <= y`, or
    /// `None` where a piece ends inside.
    fn deltas(&self, b: f64, y: &[f64]) -> Option<Vec<f64>> {
        let mut delta = vec![0.0; self.forms.len()];
        for el in &self.elems {
            match &el.kind {
                ElemKind::Junction(law) => {
                    let (f, v0) = (el.forms[0], self.forms[el.forms[0]].c0);
                    let span = y[f] * b;
                    let g0 = slope(law, v0);
                    delta[f] = (slope(law, v0 + span) - g0).max(g0 - slope(law, v0 - span));
                }
                ElemKind::Driver(curve) => {
                    let (fw, fs) = (el.forms[0], el.forms[1]);
                    let vs0 = self.forms[fs].c0;
                    let span = y[fs] * b;
                    let (d0, dg, dg1, _, _) = curve_range(curve, vs0, vs0 - span, vs0 + span)?;
                    let w_span = y[fw] * b;
                    delta[fw] = dg;
                    delta[fs] = dg1 * (self.forms[fw].c0.abs() + w_span) + d0[1].abs() * w_span;
                }
            }
        }
        Some(delta)
    }

    pub(crate) fn eval(&self, b: f64) -> Option<SupplyEval> {
        if b > rails::RAIL_RESOLVE_V {
            return None;
        }
        let owner: Vec<usize> = {
            let mut o = vec![0; self.forms.len()];
            for (e, el) in self.elems.iter().enumerate() {
                for &f in &el.forms {
                    o[f] = e;
                }
            }
            o
        };
        // The forms' rail slopes over the band: grown from the solve's and
        // checked.
        let y_s: Vec<f64> = self.forms.iter().map(|f| f.slope.abs()).collect();
        let floor = 1e-12 * (1.0 + y_s.iter().fold(0.0f64, |a, v| a.max(*v)));
        let mut y: Vec<f64> = y_s.iter().map(|v| v + floor).collect();
        for _ in 0..4 {
            let delta = self.deltas(b, &y)?;
            y = apply_q(&self.forms, &owner, &delta, &y_s, &y).iter().map(|v| v + floor).collect();
        }
        let y0: Vec<f64> = y.iter().map(|v| 1.25 * v + floor).collect();
        let delta = self.deltas(b, &y0)?;
        let y1 = apply_q(&self.forms, &owner, &delta, &y_s, &y0);
        if !y1.iter().zip(&y0).all(|(a, z)| a < z) {
            return None;
        }
        // Each element's second-order injection over the band, its ranges
        // from `y0` and its rates from `y1`.
        let mut eta = Vec::with_capacity(self.elems.len());
        let mut junction_at = Vec::new();
        for el in &self.elems {
            match &el.kind {
                ElemKind::Junction(law) => {
                    let f = el.forms[0];
                    let (v0, span) = (self.forms[f].c0, y0[f] * b);
                    let (lo, hi) = (v0 - span, v0 + span);
                    let g_hi = slope(law, hi);
                    eta.push(g_hi / law.vtn * y1[f] * y1[f]);
                    let i_min = if lo >= 0.0 {
                        law.current(lo)
                    } else if hi <= 0.0 {
                        -law.current(hi)
                    } else {
                        0.0
                    };
                    junction_at.push((g_hi, i_min));
                }
                ElemKind::Driver(curve) => {
                    let (fw, fs) = (el.forms[0], el.forms[1]);
                    let (vs0, span) = (self.forms[fs].c0, y0[fs] * b);
                    let (_, _, _, g1, g2) = curve_range(curve, vs0, vs0 - span, vs0 + span)?;
                    let w_max = self.forms[fw].c0.abs() + y0[fw] * b;
                    eta.push(g2 * y1[fs] * y1[fs] * w_max + 2.0 * g1 * y1[fs] * y1[fw]);
                }
            }
        }
        // `q = q0 + Q q`, solved and checked.
        let nf = self.forms.len();
        let q0: Vec<f64> = self.forms.iter().map(|f| f.t.iter().zip(&eta).map(|(t, h)| t.abs() * h).sum::<f64>()).collect();
        let mut q = q0.clone();
        if nf > 0 {
            let mut a = vec![0.0; nf * nf];
            for (r, f) in self.forms.iter().enumerate() {
                a[r * nf + r] += 1.0;
                for c in 0..nf {
                    a[r * nf + c] -= f.t[owner[c]].abs() * delta[c];
                }
            }
            lu::lu_factor(a, nf).ok()?.solve(&mut q).ok()?;
            q = q.iter().map(|v| v * (1.0 + 1e-9) + 1e-300).collect();
            let image = apply_q(&self.forms, &owner, &delta, &q0, &q);
            if !q.iter().zip(&image).all(|(v, im)| *v >= 0.0 && im <= v) {
                return None;
            }
        }
        // What each element injects at most, its curvature and its
        // Jacobian's move against `x''` together.
        let zeta: Vec<f64> = self.elems.iter().zip(&eta).map(|(el, h)| h + el.forms.iter().map(|&f| delta[f] * q[f]).sum::<f64>()).collect();
        let through = |t: &[f64]| t.iter().zip(&zeta).map(|(a, z)| a.abs() * z).sum::<f64>();
        let k_node = (0..self.x.len()).map(|l| self.elems.iter().zip(&zeta).map(|(el, z)| el.mp[l].abs() * z).sum::<f64>()).collect();
        let k_junction = self.junctions.iter().zip(&junction_at).map(|(&j, &(g_hi, i_min))| (through(&self.forms[self.elems[j].forms[0]].t), g_hi, i_min)).collect();
        let k_branch = self.branches.iter().map(|br| through(&br.t)).collect();
        // The draw is the rail's row: `R x''` and its own elements' share.
        let k_draw = through(&self.draw_t) + self.elems.iter().zip(&zeta).map(|(el, z)| el.r.abs() * z).sum::<f64>();
        Some(SupplyEval { k_node, k_junction, k_branch, k_draw })
    }

    fn valid(&self, b: f64) -> bool {
        let Some(ev) = self.eval(b) else { return false };
        let e = 0.5 * b * b;
        let ok_v = self.x.iter().zip(&self.s).zip(&ev.k_node).all(|((x, s), k)| {
            let err = k * e;
            err <= V_ABS + REL * (x.abs() - s.abs() * b - err).max(0.0)
        });
        let ok_j = ev.k_junction.iter().all(|&(k, g_hi, i_min)| g_hi * k * e <= A_ABS + REL * i_min);
        let ok_b = self.branches.iter().zip(&ev.k_branch).all(|(br, k)| {
            let err = k * e;
            err <= A_ABS + REL * (br.i.abs() - br.di.abs() * b - err).max(0.0)
        });
        let ed = ev.k_draw * e;
        let ok_d = ed <= A_ABS + REL * (self.draw.0.abs() - self.draw.1.abs() * b - ed).max(0.0);
        ok_v && ok_j && ok_b && ok_d
    }

    /// The widest band, at most `RAIL_RESOLVE_V`, inside W0's tolerance;
    /// where none is, the narrowest.
    pub(crate) fn band(&self) -> f64 {
        let cap = rails::RAIL_RESOLVE_V;
        if self.valid(cap) {
            return cap;
        }
        let (mut lo, mut hi) = (0.0, cap);
        for _ in 0..BAND_STEPS {
            let mid = 0.5 * (lo + hi);
            if self.valid(mid) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        lo.max(NARROWEST_BAND)
    }
}

/// The band of a hub partition holding a free IC supply at its solve (`x`
/// at rail `u`, `sens` taken with the exact Jacobian, its `jac` that
/// Jacobian, `ic` its IC stamps). Always `Some`: where the bound cannot be
/// formed the band is the narrowest.
#[allow(clippy::too_many_arguments)]
pub(crate) fn supply_band(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, bnd: &Bnd, x: &[f64], u: f64, sens: &RailSens, ic: &IcSupply) -> Option<f64> {
    let bound = supply_bound(shape, live_params, bnd, x, u, sens, ic);
    #[cfg(test)]
    hook::record_supply(&shape.nodes, u, bound.clone());
    Some(bound.map_or(NARROWEST_BAND, |b| b.band()))
}

#[allow(clippy::too_many_arguments)]
fn supply_bound(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, bnd: &Bnd, x: &[f64], u: f64, sens: &RailSens, ic: &IcSupply) -> Option<SupplyBound> {
    let n = shape.n;
    if n == 0 {
        return None;
    }
    let size = n + 1;
    let mut z = x.to_vec();
    z.push(u);
    let mut jxx = vec![0.0; n * n];
    for r in 0..n {
        jxx[r * n..(r + 1) * n].copy_from_slice(&sens.jac[r * size..r * size + n]);
    }
    let lu_j = lu::lu_factor(jxx, n).ok()?;
    let end = |p: &crate::solve::DynPin| match bnd.local(p) {
        Some(l) => End::Z(l),
        None => End::Fixed(bnd.get(p.node)),
    };
    // The curved elements, their rows and forms.
    let mut elems: Vec<Elem> = Vec::new();
    let mut forms: Vec<Form> = Vec::new();
    let mut junctions = Vec::new();
    let mut push = |kind: ElemKind, rows: [(End, f64); 2], fs: Vec<Form>| -> Option<usize> {
        let mut p = vec![0.0; n];
        let mut r = 0.0;
        for (e, sign) in rows {
            match e {
                End::Z(i) if i < n => p[i] += sign,
                End::Z(_) => r += sign,
                End::Fixed(_) => {}
            }
        }
        lu_j.solve(&mut p).ok()?;
        if !p.iter().all(|v| v.is_finite()) {
            return None;
        }
        let ids = (forms.len()..forms.len() + fs.len()).collect();
        forms.extend(fs);
        elems.push(Elem { kind, mp: p, r, forms: ids });
        Some(elems.len() - 1)
    };
    for j in &shape.junctions {
        let (a, k) = (end(&j.anode), end(&j.cathode));
        junctions.push(push(ElemKind::Junction(j.law), [(a, 1.0), (k, -1.0)], vec![Form::of(&[(a, 1.0), (k, -1.0)], &z, n)])?);
    }
    for d in &ic.drivers {
        let vs = Form::of(&[(d.vcc, 1.0), (d.gnd, -1.0)], &z, n);
        if vs.x.is_empty() && vs.slope == 0.0 {
            continue;
        }
        let w = Form::of(&[(d.a, 1.0), (d.f, -1.0)], &z, n);
        push(ElemKind::Driver(d.curve), [(d.a, 1.0), (d.f, -1.0)], vec![w, vs])?;
    }
    // A linear form's rail slope at the solve and its reading of each
    // element's `M P_e`.
    let read = |x_form: &[(usize, f64)], elems: &[Elem]| -> Vec<f64> { elems.iter().map(|el| x_form.iter().map(|&(i, c)| c * el.mp[i]).sum()).collect() };
    let finish = |f: &mut Form, elems: &[Elem]| {
        f.slope += f.x.iter().map(|&(i, c)| c * sens.s[i]).sum::<f64>();
        f.t = read(&f.x, elems);
    };
    for f in forms.iter_mut() {
        finish(f, &elems);
    }
    let pin = |p: &crate::solve::DynPin| -> (f64, f64, Option<usize>) {
        match bnd.local(p) {
            Some(l) if l < n => (x[l], sens.s[l], Some(l)),
            Some(_) => (u, 1.0, None),
            None => (bnd.get(p.node), 0.0, None),
        }
    };
    let mut branches = Vec::new();
    for st in &shape.stamped {
        let (mut i, mut di, mut form) = (0.0, 0.0, Vec::new());
        for c in crate::devices::stamp(st.kind, st.pins.len(), live_params_for(st, live_params)) {
            let (va, sa, la) = pin(&st.pins[c.pin_a]);
            let (vb, sb, lb) = pin(&st.pins[c.pin_b]);
            i += c.conductance * (va - vb);
            di += c.conductance * (sa - sb);
            form.extend(la.map(|l| (l, c.conductance)));
            form.extend(lb.map(|l| (l, -c.conductance)));
        }
        branches.push(SupplyBranch { i, di, t: read(&form, &elems), device: Some(st.device_index) });
    }
    for cap in &shape.capacitors {
        if cap.leak_g != 0.0 {
            let (_, sa, la) = pin(&cap.pins[0]);
            let (_, sb, lb) = pin(&cap.pins[1]);
            let mut form = Vec::new();
            form.extend(la.map(|l| (l, cap.leak_g)));
            form.extend(lb.map(|l| (l, -cap.leak_g)));
            branches.push(SupplyBranch { i: 0.0, di: cap.leak_g * (sa - sb), t: read(&form, &elems), device: None });
        }
    }
    let r_form: Vec<(usize, f64)> = sens.jac[n * size..n * size + n].iter().copied().enumerate().collect();
    let draw_t = read(&r_form, &elems);
    Some(SupplyBound { s: sens.s.clone(), x: x.to_vec(), elems, forms, junctions, branches, draw_t, draw: (sens.i, sens.g) })
}

/// The bound the latest band was sized from, for tests that hold a
/// partition's true error against it.
#[cfg(test)]
pub(crate) mod hook {
    use super::*;
    use std::cell::RefCell;

    #[derive(Clone)]
    pub(crate) struct Last {
        pub u_p: f64,
        bound: Option<Bound>,
        supply: Option<SupplyBound>,
    }

    thread_local! {
        /// The latest bound of each partition of the board loaded last on
        /// this thread, by the partition's nodes. A load clears it (V8: a
        /// stale entry of an earlier board holding the same node answered for
        /// a partition that sized no bound, so a bound check on it was
        /// vacuous).
        static LAST: RefCell<Vec<(Vec<usize>, Last)>> = const { RefCell::new(Vec::new()) };
    }

    /// A board loads: every partition's bound is its own from here on.
    pub(crate) fn clear() {
        LAST.with(|l| l.borrow_mut().clear());
    }

    fn put(nodes: &[usize], last: Last) {
        LAST.with(|l| {
            let mut l = l.borrow_mut();
            l.retain(|(n, _)| n[..] != nodes[..]);
            l.push((nodes.to_vec(), last));
        });
    }

    pub(crate) fn record(nodes: &[usize], u_p: f64, bound: Option<Bound>) {
        put(nodes, Last { u_p, bound, supply: None });
    }

    pub(crate) fn record_supply(nodes: &[usize], u_p: f64, supply: Option<SupplyBound>) {
        put(nodes, Last { u_p, bound: None, supply });
    }

    /// The latest bound of the partition holding `node`, on the board
    /// loaded last.
    pub(crate) fn of(node: usize) -> Option<Last> {
        LAST.with(|l| l.borrow().iter().rev().find(|(n, _)| n.contains(&node)).map(|(_, last)| last.clone()))
    }

    impl Last {
        /// The rule its draw was checked by (a rail-supplied bound's).
        pub(crate) fn draw_rule(&self) -> Option<DrawRule> {
            self.bound.as_ref().map(|b| b.draw_rule)
        }

        /// Whether a bound was formed (none: the narrowest band).
        pub(crate) fn bounded(&self) -> bool {
            self.bound.is_some() || self.supply.is_some()
        }

        /// The bound on a node's (and a junction voltage's) error at `|u -
        /// u_p| = b`.
        pub(crate) fn node(&self, b: f64) -> f64 {
            if let Some(sb) = self.supply.as_ref() {
                return sb.eval(b).map_or(f64::INFINITY, |e| e.k_node.iter().fold(0.0f64, |a, k| a.max(*k)) * 0.5 * b * b);
            }
            self.bound.as_ref().and_then(|bd| bd.eval(b)).map_or(f64::INFINITY, |e| e.k_v * 0.5 * b * b)
        }

        /// The bound on device `device`'s published current's error, and
        /// whether it was bounded as a branch in series with junctions.
        pub(crate) fn branch(&self, device: usize, b: f64) -> (f64, bool) {
            if let Some(sb) = self.supply.as_ref() {
                let k = sb.branches.iter().position(|br| br.device == Some(device)).expect("a branch of the partition");
                return (sb.eval(b).map_or(f64::INFINITY, |e| e.k_branch[k] * 0.5 * b * b), false);
            }
            let Some(bd) = self.bound.as_ref() else { return (f64::INFINITY, false) };
            let br = bd.branches.iter().find(|br| br.device == Some(device)).expect("a branch of the partition");
            (bd.eval(b).map_or(f64::INFINITY, |e| bd.branch_err(&e, br, b)), br.series.is_some())
        }

        /// The bound on junction `j`'s published current's error.
        pub(crate) fn junction(&self, j: usize, b: f64) -> f64 {
            if let Some(sb) = self.supply.as_ref() {
                return sb.eval(b).map_or(f64::INFINITY, |e| e.k_junction[j].1 * e.k_junction[j].0 * 0.5 * b * b);
            }
            self.bound.as_ref().and_then(|bd| bd.eval(b).map(|e| bd.junction_err(&e, j, b))).unwrap_or(f64::INFINITY)
        }

        /// The bound on the draw's error.
        pub(crate) fn draw(&self, b: f64) -> f64 {
            if let Some(sb) = self.supply.as_ref() {
                return sb.eval(b).map_or(f64::INFINITY, |e| e.k_draw * 0.5 * b * b);
            }
            self.bound.as_ref().and_then(|bd| bd.eval(b)).map_or(f64::INFINITY, |e| e.k_d * 0.5 * b * b)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn led() -> JunctionLaw {
        JunctionLaw::at(2.0, 0.02, 2.0, None, 25.0)
    }

    #[test]
    fn the_suprema_are_the_peaks_inside_and_the_ends_outside() {
        // Within rounding: an optimised build folds `powi` at compile time,
        // an ulp away from the product the code forms (V5 finding 5).
        let near = |got: f64, want: f64| assert!((got - want).abs() <= 4.0 * f64::EPSILON * want.abs(), "{got} against {want}");
        near(phi_sup(0.1, 0.2), 0.2 / 1.2f64.powi(3));
        near(phi_sup(1.0, 2.0), 1.0 / 8.0);
        near(phi_sup(0.1, 2.0), 4.0 / 27.0);
        near(phi_sup(0.1, f64::INFINITY), 4.0 / 27.0);
        near(psi_sup(1e-3, 1e-2, 10.0), 1e-2 / 1.1f64.powi(3));
        near(psi_sup(1.0, 2.0, 10.0), 1.0 / 11.0f64.powi(3));
        near(psi_sup(1e-3, 1.0, 10.0), 4.0 / 270.0);
        near(psi_sup(1e-3, 1.0, 0.0), 1.0);
    }

    /// The LED and `r` from the rail to ground, solved for the cathode `m`
    /// by bisection: `(m, I)`.
    fn branch(u: f64, r: f64) -> (f64, f64) {
        let law = led();
        let (mut lo, mut hi) = (0.0, u.max(0.0));
        for _ in 0..200 {
            let m = 0.5 * (lo + hi);
            if law.current(u - m) > m / r {
                lo = m;
            } else {
                hi = m;
            }
        }
        let m = 0.5 * (lo + hi);
        (m, m / r)
    }

    #[test]
    fn the_bound_holds_against_the_true_remainder_across_the_knee() {
        // An LED and 330 ohm across the rail, solved exactly at u_p and
        // anywhere in its band: the first-order correction's true error in
        // the cathode voltage, the resistor's current and the LED's
        // published current stays inside the bound's own tolerance at every
        // u_p from off to far past the knee, approached from either side.
        let (r, law) = (330.0, led());
        let d = 1e-7;
        let mut worst: f64 = 0.0;
        for k in 0..400 {
            let u_p = 1.0 + 0.005 * k as f64;
            let (m, i) = branch(u_p, r);
            let s = (branch(u_p + d, r).0 - branch(u_p - d, r).0) / (2.0 * d);
            let v = u_p - m;
            let alpha = (1.0 - s) * (1.0 + r * slope(&law, v));
            let port = Port::single(law, v, 1.0 - s, r, alpha);
            let br = Branch { i, di: s / r, g: 1.0 / r, contacts: 1.0, series: None, device: None };
            let b = band(&[port], &[(m, s)], &[br], (i, s / r));
            for j in 1..=20 {
                for sign in [-1.0, 1.0] {
                    let du = sign * b * j as f64 / 20.0;
                    let (mt, it) = branch(u_p + du, r);
                    let m_lin = m + s * du;
                    let ev = (m_lin - mt).abs() / (V_ABS + REL * mt.abs());
                    let ea = (m_lin / r - it).abs() / (A_ABS + REL * it.abs());
                    let ej = (law.current(u_p + du - m_lin) - it).abs() / (A_ABS + REL * it.abs());
                    worst = worst.max(ev).max(ea).max(ej);
                }
            }
        }
        assert!(worst <= 1.0, "the worst error is {worst} of its tolerance");
    }

    #[test]
    fn a_band_narrower_than_the_cc_one_is_not_widened() {
        // V5 finding 4. A port whose curvature allows well under the CC
        // band (RAIL_CC_RESOLVE_V, 1 uV): the band stays where the bound
        // allows, so its own bound holds there, and is not clamped up to
        // 1 uV, where the bound is a hundred times W0's tolerance.
        let law = led();
        let r = 330.0;
        let v = 1.7;
        let port = Port::single(law, v, 1e5, r, 1e5 * (1.0 + r * slope(&law, v)));
        let x = 2.0;
        let b = band(&[port], &[(x, 0.5)], &[], (1e-3, 1e-3));
        assert!(b < rails::RAIL_CC_RESOLVE_V, "the band {b:e}");
        let k_v = port.bound(b).k_v;
        assert!(k_v * 0.5 * b * b <= V_ABS + REL * x, "the bound at the band: {:e}", k_v * 0.5 * b * b);
    }

    #[test]
    fn a_partition_whose_bound_cannot_be_formed_takes_the_narrowest_band() {
        // V5 finding 4. An LED from the rail (node 1) to node 2, whose
        // Jacobian at the solve is its own slope alone: the rest of the
        // partition does not factor, so no bound exists. The band is the
        // narrowest, which re-solves at every move of the rail, never the
        // flat fallback.
        use crate::solve::{DynJunction, DynPin};
        let law = led();
        let (u, m) = (5.0, 3.0);
        let shape = DynShape { nodes: vec![2], n: 1, stamped: vec![], capacitors: vec![], junctions: vec![DynJunction { anode: DynPin { local: None, node: 1 }, cathode: DynPin { local: Some(0), node: 2 }, law }] };
        let voltages = vec![0.0, u, m];
        let bnd = Bnd::new(&voltages).with_ext(1, 1);
        let g = slope(&law, u - m);
        let sens = RailSens { i: law.current(u - m), g, s: vec![1.0], jac: vec![g, -g, -g, g] };
        let band = hub_band(&shape, &HashMap::new(), &bnd, &[m], u, &sens, &IcRail::default());
        assert_eq!(band, Some(NARROWEST_BAND));
    }
}
