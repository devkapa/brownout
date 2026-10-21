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

/// The most assumed sensitivity bounds `Bound::reach` tries, each at least
/// twice the last.
const REACH_TRIES: u32 = 6;

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
    /// Its share of the solve's rounding floor ([`Floors`]), `(at the
    /// solve, per volt of rail)`.
    pub floor: (f64, f64),
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
    /// The solve's rounding floor, added to every quantity's second-order
    /// remainder (the branches carry theirs).
    floor: Floors,
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
        Bound { ports: ports.to_vec(), a_p: ports.iter().map(|p| p.alpha).collect(), groups: Vec::new(), drivers: Vec::new(), s_max: nodes.iter().map(|n| n.1.abs()).fold(0.0, f64::max), nodes: nodes.to_vec(), branches: branches.to_vec(), series: Vec::new(), draw, draw_rule: DrawRule::General, b_max: rails::RAIL_RESOLVE_V, floor: Floors::default() }
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
    /// holds, since `|x'|` could not first reach `m0` (it starts at the
    /// solve's `|s| <= s_max`, under every `m0` tried). Without drivers,
    /// `(1, 1)`.
    ///
    /// `m0` starts at twice the larger of `s_max` and 1, and where that does
    /// not close, at twice what it yielded, a few times: a driven node held
    /// far past its supply (a gate's output pulled to twice its rail) has
    /// terminals past 2 at any band, so a fixed 2 failed at every `b`, and
    /// the partition took the narrowest band, re-solving at every move of
    /// the rail (FXH, W1gV4's frozen seed 84: on a CC capacitor ramp, an
    /// event every femtosecond).
    fn reach(&self, b: f64, ranges: &[(f64, f64, f64, f64)]) -> Option<(f64, f64)> {
        let mut m0 = 2.0 * self.s_max.max(1.0);
        for _ in 0..REACH_TRIES {
            match self.reach_at(b, ranges, m0) {
                Ok(r) => return Some(r),
                Err(m_x) if m_x.is_finite() => m0 = (2.0 * m_x).max(2.0 * m0),
                Err(_) => return None,
            }
        }
        None
    }

    /// [`Self::reach`] under the assumed bound `m0`: the bound, or `Err` with
    /// what it yielded where that is not below `m0`.
    fn reach_at(&self, b: f64, ranges: &[(f64, f64, f64, f64)], m0: f64) -> Result<(f64, f64), f64> {
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
        if m_x < m0 {
            Ok((m_x, rho_hi - rho_lo))
        } else {
            Err(m_x)
        }
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
        let (fv, fi) = self.floor.junction(j, b);
        by_voltage.min(by_current) + pb.g_hi * fv + fi
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
        err + at_b(br.floor, b)
    }

    /// Whether every published quantity's bound over `|u - u_p| <= b` is
    /// inside W0's tolerance at its smallest magnitude over the band.
    fn valid(&self, b: f64) -> bool {
        let Some(ev_b) = self.eval(b) else { return false };
        let e = 0.5 * b * b;
        let (ev, ed) = (ev_b.k_v * e, ev_b.k_d * e);
        let ok_v = self.nodes.iter().enumerate().all(|(l, &(x, s))| {
            let err = ev + self.floor.node(l, b);
            err <= V_ABS + REL * (x.abs() - s.abs() * b - err).max(0.0)
        });
        let ok_j = ev_b.ports.iter().enumerate().all(|(j, pb)| self.junction_err(&ev_b, j, b) <= A_ABS + REL * pb.i_min);
        let ok_b = self.branches.iter().all(|br| {
            let err = self.branch_err(&ev_b, br, b);
            err <= A_ABS + REL * (br.i.abs() - br.di.abs() * b - err).max(0.0)
        });
        let ok_d = match self.draw_rule {
            DrawRule::Zero => true,
            DrawRule::ShuntOnly => crate::solve::static_part::NODE_RSHUNT_G * self.nodes.len() as f64 * ev + self.floor.draw(b) <= A_ABS,
            DrawRule::General => {
                let ed = ed + self.floor.draw(b);
                ed <= A_ABS + REL * (self.draw.0.abs() - self.draw.1.abs() * b - ed).max(0.0)
            }
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

/// The ulps of a row's gross terms one residual evaluation rounds by
/// ([`Floors`]; `static_part::step_rounding` takes the same).
pub(crate) const ROUND_ULPS: f64 = 4.0;

/// The margin on the solve's own part of a floor: it is first order in the
/// distance, through `J^-1` as floating point factors it, which a
/// GMIN-held cluster beside a 1e-3 ohm resistor holds to about ten percent
/// (the cluster's 1e-12 S sits a few ulps under its diagonal's 1e3 S):
/// k8 s1017 published 1.0003 of a floor without it.
const SOLVE_MARGIN: f64 = 2.0;

/// The part of W0's absolute tolerances below which no floor is resolved:
/// a quantity that rounds to exactly 0 (a node every element at which
/// carries no current in floating point) can sit 1e-62 A from the solution
/// with no row to round, and 1e-30 of the tolerance (1e-34 V, 1e-39 A) is
/// beyond anything a band can see.
const FLOOR_ABS: f64 = 1e-30;

/// A hub solve's rounding floor (ruling R23 (i), FXE 2026-10-21): how far
/// each published quantity can sit from the solution's through the solve's
/// rounding, at the solve and per volt the rail moves. The band's
/// second-order bound measures the correction from the solution, so the
/// solve's own distance from it is added. A solve at `x` with residual `r`
/// (as computed) is `J^-1 r*` from the solution to first order (`J` the
/// exact Jacobian at `x`, `r*` the true residual), and `r*` is within the
/// residual's rounding of `r`: a few ulps of each row's gross terms (the
/// stamp's), or, after the refinement, of each element's current (kept as
/// a pair of rows, so that what a stiff element carries between two nodes
/// moves them together by little); its sensitivity `s = -J^-1 F_u`,
/// solved through the same factors, is within `|J^-1|` of a few ulps of
/// `|F_u| + |J| |s|` (the right-hand side's rounding and the solve's
/// backward error). So a quantity linear in the voltages through the row
/// `L` is within `|L J^-1 r| + sum |L J^-1 e_k| rho_k` of the solution's,
/// plus `b` times its slope's part at `|u - u_p| = b`. Each adds the
/// rounding of its own publication: a node its value's and its
/// correction's, a branch its contacts' conductance over both ends' and a
/// junction its law's evaluation. Unlike the second-order bound, it does
/// not vanish with the band, so a floor near W0's tolerance narrows the
/// band, to the narrowest where it alone passes it. GMIN-held floating
/// clusters (an unpowered gate behind a resistor, nodes between off LEDs
/// and an open switch) sat up to 1.8e-6 V from the solution against a bound
/// of 1.2e-14 V (FXDv).
#[derive(Debug, Clone, Default)]
pub(crate) struct Floors {
    /// Each local node's `(at the solve, per volt of rail)`.
    node: Vec<(f64, f64)>,
    /// Each junction's voltage's (times the slope over the band) and its
    /// law's own evaluation.
    junction: Vec<((f64, f64), f64)>,
    /// Each stamped device's current's, in `shape.stamped` order.
    stamped: Vec<(f64, f64)>,
    /// Each capacitor's leakage current's, in `shape.capacitors` order.
    leak: Vec<(f64, f64)>,
    draw: (f64, f64),
    /// The largest floor at the solve against W0's tolerance for its
    /// quantity (the refinement's gate, `static_part::solve_hub`).
    pub worst_w0: f64,
}

/// A floor `(at the solve, per volt)` at half-width `b`.
fn at_b(f: (f64, f64), b: f64) -> f64 {
    f.0 + f.1 * b
}

impl Floors {
    pub(crate) fn node(&self, l: usize, b: f64) -> f64 {
        self.node.get(l).map_or(0.0, |&f| at_b(f, b))
    }

    /// Junction `j`'s voltage floor and its law's evaluation.
    fn junction(&self, j: usize, b: f64) -> (f64, f64) {
        self.junction.get(j).map_or((0.0, 0.0), |&(v, i)| (at_b(v, b), i))
    }

    fn draw(&self, b: f64) -> f64 {
        at_b(self.draw, b)
    }
}

/// [`Floors`] of a hub solve at `x` (rail `u`, sensitivity `sens`): `lu`
/// factors the exact Jacobian at `x` (`sens.jac`'s first `n` rows and
/// columns), `r` is the residual there (signed: `J^-1 r` is the solve's
/// distance from the solution to first order, so a residual a pair of
/// nodes carries equal and opposite, as a stiff resistor between two
/// nodes whose difference floating point cannot hold does, moves them
/// together by little), and its rounding is `rho` on each row (the stamp's
/// gross terms) and `pairs` on each element (`(row, row, rounding)`, the
/// element residual's, which keeps the same cancellation). The draw is the
/// rail row's residual (`sens.gross`'s last entry the gross terms it
/// rounds by) and its slope that row through `s` (`sens.gross_s`'s). `n +
/// 1` solves through `lu`.
#[allow(clippy::too_many_arguments)]
pub(crate) fn floors(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, bnd: &Bnd, x: &[f64], u: f64, sens: &RailSens, lu: &lu::LuFactorization, r: &[f64], rho: &[f64], pairs: &[(Option<usize>, Option<usize>, f64)]) -> Result<Floors, String> {
    let n = shape.n;
    let size = n + 1;
    let eps = f64::EPSILON;
    let ulps = ROUND_ULPS * eps;
    // A junction's companion offset `id - gd v` rounds by ulps of `is
    // e^(v/vtn) = gd vtn` (its exponential less one cancels near 0 V), which
    // its own size, in the gross terms, can sit far below: each row it
    // touches, the rail's included, takes that too.
    let mut junction_rows = vec![0.0f64; size];
    for j in &shape.junctions {
        let v = pin_v(&j.anode, x, u, bnd) - pin_v(&j.cathode, x, u, bnd);
        let (g, _) = j.law.companion(v);
        for l in [bnd.local(&j.anode), bnd.local(&j.cathode)].into_iter().flatten() {
            junction_rows[l] += ulps * g * (j.law.vtn + v.abs());
        }
    }
    let w: Vec<f64> = rho.iter().zip(&junction_rows).map(|(a, b)| a + b).collect();
    let w1: Vec<f64> = sens.gross_s[..n].iter().map(|g| ulps * g).collect();
    let singular = |_| "rounding floor: singular or non-finite pivot".to_string();
    // `dist` is `J^-1 r`, the signed distance.
    let mut dist = r.to_vec();
    lu.solve(&mut dist).map_err(singular)?;
    let est = if n >= FLOW_MIN_N && conductance_matrix(&sens.jac, n) {
        // `J` is a conductance network: a symmetric Laplacian plus a
        // nonnegative diagonal (its conductances to fixed nodes, the rail and
        // the node shunts), so `J^-1 >= 0` and a unit current injected
        // anywhere flows downhill to those sinks, no branch or sink carrying
        // more than all of it. Then `|J^-1| w = J^-1 w` exactly (one solve),
        // and a quantity's share of the rounding injections is at most its
        // flow multiplier times their sum. The rate's rounding takes a solve
        // of its own: folded into the solve's at the widest band, it would
        // hold a GMIN-held cluster beside a stiff resistor at the narrowest
        // band at every band.
        let mut v = w.clone();
        let mut total: f64 = w.iter().sum();
        for &(pa, pb, q) in pairs {
            for k in [pa, pb].into_iter().flatten() {
                v[k] += q;
            }
            total += q;
        }
        lu.solve(&mut v).map_err(singular)?;
        let mut v1 = w1.clone();
        let total1: f64 = w1.iter().sum();
        lu.solve(&mut v1).map_err(singular)?;
        #[cfg(test)]
        crate::solve::newton::hooks::note_floor_solves(3);
        Estimate::Flow { v, v1, total, total1 }
    } else {
        // `cols[k]` is `J^-1 e_k`.
        let mut cols: Vec<Vec<f64>> = Vec::with_capacity(n);
        for k in 0..n {
            let mut c = vec![0.0; n];
            c[k] = 1.0;
            lu.solve(&mut c).map_err(singular)?;
            cols.push(c);
        }
        #[cfg(test)]
        crate::solve::newton::hooks::note_floor_solves(n as u64 + 1);
        Estimate::Columns { cols }
    };
    // A quantity linear in the voltages through `l`, whose share of a unit
    // injection's flow is at most `flow` (a node's is unbounded).
    let form = |l: &[(usize, f64)], flow: f64| -> (f64, f64) {
        let at = |c: &[f64]| l.iter().map(|&(i, g)| g * c[i]).sum::<f64>();
        let (mut f0, mut f1) = (at(&dist).abs(), 0.0);
        match &est {
            Estimate::Flow { v, v1, total, total1 } => {
                let abs_at = |c: &[f64]| l.iter().map(|&(i, g)| g.abs() * c[i].abs()).sum::<f64>();
                f0 += abs_at(v).min(flow * total);
                f1 += abs_at(v1).min(flow * total1);
            }
            Estimate::Columns { cols } => {
                for (k, c) in cols.iter().enumerate() {
                    let a = at(c).abs();
                    f0 += a * w[k];
                    f1 += a * w1[k];
                }
                for &(pa, pb, q) in pairs {
                    let a = pa.map_or(0.0, |k| at(&cols[k])) - pb.map_or(0.0, |k| at(&cols[k]));
                    f0 += a.abs() * q;
                }
            }
        }
        (SOLVE_MARGIN * f0, SOLVE_MARGIN * f1)
    };
    // A pin's value, rail slope and local row.
    let pin = |p: &crate::solve::DynPin| -> (f64, f64, Option<usize>) {
        match bnd.local(p) {
            Some(l) if l < n => (x[l], sens.s[l], Some(l)),
            Some(_) => (u, 1.0, None),
            None => (bnd.get(p.node), 0.0, None),
        }
    };
    let mut worst = 0.0f64;
    let node: Vec<(f64, f64)> = (0..n)
        .map(|l| {
            let (f0, f1) = form(&[(l, 1.0)], f64::INFINITY);
            (f0 + 2.0 * eps * x[l].abs() + FLOOR_ABS * V_ABS, f1 + 2.0 * eps * sens.s[l].abs())
        })
        .collect();
    for (l, f) in node.iter().enumerate() {
        worst = worst.max(f.0 / (V_ABS + REL * x[l].abs()));
    }
    let mut junction = Vec::with_capacity(shape.junctions.len());
    for j in &shape.junctions {
        let ((va, sa, la), (vk, sk, lk)) = (pin(&j.anode), pin(&j.cathode));
        let mut l = Vec::with_capacity(2);
        l.extend(la.map(|a| (a, 1.0)));
        l.extend(lk.map(|k| (k, -1.0)));
        let v = va - vk;
        let (g, _) = j.law.companion(v);
        let i = j.law.current(v);
        // A flow across the junction is at most all of it, through at
        // least its own conductance in `J`.
        let (f0, f1) = form(&l, 1.0 / g);
        let fv = (f0 + eps * (va.abs() + vk.abs()), f1 + eps * (sa.abs() + sk.abs()));
        // The law's evaluation: its exponential less one cancels near 0 V,
        // so it rounds by ulps of `is e^(v/vtn) = gd vtn` as well as of `I`.
        let fi = ulps * (i.abs() + g * (j.law.vtn + v.abs())) + FLOOR_ABS * A_ABS;
        worst = worst.max((g * fv.0 + fi) / (A_ABS + REL * i.abs()));
        junction.push((fv, fi));
    }
    let mut branch = |contacts: &[(crate::solve::DynPin, crate::solve::DynPin, f64)]| -> (f64, f64) {
        let mut l: Vec<(usize, f64)> = Vec::new();
        let (mut i, mut own0, mut own1) = (0.0, 0.0, 0.0);
        for (a, b, g) in contacts {
            let ((va, sa, la), (vb, sb, lb)) = (pin(a), pin(b));
            l.extend(la.map(|a| (a, *g)));
            l.extend(lb.map(|b| (b, -*g)));
            i += g * (va - vb);
            own0 += 2.0 * eps * g.abs() * (va.abs() + vb.abs());
            own1 += 2.0 * eps * g.abs() * (sa.abs() + sb.abs());
        }
        let (f0, f1) = form(&l, contacts.len() as f64);
        let f0 = f0 + own0 + FLOOR_ABS * A_ABS;
        worst = worst.max(f0 / (A_ABS + REL * i.abs()));
        (f0, f1 + own1)
    };
    let mut contacts = Vec::new();
    let stamped: Vec<(f64, f64)> = shape
        .stamped
        .iter()
        .map(|st| {
            contacts.clear();
            contacts.extend(crate::devices::stamp(st.kind, st.pins.len(), live_params_for(st, live_params)).iter().map(|c| (st.pins[c.pin_a], st.pins[c.pin_b], c.conductance)));
            branch(&contacts)
        })
        .collect();
    let leak: Vec<(f64, f64)> = shape.capacitors.iter().map(|cap| if cap.leak_g != 0.0 { branch(&[(cap.pins[0], cap.pins[1], cap.leak_g)]) } else { (0.0, 0.0) }).collect();
    let rail_row: Vec<(usize, f64)> = (0..n).map(|l| (l, sens.jac[n * size + l])).collect();
    let (f0, f1) = form(&rail_row, 1.0);
    let draw = (f0 + ulps * sens.gross[n] + junction_rows[n] + FLOOR_ABS * A_ABS, f1 + ulps * sens.gross_s[n]);
    worst = worst.max(draw.0 / (A_ABS + REL * sens.i.abs()));
    Ok(Floors { node, junction, stamped, leak, draw, worst_w0: worst })
}

/// The fewest nodes a hub takes its floor by flow ([`Estimate::Flow`]) at:
/// below, every column of `J^-1` (`n + 1` solves) costs about what the
/// partition's own solve does, and keeps each branch's own cancellation,
/// which the flow bound gives up for its total.
const FLOW_MIN_N: usize = 9;

/// How [`floors`] takes `|L J^-1|` to the rounding.
enum Estimate {
    /// A conductance network's `J^-1 w` (its pairs on both rows), `J^-1
    /// w1`, and the total rounding injected, at the solve and per volt.
    Flow { v: Vec<f64>, v1: Vec<f64>, total: f64, total1: f64 },
    /// Every column `J^-1 e_k`.
    Columns { cols: Vec<Vec<f64>> },
}

/// Whether the `n x n` block of `jac` (row stride `n + 1`) is a
/// conductance network: symmetric (to a few ulps), its off-diagonal
/// entries at most 0 and its rows summing to at least 0 (to a few ulps of
/// their terms). An IC whose supply is free stamps terms that follow its
/// supply (`IcSupply::extra_jacobian`), which are neither.
fn conductance_matrix(jac: &[f64], n: usize) -> bool {
    let size = n + 1;
    let tol = 8.0 * f64::EPSILON;
    (0..n).all(|r| {
        let row = &jac[r * size..r * size + n];
        let (mut sum, mut gross) = (0.0f64, 0.0f64);
        for (c, &a) in row.iter().enumerate() {
            if c != r {
                let t = jac[c * size + r];
                if a > 0.0 || (a - t).abs() > tol * a.abs().max(t.abs()) {
                    return false;
                }
            }
            sum += a;
            gross += a.abs();
        }
        row[r] > 0.0 && sum >= -tol * gross
    })
}

/// A pin's voltage at the solve `x` with the rail at `u` stamped as
/// unknown `x.len()`.
fn pin_v(p: &crate::solve::DynPin, x: &[f64], u: f64, bnd: &Bnd) -> f64 {
    match bnd.local(p) {
        Some(l) if l < x.len() => x[l],
        Some(_) => u,
        None => bnd.get(p.node),
    }
}

/// Each branch's floor from `floors` (by its device, which the branches
/// list in `shape.stamped`'s order, or for a leakage by its capacitor's
/// place).
fn branch_floors<'a>(shape: &DynShape, floors: &Floors, branches: impl Iterator<Item = (Option<usize>, &'a mut (f64, f64))>) {
    let mut leaks = shape.capacitors.iter().enumerate().filter(|(_, c)| c.leak_g != 0.0).map(|(k, _)| k);
    let mut k = 0;
    for (device, floor) in branches {
        *floor = match device {
            Some(d) => {
                while k < shape.stamped.len() && shape.stamped[k].device_index != d {
                    k += 1;
                }
                floors.stamped.get(k).copied().unwrap_or((0.0, 0.0))
            }
            None => leaks.next().and_then(|j| floors.leak.get(j)).copied().unwrap_or((0.0, 0.0)),
        };
    }
}

/// A curved hub partition's band at its solve (`x` at rail `u`, through
/// `sens`, whose `jac` is the Jacobian the residual stamps gave at the
/// solve, and its IC stamps that follow the rail `ic`): its junctions and
/// its rail-supplied drivers, either alone. `bnd` reads the boundary with
/// the rail stamped as unknown `n`, as the sensitivity's stamps did. Always
/// `Some`: where the bound cannot be formed (a rest that does not factor)
/// the band is the narrowest, never the flat one.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::too_many_arguments)]
pub(crate) fn hub_band(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, bnd: &Bnd, x: &[f64], u: f64, sens: &RailSens, ic: &IcRail, floors: Floors) -> Option<f64> {
    let mut bound = hub_bound(shape, live_params, bnd, x, u, sens, ic);
    if let Some(b) = bound.as_mut() {
        branch_floors(shape, &floors, b.branches.iter_mut().map(|br| (br.device, &mut br.floor)));
        b.floor = floors;
    }
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
        let probe = Bound { ports: Vec::new(), a_p: Vec::new(), groups: Vec::new(), drivers: rail_drivers(ic, x, u), s_max: sens.s.iter().fold(0.0f64, |m, s| m.max(s.abs())), nodes: Vec::new(), branches: Vec::new(), series: Vec::new(), draw: (0.0, 0.0), draw_rule: DrawRule::General, b_max: rails::RAIL_RESOLVE_V, floor: Floors::default() };
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
            branches.push(Branch { i, di, g, contacts, series: in_series, device: Some(st.device_index), floor: (0.0, 0.0) });
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
            branches.push(Branch { i: 0.0, di: cap.leak_g * (sa - sb), g: cap.leak_g, contacts: 1.0, series: None, device: None, floor: (0.0, 0.0) });
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
    Some(Bound { ports, a_p, groups, drivers, s_max, nodes, branches, series, draw: (sens.i, sens.g), draw_rule, b_max, floor: Floors::default() })
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
//
// The iterated solve passes Newton's tolerance short of the solution: a
// driver's `R_out` is restamped at the last iterate's supply, so the
// iteration converges linearly and stops up to ~1e-3 V short (FXB: two gates
// on a fed supply, precharged to 4.3 V; FXBv: 9.6e-4 V on a 167 ohm feed).
// `static_part::polish_hub` takes it on to the solution (R19 (a)),
// since this bound measures the correction from the solution and an offset
// at the solve would be outside its budget; what follows keeps the bound
// true for whatever distance remains, rounding after the polish. Short of
// the solution, `s`, exact at `x`, misses the solution's own slope
// `x*'(u_p)` by a first-order amount, which at a small `du` is all of the
// correction's error (before the polish: 1.25e-5 V/V, 126x a bound that was
// second-order alone at `du = 3e-7`). So every span is measured
// from the solve `x`, not from the solution: `d_f` bounds `|chi_f . (x* -
// x)|`, read from the residual `r(x, u_p)` (`x* - x = -M (r + dJ (x* -
// x))`, the same closure as `y`'s, the solution being the end of the path
// `r(x) = (1 - theta) r` from the solve), and a form's span over the band
// is `y b + d`. The error is then `sum_e |L M P_e| (zeta_e b^2 / 2 + z1_e
// b)`, `z1_e = sum_t delta_et(d) y_et` the elements' Jacobian move from the
// solve to the solution against `x*'(u_p) = s - M dJ z*'`.

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
    /// Its share of the solve's rounding floor ([`Floors`]).
    floor: (f64, f64),
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
    /// Each form's distance from the solve to the solution at `u_p`, `|chi
    /// . (x* - x)|` at most.
    d: Vec<f64>,
    /// The solve's rounding floor, added to every quantity's bound (the
    /// branches carry theirs).
    floor: Floors,
}

/// The bounds over one half-width: on each node's `|x''|`, each junction's
/// `|v''|` with its largest slope and smallest current, each branch's
/// `|I''|` and the draw's; and the first-order parts (`k1_*`), how far `s`
/// may miss the solution's slope in each.
pub(crate) struct SupplyEval {
    k_node: Vec<f64>,
    k_junction: Vec<(f64, f64, f64)>,
    k_branch: Vec<f64>,
    k_draw: f64,
    k1_node: Vec<f64>,
    k1_junction: Vec<f64>,
    k1_branch: Vec<f64>,
    k1_draw: f64,
}

impl SupplyEval {
    /// A quantity's error bound at `|u - u_p| = b` from its second- and
    /// first-order parts.
    fn err(k: f64, k1: f64, b: f64) -> f64 {
        k * 0.5 * b * b + k1 * b
    }
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
    /// Jacobian moves from the solve) while each form stays within `span`
    /// of its value there, or `None` where a piece ends inside.
    fn deltas(&self, span: &[f64]) -> Option<Vec<f64>> {
        let mut delta = vec![0.0; self.forms.len()];
        for el in &self.elems {
            match &el.kind {
                ElemKind::Junction(law) => {
                    let (f, v0) = (el.forms[0], self.forms[el.forms[0]].c0);
                    let g0 = slope(law, v0);
                    delta[f] = (slope(law, v0 + span[f]) - g0).max(g0 - slope(law, v0 - span[f]));
                }
                ElemKind::Driver(curve) => {
                    let (fw, fs) = (el.forms[0], el.forms[1]);
                    let vs0 = self.forms[fs].c0;
                    let (d0, dg, dg1, _, _) = curve_range(curve, vs0, vs0 - span[fs], vs0 + span[fs])?;
                    delta[fw] = dg;
                    delta[fs] = dg1 * (self.forms[fw].c0.abs() + span[fw]) + d0[1].abs() * span[fw];
                }
            }
        }
        Some(delta)
    }

    /// Each form's span over `|u - u_p| <= b` with `|chi . z'| <= y`: from
    /// the solve, which is `d` from the solution.
    fn spans(&self, b: f64, y: &[f64]) -> Vec<f64> {
        y.iter().zip(&self.d).map(|(y, d)| y * b + d).collect()
    }

    /// Each form's element.
    fn owner(&self) -> Vec<usize> {
        let mut o = vec![0; self.forms.len()];
        for (e, el) in self.elems.iter().enumerate() {
            for &f in &el.forms {
                o[f] = e;
            }
        }
        o
    }

    /// Bounds `d`, each form's `|chi . (x* - x)|`, from `m_r = M r`, the
    /// residual at the solve through the Jacobian there: `chi . (x* - x) =
    /// -chi . M (r + dJ (x* - x))`, `dJ` the elements' Jacobian move over
    /// the step. Like `y` below, one assumed `D0` whose image is below it
    /// holds (along `r(x) = (1 - theta) r` from the solve, `d` could not
    /// first reach it). `None` where none closes.
    fn bound_distance(&mut self, m_r: &[f64]) -> Option<()> {
        let d0: Vec<f64> = self.forms.iter().map(|f| f.x.iter().map(|&(i, c)| c * m_r[i]).sum::<f64>().abs()).collect();
        if d0.iter().all(|v| *v == 0.0) {
            self.d = d0;
            return Some(());
        }
        let owner = self.owner();
        let floor = 1e-15 * (1.0 + d0.iter().fold(0.0f64, |a, v| a.max(*v)));
        let mut d: Vec<f64> = d0.iter().map(|v| v + floor).collect();
        for _ in 0..4 {
            let delta = self.deltas(&d)?;
            d = apply_q(&self.forms, &owner, &delta, &d0, &d).iter().map(|v| v + floor).collect();
        }
        let big: Vec<f64> = d.iter().map(|v| 1.25 * v + floor).collect();
        let delta = self.deltas(&big)?;
        let image = apply_q(&self.forms, &owner, &delta, &d0, &big);
        if !image.iter().zip(&big).all(|(a, z)| a < z) {
            return None;
        }
        self.d = big;
        Some(())
    }

    pub(crate) fn eval(&self, b: f64) -> Option<SupplyEval> {
        if b > rails::RAIL_RESOLVE_V {
            return None;
        }
        let owner = self.owner();
        // The forms' rail slopes over the band: grown from the solve's and
        // checked.
        let y_s: Vec<f64> = self.forms.iter().map(|f| f.slope.abs()).collect();
        let floor = 1e-12 * (1.0 + y_s.iter().fold(0.0f64, |a, v| a.max(*v)));
        let mut y: Vec<f64> = y_s.iter().map(|v| v + floor).collect();
        for _ in 0..4 {
            let delta = self.deltas(&self.spans(b, &y))?;
            y = apply_q(&self.forms, &owner, &delta, &y_s, &y).iter().map(|v| v + floor).collect();
        }
        let y0: Vec<f64> = y.iter().map(|v| 1.25 * v + floor).collect();
        let span0 = self.spans(b, &y0);
        let delta = self.deltas(&span0)?;
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
                    let (v0, span) = (self.forms[f].c0, span0[f]);
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
                    let (vs0, span) = (self.forms[fs].c0, span0[fs]);
                    let (_, _, _, g1, g2) = curve_range(curve, vs0, vs0 - span, vs0 + span)?;
                    let w_max = self.forms[fw].c0.abs() + span0[fw];
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
        // What each element's Jacobian move from the solve to the solution
        // makes `s` miss `x*'(u_p)` by, `|chi . z*'|` being at most `y1`.
        let delta1 = self.deltas(&self.d)?;
        let zeta1: Vec<f64> = self.elems.iter().map(|el| el.forms.iter().map(|&f| delta1[f] * y1[f]).sum::<f64>()).collect();
        let through = |t: &[f64], zeta: &[f64]| t.iter().zip(zeta).map(|(a, z)| a.abs() * z).sum::<f64>();
        let at_node = |zeta: &[f64]| (0..self.x.len()).map(|l| self.elems.iter().zip(zeta).map(|(el, z)| el.mp[l].abs() * z).sum::<f64>()).collect::<Vec<f64>>();
        let at_junction = |zeta: &[f64]| self.junctions.iter().map(|&j| through(&self.forms[self.elems[j].forms[0]].t, zeta)).collect::<Vec<f64>>();
        let at_branch = |zeta: &[f64]| self.branches.iter().map(|br| through(&br.t, zeta)).collect::<Vec<f64>>();
        // The draw is the rail's row: `R x''` and its own elements' share.
        let at_draw = |zeta: &[f64]| through(&self.draw_t, zeta) + self.elems.iter().zip(zeta).map(|(el, z)| el.r.abs() * z).sum::<f64>();
        let k_junction = at_junction(&zeta).into_iter().zip(&junction_at).map(|(k, &(g_hi, i_min))| (k, g_hi, i_min)).collect();
        Some(SupplyEval {
            k_node: at_node(&zeta),
            k_junction,
            k_branch: at_branch(&zeta),
            k_draw: at_draw(&zeta),
            k1_node: at_node(&zeta1),
            k1_junction: at_junction(&zeta1),
            k1_branch: at_branch(&zeta1),
            k1_draw: at_draw(&zeta1),
        })
    }

    fn valid(&self, b: f64) -> bool {
        let Some(ev) = self.eval(b) else { return false };
        let err = SupplyEval::err;
        let ok_v = self.x.iter().zip(&self.s).zip(ev.k_node.iter().zip(&ev.k1_node)).enumerate().all(|(l, ((x, s), (&k, &k1)))| {
            let err = err(k, k1, b) + self.floor.node(l, b);
            err <= V_ABS + REL * (x.abs() - s.abs() * b - err).max(0.0)
        });
        let ok_j = ev.k_junction.iter().zip(&ev.k1_junction).enumerate().all(|(j, (&(k, g_hi, i_min), &k1))| {
            let (fv, fi) = self.floor.junction(j, b);
            g_hi * (err(k, k1, b) + fv) + fi <= A_ABS + REL * i_min
        });
        let ok_b = self.branches.iter().zip(ev.k_branch.iter().zip(&ev.k1_branch)).all(|(br, (&k, &k1))| {
            let err = err(k, k1, b) + at_b(br.floor, b);
            err <= A_ABS + REL * (br.i.abs() - br.di.abs() * b - err).max(0.0)
        });
        let ed = err(ev.k_draw, ev.k1_draw, b) + self.floor.draw(b);
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
pub(crate) fn supply_band(shape: &DynShape, live_params: &HashMap<usize, Vec<f64>>, bnd: &Bnd, x: &[f64], u: f64, sens: &RailSens, ic: &IcSupply, floors: Floors) -> Option<f64> {
    let mut bound = supply_bound(shape, live_params, bnd, x, u, sens, ic);
    if let Some(sb) = bound.as_mut() {
        branch_floors(shape, &floors, sb.branches.iter_mut().map(|br| (br.device, &mut br.floor)));
        sb.floor = floors;
    }
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
        branches.push(SupplyBranch { i, di, t: read(&form, &elems), device: Some(st.device_index), floor: (0.0, 0.0) });
    }
    for cap in &shape.capacitors {
        if cap.leak_g != 0.0 {
            let (_, sa, la) = pin(&cap.pins[0]);
            let (_, sb, lb) = pin(&cap.pins[1]);
            let mut form = Vec::new();
            form.extend(la.map(|l| (l, cap.leak_g)));
            form.extend(lb.map(|l| (l, -cap.leak_g)));
            branches.push(SupplyBranch { i: 0.0, di: cap.leak_g * (sa - sb), t: read(&form, &elems), device: None, floor: (0.0, 0.0) });
        }
    }
    let r_form: Vec<(usize, f64)> = sens.jac[n * size..n * size + n].iter().copied().enumerate().collect();
    let draw_t = read(&r_form, &elems);
    let d = vec![0.0; forms.len()];
    let mut bound = SupplyBound { s: sens.s.clone(), x: x.to_vec(), elems, forms, junctions, branches, draw_t, draw: (sens.i, sens.g), d, floor: Floors::default() };
    if !sens.res.is_empty() {
        let mut m_r = sens.res.clone();
        lu_j.solve(&mut m_r).ok()?;
        if !m_r.iter().all(|v| v.is_finite()) {
            return None;
        }
        bound.bound_distance(&m_r)?;
    }
    Some(bound)
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
        /// The partition's nodes, in local order.
        nodes: Vec<usize>,
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
        put(nodes, Last { u_p, nodes: nodes.to_vec(), bound, supply: None });
    }

    pub(crate) fn record_supply(nodes: &[usize], u_p: f64, supply: Option<SupplyBound>) {
        put(nodes, Last { u_p, nodes: nodes.to_vec(), bound: None, supply });
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

        /// The bound on node `node`'s error at `|u - u_p| = b`: its own
        /// rounding floor, not the partition's largest.
        pub(crate) fn node_at(&self, node: usize, b: f64) -> f64 {
            let l = self.nodes.iter().position(|&m| m == node).expect("a node of the partition");
            if let Some(sb) = self.supply.as_ref() {
                return sb.eval(b).map_or(f64::INFINITY, |e| SupplyEval::err(e.k_node[l], e.k1_node[l], b) + sb.floor.node(l, b));
            }
            self.bound.as_ref().and_then(|bd| bd.eval(b).map(|e| e.k_v * 0.5 * b * b + bd.floor.node(l, b))).unwrap_or(f64::INFINITY)
        }

        /// The bound on device `device`'s published current's error, and
        /// whether it was bounded as a branch in series with junctions.
        pub(crate) fn branch(&self, device: usize, b: f64) -> (f64, bool) {
            if let Some(sb) = self.supply.as_ref() {
                let k = sb.branches.iter().position(|br| br.device == Some(device)).expect("a branch of the partition");
                return (sb.eval(b).map_or(f64::INFINITY, |e| SupplyEval::err(e.k_branch[k], e.k1_branch[k], b) + at_b(sb.branches[k].floor, b)), false);
            }
            let Some(bd) = self.bound.as_ref() else { return (f64::INFINITY, false) };
            let br = bd.branches.iter().find(|br| br.device == Some(device)).expect("a branch of the partition");
            (bd.eval(b).map_or(f64::INFINITY, |e| bd.branch_err(&e, br, b)), br.series.is_some())
        }

        /// The bound on junction `j`'s published current's error.
        pub(crate) fn junction(&self, j: usize, b: f64) -> f64 {
            if let Some(sb) = self.supply.as_ref() {
                let (fv, fi) = sb.floor.junction(j, b);
                return sb.eval(b).map_or(f64::INFINITY, |e| e.k_junction[j].1 * (SupplyEval::err(e.k_junction[j].0, e.k1_junction[j], b) + fv) + fi);
            }
            self.bound.as_ref().and_then(|bd| bd.eval(b).map(|e| bd.junction_err(&e, j, b))).unwrap_or(f64::INFINITY)
        }

        /// The bound on the draw's error.
        pub(crate) fn draw(&self, b: f64) -> f64 {
            if let Some(sb) = self.supply.as_ref() {
                return sb.eval(b).map_or(f64::INFINITY, |e| SupplyEval::err(e.k_draw, e.k1_draw, b) + sb.floor.draw(b));
            }
            self.bound.as_ref().and_then(|bd| bd.eval(b).map(|e| e.k_d * 0.5 * b * b + bd.floor.draw(b))).unwrap_or(f64::INFINITY)
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
            let br = Branch { i, di: s / r, g: 1.0 / r, contacts: 1.0, series: None, device: None, floor: (0.0, 0.0) };
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
        let sens = RailSens { i: law.current(u - m), g, s: vec![1.0], jac: vec![g, -g, -g, g], res: Vec::new(), gross: Vec::new(), gross_s: Vec::new(), lu: None };
        let band = hub_band(&shape, &HashMap::new(), &bnd, &[m], u, &sens, &IcRail::default(), Floors::default());
        assert_eq!(band, Some(NARROWEST_BAND));
    }

    /// `block` as the `n x n` block of a Jacobian with row stride `n + 1`,
    /// its rail row and column filled with a value no check reads.
    fn with_rail(block: &[&[f64]]) -> Vec<f64> {
        let n = block.len();
        let mut jac = vec![-7.0; (n + 1) * (n + 1)];
        for (r, row) in block.iter().enumerate() {
            jac[r * (n + 1)..r * (n + 1) + n].copy_from_slice(row);
        }
        jac
    }

    #[test]
    fn a_conductance_matrix_must_be_symmetric_with_no_positive_coupling_and_no_negative_row() {
        // The flow floor (FXE round 2) bounds |L J^-1| through J^-1 >= 0,
        // which holds for a symmetric matrix with off-diagonals <= 0, a
        // positive diagonal and rows summing to >= 0 (W1gV3's seeds: dropping
        // any one of the three checks passed every suite). Each failing
        // matrix below breaks one condition and keeps the other two, so each
        // check is the only one that can reject it.
        let three = with_rail(&[&[3.0, -1.0, -2.0], &[-1.0, 1.5, -0.5], &[-2.0, -0.5, 2.5]]);
        assert!(conductance_matrix(&three, 3), "a three-node network with every row summing to 0");
        // Symmetric to a few ulps is symmetric: the stamps add the same
        // conductances in different orders.
        let g = 0.1 + 0.2;
        let ulps = with_rail(&[&[1.0, -g], &[-0.3, 1.0]]);
        assert!(conductance_matrix(&ulps, 2), "an off-diagonal pair an ulp apart");
        // An IC's supply terms are not symmetric: rows summing to 2 and 1,
        // off-diagonals negative.
        let skew = with_rail(&[&[3.0, -1.0], &[-2.0, 3.0]]);
        assert!(!conductance_matrix(&skew, 2), "an asymmetric pair");
        // Symmetric, rows summing to 2.5, but coupled positively.
        let positive = with_rail(&[&[2.0, 0.5], &[0.5, 2.0]]);
        assert!(!conductance_matrix(&positive, 2), "a positive off-diagonal pair");
        // Symmetric, off-diagonals negative, but row 0 sums to -1.
        let short = with_rail(&[&[1.0, -2.0], &[-2.0, 3.0]]);
        assert!(!conductance_matrix(&short, 2), "a row summing below 0");
        // A row summing below 0 by rounding alone is still a network.
        let rounding = with_rail(&[&[0.3, -g], &[-g, 0.4]]);
        assert!(conductance_matrix(&rounding, 2), "a row 0.3 - (0.1 + 0.2) below 0 by one ulp");
    }
}
