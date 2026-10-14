//! Drift (Car Wars): the car moves sideways while going forward and ends pointing the way
//! it started. Its cost is the book's drift, by how steep it is: up to ¼" sideways per 1"
//! forward is a drift, up to ½" a steep drift, and no steeper (`MAX_RATIO`).
//!
//! On the ground the car turns realistically: the path is an S-curve of two circular arcs
//! of equal length and opposite curvature. For a sideways/forward ratio `r` over the whole
//! drift, each arc turns `θ = 2·atan(r)` (independent of the drift's length), so the end
//! point is `(S·sin θ / θ, r · S·sin θ / θ)` for a path of length `S`.
//!
//! The arcs are split by distance, not time, so the heading comes back exactly whatever the
//! speed does: steering does not change speed in the kinematic model, so the controller
//! predicts the distance the segment's remaining ticks will cover (`travel`) and switches
//! curvature halfway along it.

use glam::Vec2;

use crate::fit::Fit;
use crate::orders::{Drive, Phase, Turn};
use crate::state::{CarId, SimState};
use crate::tuning::{Tuning, VehicleTuning};
use crate::vehicle::longitudinal_accel;

/// Steepest drift: ½" sideways per 1" forward.
pub const MAX_RATIO: f32 = 0.5;

/// Heading change of each arc for sideways/forward ratio `ratio` (radians, unsigned).
pub fn arc_angle(ratio: f32) -> f32 {
    2.0 * ratio.abs().min(MAX_RATIO).atan()
}

/// Forward distance per distance driven along the S-curve.
pub fn forward_per_distance(ratio: f32) -> f32 {
    let theta = arc_angle(ratio);
    if theta < 1e-4 { 1.0 } else { theta.sin() / theta }
}

/// Handling points per inch forward at `ratio`: rises linearly to `drift_cost` at ¼, then
/// to `steep_drift_cost` at ½.
pub fn cost_per_inch(ratio: f32, tuning: &Tuning) -> f32 {
    let (r, h) = (ratio.abs().min(MAX_RATIO), &tuning.handling);
    if r <= 0.25 { h.drift_cost * r / 0.25 } else { h.drift_cost + (h.steep_drift_cost - h.drift_cost) * (r - 0.25) / 0.25 }
}

/// Distance the car covers in `ticks` ticks from forward speed `v` with `drive`'s pedals
/// (the same speed sequence as the vehicle model; steering does not affect it).
pub fn travel(drive: &Drive, v: f32, ticks: u32, vt: &VehicleTuning, dt: f32) -> f32 {
    let pedals = drive.pedals();
    let mut v = v;
    let mut d = 0.0;
    for _ in 0..ticks {
        d += v.abs() * dt;
        v += longitudinal_accel(&pedals, v, vt, dt) * dt;
    }
    d
}

/// Steering command for a drift at `ratio` this tick, at forward speed `v` in phase `p` of
/// its segment.
pub fn steer(ratio: f32, drive: &Drive, v: f32, p: Phase, vt: &VehicleTuning, dt: f32) -> f32 {
    let step = v.abs() * dt;
    let total = p.traveled + travel(drive, v, p.ticks.saturating_sub(p.elapsed), vt, dt);
    if step < 1e-6 || total < 1e-3 {
        return 0.0;
    }
    let half = 0.5 * total;
    // No sharper than the lock (then the drift moves less sideways, but stays symmetric).
    let curvature = ratio.signum() * (2.0 * arc_angle(ratio) / total).min(max_curvature(vt));
    // This tick's share of each arc, so the switch falls exactly halfway.
    let overlap = |a: f32, b: f32| ((p.traveled + step).min(b) - p.traveled.max(a)).max(0.0);
    let k = curvature * (overlap(0.0, half) - overlap(half, total)) / step;
    // Kinematic bicycle: curvature = tan δ / L (backwards, the same wheel angle mirrors it).
    let max = vt.max_steer_deg.to_radians();
    let delta = (k * vt.wheelbase * v.signum()).atan().clamp(-max, max);
    delta / max
}

/// Sharpest arc the steering lock allows (1/m).
pub fn max_curvature(vt: &VehicleTuning) -> f32 {
    vt.max_steer_deg.to_radians().tan() / vt.wheelbase
}

/// Finds the drift (ratio, length up to `max_ticks`) at constant `throttle` that ends
/// closest to `target`, in closed form: the ratio comes from the target (at most
/// `MAX_RATIO`, and no sharper than the steering lock allows over the drift's length), the
/// length from the distance the car covers. Ties keep the earliest tick.
pub fn fit_drift(state: &SimState, tuning: &Tuning, car: CarId, throttle: f32, target: Vec2, max_ticks: u32) -> Fit {
    let fallback = Fit { turn: Turn::Drift(0.0), ticks: 1, miss: f32::INFINITY };
    let Some(c) = state.car(car) else { return fallback };
    let (vt, dt) = (&tuning.vehicle, tuning.dt());
    let local = Vec2::from_angle(-c.pose.heading).rotate(target - c.pose.pos);
    let wanted = if local.x > 1e-3 { (local.y / local.x).clamp(-MAX_RATIO, MAX_RATIO) } else { 0.0 };
    let drive = Drive { turn: Turn::Drift(wanted), throttle };
    let pedals = drive.pedals();
    let (mut v, mut length) = (c.speed(), 0.0);
    let mut best = fallback;
    for ticks in 1..=max_ticks.max(1) {
        length += v.abs() * dt;
        v += longitudinal_accel(&pedals, v, vt, dt) * dt;
        // Two arcs of `length / 2` turning θ each: curvature 2θ / length, within the lock.
        let theta = arc_angle(wanted).min(max_curvature(vt) * length / 2.0);
        let ratio = wanted.signum() * (theta / 2.0).tan();
        let forward = length * forward_per_distance(ratio);
        let miss = Vec2::new(forward, ratio * forward).distance(local);
        if miss < best.miss {
            best = Fit { turn: Turn::Drift(ratio), ticks, miss };
        }
    }
    best
}
