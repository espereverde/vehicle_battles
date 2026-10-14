//! Inverse of the drag tool for steering keypoints (bends): find the steering and duration
//! that bring a car closest to a point, in closed form.
//!
//! A constant wheel angle is a constant curvature at any speed (kinematic bicycle:
//! `κ = tan δ / L`), and speed does not depend on steering, so a bend is an arc whose length
//! after `n` ticks is the distance the pedals cover. The circle tangent to the car's heading
//! through the target has `κ = 2y / (x² + y²)` in the car's frame; capped by the steering
//! lock, the closest point of that arc picks the duration. How tight a turn is at speed is
//! priced by handling, not limited here. Drifts have their own fit (`drift::fit_drift`).

use std::f32::consts::TAU;

use glam::Vec2;

use crate::drift::max_curvature;
use crate::ease::Slip;
use crate::orders::{Drive, Turn};
use crate::state::{CarId, SimState};
use crate::tuning::{Tuning, VehicleTuning};
use crate::vehicle::{longitudinal_accel, wrap_angle};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fit {
    pub turn: Turn,
    /// Ticks from the start state to the point closest to the target (at least 1).
    pub ticks: u32,
    /// Distance left between the car and the target at that tick.
    pub miss: f32,
}

/// Point reached after driving `s` along an arc of curvature `k` from the origin, heading +x.
pub fn arc_point(k: f32, s: f32) -> Vec2 {
    let theta = k * s;
    if theta.abs() < 1e-4 { Vec2::new(s, 0.5 * k * s * s) } else { Vec2::new(theta.sin() / k, (1.0 - theta.cos()) / k) }
}

/// Finds the controlled skid sliding `inches` (and the duration, up to `max_ticks`) at
/// constant `throttle` that brings `car` closest to `target`: the car slides straight on
/// along its heading for the skid's time, eases onto its new nose (`regrip_seconds`), then
/// drives along it, so the nose swings to point from where the slide ends at the target (at
/// most `max_skid_deg`). The slide's length depends on the speed, so the angle is aimed per
/// candidate duration.
pub fn fit_skid(state: &SimState, tuning: &Tuning, car: CarId, throttle: f32, inches: f32, target: Vec2, max_ticks: u32) -> Fit {
    let skid = |angle| Turn::Skid { angle, slide: inches };
    let fallback = Fit { turn: skid(0.0), ticks: 1, miss: f32::INFINITY };
    let Some(c) = state.car(car) else { return fallback };
    let (vt, dt) = (&tuning.vehicle, tuning.dt());
    let inches = skid(0.0).slide();
    let (slide_ticks, _) = crate::control::skid_ticks(inches, tuning);
    let local = Vec2::from_angle(-c.pose.heading).rotate(target - c.pose.pos);
    let max = tuning.crash.max_skid_deg.to_radians();
    let pedals = Drive { turn: skid(0.0), throttle }.pedals();
    let max_ticks = max_ticks.max(1) as usize;
    let n_slide = slide_ticks as usize;

    // Distance covered by the end of each tick (the same whatever the angle): sliding, the
    // speed loss scrubs off evenly; then the car drives on.
    let scrub = crate::control::skid_speed_loss(inches) / slide_ticks as f32;
    let mut v = c.speed();
    let mut covered = vec![0.0];
    for tick in 1..=max_ticks {
        let step = if tick <= n_slide {
            let step = v.abs() * dt;
            v = (v + longitudinal_accel(&pedals, v, vt, dt) * dt - scrub).max(0.0);
            step
        } else {
            v += longitudinal_accel(&pedals, v, vt, dt) * dt;
            v.abs() * dt
        };
        covered.push(covered[tick - 1] + step);
    }
    // After the slide the direction of travel turns from the old heading to the new nose:
    // the fraction turned on each tick of the regrip.
    let mut regrip = Vec::new();
    let mut slip = Slip::new(1.0, tuning.crash.regrip_seconds);
    while let Some(s) = slip.and_then(|s| s.advance(dt)) {
        regrip.push(1.0 - s.angle());
        slip = Some(s);
    }

    // Where the car is after `n` ticks with the nose swung by `angle`.
    let end = |n: usize, angle: f32| {
        let mut p = Vec2::new(covered[n.min(n_slide)], 0.0);
        for (j, turned) in regrip.iter().enumerate() {
            let tick = n_slide + j + 1;
            if tick > n {
                return p;
            }
            p += Vec2::from_angle(angle * turned) * (covered[tick] - covered[tick - 1]);
        }
        let straight = (n_slide + regrip.len()).min(n);
        p + Vec2::from_angle(angle) * (covered[n] - covered[straight])
    };
    let mut best = fallback;
    for n in 1..=max_ticks {
        let slid = Vec2::new(covered[n.min(n_slide)], 0.0);
        let bearing = |p: Vec2| (p - slid).to_angle();
        let to = local - slid;
        let mut angle = if to.length_squared() > 1e-6 { to.to_angle().clamp(-max, max) } else { 0.0 };
        // The regrip carries the car on the old way a little: aim that much further round.
        if n > n_slide && to.length_squared() > 1e-6 {
            for _ in 0..4 {
                let reached = end(n, angle);
                if reached.distance_squared(slid) < 1e-6 {
                    break;
                }
                angle = (angle + wrap_angle(to.to_angle() - bearing(reached))).clamp(-max, max);
            }
        }
        let miss = end(n, angle).distance(local);
        if miss < best.miss {
            best = Fit { turn: skid(angle), ticks: n as u32, miss };
        }
    }
    best
}

/// Curvature of the arc tangent to +x at the origin through `local`, within the lock.
fn arc_to(local: Vec2, vt: &VehicleTuning) -> f32 {
    let wanted = if local.length_squared() > 1e-6 { 2.0 * local.y / local.length_squared() } else { 0.0 };
    wanted.signum() * wanted.abs().min(max_curvature(vt))
}

/// Steering for curvature `k` (kinematic bicycle: `κ = tan δ / L`).
fn steer_for(k: f32, vt: &VehicleTuning) -> f32 {
    (k * vt.wheelbase).atan() / vt.max_steer_deg.to_radians()
}

/// Searches the steering and duration (up to `max_ticks`) at constant `throttle` that bring
/// `car` closest to `target` from `state`. Ties keep the earliest tick, so a car that stops
/// short of the target gets its keypoint where it stops. Turns are no tighter than the
/// steering lock.
///
/// `turned` is the net heading change since the preview started (radians, + = left). A
/// candidate ends before that reaches a full circle, so keypoints stay on the first loop.
pub fn fit_bend(state: &SimState, tuning: &Tuning, car: CarId, throttle: f32, target: Vec2, max_ticks: u32, turned: f32) -> Fit {
    let fallback = Fit { turn: Turn::Steer(0.0), ticks: 1, miss: f32::INFINITY };
    let Some(c) = state.car(car) else { return fallback };
    let (vt, dt) = (&tuning.vehicle, tuning.dt());
    let local = Vec2::from_angle(-c.pose.heading).rotate(target - c.pose.pos);
    let v0 = c.forward_speed();
    let k = arc_to(local, vt);
    let turn = Turn::Steer(steer_for(k, vt));

    let pedals = Drive { turn, throttle }.pedals();
    let (mut v, mut s) = (v0, 0.0);
    let mut best = fallback;
    for ticks in 1..=max_ticks.max(1) {
        s += v.abs() * dt;
        v += longitudinal_accel(&pedals, v, vt, dt) * dt;
        if (turned + k * s).abs() >= TAU {
            break;
        }
        let miss = arc_point(k, s).distance(local);
        if miss < best.miss {
            best = Fit { turn, ticks, miss };
        }
    }
    best
}
