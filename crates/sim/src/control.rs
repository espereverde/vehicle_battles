//! Turns orders into controls: pure-pursuit steering on the planned path plus a speed
//! controller following the segment's speed profile. The controller never protects the
//! car from its orders: a path that is too tight for the speed is driven as ordered and the
//! vehicle model decides what the tires can do.

use glam::Vec2;

use crate::orders::{Hold, OrderQueue};
use crate::state::{Car, Controls, DriveMode};
use crate::tuning::{ControlTuning, VehicleTuning};

/// Below this remaining distance a segment that ends at (near) zero speed counts as done
/// once the car has stopped, so it can't creep forever towards the exact end point.
const STOP_TOLERANCE: f32 = 0.3;

/// Runs the controller for one tick. Returns how many segments were completed.
pub fn drive(car: &mut Car, vt: &VehicleTuning, ct: &ControlTuning) -> usize {
    if car.drive_mode != DriveMode::Orders {
        return 0;
    }
    let pos = car.pose.pos;
    let v = car.forward_speed();
    let completed = advance_progress(&mut car.orders, pos, v);
    let lookahead = ct.lookahead_base + ct.lookahead_per_speed * v.abs();

    let (target, v_target, a_ff) = match car.orders.segments.first() {
        Some(seg) => {
            let (s, len, profile) = (seg.progress, seg.length(), seg.speed());
            let v_t = profile.at(s, len, ct);
            // a = d(v²)/2ds over a 1 m window kept inside the segment: exact for the
            // constant-acceleration pieces of the profile, including the final meter.
            let s1 = (s + 1.0).min(len);
            let s0 = (s1 - 1.0).max(0.0);
            let (v0, v1) = (profile.at(s0, len, ct), profile.at(s1, len, ct));
            let a_ff = if s1 > s0 { (v1 * v1 - v0 * v0) / (2.0 * (s1 - s0)) } else { 0.0 };
            (lookahead_point(&car.orders, s + lookahead), v_t, a_ff)
        }
        None => {
            let hold = *car.orders.hold.get_or_insert(Hold { origin: car.pose, speed: v.max(0.0) });
            let dir = hold.origin.forward();
            let s = (pos - hold.origin.pos).dot(dir);
            (hold.origin.pos + dir * (s + lookahead), hold.speed, 0.0)
        }
    };

    car.controls = Controls {
        steer: pure_pursuit(car, target, vt),
        ..speed_control(v, v_target, a_ff, vt, ct)
    };
    completed
}

/// Projects the car onto the active segment and pops every segment it has finished.
fn advance_progress(orders: &mut OrderQueue, pos: Vec2, v: f32) -> usize {
    let mut completed = 0;
    while let Some(seg) = orders.segments.first_mut() {
        let len = seg.length();
        let s = seg.path_spec().project(pos, seg.progress - 1.0, seg.progress + 3.0 + v.abs() * 0.1);
        seg.progress = seg.progress.max(s);
        let remaining = len - seg.progress;
        let stopped_at_end =
            seg.speed().end < 0.5 && remaining < STOP_TOLERANCE && v.abs() < STOP_TOLERANCE;
        if remaining > 1e-3 && !stopped_at_end {
            break;
        }
        let done = orders.segments.remove(0);
        if orders.segments.is_empty() {
            orders.hold = Some(Hold { origin: done.end_pose(), speed: done.speed().end });
        }
        completed += 1;
    }
    completed
}

/// Point `s` meters along the queued paths, continuing straight past the last one.
fn lookahead_point(orders: &OrderQueue, mut s: f32) -> Vec2 {
    let n = orders.segments.len();
    for (i, seg) in orders.segments.iter().enumerate() {
        if s <= seg.length() || i + 1 == n {
            return seg.path_spec().point_at(s);
        }
        s -= seg.length();
    }
    unreachable!("lookahead_point called with no segments")
}

/// Steering towards `target` along the circle through the rear axle that is tangent to
/// the car's heading.
fn pure_pursuit(car: &Car, target: Vec2, vt: &VehicleTuning) -> f32 {
    let rear = car.pose.pos - car.pose.forward() * vt.wheelbase * 0.5;
    let d = target - rear;
    let dist2 = d.length_squared().max(1e-4);
    let curvature = 2.0 * d.dot(car.pose.left()) / dist2;
    let delta = (curvature * vt.wheelbase).atan();
    (delta / vt.max_steer_deg.to_radians()).clamp(-1.0, 1.0)
}

/// Throttle / brake for a target speed, with feedforward acceleration and compensation
/// for rolling resistance and drag (forward driving only).
fn speed_control(v: f32, v_target: f32, a_ff: f32, vt: &VehicleTuning, ct: &ControlTuning) -> Controls {
    if v_target <= 0.0 && v.abs() < STOP_TOLERANCE {
        return Controls { brake: 1.0, ..Default::default() };
    }
    let resist = vt.rolling_resistance + vt.drag * v * v;
    let a = a_ff + ct.speed_gain * (v_target - v);
    if a + resist >= 0.0 {
        let engine = vt.engine_accel * (1.0 - v.max(0.0) / vt.top_speed).max(0.05);
        Controls { throttle: ((a + resist) / engine).clamp(0.0, 1.0), ..Default::default() }
    } else {
        Controls { brake: (-(a + resist) / vt.brake_decel).clamp(0.0, 1.0), ..Default::default() }
    }
}
