//! Kinematic bicycle model: the car goes exactly where its controls point it.
//!
//! Each tick the front wheels take the commanded angle at once, the engine/brakes change
//! speed, and the car rolls along its heading, turning at the kinematic yaw rate
//! `v · tan(δ) / L`. There is no grip limit: no understeer, no slides. Losing control is
//! decided by handling status (`handling.rs`), not by the tires. After a slide the car
//! still goes partly the slide's way for a moment (`Car::slip`), easing back onto its nose.

use crate::state::{Car, Controls};
use crate::tuning::VehicleTuning;

pub fn step_car(car: &mut Car, vt: &VehicleTuning, dt: f32) {
    let c = car.controls;
    let v = car.speed();

    car.steer_angle = c.steer.clamp(-1.0, 1.0) * vt.max_steer_deg.to_radians();
    let a_long = longitudinal_accel(&c, v, vt, dt);
    car.yaw_rate = v * car.steer_angle.tan() / vt.wheelbase;

    car.pose.heading = wrap_angle(car.pose.heading + car.yaw_rate * dt);
    car.slip = car.slip.and_then(|s| s.advance(dt));
    let vel = car.travel_dir() * (v + a_long * dt);
    car.accel = (vel - car.vel) / dt;
    car.vel = vel;
    car.pose.pos += car.vel * dt;
}

/// Acceleration along the heading from engine, brakes and resistance. Resistive forces
/// only ever slow the car down to zero, never push it backwards.
pub fn longitudinal_accel(c: &Controls, v: f32, vt: &VehicleTuning, dt: f32) -> f32 {
    let throttle = c.throttle.clamp(-1.0, 1.0);
    let engine = if throttle >= 0.0 {
        throttle * vt.engine_accel * (1.0 - v.max(0.0) / vt.top_speed).max(0.0)
    } else {
        throttle * vt.reverse_accel * (1.0 - (-v).max(0.0) / vt.reverse_top_speed).max(0.0)
    };
    let mut resist = vt.rolling_resistance + vt.drag * v * v + c.brake.clamp(0.0, 1.0) * vt.brake_decel;
    if c.handbrake {
        resist += vt.handbrake_decel;
    }
    let v1 = v + engine * dt;
    let r = resist * dt;
    let v2 = if v1 > r {
        v1 - r
    } else if v1 < -r {
        v1 + r
    } else {
        0.0
    };
    (v2 - v) / dt
}

pub fn wrap_angle(a: f32) -> f32 {
    use std::f32::consts::{PI, TAU};
    let w = (a + PI).rem_euclid(TAU) - PI;
    if w < -PI { w + TAU } else { w }
}
