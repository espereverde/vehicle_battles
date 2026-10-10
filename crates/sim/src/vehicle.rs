//! Kinematic bicycle model with a friction-circle grip limit.
//!
//! Each tick the driver's controls define an *intended* motion: the car rolls along its
//! heading, turning at the kinematic yaw rate `v · tan(δ) / L`, while the engine/brakes
//! change speed. The tires then try to bring the velocity to that intended velocity, but
//! can deliver at most `grip · g` of acceleration (combined longitudinal + lateral).
//!
//! When the intended acceleration exceeds the friction circle the car understeers: the
//! command is scaled back onto the circle and the car runs wide. Overdriven further (past
//! `slide_threshold`, which rises at low speed) or with the handbrake, it enters a slide:
//! grip drops to `slide_grip`, steering keeps only part of its authority and the yaw rate
//! carries momentum (drift), so velocity and heading separate until grip is regained.

use crate::state::{Car, Controls, SlideState};
use crate::tuning::VehicleTuning;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SlideChange {
    Started { speed: f32 },
    Ended { duration: f32 },
}

pub fn step_car(car: &mut Car, vt: &VehicleTuning, dt: f32) -> Option<SlideChange> {
    let c = car.controls;
    let v_long = car.forward_speed();

    // Steering actuator: wheels turn at a finite rate.
    let target_steer = c.steer.clamp(-1.0, 1.0) * vt.max_steer_deg.to_radians();
    let max_delta = vt.steer_rate_deg.to_radians() * dt;
    car.steer_angle += (target_steer - car.steer_angle).clamp(-max_delta, max_delta);

    let a_long = longitudinal_accel(&c, v_long, vt, dt);
    let yaw_cmd = v_long * car.steer_angle.tan() / vt.wheelbase;

    // Intended combined acceleration relative to what the tires can do.
    let grip_limit = vt.grip * vt.gravity;
    let a_lat = v_long * yaw_cmd;
    let demand = (a_long * a_long + a_lat * a_lat).sqrt() / grip_limit;
    car.grip_demand = demand;

    // Past the friction circle the car first understeers (runs wide, keeps grip); it only
    // breaks into a slide when overdriven beyond `slide_threshold`. At low speed there is
    // little momentum to break traction, so the threshold rises as speed drops.
    let speed = car.vel.length();
    let threshold = vt.slide_threshold * (vt.slide_ref_speed / speed.max(0.1)).max(1.0);
    let handbrake = c.handbrake && speed > vt.handbrake_min_speed;
    let mut change = None;
    if car.slide.is_none() && (demand > threshold || handbrake) {
        car.slide = Some(SlideState::default());
        change = Some(SlideChange::Started { speed });
    }

    // With grip, an over-demanding command is scaled back onto the friction circle, and
    // the tires scrubbing past their limit slow the car (plowing), never reversing it.
    let on_circle = demand.max(1.0);
    let (a_long, yaw_grip) = match car.slide {
        None => {
            let scrub = vt.understeer_scrub * (on_circle - 1.0).min(1.0) * vt.gravity;
            let a = a_long / on_circle - scrub * v_long.signum();
            let a = if (v_long + a * dt) * v_long < 0.0 { -v_long / dt } else { a };
            (a, yaw_cmd / on_circle)
        }
        Some(_) => (a_long, yaw_cmd),
    };

    // Yaw: follows the (grip-limited) kinematic rate with grip, drifts towards a weaker
    // target while sliding.
    match car.slide {
        None => car.yaw_rate = yaw_grip,
        Some(_) => {
            let gain = if handbrake { vt.handbrake_yaw_gain } else { vt.slide_steer_authority };
            let k = 1.0 - (-dt / vt.slide_yaw_tau).exp();
            car.yaw_rate += (yaw_cmd * gain - car.yaw_rate) * k;
        }
    }
    car.pose.heading = wrap_angle(car.pose.heading + car.yaw_rate * dt);

    // Tires pull the velocity towards "rolling along the new heading", limited by friction.
    let v_target = car.pose.forward() * (v_long + a_long * dt);
    let mu = match car.slide {
        None => vt.grip,
        Some(_) if handbrake => vt.slide_grip * vt.handbrake_grip,
        Some(_) => vt.slide_grip,
    };
    let dv = (v_target - car.vel).clamp_length_max(mu * vt.gravity * dt);
    car.vel += dv;
    car.accel = dv / dt;
    car.pose.pos += car.vel * dt;

    let lat_speed = car.lateral_speed();
    if let Some(slide) = &mut car.slide {
        slide.time += dt;
        let yaw_err = (car.yaw_rate - yaw_cmd / on_circle).abs();
        let recovered = !handbrake
            && lat_speed.abs() < vt.slide_exit_lat_speed
            && demand < vt.slide_exit_demand
            && yaw_err < vt.slide_exit_yaw_error_deg.to_radians();
        let stopped = car.vel.length() < 0.5;
        if recovered || stopped {
            change = Some(SlideChange::Ended { duration: slide.time });
            car.slide = None;
        }
    }
    change
}

/// Acceleration along the heading from engine, brakes and resistance. Resistive forces
/// only ever slow the car down to zero, never push it backwards.
fn longitudinal_accel(c: &Controls, v: f32, vt: &VehicleTuning, dt: f32) -> f32 {
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

