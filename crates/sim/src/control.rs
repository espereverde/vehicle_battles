//! Turns orders into controls. Each segment holds constant controls (a drift's steering
//! follows its S-curve); the vehicle model decides what comes of them. A controlled skid
//! starts its slide here, at the start of its segment.

use glam::Vec2;

use crate::crash::{self, Slide};
use crate::handling::MPH;
use crate::orders::{Phase, Turn};
use crate::state::{Car, DriveMode};
use crate::tuning::Tuning;

/// What the controller did this tick.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Driven {
    /// Segments completed (their last tick is this one).
    pub completed: usize,
    /// A drift segment drove this tick: its sideways/forward ratio and the distance covered.
    pub drift: Option<(f32, f32)>,
    /// A controlled skid started its slide this tick: the handling points it adds.
    pub skid: Option<f32>,
}

/// Speed a controlled skid of `inches` loses (m/s): the book's 0 / 5 / 5 / 10 mph for
/// ¼" / ½" / ¾" / 1", interpolated.
pub fn skid_speed_loss(inches: f32) -> f32 {
    let step = |from: f32| ((inches - from) / 0.25).clamp(0.0, 1.0) * 5.0;
    (step(0.25) + step(0.75)) / MPH
}

/// How many ticks a controlled skid of `inches` slides, and over how many of them its nose
/// swings round.
pub fn skid_ticks(inches: f32, tuning: &Tuning) -> (u32, u32) {
    let c = &tuning.crash;
    let ticks = (inches * c.skid_seconds_per_inch * tuning.tick_rate as f32).round().max(1.0) as u32;
    let turn = (c.skid_turn_seconds * tuning.tick_rate as f32).round().max(1.0) as u32;
    (ticks, turn.min(ticks))
}

/// Sets the car's controls for this tick (`dt` long) from its orders.
pub fn drive(car: &mut Car, tuning: &Tuning, dt: f32) -> Driven {
    let mut out = Driven::default();
    if car.drive_mode != DriveMode::Orders {
        return out;
    }
    let vt = &tuning.vehicle;
    let v = car.speed();
    let heading = car.pose.heading;
    let Car { orders, slide, slip, vel, controls, .. } = car;
    // Zero-length segments (never created by the game, but possible in a replay file).
    while orders.segments.first().is_some_and(|s| s.remaining() == 0) {
        orders.tail = orders.segments.remove(0).drive;
        out.completed += 1;
    }
    let phase = orders.segments.first_mut().map(|s| {
        s.start_heading.get_or_insert(heading);
        Phase { elapsed: s.elapsed, ticks: s.ticks, traveled: s.traveled }
    });
    *controls = orders.active_drive().controls(v, vt, phase, dt);
    let Some(seg) = orders.segments.first_mut() else { return out };

    // Controlled skid: from the segment's start, slide along the heading while the nose
    // swings round. Costs the bend plus the slide, checked now.
    if let Turn::Skid { angle, .. } = seg.drive.turn
        && seg.elapsed == 0
        && !seg.skidding
        && slide.is_none()
    {
        let c = &tuning.crash;
        let inches = seg.drive.turn.slide();
        let angle = angle.clamp(-c.max_skid_deg.to_radians(), c.max_skid_deg.to_radians());
        seg.skidding = true;
        let (ticks, turn_ticks) = skid_ticks(inches, tuning);
        let skid = Slide::controlled(Vec2::from_angle(heading), ticks, angle, turn_ticks, skid_speed_loss(inches));
        *slip = crash::entry_slip(*vel, skid.dir, tuning);
        *slide = Some(skid);
        out.skid = Some(angle.abs() / tuning.handling.bend_deg.to_radians() + c.skid_cost_per_inch * inches);
    }

    let step = v.abs() * dt;
    if let Turn::Drift(ratio) = seg.drive.turn {
        out.drift = Some((ratio, step));
    }
    seg.elapsed += 1;
    seg.traveled += step;
    if seg.remaining() == 0 {
        orders.tail = orders.segments.remove(0).drive;
        out.completed += 1;
    }
    out
}
