//! Orders are intentions, not wheel positions: a queue of segments, each holding a constant
//! turn (steering or lateral g) and a constant throttle for a fixed number of ticks. Where
//! a segment ends is whatever the simulation makes of it, so editing one segment moves all
//! later keypoints while keeping their controls.

use serde::{Deserialize, Serialize};

use crate::state::Controls;
use crate::tuning::VehicleTuning;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TurnMode {
    /// Constant steering wheel position.
    Steer,
    /// Constant lateral acceleration: steering adapts to speed.
    Lateral,
    /// Two chained curves of equal and opposite turn rate and equal duration (an S-curve
    /// that ends pointing the way it started): steering adapts to speed so the yaw rate stays
    /// constant, and flips sign halfway through the segment.
    Swerve,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Turn {
    /// Fraction of full lock, -1 (right) ..= 1 (left).
    Steer(f32),
    /// Lateral acceleration in g, positive = left.
    Lateral(f32),
    /// Yaw rate (rad/s, positive = left) for the first half of the segment, then the
    /// opposite, with the wheels turning in and out at the steering rate (see
    /// `Turn::steer_at`). After the segment (as the queue's tail) it drives straight.
    Swerve(f32),
}

impl Turn {
    pub fn new(mode: TurnMode, value: f32) -> Self {
        match mode {
            TurnMode::Steer => Turn::Steer(value),
            TurnMode::Lateral => Turn::Lateral(value),
            TurnMode::Swerve => Turn::Swerve(value),
        }
    }

    pub fn mode(&self) -> TurnMode {
        match self {
            Turn::Steer(_) => TurnMode::Steer,
            Turn::Lateral(_) => TurnMode::Lateral,
            Turn::Swerve(_) => TurnMode::Swerve,
        }
    }

    pub fn value(&self) -> f32 {
        match *self {
            Turn::Steer(v) | Turn::Lateral(v) | Turn::Swerve(v) => v,
        }
    }

    /// Steering command at forward speed `v`; `phase` is where the car is in its segment,
    /// `None` once the segment is over and the turn is the queue's tail. `dt` is the tick
    /// length.
    ///
    /// A swerve's wheels turn in at `steer_rate_deg` and, from where they must start
    /// swinging across (a ramp centered on the midpoint), are steered back so that the car
    /// ends pointing the way it started with the wheels straight on the last tick, however
    /// short and sharp the swerve and wherever the wheels were when it began.
    pub fn steer_at(&self, v: f32, vt: &VehicleTuning, phase: Option<Phase>, dt: f32) -> f32 {
        let Turn::Swerve(_) = *self else { return self.steer(v, vt) };
        let Some(p) = phase else { return 0.0 };
        let max = vt.max_steer_deg.to_radians();
        let slew = vt.steer_rate_deg.to_radians();
        let full = self.steer(v, vt) * max;
        // Ticks the wheels take to turn from straight to the full angle; the first curve
        // turns in until the swing across has to start.
        let ramp = (full.abs() / (slew * dt)).max(1e-6);
        let (t, mid) = ((p.elapsed + 1) as f32, 0.5 * p.ticks as f32);
        if t < mid - ramp.min(0.5 * mid) {
            return full * (t / ramp).min(1.0) / max;
        }
        let left = p.ticks.saturating_sub(p.elapsed);
        let level = swerve_return(v, vt, dt, p.turned, p.wheels, left);
        // Never so far that the wheels can't be straight again on the last tick.
        let reach = slew * dt * left.saturating_sub(1) as f32;
        level.clamp(-reach, reach) / max
    }

    /// Steering command at forward speed `v` (a swerve: its first curve at full angle).
    pub fn steer(&self, v: f32, vt: &VehicleTuning) -> f32 {
        let max = vt.max_steer_deg.to_radians();
        match *self {
            Turn::Steer(s) => s.clamp(-1.0, 1.0),
            Turn::Lateral(g) => {
                // Kinematic bicycle: a_lat = v² · tan δ / L.
                let delta = (g * vt.gravity * vt.wheelbase / (v * v).max(1e-3)).atan().clamp(-max, max);
                delta / max
            }
            Turn::Swerve(rate) => {
                // Kinematic bicycle: yaw rate = v · tan δ / L.
                let delta = (rate * vt.wheelbase / v.max(1e-3)).atan().clamp(-max, max);
                delta / max
            }
        }
    }

    /// The same turn expressed in `mode`, steering identically at speed `v` (a swerve:
    /// its first curve).
    pub fn convert(&self, mode: TurnMode, v: f32, vt: &VehicleTuning) -> Turn {
        if self.mode() == mode {
            return *self;
        }
        let delta = self.steer(v, vt) * vt.max_steer_deg.to_radians();
        match mode {
            TurnMode::Steer => Turn::Steer(self.steer(v, vt)),
            TurnMode::Lateral => Turn::Lateral(v * v * delta.tan() / vt.wheelbase / vt.gravity),
            TurnMode::Swerve => Turn::Swerve(v * delta.tan() / vt.wheelbase),
        }
    }
}

/// Where a car is in its current segment, for turns that depend on it (`Turn::Swerve`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Phase {
    /// Ticks of the segment already driven.
    pub elapsed: u32,
    /// Total length of the segment in ticks.
    pub ticks: u32,
    /// Heading change since the segment started (rad, positive = left).
    pub turned: f32,
    /// Current wheel angle (rad).
    pub wheels: f32,
}

/// Most solver steps for a swerve's return, and the heading error (rad) it stops at.
const SOLVER_STEPS: usize = 12;
const SOLVER_TOLERANCE: f32 = 1e-4;

/// The wheel angle to hold for the rest of a swerve so that, moving the wheels there at
/// the steering rate, holding, and turning them straight by the end of the `left` ticks,
/// the car undoes the `turned` heading change. Each tick re-solves it, so speed changes
/// and tick rounding are corrected as the swerve goes.
fn swerve_return(v: f32, vt: &VehicleTuning, dt: f32, turned: f32, wheels: f32, left: u32) -> f32 {
    let max = vt.max_steer_deg.to_radians();
    let slew = vt.steer_rate_deg.to_radians();
    let time = left as f32 * dt;
    // Heading change while the wheels turn from `a` to `b` at the steering rate, per unit
    // of v / L: ∫ tan δ dt = (-ln cos b + ln cos a) / slew, signed by the direction.
    let g = |d: f32| -d.cos().ln();
    let g_wheels = g(wheels);
    let heading = |h: f32| {
        let g_h = g(h);
        let hold = (time - ((h - wheels).abs() + h.abs()) / slew).max(0.0);
        let sweeps = ((g_h - g_wheels) * (h - wheels).signum() + g_h * h.signum()) / slew;
        v / vt.wheelbase * (sweeps + h.tan() * hold)
    };
    // Levels reachable with time left to straighten the wheels; the heading change grows
    // with the level, so the root is bracketed (false position, Illinois variant).
    let (mut lo, mut hi) = (((wheels - slew * time) / 2.0).max(-max), ((wheels + slew * time) / 2.0).min(max));
    if lo > hi {
        return 0.0;
    }
    let (mut f_lo, mut f_hi) = (turned + heading(lo), turned + heading(hi));
    if f_lo >= 0.0 {
        return lo;
    }
    if f_hi <= 0.0 {
        return hi;
    }
    let mut side = 0;
    for _ in 0..SOLVER_STEPS {
        let h = (lo * f_hi - hi * f_lo) / (f_hi - f_lo);
        let f = turned + heading(h);
        if f.abs() < SOLVER_TOLERANCE {
            return h;
        }
        if f < 0.0 {
            (lo, f_lo) = (h, f);
            if side == -1 {
                f_hi *= 0.5;
            }
            side = -1;
        } else {
            (hi, f_hi) = (h, f);
            if side == 1 {
                f_lo *= 0.5;
            }
            side = 1;
        }
    }
    (lo * f_hi - hi * f_lo) / (f_hi - f_lo)
}

/// Constant controls for a stretch of time.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Drive {
    pub turn: Turn,
    /// -1 (full brake) ..= 1 (full throttle).
    pub throttle: f32,
}

impl Drive {
    pub const COAST: Drive = Drive { turn: Turn::Steer(0.0), throttle: 0.0 };

    /// Straight ahead with the throttle that holds speed `v`.
    pub fn straight_at(mode: TurnMode, v: f32, vt: &VehicleTuning) -> Drive {
        Drive { turn: Turn::new(mode, 0.0), throttle: hold_throttle(v, vt) }
    }

    /// Controls at forward speed `v`; `phase` and `dt` as in `Turn::steer_at`.
    pub fn controls(&self, v: f32, vt: &VehicleTuning, phase: Option<Phase>, dt: f32) -> Controls {
        let t = self.throttle.clamp(-1.0, 1.0);
        Controls { steer: self.turn.steer_at(v, vt, phase, dt), throttle: t.max(0.0), brake: (-t).max(0.0), handbrake: false }
    }
}

/// Throttle at which engine force balances rolling resistance and drag at speed `v`.
pub fn hold_throttle(v: f32, vt: &VehicleTuning) -> f32 {
    if v < 0.5 {
        return 0.0;
    }
    let resist = vt.rolling_resistance + vt.drag * v * v;
    let engine = vt.engine_accel * (1.0 - v / vt.top_speed);
    if engine <= 0.0 { 1.0 } else { (resist / engine).clamp(0.0, 1.0) }
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Segment {
    pub drive: Drive,
    /// Total length in ticks (at least 1).
    pub ticks: u32,
    /// Ticks already driven (runtime state, advanced by the controller).
    pub elapsed: u32,
    /// Heading when the segment started (runtime state, set by the controller).
    #[serde(default)]
    pub start_heading: Option<f32>,
}

impl Segment {
    pub fn new(drive: Drive, ticks: u32) -> Self {
        Self { drive, ticks: ticks.max(1), elapsed: 0, start_heading: None }
    }

    pub fn remaining(&self) -> u32 {
        self.ticks.saturating_sub(self.elapsed)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OrderQueue {
    pub segments: Vec<Segment>,
    /// What the car does once every segment is done: when the last segment completes it
    /// keeps going with that segment's controls.
    pub tail: Drive,
}

impl Default for OrderQueue {
    fn default() -> Self {
        Self { segments: Vec::new(), tail: Drive::COAST }
    }
}

impl OrderQueue {
    /// Controls in effect right now.
    pub fn active_drive(&self) -> Drive {
        self.segments.first().map_or(self.tail, |s| s.drive)
    }

    /// Controls in effect after the last segment.
    pub fn last_drive(&self) -> Drive {
        self.segments.last().map_or(self.tail, |s| s.drive)
    }

    /// Ticks from now until each segment ends (cumulative).
    pub fn end_ticks(&self) -> Vec<u32> {
        let mut t = 0;
        self.segments
            .iter()
            .map(|s| {
                t += s.remaining();
                t
            })
            .collect()
    }

    /// Ticks from now until segment `k` starts.
    pub fn start_tick(&self, k: usize) -> u32 {
        self.segments[..k].iter().map(Segment::remaining).sum()
    }
}
