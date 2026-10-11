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
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Turn {
    /// Fraction of full lock, -1 (right) ..= 1 (left).
    Steer(f32),
    /// Lateral acceleration in g, positive = left.
    Lateral(f32),
}

impl Turn {
    pub fn new(mode: TurnMode, value: f32) -> Self {
        match mode {
            TurnMode::Steer => Turn::Steer(value),
            TurnMode::Lateral => Turn::Lateral(value),
        }
    }

    pub fn mode(&self) -> TurnMode {
        match self {
            Turn::Steer(_) => TurnMode::Steer,
            Turn::Lateral(_) => TurnMode::Lateral,
        }
    }

    pub fn value(&self) -> f32 {
        match *self {
            Turn::Steer(v) | Turn::Lateral(v) => v,
        }
    }

    /// Steering command at forward speed `v`.
    pub fn steer(&self, v: f32, vt: &VehicleTuning) -> f32 {
        match *self {
            Turn::Steer(s) => s.clamp(-1.0, 1.0),
            Turn::Lateral(g) => {
                // Kinematic bicycle: a_lat = v² · tan δ / L.
                let max = vt.max_steer_deg.to_radians();
                let delta = (g * vt.gravity * vt.wheelbase / (v * v).max(1e-3)).atan().clamp(-max, max);
                delta / max
            }
        }
    }

    /// The same turn expressed in `mode`, steering identically at speed `v`.
    pub fn convert(&self, mode: TurnMode, v: f32, vt: &VehicleTuning) -> Turn {
        match (*self, mode) {
            (Turn::Steer(s), TurnMode::Lateral) => {
                let delta = s.clamp(-1.0, 1.0) * vt.max_steer_deg.to_radians();
                Turn::Lateral(v * v * delta.tan() / vt.wheelbase / vt.gravity)
            }
            (Turn::Lateral(_), TurnMode::Steer) => Turn::Steer(self.steer(v, vt)),
            _ => *self,
        }
    }
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

    pub fn controls(&self, v: f32, vt: &VehicleTuning) -> Controls {
        let t = self.throttle.clamp(-1.0, 1.0);
        Controls { steer: self.turn.steer(v, vt), throttle: t.max(0.0), brake: (-t).max(0.0), handbrake: false }
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
}

impl Segment {
    pub fn new(drive: Drive, ticks: u32) -> Self {
        Self { drive, ticks: ticks.max(1), elapsed: 0 }
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
