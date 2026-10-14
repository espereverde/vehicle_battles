//! Orders are intentions, not wheel positions: a queue of segments, each holding a constant
//! turn (steering or drift) and a constant throttle for a fixed number of ticks. Where
//! a segment ends is whatever the simulation makes of it, so editing one segment moves all
//! later keypoints while keeping their controls.

use serde::{Deserialize, Serialize};

use crate::drift;
use crate::state::Controls;
use crate::tuning::VehicleTuning;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TurnMode {
    /// Constant steering wheel position: a constant turn radius at any speed.
    Steer,
    /// Car Wars drift: move sideways while ending pointing the way the car started (see
    /// `drift.rs`).
    Drift,
    /// Car Wars controlled skid: the nose swings round quickly while the car keeps sliding
    /// the way it was going, then it drives on along its new nose.
    Skid,
}

/// Slide of a new controlled skid, in inches.
pub const DEFAULT_SKID: f32 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Turn {
    /// Fraction of full lock, -1 (right) ..= 1 (left).
    Steer(f32),
    /// Sideways movement per forward movement over the segment (positive = left), at most
    /// `drift::MAX_RATIO`: an S-curve of two arcs of equal length. After the segment (as the
    /// queue's tail) it drives straight.
    Drift(f32),
    /// From the segment's start the car slides `slide` inches (¼ ..= 1) along its heading
    /// there while the nose swings by `angle` (radians, + = left, at most
    /// `max_skid_deg`) over the first `skid_turn_inches`; then it drives straight along the
    /// new nose. Costs the bend (`angle / bend_deg`) plus `skid_cost_per_inch` per inch slid,
    /// checked when it starts (`control.rs`). Once started it can't be changed (planning locks
    /// it). As the queue's tail it drives straight.
    Skid { angle: f32, slide: f32 },
}

impl Turn {
    pub fn new(mode: TurnMode, value: f32) -> Self {
        match mode {
            TurnMode::Steer => Turn::Steer(value),
            TurnMode::Drift => Turn::Drift(value),
            TurnMode::Skid => Turn::Skid { angle: value, slide: DEFAULT_SKID },
        }
    }

    pub fn mode(&self) -> TurnMode {
        match self {
            Turn::Steer(_) => TurnMode::Steer,
            Turn::Drift(_) => TurnMode::Drift,
            Turn::Skid { .. } => TurnMode::Skid,
        }
    }

    pub fn value(&self) -> f32 {
        match *self {
            Turn::Steer(v) | Turn::Drift(v) | Turn::Skid { angle: v, .. } => v,
        }
    }

    /// Constant steering command (a drift has none, a skid drives straight: 0).
    pub fn steer(&self) -> f32 {
        match *self {
            Turn::Steer(s) => s.clamp(-1.0, 1.0),
            Turn::Drift(_) | Turn::Skid { .. } => 0.0,
        }
    }

    /// The same turn in `mode`, or a straight one when the mode changes.
    pub fn convert(&self, mode: TurnMode) -> Turn {
        if self.mode() == mode { *self } else { Turn::new(mode, 0.0) }
    }

    /// Slide of a controlled skid, in inches (0 for other turns).
    pub fn slide(&self) -> f32 {
        match *self {
            Turn::Skid { slide, .. } => slide.clamp(0.25, 1.0),
            _ => 0.0,
        }
    }
}

/// Where a car is in its current segment, for turns that depend on it (`Turn::Drift`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Phase {
    /// Ticks of the segment already driven.
    pub elapsed: u32,
    /// Total length of the segment in ticks.
    pub ticks: u32,
    /// Distance driven in the segment so far (m).
    pub traveled: f32,
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

    /// Pedals only (no steering).
    pub fn pedals(&self) -> Controls {
        let t = self.throttle.clamp(-1.0, 1.0);
        Controls { steer: 0.0, throttle: t.max(0.0), brake: (-t).max(0.0), handbrake: false }
    }

    /// Controls at forward speed `v`; `phase` is where the car is in this drive's segment,
    /// `None` once the segment is over and the drive is the queue's tail; `dt` the tick length.
    pub fn controls(&self, v: f32, vt: &VehicleTuning, phase: Option<Phase>, dt: f32) -> Controls {
        let steer = match (self.turn, phase) {
            (Turn::Drift(ratio), Some(p)) => drift::steer(ratio, self, v, p, vt, dt),
            _ => self.turn.steer(),
        };
        Controls { steer, ..self.pedals() }
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
    /// Distance driven so far (m; runtime state, advanced by the controller).
    #[serde(default)]
    pub traveled: f32,
    /// Heading when the segment started (runtime state, set by the controller).
    #[serde(default)]
    pub start_heading: Option<f32>,
    /// A controlled skid's slide has started (runtime state).
    #[serde(default)]
    pub skidding: bool,
}

impl Segment {
    pub fn new(drive: Drive, ticks: u32) -> Self {
        Self { drive, ticks: ticks.max(1), elapsed: 0, traveled: 0.0, start_heading: None, skidding: false }
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
