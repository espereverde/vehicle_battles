//! The preview *is* the simulation: fork the state, run the same `step()` in
//! `StepMode::Preview`, and keep a compact per-tick record of every car.

use std::f32::consts::TAU;

use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::crash::SlideKind;
use crate::state::{CarId, Controls, Pose, SimState};
use crate::tuning::Tuning;
use crate::vehicle::wrap_angle;
use crate::{Car, SimEvent, StepMode, World, step};

/// Everything about a car that changes tick to tick, minus the (large) order paths.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct CarFrame {
    pub pose: Pose,
    pub vel: Vec2,
    pub yaw_rate: f32,
    pub steer_angle: f32,
    pub controls: Controls,
    pub accel: Vec2,
    pub handling_lost: f32,
    pub drain: f32,
    pub risk: f32,
    pub slide: Option<SlideKind>,
    pub segments_left: usize,
}

impl CarFrame {
    pub fn of(car: &Car) -> Self {
        Self {
            pose: car.pose,
            vel: car.vel,
            yaw_rate: car.yaw_rate,
            steer_angle: car.steer_angle,
            controls: car.controls,
            accel: car.accel,
            handling_lost: car.handling_lost,
            drain: car.drain,
            risk: car.risk,
            slide: car.slide.map(|s| s.kind),
            segments_left: car.orders.segments.len(),
        }
    }

    /// Handling status.
    pub fn status(&self, tuning: &Tuning) -> f32 {
        crate::handling::class(tuning) - self.handling_lost
    }

    /// Risk zone, see `handling::zone`.
    pub fn zone(&self, tuning: &Tuning) -> usize {
        crate::handling::zone(self.status(tuning), self.vel.length(), self.risk, tuning)
    }
}

#[derive(Debug, Clone)]
pub struct Preview {
    pub start_tick: u64,
    pub cars: Vec<CarId>,
    /// `frames[i][c]`: car `cars[c]` after `i` ticks (`frames[0]` is the starting state).
    pub frames: Vec<Vec<CarFrame>>,
    pub events: Vec<SimEvent>,
}

/// Consecutive risk checks of one car in a preview, merged for display ("34% spin at
/// 2.1 s"). Frame indices are into the preview (`frames[i]`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RiskStretch {
    /// Frames of the first and last check.
    pub start: usize,
    pub end: usize,
    /// Chance of losing control somewhere in the stretch: `1 − Π(1 − p)`.
    pub chance: f32,
    /// If it happens, chance that it is a spinout or worse (weighted by each check's chance).
    pub severe_chance: f32,
    /// Frame of the first certain loss of control, if any.
    pub certain_at: Option<usize>,
}

impl RiskStretch {
    /// Where to mark it: the certain loss if there is one, else its start.
    pub fn at(&self) -> usize {
        self.certain_at.unwrap_or(self.start)
    }
}

/// How live execution compares with a preview recorded before it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Check {
    /// Live tick is outside the preview window.
    OutOfRange,
    Match,
    Diverged { car: CarId, position_error: f32 },
}

impl Preview {
    pub fn run(state: &SimState, world: &World, tuning: &Tuning, ticks: u32) -> Self {
        Self::run_in(StepMode::Preview, state, world, tuning, ticks)
    }

    /// Same as `run`, stepping in `mode` (`Preview` or `Ideal`).
    pub fn run_in(mode: StepMode, state: &SimState, world: &World, tuning: &Tuning, ticks: u32) -> Self {
        let mut s = state.clone();
        let snapshot = |s: &SimState| s.cars.iter().map(CarFrame::of).collect::<Vec<_>>();
        let mut frames = Vec::with_capacity(ticks as usize + 1);
        frames.push(snapshot(&s));
        let mut events = Vec::new();
        for _ in 0..ticks {
            events.extend(step(&mut s, world, tuning, mode).events);
            frames.push(snapshot(&s));
        }
        Self { start_tick: state.tick, cars: state.cars.iter().map(|c| c.id).collect(), frames, events }
    }

    pub fn ticks(&self) -> usize {
        self.frames.len() - 1
    }

    /// First tick at which `car` has turned a full circle (net heading change of 360° since
    /// the start), if it does: from there on the path only repeats its loop.
    pub fn first_loop(&self, car: CarId) -> Option<usize> {
        let mut turned = 0.0;
        let mut heading = self.frame(car, 0)?.pose.heading;
        for i in 1..=self.ticks() {
            let h = self.frame(car, i)?.pose.heading;
            turned += wrap_angle(h - heading);
            heading = h;
            if turned.abs() >= TAU {
                return Some(i);
            }
        }
        None
    }

    /// The car's risky control checks (`SimEvent::Risk`), merged into stretches (no more
    /// than a quarter second apart).
    pub fn risks(&self, car: CarId, tuning: &Tuning) -> Vec<RiskStretch> {
        let gap = (tuning.tick_rate as usize).div_ceil(4) + 1;
        let mut out: Vec<RiskStretch> = Vec::new();
        let mut spin_weight = 0.0;
        for e in &self.events {
            let SimEvent::Risk { car: c, tick, chance, severe_chance } = *e else { continue };
            if c != car {
                continue;
            }
            // The check ran during the step from `tick`, so it shows in the next frame.
            let i = (tick - self.start_tick) as usize + 1;
            let certain = (chance >= 1.0).then_some(i);
            match out.last_mut() {
                Some(r) if i - r.end <= gap => {
                    r.end = i;
                    r.chance = 1.0 - (1.0 - r.chance) * (1.0 - chance);
                    r.certain_at = r.certain_at.or(certain);
                    spin_weight += chance;
                    r.severe_chance += (severe_chance - r.severe_chance) * chance / spin_weight.max(1e-6);
                }
                _ => {
                    spin_weight = chance;
                    out.push(RiskStretch { start: i, end: i, chance, severe_chance, certain_at: certain });
                }
            }
        }
        out
    }

    /// Chance that `car` loses control within each second of the preview (`[0]` is the
    /// first second): `1 − Π(1 − p)` over its control checks in that second.
    pub fn risk_per_second(&self, car: CarId, tuning: &Tuning) -> Vec<f32> {
        let rate = tuning.tick_rate as usize;
        let mut keep = vec![1.0f32; self.ticks().div_ceil(rate).max(1)];
        for e in &self.events {
            let SimEvent::Risk { car: c, tick, chance, .. } = *e else { continue };
            if c == car
                && let Some(k) = keep.get_mut((tick - self.start_tick) as usize / rate)
            {
                *k *= 1.0 - chance;
            }
        }
        keep.into_iter().map(|k| 1.0 - k).collect()
    }

    /// What `car` goes through between frames `from` and `to` (a keypoint's segment):
    /// handling points drained, and the chance of losing control somewhere in it.
    pub fn span_risk(&self, car: CarId, from: usize, to: usize) -> (f32, f32) {
        let (mut drained, mut keep) = (0.0, 1.0);
        for f in (from + 1..=to).filter_map(|i| self.frame(car, i)) {
            drained += f.drain;
            keep *= 1.0 - f.risk;
        }
        (drained, 1.0 - keep)
    }

    /// Drops everything after `ticks` ticks.
    pub fn cut(&mut self, ticks: usize) {
        self.frames.truncate(ticks + 1);
        let end = self.start_tick + ticks as u64;
        self.events.retain(|e| e.tick() < end);
    }

    /// Frame of `car` at `i` ticks into the preview.
    pub fn frame(&self, car: CarId, i: usize) -> Option<&CarFrame> {
        let c = self.cars.iter().position(|&id| id == car)?;
        self.frames.get(i)?.get(c)
    }

    /// Compares the live state with the frame recorded for the same tick (bitwise).
    pub fn check(&self, state: &SimState) -> Check {
        let Some(i) = state.tick.checked_sub(self.start_tick) else { return Check::OutOfRange };
        let Some(frame) = self.frames.get(i as usize) else { return Check::OutOfRange };
        for (id, expected) in self.cars.iter().zip(frame) {
            match state.car(*id) {
                Some(car) if CarFrame::of(car) == *expected => {}
                Some(car) => {
                    let position_error = car.pose.pos.distance(expected.pose.pos);
                    return Check::Diverged { car: *id, position_error };
                }
                None => return Check::Diverged { car: *id, position_error: f32::INFINITY },
            }
        }
        Check::Match
    }
}
