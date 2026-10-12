//! The preview *is* the simulation: fork the state, run the same `step()` in
//! `StepMode::Preview`, and keep a compact per-tick record of every car.

use std::f32::consts::TAU;

use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::state::{CarId, Controls, Pose, SimState, SlideState};
use crate::stability::at_risk;
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
    pub slide: Option<SlideState>,
    pub controls: Controls,
    pub accel: Vec2,
    pub grip_demand: f32,
    pub stability: f32,
    pub segments_left: usize,
}

impl CarFrame {
    pub fn of(car: &Car) -> Self {
        Self {
            pose: car.pose,
            vel: car.vel,
            yaw_rate: car.yaw_rate,
            steer_angle: car.steer_angle,
            slide: car.slide,
            controls: car.controls,
            accel: car.accel,
            grip_demand: car.grip_demand,
            stability: car.stability,
            segments_left: car.orders.segments.len(),
        }
    }

    /// A risk check could fire here (yellow stability, above comfort, fast enough).
    pub fn at_risk(&self, tuning: &Tuning) -> bool {
        at_risk(self.grip_demand, self.stability, self.vel.length(), tuning)
    }

    /// Grip zone, see `stability::zone`.
    pub fn zone(&self, tuning: &Tuning) -> usize {
        crate::stability::zone(self.grip_demand, self.at_risk(tuning), tuning)
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

/// Why the real car may leave the (ideal) preview at a tick: see `Preview::drift`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Drift {
    /// Past the friction circle: the car runs wide.
    Understeer,
    /// The car may lose control here (see `CarFrame::at_risk`): much less grip, much wider.
    Slide,
    /// Back within grip after a drift; the offset carries on.
    Recover,
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
    /// If it happens, chance that it is a spin (weighted by each check's chance).
    pub spin_chance: f32,
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

    /// Outer bound of where the real car could drift where this preview (run in
    /// `StepMode::Ideal`) asks for more grip than the car has: for each frame, `None` on
    /// track, or the widest point it could reach and why.
    ///
    /// Same rule as the vehicle model: an over-demanding command is scaled back onto the
    /// friction circle, so the turn rate is divided by the grip demand; where the car may
    /// lose control the friction circle shrinks to `slide_grip`. Speed follows the plan (the real car also
    /// scrubs speed, so this errs wide). After a drift the bound takes the planned turn again
    /// for `recover_ticks`, keeping its offset. Only a hint: the real path depends on how
    /// the slide develops, which this does not simulate.
    pub fn drift(&self, car: CarId, tuning: &Tuning, recover_ticks: usize) -> Vec<Option<(Vec2, Drift)>> {
        let (vt, dt) = (&tuning.vehicle, tuning.dt());
        let frames: Vec<&CarFrame> = (0..=self.ticks()).map_while(|i| self.frame(car, i)).collect();
        let mut out = vec![None; frames.len()];
        // Outer bound: position, direction of travel, recovery ticks left.
        let mut bound: Option<(Vec2, f32, usize)> = None;
        for i in 1..frames.len() {
            let (prev, f) = (frames[i - 1], frames[i]);
            let speed = f.vel.length();
            let kind = if f.at_risk(tuning) {
                Some(Drift::Slide)
            } else if f.grip_demand > 1.0 {
                Some(Drift::Understeer)
            } else {
                None
            };
            let (pos, dir, left) = match (bound, kind) {
                (None, None) => continue,
                (None, Some(_)) => (prev.pose.pos, prev.pose.heading, recover_ticks),
                (Some(_), None) if bound.is_some_and(|b| b.2 == 0) => {
                    bound = None;
                    continue;
                }
                (Some(b), _) => b,
            };
            let turn = wrap_angle(f.pose.heading - prev.pose.heading);
            let mu = match kind {
                Some(Drift::Slide) => vt.slide_grip,
                _ => vt.grip,
            };
            let turn = match kind {
                Some(_) => turn / (f.grip_demand * vt.grip / mu).max(1.0),
                None => turn,
            };
            let dir = dir + turn;
            let pos = pos + Vec2::from_angle(dir) * speed * dt;
            let left = if kind.is_some() { recover_ticks } else { left - 1 };
            out[i] = Some((pos, kind.unwrap_or(Drift::Recover)));
            bound = Some((pos, dir, left));
        }
        out
    }

    /// The car's risk checks (`SimEvent::Risk`), merged into stretches of consecutive
    /// checks (no more than one check interval apart).
    pub fn risks(&self, car: CarId, tuning: &Tuning) -> Vec<RiskStretch> {
        let gap = (tuning.stability.risk_interval * tuning.tick_rate as f32).round() as usize + 1;
        let mut out: Vec<RiskStretch> = Vec::new();
        let mut spin_weight = 0.0;
        for e in &self.events {
            let SimEvent::Risk { car: c, tick, chance, spin_chance } = *e else { continue };
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
                    r.spin_chance += (spin_chance - r.spin_chance) * chance / spin_weight.max(1e-6);
                }
                _ => {
                    spin_weight = chance;
                    out.push(RiskStretch { start: i, end: i, chance, spin_chance, certain_at: certain });
                }
            }
        }
        out
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
