//! The preview *is* the simulation: fork the state, run the same `step()` in
//! `StepMode::Preview`, and keep a compact per-tick record of every car.

use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::state::{CarId, Controls, Pose, SimState, SlideState};
use crate::tuning::Tuning;
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
            segments_left: car.orders.segments.len(),
        }
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
