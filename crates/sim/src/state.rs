use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::orders::OrderQueue;
use crate::rng::SimRng;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct CarId(pub u32);

/// Position in meters, heading in radians (0 = +x, counter-clockwise positive).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Pose {
    pub pos: Vec2,
    pub heading: f32,
}

impl Pose {
    pub fn new(pos: Vec2, heading: f32) -> Self {
        Self { pos, heading }
    }

    pub fn forward(&self) -> Vec2 {
        Vec2::from_angle(self.heading)
    }

    pub fn left(&self) -> Vec2 {
        self.forward().perp()
    }
}

/// Driver commands: the output of the controller (or the debug keyboard in M0).
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct Controls {
    /// -1 (full right) ..= 1 (full left).
    pub steer: f32,
    /// -1 (full reverse) ..= 1 (full forward).
    pub throttle: f32,
    /// 0 ..= 1.
    pub brake: f32,
    pub handbrake: bool,
}

/// Who sets `Car::controls` each tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub enum DriveMode {
    /// The controller follows the car's `orders`.
    #[default]
    Orders,
    /// Controls are set from outside (debug keyboard, tests).
    Direct,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Pilot {
    /// Seconds of locked orders after a pause (M4).
    pub reaction_time: f32,
    /// 0 ..= 1, scales stability recovery (M2).
    pub skill: f32,
}

impl Default for Pilot {
    fn default() -> Self {
        Self { reaction_time: 0.6, skill: 0.5 }
    }
}

/// Present while the tires are past the grip limit.
#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct SlideState {
    /// Seconds since the slide started.
    pub time: f32,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Car {
    pub id: CarId,
    pub pose: Pose,
    /// World-space velocity, m/s. Not necessarily aligned with the heading while sliding.
    pub vel: Vec2,
    /// rad/s.
    pub yaw_rate: f32,
    /// Actual front wheel angle in radians (rate-limited towards `controls.steer`).
    pub steer_angle: f32,
    pub slide: Option<SlideState>,
    /// Placeholder until M2.
    pub stability: f32,
    pub controls: Controls,
    pub drive_mode: DriveMode,
    pub orders: OrderQueue,
    pub pilot: Pilot,
    /// Telemetry from the last tick: acceleration the tires actually delivered (world space).
    pub accel: Vec2,
    /// Telemetry from the last tick: intended acceleration / available grip (1.0 = limit).
    pub grip_demand: f32,
}

impl Car {
    pub fn new(id: CarId, pose: Pose) -> Self {
        Self {
            id,
            pose,
            vel: Vec2::ZERO,
            yaw_rate: 0.0,
            steer_angle: 0.0,
            slide: None,
            stability: 1.0,
            controls: Controls::default(),
            drive_mode: DriveMode::default(),
            orders: OrderQueue::default(),
            pilot: Pilot::default(),
            accel: Vec2::ZERO,
            grip_demand: 0.0,
        }
    }

    /// Signed speed along the heading.
    pub fn forward_speed(&self) -> f32 {
        self.vel.dot(self.pose.forward())
    }

    /// Signed sideways speed (positive = moving to the car's left).
    pub fn lateral_speed(&self) -> f32 {
        self.vel.dot(self.pose.left())
    }

    /// Angle between heading and velocity, radians. Zero when (nearly) stopped.
    pub fn slip_angle(&self) -> f32 {
        if self.vel.length_squared() < 0.25 {
            0.0
        } else {
            self.lateral_speed().atan2(self.forward_speed())
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SimState {
    pub tick: u64,
    pub cars: Vec<Car>,
    pub rng: SimRng,
}

impl SimState {
    pub fn new(seed: u64) -> Self {
        Self { tick: 0, cars: Vec::new(), rng: SimRng::new(seed) }
    }

    pub fn spawn_car(&mut self, pose: Pose) -> CarId {
        let id = CarId(self.cars.len() as u32);
        self.cars.push(Car::new(id, pose));
        id
    }

    pub fn car(&self, id: CarId) -> Option<&Car> {
        self.cars.iter().find(|c| c.id == id)
    }

    pub fn car_mut(&mut self, id: CarId) -> Option<&mut Car> {
        self.cars.iter_mut().find(|c| c.id == id)
    }

    pub fn time(&self, tuning: &crate::Tuning) -> f64 {
        self.tick as f64 / tuning.tick_rate as f64
    }
}
