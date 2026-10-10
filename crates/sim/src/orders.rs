//! Orders are intentions, not controls: a queue of maneuver segments (path + speed profile)
//! that the controller turns into steering and throttle every tick.

use serde::{Deserialize, Serialize};

use crate::path::PathSpec;
use crate::state::Pose;
use crate::tuning::{ControlTuning, PlannerTuning};

/// Target speed along one segment: leave at `start`, settle at `cruise`, arrive at `end`,
/// changing speed at the planner's comfortable acceleration / deceleration.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct SpeedProfile {
    pub start: f32,
    pub cruise: f32,
    pub end: f32,
}

/// Floor for the launch speed so a profile starting at rest still gets the car moving.
const MIN_LAUNCH_SPEED: f32 = 1.5;

impl SpeedProfile {
    pub fn at(&self, s: f32, length: f32, ct: &ControlTuning) -> f32 {
        let cruise = self.cruise.max(self.end);
        let s = s.clamp(0.0, length);
        let plateau = if self.start <= cruise {
            let launch = self.start.max(MIN_LAUNCH_SPEED.min(cruise));
            (launch * launch + 2.0 * ct.plan_accel * s).sqrt().min(cruise)
        } else {
            (self.start * self.start - 2.0 * ct.plan_decel * s).max(0.0).sqrt().max(cruise)
        };
        let arrive = (self.end * self.end + 2.0 * ct.plan_decel * (length - s)).sqrt();
        plateau.min(arrive)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Maneuver {
    /// Free-form path from the drag tool.
    Path { path: PathSpec, speed: SpeedProfile },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Segment {
    pub maneuver: Maneuver,
    /// Arc length already driven (runtime state, advanced by the controller).
    pub progress: f32,
}

impl Segment {
    pub fn path(start: Pose, end: Pose, speed: SpeedProfile, pt: &PlannerTuning) -> Self {
        Self { maneuver: Maneuver::Path { path: PathSpec::bezier(start, end, pt), speed }, progress: 0.0 }
    }

    pub fn path_spec(&self) -> &PathSpec {
        match &self.maneuver {
            Maneuver::Path { path, .. } => path,
        }
    }

    pub fn speed(&self) -> &SpeedProfile {
        match &self.maneuver {
            Maneuver::Path { speed, .. } => speed,
        }
    }

    pub fn speed_mut(&mut self) -> &mut SpeedProfile {
        match &mut self.maneuver {
            Maneuver::Path { speed, .. } => speed,
        }
    }

    /// Moves the end pose. The path is stale until `OrderQueue::rechain` re-fits it.
    pub fn set_end(&mut self, end: Pose) {
        match &mut self.maneuver {
            Maneuver::Path { path, .. } => path.end = end,
        }
    }

    pub fn start_pose(&self) -> Pose {
        self.path_spec().start
    }

    pub fn end_pose(&self) -> Pose {
        self.path_spec().end
    }

    pub fn length(&self) -> f32 {
        self.path_spec().length
    }
}

/// What the car does when it runs out of segments: keep going straight at a fixed speed
/// along the line through `origin`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Hold {
    pub origin: Pose,
    pub speed: f32,
}

#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct OrderQueue {
    pub segments: Vec<Segment>,
    /// Set when the last segment completes; if `None` with no segments, the controller
    /// holds the course and speed the car has at that moment.
    pub hold: Option<Hold>,
}

impl OrderQueue {
    /// Pose and speed where a new segment appended now would start.
    pub fn tail(&self, car_pose: Pose, car_speed: f32) -> (Pose, f32) {
        match self.segments.last() {
            Some(seg) => (seg.end_pose(), seg.speed().end),
            None => (car_pose, car_speed),
        }
    }

    /// Re-fits every segment after an edit so each starts where the previous one ends.
    /// The first segment starts at the car, unless it is already being driven, in which
    /// case it keeps its original start and its progress is re-projected onto the new path.
    pub fn rechain(&mut self, car_pose: Pose, car_speed: f32, pt: &PlannerTuning) {
        let mut start = (car_pose, car_speed);
        for (i, seg) in self.segments.iter_mut().enumerate() {
            let active = i == 0 && seg.progress > 0.0;
            if active {
                start = (seg.start_pose(), seg.speed().start);
            }
            let end = seg.end_pose();
            let speed = SpeedProfile { start: start.1, ..*seg.speed() };
            let old_progress = seg.progress;
            *seg = Segment::path(start.0, end, speed, pt);
            if active {
                seg.progress = seg.path_spec().project(car_pose.pos, 0.0, old_progress + 10.0);
            }
            start = (end, speed.end);
        }
    }
}
