//! Bevy-free simulation core. Everything that decides what happens lives here; the `game`
//! crate only renders `SimState` and turns player input into `replay::Command`s.
//!
//! Determinism contract: same `SimState` + same per-tick controls + same `Tuning` ⇒ bitwise
//! identical result (same machine). Never read clocks, global RNGs or iterate hash maps here.

pub mod control;
pub mod orders;
pub mod path;
pub mod preview;
pub mod replay;
pub mod rng;
pub mod state;
pub mod tuning;
pub mod vehicle;

pub use glam::Vec2;
pub use orders::{Hold, Maneuver, OrderQueue, Segment, SpeedProfile};
pub use path::PathSpec;
pub use preview::{CarFrame, Preview};
pub use replay::{Command, Replay};
pub use rng::SimRng;
pub use state::{Car, CarId, Controls, DriveMode, Pilot, Pose, SimState, SlideState};
pub use tuning::{ControlTuning, KeyboardTuning, PlannerTuning, Tuning, VehicleTuning};

/// Immutable map data shared by live and preview simulation. Roads and the SDF grid arrive
/// in M3; for now the world is an empty, infinite plane.
#[derive(Debug, Clone, Default)]
pub struct World {}

/// `Preview` never rolls dice; it records risk events instead (used from M2 on).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepMode {
    Live,
    Preview,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SimEvent {
    SlideStarted { car: CarId, tick: u64, speed: f32 },
    SlideEnded { car: CarId, tick: u64, duration: f32 },
    /// The car finished an order segment; `remaining` segments are left in its queue.
    SegmentCompleted { car: CarId, tick: u64, remaining: usize },
}

/// Everything that happened during one `step`, for UI, auto-pause triggers and metrics.
#[derive(Debug, Clone, Default)]
pub struct StepEvents {
    pub events: Vec<SimEvent>,
}

/// Advances the simulation by exactly one fixed tick (`tuning.dt()`).
pub fn step(state: &mut SimState, _world: &World, tuning: &Tuning, _mode: StepMode) -> StepEvents {
    let dt = tuning.dt();
    let mut out = StepEvents::default();
    for car in &mut state.cars {
        for _ in 0..control::drive(car, &tuning.vehicle, &tuning.control) {
            let remaining = car.orders.segments.len();
            out.events.push(SimEvent::SegmentCompleted { car: car.id, tick: state.tick, remaining });
        }
        if let Some(change) = vehicle::step_car(car, &tuning.vehicle, dt) {
            out.events.push(match change {
                vehicle::SlideChange::Started { speed } => {
                    SimEvent::SlideStarted { car: car.id, tick: state.tick, speed }
                }
                vehicle::SlideChange::Ended { duration } => {
                    SimEvent::SlideEnded { car: car.id, tick: state.tick, duration }
                }
            });
        }
    }
    state.tick += 1;
    out
}
