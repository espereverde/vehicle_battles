//! Bevy-free simulation core. Everything that decides what happens lives here; the `game`
//! crate only renders `SimState` and turns player input into `replay::Command`s.
//!
//! Determinism contract: same `SimState` + same per-tick controls + same `Tuning` ⇒ bitwise
//! identical result (same machine). Never read clocks, global RNGs or iterate hash maps here.

pub mod control;
pub mod fit;
pub mod orders;
pub mod preview;
pub mod replay;
pub mod rng;
pub mod state;
pub mod track;
pub mod tuning;
pub mod vehicle;

pub use glam::Vec2;
pub use fit::{Fit, fit_segment, sweep};
pub use orders::{Drive, OrderQueue, Segment, Turn, TurnMode, hold_throttle};
pub use preview::{CarFrame, Drift, Preview};
pub use replay::{Command, Replay};
pub use rng::SimRng;
pub use state::{Car, CarId, Controls, DriveMode, Pilot, Pose, SimState, SlideState};
pub use track::{Piece, Track, TrackPos, TrackSpec};
pub use tuning::{KeyboardTuning, PlannerTuning, Tuning, VehicleTuning};

/// Immutable map data shared by live and preview simulation. Roads and the SDF grid arrive
/// in M3; for now the world is an empty, infinite plane.
#[derive(Debug, Clone, Default)]
pub struct World {}

/// `Preview` never rolls dice; it records risk events instead (used from M2 on).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepMode {
    Live,
    Preview,
    /// Like `Preview`, but the tires never give up: no understeer, no slides, the car goes
    /// where its controls point it (no tighter than the steering lock). Grip demand is still
    /// computed, so the path can be colored by what the real car would face. The player's
    /// planning preview and the drag fit use it; only live play shows what really happens.
    Ideal,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SimEvent {
    SlideStarted { car: CarId, tick: u64, speed: f32 },
    SlideEnded { car: CarId, tick: u64, duration: f32 },
    /// The car finished an order segment; `remaining` segments are left in its queue.
    SegmentCompleted { car: CarId, tick: u64, remaining: usize },
}

impl SimEvent {
    pub fn tick(&self) -> u64 {
        match *self {
            SimEvent::SlideStarted { tick, .. } | SimEvent::SlideEnded { tick, .. } | SimEvent::SegmentCompleted { tick, .. } => tick,
        }
    }
}

/// Everything that happened during one `step`, for UI, auto-pause triggers and metrics.
#[derive(Debug, Clone, Default)]
pub struct StepEvents {
    pub events: Vec<SimEvent>,
}

/// Advances the simulation by exactly one fixed tick (`tuning.dt()`).
pub fn step(state: &mut SimState, _world: &World, tuning: &Tuning, mode: StepMode) -> StepEvents {
    let dt = tuning.dt();
    let ideal = mode == StepMode::Ideal;
    let mut out = StepEvents::default();
    for car in &mut state.cars {
        for _ in 0..control::drive(car, &tuning.vehicle) {
            let remaining = car.orders.segments.len();
            out.events.push(SimEvent::SegmentCompleted { car: car.id, tick: state.tick, remaining });
        }
        if let Some(change) = vehicle::step_car(car, &tuning.vehicle, dt, ideal) {
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
