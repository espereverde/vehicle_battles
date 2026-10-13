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
pub mod stability;
pub mod swerve;
pub mod state;
pub mod track;
pub mod tuning;
pub mod vehicle;

pub use glam::Vec2;
pub use fit::{Fit, ReachMap, fit_segment, sweep};
pub use swerve::{fit_swerve, sweep_swerve};
pub use orders::{Drive, OrderQueue, Phase, Segment, Turn, TurnMode, hold_throttle};
pub use preview::{CarFrame, Drift, Preview, RiskStretch};
pub use replay::{Command, Replay};
pub use rng::SimRng;
pub use stability::LossKind;
pub use state::{Car, CarId, Controls, DriveMode, Pilot, Pose, SimState, SlideState};
pub use track::{Piece, Track, TrackPos, TrackSpec};
pub use tuning::{KeyboardTuning, PlannerTuning, Tuning, VehicleTuning};

/// Immutable map data shared by live and preview simulation. Roads and the SDF grid arrive
/// in M3; for now the world is an empty, infinite plane.
#[derive(Debug, Clone, Default)]
pub struct World {}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepMode {
    Live,
    /// Exactly like `Live`, dice included (the RNG is part of the state, so a forked state
    /// rolls the same numbers): an exact forecast for the preview-vs-reality check. Never
    /// shown to the player.
    Preview,
    /// Like `Preview`, but the tires never give up: no understeer, no slides, the car goes
    /// where its controls point it (no tighter than the steering lock). Grip demand is still
    /// computed, so the path can be colored by what the real car would face. The player's
    /// planning preview and the drag fit use it; only live play shows what really happens.
    /// Never rolls dice: risk checks are reported with their chance (`SimEvent::Risk`).
    Ideal,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SimEvent {
    SlideStarted { car: CarId, tick: u64, speed: f32 },
    SlideEnded { car: CarId, tick: u64, duration: f32 },
    /// The car finished an order segment; `remaining` segments are left in its queue.
    SegmentCompleted { car: CarId, tick: u64, remaining: usize },
    /// A risk check (yellow stability zone): `chance` of losing control (1.0 = certain),
    /// and if so `spin_chance` of a spin rather than a skid.
    Risk { car: CarId, tick: u64, chance: f32, spin_chance: f32 },
    /// The car lost control (rolled modes only); a `SlideStarted` does not accompany it.
    LossOfControl { car: CarId, tick: u64, kind: LossKind, speed: f32 },
}

impl SimEvent {
    pub fn tick(&self) -> u64 {
        match *self {
            SimEvent::SlideStarted { tick, .. }
            | SimEvent::SlideEnded { tick, .. }
            | SimEvent::SegmentCompleted { tick, .. }
            | SimEvent::Risk { tick, .. }
            | SimEvent::LossOfControl { tick, .. } => tick,
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
    let SimState { tick, cars, rng } = state;
    let tick = *tick;
    for car in cars.iter_mut() {
        for _ in 0..control::drive(car, &tuning.vehicle, dt) {
            let remaining = car.orders.segments.len();
            out.events.push(SimEvent::SegmentCompleted { car: car.id, tick, remaining });
        }
        let prev_demand = car.grip_demand;
        if let Some(change) = vehicle::step_car(car, &tuning.vehicle, dt, ideal) {
            out.events.push(match change {
                vehicle::SlideChange::Started { speed } => {
                    SimEvent::SlideStarted { car: car.id, tick, speed }
                }
                vehicle::SlideChange::Ended { duration } => {
                    SimEvent::SlideEnded { car: car.id, tick, duration }
                }
            });
        }
        let dice = if ideal { None } else { Some(&mut *rng) };
        if let Some(check) = stability::update(car, prev_demand, tuning, dt, dice) {
            let (chance, spin_chance) = (check.chance, check.spin_chance);
            out.events.push(SimEvent::Risk { car: car.id, tick, chance, spin_chance });
            if let Some(kind) = check.lost {
                out.events.push(SimEvent::LossOfControl { car: car.id, tick, kind, speed: car.vel.length() });
            }
        }
    }
    state.tick += 1;
    out
}
