//! Bevy-free simulation core. Everything that decides what happens lives here; the `game`
//! crate only renders `SimState` and turns player input into `replay::Command`s.
//!
//! Determinism contract: same `SimState` + same per-tick controls + same `Tuning` ⇒ bitwise
//! identical result (same machine). Never read clocks, global RNGs or iterate hash maps here.

pub mod control;
pub mod crash;
pub mod drift;
pub mod ease;
pub mod fit;
pub mod handling;
pub mod orders;
pub mod preview;
pub mod replay;
pub mod rng;
pub mod state;
pub mod track;
pub mod tuning;
pub mod vehicle;

pub use glam::Vec2;
pub use crash::{Crash, Slide, SlideKind};
pub use fit::{Fit, fit_bend, fit_skid};
pub use drift::fit_drift;
pub use orders::{Drive, OrderQueue, Phase, Segment, Turn, TurnMode, hold_throttle};
pub use preview::{CarFrame, Preview, RiskStretch};
pub use replay::{Command, Replay};
pub use rng::SimRng;
pub use state::{Car, CarId, Controls, DriveMode, Pilot, Pose, SimState};
pub use track::{Piece, Track, TrackPos, TrackSpec};
pub use tuning::{CrashTuning, HandlingTuning, KeyboardTuning, PlannerTuning, Tuning, VehicleTuning};

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
    /// Like `Preview`, but never rolls dice: risk checks are reported with their chance
    /// (`SimEvent::Risk`) and the car never loses control. The player's planning preview
    /// and the drag fit use it; only live play shows what really happens.
    Ideal,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SimEvent {
    /// The car finished an order segment; `remaining` segments are left in its queue.
    SegmentCompleted { car: CarId, tick: u64, remaining: usize },
    /// A risky control check (a tick that drained handling): `chance` of losing control
    /// (1.0 = certain), and if so `severe_chance` of a spinout or worse.
    Risk { car: CarId, tick: u64, chance: f32, severe_chance: f32 },
    /// The car lost control (rolled modes only): the crash table result.
    LossOfControl { car: CarId, tick: u64, kind: Crash, speed: f32 },
    /// A slide (crash result or controlled skid) ended.
    SlideEnded { car: CarId, tick: u64 },
}

impl SimEvent {
    pub fn tick(&self) -> u64 {
        match *self {
            SimEvent::SegmentCompleted { tick, .. }
            | SimEvent::Risk { tick, .. }
            | SimEvent::LossOfControl { tick, .. }
            | SimEvent::SlideEnded { tick, .. } => tick,
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
    let mut out = StepEvents::default();
    let SimState { tick, cars, rng } = state;
    let tick = *tick;
    for car in cars.iter_mut() {
        // A slide already going on (not one a controlled skid starts this tick).
        let was_sliding = car.slide.is_some();
        let driven = control::drive(car, tuning, dt);
        for _ in 0..driven.completed {
            let remaining = car.orders.segments.len();
            out.events.push(SimEvent::SegmentCompleted { car: car.id, tick, remaining });
        }
        let mut dice = (mode != StepMode::Ideal).then_some(&mut *rng);

        // Motion: sliding (crash result or controlled skid) or driving.
        let sliding = car.slide.is_some();
        let (heading, speed) = (car.pose.heading, car.speed().abs());
        let distance = speed * dt;
        if sliding {
            if crash::step(car, &car.controls.clone(), tuning, dt, dice.as_deref_mut()) {
                out.events.push(SimEvent::SlideEnded { car: car.id, tick });
            }
        } else {
            vehicle::step_car(car, &tuning.vehicle, dt);
        }

        // Handling: what this tick's maneuvering demands (nothing while sliding).
        let turned = vehicle::wrap_angle(car.pose.heading - heading);
        let decel = ((speed - car.speed().abs()) / dt).max(0.0);
        let (maneuver, braking) = if sliding {
            (0.0, 0.0)
        } else {
            (handling::maneuver_drain(turned, driven.drift, tuning), handling::braking_drain(decel, tuning) * dt)
        };
        let drain = maneuver + braking + driven.skid.unwrap_or(0.0);
        let difficulty = handling::difficulty(maneuver, distance, decel, tuning) + driven.skid.unwrap_or(0.0);
        if let Some(check) = handling::update(car, drain, difficulty, was_sliding, tuning, dt, dice).filter(|c| c.chance > 0.0) {
            let (chance, severe_chance) = (check.chance, check.severe_chance);
            out.events.push(SimEvent::Risk { car: car.id, tick, chance, severe_chance });
            if let Some(kind) = check.lost {
                out.events.push(SimEvent::LossOfControl { car: car.id, tick, kind, speed: car.vel.length() });
            }
        }
    }
    state.tick += 1;
    out
}
