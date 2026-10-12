//! Stability gauge and loss of control.
//!
//! Each tick, after the vehicle step: stability drains while the car asks for more grip
//! than is comfortable (`k_drain · (demand − comfort)²`) and on harsh changes of demand
//! (`k_jerk`), and recovers while it doesn't (`k_rec · pilot.skill`). Below `yellow`, every
//! `risk_interval` spent above comfort is a risk check that may make the car lose control;
//! at 0 the loss of control is certain. Losing control means a skid or (faster) a spin: the
//! car enters the vehicle model's slide state with a yaw kick into the turn. Stability is
//! frozen while sliding and restarts from `after_loss`.
//!
//! Only `StepMode::Live` and `Preview` roll the dice (from the state's RNG, so the preview
//! check stays bitwise exact). `Ideal` never does: it reports each check's chance, and at a
//! certain loss of control it resets stability as if the car had recovered, without
//! simulating the slide.

use serde::{Deserialize, Serialize};

use crate::rng::SimRng;
use crate::state::{Car, SlideState};
use crate::tuning::Tuning;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LossKind {
    Skid,
    Spin,
}

/// One risk check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Check {
    /// Chance that this check makes the car lose control (1.0 = certain).
    pub chance: f32,
    /// If it does, chance that it is a spin rather than a skid.
    pub spin_chance: f32,
    /// What happened (rolled modes only).
    pub lost: Option<LossKind>,
}

/// A check could happen in this state: yellow zone, above comfort, fast enough.
pub fn at_risk(demand: f32, stability: f32, speed: f32, tuning: &Tuning) -> bool {
    let st = &tuning.stability;
    stability < st.yellow && demand > tuning.vehicle.comfort_demand && speed > st.min_speed
}

/// Chance per check at `stability`.
pub fn chance(stability: f32, tuning: &Tuning) -> f32 {
    let st = &tuning.stability;
    if stability <= 0.0 {
        1.0
    } else {
        st.risk_max * (1.0 - stability / st.yellow).max(0.0).powi(2)
    }
}

/// Chance that a loss of control at `speed` is a spin.
pub fn spin_chance(speed: f32, tuning: &Tuning) -> f32 {
    let st = &tuning.stability;
    ((speed - st.skid_speed) / (st.spin_speed - st.skid_speed).max(1e-3)).clamp(0.0, 1.0)
}

/// Stability after one tick at grip `demand` (`prev_demand` the tick before).
pub fn next(stability: f32, demand: f32, prev_demand: f32, skill: f32, tuning: &Tuning, dt: f32) -> f32 {
    let st = &tuning.stability;
    let comfort = tuning.vehicle.comfort_demand;
    let mut s = stability;
    if demand > comfort {
        s -= st.k_drain * (demand - comfort).powi(2) * dt;
    } else {
        s += st.k_rec * skill * dt;
    }
    s -= st.k_jerk * (demand - prev_demand).abs();
    s.clamp(0.0, 1.0)
}

/// Grip zone of a moment, as the path is colored: 0 green (comfortable), 1 yellow (above
/// comfort), 2 orange (past the grip limit: understeer), 3 red (may lose control).
pub fn zone(demand: f32, at_risk: bool, tuning: &Tuning) -> usize {
    if at_risk {
        3
    } else if demand > 1.0 {
        2
    } else if demand > tuning.vehicle.comfort_demand {
        1
    } else {
        0
    }
}

/// Updates `car.stability` after a vehicle step (`prev_demand` is the grip demand before
/// it) and runs a risk check when one is due. `rng` is `None` in `StepMode::Ideal`.
pub fn update(car: &mut Car, prev_demand: f32, tuning: &Tuning, dt: f32, rng: Option<&mut SimRng>) -> Option<Check> {
    let st = &tuning.stability;
    if car.slide.is_some() {
        car.risk_timer = 0.0;
        return None;
    }
    let demand = car.grip_demand;
    car.stability = next(car.stability, demand, prev_demand, car.pilot.skill, tuning, dt);

    let speed = car.vel.length();
    if !at_risk(demand, car.stability, speed, tuning) {
        car.risk_timer = 0.0;
        return None;
    }
    let certain = car.stability <= 0.0;
    car.risk_timer += dt;
    if !certain && car.risk_timer < st.risk_interval {
        return None;
    }
    car.risk_timer = 0.0;

    let mut check = Check { chance: chance(car.stability, tuning), spin_chance: spin_chance(speed, tuning), lost: None };
    match rng {
        Some(rng) => {
            if rng.next_f32() < check.chance {
                let kind = if rng.next_f32() < check.spin_chance { LossKind::Spin } else { LossKind::Skid };
                lose_control(car, kind, tuning);
                check.lost = Some(kind);
            }
        }
        // Ideal: never simulate the slide, but don't keep reporting the same certain loss.
        None if certain => car.stability = st.after_loss,
        None => {}
    }
    Some(check)
}

/// The rear lets go: slide state plus a yaw kick into the current turn.
pub fn lose_control(car: &mut Car, kind: LossKind, tuning: &Tuning) {
    let st = &tuning.stability;
    let kick = match kind {
        LossKind::Skid => st.skid_yaw_kick,
        LossKind::Spin => st.spin_yaw_kick,
    };
    let turning = if car.yaw_rate.abs() > 1e-3 { car.yaw_rate } else { car.steer_angle };
    car.yaw_rate += kick * if turning < 0.0 { -1.0 } else { 1.0 };
    car.slide = Some(SlideState::default());
    car.stability = st.after_loss;
    car.risk_timer = 0.0;
}
