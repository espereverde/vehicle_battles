//! Handling status and control checks, after *Car Wars* (Movement chapter): maneuvers drain
//! handling status, it recovers over time, and every point drained is a control check whose
//! odds come from the Control Table, here a formula.
//!
//! - Handling status (HS) starts at the car's handling class (HC) and never exceeds it; it
//!   recovers continuously at `max(1, HC + driver skill)` points per second and never drops
//!   below `min_status` (−6).
//! - Maneuvers drain it by geometry, independent of speed: one point per `bend_deg` of
//!   heading change; a drift by its steepness per inch forward instead (`drift.rs`; its own
//!   heading changes are free); hard braking (beyond `free_braking_mph_s`) by the book's
//!   table; a controlled skid adds its surcharge when it starts.
//! - Control check, each tick: `p_fail(HS, v)` is the chance that one book control roll
//!   fails; draining `dD` points this tick loses control with `1 − (1 − p_fail)^dD`. So one
//!   point = one roll, independent of tick rate; straight driving at a steady speed is safe.
//!   Checks use the demanded drain, not the actual HS change, so they go on at the floor.
//! - Discrete checks (`hazard`) drop HS by a difficulty, possibly 0, and check once.
//! - A lost check rolls on the crash tables (`crash.rs`) with the maneuver's difficulty: its
//!   drain per inch (one book maneuver per inch), plus hard braking's per second.
//!
//! Only `StepMode::Live` and `Preview` roll the dice (from the state's RNG, so the preview
//! check stays bitwise exact). `Ideal` never does: it reports each check's chance. A sliding
//! car (crash result or controlled skid) makes no checks.

use crate::crash::{self, Crash};
use crate::rng::SimRng;
use crate::state::Car;
use crate::tuning::Tuning;

pub const MPH: f32 = 2.236_936;
/// One Car Wars inch: 15 ft.
pub const INCH: f32 = 4.572;

/// One control check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Check {
    /// Chance that this check makes the car lose control (1.0 = certain).
    pub chance: f32,
    /// If it does, chance that the result is a spinout or worse.
    pub severe_chance: f32,
    /// What happened (rolled modes only): the crash result (for a hazard, the table 1 result
    /// if it rolled one, else the fishtail).
    pub lost: Option<Crash>,
}

/// Handling class: the car's starting and highest handling status.
pub fn class(tuning: &Tuning) -> f32 {
    tuning.vehicle.handling_class
}

/// Current handling status.
pub fn status(car: &Car, tuning: &Tuning) -> f32 {
    class(tuning) - car.handling_lost
}

/// Points recovered per second.
pub fn recovery(car: &Car, tuning: &Tuning) -> f32 {
    (class(tuning) + car.pilot.skill).max(1.0)
}

/// Control Table: chance that one control roll fails at handling status `hs` and `speed`
/// (m/s). Linear in speed above a safe speed that grows with `hs`.
pub fn p_fail(hs: f32, speed: f32, tuning: &Tuning) -> f32 {
    let h = &tuning.handling;
    ((speed.abs() * MPH - h.safe_speed_mph - h.mph_per_status * hs) / h.fail_range_mph).clamp(0.0, 1.0)
}

/// Chance of losing control while draining `drain` points at `hs` and `speed`.
pub fn chance(drain: f32, hs: f32, speed: f32, tuning: &Tuning) -> f32 {
    if drain <= 0.0 {
        return 0.0;
    }
    let p = p_fail(hs, speed, tuning);
    if p >= 1.0 { 1.0 } else { 1.0 - (1.0 - p).powf(drain) }
}

/// Drain per second from braking at `decel` m/s²: free up to `free_braking_mph_s`, then
/// the book's progression (15 mph/s → 1, 20 → 2, 25 → 3, then 2 more per 5 mph/s).
pub fn braking_drain(decel: f32, tuning: &Tuning) -> f32 {
    let over = decel * MPH - tuning.handling.free_braking_mph_s;
    if over <= 0.0 {
        0.0
    } else if over <= 15.0 {
        over / 5.0
    } else {
        3.0 + (over - 15.0) * 2.0 / 5.0
    }
}

/// Handling points a tick of maneuvering drains: `heading_change` (radians) as a bend, or,
/// while drifting (`drift`: ratio and distance covered), the drift's cost instead.
pub fn maneuver_drain(heading_change: f32, drift: Option<(f32, f32)>, tuning: &Tuning) -> f32 {
    match drift {
        Some((ratio, distance)) => crate::drift::cost_per_inch(ratio, tuning) * distance * crate::drift::forward_per_distance(ratio) / INCH,
        None => heading_change.abs() / tuning.handling.bend_deg.to_radians(),
    }
}

/// Difficulty of what the car is doing, for the crash tables: `maneuver` points drained
/// over `distance` meters, per inch (one book maneuver per inch), plus braking at `decel`
/// per second.
pub fn difficulty(maneuver: f32, distance: f32, decel: f32, tuning: &Tuning) -> f32 {
    let per_inch = if distance > 1e-4 { maneuver * INCH / distance } else { 0.0 };
    per_inch + braking_drain(decel, tuning)
}

/// Updates the car's handling after a vehicle step: drains `drain` points, checks control at
/// the drained status, then recovers. A lost check rolls the crash tables with `difficulty`
/// (a controlled skid starting this tick is replaced by the crash). No checks while a slide
/// that was already going on (`sliding`) continues. Returns the check when there was any
/// drain. `rng` is `None` in `StepMode::Ideal`.
pub fn update(car: &mut Car, drain: f32, difficulty: f32, sliding: bool, tuning: &Tuning, dt: f32, rng: Option<&mut SimRng>) -> Option<Check> {
    let floor = class(tuning) - tuning.handling.min_status;
    car.handling_lost = (car.handling_lost + drain).min(floor);
    car.drain = drain;
    let speed = car.vel.length();
    let hs = status(car, tuning);
    car.handling_lost = (car.handling_lost - recovery(car, tuning) * dt).max(0.0);

    car.risk = if sliding { 0.0 } else { chance(drain, hs, speed, tuning) };
    if drain <= 0.0 || sliding {
        return None;
    }
    let mut check = Check { chance: car.risk, severe_chance: crash::severe_chance(difficulty, speed, car.pilot.skill), lost: None };
    if let Some(rng) = rng
        && check.chance > 0.0
        && rng.next_f32() < check.chance
    {
        check.lost = Some(crash::lose_control(car, difficulty, tuning, rng));
    }
    Some(check)
}

/// A discrete event (hazard, collision, edge) of `difficulty` (may be 0): drops handling
/// status by it and checks control once, at the dropped status, or at `forced` status if
/// given. A lost check rolls Crash Table 2.
pub fn hazard(car: &mut Car, difficulty: f32, forced: Option<f32>, tuning: &Tuning, rng: Option<&mut SimRng>) -> Check {
    let floor = class(tuning) - tuning.handling.min_status;
    car.handling_lost = (car.handling_lost + difficulty.max(0.0)).min(floor);
    let speed = car.vel.length();
    let hs = forced.unwrap_or_else(|| status(car, tuning));
    let severe_chance = crash::chance_at_least(9, crash::modifier(difficulty, speed, car.pilot.skill));
    let mut check = Check { chance: p_fail(hs, speed, tuning), severe_chance, lost: None };
    if let Some(rng) = rng
        && check.chance > 0.0
        && rng.next_f32() < check.chance
    {
        let (fishtail, then) = crash::lose_control_to_hazard(car, difficulty, tuning, rng);
        check.lost = Some(then.unwrap_or(fishtail));
    }
    check
}

/// Risk zone of a moment, as the path is colored: 0 green (no risk), 1 yellow (no risk
/// now, but maneuvering here would carry some), 2 orange (risk), 3 red (certain loss).
pub fn zone(hs: f32, speed: f32, risk: f32, tuning: &Tuning) -> usize {
    if risk >= 1.0 {
        3
    } else if risk > 0.0 {
        2
    } else if p_fail(hs, speed, tuning) > 0.0 {
        1
    } else {
        0
    }
}
