//! Inverse of the drag tool: find the turn and duration that bring a car closest to a
//! point. Candidates run through `step()` in `StepMode::Ideal`: the tires never understeer
//! or slide, so each turn value lands somewhere distinct and the fit is a stable inverse.
//! Where the plan needs more grip than the car has, the live run differs from the fitted
//! path, and the preview's grip coloring warns about it.

use std::cmp::Ordering;
use std::f32::consts::TAU;

use glam::Vec2;

use crate::orders::{Drive, OrderQueue, Segment, Turn, TurnMode};
use crate::preview::CarFrame;
use crate::state::{Car, CarId, DriveMode, SimState};
use crate::tuning::Tuning;
use crate::vehicle::wrap_angle;
use crate::{StepMode, World, step};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fit {
    pub turn: Turn,
    /// Ticks from the start state to the point closest to the target (at least 1).
    pub ticks: u32,
    /// Distance left between the car and the target at that tick.
    pub miss: f32,
}

impl Fit {
    fn cmp(&self, other: &Fit) -> Ordering {
        self.miss.total_cmp(&other.miss)
    }
}

/// Where a keypoint can be dragged: rows of candidate turns, each a run of reachable
/// points (`None` where a candidate isn't valid) with the worst grip zone on the way there
/// (`stability::zone`). Neighboring rows and columns are neighbors on the ground.
pub type ReachMap = Vec<Vec<Option<(Vec2, usize)>>>;

/// Coarse samples across the turn range before refining.
const COARSE: usize = 61;
const REFINE_STEPS: usize = 16;
/// Local minima of the coarse scan that get refined.
const MAX_MINIMA: usize = 8;

/// Searches turn values in `mode` (constant `throttle`) and durations up to `max_ticks`
/// for the segment that brings `car` closest to `target`, starting from `state` and
/// assuming unlimited grip. Ties keep the earliest tick, so a car that stops short of the
/// target gets its keypoint where it stops.
///
/// `turned` is the net heading change since the preview started (radians, + = left). A
/// candidate ends before that reaches a full circle, so keypoints stay on the first loop.
#[allow(clippy::too_many_arguments)]
pub fn fit_segment(
    state: &SimState,
    world: &World,
    tuning: &Tuning,
    car: CarId,
    mode: TurnMode,
    throttle: f32,
    target: Vec2,
    max_ticks: u32,
    turned: f32,
) -> Fit {
    let eval = |value: f32| -> Fit {
        let turn = Turn::new(mode, value);
        let mut best = Fit { turn, ticks: 1, miss: f32::INFINITY };
        run_candidate(state, world, tuning, car, Drive { turn, throttle }, max_ticks, turned, |t, c| {
            let fit = Fit { turn, ticks: t, miss: c.pose.pos.distance(target) };
            if fit.cmp(&best).is_lt() {
                best = fit;
            }
        });
        best
    };

    let samples = turn_samples(state, tuning, car, mode, COARSE);
    let coarse: Vec<Fit> = samples.iter().map(|&v| eval(v)).collect();

    // Refine around the most promising local minima (plateaus count once per run).
    let mut minima: Vec<usize> = (0..COARSE)
        .filter(|&i| {
            let m = &coarse[i];
            (i == 0 || m.cmp(&coarse[i - 1]).is_lt()) && (i + 1 == COARSE || m.cmp(&coarse[i + 1]).is_le())
        })
        .collect();
    minima.sort_by(|&a, &b| coarse[a].cmp(&coarse[b]));
    minima.truncate(MAX_MINIMA);
    minima
        .iter()
        .map(|&i| {
            let lo = samples[i.saturating_sub(1)];
            let hi = samples[(i + 1).min(COARSE - 1)];
            refine(&eval, lo, hi, coarse[i])
        })
        .min_by(Fit::cmp)
        .expect("the coarse scan has at least one minimum")
}

/// Every place the drag tool can put the keypoint: for `n` turn values spread over the
/// same range `fit_segment` searches, the car's position after each tick of the segment
/// and the worst grip zone along the way (`stability::zone`); `sweep[i][t - 1]` is turn
/// `i` after `t` ticks, turns run from hard right to hard left. Same limits as the fit:
/// unlimited grip, `max_ticks`, first loop only.
#[allow(clippy::too_many_arguments)]
pub fn sweep(
    state: &SimState,
    world: &World,
    tuning: &Tuning,
    car: CarId,
    mode: TurnMode,
    throttle: f32,
    max_ticks: u32,
    turned: f32,
    n: usize,
) -> ReachMap {
    turn_samples(state, tuning, car, mode, n)
        .into_iter()
        .map(|value| {
            let mut points = Vec::new();
            let mut worst = 0;
            let drive = Drive { turn: Turn::new(mode, value), throttle };
            run_candidate(state, world, tuning, car, drive, max_ticks, turned, |_, c| {
                worst = worst.max(CarFrame::of(c).zone(tuning));
                points.push(Some((c.pose.pos, worst)));
            });
            points
        })
        .collect()
}

/// `n` turn values (at least 2) from hard right to hard left. Turns go up to
/// `max_lateral_g`; in steering and turn-rate (swerve) mode that is the steering / yaw rate
/// giving that much lateral g at the start speed. Grip-limited steering at speed is a small fraction of full lock, so
/// steering samples are spaced cubically (dense near zero).
fn turn_samples(state: &SimState, tuning: &Tuning, car: CarId, mode: TurnMode, n: usize) -> Vec<f32> {
    let n = n.max(2);
    let max_g = tuning.planner.max_lateral_g;
    let v0 = state.car(car).map_or(0.0, |c| c.forward_speed().max(0.0));
    let max_steer = Turn::Lateral(max_g).steer(v0, &tuning.vehicle).abs();
    (0..n)
        .map(|i| -1.0 + 2.0 * i as f32 / (n - 1) as f32)
        .map(|u| match mode {
            TurnMode::Steer => u * u * u * max_steer,
            TurnMode::Lateral => u * max_g,
            TurnMode::Swerve => u * max_g * tuning.vehicle.gravity / v0.max(1.0),
        })
        .collect()
}

/// Drives `car` from `state` with constant `drive` in `StepMode::Ideal`, calling
/// `visit(tick, car)` after each tick, for up to `max_ticks` (at least 1) or until its
/// net heading change (starting at `turned`) reaches a full circle.
#[allow(clippy::too_many_arguments)]
fn run_candidate(
    state: &SimState,
    world: &World,
    tuning: &Tuning,
    car: CarId,
    drive: Drive,
    max_ticks: u32,
    mut turned: f32,
    mut visit: impl FnMut(u32, &Car),
) {
    let mut s = state.clone();
    if let Some(c) = s.car_mut(car) {
        c.drive_mode = DriveMode::Orders;
        // As a segment (not the tail) so swerves turn instead of driving straight.
        c.orders = OrderQueue { segments: vec![Segment::new(drive, max_ticks.saturating_add(1))], tail: drive };
    }
    let mut heading = s.car(car).map_or(0.0, |c| c.pose.heading);
    for t in 1..=max_ticks.max(1) {
        step(&mut s, world, tuning, StepMode::Ideal);
        let Some(c) = s.car(car) else { break };
        turned += wrap_angle(c.pose.heading - heading);
        heading = c.pose.heading;
        if turned.abs() >= TAU {
            break;
        }
        visit(t, c);
    }
}

/// Golden-section search for the best fit (see `Fit::cmp`) within `[lo, hi]`.
fn refine(eval: &impl Fn(f32) -> Fit, mut lo: f32, mut hi: f32, start: Fit) -> Fit {
    let phi = 0.618_034;
    let mut a = eval(hi - phi * (hi - lo));
    let mut b = eval(lo + phi * (hi - lo));
    for _ in 0..REFINE_STEPS {
        if a.cmp(&b).is_lt() {
            hi = b.turn.value();
            b = a;
            a = eval(hi - phi * (hi - lo));
        } else {
            lo = a.turn.value();
            a = b;
            b = eval(lo + phi * (hi - lo));
        }
    }
    [a, b, start].into_iter().min_by(Fit::cmp).expect("three candidates")
}
