//! Inverse of the drag tool: find the turn and duration that bring a car closest to a
//! point. Candidates run through `step()` in `StepMode::Ideal`: the tires never understeer
//! or slide, so each turn value lands somewhere distinct and the fit is a stable inverse.
//! Where the plan needs more grip than the car has, the live run differs from the fitted
//! path, and the preview's grip coloring warns about it.

use std::cmp::Ordering;

use glam::Vec2;

use crate::orders::{Drive, OrderQueue, Turn, TurnMode};
use crate::state::{CarId, DriveMode, SimState};
use crate::tuning::Tuning;
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

/// Coarse samples across the turn range before refining.
const COARSE: usize = 61;
const REFINE_STEPS: usize = 16;
/// Local minima of the coarse scan that get refined.
const MAX_MINIMA: usize = 8;

/// Searches turn values in `mode` (constant `throttle`) and durations up to `max_ticks`
/// for the segment that brings `car` closest to `target`, starting from `state` and
/// assuming unlimited grip. Ties keep the earliest tick, so a car that stops short of the
/// target gets its keypoint where it stops.
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
) -> Fit {
    let max_ticks = max_ticks.max(1);
    let eval = |value: f32| -> Fit {
        let turn = Turn::new(mode, value);
        let mut s = state.clone();
        if let Some(c) = s.car_mut(car) {
            c.drive_mode = DriveMode::Orders;
            c.orders = OrderQueue { segments: Vec::new(), tail: Drive { turn, throttle } };
        }
        let mut best = Fit { turn, ticks: 1, miss: f32::INFINITY };
        for t in 1..=max_ticks {
            step(&mut s, world, tuning, StepMode::Ideal);
            let Some(c) = s.car(car) else { break };
            let fit = Fit { turn, ticks: t, miss: c.pose.pos.distance(target) };
            if fit.cmp(&best).is_lt() {
                best = fit;
            }
        }
        best
    };

    // Grip-limited steering at speed is a small fraction of full lock, so steering samples
    // are spaced cubically (dense near zero). Lateral g is already in grip units.
    let samples: Vec<f32> = (0..COARSE)
        .map(|i| -1.0 + 2.0 * i as f32 / (COARSE - 1) as f32)
        .map(|u| match mode {
            TurnMode::Steer => u * u * u,
            TurnMode::Lateral => u * tuning.planner.max_lateral_g,
        })
        .collect();
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
