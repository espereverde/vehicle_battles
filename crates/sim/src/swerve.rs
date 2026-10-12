//! Swerve fit: two curves of equal and opposite turn rate and equal duration
//! (`Turn::Swerve`), so the car ends pointing the way it started, shifted sideways.
//!
//! In the frame of the swerve's start, a longer swerve goes further forward and, for a
//! given length, a higher turn rate goes further sideways. So the fit is two nested
//! bisections on the real (ideal-mode) simulation: on the half-length to match the target's
//! forward distance, and inside it on the turn rate to match its sideways offset.

use std::f32::consts::FRAC_PI_2;

use glam::Vec2;

use crate::fit::{Fit, ReachMap};
use crate::orders::{Drive, OrderQueue, Segment, Turn};
use crate::preview::CarFrame;
use crate::state::{CarId, DriveMode, Pose, SimState};
use crate::tuning::Tuning;
use crate::{StepMode, World, step};

const BISECTIONS: usize = 14;

/// Drives a swerve at `rate` with halves of `half` ticks from `state`; calls `visit` with
/// the car after each tick and returns the final pose.
#[allow(clippy::too_many_arguments)]
fn run(
    state: &SimState,
    world: &World,
    tuning: &Tuning,
    car: CarId,
    rate: f32,
    throttle: f32,
    half: u32,
    mut visit: impl FnMut(&CarFrame),
) -> Option<Pose> {
    let mut s = state.clone();
    let c = s.car_mut(car)?;
    c.drive_mode = DriveMode::Orders;
    let drive = Drive { turn: Turn::Swerve(rate), throttle };
    c.orders = OrderQueue { segments: vec![Segment::new(drive, 2 * half)], tail: drive };
    for _ in 0..2 * half {
        step(&mut s, world, tuning, StepMode::Ideal);
        visit(&CarFrame::of(s.car(car)?));
    }
    Some(s.car(car)?.pose)
}

/// Highest turn rate for a swerve with halves of `half` ticks: `max_lateral_g` at the start
/// speed, and no more than a quarter turn per half (beyond that, more rate no longer means
/// further sideways).
fn max_rate(v0: f32, half: u32, tuning: &Tuning) -> f32 {
    let g_cap = tuning.planner.max_lateral_g * tuning.vehicle.gravity / v0.max(1.0);
    g_cap.min(FRAC_PI_2 / (half.max(1) as f32 * tuning.dt()))
}

/// Finds the swerve (turn rate, total length up to `max_ticks`) at constant `throttle` that
/// ends closest to `target`, assuming unlimited grip.
pub fn fit_swerve(state: &SimState, world: &World, tuning: &Tuning, car: CarId, throttle: f32, target: Vec2, max_ticks: u32) -> Fit {
    let fallback = Fit { turn: Turn::Swerve(0.0), ticks: 2, miss: f32::INFINITY };
    let Some(start) = state.car(car).map(|c| c.pose) else { return fallback };
    let v0 = state.car(car).map_or(0.0, |c| c.forward_speed().max(0.0));
    // Target in the start frame: forward and to the left.
    let local = Vec2::from_angle(-start.heading).rotate(target - start.pos);
    let end_of = |rate: f32, half: u32| {
        run(state, world, tuning, car, rate, throttle, half, |_| {}).map_or(Vec2::ZERO, |p| Vec2::from_angle(-start.heading).rotate(p.pos - start.pos))
    };
    // For a given half-length: the rate whose end is level with the target sideways.
    let rate_for = |half: u32| {
        let max = max_rate(v0, half, tuning);
        let (mut lo, mut hi) = (-max, max);
        for _ in 0..BISECTIONS {
            let mid = 0.5 * (lo + hi);
            if end_of(mid, half).y < local.y { lo = mid } else { hi = mid }
        }
        0.5 * (lo + hi)
    };
    // The half-length whose end is level with the target forward.
    let (mut lo, mut hi) = (1, (max_ticks / 2).max(1));
    while lo < hi {
        let mid = (lo + hi) / 2;
        if end_of(rate_for(mid), mid).x < local.x { lo = mid + 1 } else { hi = mid }
    }
    // Whole ticks: the neighbor below may be closer.
    [lo.saturating_sub(1).max(1), lo]
        .into_iter()
        .map(|half| {
            let rate = rate_for(half);
            let miss = end_of(rate, half).distance(local);
            Fit { turn: Turn::Swerve(rate), ticks: 2 * half, miss }
        })
        .min_by(|a, b| a.miss.total_cmp(&b.miss))
        .unwrap_or(fallback)
}

/// Every place a swerve keypoint can be dragged to: `n` turn rates (hard right-then-left to
/// hard left-then-right) by half-length (every `step_ticks`), with the worst grip zone along
/// each path. One simulation per point, so coarser than the turn sweeps.
#[allow(clippy::too_many_arguments)]
pub fn sweep_swerve(state: &SimState, world: &World, tuning: &Tuning, car: CarId, throttle: f32, max_ticks: u32, n: usize, step_ticks: u32) -> ReachMap {
    let v0 = state.car(car).map_or(0.0, |c| c.forward_speed().max(0.0));
    let n = n.max(2);
    let halves: Vec<u32> = (1..=max_ticks / 2).filter(|h| h % step_ticks.max(1) == 0).collect();
    (0..n)
        .map(|i| -1.0 + 2.0 * i as f32 / (n - 1) as f32)
        .map(|u| {
            halves
                .iter()
                .map(|&half| {
                    let rate = u * max_rate(v0, half, tuning);
                    let mut worst = 0;
                    let end = run(state, world, tuning, car, rate, throttle, half, |f| worst = worst.max(f.zone(tuning)))?;
                    Some((end.pos, worst))
                })
                .collect()
        })
        .collect()
}
