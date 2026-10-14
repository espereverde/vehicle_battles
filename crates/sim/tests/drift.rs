//! Drift keypoints: an S-curve of two arcs of equal length, ending pointing the way the car
//! started, charged as the book's drift.

use sim::drift::{self, MAX_RATIO};
use sim::handling::INCH;
use sim::{Drive, OrderQueue, Pose, Segment, SimState, StepMode, Tuning, Turn, Vec2, World, fit_drift};

fn car_at(speed: f32) -> SimState {
    let mut s = SimState::new(1);
    s.spawn_car(Pose::default());
    s.cars[0].vel = Vec2::new(speed, 0.0);
    s
}

/// What a drift segment did: end pose, distance driven, distance at which the turn
/// direction flipped, handling drained.
struct Run {
    end: Pose,
    distance: f32,
    flip_at: Option<f32>,
    drained: f32,
}

fn run(start: &SimState, ratio: f32, throttle: f32, ticks: u32, tuning: &Tuning) -> Run {
    let mut s = start.clone();
    let drive = Drive { turn: Turn::Drift(ratio), throttle };
    s.cars[0].orders = OrderQueue { segments: vec![Segment::new(drive, ticks)], tail: drive };
    let (mut distance, mut flip_at, mut drained, mut last_yaw) = (0.0, None, 0.0, 0.0f32);
    for _ in 0..ticks {
        let before = s.cars[0].pose.pos;
        sim::step(&mut s, &World::default(), tuning, StepMode::Ideal);
        let c = &s.cars[0];
        if flip_at.is_none() && c.yaw_rate * last_yaw < 0.0 {
            flip_at = Some(distance);
        }
        if c.yaw_rate != 0.0 {
            last_yaw = c.yaw_rate;
        }
        distance += c.pose.pos.distance(before);
        drained += c.drain;
    }
    Run { end: s.cars[0].pose, distance, flip_at, drained }
}

/// The heading comes back at any speed, accelerating, coasting or braking to a stop
/// (regression: equal-time halves ended up to 80° off when the car slowed down).
#[test]
fn ends_with_the_start_heading_at_any_speed() {
    let tuning = Tuning::default();
    for v in [2.0, 6.0, 12.0, 25.0] {
        for throttle in [-0.5, -0.1, 0.0, 0.6] {
            for (ratio, ticks) in [(0.5, 120), (-0.2, 60), (0.35, 240)] {
                let r = run(&car_at(v), ratio, throttle, ticks, &tuning);
                let case = format!("{v} m/s, throttle {throttle}, ratio {ratio}, {ticks} ticks");
                assert!(r.end.heading.abs() < 0.2f32.to_radians(), "{case}: heading {}°", r.end.heading.to_degrees());
            }
        }
    }
}

/// The turn flips halfway along the path, not halfway in time.
#[test]
fn halves_have_equal_length() {
    let tuning = Tuning::default();
    for (v, throttle) in [(4.0, 0.8), (20.0, -0.6), (10.0, 0.0)] {
        let r = run(&car_at(v), 0.3, throttle, 150, &tuning);
        let flip = r.flip_at.expect("turns both ways");
        assert!((flip - r.distance / 2.0).abs() < 0.03 * r.distance, "{v} m/s: flip at {flip} m of {}", r.distance);
    }
}

/// It moves `ratio` sideways per forward, turning `2·atan(ratio)` on each arc.
#[test]
fn moves_sideways_at_its_ratio() {
    let tuning = Tuning::default();
    for ratio in [0.1, 0.25, -0.5] {
        let r = run(&car_at(15.0), ratio, 0.2, 150, &tuning);
        let got = r.end.pos.y / r.end.pos.x;
        assert!((got - ratio).abs() < 0.01, "ratio {ratio}: moved {got}");
        assert!((r.end.pos.x - r.distance * drift::forward_per_distance(ratio)).abs() < 0.05);
    }
}

/// Charged as the book's drift, per inch forward by steepness (not by its heading
/// changes): twice as long at the same ratio costs twice as much.
#[test]
fn costs_by_steepness_and_length() {
    let tuning = Tuning::default();
    for (ratio, per_inch) in [(0.125, 0.5), (0.25, 1.0), (0.375, 2.0), (0.5, 3.0)] {
        assert!((drift::cost_per_inch(ratio, &tuning) - per_inch).abs() < 1e-5);
        let v = 15.0;
        let hold = sim::hold_throttle(v, &tuning.vehicle);
        let short = run(&car_at(v), ratio, hold, 60, &tuning);
        let long = run(&car_at(v), ratio, hold, 120, &tuning);
        let expected = per_inch * short.end.pos.x / INCH;
        assert!((short.drained - expected).abs() < 0.02 * expected, "ratio {ratio}: drained {} expected {expected}", short.drained);
        assert!((long.drained / short.drained - 2.0).abs() < 0.05, "ratio {ratio}: {} vs {}", long.drained, short.drained);
    }
    assert_eq!(drift::cost_per_inch(0.9, &tuning), 3.0, "never steeper than ½");
}

/// The fit reaches targets within ½ sideways per forward, caps steeper ones at ½, and
/// lands where the simulation goes.
#[test]
fn fit_reaches_targets_within_the_steepest_drift() {
    let tuning = Tuning::default();
    for (v, throttle, x, y) in [(15.0, 0.2, 30.0, 3.5), (15.0, 0.2, 30.0, -12.0), (8.0, 0.0, 20.0, 3.0), (20.0, -0.3, 40.0, 8.0), (25.0, 0.0, 60.0, -2.0), (3.0, 0.5, 20.0, 5.0)] {
        let start = car_at(v);
        let target = Vec2::new(x, y);
        let fit = fit_drift(&start, &tuning, start.cars[0].id, throttle, target, 300);
        let Turn::Drift(ratio) = fit.turn else { panic!("not a drift") };
        let r = run(&start, ratio, throttle, fit.ticks, &tuning);
        let case = format!("{v} m/s -> ({x}, {y}): {fit:?}");
        assert!(ratio.abs() <= MAX_RATIO, "{case}");
        assert!(r.end.pos.distance(target) - fit.miss < 0.2, "{case}: sim ends at {}", r.end.pos);
        if (y / x).abs() <= MAX_RATIO {
            assert!(fit.miss < 0.3, "{case}");
        } else {
            assert!((ratio.abs() - MAX_RATIO).abs() < 1e-4, "{case}: steeper than ½ should cap");
        }
    }
}

/// Short drifts are no sharper than the steering lock allows, so they stay exact.
#[test]
fn fit_respects_the_steering_lock() {
    let tuning = Tuning::default();
    let start = car_at(5.0);
    let fit = fit_drift(&start, &tuning, start.cars[0].id, 0.0, Vec2::new(4.0, 2.0), 300);
    let Turn::Drift(ratio) = fit.turn else { panic!() };
    let length = drift::travel(&Drive { turn: fit.turn, throttle: 0.0 }, 5.0, fit.ticks, &tuning.vehicle, tuning.dt());
    let curvature = 2.0 * drift::arc_angle(ratio) / length;
    assert!(curvature <= drift::max_curvature(&tuning.vehicle) * 1.001, "{fit:?}: curvature {curvature}");
    let r = run(&start, ratio, 0.0, fit.ticks, &tuning);
    assert!(r.end.heading.abs() < 0.2f32.to_radians());
}

/// Dragging the target sideways moves the keypoint smoothly.
#[test]
fn drag_is_continuous() {
    let tuning = Tuning::default();
    let start = car_at(15.0);
    let mut last: Option<Vec2> = None;
    for j in -40..=40 {
        let target = Vec2::new(30.0, j as f32 * 0.5);
        let fit = fit_drift(&start, &tuning, start.cars[0].id, 0.2, target, 300);
        let Turn::Drift(ratio) = fit.turn else { panic!() };
        let end = run(&start, ratio, 0.2, fit.ticks, &tuning).end.pos;
        if let Some(prev) = last {
            assert!(prev.distance(end) < 1.0, "jumped from {prev} to {end} at y = {}", target.y);
        }
        last = Some(end);
    }
}
