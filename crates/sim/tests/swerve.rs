//! Swerve keypoints: two curves of equal and opposite turn rate and equal duration, ending
//! pointing the way the car started.

use sim::{Drive, OrderQueue, Pose, Segment, SimState, StepMode, Tuning, Turn, Vec2, World, fit_swerve, sweep_swerve};

fn car_at(speed: f32) -> SimState {
    let mut s = SimState::new(1);
    s.spawn_car(Pose::default());
    s.cars[0].vel = Vec2::new(speed, 0.0);
    s
}

/// Runs the swerve in the ideal model; the end pose and the yaw rates seen.
fn run(start: &SimState, turn: Turn, throttle: f32, ticks: u32, tuning: &Tuning) -> (Pose, Vec<f32>) {
    let mut s = start.clone();
    s.cars[0].orders = OrderQueue { segments: vec![Segment::new(Drive { turn, throttle }, ticks)], tail: Drive::COAST };
    let rates = (0..ticks)
        .map(|_| {
            sim::step(&mut s, &World::default(), tuning, StepMode::Ideal);
            s.cars[0].yaw_rate
        })
        .collect();
    (s.cars[0].pose, rates)
}

#[test]
fn reaches_the_target_keeping_the_heading() {
    let tuning = Tuning::default();
    for (v, throttle, x, y) in [(15.0, 0.2, 30.0, 3.5), (15.0, 0.2, 30.0, -3.5), (8.0, 0.2, 20.0, 3.0), (20.0, -0.3, 45.0, 8.0), (0.0, 0.5, 20.0, 3.0), (25.0, 0.0, 60.0, -2.0)] {
        let start = car_at(v);
        let target = Vec2::new(x, y);
        let t = std::time::Instant::now();
        let fit = fit_swerve(&start, &World::default(), &tuning, start.cars[0].id, throttle, target, 300);
        let elapsed = t.elapsed();
        let (end, rates) = run(&start, fit.turn, throttle, fit.ticks, &tuning);
        let case = format!("{v} m/s -> ({x}, {y}): {fit:?} in {elapsed:?}");
        println!("{case}");
        assert!(end.pos.distance(target) < 0.3, "{case}: ended at {}", end.pos);
        assert!(end.heading.abs() < 2f32.to_radians(), "{case}: heading {}°", end.heading.to_degrees());

        // Equal halves of equal and opposite turn rate (away from the steering-lag edges).
        let Turn::Swerve(rate) = fit.turn else { panic!("{case}: not a swerve") };
        assert_eq!(fit.ticks % 2, 0, "{case}");
        let half = fit.ticks as usize / 2;
        if half > 20 && v > 5.0 {
            let (a, b) = (rates[half - 5], rates[fit.ticks as usize - 5]);
            assert!((a - rate).abs() < 0.05 * rate.abs() + 0.01, "{case}: first curve at {a} rad/s");
            assert!((b + rate).abs() < 0.05 * rate.abs() + 0.01, "{case}: second curve at {b} rad/s");
        }
    }
}

/// Dragging the target sideways moves the keypoint smoothly (no jumps between solutions).
#[test]
fn drag_is_continuous() {
    let tuning = Tuning::default();
    let start = car_at(15.0);
    let mut last: Option<Vec2> = None;
    for j in -16..=16 {
        let target = Vec2::new(30.0, j as f32 * 0.5);
        let fit = fit_swerve(&start, &World::default(), &tuning, start.cars[0].id, 0.2, target, 300);
        let (end, _) = run(&start, fit.turn, 0.2, fit.ticks, &tuning);
        if let Some(prev) = last {
            assert!(prev.distance(end.pos) < 1.0, "jumped from {prev} to {} at y = {}", end.pos, target.y);
        }
        last = Some(end.pos);
    }
}

/// The danger-zone map covers both sides; straight is green, sharp swerves reach red.
#[test]
fn sweep_covers_both_sides() {
    let tuning = Tuning::default();
    let start = car_at(15.0);
    let t = std::time::Instant::now();
    let rows = sweep_swerve(&start, &World::default(), &tuning, start.cars[0].id, 0.2, 300, 21, 5);
    println!("sweep in {:?}", t.elapsed());
    assert_eq!(rows.len(), 21);
    let points: Vec<(Vec2, usize)> = rows.iter().flatten().flatten().copied().collect();
    assert!(points.iter().any(|(p, _)| p.y > 5.0) && points.iter().any(|(p, _)| p.y < -5.0));
    assert!(rows[10].iter().flatten().all(|&(p, z)| z == 0 && p.y.abs() < 0.1));
    assert!(rows[0].iter().flatten().any(|&(_, z)| z == 3) && rows[20].iter().flatten().any(|&(_, z)| z == 3));
}

/// Short, sharp swerves (where the steering rate matters) and swerves that start with the
/// wheels still turned from the previous keypoint end with the start heading and the wheels
/// straight, so the car carries on straight after the keypoint.
#[test]
fn sharp_swerves_keep_the_heading() {
    let tuning = Tuning::default();
    let max = tuning.vehicle.max_steer_deg.to_radians();
    for (wheels, v, x, y) in [(0.0, 15.0, 12.0, 3.0), (0.0, 15.0, 8.0, 2.0), (0.0, 10.0, 6.0, 2.0), (0.0, 20.0, 15.0, 4.0), (0.0, 8.0, 5.0, 2.5), (0.5, 15.0, 12.0, 3.0), (-0.5, 15.0, 12.0, 3.0), (-0.8, 10.0, 15.0, 2.0)] {
        let mut start = car_at(v);
        start.cars[0].steer_angle = wheels * max;
        let fit = fit_swerve(&start, &World::default(), &tuning, start.cars[0].id, 0.2, Vec2::new(x, y), 300);
        let mut s = start.clone();
        s.cars[0].orders = OrderQueue { segments: vec![Segment::new(Drive { turn: fit.turn, throttle: 0.2 }, fit.ticks)], tail: Drive { turn: fit.turn, throttle: 0.2 } };
        for _ in 0..fit.ticks {
            sim::step(&mut s, &World::default(), &tuning, StepMode::Ideal);
        }
        let case = format!("wheels {wheels}, {v} m/s -> ({x}, {y}): {fit:?}");
        let car = &s.cars[0];
        assert!(car.pose.heading.abs() < 1f32.to_radians(), "{case}: heading {}°", car.pose.heading.to_degrees());
        assert!(car.steer_angle.abs() < 0.5f32.to_radians(), "{case}: wheels at {}°", car.steer_angle.to_degrees());
    }
}
