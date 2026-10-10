//! Path fitting, path following, preview-vs-reality and replays.

use std::f32::consts::FRAC_PI_2;

use sim::preview::Check;
use sim::{
    Command, OrderQueue, PathSpec, Pose, Preview, Replay, Segment, SimEvent, SimState, SpeedProfile,
    StepMode, Tuning, Vec2, World,
};

fn pose(x: f32, y: f32, heading_deg: f32) -> Pose {
    Pose::new(Vec2::new(x, y), heading_deg.to_radians())
}

/// Orders for car 0 through `ends`, chained from the car's pose.
fn orders(state: &SimState, ends: &[(Pose, f32, f32)], tuning: &Tuning) -> OrderQueue {
    let car = &state.cars[0];
    let mut q = OrderQueue::default();
    for &(end, cruise, end_speed) in ends {
        let (start, v0) = q.tail(car.pose, car.forward_speed());
        q.segments.push(Segment::path(start, end, SpeedProfile { start: v0, cruise, end: end_speed }, &tuning.planner));
    }
    q.rechain(car.pose, car.forward_speed(), &tuning.planner);
    q
}

fn run_ticks(state: &mut SimState, tuning: &Tuning, ticks: u32) -> Vec<SimEvent> {
    let world = World::default();
    (0..ticks).flat_map(|_| sim::step(state, &world, tuning, StepMode::Live).events).collect()
}

fn one_car() -> SimState {
    let mut s = SimState::new(7);
    s.spawn_car(Pose::default());
    s
}

#[test]
fn bezier_hits_both_poses_with_uniform_spacing() {
    let pt = Tuning::default().planner;
    let p = PathSpec::bezier(pose(0.0, 0.0, 0.0), pose(30.0, 20.0, 90.0), &pt);
    assert_eq!(p.points[0], Vec2::ZERO);
    assert_eq!(*p.points.last().unwrap(), Vec2::new(30.0, 20.0));
    for w in p.points.windows(2).take(p.points.len() - 2) {
        assert!((w[0].distance(w[1]) - pt.path_spacing).abs() < 1e-3);
    }
    assert!(p.tangent_at(0.0).dot(Vec2::X) > 0.99);
    assert!(p.tangent_at(p.length).dot(Vec2::Y) > 0.99);
    // Straight path: length equals the chord.
    let straight = PathSpec::bezier(pose(0.0, 0.0, 0.0), pose(50.0, 0.0, 0.0), &pt);
    assert!((straight.length - 50.0).abs() < 1e-2);
    assert!((straight.project(Vec2::new(20.0, 3.0), 0.0, 50.0) - 20.0).abs() < 1e-3);
}

#[test]
fn speed_profile_launches_cruises_and_arrives() {
    let ct = Tuning::default().control;
    let p = SpeedProfile { start: 0.0, cruise: 15.0, end: 0.0 };
    assert!(p.at(0.0, 200.0, &ct) > 0.0, "must launch from rest");
    assert_eq!(p.at(100.0, 200.0, &ct), 15.0);
    assert_eq!(p.at(200.0, 200.0, &ct), 0.0);
    let slow_down = SpeedProfile { start: 25.0, cruise: 10.0, end: 10.0 };
    assert!(slow_down.at(1.0, 200.0, &ct) > 20.0, "should brake gradually, not instantly");
}

#[test]
fn drives_to_a_stop_at_the_end_of_a_straight_path() {
    let tuning = Tuning::default();
    let mut s = one_car();
    s.cars[0].orders = orders(&s, &[(pose(40.0, 0.0, 0.0), 12.0, 0.0)], &tuning);
    let events = run_ticks(&mut s, &tuning, 15 * 60);
    let car = &s.cars[0];
    assert!(events.iter().any(|e| matches!(e, SimEvent::SegmentCompleted { remaining: 0, .. })));
    assert!(car.pose.pos.distance(Vec2::new(40.0, 0.0)) < 0.5, "stopped at {}", car.pose.pos);
    assert!(car.vel.length() < 0.1);
    assert!(car.orders.segments.is_empty() && car.orders.hold.is_some());
}

#[test]
fn follows_a_turn_and_parks_with_the_right_heading() {
    let tuning = Tuning::default();
    let mut s = one_car();
    let target = pose(25.0, 25.0, 90.0);
    s.cars[0].orders = orders(&s, &[(target, 10.0, 0.0)], &tuning);
    run_ticks(&mut s, &tuning, 15 * 60);
    let car = &s.cars[0];
    assert!(car.pose.pos.distance(target.pos) < 1.0, "parked at {}", car.pose.pos);
    let heading_err = sim::vehicle::wrap_angle(car.pose.heading - FRAC_PI_2).abs().to_degrees();
    assert!(heading_err < 8.0, "heading error {heading_err}°");
    assert!(car.slide.is_none());
}

#[test]
fn chained_segments_complete_in_order() {
    let tuning = Tuning::default();
    let mut s = one_car();
    let ends = [(pose(30.0, 0.0, 0.0), 12.0, 10.0), (pose(60.0, 20.0, 45.0), 12.0, 8.0), (pose(80.0, 50.0, 90.0), 10.0, 0.0)];
    s.cars[0].orders = orders(&s, &ends, &tuning);
    let events = run_ticks(&mut s, &tuning, 30 * 60);
    let remaining: Vec<usize> = events
        .iter()
        .filter_map(|e| match e {
            SimEvent::SegmentCompleted { remaining, .. } => Some(*remaining),
            _ => None,
        })
        .collect();
    assert_eq!(remaining, vec![2, 1, 0]);
    assert!(s.cars[0].pose.pos.distance(Vec2::new(80.0, 50.0)) < 1.0);
}

#[test]
fn too_tight_a_path_is_not_followed() {
    // A 90° turn with a ~10 m radius at 30 m/s needs ~9 g: the car must fail to follow it.
    let tuning = Tuning::default();
    let mut s = one_car();
    s.cars[0].vel = Vec2::new(30.0, 0.0);
    s.cars[0].orders = orders(&s, &[(pose(10.0, 10.0, 90.0), 30.0, 30.0)], &tuning);
    let events = run_ticks(&mut s, &tuning, 60);
    assert!(events.iter().any(|e| matches!(e, SimEvent::SlideStarted { .. })), "expected loss of grip");
}

#[test]
fn live_execution_matches_the_preview_exactly() {
    let tuning = Tuning::default();
    let world = World::default();
    let mut s = one_car();
    // Fast and tight enough to include understeer and a slide.
    let ends = [(pose(40.0, 0.0, 0.0), 20.0, 20.0), (pose(55.0, 25.0, 120.0), 20.0, 5.0)];
    let q = orders(&s, &ends, &tuning);
    Command::SetOrders { car: s.cars[0].id, orders: q }.apply(&mut s);

    let preview = Preview::run(&s, &world, &tuning, tuning.preview_ticks());
    assert_eq!(preview.check(&s), Check::Match);
    for _ in 0..preview.ticks() {
        sim::step(&mut s, &world, &tuning, StepMode::Live);
        assert_eq!(preview.check(&s), Check::Match, "diverged at tick {}", s.tick);
    }
    sim::step(&mut s, &world, &tuning, StepMode::Live);
    assert_eq!(preview.check(&s), Check::OutOfRange);
}

/// Records a session with mid-run order changes, a manual-driving stretch and a tuning
/// change, then replays it from the initial state.
fn recorded_session() -> (Replay, SimState) {
    let mut tuning = Tuning::default();
    let world = World::default();
    let mut s = one_car();
    let mut replay = Replay::new(s.clone(), tuning.clone());
    let car = s.cars[0].id;
    let issue = |s: &mut SimState, replay: &mut Replay, cmd: Command| {
        replay.record(s.tick, cmd.clone());
        cmd.apply(s);
    };

    let q = orders(&s, &[(pose(30.0, 10.0, 30.0), 15.0, 10.0)], &tuning);
    issue(&mut s, &mut replay, Command::SetOrders { car, orders: q });
    for _ in 0..120 {
        sim::step(&mut s, &world, &tuning, StepMode::Live);
    }
    let q = orders(&s, &[(pose(10.0, 40.0, 180.0), 12.0, 0.0)], &tuning);
    issue(&mut s, &mut replay, Command::SetOrders { car, orders: q });
    tuning.vehicle.grip = 1.1;
    replay.record_tuning(s.tick, tuning.clone());
    for _ in 0..200 {
        sim::step(&mut s, &world, &tuning, StepMode::Live);
    }
    issue(&mut s, &mut replay, Command::SetDriveMode { car, mode: sim::DriveMode::Direct });
    let controls = sim::Controls { throttle: 1.0, steer: -0.5, ..Default::default() };
    issue(&mut s, &mut replay, Command::SetControls { car, controls });
    for _ in 0..100 {
        sim::step(&mut s, &world, &tuning, StepMode::Live);
    }
    replay.finish(&s);
    (replay, s)
}

#[test]
fn replay_reproduces_the_session() {
    let (replay, live) = recorded_session();
    let replayed = replay.run(&World::default());
    assert_eq!(format!("{live:?}"), format!("{replayed:?}"));
    replay.verify(&World::default()).unwrap();
}

#[test]
fn replay_survives_a_ron_round_trip() {
    let (replay, _) = recorded_session();
    let text = replay.to_ron().unwrap();
    let loaded = Replay::from_ron(&text).unwrap();
    assert_eq!(format!("{replay:?}"), format!("{loaded:?}"), "floats must round-trip exactly");
    loaded.verify(&World::default()).unwrap();
}
