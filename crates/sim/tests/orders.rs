//! Keypoint orders (constant controls per segment), the drag-tool fit, preview-vs-reality
//! and replays.

use sim::preview::Check;
use sim::{
    Command, Drive, OrderQueue, Pose, Preview, Replay, Segment, SimEvent, SimState, StepMode, Tuning, Turn, TurnMode,
    Vec2, World, fit_segment, hold_throttle,
};

fn run_ticks(state: &mut SimState, tuning: &Tuning, ticks: u32) -> Vec<SimEvent> {
    run_ticks_in(StepMode::Live, state, tuning, ticks)
}

fn run_ticks_in(mode: StepMode, state: &mut SimState, tuning: &Tuning, ticks: u32) -> Vec<SimEvent> {
    let world = World::default();
    (0..ticks).flat_map(|_| sim::step(state, &world, tuning, mode).events).collect()
}

fn car_at(speed: f32) -> SimState {
    let mut s = SimState::new(7);
    s.spawn_car(Pose::default());
    s.cars[0].vel = Vec2::new(speed, 0.0);
    s
}

fn queue(segments: &[(Drive, u32)]) -> OrderQueue {
    OrderQueue { segments: segments.iter().map(|&(d, t)| Segment::new(d, t)).collect(), ..Default::default() }
}

fn drive(turn: Turn, throttle: f32) -> Drive {
    Drive { turn, throttle }
}

#[test]
fn segments_run_for_their_ticks_then_keep_the_last_controls() {
    let tuning = Tuning::default();
    let mut s = car_at(10.0);
    let last = drive(Turn::Steer(0.2), 0.3);
    s.cars[0].orders = queue(&[(drive(Turn::Steer(0.0), 0.5), 30), (last, 20)]);

    let events = run_ticks(&mut s, &tuning, 30);
    assert!(matches!(events.as_slice(), [SimEvent::SegmentCompleted { tick: 29, remaining: 1, .. }]), "{events:?}");
    let events = run_ticks(&mut s, &tuning, 20);
    assert!(matches!(events.as_slice(), [SimEvent::SegmentCompleted { remaining: 0, .. }]), "{events:?}");
    assert_eq!(s.cars[0].orders.tail, last);

    run_ticks(&mut s, &tuning, 60);
    let c = &s.cars[0];
    assert_eq!(c.controls.throttle, 0.3, "throttle kept after the last keypoint");
    assert_eq!(c.controls.steer, 0.2, "steering kept after the last keypoint");
}

#[test]
fn brake_throttle_slows_without_reversing() {
    let tuning = Tuning::default();
    let mut s = car_at(15.0);
    s.cars[0].orders = queue(&[(drive(Turn::Steer(0.0), -1.0), 300)]);
    run_ticks(&mut s, &tuning, 300);
    assert_eq!(s.cars[0].vel, Vec2::ZERO);
}

#[test]
fn lateral_mode_holds_the_requested_g() {
    let tuning = Tuning::default();
    for v in [8.0, 15.0, 25.0] {
        let mut s = car_at(v);
        let throttle = hold_throttle(v, &tuning.vehicle);
        s.cars[0].orders = queue(&[(drive(Turn::Lateral(0.5), throttle), 600)]);
        run_ticks(&mut s, &tuning, 120);
        let c = &s.cars[0];
        let g = c.forward_speed() * c.yaw_rate / tuning.vehicle.gravity;
        assert!((g - 0.5).abs() < 0.03, "at {v} m/s: {g} g");
        assert!(c.slide.is_none());
    }
}

#[test]
fn hold_throttle_keeps_speed() {
    let tuning = Tuning::default();
    for v in [5.0, 15.0, 30.0] {
        let mut s = car_at(v);
        s.cars[0].orders = queue(&[(Drive::straight_at(TurnMode::Steer, v, &tuning.vehicle), 600)]);
        run_ticks(&mut s, &tuning, 300);
        let speed = s.cars[0].vel.length();
        assert!((speed - v).abs() < 0.05, "{v} m/s drifted to {speed}");
    }
}

#[test]
fn turn_conversion_steers_the_same() {
    let vt = Tuning::default().vehicle;
    let t = Turn::Steer(0.1);
    let g = t.convert(TurnMode::Lateral, 12.0, &vt);
    assert!(matches!(g, Turn::Lateral(_)));
    assert!((g.steer(12.0, &vt) - 0.1).abs() < 1e-5);
    let back = g.convert(TurnMode::Steer, 12.0, &vt);
    assert!((back.value() - 0.1).abs() < 1e-5);
}

#[test]
fn queue_end_ticks_count_from_now() {
    let mut q = queue(&[(Drive::COAST, 30), (Drive::COAST, 20), (Drive::COAST, 10)]);
    q.segments[0].elapsed = 12;
    assert_eq!(q.end_ticks(), vec![18, 38, 48]);
    assert_eq!(q.start_tick(2), 38);
}

/// The fit must recover a point that a known turn + duration actually reaches.
#[test]
fn fit_finds_a_reachable_point() {
    let tuning = Tuning::default();
    let world = World::default();
    let start = car_at(12.0);
    for (mode, value, ticks) in [(TurnMode::Steer, 0.15, 150), (TurnMode::Steer, -0.4, 90), (TurnMode::Lateral, 0.6, 200)] {
        let d = drive(Turn::new(mode, value), 0.3);
        let mut s = start.clone();
        s.cars[0].orders = OrderQueue { segments: Vec::new(), tail: d };
        run_ticks_in(StepMode::Ideal, &mut s, &tuning, ticks);
        let target = s.cars[0].pose.pos;

        let fit = fit_segment(&start, &world, &tuning, s.cars[0].id, mode, 0.3, target, 300, 0.0);
        assert!(fit.miss < 0.3, "{mode:?} {value}: missed by {} m", fit.miss);
        assert!((fit.ticks as i64 - ticks as i64).abs() <= 3, "{mode:?} {value}: {} ticks, expected {ticks}", fit.ticks);
        assert!((fit.turn.value() - value).abs() < 0.02, "{mode:?}: fitted {} expected {value}", fit.turn.value());
    }
}

#[test]
fn fit_stops_at_the_closest_point_for_unreachable_targets() {
    let tuning = Tuning::default();
    let start = car_at(10.0);
    // Far beyond what 1 s can cover: the fit uses the whole window, straight at the target.
    let fit = fit_segment(&start, &World::default(), &tuning, start.cars[0].id, TurnMode::Steer, 0.0, Vec2::new(200.0, 0.0), 60, 0.0);
    assert_eq!(fit.ticks, 60);
    assert!(fit.turn.value().abs() < 0.01);
}

/// Within the grip limit the ideal model is the real one, bit for bit: the drag preview
/// only differs from the real one where the plan asks for more grip than the car has.
#[test]
fn ideal_matches_real_within_grip() {
    let tuning = Tuning::default();
    for (v, d) in [(15.0, drive(Turn::Steer(0.05), 0.3)), (8.0, drive(Turn::Lateral(-0.8), -0.3)), (20.0, drive(Turn::Steer(0.0), -1.0))] {
        let mut real = car_at(v);
        real.cars[0].orders = OrderQueue { segments: Vec::new(), tail: d };
        let mut ideal = real.clone();
        run_ticks(&mut real, &tuning, 180);
        run_ticks_in(StepMode::Ideal, &mut ideal, &tuning, 180);
        assert!(real.cars[0].grip_demand < 1.0, "{d:?} exceeds grip");
        assert_eq!(real.cars[0].pose, ideal.cars[0].pose, "{d:?}");
        assert_eq!(real.cars[0].vel, ideal.cars[0].vel, "{d:?}");
    }
}

/// The drag tool never plans a turn tighter than `max_lateral_g`, in either turn mode
/// (steering mode: measured at the segment's start speed).
#[test]
fn fit_caps_turns_at_max_lateral_g() {
    let tuning = Tuning::default();
    let vt = &tuning.vehicle;
    let max_g = tuning.planner.max_lateral_g;
    let start = car_at(20.0);
    // Just ahead and to the side: reaching it takes about 6 g, so the tighter the better.
    // One second only, so the car cannot loop around and come back closer.
    for target in [Vec2::new(5.0, 10.0), Vec2::new(5.0, -10.0)] {
        for mode in [TurnMode::Steer, TurnMode::Lateral] {
            let fit = fit_segment(&start, &World::default(), &tuning, start.cars[0].id, mode, 0.0, target, 60, 0.0);
            let g = fit.turn.convert(TurnMode::Lateral, 20.0, vt).value();
            assert!(g.abs() <= max_g * 1.001, "{mode:?} {target}: planned {g} g");
            assert!(g.abs() > max_g * 0.95, "{mode:?} {target}: expected the full {max_g} g, got {g}");
        }
    }
}

/// Ignoring grip does not mean turning on the spot: the ideal car still turns no tighter
/// than its steering lock allows (lateral-g orders included).
#[test]
fn ideal_turns_are_limited_by_the_steering_lock() {
    let tuning = Tuning::default();
    let vt = &tuning.vehicle;
    for turn in [Turn::Steer(1.0), Turn::Steer(-1.0), Turn::Lateral(5.0), Turn::Lateral(-5.0)] {
        let mut s = car_at(20.0);
        s.cars[0].orders = OrderQueue { segments: Vec::new(), tail: drive(turn, 0.0) };
        for _ in 0..120 {
            // The yaw rate comes from the speed at the start of the tick.
            let max_yaw = s.cars[0].forward_speed() * vt.max_steer_deg.to_radians().tan() / vt.wheelbase;
            run_ticks_in(StepMode::Ideal, &mut s, &tuning, 1);
            let c = &s.cars[0];
            assert!(c.yaw_rate.abs() <= max_yaw * 1.0001, "{turn:?}: yaw rate {} > lock {max_yaw}", c.yaw_rate);
        }
        assert!(s.cars[0].grip_demand > 1.0, "{turn:?}: expected a turn past the grip limit");
    }
}

/// The fit ignores grip: a turn far past the limit is fitted exactly on its ideal path
/// (regression for the old model: every slide landed near a gentle turn, steering jumped
/// from 9% to 100%). The real car then slides, which the real preview shows.
#[test]
fn fit_assumes_unlimited_grip() {
    let tuning = Tuning::default();
    let world = World::default();
    let start = car_at(19.0);
    // About 2.5 g: well past the grip limit (1.5 g), within the planning cap (3 g).
    let hard = drive(Turn::Steer(0.3), -0.2);
    let mut s = start.clone();
    s.cars[0].orders = OrderQueue { segments: Vec::new(), tail: hard };
    run_ticks_in(StepMode::Ideal, &mut s, &tuning, 48);
    assert!(s.cars[0].grip_demand > 1.2, "not past the limit: {}", s.cars[0].grip_demand);
    let target = s.cars[0].pose.pos;
    let fit = fit_segment(&start, &world, &tuning, start.cars[0].id, TurnMode::Steer, -0.2, target, 120, 0.0);
    assert!(fit.miss < 0.1, "missed by {} m with {:?}", fit.miss, fit.turn);
    assert!((fit.turn.value() - 0.3).abs() < 0.01, "fitted {:?}", fit.turn);

    let mut real = start.clone();
    real.cars[0].orders = OrderQueue { segments: Vec::new(), tail: hard };
    let events = run_ticks(&mut real, &tuning, 48);
    assert!(events.iter().any(|e| matches!(e, SimEvent::SlideStarted { .. })));

    // Sweeping the target sideways must increase steering monotonically, never jump.
    let mut last = 0.0;
    for i in 0..=20 {
        let p = Vec2::new(target.x, i as f32 * 0.25);
        let v = fit_segment(&start, &world, &tuning, start.cars[0].id, TurnMode::Steer, -0.2, p, 120, 0.0).turn.value();
        assert!(v >= last - 1e-3 && v - last < 0.05, "offset {} m: steer {last} -> {v}", i as f32 * 0.25);
        last = v;
    }
}

#[test]
fn live_execution_matches_the_preview_exactly() {
    let tuning = Tuning::default();
    let world = World::default();
    let mut s = car_at(5.0);
    // Hard enough to include understeer and a slide.
    let q = queue(&[(drive(Turn::Steer(0.0), 1.0), 120), (drive(Turn::Lateral(1.8), 0.6), 90), (drive(Turn::Steer(-0.3), -0.5), 60)]);
    Command::SetOrders { car: s.cars[0].id, orders: q }.apply(&mut s);

    let preview = Preview::run(&s, &world, &tuning, tuning.preview_ticks());
    assert!(preview.events.iter().any(|e| matches!(e, SimEvent::SlideStarted { .. })), "test should slide");
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
    let mut s = car_at(0.0);
    let mut replay = Replay::new(s.clone(), tuning.clone());
    let car = s.cars[0].id;
    let issue = |s: &mut SimState, replay: &mut Replay, cmd: Command| {
        replay.record(s.tick, cmd.clone());
        cmd.apply(s);
    };

    let q = queue(&[(drive(Turn::Steer(0.1), 0.6), 100), (drive(Turn::Lateral(-0.5), 0.2), 80)]);
    issue(&mut s, &mut replay, Command::SetOrders { car, orders: q });
    for _ in 0..120 {
        sim::step(&mut s, &world, &tuning, StepMode::Live);
    }
    let q = queue(&[(drive(Turn::Steer(-0.2), -0.4), 60)]);
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

/// Dragging a keypoint across neighboring cursor positions must move it smoothly: no jumps
/// between far-apart solutions (regression: preview trajectory jumped while dragging).
#[test]
fn drag_fit_is_continuous() {
    let tuning = Tuning::default();
    let world = World::default();
    for (v, throttle, pre_steer) in [(5.0, 0.4, 0.0), (19.0, -0.2, 0.0), (15.0, 0.2, 0.25)] {
        for mode in [TurnMode::Steer, TurnMode::Lateral] {
            // Start straight or mid-turn, as a later keypoint would.
            let mut start = car_at(v);
            start.cars[0].orders = OrderQueue { segments: Vec::new(), tail: drive(Turn::Steer(pre_steer), throttle) };
            run_ticks(&mut start, &tuning, 30);
            let base = start.cars[0].pose;
            let keypoint = |cursor: Vec2| {
                let fit = fit_segment(&start, &world, &tuning, start.cars[0].id, mode, throttle, cursor, 300, 0.0);
                let mut s = start.clone();
                s.cars[0].orders = OrderQueue { segments: Vec::new(), tail: drive(fit.turn, throttle) };
                run_ticks_in(StepMode::Ideal, &mut s, &tuning, fit.ticks);
                s.cars[0].pose.pos
            };
            let reach = (v * 2.5).max(15.0);
            for i in 0..4 {
                let along = reach * (0.4 + 0.2 * i as f32);
                let mut last: Option<Vec2> = None;
                for j in -12..=12 {
                    let cursor = base.pos + base.forward() * along + base.left() * (j as f32 * 0.5);
                    let k = keypoint(cursor);
                    if let Some(prev) = last {
                        let jump = prev.distance(k);
                        assert!(jump < 1.5, "{v} m/s {mode:?}: keypoint jumped {jump} m near {cursor}");
                    }
                    last = Some(k);
                }
            }
        }
    }
}

/// Braking hard towards a point beyond the stopping distance: the keypoint lands where
/// the car stops, not at the end of the search window.
#[test]
fn fit_stops_where_the_car_stops_when_braking() {
    let tuning = Tuning::default();
    let world = World::default();
    let start = car_at(20.0);
    for mode in [TurnMode::Steer, TurnMode::Lateral] {
        for target in [Vec2::new(25.0, 0.0), Vec2::new(30.0, 4.0), Vec2::new(28.0, -6.0)] {
            let fit = fit_segment(&start, &world, &tuning, start.cars[0].id, mode, -1.0, target, 300, 0.0);
            let mut s = start.clone();
            s.cars[0].orders = OrderQueue { segments: Vec::new(), tail: drive(fit.turn, -1.0) };
            run_ticks_in(StepMode::Ideal, &mut s, &tuning, fit.ticks);
            let at_keypoint = s.cars[0].pose.pos;
            run_ticks_in(StepMode::Ideal, &mut s, &tuning, 300);
            let rest = s.cars[0].pose.pos;
            assert!(at_keypoint.distance(rest) < 0.01, "{mode:?} {target}: keypoint {at_keypoint} before the stop at {rest}");
            assert!(fit.ticks < 150, "{mode:?} {target}: keypoint at tick {}, long after stopping", fit.ticks);
        }
    }
}

/// A tight constant turn closes a loop well within the horizon; the preview finds where
/// and can be cut there.
#[test]
fn preview_finds_the_first_loop() {
    let tuning = Tuning::default();
    let mut s = car_at(10.0);
    s.cars[0].orders = OrderQueue { segments: Vec::new(), tail: drive(Turn::Lateral(-3.0), hold_throttle(10.0, &tuning.vehicle)) };
    let mut preview = Preview::run_in(StepMode::Ideal, &s, &World::default(), &tuning, tuning.preview_ticks());
    // 3 g needs more than full lock at 10 m/s: r ≈ 3.9 m, one lap ≈ 2.4 s plus the ramp.
    let i = preview.first_loop(s.cars[0].id).expect("loops");
    assert!((140..=170).contains(&i), "loop closes at tick {i}");
    let start = preview.frame(s.cars[0].id, 0).unwrap().pose.pos;
    let closed = preview.frame(s.cars[0].id, i).unwrap().pose.pos;
    // Not exact: the lap starts with the steering ramping up.
    assert!(start.distance(closed) < 2.5, "loop end {closed} far from start {start}");
    preview.cut(i);
    assert_eq!(preview.ticks(), i);

    // A gentle turn never closes.
    s.cars[0].orders.tail = drive(Turn::Lateral(0.3), 0.0);
    let preview = Preview::run_in(StepMode::Ideal, &s, &World::default(), &tuning, tuning.preview_ticks());
    assert_eq!(preview.first_loop(s.cars[0].id), None);
}

/// With room for several laps, the fit must not reach the target on a later lap
/// (regression: a 2.2 g second pass beat the 3 g first one).
#[test]
fn fit_stays_on_the_first_loop() {
    let tuning = Tuning::default();
    let world = World::default();
    let start = car_at(20.0);
    for turned in [0.0, 3.0] {
        let target = Vec2::new(-5.0, 10.0);
        let fit = fit_segment(&start, &world, &tuning, start.cars[0].id, TurnMode::Lateral, 0.0, target, 300, turned);
        let mut s = start.clone();
        s.cars[0].orders = OrderQueue { segments: Vec::new(), tail: drive(fit.turn, 0.0) };
        let mut net = turned;
        let mut heading = s.cars[0].pose.heading;
        for _ in 0..fit.ticks {
            run_ticks_in(StepMode::Ideal, &mut s, &tuning, 1);
            net += sim::vehicle::wrap_angle(s.cars[0].pose.heading - heading);
            heading = s.cars[0].pose.heading;
        }
        assert!(net.abs() < std::f32::consts::TAU, "turned {turned}: keypoint after a full circle ({net} rad, {:?})", fit);
    }
}

/// The danger-zone fan covers the fit's search space: one arc per turn value, from hard
/// right to hard left, no longer than the window, and fitted keypoints land inside it.
#[test]
fn sweep_spans_the_fit_search_space() {
    let tuning = Tuning::default();
    let world = World::default();
    let start = car_at(15.0);
    let id = start.cars[0].id;
    for mode in [TurnMode::Steer, TurnMode::Lateral] {
        let fan = sim::sweep(&start, &world, &tuning, id, mode, 0.2, 180, 0.0, 41);
        assert_eq!(fan.len(), 41);
        assert!(fan.iter().all(|arc| !arc.is_empty() && arc.len() <= 180));
        // The hardest turns nearly close a loop, so look at how far they reach sideways.
        let reach = |arc: &Vec<sim::CarFrame>| arc.iter().map(|f| f.pose.pos.y).fold(0.0f32, |m, y| if y.abs() > m.abs() { y } else { m });
        assert!(reach(&fan[0]) < -5.0, "{mode:?}: first arc should turn right");
        assert!(reach(&fan[40]) > 5.0, "{mode:?}: last arc should turn left");
        let straight = fan[20].last().unwrap().pose.pos;
        assert!(straight.y.abs() < 1e-3, "{mode:?}: middle arc should be straight, ends at {straight}");

        let target = Vec2::new(25.0, 6.0);
        let fit = fit_segment(&start, &world, &tuning, id, mode, 0.2, target, 180, 0.0);
        let nearest = fan.iter().flatten().map(|f| f.pose.pos.distance(target)).fold(f32::INFINITY, f32::min);
        assert!(fit.miss < 0.1 && nearest < 1.0, "{mode:?}: fit miss {} m, fan nearest {nearest} m", fit.miss);
    }
}
