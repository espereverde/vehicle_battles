//! Handling status: drain, recovery, Control Table, control checks and loss of control.

use sim::handling::{self, MPH};
use sim::{Controls, DriveMode, Pose, SimEvent, SimState, StepMode, Tuning, Vec2, World};

/// Default tuning with handling class 2 (the numbers below assume it).
fn tuning() -> Tuning {
    let mut t = Tuning::default();
    t.vehicle.handling_class = 2.0;
    t
}

fn car_at(speed: f32) -> SimState {
    let mut s = SimState::new(3);
    s.spawn_car(Pose::default());
    let car = &mut s.cars[0];
    car.vel = Vec2::new(speed, 0.0);
    car.drive_mode = DriveMode::Direct;
    s
}

fn run(s: &mut SimState, mode: StepMode, controls: Controls, ticks: u32, tuning: &Tuning) -> Vec<SimEvent> {
    s.cars[0].controls = controls;
    (0..ticks).flat_map(|_| sim::step(s, &World::default(), tuning, mode).events).collect()
}

/// Steering for a constant turn of `deg_per_s` at `v` m/s.
fn turning(deg_per_s: f32, v: f32, tuning: &Tuning) -> Controls {
    let vt = &tuning.vehicle;
    let delta = (deg_per_s.to_radians() * vt.wheelbase / v).atan();
    Controls { steer: delta / vt.max_steer_deg.to_radians(), throttle: sim::hold_throttle(v, vt), ..Default::default() }
}

fn status(s: &SimState, tuning: &Tuning) -> f32 {
    handling::status(&s.cars[0], tuning)
}

/// Spot checks against the book's Control Table (chance that one roll fails).
#[test]
fn control_table_formula_follows_the_book() {
    let t = tuning();
    let at = |hs: f32, mph: f32| handling::p_fail(hs, mph / MPH, &t);
    for (hs, mph, book) in [(0.0, 120.0, 1.0 / 6.0), (0.0, 170.0, 3.0 / 6.0), (3.0, 230.0, 2.0 / 6.0), (-3.0, 50.0, 1.0 / 6.0)] {
        assert!((at(hs, mph) - book).abs() < 0.1, "HS {hs} at {mph} mph: {} vs book {book}", at(hs, mph));
    }
    assert_eq!(at(3.0, 160.0), 0.0, "safe");
    assert_eq!(at(0.0, 240.0), 1.0, "XX");
    // Lower status, higher speed: never safer.
    assert!(at(-1.0, 100.0) > at(0.0, 100.0) && at(0.0, 120.0) > at(0.0, 100.0));
}

/// Draining one point in any number of ticks is one book roll.
#[test]
fn one_point_is_one_roll_at_any_tick_rate() {
    let t = tuning();
    let (hs, v) = (-2.0, 30.0);
    let p = handling::p_fail(hs, v, &t);
    assert!(p > 0.1 && p < 0.9, "test needs a real roll, p = {p}");
    for n in [1, 4, 60] {
        let keep: f32 = (0..n).map(|_| 1.0 - handling::chance(1.0 / n as f32, hs, v, &t)).product();
        assert!((1.0 - keep - p).abs() < 1e-4, "{n} slices: {} vs {p}", 1.0 - keep);
    }
    assert_eq!(handling::chance(0.0, -6.0, 40.0, &t), 0.0, "no drain, no risk");
}

/// A 45° bend drains 3 points at any speed and radius; driving straight recovers
/// `HC + skill` per second (at least 1), up to the handling class.
#[test]
fn bends_drain_by_angle_and_straights_recover() {
    let t = tuning();
    for (v, deg_per_s) in [(10.0, 45.0), (25.0, 90.0), (15.0, 15.0)] {
        let mut s = car_at(v);
        let ticks = (45.0 / deg_per_s * t.tick_rate as f32).round() as u32;
        let mut drained = 0.0;
        s.cars[0].controls = turning(deg_per_s, v, &t);
        for _ in 0..ticks {
            sim::step(&mut s, &World::default(), &t, StepMode::Ideal);
            drained += s.cars[0].drain;
        }
        let turned = s.cars[0].pose.heading.to_degrees();
        assert!((drained - turned / 15.0).abs() < 0.01 && (turned - 45.0).abs() < 2.0, "{v} m/s: {drained} points for {turned}°");
    }

    for (skill, per_s) in [(0.0, 2.0), (1.0, 3.0), (-5.0, 1.0)] {
        let mut s = car_at(20.0);
        s.cars[0].pilot.skill = skill;
        s.cars[0].handling_lost = 5.0;
        run(&mut s, StepMode::Ideal, turning(0.0, 20.0, &t), 60, &t);
        assert!((s.cars[0].handling_lost - (5.0 - per_s)).abs() < 0.02, "skill {skill}: {}", s.cars[0].handling_lost);
    }
    let mut s = car_at(20.0);
    s.cars[0].handling_lost = 1.0;
    run(&mut s, StepMode::Ideal, turning(0.0, 20.0, &t), 120, &t);
    assert_eq!(status(&s, &t), handling::class(&t), "recovers up to the class, no further");
}

#[test]
fn hard_braking_drains_by_the_book() {
    let t = tuning();
    for (mph_s, points) in [(5.0, 0.0), (10.0, 0.0), (15.0, 1.0), (20.0, 2.0), (25.0, 3.0), (30.0, 5.0), (35.0, 7.0)] {
        let d = handling::braking_drain(mph_s / MPH, &t);
        assert!((d - points).abs() < 1e-3, "{mph_s} mph/s: {d}");
    }
    // Full brakes (≈ 23 mph/s) for half a second, then coasting: about 1.3 points.
    let mut s = car_at(30.0);
    run(&mut s, StepMode::Ideal, Controls { brake: 1.0, ..Default::default() }, 30, &t);
    let lost = s.cars[0].handling_lost;
    assert!(lost > 0.0 && lost < 1.5, "lost {lost}");
}

/// Status never drops below −6, but maneuvering there keeps rolling.
#[test]
fn checks_go_on_at_the_floor() {
    let t = tuning();
    let mut s = car_at(30.0);
    s.cars[0].handling_lost = 7.9;
    let events = run(&mut s, StepMode::Ideal, turning(120.0, 30.0, &t), 60, &t);
    // At the floor after the drain; the tick's recovery lifts it a little.
    assert!(status(&s, &t) < t.handling.min_status + 0.05, "status {}", status(&s, &t));
    let risks = events.iter().filter(|e| matches!(e, SimEvent::Risk { .. })).count();
    assert!(risks >= 59, "{risks} checks in 60 ticks at the floor");
}

/// At speed but low status, driving straight is safe; turning is not.
#[test]
fn only_maneuvers_carry_risk() {
    let t = tuning();
    let mut s = car_at(40.0);
    s.cars[0].handling_lost = 5.0;
    let events = run(&mut s, StepMode::Live, turning(0.0, 40.0, &t), 30, &t);
    assert!(events.is_empty(), "{events:?}");
    let events = run(&mut s, StepMode::Ideal, turning(60.0, 40.0, &t), 30, &t);
    assert!(events.iter().any(|e| matches!(e, SimEvent::Risk { chance, .. } if *chance > 0.0)), "{events:?}");
}

/// The ideal preview reports chances but never loses control.
#[test]
fn ideal_reports_risk_without_losing_control() {
    let t = tuning();
    let mut s = car_at(40.0);
    // 8 points/s against 2 recovered: the floor after ~1.3 s, certain loss below −4.3.
    let events = run(&mut s, StepMode::Ideal, turning(120.0, 40.0, &t), 120, &t);
    let chances: Vec<f32> = events.iter().filter_map(|e| if let SimEvent::Risk { chance, .. } = e { Some(*chance) } else { None }).collect();
    assert!(chances.len() > 10, "{events:?}");
    assert!(chances.windows(2).all(|w| w[1] >= w[0]), "risk should grow as status drops: {chances:?}");
    assert!(chances.contains(&1.0), "should reach a certain loss: {chances:?}");
    assert!(!events.iter().any(|e| matches!(e, SimEvent::LossOfControl { .. })));
}

/// A certain loss starts a crash slide; the car makes no checks until it ends.
#[test]
fn losing_control_starts_a_crash_and_pauses_checks() {
    let t = tuning();
    for v in [10.0, 30.0] {
        let mut s = car_at(v);
        s.cars[0].handling_lost = 8.0;
        // At −6, even 10 m/s is risky; a hard turn there loses control quickly.
        s.cars[0].controls = turning(180.0, v, &t);
        let mut lost_at = None;
        for _ in 0..240 {
            let events = sim::step(&mut s, &World::default(), &t, StepMode::Live).events;
            if lost_at.is_none() && events.iter().any(|e| matches!(e, SimEvent::LossOfControl { .. })) {
                lost_at = Some(s.tick);
                assert!(s.cars[0].slide.is_some(), "{v} m/s: no slide after losing control");
            } else if lost_at.is_some() && s.cars[0].slide.is_some() {
                assert!(!events.iter().any(|e| matches!(e, SimEvent::Risk { .. })), "{v} m/s: checked while sliding");
            }
        }
        assert!(lost_at.is_some(), "{v} m/s: never lost control");
    }
}

/// The dice come from the state's RNG: the same state rolls the same outcome, a different
/// seed may not.
#[test]
fn dice_are_part_of_the_state() {
    let t = tuning();
    let outcome = |seed: u64| {
        let mut s = car_at(35.0);
        s.rng = sim::SimRng::new(seed);
        s.cars[0].handling_lost = 3.0;
        let events = run(&mut s, StepMode::Live, turning(40.0, 35.0, &t), 120, &t);
        events.iter().find_map(|e| if let SimEvent::LossOfControl { tick, .. } = e { Some(*tick) } else { None })
    };
    for seed in 0..5 {
        assert_eq!(outcome(seed), outcome(seed));
    }
    let ticks: Vec<_> = (0..20).map(outcome).collect();
    assert!(ticks.iter().any(|t| *t != ticks[0]), "every seed lost control at the same tick: {ticks:?}");
}

/// Discrete events drop status by their difficulty (D0 included) and check once.
#[test]
fn hazards_drop_status_and_check_once() {
    let t = tuning();
    let mut s = car_at(40.0);
    let car = &mut s.cars[0];
    let check = handling::hazard(car, 3.0, None, &t, None);
    assert_eq!(handling::status(car, &t), handling::class(&t) - 3.0);
    assert_eq!(check.chance, handling::p_fail(-1.0, 40.0, &t));
    assert!(check.chance > 0.0);
    let d0 = handling::hazard(car, 0.0, None, &t, None);
    assert_eq!(handling::status(car, &t), handling::class(&t) - 3.0, "D0 costs nothing");
    assert_eq!(d0.chance, check.chance, "but still checks");
    let forced = handling::hazard(car, 0.0, Some(-6.0), &t, None);
    assert_eq!(forced.chance, handling::p_fail(-6.0, 40.0, &t));
}

/// Preview risk: stretches merge consecutive checks; per-second chances combine the
/// checks in each second.
#[test]
fn preview_summarizes_risk() {
    let t = tuning();
    let mut s = car_at(40.0);
    s.cars[0].drive_mode = DriveMode::Orders;
    // Status −1 and slow recovery (1/s), so both hard turns are risky at 40 m/s.
    s.cars[0].handling_lost = 3.0;
    s.cars[0].pilot.skill = -5.0;
    let hard = sim::Drive { turn: sim::Turn::Steer(0.12), throttle: 0.3 };
    let calm = sim::Drive { turn: sim::Turn::Steer(0.0), throttle: 0.3 };
    // Two risky stretches separated by a calm second.
    s.cars[0].orders = sim::OrderQueue { segments: vec![sim::Segment::new(hard, 30), sim::Segment::new(calm, 90), sim::Segment::new(hard, 30)], tail: calm };
    let preview = sim::Preview::run_in(StepMode::Ideal, &s, &World::default(), &t, 180);
    let id = s.cars[0].id;
    let risks = preview.risks(id, &t);
    assert_eq!(risks.len(), 2, "{risks:?}");
    let first: Vec<f32> = preview.events.iter().filter_map(|e| if let SimEvent::Risk { chance, tick, .. } = e { (*tick < 60).then_some(*chance) } else { None }).collect();
    let combined = 1.0 - first.iter().map(|p| 1.0 - p).product::<f32>();
    assert!(combined > 0.0);
    assert!((risks[0].chance - combined).abs() < 1e-5, "{} vs {combined}", risks[0].chance);
    // A keypoint's span: the same chance, and the handling it drained.
    let (drained, chance) = preview.span_risk(id, 0, 30);
    assert!((chance - combined).abs() < 1e-5, "{chance} vs {combined}");
    let expected: f32 = (1..=30).map(|i| preview.frame(id, i).unwrap().drain).sum();
    assert!(drained > 1.0 && (drained - expected).abs() < 1e-5);
    let per_second = preview.risk_per_second(id, &t);
    assert_eq!(per_second.len(), 3);
    assert!((per_second[0] - combined).abs() < 1e-5 && per_second[1] == 0.0 && per_second[2] > 0.0, "{per_second:?}");
}
