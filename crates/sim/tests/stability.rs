//! Stability gauge: drain, recovery, risk checks and loss of control.

use sim::{Controls, DriveMode, LossKind, Pose, SimEvent, SimState, StepMode, Tuning, Vec2, World};

fn car_at(speed: f32) -> SimState {
    let mut s = SimState::new(3);
    s.spawn_car(Pose::default());
    let car = &mut s.cars[0];
    car.vel = Vec2::new(speed, 0.0);
    car.drive_mode = DriveMode::Direct;
    s
}

/// Steering that asks for grip demand `demand` at speed `v`, with throttle to hold it.
fn turn(demand: f32, v: f32, tuning: &Tuning) -> Controls {
    let vt = &tuning.vehicle;
    let delta = (demand * vt.grip * vt.gravity * vt.wheelbase / (v * v)).atan();
    Controls { steer: delta / vt.max_steer_deg.to_radians(), throttle: 0.3, ..Default::default() }
}

fn run(s: &mut SimState, mode: StepMode, controls: Controls, seconds: f32, tuning: &Tuning) -> Vec<SimEvent> {
    s.cars[0].controls = controls;
    let ticks = (seconds * tuning.tick_rate as f32) as u32;
    (0..ticks).flat_map(|_| sim::step(s, &World::default(), tuning, mode).events).collect()
}

#[test]
fn drains_above_comfort_and_recovers_below() {
    let tuning = Tuning::default();
    let mut s = car_at(20.0);
    run(&mut s, StepMode::Ideal, turn(0.9, 20.0, &tuning), 1.0, &tuning);
    let drained = s.cars[0].stability;
    assert!(drained < 0.95, "stability {drained} after a second at 0.9 demand");

    run(&mut s, StepMode::Ideal, Controls { throttle: 0.3, ..Default::default() }, 1.0, &tuning);
    let recovered = s.cars[0].stability;
    let expected = tuning.stability.k_rec * s.cars[0].pilot.skill;
    assert!(recovered > drained + 0.5 * expected, "{drained} -> {recovered}");

    // A better pilot recovers faster.
    let mut good = car_at(20.0);
    good.cars[0].pilot.skill = 1.0;
    good.cars[0].stability = 0.5;
    let mut poor = good.clone();
    poor.cars[0].pilot.skill = 0.2;
    let coast = Controls::default();
    run(&mut good, StepMode::Ideal, coast, 1.0, &tuning);
    run(&mut poor, StepMode::Ideal, coast, 1.0, &tuning);
    assert!(good.cars[0].stability > poor.cars[0].stability);
}

#[test]
fn no_risk_checks_while_green_or_below_comfort() {
    let tuning = Tuning::default();
    // Green zone, hard but comfortable-ish cornering.
    let mut s = car_at(20.0);
    let events = run(&mut s, StepMode::Live, turn(0.75, 20.0, &tuning), 0.5, &tuning);
    assert!(!events.iter().any(|e| matches!(e, SimEvent::Risk { .. })), "{events:?}");
    // Yellow zone, but below comfort.
    let mut s = car_at(20.0);
    s.cars[0].stability = 0.2;
    let events = run(&mut s, StepMode::Live, turn(0.5, 20.0, &tuning), 1.0, &tuning);
    assert!(events.is_empty(), "{events:?}");
}

/// The ideal preview reports chances but never loses control; a certain loss resets the
/// gauge as if the car had recovered.
#[test]
fn ideal_reports_risk_without_losing_control() {
    let tuning = Tuning::default();
    let mut s = car_at(20.0);
    s.cars[0].stability = 0.5;
    let events = run(&mut s, StepMode::Ideal, turn(1.3, 20.0, &tuning), 1.5, &tuning);
    let chances: Vec<f32> = events.iter().filter_map(|e| if let SimEvent::Risk { chance, .. } = e { Some(*chance) } else { None }).collect();
    assert!(chances.len() >= 2, "{events:?}");
    assert!(chances.windows(2).take_while(|w| w[1] < 1.0).all(|w| w[1] >= w[0]), "chance should grow as stability drops: {chances:?}");
    assert!(chances.contains(&1.0), "should reach a certain loss: {chances:?}");
    assert!(!events.iter().any(|e| matches!(e, SimEvent::LossOfControl { .. })));
    assert!(s.cars[0].slide.is_none());
}

/// At zero stability the loss is certain; slow it's a skid, fast a spin (with a big yaw kick).
#[test]
fn loss_of_control_is_skid_when_slow_spin_when_fast() {
    let tuning = Tuning::default();
    for (v, kind) in [(11.0, LossKind::Skid), (30.0, LossKind::Spin)] {
        let mut s = car_at(v);
        s.cars[0].stability = 0.01;
        let events = run(&mut s, StepMode::Live, turn(1.4, v, &tuning), 0.4, &tuning);
        let lost = events.iter().find_map(|e| if let SimEvent::LossOfControl { kind, .. } = e { Some(*kind) } else { None });
        assert_eq!(lost, Some(kind), "{v} m/s: {events:?}");
        let car = &s.cars[0];
        assert!(car.slide.is_some());
        assert!(car.yaw_rate > 0.0, "kick should go into the (left) turn");
    }
    // Below min_speed nothing happens, however low the gauge.
    let mut s = car_at(8.0);
    s.cars[0].stability = 0.0;
    let events = run(&mut s, StepMode::Live, turn(2.0, 8.0, &tuning), 1.0, &tuning);
    assert!(!events.iter().any(|e| matches!(e, SimEvent::LossOfControl { .. })), "{events:?}");
}

/// The dice come from the state's RNG: the same state rolls the same outcome, a different
/// seed may not.
#[test]
fn dice_are_part_of_the_state() {
    let tuning = Tuning::default();
    let outcome = |seed: u64| {
        let mut s = car_at(20.0);
        s.rng = sim::SimRng::new(seed);
        s.cars[0].stability = 0.45;
        let events = run(&mut s, StepMode::Live, turn(1.0, 20.0, &tuning), 2.0, &tuning);
        events.iter().find_map(|e| if let SimEvent::LossOfControl { tick, .. } = e { Some(*tick) } else { None })
    };
    for seed in 0..5 {
        assert_eq!(outcome(seed), outcome(seed));
    }
    let ticks: Vec<_> = (0..20).map(outcome).collect();
    assert!(ticks.iter().any(|t| *t != ticks[0]), "every seed lost control at the same tick: {ticks:?}");
}

/// Preview risk stretches merge consecutive checks and combine their chances.
#[test]
fn preview_merges_risk_checks_into_stretches() {
    let tuning = Tuning::default();
    let mut s = car_at(20.0);
    s.cars[0].stability = 0.55;
    s.cars[0].drive_mode = sim::DriveMode::Orders;
    let hard = sim::Drive { turn: sim::Turn::Lateral(1.25), throttle: 0.3 };
    let calm = sim::Drive { turn: sim::Turn::Lateral(0.0), throttle: 0.3 };
    // Two risky stretches separated by a calm second.
    s.cars[0].orders = sim::OrderQueue { segments: vec![sim::Segment::new(hard, 45), sim::Segment::new(calm, 60), sim::Segment::new(hard, 45)], tail: calm };
    let preview = sim::Preview::run_in(StepMode::Ideal, &s, &World::default(), &tuning, 200);
    let risks = preview.risks(s.cars[0].id, &tuning);
    assert_eq!(risks.len(), 2, "{risks:?}");
    let checks: Vec<f32> = preview.events.iter().filter_map(|e| if let SimEvent::Risk { chance, tick, .. } = e { (*tick < 60).then_some(*chance) } else { None }).collect();
    let combined = 1.0 - checks.iter().map(|p| 1.0 - p).product::<f32>();
    assert!((risks[0].chance - combined).abs() < 1e-5, "{} vs {combined}", risks[0].chance);
    assert!(risks[0].start < risks[0].end && risks[0].end < risks[1].start);
    assert!(risks.iter().all(|r| r.spin_chance > 0.0 && r.spin_chance < 1.0));
}
