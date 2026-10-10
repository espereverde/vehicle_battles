//! Sanity checks that the car drives, brakes and slides believably.

use sim::{Controls, DriveMode, Pose, SimEvent, SimState, StepMode, Tuning, Vec2, World};

fn car_at_speed(speed: f32) -> SimState {
    let mut s = SimState::new(0);
    s.spawn_car(Pose::new(Vec2::ZERO, 0.0));
    s.cars[0].vel = Vec2::new(speed, 0.0);
    s.cars[0].drive_mode = DriveMode::Direct;
    s
}

fn drive(s: &mut SimState, controls: Controls, seconds: f32) -> Vec<SimEvent> {
    let (world, tuning) = (World::default(), Tuning::default());
    let ticks = (seconds * tuning.tick_rate as f32).round() as u32;
    let mut events = Vec::new();
    for _ in 0..ticks {
        s.cars[0].controls = controls;
        events.extend(sim::step(s, &world, &tuning, StepMode::Live).events);
    }
    events
}

fn full(throttle: f32, steer: f32) -> Controls {
    Controls { throttle, steer, ..Default::default() }
}

#[test]
fn accelerates_towards_top_speed() {
    let mut s = car_at_speed(0.0);
    drive(&mut s, full(1.0, 0.0), 4.0);
    let v4 = s.cars[0].forward_speed();
    assert!((15.0..24.0).contains(&v4), "0-4 s speed {v4}");
    drive(&mut s, full(1.0, 0.0), 30.0);
    let top = Tuning::default().vehicle.top_speed;
    let v = s.cars[0].forward_speed();
    // Resistance balances the fading engine a little below the nominal top speed.
    assert!(v > 0.85 * top && v < top, "top speed {v}");
    assert!(s.cars[0].pose.pos.y.abs() < 1e-3, "drifted sideways going straight");
}

#[test]
fn brakes_to_a_stop_without_reversing() {
    let mut s = car_at_speed(30.0);
    drive(&mut s, Controls { brake: 1.0, ..Default::default() }, 6.0);
    let car = &s.cars[0];
    assert_eq!(car.vel, Vec2::ZERO);
    // v²/(2a) with a ≈ 9.3 m/s² is ~48 m.
    assert!((40.0..60.0).contains(&car.pose.pos.x), "stopping distance {}", car.pose.pos.x);
    assert!(car.slide.is_none());
}

#[test]
fn gentle_turn_keeps_grip() {
    let mut s = car_at_speed(10.0);
    let events = drive(&mut s, full(0.3, 0.1), 5.0);
    assert!(events.is_empty(), "unexpected slide: {events:?}");
    assert!(s.cars[0].slip_angle().abs() < 0.01);
}

#[test]
fn full_lock_at_speed_slides_then_recovers() {
    let mut s = car_at_speed(30.0);
    let events = drive(&mut s, full(0.0, 1.0), 1.0);
    assert!(matches!(events.first(), Some(SimEvent::SlideStarted { .. })), "{events:?}");
    assert!(s.cars[0].slip_angle().abs() > 0.1, "slide without slip angle");

    let events = drive(&mut s, full(0.0, 0.0), 4.0);
    assert!(events.iter().any(|e| matches!(e, SimEvent::SlideEnded { .. })), "never recovered");
    assert!(s.cars[0].slide.is_none());
}

#[test]
fn handbrake_turn_rotates_the_car() {
    let mut s = car_at_speed(15.0);
    drive(&mut s, Controls { steer: 1.0, handbrake: true, ..Default::default() }, 1.0);
    let car = &s.cars[0];
    assert!(car.pose.heading > 1.0, "heading {}", car.pose.heading);
}

#[test]
fn full_lock_at_low_speed_understeers_without_sliding() {
    // ~32 km/h, full lock and some throttle: well past the friction circle, but too slow to spin.
    let mut s = car_at_speed(9.0);
    let events = drive(&mut s, full(0.3, 1.0), 3.0);
    assert!(events.is_empty(), "slid at low speed: {events:?}");
    let car = &s.cars[0];
    assert!(car.grip_demand > 1.0, "test should be past the limit, demand {}", car.grip_demand);
    // Runs wide: the turn is limited to the grip limit, not the kinematic yaw rate.
    let vt = Tuning::default().vehicle;
    let max_lat = car.forward_speed() * car.yaw_rate;
    assert!(max_lat.abs() <= vt.grip * vt.gravity * 1.001, "lateral accel {max_lat}");
}

#[test]
fn mild_overdrive_at_speed_understeers_strong_overdrive_slides() {
    let tuning = Tuning::default();
    let v: f32 = 25.0;
    // Wheel angle that asks for a given lateral grip demand at speed v.
    let steer_for = |demand: f32| {
        let vt = &tuning.vehicle;
        let delta = (demand * vt.grip * vt.gravity * vt.wheelbase / (v * v)).atan();
        delta / tuning.vehicle.max_steer_deg.to_radians()
    };
    let mut s = car_at_speed(v);
    assert!(drive(&mut s, full(0.3, steer_for(1.1)), 1.0).is_empty(), "1.1 g should understeer");
    let mut s = car_at_speed(v);
    let events = drive(&mut s, full(0.3, steer_for(1.5)), 1.0);
    assert!(matches!(events.first(), Some(SimEvent::SlideStarted { .. })), "1.5 g should slide");
}
