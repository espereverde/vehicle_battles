//! Sanity checks for the kinematic car: it drives, brakes and turns exactly as commanded.

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
}

/// The handbrake only adds braking: shorter stop, no rotation.
#[test]
fn handbrake_brakes_harder_without_turning() {
    let mut brake = car_at_speed(20.0);
    drive(&mut brake, Controls { brake: 1.0, ..Default::default() }, 4.0);
    let mut both = car_at_speed(20.0);
    drive(&mut both, Controls { brake: 1.0, handbrake: true, ..Default::default() }, 4.0);
    assert!(both.cars[0].pose.pos.x < brake.cars[0].pose.pos.x - 2.0);
    assert_eq!(both.cars[0].pose.heading, 0.0);
}

/// The wheels take the commanded angle at once, and the car turns on the kinematic circle
/// of radius `L / tan δ` at any speed: no understeer, never moving sideways.
#[test]
fn turns_on_the_kinematic_circle_at_any_speed() {
    let vt = Tuning::default().vehicle;
    for (v, steer) in [(5.0, 1.0), (15.0, 0.3), (30.0, 0.15)] {
        let mut s = car_at_speed(v);
        drive(&mut s, full(0.3, steer), 1.0 / 60.0);
        let delta = steer * vt.max_steer_deg.to_radians();
        assert_eq!(s.cars[0].steer_angle, delta, "wheels lag the command");

        let radius = vt.wheelbase / delta.tan();
        let center = Vec2::new(0.0, radius);
        for _ in 0..20 {
            drive(&mut s, full(0.3, steer), 0.1);
            let car = &s.cars[0];
            let off = car.pose.pos.distance(center) - radius;
            assert!(off.abs() < 0.02 * radius, "{v} m/s, steer {steer}: {off} m off the circle");
            let sideways = car.vel.dot(car.pose.left());
            assert!(sideways.abs() < 1e-3, "{v} m/s: moving sideways at {sideways} m/s");
        }
    }
}

/// Full lock at speed: the car still follows the lock, but drains its stability and loses
/// control (decided by the gauge, not the tires).
#[test]
fn full_lock_at_speed_loses_control() {
    let mut s = car_at_speed(30.0);
    let events = drive(&mut s, full(0.0, 1.0), 1.0);
    assert!(events.iter().any(|e| matches!(e, SimEvent::LossOfControl { .. })), "{events:?}");
}

#[test]
fn gentle_turn_is_uneventful() {
    let mut s = car_at_speed(10.0);
    let events = drive(&mut s, full(0.3, 0.1), 5.0);
    assert!(events.is_empty(), "{events:?}");
}
