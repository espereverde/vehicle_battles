//! Crash tables, the slides they start, and controlled skids.

use std::f32::consts::FRAC_PI_2;

use sim::crash::{self, Crash};
use sim::handling::{INCH, MPH};
use sim::{
    Drive, DriveMode, OrderQueue, Pose, Segment, SimEvent, SimRng, SimState, SlideKind, StepMode, Tuning, Turn, Vec2, World, fit_skid,
};

fn car_at(speed: f32) -> SimState {
    let mut s = SimState::new(5);
    s.spawn_car(Pose::default());
    s.cars[0].vel = Vec2::new(speed, 0.0);
    s.cars[0].drive_mode = DriveMode::Direct;
    s
}

fn step(s: &mut SimState, mode: StepMode, t: &Tuning) -> Vec<SimEvent> {
    sim::step(s, &World::default(), t, mode).events
}

#[test]
fn tables_follow_the_book() {
    // Control Table modifier column.
    for (mph, m) in [(10.0, -3), (20.0, -2), (30.0, -1), (40.0, 0), (50.0, 1), (60.0, 1), (70.0, 2), (90.0, 2), (100.0, 3), (120.0, 3), (130.0, 4)] {
        assert_eq!(crash::speed_modifier(mph / MPH), m, "{mph} mph");
    }
    // D4 at 20 mph by a Driver +1: 4 − 3 − 2 − 1.
    assert_eq!(crash::modifier(4.0, 20.0 / MPH, 1.0), -2);
    let t1: Vec<Crash> = (1..=16).map(crash::table1).collect();
    assert_eq!(&t1[..4], &[Crash::TrivialSkid, Crash::TrivialSkid, Crash::MinorSkid, Crash::MinorSkid]);
    assert_eq!((t1[4], t1[6], t1[8], t1[10], t1[12], t1[14]), (Crash::ModerateSkid, Crash::SevereSkid, Crash::Spinout, Crash::Roll, Crash::BurningRoll, Crash::Vault));
    assert_eq!(crash::table2(4), (Crash::MinorFishtail, false));
    assert_eq!(crash::table2(8), (Crash::MajorFishtail, false));
    assert_eq!(crash::table2(10), (Crash::MinorFishtail, true));
    assert_eq!(crash::table2(15), (Crash::DoubleFishtail, true));
    // 2d6: 9+ is 10/36 unmodified, everything at +7, nothing at −8.
    assert!((crash::chance_at_least(9, 0) - 10.0 / 36.0).abs() < 1e-6);
    assert_eq!((crash::chance_at_least(9, 7), crash::chance_at_least(9, -8)), (1.0, 0.0));
}

/// Skids slide straight on, along the tangent of the bend the car was in, while the nose
/// keeps turning at the bend's rate, losing the book's speed as they go, then ease back onto
/// the nose.
#[test]
fn skids_slide_along_the_tangent() {
    let t = Tuning::default();
    for (crash, inches, mph_lost) in [(Crash::TrivialSkid, 0.25, 0.0), (Crash::MinorSkid, 0.5, 5.0), (Crash::ModerateSkid, 1.0, 10.0), (Crash::SevereSkid, 1.5, 20.0)] {
        let mut s = in_a_bend(20.0, 0.3, &t);
        let tangent = s.cars[0].vel.to_angle();
        let (mut heading, yaw_rate) = (s.cars[0].pose.heading, s.cars[0].yaw_rate);
        crash::start(&mut s.cars[0], crash, &t, &mut SimRng::new(1));
        let start = s.cars[0].pose.pos;
        let (mut path, mut going) = (0.0, tangent);
        let mut ended = false;
        for _ in 0..120 {
            let before = s.cars[0].pose.pos;
            let events = step(&mut s, StepMode::Ideal, &t);
            let car = &s.cars[0];
            path += car.pose.pos.distance(before);
            let now = car.vel.to_angle();
            assert!((now - going).abs() < 0.1, "{crash:?}: direction jumped {going} -> {now}");
            going = now;
            if events.iter().any(|e| matches!(e, SimEvent::SlideEnded { .. })) {
                ended = true;
                break;
            }
            heading += yaw_rate * t.dt();
            assert!((car.pose.heading - heading).abs() < 1e-4, "{crash:?}: nose not turning as in the bend");
            assert!((now - tangent).abs() < 1e-4, "{crash:?}: left the tangent");
        }
        let car = &s.cars[0];
        assert!(ended, "{crash:?}: never ended");
        assert!((path - inches * INCH).abs() < 0.4, "{crash:?}: slid {path} m");
        let moved = car.pose.pos - start;
        assert!((moved.to_angle() - tangent).abs() < 1e-3, "{crash:?}: slid off the tangent");
        let speed = car.vel.length();
        assert!((speed - (20.0 - mph_lost / MPH)).abs() < 0.05, "{crash:?}: speed {speed}");
        // Then the car eases back onto its nose.
        for _ in 0..30 {
            step(&mut s, StepMode::Ideal, &t);
            let now = s.cars[0].vel.to_angle();
            assert!((now - going).abs() < 0.1, "{crash:?}: direction jumped {going} -> {now} regripping");
            going = now;
        }
        let car = &s.cars[0];
        assert!(car.slip.is_none() && car.vel.normalize().dot(car.pose.forward()) > 0.999, "drives on along the nose");
    }
}

/// Spinout: 90° per 0.2 s and −20 mph/s along the old direction. Without dice it never
/// recovers and spins to a stop; with dice it may recover first.
#[test]
fn spinouts_spin_down_or_recover() {
    let t = Tuning::default();
    let mut s = car_at(20.0);
    s.cars[0].yaw_rate = 0.3;
    crash::start(&mut s.cars[0], Crash::Spinout, &t, &mut SimRng::new(1));
    // The rotation builds up from nothing...
    step(&mut s, StepMode::Ideal, &t);
    assert!(s.cars[0].pose.heading.abs() < 1f32.to_radians(), "first tick: {}°", s.cars[0].pose.heading.to_degrees());
    for _ in 1..30 {
        step(&mut s, StepMode::Ideal, &t);
    }
    // ...to 90° per 0.2 s.
    let before = s.cars[0].pose.heading;
    for _ in 0..12 {
        step(&mut s, StepMode::Ideal, &t);
    }
    let car = &s.cars[0];
    let turned = sim::vehicle::wrap_angle(car.pose.heading - before);
    assert!((turned - FRAC_PI_2).abs() < 0.01, "in 0.2 s: {}°", turned.to_degrees());
    assert!((car.vel.length() - (20.0 - 20.0 / MPH * 0.7)).abs() < 0.05);
    assert!(car.vel.y.abs() < 1e-4, "slides along the old direction");
    let mut ticks = 42;
    while s.cars[0].slide.is_some() && ticks < 600 {
        step(&mut s, StepMode::Ideal, &t);
        ticks += 1;
    }
    let stop_time = 20.0 / (20.0 / MPH) * 60.0;
    assert!((ticks as f32 - stop_time).abs() < 3.0 && s.cars[0].vel == Vec2::ZERO, "stopped after {ticks} ticks");
    // It wound down as it stopped.
    assert!(s.cars[0].yaw_rate.abs() < 0.5 * t.crash.spin_deg_per_s.to_radians(), "still spinning at {} rad/s", s.cars[0].yaw_rate);

    // At 15 m/s a roll at −6 succeeds 38% of the time: some seeds recover before stopping.
    let recovered = (0..20)
        .filter(|&seed| {
            let mut s = car_at(15.0);
            s.rng = SimRng::new(seed);
            crash::start(&mut s.cars[0], Crash::Spinout, &t, &mut SimRng::new(seed));
            while s.cars[0].slide.is_some_and(|sl| sl.kind == SlideKind::Spinout) {
                step(&mut s, StepMode::Live, &t);
            }
            s.cars[0].vel.length() > 1.0
        })
        .count();
    assert!(recovered > 0 && recovered < 20, "{recovered}/20 recovered");
}

/// A spinout's rotation builds up from the car's turn and dies down again (to a stop, or
/// through a T-stop) without ever jumping.
#[test]
fn spinout_rotation_is_eased() {
    let t = Tuning::default();
    let full = t.crash.spin_deg_per_s.to_radians();
    let mut t_stopped = 0;
    for seed in 0..100 {
        let mut s = in_a_bend(25.0, 0.2, &t);
        s.rng = SimRng::new(seed);
        let mut rate = s.cars[0].yaw_rate;
        crash::start(&mut s.cars[0], Crash::Spinout, &t, &mut SimRng::new(seed));
        step(&mut s, StepMode::Live, &t);
        assert!((s.cars[0].yaw_rate - rate).abs() < 0.05 * full, "seed {seed}: starts at {} rad/s, was turning at {rate}", s.cars[0].yaw_rate);
        while s.cars[0].slide.is_some() {
            rate = s.cars[0].yaw_rate;
            step(&mut s, StepMode::Live, &t);
            if s.cars[0].slide.is_some() {
                assert!((s.cars[0].yaw_rate - rate).abs() < 0.1 * full, "seed {seed}: rotation jumped {rate} -> {}", s.cars[0].yaw_rate);
            }
            if s.cars[0].slide.is_some_and(|sl| sl.kind == SlideKind::TStop) && s.cars[0].slide.unwrap().age == 1 {
                t_stopped += 1;
            }
        }
    }
    assert!(t_stopped > 0, "no seed T-stopped");
}

/// Roll: turns sideways (eased), −20 mph/s to a stop. Vault: flies 1–6" first.
#[test]
fn rolls_and_vaults() {
    let t = Tuning::default();
    let mut s = car_at(25.0);
    s.cars[0].yaw_rate = -1.0;
    crash::start(&mut s.cars[0], Crash::Roll, &t, &mut SimRng::new(1));
    step(&mut s, StepMode::Ideal, &t);
    assert!(s.cars[0].pose.heading.abs() < 0.1, "turned sideways at once");
    for _ in 1..(t.crash.roll_turn_seconds * 60.0).round() as u32 {
        step(&mut s, StepMode::Ideal, &t);
    }
    assert!((s.cars[0].pose.heading + FRAC_PI_2).abs() < 1e-5, "turns sideways into the turn");
    while s.cars[0].slide.is_some() {
        step(&mut s, StepMode::Ideal, &t);
    }
    // v² / 2a with a = 20 mph/s.
    let expected = 25.0 * 25.0 / (2.0 * 20.0 / MPH);
    assert!((s.cars[0].pose.pos.x - expected).abs() < 1.0 && s.cars[0].vel == Vec2::ZERO, "rolled {} m", s.cars[0].pose.pos.x);

    let mut s = car_at(25.0);
    crash::start(&mut s.cars[0], Crash::Vault, &t, &mut SimRng::new(1));
    let flight = s.cars[0].slide.unwrap().distance;
    assert!((INCH - 0.01..=6.0 * INCH + 0.01).contains(&flight));
    while s.cars[0].slide.is_some_and(|sl| sl.kind == SlideKind::Airborne) {
        step(&mut s, StepMode::Ideal, &t);
    }
    assert_eq!(s.cars[0].slide.map(|sl| sl.kind), Some(SlideKind::Roll), "lands rolling");
}

/// Fishtails swing the rear about a front corner while the car keeps going, then it eases
/// onto its new heading. One that also rolled on table 1 goes on into that result.
#[test]
fn fishtails_pivot_on_a_front_corner() {
    let t = Tuning::default();
    let vt = &t.vehicle;
    for (crash, deg) in [(Crash::MinorFishtail, 15.0f32), (Crash::MajorFishtail, 30.0), (Crash::DoubleFishtail, 45.0)] {
        let mut s = car_at(20.0);
        s.cars[0].yaw_rate = 0.5;
        let corner = |c: &sim::Car| c.pose.pos + c.pose.forward() * vt.length * 0.5 - c.pose.left() * vt.width * 0.5;
        let before = corner(&s.cars[0]);
        crash::start(&mut s.cars[0], crash, &t, &mut SimRng::new(1));
        let mut ticks = 0;
        while s.cars[0].slide.is_some() {
            step(&mut s, StepMode::Ideal, &t);
            ticks += 1;
            // Going straight on, the pivot corner only moves along the old direction.
            assert!((corner(&s.cars[0]).y - before.y).abs() < 1e-3, "{crash:?}: the pivot corner moved sideways");
        }
        assert!(ticks > 1, "{crash:?}: swung at once");
        assert!((s.cars[0].pose.heading - deg.to_radians()).abs() < 1e-5, "{crash:?}");
        for _ in 0..30 {
            step(&mut s, StepMode::Ideal, &t);
        }
        let car = &s.cars[0];
        assert!(car.vel.normalize().dot(car.pose.forward()) > 0.999, "{crash:?}: not along the nose");
    }

    let followed = (0..40).find_map(|seed| {
        let mut s = car_at(20.0);
        let (_, then) = crash::lose_control_to_hazard(&mut s.cars[0], 6.0, &t, &mut SimRng::new(seed));
        let then = then?;
        assert_eq!(s.cars[0].slide.map(|sl| sl.kind), Some(SlideKind::Fishtail));
        while s.cars[0].slide.is_some_and(|sl| sl.kind == SlideKind::Fishtail) {
            step(&mut s, StepMode::Live, &t);
        }
        Some((then, s.cars[0].slide.map(|sl| sl.kind)))
    });
    let Some((then, kind)) = followed else { panic!("no hazard rolled on table 1") };
    let expected = match then {
        Crash::TrivialSkid | Crash::MinorSkid | Crash::ModerateSkid | Crash::SevereSkid => SlideKind::Skid,
        Crash::Spinout => SlideKind::Spinout,
        Crash::Vault => SlideKind::Airborne,
        _ => SlideKind::Roll,
    };
    assert_eq!(kind, Some(expected), "after the fishtail: {then:?}");
}

/// A loss during a sharp maneuver at speed rolls a worse result than one during a gentle
/// one at low speed.
#[test]
fn harder_and_faster_crashes_worse() {
    let severity = |c: Crash| match c {
        Crash::TrivialSkid => 0,
        Crash::MinorSkid => 1,
        Crash::ModerateSkid => 2,
        Crash::SevereSkid => 3,
        Crash::Spinout => 4,
        _ => 5,
    };
    let t = Tuning::default();
    let mean = |difficulty: f32, speed: f32| {
        (0..200)
            .map(|seed| {
                let mut s = car_at(speed);
                severity(crash::lose_control(&mut s.cars[0], difficulty, &t, &mut SimRng::new(seed)))
            })
            .sum::<u32>() as f32
            / 200.0
    };
    assert!(mean(6.0, 40.0) > mean(1.0, 8.0) + 2.0, "{} vs {}", mean(6.0, 40.0), mean(1.0, 8.0));
    assert!(crash::severe_chance(6.0, 40.0, 0.0) > crash::severe_chance(1.0, 8.0, 0.0));
}

/// A car that has driven half a second round a steady bend at `speed` with `steer`.
fn in_a_bend(speed: f32, steer: f32, t: &Tuning) -> SimState {
    let mut s = car_at(speed);
    s.cars[0].controls = sim::Controls { steer, throttle: sim::hold_throttle(speed, &t.vehicle), ..Default::default() };
    for _ in 0..30 {
        step(&mut s, StepMode::Ideal, t);
    }
    s
}

/// Losing control in a bend, the car goes on along the tangent, not out of it.
#[test]
fn loss_of_control_in_a_bend_goes_on_along_the_tangent() {
    let t = Tuning::default();
    for crash in [Crash::ModerateSkid, Crash::Spinout, Crash::Roll] {
        let mut s = in_a_bend(15.0, 0.4, &t);
        let tangent = s.cars[0].vel.normalize();
        crash::start(&mut s.cars[0], crash, &t, &mut SimRng::new(1));
        let start = s.cars[0].pose.pos;
        for _ in 0..10 {
            step(&mut s, StepMode::Ideal, &t);
        }
        let moved = s.cars[0].pose.pos - start;
        assert!(moved.perp_dot(tangent).abs() < 1e-3, "{crash:?}: {} m off the tangent", moved.perp_dot(tangent));
    }
}

fn skid_run(start: &SimState, angle: f32, slide: f32, throttle: f32, ticks: u32, t: &Tuning) -> SimState {
    let mut s = start.clone();
    s.cars[0].drive_mode = DriveMode::Orders;
    let drive = Drive { turn: Turn::Skid { angle, slide }, throttle };
    s.cars[0].orders = OrderQueue { segments: vec![Segment::new(drive, ticks)], tail: Drive::COAST };
    for _ in 0..ticks {
        step(&mut s, StepMode::Ideal, t);
    }
    s
}

/// A controlled skid keeps moving the way the car was going for its time (¼" → 0.25 s …
/// 1" → 1 s) while the nose swings round over the first 0.25 s, then drives off along the
/// new nose. It costs the bend plus 1 point per ¼", at the start, and slows the car by the
/// book.
#[test]
fn controlled_skid_swings_the_nose_and_keeps_going() {
    let t = Tuning::default();
    let start = car_at(15.0);
    let hold = sim::hold_throttle(15.0, &t.vehicle);
    for (deg, slide, ticks) in [(30.0f32, 0.25, 15), (45.0, 0.5, 30), (-60.0, 1.0, 60)] {
        let angle = deg.to_radians();
        let mut s = start.clone();
        s.cars[0].drive_mode = DriveMode::Orders;
        let drive = Drive { turn: Turn::Skid { angle, slide }, throttle: hold };
        s.cars[0].orders = OrderQueue { segments: vec![Segment::new(drive, 90)], tail: Drive::COAST };
        let (mut drained, mut first_drain, mut slid_ticks, mut turned_at) = (0.0, None, 0, None);
        let mut going = 0.0f32;
        for tick in 1..=90 {
            let sliding = s.cars[0].slide.is_some() || tick == 1;
            step(&mut s, StepMode::Ideal, &t);
            let c = &s.cars[0];
            drained += c.drain;
            first_drain.get_or_insert(c.drain);
            if sliding {
                slid_ticks += 1;
                assert!(c.pose.pos.y.abs() < 1e-4, "{deg}°: left the old direction while sliding");
                if c.slide.is_some() {
                    assert!(c.vel.y.abs() < 1e-4, "{deg}°: moving off the old direction while sliding");
                }
            }
            // The direction of travel eases onto the new nose, never jumping.
            let now = c.vel.to_angle();
            assert!((now - going).abs() < 0.1, "{deg}°: direction jumped {going} -> {now}");
            going = now;
            if turned_at.is_none() && (c.pose.heading - angle).abs() < 1e-4 {
                turned_at = Some(tick);
            }
        }
        let c = &s.cars[0];
        let case = format!("{deg}°, {slide}\"");
        assert_eq!(slid_ticks, ticks, "{case}: slid {slid_ticks} ticks");
        assert_eq!(turned_at, Some(15), "{case}: nose done at tick {turned_at:?}");
        assert!((c.pose.heading - angle).abs() < 1e-4, "{case}: heading {}", c.pose.heading.to_degrees());
        assert!(c.vel.normalize().dot(c.pose.forward()) > 0.9999, "{case}: not along the nose");
        let cost = deg.abs() / 15.0 + 4.0 * slide;
        assert!((first_drain.unwrap() - cost).abs() < 1e-3 && (drained - cost).abs() < 1e-3, "{case}: drained {drained}, expected {cost} at once");
        let lost = sim::control::skid_speed_loss(slide);
        // (Losing it while sliding, the throttle that held 15 m/s pushes a little harder.)
        assert!((c.forward_speed() - (15.0 - lost)).abs() < 0.15, "{case}: speed {}", c.forward_speed());
    }
    assert_eq!(sim::control::skid_speed_loss(0.25), 0.0);
    assert!((sim::control::skid_speed_loss(1.0) * MPH - 10.0).abs() < 1e-4);
}

/// The skid fit swings the nose to point from the slide's end at the target, and predicts
/// where the car really ends (also when the target needs more than 90°: at 20 m/s a ½" skid
/// slides 10 m on, past a target 6 m ahead).
#[test]
fn fit_skid_reaches_targets() {
    let t = Tuning::default();
    for (v, x, y, reachable) in [(15.0, 25.0, 6.0, true), (15.0, 25.0, -6.0, true), (10.0, 12.0, 8.0, true), (20.0, 6.0, 15.0, false)] {
        let start = car_at(v);
        let fit = fit_skid(&start, &t, start.cars[0].id, 0.0, 0.5, Vec2::new(x, y), 300);
        let Turn::Skid { angle, slide } = fit.turn else { panic!("not a skid") };
        let s = skid_run(&start, angle, slide, 0.0, fit.ticks, &t);
        let end = s.cars[0].pose.pos;
        let case = format!("{v} m/s -> ({x}, {y}): {fit:?}, ended at {end}");
        assert!((end.distance(Vec2::new(x, y)) - fit.miss).abs() < 0.3, "{case}");
        assert!(!reachable || fit.miss < 0.3, "{case}");
        assert!(angle.abs() <= FRAC_PI_2 + 1e-5);
    }
}

/// The skid is checked when it starts (bend plus slide at once); losing control there
/// replaces the skid with the crash result.
#[test]
fn controlled_skid_is_checked_when_it_starts() {
    let t = Tuning::default();
    let skid = |s: &mut SimState| {
        s.cars[0].drive_mode = DriveMode::Orders;
        // 45° and ½": 5 points at once from status 3 at 30 m/s (67 mph): about 46%.
        let drive = Drive { turn: Turn::Skid { angle: 45f32.to_radians(), slide: 0.5 }, throttle: 0.0 };
        s.cars[0].orders = OrderQueue { segments: vec![Segment::new(drive, 60)], tail: Drive::COAST };
    };
    let mut s = car_at(30.0);
    skid(&mut s);
    let events = step(&mut s, StepMode::Ideal, &t);
    let chance = events.iter().find_map(|e| if let SimEvent::Risk { chance, .. } = e { Some(*chance) } else { None });
    // (Checked at the speed after this tick's coasting, a hair below 30 m/s.)
    let expected = sim::handling::chance(5.0, 3.0 - 5.0, 30.0, &t);
    assert!(chance.is_some_and(|c| (c - expected).abs() < 0.005), "check at the start: {chance:?}, expected {expected}");
    assert_eq!(Some(s.cars[0].risk), chance);

    // Rolled: some seeds keep control and skid, others crash instead.
    let (mut skidded, mut crashed) = (0, 0);
    for seed in 0..40 {
        let mut s = car_at(30.0);
        s.rng = SimRng::new(seed);
        skid(&mut s);
        let events = step(&mut s, StepMode::Live, &t);
        if events.iter().any(|e| matches!(e, SimEvent::LossOfControl { .. })) {
            crashed += 1;
            assert_ne!(s.cars[0].slide.map(|sl| sl.kind), Some(SlideKind::Controlled));
        } else {
            skidded += 1;
            assert_eq!(s.cars[0].slide.map(|sl| sl.kind), Some(SlideKind::Controlled));
        }
    }
    assert!(skidded > 0 && crashed > 0, "{skidded} skidded, {crashed} crashed");
}
