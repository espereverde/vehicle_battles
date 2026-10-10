use sim::{Controls, DriveMode, Pose, SimEvent, SimRng, SimState, StepMode, Tuning, Vec2, World};

/// A scripted but erratic driver: holds random controls for random durations, including
/// full-lock steering and handbrake turns, so the run exercises grip and slide paths.
struct ScriptedDriver {
    rng: SimRng,
    current: Controls,
    hold: u32,
}

impl ScriptedDriver {
    fn new(seed: u64) -> Self {
        Self { rng: SimRng::new(seed), current: Controls::default(), hold: 0 }
    }

    fn next(&mut self) -> Controls {
        if self.hold == 0 {
            let r = &mut self.rng;
            self.current = Controls {
                steer: r.range_f32(-1.0, 1.0),
                throttle: r.range_f32(-0.3, 1.0),
                brake: if r.next_f32() < 0.2 { r.next_f32() } else { 0.0 },
                handbrake: r.next_f32() < 0.1,
            };
            self.hold = 10 + (self.rng.next_u64() % 50) as u32;
        }
        self.hold -= 1;
        self.current
    }
}

fn initial_state() -> SimState {
    let mut s = SimState::new(42);
    s.spawn_car(Pose::new(Vec2::ZERO, 0.0));
    s.spawn_car(Pose::new(Vec2::new(10.0, -5.0), 1.0));
    s.spawn_car(Pose::new(Vec2::new(-20.0, 8.0), -2.5));
    for (i, car) in s.cars.iter_mut().enumerate() {
        car.drive_mode = DriveMode::Direct;
        car.vel = car.pose.forward() * (10.0 + 8.0 * i as f32);
    }
    s
}

/// Runs `ticks` ticks, feeding each car from its own scripted driver.
fn run(state: &mut SimState, drivers: &mut [ScriptedDriver], ticks: u32) -> Vec<SimEvent> {
    let (world, tuning) = (World::default(), Tuning::default());
    let mut events = Vec::new();
    for _ in 0..ticks {
        for (car, d) in state.cars.iter_mut().zip(drivers.iter_mut()) {
            car.controls = d.next();
        }
        events.extend(sim::step(state, &world, &tuning, StepMode::Live).events);
    }
    events
}

fn drivers() -> Vec<ScriptedDriver> {
    (0..3).map(|i| ScriptedDriver::new(1000 + i)).collect()
}

/// `{:?}` of an f32 is the shortest string that round-trips, so equal debug output means
/// bitwise-equal floats (it also distinguishes -0.0 from 0.0).
fn assert_bitwise_eq(a: &SimState, b: &SimState) {
    assert_eq!(format!("{a:?}"), format!("{b:?}"));
}

#[test]
fn cloned_state_steps_bitwise_identically() {
    let mut a = initial_state();
    run(&mut a, &mut drivers(), 120); // diverge from the trivial initial state first

    let mut b = a.clone();
    let mut da = drivers();
    let mut db = drivers();
    let ea = run(&mut a, &mut da, 1000);
    let eb = run(&mut b, &mut db, 1000);

    assert_bitwise_eq(&a, &b);
    assert_eq!(ea, eb);
    assert_eq!(a.tick, 1120);

    // Make sure the test actually exercised the interesting code paths.
    assert!(ea.iter().any(|e| matches!(e, SimEvent::SlideStarted { .. })), "no slides happened");
    assert!(ea.iter().any(|e| matches!(e, SimEvent::SlideEnded { .. })), "no slide recovered");
}

#[test]
fn rng_is_part_of_state() {
    let mut a = initial_state();
    let mut b = a.clone();
    let xs: Vec<u64> = (0..10).map(|_| a.rng.next_u64()).collect();
    let ys: Vec<u64> = (0..10).map(|_| b.rng.next_u64()).collect();
    assert_eq!(xs, ys);
}

#[test]
fn tuning_file_parses() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/tuning.ron");
    let text = std::fs::read_to_string(path).unwrap();
    let parsed: Tuning = ron::from_str(&text).unwrap();
    assert_eq!(parsed.tick_rate, 60);
}
