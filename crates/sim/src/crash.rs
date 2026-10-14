//! Loss of control (Car Wars crash tables) and the slide motion it puts the car in.
//!
//! A loss of control rolls 2d6 + (D − 3) + speed modifier − driver skill on Crash Table 1
//! (maneuvers) or 2 (hazards). D is the difficulty of what the car was doing (see
//! `handling::update`). Results move the car kinematically:
//!
//! - Skids slide straight on along the direction the car was going (in a bend, the
//!   tangent) while the nose keeps turning as it was: trivial ¼", minor ½" (−5 mph), moderate 1" (−10 mph), severe 1½"
//!   (−20 mph). The book's follow-up skids are added into these distances.
//! - Spinout: rotates 90° per 0.2 s while sliding and slowing 20 mph/s until stopped. Every
//!   `recovery_seconds` the driver tries to recover (a control roll at handling −6): facing
//!   the way the car goes, it drives on; otherwise it T-stops (sliding sideways, slowing
//!   20 mph per inch).
//! - Roll: turns sideways and rolls, slowing 20 mph/s until stopped (burning on 4–6). Vault:
//!   flies 1d6" first, then rolls.
//! - Fishtails (table 2): the rear swings 15° / 30° / 45° about a front corner; worse results
//!   also roll on table 1, once the rear has swung.
//!
//! While sliding the car ignores its controls (its orders keep their time) and makes no
//! control checks; a slide ends with the car moving along its heading again, or stopped. A
//! controlled skid (`orders::Turn::Skid`) is the same slide, its nose swinging round first.
//!
//! The book's results are kept (distances, final headings, speed lost) but nothing happens
//! in one tick (`ease.rs`): the direction of travel swings into the slide and back onto the
//! nose after it, the nose swings are eased in and out, a spinout's rotation builds up and
//! winds down, and a skid's speed loss scrubs off as it slides.

use std::f32::consts::{FRAC_PI_2, PI};

use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::ease::{Slip, smoothstep};
use crate::handling::{INCH, MPH};
use crate::rng::SimRng;
use crate::state::{Car, Controls};
use crate::tuning::{Tuning, VehicleTuning};
use crate::vehicle::{longitudinal_accel, wrap_angle};

/// A crash table result.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Crash {
    TrivialSkid,
    MinorSkid,
    ModerateSkid,
    SevereSkid,
    Spinout,
    Roll,
    BurningRoll,
    Vault,
    MinorFishtail,
    MajorFishtail,
    DoubleFishtail,
}

impl Crash {
    /// Spinout or worse.
    pub fn severe(&self) -> bool {
        matches!(self, Crash::Spinout | Crash::Roll | Crash::BurningRoll | Crash::Vault)
    }
}

/// What a sliding car is doing.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SlideKind {
    /// A deliberate controlled skid (`Turn::Skid`): speed follows the pedals.
    Controlled,
    Skid,
    Spinout,
    TStop,
    Roll,
    /// Vault: in the air, then a roll.
    Airborne,
    /// The rear swings round about a front corner while the car keeps going.
    Fishtail,
}

/// The car moves along `dir` while its heading does its own thing.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Slide {
    pub kind: SlideKind,
    /// Direction of travel (unit, world space).
    pub dir: Vec2,
    /// Meters left (`f32::INFINITY`: until stopped).
    pub distance: f32,
    /// Heading rotation, rad/s, eased from `spin_from` (what it was when the slide began)
    /// over `spin_up_seconds`: a spinout spins up, a T-stop's spin dies down, a skid's nose
    /// keeps turning as it was.
    pub spin: f32,
    #[serde(default)]
    pub spin_from: f32,
    /// Heading change (radians, signed) eased in and out over the first `swing_ticks`.
    #[serde(default)]
    pub swing: f32,
    #[serde(default)]
    pub swing_ticks: u32,
    /// Ticks slid so far.
    #[serde(default)]
    pub age: u32,
    /// Ticks left, for slides that last a time rather than a distance.
    #[serde(default)]
    pub ticks_left: Option<u32>,
    /// Speed still to lose (m/s), scrubbed off as the slide goes on.
    pub speed_loss: f32,
    /// Spinout: seconds to the next recovery attempt.
    pub recover_in: f32,
    pub burning: bool,
    /// Fishtail: the table 1 result that follows once the rear has swung.
    #[serde(default)]
    pub then: Option<Crash>,
}

impl Slide {
    fn new(kind: SlideKind, dir: Vec2, distance: f32) -> Self {
        Self {
            kind,
            dir,
            distance,
            spin: 0.0,
            spin_from: 0.0,
            swing: 0.0,
            swing_ticks: 0,
            age: 0,
            ticks_left: None,
            speed_loss: 0.0,
            recover_in: f32::INFINITY,
            burning: false,
            then: None,
        }
    }

    /// A controlled skid: slides along `dir` for `ticks` ticks while the nose turns by
    /// `angle` over the first `turn_ticks`.
    pub fn controlled(dir: Vec2, ticks: u32, angle: f32, turn_ticks: u32, speed_loss: f32) -> Self {
        let ticks = ticks.max(1);
        Self {
            swing: angle,
            swing_ticks: turn_ticks.clamp(1, ticks),
            ticks_left: Some(ticks),
            speed_loss,
            ..Self::new(SlideKind::Controlled, dir, f32::INFINITY)
        }
    }

    /// Heading change from the swing over this slide's tick `age` (counting from 1).
    fn swing_at(&self, age: u32) -> f32 {
        let eased = |a: u32| smoothstep(a as f32 / self.swing_ticks.max(1) as f32);
        self.swing * (eased(age) - eased(age - 1))
    }
}

/// Ticks in `seconds` (at least 1).
fn ticks(seconds: f32, tuning: &Tuning) -> u32 {
    (seconds * tuning.tick_rate as f32).round().max(1.0) as u32
}

/// Puts the car into `slide`. Its direction of travel swings over to the slide's from where
/// it is going now.
pub fn begin(car: &mut Car, slide: Slide, tuning: &Tuning) {
    car.slip = entry_slip(car.vel, slide.dir, tuning);
    car.slide = Some(slide);
}

/// Slip of a car going at `vel` that starts sliding along `dir`.
pub fn entry_slip(vel: Vec2, dir: Vec2, tuning: &Tuning) -> Option<Slip> {
    let going = if vel.length_squared() > 1e-6 { vel.to_angle() } else { dir.to_angle() };
    Slip::new(wrap_angle(going - dir.to_angle()), tuning.crash.slide_entry_seconds)
}

/// Control Table's crash modifier at `speed` (m/s): −3 up to 10 mph, 0 at 35–40, +1 at
/// 45–60, then +1 per 30 mph.
pub fn speed_modifier(speed: f32) -> i32 {
    let mph = speed.abs() * MPH;
    if mph <= 40.0 { ((mph / 10.0).ceil() as i32 - 4).max(-3) } else if mph <= 60.0 { 1 } else { 1 + ((mph - 35.0) / 30.0).floor() as i32 }
}

/// Crash table modifier: (D − 3) + speed modifier − skill, rounded.
pub fn modifier(difficulty: f32, speed: f32, skill: f32) -> i32 {
    (difficulty - 3.0 - skill).round() as i32 + speed_modifier(speed)
}

/// Ways to roll each total on 2d6 (index 2..=12).
const TWO_D6: [u32; 13] = [0, 0, 1, 2, 3, 4, 5, 6, 5, 4, 3, 2, 1];

/// Chance that a 2d6 roll plus `modifier` is at least `n`.
pub fn chance_at_least(n: i32, modifier: i32) -> f32 {
    (2..=12).filter(|&r| r as i32 + modifier >= n).map(|r| TWO_D6[r]).sum::<u32>() as f32 / 36.0
}

/// Crash Table 1 (maneuvers).
pub fn table1(roll: i32) -> Crash {
    match roll {
        ..=2 => Crash::TrivialSkid,
        3..=4 => Crash::MinorSkid,
        5..=6 => Crash::ModerateSkid,
        7..=8 => Crash::SevereSkid,
        9..=10 => Crash::Spinout,
        11..=12 => Crash::Roll,
        13..=14 => Crash::BurningRoll,
        _ => Crash::Vault,
    }
}

/// Crash Table 2 (hazards): the fishtail, and whether table 1 is rolled too.
pub fn table2(roll: i32) -> (Crash, bool) {
    match roll {
        ..=4 => (Crash::MinorFishtail, false),
        5..=8 => (Crash::MajorFishtail, false),
        9..=10 => (Crash::MinorFishtail, true),
        11..=14 => (Crash::MajorFishtail, true),
        _ => (Crash::DoubleFishtail, true),
    }
}

fn d6(rng: &mut SimRng) -> i32 {
    (rng.next_u64() % 6) as i32 + 1
}

/// Loses control of `car` after a maneuver of `difficulty`: rolls table 1 and starts the
/// result. Returns it.
pub fn lose_control(car: &mut Car, difficulty: f32, tuning: &Tuning, rng: &mut SimRng) -> Crash {
    let m = modifier(difficulty, car.vel.length(), car.pilot.skill);
    let crash = table1(d6(rng) + d6(rng) + m);
    start(car, crash, tuning, rng);
    crash
}

/// Loses control of `car` after a hazard of `difficulty`: rolls table 2 (and table 1 when
/// it says so). Returns the fishtail and the table 1 result, if any.
pub fn lose_control_to_hazard(car: &mut Car, difficulty: f32, tuning: &Tuning, rng: &mut SimRng) -> (Crash, Option<Crash>) {
    let m = modifier(difficulty, car.vel.length(), car.pilot.skill);
    let (fishtail, more) = table2(d6(rng) + d6(rng) + m);
    let then = more.then(|| table1(d6(rng) + d6(rng) + m));
    start(car, fishtail, tuning, rng);
    if let Some(slide) = &mut car.slide {
        slide.then = then;
    }
    (fishtail, then)
}

/// Puts the car into a crash result.
pub fn start(car: &mut Car, crash: Crash, tuning: &Tuning, rng: &mut SimRng) {
    let c = &tuning.crash;
    // Straight on the way the car was going: in a bend, along the tangent.
    let dir = car.travel_dir();
    // Which way it rotates: into the turn, or at random when going straight.
    let side = if car.yaw_rate.abs() > 1e-3 { car.yaw_rate.signum() } else if rng.next_f32() < 0.5 { -1.0 } else { 1.0 };
    let yaw_rate = car.yaw_rate;
    let skid = |inches: f32, mph_lost: f32| Slide { spin: yaw_rate, spin_from: yaw_rate, speed_loss: mph_lost / MPH, ..Slide::new(SlideKind::Skid, dir, inches * INCH) };
    let roll = |dir: Vec2| Slide::new(SlideKind::Roll, dir, f32::INFINITY);
    let slide = match crash {
        Crash::TrivialSkid => skid(0.25, 0.0),
        Crash::MinorSkid => skid(0.5, 5.0),
        Crash::ModerateSkid => skid(1.0, 10.0),
        Crash::SevereSkid => skid(1.5, 20.0),
        Crash::Spinout => Slide {
            spin: side * c.spin_deg_per_s.to_radians(),
            spin_from: car.yaw_rate,
            recover_in: c.recovery_seconds,
            ..Slide::new(SlideKind::Spinout, dir, f32::INFINITY)
        },
        // Turns sideways (into the turn) as it starts to roll.
        Crash::Roll | Crash::BurningRoll => Slide {
            swing: wrap_angle(dir.to_angle() + side * FRAC_PI_2 - car.pose.heading),
            swing_ticks: ticks(c.roll_turn_seconds, tuning),
            burning: crash == Crash::BurningRoll && d6(rng) >= 4,
            ..roll(dir)
        },
        Crash::Vault => Slide::new(SlideKind::Airborne, dir, d6(rng) as f32 * INCH),
        // The rear swings round while the car keeps going the way it goes now.
        Crash::MinorFishtail | Crash::MajorFishtail | Crash::DoubleFishtail => {
            let deg: f32 = match crash {
                Crash::MinorFishtail => 15.0,
                Crash::MajorFishtail => 30.0,
                _ => 45.0,
            };
            let n = ticks(c.fishtail_seconds, tuning);
            let going = if car.vel.length_squared() > 1e-6 { car.vel.normalize() } else { car.pose.forward() };
            Slide { swing: side * deg.to_radians(), swing_ticks: n, ticks_left: Some(n), ..Slide::new(SlideKind::Fishtail, going, f32::INFINITY) }
        }
    };
    begin(car, slide, tuning);
}

/// Turns the car by `angle` about the front corner on the far side (a fishtail's swing).
fn pivot(car: &mut Car, angle: f32, vt: &VehicleTuning) {
    let (fwd, left) = (car.pose.forward(), car.pose.left());
    // Rear swinging left (heading turning right) pivots on the right front corner, and back.
    let corner = car.pose.pos + fwd * vt.length * 0.5 - left * angle.signum() * vt.width * 0.5;
    car.pose.pos = corner + Vec2::from_angle(angle).rotate(car.pose.pos - corner);
}

/// One tick of a slide. `controls` drive the speed of a controlled skid; `rng` (rolled modes
/// only) is for spinout recovery. Returns whether the slide ended.
pub fn step(car: &mut Car, controls: &Controls, tuning: &Tuning, dt: f32, mut rng: Option<&mut SimRng>) -> bool {
    let Some(mut slide) = car.slide else { return false };
    let c = &tuning.crash;
    let v = car.vel.length();
    let decel = c.decel_mph_s / MPH;
    let mut v_new = match slide.kind {
        SlideKind::Controlled => (v + longitudinal_accel(controls, v, &tuning.vehicle, dt) * dt).max(0.0),
        SlideKind::Skid | SlideKind::Airborne | SlideKind::Fishtail => v,
        SlideKind::Spinout | SlideKind::Roll => (v - decel * dt).max(0.0),
        SlideKind::TStop => (v - c.tstop_mph_per_inch / MPH / INCH * v * dt).max(0.0),
    };
    let moved = (v * dt).min(slide.distance);
    // The speed loss scrubs off evenly over the slide's time or distance.
    let progress = match slide.ticks_left {
        Some(left) => 1.0 / left.max(1) as f32,
        None if slide.distance > 0.0 => moved / slide.distance,
        None => 1.0,
    };
    let scrub = slide.speed_loss * progress;
    slide.speed_loss -= scrub;
    v_new = (v_new - scrub).max(0.0);
    if let Some(left) = &mut slide.ticks_left {
        *left = left.saturating_sub(1);
    }

    // The nose: an eased swing, and a spinout's rotation, building up and winding down.
    slide.age += 1;
    let swing = slide.swing_at(slide.age);
    let spin_up = smoothstep(slide.age as f32 * dt / c.spin_up_seconds.max(1e-3));
    let spin = (slide.spin_from + (slide.spin - slide.spin_from) * spin_up) * (v_new / c.spin_fade_speed.max(1e-3)).min(1.0);
    if slide.kind == SlideKind::Fishtail {
        pivot(car, swing, &tuning.vehicle);
    }
    car.yaw_rate = spin + swing / dt;
    car.pose.heading = wrap_angle(car.pose.heading + spin * dt + swing);

    // The direction of travel eases into the slide's.
    car.slip = car.slip.and_then(|s| s.advance(dt));
    let travel = car.travel_dir();
    car.pose.pos += travel * moved;
    slide.distance -= moved;
    car.accel = (travel * v_new - car.vel) / dt;
    car.vel = travel * v_new;

    // Spinout recovery: a control roll at handling −6.
    if slide.kind == SlideKind::Spinout {
        slide.recover_in -= dt;
        if slide.recover_in <= 0.0 {
            slide.recover_in = c.recovery_seconds;
            let keep = 1.0 - crate::handling::p_fail(tuning.handling.min_status, v_new, tuning);
            if let Some(rng) = rng.as_deref_mut()
                && rng.next_f32() < keep
            {
                let off = wrap_angle(travel.to_angle() - car.pose.heading).abs();
                if off < PI / 4.0 {
                    slide.distance = 0.0;
                } else {
                    slide = Slide { spin_from: car.yaw_rate, ..Slide::new(SlideKind::TStop, slide.dir, f32::INFINITY) };
                }
            }
        }
    }

    let stopped = v_new < c.stop_speed;
    let done = match slide.ticks_left {
        Some(left) => left == 0,
        None => slide.distance <= 1e-4 || (slide.distance.is_infinite() && stopped),
    };
    if !done {
        car.slide = Some(slide);
        return false;
    }
    car.slide = None;
    // A fishtail that also rolled on table 1 goes on into that.
    if let (Some(crash), Some(rng)) = (slide.then, rng) {
        start(car, crash, tuning, rng);
        return false;
    }
    match slide.kind {
        // A vault lands and rolls on.
        SlideKind::Airborne => {
            car.slide = Some(Slide::new(SlideKind::Roll, slide.dir, f32::INFINITY));
            return false;
        }
        // Drives on (a recovered spinout too), easing from the slide onto its nose.
        SlideKind::Controlled | SlideKind::Skid | SlideKind::Fishtail | SlideKind::Spinout if !stopped => {
            car.slip = Slip::new(wrap_angle(travel.to_angle() - car.pose.heading), c.regrip_seconds);
        }
        _ => {
            car.vel = Vec2::ZERO;
            car.slip = None;
        }
    }
    true
}

/// Chance that a loss of control after a maneuver of `difficulty` at `speed` is a spinout
/// or worse.
pub fn severe_chance(difficulty: f32, speed: f32, skill: f32) -> f32 {
    chance_at_least(9, modifier(difficulty, speed, skill))
}
