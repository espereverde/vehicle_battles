//! "Parking challenge": reach a sequence of target boxes (position + heading + max speed)
//! as fast as possible in sim time. R restarts the course, N generates a new one.

use std::f32::consts::PI;

use bevy::color::palettes::css;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use sim::{CarId, Pose, SimRng, SimState};

use crate::render::{WorldView, car_outline};
use crate::track::Mode;
use crate::{AppState, Sim, TuningRes};

pub struct ChallengePlugin;

impl Plugin for ChallengePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Challenge::new(1))
            .add_systems(OnEnter(AppState::Paused), count_pause.run_if(resource_equals(Mode::Parking)))
            .add_systems(Update, draw_targets.run_if(resource_equals(Mode::Parking)))
            .add_systems(EguiPrimaryContextPass, (challenge_panel, target_labels).run_if(resource_equals(Mode::Parking)));
    }
}

const TARGETS: usize = 5;
const ARENA_HALF: f32 = 75.0;
/// Speed that counts as "stopped" for STOP targets.
const STOP_SPEED: f32 = 1.0;

#[derive(Debug, Clone, Copy)]
pub struct Target {
    pub pose: Pose,
    /// Box size (length along the target heading, width).
    pub size: Vec2,
    pub max_speed: f32,
    pub heading_tol: f32,
}

impl Target {
    /// Car center inside the box, heading within tolerance, and slow enough.
    pub fn satisfied(&self, pose: Pose, speed: f32) -> bool {
        let d = pose.pos - self.pose.pos;
        let along = d.dot(self.pose.forward()).abs();
        let across = d.dot(self.pose.left()).abs();
        let heading_err = sim::vehicle::wrap_angle(pose.heading - self.pose.heading).abs();
        along <= self.size.x * 0.5
            && across <= self.size.y * 0.5
            && heading_err <= self.heading_tol
            && speed <= self.max_speed
    }

    pub fn label(&self) -> String {
        if self.max_speed <= STOP_SPEED { "STOP".into() } else { format!("≤ {:.0} km/h", crate::ui::kmh(self.max_speed)) }
    }
}

#[derive(Resource)]
pub struct Challenge {
    pub seed: u64,
    pub targets: Vec<Target>,
    /// Index of the next target to reach.
    pub next: usize,
    /// Tick at which each reached target was hit.
    pub hits: Vec<u64>,
    pub pauses: u32,
    /// Best completion time of this course, in ticks.
    pub best: Option<u64>,
}

impl Challenge {
    fn new(seed: u64) -> Self {
        Self { seed, targets: generate(seed), next: 0, hits: Vec::new(), pauses: 0, best: None }
    }

    pub fn restart(&mut self, new_course: bool) {
        if new_course {
            *self = Self::new(self.seed + 1);
        } else {
            *self = Self { best: self.best, ..Self::new(self.seed) };
        }
    }

    pub fn done(&self) -> bool {
        self.next >= self.targets.len()
    }

    pub fn current(&self) -> Option<&Target> {
        self.targets.get(self.next)
    }

    /// Checks the live state after a tick.
    pub fn update(&mut self, state: &SimState, player: CarId) {
        let (Some(target), Some(car)) = (self.current(), state.car(player)) else { return };
        if target.satisfied(car.pose, car.vel.length()) {
            self.hits.push(state.tick);
            self.next += 1;
            if self.done() {
                self.best = Some(self.best.map_or(state.tick, |b| b.min(state.tick)));
            }
        }
    }
}

fn generate(seed: u64) -> Vec<Target> {
    let mut rng = SimRng::new(seed.wrapping_mul(0x9E37_79B9));
    let (mut pos, mut heading) = (Vec2::ZERO, 0.0f32);
    (0..TARGETS)
        .map(|i| {
            let dir = heading + rng.range_f32(-2.0, 2.0);
            let dist = rng.range_f32(25.0, 50.0);
            pos = (pos + Vec2::from_angle(dir) * dist).clamp(Vec2::splat(-ARENA_HALF), Vec2::splat(ARENA_HALF));
            heading = rng.range_f32(-PI, PI);
            let max_speed = if i + 1 == TARGETS { STOP_SPEED } else { [STOP_SPEED, STOP_SPEED, 6.0, 12.0][(rng.next_u64() % 4) as usize] };
            Target { pose: Pose::new(pos, heading), size: Vec2::new(8.0, 4.5), max_speed, heading_tol: 15f32.to_radians() }
        })
        .collect()
}

fn count_pause(sim: Res<Sim>, mut challenge: ResMut<Challenge>) {
    if sim.state.tick > 0 && !challenge.done() {
        challenge.pauses += 1;
    }
}

fn draw_targets(mut gizmos: Gizmos, challenge: Res<Challenge>, tuning: Res<TuningRes>) {
    let a = ARENA_HALF + 15.0;
    gizmos.rect_2d(Isometry2d::IDENTITY, Vec2::splat(2.0 * a), css::SLATE_GRAY);
    for (i, t) in challenge.targets.iter().enumerate() {
        let color = match i.cmp(&challenge.next) {
            std::cmp::Ordering::Less => css::GREEN.with_alpha(0.3),
            std::cmp::Ordering::Equal => css::GOLD,
            std::cmp::Ordering::Greater => css::GOLD.with_alpha(0.25),
        };
        let iso = Isometry2d::new(t.pose.pos, Rot2::radians(t.pose.heading));
        gizmos.rect_2d(iso, t.size, color);
        let tip = t.pose.pos + t.pose.forward() * (t.size.x * 0.5 + 1.5);
        gizmos.arrow_2d(t.pose.pos, tip, color);
        if i == challenge.next {
            car_outline(&mut gizmos, t.pose, &tuning.vehicle, css::GOLD.with_alpha(0.25));
        }
    }
}

fn target_labels(mut contexts: EguiContexts, view: WorldView, challenge: Res<Challenge>) -> Result {
    let ctx = contexts.ctx_mut()?;
    let painter = ctx.layer_painter(egui::LayerId::background());
    for (i, t) in challenge.targets.iter().enumerate().skip(challenge.next) {
        if let Some(p) = view.to_screen(t.pose.pos - Vec2::Y * 3.5) {
            let alpha = if i == challenge.next { 255 } else { 90 };
            let text = format!("T{}  {}", i + 1, t.label());
            painter.text(p, egui::Align2::CENTER_TOP, text, egui::FontId::proportional(13.0), egui::Color32::from_rgba_unmultiplied(255, 215, 0, alpha));
        }
    }
    Ok(())
}

fn challenge_panel(mut contexts: EguiContexts, challenge: Res<Challenge>, sim: Res<Sim>, tuning: Res<TuningRes>) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("Parking challenge").anchor(egui::Align2::RIGHT_TOP, [-12.0, 12.0]).resizable(false).show(ctx, |ui| {
        let rate = tuning.tick_rate as f32;
        let end_tick = if challenge.done() { *challenge.hits.last().unwrap_or(&0) } else { sim.state.tick };
        ui.label(format!("Course #{}  —  reached {}/{}", challenge.seed, challenge.hits.len(), TARGETS));
        ui.heading(format!("{:.1} s", end_tick as f32 / rate));
        if let Some(t) = challenge.current() {
            ui.label(format!("Next: T{} — {}, heading ±{:.0}°", challenge.next + 1, t.label(), t.heading_tol.to_degrees()));
        } else {
            ui.colored_label(egui::Color32::LIGHT_GREEN, "Course complete!");
        }
        for (i, hit) in challenge.hits.iter().enumerate() {
            ui.small(format!("T{} at {:.1} s", i + 1, *hit as f32 / rate));
        }
        ui.label(format!("pauses: {}", challenge.pauses));
        if let Some(best) = challenge.best {
            ui.label(format!("best: {:.1} s", best as f32 / rate));
        }
        ui.small("R restart · N new course · C test track");
    });
    Ok(())
}
