//! "Slalom" (M2 playtest): weave through alternating gates and cross the finish line within
//! a time limit tight enough that comfortable driving fails, so the player has to push into
//! the yellow stability zone. Layout and limit come from `assets/maps/slalom.ron`.

use std::path::PathBuf;

use bevy::color::palettes::css;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use serde::Deserialize;
use sim::{CarId, Pose, SimEvent, SimState};

use crate::render::WorldView;
use crate::track::Mode;
use crate::{AppState, Sim, TuningRes};

pub struct SlalomPlugin;

impl Plugin for SlalomPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(load())
            .add_systems(OnEnter(AppState::Paused), count_pause.run_if(resource_equals(Mode::Slalom)))
            .add_systems(Update, draw.run_if(resource_equals(Mode::Slalom)))
            .add_systems(EguiPrimaryContextPass, (panel, labels).run_if(resource_equals(Mode::Slalom)));
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct SlalomSpec {
    pub time_limit: f32,
    pub miss_penalty: f32,
    pub gate_width: f32,
    pub gates: Vec<(f32, f32)>,
    pub finish_x: f32,
}

/// Where the car crossed a gate's line: inside it or not.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GatePass {
    pub gate: usize,
    pub tick: u64,
    pub ok: bool,
}

/// Follows one run (live, or over a preview) gate by gate.
#[derive(Debug, Clone, Default)]
pub struct Course {
    prev_x: Option<f32>,
    pub passes: Vec<GatePass>,
    pub finish: Option<u64>,
}

impl Course {
    /// Feeds the car's pose at `tick`. Returns the gates crossed by this move.
    pub fn update(&mut self, spec: &SlalomSpec, pose: Pose, tick: u64) -> &[GatePass] {
        let before = self.passes.len();
        let x = pose.pos.x;
        if let Some(prev) = self.prev_x
            && self.finish.is_none()
        {
            while let Some(&(gx, gy)) = spec.gates.get(self.passes.len()) {
                if !(prev < gx && x >= gx) {
                    break;
                }
                let ok = (pose.pos.y - gy).abs() <= spec.gate_width * 0.5;
                self.passes.push(GatePass { gate: self.passes.len(), tick, ok });
            }
            if self.passes.len() == spec.gates.len() && prev < spec.finish_x && x >= spec.finish_x {
                self.finish = Some(tick);
            }
        }
        self.prev_x = Some(x);
        &self.passes[before..]
    }

    pub fn missed(&self) -> usize {
        self.passes.iter().filter(|p| !p.ok).count()
    }

    /// Run time including penalties, once finished.
    pub fn result(&self, spec: &SlalomSpec, rate: f32) -> Option<f32> {
        Some(self.finish? as f32 / rate + self.missed() as f32 * spec.miss_penalty)
    }
}

#[derive(Resource)]
pub struct Slalom {
    pub spec: Option<SlalomSpec>,
    pub error: Option<String>,
    pub path: PathBuf,
    pub course: Course,
    pub losses: u32,
    pub pauses: u32,
    /// Best result (s, with penalties) of runs that finished.
    pub best: Option<f32>,
}

impl Slalom {
    pub fn update(&mut self, state: &SimState, player: CarId, events: &[SimEvent], rate: f32) {
        let Some(spec) = &self.spec else { return };
        self.losses += events.iter().filter(|e| matches!(e, SimEvent::LossOfControl { car, .. } if *car == player)).count() as u32;
        let Some(car) = state.car(player) else { return };
        let was_done = self.course.finish.is_some();
        self.course.update(spec, car.pose, state.tick);
        if !was_done && let Some(result) = self.course.result(spec, rate) {
            self.best = Some(self.best.map_or(result, |b| b.min(result)));
        }
    }

    pub fn restart(&mut self) {
        self.course = Course::default();
        self.losses = 0;
        self.pauses = 0;
    }
}

fn path() -> PathBuf {
    let cwd = PathBuf::from("assets/maps/slalom.ron");
    if cwd.exists() {
        return cwd;
    }
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/maps/slalom.ron"))
}

fn load() -> Slalom {
    let path = path();
    let result = std::fs::read_to_string(&path)
        .map_err(|e| format!("{}: {e}", path.display()))
        .and_then(|text| ron::from_str::<SlalomSpec>(&text).map_err(|e| e.to_string()));
    let (spec, error) = match result {
        Ok(spec) => (Some(spec), None),
        Err(e) => {
            error!("slalom: {e}");
            (None, Some(e))
        }
    };
    Slalom { spec, error, path, course: Course::default(), losses: 0, pauses: 0, best: None }
}

fn count_pause(sim: Res<Sim>, mut slalom: ResMut<Slalom>) {
    if sim.state.tick > 0 && slalom.course.finish.is_none() {
        slalom.pauses += 1;
    }
}

fn draw(mut gizmos: Gizmos, slalom: Res<Slalom>) {
    let Some(spec) = &slalom.spec else { return };
    let next = slalom.course.passes.len();
    let half = spec.gate_width * 0.5;
    for (i, &(gx, gy)) in spec.gates.iter().enumerate() {
        let (cone, line) = match slalom.course.passes.get(i) {
            Some(p) if p.ok => (css::GREEN.with_alpha(0.4), css::GREEN.with_alpha(0.2)),
            Some(_) => (css::RED.with_alpha(0.5), css::RED.with_alpha(0.3)),
            None if i == next => (css::ORANGE, css::GOLD.with_alpha(0.6)),
            None => (css::ORANGE.with_alpha(0.6), css::GOLD.with_alpha(0.2)),
        };
        let (a, b) = (Vec2::new(gx, gy - half), Vec2::new(gx, gy + half));
        gizmos.line_2d(a, b, line);
        for c in [a, b] {
            gizmos.circle_2d(Isometry2d::from_translation(c), 0.35, cone);
        }
    }
    let (y0, y1) = spec.gates.iter().fold((-8.0f32, 8.0f32), |(lo, hi), g| (lo.min(g.1 - 4.0), hi.max(g.1 + 4.0)));
    let finish = if slalom.course.finish.is_some() { css::GREEN } else { css::WHITE };
    gizmos.line_2d(Vec2::new(spec.finish_x, y0), Vec2::new(spec.finish_x, y1), finish);
}

fn labels(mut contexts: EguiContexts, view: WorldView, slalom: Res<Slalom>) -> Result {
    let ctx = contexts.ctx_mut()?;
    let Some(spec) = &slalom.spec else { return Ok(()) };
    let painter = ctx.layer_painter(egui::LayerId::background());
    for (i, &(gx, gy)) in spec.gates.iter().enumerate().skip(slalom.course.passes.len()) {
        let offset = if gy >= 0.0 { spec.gate_width * 0.5 + 1.5 } else { -(spec.gate_width * 0.5 + 1.5) };
        if let Some(p) = view.to_screen(Vec2::new(gx, gy + offset)) {
            painter.text(p, egui::Align2::CENTER_CENTER, format!("G{}", i + 1), egui::FontId::proportional(12.0), egui::Color32::GOLD);
        }
    }
    Ok(())
}

fn panel(mut contexts: EguiContexts, slalom: Res<Slalom>, sim: Res<Sim>, tuning: Res<TuningRes>) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("Slalom").anchor(egui::Align2::RIGHT_TOP, [-12.0, 12.0]).resizable(false).show(ctx, |ui| {
        let Some(spec) = &slalom.spec else {
            let error = slalom.error.clone().unwrap_or_default();
            ui.colored_label(egui::Color32::LIGHT_RED, format!("no slalom loaded from {}:\n{error}", slalom.path.display()));
            return;
        };
        let rate = tuning.tick_rate as f32;
        let course = &slalom.course;
        let missed = course.missed();
        let elapsed = course.finish.unwrap_or(sim.state.tick) as f32 / rate + missed as f32 * spec.miss_penalty;
        ui.label(format!("{} gates, limit {:.1} s (+{:.0} s per missed gate)", spec.gates.len(), spec.time_limit, spec.miss_penalty));
        let color = if elapsed <= spec.time_limit { egui::Color32::WHITE } else { egui::Color32::LIGHT_RED };
        ui.heading(egui::RichText::new(format!("{elapsed:.1} s")).color(color));
        ui.label(format!("gates: {}/{} passed, {missed} missed", course.passes.len() - missed, spec.gates.len()));
        match course.result(spec, rate) {
            Some(r) if r <= spec.time_limit => {
                ui.colored_label(egui::Color32::LIGHT_GREEN, format!("PASSED — {r:.2} s"));
            }
            Some(r) => {
                ui.colored_label(egui::Color32::LIGHT_RED, format!("too slow — {r:.2} s"));
            }
            None => {}
        }
        ui.label(format!("losses of control: {}   pauses: {}", slalom.losses, slalom.pauses));
        if let Some(best) = slalom.best {
            ui.label(format!("best: {best:.2} s"));
        }
        ui.small("R restart · C next mode");
    });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> SlalomSpec {
        SlalomSpec { time_limit: 10.0, miss_penalty: 2.0, gate_width: 4.0, gates: vec![(10.0, 5.0), (20.0, -5.0)], finish_x: 30.0 }
    }

    #[test]
    fn counts_gates_in_order_and_finishes() {
        let spec = spec();
        let mut course = Course::default();
        // Through gate 1 (y = 4 is inside 5 ± 2), past gate 2 on the wrong side, then finish.
        for (tick, (x, y)) in [(0.0, 0.0), (9.9, 4.0), (10.1, 4.0), (19.0, 2.0), (21.0, 2.0), (29.0, 0.0), (31.0, 0.0)].into_iter().enumerate() {
            course.update(&spec, Pose::new(Vec2::new(x, y), 0.0), tick as u64);
        }
        assert_eq!(course.passes.iter().map(|p| p.ok).collect::<Vec<_>>(), vec![true, false]);
        assert_eq!(course.finish, Some(6));
        // 6 ticks at 60 Hz plus one missed gate.
        assert!((course.result(&spec, 60.0).unwrap() - (0.1 + 2.0)).abs() < 1e-4);
    }

    #[test]
    fn finish_needs_every_gate() {
        let spec = spec();
        let mut course = Course::default();
        course.update(&spec, Pose::new(Vec2::new(25.0, 0.0), 0.0), 0);
        course.update(&spec, Pose::new(Vec2::new(31.0, 0.0), 0.0), 1);
        assert_eq!(course.finish, None, "skipped the gates by starting past them");
    }

    #[test]
    fn slalom_file_parses() {
        let text = std::fs::read_to_string(path()).unwrap();
        let spec: SlalomSpec = ron::from_str(&text).unwrap();
        assert!(spec.gates.len() >= 5 && spec.finish_x > spec.gates.last().unwrap().0);
    }
}
