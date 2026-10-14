//! egui inspector: live car state, tuning / replay status, preview-vs-reality check.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use sim::{DriveMode, SimEvent};

use crate::planning::{CheckResult, RealityCheck};
use crate::replay::Recorder;
use crate::tuning_file::TuningFile;
use crate::{AppState, EventLog, Sim, TuningRes};

pub struct UiPlugin;

/// m/s (or m/s²) → km/h (or km/h per second). The sim works in SI units; the UI shows km/h.
pub fn kmh(v: f32) -> f32 {
    v * 3.6
}

impl Plugin for UiPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(EguiPrimaryContextPass, inspector);
    }
}

#[allow(clippy::too_many_arguments)]
fn inspector(
    mut contexts: EguiContexts,
    app_state: Res<State<AppState>>,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    file: Res<TuningFile>,
    check: Res<RealityCheck>,
    recorder: Res<Recorder>,
    log: Res<EventLog>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    egui::Window::new("Inspector").default_pos([12.0, 12.0]).show(ctx, |ui| {
        let paused = *app_state.get() == AppState::Paused;
        let car = sim.player_car();
        ui.label(format!(
            "{}  tick {}  t = {:.2} s  {}",
            if paused { "PAUSED" } else { "running" },
            sim.state.tick,
            sim.state.time(&tuning),
            if car.drive_mode == DriveMode::Direct { "MANUAL" } else { "" }
        ));

        let (text, color) = match check.result {
            CheckResult::Idle => ("preview check: idle".to_string(), egui::Color32::GRAY),
            CheckResult::Running => (format!("preview check: {}/{} ticks match", check.matched, check.total), egui::Color32::LIGHT_GRAY),
            CheckResult::Passed => (format!("preview check: OK, {}/{} ticks identical", check.matched, check.total), egui::Color32::LIGHT_GREEN),
            CheckResult::Diverged { tick, error } => (format!("preview check: FAILED, diverged at tick {tick} ({error:.3} m)"), egui::Color32::LIGHT_RED),
        };
        ui.colored_label(color, text);

        ui.separator();
        match &file.status {
            Ok(()) => ui.label(format!("tuning: {} (reloads: {})", file.path.display(), file.reloads)),
            Err(e) => ui.colored_label(egui::Color32::LIGHT_RED, format!("tuning error: {e}")),
        };
        if !recorder.status.is_empty() {
            ui.label(&recorder.status);
        }

        ui.separator();
        egui::Grid::new("car").num_columns(2).striped(true).show(ui, |ui| {
            let mut row = |k: &str, v: String| {
                ui.label(k);
                ui.monospace(v);
                ui.end_row();
            };
            let speed = car.vel.length();
            row("speed", format!("{:6.1} km/h", kmh(speed)));
            row("yaw rate", format!("{:6.1}°/s", car.yaw_rate.to_degrees()));
            row("steer angle", format!("{:6.1}°", car.steer_angle.to_degrees()));
            let a = car.accel;
            let along = a.dot(car.pose.forward());
            let side = a.dot(car.pose.left());
            row("accel long / lat", format!("{:+6.1} / {:+6.1} km/h/s", kmh(along), kmh(side)));
            row("accel total", format!("{:6.1} km/h/s ({:.2} g)", kmh(a.length()), a.length() / tuning.vehicle.gravity));
            row("heading", format!("{:6.1}°", car.pose.heading.to_degrees()));
            row("position", format!("{:7.1}, {:7.1}", car.pose.pos.x, car.pose.pos.y));
            let c = car.controls;
            row(
                "controls",
                format!("steer {:+.2} thr {:+.2} brk {:.1}{}", c.steer, c.throttle, c.brake, if c.handbrake { " HB" } else { "" }),
            );
            let orders = match car.orders.segments.first() {
                Some(seg) => format!("{} kp, {}/{} ticks", car.orders.segments.len(), seg.elapsed, seg.ticks),
                None => "continue".into(),
            };
            row("orders", format!("{orders}  {}", crate::planning::describe(&car.orders.active_drive())));
            if let Some(s) = car.slide {
                let left = if s.distance.is_finite() { format!("{:.1} m left", s.distance) } else { "until stopped".into() };
                row("slide", format!("{:?}{} — {left}", s.kind, if s.burning { " (BURNING)" } else { "" }));
            }
        });
        let hs = sim::handling::status(car, &tuning);
        let (lo, hi) = (tuning.handling.min_status, sim::handling::class(&tuning));
        let zone = sim::CarFrame::of(car).zone(&tuning);
        let note = match zone {
            0 => "safe".to_string(),
            1 => format!("maneuvering is risky: {:.0}% per point", sim::handling::p_fail(hs, car.vel.length(), &tuning) * 100.0),
            _ if car.slide.is_some() => "sliding".to_string(),
            _ => format!("{:.1}% this tick", car.risk * 100.0),
        };
        ui.add(
            egui::ProgressBar::new(((hs - lo) / (hi - lo).max(1e-3)).clamp(0.0, 1.0))
                .text(format!("handling {hs:+.1} / {hi:+.0} — {note}"))
                .fill(crate::render::to_egui(crate::render::zone_color(zone)).gamma_multiply(0.7)),
        );

        ui.separator();
        egui::CollapsingHeader::new("Events").default_open(false).show(ui, |ui| {
            for e in log.0.iter().rev() {
                ui.monospace(match e {
                    SimEvent::SegmentCompleted { tick, remaining, .. } => format!("{tick:>6}  segment done, {remaining} left"),
                    SimEvent::Risk { tick, chance, .. } => format!("{tick:>6}  risk check {:.0}%", chance * 100.0),
                    SimEvent::LossOfControl { tick, kind, speed, .. } => {
                        format!("{tick:>6}  LOST CONTROL: {kind:?} @ {:.0} km/h", kmh(*speed))
                    }
                    SimEvent::SlideEnded { tick, .. } => format!("{tick:>6}  slide ended"),
                });
            }
        });
        egui::CollapsingHeader::new("Controls").default_open(false).show(ui, |ui| {
            ui.label("Space       run / pause (commits the plan)");
            ui.label("LMB ground  add keypoint (drag to place)");
            ui.label("LMB ghost   drag: further = later, sideways = turn");
            ui.label("Wheel       throttle ±5% (dragged/hovered/selected)");
            ui.label("T turn mode: steering / drift / skid");
            ui.label("S straight at current speed");
            ui.label("RMB ghost / Del  delete · Esc revert");
            ui.label(".           single tick");
            ui.label("Ctrl+Wheel / MMB zoom / pan · F follow car");
            ui.label("C track / parking · R restart · N new course");
            ui.label("M manual drive");
            ui.label("F5 save replay · F9 verify replay");
        });
    });
    Ok(())
}
