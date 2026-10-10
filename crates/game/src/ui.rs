//! egui inspector: live car state, tuning / replay status, preview-vs-reality check.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use sim::{DriveMode, SimEvent};

use crate::planning::{CheckResult, RealityCheck};
use crate::replay::Recorder;
use crate::tuning_file::TuningFile;
use crate::{AppState, EventLog, Sim, TuningRes};

pub struct UiPlugin;

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
            row("speed", format!("{speed:6.2} m/s  {:5.1} km/h", speed * 3.6));
            row("v long / lat", format!("{:6.2} / {:6.2} m/s", car.forward_speed(), car.lateral_speed()));
            row("slip angle", format!("{:6.1}°", car.slip_angle().to_degrees()));
            row("yaw rate", format!("{:6.1}°/s", car.yaw_rate.to_degrees()));
            row("steer angle", format!("{:6.1}°", car.steer_angle.to_degrees()));
            row("accel", format!("{:6.2} m/s² ({:.2} g)", car.accel.length(), car.accel.length() / tuning.vehicle.gravity));
            row("heading", format!("{:6.1}°", car.pose.heading.to_degrees()));
            row("position", format!("{:7.1}, {:7.1}", car.pose.pos.x, car.pose.pos.y));
            row(
                "traction",
                match car.slide {
                    Some(s) => format!("SLIDING {:.2} s", s.time),
                    None if car.grip_demand > 1.0 => "understeer".into(),
                    None => "—".into(),
                },
            );
            let c = car.controls;
            row(
                "controls",
                format!("steer {:+.2} thr {:+.2} brk {:.1}{}", c.steer, c.throttle, c.brake, if c.handbrake { " HB" } else { "" }),
            );
            let orders = match car.orders.segments.first() {
                Some(seg) => format!("{} seg, {:.0}/{:.0} m", car.orders.segments.len(), seg.progress, seg.length()),
                None => car.orders.hold.map_or("hold (current)".into(), |h| format!("hold {:.0} m/s", h.speed)),
            };
            row("orders", orders);
        });
        ui.add(
            egui::ProgressBar::new((car.grip_demand / 1.5).min(1.0))
                .text(format!("grip demand {:.0}%", car.grip_demand * 100.0))
                .fill(crate::render::to_egui(crate::render::zone_color(car.grip_demand, car.slide.is_some(), &tuning.vehicle)).gamma_multiply(0.7)),
        );

        ui.separator();
        egui::CollapsingHeader::new("Events").default_open(false).show(ui, |ui| {
            for e in log.0.iter().rev() {
                ui.monospace(match e {
                    SimEvent::SlideStarted { tick, speed, .. } => format!("{tick:>6}  slide start @ {speed:.1} m/s"),
                    SimEvent::SlideEnded { tick, duration, .. } => format!("{tick:>6}  slide end after {duration:.2} s"),
                    SimEvent::SegmentCompleted { tick, remaining, .. } => format!("{tick:>6}  segment done, {remaining} left"),
                });
            }
        });
        egui::CollapsingHeader::new("Controls").default_open(true).show(ui, |ui| {
            ui.label("Space       run / pause (commits the plan)");
            ui.label("LMB ground  add segment (drag to place)");
            ui.label("LMB ghost   move end · knob: end heading");
            ui.label("Wheel ghost end speed (Shift: cruise)");
            ui.label("RMB ghost / Del  delete · Esc revert");
            ui.label(".           single tick");
            ui.label("Wheel / MMB zoom / pan · F follow car");
            ui.label("R restart · N new course · M manual drive");
            ui.label("F5 save replay · F9 verify replay");
        });
    });
    Ok(())
}
