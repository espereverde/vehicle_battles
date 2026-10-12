//! Plan window (while paused): turn mode and straight buttons, a timeline with the
//! grip-demand strip, keypoint spans and target hits along the preview horizon, and an
//! editor for the selected keypoint. Drag the timeline to scrub the ghost car; click a span
//! to select its keypoint.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use sim::{Turn, TurnMode};

use crate::challenge::Challenge;
use crate::zones::DangerZones;
use crate::planning::{Draft, PlanPreview};
use crate::render::{to_egui, zone_color};
use crate::{AppState, Sim, TuningRes};

pub struct TimelinePlugin;

impl Plugin for TimelinePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(EguiPrimaryContextPass, timeline.run_if(in_state(AppState::Paused)));
    }
}

const WIDTH: f32 = 720.0;
const STRIP: f32 = 12.0;
const SPANS: f32 = 20.0;
const MARKS: f32 = 16.0;
/// Stability graph (1 at the top, 0 at the bottom).
const STAB: f32 = 30.0;

#[allow(clippy::too_many_arguments)]
fn timeline(
    mut contexts: EguiContexts,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    challenge: Res<Challenge>,
    mut draft: ResMut<Draft>,
    mut plan: ResMut<PlanPreview>,
    mut next: ResMut<NextState<AppState>>,
    mut zones: ResMut<DangerZones>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    // Reborrow so disjoint fields can be borrowed separately.
    let (draft, plan) = (&mut *draft, &mut *plan);
    let rate = tuning.tick_rate as f32;
    egui::Window::new("Plan")
        .anchor(egui::Align2::CENTER_BOTTOM, [0.0, -10.0])
        .resizable(false)
        .collapsible(false)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.button("▶ Run (Space)").clicked() {
                    next.set(AppState::Running);
                }
                if ui.add_enabled(draft.edited, egui::Button::new("Revert (Esc)")).clicked() {
                    draft.reset(&sim.player_car().orders);
                }
                if ui.add_enabled(!draft.orders.segments.is_empty(), egui::Button::new("Clear")).clicked() {
                    draft.clear();
                }
                ui.checkbox(&mut zones.enabled, "Danger zones (Z)")
                    .on_hover_text("Shade where the selected (or next) keypoint can go, in the grip color its path would get");
                ui.checkbox(&mut zones.drift, "Drift (X)")
                    .on_hover_text("Where the path is orange/red: how far out the real car could run wide or slide");
                ui.separator();
                ui.label("Turn (T):");
                let shown = draft.shown_mode();
                for (mode, label) in [(TurnMode::Steer, "Steering"), (TurnMode::Lateral, "G-force"), (TurnMode::Swerve, "Swerve")] {
                    if ui.selectable_label(shown == mode, label).clicked() && shown != mode {
                        let keypoint = draft.selected.and_then(|k| plan.keypoint(k)).map(|f| f.pose.pos);
                        draft.set_mode(mode, &sim, &tuning, keypoint);
                    }
                }
                ui.separator();
                let straight = if draft.selected.is_some() { "Straight (S)" } else { "+ Straight (S)" };
                if ui.button(straight).on_hover_text("Drive straight, holding the current speed").clicked() {
                    draft.straight(&sim, &tuning);
                }
            });

            let Some(preview) = &plan.preview else { return };
            let n = preview.ticks().max(1);
            let (rect, response) = ui.allocate_exact_size(egui::vec2(WIDTH, STRIP + SPANS + STAB + MARKS + 8.0), egui::Sense::click_and_drag());
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 3.0, egui::Color32::from_gray(25));
            let x = |i: usize| rect.left() + rect.width() * i.min(n) as f32 / n as f32;
            let tick_at = |px: f32| (((px - rect.left()) / rect.width()) * n as f32).round().clamp(0.0, n as f32) as usize;

            // Grip-demand strip, one run of equal color at a time.
            let frame = |i: usize| preview.frame(sim.player, i);
            let color = |i: usize| frame(i).map_or(egui::Color32::BLACK, |f| to_egui(zone_color(f.grip_demand, f.at_risk(&tuning), &tuning.vehicle)));
            let strip_y = rect.top() + 2.0..=rect.top() + 2.0 + STRIP;
            let mut run_start = 0;
            for i in 1..=n {
                if i == n || color(i + 1) != color(run_start + 1) {
                    let r = egui::Rect::from_x_y_ranges(x(run_start)..=x(i), strip_y.clone());
                    painter.rect_filled(r, 0.0, color(run_start + 1));
                    run_start = i;
                }
            }

            // Keypoint spans, then "continue" (last controls kept) up to the horizon.
            let span_y = rect.top() + 4.0 + STRIP..=rect.top() + 4.0 + STRIP + SPANS;
            let mut spans = Vec::new();
            let mut start = 0;
            for (k, &end) in plan.ends.iter().enumerate() {
                if start >= n {
                    break;
                }
                spans.push((start, end.min(n), Some(k)));
                start = end;
            }
            if start < n {
                spans.push((start, n, None));
            }
            for &(a, b, k) in &spans {
                let r = egui::Rect::from_x_y_ranges(x(a) + 1.0..=x(b) - 1.0, span_y.clone());
                let (fill, label) = match k {
                    Some(k) if draft.selected == Some(k) => (egui::Color32::from_rgb(0, 110, 120), format!("#{}", k + 1)),
                    Some(k) => (egui::Color32::from_gray(if k % 2 == 0 { 60 } else { 75 }), format!("#{}", k + 1)),
                    None => (egui::Color32::from_gray(38), "continue".into()),
                };
                painter.rect_filled(r, 2.0, fill);
                if r.width() > 24.0 {
                    painter.text(r.center(), egui::Align2::CENTER_CENTER, label, egui::FontId::proportional(12.0), egui::Color32::WHITE);
                }
            }

            // Stability graph: the gauge over time, the yellow-zone line, and risk stretches as
            // red bands labelled with their chance.
            let stab_top = rect.top() + 6.0 + STRIP + SPANS;
            let stab_y = |s: f32| stab_top + STAB * (1.0 - s.clamp(0.0, 1.0));
            let st = &tuning.stability;
            let yellow = stab_y(st.yellow);
            painter.line_segment([egui::pos2(rect.left(), yellow), egui::pos2(rect.right(), yellow)], (1.0, egui::Color32::from_rgba_unmultiplied(255, 220, 0, 60)));
            let red = egui::Color32::from_rgb(255, 70, 70);
            for r in &plan.risks {
                let band = egui::Rect::from_x_y_ranges(x(r.start) - 1.0..=x(r.end) + 1.0, stab_top..=stab_top + STAB);
                painter.rect_filled(band, 0.0, red.gamma_multiply(if r.certain_at.is_some() { 0.45 } else { 0.1 + 0.4 * r.chance }));
                let label = if r.certain_at.is_some() { "LOSS".to_string() } else { format!("{:.0}%", r.chance * 100.0) };
                painter.text(egui::pos2(x(r.at()) + 2.0, stab_top + 1.0), egui::Align2::LEFT_TOP, label, egui::FontId::proportional(10.0), egui::Color32::WHITE);
            }
            for i in 1..=n {
                let (Some(a), Some(b)) = (frame(i - 1), frame(i)) else { break };
                let c = if b.stability >= st.yellow { egui::Color32::LIGHT_GREEN } else { egui::Color32::YELLOW };
                painter.line_segment([egui::pos2(x(i - 1), stab_y(a.stability)), egui::pos2(x(i), stab_y(b.stability))], (1.5, c));
            }

            // Seconds and target hits.
            let marks_y = stab_top + STAB + 2.0;
            for s in 1..(n as f32 / rate).ceil() as usize {
                let px = x((s as f32 * rate) as usize);
                painter.line_segment([egui::pos2(px, rect.top()), egui::pos2(px, marks_y)], (1.0, egui::Color32::from_white_alpha(40)));
                painter.text(egui::pos2(px, marks_y + 1.0), egui::Align2::CENTER_TOP, format!("{s}s"), egui::FontId::proportional(10.0), egui::Color32::GRAY);
            }
            let magenta = egui::Color32::from_rgb(255, 90, 200);
            for &tick in &plan.off_track {
                let px = x(tick);
                painter.line_segment([egui::pos2(px, rect.top()), egui::pos2(px, marks_y)], (2.0, magenta));
                painter.text(egui::pos2(px + 3.0, marks_y + 1.0), egui::Align2::LEFT_TOP, "OFF", egui::FontId::proportional(11.0), magenta);
            }
            for &(target, tick) in &plan.hits {
                let p = egui::pos2(x(tick), marks_y + 1.0);
                painter.text(p, egui::Align2::CENTER_TOP, format!("T{}", target + 1), egui::FontId::proportional(11.0), egui::Color32::LIGHT_GREEN);
            }
            for &(gate, tick, ok) in &plan.gates {
                let (label, color) = if ok { (format!("G{}", gate + 1), egui::Color32::LIGHT_GREEN) } else { (format!("G{}✗", gate + 1), red) };
                painter.text(egui::pos2(x(tick), marks_y + 1.0), egui::Align2::CENTER_TOP, label, egui::FontId::proportional(11.0), color);
            }
            if let Some((tick, result)) = plan.finish {
                let p = egui::pos2(x(tick), marks_y + 1.0);
                painter.text(p, egui::Align2::CENTER_TOP, format!("FIN {result:.1}s"), egui::FontId::proportional(11.0), egui::Color32::WHITE);
            }

            // Scrubbing and span selection.
            if let Some(pos) = response.interact_pointer_pos() {
                plan.scrub = tick_at(pos.x);
                if response.clicked() && span_y.contains(&pos.y) {
                    let hit = spans.iter().find(|(a, b, _)| (*a..=*b).contains(&plan.scrub));
                    draft.selected = hit.and_then(|(_, _, k)| *k);
                }
            }
            let px = x(plan.scrub);
            painter.line_segment([egui::pos2(px, rect.top()), egui::pos2(px, rect.bottom())], (2.0, egui::Color32::YELLOW));

            if let Some(f) = frame(plan.scrub) {
                let traction = if f.at_risk(&tuning) {
                    "may LOSE CONTROL (not simulated)"
                } else if f.grip_demand > 1.0 {
                    "would understeer (not simulated)"
                } else {
                    ""
                };
                ui.monospace(format!(
                    "t = {:4.2} s   speed {:5.1} km/h   accel {:+6.1} km/h/s   grip {:3.0}%   stability {:3.0}%  {traction}",
                    plan.scrub as f32 / rate,
                    crate::ui::kmh(f.vel.length()),
                    crate::ui::kmh(f.accel.dot(f.vel.normalize_or_zero())),
                    f.grip_demand * 100.0,
                    f.stability * 100.0
                ));
                for r in &plan.risks {
                    ui.colored_label(red, PlanPreview::risk_label(r, rate));
                }
            }
            let beyond = plan.ends.iter().filter(|&&e| e > n).count();
            if beyond > 0 {
                let why = if (n as u32) < tuning.preview_ticks() { ", cut where the path closes a loop" } else { "" };
                ui.small(format!("{beyond} keypoint(s) end beyond the {:.1} s preview{why}", n as f32 / rate));
            }
            if challenge.done() {
                ui.small("Course complete — R to retry, N for a new course.");
            }
            keypoint_editor(ui, draft, &tuning);
        });

    Ok(())
}

/// Exact values for the selected keypoint's segment.
fn keypoint_editor(ui: &mut egui::Ui, draft: &mut Draft, tuning: &sim::Tuning) {
    let Some(k) = draft.selected.filter(|&k| k < draft.orders.segments.len()) else { return };
    let rate = tuning.tick_rate as f32;
    let (mut changed, mut delete) = (false, false);
    let max_secs = draft.max_ticks(k, tuning) as f32 / rate;
    let mut new_ticks = None;
    ui.separator();
    ui.horizontal(|ui| {
        let seg = &mut draft.orders.segments[k];
        ui.strong(format!("Keypoint #{}", k + 1));

        let mut throttle = seg.drive.throttle * 100.0;
        ui.label("throttle");
        if ui.add(egui::DragValue::new(&mut throttle).speed(1.0).range(-100.0..=100.0).fixed_decimals(0).suffix(" %")).changed() {
            seg.drive.throttle = throttle / 100.0;
            changed = true;
        }

        match &mut seg.drive.turn {
            Turn::Steer(s) => {
                let mut pct = *s * 100.0;
                ui.label("steer");
                if ui.add(egui::DragValue::new(&mut pct).speed(0.5).range(-100.0..=100.0).fixed_decimals(1).suffix(" %")).changed() {
                    *s = pct / 100.0;
                    changed = true;
                }
            }
            Turn::Lateral(g) => {
                ui.label("lateral");
                let max = tuning.planner.max_lateral_g;
                changed |= ui.add(egui::DragValue::new(g).speed(0.01).range(-max..=max).fixed_decimals(2).suffix(" g")).changed();
            }
            Turn::Swerve(rate) => {
                let mut deg = rate.to_degrees();
                ui.label("turn rate");
                if ui.add(egui::DragValue::new(&mut deg).speed(0.5).range(-180.0..=180.0).fixed_decimals(1).suffix(" °/s")).changed() {
                    *rate = deg.to_radians();
                    changed = true;
                }
            }
        }

        // Later keypoints keep their times: the duration is bounded by the next keypoint.
        let mut secs = seg.ticks as f32 / rate;
        let min = (seg.elapsed + 1) as f32 / rate;
        ui.label("duration");
        if ui.add(egui::DragValue::new(&mut secs).speed(0.02).range(min..=max_secs).fixed_decimals(2).suffix(" s")).changed() {
            new_ticks = Some((secs * rate).round() as u32);
        }
        delete = ui.button("Delete (Del)").clicked();
    });
    if delete {
        draft.delete(k);
        return;
    }
    if changed {
        draft.changed();
    }
    if let Some(ticks) = new_ticks {
        draft.set_ticks(k, ticks, tuning);
    }
}
