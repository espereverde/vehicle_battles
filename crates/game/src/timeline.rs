//! Timeline bar (while paused): grip-demand strip, segment spans and target hits along the
//! preview horizon. Drag to scrub the ghost car; click a span to select its segment.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};

use crate::challenge::Challenge;
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

fn timeline(
    mut contexts: EguiContexts,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    challenge: Res<Challenge>,
    mut draft: ResMut<Draft>,
    mut plan: ResMut<PlanPreview>,
    mut next: ResMut<NextState<AppState>>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    // Reborrow so disjoint fields can be borrowed separately.
    let (draft, plan) = (&mut *draft, &mut *plan);
    let car = sim.player_car();
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
                    draft.reset(&car.orders);
                }
                if ui.add_enabled(!draft.orders.segments.is_empty(), egui::Button::new("Clear")).clicked() {
                    draft.clear(car, &tuning.planner);
                }
                ui.label(format!("{} segment(s){}", draft.orders.segments.len(), if draft.edited { " — edited" } else { "" }));
            });

            let Some(preview) = &plan.preview else { return };
            let n = preview.ticks().max(1);
            let (rect, response) = ui.allocate_exact_size(egui::vec2(WIDTH, STRIP + SPANS + MARKS + 6.0), egui::Sense::click_and_drag());
            let painter = ui.painter_at(rect);
            painter.rect_filled(rect, 3.0, egui::Color32::from_gray(25));
            let x = |i: usize| rect.left() + rect.width() * i as f32 / n as f32;
            let tick_at = |px: f32| (((px - rect.left()) / rect.width()) * n as f32).round().clamp(0.0, n as f32) as usize;

            // Grip-demand strip, one run of equal color at a time.
            let frame = |i: usize| preview.frame(sim.player, i);
            let color = |i: usize| frame(i).map_or(egui::Color32::BLACK, |f| to_egui(zone_color(f.grip_demand, f.slide.is_some(), &tuning.vehicle)));
            let strip_y = rect.top() + 2.0..=rect.top() + 2.0 + STRIP;
            let mut run_start = 0;
            for i in 1..=n {
                if i == n || color(i + 1) != color(run_start + 1) {
                    let r = egui::Rect::from_x_y_ranges(x(run_start)..=x(i), strip_y.clone());
                    painter.rect_filled(r, 0.0, color(run_start + 1));
                    run_start = i;
                }
            }

            // Segment spans: completed segments, then the one still running at the horizon,
            // then "hold" once all orders are done.
            let span_y = rect.top() + 4.0 + STRIP..=rect.top() + 4.0 + STRIP + SPANS;
            let mut spans = Vec::new();
            let mut start = 0;
            for (k, &end) in plan.segment_ends.iter().enumerate() {
                spans.push((start, end, Some(k)));
                start = end;
            }
            if start < n {
                let k = plan.segment_ends.len();
                spans.push((start, n, (k < draft.orders.segments.len()).then_some(k)));
            }
            for &(a, b, k) in &spans {
                let r = egui::Rect::from_x_y_ranges(x(a) + 1.0..=x(b) - 1.0, span_y.clone());
                let (fill, label) = match k {
                    Some(k) if draft.selected == Some(k) => (egui::Color32::from_rgb(0, 110, 120), format!("#{}", k + 1)),
                    Some(k) => (egui::Color32::from_gray(if k % 2 == 0 { 60 } else { 75 }), format!("#{}", k + 1)),
                    None => (egui::Color32::from_gray(38), "hold".into()),
                };
                painter.rect_filled(r, 2.0, fill);
                if r.width() > 24.0 {
                    painter.text(r.center(), egui::Align2::CENTER_CENTER, label, egui::FontId::proportional(12.0), egui::Color32::WHITE);
                }
            }

            // Seconds and target hits.
            let marks_y = rect.top() + 6.0 + STRIP + SPANS;
            for s in 1..(n as f32 / rate).ceil() as usize {
                let px = x((s as f32 * rate) as usize);
                painter.line_segment([egui::pos2(px, rect.top()), egui::pos2(px, marks_y)], (1.0, egui::Color32::from_white_alpha(40)));
                painter.text(egui::pos2(px, marks_y + 1.0), egui::Align2::CENTER_TOP, format!("{s}s"), egui::FontId::proportional(10.0), egui::Color32::GRAY);
            }
            for &(target, tick) in &plan.hits {
                let p = egui::pos2(x(tick), marks_y + 1.0);
                painter.text(p, egui::Align2::CENTER_TOP, format!("T{}", target + 1), egui::FontId::proportional(11.0), egui::Color32::LIGHT_GREEN);
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
                let traction = match f.slide {
                    Some(_) => "SLIDE",
                    None if f.grip_demand > 1.0 => "understeer",
                    None => "",
                };
                ui.monospace(format!(
                    "t = {:4.2} s   {:5.1} m/s   grip {:3.0}%  {traction}",
                    plan.scrub as f32 / rate,
                    f.vel.length(),
                    f.grip_demand * 100.0
                ));
            }
            if challenge.done() {
                ui.small("Course complete — R to retry, N for a new course.");
            }
            segment_editor(ui, draft, car, &tuning);
        });

    Ok(())
}

/// Exact values for the selected segment (scroll over its ghost for quick changes).
fn segment_editor(ui: &mut egui::Ui, draft: &mut Draft, car: &sim::Car, tuning: &sim::Tuning) {
    let Some(k) = draft.selected.filter(|&k| k < draft.orders.segments.len()) else { return };
    let top = tuning.vehicle.top_speed.floor();
    let (mut changed, mut delete) = (false, false);
    ui.separator();
    ui.horizontal(|ui| {
        let seg = &mut draft.orders.segments[k];
        let (length, start) = (seg.length(), seg.speed().start);
        let speed = seg.speed_mut();
        ui.strong(format!("Segment #{}", k + 1));
        ui.label("end");
        changed |= ui.add(egui::DragValue::new(&mut speed.end).speed(0.2).range(0.0..=top).suffix(" m/s")).changed();
        ui.label("cruise");
        changed |= ui.add(egui::DragValue::new(&mut speed.cruise).speed(0.2).range(1.0..=top).suffix(" m/s")).changed();
        ui.small(format!("{length:.1} m from {start:.1} m/s"));
        delete = ui.button("Delete (Del)").clicked();
    });
    if delete {
        draft.delete(k, car, &tuning.planner);
    } else if changed {
        draft.changed(car, &tuning.planner);
    }
}
