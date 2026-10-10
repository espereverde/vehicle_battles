//! Planning while paused. The drag tool edits a *draft* of the player's orders; the preview
//! forks the simulation and runs the same `step()` ahead; resuming commits the draft as a
//! `Command::SetOrders` and starts a preview-vs-reality check.
//!
//! Mouse: LMB on empty ground appends a segment ending there (drag to place it), drag a
//! ghost car to move a segment end, drag its knob to set the end heading, scroll over a
//! ghost to change its end speed (Shift: cruise speed), RMB on a ghost deletes it.
//! Keys: Del deletes the selected segment, Esc reverts to the live orders, `.` steps a tick.

use bevy::color::palettes::css;
use bevy::input::mouse::AccumulatedMouseScroll;
use bevy::prelude::*;
use bevy_egui::input::{egui_wants_any_keyboard_input, egui_wants_any_pointer_input};
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use sim::path::arc_heading;
use sim::preview::Check;
use sim::{Car, Command, DriveMode, OrderQueue, PlannerTuning, Pose, Preview, Segment, SpeedProfile, Tuning, VehicleTuning};

use crate::challenge::Challenge;
use crate::render::{self, ScrollUsed, WorldView, car_outline, zone_color};
use crate::{AppState, Sim, Stepper, TuningRes};

pub struct PlanningPlugin;

impl Plugin for PlanningPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Draft>()
            .init_resource::<PlanPreview>()
            .init_resource::<RealityCheck>()
            .init_resource::<Drag>()
            .add_systems(OnEnter(AppState::Paused), sync_draft)
            .add_systems(OnExit(AppState::Paused), commit_on_resume)
            .add_systems(
                Update,
                (
                    (
                        edit_mouse.run_if(not(egui_wants_any_pointer_input)).before(render::zoom_camera),
                        edit_keys.run_if(not(egui_wants_any_keyboard_input)),
                        rebuild_preview,
                    )
                        .chain()
                        .run_if(in_state(AppState::Paused)),
                    draw_plan,
                ),
            )
            .add_systems(EguiPrimaryContextPass, plan_labels.run_if(in_state(AppState::Paused)));
    }
}

/// The player's orders as being edited while paused.
#[derive(Resource, Default)]
pub struct Draft {
    pub orders: OrderQueue,
    /// Per segment: the end heading follows the drag (circular arc) until set explicitly.
    auto_heading: Vec<bool>,
    pub selected: Option<usize>,
    /// The draft differs from the live orders.
    pub edited: bool,
    /// The preview is out of date.
    dirty: bool,
}

impl Draft {
    /// Discards edits and starts over from the live orders.
    pub fn reset(&mut self, live: &OrderQueue) {
        *self = Self {
            orders: live.clone(),
            auto_heading: vec![false; live.segments.len()],
            selected: None,
            edited: false,
            dirty: true,
        };
    }

    /// Catches up with the live orders after ticks ran (completed segments drop off the
    /// front), keeping per-segment state and selection.
    fn follow(&mut self, live: &OrderQueue) {
        let done = self.orders.segments.len().saturating_sub(live.segments.len()).min(self.auto_heading.len());
        self.auto_heading.drain(..done);
        self.auto_heading.resize(live.segments.len(), false);
        self.selected = self.selected.and_then(|i| i.checked_sub(done)).filter(|&i| i < live.segments.len());
        self.orders = live.clone();
        self.edited = false;
        self.dirty = true;
    }

    pub fn changed(&mut self, car: &Car, pt: &PlannerTuning) {
        self.orders.rechain(car.pose, car.forward_speed().max(0.0), pt);
        self.edited = true;
        self.dirty = true;
    }

    fn push(&mut self, car: &Car, p: Vec2, tuning: &Tuning) {
        let (start, v0) = self.orders.tail(car.pose, car.forward_speed().max(0.0));
        let end = Pose::new(p, arc_heading(start, p));
        let speed = SpeedProfile {
            start: v0,
            cruise: v0.max(tuning.planner.default_cruise).round(),
            end: if v0 < 1.0 { 0.0 } else { v0.round() },
        };
        self.orders.segments.push(Segment::path(start, end, speed, &tuning.planner));
        self.auto_heading.push(true);
        self.selected = Some(self.orders.segments.len() - 1);
        self.changed(car, &tuning.planner);
    }

    fn move_end(&mut self, i: usize, p: Vec2, car: &Car, pt: &PlannerTuning) {
        let seg = &mut self.orders.segments[i];
        let heading = if self.auto_heading[i] { arc_heading(seg.start_pose(), p) } else { seg.end_pose().heading };
        let end = Pose::new(p, heading);
        if end != seg.end_pose() {
            seg.set_end(end);
            self.changed(car, pt);
        }
    }

    fn aim(&mut self, i: usize, p: Vec2, car: &Car, pt: &PlannerTuning) {
        let seg = &mut self.orders.segments[i];
        let end = seg.end_pose();
        let d = p - end.pos;
        if d.length() < 0.5 || d.to_angle() == end.heading {
            return;
        }
        seg.set_end(Pose::new(end.pos, d.to_angle()));
        self.auto_heading[i] = false;
        self.changed(car, pt);
    }

    pub fn delete(&mut self, i: usize, car: &Car, pt: &PlannerTuning) {
        self.orders.segments.remove(i);
        self.auto_heading.remove(i);
        self.selected = match self.selected {
            Some(s) if s == i => None,
            Some(s) if s > i => Some(s - 1),
            s => s,
        };
        self.changed(car, pt);
    }

    pub fn clear(&mut self, car: &Car, pt: &PlannerTuning) {
        self.orders = OrderQueue::default();
        self.auto_heading.clear();
        self.selected = None;
        self.changed(car, pt);
    }

    fn adjust_speed(&mut self, i: usize, step: f32, cruise: bool, car: &Car, tuning: &Tuning) {
        let top = tuning.vehicle.top_speed.floor();
        let speed = self.orders.segments[i].speed_mut();
        if cruise {
            speed.cruise = (speed.cruise + step).round().clamp(1.0, top);
        } else {
            speed.end = (speed.end + step).round().clamp(0.0, top);
        }
        self.changed(car, &tuning.planner);
    }
}

/// Preview of the draft, rebuilt whenever the draft or tuning changes.
#[derive(Resource, Default)]
pub struct PlanPreview {
    pub preview: Option<Preview>,
    /// Scrub position, in ticks into the preview.
    pub scrub: usize,
    /// Preview tick at which each draft segment is completed (within the horizon).
    pub segment_ends: Vec<usize>,
    /// Challenge targets the preview reaches: (target index, preview tick).
    pub hits: Vec<(usize, usize)>,
}

impl PlanPreview {
    pub fn player_frame(&self, i: usize) -> Option<&sim::CarFrame> {
        self.preview.as_ref()?.frames.get(i)?.first()
    }
}

/// Compares live execution with the preview recorded when the plan was committed.
#[derive(Resource, Default)]
pub struct RealityCheck {
    preview: Option<Preview>,
    pub matched: usize,
    pub total: usize,
    pub result: CheckResult,
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum CheckResult {
    #[default]
    Idle,
    Running,
    Passed,
    Diverged { tick: u64, error: f32 },
}

impl RealityCheck {
    pub fn start(&mut self, preview: Preview) {
        self.total = preview.ticks();
        self.matched = 0;
        self.result = CheckResult::Running;
        self.preview = Some(preview);
    }

    pub fn stop(&mut self) {
        self.preview = None;
        self.result = CheckResult::Idle;
    }

    pub fn observe(&mut self, state: &sim::SimState) {
        let Some(preview) = &self.preview else { return };
        match preview.check(state) {
            Check::Match => self.matched += 1,
            Check::OutOfRange => {
                self.result = CheckResult::Passed;
                self.preview = None;
            }
            Check::Diverged { position_error, .. } => {
                warn!("live execution diverged from the preview at tick {} ({position_error} m)", state.tick);
                self.result = CheckResult::Diverged { tick: state.tick, error: position_error };
                self.preview = None;
            }
        }
    }
}

#[derive(Resource, Default)]
enum Drag {
    #[default]
    None,
    End(usize),
    Heading(usize),
}

enum Handle {
    End,
    Heading,
}

/// Where the heading knob of a segment end is drawn: in front of the ghost car, at least
/// a comfortable click distance away from it on screen.
fn knob_pos(end: Pose, m_per_px: f32, vt: &VehicleTuning) -> Vec2 {
    end.pos + end.forward() * (vt.length * 0.5 + (25.0 * m_per_px).max(2.5))
}

fn handle_at(draft: &Draft, p: Vec2, m_per_px: f32, vt: &VehicleTuning) -> Option<(usize, Handle)> {
    let n = draft.orders.segments.len();
    let pick = (10.0 * m_per_px).max(0.8);
    let order = draft.selected.filter(|&i| i < n).into_iter().chain((0..n).rev());
    for i in order {
        let end = draft.orders.segments[i].end_pose();
        if p.distance(knob_pos(end, m_per_px, vt)) < pick {
            return Some((i, Handle::Heading));
        }
        if p.distance(end.pos) < pick.max(vt.length * 0.5) {
            return Some((i, Handle::End));
        }
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn edit_mouse(
    buttons: Res<ButtonInput<MouseButton>>,
    keys: Res<ButtonInput<KeyCode>>,
    scroll: Res<AccumulatedMouseScroll>,
    view: WorldView,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    mut draft: ResMut<Draft>,
    mut drag: ResMut<Drag>,
    mut scroll_used: ResMut<ScrollUsed>,
) {
    let Some(cursor) = view.cursor() else { return };
    let car = sim.player_car();
    let (vt, pt) = (&tuning.vehicle, &tuning.planner);
    let m_per_px = view.meters_per_pixel();
    let hit = handle_at(&draft, cursor, m_per_px, vt);

    if buttons.just_pressed(MouseButton::Left) {
        *drag = match hit {
            Some((i, Handle::End)) => Drag::End(i),
            Some((i, Handle::Heading)) => Drag::Heading(i),
            None => {
                draft.push(car, cursor, &tuning);
                Drag::End(draft.orders.segments.len() - 1)
            }
        };
        if let Drag::End(i) | Drag::Heading(i) = *drag {
            draft.selected = Some(i);
        }
    }
    if !buttons.pressed(MouseButton::Left) {
        *drag = Drag::None;
    }
    match *drag {
        Drag::End(i) => draft.move_end(i, cursor, car, pt),
        Drag::Heading(i) => draft.aim(i, cursor, car, pt),
        Drag::None => {}
    }

    if buttons.just_pressed(MouseButton::Right)
        && let Some((i, _)) = hit
    {
        draft.delete(i, car, pt);
    }
    if scroll.delta.y != 0.0
        && let Some((i, _)) = hit
    {
        let cruise = keys.any_pressed([KeyCode::ShiftLeft, KeyCode::ShiftRight]);
        draft.adjust_speed(i, scroll.delta.y.signum(), cruise, car, &tuning);
        draft.selected = Some(i);
        scroll_used.0 = true;
    }
}

fn edit_keys(keys: Res<ButtonInput<KeyCode>>, mut draft: ResMut<Draft>, mut stepper: Stepper) {
    let car = stepper.sim.player_car();
    if keys.any_just_pressed([KeyCode::Delete, KeyCode::Backspace])
        && let Some(i) = draft.selected
    {
        let pt = stepper.tuning.planner.clone();
        draft.delete(i, car, &pt);
    }
    if keys.just_pressed(KeyCode::Escape) {
        draft.reset(&car.orders);
    }
    if keys.just_pressed(KeyCode::Period) {
        if commit(&mut draft, &mut stepper) || stepper.check.result != CheckResult::Running {
            stepper.start_reality_check();
        }
        stepper.advance();
        draft.follow(&stepper.sim.player_car().orders);
    }
}

/// Sends the draft to the live car. Returns whether anything changed.
fn commit(draft: &mut Draft, stepper: &mut Stepper) -> bool {
    if !draft.edited {
        return false;
    }
    let player = stepper.sim.player;
    if stepper.sim.player_car().drive_mode != DriveMode::Orders {
        stepper.command(Command::SetDriveMode { car: player, mode: DriveMode::Orders });
    }
    stepper.command(Command::SetOrders { car: player, orders: draft.orders.clone() });
    draft.edited = false;
    true
}

fn commit_on_resume(mut draft: ResMut<Draft>, mut stepper: Stepper) {
    commit(&mut draft, &mut stepper);
    stepper.start_reality_check();
}

fn sync_draft(mut draft: ResMut<Draft>, sim: Res<Sim>) {
    draft.follow(&sim.player_car().orders);
}

fn rebuild_preview(
    mut draft: ResMut<Draft>,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    challenge: Res<Challenge>,
    mut plan: ResMut<PlanPreview>,
) {
    if !(draft.dirty || tuning.is_changed() || challenge.is_changed()) {
        return;
    }
    draft.dirty = false;
    let mut state = sim.state.clone();
    Command::SetDriveMode { car: sim.player, mode: DriveMode::Orders }.apply(&mut state);
    Command::SetOrders { car: sim.player, orders: draft.orders.clone() }.apply(&mut state);
    let preview = Preview::run(&state, &sim.world, &tuning, tuning.preview_ticks());

    let frames: Vec<&sim::CarFrame> = (0..=preview.ticks()).filter_map(|i| preview.frame(sim.player, i)).collect();
    plan.segment_ends.clear();
    for (i, w) in frames.windows(2).enumerate() {
        for _ in w[1].segments_left..w[0].segments_left {
            plan.segment_ends.push(i + 1);
        }
    }
    plan.hits.clear();
    let mut next = challenge.next;
    for (i, f) in frames.iter().enumerate() {
        if challenge.targets.get(next).is_some_and(|t| t.satisfied(f.pose, f.vel.length())) {
            plan.hits.push((next, i));
            next += 1;
        }
    }
    plan.scrub = plan.scrub.min(preview.ticks());
    plan.preview = Some(preview);
}

fn draw_plan(
    mut gizmos: Gizmos,
    app_state: Res<State<AppState>>,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    draft: Res<Draft>,
    plan: Res<PlanPreview>,
    view: WorldView,
) {
    let vt = &tuning.vehicle;
    if *app_state.get() == AppState::Running {
        for seg in &sim.player_car().orders.segments {
            gizmos.linestrip_2d(seg.path_spec().points.iter().copied(), css::WHITE.with_alpha(0.2));
            car_outline(&mut gizmos, seg.end_pose(), vt, css::WHITE.with_alpha(0.2));
        }
        return;
    }

    let m_per_px = view.meters_per_pixel();
    for (i, seg) in draft.orders.segments.iter().enumerate() {
        let color = if draft.selected == Some(i) { css::AQUA } else { css::WHITE };
        gizmos.linestrip_2d(seg.path_spec().points.iter().copied(), color.with_alpha(0.3));
        let end = seg.end_pose();
        car_outline(&mut gizmos, end, vt, color.with_alpha(0.7));
        let knob = knob_pos(end, m_per_px, vt);
        gizmos.line_2d(end.pos + end.forward() * vt.length * 0.5, knob, color.with_alpha(0.5));
        gizmos.circle_2d(Isometry2d::from_translation(knob), 6.0 * m_per_px, color);
    }

    let Some(preview) = &plan.preview else { return };
    let frames: Vec<&sim::CarFrame> = (0..=preview.ticks()).filter_map(|i| preview.frame(sim.player, i)).collect();
    for w in frames.windows(2) {
        gizmos.line_2d(w[0].pose.pos, w[1].pose.pos, zone_color(w[1].grip_demand, w[1].slide.is_some(), vt));
    }
    for f in frames.iter().step_by(tuning.tick_rate as usize).skip(1) {
        car_outline(&mut gizmos, f.pose, vt, css::WHITE.with_alpha(0.15));
    }
    if let Some(f) = frames.get(plan.scrub) {
        car_outline(&mut gizmos, f.pose, vt, css::YELLOW);
        gizmos.arrow_2d(f.pose.pos, f.pose.pos + f.vel * 0.5, css::YELLOW.with_alpha(0.7));
    }
}

fn plan_labels(
    mut contexts: EguiContexts,
    view: WorldView,
    draft: Res<Draft>,
    plan: Res<PlanPreview>,
    tuning: Res<TuningRes>,
) -> Result {
    let ctx = contexts.ctx_mut()?;
    let painter = ctx.layer_painter(egui::LayerId::background());
    let font = egui::FontId::proportional(13.0);
    let below = Vec2::Y * (tuning.vehicle.length * 0.5 + 1.0);
    for (i, seg) in draft.orders.segments.iter().enumerate() {
        let Some(p) = view.to_screen(seg.end_pose().pos - below) else { continue };
        let color = if draft.selected == Some(i) { egui::Color32::from_rgb(0, 255, 255) } else { egui::Color32::WHITE };
        let s = seg.speed();
        let text = format!("#{}  end {:.0} m/s\ncruise {:.0}", i + 1, s.end, s.cruise);
        painter.text(p, egui::Align2::CENTER_TOP, text, font.clone(), color);
    }
    let rate = tuning.tick_rate as f32;
    for &(target, tick) in &plan.hits {
        let Some(f) = plan.player_frame(tick) else { continue };
        if let Some(p) = view.to_screen(f.pose.pos + below) {
            let text = format!("T{} reached at {:.1} s", target + 1, tick as f32 / rate);
            painter.text(p, egui::Align2::CENTER_BOTTOM, text, font.clone(), egui::Color32::LIGHT_GREEN);
        }
    }
    Ok(())
}
