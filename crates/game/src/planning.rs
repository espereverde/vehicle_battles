//! Planning while paused, with keypoints. Each segment between keypoints holds a constant
//! turn (steering or drift) and a constant throttle. Keypoints are anchored to instants
//! in time: refitting or deleting one only changes how long the next segment lasts, so
//! later keypoints keep their times (their positions follow from the simulation). Resuming
//! commits the draft as a `Command::SetOrders` and starts a preview-vs-reality check.
//!
//! Planning runs without dice (`StepMode::Ideal`): the fit and the preview never lose
//! control, so the path follows the cursor steadily and shows the intended outcome. The car
//! is kinematic, so live it drives exactly that path unless it loses control; the coloring
//! marks where it might. Turning is limited by the steering lock.
//!
//! Mouse: LMB on the ground adds a keypoint there (drag to place it); drag a keypoint ghost
//! to refit its segment (further = later, sideways = more turn); the wheel changes the
//! throttle of the dragged / hovered / selected keypoint; RMB on a ghost deletes it.
//! Keys: T turn mode (steering → drift → skid), S straight, Del delete, Esc revert,
//! `.` single tick. A drift moves sideways at most ½" per inch forward along two arcs of
//! equal length: it ends at the cursor pointing the way the car started.

use bevy::color::palettes::css;
use bevy::input::mouse::AccumulatedMouseScroll;
use bevy::prelude::*;
use bevy_egui::input::{egui_wants_any_keyboard_input, egui_wants_any_pointer_input};
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use sim::preview::Check;
use sim::{CarFrame, Command, Drive, DriveMode, OrderQueue, Preview, RiskStretch, Segment, SimState, StepMode, Turn, TurnMode, fit_bend, fit_drift, fit_skid};

use crate::challenge::Challenge;
use crate::slalom::Slalom;
use crate::track::{Mode, TrackRes};
use crate::ui::kmh;
use crate::render::{self, WorldView, car_outline, zone_color};
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
                        edit_mouse.run_if(not(egui_wants_any_pointer_input)),
                        edit_keys.run_if(not(egui_wants_any_keyboard_input)),
                        rebuild_preview,
                    )
                        .chain()
                        .in_set(PlanEdit)
                        .run_if(in_state(AppState::Paused)),
                    follow_live.run_if(in_state(AppState::Running)),
                    draw_plan,
                ),
            )
            .add_systems(EguiPrimaryContextPass, plan_labels.run_if(in_state(AppState::Paused)));
    }
}


/// Editing the draft and rebuilding its preview (while paused).
#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub struct PlanEdit;

/// The player's orders as being edited while paused.
#[derive(Resource)]
pub struct Draft {
    pub orders: OrderQueue,
    pub selected: Option<usize>,
    /// Turn mode for new keypoints (and the one highlighted when nothing is selected).
    pub mode: TurnMode,
    /// The draft differs from the live orders.
    pub edited: bool,
    /// The preview is out of date.
    dirty: bool,
}

impl Default for Draft {
    fn default() -> Self {
        Self { orders: OrderQueue::default(), selected: None, mode: TurnMode::Steer, edited: false, dirty: true }
    }
}

/// Live state with the draft orders applied, ready to be stepped.
fn draft_state(sim: &Sim, orders: &OrderQueue) -> SimState {
    let mut state = sim.state.clone();
    Command::SetDriveMode { car: sim.player, mode: DriveMode::Orders }.apply(&mut state);
    Command::SetOrders { car: sim.player, orders: orders.clone() }.apply(&mut state);
    state
}

impl Draft {
    /// Discards edits and starts over from the live orders.
    pub fn reset(&mut self, live: &OrderQueue) {
        *self = Self { orders: live.clone(), mode: self.mode, ..Self::default() };
    }

    /// Catches up with the live orders after ticks ran (completed segments drop off the
    /// front), keeping the selection on the same keypoint.
    fn follow(&mut self, live: &OrderQueue) {
        let done = self.orders.segments.len().saturating_sub(live.segments.len());
        self.selected = self.selected.and_then(|i| i.checked_sub(done)).filter(|&i| i < live.segments.len());
        self.orders = live.clone();
        self.edited = false;
        self.dirty = true;
    }

    pub fn changed(&mut self) {
        self.edited = true;
        self.dirty = true;
    }

    /// The simulation at the start of segment `k`, as the preview sees it, and the net
    /// heading change up to there (`None` if it starts beyond the preview or after the
    /// first loop).
    fn start_state(&self, k: usize, sim: &Sim, tuning: &TuningRes) -> Option<(SimState, f32)> {
        let start = self.orders.start_tick(k);
        if start >= tuning.preview_ticks() {
            return None;
        }
        let mut state = draft_state(sim, &self.orders);
        let mut turned = 0.0;
        for _ in 0..start {
            let before = state.car(sim.player)?.pose.heading;
            sim::step(&mut state, &sim.world, tuning, StepMode::Ideal);
            turned += sim::vehicle::wrap_angle(state.car(sim.player)?.pose.heading - before);
        }
        (turned.abs() < std::f32::consts::TAU).then_some((state, turned))
    }

    fn start_speed(&self, k: usize, sim: &Sim, tuning: &TuningRes) -> Option<f32> {
        let (state, _) = self.start_state(k, sim, tuning)?;
        Some(state.car(sim.player)?.speed().max(0.0))
    }

    /// Longest segment `k` may be (total ticks): its keypoint stays before the next one, and
    /// the last one stays within the preview horizon.
    pub fn max_ticks(&self, k: usize, tuning: &sim::Tuning) -> u32 {
        let seg = &self.orders.segments[k];
        match self.orders.segments.get(k + 1) {
            Some(next) => seg.ticks + next.remaining() - 1,
            None => seg.elapsed + tuning.preview_ticks().saturating_sub(self.orders.start_tick(k)).max(1),
        }
    }

    /// Sets segment `k`'s length (clamped). Keypoints are anchored in time: the next segment
    /// absorbs the change, so later keypoints keep their times.
    /// A controlled skid that has started can't be changed (its commitment window).
    pub fn locked(&self, k: usize) -> bool {
        self.orders.segments.get(k).is_some_and(|s| s.elapsed > 0 && s.drive.turn.mode() == TurnMode::Skid)
    }

    pub fn set_ticks(&mut self, k: usize, ticks: u32, tuning: &sim::Tuning) {
        if self.locked(k) {
            return;
        }
        let max = self.max_ticks(k, tuning);
        let seg = &mut self.orders.segments[k];
        let ticks = ticks.min(max).max(seg.elapsed + 1);
        if ticks == seg.ticks {
            return;
        }
        let old = std::mem::replace(&mut seg.ticks, ticks);
        if let Some(next) = self.orders.segments.get_mut(k + 1) {
            next.ticks = next.ticks + old - ticks;
        }
        self.changed();
    }

    /// Refits segment `k`'s turn and duration so its keypoint lands as close as possible to
    /// `target`, keeping its throttle and turn mode (see `set_ticks` for the timing).
    fn refit(&mut self, k: usize, target: Vec2, sim: &Sim, tuning: &TuningRes) {
        if self.locked(k) {
            return;
        }
        let Some((state, turned)) = self.start_state(k, sim, tuning) else { return };
        let seg = self.orders.segments[k];
        let (mode, throttle, max_ticks) = (seg.drive.turn.mode(), seg.drive.throttle, self.max_ticks(k, tuning) - seg.elapsed);
        let fit = match mode {
            TurnMode::Drift => fit_drift(&state, tuning, sim.player, throttle, target, max_ticks),
            TurnMode::Skid => fit_skid(&state, tuning, sim.player, throttle, seg.drive.turn.slide(), target, max_ticks),
            TurnMode::Steer => fit_bend(&state, tuning, sim.player, throttle, target, max_ticks, turned),
        };
        if fit.turn != seg.drive.turn {
            self.orders.segments[k].drive.turn = fit.turn;
            self.changed();
        }
        self.set_ticks(k, seg.elapsed + fit.ticks, tuning);
    }

    /// Throttle of a new keypoint after the last, starting at speed `v0`: the last one's,
    /// unless that would leave the car sitting still.
    fn new_throttle(&self, v0: f32, tuning: &TuningRes) -> f32 {
        let throttle = self.orders.last_drive().throttle;
        if v0 < 1.0 && throttle <= 0.0 { tuning.planner.default_throttle } else { throttle }
    }

    /// Appends a keypoint aimed at `target`. Returns its index, or `None` when the existing
    /// keypoints already fill the preview.
    fn push(&mut self, target: Vec2, sim: &Sim, tuning: &TuningRes) -> Option<usize> {
        let k = self.orders.segments.len();
        let throttle = self.new_throttle(self.start_speed(k, sim, tuning)?, tuning);
        self.orders.segments.push(Segment::new(Drive { turn: Turn::new(self.mode, 0.0), throttle }, 1));
        self.refit(k, target, sim, tuning);
        self.selected = Some(k);
        self.changed();
        Some(k)
    }

    /// Makes the selected segment drive straight at its starting speed, or appends a
    /// straight keypoint when nothing is selected.
    pub fn straight(&mut self, sim: &Sim, tuning: &TuningRes) {
        let vt = &tuning.vehicle;
        match self.selected {
            Some(k) => {
                if self.locked(k) {
                    return;
                }
                let Some(v0) = self.start_speed(k, sim, tuning) else { return };
                let seg = &mut self.orders.segments[k];
                seg.drive = Drive::straight_at(seg.drive.turn.mode(), v0, vt);
            }
            None => {
                let k = self.orders.segments.len();
                let Some(v0) = self.start_speed(k, sim, tuning) else { return };
                let room = tuning.preview_ticks() - self.orders.start_tick(k);
                let ticks = ((tuning.planner.straight_seconds * tuning.tick_rate as f32) as u32).min(room);
                self.orders.segments.push(Segment::new(Drive::straight_at(self.mode, v0, vt), ticks));
                self.selected = Some(k);
            }
        }
        self.changed();
    }

    /// Switches to `mode`. A selected segment is converted and refitted so its keypoint
    /// stays where it is (a drift ends pointing the way it started, so its keypoint turns).
    pub fn set_mode(&mut self, mode: TurnMode, sim: &Sim, tuning: &TuningRes, keypoint: Option<Vec2>) {
        self.mode = mode;
        let Some(k) = self.selected else { return };
        if self.orders.segments[k].drive.turn.mode() == mode || self.locked(k) {
            return;
        }
        let seg = &mut self.orders.segments[k];
        seg.drive.turn = seg.drive.turn.convert(mode);
        self.changed();
        if let Some(p) = keypoint {
            self.refit(k, p, sim, tuning);
        }
    }

    fn adjust_throttle(&mut self, k: usize, notches: f32, step: f32) {
        if self.locked(k) {
            return;
        }
        let seg = &mut self.orders.segments[k];
        let t = ((seg.drive.throttle + notches * step) / step).round() * step;
        seg.drive.throttle = t.clamp(-1.0, 1.0);
        self.changed();
    }

    /// Removes keypoint `k`; the next segment starts earlier so its keypoint keeps its time.
    pub fn delete(&mut self, k: usize) {
        if self.locked(k) {
            return;
        }
        let removed = self.orders.segments.remove(k);
        if let Some(next) = self.orders.segments.get_mut(k) {
            next.ticks += removed.remaining();
        }
        self.selected = match self.selected {
            Some(s) if s == k => None,
            Some(s) if s > k => Some(s - 1),
            s => s,
        };
        self.changed();
    }

    /// Removes every keypoint but a committed skid.
    pub fn clear(&mut self) {
        let keep = usize::from(self.locked(0));
        self.orders.segments.truncate(keep);
        self.selected = None;
        self.changed();
    }

    /// Mode shown in the UI: the selected segment's, else the one for new keypoints.
    pub fn shown_mode(&self) -> TurnMode {
        self.selected.and_then(|k| self.orders.segments.get(k)).map_or(self.mode, |s| s.drive.turn.mode())
    }
}

/// Preview of the draft, rebuilt whenever the draft or tuning changes.
#[derive(Resource, Default)]
pub struct PlanPreview {
    pub preview: Option<Preview>,
    /// Scrub position, in ticks into the preview.
    pub scrub: usize,
    /// Preview tick at which each draft segment ends (may exceed the horizon).
    pub ends: Vec<usize>,
    /// Challenge targets the preview reaches: (target index, preview tick).
    pub hits: Vec<(usize, usize)>,
    /// Preview ticks where the car leaves the track (track mode).
    pub off_track: Vec<usize>,
    /// Where the player's car may lose control (`Preview::risks`).
    pub risks: Vec<RiskStretch>,
    /// Slalom gates the preview crosses: (gate index, preview tick, inside the gate).
    pub gates: Vec<(usize, usize, bool)>,
    /// Slalom finish within the preview: (preview tick, result with penalties in s).
    pub finish: Option<(usize, f32)>,
    /// While dragging a keypoint, the scrub follows its end.
    follow: Option<usize>,
}

impl PlanPreview {
    pub fn player_frame(&self, i: usize) -> Option<&CarFrame> {
        self.preview.as_ref()?.frames.get(i)?.first()
    }

    /// Keypoint `k`'s end state, if it falls inside the preview.
    pub fn keypoint(&self, k: usize) -> Option<&CarFrame> {
        self.player_frame(*self.ends.get(k)?)
    }

    /// Handling drained over keypoint `k`'s segment and its chance of losing control.
    pub fn keypoint_risk(&self, k: usize) -> Option<(f32, f32)> {
        let preview = self.preview.as_ref()?;
        let car = *preview.cars.first()?;
        let end = (*self.ends.get(k)?).min(preview.ticks());
        let start = if k == 0 { 0 } else { self.ends[k - 1].min(end) };
        Some(preview.span_risk(car, start, end))
    }

    /// Label for a risk stretch: "34% loss (20% spinout+) at 2.1 s", or "LOSS OF CONTROL"
    /// when certain. "Spinout+" is a spinout or worse; below it the crash is a skid.
    pub fn risk_label(r: &RiskStretch, rate: f32) -> String {
        let kind = match r.severe_chance {
            p if p >= 0.995 => "spinout+".to_string(),
            p if p > 0.0 => format!("{:.0}% spinout+", (p * 100.0).max(1.0)),
            _ => "skid".to_string(),
        };
        match r.certain_at {
            Some(i) => format!("LOSS OF CONTROL ({kind}) at {:.1} s", i as f32 / rate),
            None => format!("{:.0}% loss ({kind}) at {:.1} s", r.chance * 100.0, r.start as f32 / rate),
        }
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
struct Drag(Option<usize>);

/// Keypoint ghost under `p`, preferring the selected one.
fn keypoint_at(draft: &Draft, plan: &PlanPreview, p: Vec2, m_per_px: f32, car_len: f32) -> Option<usize> {
    let n = draft.orders.segments.len();
    let pick = (10.0 * m_per_px).max(car_len * 0.5);
    let order = draft.selected.filter(|&k| k < n).into_iter().chain((0..n).rev());
    order.into_iter().find(|&k| plan.keypoint(k).is_some_and(|f| f.pose.pos.distance(p) < pick))
}

#[allow(clippy::too_many_arguments)]
fn edit_mouse(
    buttons: Res<ButtonInput<MouseButton>>,
    scroll: Res<AccumulatedMouseScroll>,
    view: WorldView,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    mut draft: ResMut<Draft>,
    mut plan: ResMut<PlanPreview>,
    mut drag: ResMut<Drag>,
    keys: Res<ButtonInput<KeyCode>>,
) {
    let Some(cursor) = view.cursor() else { return };
    let hit = keypoint_at(&draft, &plan, cursor, view.meters_per_pixel(), tuning.vehicle.length);

    if buttons.just_pressed(MouseButton::Left) {
        drag.0 = hit.or_else(|| draft.push(cursor, &sim, &tuning));
        if drag.0.is_some() {
            draft.selected = drag.0;
        }
    }
    if !buttons.pressed(MouseButton::Left) {
        drag.0 = None;
    }

    // Throttle: wheel (Ctrl+wheel zooms) on the dragged keypoint, else the one under the
    // cursor, else the selected one.
    if scroll.delta.y != 0.0
        && !render::zoom_modifier(&keys)
        && let Some(k) = drag.0.or(hit).or(draft.selected)
    {
        draft.adjust_throttle(k, scroll.delta.y.signum(), tuning.planner.throttle_step);
        draft.selected = Some(k);
    }
    if let Some(k) = drag.0 {
        draft.refit(k, cursor, &sim, &tuning);
    }
    plan.follow = drag.0;

    if buttons.just_pressed(MouseButton::Right)
        && let Some(k) = hit
    {
        draft.delete(k);
    }
}

fn edit_keys(keys: Res<ButtonInput<KeyCode>>, mut draft: ResMut<Draft>, plan: Res<PlanPreview>, mut stepper: Stepper) {
    if keys.any_just_pressed([KeyCode::Delete, KeyCode::Backspace])
        && let Some(k) = draft.selected
    {
        draft.delete(k);
    }
    if keys.just_pressed(KeyCode::Escape) {
        draft.reset(&stepper.sim.player_car().orders);
    }
    if keys.just_pressed(KeyCode::KeyT) {
        let mode = match draft.shown_mode() {
            TurnMode::Steer => TurnMode::Drift,
            TurnMode::Drift => TurnMode::Skid,
            TurnMode::Skid => TurnMode::Steer,
        };
        let keypoint = draft.selected.and_then(|k| plan.keypoint(k)).map(|f| f.pose.pos);
        draft.set_mode(mode, &stepper.sim, &stepper.tuning, keypoint);
    }
    if keys.just_pressed(KeyCode::KeyS) {
        draft.straight(&stepper.sim, &stepper.tuning);
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

#[allow(clippy::too_many_arguments)]
fn rebuild_preview(
    slalom: Res<Slalom>,
    mut draft: ResMut<Draft>,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    challenge: Res<Challenge>,
    mode: Res<Mode>,
    track: Res<TrackRes>,
    mut plan: ResMut<PlanPreview>,
) {
    if draft.dirty || tuning.is_changed() || challenge.is_changed() || mode.is_changed() || slalom.is_changed() {
        draft.dirty = false;
        let state = draft_state(&sim, &draft.orders);
        let preview = ideal_preview(&state, &sim, &tuning);
        plan.ends = draft.orders.end_ticks().into_iter().map(|t| t as usize).collect();
        plan.hits.clear();
        plan.off_track.clear();
        plan.gates.clear();
        plan.finish = None;
        let track = track.track.as_ref().filter(|_| *mode == Mode::Track);
        // Slalom: carry on the live run's gate count over the preview.
        let spec = slalom.spec.as_ref().filter(|_| *mode == Mode::Slalom);
        let mut course = slalom.course.clone();
        let mut next = challenge.next;
        let mut was_on = true;
        for i in 0..=preview.ticks() {
            let Some(f) = preview.frame(sim.player, i) else { break };
            if *mode == Mode::Parking && challenge.targets.get(next).is_some_and(|t| t.satisfied(f.pose, f.vel.length())) {
                plan.hits.push((next, i));
                next += 1;
            }
            if let Some(spec) = spec {
                let tick = preview.start_tick + i as u64;
                plan.gates.extend(course.update(spec, f.pose, tick).iter().map(|p| (p.gate, i, p.ok)));
                if course.finish == Some(tick) {
                    plan.finish = course.result(spec, tuning.tick_rate as f32).map(|r| (i, r));
                }
            }
            if let Some(track) = track {
                let on = track.on_track(f.pose.pos);
                if was_on && !on && i > 0 {
                    plan.off_track.push(i);
                }
                was_on = on;
            }
        }
        plan.scrub = plan.scrub.min(preview.ticks());
        plan.risks = preview.risks(sim.player, &tuning);
        plan.preview = Some(preview);
    }
    if let Some(&end) = plan.follow.and_then(|k| plan.ends.get(k)) {
        plan.scrub = end.min(tuning.preview_ticks() as usize);
    }
}

/// While running, the expected path is re-previewed from the live state whenever the
/// simulation advanced, so it always starts at the car.
fn follow_live(sim: Res<Sim>, tuning: Res<TuningRes>, mut plan: ResMut<PlanPreview>) {
    if sim.player_car().drive_mode != DriveMode::Orders {
        plan.preview = None;
        return;
    }
    if plan.preview.as_ref().is_some_and(|p| p.start_tick == sim.state.tick) {
        return;
    }
    let preview = ideal_preview(&sim.state, &sim, &tuning);
    plan.risks = preview.risks(sim.player, &tuning);
    plan.preview = Some(preview);
}

/// The player's preview without dice, cut where the path closes its first loop (after
/// that it would only go round again).
fn ideal_preview(state: &SimState, sim: &Sim, tuning: &TuningRes) -> Preview {
    let mut preview = Preview::run_in(StepMode::Ideal, state, &sim.world, tuning, tuning.preview_ticks());
    if let Some(i) = preview.first_loop(sim.player) {
        preview.cut(i);
    }
    preview
}

fn player_frames<'a>(preview: &'a Preview, sim: &Sim) -> Vec<&'a CarFrame> {
    (0..=preview.ticks()).filter_map(|i| preview.frame(sim.player, i)).collect()
}

fn draw_plan(
    mut gizmos: Gizmos,
    app_state: Res<State<AppState>>,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    draft: Res<Draft>,
    plan: Res<PlanPreview>,
) {
    let vt = &tuning.vehicle;
    if *app_state.get() == AppState::Running {
        // The expected path from where the car is now, faintly.
        if let Some(preview) = &plan.preview {
            let frames = player_frames(preview, &sim);
            gizmos.linestrip_2d(frames.iter().map(|f| f.pose.pos), css::WHITE.with_alpha(0.15));
            draw_risks(&mut gizmos, &plan, &frames, 0.5);
        }
        return;
    }

    let Some(preview) = &plan.preview else { return };
    let frames = player_frames(preview, &sim);
    for w in frames.windows(2) {
        gizmos.line_2d(w[0].pose.pos, w[1].pose.pos, zone_color(w[1].zone(&tuning)));
    }
    for f in frames.iter().step_by(tuning.tick_rate as usize).skip(1) {
        car_outline(&mut gizmos, f.pose, vt, css::WHITE.with_alpha(0.12));
    }
    for k in 0..draft.orders.segments.len() {
        let Some(f) = plan.keypoint(k) else { continue };
        let color = if draft.selected == Some(k) { css::AQUA } else { css::WHITE };
        car_outline(&mut gizmos, f.pose, vt, color.with_alpha(0.8));
        gizmos.circle_2d(Isometry2d::from_translation(f.pose.pos), 0.4, color);
    }
    for &i in &plan.off_track {
        if let Some(f) = frames.get(i) {
            gizmos.circle_2d(Isometry2d::from_translation(f.pose.pos), 1.5, css::MAGENTA);
            gizmos.circle_2d(Isometry2d::from_translation(f.pose.pos), 2.0, css::MAGENTA);
        }
    }
    draw_risks(&mut gizmos, &plan, &frames, 1.0);
    if let Some(f) = frames.get(plan.scrub) {
        car_outline(&mut gizmos, f.pose, vt, css::YELLOW);
        gizmos.arrow_2d(f.pose.pos, f.pose.pos + f.vel * 0.5, css::YELLOW.with_alpha(0.7));
    }
}

/// Risk markers on the path: a red ring where a stretch starts (larger for higher chance),
/// a red cross at a certain loss of control.
fn draw_risks(gizmos: &mut Gizmos, plan: &PlanPreview, frames: &[&CarFrame], alpha: f32) {
    let red = css::RED.with_alpha(alpha);
    for r in &plan.risks {
        let Some(f) = frames.get(r.at()) else { continue };
        let p = f.pose.pos;
        if r.certain_at.is_some() {
            gizmos.circle_2d(Isometry2d::from_translation(p), 2.5, red);
            let d = Vec2::splat(1.6);
            gizmos.line_2d(p - d, p + d, red);
            gizmos.line_2d(p + Vec2::new(-d.x, d.y), p + Vec2::new(d.x, -d.y), red);
        } else {
            gizmos.circle_2d(Isometry2d::from_translation(p), 1.0 + 2.0 * r.chance, red);
        }
        // The rest of the stretch, as a thin red trace beside the path.
        for w in frames.get(r.start..=r.end.min(frames.len() - 1)).unwrap_or(&[]).windows(2) {
            let off = w[0].pose.left() * 1.2;
            gizmos.line_2d(w[0].pose.pos + off, w[1].pose.pos + off, red.with_alpha(alpha * 0.6));
        }
    }
}

/// "D 3.2 · 18% loss" for a maneuver's handling drain and chance of losing control.
pub fn risk_text(drained: f32, chance: f32) -> String {
    match chance {
        c if c >= 0.995 => format!("D {drained:.1} · LOSS"),
        c if c > 0.0 => format!("D {drained:.1} · {:.0}% loss", (c * 100.0).max(1.0)),
        _ => format!("D {drained:.1} · safe"),
    }
}

/// Short description of a segment's controls.
pub fn describe(drive: &Drive) -> String {
    let turn = match drive.turn {
        Turn::Steer(s) => format!("steer {:+.0}%", s * 100.0),
        Turn::Drift(r) => format!("drift {:+.2}\"/in", r),
        Turn::Skid { angle, .. } => format!("skid {:+.0}° over {:.2}\"", angle.to_degrees(), drive.turn.slide()),
    };
    format!("thr {:+.0}%  {turn}", drive.throttle * 100.0)
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
    let rate = tuning.tick_rate as f32;
    for (k, seg) in draft.orders.segments.iter().enumerate() {
        let Some(f) = plan.keypoint(k) else { continue };
        let Some(p) = view.to_screen(f.pose.pos - below) else { continue };
        let color = if draft.selected == Some(k) { egui::Color32::from_rgb(0, 255, 255) } else { egui::Color32::WHITE };
        let mut text = format!("#{}  {:.1} s  {:.0} km/h\n{}", k + 1, plan.ends[k] as f32 / rate, kmh(f.vel.length()), describe(&seg.drive));
        if let Some((drained, chance)) = plan.keypoint_risk(k) {
            text += &format!("\n{}", risk_text(drained, chance));
        }
        painter.text(p, egui::Align2::CENTER_TOP, text, font.clone(), color);
    }
    // Expected speed at the scrubbed instant, next to the scrub ghost.
    if let Some(f) = plan.player_frame(plan.scrub)
        && let Some(p) = view.to_screen(f.pose.pos + below)
    {
        let accel = kmh(f.accel.dot(f.vel.normalize_or_zero()));
        let text = format!("{:.0} km/h  {accel:+.1} km/h/s  @ {:.2} s", kmh(f.vel.length()), plan.scrub as f32 / rate);
        painter.text(p, egui::Align2::CENTER_BOTTOM, text, font.clone(), egui::Color32::YELLOW);
    }
    for &tick in &plan.off_track {
        let Some(f) = plan.player_frame(tick) else { continue };
        if let Some(p) = view.to_screen(f.pose.pos + below * 2.0) {
            let text = format!("OFF TRACK at {:.1} s", tick as f32 / rate);
            painter.text(p, egui::Align2::CENTER_BOTTOM, text, font.clone(), egui::Color32::from_rgb(255, 90, 200));
        }
    }
    for &(target, tick) in &plan.hits {
        let Some(f) = plan.player_frame(tick) else { continue };
        if let Some(p) = view.to_screen(f.pose.pos + below * 2.0) {
            let text = format!("T{} reached at {:.1} s", target + 1, tick as f32 / rate);
            painter.text(p, egui::Align2::CENTER_BOTTOM, text, font.clone(), egui::Color32::LIGHT_GREEN);
        }
    }
    for r in &plan.risks {
        let Some(f) = plan.player_frame(r.at()) else { continue };
        if let Some(p) = view.to_screen(f.pose.pos + Vec2::new(3.0, 0.0)) {
            let red = egui::Color32::from_rgb(255, 80, 80);
            painter.text(p, egui::Align2::LEFT_CENTER, PlanPreview::risk_label(r, rate), font.clone(), red);
        }
    }
    Ok(())
}

