//! Bevy front end: renders `sim::SimState` with gizmos, lets the player plan orders while
//! paused, and runs `sim::step` on the fixed 60 Hz tick while the game is `Running`.
//! Every change to the simulation goes through a `sim::Command`, so it lands in the replay.

mod challenge;
mod manual;
mod planning;
mod render;
mod replay;
mod timeline;
mod track;
mod tuning_file;
mod ui;

use bevy::ecs::system::SystemParam;
use bevy::prelude::*;
use bevy_egui::EguiPlugin;
use bevy_egui::input::egui_wants_any_keyboard_input;
use sim::{CarId, Command, Drive, DriveMode, OrderQueue, Pose, Preview, SimEvent, SimState, StepMode, Tuning, World};

#[derive(States, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum AppState {
    Running,
    /// The game starts paused: plan first, then watch.
    #[default]
    Paused,
}

#[derive(Resource)]
pub struct Sim {
    pub state: SimState,
    pub world: World,
    pub player: CarId,
}

impl Sim {
    fn new() -> Self {
        let mut state = SimState::new(1);
        let player = state.spawn_car(Pose::new(Vec2::ZERO, 0.0));
        Self { state, world: World::default(), player }
    }

    pub fn player_car(&self) -> &sim::Car {
        self.state.car(self.player).expect("player car exists")
    }
}

#[derive(Resource, Deref, DerefMut)]
pub struct TuningRes(pub Tuning);

/// Recent sim events, newest last, for the inspector.
#[derive(Resource, Default)]
pub struct EventLog(pub Vec<SimEvent>);

fn main() {
    let tuning = tuning_file::load_initial();
    let sim = Sim::new();
    let recorder = replay::Recorder::new(&sim.state, &tuning.0);
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Vehicle Battles — M1 (keypoints)".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .init_state::<AppState>()
        .insert_resource(Time::<Fixed>::from_hz(tuning.0.tick_rate as f64))
        .insert_resource(TuningRes(tuning.0))
        .insert_resource(tuning.1)
        .insert_resource(sim)
        .insert_resource(recorder)
        .init_resource::<EventLog>()
        .add_plugins((
            render::RenderPlugin,
            planning::PlanningPlugin,
            timeline::TimelinePlugin,
            challenge::ChallengePlugin,
            track::TrackPlugin,
            manual::ManualPlugin,
            replay::ReplayPlugin,
            ui::UiPlugin,
        ))
        .add_systems(Update, (hotkeys.run_if(not(egui_wants_any_keyboard_input)), tuning_file::hot_reload))
        .add_systems(FixedUpdate, fixed_tick.run_if(in_state(AppState::Running)))
        .run();
}

fn hotkeys(
    keys: Res<ButtonInput<KeyCode>>,
    app_state: Res<State<AppState>>,
    mut next: ResMut<NextState<AppState>>,
    mut stepper: Stepper,
    mut draft: ResMut<planning::Draft>,
) {
    if keys.just_pressed(KeyCode::Space) {
        next.set(match app_state.get() {
            AppState::Running => AppState::Paused,
            AppState::Paused => AppState::Running,
        });
    }
    let switch_mode = keys.just_pressed(KeyCode::KeyC);
    if switch_mode {
        let mode = &mut *stepper.mode;
        *mode = match *mode {
            track::Mode::Track => track::Mode::Parking,
            track::Mode::Parking => track::Mode::Track,
        };
    }
    let restart = keys.just_pressed(KeyCode::KeyR) || switch_mode;
    let new_course = keys.just_pressed(KeyCode::KeyN) && *stepper.mode == track::Mode::Parking;
    if restart || new_course {
        stepper.reset(new_course);
        draft.reset(&stepper.sim.player_car().orders);
        next.set(AppState::Paused);
    }
    if keys.just_pressed(KeyCode::KeyM) {
        let player = stepper.sim.player;
        let mode = match stepper.sim.player_car().drive_mode {
            DriveMode::Orders => DriveMode::Direct,
            DriveMode::Direct => DriveMode::Orders,
        };
        stepper.command(Command::SetDriveMode { car: player, mode });
        // Back to orders: drive straight on at the current speed.
        let car = stepper.sim.player_car();
        let tail = Drive::straight_at(draft.mode, car.forward_speed().max(0.0), &stepper.tuning.vehicle);
        stepper.command(Command::SetOrders { car: player, orders: OrderQueue { segments: Vec::new(), tail } });
        stepper.check.stop();
        draft.reset(&stepper.sim.player_car().orders);
    }
}

fn fixed_tick(mut stepper: Stepper) {
    stepper.advance();
}

/// Everything needed to change, advance or reset the simulation from any system.
#[derive(SystemParam)]
pub struct Stepper<'w> {
    pub sim: ResMut<'w, Sim>,
    pub tuning: Res<'w, TuningRes>,
    pub log: ResMut<'w, EventLog>,
    pub recorder: ResMut<'w, replay::Recorder>,
    pub check: ResMut<'w, planning::RealityCheck>,
    pub challenge: ResMut<'w, challenge::Challenge>,
    track_run: ResMut<'w, track::TrackRun>,
    track: Res<'w, track::TrackRes>,
    pub mode: ResMut<'w, track::Mode>,
    input: Res<'w, manual::DriveInput>,
    trails: ResMut<'w, render::Trails>,
}

impl Stepper<'_> {
    /// Applies a command to the live state and records it for the replay.
    pub fn command(&mut self, cmd: Command) {
        self.recorder.replay.record(self.sim.state.tick, cmd.clone());
        cmd.apply(&mut self.sim.state);
    }

    /// One simulation tick: forward manual controls, step, then update trails, the
    /// preview-vs-reality check and the challenge.
    pub fn advance(&mut self) {
        let player = self.sim.player;
        let car = self.sim.player_car();
        if car.drive_mode == DriveMode::Direct && car.controls != self.input.0 {
            self.command(Command::SetControls { car: player, controls: self.input.0 });
        }
        let sim = &mut *self.sim;
        let events = sim::step(&mut sim.state, &sim.world, &self.tuning, StepMode::Live);
        self.log.0.extend(events.events);
        let excess = self.log.0.len().saturating_sub(12);
        self.log.0.drain(..excess);
        self.trails.record(&sim.state);
        self.check.observe(&sim.state);
        match *self.mode {
            track::Mode::Parking => self.challenge.update(&sim.state, player),
            track::Mode::Track => {
                if let Some(track) = &self.track.track {
                    self.track_run.update(&sim.state, player, track);
                }
            }
        }
    }

    /// Records a preview of the current state so live execution can be compared with it.
    pub fn start_reality_check(&mut self) {
        let sim = &self.sim;
        if sim.player_car().drive_mode != DriveMode::Orders {
            self.check.stop();
            return;
        }
        let preview = Preview::run(&sim.state, &sim.world, &self.tuning, self.tuning.preview_ticks());
        self.check.start(preview);
    }

    pub fn reset(&mut self, new_course: bool) {
        *self.sim = Sim::new();
        self.trails.clear();
        self.log.0.clear();
        *self.recorder = replay::Recorder::new(&self.sim.state, &self.tuning);
        self.check.stop();
        self.challenge.restart(new_course);
        self.track_run.restart();
    }
}
