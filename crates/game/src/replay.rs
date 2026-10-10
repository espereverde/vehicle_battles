//! Replay recording: F5 saves the session to `replays/`, F9 re-simulates it in memory and
//! checks it against the live state. Verify saved files with the `replay_check` binary.

use bevy::prelude::*;
use bevy_egui::input::egui_wants_any_keyboard_input;
use sim::{Replay, SimState, Tuning};

use crate::Sim;

pub struct ReplayPlugin;

impl Plugin for ReplayPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, replay_keys.run_if(not(egui_wants_any_keyboard_input)));
    }
}

#[derive(Resource)]
pub struct Recorder {
    pub replay: Replay,
    /// Result of the last save / verify, for the inspector.
    pub status: String,
}

impl Recorder {
    pub fn new(state: &SimState, tuning: &Tuning) -> Self {
        Self { replay: Replay::new(state.clone(), tuning.clone()), status: String::new() }
    }
}

fn replay_keys(keys: Res<ButtonInput<KeyCode>>, sim: Res<Sim>, mut recorder: ResMut<Recorder>) {
    if keys.just_pressed(KeyCode::F5) {
        let mut replay = recorder.replay.clone();
        replay.finish(&sim.state);
        recorder.status = match save(&replay) {
            Ok(path) => format!("saved {path}"),
            Err(e) => format!("save failed: {e}"),
        };
        info!("{}", recorder.status);
    }
    if keys.just_pressed(KeyCode::F9) {
        let mut replay = recorder.replay.clone();
        replay.finish(&sim.state);
        recorder.status = match replay.verify(&sim.world) {
            Ok(()) => format!("replay OK ({} ticks, {} commands)", replay.end_tick, replay.commands.len()),
            Err(e) => format!("replay MISMATCH: {e}"),
        };
        info!("{}", recorder.status);
    }
}

fn save(replay: &Replay) -> Result<String, String> {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_secs());
    std::fs::create_dir_all("replays").map_err(|e| e.to_string())?;
    let path = format!("replays/replay-{secs}-t{}.ron", replay.end_tick);
    let text = replay.to_ron().map_err(|e| e.to_string())?;
    std::fs::write(&path, text).map_err(|e| e.to_string())?;
    Ok(path)
}
