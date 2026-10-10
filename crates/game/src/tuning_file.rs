//! `assets/tuning.ron` loading with hot reload (mtime polling, no asset server needed).

use std::path::PathBuf;
use std::time::SystemTime;

use bevy::prelude::*;
use sim::Tuning;

use crate::planning::RealityCheck;
use crate::replay::Recorder;
use crate::{Sim, TuningRes};

#[derive(Resource)]
pub struct TuningFile {
    pub path: PathBuf,
    modified: Option<SystemTime>,
    /// `Ok(())` after a successful load, otherwise the last parse / IO error.
    pub status: Result<(), String>,
    pub reloads: u32,
    poll: Timer,
}

fn tuning_path() -> PathBuf {
    if let Ok(p) = std::env::var("TUNING_PATH") {
        return p.into();
    }
    let cwd = PathBuf::from("assets/tuning.ron");
    if cwd.exists() {
        return cwd;
    }
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/tuning.ron"))
}

fn read(path: &PathBuf) -> Result<Tuning, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    ron::from_str(&text).map_err(|e| e.to_string())
}

fn mtime(path: &PathBuf) -> Option<SystemTime> {
    std::fs::metadata(path).and_then(|m| m.modified()).ok()
}

/// Falls back to built-in defaults (and reports the error) if the file is missing or broken.
pub fn load_initial() -> (Tuning, TuningFile) {
    let path = tuning_path();
    let (tuning, status) = match read(&path) {
        Ok(t) => (t, Ok(())),
        Err(e) => {
            error!("tuning: {e}; using defaults");
            (Tuning::default(), Err(e))
        }
    };
    let file = TuningFile {
        modified: mtime(&path),
        path,
        status,
        reloads: 0,
        poll: Timer::from_seconds(0.25, TimerMode::Repeating),
    };
    (tuning, file)
}

pub fn hot_reload(
    time: Res<Time>,
    mut file: ResMut<TuningFile>,
    mut tuning: ResMut<TuningRes>,
    mut fixed: ResMut<Time<Fixed>>,
    mut recorder: ResMut<Recorder>,
    mut check: ResMut<RealityCheck>,
    sim: Res<Sim>,
) {
    if !file.poll.tick(time.delta()).just_finished() {
        return;
    }
    let modified = mtime(&file.path);
    if modified == file.modified {
        return;
    }
    file.modified = modified;
    match read(&file.path) {
        Ok(t) => {
            if t.tick_rate != tuning.tick_rate {
                fixed.set_timestep_hz(t.tick_rate as f64);
            }
            recorder.replay.record_tuning(sim.state.tick, t.clone());
            // New tuning changes the outcome: the running preview no longer applies.
            check.stop();
            tuning.0 = t;
            file.status = Ok(());
            file.reloads += 1;
            info!("tuning reloaded from {}", file.path.display());
        }
        Err(e) => {
            warn!("tuning reload failed, keeping previous values: {e}");
            file.status = Err(e);
        }
    }
}
