//! Debug keyboard driving (M toggles the player car into `DriveMode::Direct`).

use bevy::prelude::*;
use bevy_egui::input::egui_wants_any_keyboard_input;
use sim::{Controls, DriveMode};

use crate::{Sim, TuningRes};

pub struct ManualPlugin;

impl Plugin for ManualPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DriveInput>()
            .add_systems(Update, read_drive_input.run_if(not(egui_wants_any_keyboard_input)));
    }
}

/// Controls for the player car, sampled every frame and sent on the next fixed tick.
#[derive(Resource, Default)]
pub struct DriveInput(pub Controls);

fn read_drive_input(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    mut steer_in: Local<f32>,
    mut input: ResMut<DriveInput>,
) {
    let car = sim.player_car();
    if car.drive_mode != DriveMode::Direct {
        *steer_in = 0.0;
        input.0 = Controls::default();
        return;
    }
    let any = |codes: &[KeyCode]| keys.any_pressed(codes.iter().copied());
    let v = car.speed();
    let (kb, vt) = (&tuning.keyboard, &tuning.vehicle);

    // Steering input builds up while a key is held, and returns through zero when released
    // or reversed, so turns can be tightened gradually with a digital key.
    let mut dir = 0.0;
    if any(&[KeyCode::KeyA, KeyCode::ArrowLeft]) {
        dir += 1.0;
    }
    if any(&[KeyCode::KeyD, KeyCode::ArrowRight]) {
        dir -= 1.0;
    }
    let rate = if dir == 0.0 || dir * *steer_in < 0.0 { kb.steer_return_per_s } else { kb.steer_ramp_per_s };
    let target = if dir * *steer_in < 0.0 { 0.0 } else { dir };
    *steer_in += (target - *steer_in).clamp(-rate * time.delta_secs(), rate * time.delta_secs());

    // Input = fraction of a target lateral acceleration: tan δ = a_lat · L / v².
    let g = if any(&[KeyCode::ControlLeft, KeyCode::ControlRight]) { kb.push_steer_g } else { kb.full_steer_g };
    let max_steer = vt.max_steer_deg.to_radians();
    let a_lat = *steer_in * g * vt.gravity;
    let delta = (a_lat * vt.wheelbase / (v * v).max(1e-3)).atan().clamp(-max_steer, max_steer);
    let steer = delta / max_steer;

    // W/S accelerate in their direction, or brake when the car is rolling the other way.
    let (mut throttle, mut brake) = (0.0, 0.0);
    if any(&[KeyCode::KeyW, KeyCode::ArrowUp]) {
        if v < -0.5 { brake = 1.0 } else { throttle = 1.0 }
    }
    if any(&[KeyCode::KeyS, KeyCode::ArrowDown]) {
        if v > 0.5 { brake = 1.0 } else { throttle = -1.0 }
    }

    input.0 = Controls {
        steer,
        throttle,
        brake,
        handbrake: any(&[KeyCode::ShiftLeft, KeyCode::ShiftRight]),
    };
}
