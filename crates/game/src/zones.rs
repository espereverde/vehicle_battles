//! Danger zones: while paused, shades every place the selected keypoint (or a new one after
//! the last) can be dragged to, in the grip color its segment would end up with there:
//! green if the whole segment up to that point stays comfortable, yellow, orange
//! (understeer) or red (slide) otherwise. Z or the timeline checkbox toggles it.
//!
//! The map is the fit's own search space (`sim::sweep`): a fan of constant turns, one per
//! sampled turn value, meshed as quads between neighboring turns at consecutive ticks.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy_egui::input::egui_wants_any_keyboard_input;
use sim::{Segment, TurnMode};

use crate::planning::{Draft, PlanEdit, PlanPreview};
use crate::render::{ZONE_COLORS, zone};
use crate::{AppState, Sim, TuningRes};

/// Turn values sampled across the fit's range.
const TURNS: usize = 81;
const ALPHA: f32 = 0.18;
/// Below the road (z = -1 is the road, gizmos draw on top).
const Z: f32 = -0.5;

pub struct ZonesPlugin;

impl Plugin for ZonesPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DangerZones>().add_systems(Startup, spawn).add_systems(
            Update,
            (toggle.run_if(not(egui_wants_any_keyboard_input)), update.after(PlanEdit)).chain(),
        );
    }
}

#[derive(Resource)]
pub struct DangerZones {
    pub enabled: bool,
    /// What the current mesh was built for; `None` forces a rebuild.
    built_for: Option<Key>,
}

impl Default for DangerZones {
    fn default() -> Self {
        Self { enabled: true, built_for: None }
    }
}

#[derive(Clone, PartialEq)]
struct Key {
    tick: u64,
    keypoint: usize,
    sweep: (Vec<Segment>, Option<(f32, TurnMode, u32)>, TurnMode),
}

#[derive(Component)]
struct ZoneMesh;

fn spawn(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<ColorMaterial>>) {
    commands.spawn((
        ZoneMesh,
        Mesh2d(meshes.add(zone_mesh(Vec::new(), Vec::new(), Vec::new()))),
        MeshMaterial2d(materials.add(ColorMaterial::default())),
        Transform::default(),
        Visibility::Hidden,
    ));
}

fn toggle(keys: Res<ButtonInput<KeyCode>>, mut zones: ResMut<DangerZones>) {
    if keys.just_pressed(KeyCode::KeyZ) {
        zones.enabled = !zones.enabled;
    }
}

#[allow(clippy::too_many_arguments)]
fn update(
    mut zones: ResMut<DangerZones>,
    app_state: Res<State<AppState>>,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    draft: Res<Draft>,
    plan: Res<PlanPreview>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut shown: Single<(&Mesh2d, &mut Visibility), With<ZoneMesh>>,
) {
    let (handle, visibility) = &mut *shown;
    if !zones.enabled || *app_state.get() != AppState::Paused || plan.preview.is_none() {
        **visibility = Visibility::Hidden;
        zones.built_for = None;
        return;
    }
    **visibility = Visibility::Inherited;

    let keypoint = draft.zone_keypoint();
    let key = Key { tick: sim.state.tick, keypoint, sweep: draft.sweep_key(keypoint, &tuning) };
    if zones.built_for.as_ref() == Some(&key) && !tuning.is_changed() {
        return;
    }
    let fan = draft.sweep(keypoint, &sim, &tuning, TURNS).unwrap_or_default();

    // Worst zone along each candidate segment so far: the color its path would have if
    // the keypoint ended there.
    let vt = &tuning.vehicle;
    let mut positions = Vec::new();
    let mut colors = Vec::new();
    let mut rows = Vec::with_capacity(fan.len());
    for frames in &fan {
        let first = positions.len() as u32;
        let mut worst = 0;
        for f in frames {
            worst = worst.max(zone(f.grip_demand, PlanPreview::would_slide(f, vt), vt));
            positions.push(f.pose.pos.extend(Z).to_array());
            colors.push(LinearRgba::from(ZONE_COLORS[worst].with_alpha(ALPHA)).to_f32_array());
        }
        rows.push((first, frames.len() as u32));
    }
    // Quads between neighboring turns at consecutive ticks, where both have them.
    let mut indices = Vec::new();
    for pair in rows.windows(2) {
        let ((a, na), (b, nb)) = (pair[0], pair[1]);
        for t in 0..na.min(nb).saturating_sub(1) {
            let (a0, a1, b0, b1) = (a + t, a + t + 1, b + t, b + t + 1);
            indices.extend_from_slice(&[a0, b0, b1, a0, b1, a1]);
        }
    }
    if let Some(mut mesh) = meshes.get_mut(&handle.0) {
        *mesh = zone_mesh(positions, colors, indices);
    }
    zones.built_for = Some(key);
}

fn zone_mesh(positions: Vec<[f32; 3]>, colors: Vec<[f32; 4]>, indices: Vec<u32>) -> Mesh {
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
        .with_inserted_indices(Indices::U32(indices))
}
