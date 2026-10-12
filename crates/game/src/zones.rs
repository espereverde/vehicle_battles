//! Danger zones: while paused, shades every place the selected keypoint (or a new one after
//! the last) can be dragged to, in the grip color its segment would end up with there:
//! green if the whole segment up to that point stays comfortable, yellow, orange
//! (understeer) or red (slide) otherwise. Z or the timeline checkbox toggles it.
//!
//! The map is the fit's own search space (`sim::sweep`): a fan of constant turns, one per
//! sampled turn value, meshed as quads between neighboring turns at consecutive ticks.
//!
//! Drift fans (X toggles): where the preview path is orange or red, the band between it and
//! the widest line the real car could take (`Preview::drift`), fading outward. A hint at
//! the consequences, not the real path.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy_egui::input::egui_wants_any_keyboard_input;
use sim::{Drift, Segment, TurnMode};

use crate::planning::{Draft, PlanEdit, PlanPreview};
use crate::render::{ZONE_COLORS, zone};
use crate::{AppState, Sim, TuningRes};

/// Turn values sampled across the fit's range.
const TURNS: usize = 81;
const ALPHA: f32 = 0.18;
/// Above the road (z = -1), below the gizmos.
const Z: f32 = -0.5;
const DRIFT_Z: f32 = -0.4;
/// Drift fan opacity at the planned path and at its outer bound.
const DRIFT_ALPHA: (f32, f32) = (0.35, 0.04);

pub struct ZonesPlugin;

impl Plugin for ZonesPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<DangerZones>().add_systems(Startup, spawn).add_systems(
            Update,
            (toggle.run_if(not(egui_wants_any_keyboard_input)), (update, update_drift).after(PlanEdit)).chain(),
        );
    }
}

#[derive(Resource)]
pub struct DangerZones {
    pub enabled: bool,
    pub drift: bool,
    /// What the current mesh was built for; `None` forces a rebuild.
    built_for: Option<Key>,
}

impl Default for DangerZones {
    fn default() -> Self {
        Self { enabled: true, drift: true, built_for: None }
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

#[derive(Component)]
struct DriftMesh;

fn spawn(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<ColorMaterial>>) {
    let material = materials.add(ColorMaterial::default());
    // Bevy 0.19 logs a use-after-free error for meshes without vertices, so start from a
    // degenerate, transparent triangle (never shown: the entities start hidden).
    let mut empty = || Mesh2d(meshes.add(zone_mesh(vec![[0.0; 3]; 3], vec![[0.0; 4]; 3], vec![0, 1, 2])));
    commands.spawn((ZoneMesh, empty(), MeshMaterial2d(material.clone()), Transform::default(), Visibility::Hidden));
    commands.spawn((DriftMesh, empty(), MeshMaterial2d(material), Transform::default(), Visibility::Hidden));
}

fn toggle(keys: Res<ButtonInput<KeyCode>>, mut zones: ResMut<DangerZones>) {
    if keys.just_pressed(KeyCode::KeyZ) {
        zones.enabled = !zones.enabled;
    }
    if keys.just_pressed(KeyCode::KeyX) {
        zones.drift = !zones.drift;
    }
}

/// Rebuilds the drift fans whenever the preview changes (paused or running).
fn update_drift(
    zones: Res<DangerZones>,
    sim: Res<Sim>,
    tuning: Res<TuningRes>,
    plan: Res<PlanPreview>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut shown: Single<(&Mesh2d, &mut Visibility), With<DriftMesh>>,
) {
    let (handle, visibility) = &mut *shown;
    let Some(preview) = plan.preview.as_ref().filter(|_| zones.drift) else {
        **visibility = Visibility::Hidden;
        return;
    };
    if !(plan.is_changed() || zones.is_changed() || tuning.is_changed()) {
        return;
    }
    let recover = (tuning.planner.drift_recover_seconds * tuning.tick_rate as f32).round() as usize;
    let drift = preview.drift(sim.player, &tuning.vehicle, tuning.dt(), recover);

    // Per tick: the planned point and the outer bound, colored by why it drifts (recovery
    // keeps the color of the drift before it); quads between consecutive ticks.
    let (mut positions, mut colors, mut indices) = (Vec::new(), Vec::new(), Vec::new());
    let mut last_kind = Drift::Understeer;
    let mut prev_on = false;
    for (i, d) in drift.iter().enumerate() {
        let (Some((outer, kind)), Some(f)) = (d, plan.player_frame(i)) else {
            prev_on = false;
            continue;
        };
        if *kind != Drift::Recover {
            last_kind = *kind;
        }
        let color = if last_kind == Drift::Slide { ZONE_COLORS[3] } else { ZONE_COLORS[2] };
        let n = positions.len() as u32;
        positions.push(f.pose.pos.extend(DRIFT_Z).to_array());
        positions.push(outer.extend(DRIFT_Z).to_array());
        colors.push(LinearRgba::from(color.with_alpha(DRIFT_ALPHA.0)).to_f32_array());
        colors.push(LinearRgba::from(color.with_alpha(DRIFT_ALPHA.1)).to_f32_array());
        if prev_on {
            let (p0, o0, p1, o1) = (n - 2, n - 1, n, n + 1);
            indices.extend_from_slice(&[p0, o0, o1, p0, o1, p1]);
        }
        prev_on = true;
    }
    show(&mut meshes, &handle.0, visibility, positions, colors, indices);
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
    show(&mut meshes, &handle.0, visibility, positions, colors, indices);
    zones.built_for = Some(key);
}

/// Replaces the mesh behind `handle` and shows it, or just hides it when there is nothing
/// to draw (never uploads an empty mesh, see `spawn`).
fn show(
    meshes: &mut Assets<Mesh>,
    handle: &Handle<Mesh>,
    visibility: &mut Visibility,
    positions: Vec<[f32; 3]>,
    colors: Vec<[f32; 4]>,
    indices: Vec<u32>,
) {
    if indices.is_empty() {
        *visibility = Visibility::Hidden;
        return;
    }
    if let Some(mut mesh) = meshes.get_mut(handle) {
        *mesh = zone_mesh(positions, colors, indices);
    }
    *visibility = Visibility::Inherited;
}

fn zone_mesh(positions: Vec<[f32; 3]>, colors: Vec<[f32; 4]>, indices: Vec<u32>) -> Mesh {
    Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_attribute(Mesh::ATTRIBUTE_COLOR, colors)
        .with_inserted_indices(Indices::U32(indices))
}
