//! Test track: drawn at true scale from `assets/maps/track.ron`, with lap timing and an
//! off-track counter. C switches between the track and the parking challenge.
//! Leaving the road has no physical effect yet (terrain arrives in M3).

use std::path::PathBuf;

use bevy::asset::RenderAssetUsages;
use bevy::color::palettes::css;
use bevy::mesh::{Indices, PrimitiveTopology};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use sim::{CarId, SimState, Track, TrackSpec};

use crate::{Sim, TuningRes};

pub struct TrackPlugin;

impl Plugin for TrackPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Mode>()
            .insert_resource(load())
            .init_resource::<TrackRun>()
            .add_systems(Startup, spawn_road)
            .add_systems(Update, (show_road, draw_track.run_if(track_mode)))
            .add_systems(EguiPrimaryContextPass, track_panel.run_if(resource_equals(Mode::Track)));
    }
}

/// Which test scenario is active.
#[derive(Resource, Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Mode {
    #[default]
    Track,
    Parking,
    Slalom,
}

impl Mode {
    /// The next mode for the C key.
    pub fn next(self) -> Self {
        match self {
            Mode::Track => Mode::Parking,
            Mode::Parking => Mode::Slalom,
            Mode::Slalom => Mode::Track,
        }
    }
}

pub fn track_mode(mode: Res<Mode>, track: Res<TrackRes>) -> bool {
    *mode == Mode::Track && track.track.is_some()
}


#[derive(Resource)]
pub struct TrackRes {
    pub track: Option<Track>,
    pub path: PathBuf,
    pub error: Option<String>,
}

fn track_path() -> PathBuf {
    let cwd = PathBuf::from("assets/maps/track.ron");
    if cwd.exists() {
        return cwd;
    }
    PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/maps/track.ron"))
}

fn load() -> TrackRes {
    let path = track_path();
    let result = std::fs::read_to_string(&path)
        .map_err(|e| format!("{}: {e}", path.display()))
        .and_then(|text| ron::from_str::<TrackSpec>(&text).map_err(|e| e.to_string()));
    match result {
        Ok(spec) => {
            let track = Track::from_spec(&spec);
            if track.closure_gap > 1.0 && track.closure_heading_deg.abs() > 1.0 {
                warn!("track does not close: {:.1} m / {:.1}° off; joined with a straight", track.closure_gap, track.closure_heading_deg);
            }
            TrackRes { track: Some(track), path, error: None }
        }
        Err(e) => {
            error!("track: {e}");
            TrackRes { track: None, path, error: Some(e) }
        }
    }
}

/// Lap timing and off-track counting for the player car.
#[derive(Resource, Default)]
pub struct TrackRun {
    s: Option<f32>,
    halfway: bool,
    lap_start: u64,
    /// Completed lap times, in ticks.
    pub laps: Vec<u64>,
    pub best: Option<u64>,
    pub off: bool,
    pub off_count: u32,
    pub off_ticks: u64,
}

impl TrackRun {
    pub fn update(&mut self, state: &SimState, player: CarId, track: &Track) {
        let Some(car) = state.car(player) else { return };
        let p = track.project(car.pose.pos);
        let off = p.d.abs() > track.width * 0.5;
        if off && !self.off {
            self.off_count += 1;
        }
        self.off = off;
        self.off_ticks += u64::from(off);

        // A lap counts when the car crosses the start line forwards after passing halfway.
        let l = track.length;
        if (0.4 * l..0.6 * l).contains(&p.s) {
            self.halfway = true;
        }
        if let Some(prev) = self.s
            && prev > 0.75 * l
            && p.s < 0.25 * l
            && self.halfway
        {
            let lap = state.tick - self.lap_start;
            self.laps.push(lap);
            self.best = Some(self.best.map_or(lap, |b| b.min(lap)));
            self.lap_start = state.tick;
            self.halfway = false;
        }
        self.s = Some(p.s);
    }

    /// Starts over, keeping the best lap.
    pub fn restart(&mut self) {
        *self = Self { best: self.best, ..Self::default() };
    }
}

#[derive(Component)]
struct Road;

fn spawn_road(
    mut commands: Commands,
    track: Res<TrackRes>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ColorMaterial>>,
) {
    let Some(track) = &track.track else { return };
    // Triangle strip between the two road edges, as a list.
    let n = track.points.len();
    let half = track.width * 0.5;
    let mut positions = Vec::with_capacity(2 * n);
    for i in 0..n {
        let prev = track.points[(i + n - 1) % n];
        let next = track.points[(i + 1) % n];
        let normal = (next - prev).normalize_or(Vec2::X).perp();
        let p = track.points[i];
        positions.push((p + normal * half).extend(-1.0).to_array());
        positions.push((p - normal * half).extend(-1.0).to_array());
    }
    let mut indices = Vec::with_capacity(6 * n);
    for i in 0..n as u32 {
        let j = (i + 1) % n as u32;
        let (l0, r0, l1, r1) = (2 * i, 2 * i + 1, 2 * j, 2 * j + 1);
        indices.extend_from_slice(&[l0, r0, r1, l0, r1, l1]);
    }
    let mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default())
        .with_inserted_attribute(Mesh::ATTRIBUTE_POSITION, positions)
        .with_inserted_indices(Indices::U32(indices));
    commands.spawn((
        Road,
        Mesh2d(meshes.add(mesh)),
        MeshMaterial2d(materials.add(Color::srgb(0.19, 0.20, 0.22))),
        Transform::default(),
    ));
}

fn show_road(mode: Res<Mode>, mut road: Query<&mut Visibility, With<Road>>) {
    if !mode.is_changed() {
        return;
    }
    for mut v in &mut road {
        *v = if *mode == Mode::Track { Visibility::Inherited } else { Visibility::Hidden };
    }
}

fn draw_track(mut gizmos: Gizmos, track: Res<TrackRes>) {
    let Some(track) = &track.track else { return };
    let n = track.points.len();
    let half = track.width * 0.5;
    let edge = |side: f32| {
        (0..=n).map(move |i| {
            let i = i % n;
            let prev = track.points[(i + n - 1) % n];
            let next = track.points[(i + 1) % n];
            track.points[i] + (next - prev).normalize_or(Vec2::X).perp() * half * side
        })
    };
    gizmos.linestrip_2d(edge(1.0), css::WHITE.with_alpha(0.8));
    gizmos.linestrip_2d(edge(-1.0), css::WHITE.with_alpha(0.8));
    // Dashed centerline: 3 m dash every 9 m.
    let mut s = 0.0;
    while s < track.length {
        let (a, _) = track.frame_at(s);
        let (b, _) = track.frame_at(s + 3.0);
        gizmos.line_2d(a, b, css::WHITE.with_alpha(0.25));
        s += 9.0;
    }
    // Checkered start / finish line.
    let (p, t) = track.frame_at(0.0);
    let across = t.perp();
    let squares = 8;
    let size = track.width / squares as f32;
    for i in 0..squares {
        for row in 0..2 {
            if (i + row) % 2 == 0 {
                let c = p + across * (-half + size * (i as f32 + 0.5)) + t * size * (row as f32 - 0.5);
                let iso = Isometry2d::new(c, Rot2::radians(t.to_angle()));
                gizmos.rect_2d(iso, Vec2::splat(size * 0.9), css::WHITE);
            }
        }
    }
}

fn track_panel(mut contexts: EguiContexts, track: Res<TrackRes>, run: Res<TrackRun>, sim: Res<Sim>, tuning: Res<TuningRes>) -> Result {
    let ctx = contexts.ctx_mut()?;
    let Some(t) = &track.track else {
        let error = track.error.clone().unwrap_or_default();
        egui::Window::new("Track").anchor(egui::Align2::RIGHT_TOP, [-12.0, 12.0]).show(ctx, |ui| {
            ui.colored_label(egui::Color32::LIGHT_RED, format!("no track loaded from {}:\n{error}", track.path.display()));
        });
        return Ok(());
    };
    let rate = tuning.tick_rate as f32;
    egui::Window::new("Track").anchor(egui::Align2::RIGHT_TOP, [-12.0, 12.0]).resizable(false).show(ctx, |ui| {
        ui.label(format!("{} — {:.0} m, {:.0} m wide", t.name, t.length, t.width));
        let current = sim.state.tick - run.lap_start;
        ui.heading(format!("lap {}: {:.1} s", run.laps.len() + 1, current as f32 / rate));
        if let Some(best) = run.best {
            ui.label(format!("best: {:.2} s", best as f32 / rate));
        }
        for (i, lap) in run.laps.iter().enumerate().rev().take(5) {
            ui.small(format!("lap {}: {:.2} s", i + 1, *lap as f32 / rate));
        }
        let off = format!("off track: {}× ({:.1} s)", run.off_count, run.off_ticks as f32 / rate);
        if run.off {
            ui.colored_label(egui::Color32::from_rgb(255, 90, 200), format!("OFF TRACK — {off}"));
        } else {
            ui.label(off);
        }
        if t.closure_gap > 1.0 && t.closure_heading_deg.abs() > 1.0 {
            ui.colored_label(egui::Color32::LIGHT_RED, format!("track doesn't close ({:.1} m)", t.closure_gap));
        }
        ui.small("R restart · C next mode");
    });
    Ok(())
}
