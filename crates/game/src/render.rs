//! Gizmo rendering of the simulation, the follow / zoom / pan camera, and drawing helpers
//! shared by the planning tools.

use std::collections::VecDeque;

use bevy::camera::ScalingMode;
use bevy::color::palettes::css;
use bevy::ecs::system::SystemParam;
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_egui::egui;
use bevy_egui::input::egui_wants_any_pointer_input;
use sim::{Car, Pose, SimState, SlideKind, VehicleTuning};

use crate::{Sim, TuningRes};

pub struct RenderPlugin;

impl Plugin for RenderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<Trails>()
            .init_resource::<CameraFollow>()
            .insert_resource(ClearColor(Color::srgb(0.09, 0.10, 0.11)))
            .add_systems(Startup, spawn_camera)
            .add_systems(
                Update,
                (
                    (zoom_camera, pan_camera).run_if(not(egui_wants_any_pointer_input)),
                    follow_camera,
                    draw_ground,
                    draw_trails,
                    draw_cars,
                )
                    .chain(),
            );
    }
}

const TRAIL_LEN: usize = 600;

/// Whether the car was risking a loss of control at a trail point.
#[derive(Clone, Copy)]
enum Traction {
    Calm,
    Hard,
}

impl Traction {
    fn of(car: &Car) -> Self {
        if car.risk > 0.0 { Self::Hard } else { Self::Calm }
    }

    fn trail_color(self) -> Color {
        match self {
            Self::Calm => css::WHITE.with_alpha(0.25),
            Self::Hard => css::ORANGE.with_alpha(0.8),
        }
        .into()
    }
}

/// Recent positions per car (index = position in `SimState::cars`) with their risk.
#[derive(Resource, Default)]
pub struct Trails(Vec<VecDeque<(Vec2, Traction)>>);

impl Trails {
    pub fn record(&mut self, state: &SimState) {
        self.0.resize_with(state.cars.len(), VecDeque::new);
        for (trail, car) in self.0.iter_mut().zip(&state.cars) {
            if trail.len() == TRAIL_LEN {
                trail.pop_front();
            }
            trail.push_back((car.pose.pos, Traction::of(car)));
        }
    }

    pub fn clear(&mut self) {
        self.0.clear();
    }
}

/// Risk zone colors (`sim::handling::zone`): green no risk, yellow no risk but maneuvering
/// would carry some, orange risk, red certain loss of control.
pub const ZONE_COLORS: [Srgba; 4] = [css::LIME, css::YELLOW, css::ORANGE, css::RED];

pub fn zone_color(zone: usize) -> Color {
    ZONE_COLORS[zone.min(3)].into()
}

pub fn to_egui(c: Color) -> egui::Color32 {
    let s = c.to_srgba();
    egui::Color32::from_rgba_unmultiplied((s.red * 255.0) as u8, (s.green * 255.0) as u8, (s.blue * 255.0) as u8, (s.alpha * 255.0) as u8)
}

pub fn car_outline(gizmos: &mut Gizmos, pose: Pose, vt: &VehicleTuning, color: impl Into<Color>) {
    let iso = Isometry2d::new(pose.pos, Rot2::radians(pose.heading));
    gizmos.rect_2d(iso, Vec2::new(vt.length, vt.width), color);
}

#[derive(Resource)]
pub struct CameraFollow(pub bool);

impl Default for CameraFollow {
    fn default() -> Self {
        Self(true)
    }
}

/// Ctrl+wheel zooms; the plain wheel belongs to the planning tools.
pub fn zoom_modifier(keys: &ButtonInput<KeyCode>) -> bool {
    keys.any_pressed([KeyCode::ControlLeft, KeyCode::ControlRight])
}

/// Camera access for world ↔ screen conversions.
#[derive(SystemParam)]
pub struct WorldView<'w, 's> {
    camera: Single<'w, 's, (&'static Camera, &'static GlobalTransform)>,
    window: Single<'w, 's, &'static Window, With<PrimaryWindow>>,
}

impl WorldView<'_, '_> {
    /// World position under the mouse cursor.
    pub fn cursor(&self) -> Option<Vec2> {
        let (camera, gt) = *self.camera;
        camera.viewport_to_world_2d(gt, self.window.cursor_position()?).ok()
    }

    /// Screen position (egui points) of a world position.
    pub fn to_screen(&self, p: Vec2) -> Option<egui::Pos2> {
        let (camera, gt) = *self.camera;
        let v = camera.world_to_viewport(gt, p.extend(0.0)).ok()?;
        Some(egui::pos2(v.x, v.y))
    }

    /// World meters covered by one screen pixel.
    pub fn meters_per_pixel(&self) -> f32 {
        let (camera, gt) = *self.camera;
        let a = camera.viewport_to_world_2d(gt, Vec2::ZERO);
        let b = camera.viewport_to_world_2d(gt, Vec2::new(0.0, 100.0));
        match (a, b) {
            (Ok(a), Ok(b)) => a.distance(b) / 100.0,
            _ => 0.1,
        }
    }
}

/// Visible height of the world in meters at zoom 1.
const VIEW_HEIGHT_M: f32 = 80.0;

fn spawn_camera(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: ScalingMode::FixedVertical { viewport_height: VIEW_HEIGHT_M },
            ..OrthographicProjection::default_2d()
        }),
    ));
}

fn zoom_camera(scroll: Res<AccumulatedMouseScroll>, keys: Res<ButtonInput<KeyCode>>, mut projection: Single<&mut Projection>) {
    if scroll.delta.y == 0.0 || !zoom_modifier(&keys) {
        return;
    }
    if let Projection::Orthographic(ortho) = projection.as_mut() {
        ortho.scale = (ortho.scale * 1.1f32.powf(-scroll.delta.y.signum())).clamp(0.1, 10.0);
    }
}

/// Middle-drag pans the view (and stops following the car).
fn pan_camera(
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    view: WorldView,
    mut follow: ResMut<CameraFollow>,
    mut camera: Single<&mut Transform, With<Camera2d>>,
) {
    if !buttons.pressed(MouseButton::Middle) || motion.delta == Vec2::ZERO {
        return;
    }
    follow.0 = false;
    let m = view.meters_per_pixel();
    camera.translation.x -= motion.delta.x * m;
    camera.translation.y += motion.delta.y * m;
}

fn follow_camera(
    keys: Res<ButtonInput<KeyCode>>,
    mut follow: ResMut<CameraFollow>,
    sim: Res<Sim>,
    mut camera: Single<&mut Transform, With<Camera2d>>,
) {
    if keys.just_pressed(KeyCode::KeyF) {
        follow.0 = !follow.0;
    }
    if follow.0 {
        camera.translation = sim.player_car().pose.pos.extend(camera.translation.z);
    }
}

fn draw_ground(mut gizmos: Gizmos, camera: Single<&Transform, With<Camera2d>>) {
    // A grid that scrolls with the camera, snapped to 50 m so it looks fixed to the ground.
    let center = (camera.translation.truncate() / 50.0).round() * 50.0;
    gizmos.grid_2d(Isometry2d::from_translation(center), UVec2::splat(80), Vec2::splat(5.0), Color::srgb(0.16, 0.17, 0.19));
    gizmos.grid_2d(Isometry2d::from_translation(center), UVec2::splat(8), Vec2::splat(50.0), Color::srgb(0.24, 0.25, 0.28));
    gizmos.circle_2d(Isometry2d::IDENTITY, 1.0, css::GRAY);
}

fn draw_trails(mut gizmos: Gizmos, trails: Res<Trails>) {
    for trail in &trails.0 {
        for ((a, _), (b, traction)) in trail.iter().zip(trail.iter().skip(1)) {
            gizmos.line_2d(*a, *b, traction.trail_color());
        }
    }
}

fn draw_cars(mut gizmos: Gizmos, sim: Res<Sim>, tuning: Res<TuningRes>) {
    let vt = &tuning.vehicle;
    for car in &sim.state.cars {
        // Sliding shows in the body color: controlled skid cyan, skid/T-stop/fishtail orange,
        // spinout/roll red (a roll crossed out), in the air white; burning gets a ring.
        let body = match car.slide.map(|s| s.kind) {
            None => css::LIME,
            Some(SlideKind::Controlled) => css::AQUA,
            Some(SlideKind::Skid | SlideKind::TStop | SlideKind::Fishtail) => css::ORANGE,
            Some(SlideKind::Spinout | SlideKind::Roll) => css::RED,
            Some(SlideKind::Airborne) => css::WHITE,
        };
        car_outline(&mut gizmos, car.pose, vt, body);
        if let Some(s) = car.slide {
            if s.kind == SlideKind::Roll {
                let (f, l) = (car.pose.forward() * vt.length * 0.5, car.pose.left() * vt.width * 0.5);
                gizmos.line_2d(car.pose.pos - f - l, car.pose.pos + f + l, css::RED);
                gizmos.line_2d(car.pose.pos - f + l, car.pose.pos + f - l, css::RED);
            }
            if s.burning {
                gizmos.circle_2d(Isometry2d::from_translation(car.pose.pos), vt.length * 0.7, css::ORANGE_RED);
            }
        }

        // Front wheels at their steer angle.
        let fwd = car.pose.forward();
        let left = car.pose.left();
        let front_axle = car.pose.pos + fwd * vt.wheelbase * 0.5;
        let wheel = Vec2::from_angle(car.pose.heading + car.steer_angle) * 0.4;
        for side in [-1.0, 1.0] {
            let c = front_axle + left * side * vt.width * 0.4;
            gizmos.line_2d(c - wheel, c + wheel, css::WHITE);
        }

        gizmos.arrow_2d(car.pose.pos, car.pose.pos + car.vel * 0.5, css::SKY_BLUE);
        draw_handling(&mut gizmos, car, &tuning);
    }
}

/// Handling status under the car: a bar from the floor (−6) to the handling class, in the
/// car's risk zone color; the tick marks 0.
fn draw_handling(gizmos: &mut Gizmos, car: &Car, tuning: &sim::Tuning) {
    const W: f32 = 4.0;
    let (lo, hi) = (tuning.handling.min_status, sim::handling::class(tuning));
    let frac = |hs: f32| ((hs - lo) / (hi - lo).max(1e-3)).clamp(0.0, 1.0);
    let hs = sim::handling::status(car, tuning);
    let left = car.pose.pos - Vec2::new(W * 0.5, tuning.vehicle.length + 0.5);
    let color = zone_color(sim::CarFrame::of(car).zone(tuning));
    gizmos.line_2d(left, left + Vec2::X * W, css::DIM_GRAY);
    gizmos.line_2d(left, left + Vec2::X * (W * frac(hs)).max(0.05), color);
    let mark = left + Vec2::X * W * frac(0.0);
    gizmos.line_2d(mark - Vec2::Y * 0.3, mark + Vec2::Y * 0.3, css::GRAY);
}
