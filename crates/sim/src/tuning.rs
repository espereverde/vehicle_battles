use serde::{Deserialize, Serialize};

/// All feel parameters. Loaded from `assets/tuning.ron` (hot-reloaded by the game);
/// `Default` mirrors that file so tests run without it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct Tuning {
    pub tick_rate: u32,
    /// Seconds simulated ahead for the player's own preview.
    pub preview_horizon: f32,
    pub vehicle: VehicleTuning,
    pub control: ControlTuning,
    pub planner: PlannerTuning,
    pub keyboard: KeyboardTuning,
}

impl Tuning {
    pub fn dt(&self) -> f32 {
        1.0 / self.tick_rate as f32
    }

    pub fn preview_ticks(&self) -> u32 {
        (self.preview_horizon * self.tick_rate as f32).round() as u32
    }
}

impl Default for Tuning {
    fn default() -> Self {
        Self {
            tick_rate: 60,
            preview_horizon: 6.0,
            vehicle: VehicleTuning::default(),
            control: ControlTuning::default(),
            planner: PlannerTuning::default(),
            keyboard: KeyboardTuning::default(),
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct VehicleTuning {
    pub length: f32,
    pub width: f32,
    pub wheelbase: f32,
    pub max_steer_deg: f32,
    /// How fast the front wheels turn, degrees per second.
    pub steer_rate_deg: f32,
    /// Full-throttle acceleration at standstill; fades linearly to zero at `top_speed`.
    pub engine_accel: f32,
    pub top_speed: f32,
    pub reverse_accel: f32,
    pub reverse_top_speed: f32,
    pub brake_decel: f32,
    pub rolling_resistance: f32,
    /// Quadratic drag, (m/s²) per (m/s)².
    pub drag: f32,
    pub gravity: f32,
    /// Static friction coefficient: radius of the friction circle is `grip · gravity`.
    pub grip: f32,
    /// Grip demand up to which driving feels safe; between this and 1.0 is the yellow zone.
    pub comfort_demand: f32,
    /// Grip demand (with grip) above which the car breaks into a slide. Between 1.0 and
    /// this the car understeers instead.
    pub slide_threshold: f32,
    /// Deceleration while understeering, in g per unit of demand over 1.0 (capped at 1.0).
    pub understeer_scrub: f32,
    /// Below this speed (m/s) the slide threshold grows as `slide_ref_speed / speed`.
    pub slide_ref_speed: f32,
    /// Kinetic friction while sliding (lower than `grip`).
    pub slide_grip: f32,
    /// Fraction of the kinematic yaw rate the steering can still command while sliding.
    pub slide_steer_authority: f32,
    /// Time constant (s) of yaw-rate relaxation while sliding; larger = more yaw drift.
    pub slide_yaw_tau: f32,
    /// Grip is regained once sideways speed, demand and yaw error all fall below these.
    pub slide_exit_lat_speed: f32,
    pub slide_exit_demand: f32,
    pub slide_exit_yaw_error_deg: f32,
    pub handbrake_decel: f32,
    /// Multiplier on `slide_grip` while the handbrake is held.
    pub handbrake_grip: f32,
    /// Multiplier on the kinematic yaw rate while the handbrake is held (rear steps out).
    pub handbrake_yaw_gain: f32,
    /// Below this speed the handbrake just stops the car.
    pub handbrake_min_speed: f32,
}

impl Default for VehicleTuning {
    fn default() -> Self {
        Self {
            length: 4.5,
            width: 1.9,
            wheelbase: 2.7,
            max_steer_deg: 35.0,
            steer_rate_deg: 120.0,
            engine_accel: 8.0,
            top_speed: 42.0,
            reverse_accel: 3.0,
            reverse_top_speed: 8.0,
            brake_decel: 10.0,
            rolling_resistance: 0.3,
            drag: 0.0004,
            gravity: 9.81,
            grip: 1.5,
            comfort_demand: 0.7,
            slide_threshold: 1.2,
            understeer_scrub: 0.3,
            slide_ref_speed: 15.0,
            slide_grip: 0.75,
            slide_steer_authority: 0.35,
            slide_yaw_tau: 0.5,
            slide_exit_lat_speed: 1.0,
            slide_exit_demand: 0.85,
            slide_exit_yaw_error_deg: 15.0,
            handbrake_decel: 3.0,
            handbrake_grip: 0.6,
            handbrake_yaw_gain: 1.8,
            handbrake_min_speed: 3.0,
        }
    }
}

/// Path follower and speed controller.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct ControlTuning {
    /// Pure-pursuit lookahead distance: `lookahead_base + lookahead_per_speed · speed` (m).
    pub lookahead_base: f32,
    pub lookahead_per_speed: f32,
    /// Speed controller gain, (m/s²) per (m/s) of speed error.
    pub speed_gain: f32,
    /// Acceleration / deceleration used by speed profiles (m/s²).
    pub plan_accel: f32,
    pub plan_decel: f32,
}

impl Default for ControlTuning {
    fn default() -> Self {
        Self { lookahead_base: 3.0, lookahead_per_speed: 0.3, speed_gain: 2.0, plan_accel: 4.0, plan_decel: 6.0 }
    }
}

/// Path fitting and drag-tool defaults.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PlannerTuning {
    /// Bézier handle length as a fraction of the chord: `handle_base + handle_turn · turn/π`.
    pub handle_base: f32,
    pub handle_turn: f32,
    /// Distance between resampled path points (m).
    pub path_spacing: f32,
    /// Cruise speed of a new segment that starts from (near) standstill (m/s).
    pub default_cruise: f32,
}

impl Default for PlannerTuning {
    fn default() -> Self {
        Self { handle_base: 0.33, handle_turn: 0.25, path_spacing: 0.25, default_cruise: 12.0 }
    }
}

/// Debug keyboard driving assist (used by the game only; removed once orders exist).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct KeyboardTuning {
    /// Holding a steer key builds input from 0 to 1 at this rate (per second).
    pub steer_ramp_per_s: f32,
    /// Releasing (or reversing) returns input towards 0 at this rate.
    pub steer_return_per_s: f32,
    /// Full input asks for this lateral grip demand (capped by the steering lock).
    pub full_steer_demand: f32,
    /// Same, with Ctrl held: deliberately beyond the grip limit.
    pub push_steer_demand: f32,
}

impl Default for KeyboardTuning {
    fn default() -> Self {
        Self {
            steer_ramp_per_s: 2.0,
            steer_return_per_s: 5.0,
            full_steer_demand: 0.9,
            push_steer_demand: 3.0,
        }
    }
}
