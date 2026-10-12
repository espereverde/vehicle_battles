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
    pub stability: StabilityTuning,
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
            preview_horizon: 5.0,
            vehicle: VehicleTuning::default(),
            stability: StabilityTuning::default(),
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
    /// Deceleration while understeering, in g per unit of demand over 1.0 (capped at 1.0).
    pub understeer_scrub: f32,
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
            understeer_scrub: 0.3,
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

/// Stability gauge and loss of control (see `stability.rs`). Stability runs from 1 (calm)
/// to 0; demand is the car's grip demand and comfort is `VehicleTuning::comfort_demand`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct StabilityTuning {
    /// Drain per second: `k_drain · (demand − comfort)²` while above comfort.
    pub k_drain: f32,
    /// Drain per unit of grip demand change: harsh transitions (swerves) cost stability.
    pub k_jerk: f32,
    /// Recovery per second at skill 1.0 (`k_rec · skill`) while below comfort.
    pub k_rec: f32,
    /// Below this stability the car is in the yellow zone: risk checks roll for a loss of
    /// control. At 0 the loss of control is certain.
    pub yellow: f32,
    /// Seconds between risk checks while in the yellow zone and above comfort.
    pub risk_interval: f32,
    /// Chance per check at stability → 0; it grows as `risk_max · (1 − s / yellow)²`.
    pub risk_max: f32,
    /// No loss of control below this speed (m/s): a slow car just understeers.
    pub min_speed: f32,
    /// Spin vs skid: below `skid_speed` a loss of control is always a skid, above
    /// `spin_speed` always a spin, linear in between.
    pub skid_speed: f32,
    pub spin_speed: f32,
    /// Yaw rate kick (rad/s, into the turn) when the rear lets go.
    pub skid_yaw_kick: f32,
    pub spin_yaw_kick: f32,
    /// Stability right after a loss of control (frozen while sliding).
    pub after_loss: f32,
}

impl Default for StabilityTuning {
    fn default() -> Self {
        Self {
            k_drain: 3.0,
            k_jerk: 0.05,
            k_rec: 0.3,
            yellow: 0.6,
            risk_interval: 0.25,
            risk_max: 0.35,
            min_speed: 10.0,
            skid_speed: 12.0,
            spin_speed: 25.0,
            skid_yaw_kick: 1.0,
            spin_yaw_kick: 5.0,
            after_loss: 0.3,
        }
    }
}

/// Planning tool defaults.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct PlannerTuning {
    /// Throttle of a new keypoint when the car would otherwise sit still.
    pub default_throttle: f32,
    /// Throttle change per mouse-wheel notch.
    pub throttle_step: f32,
    /// Tightest turn the drag tool plans (± g). In steering mode it caps steering at the
    /// segment's start speed (never past full lock).
    pub max_lateral_g: f32,
    /// Length of a keypoint added with the "straight" button (s).
    pub straight_seconds: f32,
    /// Drift fans: how long (s) the outer bound keeps its offset after the plan is back
    /// within grip.
    pub drift_recover_seconds: f32,
}

impl Default for PlannerTuning {
    fn default() -> Self {
        Self { default_throttle: 0.4, throttle_step: 0.05, max_lateral_g: 3.0, straight_seconds: 1.0, drift_recover_seconds: 0.5 }
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
