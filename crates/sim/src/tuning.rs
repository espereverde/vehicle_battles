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
    pub handling: HandlingTuning,
    pub crash: CrashTuning,
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
            handling: HandlingTuning::default(),
            crash: CrashTuning::default(),
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
    /// Car Wars handling class: starting and highest handling status, and (plus driver
    /// skill) handling points recovered per second.
    pub handling_class: f32,
    /// Extra deceleration while the handbrake is held.
    pub handbrake_decel: f32,
}

impl Default for VehicleTuning {
    fn default() -> Self {
        Self {
            length: 4.5,
            width: 1.9,
            wheelbase: 2.7,
            max_steer_deg: 35.0,
            engine_accel: 8.0,
            top_speed: 42.0,
            reverse_accel: 3.0,
            reverse_top_speed: 8.0,
            brake_decel: 10.0,
            rolling_resistance: 0.3,
            drag: 0.0004,
            gravity: 9.81,
            handling_class: 3.0,
            handbrake_decel: 3.0,
        }
    }
}

/// Handling status and control checks (see `handling.rs`). Speeds in the Control Table are
/// in mph, as in the book.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct HandlingTuning {
    /// Lowest handling status.
    pub min_status: f32,
    /// Control Table: one roll fails with `clamp((v − safe_speed_mph − mph_per_status · HS)
    /// / fail_range_mph, 0, 1)`. Fitted to the book's table.
    pub safe_speed_mph: f32,
    pub mph_per_status: f32,
    pub fail_range_mph: f32,
    /// Heading change that drains one handling point (a D1 bend).
    pub bend_deg: f32,
    /// Handling points per inch forward of a drift at ¼" sideways per inch (D1), and of a
    /// steep drift at ½" (D3); linear in between, and from 0 below ¼".
    pub drift_cost: f32,
    pub steep_drift_cost: f32,
    /// Braking up to this deceleration (mph per second) drains nothing.
    pub free_braking_mph_s: f32,
}

impl Default for HandlingTuning {
    fn default() -> Self {
        Self {
            min_status: -6.0,
            safe_speed_mph: 106.0,
            mph_per_status: 27.0,
            fail_range_mph: 130.0,
            bend_deg: 15.0,
            drift_cost: 1.0,
            steep_drift_cost: 3.0,
            free_braking_mph_s: 10.0,
        }
    }
}

/// Crash results and controlled skids (see `crash.rs`, `orders::Turn::Skid`). Book values.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CrashTuning {
    /// Spinout rotation: 90° per phase (0.2 s).
    pub spin_deg_per_s: f32,
    /// Spinout and roll deceleration.
    pub decel_mph_s: f32,
    /// T-stop deceleration per inch slid.
    pub tstop_mph_per_inch: f32,
    /// Seconds between spinout recovery attempts.
    pub recovery_seconds: f32,
    /// Below this speed (m/s) a spinout, T-stop or roll has stopped.
    pub stop_speed: f32,
    /// Controlled skid: handling points added per book inch of slide (¼" → 1, 1" → 4).
    pub skid_cost_per_inch: f32,
    /// Controlled skid: seconds of sliding per book inch of slide (¼" → 0.25 s at 1.0).
    pub skid_seconds_per_inch: f32,
    /// Controlled skid: the nose swings round over the slide's first seconds.
    pub skid_turn_seconds: f32,
    /// Controlled skid: sharpest swing of the nose (degrees).
    pub max_skid_deg: f32,
    /// Easing (`ease.rs`): a slide's direction of travel swings over from the car's in this
    /// long, and back onto the nose after it (regrip).
    pub slide_entry_seconds: f32,
    pub regrip_seconds: f32,
    /// A spinout's rotation builds up (from the car's turn) over this long, and a T-stop's
    /// dies down; it winds down below this speed (m/s).
    pub spin_up_seconds: f32,
    pub spin_fade_speed: f32,
    /// A roll turns the car sideways, and a fishtail swings its rear, over this long.
    pub roll_turn_seconds: f32,
    pub fishtail_seconds: f32,
}

impl Default for CrashTuning {
    fn default() -> Self {
        Self {
            spin_deg_per_s: 450.0,
            decel_mph_s: 20.0,
            tstop_mph_per_inch: 20.0,
            recovery_seconds: 1.0,
            stop_speed: 0.3,
            skid_cost_per_inch: 4.0,
            skid_seconds_per_inch: 1.0,
            skid_turn_seconds: 0.25,
            max_skid_deg: 90.0,
            slide_entry_seconds: 0.15,
            regrip_seconds: 0.35,
            spin_up_seconds: 0.4,
            spin_fade_speed: 6.0,
            roll_turn_seconds: 0.3,
            fishtail_seconds: 0.25,
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
    /// Length of a keypoint added with the "straight" button (s).
    pub straight_seconds: f32,
}

impl Default for PlannerTuning {
    fn default() -> Self {
        Self { default_throttle: 0.4, throttle_step: 0.05, straight_seconds: 1.0 }
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
    /// Full input asks for this lateral acceleration in g (capped by the steering lock).
    pub full_steer_g: f32,
    /// Same, with Ctrl held.
    pub push_steer_g: f32,
}

impl Default for KeyboardTuning {
    fn default() -> Self {
        Self {
            steer_ramp_per_s: 2.0,
            steer_return_per_s: 5.0,
            full_steer_g: 1.35,
            push_steer_g: 4.5,
        }
    }
}
