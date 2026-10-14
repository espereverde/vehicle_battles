//! Easing for motion changes the rules make at once (a slide starting or ending, the nose
//! swinging round): the book says where the car ends up, easing says how it gets there.

use serde::{Deserialize, Serialize};

/// 0 at `x ≤ 0`, 1 at `x ≥ 1`, eased in and out between.
pub fn smoothstep(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// The direction of travel lagging the one it should have (the slide's while sliding, the
/// nose otherwise) by an angle that eases away to nothing over `seconds`.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Slip {
    /// Starting offset (radians, + = left of the target direction).
    pub from: f32,
    pub elapsed: f32,
    pub seconds: f32,
}

impl Slip {
    /// A slip of `from` radians, or none when it is negligible or there is no time to ease.
    pub fn new(from: f32, seconds: f32) -> Option<Self> {
        (from.abs() > 1e-4 && seconds > 0.0).then_some(Self { from, elapsed: 0.0, seconds })
    }

    /// Offset now (radians).
    pub fn angle(&self) -> f32 {
        self.from * (1.0 - smoothstep(self.elapsed / self.seconds))
    }

    /// The slip `dt` later, or none once it has eased away.
    pub fn advance(self, dt: f32) -> Option<Self> {
        let elapsed = self.elapsed + dt;
        (elapsed < self.seconds).then_some(Self { elapsed, ..self })
    }
}

/// Offset of a slip (0 without one).
pub fn slip_angle(slip: Option<Slip>) -> f32 {
    slip.map_or(0.0, |s| s.angle())
}
