//! Planned paths: a cubic Bézier between two poses, resampled at uniform arc length so the
//! controller can address it by distance `s` along the path.

use glam::Vec2;
use serde::{Deserialize, Serialize};

use crate::state::Pose;
use crate::tuning::PlannerTuning;
use crate::vehicle::wrap_angle;

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PathSpec {
    pub start: Pose,
    pub end: Pose,
    /// Samples at `s = i · spacing`, plus `end.pos` as the last point (closer than `spacing`).
    pub points: Vec<Vec2>,
    pub spacing: f32,
    pub length: f32,
}

impl PathSpec {
    /// Cubic Bézier leaving `start` along its heading and arriving at `end` along its heading.
    /// Handles grow with the turn angle so U-turns bulge instead of folding into a cusp.
    pub fn bezier(start: Pose, end: Pose, pt: &PlannerTuning) -> Self {
        let chord = start.pos.distance(end.pos);
        let turn = wrap_angle(end.heading - start.heading).abs() / std::f32::consts::PI;
        let h = chord * (pt.handle_base + pt.handle_turn * turn);
        let p = [start.pos, start.pos + start.forward() * h, end.pos - end.forward() * h, end.pos];
        let eval = |t: f32| {
            let u = 1.0 - t;
            p[0] * (u * u * u) + p[1] * (3.0 * u * u * t) + p[2] * (3.0 * u * t * t) + p[3] * (t * t * t)
        };

        // Dense sampling, then resample at uniform arc length.
        let n = ((chord / pt.path_spacing) as usize * 4).clamp(64, 8192);
        let dense: Vec<Vec2> = (0..=n).map(|i| eval(i as f32 / n as f32)).collect();
        let spacing = pt.path_spacing;
        let mut points = vec![dense[0]];
        let mut next_s = spacing;
        let mut s = 0.0;
        for w in dense.windows(2) {
            let d = w[0].distance(w[1]);
            while d > 0.0 && s + d >= next_s {
                points.push(w[0].lerp(w[1], (next_s - s) / d));
                next_s += spacing;
            }
            s += d;
        }
        let length = s;
        if length - (points.len() - 1) as f32 * spacing > 1e-4 {
            points.push(end.pos);
        } else if let Some(last) = points.last_mut() {
            *last = end.pos;
        }
        Self { start, end, points, spacing, length }
    }

    /// Arc length of sample `i`.
    fn s_of(&self, i: usize) -> f32 {
        if i + 1 == self.points.len() { self.length } else { i as f32 * self.spacing }
    }

    /// Point at arc length `s`; beyond the ends the path continues straight along the
    /// start/end headings.
    pub fn point_at(&self, s: f32) -> Vec2 {
        if s <= 0.0 {
            return self.start.pos + self.start.forward() * s;
        }
        if s >= self.length {
            return self.end.pos + self.end.forward() * (s - self.length);
        }
        let i = ((s / self.spacing) as usize).min(self.points.len() - 2);
        let (s0, s1) = (self.s_of(i), self.s_of(i + 1));
        self.points[i].lerp(self.points[i + 1], (s - s0) / (s1 - s0).max(1e-6))
    }

    /// Unit tangent at arc length `s`.
    pub fn tangent_at(&self, s: f32) -> Vec2 {
        if self.points.len() < 2 || s >= self.length {
            return self.end.forward();
        }
        if s <= 0.0 {
            return self.start.forward();
        }
        let i = ((s / self.spacing) as usize).min(self.points.len() - 2);
        (self.points[i + 1] - self.points[i]).normalize_or(self.end.forward())
    }

    /// Arc length of the point on the path closest to `pos`, searching only `s` within
    /// `[from, to]` (keeps the search local so loops and crossings don't confuse it).
    pub fn project(&self, pos: Vec2, from: f32, to: f32) -> f32 {
        if self.points.len() < 2 {
            return self.length;
        }
        let last = self.points.len() - 2;
        let i0 = ((from.max(0.0) / self.spacing) as usize).min(last);
        let i1 = ((to.max(0.0) / self.spacing).ceil() as usize).min(last);
        let mut best = (f32::INFINITY, from.clamp(0.0, self.length));
        for i in i0..=i1 {
            let (a, b) = (self.points[i], self.points[i + 1]);
            let ab = b - a;
            let t = ((pos - a).dot(ab) / ab.length_squared().max(1e-9)).clamp(0.0, 1.0);
            let d = pos.distance_squared(a + ab * t);
            if d < best.0 {
                let (s0, s1) = (self.s_of(i), self.s_of(i + 1));
                best = (d, s0 + (s1 - s0) * t);
            }
        }
        best.1
    }
}

/// End heading that makes the path from `start` to `p` a circular arc: the natural default
/// while the player drags a new end point.
pub fn arc_heading(start: Pose, p: Vec2) -> f32 {
    let d = p - start.pos;
    if d.length_squared() < 1e-6 {
        return start.heading;
    }
    wrap_angle(2.0 * d.y.atan2(d.x) - start.heading)
}
