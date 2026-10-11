//! Closed test tracks built from straights and circular arcs, starting at the origin heading
//! +x. Queries give the position along the track (`s`) and the signed offset from the
//! centerline (`d`, positive = left), the Frenet frame M3 builds on.
//!
//! Track physics (off-track grip) arrives with roads and terrain in M3; for now the game
//! only uses tracks for lap timing and off-track detection.

use glam::Vec2;
use serde::{Deserialize, Serialize};

/// Spacing of the sampled centerline (m).
const SPACING: f32 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Piece {
    Straight(f32),
    Left { deg: f32, radius: f32 },
    Right { deg: f32, radius: f32 },
}

/// Track file contents. The loop is closed with a straight from the end of the last piece
/// back to the start; `closure_gap` reports how far off that join is.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TrackSpec {
    pub name: String,
    /// Road width (m).
    pub width: f32,
    pub pieces: Vec<Piece>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TrackPos {
    /// Distance along the centerline from the start line, in `[0, length)`.
    pub s: f32,
    /// Signed offset from the centerline, positive = left of the driving direction.
    pub d: f32,
}

#[derive(Debug, Clone)]
pub struct Track {
    pub name: String,
    pub width: f32,
    /// Closed centerline: the last point connects back to the first.
    pub points: Vec<Vec2>,
    /// Arc length at each point.
    s: Vec<f32>,
    pub length: f32,
    /// Distance between the end of the last piece and the start, before closing.
    pub closure_gap: f32,
    /// Heading error at the join, degrees.
    pub closure_heading_deg: f32,
}

impl Track {
    pub fn from_spec(spec: &TrackSpec) -> Self {
        let mut pos = Vec2::ZERO;
        let mut heading = 0.0f32;
        let mut points = vec![pos];
        for piece in &spec.pieces {
            match *piece {
                Piece::Straight(len) => {
                    let n = (len / SPACING).ceil().max(1.0) as usize;
                    let step = Vec2::from_angle(heading) * (len / n as f32);
                    for _ in 0..n {
                        pos += step;
                        points.push(pos);
                    }
                }
                Piece::Left { deg, radius } | Piece::Right { deg, radius } => {
                    let sign = if matches!(piece, Piece::Left { .. }) { 1.0 } else { -1.0 };
                    let center = pos + Vec2::from_angle(heading).perp() * radius * sign;
                    let total = deg.to_radians();
                    let n = (total * radius / SPACING).ceil().max(1.0) as usize;
                    for i in 1..=n {
                        let h = heading + sign * total * i as f32 / n as f32;
                        pos = center - Vec2::from_angle(h).perp() * radius * sign;
                        points.push(pos);
                    }
                    heading += sign * total;
                }
            }
        }
        // Close the loop: drop the end point if it lands on the start, otherwise join with a
        // straight back to it.
        let closure_gap = pos.length();
        let closure_heading_deg = crate::vehicle::wrap_angle(heading).to_degrees();
        if closure_gap < SPACING * 0.5 {
            points.pop();
        } else {
            let n = (closure_gap / SPACING).ceil() as usize;
            let end = pos;
            for i in 1..n {
                points.push(end.lerp(Vec2::ZERO, i as f32 / n as f32));
            }
        }

        let mut s = Vec::with_capacity(points.len());
        let mut acc = 0.0;
        for i in 0..points.len() {
            s.push(acc);
            acc += points[i].distance(points[(i + 1) % points.len()]);
        }
        Self { name: spec.name.clone(), width: spec.width, points, s, length: acc, closure_gap, closure_heading_deg }
    }

    fn segment(&self, i: usize) -> (Vec2, Vec2) {
        (self.points[i], self.points[(i + 1) % self.points.len()])
    }

    /// Closest point on the centerline.
    pub fn project(&self, p: Vec2) -> TrackPos {
        let mut best = (f32::INFINITY, 0usize, 0.0f32);
        for i in 0..self.points.len() {
            let (a, b) = self.segment(i);
            let ab = b - a;
            let t = ((p - a).dot(ab) / ab.length_squared().max(1e-9)).clamp(0.0, 1.0);
            let d2 = p.distance_squared(a + ab * t);
            if d2 < best.0 {
                best = (d2, i, t);
            }
        }
        let (_, i, t) = best;
        let (a, b) = self.segment(i);
        let tangent = (b - a).normalize_or(Vec2::X);
        let seg_len = a.distance(b);
        let s = (self.s[i] + seg_len * t) % self.length;
        TrackPos { s, d: (p - (a + (b - a) * t)).dot(tangent.perp()) }
    }

    pub fn on_track(&self, p: Vec2) -> bool {
        self.project(p).d.abs() <= self.width * 0.5
    }

    /// Centerline point and unit tangent at arc length `s` (wraps around).
    pub fn frame_at(&self, s: f32) -> (Vec2, Vec2) {
        let s = s.rem_euclid(self.length);
        let i = self.s.partition_point(|&x| x <= s).saturating_sub(1);
        let (a, b) = self.segment(i);
        let seg_len = a.distance(b).max(1e-6);
        let tangent = (b - a) / seg_len;
        (a + tangent * (s - self.s[i]), tangent)
    }
}
