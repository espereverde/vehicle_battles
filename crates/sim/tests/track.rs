//! Test track geometry: closure, length, projection.

use sim::{Piece, Track, TrackSpec, Vec2};

fn load() -> Track {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets/maps/track.ron");
    let spec: TrackSpec = ron::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    Track::from_spec(&spec)
}

#[test]
fn test_track_closes_cleanly() {
    let t = load();
    assert!(t.closure_heading_deg.abs() < 0.01, "heading off by {}°", t.closure_heading_deg);
    // Ends on the x axis behind the start, so the closing straight runs along the heading.
    let end = t.points[t.points.len() - 1];
    assert!(end.y.abs() < 0.1 && end.x < 0.0, "closing straight not aligned: {end}");
    assert!((t.length - 733.0).abs() < 3.0, "length {}", t.length);
}

#[test]
fn projection_gives_s_and_signed_offset() {
    let t = load();
    let p = t.project(Vec2::new(30.0, 3.0));
    assert!((p.s - 30.0).abs() < 0.01 && (p.d - 3.0).abs() < 0.01, "{p:?}");
    let p = t.project(Vec2::new(30.0, -4.0));
    assert!((p.d + 4.0).abs() < 0.01);
    assert!(t.on_track(Vec2::new(30.0, 4.9)));
    assert!(!t.on_track(Vec2::new(30.0, 5.1)));
    // Just behind the start line wraps to the end of the lap.
    let p = t.project(Vec2::new(-1.0, 0.0));
    assert!((p.s - (t.length - 1.0)).abs() < 0.01, "{p:?}");
    for s in [0.0, 100.0, 333.3, t.length - 0.2] {
        let (c, _) = t.frame_at(s);
        assert!((t.project(c).s - s).abs() < 0.01);
    }
}

#[test]
fn arcs_have_the_requested_radius() {
    let spec = TrackSpec { name: "circle".into(), width: 8.0, pieces: vec![Piece::Left { deg: 360.0, radius: 20.0 }] };
    let t = Track::from_spec(&spec);
    let center = Vec2::new(0.0, 20.0);
    assert!(t.points.iter().all(|p| (p.distance(center) - 20.0).abs() < 1e-3));
    assert!(t.closure_gap < 1e-3);
    assert!((t.length - 2.0 * std::f32::consts::PI * 20.0).abs() < 0.1);
}
