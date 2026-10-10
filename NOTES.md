# Playtest notes

Record answers here at the end of each milestone (see WORK_PLAN.md §8).

## M0 — Foundations

Built:
- Workspace: `crates/sim` (pure Rust, glam + serde only) and `crates/game` (Bevy 0.19.1 + bevy_egui 0.42).
- Toolchain pinned to Rust 1.99.0 in `rust-toolchain.toml` (Bevy 0.19 needs ≥ 1.95).
- Vehicle: kinematic bicycle + friction circle; slide state with reduced steering
  authority, yaw drift, lower kinetic grip, and hysteresis for regaining grip. Handbrake on Shift.
- `assets/tuning.ron`, hot-reloaded by polling the file's mtime every 0.25 s. A broken file keeps
  the previous values and shows the error in the inspector.
- Tests: bitwise determinism (3 cars, 1000 ticks of erratic input with slides), and
  drive/brake/slide sanity checks. Run with `cargo test -p sim`.

Playtest 1 feedback and changes:
- Car slid at ~25 km/h, and the yellow zone was skipped (green → red).
  - Past the friction circle the car now **understeers** first: the command is scaled back
    onto the circle and the tires scrub speed (`understeer_scrub`). It slides only above
    `slide_threshold`, which grows as `slide_ref_speed / speed` at low speed.
  - Keyboard steering now asks for a *grip demand* (A/D = 0.9, Ctrl+A/D = 1.4) and builds up
    over time, instead of a wheel angle. At 30 m/s one degree of wheel angle is ~0.6 g, so
    angle-based keys could never land in yellow.
  - Yellow zone widened: `comfort_demand` 0.7 (was a hard-coded 0.8).

Playtest 2: tuned in tuning.ron to engine_accel 8, top_speed 42, brake_decel 10, grip 1.5,
push_steer_demand 3.0. Code defaults (`Tuning::default`) updated to match.

Answers:
- [x] Does the car drive, brake and slide *believably*? Yes, after the understeer/slide split
  and the tuning above.
- [x] Is kinematic + slide state enough, or do we need a dynamic bicycle model? (§7)
  Enough for now. The plan keeps this open through M2, so check again once the stability
  gauge sits on top of it.

M0 done.

## M1 — Plan & preview one car

Built:
- sim: `path.rs` (Bézier between poses, uniform arc-length resampling, windowed projection),
  `orders.rs` (segments with speed profiles start → cruise → end, hold course when orders run
  out, re-chaining after edits), `control.rs` (pure pursuit + speed controller with
  feedforward; never protects the car from its orders), `preview.rs` (fork + step, per-tick
  car frames, bitwise preview-vs-reality check), `replay.rs` (initial state + command log +
  tuning changes, RON files, digest verification), `replay_check` binary.
- Cars have `DriveMode::Orders` (default) or `Direct` (manual keyboard / tests). All changes
  from the game go through `sim::Command`, so they are recorded in the replay.
- Game: starts paused; drag tool (new segment / move end / heading knob / scroll speeds),
  6 s preview colored by grip zone with 1 s ghosts and a scrub ghost, timeline with grip
  strip, segment spans and predicted target hits, segment editor, parking challenge
  (5 seeded targets: box + heading ±15° + max speed, clock in sim time, pause count).
- Tests: path fitting, speed profile, stopping within ~0.1 m, parking with heading,
  chained segments, too-tight path loses grip, preview == live for 360 ticks, replay
  reproduces a session (orders, manual driving, tuning change) and survives RON round trip.
  In-game check verified: "360/360 ticks identical" after committing a 3-segment plan.

Decisions / known limits:
- Forward driving only: no reverse segments yet. If parking needs it, add a reverse flag to
  segments (pure pursuit works mirrored).
- A new segment's end heading follows a circular arc from its start until the knob is
  dragged; its end speed defaults to the previous end speed (0 when starting from rest).
- Editing the active segment keeps its original start and re-projects progress.

Playtest: "Parking challenge" (needs a human)
- [ ] Do players hit targets reliably after a few minutes?
- [ ] Is dragging precise enough? Do players want waypoints mid-segment?
- [ ] Is the speed model (scroll end speed, Shift+scroll cruise) understandable?
- [x] Preview-vs-reality check passes (tests + in-game).
