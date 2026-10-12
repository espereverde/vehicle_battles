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

### M1 rework — keypoints with constant controls (after playtest 1)

Feedback: the path + cruise/end speed model was not intuitive. Replaced with:
- Segments between keypoints hold constant controls: a turn (constant steering, or constant
  lateral g with steering adapting to speed; T / buttons) and a constant throttle (wheel,
  ±5%, negative = brake), for a set number of ticks. Keypoint positions are simulated,
  never stored, so editing one keypoint moves the later ones and keeps their controls.
- Dragging a keypoint fits its turn and duration to the cursor by simulating candidates
  (`sim::fit_segment`: 61 coarse samples, cubic in steering so they are dense near zero,
  golden-section refinement of each local minimum, ~3.5 ms in dev builds). It minimizes
  `miss² + fit_g_weight · ∫ grip demand² dt` (integral, so a stopped car gains nothing by
  waiting and the keypoint lands where it stops), and never plans a slide when a slide-free
  turn exists: with the cursor out of reach while braking it used to lock the car into a
  slide to travel further (sliding brakes less than gripping). Earlier issue: past the grip limit the car slides and every
  hard turn lands near the gentle one (steering jumped 9% → 100%); the g term makes the gentle
  one win, and as a smooth score it doesn't flip between solutions like a distance cutoff.
  Sweep over cursor grids: no keypoint jumps > 1.5 m between cursor positions 0.5 m apart,
  except starting mid-slide with the cursor 12+ m outside the reachable area.
  Further away = later in time, sideways = more turn. The scrub follows the dragged keypoint.
- After the last keypoint the car keeps its last throttle and turn ("continue").
- "Straight" sets throttle to the value that holds the start speed (`hold_throttle`).
- Preview horizon 5 s. Keypoints pushed past it by earlier edits are kept, but are not
  editable until they come into view.
- Removed: Bézier paths, pure pursuit, speed profiles (`path.rs`, `ControlTuning`).
- In-game check: committed 2-keypoint plan, "300/300 ticks identical".

Playtest (needs a human):
- [ ] Is dragging a keypoint (further = later, sideways = turn) intuitive?
- [ ] Steering vs g-force: which mode do players use, and when?
- [ ] Is constant throttle per segment enough, or is a speed target wanted back?

### Test track

- `assets/maps/track.ron`: 733 m loop, 10 m wide, built from straights and arcs (exact radii):
  T1 L90° r45 (~93 km/h at 1.5 g), T2 L135° r16 hairpin (~55), T3 R45° r40 kink (~87),
  T4 L90° r25 (~69), T5 L90° r30 (~76); 140 m start straight. The loop is closed with an
  automatic straight back to the start.
- `sim::track`: centerline sampled every 0.5 m, `project` → (s, d), `frame_at(s)`. This is the
  Frenet groundwork for M3; tracks are not yet part of `sim::World` (no physics effect).
- Game: road mesh + edges + dashed centerline + checkered start line, lap timer (lap counts
  after passing halfway), off-track counter, preview marks where the plan leaves the road.
  Mode switch C (track / parking), track is the default.

### M1 rework 2 — drag ignores sliding (after playtest 2)

Feedback: the preview jumped around while dragging, so the result of a slide was hard to predict.
Simplified:
- New `StepMode::Ideal`: the same `step()` with unlimited tire grip (no understeer scaling or
  scrub, no friction-circle clamp, no slide; an ongoing slide is dropped). Grip demand is still
  computed. Within the grip limit it is bitwise identical to the real model (tested).
- `fit_segment` always runs in `Ideal` and now just minimizes the distance to the cursor (earliest
  tick on ties). Removed: the grip-force term (`fit_g_weight`) and the "never plan a slide"
  preference. Both existed only to resolve the many-to-one mapping that sliding caused.
  Hard turns now land exactly on the cursor.
- The planning preview always runs in `Ideal` (first only while dragging; after playtest it
  stays on after release too). The player sees the intended outcome; things can still get out of
  hand live. Turning stays limited by the steering lock and steering rate (tested). The path keeps
  its grip coloring: orange past 1.0 (would understeer), red past the speed-dependent slide
  threshold (would slide); the timeline says "would SLIDE (not simulated)". While running, a faint
  ideal path is re-previewed from the live state every tick, so it starts at the car.
- This is a deliberate exception to "preview is the simulation" (§2). The reality check still
  compares live play with a real-model preview, so it remains a determinism check, but the
  player-facing preview no longer matches live play once grip is exceeded.
- Keypoints are anchored to instants in time (bug: slowing an earlier segment pushed the last
  keypoint past the horizon, where it vanished). Refitting a keypoint only lets it move between
  the previous and the next keypoint's times; the next segment absorbs the change. Deleting one
  lengthens the next segment. Positions of later keypoints still follow from the simulation.
  The segment editor's duration field follows the same rule (`Draft::set_ticks`).
  This lives in the game's draft editing; `sim::Segment` still stores durations.
- The preview is cut where the path closes its first loop (net heading change of 360° since the
  preview start; `Preview::first_loop`), paused and running. The fit gets the heading turned so
  far and stops candidates before a full circle, so keypoints always sit on the first loop
  (it used to reach a target on the second lap with a gentler turn).
- Turn cap for the drag tool: `max_lateral_g` raised from 2.0 to 3.0 g (2× grip) and now applied
  to steering mode too, as the steering that gives that much lateral g at the segment's start speed.
- Danger zones (Z, on by default): while paused, the area the selected keypoint (or a new one
  after the last) can be dragged to is shaded in the color its segment would have there (worst
  zone along the segment so far). Built from `sim::sweep`, the fit's own search space (81 turn
  values × up to 5 s, first loop only, ~2.4 ms), as one vertex-colored mesh; rebuilt only when
  the keypoint, its throttle/mode or the segments before it change, so dragging is free.

Playtest (needs a human):
- [ ] Are the danger zones readable, and do players use them to pick keypoints?
- [ ] Is the drag stable now? Do red/orange warnings make slides feel fair when they happen live?
