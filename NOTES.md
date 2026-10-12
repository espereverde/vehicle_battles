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
- Drift fans (X, on by default; paused and running): where the preview is orange/red, shade the
  band between the planned path and an outer bound of where the real car could go
  (`Preview::drift`). Same rule as the vehicle model: the turn rate is divided by the grip demand
  (understeer), and in a slide the friction circle shrinks to `slide_grip`, so slides fan out
  much wider. Speed follows the plan (errs wide: the real car scrubs speed). The offset lingers
  `drift_recover_seconds` (0.5 s) after the plan is back in grip. Only a hint: it does not
  simulate the slide, so the exact real path stays hidden.

Playtest (needs a human):
- [ ] Do drift fans make slides feel fair without giving the outcome away? Too wide / too narrow?
- [ ] Are the danger zones readable, and do players use them to pick keypoints?
- [ ] Is the drag stable now? Do red/orange warnings make slides feel fair when they happen live?

## M2 — Stability & loss of control

Decisions (before building):
- **Stability decides.** Past the friction circle the tires only understeer (no fixed
  `slide_threshold` any more, removed with `slide_ref_speed`). Skids and spins come only from
  the gauge (or the handbrake). So the gauge is the one thing to manage, and the preview can
  give exact odds.
- **Swerve only** of the named maneuvers. Bootlegger, T-stop and controlled skid need handbrake
  and reverse in segments, and the slide-free preview would have to simulate intended slides.
  Deferred until the slalom playtest has answered the gate questions.

Built:
- `sim::stability`: per tick, drain `k_drain · (demand − comfort)²` above comfort plus
  `k_jerk · |Δdemand|`; recovery `k_rec · pilot.skill` below comfort; frozen while sliding.
  Below `yellow` (0.6), every `risk_interval` (0.25 s) spent above comfort is a risk check with
  chance `risk_max · (1 − s/yellow)²`; at 0 it is certain. No checks under `min_speed`
  (10 m/s = 36 km/h: playtest 1 complained about sliding at 25 km/h). Loss of control = slide
  state plus a yaw kick into the turn: skid below `skid_speed` (12 m/s), spin above
  `spin_speed` (25 m/s), linear odds between; stability restarts at `after_loss` (0.3).
  All constants in the new `stability` section of `tuning.ron`.
- Dice: `Live` and `Preview` roll from the state's RNG, so the preview-vs-reality check stays
  bitwise exact. `Ideal` (the player's preview) never rolls: it emits `SimEvent::Risk` with the
  chance, and at a certain loss resets the gauge to `after_loss` as if the car had recovered.
- `Preview::risks` merges consecutive checks into stretches (`1 − Π(1 − p)`, spin odds weighted
  by chance). Shown as red rings / a red cross on the path, labels ("34% spin at 2.1 s",
  "LOSS OF CONTROL (spin) at 3.1 s"), red bands on a new stability graph under the timeline
  strip, and in the scrub readout. Also on the faint path while running.
- Path red now means "may lose control here" (yellow stability, above comfort, fast enough),
  for the path, the timeline strip, danger zones and drift fans (`CarFrame::at_risk`).
- Stability bar under the car (tick = yellow line) and in the inspector; event log shows risk
  checks and losses of control.
- Swerve: first a 3-keypoint template (Q/E); replaced after feedback by a turn mode, see below.
- Slalom mode (C cycles track → parking → slalom): `assets/maps/slalom.ron`, 10 gates 20 m
  apart at ±5 m, 4 m wide, finish at 235 m, limit 20.5 s, +2 s per missed gate. Panel shows time,
  gates, losses of control, pauses, best; the timeline predicts gate passes (G3 / G3✗) and the
  finish time. Calibrated with a scripted pure-pursuit driver: comfortable driving (demand
  ≤ 0.7) ~21.7 s, hard but green ~18.8 s, at 14.5 m/s the gauge drops to yellow, at 15 m/s it
  loses control and misses gates.
- Tests: `tests/stability.rs` (drain/recovery/skill, no checks when green or below comfort,
  ideal reports without losing control, skid vs spin by speed, dice from the state, risk
  stretches), vehicle tests updated (understeer until the gauge runs out), swerve, slalom gate
  logic and file parsing. 44 tests.

Playtest: "Slalom" — FUN GATE 1 (needs a human)
- [ ] Do players push into yellow on purpose?
- [ ] Do crashes feel fair (did the preview warn them)?
- [ ] Is the pause → plan → watch rhythm enjoyable without any combat?
- [ ] Is 20.5 s the right limit? Does the gauge drain/recover at a readable pace?
- [ ] Is the swerve mode useful? Which other maneuvers are wanted first?

### M2 rework — swerve as a turn mode

Feedback: make swerve selectable like steering and g-force. A swerve always ends in the same
orientation as the previous keypoint, and is two turns of equal but reversed turn rate and the
same duration. (A first version fitted an arbitrary target heading with a speed-profile model
and correction passes; judged overcomplicated and replaced.)
- `Turn::Swerve(rate)` (`TurnMode::Swerve`): yaw rate `rate` for the first half of the segment,
  `-rate` for the second; steering adapts to speed (`δ = atan(rate·L/v)`). As the queue's tail it
  drives straight. `Drive::controls` now takes the segment's (elapsed, ticks).
- `sim::fit_swerve`: in the start frame, a longer swerve goes further forward and a higher rate
  further sideways, so two nested bisections on the real ideal-mode simulation: half-length for
  the forward distance, rate (up to `max_lateral_g` at the start speed, ≤ 90° per half) for the
  sideways offset. ~2–3 ms, misses ≤ 0.1 m (whole ticks), heading back within 2°.
- T cycles steering → g-force → swerve; the segment editor shows the turn rate (°/s); the
  duration field keeps the halves equal automatically.
- Danger zones in swerve mode: `sim::sweep_swerve`, one simulation per (rate, half-length every
  5 ticks), colored by the worst zone on the way (~10 ms, only when the plan before the keypoint
  changes). `sim::sweep` returns the same `ReachMap`, so zones.rs just meshes it.
- Removed: the 3-keypoint swerve template (`sim::swerve`, Q/E, buttons, `swerve_*` tuning).
- Tests (`tests/swerve.rs`): reaches the target with the start heading (lane changes both ways,
  slow car, standing start, braking, small offset at 25 m/s), equal and opposite halves, drag
  never jumps, danger-zone map covers both sides and reaches red.
