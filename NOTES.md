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

Playtest feedback: short, sharp swerves don't keep the orientation.
- Cause: the steering actuator (`steer_rate_deg`, 120°/s). The swerve flipped its command at
  the midpoint, so the swing across ended late and the segment ended with the wheels still
  turned (the car kept yawing after the keypoint). Negligible for long swerves; up to 40° off
  for sharp ones, and as bad when the swerve started with the wheels turned from the previous
  keypoint.
- Fix (`Turn::steer_at`): the wheels turn in at the steering rate, and from where the swing
  across must start (a ramp centered on the midpoint) a closed loop re-solves each tick the
  wheel level that undoes the heading change so far with the wheels straight on the last tick
  (ramps integrated in closed form, false-position solve). Nominally this is the antisymmetric
  profile; it also absorbs speed changes and a turned start. Segments remember their
  `start_heading` (runtime state like `elapsed`); `Drive::controls` takes a `Phase`.
- Heading within 1° and wheels straight at the end in every case tried. Very sharp targets
  that the old swerve "reached" by ending crooked are now out of reach (the keypoint lands
  where the car really can go). Fit ~3–6 ms (was ~2–3), sweep ~18 ms (was ~10).
- Test: `sharp_swerves_keep_the_heading` (short swerves at 8–20 m/s, turned start both ways).

### M2 rework, step 1 — kinematic car

Playtest verdict: the grip simulation costs too much and is hard to gamify. Plan: Car Wars
handling in continuous time (WORK_PLAN.md, "M2 rework"). Step 1 removes the physics:
- `vehicle.rs` is a kinematic bicycle: the wheels take the commanded angle at once, the car
  rolls along its heading at `v · tan δ / L`. No friction circle, understeer, slides or
  handbrake turns (the handbrake only brakes). `grip_demand` is still computed, only as the
  stability gauge's input, until step 2 replaces the gauge with handling status.
- Loss of control is only reported (event, gauge reset to `after_loss`); crash results that
  move the car come in step 5.
- Removed: `SlideState`/`Car::slide`, slide events, slip angle and lateral speed, the
  steering actuator (`steer_rate_deg`), slide/understeer/handbrake-slide tuning, yaw kicks,
  uncertainty fans (`Preview::drift`, X toggle, `drift_recover_seconds`), the friction-circle
  slide ring. The closed-loop swerve controller (above) is reverted to the plain midpoint flip:
  with instant steering the halves cancel exactly. Swerve rates are capped at the steering
  lock so the halves stay equal.
- Tests: vehicle tests rewritten (kinematic circle at any speed, instant wheels, handbrake
  only brakes, full lock at speed loses control via the gauge); determinism now checks risk
  checks and losses of control instead of slides.
- Also removed after step 1: the g-force turn mode (`Turn::Lateral`; steering already is a
  constant turn radius, which is what a Car Wars bend is) and the danger zones (`zones.rs`,
  `sim::sweep`, `sweep_swerve`, `ReachMap`): they shaded by grip demand from simulation
  sweeps. Risk shading may come back in closed form with the maneuvers (step 3). T now
  toggles steering ↔ swerve. Tests ported to steering turns of the same lateral g.

### M2 rework, step 2 — handling status

- `sim::handling` replaces the stability gauge (`stability.rs`, `StabilityTuning`, grip demand,
  `grip`/`comfort_demand`, risk interval, min speed, after-loss reset all gone).
  - HS = handling class (`vehicle.handling_class`) − `Car::handling_lost`; recovers
    `max(1, HC + pilot.skill)`/s (skill is now a Car Wars driver bonus, default 0); floor −6.
  - Drain per tick: |heading change| / 15°, plus braking beyond 10 mph/s by the book's
    progression (15 → 1/s, 20 → 2, 25 → 3, +2 per 5 mph/s).
  - Control Table: `p_fail = clamp((mph − 106 − 27·HS) / 130, 0, 1)`; per tick
    `1 − (1 − p_fail)^drain`, at the drained status, before recovery.
  - `handling::hazard`: discrete check after dropping HS by a difficulty (D0 allowed), or at a
    forced status. No callers yet (hazards come in step 6).
  - Loss of control still only reported; no checks for `loss_seconds` (1 s) after one.
  - `SimEvent::Risk` only for checks with a chance > 0.
- Display: risk zones (`handling::zone`: green safe, yellow risky to maneuver, orange risky,
  red certain) color the path, trails, timeline strip and handling bar; the timeline graph
  shows HS and, per second, the chance of losing control (`Preview::risk_per_second`).
  Inspector shows HS and the chance per point or this tick. Acceleration circle removed.
  Manual keyboard steering now asks for lateral g (`full_steer_g`, `push_steer_g`).
- Calibration finding: HS is a threshold system. While the drain rate (°/s / 15) stays below
  recovery (HC/s), HS stays full and there is no risk; above it HS falls linearly. On the slalom
  the drain is about speed/4 points/s (≈ 85° of heading per gate). With HC 2 only ~8 m/s is safe
  (33 s, the old 20.5 s limit was unwinnable). Default HC is now 3 and the slalom limit 18.0 s:
  14 m/s finishes in 18.7 s safely, 15 m/s in 17.4 s with a 60% loss chance, 16 m/s certainly
  loses control (smooth-line scripted driver, not kept).
- Tests: `tests/handling.rs` (Control Table spot checks against the book, one point = one roll
  at any tick rate, bends drain by angle at any speed, recovery and cap, braking table, checks
  at the floor, straights are safe, ideal never loses control, skid/spin and the loss timer,
  dice from the state, hazards incl. D0 and forced status, preview stretches and per-second
  risk). Handling tests pin HC 2.

### M2 rework — drift replaces the swerve

Playtest feedback: (1) the swerve swung at low speed; (2) a longer swerve with the same
sideways/forward ratio cost nothing while a shorter one drained handling. Causes: the halves
were equal in *time*, so a car slowing down (or stopping) never turned back (+31° to +82°
measured when braking); and swerves were charged as bends, whose total (90° each way in the
screenshots) does not depend on length, so a longer one just recovered more meanwhile.

- `Turn::Swerve` → `Turn::Drift(ratio)` (`TurnMode::Drift`, `sim::drift`): ratio = sideways per
  forward over the drift, at most ½ (the book's steep drift).
- Path: S-curve of two arcs of equal *length*, each turning `θ = 2·atan(ratio)` (53° at ½).
  Steering does not affect speed in the kinematic model, so the controller predicts the
  distance the segment's remaining ticks will cover (`drift::travel`) and switches curvature
  exactly halfway (the tick straddling the switch gets the weighted mix). Heading comes back
  within 0.2° (in practice ~0) at any speed, braking to a stop included. Curvature is capped at
  the steering lock before splitting, so a lock-limited drift stays symmetric (it just moves
  less sideways). Segments carry `traveled` (runtime, like `elapsed`); `Drive::controls` takes a
  `Phase` and `dt`.
- Cost (`handling::drain`): while drifting, `drift::cost_per_inch(ratio)` per inch forward
  (0 → 1 at ¼, 1 → 3 at ½; `drift_cost`, `steep_drift_cost` in tuning) instead of the bend
  cost of its heading changes. Twice as long at the same ratio = twice the cost.
- Fit (`fit_drift`): closed form. Ratio from the target (capped at ½), then for each candidate
  length (1D speed integration) the arc angle is limited by the lock; keeps the closest end
  point. No simulation runs.
- Note: the slalom (10 m sideways per 20 m forward) is exactly a steep drift: ~13 points per
  gate drifting vs ~5–6 bending, so bending through it stays the better line.
- Tests: `tests/drift.rs` (heading kept at any speed/throttle incl. stopping, flip at half the
  distance, sideways ratio, cost by steepness and length, fit within ½ and capped beyond,
  steering lock, drag continuity); `tests/swerve.rs` removed.

### M2 rework, step 3 — closed-form keypoints, per-keypoint risk

- `fit_bend` replaces the simulation-based `fit_segment` (61 coarse candidates + golden-section
  refinement, each a full simulation): a constant wheel angle is a constant curvature at any
  speed, so the bend is the arc tangent to the car through the target, `κ = 2y / (x² + y²)` in
  the car's frame, capped by the lock and `max_lateral_g` at the start speed; its length after
  `n` ticks comes from the 1D speed integration, and the closest arc point picks the duration
  (first loop only, as before). Drifts already fit in closed form (`fit_drift`). The orders
  tests run in ~0.01 s instead of ~0.75 s.
- Keypoint labels and the keypoint editor show "D 3.2 · 18% loss": handling drained over the
  segment and its chance of losing control (`Preview::span_risk`); per-second risk was step 2.
- Removed `planner.max_lateral_g` (3 g): a grip-model leftover that capped only dragged bends
  (radius ≥ 13.6 m at 72 km/h, 31 m at 108 km/h; the steering lock limited below 38 km/h).
  Dragged bends are now limited only by the steering lock (radius 3.9 m), like the editor,
  drifts and the simulation; tightness at speed is priced by handling (the book allows a 90°
  bend in 1" at any speed).

### M2 rework, steps 4–5 — crash tables, slides, controlled skid

- `sim::crash`: loss of control rolls Crash Table 1 (maneuvers: 2d6 + (D − 3) + speed
  modifier − skill) or 2 (hazards, `handling::hazard`: fishtails, then table 1 on 9+). Speed
  modifier from the Control Table's column (−3 at ≤10 mph … 0 at 35–40, +1 at 45–60, then +1
  per 30 mph). D = the maneuver's drain per inch at that tick (one book maneuver per inch: a
  90° turn within an inch is D6) plus hard braking's D per second. `Crash` replaces
  `LossKind`; `Risk` events carry `severe_chance` (spinout or worse) instead of spin chance;
  the 1 s `loss_seconds` pause and skid/spin speeds are gone.
- Results are kinematic `Slide`s (`Car::slide`): the car moves along a fixed direction while
  its heading does its own thing; controls are ignored (orders keep their time), handling
  recovers, no control checks. Skids (¼", ½" −5 mph, 1" −10 mph, 1½" −20 mph: follow-up skids
  added in) slide straight on along the travel direction (in a bend, the tangent; it was the
  direction of an inch earlier, which threw cars out of bends) while the nose keeps turning
  at the yaw rate the car had, then
  ease back onto the nose (`Car::slip`). Spinout: 450°/s, −20 mph/s, recovery
  roll at HS −6 every second (facing the travel direction → drives on, else T-stop at
  −20 mph per inch); without dice (Ideal) it spins to a stop. Roll: sideways, −20 mph/s to a
  stop (burning on 4–6, display only). Vault: 1d6" airborne, then roll. Fishtails: rear swings
  15/30/45° about the far front corner, car drives on along the new heading. `CrashTuning`
  holds the book values. `SlideEnded` event.
- Controlled skid (`Turn::Skid { steer, slide }`, T cycles steering → drift → skid): steers as
  a bend until the segment's last `slide` inches (predicted with `drift::travel`), then slides
  them along the segment's start heading keeping the turned heading (speed still follows the
  pedals, so it ends with the segment), +`skid_cost_per_inch` (4) points per inch at the start
  (checked like any drain), speed loss 0/5/5/10 mph at ¼/½/¾/1" applied at the end.
  Segments carry `start_heading` and `skidding` (runtime). `fit_skid`: closed form, the bend
  aims at the target minus the slide. Planning locks a started skid (`Draft::locked`: no
  refit, throttle, duration, mode or delete; Clear keeps it): the commitment window.
- Display: car body color by slide (cyan controlled, orange skid/T-stop, red spinout/roll with
  a cross for rolls, white airborne, ring when burning); inspector slide row; risk labels
  "34% loss (20% spinout+) at 2.1 s".
- Not done: tire damage (no damage model yet), crash damage, fire.
- Tests: `tests/crash.rs` (tables and modifiers vs the book, skids' distance/heading/speed,
  spinout rate/decel/stop and seeded recovery, roll and vault, fishtail pivot, worse crashes
  for harder/faster maneuvers, lag heading ≈ an inch of turning, controlled skid geometry,
  cost and speed loss, skid fit); handling test: a loss starts a slide and pauses checks.

Playtest feedback: the controlled skid was backwards (it bent, then suddenly changed its
direction of travel back while keeping the turned nose). Now `Turn::Skid { angle, slide }`:
from the segment's start the car keeps moving along its heading for `slide` inches while the
nose swings by `angle` (≤ `max_skid_deg`, 90°) over the first `skid_turn_inches` (¼"), then
drives straight along the new nose; the bend (angle / 15°) and the slide (4 per inch) are
charged together when it starts, like the book's roll after the maneuver and before the
skid. `Slide` got `turn`/`turn_per_m` for the swing. `fit_skid`: the nose points from the
slide's end at the target, and the predicted distance includes the slide's speed loss.

Playtest feedback: the skid was invisible (the car just turned). The book's slide is at most a
car length: at 72 km/h the nose swung in 3–4 ticks and the whole ½" skid took 0.11 s. Now the
slide is measured in time: `skid_seconds_per_inch` (1.0) turns the book's ¼"–1" into ¼–1 s of
sliding, the nose swings over the first `skid_turn_seconds` (0.25 s), counted in ticks
(`Slide::ticks_left`, `turn_step`; `control::skid_ticks`) so the fit's prediction is exact.
Costs stay the book's (per 15° and per ¼"). The sideways carry grows with speed. `fit_skid`
aims the nose from where the slide ends per candidate duration (the slide's length depends on
speed); targets needing more than 90° are capped and predicted correctly.
- Bug (playtest): the controlled skid was never checked. The controller starts its slide and
  charges its cost in the same tick, and `handling::update` skipped sliding cars, so the skid
  suppressed its own check (and showed as safe in the preview). Now only a slide already
  going on when the tick began skips checks (`was_sliding`); a skid is checked as it starts
  (bend + slide points at once, crash difficulty the same), and losing control there replaces
  the skid with the crash result. Test: `controlled_skid_is_checked_when_it_starts`.
