# Continuous Car Combat — Prototype Work Plan

Working title. Strategic top-down car combat in the spirit of *Car Wars*, but resolved in
continuous time: the simulation runs in real time, the player pauses whenever they want,
plans the next few seconds by dragging cars, previews the outcome on a scrubbable timeline,
and resumes. Pilots have reaction times, cars have a stability gauge, and enemies are shown
only as their current course plus a cloud of reachable positions.

Engine: **Rust + Bevy 0.19**. Verify any version-specific API (fixed timestep, gizmos,
states, picking, egui integration) against current docs before relying on it.

---

## 1. What the prototype must prove

Each milestone answers one or more of these questions. If an answer is "no" and tuning
doesn't fix it, stop and rethink before building further.

1. **Planning feel:** is drag-to-plan driving intuitive, precise and satisfying?
2. **Speed vs. control:** does the stability gauge create real tension (go fast vs. stay safe)?
3. **Reflexes:** does the reaction-time lock create tension rather than frustration?
4. **Reading the enemy:** can players read intent from course + reachable-set display and act on it?
5. **Opponents:** does the AI feel believable and challenging under the same rules as the player?
6. **The loop:** is pause → plan → watch fun across a whole 5–10 minute fight?

Two **fun gates**: after M2 (is driving alone fun?) and after M7 (is fighting fun?).

---

## 2. Principles

- **Bevy-free simulation core.** All rules live in a plain Rust crate (`sim`): cloneable state,
  `step(dt)` at a fixed 60 Hz. Bevy only renders it and collects input.
- **Preview is the simulation.** The preview forks the state and runs the *same* `step()`.
  Never write a separate "approximate" preview path.
- **Determinism.** Same state + same orders + same seed ⇒ same result. Single-player only, so
  same-machine f32 determinism is enough. Covered by tests from day one.
- **Orders are intentions, not controls.** Orders describe paths and speed profiles; a
  controller turns them into steering/throttle; the vehicle model enforces grip. If a path is
  too tight, the car physically fails to follow it, and the preview shows exactly that.
- **Ugly is fine.** Gizmos, flat shapes, debug text. No art until the fun gates pass.
- **Data-driven tuning.** All feel parameters in `assets/tuning.ron`, hot-reloaded.
- **Every milestone ends playable**, with a short playtest script and done criteria.

---

## 3. Architecture

```
carcombat/
  Cargo.toml            # workspace
  crates/
    sim/                # pure Rust, no Bevy: state, vehicle, road, orders, weapons, step()
      src/
        state.rs        # SimState, Car, Pilot, ids
        vehicle.rs      # bicycle model + friction circle + slide state
        control.rs      # path follower (pure pursuit / Stanley) + speed controller
        orders.rs       # OrderQueue, Maneuver, segments, locked window
        stability.rs    # drain/recovery, risk zones, loss-of-control events
        road.rs         # road graph, splines, SDF grid, terrain types
        planner/        # path fitting, racing line, Frenet sampling, reachable sets
        weapons.rs      # weapons, projectiles, hit chance, fire policies
        ai/             # utility-based sampling planner, personalities
        replay.rs       # seed + order log → deterministic replay
    game/               # Bevy app: rendering, input, timeline UI, debug tools
  assets/
    tuning.ron
    maps/
```

Start with just `sim` and `game`. Split `planner`/`ai` into crates only if compile times hurt.

### Core types (sketch)

```rust
pub struct SimState {
    pub tick: u64,
    pub cars: Vec<Car>,
    pub projectiles: Vec<Projectile>,
    pub hazards: Vec<Hazard>,      // mines, oil, debris
    pub rng: SimRng,               // seeded, part of state
}

pub struct Car {
    pub id: CarId,
    pub pose: Pose,                // position, heading
    pub vel: Vec2,
    pub yaw_rate: f32,
    pub slide: Option<SlideState>, // present while grip is exceeded / spinning
    pub stability: f32,
    pub controls: Controls,        // output of controller: steer, throttle/brake
    pub orders: OrderQueue,
    pub pilot: Pilot,              // reaction_time, skill
    pub weapons: Vec<Weapon>,
    pub damage: Damage,
}

pub enum Maneuver {
    Path { path: PathSpec, speed: SpeedProfile },   // from drag tool
    RoadFollow { d_profile: OffsetProfile, speed: SpeedProfile },
    Named(NamedManeuver),          // bootlegger, T-stop, controlled skid...
    Hold,                          // default when the queue is empty
}

pub enum StepMode { Live, Preview } // Preview never rolls dice; it records risk events

pub fn step(state: &mut SimState, world: &World, tuning: &Tuning, mode: StepMode)
    -> StepEvents;
```

`World` holds immutable data (map, road SDF). `StepEvents` collects hits, risk events,
collisions and loss of control, for UI, auto-pause triggers and metrics.

### Preview and enemies

When the player previews, enemies are **not** simulated with their real orders. The preview
replaces each enemy's order queue with its *predicted course* (see M6), so the preview never
leaks AI intent. The AI does the same to the player.

### Bevy side

- States: `Running`, `Paused`.
- `Running`: `FixedUpdate` calls `sim::step` once per tick.
- `Paused`: input edits orders → re-run preview (clone, step N ticks, keep a snapshot every
  k ticks) → draw ghosts, paths and timeline. Scrubbing only reads stored snapshots.

---

## 4. Milestones

Sizes: **S** ≈ a few evenings, **M** ≈ 1–2 weeks of evenings, **L** ≈ more.

### M0 — Foundations (S)

**Build**
- Workspace, `sim` crate, fixed-tick loop, Bevy app drawing cars as rectangles with gizmos.
- Vehicle model: kinematic bicycle with a friction-circle grip limit (combined longitudinal +
  lateral). When demand exceeds grip, enter a simple slide state (reduced steering authority,
  yaw drift). Upgrade to a dynamic bicycle model only if the feel requires it.
- Direct keyboard control (debug only), Running/Paused toggle.
- `tuning.ron` with hot reload; egui inspector for live state.

**Done when**
- Determinism test: clone state, step both copies 1000 ticks with the same inputs, states are
  bitwise equal.
- Car drives, brakes, slides believably with keyboard.

### M1 — Plan & preview one car (M)

**Build**
- Order queue with maneuver segments; path follower + speed controller.
- Drag tool: drag ghost car to end position, rotate handle for end heading, scroll for end
  speed. Path fitting: start with cubic Hermite / Bézier in world space; clothoids later if
  needed.
- Preview: fork + step 6 s, draw predicted path colored by grip demand
  (green < comfort < yellow < grip limit < red).
- Timeline bar: scrub to see the ghost car at any instant; edit pending segments.
- Replay recording (seed + order log) and a "preview vs. reality" debug check: with no
  randomness or enemies, live execution must match the preview exactly.

**Playable test: "Parking challenge"**
Reach marked target boxes (position + heading + max speed) in an empty arena, against a clock.

**Done when**
- Players hit targets reliably after a few minutes.
- Preview-vs-reality check passes.

**Watch for**: is dragging precise enough? Do players want waypoints mid-segment?

### M2 — Stability & loss of control (M) — **FUN GATE 1**

**Build**
- Stability gauge (sketch, all constants in tuning):
  - `demand = |a_combined| / (μ·g)`; `comfort = car.handling`
  - drain: `k_drain · max(0, demand − comfort)² + k_jerk · |Δsteer/Δt|`
  - recovery: `k_rec · pilot.skill` while `demand < comfort`
  - zones: green (safe), yellow (each risk event rolls a loss-of-control chance that grows as
    stability drops), ≤ 0 (guaranteed loss of control)
- Loss of control outcomes: skid, spin, roll (later), with severity by speed.
- Preview runs in `StepMode::Preview`: no rolls; shows risk markers on the path
  ("18% spin at 2.4 s") and a stability graph under the timeline.
- Named maneuvers as templates: bootlegger reverse, T-stop, controlled skid, swerve.

**Playable test: "Slalom"**
Gate course with a time limit tight enough that safe driving fails it.

**Gate questions**
- Do players push into yellow on purpose? Do crashes feel fair (preview warned them)?
- Is the pause → plan → watch rhythm enjoyable without any combat?
- If not: tune; if still not, rethink before continuing.

### M3 — Roads, terrain & racing line (M)

**Build**
- Road graph: spline centerlines with width/lanes; baked SDF grid (distance to edge, terrain
  type). Off-road terrain modifies grip and stability drain.
- Frenet frame utilities (s, d) per road edge.
- Minimum-curvature racing line per segment (small QP over lateral offsets).
- Drag tool on roads: dragging along the road means "follow road to here"; racing-line assist
  by default, modifier key to hold the current lane.

**Playable test: "Time trial"**
A winding road with a hairpin and a junction. Compare lap times with and without the assist.

**Done when**
- Assisted lines look like real driving lines; the drag tool feels faster than free-form.

### M4 — Reflexes & auto-pause (S)

**Build**
- On pause at tick t, orders before `t + reaction_time` are locked (greyed on timeline).
- Reaction time = pilot stat × modifiers (low stability, damage, threat outside view cone).
- Default behavior when orders run out: hold course/speed along the road.
- Auto-pause triggers (toggleable): enemy enters range, damage taken, orders ending,
  stability turns red.

**Playable test: "Hazard run"**
Scripted cars swerve in front of the player and debris drops onto the road at random times.

**Questions**
- Tune reaction time in the 0.3–1.0 s range: where does tension turn into frustration?
- Does the lock change how far ahead players plan?

### M5 — Weapons vs. scripted targets (M)

**Build**
- Weapons with range, firing arc, cooldown, ammo: fixed-forward gun, turret, rear mine
  dropper, oil slick.
- Hit chance (sketch): `p = base · f_range(r) · f_cross(ω_rel) · f_stab(shooter) · f_size(aspect)`.
- Fire policies per weapon: hold, fire at will, fire above X%, scheduled shot on timeline.
- Timeline shows each weapon's firing windows with hit chance.
- Basic armor per side + damage; simple car–car collision (OBB, impulse, damage, stability
  hit), since ramming is core to the genre.

**Playable test: "Convoy gallery"**
Scripted target cars on fixed routes; destroy as many as possible within the ammo limit.

**Questions**
- Is "fire above X%" the policy players actually use? Is fixed-forward aiming fun?

### M6 — Enemy uncertainty display (M)

**Build**
- Speed-aware current course: lane keeping when comfortable, racing line when too fast for
  the current offset, racing line + braking when too fast for both; blend in between.
- Reachable set per enemy, computed once per pause:
  - ~300 Frenet rollouts sampling offset profiles d(s) (entry/apex/exit knots), target speeds
    and timings, seeded with the racing line;
  - ~100 raw control rollouts for the off-road / loss-of-control tier;
  - every rollout runs through the real vehicle model and is tagged by worst stability and
    on/off-road.
- Rendering: density grid slice at the scrubbed time (not a convex hull, so obstacles carve
  it). Tiers: on-road green (solid), on-road risky (faded), off-road (hatched).
- Threat map: union of enemy weapon arcs over reachable states at time t.
- Player hit chance shown as nominal + range ("45%, 10–70%").
- Enemy layers fade out beyond ~3.5 s; own preview continues to ~6 s.

**Playable test**
Same scripted enemies as M5, now displayed only through the uncertainty view.

**Done when**
- Rollouts for 3 enemies take under ~50 ms in release builds (budget; measure).
- Players use the threat map to choose lines without being told to.

### M7 — AI v1 & first duel (L) — **FUN GATE 2**

**Build**
- Decision points: when the AI's locked window expires (same reflex rule as the player).
- Sampling planner: 10–30 candidate maneuvers from the same template library → simulate
  2–4 s against the *predicted* player (reachable-set model, never the player's real orders)
  → score → softmax pick among the best.
- Utility terms: expected damage dealt, expected damage taken (player threat map), stability
  risk, off-road penalty, position (in my arc / out of theirs), distance to cover.
- One personality to start ("balanced duelist").
- AI debug overlay: candidates, scores, chosen maneuver.

**Playable test: "1v1 duel"** on a road map and in a small arena.

**Gate questions**
- Does the AI feel like a driver, not a script? Does it ever surprise in a good way?
- Can a skilled player beat it by reading it, rather than by exploiting it?
- Is a full fight fun from start to finish?

### M8 — Vertical slice (L)

**Build**
- Personalities as utility weight sets: rammer, sniper, coward (flees at low armor), 4×4
  off-roader.
- Difficulty knobs: candidate count, horizon, prediction noise, reaction time.
- 1v2 and 1v3 encounters; two maps (arena + highway with junctions).
- Component damage (tires → grip, engine → acceleration, weapons → disabled).
- Metrics logging (see §5).

**Done when**
- Five playtesters each play three fights; most want to play another.

---

## 5. Tooling & playtest support

- **Replays** from seed + order log (from M1). Every bug report = a replay file.
- **Metrics per fight:** pauses per minute, time paused vs. running, % of maneuver time in
  yellow/red, loss-of-control count, shots fired by policy, hit rate, cause of death.
- **Debug overlays:** controller target, grip circle, stability graph, AI candidates, SDF.
- **Tests:** determinism, preview-vs-reality, path fitting edge cases, SDF queries,
  reachable-set coverage (the live enemy position should fall inside its predicted region
  in the vast majority of sampled situations).

---

## 6. Starting defaults (all in tuning.ron)

| Parameter | Start value |
|---|---|
| Tick rate | 60 Hz |
| Units | meters, seconds; top speed ~30–40 m/s |
| Own preview horizon | 6 s |
| Enemy prediction horizon | 3.5 s |
| Reaction time | 0.3–1.0 s (pilot stat) |
| Frenet / raw rollouts per enemy | 300 / 100 |
| Density grid cell | 0.5 m |
| AI candidates / horizon | 20 / 3 s |

---

## 7. Open questions (decide when the relevant milestone arrives)

- Vehicle model: is kinematic + slide state enough, or is a dynamic bicycle model needed? (M0–M2)
- How does the preview treat collisions with *predicted* enemies? Ignore, or warn? (M6)
- Line of sight: do buildings/cars block fire? (M5)
- How much randomness in loss-of-control outcomes is fun vs. unfair? (M2)
- Car design / loadout screen, campaign structure, art style: after Fun Gate 2.

---

## 8. Using this plan in a code session

1. Start with **M0** only. Set up the workspace and keep `sim` free of any Bevy dependency.
2. Write the determinism test before the vehicle model is tuned.
3. At the end of each milestone: run its playtest, record answers in `NOTES.md`, then adjust
   this plan before starting the next one.