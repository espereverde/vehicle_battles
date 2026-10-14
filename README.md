# Vehicle Battles (prototype)

Continuous-time, pausable top-down car combat. See [WORK_PLAN.md](WORK_PLAN.md).

```sh
cargo run -p game                 # play (first build takes a few minutes)
cargo run -p game --features dyn  # faster incremental builds (Bevy dynamic linking)
cargo test -p sim                 # simulation tests (no Bevy, fast)
cargo run -p sim --bin replay_check -- replays/*.ron   # re-verify saved replays
```

## Controls

The game starts paused. Plan, then press Space to watch it play out.

| Input | Action |
|---|---|
| Space | run / pause (resuming commits the plan) |
| LMB on ground | add a keypoint there (drag to place it) |
| LMB drag a keypoint ghost | further from the car = later, sideways = more turn |
| Wheel | throttle ±5% of the dragged, hovered or selected keypoint (negative = brake) |
| T / "Steering · Drift · Skid" buttons | turn mode: constant steering (a constant turn radius); drift (Car Wars: moves sideways at most ½" per inch forward and ends pointing the way the car started; on the ground an S-curve of two arcs of equal length; costs handling by steepness per inch, not by its turning); or controlled skid (the car keeps going the way it was for ¼–1 s (the book's ¼"–1" slide) while its nose swings round over the first 0.25 s, up to 90°, then drives off along the new nose; costs 1 handling point per 15° plus 1 per ¼" of slide; once started it can't be changed) |
| S / "Straight" button | selected keypoint drives straight at its start speed; with none selected, adds a 1 s straight keypoint |
| RMB on a ghost, Del | delete keypoint |
| Esc | revert the plan to what the car is currently doing |
| `.` | single tick |
| Timeline (bottom) | drag to scrub the preview ghost; click a span to select its keypoint |
| Ctrl+Wheel / MMB drag / F | zoom / pan / toggle camera follow |
| C | cycle test track → parking challenge → slalom |
| R / N | restart / new parking course |
| M | manual keyboard driving (WASD, Shift handbrake, Ctrl push) |
| F5 / F9 | save replay to `replays/` / re-simulate and verify in memory |

Tuning: edit `assets/tuning.ron` while the game runs; changes apply on save.

Handling (M2, after Car Wars): the bar under the car and the graph under the timeline show
handling status, from −6 to the car's handling class (tick at 0). Turning drains one point per
15° of heading change, and braking harder than 10 mph/s drains too; it recovers at HC points per
second. Every point drained is a control check whose odds depend on handling status and speed
(the Control Table): risky once status is low for the speed (skid when slow, spin when fast).
Colors: green safe, yellow maneuvering here would be risky, orange risky, red certain loss. The
preview never rolls: it marks the risk on the path ("34% spin at 2.1 s"), under each keypoint
("D 3.2 · 18% loss": handling drained by its segment and its chance of losing control) and, for
each second, on the timeline graph.

Losing control rolls the Car Wars crash tables (2d6 + maneuver difficulty − 3 + speed modifier −
driver skill): skids slide straight on along the way the car was going (in a bend, the
tangent) while the nose keeps turning as it was, then ease back onto the nose (orange car), spinouts spin 90° per 0.2 s and slow to a stop unless the driver recovers (red),
rolls go sideways and stop (red, crossed), vaults fly first (white); burning shows a ring.
Controlled skids show cyan. Path labels give the chance of losing control and of a spinout or
worse.

Slalom: `assets/maps/slalom.ron` (gates, time limit, penalty).

Test track: `assets/maps/track.ron`, built from straights and arcs starting at the origin heading
east (loaded at startup). Off-track is counted and shown but has no physical effect until M3.
