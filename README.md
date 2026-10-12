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
| Z / "Danger zones" checkbox | shade where the selected (or next) keypoint can go, in the grip color its path would get |
| X / "Drift" checkbox | where the path is orange/red, shade how far out the real car could run wide (orange) or slide (red) |
| T / "Steering · G-force" buttons | turn mode: constant steering or constant lateral g |
| S / "Straight" button | selected keypoint drives straight at its start speed; with none selected, adds a 1 s straight keypoint |
| RMB on a ghost, Del | delete keypoint |
| Esc | revert the plan to what the car is currently doing |
| `.` | single tick |
| Timeline (bottom) | drag to scrub the preview ghost; click a span to select its keypoint |
| Ctrl+Wheel / MMB drag / F | zoom / pan / toggle camera follow |
| C | switch between the test track and the parking challenge |
| R / N | restart / new parking course |
| M | manual keyboard driving (WASD, Shift handbrake, Ctrl push) |
| F5 / F9 | save replay to `replays/` / re-simulate and verify in memory |

Tuning: edit `assets/tuning.ron` while the game runs; changes apply on save.

Test track: `assets/maps/track.ron`, built from straights and arcs starting at the origin heading
east (loaded at startup). Off-track is counted and shown but has no physical effect until M3.
