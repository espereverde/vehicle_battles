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
| LMB on ground | add a segment ending there (drag to place it) |
| LMB drag on a ghost car / its knob | move the segment end / set its end heading |
| Wheel over a ghost | end speed ±1 m/s (Shift: cruise speed) |
| RMB on a ghost, Del | delete segment |
| Esc | revert the plan to what the car is currently doing |
| `.` | single tick |
| Timeline (bottom) | drag to scrub the preview ghost; click a span to select its segment |
| Wheel / MMB drag / F | zoom / pan / toggle camera follow |
| R / N | restart the course / new course |
| M | manual keyboard driving (WASD, Shift handbrake, Ctrl push) |
| F5 / F9 | save replay to `replays/` / re-simulate and verify in memory |

Tuning: edit `assets/tuning.ron` while the game runs; changes apply on save.
