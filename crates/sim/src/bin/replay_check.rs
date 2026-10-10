//! Re-simulates replay files and checks them against their recorded end state.
//! Usage: `cargo run -p sim --bin replay_check -- replays/*.ron`

use std::process::ExitCode;

use sim::{Replay, World};

fn main() -> ExitCode {
    let files: Vec<String> = std::env::args().skip(1).collect();
    if files.is_empty() {
        eprintln!("usage: replay_check <replay.ron>...");
        return ExitCode::FAILURE;
    }
    let mut ok = true;
    for file in &files {
        let result = std::fs::read_to_string(file)
            .map_err(|e| e.to_string())
            .and_then(|text| Replay::from_ron(&text).map_err(|e| e.to_string()))
            .and_then(|replay| {
                let ticks = replay.end_tick - replay.initial.tick;
                replay.verify(&World::default()).map(|()| ticks)
            });
        match result {
            Ok(ticks) => println!("OK    {file} ({ticks} ticks)"),
            Err(e) => {
                println!("FAIL  {file}: {e}");
                ok = false;
            }
        }
    }
    if ok { ExitCode::SUCCESS } else { ExitCode::FAILURE }
}
