//! Deterministic replays: initial state + every command with the tick it was issued on.
//! Every bug report should be a replay file.

use serde::{Deserialize, Serialize};

use crate::orders::OrderQueue;
use crate::state::{CarId, Controls, DriveMode, SimState};
use crate::tuning::Tuning;
use crate::{StepMode, World, step};

/// The only ways the outside world changes the simulation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum Command {
    SetOrders { car: CarId, orders: OrderQueue },
    SetControls { car: CarId, controls: Controls },
    SetDriveMode { car: CarId, mode: DriveMode },
}

impl Command {
    pub fn apply(&self, state: &mut SimState) {
        match self {
            Command::SetOrders { car, orders } => {
                if let Some(c) = state.car_mut(*car) {
                    c.orders = orders.clone();
                }
            }
            Command::SetControls { car, controls } => {
                if let Some(c) = state.car_mut(*car) {
                    c.controls = *controls;
                }
            }
            Command::SetDriveMode { car, mode } => {
                if let Some(c) = state.car_mut(*car) {
                    c.drive_mode = *mode;
                }
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Replay {
    pub initial: SimState,
    pub tuning: Tuning,
    /// Commands, applied before stepping the tick they are tagged with.
    pub commands: Vec<(u64, Command)>,
    /// Tuning hot-reloads, applied like commands.
    pub tuning_changes: Vec<(u64, Tuning)>,
    /// Where the recording stopped, and the state digest expected there.
    pub end_tick: u64,
    pub end_digest: u64,
}

impl Replay {
    pub fn new(initial: SimState, tuning: Tuning) -> Self {
        let end_tick = initial.tick;
        let end_digest = digest(&initial);
        Self { initial, tuning, commands: Vec::new(), tuning_changes: Vec::new(), end_tick, end_digest }
    }

    pub fn record(&mut self, tick: u64, cmd: Command) {
        self.commands.push((tick, cmd));
    }

    pub fn record_tuning(&mut self, tick: u64, tuning: Tuning) {
        self.tuning_changes.push((tick, tuning));
    }

    /// Marks `state` as the end of the recording.
    pub fn finish(&mut self, state: &SimState) {
        self.end_tick = state.tick;
        self.end_digest = digest(state);
    }

    /// Re-simulates from the initial state up to `end_tick`.
    pub fn run(&self, world: &World) -> SimState {
        let mut state = self.initial.clone();
        let mut tuning = self.tuning.clone();
        let (mut ci, mut ti) = (0, 0);
        loop {
            while let Some((t, tn)) = self.tuning_changes.get(ti).filter(|(t, _)| *t <= state.tick) {
                debug_assert_eq!(*t, state.tick);
                tuning = tn.clone();
                ti += 1;
            }
            while let Some((_, cmd)) = self.commands.get(ci).filter(|(t, _)| *t <= state.tick) {
                cmd.apply(&mut state);
                ci += 1;
            }
            if state.tick >= self.end_tick {
                return state;
            }
            step(&mut state, world, &tuning, StepMode::Live);
        }
    }

    /// Re-simulates and checks the result against the recorded digest.
    pub fn verify(&self, world: &World) -> Result<(), String> {
        let state = self.run(world);
        let d = digest(&state);
        if d == self.end_digest {
            Ok(())
        } else {
            Err(format!("digest mismatch at tick {}: expected {:016x}, got {d:016x}", state.tick, self.end_digest))
        }
    }

    pub fn to_ron(&self) -> Result<String, ron::Error> {
        ron::ser::to_string_pretty(self, ron::ser::PrettyConfig::default())
    }

    pub fn from_ron(text: &str) -> Result<Self, ron::error::SpannedError> {
        ron::from_str(text)
    }
}

/// FNV-1a over the state's debug representation. `{:?}` prints floats as their shortest
/// round-trip form, so equal digests mean bitwise-equal states (in practice).
pub fn digest(state: &SimState) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in format!("{state:?}").bytes() {
        h ^= b as u64;
        h = h.wrapping_mul(0x0100_0000_01b3);
    }
    h
}
