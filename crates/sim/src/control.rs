//! Turns orders into controls. Each segment holds constant controls; the vehicle model
//! decides what the tires make of them.

use crate::state::{Car, DriveMode};
use crate::tuning::VehicleTuning;

/// Sets the car's controls for this tick from its orders. Returns how many segments were
/// completed (their last tick is this one).
pub fn drive(car: &mut Car, vt: &VehicleTuning) -> usize {
    if car.drive_mode != DriveMode::Orders {
        return 0;
    }
    let v = car.forward_speed();
    let orders = &mut car.orders;
    let mut completed = 0;
    // Zero-length segments (never created by the game, but possible in a replay file).
    while orders.segments.first().is_some_and(|s| s.remaining() == 0) {
        orders.tail = orders.segments.remove(0).drive;
        completed += 1;
    }
    car.controls = orders.active_drive().controls(v, vt);
    if let Some(seg) = orders.segments.first_mut() {
        seg.elapsed += 1;
        if seg.remaining() == 0 {
            orders.tail = orders.segments.remove(0).drive;
            completed += 1;
        }
    }
    completed
}
