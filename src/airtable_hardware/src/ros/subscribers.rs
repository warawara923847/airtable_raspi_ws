use std::time::Duration;

use crate::hardware::HardwareState;

use airtable_interfaces::msg::ReactionWheelCommand;
use airtable_interfaces::msg::ReactionWheelSpeedCommand;
use airtable_interfaces::msg::ThrusterPulseCommand;
use trajectory_msgs::msg::JointTrajectory;

/// Paired arrays with unique names and finite values.
fn valid_named_values(names: &[String], values: &[f64]) -> bool {
    names.len() == values.len()
        && values.iter().all(|value| value.is_finite())
        && !names
            .iter()
            .enumerate()
            .any(|(i, name)| names[..i].contains(name))
}

pub fn handle_reaction_wheel_command(hardware: &mut HardwareState, msg: ReactionWheelCommand) {
    // Validate the complete message before changing output or refreshing the watchdog.
    if !valid_named_values(&msg.names, &msg.torques) {
        eprintln!(
            "[ros] malformed RW command: mismatched arrays, duplicate names or non-finite torque"
        );
        return;
    }
    for (name, torque) in msg.names.iter().zip(&msg.torques) {
        if let Err(error) = hardware.set_reaction_wheel_torque(name, *torque) {
            eprintln!("[ros] RW command rejected: {error}");
        }
    }
}

pub fn handle_reaction_wheel_speed_command(
    hardware: &mut HardwareState,
    msg: ReactionWheelSpeedCommand,
) {
    if !valid_named_values(&msg.names, &msg.speeds)
        || msg.speeds.iter().any(|speed| speed.abs() > 1.0)
    {
        eprintln!(
            "[ros] malformed RW speed command: mismatched arrays, duplicate names \
             or speed outside [-1, 1]"
        );
        return;
    }
    for (name, speed) in msg.names.iter().zip(&msg.speeds) {
        if let Err(error) = hardware.set_reaction_wheel_speed(name, *speed) {
            eprintln!("[ros] RW speed command rejected: {error}");
        }
    }
}

pub fn handle_thruster_pulse(hardware: &mut HardwareState, msg: ThrusterPulseCommand) {
    if msg.pulse_duration.sec < 0 {
        eprintln!("[ros] negative pulse duration");
        return;
    }

    let duration = Duration::new(msg.pulse_duration.sec as u64, msg.pulse_duration.nanosec);

    if duration.is_zero() {
        return;
    }

    for name in &msg.names {
        if let Err(error) = hardware.fire_thruster(name, duration) {
            eprintln!("[ros] failed to fire {}: {}", name, error);
        }
    }
}

pub fn handle_xy_table_command(hardware: &mut HardwareState, msg: JointTrajectory) {
    let Some(point) = msg.points.last() else {
        eprintln!("[ros] JointTrajectory has no points");
        return;
    };

    let mut x = None;
    let mut y = None;

    for (index, name) in msg.joint_names.iter().enumerate() {
        let Some(position) = point.positions.get(index) else {
            eprintln!("[ros] Position missing for joint {name}");
            return;
        };

        match name.as_str() {
            "table_x" => x = Some(*position),
            "table_y" => y = Some(*position),
            _ => {}
        }
    }

    let (Some(x), Some(y)) = (x, y) else {
        eprintln!("[ros] table_x/table_y missing");
        return;
    };

    if let Err(error) = hardware.move_xy_table(x, y) {
        eprintln!("[ros] XY table command failed: {error}");
    }
}
