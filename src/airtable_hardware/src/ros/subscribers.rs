use std::time::Duration;

use crate::hardware::HardwareState;

use airtable_interfaces::msg::ThrusterPulseCommand;
use trajectory_msgs::msg::JointTrajectory;

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
