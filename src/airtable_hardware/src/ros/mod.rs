use airtable_interfaces::msg::ThrusterSetpoint;

use crate::hardware::HardwareState;

pub fn handle_thruster_setpoint(hardware: &mut HardwareState, msg: ThrusterSetpoint) {
    if msg.names.len() != msg.thrust.len() {
        eprintln!(
            "[ros] Invalid ThrusterSetpoint: names={}, thrust={}",
            msg.names.len(),
            msg.thrust.len()
        );

        return;
    }

    for (name, thrust_n) in msg.names.iter().zip(msg.thrust.iter()) {
        if let Err(error) = hardware.set_thruster(name, *thrust_n) {
            eprintln!("[ros] Failed to command {}: {}", name, error);
        }
    }
}
