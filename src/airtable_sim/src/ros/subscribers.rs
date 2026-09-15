use airtable_interfaces::msg::ThrusterPulseCommand;

use crate::simulation::Simulation;

pub fn handle_thruster_pulse(simulation: &mut Simulation, msg: ThrusterPulseCommand) {
    if msg.pulse_duration.sec < 0 {
        eprintln!("[sim] negative pulse duration: {}", msg.pulse_duration.sec);
        return;
    }

    let duration_s = msg.pulse_duration.sec as f64 + msg.pulse_duration.nanosec as f64 * 1e-9;

    if duration_s <= 0.0 {
        return;
    }

    for name in &msg.names {
        if let Err(error) = simulation.fire_thruster(name, duration_s) {
            eprintln!("[sim] error firing thruster {}: {}", name, error);
        }
    }
}
