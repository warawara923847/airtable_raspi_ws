use airtable_interfaces::msg::ReactionWheelCommand;
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
        if let Err(error) = simulation.fire_thruster("robot_1", name, duration_s) {
            eprintln!("[sim] error firing thruster {}: {}", name, error);
        }
    }
}

pub fn create_thruster_subscription(
    simulation_worker: &rclrs::Worker<Simulation>,
    vehicle_name: String,
) -> Result<rclrs::WorkerSubscription<ThrusterPulseCommand, Simulation>, rclrs::RclrsError> {
    let topic = format!("/{vehicle_name}/actuators/thrusters/pulse");

    simulation_worker.create_subscription(
        topic.as_str(),
        move |simulation: &mut Simulation, msg: ThrusterPulseCommand| {
            if msg.pulse_duration.sec < 0 {
                eprintln!("[sim] negative pulse duration: {}", msg.pulse_duration.sec);
                return;
            }

            let duration_s =
                msg.pulse_duration.sec as f64 + msg.pulse_duration.nanosec as f64 * 1e-9;

            if duration_s <= 0.0 {
                return;
            }

            for name in &msg.names {
                if let Err(error) = simulation.fire_thruster(&vehicle_name, name, duration_s) {
                    eprintln!(
                        "[sim] error firing thruster {}/{}: {}",
                        vehicle_name, name, error,
                    );
                }
            }
        },
    )
}

pub fn create_reaction_wheel_subscription(
    simulation_worker: &rclrs::Worker<Simulation>,
    vehicle_name: String,
) -> Result<rclrs::WorkerSubscription<ReactionWheelCommand, Simulation>, rclrs::RclrsError> {
    let topic = format!("/{}/actuators/reaction_wheels/command", vehicle_name);

    simulation_worker.create_subscription(
        topic.as_str(),
        move |simulation: &mut Simulation, msg: ReactionWheelCommand| {
            if msg.names.len() != msg.torques.len() {
                eprintln!(
                    "[sim] invalid RW command for {}: \
                        names.len() = {} torques.len() = {}",
                    vehicle_name,
                    msg.names.len(),
                    msg.torques.len(),
                );
                return;
            }

            for (name, torque) in msg.names.iter().zip(msg.torques.iter()) {
                if let Err(error) =
                    simulation.set_reaction_wheel_torque(&vehicle_name, name, *torque)
                {
                    eprintln!(
                        "[sim] failed to set RW torque \
                        {}/{}: {}",
                        vehicle_name, name, error,
                    );
                }
            }
        },
    )
}
