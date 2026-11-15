use airtable_interfaces::msg::ReactionWheelCommand;
use airtable_interfaces::msg::ThrusterPulseCommand;
use geometry_msgs::msg::WrenchStamped;

use crate::simulation::Simulation;

pub fn create_disturbance_subscription(
    worker: &rclrs::Worker<Simulation>,
    vehicle: String,
    timeout: f64,
) -> Result<rclrs::WorkerSubscription<WrenchStamped, Simulation>, rclrs::RclrsError> {
    let topic = format!("/{vehicle}/sim/disturbance/wrench");
    worker.create_subscription(
        topic.as_str(),
        move |sim: &mut Simulation, msg: WrenchStamped| {
            let force = &msg.wrench.force;
            let torque = &msg.wrench.torque;
            if force.z != 0.0
                || torque.x != 0.0
                || torque.y != 0.0
                || msg.header.stamp.sec < 0
                || msg.header.stamp.nanosec >= 1_000_000_000
            {
                sim.clear_disturbance(&vehicle);
                eprintln!("[sim] rejected non-planar or invalid disturbance command");
                return;
            }
            let stamp = if msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0 {
                None
            } else {
                Some(msg.header.stamp.sec as f64 + msg.header.stamp.nanosec as f64 * 1e-9)
            };
            if let Err(error) = sim.set_disturbance(
                &vehicle,
                &msg.header.frame_id,
                [force.x, force.y, torque.z],
                stamp,
                timeout,
            ) {
                eprintln!("[sim] rejected disturbance: {error}");
            }
        },
    )
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
