mod ros;
mod simulation;
mod state;
mod table;
mod thruster;
mod vehicle;

use std::{collections::HashMap, time::Duration};

use nav_msgs::msg::Odometry;
use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};

use crate::ros::publishers::GroundTruthPublisher;
use crate::simulation::Simulation;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    //
    // ROS2 context
    //
    let context = Context::default_from_env()?;

    let mut executor = context.create_basic_executor();

    let node = executor.create_node("airtable_sim")?;

    //
    // Mujoco simulation
    //
    let model_path = concat!(env!("CARGO_MANIFEST_DIR"), "/mujoco/airtable.xml");

    let (simulation, mut viewer) =
        Simulation::new_with_viewer(model_path).map_err(std::io::Error::other)?;

    // ------------------------------------------------
    // Ground Truth publishers
    // ------------------------------------------------
    let mut gt_publishers = HashMap::new();

    for vehicle_name in simulation.vehicle_names() {
        let table_name = simulation.vehicle_table(&vehicle_name)?.name().to_string();

        let topic = format!("/{vehicle_name}/sim/ground_truth");

        let child_frame_id = format!("{vehicle_name}/base_link");

        let publisher = node.create_publisher::<Odometry>(topic.as_str())?;

        println!(
            "[sim] GT: {} -> {} [{} -> {}]",
            vehicle_name, topic, table_name, child_frame_id,
        );

        gt_publishers.insert(
            vehicle_name,
            GroundTruthPublisher::new(publisher, table_name, child_frame_id),
        );
    }

    let simulation_worker = node.create_worker(simulation);

    let _thruster_pulse_subscription = simulation_worker.create_subscription(
        "actuators/thrusters/pulse",
        ros::subscribers::handle_thruster_pulse,
    )?;

    println!("[sim] thruster subscription created");

    let _simulation_timer = simulation_worker.create_timer_repeating(
        Duration::from_millis(2),
        |simulation: &mut Simulation, _time: rclrs::Time| {
            simulation.step();
        },
    )?;

    let _viewer_sync_timer = simulation_worker.create_timer_repeating(
        Duration::from_millis(16),
        |simulation: &mut Simulation, _time: rclrs::Time| {
            simulation.sync_viewer();
        },
    )?;

    let _ground_truth_timer = simulation_worker.create_timer_repeating(
        Duration::from_millis(10),
        move |simulation: &mut Simulation, _time: rclrs::Time| {
            for (vehicle_name, publisher) in &gt_publishers {
                let state = match simulation.vehicle_state(vehicle_name) {
                    Ok(state) => state,
                    Err(error) => {
                        eprintln!("[sim] failed to get GT for {}: {}", vehicle_name, error);
                        continue;
                    }
                };

                if let Err(error) = publisher.publish(&state) {
                    eprintln!("[sim] failed to publish GT for {}: {}", vehicle_name, error);
                }
            }
        },
    )?;

    // ------------------------------------------------
    // ROS executor -> another thread
    // ------------------------------------------------
    let executor_commands = executor.commands().clone();

    let executor_thread = std::thread::spawn(move || executor.spin(SpinOptions::default()));
    // ------------------------------------------------
    // Viewer -> main thread
    // ------------------------------------------------
    while viewer.running() {
        viewer.render()?;
    }

    // Viewer window was closed:
    // stop ROS executor
    executor_commands.halt_spinning();

    let errors = executor_thread
        .join()
        .map_err(|_| std::io::Error::other("ROS executor thread panicked"))?;

    errors.first_error()?;

    Ok(())
}
