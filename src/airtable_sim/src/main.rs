mod ros;
mod simulation;
mod state;
mod table;
mod thruster;
mod vehicle;

use std::{collections::HashMap, time::Duration};

use nav_msgs::msg::Odometry;
use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, RclrsErrorFilter, SpinOptions};
use tf2_msgs::msg::TFMessage;

use crate::ros::publishers::GroundTruthPublisher;
use crate::ros::tf::TfPublishers;
use crate::simulation::Simulation;

struct VehicleRosHandles {
    gt_publisher: GroundTruthPublisher,
    table_frame: String,
}

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
    let mut vehicle_ros_handles = HashMap::new();

    for vehicle_name in simulation.vehicle_names() {
        let table_frame = simulation.vehicle_table(&vehicle_name)?.name().to_string();

        let topic = format!("/{vehicle_name}/sim/ground_truth");

        let child_frame_id = format!("{vehicle_name}/base_link");

        let publisher = node.create_publisher::<Odometry>(topic.as_str())?;

        let gt_publisher =
            GroundTruthPublisher::new(publisher, table_frame.clone(), child_frame_id);
        vehicle_ros_handles.insert(
            vehicle_name,
            VehicleRosHandles {
                gt_publisher,
                table_frame,
            },
        );
    }

    // ------------------------------------------------
    // TF publishers
    // ------------------------------------------------
    let static_tf_publisher =
        node.create_publisher::<TFMessage>("/tf_static".keep_last(1).transient_local())?;

    let dynamic_tf_publisher = node.create_publisher::<TFMessage>("/tf".keep_last(100))?;

    // Table TF publish
    let table_poses = simulation.table_poses();

    let tf_publishers = TfPublishers::new(dynamic_tf_publisher, static_tf_publisher);

    tf_publishers.publish_static_tables(&table_poses)?;

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

    let _state_publish_timer = simulation_worker.create_timer_repeating(
        Duration::from_millis(10),
        move |simulation: &mut Simulation, _time: rclrs::Time| {
            for (vehicle_name, handles) in &vehicle_ros_handles {
                let state = match simulation.vehicle_state(vehicle_name) {
                    Ok(state) => state,
                    Err(error) => {
                        eprintln!("[sim] failed to get GT for {}: {}", vehicle_name, error);
                        continue;
                    }
                };

                // ------------------------------------------------
                // Ground Truth
                // ------------------------------------------------
                if let Err(error) = handles.gt_publisher.publish(&state) {
                    eprintln!("[sim] failed to publish GT for {}: {}", vehicle_name, error);
                }

                // ------------------------------------------------
                // Dynamic TF
                // table_N -> robot_N/base_link
                // ------------------------------------------------
                if let Err(error) =
                    tf_publishers.publish_vehicle(&handles.table_frame, vehicle_name, &state)
                {
                    eprintln!("[sim] failed to publish TF for {}: {}", vehicle_name, error);
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
