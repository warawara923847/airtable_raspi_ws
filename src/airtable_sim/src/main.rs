mod actuators;
mod model;
mod ros;
mod sensors;
mod simulation;
mod state;

use std::{collections::HashMap, time::Duration};

use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, RclrsErrorFilter, SpinOptions};
use tf2_msgs::msg::TFMessage;

use crate::ros::publishers::VehicleRosHandles;
use crate::ros::subscribers::{create_reaction_wheel_subscription, create_thruster_subscription};
use crate::ros::tf::TfPublishers;
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
    let vehicle_names = simulation.vehicle_names();
    let mut vehicle_ros_handles = HashMap::new();

    for vehicle_name in &vehicle_names {
        let table_frame = simulation.vehicle_table(vehicle_name)?.name().to_string();

        let imu_names = simulation.imu_names(vehicle_name)?;

        let handles = VehicleRosHandles::new(&node, vehicle_name, table_frame, imu_names)?;

        vehicle_ros_handles.insert(vehicle_name.clone(), handles);
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

    let mut thruster_subscriptions = Vec::new();
    let mut reaction_wheel_subscriptions = Vec::new();

    for vehicle_name in &vehicle_names {
        let thruster_subscription =
            create_thruster_subscription(&simulation_worker, vehicle_name.clone())?;
        thruster_subscriptions.push(thruster_subscription);

        println!("[sim] created thruster subscription for {}", vehicle_name);

        let reaction_wheel_subscription =
            create_reaction_wheel_subscription(&simulation_worker, vehicle_name.clone())?;
        reaction_wheel_subscriptions.push(reaction_wheel_subscription);

        println!(
            "[sim] created reaction wheel subscription for {}",
            vehicle_name
        );
    }

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

                // ------------------------------------------------
                // RW state
                // ------------------------------------------------
                let rw_states = match simulation.reaction_wheel_states(vehicle_name) {
                    Ok(states) => states,
                    Err(error) => {
                        eprintln!(
                            "[sim] failed to get RW states fo {}: {}",
                            vehicle_name, error
                        );

                        continue;
                    }
                };

                if let Err(error) = handles.rw_state_publisher.publish(state.time, &rw_states) {
                    eprintln!(
                        "[sim] failed to publish RW state for {}: {}",
                        vehicle_name, error
                    );
                }

                // ------------------------------------------------
                // IMU state
                // ------------------------------------------------
                let imu_states = match simulation.imu_states(vehicle_name) {
                    Ok(states) => states,
                    Err(error) => {
                        eprintln!(
                            "[sim] failed to get IMU states for {}: {}",
                            vehicle_name, error
                        );

                        continue;
                    }
                };

                for imu_state in &imu_states {
                    let Some(publisher) = handles.imu_publishers.get(imu_state.name.as_str())
                    else {
                        eprintln!(
                            "[sim] no IMU publisher for {}/{}",
                            vehicle_name, imu_state.name
                        );
                        continue;
                    };

                    if let Err(error) = publisher.publish(state.time, imu_state) {
                        eprintln!(
                            "[sim] failed to publish IMU state for {}/{}: {}",
                            vehicle_name, imu_state.name, error
                        );
                    }
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
