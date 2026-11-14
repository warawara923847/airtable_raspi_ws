mod actuators;
mod camera;
mod model;
mod ros;
mod sensors;
mod simulation;
mod state;

use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
    time::Duration,
};

use rclrs::{Context, CreateBasicExecutor, IntoPrimitiveOptions, RclrsErrorFilter, SpinOptions};
use rosgraph_msgs::msg::Clock;
use tf2_msgs::msg::TFMessage;

use crate::camera::{CameraSensor, CameraSnapshot};
use crate::ros::conversions::sim_time_to_ros_time;
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
    let model_param = node
        .declare_parameter::<Arc<str>>("model_path")
        .default(concat!(env!("CARGO_MANIFEST_DIR"), "/mujoco/airtable.xml").into())
        .read_only()?;
    let viewer_param = node.declare_parameter("viewer").default(true).read_only()?;
    let camera_param = node
        .declare_parameter("enable_camera")
        .default(false)
        .read_only()?;
    let camera_name_param = node
        .declare_parameter::<Arc<str>>("camera_name")
        .default("tracking_camera".into())
        .read_only()?;
    let fps_param = node
        .declare_parameter("camera_fps")
        .default(30.0)
        .read_only()?;
    let model_path = model_param.get();
    let camera_name = camera_name_param.get();
    let enable_camera = camera_param.get();

    let (simulation, mut viewer) = if viewer_param.get() {
        let (simulation, viewer) =
            Simulation::new_with_viewer(&model_path).map_err(std::io::Error::other)?;
        (simulation, Some(viewer))
    } else {
        (
            Simulation::new(&model_path).map_err(std::io::Error::other)?,
            None,
        )
    };
    let mut camera = if enable_camera {
        Some(CameraSensor::new(
            &node,
            &model_path,
            &camera_name,
            fps_param.get(),
        )?)
    } else {
        None
    };
    let camera_snapshot = Arc::new(Mutex::new(CameraSnapshot::default()));
    let physics_snapshot = camera_snapshot.clone();
    let clock_publisher = node.create_publisher::<Clock>("/clock".keep_last(1))?;

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

    tf_publishers.publish_static_frames(
        &table_poses,
        camera
            .as_ref()
            .map(|sensor| sensor.optical_transform(&camera_name)),
    )?;

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
            let time = simulation.time();
            if let Err(error) = clock_publisher.publish(Clock {
                clock: sim_time_to_ros_time(time),
            }) {
                eprintln!("[sim] failed to publish clock: {error}");
            }
            if enable_camera {
                match physics_snapshot.lock() {
                    Ok(mut snapshot) => {
                        snapshot.time = time;
                        snapshot.qpos.clear();
                        snapshot.qpos.extend_from_slice(simulation.qpos());
                    }
                    Err(error) => eprintln!("[sim] camera snapshot lock failed: {error}"),
                }
            }
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
    while context.ok() {
        if let Some(viewer) = &mut viewer {
            if !viewer.running() {
                break;
            }
            viewer.render()?;
        }
        if let Some(camera) = &mut camera {
            // Release the physics lock before rendering or publishing an image.
            let snapshot = camera_snapshot
                .lock()
                .map_err(|e| std::io::Error::other(e.to_string()))?
                .clone();
            camera.publish(&snapshot)?;
        }
        if viewer.is_none() {
            std::thread::sleep(Duration::from_millis(2));
        }
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
