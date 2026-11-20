mod actuators;
mod camera;
mod disturbance;
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

use crate::camera::{CameraSensor, CameraSnapshot, CameraSpec};
use crate::ros::conversions::sim_time_to_ros_time;
use crate::ros::publishers::VehicleRosHandles;
use crate::ros::subscribers::{
    create_disturbance_subscription, create_reaction_wheel_subscription,
    create_thruster_subscription,
};
use crate::ros::tf::TfPublishers;
use crate::simulation::{Simulation, VehicleConfig};

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
    // Several cameras: parallel lists of MJCF camera names, absolute topic namespaces
    // (<ns>/infra1/image_rect_raw) and optical frames. Empty: camera_name with the
    // original /sim/camera topics and sim_camera_infra1_optical_frame.
    let camera_names_param = node
        .declare_parameter::<Arc<[Arc<str>]>>("camera_names")
        .default(Arc::from([]))
        .read_only()?;
    let camera_namespaces_param = node
        .declare_parameter::<Arc<[Arc<str>]>>("camera_namespaces")
        .default(Arc::from([]))
        .read_only()?;
    let camera_frames_param = node
        .declare_parameter::<Arc<[Arc<str>]>>("camera_frames")
        .default(Arc::from([]))
        .read_only()?;
    let fps_param = node
        .declare_parameter("camera_fps")
        .default(90.0)
        .read_only()?;
    // Legacy single-robot form: applies to robot_1 only.
    let initial_pose_param = node
        .declare_parameter::<Arc<[f64]>>("initial_pose")
        .default(Arc::from([]))
        .read_only()?;
    // Vehicles to register. Each MuJoCo body is named after its vehicle; actuators
    // and sensors use "<prefix><local name>". Defaults keep the original model.
    let vehicles_param = node
        .declare_parameter::<Arc<[Arc<str>]>>("vehicles")
        .default(Arc::from([Arc::from("robot_1")]))
        .read_only()?;
    let prefixes_param = node
        .declare_parameter::<Arc<[Arc<str>]>>("vehicle_element_prefixes")
        .default(Arc::from([Arc::from("")]))
        .read_only()?;
    // Per-vehicle table-local [x, y, yaw] triples, in initial_pose_vehicles order.
    let initial_pose_vehicles_param = node
        .declare_parameter::<Arc<[Arc<str>]>>("initial_pose_vehicles")
        .default(Arc::from([]))
        .read_only()?;
    let initial_poses_param = node
        .declare_parameter::<Arc<[f64]>>("initial_poses")
        .default(Arc::from([]))
        .read_only()?;
    let disturbance_timeout_param = node
        .declare_parameter("disturbance_timeout_s")
        .default(0.25_f64)
        .read_only()?;
    let disturbance_timeout = disturbance_timeout_param.get();
    if !disturbance_timeout.is_finite() || disturbance_timeout <= 0.0 {
        return Err("disturbance_timeout_s must be finite and positive".into());
    }
    let model_path = model_param.get();
    let enable_camera = camera_param.get();
    let camera_names = camera_names_param.get();
    let camera_namespaces = camera_namespaces_param.get();
    let camera_frames = camera_frames_param.get();
    let camera_specs: Vec<CameraSpec> = if camera_names.is_empty() {
        if !camera_namespaces.is_empty() || !camera_frames.is_empty() {
            return Err("camera_namespaces/camera_frames require camera_names".into());
        }
        vec![CameraSpec::legacy(camera_name_param.get().as_ref())]
    } else {
        if camera_namespaces.len() != camera_names.len()
            || camera_frames.len() != camera_names.len()
        {
            return Err(
                "camera_names, camera_namespaces and camera_frames must have the same length"
                    .into(),
            );
        }
        camera_names
            .iter()
            .zip(camera_namespaces.iter())
            .zip(camera_frames.iter())
            .map(|((name, namespace), frame)| CameraSpec {
                name: name.to_string(),
                namespace: namespace.to_string(),
                optical_frame: frame.to_string(),
            })
            .collect()
    };

    let vehicle_names = vehicles_param.get();
    let prefixes = prefixes_param.get();
    if vehicle_names.len() != prefixes.len() {
        return Err(format!(
            "vehicles ({}) and vehicle_element_prefixes ({}) must have the same length",
            vehicle_names.len(),
            prefixes.len()
        )
        .into());
    }
    let vehicle_configs: Vec<VehicleConfig> = vehicle_names
        .iter()
        .zip(prefixes.iter())
        .map(|(name, prefix)| VehicleConfig::new(name.as_ref(), prefix.as_ref()))
        .collect();
    let mut initial_poses: Vec<(String, [f64; 3])> = Vec::new();
    let legacy_pose = initial_pose_param.get();
    if !legacy_pose.is_empty() {
        if legacy_pose.len() != 3 {
            return Err("initial_pose must be empty or [x_m, y_m, yaw_rad]".into());
        }
        initial_poses.push((
            "robot_1".into(),
            [legacy_pose[0], legacy_pose[1], legacy_pose[2]],
        ));
    }
    let pose_vehicles = initial_pose_vehicles_param.get();
    let pose_values = initial_poses_param.get();
    if pose_values.len() != 3 * pose_vehicles.len() {
        return Err(
            "initial_poses must contain [x_m, y_m, yaw_rad] for each initial_pose_vehicles entry"
                .into(),
        );
    }
    for (index, name) in pose_vehicles.iter().enumerate() {
        if initial_poses
            .iter()
            .any(|(other, _)| other == name.as_ref())
        {
            return Err(format!("initial pose for {name} is specified twice").into());
        }
        let values = &pose_values[3 * index..3 * index + 3];
        initial_poses.push((name.to_string(), [values[0], values[1], values[2]]));
    }

    let (mut simulation, mut viewer) = if viewer_param.get() {
        let (simulation, viewer) = Simulation::new_with_viewer(&model_path, &vehicle_configs)
            .map_err(std::io::Error::other)?;
        (simulation, Some(viewer))
    } else {
        (
            Simulation::with_vehicles(&model_path, &vehicle_configs)
                .map_err(std::io::Error::other)?,
            None,
        )
    };
    // Apply every pose before the first physics step.
    for (name, [x, y, yaw]) in &initial_poses {
        simulation
            .set_initial_pose(name, *x, *y, *yaw)
            .map_err(|error| std::io::Error::other(format!("initial pose for {name}: {error}")))?;
    }
    if !initial_poses.is_empty() {
        simulation.sync_viewer();
    }
    println!(
        "[sim] model={model_path}, camera={enable_camera}, vehicles={vehicle_configs:?}, \
         initial_poses={initial_poses:?}"
    );
    if enable_camera {
        println!("[sim] cameras={camera_specs:?}");
    }
    let mut camera = if enable_camera {
        Some(CameraSensor::new(
            &node,
            &model_path,
            &camera_specs,
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
            .map(CameraSensor::optical_transforms)
            .unwrap_or_default(),
    )?;

    let simulation_worker = node.create_worker(simulation);

    let mut thruster_subscriptions = Vec::new();
    let mut reaction_wheel_subscriptions = Vec::new();
    let mut disturbance_subscriptions = Vec::new();

    for vehicle_name in &vehicle_names {
        disturbance_subscriptions.push(create_disturbance_subscription(
            &simulation_worker,
            vehicle_name.clone(),
            disturbance_timeout,
        )?);
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
                if let Ok(applied) = simulation.applied_disturbance(vehicle_name) {
                    if let Err(error) = handles.disturbance_publisher.publish(&applied) {
                        eprintln!("[sim] failed to publish applied disturbance: {error}");
                    }
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
