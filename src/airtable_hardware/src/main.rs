mod devices;
mod drivers;
mod hardware;
mod io;
mod ros;

use hardware::HardwareState;
use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};
use std::time::Duration;

use rclrs::{
    Context, CreateBasicExecutor, IntoTimerOptions, RclrsErrorFilter, SpinOptions, TimerOptions,
};

use devices::imu::DummyImu;
use sensor_msgs::msg::Imu;
use sensor_msgs::msg::JointState;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // rclrs 0.7 has no signal handler. Return normally so the Drop of each output
    // (valves closed, PWM disabled, RW stopped) runs.
    let stopping = Arc::new(AtomicBool::new(false));
    for signal in [signal_hook::consts::SIGINT, signal_hook::consts::SIGTERM] {
        signal_hook::flag::register(signal, Arc::clone(&stopping))?;
    }
    let context = Context::default_from_env()?;

    let mut executor = context.create_basic_executor();

    let node = executor.create_node("airtable_hardware")?;

    let rw_parameters = ros::parameters::WheelParameters::declare(&node)?;
    let reaction_wheel = rw_parameters.create_wheel()?;
    let rw_enabled = reaction_wheel.is_some();
    let rw_speed_commanded = reaction_wheel
        .as_ref()
        .is_some_and(hardware::ReactionWheel::is_speed_commanded);
    let thruster_parameters = ros::parameters::ThrusterParameters::declare(&node)?;
    let thrusters = thruster_parameters.create_thrusters()?;
    let hardware = node.create_worker(HardwareState::new(thrusters, reaction_wheel));

    let (_rw_subscription, _rw_speed_subscription, _rw_timer) = if rw_enabled {
        // A wheel takes either torque or speed commands; subscribe to that topic only.
        let (subscription, speed_subscription) = if rw_speed_commanded {
            let speed_subscription = hardware.create_subscription(
                "actuators/reaction_wheels/speed_command",
                ros::subscribers::handle_reaction_wheel_speed_command,
            )?;
            (None, Some(speed_subscription))
        } else {
            let subscription = hardware.create_subscription(
                "actuators/reaction_wheels/command",
                ros::subscribers::handle_reaction_wheel_command,
            )?;
            (Some(subscription), None)
        };
        let publisher = node.create_publisher::<airtable_interfaces::msg::ReactionWheelState>(
            "actuators/reaction_wheels/state",
        )?;
        let clock = node.get_clock();
        // A steady timer keeps command timeout independent of /clock and use_sim_time.
        let timer = hardware.create_timer_repeating(
            Duration::from_millis(20),
            move |hardware: &mut HardwareState, _now: rclrs::Time| match hardware
                .update_reaction_wheel()
            {
                Ok(Some(state)) => {
                    let msg = ros::publishers::wheel_state_to_msg(state, clock.now());
                    if let Err(error) = publisher.publish(msg) {
                        eprintln!("[ros] RW state publish failed: {error}");
                    }
                }
                Ok(None) => {}
                Err(error) => eprintln!("[hardware] RW update failed: {error}"),
            },
        )?;
        (subscription, speed_subscription, Some(timer))
    } else {
        (None, None, None)
    };

    let _thruster_pulse_subscription = hardware.create_subscription(
        "actuators/thrusters/pulse",
        ros::subscribers::handle_thruster_pulse,
    )?;

    // This period bounds how late a pulse is closed. At 5 ms a 10 ms pulse was closed
    // 3 ms late on average on the Pi 5 (+30 % impulse).
    let _thruster_update_timer = hardware.create_timer_repeating(
        Duration::from_millis(1),
        move |hardware: &mut HardwareState, _now: rclrs::Time| {
            if let Err(error) = hardware.update_thrusters() {
                eprintln!("[hardware] thruster update failed: {}", error);
            }
        },
    )?;

    // ---- xy_table ----
    let _xy_subscription = hardware.create_subscription(
        "actuators/xy_table/trajectory",
        ros::subscribers::handle_xy_table_command,
    )?;

    // ---- sensor side ----

    let imu_publisher = node.create_publisher::<Imu>("imu/data_raw")?;
    // Namespaces do not rename frame IDs. Multi-robot launches set robot_N/imu_link;
    // the default keeps the frame used by the existing single-robot launch.
    let imu_frame_param = node
        .declare_parameter::<Arc<str>>("imu_frame_id")
        .default(Arc::from("imu_link"))
        .read_only()?;
    let imu_frame_id = imu_frame_param.get();
    if imu_frame_id.is_empty() {
        return Err("imu_frame_id must not be empty".into());
    }

    let mut imu = DummyImu::new();

    let _imu_timer =
        node.create_timer_repeating(Duration::from_millis(20), move |now: rclrs::Time| {
            let sample = imu.read();

            let msg = ros::publishers::imu_sample_to_msg(sample, now, &imu_frame_id);

            if let Err(error) = imu_publisher.publish(msg) {
                eprintln!("[ros] Failed to publish IMU: {}", error);
            }
        })?;

    let joint_state_publisher = node.create_publisher::<JointState>("joint_states")?;

    let joint_state_publisher_for_timer = joint_state_publisher.clone();

    let _joint_state_timer = hardware.create_timer_repeating(
        TimerOptions::new(Duration::from_millis(50)).node_time(),
        move |hardware: &mut HardwareState, now: rclrs::Time| {
            let (x, y) = hardware.xy_table_position();

            let msg = ros::publishers::xy_table_to_joint_state(x, y, now);

            if let Err(error) = joint_state_publisher_for_timer.publish(msg) {
                eprintln!("[ros] Failed to publish JointState: {}", error);
            }
        },
    )?;

    while !stopping.load(Ordering::Relaxed) {
        executor
            .spin(SpinOptions::default().timeout(Duration::from_millis(50)))
            .timeout_ok()
            .first_error()?;
    }

    Ok(())
}
