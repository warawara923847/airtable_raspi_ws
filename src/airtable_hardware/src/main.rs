mod devices;
mod drivers;
mod hardware;
mod io;
mod ros;

use hardware::HardwareState;
use std::time::Duration;

use rclrs::{
    Context, CreateBasicExecutor, IntoTimerOptions, RclrsErrorFilter, SpinOptions, TimerOptions,
};

use devices::imu::DummyImu;
use sensor_msgs::msg::Imu;
use sensor_msgs::msg::JointState;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env()?;

    let mut executor = context.create_basic_executor();

    let node = executor.create_node("airtable_hardware")?;

    let hardware = node.create_worker(HardwareState::new_dummy());

    let _thruster_pulse_subscription = hardware.create_subscription(
        "actuators/thrusters/pulse",
        ros::subscribers::handle_thruster_pulse,
    )?;

    let _thruster_update_timer = hardware.create_timer_repeating(
        Duration::from_millis(5),
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

    let mut imu = DummyImu::new();

    let _imu_timer =
        node.create_timer_repeating(Duration::from_millis(20), move |now: rclrs::Time| {
            let sample = imu.read();

            let msg = ros::publishers::imu_sample_to_msg(sample, now);

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

    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}
