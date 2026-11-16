use crate::devices::imu::ImuSample;
use crate::devices::reaction_wheel::WheelState;
use airtable_interfaces::msg::ReactionWheelState;

use rclrs::Time;
use sensor_msgs::msg::Imu;
use sensor_msgs::msg::JointState;

pub fn wheel_state_to_msg(state: WheelState, stamp: Time) -> ReactionWheelState {
    let mut msg = ReactionWheelState::default();
    let (sec, nanosec) = stamp.to_sec_nanosec().expect("failed to convert ROS time");
    msg.stamp.sec = sec;
    msg.stamp.nanosec = nanosec;
    msg.names = vec![state.name];
    // All RW feedback fields describe the virtual model, not the SG90 shaft.
    msg.angular_velocities = vec![state.angular_velocity];
    msg.commanded_torques = vec![state.commanded_torque];
    msg.applied_torques = vec![state.applied_torque];
    msg
}

pub fn imu_sample_to_msg(sample: ImuSample, stamp: Time, frame_id: &str) -> Imu {
    let mut msg = Imu::default();

    let (sec, nanosec) = stamp.to_sec_nanosec().expect("failed to convert ROS time");

    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;
    msg.header.frame_id = frame_id.to_string();

    msg.angular_velocity.x = sample.angular_velocity_rad_s[0];
    msg.angular_velocity.y = sample.angular_velocity_rad_s[1];
    msg.angular_velocity.z = sample.angular_velocity_rad_s[2];

    msg.linear_acceleration.x = sample.linear_acceleration_m_s2[0];
    msg.linear_acceleration.y = sample.linear_acceleration_m_s2[1];
    msg.linear_acceleration.z = sample.linear_acceleration_m_s2[2];

    // Dummy IMUでは姿勢は提供しない
    msg.orientation_covariance[0] = -1.0;

    msg
}

pub fn xy_table_to_joint_state(x: f64, y: f64, stamp: rclrs::Time) -> JointState {
    let mut msg = JointState::default();

    let (sec, nanosec) = stamp.to_sec_nanosec().expect("failed to convert ROS time");

    msg.header.stamp.sec = sec;
    msg.header.stamp.nanosec = nanosec;

    msg.name = vec!["table_x".to_string(), "table_y".to_string()];

    msg.position = vec![x, y];

    msg
}
