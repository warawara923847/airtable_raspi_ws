use airtable_interfaces::msg::ReactionWheelState;
use nav_msgs::msg::Odometry;
use rclrs::Publisher;

use crate::actuators::ReactionWheelStateData;
use crate::state::PlanarState;

pub struct GroundTruthPublisher {
    publisher: Publisher<Odometry>,

    frame_id: String,
    child_frame_id: String,
}

pub struct ReactionWheelStatePublisher {
    publisher: Publisher<ReactionWheelState>,
}

impl GroundTruthPublisher {
    pub fn new(
        publisher: Publisher<Odometry>,
        frame_id: impl Into<String>,
        child_frame_id: impl Into<String>,
    ) -> Self {
        Self {
            publisher,
            frame_id: frame_id.into(),
            child_frame_id: child_frame_id.into(),
        }
    }

    pub fn publish(&self, state: &PlanarState) -> Result<(), rclrs::RclrsError> {
        let mut msg = Odometry::default();

        // ------------------------------------------------
        // Timestamp
        // ------------------------------------------------
        let (sec, nanosec) = simulation_time_to_ros_time(state.time);

        msg.header.stamp.sec = sec;
        msg.header.stamp.nanosec = nanosec;

        // pose is expressed in world frame
        msg.header.frame_id = self.frame_id.clone();

        // twist is expressed in vehicle frame
        msg.child_frame_id = self.child_frame_id.clone();

        // ------------------------------------------------
        // Pose
        // ------------------------------------------------
        msg.pose.pose.position.x = state.x;
        msg.pose.pose.position.y = state.y;
        msg.pose.pose.position.z = 0.0;

        let half_yaw = state.yaw * 0.5;

        msg.pose.pose.orientation.x = 0.0;
        msg.pose.pose.orientation.y = 0.0;
        msg.pose.pose.orientation.z = half_yaw.sin();
        msg.pose.pose.orientation.w = half_yaw.cos();

        // ------------------------------------------------
        // World velocity -> body velocity
        // ------------------------------------------------
        let cos_yaw = state.yaw.cos();
        let sin_yaw = state.yaw.sin();

        let vx_body = cos_yaw * state.vx + sin_yaw * state.vy;
        let vy_body = -sin_yaw * state.vx + cos_yaw * state.vy;

        // ------------------------------------------------
        // Twist
        // ------------------------------------------------
        msg.twist.twist.linear.x = vx_body;
        msg.twist.twist.linear.y = vy_body;
        msg.twist.twist.linear.z = 0.0;

        msg.twist.twist.angular.x = 0.0;
        msg.twist.twist.angular.y = 0.0;
        msg.twist.twist.angular.z = state.wz;

        self.publisher.publish(msg)?;

        Ok(())
    }
}

impl ReactionWheelStatePublisher {
    pub fn new(publisher: rclrs::Publisher<ReactionWheelState>) -> Self {
        Self { publisher }
    }

    pub fn publish(
        &self,
        time: f64,
        states: &[ReactionWheelStateData],
    ) -> Result<(), rclrs::RclrsError> {
        let mut msg = ReactionWheelState::default();

        let (sec, nanosec) = simulation_time_to_ros_time(time);

        msg.stamp.sec = sec;
        msg.stamp.nanosec = nanosec;

        for state in states {
            msg.names.push(state.name.clone());
            msg.angular_velocities.push(state.angular_velocity);
            msg.commanded_torques.push(state.commanded_torque);
            msg.applied_torques.push(state.applied_torque);
        }

        self.publisher.publish(msg)?;

        Ok(())
    }
}

fn simulation_time_to_ros_time(time_s: f64) -> (i32, u32) {
    let total_ns = (time_s * 1.0e9).round() as i64;

    let sec = total_ns.div_euclid(1_000_000_000);

    let nanosec = total_ns.rem_euclid(1_000_000_000);

    (sec as i32, nanosec as u32)
}
