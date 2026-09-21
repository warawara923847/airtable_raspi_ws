use airtable_interfaces::msg::ReactionWheelState;
use nav_msgs::msg::Odometry;
use rclrs::{IntoPrimitiveOptions, Node, Publisher};
use sensor_msgs::msg::Imu;
use std::collections::HashMap;

use crate::actuators::ReactionWheelStateData;
use crate::ros::conversions::sim_time_to_ros_time;
use crate::sensors::ImuState;
use crate::state::PlanarState;

pub struct VehicleRosHandles {
    pub(crate) gt_publisher: GroundTruthPublisher,
    pub(crate) rw_state_publisher: ReactionWheelStatePublisher,
    pub(crate) imu_publishers: HashMap<String, ImuPublisher>,

    pub(crate) table_frame: String,
}

pub struct GroundTruthPublisher {
    publisher: Publisher<Odometry>,

    frame_id: String,
    child_frame_id: String,
}

pub struct ReactionWheelStatePublisher {
    publisher: Publisher<ReactionWheelState>,
}

pub struct ImuPublisher {
    publisher: Publisher<Imu>,
    frame_id: String,
}

impl VehicleRosHandles {
    pub fn new(
        node: &Node,
        vehicle_name: &str,
        table_frame: String,
        imu_names: Vec<String>,
    ) -> Result<Self, rclrs::RclrsError> {
        // GT publisher
        let gt_topic = format!("/{vehicle_name}/sim/ground_truth");
        let gt_ros_publisher = node.create_publisher::<Odometry>(gt_topic.as_str())?;

        let gt_publisher = GroundTruthPublisher::new(
            gt_ros_publisher,
            table_frame.clone(),
            format!("{vehicle_name}/base_link"),
        );

        // RW state publisher
        let rw_state_topic = format!("/{vehicle_name}/actuators/reaction_wheels/state");
        let rw_ros_publisher =
            node.create_publisher::<ReactionWheelState>(rw_state_topic.as_str())?;
        let rw_state_publisher = ReactionWheelStatePublisher::new(rw_ros_publisher);

        // IMU publishers
        let mut imu_publishers = HashMap::new();

        for imu_name in imu_names {
            let topic = format!("/{vehicle_name}/{imu_name}");

            let ros_publisher = node.create_publisher::<Imu>(topic.as_str().sensor_data_qos())?;

            let imu_publisher =
                ImuPublisher::new(ros_publisher, format!("{vehicle_name}/{imu_name}"));

            imu_publishers.insert(imu_name, imu_publisher);
        }

        Ok(Self {
            gt_publisher,
            rw_state_publisher,
            imu_publishers,
            table_frame,
        })
    }
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

impl ImuPublisher {
    pub fn new(publisher: rclrs::Publisher<Imu>, frame_id: impl Into<String>) -> Self {
        Self {
            publisher: publisher,
            frame_id: frame_id.into(),
        }
    }

    pub fn publish(&self, time: f64, state: &ImuState) -> Result<(), rclrs::RclrsError> {
        let mut msg = Imu::default();

        msg.header.stamp = sim_time_to_ros_time(time);
        msg.header.frame_id = self.frame_id.clone();

        msg.orientation.x = state.orientation_wxyz[1];
        msg.orientation.y = state.orientation_wxyz[2];
        msg.orientation.z = state.orientation_wxyz[3];
        msg.orientation.w = state.orientation_wxyz[0];

        msg.angular_velocity.x = state.angular_velocity[0];
        msg.angular_velocity.y = state.angular_velocity[1];
        msg.angular_velocity.z = state.angular_velocity[2];

        msg.linear_acceleration.x = state.linear_acceleration[0];
        msg.linear_acceleration.y = state.linear_acceleration[1];
        msg.linear_acceleration.z = state.linear_acceleration[2];

        self.publisher.publish(msg)
    }
}

fn simulation_time_to_ros_time(time_s: f64) -> (i32, u32) {
    let total_ns = (time_s * 1.0e9).round() as i64;

    let sec = total_ns.div_euclid(1_000_000_000);

    let nanosec = total_ns.rem_euclid(1_000_000_000);

    (sec as i32, nanosec as u32)
}
