use geometry_msgs::msg::TransformStamped;
use rclrs::Publisher;
use tf2_msgs::msg::TFMessage;

use crate::state::{PlanarState, TablePose};

pub struct TfPublishers {
    dynamic_tf: Publisher<TFMessage>,
    static_tf: Publisher<TFMessage>,
}

impl TfPublishers {
    pub fn new(dynamic_tf: Publisher<TFMessage>, static_tf: Publisher<TFMessage>) -> Self {
        Self {
            dynamic_tf,
            static_tf,
        }
    }

    pub fn publish_static_tables(&self, tables: &[TablePose]) -> Result<(), rclrs::RclrsError> {
        let transforms = tables.iter().map(table_pose_to_transform).collect();

        let msg = TFMessage { transforms };

        self.static_tf.publish(msg)?;

        Ok(())
    }

    pub fn publish_vehicle(
        &self,
        table_name: &str,
        vehicle_name: &str,
        state: &PlanarState,
    ) -> Result<(), rclrs::RclrsError> {
        let mut tf = TransformStamped::default();

        let (sec, nanosec) = simulation_time_to_ros_time(state.time);

        tf.header.stamp.sec = sec;
        tf.header.stamp.nanosec = nanosec;

        tf.header.frame_id = table_name.to_string();

        tf.child_frame_id = format!("{vehicle_name}/base_link");

        tf.transform.translation.x = state.x;
        tf.transform.translation.y = state.y;
        tf.transform.translation.z = 0.0;

        let half_yaw = state.yaw * 0.5;

        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = half_yaw.sin();
        tf.transform.rotation.w = half_yaw.cos();

        self.dynamic_tf.publish(TFMessage {
            transforms: vec![tf],
        })?;

        Ok(())
    }
}

fn table_pose_to_transform(table: &TablePose) -> TransformStamped {
    let mut tf = TransformStamped::default();

    tf.header.frame_id = "world".to_string();
    tf.child_frame_id = table.name.clone();

    tf.transform.translation.x = table.position[0];
    tf.transform.translation.y = table.position[1];
    tf.transform.translation.z = table.position[2];

    // MuJoCo: [w, x, y, z]
    // ROS: [x, y, z, w]
    tf.transform.rotation.w = table.orientation_wxyz[0];
    tf.transform.rotation.x = table.orientation_wxyz[1];
    tf.transform.rotation.y = table.orientation_wxyz[2];
    tf.transform.rotation.z = table.orientation_wxyz[3];

    tf
}

fn simulation_time_to_ros_time(time: f64) -> (i32, u32) {
    let sec = time.floor() as i32;
    let nanosec = ((time - sec as f64) * 1e9) as u32;

    (sec, nanosec)
}
