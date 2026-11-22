use geometry_msgs::msg::TransformStamped;
use rclrs::Publisher;
use tf2_msgs::msg::TFMessage;

use crate::ros::conversions::SimClock;
use crate::state::{PlanarState, TablePose};

pub struct TfPublishers {
    dynamic_tf: Publisher<TFMessage>,
    static_tf: Publisher<TFMessage>,
    clock: SimClock,
}

impl TfPublishers {
    pub fn new(
        dynamic_tf: Publisher<TFMessage>,
        static_tf: Publisher<TFMessage>,
        clock: SimClock,
    ) -> Self {
        Self {
            dynamic_tf,
            static_tf,
            clock,
        }
    }

    pub fn publish_static_frames(
        &self,
        tables: &[TablePose],
        cameras: Vec<TransformStamped>,
    ) -> Result<(), rclrs::RclrsError> {
        let mut transforms: Vec<_> = tables.iter().map(table_pose_to_transform).collect();
        transforms.extend(cameras);

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

        tf.header.stamp = self.clock.to_ros(state.time);

        tf.header.frame_id = table_name.to_string();

        tf.child_frame_id = format!("{vehicle_name}/base_link");

        tf.transform.translation.x = state.x;
        tf.transform.translation.y = state.y;
        tf.transform.translation.z = state.z;

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
