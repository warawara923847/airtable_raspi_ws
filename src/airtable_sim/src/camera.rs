//! Offscreen camera rendering on the main thread from an immutable physics snapshot.
use geometry_msgs::msg::TransformStamped;
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;
use rclrs::{IntoPrimitiveOptions, Node, Publisher};
use sensor_msgs::msg::{CameraInfo, Image};

use crate::ros::conversions::sim_time_to_ros_time;

pub const OPTICAL_FRAME: &str = "sim_camera_infra1_optical_frame";

#[derive(Clone, Default)]
pub struct CameraSnapshot {
    pub time: f64,
    pub qpos: Vec<f64>,
}

pub struct CameraSensor {
    // Drop GL resources before the model/data on which their scene was based.
    renderer: MjRenderer,
    data: MjData<Box<MjModel>>,
    info: CameraInfo,
    image_publisher: Publisher<Image>,
    info_publisher: Publisher<CameraInfo>,
    fps: f64,
    last_frame: Option<u64>,
}

impl CameraSensor {
    pub fn new(
        node: &Node,
        model_path: &str,
        camera_name: &str,
        fps: f64,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        if !fps.is_finite() || fps <= 0.0 || fps > 100.0 {
            return Err("camera_fps must be finite and in (0, 100]".into());
        }
        let model = MjModel::from_xml(model_path)?;
        let id = model
            .camera(camera_name)
            .ok_or("Camera not found in model")?
            .id;
        let body_id = model.cam_bodyid()[id] as usize;
        if model.body_weldid()[body_id] != 0
            || model.cam_mode()[id] != MjtCamLight::mjCAMLIGHT_FIXED
        {
            return Err("Tracking camera must be fixed to the world".into());
        }
        let [width, height] = model.cam_resolution()[id];
        if width <= 0 || height <= 0 {
            return Err("Camera resolution must be specified in the model".into());
        }
        let info = camera_info(width as u32, height as u32, model.cam_fovy()[id]);
        let mut options = MjvOption::default();
        // Hide debug sites (IMU, thruster axes, etc.) from the sensor image.
        options.sitegroup.fill(0);
        let renderer = MjRenderer::builder()
            .width(width as u32)
            .height(height as u32)
            .camera(MjvCamera::new_fixed(id))
            .opts(options)
            .rgb(true)
            .depth(false)
            .build(&model)?;
        let mut data = MjData::new(Box::new(model));
        data.forward();
        Ok(Self {
            renderer,
            data,
            info,
            image_publisher: node
                .create_publisher::<Image>("/sim/camera/infra1/image_rect_raw".keep_last(1))?,
            info_publisher: node
                .create_publisher::<CameraInfo>("/sim/camera/infra1/camera_info".keep_last(1))?,
            fps,
            last_frame: None,
        })
    }

    pub fn optical_transform(&self, camera_name: &str) -> TransformStamped {
        let id = self.data.model().camera(camera_name).unwrap().id;
        let position = self.data.cam_xpos()[id];
        let matrix = self.data.cam_xmat()[id];
        // MuJoCo camera: +X right, +Y up, -Z forward. ROS optical: +Y down, +Z forward.
        let optical = [
            matrix[0], -matrix[1], -matrix[2], matrix[3], -matrix[4], -matrix[5], matrix[6],
            -matrix[7], -matrix[8],
        ];
        let mut quat = [0.0; 4];
        // SAFETY: MuJoCo reads a 9-element rotation and writes a 4-element quaternion.
        unsafe { mujoco_rs::mujoco_c::mju_mat2Quat(&mut quat, &optical) };
        let mut tf = TransformStamped::default();
        tf.header.frame_id = "world".into();
        tf.child_frame_id = OPTICAL_FRAME.into();
        tf.transform.translation.x = position[0];
        tf.transform.translation.y = position[1];
        tf.transform.translation.z = position[2];
        tf.transform.rotation.w = quat[0];
        tf.transform.rotation.x = quat[1];
        tf.transform.rotation.y = quat[2];
        tf.transform.rotation.z = quat[3];
        tf
    }

    pub fn publish(&mut self, snapshot: &CameraSnapshot) -> Result<(), Box<dyn std::error::Error>> {
        if snapshot.qpos.is_empty() {
            return Ok(());
        }
        let frame = (snapshot.time * self.fps).floor() as u64;
        if self.last_frame == Some(frame) {
            return Ok(());
        }
        self.data.qpos_mut().copy_from_slice(&snapshot.qpos);
        self.data.set_time(snapshot.time);
        self.data.forward();
        self.renderer.sync_data(&mut self.data)?;
        self.renderer.render()?;
        let rgb = self
            .renderer
            .rgb_flat()
            .ok_or("RGB rendering is disabled")?;
        let mut image = Image::default();
        image.header.stamp = sim_time_to_ros_time(snapshot.time);
        image.header.frame_id = OPTICAL_FRAME.into();
        image.width = self.info.width;
        image.height = self.info.height;
        image.encoding = "mono8".into();
        image.step = image.width;
        image.data = rgb
            .chunks_exact(3)
            .map(|pixel| {
                ((77 * pixel[0] as u32 + 150 * pixel[1] as u32 + 29 * pixel[2] as u32) >> 8) as u8
            })
            .collect();
        let mut info = self.info.clone();
        info.header = image.header.clone();
        self.info_publisher.publish(info)?;
        self.image_publisher.publish(image)?;
        self.last_frame = Some(frame);
        Ok(())
    }
}

fn camera_info(width: u32, height: u32, fovy_degrees: f64) -> CameraInfo {
    let f = height as f64 / (2.0 * (0.5 * fovy_degrees.to_radians()).tan());
    let cx = (width as f64 - 1.0) / 2.0;
    let cy = (height as f64 - 1.0) / 2.0;
    let mut info = CameraInfo::default();
    info.width = width;
    info.height = height;
    info.distortion_model = "plumb_bob".into();
    info.d = vec![0.0; 5];
    info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0];
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0];
    info
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intrinsics_match_the_render_frustum() {
        let info = camera_info(1280, 960, 55.0);
        assert!((2.0 * (480.0 / info.k[4]).atan().to_degrees() - 55.0).abs() < 1e-10);
        assert_eq!(info.k[2], 639.5);
        assert_eq!(info.k[5], 479.5);
        assert_eq!(info.k[0], info.p[0]);
        assert_eq!(info.k[4], info.p[5]);
        assert_eq!(info.d, vec![0.0; 5]);
    }

    #[test]
    fn marker_assets_preserve_vehicle_dynamics() {
        let model = MjModel::from_xml(concat!(env!("CARGO_MANIFEST_DIR"), "/mujoco/airtable.xml"))
            .expect("Scene and all marker textures must load");
        let robot = model.body("robot_1").unwrap().id;
        assert_eq!(model.ffi().nq, 3);
        assert_eq!(model.body_mass()[robot], 10.0);
        let inertia = model.body_inertia()[robot];
        assert!((inertia[0] - 10.0 / 12.0 * (0.2_f64.powi(2) + 0.02_f64.powi(2))).abs() < 1e-12);
        assert!((inertia[1] - inertia[0]).abs() < 1e-12);
        assert!((inertia[2] - 10.0 / 6.0 * 0.2_f64.powi(2)).abs() < 1e-12);
        for name in [
            "fixed_marker_10",
            "fixed_marker_11",
            "fixed_marker_12",
            "fixed_marker_13",
            "moving_marker_50",
        ] {
            let id = model.geom(name).unwrap().id;
            assert_eq!(model.geom_contype()[id], 0);
            assert_eq!(model.geom_conaffinity()[id], 0);
        }
    }
}
