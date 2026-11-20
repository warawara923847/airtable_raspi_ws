//! Offscreen camera rendering on the main thread from an immutable physics snapshot.
//!
//! One renderer draws every configured fixed camera in turn, so all cameras must
//! share one resolution. Each camera publishes its own image/CameraInfo topics and
//! static optical frame.
use std::collections::HashSet;

use geometry_msgs::msg::TransformStamped;
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRenderer;
use rclrs::{IntoPrimitiveOptions, Node, Publisher};
use sensor_msgs::msg::{CameraInfo, Image};

use crate::ros::conversions::sim_time_to_ros_time;

/// Topic namespace and optical frame of the original single-camera setup.
pub const LEGACY_NAMESPACE: &str = "/sim/camera";
pub const OPTICAL_FRAME: &str = "sim_camera_infra1_optical_frame";

/// One MJCF camera and where its images are published.
#[derive(Clone, Debug, PartialEq)]
pub struct CameraSpec {
    /// MJCF camera name.
    pub name: String,
    /// Images go to `<namespace>/infra1/image_rect_raw` (+ `camera_info`).
    pub namespace: String,
    /// `header.frame_id` of the images and child of the static world TF.
    pub optical_frame: String,
}

impl CameraSpec {
    pub fn legacy(name: &str) -> Self {
        Self {
            name: name.into(),
            namespace: LEGACY_NAMESPACE.into(),
            optical_frame: OPTICAL_FRAME.into(),
        }
    }
}

#[derive(Clone, Default)]
pub struct CameraSnapshot {
    pub time: f64,
    pub qpos: Vec<f64>,
}

struct CameraView {
    id: usize,
    optical_frame: String,
    info: CameraInfo,
    image_publisher: Publisher<Image>,
    info_publisher: Publisher<CameraInfo>,
}

pub struct CameraSensor {
    // Drop GL resources before the model/data on which their scene was based.
    renderer: MjRenderer,
    data: MjData<Box<MjModel>>,
    views: Vec<CameraView>,
    fps: f64,
    last_frame: Option<u64>,
}

/// Validate camera specs against a model; returns (camera id, width, height) each.
pub fn resolve_cameras(
    model: &MjModel,
    specs: &[CameraSpec],
) -> Result<Vec<(usize, u32, u32)>, String> {
    if specs.is_empty() {
        return Err("at least one camera is required".into());
    }
    let mut names = HashSet::new();
    let mut namespaces = HashSet::new();
    let mut frames = HashSet::new();
    let mut resolved = Vec::new();
    for spec in specs {
        if spec.name.is_empty() || spec.namespace.is_empty() || spec.optical_frame.is_empty() {
            return Err(format!(
                "camera {spec:?}: name, namespace and frame must be nonempty"
            ));
        }
        if !spec.namespace.starts_with('/') || spec.namespace.ends_with('/') {
            return Err(format!(
                "camera {}: namespace {:?} must be absolute without a trailing '/'",
                spec.name, spec.namespace
            ));
        }
        if !names.insert(&spec.name)
            || !namespaces.insert(&spec.namespace)
            || !frames.insert(&spec.optical_frame)
        {
            return Err(format!(
                "camera {}: name, namespace and optical frame must be unique",
                spec.name
            ));
        }
        let id = model
            .camera(&spec.name)
            .ok_or_else(|| format!("camera {:?} not found in model", spec.name))?
            .id;
        let body_id = model.cam_bodyid()[id] as usize;
        if model.body_weldid()[body_id] != 0
            || model.cam_mode()[id] != MjtCamLight::mjCAMLIGHT_FIXED
        {
            return Err(format!("camera {} must be fixed to the world", spec.name));
        }
        let [width, height] = model.cam_resolution()[id];
        if width <= 0 || height <= 0 {
            return Err(format!(
                "camera {}: resolution must be specified in the model",
                spec.name
            ));
        }
        resolved.push((id, width as u32, height as u32));
    }
    let (_, width, height) = resolved[0];
    if resolved.iter().any(|&(_, w, h)| (w, h) != (width, height)) {
        return Err("all cameras must share one resolution (one renderer draws them)".into());
    }
    Ok(resolved)
}

impl CameraSensor {
    pub fn new(
        node: &Node,
        model_path: &str,
        specs: &[CameraSpec],
        fps: f64,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        if !fps.is_finite() || fps <= 0.0 || fps > 100.0 {
            return Err("camera_fps must be finite and in (0, 100]".into());
        }
        let model = MjModel::from_xml(model_path)?;
        let resolved = resolve_cameras(&model, specs)?;
        let (first, width, height) = resolved[0];
        let mut views = Vec::new();
        for (spec, &(id, _, _)) in specs.iter().zip(&resolved) {
            views.push(CameraView {
                id,
                optical_frame: spec.optical_frame.clone(),
                info: camera_info(width, height, model.cam_fovy()[id]),
                image_publisher: node.create_publisher::<Image>(
                    format!("{}/infra1/image_rect_raw", spec.namespace).keep_last(1),
                )?,
                info_publisher: node.create_publisher::<CameraInfo>(
                    format!("{}/infra1/camera_info", spec.namespace).keep_last(1),
                )?,
            });
        }
        let mut options = MjvOption::default();
        // Hide debug sites (IMU, thruster axes, etc.) from the sensor image.
        options.sitegroup.fill(0);
        let renderer = MjRenderer::builder()
            .width(width)
            .height(height)
            .camera(MjvCamera::new_fixed(first))
            .opts(options)
            .rgb(true)
            .depth(false)
            .build(&model)?;
        let mut data = MjData::new(Box::new(model));
        data.forward();
        Ok(Self {
            renderer,
            data,
            views,
            fps,
            last_frame: None,
        })
    }

    /// Static world -> optical frame transform of every camera.
    pub fn optical_transforms(&self) -> Vec<TransformStamped> {
        self.views
            .iter()
            .map(|view| optical_transform(&self.data, view.id, &view.optical_frame))
            .collect()
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
        let stamp = sim_time_to_ros_time(snapshot.time);
        for view in &self.views {
            self.renderer.set_camera(MjvCamera::new_fixed(view.id));
            self.renderer.sync_data(&mut self.data)?;
            self.renderer.render()?;
            let rgb = self
                .renderer
                .rgb_flat()
                .ok_or("RGB rendering is disabled")?;
            let mut image = Image::default();
            image.header.stamp = stamp.clone();
            image.header.frame_id = view.optical_frame.clone();
            image.width = view.info.width;
            image.height = view.info.height;
            image.encoding = "mono8".into();
            image.step = image.width;
            image.data = rgb
                .chunks_exact(3)
                .map(|pixel| {
                    ((77 * pixel[0] as u32 + 150 * pixel[1] as u32 + 29 * pixel[2] as u32) >> 8)
                        as u8
                })
                .collect();
            let mut info = view.info.clone();
            info.header = image.header.clone();
            view.info_publisher.publish(info)?;
            view.image_publisher.publish(image)?;
        }
        self.last_frame = Some(frame);
        Ok(())
    }
}

fn optical_transform(data: &MjData<Box<MjModel>>, id: usize, frame: &str) -> TransformStamped {
    let position = data.cam_xpos()[id];
    let matrix = data.cam_xmat()[id];
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
    tf.child_frame_id = frame.into();
    tf.transform.translation.x = position[0];
    tf.transform.translation.y = position[1];
    tf.transform.translation.z = position[2];
    tf.transform.rotation.w = quat[0];
    tf.transform.rotation.x = quat[1];
    tf.transform.rotation.y = quat[2];
    tf.transform.rotation.z = quat[3];
    tf
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
        let mass = model.body_mass()[robot];
        assert_eq!(mass, 1.0);
        let inertia = model.body_inertia()[robot];
        assert!((inertia[0] - mass / 12.0 * (0.2_f64.powi(2) + 0.02_f64.powi(2))).abs() < 1e-12);
        assert!((inertia[1] - inertia[0]).abs() < 1e-12);
        assert!((inertia[2] - mass / 6.0 * 0.2_f64.powi(2)).abs() < 1e-12);
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

    fn dual_specs() -> Vec<CameraSpec> {
        ["table_1", "table_2"]
            .iter()
            .map(|table| CameraSpec {
                name: format!("{table}/tracking_camera"),
                namespace: format!("/sim/{table}/camera"),
                optical_frame: format!("sim_{table}_camera_infra1_optical_frame"),
            })
            .collect()
    }

    #[test]
    fn dual_model_has_one_world_fixed_camera_per_table() {
        let model = MjModel::from_xml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/mujoco/airtable_dual.xml"
        ))
        .expect("Dual scene and all marker textures must load");
        let resolved = resolve_cameras(&model, &dual_specs()).unwrap();
        assert_eq!(resolved.len(), 2);
        let mut data = MjData::new(Box::new(model));
        data.forward();
        for ((id, width, height), table) in resolved.iter().zip(["table_1", "table_2"]) {
            assert_eq!((*width, *height), (848, 480));
            let table_id = data.model().body(table).unwrap().id;
            assert_eq!(data.model().cam_bodyid()[*id] as usize, table_id);
            let table_xy = data.xpos()[table_id];
            let camera = data.cam_xpos()[*id];
            assert!((camera[0] - table_xy[0] - 0.5).abs() < 1e-12);
            assert!((camera[1] - table_xy[1] - 1.0).abs() < 1e-12);
        }
        // Table origins are 5 m apart along world +X (the tables' short side).
        let t2 = data.model().body("table_2").unwrap().id;
        assert_eq!(data.xpos()[t2], [5.0, 0.0, 0.0]);
        for name in [
            "table_1/fixed_marker_10",
            "table_2/fixed_marker_20",
            "robot_1/moving_marker_50",
            "robot_2/moving_marker_51",
        ] {
            let id = data.model().geom(name).unwrap().id;
            assert_eq!(data.model().geom_contype()[id], 0);
            assert_eq!(data.model().geom_conaffinity()[id], 0);
        }
    }

    #[test]
    fn camera_specs_are_validated() {
        let model = MjModel::from_xml(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/mujoco/airtable_dual.xml"
        ))
        .unwrap();
        assert!(resolve_cameras(&model, &[]).is_err());
        let mut duplicate = dual_specs();
        duplicate[1].namespace = duplicate[0].namespace.clone();
        assert!(resolve_cameras(&model, &duplicate).is_err());
        let mut relative = dual_specs();
        relative[0].namespace = "sim/table_1/camera".into();
        assert!(resolve_cameras(&model, &relative).is_err());
        let mut missing = dual_specs();
        missing[0].name = "tracking_camera".into();
        assert!(resolve_cameras(&model, &missing).is_err());
    }
}
