use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;
use mujoco_rs::viewer::ViewerSharedState;

use crate::actuators::{ReactionWheelStateData, SimReactionWheel, SimThruster};
use crate::disturbance::{AppliedDisturbance, Disturbance, WrenchFrame};
use crate::model::{SimTable, SimVehicle};
use crate::sensors::{ImuState, SimImu};
use crate::state::{PlanarState, TablePose};

/// One simulated vehicle selected at startup.
///
/// The MuJoCo body is named after the vehicle (namespace). Actuators and
/// sensors are named `<element_prefix><local name>`, e.g. `robot_2/thruster_1_actuator`.
/// The original single-robot model uses an empty prefix.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleConfig {
    pub name: String,
    pub element_prefix: String,
}

impl VehicleConfig {
    pub fn new(name: impl Into<String>, element_prefix: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            element_prefix: element_prefix.into(),
        }
    }

    /// The original model: robot_1 with unprefixed element names.
    pub fn legacy() -> Self {
        Self::new("robot_1", "")
    }
}

/// Local (unprefixed) names shared by every vehicle; commands use these names.
const THRUSTERS: &[&str] = &["thruster_1", "thruster_2", "thruster_3", "thruster_4"];

struct ReactionWheelDefinition {
    name: &'static str,
    actuator_name: &'static str,

    inertia: f64,
    max_torque: f64,
    max_angular_velocity: f64,
}

struct ImuDefinition {
    name: &'static str,
    accel_sensor_name: &'static str,
    gyro_sensor_name: &'static str,
    orientation_sensor_name: &'static str,
}

const REACTION_WHEELS: &[ReactionWheelDefinition] = &[ReactionWheelDefinition {
    name: "rw_1",
    actuator_name: "reaction_wheel_1_actuator",

    inertia: 1.0e-3,
    max_torque: 0.02,
    max_angular_velocity: 500.0,
}];

const IMUS: &[ImuDefinition] = &[ImuDefinition {
    name: "imu_1",
    accel_sensor_name: "imu_1_accel",
    gyro_sensor_name: "imu_1_gyro",
    orientation_sensor_name: "imu_1_orientation",
}];

pub struct Simulation {
    data: MjData<Box<MjModel>>,
    vehicles: HashMap<String, SimVehicle>,
    tables: HashMap<String, SimTable>,
    disturbances: HashMap<String, Disturbance>,
    disturbance_generalized: Vec<f64>,
    viewer_state: Option<Arc<Mutex<ViewerSharedState>>>,
}

impl Simulation {
    /// Load the model with the legacy robot_1 vehicle (original single-robot MJCF).
    #[cfg(test)]
    pub fn new(model_path: &str) -> Result<Self, String> {
        Self::with_vehicles(model_path, &[VehicleConfig::legacy()])
    }

    pub fn with_vehicles(model_path: &str, vehicles: &[VehicleConfig]) -> Result<Self, String> {
        let model =
            MjModel::from_xml(model_path).map_err(|e| format!("Failed to load model: {}", e))?;

        let data = MjData::new(Box::new(model));

        Self::from_data(data, vehicles, None)
    }

    #[cfg(test)]
    fn from_xml_string(xml: &str, vehicles: &[VehicleConfig]) -> Result<Self, String> {
        let model =
            MjModel::from_xml_string(xml).map_err(|e| format!("Failed to load model: {e}"))?;
        Self::from_data(MjData::new(Box::new(model)), vehicles, None)
    }

    pub fn new_with_viewer(
        model_path: &str,
        vehicles: &[VehicleConfig],
    ) -> Result<(Self, MjViewer), String> {
        // MuJoCo model
        let model =
            MjModel::from_xml(model_path).map_err(|e| format!("failed to load model: {e}"))?;

        // Simulation data
        let data = MjData::new(Box::new(model));

        // Viewer
        let viewer = MjViewer::builder()
            .window_name("Airtable Simulation")
            .vsync(true)
            .build_passive(data.model())
            .map_err(|e| format!("failed to create viewer: {e}"))?;

        let viewer_state = viewer.state().clone();

        let simulation = Self::from_data(data, vehicles, Some(viewer_state))?;

        Ok((simulation, viewer))
    }

    fn from_data(
        mut data: MjData<Box<MjModel>>,
        vehicle_configs: &[VehicleConfig],
        viewer_state: Option<Arc<Mutex<ViewerSharedState>>>,
    ) -> Result<Self, String> {
        // qposからxpos/xquatを初期計算
        data.forward();

        let vehicles = Self::build_vehicles(&data, vehicle_configs)?;

        let tables = Self::build_tables(&data, &vehicles)?;
        let disturbances = vehicles
            .keys()
            .map(|name| (name.clone(), Disturbance::default()))
            .collect();
        let disturbance_generalized = vec![0.0; data.model().ffi().nv as usize];

        Ok(Self {
            data,
            vehicles,
            tables,
            disturbances,
            disturbance_generalized,
            viewer_state,
        })
    }

    fn build_tables(
        data: &MjData<Box<MjModel>>,
        vehicles: &HashMap<String, SimVehicle>,
    ) -> Result<HashMap<String, SimTable>, String> {
        let model = data.model();

        let mut tables = HashMap::new();

        for vehicle in vehicles.values() {
            let parent_id = model.body_parentid()[vehicle.body_id];

            if parent_id <= 0 {
                return Err(format!(
                    "vehicle '{}' is not attached to a table body",
                    vehicle.name
                ));
            }

            let table_body_id = parent_id as usize;

            let table_name = model
                .id_to_name(MjtObj::mjOBJ_BODY, table_body_id)
                .ok_or_else(|| format!("MuJoCo body name not found for id {}", table_body_id))?;

            println!(
                "[sim] vehicle {} -> table {} (body id={})",
                vehicle.name, table_name, table_body_id,
            );

            tables
                .entry(table_name.to_string())
                .or_insert_with(|| SimTable::new(table_name, table_body_id));
        }

        Ok(tables)
    }

    fn build_vehicles(
        data: &MjData<Box<MjModel>>,
        configs: &[VehicleConfig],
    ) -> Result<HashMap<String, SimVehicle>, String> {
        if configs.is_empty() {
            return Err("at least one vehicle must be selected".into());
        }
        let mut vehicles = HashMap::new();
        let mut prefixes = HashMap::new();

        for config in configs {
            let vehicle_name = config.name.as_str();
            let prefix = config.element_prefix.as_str();
            if vehicle_name.is_empty() {
                return Err("vehicle name must not be empty".into());
            }
            if vehicles.contains_key(vehicle_name) {
                return Err(format!("duplicate vehicle: {vehicle_name}"));
            }
            if let Some(other) = prefixes.insert(prefix.to_string(), vehicle_name.to_string()) {
                return Err(format!(
                    "vehicles {other} and {vehicle_name} share element prefix '{prefix}'"
                ));
            }
            // ------------------------------------------------
            // MuJoCo body
            // ------------------------------------------------
            let body_info = data.body(vehicle_name).ok_or_else(|| {
                format!("MuJoCo body not found for vehicle {vehicle_name}; check model_path")
            })?;

            let body_id = body_info.id;

            println!(
                "[sim] vehicle: {} -> body {} (id={}, element prefix='{}')",
                vehicle_name, vehicle_name, body_id, prefix
            );

            // ------------------------------------------------
            // thrusters
            // ------------------------------------------------
            let mut thrusters = HashMap::new();

            for thruster in THRUSTERS {
                let actuator_name = format!("{prefix}{thruster}_actuator");
                let actuator_info = data.actuator(&actuator_name).ok_or_else(|| {
                    format!("MuJoCo actuator not found for {vehicle_name}: {actuator_name}")
                })?;

                let actuator_id = actuator_info.id;
                let model = data.model();
                let control_id = model.actuator_ctrladr()[actuator_id] as usize;
                // The MJCF control range is the physical ON thrust of a binary valve.
                let range = model.actuator_ctrlrange()[actuator_id];
                if !model.actuator_ctrllimited()[actuator_id]
                    || range[0] != 0.0
                    || !(range[1].is_finite() && range[1] > 0.0)
                {
                    return Err(format!(
                        "{actuator_name}: thruster needs ctrllimited ctrlrange=\"0 <thrust_n>\""
                    ));
                }
                let thrust_n = range[1];

                println!(
                    "[sim] {}/{} -> {} (actuator id={}, control_id={}, thrust={} N)",
                    vehicle_name, thruster, actuator_name, actuator_id, control_id, thrust_n
                );

                thrusters.insert(
                    thruster.to_string(),
                    SimThruster::new(*thruster, control_id, thrust_n),
                );
            }

            // ------------------------------------------------
            // reaction wheels
            // ------------------------------------------------
            let mut reaction_wheels = HashMap::new();

            for rw_def in REACTION_WHEELS {
                let actuator_name = format!("{prefix}{}", rw_def.actuator_name);
                let actuator_info = data.actuator(&actuator_name).ok_or_else(|| {
                    format!("MuJoCo actuator not found for {vehicle_name}: {actuator_name}")
                })?;

                let actuator_id = actuator_info.id;

                let control_id = data.model().actuator_ctrladr()[actuator_id] as usize;

                println!(
                    "[sim] {}/{} -> {} (actuator id={}, control id={})",
                    vehicle_name, rw_def.name, actuator_name, actuator_id, control_id
                );

                reaction_wheels.insert(
                    rw_def.name.to_string(),
                    SimReactionWheel::new(
                        rw_def.name,
                        control_id,
                        rw_def.inertia,
                        rw_def.max_torque,
                        rw_def.max_angular_velocity,
                    ),
                );
            }

            // -------------------------------------------------
            // IMUs
            // -------------------------------------------------
            let mut imus = HashMap::new();

            for imu_def in IMUS {
                let sensor = |local: &str| {
                    let name = format!("{prefix}{local}");
                    data.sensor(&name).map(|info| info.id).ok_or_else(|| {
                        format!("MuJoCo sensor not found for {vehicle_name}: {name}")
                    })
                };
                let accel = sensor(imu_def.accel_sensor_name)?;
                let gyro = sensor(imu_def.gyro_sensor_name)?;
                let orientation = sensor(imu_def.orientation_sensor_name)?;

                imus.insert(
                    imu_def.name.to_string(),
                    SimImu::new(imu_def.name, accel, gyro, orientation),
                );
            }
            // ------------------------------------------------
            // Vehicle
            // ------------------------------------------------
            vehicles.insert(
                vehicle_name.to_string(),
                SimVehicle::new(vehicle_name, body_id, thrusters, reaction_wheels, imus),
            );
        }

        Ok(vehicles)
    }

    pub fn vehicle_names(&self) -> Vec<String> {
        let mut names: Vec<String> = self.vehicles.keys().cloned().collect();

        names.sort();

        names
    }

    pub fn vehicle_state(&self, vehicle_name: &str) -> Result<PlanarState, String> {
        let vehicle = self
            .vehicles
            .get(vehicle_name)
            .ok_or_else(|| format!("unknown vehicle: {vehicle_name}"))?;

        let table = self.vehicle_table(vehicle_name)?;

        let body_id = vehicle.body_id;

        // ------------------------------------------------
        // World pose
        // ------------------------------------------------
        let robot_pos = self.data.xpos()[vehicle.body_id];
        let table_pos = self.data.xpos()[table.body_id()];

        let robot_quat = self.data.xquat()[vehicle.body_id];
        let table_quat = self.data.xquat()[table.body_id()];

        let robot_yaw = yaw_from_quat(robot_quat);
        let table_yaw = yaw_from_quat(table_quat);

        // ------------------------------------------------
        // Position: world -> table
        // ------------------------------------------------
        let dx = robot_pos[0] - table_pos[0];
        let dy = robot_pos[1] - table_pos[1];

        let c = table_yaw.cos();
        let s = table_yaw.sin();

        let x = c * dx + s * dy;
        let y = -s * dx + c * dy;

        let yaw = normalize_angle(robot_yaw - table_yaw);

        // ------------------------------------------------
        // Velocity
        // ------------------------------------------------
        let velocity = self
            .data
            .object_velocity(MjtObj::mjOBJ_BODY, body_id, false);

        let vx_world = velocity[3];
        let vy_world = velocity[4];

        let vx = c * vx_world + s * vy_world;
        let vy = -s * vx_world + c * vy_world;

        let wz = velocity[2];

        Ok(PlanarState {
            time: self.data.time(),
            x,
            y,
            z: robot_pos[2] - table_pos[2],
            yaw,
            vx,
            vy,
            wz,
        })
    }

    pub fn vehicle_table(&self, vehicle_name: &str) -> Result<&SimTable, String> {
        let vehicle = self
            .vehicles
            .get(vehicle_name)
            .ok_or_else(|| format!("unknown vehilcle: {}", vehicle_name))?;

        let parent_id = self.data.model().body_parentid()[vehicle.body_id];

        if parent_id <= 0 {
            return Err(format!("vehicle '{}' has no table parent", vehicle_name));
        }

        let table_body_id = parent_id as usize;

        let table_name = self
            .data
            .model()
            .id_to_name(MjtObj::mjOBJ_BODY, table_body_id)
            .ok_or_else(|| format!("body name not found for id {}", table_body_id))?;

        self.tables
            .get(table_name)
            .ok_or_else(|| format!("table '{}' not registered", table_name))
    }

    pub fn table_poses(&self) -> Vec<TablePose> {
        self.tables
            .values()
            .map(|table| {
                let position = self.data.xpos()[table.body_id()];

                let orientation = self.data.xquat()[table.body_id()];

                TablePose {
                    name: table.name().to_string(),
                    position,
                    orientation_wxyz: orientation,
                }
            })
            .collect()
    }

    pub fn sync_viewer(&mut self) {
        let Some(viewer_state) = &self.viewer_state else {
            return;
        };

        let Ok(mut state) = viewer_state.lock() else {
            eprintln!("viewer state mutex poisoned");
            return;
        };

        state.sync_data(&mut self.data);
    }

    pub fn time(&self) -> f64 {
        self.data.time()
    }

    /// Set a planar vehicle pose in its parent table before physics starts.
    /// Marker sites/geoms and IMU poses follow the same forward kinematics.
    pub fn set_initial_pose(
        &mut self,
        vehicle_name: &str,
        x: f64,
        y: f64,
        yaw: f64,
    ) -> Result<(), String> {
        if self.data.time() != 0.0 || ![x, y, yaw].iter().all(|value| value.is_finite()) {
            return Err("Initial pose must be finite and set before stepping".into());
        }
        let vehicle = self
            .vehicles
            .get(vehicle_name)
            .ok_or_else(|| format!("unknown vehicle: {vehicle_name}"))?;
        let model = self.data.model();
        let body = vehicle.body_id;
        if model.body_jntnum()[body] != 3 || model.body_quat()[body] != [1.0, 0.0, 0.0, 0.0] {
            return Err("Initial pose requires an unrotated x/y/yaw planar body".into());
        }
        let first = model.body_jntadr()[body] as usize;
        let expected = [
            (MjtJoint::mjJNT_SLIDE, [1.0, 0.0, 0.0]),
            (MjtJoint::mjJNT_SLIDE, [0.0, 1.0, 0.0]),
            (MjtJoint::mjJNT_HINGE, [0.0, 0.0, 1.0]),
        ];
        let relative = [
            x - model.body_pos()[body][0],
            y - model.body_pos()[body][1],
            yaw,
        ];
        let mut coordinates = Vec::new();
        for (offset, (kind, axis)) in expected.iter().enumerate() {
            let joint = first + offset;
            if model.jnt_type()[joint] != *kind || model.jnt_axis()[joint] != *axis {
                return Err("Initial pose requires x slide, y slide, yaw hinge joints".into());
            }
            let qpos = model.jnt_qposadr()[joint] as usize;
            coordinates.push((
                qpos,
                model.qpos0()[qpos] + relative[offset],
                model.jnt_dofadr()[joint] as usize,
            ));
        }
        for (qpos, value, dof) in coordinates {
            self.data.qpos_mut()[qpos] = value;
            self.data.qvel_mut()[dof] = 0.0;
        }
        self.data.forward();
        Ok(())
    }

    pub fn fire_thruster(
        &mut self,
        vehicle_name: &str,
        thruster_name: &str,
        duration_s: f64,
    ) -> Result<(), String> {
        let now = self.data.time();

        let vehicle = self
            .vehicles
            .get_mut(vehicle_name)
            .ok_or_else(|| format!("unknown thruster: {vehicle_name}"))?;

        vehicle.fire_thruster(thruster_name, now, duration_s)
    }

    // ------------------------------------------------
    // Reaction wheels
    // ------------------------------------------------
    pub fn set_reaction_wheel_torque(
        &mut self,
        vehicle_name: &str,
        wheel_name: &str,
        torque_nm: f64,
    ) -> Result<(), String> {
        let vehicle = self
            .vehicles
            .get_mut(vehicle_name)
            .ok_or_else(|| format!("unknown vehicle: {vehicle_name}"))?;

        vehicle.set_reaction_wheel_torque(wheel_name, torque_nm)
    }

    pub fn reaction_wheel_speed(
        &self,
        vehicle_name: &str,
        wheel_name: &str,
    ) -> Result<f64, String> {
        let vehicle = self
            .vehicles
            .get(vehicle_name)
            .ok_or_else(|| format!("unknown vehicle: {vehicle_name}"))?;

        let wheel = vehicle.reaction_wheel(wheel_name)?;

        Ok(wheel.angular_velocity())
    }

    pub fn reaction_wheel_states(
        &self,
        vehicle_name: &str,
    ) -> Result<Vec<ReactionWheelStateData>, String> {
        let vehicle = self
            .vehicles
            .get(vehicle_name)
            .ok_or_else(|| format!("unknown vehicle: {vehicle_name}"))?;

        let mut states: Vec<_> = vehicle
            .reaction_wheels
            .values()
            .map(|wheel| wheel.state())
            .collect();

        states.sort_by(|a, b| a.name.cmp(&b.name));

        Ok(states)
    }

    // ------------------------------------------------
    // IMUs
    // ------------------------------------------------
    pub fn imu_states(&self, vehicle_name: &str) -> Result<Vec<ImuState>, String> {
        let vehicle = self
            .vehicles
            .get(vehicle_name)
            .ok_or_else(|| format!("unknown vehicle: {vehicle_name}"))?;

        let mut states: Vec<ImuState> = vehicle
            .imus
            .values()
            .map(|imu| imu.read(&self.data))
            .collect();

        states.sort_by(|a, b| a.name.cmp(&b.name));

        Ok(states)
    }

    pub fn imu_names(&self, vehicle_name: &str) -> Result<Vec<String>, String> {
        let vehicle = self
            .vehicles
            .get(vehicle_name)
            .ok_or_else(|| format!("unknown vehicle: {vehicle_name}"))?;

        Ok(vehicle.imus.keys().cloned().collect())
    }

    pub fn step(&mut self) {
        let now = self.data.time();
        let dt = self.data.model().opt().timestep;

        {
            let ctrl = self.data.ctrl_mut();

            for vehicle in self.vehicles.values_mut() {
                vehicle.update_thrusters(now, ctrl);
                vehicle.update_reaction_wheels(ctrl, dt);
            }
        }

        self.disturbance_generalized.fill(0.0);
        for (name, vehicle) in &self.vehicles {
            let body = vehicle.body_id;
            let table = self.data.model().body_parentid()[body] as usize;
            let body_yaw = yaw_from_quat(self.data.xquat()[body]);
            let table_yaw = yaw_from_quat(self.data.xquat()[table]);
            let wrench = self
                .disturbances
                .get_mut(name)
                .unwrap()
                .advance(now, table_yaw, body_yaw);
            let point = self.data.xipos()[body];
            // The passive viewer clears xfrc_applied during synchronization.
            // Keep scripted loads in qfrc_applied so GUI perturbations remain
            // independent and additive rather than overwriting one another.
            self.data
                .apply_ft(
                    &[wrench[0], wrench[1], wrench[2]],
                    &[wrench[3], wrench[4], wrench[5]],
                    &point,
                    body,
                    &mut self.disturbance_generalized,
                )
                .expect("Vehicle IDs and generalized-force buffer match the loaded model");
        }
        self.data
            .qfrc_applied_mut()
            .copy_from_slice(&self.disturbance_generalized);
        self.data.step();
        // mj_step integrates qpos/time last; refresh derived poses and sensors at that time.
        self.data.forward();
    }

    pub fn qpos(&self) -> &[f64] {
        self.data.qpos()
    }

    pub fn set_disturbance(
        &mut self,
        vehicle: &str,
        frame_id: &str,
        wrench: [f64; 3],
        stamp: Option<f64>,
        timeout: f64,
    ) -> Result<(), String> {
        let table_frame = self.vehicle_table(vehicle)?.name().to_string();
        let frame = if frame_id == "world" {
            Some(WrenchFrame::World)
        } else if frame_id == table_frame {
            Some(WrenchFrame::Table)
        } else if frame_id == format!("{vehicle}/base_link") {
            Some(WrenchFrame::Body)
        } else {
            None
        };
        let now = self.data.time();
        let disturbance = self.disturbances.get_mut(vehicle).unwrap();
        let Some(frame) = frame else {
            disturbance.clear();
            return Err(format!("Unknown disturbance frame: {frame_id}"));
        };
        disturbance.set(now, stamp, frame, wrench, timeout)
    }

    pub fn clear_disturbance(&mut self, vehicle: &str) {
        if let Some(disturbance) = self.disturbances.get_mut(vehicle) {
            disturbance.clear();
        }
    }

    pub fn applied_disturbance(&self, vehicle: &str) -> Result<AppliedDisturbance, String> {
        self.disturbances
            .get(vehicle)
            .map(|value| value.applied)
            .ok_or_else(|| format!("unknown vehicle: {vehicle}"))
    }

    pub fn qvel(&self) -> &[f64] {
        self.data.qvel()
    }
}

fn yaw_from_quat(q: [f64; 4]) -> f64 {
    let w = q[0];
    let x = q[1];
    let y = q[2];
    let z = q[3];

    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

fn normalize_angle(angle: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;

    (angle + std::f64::consts::PI).rem_euclid(two_pi) - std::f64::consts::PI
}

#[cfg(test)]
mod initial_pose_tests {
    use super::*;

    fn simulation() -> Simulation {
        Simulation::new(concat!(env!("CARGO_MANIFEST_DIR"), "/mujoco/airtable.xml")).unwrap()
    }

    #[test]
    fn initial_pose_moves_vehicle_and_marker_before_first_step() {
        let mut sim = simulation();
        sim.set_initial_pose("robot_1", 0.7, 1.2, 0.4).unwrap();
        let pose = sim.vehicle_state("robot_1").unwrap();
        assert!((pose.x - 0.7).abs() < 1e-12);
        assert!((pose.y - 1.2).abs() < 1e-12);
        assert!((pose.yaw - 0.4).abs() < 1e-12);
        assert!((pose.z - 0.01).abs() < 1e-12);
        assert_eq!([pose.time, pose.vx, pose.vy, pose.wz], [0.0; 4]);
        let marker = sim.data.model().geom("moving_marker_50").unwrap().id;
        let marker_pos = sim.data.geom_xpos()[marker];
        assert!((marker_pos[0] - pose.x).abs() < 1e-12);
        assert!((marker_pos[1] - pose.y).abs() < 1e-12);
        // MuJoCo recenters mesh geoms at their mesh center of mass. Test the
        // compiled transform, including that offset, rather than the MJCF origin.
        let local = sim.data.model().geom_pos()[marker];
        assert!((marker_pos[2] - (pose.z + local[2])).abs() < 1e-12);
    }

    #[test]
    fn invalid_initial_pose_does_not_modify_state() {
        let mut sim = simulation();
        assert!(sim.set_initial_pose("robot_1", f64::NAN, 1.0, 0.0).is_err());
        let pose = sim.vehicle_state("robot_1").unwrap();
        assert_eq!([pose.x, pose.y, pose.yaw], [0.5, 1.0, 0.0]);
        sim.step();
        assert!(sim.set_initial_pose("robot_1", 0.7, 1.0, 0.0).is_err());
    }

    #[test]
    fn external_force_and_torque_produce_expected_impulse_and_then_expire() {
        let mut sim = simulation();
        let body = sim.vehicles["robot_1"].body_id;
        let mass = sim.data.model().body_mass()[body];
        let iz = sim.data.model().body_inertia()[body][2];
        sim.set_disturbance("robot_1", "table_1", [0.05, -0.02, 0.0006], None, 0.1)
            .unwrap();
        for _ in 0..100 {
            sim.step();
        }
        let state = sim.vehicle_state("robot_1").unwrap();
        assert!((state.vx - 0.05 * 0.1 / mass).abs() < 1e-10);
        assert!((state.vy + 0.02 * 0.1 / mass).abs() < 1e-10);
        assert!((state.wz - 0.0006 * 0.1 / iz).abs() < 1e-10);
        assert_eq!(sim.data.xfrc_applied()[body], [0.0; 6]);
        assert!(sim.data.qfrc_applied().iter().all(|value| *value == 0.0));
        assert_eq!(
            sim.applied_disturbance("robot_1").unwrap().body_wrench,
            [0.0; 3]
        );
    }

    #[test]
    fn body_force_at_ninety_degrees_pushes_along_table_y() {
        let mut sim = simulation();
        sim.set_initial_pose("robot_1", 0.5, 1.0, std::f64::consts::FRAC_PI_2)
            .unwrap();
        sim.set_disturbance("robot_1", "robot_1/base_link", [0.05, 0.0, 0.0], None, 0.1)
            .unwrap();
        for _ in 0..50 {
            sim.step();
        }
        let state = sim.vehicle_state("robot_1").unwrap();
        assert!(state.vx.abs() < 1e-10);
        assert!((state.vy - 0.005).abs() < 1e-10);
        assert!(state.wz.abs() < 1e-10);
    }

    #[test]
    fn body_force_follows_rotation_while_world_force_does_not() {
        for frame in ["robot_1/base_link", "world"] {
            let mut sim = simulation();
            sim.set_disturbance("robot_1", frame, [0.05, 0.0, 0.001], None, 1.0)
                .unwrap();
            for _ in 0..100 {
                sim.step();
            }
            // The fixture's x/y translation joints are aligned with world axes.
            let world = sim.data.qfrc_applied();
            if frame == "world" {
                assert_eq!(world[0], 0.05);
                assert_eq!(world[1], 0.0);
            } else {
                assert!(world[1] > 0.0001);
                assert!((world[0].hypot(world[1]) - 0.05).abs() < 1e-12);
                let applied = sim.applied_disturbance("robot_1").unwrap();
                assert!((applied.body_wrench[0] - 0.05).abs() < 1e-12);
                assert!(applied.body_wrench[1].abs() < 1e-12);
            }
        }
    }

    #[test]
    fn wrong_frame_cancels_the_previous_disturbance() {
        let mut sim = simulation();
        sim.set_disturbance("robot_1", "world", [0.05, 0.0, 0.0], None, 1.0)
            .unwrap();
        assert!(
            sim.set_disturbance("robot_1", "typo", [0.05, 0.0, 0.0], None, 1.0)
                .is_err()
        );
        sim.step();
        let state = sim.vehicle_state("robot_1").unwrap();
        assert_eq!([state.vx, state.vy, state.wz], [0.0; 3]);
    }

    #[test]
    fn scripted_force_adds_to_viewer_perturbation_without_overwriting_it() {
        let mut sim = simulation();
        let body = sim.vehicles["robot_1"].body_id;
        sim.data.xfrc_applied_mut()[body] = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0];
        sim.set_disturbance("robot_1", "world", [0.0, 0.02, 0.0], None, 1.0)
            .unwrap();
        sim.step();
        let state = sim.vehicle_state("robot_1").unwrap();
        let dt = sim.data.model().opt().timestep;
        assert!((state.vx - 0.01 * dt).abs() < 1e-12);
        assert!((state.vy - 0.02 * dt).abs() < 1e-12);
        assert_eq!(sim.data.xfrc_applied()[body][0], 0.01);
    }
}

#[cfg(test)]
mod multi_vehicle_tests {
    use super::*;

    const DUAL: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/mujoco/airtable_dual.xml");
    const ROBOT_2: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/mujoco/airtable_robot_2.xml");

    fn both() -> Vec<VehicleConfig> {
        vec![
            VehicleConfig::new("robot_1", "robot_1/"),
            VehicleConfig::new("robot_2", "robot_2/"),
        ]
    }

    fn state(sim: &Simulation, name: &str) -> [f64; 7] {
        let s = sim.vehicle_state(name).unwrap();
        [s.x, s.y, s.z, s.yaw, s.vx, s.vy, s.wz]
    }

    #[test]
    fn dual_model_registers_each_vehicle_on_its_own_table() {
        let sim = Simulation::with_vehicles(DUAL, &both()).unwrap();
        assert_eq!(sim.vehicle_names(), ["robot_1", "robot_2"]);
        assert_eq!(sim.vehicle_table("robot_1").unwrap().name(), "table_1");
        assert_eq!(sim.vehicle_table("robot_2").unwrap().name(), "table_2");
        let mut tables: Vec<_> = sim.table_poses().into_iter().map(|t| t.name).collect();
        tables.sort();
        assert_eq!(tables, ["table_1", "table_2"]);
        // Both start at the same table-local pose even though the tables differ in world.
        assert_eq!(state(&sim, "robot_1"), state(&sim, "robot_2"));
    }

    #[test]
    fn robot_2_runs_without_robot_1_and_selection_errors_are_specific() {
        let sim = Simulation::with_vehicles(ROBOT_2, &[VehicleConfig::new("robot_2", "robot_2/")])
            .unwrap();
        assert_eq!(sim.vehicle_names(), ["robot_2"]);
        let error = Simulation::with_vehicles(ROBOT_2, &[VehicleConfig::legacy()])
            .err()
            .unwrap();
        assert!(error.contains("robot_1"), "{error}");
        let error = Simulation::with_vehicles(DUAL, &[VehicleConfig::new("robot_2", "")])
            .err()
            .unwrap();
        assert!(error.contains("thruster_1_actuator"), "{error}");
        let duplicate = vec![VehicleConfig::new("robot_1", "robot_1/"); 2];
        assert!(Simulation::with_vehicles(DUAL, &duplicate).is_err());
        let shared = vec![
            VehicleConfig::new("robot_1", "robot_1/"),
            VehicleConfig::new("robot_2", "robot_1/"),
        ];
        assert!(Simulation::with_vehicles(DUAL, &shared).is_err());
        assert!(Simulation::with_vehicles(DUAL, &[]).is_err());
    }

    #[test]
    fn selecting_one_vehicle_of_the_dual_model_leaves_the_other_unregistered() {
        let sim =
            Simulation::with_vehicles(DUAL, &[VehicleConfig::new("robot_2", "robot_2/")]).unwrap();
        assert_eq!(sim.vehicle_names(), ["robot_2"]);
        assert!(sim.vehicle_state("robot_1").is_err());
    }

    #[test]
    fn commands_disturbances_and_initial_poses_stay_on_one_vehicle() {
        let mut sim = Simulation::with_vehicles(DUAL, &both()).unwrap();
        sim.set_initial_pose("robot_1", 0.6, 1.0, 0.0).unwrap();
        sim.set_initial_pose("robot_2", 0.3, 0.7, 0.5).unwrap();
        let r2 = state(&sim, "robot_2");
        assert!((r2[0] - 0.3).abs() < 1e-12 && (r2[1] - 0.7).abs() < 1e-12);
        assert!((state(&sim, "robot_1")[0] - 0.6).abs() < 1e-12);
        sim.fire_thruster("robot_1", "thruster_3", 0.1).unwrap();
        sim.set_disturbance("robot_1", "table_1", [0.02, 0.0, 0.0], None, 0.1)
            .unwrap();
        // Frames are per vehicle: robot_1 cannot use table_2 and vice versa.
        assert!(
            sim.set_disturbance("robot_2", "table_1", [0.02, 0.0, 0.0], None, 0.1)
                .is_err()
        );
        for _ in 0..100 {
            sim.step();
        }
        let r1 = state(&sim, "robot_1");
        assert!(r1[4].hypot(r1[5]) > 1e-3, "robot_1 did not move: {r1:?}");
        let r2 = state(&sim, "robot_2");
        assert_eq!([r2[4], r2[5], r2[6]], [0.0; 3]);
        assert!((r2[0] - 0.3).abs() < 1e-12 && (r2[1] - 0.7).abs() < 1e-12);
        assert_eq!(
            sim.applied_disturbance("robot_2").unwrap().body_wrench,
            [0.0; 3]
        );
    }

    /// Moving/rotating table_2 in world must not change robot_2's table-local motion.
    #[test]
    fn table_world_placement_does_not_change_local_states() {
        // An in-memory model has no directory: resolve marker meshes/textures absolutely.
        let xml = std::fs::read_to_string(DUAL).unwrap().replace(
            r#"<compiler angle="degree"/>"#,
            concat!(
                r#"<compiler angle="degree" assetdir=""#,
                env!("CARGO_MANIFEST_DIR"),
                r#"/mujoco"/>"#
            ),
        );
        let original = r#"<body name="table_2" pos="5.0 0 0">"#;
        assert!(xml.contains(original));
        let run = |placement: &str| {
            let model = xml.replace(original, placement);
            let mut sim = Simulation::from_xml_string(&model, &both()).unwrap();
            sim.set_initial_pose("robot_2", 0.4, 1.2, 0.3).unwrap();
            sim.set_disturbance("robot_2", "table_2", [0.01, -0.02, 0.0002], None, 0.2)
                .unwrap();
            let mut samples = Vec::new();
            for index in 0..300 {
                if index % 50 == 0 {
                    sim.fire_thruster("robot_2", "thruster_1", 0.02).unwrap();
                    sim.fire_thruster("robot_2", "thruster_4", 0.04).unwrap();
                }
                sim.step();
                samples.push(state(&sim, "robot_2"));
            }
            let table = sim
                .table_poses()
                .into_iter()
                .find(|t| t.name == "table_2")
                .unwrap();
            (samples, table.position)
        };
        let (reference, origin) = run(original);
        assert_eq!(origin, [5.0, 0.0, 0.0]);
        for placement in [
            r#"<body name="table_2" pos="4.0 -3.0 0.2">"#,
            r#"<body name="table_2" pos="-2.0 5.0 0" euler="0 0 37">"#,
        ] {
            let (moved, origin) = run(placement);
            assert_ne!(origin, [5.0, 0.0, 0.0]);
            for (a, b) in reference.iter().zip(&moved) {
                for (x, y) in a.iter().zip(b) {
                    assert!((x - y).abs() < 1e-9, "{placement}: {a:?} vs {b:?}");
                }
            }
        }
    }
}
