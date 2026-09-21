use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;
use mujoco_rs::viewer::ViewerSharedState;

use crate::actuators::{ReactionWheelStateData, SimReactionWheel, SimThruster};
use crate::model::{SimTable, SimVehicle};
use crate::sensors::{ImuState, SimImu};
use crate::state::{PlanarState, TablePose};

struct ThrusterDefinition {
    name: &'static str,
    actuator_name: &'static str,
    thrust_n: f64,
}

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

struct VehicleDefinition {
    name: &'static str,
    body_name: &'static str,
    thrusters: &'static [ThrusterDefinition],
    reaction_wheels: &'static [ReactionWheelDefinition],
    imus: &'static [ImuDefinition],
}

const ROBOT_1_THRUSTERS: &[ThrusterDefinition] = &[
    ThrusterDefinition {
        name: "thruster_1",
        actuator_name: "thruster_1_actuator",
        thrust_n: 0.5,
    },
    ThrusterDefinition {
        name: "thruster_2",
        actuator_name: "thruster_2_actuator",
        thrust_n: 0.5,
    },
    ThrusterDefinition {
        name: "thruster_3",
        actuator_name: "thruster_3_actuator",
        thrust_n: 0.5,
    },
    ThrusterDefinition {
        name: "thruster_4",
        actuator_name: "thruster_4_actuator",
        thrust_n: 0.5,
    },
];

const ROBOT_1_REACTION_WHEELS: &[ReactionWheelDefinition] = &[ReactionWheelDefinition {
    name: "rw_1",
    actuator_name: "reaction_wheel_1_actuator",

    inertia: 1.0e-3,
    max_torque: 0.02,
    max_angular_velocity: 500.0,
}];

const ROBOT_1_IMUS: &[ImuDefinition] = &[ImuDefinition {
    name: "imu_1",
    accel_sensor_name: "imu_1_accel",
    gyro_sensor_name: "imu_1_gyro",
    orientation_sensor_name: "imu_1_orientation",
}];

const VEHICLE_DEFINITIONS: &[VehicleDefinition] = &[VehicleDefinition {
    name: "robot_1",
    body_name: "robot_1",
    thrusters: ROBOT_1_THRUSTERS,
    reaction_wheels: ROBOT_1_REACTION_WHEELS,
    imus: ROBOT_1_IMUS,
}];

pub struct Simulation {
    data: MjData<Box<MjModel>>,
    vehicles: HashMap<String, SimVehicle>,
    tables: HashMap<String, SimTable>,
    viewer_state: Option<Arc<Mutex<ViewerSharedState>>>,
}

impl Simulation {
    pub fn new(model_path: &str) -> Result<Self, String> {
        let model =
            MjModel::from_xml(model_path).map_err(|e| format!("Failed to load model: {}", e))?;

        let data = MjData::new(Box::new(model));

        Self::from_data(data, None)
    }

    pub fn new_with_viewer(model_path: &str) -> Result<(Self, MjViewer), String> {
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

        let simulation = Self::from_data(data, Some(viewer_state))?;

        Ok((simulation, viewer))
    }

    fn from_data(
        mut data: MjData<Box<MjModel>>,
        viewer_state: Option<Arc<Mutex<ViewerSharedState>>>,
    ) -> Result<Self, String> {
        // qposからxpos/xquatを初期計算
        data.forward();

        let vehicles = Self::build_vehicles(&data)?;

        let tables = Self::build_tables(&data, &vehicles)?;

        Ok(Self {
            data,
            vehicles,
            tables,
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

    fn build_vehicles(data: &MjData<Box<MjModel>>) -> Result<HashMap<String, SimVehicle>, String> {
        let mut vehicles = HashMap::new();

        for vehicle_def in VEHICLE_DEFINITIONS {
            // ------------------------------------------------
            // MuJoCo body
            // ------------------------------------------------
            let body_info = data
                .body(vehicle_def.body_name)
                .ok_or_else(|| format!("MuJoCo body not found: {}", vehicle_def.body_name))?;

            let body_id = body_info.id;

            println!(
                "[sim] vehicle: {} -> {} (body id={})",
                vehicle_def.name, vehicle_def.body_name, body_id
            );

            // ------------------------------------------------
            // thrusters
            // ------------------------------------------------
            let mut thrusters = HashMap::new();

            for thruster_def in vehicle_def.thrusters {
                let actuator_info = data.actuator(thruster_def.actuator_name).ok_or_else(|| {
                    format!("MuJoCo actuator not found: {}", thruster_def.actuator_name)
                })?;

                let actuator_id = actuator_info.id;

                let control_id = data.model().actuator_ctrladr()[actuator_id] as usize;

                println!(
                    "[sim] {}/{} -> {} (actuator id={}, control_id={})",
                    vehicle_def.name,
                    thruster_def.name,
                    thruster_def.actuator_name,
                    actuator_id,
                    control_id
                );

                thrusters.insert(
                    thruster_def.name.to_string(),
                    SimThruster::new(thruster_def.name, control_id, thruster_def.thrust_n),
                );
            }

            // ------------------------------------------------
            // reaction wheels
            // ------------------------------------------------
            let mut reaction_wheels = HashMap::new();

            for rw_def in vehicle_def.reaction_wheels {
                let actuator_info = data.actuator(rw_def.actuator_name).ok_or_else(|| {
                    format!("MuJoCo actuator not found: {}", rw_def.actuator_name)
                })?;

                let actuator_id = actuator_info.id;

                let control_id = data.model().actuator_ctrladr()[actuator_id] as usize;

                println!(
                    "[sim] {}/{} -> {} (actuator id={}, control id={})",
                    vehicle_def.name, rw_def.name, rw_def.actuator_name, actuator_id, control_id
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

            for imu_def in vehicle_def.imus {
                let accel_info = data.sensor(imu_def.accel_sensor_name).ok_or_else(|| {
                    format!("MuJoCo sensor not found: {}", imu_def.accel_sensor_name)
                })?;

                let gyro_info = data.sensor(imu_def.gyro_sensor_name).ok_or_else(|| {
                    format!("MuJoCo sensor not found: {}", imu_def.gyro_sensor_name)
                })?;

                let orientation_info =
                    data.sensor(imu_def.orientation_sensor_name)
                        .ok_or_else(|| {
                            format!(
                                "MuJoCo sensor not found: {}",
                                imu_def.orientation_sensor_name
                            )
                        })?;

                imus.insert(
                    imu_def.name.to_string(),
                    SimImu::new(
                        imu_def.name,
                        accel_info.id,
                        gyro_info.id,
                        orientation_info.id,
                    ),
                );
            }
            // ------------------------------------------------
            // Vehicle
            // ------------------------------------------------
            vehicles.insert(
                vehicle_def.name.to_string(),
                SimVehicle::new(vehicle_def.name, body_id, thrusters, reaction_wheels, imus),
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

        self.data.step();
    }

    pub fn qpos(&self) -> &[f64] {
        self.data.qpos()
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
