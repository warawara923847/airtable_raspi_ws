use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;
use mujoco_rs::viewer::ViewerSharedState;

use crate::state::PlanarState;
use crate::table::SimTable;
use crate::thruster::SimThruster;
use crate::vehicle::SimVehicle;

struct ThrusterDefinition {
    name: &'static str,
    actuator_name: &'static str,
    thrust_n: f64,
}

struct VehicleDefinition {
    name: &'static str,
    body_name: &'static str,
    thrusters: &'static [ThrusterDefinition],
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

const VEHICLE_DEFINITIONS: &[VehicleDefinition] = &[VehicleDefinition {
    name: "robot_1",
    body_name: "robot_1",
    thrusters: ROBOT_1_THRUSTERS,
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
        data: MjData<Box<MjModel>>,
        viewer_state: Option<Arc<Mutex<ViewerSharedState>>>,
    ) -> Result<Self, String> {
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

                println!(
                    "[sim] {}/{} -> {} (actuator id={})",
                    vehicle_def.name, thruster_def.name, thruster_def.actuator_name, actuator_id
                );

                thrusters.insert(
                    thruster_def.name.to_string(),
                    SimThruster::new(thruster_def.name, actuator_id, thruster_def.thrust_n),
                );
            }

            // ------------------------------------------------
            // Vehicle
            // ------------------------------------------------
            vehicles.insert(
                vehicle_def.name.to_string(),
                SimVehicle::new(vehicle_def.name, body_id, thrusters),
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

    pub fn step(&mut self) {
        let now = self.data.time();

        {
            let ctrl = self.data.ctrl_mut();

            for vehicle in self.vehicles.values_mut() {
                vehicle.update_thrusters(now, ctrl);
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
