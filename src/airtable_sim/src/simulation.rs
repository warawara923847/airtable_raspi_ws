use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;
use mujoco_rs::viewer::ViewerSharedState;

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
    body_name: "airtable",
    thrusters: ROBOT_1_THRUSTERS,
}];

pub struct Simulation {
    data: MjData<Box<MjModel>>,
    vehicles: HashMap<String, SimVehicle>,
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

        Ok(Self {
            data,
            vehicles,
            viewer_state,
        })
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
