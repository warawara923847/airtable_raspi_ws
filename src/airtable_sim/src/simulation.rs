use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use mujoco_rs::prelude::*;
use mujoco_rs::viewer::MjViewer;
use mujoco_rs::viewer::ViewerSharedState;

use crate::thruster::SimThruster;

pub struct Simulation {
    data: MjData<Box<MjModel>>,
    thrusters: HashMap<String, SimThruster>,

    viewer_state: Option<Arc<Mutex<ViewerSharedState>>>,
}

impl Simulation {
    pub fn new(
        model_path: &str,
        viewer_state: Option<Arc<Mutex<ViewerSharedState>>>,
    ) -> Result<Self, String> {
        let model =
            MjModel::from_xml(model_path).map_err(|e| format!("Failed to load model: {}", e))?;

        let data = MjData::new(Box::new(model));

        let definitions = [
            ("thruster_1", "thruster_1_actuator", 0.5),
            ("thruster_2", "thruster_2_actuator", 0.5),
            ("thruster_3", "thruster_3_actuator", 0.5),
            ("thruster_4", "thruster_4_actuator", 0.5),
        ];

        let mut thrusters = HashMap::new();

        for (name, actuator_name, thrust_n) in definitions {
            let actuator_info = data
                .actuator(actuator_name)
                .ok_or_else(|| format!("MuJoCo actuator not found: {actuator_name}"))?;

            let actuator_id = actuator_info.id;

            println!("[sim] {name} -> {actuator_name} (id={actuator_id})");

            thrusters.insert(
                name.to_string(),
                SimThruster::new(name, actuator_id, thrust_n),
            );
        }

        Ok(Self {
            data,
            thrusters,
            viewer_state,
        })
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
        let definitions = [
            ("thruster_1", "thruster_1_actuator", 0.5),
            ("thruster_2", "thruster_2_actuator", 0.5),
            ("thruster_3", "thruster_3_actuator", 0.5),
            ("thruster_4", "thruster_4_actuator", 0.5),
        ];

        let mut thrusters = HashMap::new();

        for (name, actuator_name, thrust_n) in definitions {
            let actuator_info = data
                .actuator(actuator_name)
                .ok_or_else(|| format!("MuJoCo actuator not found: {actuator_name}"))?;

            let actuator_id = actuator_info.id;

            println!("[sim] {name} -> {actuator_name} (id={actuator_id})");

            thrusters.insert(
                name.to_string(),
                SimThruster::new(name, actuator_id, thrust_n),
            );
        }

        Ok(Self {
            data,
            thrusters,
            viewer_state,
        })
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

    pub fn fire_thruster(&mut self, name: &str, duration_s: f64) -> Result<(), String> {
        let now = self.data.time();

        let thruster = self
            .thrusters
            .get_mut(name)
            .ok_or_else(|| format!("unknown thruster: {name}"))?;

        thruster.fire_for(now, duration_s);

        Ok(())
    }

    pub fn step(&mut self) {
        let now = self.data.time();

        {
            let ctrl = self.data.ctrl_mut();

            for thruster in self.thrusters.values_mut() {
                thruster.update(now, ctrl);
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
