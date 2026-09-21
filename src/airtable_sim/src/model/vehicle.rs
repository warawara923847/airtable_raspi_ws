use std::collections::HashMap;

use crate::actuators::{SimReactionWheel, SimThruster};
use crate::sensors::SimImu;

#[derive(Debug)]
pub struct SimVehicle {
    pub name: String,
    pub body_id: usize,

    pub thrusters: HashMap<String, SimThruster>,
    pub reaction_wheels: HashMap<String, SimReactionWheel>,
    pub imus: HashMap<String, SimImu>,
}

impl SimVehicle {
    pub fn new(
        name: impl Into<String>,
        body_id: usize,
        thrusters: HashMap<String, SimThruster>,
        reaction_wheels: HashMap<String, SimReactionWheel>,
        imus: HashMap<String, SimImu>,
    ) -> Self {
        Self {
            name: name.into(),
            body_id,
            thrusters,
            reaction_wheels,
            imus,
        }
    }

    pub fn fire_thruster(
        &mut self,
        thruster_name: &str,
        now: f64,
        duration_s: f64,
    ) -> Result<(), String> {
        let thruster = self
            .thrusters
            .get_mut(thruster_name)
            .ok_or_else(|| format!("unknown thruster: {}/{}", self.name, thruster_name))?;

        thruster.fire_for(now, duration_s);

        Ok(())
    }

    pub fn set_reaction_wheel_torque(
        &mut self,
        wheel_name: &str,
        torque_nm: f64,
    ) -> Result<(), String> {
        let wheel = self
            .reaction_wheels
            .get_mut(wheel_name)
            .ok_or_else(|| format!("unknown thruster: {} on vehicle {}", wheel_name, self.name))?;

        wheel.set_torque(torque_nm);

        Ok(())
    }

    pub fn reaction_wheel(&self, wheel_name: &str) -> Result<&SimReactionWheel, String> {
        self.reaction_wheels.get(wheel_name).ok_or_else(|| {
            format!(
                "unknown reaction wheel: '{}' on vehicle '{}'",
                wheel_name, self.name
            )
        })
    }

    pub fn update_thrusters(&mut self, now: f64, ctrl: &mut [f64]) {
        for thruster in self.thrusters.values_mut() {
            thruster.update(now, ctrl);
        }
    }

    pub fn update_reaction_wheels(&mut self, ctrl: &mut [f64], dt: f64) {
        for wheel in self.reaction_wheels.values_mut() {
            wheel.update(ctrl, dt);
        }
    }
}
