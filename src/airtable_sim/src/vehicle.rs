use std::collections::HashMap;

use crate::thruster::SimThruster;

#[derive(Debug)]
pub struct SimVehicle {
    pub name: String,
    pub body_id: usize,
    pub thrusters: HashMap<String, SimThruster>,
}

impl SimVehicle {
    pub fn new(
        name: impl Into<String>,
        body_id: usize,
        thrusters: HashMap<String, SimThruster>,
    ) -> Self {
        Self {
            name: name.into(),
            body_id,
            thrusters,
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

    pub fn update_thrusters(&mut self, now: f64, ctrl: &mut [f64]) {
        for thruster in self.thrusters.values_mut() {
            thruster.update(now, ctrl);
        }
    }
}
