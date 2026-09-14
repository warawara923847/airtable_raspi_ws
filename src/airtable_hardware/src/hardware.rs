use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::devices::thruster::Thruster;
use crate::devices::xy_table::XyTable;

pub struct HardwareState {
    thrusters: HashMap<String, Thruster>,
    xy_table: XyTable,
}

impl HardwareState {
    pub fn new_dummy() -> Self {
        let mut thrusters = HashMap::new();

        for name in ["thruster_1", "thruster_2", "thruster_3", "thruster_4"] {
            thrusters.insert(name.to_string(), Thruster::new(name, 0.5));
        }

        Self {
            thrusters,
            xy_table: XyTable::new_dummy(),
        }
    }

    pub fn fire_thruster(&mut self, name: &str, duration: Duration) -> Result<(), String> {
        let thruster = self
            .thrusters
            .get_mut(name)
            .ok_or_else(|| format!("Unknown thruster: {name}"))?;

        thruster.fire_for(duration)
    }

    pub fn set_thruster_enabled(&mut self, name: &str, enabled: bool) -> Result<(), String> {
        let thruster = self
            .thrusters
            .get_mut(name)
            .ok_or_else(|| format!("Unknown thruster: {name}"))?;

        if enabled {
            thruster.turn_on()
        } else {
            thruster.turn_off()
        }
    }

    pub fn update_thrusters(&mut self) -> Result<(), String> {
        let now = Instant::now();

        for thruster in self.thrusters.values_mut() {
            thruster.update(now)?;
        }

        Ok(())
    }

    pub fn move_xy_table(&mut self, x_m: f64, y_m: f64) -> Result<(), String> {
        self.xy_table.move_to(x_m, y_m)
    }

    pub fn xy_table_position(&self) -> (f64, f64) {
        self.xy_table.position()
    }
}
