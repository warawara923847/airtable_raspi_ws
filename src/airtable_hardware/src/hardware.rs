use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::devices::reaction_wheel::{DebugReactionWheel, DebugWheelState};
use crate::devices::thruster::Thruster;
use crate::devices::xy_table::XyTable;

pub struct HardwareState {
    thrusters: HashMap<String, Thruster>,
    xy_table: XyTable,
    reaction_wheel: Option<DebugReactionWheel>,
}

impl HardwareState {
    pub fn new(reaction_wheel: Option<DebugReactionWheel>) -> Self {
        let mut thrusters = HashMap::new();

        for name in ["thruster_1", "thruster_2", "thruster_3", "thruster_4"] {
            thrusters.insert(name.to_string(), Thruster::new(name, 0.5));
        }

        Self {
            thrusters,
            xy_table: XyTable::new_dummy(),
            reaction_wheel,
        }
    }

    pub fn set_reaction_wheel_torque(&mut self, name: &str, torque_nm: f64) -> Result<(), String> {
        let wheel = self.reaction_wheel.as_mut().ok_or("debug RW is disabled")?;
        if wheel.name() != name {
            return Err(format!("unknown reaction wheel: {name}"));
        }
        wheel.set_torque(torque_nm, Instant::now())
    }

    pub fn update_reaction_wheel(&mut self) -> Result<Option<DebugWheelState>, String> {
        self.reaction_wheel
            .as_mut()
            .map(|wheel| wheel.update(Instant::now()))
            .transpose()
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
