use std::collections::HashMap;
use std::time::{Duration, Instant};

use crate::devices::arw3m_wheel::Arw3mWheel;
use crate::devices::reaction_wheel::{DebugReactionWheel, WheelState};
use crate::devices::thruster::Thruster;
use crate::devices::xy_table::XyTable;

pub enum ReactionWheel {
    /// SG90 display with a virtual wheel model.
    Debug(DebugReactionWheel),
    Arw3m(Arw3mWheel),
}

impl ReactionWheel {
    fn name(&self) -> &str {
        match self {
            Self::Debug(wheel) => wheel.name(),
            Self::Arw3m(wheel) => wheel.name(),
        }
    }

    fn set_torque(&mut self, torque_nm: f64, now: Instant) -> Result<(), String> {
        match self {
            Self::Debug(wheel) => wheel.set_torque(torque_nm, now),
            Self::Arw3m(wheel) => wheel.set_torque(torque_nm, now),
        }
    }

    fn update(&mut self, now: Instant) -> Result<WheelState, String> {
        match self {
            Self::Debug(wheel) => wheel.update(now),
            // The ARW-3m thread does the I/O; this only reads its latest state.
            Self::Arw3m(wheel) => Ok(wheel.state()),
        }
    }
}

pub struct HardwareState {
    thrusters: HashMap<String, Thruster>,
    xy_table: XyTable,
    reaction_wheel: Option<ReactionWheel>,
}

impl HardwareState {
    pub fn new(thrusters: Vec<Thruster>, reaction_wheel: Option<ReactionWheel>) -> Self {
        Self {
            thrusters: thrusters
                .into_iter()
                .map(|thruster| (thruster.name().to_string(), thruster))
                .collect(),
            xy_table: XyTable::new_dummy(),
            reaction_wheel,
        }
    }

    pub fn set_reaction_wheel_torque(&mut self, name: &str, torque_nm: f64) -> Result<(), String> {
        let wheel = self.reaction_wheel.as_mut().ok_or("RW is disabled")?;
        if wheel.name() != name {
            return Err(format!("unknown reaction wheel: {name}"));
        }
        wheel.set_torque(torque_nm, Instant::now())
    }

    pub fn update_reaction_wheel(&mut self) -> Result<Option<WheelState>, String> {
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

        thruster.fire_for(duration, Instant::now())
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
