use std::collections::HashMap;

use crate::devices::thruster::Thruster;
use crate::drivers::esc::EscDriver;
use crate::io::pwm::DummyPwm;

pub struct HardwareState {
    thrusters: HashMap<String, Thruster>,
}

impl HardwareState {
    pub fn new_dummy() -> Self {
        let mut thrusters = HashMap::new();

        let configs = [
            ("thruster_1", 0_u8, 0.5_f64),
            ("thruster_2", 1_u8, 0.5_f64),
            ("thruster_3", 2_u8, 0.5_f64),
        ];

        for (name, pwm_channel, max_thrust_n) in configs {
            let pwm = DummyPwm::new(pwm_channel);

            let esc = EscDriver::new(pwm, 1000, 2000);

            let thruster = Thruster::new(name, max_thrust_n, esc);

            thrusters.insert(name.to_string(), thruster);
        }

        Self { thrusters }
    }

    pub fn set_thruster(&mut self, name: &str, thrust_n: f64) -> Result<(), String> {
        let thruster = self
            .thrusters
            .get_mut(name)
            .ok_or_else(|| format!("Unknown thruster: {name}"))?;

        thruster.set_nominal_thrust(thrust_n)
    }
}
