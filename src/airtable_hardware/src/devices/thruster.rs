use crate::drivers::esc::EscDriver;

#[derive(Debug)]
pub struct Thruster {
    name: String,
    max_thrust_n: f64,
    esc: EscDriver,
}

impl Thruster {
    pub fn new(name: impl Into<String>, max_thrust_n: f64, esc: EscDriver) -> Self {
        Self {
            name: name.into(),
            max_thrust_n,
            esc,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn set_nominal_thrust(&mut self, thrust_n: f64) -> Result<(), String> {
        if !thrust_n.is_finite() {
            return Err(format!("{}: thrust command is not finite", self.name));
        }

        if !(0.0..=self.max_thrust_n).contains(&thrust_n) {
            return Err(format!(
                "{}: thrust {:.3} N is outside [0, {:.3}] N",
                self.name, thrust_n, self.max_thrust_n
            ));
        }

        // Dummy linear model.
        //
        // Later this should be replaced with the actual thrust calibration model.
        let normalized_command = thrust_n / self.max_thrust_n;

        println!(
            "[device] {}: {:.3} N -> {:.3}",
            self.name, thrust_n, normalized_command
        );

        self.esc.set_command(normalized_command)
    }
}
