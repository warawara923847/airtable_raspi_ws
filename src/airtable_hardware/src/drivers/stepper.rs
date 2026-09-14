use crate::io::gpio::{Direction, DummyGpio};

#[derive(Debug)]
pub struct StepperDriver {
    steps_per_revolution: u32,
    microsteps: u32,
    gpio: DummyGpio,
}

impl StepperDriver {
    pub fn new(axis_name: impl Into<String>, steps_per_revolution: u32, microsteps: u32) -> Self {
        Self {
            steps_per_revolution,
            microsteps,
            gpio: DummyGpio::new(axis_name),
        }
    }

    pub fn move_revolutions(&mut self, revolutions: f64) -> Result<u64, String> {
        if !revolutions.is_finite() {
            return Err("revolutions must be finite".to_string());
        }

        if revolutions == 0.0 {
            return Ok(0);
        }

        let direction = if revolutions > 0.0 {
            Direction::Positive
        } else {
            Direction::Negative
        };

        let microsteps_per_revolution = self.steps_per_revolution as f64 * self.microsteps as f64;

        let step_count = (revolutions.abs() * microsteps_per_revolution).round() as u64;

        println!(
            "[driver] {:.4} rev -> {} microsteps",
            revolutions, step_count
        );

        self.gpio.set_direction(direction);
        self.gpio.pulse_steps(step_count);

        Ok(step_count)
    }
}
