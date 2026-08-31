use crate::io::pwm::DummyPwm;

#[derive(Debug)]
pub struct EscDriver {
    pwm: DummyPwm,
    min_pulse_width_us: u16,
    max_pulse_width_us: u16,
}

impl EscDriver {
    pub fn new(pwm: DummyPwm, min_pulse_width_us: u16, max_pulse_width_us: u16) -> Self {
        Self {
            pwm,
            min_pulse_width_us,
            max_pulse_width_us,
        }
    }

    pub fn set_command(&mut self, normalized_command: f64) -> Result<(), String> {
        if !(0.0..=1.0).contains(&normalized_command) {
            return Err(format!("ESC command out of range: {}", normalized_command));
        }

        let range = f64::from(self.max_pulse_width_us - self.min_pulse_width_us);

        let pulse_width_us = f64::from(self.min_pulse_width_us) + normalized_command * range;
        let pulse_width_us = pulse_width_us.round() as u16;

        println!(
            "[driver] ESC command: {:.3} -> {} us",
            normalized_command, pulse_width_us
        );

        self.pwm.set_pulse_width_us(pulse_width_us)
    }
}
