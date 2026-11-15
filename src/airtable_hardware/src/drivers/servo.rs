use crate::io::servo_pwm::ServoPwm;

#[derive(Debug, Clone, Copy)]
pub struct ServoCalibration {
    pub min_us: u16,
    pub neutral_us: u16,
    pub max_us: u16,
}

impl ServoCalibration {
    pub fn validate(&self) -> Result<(), String> {
        if !(500 <= self.min_us
            && self.min_us < self.neutral_us
            && self.neutral_us < self.max_us
            && self.max_us <= 2500)
        {
            return Err("require 500 <= min_us < neutral_us < max_us <= 2500".into());
        }
        Ok(())
    }

    pub fn pulse_us(&self, normalized: f64) -> u16 {
        let value = normalized.clamp(-1.0, 1.0);
        let span = if value < 0.0 {
            self.neutral_us - self.min_us
        } else {
            self.max_us - self.neutral_us
        };
        (f64::from(self.neutral_us) + value * f64::from(span)).round() as u16
    }
}

#[derive(Debug)]
pub struct ServoDriver {
    pwm: ServoPwm,
    calibration: ServoCalibration,
}

impl ServoDriver {
    pub fn new(mut pwm: ServoPwm, calibration: ServoCalibration) -> Result<Self, String> {
        calibration.validate()?;
        pwm.set_pulse_width_us(calibration.neutral_us)?;
        Ok(Self { pwm, calibration })
    }

    pub fn set_normalized(&mut self, value: f64) -> Result<(), String> {
        if !value.is_finite() {
            return Err("non-finite servo command".into());
        }
        self.pwm
            .set_pulse_width_us(self.calibration.pulse_us(value))
    }

    pub fn disable(&mut self) -> Result<(), String> {
        self.pwm.disable()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn asymmetric_calibration_clamps_both_directions() {
        let calibration = ServoCalibration {
            min_us: 1200,
            neutral_us: 1500,
            max_us: 1700,
        };
        calibration.validate().unwrap();
        assert_eq!(calibration.pulse_us(-2.0), 1200);
        assert_eq!(calibration.pulse_us(-0.5), 1350);
        assert_eq!(calibration.pulse_us(0.0), 1500);
        assert_eq!(calibration.pulse_us(0.5), 1600);
        assert_eq!(calibration.pulse_us(2.0), 1700);
        assert!(
            ServoCalibration {
                min_us: 1500,
                ..calibration
            }
            .validate()
            .is_err()
        );
    }
}
