use crate::io::servo_pwm::ServoPwm;

#[derive(Debug, Clone, Copy)]
pub struct ServoCalibration {
    pub min_us: u16,
    pub neutral_us: u16,
    pub max_us: u16,
    /// Half-width around `neutral_us` that a continuous-rotation servo ignores. A non-zero
    /// command starts at `neutral_us ± deadband_us`. 0 for position servos.
    pub deadband_us: u16,
}

impl ServoCalibration {
    pub fn validate(&self) -> Result<(), String> {
        let low = self.neutral_us.checked_sub(self.deadband_us);
        let high = self.neutral_us.checked_add(self.deadband_us);
        if !(500 <= self.min_us
            && low.is_some_and(|low| self.min_us < low)
            && high.is_some_and(|high| high < self.max_us)
            && self.max_us <= 2500)
        {
            return Err("require 500 <= min_us < neutral_us - deadband_us \
                 and neutral_us + deadband_us < max_us <= 2500"
                .into());
        }
        Ok(())
    }

    pub fn pulse_us(&self, normalized: f64) -> u16 {
        let value = normalized.clamp(-1.0, 1.0);
        if value == 0.0 {
            return self.neutral_us;
        }
        let neutral = f64::from(self.neutral_us);
        let deadband = f64::from(self.deadband_us);
        let (start, end) = if value < 0.0 {
            (neutral - deadband, f64::from(self.min_us))
        } else {
            (neutral + deadband, f64::from(self.max_us))
        };
        (start + value.abs() * (end - start)).round() as u16
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
            deadband_us: 0,
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

    #[test]
    fn deadband_skips_the_stopped_band_in_both_directions() {
        let calibration = ServoCalibration {
            min_us: 1000,
            neutral_us: 1500,
            max_us: 2000,
            deadband_us: 50,
        };
        calibration.validate().unwrap();
        assert_eq!(calibration.pulse_us(0.0), 1500);
        assert_eq!(calibration.pulse_us(0.001), 1550);
        assert_eq!(calibration.pulse_us(-0.001), 1450);
        assert_eq!(calibration.pulse_us(0.5), 1775);
        assert_eq!(calibration.pulse_us(-0.5), 1225);
        assert_eq!(calibration.pulse_us(1.0), 2000);
        assert_eq!(calibration.pulse_us(-3.0), 1000);
        for deadband_us in [500, 1600] {
            assert!(
                ServoCalibration {
                    deadband_us,
                    ..calibration
                }
                .validate()
                .is_err()
            );
        }
    }
}
