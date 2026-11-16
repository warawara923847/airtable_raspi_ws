//! Continuous-rotation servo (e.g. SG90-HV 360°) standing in for an RW until the ARW-3m works.
//! A normalized speed command in [-1, 1] maps to a pulse width. The servo has no feedback,
//! so neither its speed nor the torque it produces is measured or reported.
use std::time::{Duration, Instant};

use crate::drivers::servo::ServoDriver;

#[derive(Debug, Clone)]
pub struct ContinuousServoConfig {
    pub name: String,
    /// Flip the pulse side so that +1 turns the wheel CCW about base_link +z as mounted.
    pub reversed: bool,
    pub command_timeout: Duration,
}

impl ContinuousServoConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.name.is_empty() || self.command_timeout.is_zero() {
            return Err("RW name and positive command timeout are required".into());
        }
        Ok(())
    }
}

pub struct ContinuousServoWheel {
    config: ContinuousServoConfig,
    driver: ServoDriver,
    commanded: f64,
    last_command: Option<Instant>,
    faulted: bool,
}

impl ContinuousServoWheel {
    /// `driver` must already output the stop pulse (ServoDriver::new does).
    pub fn new(config: ContinuousServoConfig, driver: ServoDriver) -> Result<Self, String> {
        config.validate()?;
        eprintln!(
            "[hardware] {}: continuous-rotation servo stand-in; speed commands only, \
             no speed or torque feedback",
            config.name
        );
        Ok(Self {
            config,
            driver,
            commanded: 0.0,
            last_command: None,
            faulted: false,
        })
    }

    pub fn name(&self) -> &str {
        &self.config.name
    }

    pub fn set_speed(&mut self, speed: f64, now: Instant) -> Result<(), String> {
        if self.faulted {
            return Err("faulted servo output; restart required".into());
        }
        if !speed.is_finite() || speed.abs() > 1.0 {
            return Err(format!("speed command {speed} is outside [-1, 1]"));
        }
        self.commanded = speed;
        self.last_command = Some(now);
        Ok(())
    }

    /// Applies the command timeout and writes the pulse. Returns the commanded speed.
    pub fn update(&mut self, now: Instant) -> Result<f64, String> {
        if self
            .last_command
            .is_some_and(|last| now.duration_since(last) >= self.config.command_timeout)
        {
            self.commanded = 0.0;
            self.last_command = None;
            eprintln!(
                "[hardware] {}: speed command timed out; stopping servo",
                self.config.name
            );
        }
        if self.faulted {
            return Ok(0.0);
        }
        let output = if self.config.reversed {
            -self.commanded
        } else {
            self.commanded
        };
        if let Err(error) = self.driver.set_normalized(output) {
            self.faulted = true;
            self.commanded = 0.0;
            let disable_result = self.driver.disable();
            return Err(format!(
                "servo PWM fault: {error}; disable: {disable_result:?}; restart required"
            ));
        }
        Ok(self.commanded)
    }
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::path::{Path, PathBuf};

    use super::*;
    use crate::drivers::servo::ServoCalibration;
    use crate::io::servo_pwm::ServoPwm;

    fn fake_pwm(tag: &str) -> PathBuf {
        let path =
            std::env::temp_dir().join(format!("airtable-continuous-{tag}-{}", std::process::id()));
        fs::create_dir(&path).unwrap();
        for (name, value) in [
            ("enable", "0"),
            ("period", "0"),
            ("duty_cycle", "0"),
            ("polarity", "normal"),
        ] {
            fs::write(path.join(name), value).unwrap();
        }
        path
    }

    fn wheel(path: &Path, reversed: bool) -> ContinuousServoWheel {
        let calibration = ServoCalibration {
            min_us: 1000,
            neutral_us: 1500,
            max_us: 2000,
            deadband_us: 50,
        };
        ContinuousServoWheel::new(
            ContinuousServoConfig {
                name: "rw_1".into(),
                reversed,
                command_timeout: Duration::from_millis(500),
            },
            ServoDriver::new(ServoPwm::linux(path, 1500).unwrap(), calibration).unwrap(),
        )
        .unwrap()
    }

    fn duty_us(path: &Path) -> u32 {
        fs::read_to_string(path.join("duty_cycle"))
            .unwrap()
            .parse::<u32>()
            .unwrap()
            / 1000
    }

    #[test]
    fn maps_speed_through_deadband_and_stops_on_timeout() {
        let path = fake_pwm("map");
        {
            let now = Instant::now();
            let mut wheel = wheel(&path, false);
            assert_eq!(duty_us(&path), 1500);
            wheel.set_speed(0.5, now).unwrap();
            assert_eq!(wheel.update(now).unwrap(), 0.5);
            assert_eq!(duty_us(&path), 1775);
            wheel.set_speed(-1.0, now).unwrap();
            wheel.update(now).unwrap();
            assert_eq!(duty_us(&path), 1000);
            // Rejected commands neither move the servo nor refresh the watchdog.
            for bad in [f64::NAN, f64::INFINITY, 1.5] {
                assert!(wheel.set_speed(bad, now).is_err());
            }
            let late = now + Duration::from_millis(500);
            assert_eq!(wheel.update(late).unwrap(), 0.0);
            assert_eq!(duty_us(&path), 1500);
        }
        assert_eq!(fs::read_to_string(path.join("enable")).unwrap(), "0");
        fs::remove_dir_all(path).unwrap();
    }

    #[test]
    fn reversed_mounting_flips_the_pulse_side() {
        let path = fake_pwm("reversed");
        {
            let now = Instant::now();
            let mut wheel = wheel(&path, true);
            wheel.set_speed(0.5, now).unwrap();
            assert_eq!(wheel.update(now).unwrap(), 0.5);
            assert_eq!(duty_us(&path), 1225);
        }
        fs::remove_dir_all(path).unwrap();
    }

    #[test]
    fn pwm_failure_faults_until_restart() {
        let path = fake_pwm("fault");
        let now = Instant::now();
        let mut wheel = wheel(&path, false);
        fs::remove_dir_all(&path).unwrap();
        wheel.set_speed(0.5, now).unwrap();
        assert!(wheel.update(now).is_err());
        assert!(wheel.set_speed(0.5, now).is_err());
        assert_eq!(wheel.update(now).unwrap(), 0.0);
    }
}
