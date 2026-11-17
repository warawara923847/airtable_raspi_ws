//! 50 Hz servo pulses. The Linux backend owns an already exported PWM channel.
use std::fs::{self, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};

const PERIOD_NS: u32 = 20_000_000;

#[derive(Debug)]
pub enum ServoPwm {
    Dummy { last_us: Option<u16> },
    Linux { path: PathBuf },
}

fn write_attribute(path: &Path, name: &str, value: impl ToString) -> Result<(), String> {
    let attribute = path.join(name);
    OpenOptions::new()
        .write(true)
        .truncate(true)
        .open(&attribute)
        .and_then(|mut file| file.write_all(value.to_string().as_bytes()))
        .map_err(|error| format!("{}: {error}", attribute.display()))
}

impl ServoPwm {
    pub fn dummy() -> Self {
        Self::Dummy { last_us: None }
    }

    pub fn linux(path: impl Into<PathBuf>, neutral_us: u16) -> Result<Self, String> {
        let path = path.into();
        let enabled = fs::read_to_string(path.join("enable")).map_err(|error| {
            format!("{}: {error}; export the PWM channel first", path.display())
        })?;
        if enabled.trim() != "0" {
            return Err(format!(
                "{} is already enabled; refusing to take over",
                path.display()
            ));
        }
        let mut pwm = Self::Linux { path: path.clone() };
        // The kernel rejects every attribute write (EINVAL) unless 0 < period and
        // duty_cycle <= period hold afterwards. A freshly exported Pi 4 (BCM2711) channel
        // has period 0, so the period goes first there; otherwise the duty is cleared first
        // so that an old duty longer than the new period cannot block the period write.
        let period = fs::read_to_string(path.join("period"))
            .map_err(|error| format!("{}/period: {error}", path.display()))?;
        if period.trim() == "0" {
            write_attribute(&path, "period", PERIOD_NS)?;
            write_attribute(&path, "duty_cycle", 0)?;
        } else {
            write_attribute(&path, "duty_cycle", 0)?;
            write_attribute(&path, "period", PERIOD_NS)?;
        }
        write_attribute(&path, "polarity", "normal")?;
        pwm.set_pulse_width_us(neutral_us)?;
        write_attribute(&path, "enable", 1)?;
        Ok(pwm)
    }

    pub fn set_pulse_width_us(&mut self, pulse_us: u16) -> Result<(), String> {
        if !(500..=2500).contains(&pulse_us) {
            return Err("servo pulse must be within 500..2500 us".into());
        }
        match self {
            Self::Dummy { last_us } => {
                if *last_us != Some(pulse_us) {
                    println!("[io] dummy SG90 PWM: {pulse_us} us (50 Hz)");
                    *last_us = Some(pulse_us);
                }
                Ok(())
            }
            Self::Linux { path } => write_attribute(path, "duty_cycle", u32::from(pulse_us) * 1000),
        }
    }

    pub fn disable(&mut self) -> Result<(), String> {
        match self {
            Self::Dummy { .. } => Ok(()),
            Self::Linux { path } => write_attribute(path, "enable", 0),
        }
    }
}

impl Drop for ServoPwm {
    fn drop(&mut self) {
        if let Err(error) = self.disable() {
            eprintln!("[io] failed to disable SG90 PWM: {error}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn linux_pwm_initializes_updates_and_disables_without_taking_over() {
        let path = std::env::temp_dir().join(format!("airtable-pwm-{}", std::process::id()));
        fs::create_dir(&path).unwrap();
        for (name, value) in [
            ("enable", "0"),
            ("period", "0"),
            ("duty_cycle", "0"),
            ("polarity", "inversed"),
        ] {
            fs::write(path.join(name), value).unwrap();
        }
        {
            let mut pwm = ServoPwm::linux(&path, 1500).unwrap();
            assert_eq!(fs::read_to_string(path.join("period")).unwrap(), "20000000");
            assert_eq!(fs::read_to_string(path.join("polarity")).unwrap(), "normal");
            assert_eq!(fs::read_to_string(path.join("enable")).unwrap(), "1");
            assert!(ServoPwm::linux(&path, 1500).is_err());
            pwm.set_pulse_width_us(1700).unwrap();
            assert_eq!(
                fs::read_to_string(path.join("duty_cycle")).unwrap(),
                "1700000"
            );
            assert!(pwm.set_pulse_width_us(0).is_err());
        }
        assert_eq!(fs::read_to_string(path.join("enable")).unwrap(), "0");
        fs::remove_dir_all(path).unwrap();
    }
}
