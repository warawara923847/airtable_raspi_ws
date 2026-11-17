use std::path::{Path, PathBuf};

#[derive(Debug, Clone, Copy)]
pub enum Direction {
    Positive,
    Negative,
}

#[derive(Debug)]
pub struct DummyGpio {
    axis_name: String,
}

impl DummyGpio {
    pub fn new(axis_name: impl Into<String>) -> Self {
        Self {
            axis_name: axis_name.into(),
        }
    }

    pub fn set_direction(&mut self, direction: Direction) {
        match direction {
            Direction::Positive => {
                println!("[io] {} DIR = positive", self.axis_name);
            }
            Direction::Negative => {
                println!("[io] {} DIR = negative", self.axis_name);
            }
        }
    }

    pub fn pulse_steps(&mut self, steps: u64) {
        println!("[io] {} STEP x{}", self.axis_name, steps);
    }
}

/// One digital output line. `Cdev` owns a line of the Linux GPIO character device.
pub enum OutputLine {
    Dummy,
    Cdev {
        request: gpiocdev::Request,
        offset: u32,
    },
}

impl OutputLine {
    /// Request `offset` of `chip` as an output driven low. Fails if the line is in use.
    pub fn cdev(chip: &Path, offset: u32, consumer: &str) -> Result<Self, String> {
        let request = gpiocdev::Request::builder()
            .on_chip(chip)
            .with_consumer(consumer)
            .with_line(offset)
            .as_output(gpiocdev::line::Value::Inactive)
            .request()
            .map_err(|error| format!("{} line {offset}: {error}", chip.display()))?;
        Ok(Self::Cdev { request, offset })
    }

    pub fn set(&mut self, high: bool) -> Result<(), String> {
        match self {
            Self::Dummy => Ok(()),
            Self::Cdev { request, offset } => {
                let value = if high {
                    gpiocdev::line::Value::Active
                } else {
                    gpiocdev::line::Value::Inactive
                };
                request
                    .set_value(*offset, value)
                    .map_err(|error| format!("GPIO line {offset}: {error}"))
            }
        }
    }
}

/// Label of the 40-pin header's GPIO controller on Raspberry Pi 4 (BCM2711).
/// The Pi 5 header was "pinctrl-rp1" (docs/adr/0021).
pub const HEADER_CHIP_LABEL: &str = "pinctrl-bcm2711";

/// Find a GPIO chip by label. Chip numbers are not stable across kernels (the Pi 5
/// header has been gpiochip4 and gpiochip0), so the label identifies the header.
pub fn find_chip(label: &str) -> Result<PathBuf, String> {
    let chips = gpiocdev::chip::chips().map_err(|error| format!("GPIO chip scan: {error}"))?;
    let mut seen = Vec::new();
    for path in chips {
        // Chips of other controllers may be root-only; skip what cannot be read.
        let Ok(info) = gpiocdev::chip::Chip::from_path(&path).and_then(|chip| chip.info()) else {
            continue;
        };
        if info.label == label {
            return Ok(path);
        }
        seen.push(info.label);
    }
    Err(format!(
        "no readable GPIO chip labelled {label:?} (readable labels: {seen:?})"
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Drives a real line, so it only runs on request, on the Pi:
    /// `AIRTABLE_TEST_GPIO_LINE=19 cargo test -- --ignored real_gpio`
    #[test]
    #[ignore = "drives a real GPIO line"]
    fn real_gpio_line_follows_set_and_is_exclusive() {
        let offset: u32 = std::env::var("AIRTABLE_TEST_GPIO_LINE")
            .expect("set AIRTABLE_TEST_GPIO_LINE to a free BCM line")
            .parse()
            .unwrap();
        let chip = find_chip(HEADER_CHIP_LABEL).unwrap();
        let mut line = OutputLine::cdev(&chip, offset, "airtable_test").unwrap();
        let OutputLine::Cdev { request, .. } = &line else {
            unreachable!()
        };
        let level = |request: &gpiocdev::Request| request.value(offset).unwrap();
        assert_eq!(level(request), gpiocdev::line::Value::Inactive);
        // A second owner must be refused while the valve driver holds the line.
        assert!(OutputLine::cdev(&chip, offset, "airtable_test_2").is_err());
        line.set(true).unwrap();
        let OutputLine::Cdev { request, .. } = &line else {
            unreachable!()
        };
        assert_eq!(level(request), gpiocdev::line::Value::Active);
        line.set(false).unwrap();
        let OutputLine::Cdev { request, .. } = &line else {
            unreachable!()
        };
        assert_eq!(level(request), gpiocdev::line::Value::Inactive);
    }
}
