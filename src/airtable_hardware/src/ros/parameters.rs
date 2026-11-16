use std::sync::Arc;
use std::time::{Duration, Instant};

use rclrs::{Node, ReadOnlyParameter};

use crate::devices::arw3m_wheel::{Arw3mConfig, Arw3mWheel};
use crate::devices::continuous_servo_wheel::{ContinuousServoConfig, ContinuousServoWheel};
use crate::devices::reaction_wheel::{DebugReactionWheel, DebugWheelConfig};
use crate::devices::thruster::Thruster;
use crate::drivers::arw3m::DEFAULT_ADDRESS;
use crate::drivers::servo::{ServoCalibration, ServoDriver};
use crate::drivers::valve::ValveDriver;
use crate::hardware::ReactionWheel;
use crate::io::gpio::{OutputLine, find_chip};
use crate::io::i2c::LinuxI2c;
use crate::io::servo_pwm::ServoPwm;

pub const THRUSTER_NAMES: [&str; 4] = ["thruster_1", "thruster_2", "thruster_3", "thruster_4"];
// Label of the 40-pin header's GPIO controller on Raspberry Pi 5.
const PI5_HEADER_CHIP_LABEL: &str = "pinctrl-rp1";

pub struct ThrusterParameters {
    backend: ReadOnlyParameter<Arc<str>>,
    gpio_chip: ReadOnlyParameter<Arc<str>>,
    gpio_lines: ReadOnlyParameter<Arc<[i64]>>,
    max_pulse_ms: ReadOnlyParameter<i64>,
}

impl ThrusterParameters {
    pub fn declare(node: &Node) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            backend: node
                .declare_parameter("thrusters.backend")
                .default(Arc::<str>::from("dummy"))
                .read_only()?,
            // Empty: find the Pi 5 header by label. Otherwise a path such as /dev/gpiochip4.
            gpio_chip: node
                .declare_parameter("thrusters.gpio_chip")
                .default(Arc::<str>::from(""))
                .read_only()?,
            // BCM numbers of thruster_1..4, as in the C sample's VALVE1..4.
            gpio_lines: node
                .declare_parameter("thrusters.gpio_lines")
                .default(Arc::<[i64]>::from([19_i64, 26, 20, 21]))
                .read_only()?,
            max_pulse_ms: node
                .declare_parameter("thrusters.max_pulse_ms")
                .default(1000_i64)
                .read_only()?,
        })
    }

    pub fn create_thrusters(&self) -> Result<Vec<Thruster>, Box<dyn std::error::Error>> {
        let max_pulse = Duration::from_millis(u64::try_from(self.max_pulse_ms.get())?);
        if max_pulse.is_zero() {
            return Err("thrusters.max_pulse_ms must be positive".into());
        }
        let backend = self.backend.get();
        let lines: Vec<OutputLine> = match backend.as_ref() {
            "dummy" => THRUSTER_NAMES.iter().map(|_| OutputLine::Dummy).collect(),
            "gpio" => {
                let offsets = self
                    .gpio_lines
                    .get()
                    .iter()
                    .map(|line| u32::try_from(*line))
                    .collect::<Result<Vec<_>, _>>()?;
                if offsets.len() != THRUSTER_NAMES.len()
                    || (1..offsets.len()).any(|i| offsets[..i].contains(&offsets[i]))
                {
                    return Err(format!(
                        "thrusters.gpio_lines needs {} distinct lines, got {offsets:?}",
                        THRUSTER_NAMES.len()
                    )
                    .into());
                }
                let chip = self.gpio_chip.get();
                let chip = if chip.is_empty() {
                    find_chip(PI5_HEADER_CHIP_LABEL).map_err(std::io::Error::other)?
                } else {
                    chip.as_ref().into()
                };
                println!(
                    "[hardware] thruster valves on {} lines {offsets:?}",
                    chip.display()
                );
                // If a later line fails, the lines already requested are released here.
                offsets
                    .iter()
                    .map(|offset| OutputLine::cdev(&chip, *offset, "airtable_valve"))
                    .collect::<Result<_, _>>()
                    .map_err(std::io::Error::other)?
            }
            backend => {
                return Err(
                    format!("unknown thrusters.backend: {backend}; use dummy or gpio").into(),
                );
            }
        };
        Ok(THRUSTER_NAMES
            .iter()
            .zip(lines)
            .map(|(name, line)| {
                Thruster::new(
                    *name,
                    0.5,
                    ValveDriver::new(*name, line),
                    max_pulse,
                    backend.as_ref() == "dummy",
                )
            })
            .collect())
    }
}

// Keep the handles alive so rclrs keeps these parameters declared.
pub struct WheelParameters {
    enabled: ReadOnlyParameter<bool>,
    backend: ReadOnlyParameter<Arc<str>>,
    name: ReadOnlyParameter<Arc<str>>,
    pwm_path: ReadOnlyParameter<Arc<str>>,
    min_us: ReadOnlyParameter<i64>,
    neutral_us: ReadOnlyParameter<i64>,
    max_us: ReadOnlyParameter<i64>,
    deadband_us: ReadOnlyParameter<i64>,
    reversed: ReadOnlyParameter<bool>,
    max_torque: ReadOnlyParameter<f64>,
    inertia: ReadOnlyParameter<f64>,
    max_speed: ReadOnlyParameter<f64>,
    timeout_ms: ReadOnlyParameter<i64>,
    i2c_bus: ReadOnlyParameter<Arc<str>>,
    i2c_address: ReadOnlyParameter<i64>,
    speed_limit: ReadOnlyParameter<f64>,
    update_period_ms: ReadOnlyParameter<i64>,
}

impl WheelParameters {
    pub fn declare(node: &Node) -> Result<Self, Box<dyn std::error::Error>> {
        Ok(Self {
            enabled: node
                .declare_parameter("rw.enabled")
                .default(false)
                .read_only()?,
            backend: node
                .declare_parameter("rw.backend")
                .default(Arc::<str>::from("dummy"))
                .read_only()?,
            name: node
                .declare_parameter("rw.name")
                .default(Arc::<str>::from("rw_1"))
                .read_only()?,
            pwm_path: node
                .declare_parameter("rw.pwm_path")
                .default(Arc::<str>::from(""))
                .read_only()?,
            min_us: node
                .declare_parameter("rw.min_pulse_us")
                .default(1300_i64)
                .read_only()?,
            neutral_us: node
                .declare_parameter("rw.neutral_pulse_us")
                .default(1500_i64)
                .read_only()?,
            max_us: node
                .declare_parameter("rw.max_pulse_us")
                .default(1700_i64)
                .read_only()?,
            // continuous_servo: half-width of the band around the stop pulse that the servo
            // ignores, and flipping the pulse side to match the mounting.
            deadband_us: node
                .declare_parameter("rw.deadband_us")
                .default(0_i64)
                .read_only()?,
            reversed: node
                .declare_parameter("rw.reversed")
                .default(false)
                .read_only()?,
            max_torque: node
                .declare_parameter("rw.max_torque_nm")
                .default(0.01)
                .read_only()?,
            inertia: node
                .declare_parameter("rw.virtual_inertia_kg_m2")
                .default(0.001)
                .read_only()?,
            max_speed: node
                .declare_parameter("rw.virtual_max_speed_rad_s")
                .default(100.0)
                .read_only()?,
            timeout_ms: node
                .declare_parameter("rw.command_timeout_ms")
                .default(500_i64)
                .read_only()?,
            // The parameters below are used by the arw3m backend only.
            i2c_bus: node
                .declare_parameter("rw.i2c_bus")
                .default(Arc::<str>::from("/dev/i2c-1"))
                .read_only()?,
            i2c_address: node
                .declare_parameter("rw.i2c_address")
                .default(i64::from(DEFAULT_ADDRESS))
                .read_only()?,
            speed_limit: node
                .declare_parameter("rw.speed_limit_rad_s")
                .default(628.3)
                .read_only()?,
            update_period_ms: node
                .declare_parameter("rw.update_period_ms")
                .default(20_i64)
                .read_only()?,
        })
    }

    pub fn create_wheel(&self) -> Result<Option<ReactionWheel>, Box<dyn std::error::Error>> {
        if !self.enabled.get() {
            return Ok(None);
        }
        let timeout = u64::try_from(self.timeout_ms.get())?;
        if self.backend.get().as_ref() == "arw3m" {
            let config = Arw3mConfig {
                name: self.name.get().to_string(),
                max_torque_nm: self.max_torque.get(),
                speed_limit_rad_s: self.speed_limit.get(),
                command_timeout: Duration::from_millis(timeout),
                period: Duration::from_millis(u64::try_from(self.update_period_ms.get())?),
            };
            let bus = LinuxI2c::open(
                self.i2c_bus.get().as_ref(),
                u8::try_from(self.i2c_address.get())?,
            )
            .map_err(std::io::Error::other)?;
            return Ok(Some(ReactionWheel::Arw3m(
                Arw3mWheel::new(config, Box::new(bus)).map_err(std::io::Error::other)?,
            )));
        }
        let calibration = ServoCalibration {
            min_us: u16::try_from(self.min_us.get())?,
            neutral_us: u16::try_from(self.neutral_us.get())?,
            max_us: u16::try_from(self.max_us.get())?,
            deadband_us: u16::try_from(self.deadband_us.get())?,
        };
        calibration.validate().map_err(std::io::Error::other)?;
        let backend = self.backend.get();
        let path = self.pwm_path.get();
        // Validate the device configuration before taking over the PWM channel.
        if backend.as_ref() == "continuous_servo" {
            let config = ContinuousServoConfig {
                name: self.name.get().to_string(),
                reversed: self.reversed.get(),
                command_timeout: Duration::from_millis(timeout),
            };
            config.validate().map_err(std::io::Error::other)?;
            let pwm = if path.is_empty() {
                eprintln!("[hardware] rw.pwm_path is empty: continuous servo uses dummy PWM");
                ServoPwm::dummy()
            } else {
                ServoPwm::linux(path.as_ref(), calibration.neutral_us)
                    .map_err(std::io::Error::other)?
            };
            let driver = ServoDriver::new(pwm, calibration).map_err(std::io::Error::other)?;
            return Ok(Some(ReactionWheel::ContinuousServo(
                ContinuousServoWheel::new(config, driver).map_err(std::io::Error::other)?,
            )));
        }
        let config = DebugWheelConfig {
            name: self.name.get().to_string(),
            max_torque_nm: self.max_torque.get(),
            inertia_kg_m2: self.inertia.get(),
            max_speed_rad_s: self.max_speed.get(),
            command_timeout: Duration::from_millis(timeout),
        };
        config.validate().map_err(std::io::Error::other)?;
        let pwm = match backend.as_ref() {
            "dummy" => ServoPwm::dummy(),
            "sysfs" => {
                if path.is_empty() {
                    return Err(
                        "rw.pwm_path is required for sysfs (exported PWM channel directory)".into(),
                    );
                }
                ServoPwm::linux(path.as_ref(), calibration.neutral_us)
                    .map_err(std::io::Error::other)?
            }
            backend => {
                return Err(format!(
                    "unknown rw.backend: {backend}; use dummy, sysfs, continuous_servo or arw3m"
                )
                .into());
            }
        };
        let driver = ServoDriver::new(pwm, calibration).map_err(std::io::Error::other)?;
        Ok(Some(ReactionWheel::Debug(
            DebugReactionWheel::new(config, driver, Instant::now())
                .map_err(std::io::Error::other)?,
        )))
    }
}
