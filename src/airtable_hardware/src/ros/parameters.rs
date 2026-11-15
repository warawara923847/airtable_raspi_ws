use std::sync::Arc;
use std::time::{Duration, Instant};

use rclrs::{Node, ReadOnlyParameter};

use crate::devices::reaction_wheel::{DebugReactionWheel, DebugWheelConfig};
use crate::drivers::servo::{ServoCalibration, ServoDriver};
use crate::io::servo_pwm::ServoPwm;

// Keep the handles alive so rclrs keeps these parameters declared.
pub struct DebugWheelParameters {
    enabled: ReadOnlyParameter<bool>,
    backend: ReadOnlyParameter<Arc<str>>,
    name: ReadOnlyParameter<Arc<str>>,
    pwm_path: ReadOnlyParameter<Arc<str>>,
    min_us: ReadOnlyParameter<i64>,
    neutral_us: ReadOnlyParameter<i64>,
    max_us: ReadOnlyParameter<i64>,
    max_torque: ReadOnlyParameter<f64>,
    inertia: ReadOnlyParameter<f64>,
    max_speed: ReadOnlyParameter<f64>,
    timeout_ms: ReadOnlyParameter<i64>,
}

impl DebugWheelParameters {
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
        })
    }

    pub fn create_wheel(&self) -> Result<Option<DebugReactionWheel>, Box<dyn std::error::Error>> {
        if !self.enabled.get() {
            return Ok(None);
        }
        let timeout = u64::try_from(self.timeout_ms.get())?;
        let config = DebugWheelConfig {
            name: self.name.get().to_string(),
            max_torque_nm: self.max_torque.get(),
            inertia_kg_m2: self.inertia.get(),
            max_speed_rad_s: self.max_speed.get(),
            command_timeout: Duration::from_millis(timeout),
        };
        config.validate().map_err(std::io::Error::other)?;
        let calibration = ServoCalibration {
            min_us: u16::try_from(self.min_us.get())?,
            neutral_us: u16::try_from(self.neutral_us.get())?,
            max_us: u16::try_from(self.max_us.get())?,
        };
        calibration.validate().map_err(std::io::Error::other)?;
        let pwm = match self.backend.get().as_ref() {
            "dummy" => ServoPwm::dummy(),
            "sysfs" => {
                let path = self.pwm_path.get();
                if path.is_empty() {
                    return Err(
                        "rw.pwm_path is required for sysfs (exported PWM channel directory)".into(),
                    );
                }
                ServoPwm::linux(path.as_ref(), calibration.neutral_us)
                    .map_err(std::io::Error::other)?
            }
            backend => {
                return Err(format!("unknown rw.backend: {backend}; use dummy or sysfs").into());
            }
        };
        let driver = ServoDriver::new(pwm, calibration).map_err(std::io::Error::other)?;
        Ok(Some(
            DebugReactionWheel::new(config, driver, Instant::now())
                .map_err(std::io::Error::other)?,
        ))
    }
}
