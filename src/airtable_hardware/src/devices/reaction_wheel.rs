//! Debug RW: torque is displayed as SG90 position; feedback is a virtual wheel.
use std::time::{Duration, Instant};

use crate::drivers::servo::ServoDriver;

#[derive(Debug, Clone)]
pub struct DebugWheelConfig {
    pub name: String,
    pub max_torque_nm: f64,
    pub inertia_kg_m2: f64,
    pub max_speed_rad_s: f64,
    pub command_timeout: Duration,
}

impl DebugWheelConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.name.is_empty() || self.command_timeout.is_zero() {
            return Err("RW name and positive command timeout are required".into());
        }
        for value in [self.max_torque_nm, self.inertia_kg_m2, self.max_speed_rad_s] {
            if !value.is_finite() || value <= 0.0 {
                return Err(
                    "RW torque, virtual inertia and speed limits must be finite and positive"
                        .into(),
                );
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct WheelState {
    pub name: String,
    pub angular_velocity: f64,
    pub commanded_torque: f64,
    pub applied_torque: f64,
}

pub struct DebugReactionWheel {
    config: DebugWheelConfig,
    driver: ServoDriver,
    state: WheelState,
    last_command: Option<Instant>,
    last_update: Instant,
    faulted: bool,
}

impl DebugReactionWheel {
    pub fn new(
        config: DebugWheelConfig,
        driver: ServoDriver,
        now: Instant,
    ) -> Result<Self, String> {
        config.validate()?;
        eprintln!(
            "[hardware] {}: DEBUG SG90; RW feedback is VIRTUAL, not measured",
            config.name
        );
        let state = WheelState {
            name: config.name.clone(),
            angular_velocity: 0.0,
            commanded_torque: 0.0,
            applied_torque: 0.0,
        };
        Ok(Self {
            config,
            driver,
            state,
            last_command: None,
            last_update: now,
            faulted: false,
        })
    }

    pub fn name(&self) -> &str {
        &self.config.name
    }

    pub fn set_torque(&mut self, torque_nm: f64, now: Instant) -> Result<(), String> {
        if self.faulted || !torque_nm.is_finite() {
            return Err("faulted RW output or non-finite torque".into());
        }
        self.state.commanded_torque =
            torque_nm.clamp(-self.config.max_torque_nm, self.config.max_torque_nm);
        self.last_command = Some(now);
        Ok(())
    }

    pub fn update(&mut self, now: Instant) -> Result<WheelState, String> {
        let dt = now.duration_since(self.last_update).as_secs_f64().min(0.1);
        self.last_update = now;
        if self
            .last_command
            .is_some_and(|last| now.duration_since(last) >= self.config.command_timeout)
        {
            self.state.commanded_torque = 0.0;
            self.last_command = None;
            eprintln!("[hardware] RW command timed out; returning SG90 to neutral");
        }
        if self.faulted {
            return Ok(self.state.clone());
        }
        let previous_speed = self.state.angular_velocity;
        let next_speed = (previous_speed
            + self.state.commanded_torque / self.config.inertia_kg_m2 * dt)
            .clamp(-self.config.max_speed_rad_s, self.config.max_speed_rad_s);
        let applied = if dt > 0.0 {
            (next_speed - previous_speed) * self.config.inertia_kg_m2 / dt
        } else {
            0.0
        };
        // Display the received torque, including commands limited by the virtual speed cap.
        if let Err(error) = self
            .driver
            .set_normalized(self.state.commanded_torque / self.config.max_torque_nm)
        {
            self.faulted = true;
            self.state.commanded_torque = 0.0;
            self.state.applied_torque = 0.0;
            let disable_result = self.driver.disable();
            return Err(format!(
                "SG90 PWM fault: {error}; disable: {disable_result:?}; restart required"
            ));
        }
        self.state.angular_velocity = next_speed;
        self.state.applied_torque = applied;
        Ok(self.state.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::servo::ServoCalibration;
    use crate::io::servo_pwm::ServoPwm;

    fn wheel(now: Instant) -> DebugReactionWheel {
        DebugReactionWheel::new(
            DebugWheelConfig {
                name: "rw_1".into(),
                max_torque_nm: 0.01,
                inertia_kg_m2: 0.001,
                max_speed_rad_s: 100.0,
                command_timeout: Duration::from_millis(500),
            },
            ServoDriver::new(
                ServoPwm::dummy(),
                ServoCalibration {
                    min_us: 1300,
                    neutral_us: 1500,
                    max_us: 1700,
                },
            )
            .unwrap(),
            now,
        )
        .unwrap()
    }

    #[test]
    fn clamps_commands_rejects_nan_and_times_out() {
        let now = Instant::now();
        let mut wheel = wheel(now);
        assert!(wheel.set_torque(f64::NAN, now).is_err());
        assert!(wheel.set_torque(f64::INFINITY, now).is_err());
        wheel.set_torque(1.0, now).unwrap();
        let state = wheel.update(now + Duration::from_millis(20)).unwrap();
        assert!((state.angular_velocity - 0.2).abs() < 1e-9);
        assert_eq!(state.commanded_torque, 0.01);
        let state = wheel.update(now + Duration::from_millis(500)).unwrap();
        assert_eq!(state.commanded_torque, 0.0);
        assert_eq!(state.applied_torque, 0.0);
        assert!((state.angular_velocity - 0.2).abs() < 1e-9);
    }

    #[test]
    fn virtual_feedback_converges_and_reverses_with_debug_controller_gain() {
        let start = Instant::now();
        let mut wheel = wheel(start);
        let mut speed = 0.0;
        for i in 1..=2000 {
            let target = if i <= 1000 { 5.0 } else { -5.0 };
            let now = start + Duration::from_millis(i * 20);
            wheel.set_torque(0.02 * (target - speed), now).unwrap();
            speed = wheel.update(now).unwrap().angular_velocity;
            assert!(speed.abs() <= 100.0);
            if i == 1000 || i == 2000 {
                assert!((speed - target).abs() < 1e-6);
            }
        }
    }

    #[test]
    fn virtual_speed_limit_still_allows_braking() {
        let start = Instant::now();
        let mut wheel = wheel(start);
        for i in 1..=1100 {
            let now = start + Duration::from_millis(i * 20);
            wheel.set_torque(0.01, now).unwrap();
            wheel.update(now).unwrap();
        }
        let now = start + Duration::from_millis(22020);
        let state = wheel.update(now).unwrap();
        assert_eq!(state.angular_velocity, 100.0);
        assert_eq!(state.applied_torque, 0.0);
        wheel.set_torque(-0.01, now).unwrap();
        assert!(
            wheel
                .update(now + Duration::from_millis(20))
                .unwrap()
                .angular_velocity
                < 100.0
        );
    }
}
