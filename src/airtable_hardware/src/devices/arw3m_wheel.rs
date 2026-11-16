//! ARW-3m as a reaction wheel device.
//!
//! One torque command plus telemetry is about 150 bytes on a 100 kHz bus and the wheel
//! is polled while busy, so a cycle takes well over 10 ms. The bus therefore runs on its
//! own thread; the hardware worker, which also times the thruster valves, only exchanges
//! the latest command and state through a mutex.
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::devices::reaction_wheel::WheelState;
use crate::drivers::arw3m::{Arw3m, RecoveryAction, describe_errors, recovery_action};
use crate::io::i2c::I2cBus;

#[derive(Debug, Clone)]
pub struct Arw3mConfig {
    pub name: String,
    pub max_torque_nm: f64,
    pub speed_limit_rad_s: f64,
    pub command_timeout: Duration,
    pub period: Duration,
}

impl Arw3mConfig {
    pub fn validate(&self) -> Result<(), String> {
        if self.name.is_empty() || self.command_timeout.is_zero() || self.period.is_zero() {
            return Err("RW name, command timeout and update period are required".into());
        }
        for value in [self.max_torque_nm, self.speed_limit_rad_s] {
            if !value.is_finite() || value <= 0.0 {
                return Err("RW torque and speed limits must be finite and positive".into());
            }
        }
        Ok(())
    }
}

struct Shared {
    command_nm: f64,
    last_command: Option<Instant>,
    state: WheelState,
}

/// Bus-side bookkeeping of the wheel thread.
struct Link {
    driver: Arw3m,
    failures: u32,
    error_status: u32,
}

impl Link {
    /// One control cycle: apply the command timeout, send the torque, read telemetry.
    fn cycle(&mut self, config: &Arw3mConfig, shared: &Mutex<Shared>, now: Instant) {
        let mut command = {
            let mut shared = shared.lock().unwrap();
            if shared
                .last_command
                .is_some_and(|last| now.duration_since(last) >= config.command_timeout)
            {
                shared.command_nm = 0.0;
                shared.last_command = None;
                eprintln!(
                    "[hardware] {}: RW command timed out; torque set to 0",
                    config.name
                );
            }
            shared.command_nm
        };
        // Errors that need a clear or reset (ICD actions B and C) are not recovered
        // automatically yet: hold zero torque until the status clears or the node restarts.
        if recovery_action(self.error_status) >= RecoveryAction::B {
            command = 0.0;
        }

        let result = self
            .driver
            .set_torque(command as f32, config.speed_limit_rad_s as f32)
            .and_then(|()| self.driver.telemetry());
        match result {
            Ok(telemetry) => {
                if self.failures > 0 {
                    eprintln!(
                        "[hardware] {}: ARW-3m link restored after {} failed cycles",
                        config.name, self.failures
                    );
                    self.failures = 0;
                }
                if telemetry.error_status != self.error_status {
                    self.error_status = telemetry.error_status;
                    eprintln!(
                        "[hardware] {}: ARW-3m error status 0x{:08x} {:?}, recommended action {:?}",
                        config.name,
                        telemetry.error_status,
                        describe_errors(telemetry.error_status),
                        recovery_action(telemetry.error_status)
                    );
                }
                // After a wheel-side restart the control loop is stopped; start it again.
                if telemetry.control_status == 0
                    && let Err(error) = self.driver.control_start()
                {
                    eprintln!(
                        "[hardware] {}: ARW-3m control start failed: {error}",
                        config.name
                    );
                }
                let mut shared = shared.lock().unwrap();
                shared.state.angular_velocity = f64::from(telemetry.rotation_speed_rad_s);
                shared.state.commanded_torque = command;
                shared.state.applied_torque = f64::from(telemetry.torque_nm);
            }
            Err(error) => {
                self.failures += 1;
                if self.failures == 1 || self.failures % 250 == 0 {
                    eprintln!(
                        "[hardware] {}: ARW-3m cycle failed ({} in a row): {error}",
                        config.name, self.failures
                    );
                }
                // Keep the last measured speed, but do not claim torque that was not confirmed.
                let mut shared = shared.lock().unwrap();
                shared.state.commanded_torque = command;
                shared.state.applied_torque = 0.0;
            }
        }
    }

    fn shutdown(&mut self, config: &Arw3mConfig) {
        let name = &config.name;
        let torque = self.driver.set_torque(0.0, config.speed_limit_rad_s as f32);
        let stop = self.driver.control_stop();
        match (torque, stop) {
            (_, Ok(())) => println!("[hardware] {name}: ARW-3m control stopped at shutdown"),
            (torque, Err(error)) => eprintln!(
                "[hardware] {name}: ARW-3m FAILED to stop at shutdown: {error} (zero torque: {torque:?})"
            ),
        }
    }
}

pub struct Arw3mWheel {
    config: Arw3mConfig,
    shared: Arc<Mutex<Shared>>,
    stopping: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
}

impl Arw3mWheel {
    /// Check that the wheel answers, start its control loop and the bus thread.
    pub fn new(config: Arw3mConfig, bus: Box<dyn I2cBus>) -> Result<Self, String> {
        config.validate()?;
        let mut driver = Arw3m::new(bus);
        let telemetry = driver
            .telemetry()
            .map_err(|error| format!("{}: ARW-3m does not answer: {error}", config.name))?;
        println!(
            "[hardware] {}: ARW-3m online: {:.2} rad/s, {:.2} V, error status 0x{:08x}, {} resets",
            config.name,
            telemetry.rotation_speed_rad_s,
            telemetry.motor_voltage_v,
            telemetry.error_status,
            telemetry.total_reset_count
        );
        driver
            .set_torque(0.0, config.speed_limit_rad_s as f32)
            .and_then(|()| driver.control_start())
            .map_err(|error| format!("{}: ARW-3m control start failed: {error}", config.name))?;

        let shared = Arc::new(Mutex::new(Shared {
            command_nm: 0.0,
            last_command: None,
            state: WheelState {
                name: config.name.clone(),
                angular_velocity: f64::from(telemetry.rotation_speed_rad_s),
                commanded_torque: 0.0,
                applied_torque: 0.0,
            },
        }));
        let stopping = Arc::new(AtomicBool::new(false));
        let mut link = Link {
            driver,
            failures: 0,
            // Start from 0 so that a non-zero initial status is logged by the first cycle.
            error_status: 0,
        };
        let thread = {
            let (config, shared, stopping) =
                (config.clone(), Arc::clone(&shared), Arc::clone(&stopping));
            std::thread::Builder::new()
                .name("arw3m".into())
                .spawn(move || {
                    let mut next = Instant::now();
                    while !stopping.load(Ordering::Relaxed) {
                        link.cycle(&config, &shared, Instant::now());
                        next += config.period;
                        let now = Instant::now();
                        if next > now {
                            std::thread::sleep(next - now);
                        } else {
                            // The bus took longer than one period; do not try to catch up.
                            next = now;
                        }
                    }
                    link.shutdown(&config);
                })
                .map_err(|error| format!("ARW-3m thread: {error}"))?
        };
        Ok(Self {
            config,
            shared,
            stopping,
            thread: Some(thread),
        })
    }

    pub fn name(&self) -> &str {
        &self.config.name
    }

    pub fn set_torque(&mut self, torque_nm: f64, now: Instant) -> Result<(), String> {
        if !torque_nm.is_finite() {
            return Err("non-finite torque".into());
        }
        let mut shared = self.shared.lock().unwrap();
        shared.command_nm = torque_nm.clamp(-self.config.max_torque_nm, self.config.max_torque_nm);
        shared.last_command = Some(now);
        Ok(())
    }

    /// Latest state measured by the bus thread.
    pub fn state(&self) -> WheelState {
        self.shared.lock().unwrap().state.clone()
    }
}

impl Drop for Arw3mWheel {
    fn drop(&mut self) {
        self.stopping.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            eprintln!("[hardware] {}: ARW-3m thread panicked", self.config.name);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::arw3m::sim::{SimBus, WheelModel};

    fn config() -> Arw3mConfig {
        Arw3mConfig {
            name: "rw_1".into(),
            max_torque_nm: 0.001,
            speed_limit_rad_s: 628.3,
            command_timeout: Duration::from_millis(500),
            period: Duration::from_millis(5),
        }
    }

    fn link() -> (Link, Arc<Mutex<WheelModel>>, Mutex<Shared>) {
        let (bus, model) = SimBus::new();
        let link = Link {
            driver: Arw3m::new(Box::new(bus)),
            failures: 0,
            error_status: 0,
        };
        let shared = Mutex::new(Shared {
            command_nm: 0.0,
            last_command: None,
            state: WheelState {
                name: "rw_1".into(),
                angular_velocity: 0.0,
                commanded_torque: 0.0,
                applied_torque: 0.0,
            },
        });
        (link, model, shared)
    }

    fn command(shared: &Mutex<Shared>, torque: f64, now: Instant) {
        let mut shared = shared.lock().unwrap();
        shared.command_nm = torque;
        shared.last_command = Some(now);
    }

    #[test]
    fn cycle_sends_the_command_reports_telemetry_and_times_out() {
        let (mut link, model, shared) = link();
        let now = Instant::now();
        model.lock().unwrap().running = true;
        model.lock().unwrap().speed_rad_s = 12.5;
        command(&shared, 0.0005, now);
        link.cycle(&config(), &shared, now + Duration::from_millis(20));
        assert_eq!(model.lock().unwrap().torque_nm, 0.0005);
        assert_eq!(model.lock().unwrap().speed_limit_rad_s, 628.3);
        {
            let state = &shared.lock().unwrap().state;
            assert_eq!(state.angular_velocity, 12.5);
            assert_eq!(state.commanded_torque, 0.0005);
            assert!((state.applied_torque - 0.0005).abs() < 1e-9);
        }
        link.cycle(&config(), &shared, now + Duration::from_millis(500));
        assert_eq!(model.lock().unwrap().torque_nm, 0.0);
        assert_eq!(shared.lock().unwrap().state.commanded_torque, 0.0);
    }

    #[test]
    fn serious_wheel_error_holds_zero_torque_until_it_clears() {
        let (mut link, model, shared) = link();
        let now = Instant::now();
        model.lock().unwrap().running = true;
        model.lock().unwrap().error_status = 1 << 17; // motor lock: action C
        command(&shared, 0.0005, now);
        link.cycle(&config(), &shared, now); // the error is learned from this telemetry
        link.cycle(&config(), &shared, now);
        assert_eq!(model.lock().unwrap().torque_nm, 0.0);
        assert_eq!(shared.lock().unwrap().state.commanded_torque, 0.0);
        // A warning-level error (action A) does not block the command.
        model.lock().unwrap().error_status = 1 << 9;
        link.cycle(&config(), &shared, now);
        link.cycle(&config(), &shared, now);
        assert_eq!(model.lock().unwrap().torque_nm, 0.0005);
    }

    #[test]
    fn lost_link_reports_no_torque_and_restarts_control_when_the_wheel_returns() {
        let (mut link, model, shared) = link();
        let now = Instant::now();
        model.lock().unwrap().running = true;
        command(&shared, 0.0005, now);
        link.cycle(&config(), &shared, now);
        model.lock().unwrap().absent = true;
        link.cycle(&config(), &shared, now);
        assert_eq!(link.failures, 1);
        assert_eq!(shared.lock().unwrap().state.applied_torque, 0.0);
        // The wheel comes back after a power cycle with its control loop stopped.
        *model.lock().unwrap() = WheelModel::default();
        link.cycle(&config(), &shared, now);
        assert_eq!(link.failures, 0);
        assert!(model.lock().unwrap().running);
    }

    #[test]
    fn wheel_starts_control_clamps_commands_and_stops_on_drop() {
        let (bus, model) = SimBus::new();
        let mut wheel = Arw3mWheel::new(config(), Box::new(bus)).unwrap();
        assert!(model.lock().unwrap().running);
        assert!(wheel.set_torque(f64::NAN, Instant::now()).is_err());
        wheel.set_torque(1.0, Instant::now()).unwrap();
        std::thread::sleep(Duration::from_millis(50));
        assert_eq!(model.lock().unwrap().torque_nm, 0.001);
        assert_eq!(wheel.state().commanded_torque, 0.001);
        drop(wheel);
        let model = model.lock().unwrap();
        assert!(!model.running);
        assert_eq!(model.torque_nm, 0.0);
    }

    #[test]
    fn absent_wheel_fails_at_startup() {
        let (bus, model) = SimBus::new();
        model.lock().unwrap().absent = true;
        let error = Arw3mWheel::new(config(), Box::new(bus)).err().unwrap();
        assert!(error.contains("does not answer"), "{error}");
    }
}
