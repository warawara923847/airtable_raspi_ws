use crate::drivers::esc::EscDriver;
use std::time::{Duration, Instant};

use crate::drivers::valve::ValveDriver;

#[derive(Debug, Clone, Copy)]
pub enum ThrusterMode {
    Off,
    Continuous,
    PulseUntil(Instant),
}

#[derive(Debug)]
pub struct Thruster {
    name: String,
    on_thrust_n: f64,
    mode: ThrusterMode,
    driver: ValveDriver,
}

impl Thruster {
    pub fn new(name: impl Into<String>, on_thrust_n: f64) -> Self {
        let name = name.into();
        Self {
            driver: ValveDriver::new(name.clone()),
            name,
            on_thrust_n,
            mode: ThrusterMode::Off,
        }
    }

    pub fn fire_for(&mut self, duration: Duration) -> Result<(), String> {
        if duration.is_zero() {
            return Ok(());
        }

        self.driver.set_enabled(true)?;

        self.mode = ThrusterMode::PulseUntil(Instant::now() + duration);

        println!("[thruster] {} pulse for {:?}", self.name, duration);

        Ok(())
    }

    pub fn turn_on(&mut self) -> Result<(), String> {
        self.driver.set_enabled(true)?;
        self.mode = ThrusterMode::Continuous;

        println!("[thruster] {} continuous ON", self.name);

        Ok(())
    }

    pub fn turn_off(&mut self) -> Result<(), String> {
        self.driver.set_enabled(false)?;
        self.mode = ThrusterMode::Off;

        println!("[thruster] {} OFF", self.name);

        Ok(())
    }

    pub fn update(&mut self, now: Instant) -> Result<(), String> {
        // enumのmatch構文, PulseUntilの場合はdeadlineを過ぎているかを判定
        let should_stop = match self.mode {
            ThrusterMode::PulseUntil(deadline) => now >= deadline,

            _ => false,
        };

        if should_stop {
            self.turn_off()?;
        }

        Ok(())
    }

    pub fn mode(&self) -> ThrusterMode {
        self.mode
    }

    pub fn on_thrust_n(&self) -> f64 {
        self.on_thrust_n
    }
}
