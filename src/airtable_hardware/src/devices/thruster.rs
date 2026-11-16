use std::time::{Duration, Instant};

use crate::drivers::valve::ValveDriver;

#[derive(Debug, Clone, Copy)]
pub enum ThrusterMode {
    Off,
    Continuous,
    PulseUntil(Instant),
}

pub struct Thruster {
    name: String,
    on_thrust_n: f64,
    mode: ThrusterMode,
    driver: ValveDriver,
    max_pulse: Duration,
    // Per-pulse logging is for the dummy backend; real valves switch up to 100 Hz.
    verbose: bool,
    warned_clamp: bool,
    // How late pulses were closed relative to their deadline; reported at shutdown.
    pulses: u64,
    late_sum: Duration,
    late_max: Duration,
}

impl Thruster {
    pub fn new(
        name: impl Into<String>,
        on_thrust_n: f64,
        driver: ValveDriver,
        max_pulse: Duration,
        verbose: bool,
    ) -> Self {
        Self {
            driver,
            name: name.into(),
            on_thrust_n,
            mode: ThrusterMode::Off,
            max_pulse,
            verbose,
            warned_clamp: false,
            pulses: 0,
            late_sum: Duration::ZERO,
            late_max: Duration::ZERO,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// Open the valve until `now + duration`. The deadline is kept on this side of the
    /// network, so a lost or late follow-up command can never leave the valve open.
    pub fn fire_for(&mut self, duration: Duration, now: Instant) -> Result<(), String> {
        if duration.is_zero() {
            return Ok(());
        }
        let duration = if duration > self.max_pulse {
            if !self.warned_clamp {
                eprintln!(
                    "[thruster] {}: pulse {:?} limited to {:?} (further limits not logged)",
                    self.name, duration, self.max_pulse
                );
                self.warned_clamp = true;
            }
            self.max_pulse
        } else {
            duration
        };

        self.driver.set_enabled(true)?;
        self.mode = ThrusterMode::PulseUntil(now + duration);
        if self.verbose {
            println!("[thruster] {} pulse for {:?}", self.name, duration);
        }
        Ok(())
    }

    pub fn turn_on(&mut self) -> Result<(), String> {
        self.driver.set_enabled(true)?;
        self.mode = ThrusterMode::Continuous;
        if self.verbose {
            println!("[thruster] {} continuous ON", self.name);
        }
        Ok(())
    }

    pub fn turn_off(&mut self) -> Result<(), String> {
        // Mode changes only after the valve write succeeds, so `update` retries a failed close.
        self.driver.set_enabled(false)?;
        self.mode = ThrusterMode::Off;
        if self.verbose {
            println!("[thruster] {} OFF", self.name);
        }
        Ok(())
    }

    pub fn update(&mut self, now: Instant) -> Result<(), String> {
        if let ThrusterMode::PulseUntil(deadline) = self.mode
            && now >= deadline
        {
            self.turn_off()?;
            let late = now - deadline;
            self.pulses += 1;
            self.late_sum += late;
            self.late_max = self.late_max.max(late);
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

impl Drop for Thruster {
    fn drop(&mut self) {
        if self.pulses > 0 {
            println!(
                "[thruster] {}: {} pulses, closed late by mean {:.2} ms, max {:.2} ms",
                self.name,
                self.pulses,
                self.late_sum.as_secs_f64() * 1e3 / self.pulses as f64,
                self.late_max.as_secs_f64() * 1e3
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::gpio::OutputLine;

    fn thruster(max_pulse_ms: u64) -> Thruster {
        Thruster::new(
            "thruster_1",
            0.5,
            ValveDriver::new("thruster_1", OutputLine::Dummy),
            Duration::from_millis(max_pulse_ms),
            false,
        )
    }

    #[test]
    fn pulse_closes_at_its_deadline_without_further_commands() {
        let now = Instant::now();
        let mut thruster = thruster(1000);
        thruster.fire_for(Duration::from_millis(10), now).unwrap();
        thruster.update(now + Duration::from_millis(9)).unwrap();
        assert!(matches!(thruster.mode(), ThrusterMode::PulseUntil(_)));
        thruster.update(now + Duration::from_millis(10)).unwrap();
        assert!(matches!(thruster.mode(), ThrusterMode::Off));
    }

    #[test]
    fn pulse_is_limited_to_the_configured_maximum() {
        let now = Instant::now();
        let mut thruster = thruster(100);
        thruster.fire_for(Duration::from_secs(3600), now).unwrap();
        thruster.update(now + Duration::from_millis(99)).unwrap();
        assert!(matches!(thruster.mode(), ThrusterMode::PulseUntil(_)));
        thruster.update(now + Duration::from_millis(100)).unwrap();
        assert!(matches!(thruster.mode(), ThrusterMode::Off));
    }
}
