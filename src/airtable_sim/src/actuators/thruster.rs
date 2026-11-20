/// Far below any timestep, far above the floating-point drift of the accumulated time.
const DEADLINE_TOLERANCE_S: f64 = 1e-6;

#[derive(Debug, Clone, Copy)]
pub enum ThrusterMode {
    Off,
    Continuous,
    PulseUntil(f64),
}

#[derive(Debug)]
pub struct SimThruster {
    name: String,
    control_id: usize,
    thrust_n: f64,
    mode: ThrusterMode,
}

impl SimThruster {
    pub fn new(name: impl Into<String>, control_id: usize, thrust_n: f64) -> Self {
        Self {
            name: name.into(),
            control_id,
            thrust_n,
            mode: ThrusterMode::Off,
        }
    }

    pub fn fire_for(&mut self, now: f64, duration_s: f64) {
        self.mode = ThrusterMode::PulseUntil(now + duration_s);
    }

    pub fn update(&mut self, now: f64, ctrl: &mut [f64]) {
        let enabled = match self.mode {
            ThrusterMode::Off => false,
            ThrusterMode::Continuous => true,
            ThrusterMode::PulseUntil(deadline) => {
                // MuJoCo time is a running sum of timesteps. Past about 2 s of sim time the
                // step that should land on the deadline is often a few ulp short, which kept
                // half of the pulses on for one extra step (12 ms instead of 10 ms).
                if now + DEADLINE_TOLERANCE_S >= deadline {
                    self.mode = ThrusterMode::Off;
                    false
                } else {
                    true
                }
            }
        };

        ctrl[self.control_id] = if enabled { self.thrust_n } else { 0.0 };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pulse_lasts_exactly_its_duration_at_any_sim_time() {
        // Advance time by repeated addition like mj_step and fire at every step.
        let dt = 0.002;
        let mut start = 0.0;
        for _ in 0..20_000 {
            for (duration, steps) in [(0.01, 5), (0.02, 10), (0.03, 15), (0.05, 25)] {
                let mut thruster = SimThruster::new("t", 0, 0.5);
                let mut ctrl = [0.0];
                thruster.fire_for(start, duration);
                let (mut now, mut on) = (start, 0);
                for _ in 0..40 {
                    thruster.update(now, &mut ctrl);
                    on += usize::from(ctrl[0] > 0.0);
                    now += dt;
                }
                assert_eq!(on, steps, "pulse of {duration} s fired at {start} s");
            }
            start += dt;
        }
    }
}
