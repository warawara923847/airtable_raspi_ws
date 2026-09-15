#[derive(Debug, Clone, Copy)]
pub enum ThrusterMode {
    Off,
    Continuous,
    PulseUntil(f64),
}

#[derive(Debug)]
pub struct SimThruster {
    name: String,
    actuator_id: usize,
    thrust_n: f64,
    mode: ThrusterMode,
}

impl SimThruster {
    pub fn new(name: impl Into<String>, actuator_id: usize, thrust_n: f64) -> Self {
        Self {
            name: name.into(),
            actuator_id,
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
                if now >= deadline {
                    self.mode = ThrusterMode::Off;
                    false
                } else {
                    true
                }
            }
        };

        ctrl[self.actuator_id] = if enabled { self.thrust_n } else { 0.0 };
    }
}
