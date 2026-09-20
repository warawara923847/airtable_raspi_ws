#[derive(Debug)]
pub struct SimReactionWheel {
    name: String,
    control_id: usize,

    inertia: f64,
    angular_velocity: f64,

    commanded_torque: f64,
    applied_torque: f64,

    max_torque: f64,
    max_angular_velocity: f64,
}

#[derive(Debug, Clone)]
pub struct ReactionWheelStateData {
    pub name: String,
    pub angular_velocity: f64,
    pub commanded_torque: f64,
    pub applied_torque: f64,
}

impl SimReactionWheel {
    pub fn new(
        name: impl Into<String>,
        control_id: usize,
        inertia: f64,
        max_torque: f64,
        max_angular_velocity: f64,
    ) -> Self {
        Self {
            name: name.into(),
            control_id,

            inertia,
            angular_velocity: 0.0,

            commanded_torque: 0.0,
            applied_torque: 0.0,

            max_torque,
            max_angular_velocity,
        }
    }

    pub fn state(&self) -> ReactionWheelStateData {
        ReactionWheelStateData {
            name: self.name.clone(),
            angular_velocity: self.angular_velocity,
            commanded_torque: self.commanded_torque,
            applied_torque: self.applied_torque,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn control_id(&self) -> usize {
        self.control_id
    }

    pub fn angular_velocity(&self) -> f64 {
        self.angular_velocity
    }

    pub fn commanded_torque(&self) -> f64 {
        self.commanded_torque
    }

    pub fn applied_torque(&self) -> f64 {
        self.applied_torque
    }

    pub fn set_torque(&mut self, torque_nm: f64) {
        self.commanded_torque = torque_nm;
    }

    pub fn stop(&mut self) {
        self.commanded_torque = 0.0;
    }

    pub fn update(&mut self, curl: &mut [f64], dt: f64) {
        let mut torque = self
            .commanded_torque
            .clamp(-self.max_torque, self.max_torque);

        // Positive speed limit
        if self.angular_velocity >= self.max_angular_velocity && torque > 0.0 {
            torque = 0.0;
        }

        // Negative speed limit
        if self.angular_velocity <= -self.max_angular_velocity && torque < 0.0 {
            torque = 0.0;
        }

        // Wheel dynamics:
        //
        // tau = I * alpha
        // alpha = tau / I
        let omega_next = self.angular_velocity + torque / self.inertia * dt;

        // Numerical safety
        let omega_next_clamped =
            omega_next.clamp(-self.max_angular_velocity, self.max_angular_velocity);

        let applied_torque = (omega_next_clamped - self.angular_velocity) * self.inertia / dt;

        self.angular_velocity = omega_next_clamped;
        self.applied_torque = applied_torque;

        curl[self.control_id] = applied_torque;
    }
}
