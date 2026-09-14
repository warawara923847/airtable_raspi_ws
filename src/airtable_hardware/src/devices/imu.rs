#[derive(Debug, Clone, Copy)]
pub struct ImuSample {
    pub angular_velocity_rad_s: [f64; 3],
    pub linear_acceleration_m_s2: [f64; 3],
}

#[derive(Debug)]
pub struct DummyImu;

impl DummyImu {
    pub fn new() -> Self {
        Self
    }

    pub fn read(&mut self) -> ImuSample {
        ImuSample {
            angular_velocity_rad_s: [0.0, 0.0, 0.0],
            linear_acceleration_m_s2: [0.0, 0.0, 9.80665],
        }
    }
}
