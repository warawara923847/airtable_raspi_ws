#[derive(Debug, Clone, Copy)]
pub struct PlanarState {
    pub time: f64,

    pub x: f64,
    pub y: f64,
    pub yaw: f64,

    pub vx: f64,
    pub vy: f64,
    pub wz: f64,
}
