use crate::drivers::stepper::StepperDriver;

#[derive(Debug)]
pub struct XyTable {
    x_m: f64,
    y_m: f64,
    x_min_m: f64,
    x_max_m: f64,
    y_min_m: f64,
    y_max_m: f64,

    x_m_per_revolution: f64,
    y_m_per_revolution: f64,
    x_stepper: StepperDriver,
    y_stepper: StepperDriver,
}

impl XyTable {
    pub fn new_dummy() -> Self {
        Self {
            x_m: 0.0,
            y_m: 0.0,
            x_min_m: -0.1,
            x_max_m: 0.1,
            y_min_m: -0.1,
            y_max_m: 0.1,

            x_m_per_revolution: 0.005,
            y_m_per_revolution: 0.005,

            x_stepper: StepperDriver::new("X", 200, 16), // 200 steps/rev, 16 microsteps
            y_stepper: StepperDriver::new("Y", 200, 16), // 200 steps/rev, 16 microsteps
        }
    }

    pub fn move_to(&mut self, target_x_m: f64, target_y_m: f64) -> Result<(), String> {
        if !(self.x_min_m..=self.x_max_m).contains(&target_x_m) {
            return Err(format!("x out of range: {target_x_m}"));
        }

        if !(self.y_min_m..=self.y_max_m).contains(&target_y_m) {
            return Err(format!("y out of range: {target_y_m}"));
        }

        let delta_x_m = target_x_m - self.x_m;
        let delta_y_m = target_y_m - self.y_m;

        let x_revolutions = delta_x_m / self.x_m_per_revolution;
        let y_revolutions = delta_y_m / self.y_m_per_revolution;

        println!(
            "[device] XY target: x={:.4} m, y={:.4} m",
            target_x_m, target_y_m
        );

        println!(
            "[device] X delta: {:.4} m -> {:.4} rev",
            delta_x_m, x_revolutions
        );

        println!(
            "[device] Y delta: {:.4} m -> {:.4} rev",
            delta_y_m, y_revolutions
        );

        self.x_stepper.move_revolutions(x_revolutions)?;
        self.y_stepper.move_revolutions(y_revolutions)?;

        self.x_m = target_x_m;
        self.y_m = target_y_m;

        Ok(())
    }

    pub fn position(&self) -> (f64, f64) {
        (self.x_m, self.y_m)
    }
}
