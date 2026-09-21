use mujoco_rs::wrappers::mj_data::MjData;
use mujoco_rs::wrappers::mj_model::traits::ModelType;

#[derive(Debug, Clone)]
pub struct SimImu {
    name: String,

    accel_sensor_id: usize,
    gyro_sensor_id: usize,
    orientation_sensor_id: usize,
}

#[derive(Debug, Clone)]
pub struct ImuState {
    pub name: String,
    pub orientation_wxyz: [f64; 4],
    pub linear_acceleration: [f64; 3],
    pub angular_velocity: [f64; 3],
}

impl SimImu {
    pub fn new(
        name: impl Into<String>,
        accel_sensor_id: usize,
        gyro_sensor_id: usize,
        orientation_sensor_id: usize,
    ) -> Self {
        Self {
            name: name.into(),
            accel_sensor_id,
            gyro_sensor_id,
            orientation_sensor_id,
        }
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn read<M>(&self, data: &MjData<M>) -> ImuState
    where
        M: ModelType,
    {
        let time = data.time();

        let accel = data.read_sensor_fixed::<3>(self.accel_sensor_id, time, -1);

        let gyro = data.read_sensor_fixed::<3>(self.gyro_sensor_id, time, -1);

        let orientation = data.read_sensor_fixed::<4>(self.orientation_sensor_id, time, -1);

        ImuState {
            name: self.name.clone(),
            orientation_wxyz: orientation,
            linear_acceleration: accel,
            angular_velocity: gyro,
        }
    }
}
