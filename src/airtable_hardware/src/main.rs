mod devices;
mod drivers;
mod hardware;
mod io;
mod ros;

use hardware::HardwareState;

use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let context = Context::default_from_env()?;

    let mut executor = context.create_basic_executor();

    let node = executor.create_node("airtable_hardware")?;

    let hardware = node.create_worker(HardwareState::new_dummy());

    let _thruster_subscription = hardware.create_subscription(
        "actuators/thrusters/setpoint",
        ros::handle_thruster_setpoint,
    )?;

    executor.spin(SpinOptions::default()).first_error()?;

    Ok(())
}
