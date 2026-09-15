mod ros;
mod simulation;
mod thruster;
mod vehicle;

use std::time::Duration;

use rclrs::{Context, CreateBasicExecutor, RclrsErrorFilter, SpinOptions};

use crate::simulation::Simulation;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    //
    // ROS2 context
    //
    let context = Context::default_from_env()?;

    let mut executor = context.create_basic_executor();

    let node = executor.create_node("airtable_sim")?;

    //
    // Mujoco simulation
    //
    let model_path = concat!(env!("CARGO_MANIFEST_DIR"), "/mujoco/airtable.xml");

    let (simulation, mut viewer) =
        Simulation::new_with_viewer(model_path).map_err(std::io::Error::other)?;

    let simulation_worker = node.create_worker(simulation);

    let _thruster_pulse_subscription = simulation_worker.create_subscription(
        "actuators/thrusters/pulse",
        ros::subscribers::handle_thruster_pulse,
    )?;

    println!("[sim] thruster subscription created");

    let _simulation_timer = simulation_worker.create_timer_repeating(
        Duration::from_millis(2),
        |simulation: &mut Simulation, _time: rclrs::Time| {
            simulation.step();
        },
    )?;

    let _viewer_sync_timer = simulation_worker.create_timer_repeating(
        Duration::from_millis(16),
        |simulation: &mut Simulation, _time: rclrs::Time| {
            simulation.sync_viewer();
        },
    )?;

    // ------------------------------------------------
    // ROS executor -> another thread
    // ------------------------------------------------
    let executor_commands = executor.commands().clone();

    let executor_thread = std::thread::spawn(move || executor.spin(SpinOptions::default()));
    // ------------------------------------------------
    // Viewer -> main thread
    // ------------------------------------------------
    while viewer.running() {
        viewer.render()?;
    }

    // Viewer window was closed:
    // stop ROS executor
    executor_commands.halt_spinning();

    let errors = executor_thread
        .join()
        .map_err(|_| std::io::Error::other("ROS executor thread panicked"))?;

    errors.first_error()?;

    Ok(())
}
