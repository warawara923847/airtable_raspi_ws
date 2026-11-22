"""
Manual operation of one robot from the gamepad (PC side). docs/adr/0019.

- ros2 launch airtable_bringup manual.launch.py                       (MuJoCo, robot_1)
- ros2 launch airtable_bringup manual.launch.py tracking:=true rviz:=true
- ros2 launch airtable_bringup manual.launch.py plant:=real tracking:=true
    with hardware.launch.py running on the robot's Raspberry Pi
- ros2 launch airtable_bringup manual.launch.py robot_config:=<.../dual_robot.yaml> robot:=robot_2
- ros2 launch airtable_bringup manual.launch.py foxglove:=true record:=true record_label:=trial01
    live view in Foxglove (ws://127.0.0.1:8765) and an MCAP under recordings/

Thrusters: airtable_controller_node maps the D-pad / rotate buttons to valve pulses.
Reaction wheel:
- plant:=sim, or hardware.rw: arw3m - the controller's wheel-speed loop (torque command,
  RW state feedback).
- hardware.rw: sg90hv - airtable_rw_attitude_node: manual speed levels and yaw-hold PID
  on the ArUco pose (needs tracking:=true for PID). docs/sg90hv-rw.md.
"""

from airtable_bringup.launch_helpers import (
    CONFIG_ARGUMENT, default_robot_config, entry_description, is_true, launch_arguments,
    pc_observation_actions, PC_OBSERVATION_ARGUMENTS, PLANT_ARGUMENT, plant_of,
    real_tracking_include, reject_for_real, rviz_node, select_one, SIM_CLOCK_ARGUMENT,
    simulation_actions, static_rviz_node)
from airtable_bringup.robot_config import ConfigError, flatten, load_robot_config
from launch.actions import LogInfo


ARGUMENTS = [
    PLANT_ARGUMENT,
    CONFIG_ARGUMENT,
    ('robot', '', 'Robot to drive; required when the configuration has several'),
    ('start_joy', 'true', 'Start the gamepad driver (false: publish commands yourself)'),
    ('tracking', 'false', 'ArUco observation: rendered camera (sim) or the RealSense (real)'),
    ('start_camera', 'true', 'plant:=real with tracking: start the RealSense on this PC'),
    ('rviz', 'false', 'Open RViz (camera view with tracking, otherwise robot overview)'),
    ('viewer', 'true', 'plant:=sim: open the MuJoCo viewer'),
    ('camera_fps', '90.0', 'plant:=sim: rendered camera rate (real RealSense: 90)'),
    SIM_CLOCK_ARGUMENT,
    *PC_OBSERVATION_ARGUMENTS,
]
SIM_ONLY = {'viewer': 'true', 'camera_fps': '90.0', 'sim_clock_origin': 'wall'}


def setup(context):
    """Validate the configuration, then start the operator side and the plant if simulated."""
    from ament_index_python.packages import get_package_share_directory
    from launch_ros.actions import Node

    args = launch_arguments(context, ARGUMENTS)
    plant = plant_of(args)
    if plant == 'real':
        reject_for_real(args, SIM_ONLY)
    config = load_robot_config(args['robot_config'] or default_robot_config())
    robot = select_one(config, args['robot'])
    namespace = robot['namespace']
    tracking = is_true(args['tracking'])
    rw = 'torque (simulated wheel)' if plant == 'sim' else robot['hardware']['rw']
    actions = [LogInfo(msg=f'manual: plant={plant}, robot={namespace}, rw={rw}, '
                           f'tracking={tracking}, config={config["path"]}')]
    if is_true(args['start_joy']):
        actions.append(Node(package='joy', executable='game_controller_node',
                            name='game_controller_node', output='screen'))
    actions += [
        Node(package='airtable_teleop', executable='airtable_teleop_node',
             name='airtable_teleop', namespace=namespace, output='screen',
             parameters=[str(get_package_share_directory('airtable_teleop')
                             + '/config/teleop.yaml')]),
        Node(package='airtable_controller', executable='airtable_controller_node',
             name='airtable_controller', namespace=namespace, output='screen',
             parameters=[str(robot['thruster_params']),
                         flatten(robot['thruster_overrides'])]),
    ]
    if plant == 'sim':
        if config['model'] is None:
            raise ConfigError(f'{config["path"]}: simulation.model is not set')
        # Manual runs start where the configuration (or the MJCF) puts the robot.
        pose = robot['simulation']['initial_pose']
        actions += simulation_actions(
            [robot], model=str(config['model']), viewer=is_true(args['viewer']),
            camera_fps=args['camera_fps'], tracking=tracking,
            poses=[] if pose is None else [(namespace, pose)],
            clock_origin=args['sim_clock_origin'])
    else:
        if robot['hardware']['rw'] == 'sg90hv':
            actions.append(Node(
                package='airtable_controller', executable='airtable_rw_attitude_node',
                name='airtable_rw_attitude', namespace=namespace, output='screen',
                parameters=[str(get_package_share_directory('airtable_controller')
                                + '/config/rw_attitude_sg90hv.yaml')]))
        if tracking:
            actions.append(real_tracking_include(
                config, robot, start_camera=is_true(args['start_camera'])))
    if is_true(args['rviz']):
        if tracking:
            actions.append(static_rviz_node('tracking' if plant == 'sim' else 'tracking_real',
                                            use_sim_time=plant == 'sim'))
        else:
            actions.append(rviz_node([robot], simulation=plant == 'sim',
                                     fixed_frame=robot['table_frame'], table_xy={},
                                     use_sim_time=plant == 'sim'))
    actions += pc_observation_actions(args, entry='manual', plant=plant, config=config,
                                      robots=[robot])
    return actions


def generate_launch_description():
    """Declare arguments; robots, files and the wheel type come from the configuration."""
    return entry_description(ARGUMENTS, setup)
