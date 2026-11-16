"""
Raspberry Pi side: start only the selected robot's airtable_hardware node.

- ros2 launch airtable_bringup robot_hardware.launch.py
  robot_config:=<.../dual_robot.yaml> robot:=robot_2

No teleop, manual controller or trajectory controller is started here; the PC
runs robots_trajectory_control.launch.py. The thruster/IMU I/O of
airtable_hardware defaults to dummy backends; real I/O is enabled only by a parameter
file: hardware.params of the robot configuration or hardware_params:=, e.g.
package://airtable_bringup/config/hardware_pi.yaml (GPIO valves, optional ARW-3m).
"""

from pathlib import Path

from airtable_bringup.robot_config import (
    ConfigError, load_robot_config, resolve_path, select_robots)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch_ros.actions import Node


ARGUMENTS = [
    ('robot_config', '', 'Robot configuration YAML; empty: config/robots/single_robot.yaml'),
    ('robot', '', 'Namespace of the robot this Pi drives; required with several robots'),
    ('hardware_params', '', 'ROS parameter file replacing hardware.params of the configuration '
                            '(path or package://...); empty: use the configuration'),
]


def setup(context):
    """Select exactly one robot and start its hardware node with per-robot frames."""
    from ament_index_python.packages import get_package_share_directory

    args = {name: context.launch_configurations.get(name, '') for name, _, _ in ARGUMENTS}
    config_path = args['robot_config'] or str(Path(
        get_package_share_directory('airtable_bringup'), 'config', 'robots',
        'single_robot.yaml'))
    # The Pi has no sim/controller/tracking packages; resolve only the hardware section.
    config = load_robot_config(config_path, hardware_only=True)
    names = [robot['namespace'] for robot in config['robots']]
    if not args['robot'] and len(names) != 1:
        raise ConfigError(f'robot:= is required; {config_path} defines {names}')
    robots = select_robots(config, args['robot'] or names[0])
    if len(robots) != 1:
        raise ConfigError('robot:= must name exactly one robot')
    robot = robots[0]
    hardware = robot['hardware']
    if args['hardware_params']:
        hardware['params'] = resolve_path(args['hardware_params'], Path.cwd(), 'hardware_params')
    parameters = [] if hardware['params'] is None else [str(hardware['params'])]
    parameters.append({'use_sim_time': False, 'imu_frame_id': hardware['imu_frame']})
    actions = [
        LogInfo(msg=f'{robot["namespace"]}: hardware params={hardware["params"]}, '
                    f'imu_frame={hardware["imu_frame"]}, imu_mount={hardware["imu_mount"]}'),
        Node(package='airtable_hardware', executable='airtable_hardware',
             name='airtable_hardware', namespace=robot['namespace'], output='screen',
             parameters=parameters),
    ]
    mount = hardware['imu_mount']
    if mount is None:
        actions.append(LogInfo(msg=f'{robot["namespace"]}: hardware.imu_mount is not measured; '
                                   f'no {robot["body_frame"]} -> {hardware["imu_frame"]} TF'))
    else:
        keys = ['--x', '--y', '--z', '--roll', '--pitch', '--yaw']
        actions.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='imu_mount_tf', namespace=robot['namespace'], output='log',
            arguments=[item for pair in zip(keys, map(str, mount)) for item in pair] + [
                '--frame-id', robot['body_frame'], '--child-frame-id', hardware['imu_frame']]))
    return actions


def generate_launch_description():
    """Declare arguments; the selected robot's values come from the configuration."""
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS),
        OpaqueFunction(function=setup),
    ])
