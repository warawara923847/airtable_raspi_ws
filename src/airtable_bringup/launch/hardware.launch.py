"""
Raspberry Pi side: start only the selected robot's airtable_hardware node. docs/adr/0019.

- ros2 launch airtable_bringup hardware.launch.py                 (single_robot.yaml)
- ros2 launch airtable_bringup hardware.launch.py \
    robot_config:=<.../dual_robot.yaml> robot:=robot_2

Parameters, lowest first: hardware.params of the robot configuration (or
hardware_params:=; e.g. config/hardware_pi.yaml for the GPIO valves), then the wheel
file of hardware.rw (config/arw3m.yaml, config/sg90hv.yaml; hardware.rw_params to use
another file), then hardware.sun_sensor (config/nanossoc.yaml), then the IMU and sun
sensor frames. Without them every backend is dummy.
The PC runs manual.launch.py or trajectory.launch.py with plant:=real.

record:=true also records what this Pi receives and publishes (profile hardware of
config/recording.yaml) to recordings/ under the current directory, independent of the
Wi-Fi link; the PC records the full experiment with its own record:=true.
"""

from pathlib import Path

from airtable_bringup.launch_helpers import (
    CONFIG_ARGUMENT, default_robot_config, entry_description, is_true, launch_arguments,
    RECORD_ARGUMENTS, record_process, record_snapshots, select_one)
from airtable_bringup.robot_config import ConfigError, load_robot_config, resolve_path
from launch.actions import LogInfo
from launch_ros.actions import Node


ARGUMENTS = [
    CONFIG_ARGUMENT,
    ('robot', '', 'Namespace of the robot this Pi drives; required with several robots'),
    ('hardware_params', '', 'ROS parameter file replacing hardware.params of the configuration '
                            '(path or package://...); empty: use the configuration'),
    ('sun_sensor_params', '', 'ROS parameter file replacing hardware.sun_sensor.params, e.g. '
                              'package://airtable_bringup/config/nanossoc_dummy.yaml'),
    *RECORD_ARGUMENTS,
]


def mount_tf(name, robot, child_frame, mount):
    """Publish the static body -> sensor transform [x, y, z, roll, pitch, yaw]."""
    keys = ['--x', '--y', '--z', '--roll', '--pitch', '--yaw']
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=name, namespace=robot['namespace'], output='log',
        arguments=[item for pair in zip(keys, map(str, mount)) for item in pair] + [
            '--frame-id', robot['body_frame'], '--child-frame-id', child_frame])


def setup(context):
    """Select exactly one robot and start its hardware node with per-robot frames."""
    args = launch_arguments(context, ARGUMENTS)
    # The Pi has no sim/controller/tracking packages; resolve only the hardware section.
    config = load_robot_config(args['robot_config'] or default_robot_config(),
                               hardware_only=True)
    robot = select_one(config, args['robot'])
    hardware = robot['hardware']
    if args['hardware_params']:
        hardware['params'] = resolve_path(args['hardware_params'], Path.cwd(), 'hardware_params')
    sun_sensor = hardware['sun_sensor']
    if args['sun_sensor_params']:
        if sun_sensor is None:
            raise ConfigError('sun_sensor_params:= needs hardware.sun_sensor in the configuration')
        sun_sensor['params'] = resolve_path(args['sun_sensor_params'], Path.cwd(),
                                            'sun_sensor_params')
    parameters = [str(path) for path in (hardware['params'], hardware['rw_params'],
                                         sun_sensor and sun_sensor['params'])
                  if path is not None]
    parameters.append({'use_sim_time': False, 'imu_frame_id': hardware['imu_frame']})
    if sun_sensor is not None:
        parameters.append({'sun_sensor.frame_id': sun_sensor['frame']})
    actions = [
        LogInfo(msg=f'{robot["namespace"]}: hardware params={hardware["params"]}, '
                    f'rw={hardware["rw"]} ({hardware["rw_params"]}), '
                    f'imu_frame={hardware["imu_frame"]}, imu_mount={hardware["imu_mount"]}, '
                    f'sun_sensor={sun_sensor}'),
        Node(package='airtable_hardware', executable='airtable_hardware',
             name='airtable_hardware', namespace=robot['namespace'], output='screen',
             parameters=parameters),
    ]
    mount = hardware['imu_mount']
    if mount is None:
        actions.append(LogInfo(msg=f'{robot["namespace"]}: hardware.imu_mount is not measured; '
                                   f'no {robot["body_frame"]} -> {hardware["imu_frame"]} TF'))
    else:
        actions.append(mount_tf('imu_mount_tf', robot, hardware['imu_frame'], mount))
    if sun_sensor is not None:
        if sun_sensor['mount'] is None:
            actions.append(LogInfo(msg=f'{robot["namespace"]}: hardware.sun_sensor.mount is not '
                                       f'set; no {robot["body_frame"]} -> {sun_sensor["frame"]} '
                                       'TF, so the sun direction cannot be shown on the robot'))
        else:
            actions.append(mount_tf('sun_sensor_mount_tf', robot, sun_sensor['frame'],
                                    sun_sensor['mount']))
    if is_true(args['record']):
        actions.append(record_process(
            args, entry='hardware', plant='real', profile='hardware', robots=[robot],
            snapshots=record_snapshots(config, [robot], simulation=False)))
    return actions


def generate_launch_description():
    """Declare arguments; the selected robot's values come from the configuration."""
    return entry_description(ARGUMENTS, setup)
