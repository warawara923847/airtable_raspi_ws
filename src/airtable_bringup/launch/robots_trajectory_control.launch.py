"""
PC side: trajectory control for every configured robot, without hardware or sim.

Each robot's controller reads external Odometry on /<namespace>/<state_topic>
(state/odometry by default; an estimator is still required). Output starts
disabled; enable one robot with:
- ros2 service call /robot_2/trajectory/enable std_srvs/srv/SetBool "{data: true}"

Hardware runs separately on each Raspberry Pi (robot_hardware.launch.py robot:=...).
"""

from pathlib import Path

from airtable_bringup.launch_helpers import control_include, rviz_node, table_tf_nodes
from airtable_bringup.robot_config import (
    effective_trajectory, launch_trajectory_overrides, load_robot_config, select_robots, summary)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
import launch.logging


ARGUMENTS = [
    ('robot_config', '', 'Robot configuration YAML; empty: config/robots/single_robot.yaml'),
    ('robots', '', 'Comma-separated robots; empty: all robots in the file'),
    ('enabled', 'false', 'Start output enabled; keep false on hardware'),
    ('use_sim_time', 'false', 'Hardware uses wall time; true only for replay/sim tests'),
    ('trajectory', '', 'line|circle for every selected robot (overrides YAML)'),
    ('period', '', 'Trajectory period [s]'),
    ('amplitude', '', 'Line amplitude [m]'),
    ('radius', '', 'Circle radius [m]'),
    ('direction', '', 'Line direction [rad]'),
    ('fixed_yaw', '', 'Fixed yaw [rad]'),
    ('circle_center_x', '', 'Circle center in each table frame [m]'),
    ('circle_center_y', '', 'Circle center in each table frame [m]'),
    ('publish_table_tf', 'true', 'Publish world->table TF for calibrated tables.world_pose'),
    ('rviz', 'false', 'Start RViz for the selected robots'),
    ('fixed_frame', 'world', 'RViz fixed frame; use a table frame if world is uncalibrated'),
]


def setup(context):
    """Validate the configuration and include the common control launch per robot."""
    from ament_index_python.packages import get_package_share_directory

    args = {name: context.launch_configurations.get(name, '') for name, _, _ in ARGUMENTS}
    config_path = args['robot_config'] or str(Path(
        get_package_share_directory('airtable_bringup'), 'config', 'robots',
        'single_robot.yaml'))
    config = load_robot_config(config_path)
    robots = select_robots(config, args['robots'])
    launch_overrides = launch_trajectory_overrides(args)
    for robot in robots:
        effective_trajectory(robot, launch_overrides)
    sim_time = args['use_sim_time'].lower() == 'true'
    actions = [LogInfo(msg=summary(config, robots))]
    for robot in robots:
        actions.append(control_include(
            robot, use_sim_time=sim_time, enabled=args['enabled'],
            state_topic=robot['state_topic'], launch_overrides=launch_overrides))
    if args['publish_table_tf'].lower() == 'true':
        actions += table_tf_nodes(config, robots, launch.logging.get_logger('robots_control'))
    if args['rviz'].lower() == 'true':
        table_xy = {name: spec['world_pose'][:2] for name, spec in config['tables'].items()
                    if spec['world_pose'] is not None}
        actions.append(rviz_node(robots, simulation=False, fixed_frame=args['fixed_frame'],
                                 table_xy=table_xy, use_sim_time=sim_time))
    return actions


def generate_launch_description():
    """Declare arguments; per-robot values come from the robot configuration."""
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS),
        OpaqueFunction(function=setup),
    ])
