"""
Open RViz for configured robots, attaching to an already running sim or hardware.

- ros2 launch airtable_bringup robots_rviz.launch.py robot_config:=<.../dual_robot.yaml>

The config is generated for the selected robots: per robot a table grid, target
path/pose and actual path/pose in a distinct colour pair, all in fixed_frame.
"""

from pathlib import Path

from airtable_bringup.launch_helpers import model_table_positions, rviz_node
from airtable_bringup.robot_config import load_robot_config, select_robots
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


ARGUMENTS = [
    ('robot_config', '', 'Robot configuration YAML; empty: config/robots/single_robot.yaml'),
    ('robots', '', 'Comma-separated robots; empty: all'),
    ('simulation', 'true', 'true: sim truth and /clock; false: state_topic and wall time'),
    ('fixed_frame', 'world', 'Fixed frame; a table frame when world is not calibrated'),
]


def setup(context):
    """Generate an RViz config for the selected robots."""
    from ament_index_python.packages import get_package_share_directory

    args = {name: context.launch_configurations.get(name, '') for name, _, _ in ARGUMENTS}
    config_path = args['robot_config'] or str(Path(
        get_package_share_directory('airtable_bringup'), 'config', 'robots',
        'single_robot.yaml'))
    config = load_robot_config(config_path)
    robots = select_robots(config, args['robots'])
    simulation = args['simulation'].lower() == 'true'
    if simulation and config['model'] is not None:
        table_xy = model_table_positions(config['model'])
    else:
        table_xy = {name: spec['world_pose'][:2] for name, spec in config['tables'].items()
                    if spec['world_pose'] is not None}
    return [rviz_node(robots, simulation=simulation, fixed_frame=args['fixed_frame'],
                      table_xy=table_xy, use_sim_time=simulation)]


def generate_launch_description():
    """Declare arguments; displays come from the robot configuration."""
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS),
        OpaqueFunction(function=setup),
    ])
