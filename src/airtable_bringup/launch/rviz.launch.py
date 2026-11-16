"""
Open RViz, attaching to an already running sim or real robot. docs/adr/0019.

- ros2 launch airtable_bringup rviz.launch.py                      (sim, generated view)
- ros2 launch airtable_bringup rviz.launch.py robot_config:=<.../dual_robot.yaml>
- ros2 launch airtable_bringup rviz.launch.py plant:=real fixed_frame:=table_1
- ros2 launch airtable_bringup rviz.launch.py rviz_config:=tracking       (sim camera view)

The generated view shows per robot a table grid, target path/pose, actual path/pose
and the ArUco pose in a distinct colour set. rviz_config:=tracking / tracking_real
open the fixed robot_1 camera layouts in rviz/, or give a .rviz path.
"""

from airtable_bringup.launch_helpers import (
    CONFIG_ARGUMENT, default_robot_config, entry_description, launch_arguments,
    model_table_positions, PLANT_ARGUMENT, plant_of, rviz_node, static_rviz_node)
from airtable_bringup.robot_config import load_robot_config, select_robots


ARGUMENTS = [
    PLANT_ARGUMENT,
    CONFIG_ARGUMENT,
    ('robots', '', 'Comma-separated robots; empty: all'),
    ('fixed_frame', 'world', 'Fixed frame; a table frame when world is not calibrated'),
    ('rviz_config', '', 'Empty: generated; tracking | tracking_real | path to a .rviz file'),
]


def setup(context):
    """Generate an RViz config for the selected robots, or open a fixed layout."""
    args = launch_arguments(context, ARGUMENTS)
    simulation = plant_of(args) == 'sim'
    if args['rviz_config']:
        return [static_rviz_node(args['rviz_config'], use_sim_time=simulation)]
    config = load_robot_config(args['robot_config'] or default_robot_config())
    robots = select_robots(config, args['robots'])
    if simulation and config['model'] is not None:
        table_xy = model_table_positions(config['model'])
    else:
        table_xy = {name: spec['world_pose'][:2] for name, spec in config['tables'].items()
                    if spec['world_pose'] is not None}
    return [rviz_node(robots, simulation=simulation, fixed_frame=args['fixed_frame'],
                      table_xy=table_xy, use_sim_time=simulation)]


def generate_launch_description():
    """Declare arguments; displays come from the robot configuration."""
    return entry_description(ARGUMENTS, setup)
