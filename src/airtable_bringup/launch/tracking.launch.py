"""
Real camera observation of one robot on its own (calibration, marker checks). docs/adr/0019.

- ros2 launch airtable_bringup tracking.launch.py start_camera:=true rviz:=true
- ros2 launch airtable_bringup tracking.launch.py \
    robot_config:=<.../dual_robot.yaml> robot:=robot_1

Output: /<robot>/measurements/external_pose (PoseWithCovarianceStamped in the robot's
table frame). manual.launch.py / trajectory.launch.py start the same observation with
tracking:=true; the rendered simulator camera is started there with plant:=sim.
"""

from airtable_bringup.launch_helpers import (
    CONFIG_ARGUMENT, default_robot_config, entry_description, is_true, launch_arguments,
    real_tracking_include, select_one, static_rviz_node)
from airtable_bringup.robot_config import load_robot_config


ARGUMENTS = [
    CONFIG_ARGUMENT,
    ('robot', '', 'Robot to observe; required with several robots'),
    ('start_camera', 'false', 'Start the RealSense configured for this robot'),
    ('start_detectors', 'true', 'Start ArUco detectors'),
    ('rviz', 'false', 'Open RViz (rviz/tracking_real.rviz: table area, markers, camera)'),
]


def setup(context):
    """Refuse unconfigured cameras/markers instead of guessing them."""
    args = launch_arguments(context, ARGUMENTS)
    config = load_robot_config(args['robot_config'] or default_robot_config())
    robot = select_one(config, args['robot'])
    actions = [real_tracking_include(config, robot, start_camera=is_true(args['start_camera']),
                                     start_detectors=is_true(args['start_detectors']))]
    if is_true(args['rviz']):
        # The layout names robot_1 and the default RealSense topics/frames.
        actions.append(static_rviz_node('tracking_real', use_sim_time=False))
    return actions


def generate_launch_description():
    """Declare arguments; camera and marker settings come from the configuration."""
    return entry_description(ARGUMENTS, setup)
