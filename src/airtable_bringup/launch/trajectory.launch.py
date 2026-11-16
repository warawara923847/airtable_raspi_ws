"""
Fixed-yaw trajectory tracking for the configured robots (PC side). docs/adr/0019.

CFG=package://airtable_bringup/config/robots (or an absolute path; a relative path is
relative to the current directory):
- ros2 launch airtable_bringup trajectory.launch.py                    (MuJoCo, robot_1)
- ros2 launch airtable_bringup trajectory.launch.py trajectory:=circle tracking:=true
- ros2 launch airtable_bringup trajectory.launch.py robot_config:=$CFG/dual_robot.yaml
- ros2 launch airtable_bringup trajectory.launch.py plant:=real robot_config:=...
    with hardware.launch.py robot:=... on each Raspberry Pi

plant:=sim feeds the controllers the simulator's ground truth (per robot
simulation.state_topic) and starts output enabled. plant:=real reads external Odometry
on /<namespace>/<state_topic> (an estimator is still required) and starts disabled:
- ros2 service call /robot_1/trajectory/enable std_srvs/srv/SetBool "{data: true}"
tracking:=true adds ArUco observation (/<ns>/measurements/external_pose) for comparison;
it does not feed the controllers.
"""

from pathlib import Path

from airtable_bringup.launch_helpers import (
    bringup_share, CONFIG_ARGUMENT, control_include, default_robot_config, entry_description,
    is_true, launch_arguments, model_table_positions, PLANT_ARGUMENT, plant_of,
    real_tracking_include, reject_for_real, rviz_node, simulation_actions, table_tf_nodes)
from airtable_bringup.robot_config import (
    ConfigError, effective_trajectory, initial_pose, launch_trajectory_overrides,
    load_robot_config, ros_parameters, select_robots, summary)
from launch.actions import LogInfo
import launch.logging
from launch_ros.actions import Node


ARGUMENTS = [
    PLANT_ARGUMENT,
    CONFIG_ARGUMENT,
    ('robots', '', 'Comma-separated robots to run; empty: all robots in the file'),
    ('trajectory', '', 'line|circle for every selected robot (overrides YAML)'),
    ('period', '', 'Trajectory period [s] for every selected robot'),
    ('amplitude', '', 'Line amplitude [m]'),
    ('radius', '', 'Circle radius [m]'),
    ('direction', '', 'Line direction [rad]'),
    ('fixed_yaw', '', 'Fixed yaw [rad]'),
    ('circle_center_x', '', 'Circle center in each table frame [m]'),
    ('circle_center_y', '', 'Circle center in each table frame [m]'),
    ('enabled', '', 'Start output enabled; empty: true for sim, false for real'),
    ('state_topic', '', 'Controller state input for every robot; empty: from the configuration'),
    ('tracking', 'false', 'ArUco observation: per-table rendered camera (sim) or RealSense'),
    ('start_camera', 'true', 'plant:=real with tracking: start the RealSense on this PC'),
    ('rviz', 'false', 'Start RViz for the selected robots'),
    ('fixed_frame', 'world', 'RViz fixed frame; a table frame if world is not calibrated'),
    ('publish_table_tf', 'true', 'plant:=real: world->table TF for calibrated tables'),
    # Simulator only.
    ('model_path', '', 'plant:=sim: override simulation.model of the configuration'),
    ('initial_x', '', 'plant:=sim: initial x in each table frame [m], overrides YAML'),
    ('initial_y', '', 'plant:=sim: initial y in each table frame [m], overrides YAML'),
    ('initial_yaw', '', 'plant:=sim: initial yaw [rad]'),
    ('viewer', 'true', 'plant:=sim: open the MuJoCo viewer'),
    ('camera_fps', '30.0', 'plant:=sim: rendered camera rate'),
    ('disturbance', 'false', 'plant:=sim: scripted disturbance publisher for one robot'),
    ('disturbance_robot', '', 'Target of disturbance:=true; required with several robots'),
    ('disturbance_params_file', '', 'Disturbance ROS parameters; empty: config/disturbance.yaml'),
]
SIM_ONLY = {'model_path': '', 'initial_x': '', 'initial_y': '', 'initial_yaw': '',
            'viewer': 'true', 'camera_fps': '30.0', 'disturbance': 'false',
            'disturbance_robot': '', 'disturbance_params_file': ''}


def disturbance_frame(frame, robot):
    """Map a robot_1-style disturbance frame to the selected robot's frames."""
    if frame == 'world':
        return frame
    if frame in ('table_1', robot['table_frame']):
        return robot['table_frame']
    if frame in ('robot_1/base_link', robot['body_frame']):
        return robot['body_frame']
    raise ConfigError(f'disturbance frame_id {frame!r} must be world, a table or a body frame')


def simulation(args, config, robots, launch_overrides):
    """Start MuJoCo at the trajectory start poses and the optional disturbance publisher."""
    model = args['model_path'] or config['model']
    if model is None:
        raise ConfigError(f'{config["path"]}: simulation.model is not set')
    model = str(Path(model).expanduser().resolve())
    poses = []
    for robot in robots:
        pose = initial_pose(robot, effective_trajectory(robot, launch_overrides), args)
        if pose is not None:
            poses.append((robot['namespace'], pose))
    actions = [LogInfo(msg=f'initial poses (table frames): {poses}')]
    actions += simulation_actions(robots, model=model, viewer=is_true(args['viewer']),
                                  camera_fps=args['camera_fps'],
                                  tracking=is_true(args['tracking']), poses=poses)
    if is_true(args['disturbance']):
        names = [robot['namespace'] for robot in robots]
        target = args['disturbance_robot'] or (names[0] if len(names) == 1 else '')
        if target not in names:
            raise ConfigError(f'disturbance:=true needs disturbance_robot:= one of {names}')
        robot = robots[names.index(target)]
        params_file = args['disturbance_params_file'] or bringup_share(
            'config', 'disturbance.yaml')
        frame = ros_parameters(params_file, 'disturbance_params_file').get('frame_id', 'table_1')
        actions.append(Node(
            package='airtable_bringup', executable='sim_disturbance',
            name='airtable_sim_disturbance', namespace=target, output='screen',
            parameters=[params_file, {'use_sim_time': True,
                                      'frame_id': disturbance_frame(frame, robot)}]))
    return actions, model


def setup(context):
    """Validate everything before any process starts, then build the nodes."""
    args = launch_arguments(context, ARGUMENTS)
    plant = plant_of(args)
    if plant == 'real':
        reject_for_real(args, SIM_ONLY)
    config = load_robot_config(args['robot_config'] or default_robot_config())
    robots = select_robots(config, args['robots'])
    launch_overrides = launch_trajectory_overrides(args)
    for robot in robots:
        effective_trajectory(robot, launch_overrides)
    sim = plant == 'sim'
    enabled = args['enabled'] or ('true' if sim else 'false')
    actions = [LogInfo(msg=f'trajectory: plant={plant}\n' + summary(config, robots))]
    if sim:
        sim_actions, model = simulation(args, config, robots, launch_overrides)
        actions += sim_actions
    for robot in robots:
        default_state = robot['simulation']['state_topic'] if sim else robot['state_topic']
        actions.append(control_include(
            robot, use_sim_time=sim, enabled=enabled,
            state_topic=args['state_topic'] or default_state,
            launch_overrides=launch_overrides))
    if not sim:
        if is_true(args['publish_table_tf']):
            actions += table_tf_nodes(config, robots, launch.logging.get_logger('trajectory'))
        if is_true(args['tracking']):
            actions += [real_tracking_include(config, robot,
                                              start_camera=is_true(args['start_camera']))
                        for robot in robots]
    if is_true(args['rviz']):
        if sim:
            table_xy = model_table_positions(model)
        else:
            table_xy = {name: spec['world_pose'][:2] for name, spec in config['tables'].items()
                        if spec['world_pose'] is not None}
        actions.append(rviz_node(robots, simulation=sim, fixed_frame=args['fixed_frame'],
                                 table_xy=table_xy, use_sim_time=sim))
    return actions


def generate_launch_description():
    """Declare arguments; per-robot values come from the robot configuration."""
    return entry_description(ARGUMENTS, setup)
