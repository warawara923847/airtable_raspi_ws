"""
Run one MuJoCo simulator and independent trajectory control for configured robots.

Typical use (CFG=package://airtable_bringup/config/robots, or an absolute path such as
$(ros2 pkg prefix airtable_bringup)/share/airtable_bringup/config/robots; a relative
path is relative to the current directory):
- ros2 launch airtable_bringup simulation_robots.launch.py  (single_robot.yaml, robot_1)
- ros2 launch airtable_bringup simulation_robots.launch.py robot_config:=$CFG/dual_robot.yaml
- ros2 launch airtable_bringup simulation_robots.launch.py robot_config:=... robots:=robot_2
- ... robot_config:=$CFG/dual_robot.yaml tracking:=true viewer:=false

Control inputs are the simulator's ground truth (per robot simulation.state_topic).
tracking:=true renders each robot's own table camera (tracking.simulation_camera)
and runs its ArUco observation into /<ns>/measurements/external_pose, for
comparison only; it does not feed the controllers.
"""

from pathlib import Path

from airtable_bringup.launch_helpers import control_include, model_table_positions, rviz_node
from airtable_bringup.robot_config import (
    check_simulation_cameras, check_simulation_model, ConfigError, effective_trajectory,
    initial_pose, launch_trajectory_overrides, load_robot_config, ros_parameters,
    select_robots, summary)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    ('robot_config', '', 'Robot configuration YAML; empty: config/robots/single_robot.yaml'),
    ('robots', '', 'Comma-separated robots to start; empty: all robots in the file'),
    ('model_path', '', 'Override simulation.model of the robot configuration'),
    ('trajectory', '', 'line|circle for every selected robot (overrides YAML)'),
    ('period', '', 'Trajectory period [s] for every selected robot'),
    ('amplitude', '', 'Line amplitude [m]'),
    ('radius', '', 'Circle radius [m]'),
    ('direction', '', 'Line direction [rad]'),
    ('fixed_yaw', '', 'Fixed yaw [rad]'),
    ('circle_center_x', '', 'Circle center in each table frame [m]'),
    ('circle_center_y', '', 'Circle center in each table frame [m]'),
    ('initial_x', '', 'Initial x in each table frame [m], overrides YAML'),
    ('initial_y', '', 'Initial y in each table frame [m], overrides YAML'),
    ('initial_yaw', '', 'Initial yaw [rad]'),
    ('enabled', 'true', 'Start trajectory output enabled'),
    ('viewer', 'true', 'Open the MuJoCo viewer'),
    ('rviz', 'false', 'Start RViz for the selected robots (world fixed frame)'),
    ('tracking', 'false', 'Per-table camera + ArUco observation for every selected robot'),
    ('camera_fps', '30.0', 'Simulated camera rate'),
    ('disturbance', 'false', 'Start the scripted disturbance publisher for one robot'),
    ('disturbance_robot', '', 'Target of disturbance:=true; required with several robots'),
    ('disturbance_params_file', '', 'Disturbance ROS parameters; empty: config/disturbance.yaml'),
]


def default_share(*parts):
    """Resolve a file in this package's share directory."""
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('airtable_bringup'), *parts))


def disturbance_frame(frame, robot):
    """Map a robot_1-style disturbance frame to the selected robot's frames."""
    if frame == 'world':
        return frame
    if frame in ('table_1', robot['table_frame']):
        return robot['table_frame']
    if frame in ('robot_1/base_link', robot['body_frame']):
        return robot['body_frame']
    raise ConfigError(f'disturbance frame_id {frame!r} must be world, a table or a body frame')


def setup(context):
    """Validate everything before any process starts, then build the nodes."""
    args = {name: context.launch_configurations.get(name, '') for name, _, _ in ARGUMENTS}
    config_path = args['robot_config'] or default_share('config', 'robots', 'single_robot.yaml')
    config = load_robot_config(config_path)
    robots = select_robots(config, args['robots'])
    model = args['model_path'] or config['model']
    if model is None:
        raise ConfigError(f'{config_path}: simulation.model is not set')
    model = str(Path(model).expanduser().resolve())
    check_simulation_model(model, robots)
    launch_overrides = launch_trajectory_overrides(args)
    poses = []
    for robot in robots:
        params = effective_trajectory(robot, launch_overrides)
        pose = initial_pose(robot, params, args)
        if pose is not None:
            poses.append((robot['namespace'], pose))
    tracking = args['tracking'].lower() == 'true'
    if tracking:
        missing = [robot['namespace'] for robot in robots
                   if robot['tracking'].get('simulation') is None]
        if missing:
            raise ConfigError(f'tracking:=true: {missing} have no tracking.simulation_params; '
                              'select robots:= with simulation tracking or add it')
        check_simulation_cameras(model, robots)
    sim_params = {
        'use_sim_time': False,
        'model_path': model,
        'viewer': args['viewer'].lower() == 'true',
        'enable_camera': tracking,
        'camera_fps': float(args['camera_fps']),
        'vehicles': [robot['namespace'] for robot in robots],
        'vehicle_element_prefixes': [robot['simulation']['element_prefix'] for robot in robots],
    }
    if tracking:
        cameras = [robot['tracking']['simulation'] for robot in robots]
        sim_params['camera_names'] = [camera['camera'] for camera in cameras]
        sim_params['camera_namespaces'] = [camera['namespace'] for camera in cameras]
        sim_params['camera_frames'] = [camera['camera_frame'] for camera in cameras]
    if poses:
        sim_params['initial_pose_vehicles'] = [name for name, _ in poses]
        sim_params['initial_poses'] = [value for _, pose in poses for value in pose]
    actions = [
        LogInfo(msg=summary(config, robots) + f'\ninitial poses (table frames): {poses}'),
        Node(package='airtable_sim', executable='airtable_sim', name='airtable_sim',
             output='screen', parameters=[sim_params]),
    ]
    for robot in robots:
        actions.append(control_include(
            robot, use_sim_time=True, enabled=args['enabled'],
            state_topic=robot['simulation']['state_topic'],
            launch_overrides=launch_overrides))
    if args['disturbance'].lower() == 'true':
        names = [robot['namespace'] for robot in robots]
        target = args['disturbance_robot'] or (names[0] if len(names) == 1 else '')
        if target not in names:
            raise ConfigError(f'disturbance:=true needs disturbance_robot:= one of {names}')
        robot = robots[names.index(target)]
        params_file = args['disturbance_params_file'] or default_share(
            'config', 'disturbance.yaml')
        frame = ros_parameters(params_file, 'disturbance_params_file').get('frame_id', 'table_1')
        actions.append(Node(
            package='airtable_bringup', executable='sim_disturbance',
            name='airtable_sim_disturbance', namespace=target, output='screen',
            parameters=[params_file, {'use_sim_time': True,
                                      'frame_id': disturbance_frame(frame, robot)}]))
    for robot in robots if tracking else []:
        camera = robot['tracking']['simulation']
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('airtable_tracking'), 'launch',
                                  'tracking.launch.py'])),
            launch_arguments={
                'namespace': robot['namespace'], 'use_sim_time': 'true',
                'params_file': str(robot['tracking']['simulation_params']),
                'start_camera': 'false', 'start_detectors': 'true',
                'camera_frame': camera['camera_frame'],
                'image_topic': camera['image_topic'],
                'camera_info_topic': camera['camera_info_topic'],
                'marker_size': str(camera['marker_size']),
            }.items()))
    if args['rviz'].lower() == 'true':
        actions.append(rviz_node(robots, simulation=True, fixed_frame='world',
                                 table_xy=model_table_positions(model), use_sim_time=True))
    return actions


def generate_launch_description():
    """Declare arguments; everything else is resolved from the robot configuration."""
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS),
        OpaqueFunction(function=setup),
    ])
