"""
Hardware observation input for one configured robot (camera, ArUco, Pose adapter).

- ros2 launch airtable_bringup robot_tracking.launch.py
  robot_config:=<.../dual_robot.yaml> robot:=robot_1 start_camera:=false

Output: /<robot>/measurements/external_pose (PoseWithCovarianceStamped in the
robot's table frame). It is an observation, not the controller's Odometry input:
a velocity estimator/EKF must publish /<robot>/state/odometry.
"""

from pathlib import Path

from airtable_bringup.robot_config import ConfigError, load_robot_config, select_robots
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    ('robot_config', '', 'Robot configuration YAML; empty: config/robots/single_robot.yaml'),
    ('robot', '', 'Robot to observe; required with several robots'),
    ('start_camera', 'false', 'Start the RealSense configured for this robot'),
    ('start_detectors', 'true', 'Start ArUco detectors'),
]
REQUIRED = ('params', 'camera_frame', 'image_topic', 'camera_info_topic', 'marker_size')


def setup(context):
    """Refuse unconfigured cameras/markers instead of guessing them."""
    from ament_index_python.packages import get_package_share_directory

    args = {name: context.launch_configurations.get(name, '') for name, _, _ in ARGUMENTS}
    config_path = args['robot_config'] or str(Path(
        get_package_share_directory('airtable_bringup'), 'config', 'robots',
        'single_robot.yaml'))
    config = load_robot_config(config_path)
    names = [robot['namespace'] for robot in config['robots']]
    if not args['robot'] and len(names) != 1:
        raise ConfigError(f'robot:= is required; {config_path} defines {names}')
    robot = select_robots(config, args['robot'] or names[0])[0]
    tracking = robot['tracking']
    missing = [key for key in REQUIRED if tracking.get(key) in (None, '')]
    if missing:
        raise ConfigError(f'{robot["namespace"]}: tracking {missing} not configured in '
                          f'{config_path}; measure and set them before starting observation')
    start_camera = args['start_camera'].lower() == 'true'
    serial = tracking.get('camera_serial_no')
    others = [r for r in config['robots'] if r is not robot and r['tracking'].get('params')]
    if start_camera and serial in (None, '') and others:
        raise ConfigError(f'{robot["namespace"]}: tracking.camera_serial_no is required to '
                          'start one of several cameras')
    # Topics are /<camera_namespace>/<camera_name>/infra1/...; derive both from image_topic.
    parts = tracking['image_topic'].strip('/').split('/')
    camera_namespace, camera_name = (parts[0], parts[1]) if len(parts) >= 4 else (
        'camera', 'camera')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('airtable_tracking'), 'launch', 'tracking.launch.py'])),
        launch_arguments={
            'namespace': robot['namespace'], 'use_sim_time': 'false',
            'params_file': str(tracking['params']),
            'camera_frame': tracking['camera_frame'],
            'image_topic': tracking['image_topic'],
            'camera_info_topic': tracking['camera_info_topic'],
            'marker_size': str(float(tracking['marker_size'])),
            'start_camera': 'true' if start_camera else 'false',
            'start_detectors': args['start_detectors'],
            # rs_launch.py keeps the serial a string only when it is quoted.
            'camera_serial_no': "''" if serial in (None, '') else f"'{serial}'",
            'camera_name': camera_name, 'camera_namespace': camera_namespace,
        }.items())]


def generate_launch_description():
    """Declare arguments; camera and marker settings come from the configuration."""
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in ARGUMENTS),
        OpaqueFunction(function=setup),
    ])
