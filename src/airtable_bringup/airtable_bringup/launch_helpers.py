"""
Launch building blocks shared by the entry launch files (docs/adr/0019).

Entry points: manual / trajectory (PC, plant:=sim|real), hardware (Pi), tracking (real
camera only) and rviz. They all read a robot configuration (config/robots/*.yaml).
"""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from airtable_bringup.robot_config import (
    check_simulation_cameras, check_simulation_model, ConfigError, effective_trajectory, merge,
    package_share, pulse_thrust_caps, resolve_path, select_robots, yaml_argument)
import yaml

PLANTS = ('sim', 'real')
PLANT_ARGUMENT = ('plant', 'sim', "sim: run MuJoCo on this PC; real: the robot's Raspberry Pi "
                  'runs hardware.launch.py')
CONFIG_ARGUMENT = ('robot_config', '',
                   'Robot configuration YAML; empty: config/robots/single_robot.yaml')
SIM_CLOCK_ORIGINS = ('wall', 'zero')
SIM_CLOCK_ARGUMENT = ('sim_clock_origin', 'wall',
                      'plant:=sim: wall: /clock starts at the current date (recordings of '
                      'different runs do not overlap); zero: /clock starts at 0')


def bringup_share(*parts):
    """Return a path inside this package's share directory."""
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('airtable_bringup'), *parts))


def default_robot_config():
    """Return the robot configuration used when robot_config:= is empty."""
    return bringup_share('config', 'robots', 'single_robot.yaml')


def launch_arguments(context, arguments):
    """Return (name, default, description) launch argument values as strings."""
    return {name: context.launch_configurations.get(name, '') for name, _, _ in arguments}


def entry_description(arguments, setup):
    """Declare the arguments and resolve everything else in an OpaqueFunction."""
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction

    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in arguments),
        OpaqueFunction(function=setup),
    ])


def is_true(value):
    """Convert a launch argument string to bool."""
    return str(value).lower() == 'true'


def plant_of(args):
    """Return the validated plant:= value."""
    plant = args['plant'].lower()
    if plant not in PLANTS:
        raise ConfigError(f'plant must be one of {list(PLANTS)}, got {args["plant"]!r}')
    return plant


def reject_for_real(args, defaults):
    """Refuse simulator-only arguments given with plant:=real instead of ignoring them."""
    given = [name for name, default in defaults.items() if args.get(name, default) != default]
    if given:
        raise ConfigError(f'{given} only apply to plant:=sim')


def select_one(config, name):
    """Return the robot named by robot:=, which may be omitted for one-robot files."""
    names = [robot['namespace'] for robot in config['robots']]
    if not name and len(names) != 1:
        raise ConfigError(f'robot:= is required; {config["path"]} defines {names}')
    robots = select_robots(config, name or names[0])
    if len(robots) != 1:
        raise ConfigError('robot:= must name exactly one robot')
    return robots[0]


def simulation_actions(robots, *, model, viewer, camera_fps, tracking, poses,
                       clock_origin='wall'):
    """Start one MuJoCo simulator for the robots, plus per-table camera observation."""
    from launch_ros.actions import Node

    if clock_origin not in SIM_CLOCK_ORIGINS:
        raise ConfigError(f'sim_clock_origin must be one of {list(SIM_CLOCK_ORIGINS)}, '
                          f'got {clock_origin!r}')
    check_simulation_model(model, robots)
    if tracking:
        missing = [robot['namespace'] for robot in robots
                   if robot['tracking'].get('simulation') is None]
        if missing:
            raise ConfigError(f'tracking:=true: {missing} have no tracking.simulation_params; '
                              'select robots:= with simulation tracking or add it')
        check_simulation_cameras(model, robots)
    sim_params = {
        # The simulator's wall timers produce /clock; they must not wait on that clock.
        'use_sim_time': False,
        'clock_origin': clock_origin,
        'model_path': model,
        'viewer': viewer,
        'enable_camera': tracking,
        'camera_fps': float(camera_fps),
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
    actions = [Node(package='airtable_sim', executable='airtable_sim', name='airtable_sim',
                    output='screen', parameters=[sim_params])]
    for robot in robots if tracking else []:
        camera = robot['tracking']['simulation']
        actions.append(_tracking_include({
            'namespace': robot['namespace'], 'use_sim_time': 'true',
            'params_file': str(robot['tracking']['simulation_params']),
            'start_camera': 'false', 'start_detectors': 'true',
            'camera_frame': camera['camera_frame'],
            'image_topic': camera['image_topic'],
            'camera_info_topic': camera['camera_info_topic'],
            'marker_size': str(camera['marker_size']),
        }))
    for robot in robots:
        actions += sim_sun_sensor_actions(robot)
    return actions


def mount_tf(name, robot, child_frame, mount):
    """Publish the static body -> sensor transform [x, y, z, roll, pitch, yaw]."""
    from launch_ros.actions import Node

    keys = ['--x', '--y', '--z', '--roll', '--pitch', '--yaw']
    return Node(
        package='tf2_ros', executable='static_transform_publisher',
        name=name, namespace=robot['namespace'], output='log',
        arguments=[item for pair in zip(keys, map(str, mount)) for item in pair] + [
            '--frame-id', robot['body_frame'], '--child-frame-id', child_frame])


def sim_sun_sensor_actions(robot):
    """Simulated sun sensor and its mount TF; nothing for a robot without the sensor."""
    sensor = robot['simulation']['sun_sensor']
    if sensor is None:
        return []
    missing = [name for name, value in (('sun_sensor.mount', sensor['mount']),
                                        (f'tables.{robot["table_frame"]}.sun', sensor['sun']))
               if value is None]
    if missing:
        raise ConfigError(f'{robot["namespace"]}: the simulated sun sensor needs {missing} '
                          'in the robot configuration')
    from launch_ros.actions import Node

    return [
        mount_tf('sun_sensor_mount_tf', robot, sensor['frame'], sensor['mount']),
        Node(package='airtable_bringup', executable='sim_sun_sensor',
             name='airtable_sim_sun_sensor', namespace=robot['namespace'], output='screen',
             parameters=[str(sensor['params']), {
                 'use_sim_time': True,
                 'sun_sensor.frame_id': sensor['frame'],
                 'sun_sensor.mount': sensor['mount'],
                 'sun_sensor.azimuth_deg': sensor['sun']['azimuth_deg'],
                 'sun_sensor.elevation_deg': sensor['sun']['elevation_deg'],
             }]),
    ]


REAL_TRACKING_REQUIRED = ('params', 'camera_frame', 'image_topic', 'camera_info_topic',
                          'marker_size')


def real_tracking_include(config, robot, *, start_camera, start_detectors=True):
    """Observe one robot with its RealSense and ArUco; refuse unmeasured settings."""
    tracking = robot['tracking']
    missing = [key for key in REAL_TRACKING_REQUIRED if tracking.get(key) in (None, '')]
    if missing:
        raise ConfigError(f'{robot["namespace"]}: tracking {missing} not configured in '
                          f'{config["path"]}; measure and set them before starting observation')
    serial = tracking.get('camera_serial_no')
    others = [r for r in config['robots'] if r is not robot and r['tracking'].get('params')]
    if start_camera and serial in (None, '') and others:
        raise ConfigError(f'{robot["namespace"]}: tracking.camera_serial_no is required to '
                          'start one of several cameras')
    # Topics are /<camera_namespace>/<camera_name>/infra1/...; derive both from image_topic.
    parts = tracking['image_topic'].strip('/').split('/')
    camera_namespace, camera_name = (parts[0], parts[1]) if len(parts) >= 4 else (
        'camera', 'camera')
    return _tracking_include({
        'namespace': robot['namespace'], 'use_sim_time': 'false',
        'params_file': str(tracking['params']),
        'camera_frame': tracking['camera_frame'],
        'image_topic': tracking['image_topic'],
        'camera_info_topic': tracking['camera_info_topic'],
        'marker_size': str(float(tracking['marker_size'])),
        'start_camera': 'true' if start_camera else 'false',
        'start_detectors': 'true' if start_detectors else 'false',
        # rs_launch.py keeps the serial a string only when it is quoted.
        'camera_serial_no': "''" if serial in (None, '') else f"'{serial}'",
        'camera_name': camera_name, 'camera_namespace': camera_namespace,
    })


def _tracking_include(arguments):
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('airtable_tracking'), 'launch', 'tracking.launch.py'])),
        launch_arguments=arguments.items())


# Fixed RViz layouts in rviz/ (robot_1, single camera); generated otherwise.
STATIC_RVIZ = {'tracking': 'tracking.rviz', 'tracking_real': 'tracking_real.rviz',
               'sun_sensor': 'sun_sensor.rviz'}


def static_rviz_node(name, *, use_sim_time):
    """Start RViz with rviz/<STATIC_RVIZ[name]> or an explicit .rviz path."""
    from launch_ros.actions import Node

    path = bringup_share('rviz', STATIC_RVIZ[name]) if name in STATIC_RVIZ else name
    return Node(package='rviz2', executable='rviz2', name='airtable_rviz', output='screen',
                arguments=['-d', path], parameters=[{'use_sim_time': use_sim_time}])


# Distinct colours per robot slot: (reference, actual, ArUco measurement).
PALETTE = [
    ('40; 230; 100', '70; 150; 255', '255; 80; 200'),     # robot 1: green, blue, magenta
    ('255; 200; 40', '230; 90; 40', '80; 220; 255'),      # robot 2: yellow, orange, cyan
    ('200; 120; 255', '255; 255; 255', '255; 150; 150'),  # robot 3: violet, white, salmon
    ('120; 255; 255', '255; 120; 180', '180; 180; 180'),
]


def control_include(robot, *, use_sim_time, enabled, state_topic, launch_overrides=None):
    """Include the common trajectory_control.launch.py for one robot."""
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PathJoinSubstitution
    from launch_ros.substitutions import FindPackageShare

    trajectory = merge(robot['trajectory_overrides'], launch_overrides or {})
    thruster_overrides = merge(robot['thruster_overrides'], pulse_thrust_caps(
        robot, effective_trajectory(robot, launch_overrides)))
    arguments = {
        'namespace': robot['namespace'],
        'use_sim_time': 'true' if use_sim_time else 'false',
        'state_topic': state_topic,
        'enabled': enabled,
        'reference_frame': robot['table_frame'],
        'params_file': str(robot['trajectory_params']),
        'thruster_params_file': str(robot['thruster_params']),
        'trajectory_overrides': yaml_argument(trajectory) if trajectory else '',
        'thruster_overrides': (yaml_argument(thruster_overrides)
                               if thruster_overrides else ''),
    }
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('airtable_bringup'), 'launch', 'include',
            'trajectory_control.launch.py'])),
        launch_arguments=arguments.items())


DEFAULT_ESTIMATOR_PARAMS = 'package://airtable_estimator/config/ekf_external_pose.yaml'
STATE_SOURCES = ('truth', 'estimator', 'external')


def estimator_params_path(params):
    """Resolve estimator_params:= (empty: the pose-only EKF of airtable_estimator)."""
    return resolve_path(params or DEFAULT_ESTIMATOR_PARAMS, Path.cwd(), 'estimator_params',
                        share=package_share)


def estimator_node(robot, *, params, use_sim_time, publish_tf, state_topic):
    """
    Start robot_localization's EKF on the ArUco pose of one robot (docs/adr/0024).

    The frames follow the robot configuration: pose in the table frame, twist in
    <namespace>/base_link. Its odometry/filtered output is remapped to state_topic, the
    controller input. publish_tf adds table -> base_link (only where nothing else does).
    """
    from launch_ros.actions import Node

    path = estimator_params_path(params)
    namespace = robot['namespace']
    return Node(
        package='robot_localization', executable='ekf_node', name='airtable_estimator',
        namespace=namespace, output='screen',
        parameters=[str(path), {
            'use_sim_time': use_sim_time,
            'publish_tf': publish_tf,
            'world_frame': robot['table_frame'],
            'odom_frame': robot['table_frame'],
            'map_frame': f'{namespace}/map',
            'base_link_frame': robot['body_frame'],
        }],
        remappings=[('odometry/filtered', state_topic)])


def table_tf_nodes(config, robots, logger):
    """Publish static world->table TF for calibrated tables only (hardware use)."""
    from launch_ros.actions import Node

    nodes = []
    for table in dict.fromkeys(robot['table_frame'] for robot in robots):
        pose = config['tables'].get(table, {}).get('world_pose')
        if pose is None:
            logger.warning(f'{table}: tables.{table}.world_pose is not calibrated; no '
                           f'world->{table} TF. View it with fixed_frame:={table}.')
            continue
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'{table}_world_tf', output='log',
            arguments=['--x', str(pose[0]), '--y', str(pose[1]), '--z', '0',
                       '--yaw', str(pose[2]), '--frame-id', 'world',
                       '--child-frame-id', table]))
    return nodes


def model_table_positions(model_path):
    """World XY of top-level table bodies in an MJCF model (for the RViz view)."""
    positions = {}
    try:
        root = ET.parse(model_path).getroot()
    except (ET.ParseError, OSError):
        return positions
    worldbody = root.find('worldbody')
    for body in [] if worldbody is None else worldbody.findall('body'):
        values = [float(v) for v in body.get('pos', '0 0 0').split()]
        positions[body.get('name')] = values[:2]
    return positions


def _odometry(name, topic, color, length):
    return {
        'Class': 'rviz_default_plugins/Odometry', 'Name': name, 'Enabled': True,
        'Topic': {'Value': topic, 'Reliability Policy': 'Reliable',
                  'Durability Policy': 'Volatile', 'History Policy': 'Keep Last',
                  'Depth': 1},
        'Keep': 1, 'Position Tolerance': 0, 'Angle Tolerance': 0,
        'Shape': {'Value': 'Arrow', 'Color': color, 'Shaft Length': length,
                  'Shaft Radius': 0.005, 'Head Length': 0.03, 'Head Radius': 0.015},
        'Covariance': {'Value': False},
    }


def _path(name, topic, color, latched):
    return {
        'Class': 'rviz_default_plugins/Path', 'Name': name, 'Enabled': True,
        'Topic': {'Value': topic, 'Reliability Policy': 'Reliable',
                  'Durability Policy': 'Transient Local' if latched else 'Volatile',
                  'History Policy': 'Keep Last', 'Depth': 1},
        'Line Style': 'Lines', 'Line Width': 0.01, 'Color': color, 'Alpha': 1,
        'Buffer Length': 1,
    }


def _measured_pose(topic, color):
    return {
        'Class': 'rviz_default_plugins/PoseWithCovariance', 'Name': 'Measured pose (ArUco)',
        'Enabled': True,
        'Topic': {'Value': topic, 'Reliability Policy': 'Reliable',
                  'Durability Policy': 'Volatile', 'History Policy': 'Keep Last',
                  'Depth': 5},
        'Shape': 'Arrow', 'Color': color, 'Alpha': 1, 'Shaft Length': 0.12,
        'Shaft Radius': 0.008, 'Head Length': 0.04, 'Head Radius': 0.025,
        'Covariance': {'Value': False},
    }


def _sun_marker(topic):
    return {
        'Class': 'rviz_default_plugins/Marker', 'Name': 'Sun direction from alpha (yellow)',
        'Enabled': True,
        'Topic': {'Value': topic, 'Reliability Policy': 'Reliable',
                  'Durability Policy': 'Volatile', 'History Policy': 'Keep Last', 'Depth': 5},
    }


def rviz_config(robots, *, simulation, fixed_frame='world', center=(0.5, 1.0), scale=450):
    """Build an RViz config showing each robot's target, actual path and pose."""
    displays = [{
        'Class': 'rviz_default_plugins/TF', 'Name': 'TF frames', 'Enabled': True,
        'Show Names': True, 'Show Axes': True, 'Show Arrows': False,
        'Marker Scale': 0.3, 'Update Interval': 0,
    }]
    for index, robot in enumerate(robots):
        ns = '/' + robot['namespace']
        reference, actual, measured = PALETTE[index % len(PALETTE)]
        state = robot['simulation']['state_topic'] if simulation else robot['state_topic']
        state = state if state.startswith('/') else f'{ns}/{state}'
        group = [
            {'Class': 'rviz_default_plugins/Grid', 'Name': f'{robot["table_frame"]} grid',
             'Enabled': True, 'Cell Size': 0.1, 'Plane Cell Count': 20, 'Plane': 'XY',
             'Alpha': 0.4, 'Reference Frame': robot['table_frame'],
             'Offset': {'X': 0.5, 'Y': 1, 'Z': 0}},
            _path('Reference path (target)', f'{ns}/trajectory/path', reference, True),
            _path('Actual path', f'{ns}/trajectory/actual_path', actual, False),
            _odometry('Reference pose (target)', f'{ns}/trajectory/reference', reference, 0.15),
            _odometry(f'Actual pose ({state})', state, actual, 0.20),
            _measured_pose(f'{ns}/measurements/external_pose', measured),
        ]
        if robot['hardware']['sun_sensor'] is not None:
            group.append(_sun_marker(f'{ns}/measurements/sun_direction_marker'))
        if simulation:
            group.append({
                'Class': 'rviz_default_plugins/Wrench', 'Name': 'External disturbance',
                'Enabled': True,
                'Topic': {'Value': f'{ns}/sim/disturbance/applied',
                          'Reliability Policy': 'Reliable', 'Durability Policy': 'Volatile',
                          'History Policy': 'Keep Last', 'Depth': 1},
                'Force Color': '255; 70; 70', 'Torque Color': '255; 220; 40',
                'Force Arrow Scale': 10, 'Torque Arrow Scale': 200, 'Arrow Width': 0.015,
                'History Length': 1})
        displays.append({
            'Class': 'rviz_common/Group', 'Enabled': True,
            'Name': f'{robot["namespace"]} on {robot["table_frame"]}', 'Displays': group})
    return {
        'Panels': [{'Class': 'rviz_common/Displays', 'Name': 'Displays'},
                   {'Class': 'rviz_common/Views', 'Name': 'Views'}],
        'Visualization Manager': {
            'Class': '', 'Enabled': True,
            'Global Options': {'Background Color': '35; 35; 40',
                               'Fixed Frame': fixed_frame, 'Frame Rate': 30},
            'Displays': displays,
            'Views': {'Current': {
                'Class': 'rviz_default_plugins/TopDownOrtho', 'Name': 'Top View',
                'Target Frame': fixed_frame, 'Scale': scale,
                'X': float(center[0]), 'Y': float(center[1]), 'Angle': 0}},
        },
        'Window Geometry': {'Width': 1300, 'Height': 900},
    }


def view_for(robots, table_xy, fixed_frame):
    """Center and zoom the top view on the selected tables."""
    if fixed_frame != 'world':
        return (0.5, 1.0), 450
    corners = []
    for robot in robots:
        x, y = table_xy.get(robot['table_frame'], (0.0, 0.0))
        corners += [(x, y), (x + 1.0, y + 2.0)]
    xs, ys = [c[0] for c in corners], [c[1] for c in corners]
    width = max(max(xs) - min(xs), 1.0)
    height = max(max(ys) - min(ys), 2.0)
    scale = min(450, int(1100 / (width + 0.3)), int(800 / (height + 0.3)))
    return ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2), scale


def write_rviz_config(robots, *, simulation, fixed_frame, table_xy, path=None):
    """Write a generated RViz config file and return its path."""
    center, scale = view_for(robots, table_xy, fixed_frame)
    data = rviz_config(robots, simulation=simulation, fixed_frame=fixed_frame,
                       center=center, scale=scale)
    if path is None:
        directory = Path(tempfile.mkdtemp(prefix='airtable_rviz_'))
        path = directory / 'robots.rviz'
    Path(path).write_text(yaml.safe_dump(data, sort_keys=False))
    return Path(path)


def rviz_node(robots, *, simulation, fixed_frame, table_xy, use_sim_time):
    """Start RViz with a config generated for the selected robots."""
    from launch_ros.actions import Node

    path = write_rviz_config(robots, simulation=simulation, fixed_frame=fixed_frame,
                             table_xy=table_xy)
    return Node(package='rviz2', executable='rviz2', name='robots_rviz', output='screen',
                arguments=['-d', str(path)], parameters=[{'use_sim_time': use_sim_time}])


# Live viewing and recording, shared by manual / trajectory (PC) and hardware (Pi).
RECORD_ARGUMENTS = [
    ('record', 'false', 'Record an MCAP experiment with experiment_bag (config/recording.yaml); '
                        'stop with Ctrl+C on this launch and wait for the bag to be saved'),
    ('record_label', '', 'Experiment label (letters, digits, _ . -); empty: <launch>_<plant>'),
    ('record_profile', '', 'Profile of the recording YAML; empty: sim / real (PC), hardware (Pi)'),
    ('record_note', '', 'Free-form note saved in experiment.yaml'),
    ('record_output_root', '', 'Output root; empty: recordings/ under the current directory'),
    ('record_config', '', 'Recording YAML; empty: config/recording.yaml of airtable_bringup'),
]
PC_OBSERVATION_ARGUMENTS = [
    ('foxglove', 'false', 'Start foxglove_bridge; open ws://<foxglove_address>:<foxglove_port> '
                          'in Foxglove'),
    ('foxglove_port', '8765', 'foxglove_bridge WebSocket port'),
    ('foxglove_address', '127.0.0.1', 'foxglove_bridge listen address; 0.0.0.0 lets other '
                                      'machines on the network connect'),
    ('foxglove_readonly', 'true', 'true: Foxglove only views (no publishing, parameter changes '
                                  'or service calls, so it cannot command the robot)'),
    *RECORD_ARGUMENTS,
    ('record_images', 'false', 'Also record the camera image, camera_info and, with tracking, '
                               'the ArUco result image (large: the real camera is 848x480 at '
                               '90 Hz)'),
    ('upload', 'false', 'record:=true: after the recording is saved, upload its MCAP files to '
                        'Foxglove in the background (needs foxglove auth login)'),
    ('upload_device', 'airtable-pc', 'Foxglove device name for upload:=true'),
    ('upload_project', '', 'Foxglove project ID; empty: the foxglove CLI default'),
]
# Bridge capabilities without clientPublish / parameters / services (foxglove_bridge 3.x).
FOXGLOVE_READONLY_CAPABILITIES = ['connectionGraph', 'assets']
RECORD_LABEL = r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}'


def foxglove_node(args, *, use_sim_time):
    """Return the WebSocket bridge for Foxglove; read-only unless foxglove_readonly:=false."""
    import re

    from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    from launch_ros.actions import Node

    try:
        get_package_prefix('foxglove_bridge')
    except PackageNotFoundError:
        raise ConfigError('foxglove:=true needs foxglove_bridge '
                          '(sudo apt install ros-jazzy-foxglove-bridge)') from None
    if not re.fullmatch(r'[0-9]{1,5}', args['foxglove_port']) or not \
            0 < int(args['foxglove_port']) < 65536:
        raise ConfigError(f'foxglove_port must be 1-65535, got {args["foxglove_port"]!r}')
    parameters = {'port': int(args['foxglove_port']), 'address': args['foxglove_address'],
                  'use_sim_time': use_sim_time}
    if is_true(args['foxglove_readonly']):
        parameters['capabilities'] = FOXGLOVE_READONLY_CAPABILITIES
    return Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
                output='screen', parameters=[parameters])


def record_snapshots(config, robots, *, simulation):
    """Return the files that define a run: robot configuration, parameters, model."""
    paths = [config['path']]
    if simulation and config.get('model') is not None:
        paths.append(config['model'])
    for robot in robots:
        tracking = robot.get('tracking', {})
        hardware = robot.get('hardware', {})
        sun_sensor = robot.get('simulation', {}).get('sun_sensor') if simulation \
            else hardware.get('sun_sensor')
        paths += [robot.get('trajectory_params'), robot.get('thruster_params'),
                  tracking.get('simulation_params' if simulation else 'params'),
                  None if simulation else hardware.get('params'),
                  None if simulation else hardware.get('rw_params'),
                  (sun_sensor or {}).get('params')]
    return list(dict.fromkeys(str(path) for path in paths if path is not None))


def image_topics(robots, *, simulation):
    """Camera image and camera_info of each robot's observation, from the configuration."""
    topics = []
    for robot in robots:
        tracking = robot.get('tracking', {})
        camera = tracking.get('simulation') if simulation else tracking
        if camera and camera.get('image_topic'):
            topics += [camera['image_topic'], camera['camera_info_topic']]
    return list(dict.fromkeys(topic for topic in topics if topic))


def moving_marker_topics(robots, *, simulation, name='pose'):
    """ArUco pose/result of each robot's moving marker; fixed markers are left out (adr/0023)."""
    topics = []
    for robot in robots:
        tracking = robot.get('tracking', {})
        source = tracking.get('simulation') if simulation else tracking
        if source and source.get('moving_marker_id') is not None:
            topics.append(f"/{robot['namespace']}/tracking/aruco_single_"
                          f"{source['moving_marker_id']}/{name}")
    return topics


def record_process(args, *, entry, plant, profile, robots, snapshots, topics=(), session=None):
    """
    Run experiment_bag beside the launched system (README of airtable_bringup).

    Every launch argument is saved as --meta launch.<name>=<value> and the given files as
    --snapshot. The settings are checked with --dry-run before anything starts. On Ctrl+C
    the launch forwards SIGINT; experiment_bag stops rosbag2 and waits for the MCAP to be
    finalized, so it gets more time than the default 5 s before SIGTERM.
    """
    import re
    import subprocess

    from ament_index_python.packages import get_package_prefix
    from launch.actions import ExecuteProcess

    label = args['record_label'] or f'{entry}_{plant}'
    if not re.fullmatch(RECORD_LABEL, label):
        raise ConfigError(f'record_label must be 1-80 letters, digits, _ . or -, got {label!r}')
    command = [str(Path(get_package_prefix('airtable_bringup'), 'lib', 'airtable_bringup',
                        'experiment_bag')),
               'record', '--profile', args['record_profile'] or profile, '--label', label]
    for robot in robots:
        command += ['--namespace', robot['namespace']]
    for topic in topics:
        command += ['--topic', topic]
    for option in ('note', 'output_root', 'config'):
        if args[f'record_{option}']:
            command += [f'--{option.replace("_", "-")}', args[f'record_{option}']]
    if is_true(args.get('upload', 'false')):
        command += ['--upload', '--upload-device', args['upload_device']]
        if args['upload_project']:
            command += ['--upload-project', args['upload_project']]
    command += ['--meta', f'launch.file={entry}.launch.py', '--meta', f'launch.plant={plant}']
    command += [item for name, value in args.items() if value != '' and name != 'plant'
                for item in ('--meta', f'launch.{name}={value}')]
    # Resolved conditions (not just the arguments given) for the Foxglove session.
    command += [item for name, value in (session or {}).items()
                for item in ('--meta', f'session.{name}={value}')]
    for path in snapshots:
        command += ['--snapshot', path]
    check = subprocess.run(command + ['--dry-run'], capture_output=True, text=True)
    if check.returncode:
        raise ConfigError('record:=true: ' + (check.stderr or check.stdout).strip())
    return ExecuteProcess(cmd=command, name='experiment_bag', output='screen',
                          sigterm_timeout='30', sigkill_timeout='10')


def valve_state_node(robot, *, use_sim_time):
    """Per-valve open/closed timeline from the pulse commands (scripts/valve_state.py)."""
    from launch_ros.actions import Node

    names = merge(robot['thruster_base'], robot['thruster_overrides'])['thrusters']['names']
    return Node(package='airtable_bringup', executable='valve_state',
                name='airtable_valve_state', namespace=robot['namespace'], output='screen',
                parameters=[{'use_sim_time': use_sim_time, 'thruster_names': list(names)}])


def sun_direction_node(robot, *, use_sim_time):
    """Arrow marker of the measured sun direction (scripts/sun_direction.py)."""
    from launch_ros.actions import Node

    return Node(package='airtable_bringup', executable='sun_direction',
                name='airtable_sun_direction', namespace=robot['namespace'], output='screen',
                parameters=[{'use_sim_time': use_sim_time}])


def pc_observation_actions(args, *, entry, plant, config, robots, snapshots=(), session=None):
    """Start foxglove:= and record:= of the PC entry points (PC_OBSERVATION_ARGUMENTS)."""
    simulation = plant == 'sim'
    if is_true(args['upload']) and not is_true(args['record']):
        raise ConfigError('upload:=true needs record:=true')
    actions = []
    actions += [sun_direction_node(robot, use_sim_time=simulation) for robot in robots
                if robot['hardware']['sun_sensor'] is not None]
    if is_true(args['foxglove']) or is_true(args['record']):
        # Display/recording aid only; nothing subscribes to it for control.
        actions += [valve_state_node(robot, use_sim_time=simulation) for robot in robots]
    if is_true(args['foxglove']):
        actions.append(foxglove_node(args, use_sim_time=simulation))
    if is_true(args['record']):
        topics = moving_marker_topics(robots, simulation=simulation) \
            if is_true(args['tracking']) else []
        if is_true(args['record_images']):
            topics += image_topics(robots, simulation=simulation)
            if is_true(args['tracking']):
                topics += moving_marker_topics(robots, simulation=simulation, name='result')
        actions.append(record_process(
            args, entry=entry, plant=plant, profile=plant, robots=robots, topics=topics,
            session={'plant': plant, 'launch': entry, **(session or {})},
            snapshots=[*record_snapshots(config, robots, simulation=simulation),
                       *map(str, snapshots)]))
    return actions
