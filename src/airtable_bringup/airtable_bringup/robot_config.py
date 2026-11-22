"""
Load, validate and resolve single/multi-robot launch configurations.

A *robot configuration* (``config/robots/*.yaml``) selects which vehicles exist,
which table each one belongs to and which ROS parameter files feed its nodes.
It is not itself a ROS parameter file; ``params_file`` launch arguments keep
pointing at ``ros__parameters`` YAML.

Path rules (any field ending in ``params``/``model``):

* ``package://<package>/<path>`` resolves to ``<share of package>/<path>``.
* An absolute path (``~`` is expanded) is used as is.
* A relative path is relative to the directory containing the robot
  configuration file, never to the launch working directory.

Parameter precedence for each robot, lowest first:

1. the referenced ``trajectory_params`` / ``thruster_params`` file,
2. ``trajectory_overrides`` / ``thruster_overrides`` of that robot,
3. explicit launch arguments (``trajectory:=`` etc.), applied to every
   selected robot,
4. values derived by the launch itself: ``reference_frame`` (from
   ``table_frame``), ``body_frame`` (``<namespace>/base_link``), state topic,
   ``start_enabled`` and ``use_sim_time``.
"""

import copy
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import yaml


SCHEMA_VERSION = 1
NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')
TOPIC = re.compile(r'^~?/?[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*$')
DEFAULT_TRAJECTORY = 'package://airtable_controller/config/trajectory.yaml'
DEFAULT_THRUSTERS = 'package://airtable_controller/config/thrusters.yaml'
# Local actuator/sensor names the simulator expects under each vehicle prefix.
SIM_THRUSTERS = ('thruster_1', 'thruster_2', 'thruster_3', 'thruster_4')
SIM_ELEMENTS = ('reaction_wheel_1_actuator',)
SIM_SENSORS = ('imu_1_accel', 'imu_1_gyro', 'imu_1_orientation')

TOP_KEYS = {'version', 'description', 'simulation', 'tables', 'defaults', 'robots'}
SIMULATION_KEYS = {'model'}
TABLE_KEYS = {'world_pose', 'note'}
DEFAULT_KEYS = {'trajectory_params', 'thruster_params'}
ROBOT_KEYS = {'namespace', 'table_frame', 'trajectory_params', 'thruster_params',
              'trajectory_overrides', 'thruster_overrides', 'state_topic',
              'simulation', 'tracking', 'hardware'}
ROBOT_SIM_KEYS = {'element_prefix', 'initial_pose', 'state_topic'}
TRACKING_KEYS = {'params', 'simulation_params', 'simulation_camera', 'camera_frame',
                 'image_topic', 'camera_info_topic', 'marker_size', 'camera_serial_no'}
# MJCF camera of the original single-table model; keeps the /sim/camera topics.
LEGACY_SIM_CAMERA = 'tracking_camera'
# Black edge of the ArUco cards in the MJCF models (markers/card.obj).
SIM_MARKER_SIZE = 0.1
HARDWARE_KEYS = {'params', 'imu_frame', 'imu_mount', 'rw', 'rw_params', 'sun_sensor'}
SUN_SENSOR_KEYS = {'params', 'frame', 'mount'}
SUN_SENSOR_DEFAULT_PARAMS = 'package://airtable_bringup/config/nanossoc.yaml'
# Reaction wheel fitted to the real robot. It selects both the Pi's airtable_hardware
# parameters (rw_params, layered over params) and the PC's manual RW control:
#   none   - no wheel
#   arw3m  - torque-commanded ARW-3m with telemetry (airtable_controller_node speed loop)
#   sg90hv - continuous-rotation servo, speed-commanded (airtable_rw_attitude_node)
# The simulator always models a torque wheel, so this does not affect plant:=sim.
RW_TYPES = ('none', 'arw3m', 'sg90hv')
RW_DEFAULT_PARAMS = {
    'arw3m': 'package://airtable_bringup/config/arw3m.yaml',
    'sg90hv': 'package://airtable_bringup/config/sg90hv.yaml',
}


class ConfigError(ValueError):
    """A robot configuration that must be fixed before launching anything."""


def _mapping(value, where, allowed=None, required=()):
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ConfigError(f'{where} must be a mapping')
    if allowed is not None:
        unknown = sorted(set(value) - set(allowed))
        if unknown:
            raise ConfigError(f'{where}: unknown keys {unknown}; allowed {sorted(allowed)}')
    missing = [key for key in required if value.get(key) in (None, '')]
    if missing:
        raise ConfigError(f'{where}: missing required keys {missing}')
    return value


def _name(value, where):
    if not isinstance(value, str) or not NAME.match(value):
        raise ConfigError(f'{where} must be a ROS name segment like robot_1, got {value!r}')
    return value


def _topic(value, where):
    if not isinstance(value, str) or not TOPIC.match(value):
        raise ConfigError(f'{where} must be a ROS topic name, got {value!r}')
    return value


def _finite_list(value, length, where):
    if (not isinstance(value, list) or len(value) != length or
            not all(isinstance(item, (int, float)) and not isinstance(item, bool)
                    for item in value)):
        raise ConfigError(f'{where} must be a list of {length} numbers, got {value!r}')
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ConfigError(f'{where} must be finite, got {value!r}')
    return result


def package_share(package):
    """Return a package share directory or raise a configuration error."""
    from ament_index_python.packages import (
        get_package_share_directory, PackageNotFoundError)
    try:
        return Path(get_package_share_directory(package))
    except PackageNotFoundError as error:
        raise ConfigError(f'package://{package}: package not found; source install/setup.bash'
                          ) from error


def resolve_path(value, base_dir, where, share=package_share, must_exist=True):
    """Resolve package://, absolute or configuration-relative paths."""
    if not isinstance(value, str) or not value.strip():
        raise ConfigError(f'{where} must be a non-empty path string')
    if value.startswith('package://'):
        package, _, relative = value[len('package://'):].partition('/')
        if not package or not relative:
            raise ConfigError(f'{where}: use package://<package>/<path>, got {value!r}')
        path = Path(share(package)) / relative
    else:
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = Path(base_dir) / path
    path = path.resolve()
    if must_exist and not path.is_file():
        raise ConfigError(f'{where}: file not found: {path} (from {value!r})')
    return path


def ros_parameters(path, where):
    """Read the wildcard ``/**: ros__parameters`` section used by these launches."""
    try:
        data = yaml.safe_load(Path(path).read_text())
    except yaml.YAMLError as error:
        raise ConfigError(f'{where}: invalid YAML in {path}: {error}') from error
    try:
        params = data['/**']['ros__parameters']
    except (TypeError, KeyError) as error:
        raise ConfigError(f'{where}: {path} must contain /**: ros__parameters') from error
    if not isinstance(params, dict):
        raise ConfigError(f'{where}: {path} ros__parameters must be a mapping')
    return params


def flatten(params, prefix=''):
    """Flatten nested ROS parameters to dotted names."""
    flat = {}
    for key, value in params.items():
        name = f'{prefix}{key}'
        if isinstance(value, dict):
            flat.update(flatten(value, name + '.'))
        else:
            flat[name] = value
    return flat


def merge(base, overrides):
    """Deep-merge overrides into a copy of base."""
    result = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _check_overrides(overrides, base, where):
    """Reject typos: every override must name a parameter of the referenced file."""
    known = flatten(base)
    for name, value in flatten(overrides).items():
        if name not in known:
            raise ConfigError(f'{where}.{name} is not a parameter of the referenced file')
        if type(value) is not type(known[name]) and not (
                isinstance(value, (int, float)) and isinstance(known[name], float)
                and not isinstance(value, bool)):
            raise ConfigError(f'{where}.{name}: type {type(value).__name__} differs from '
                              f'{type(known[name]).__name__} in the referenced file')


def _sun_sensor(spec, namespace, where, resolve):
    """Resolve ``hardware.sun_sensor``; absent or null means no sensor on this robot."""
    if spec is None:
        return None
    where = f'{where}.hardware.sun_sensor'
    spec = _mapping(spec, where, SUN_SENSOR_KEYS)
    frame = spec.get('frame', f'{namespace}/sun_sensor_link')
    if not isinstance(frame, str) or not frame:
        raise ConfigError(f'{where}.frame must be a non-empty string')
    mount = spec.get('mount')
    return {
        'params': resolve(spec.get('params') or SUN_SENSOR_DEFAULT_PARAMS, f'{where}.params'),
        'frame': frame,
        'mount': None if mount is None else _finite_list(mount, 6, f'{where}.mount'),
    }


def _hardware(hardware, namespace, where, resolve):
    """Resolve one robot's ``hardware`` section."""
    imu_mount = hardware.get('imu_mount')
    imu_frame = hardware.get('imu_frame', f'{namespace}/imu_link')
    if not isinstance(imu_frame, str) or not imu_frame:
        raise ConfigError(f'{where}.hardware.imu_frame must be a non-empty string')
    rw = hardware.get('rw', 'none')
    if rw not in RW_TYPES:
        raise ConfigError(f'{where}.hardware.rw must be one of {list(RW_TYPES)}, got {rw!r}')
    rw_params = hardware.get('rw_params')
    if rw == 'none' and rw_params is not None:
        raise ConfigError(f'{where}.hardware.rw_params needs hardware.rw')
    if rw_params is None:
        rw_params = RW_DEFAULT_PARAMS.get(rw)
    return {
        'params': None if hardware.get('params') is None else resolve(
            hardware['params'], f'{where}.hardware.params'),
        'imu_frame': imu_frame,
        'imu_mount': None if imu_mount is None else _finite_list(
            imu_mount, 6, f'{where}.hardware.imu_mount'),
        'rw': rw,
        'rw_params': None if rw_params is None else resolve(
            rw_params, f'{where}.hardware.rw_params'),
        'sun_sensor': _sun_sensor(hardware.get('sun_sensor'), namespace, where, resolve),
    }


def load_robot_config(path, share=package_share, hardware_only=False):
    """
    Load and strictly validate a robot configuration; return resolved data.

    ``path`` is absolute, relative to the current directory, or
    ``package://<package>/<path>`` (e.g. package://airtable_bringup/config/robots/dual_robot.yaml).

    ``hardware_only`` is for the Raspberry Pi, which installs only airtable_hardware and
    airtable_bringup: keys are still checked, but the simulation model and the
    controller/tracking parameter files (PC-only packages) are not resolved, and each
    robot carries only namespace, table_frame, body_frame and hardware.
    """
    text = str(path)
    if text.startswith('package://'):
        path = resolve_path(text, Path.cwd(), 'robot_config', share, must_exist=False)
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise ConfigError(
            f'robot configuration not found: {path} (from {text!r}). Relative paths are '
            'relative to the current directory; use an absolute path or '
            'package://airtable_bringup/config/robots/<name>.yaml')
    try:
        raw = yaml.safe_load(path.read_text())
    except yaml.YAMLError as error:
        raise ConfigError(f'{path}: invalid YAML: {error}') from error
    raw = _mapping(raw, str(path), TOP_KEYS, ['version', 'robots'])
    if raw['version'] != SCHEMA_VERSION:
        raise ConfigError(f'{path}: version must be {SCHEMA_VERSION}')
    base = path.parent

    def resolve(value, where, must_exist=True):
        return resolve_path(value, base, where, share, must_exist)

    simulation = _mapping(raw.get('simulation'), 'simulation', SIMULATION_KEYS)
    model = resolve(simulation['model'], 'simulation.model') if simulation.get(
        'model') is not None and not hardware_only else None
    tables = {}
    for table, spec in _mapping(raw.get('tables'), 'tables').items():
        _name(table, 'tables key')
        spec = _mapping(spec, f'tables.{table}', TABLE_KEYS)
        pose = spec.get('world_pose')
        tables[table] = {
            'world_pose': None if pose is None else _finite_list(
                pose, 3, f'tables.{table}.world_pose'),
        }
    defaults = _mapping(raw.get('defaults'), 'defaults', DEFAULT_KEYS)
    robots_raw = raw['robots']
    if not isinstance(robots_raw, list) or not robots_raw:
        raise ConfigError('robots must be a non-empty list')
    robots, seen_tables, seen_prefixes = [], {}, {}
    for index, entry in enumerate(robots_raw):
        where = f'robots[{index}]'
        entry = _mapping(entry, where, ROBOT_KEYS, ['namespace', 'table_frame'])
        namespace = _name(entry['namespace'], f'{where}.namespace')
        where = f'robots[{namespace}]'
        if any(robot['namespace'] == namespace for robot in robots):
            raise ConfigError(f'duplicate robot namespace {namespace!r}')
        table = _name(entry['table_frame'], f'{where}.table_frame')
        if tables and table not in tables:
            raise ConfigError(f'{where}.table_frame {table!r} is not listed in tables '
                              f'{sorted(tables)}')
        if table in seen_tables:
            raise ConfigError(f'{where}: table {table!r} is already used by '
                              f'{seen_tables[table]}; one robot per table is supported')
        seen_tables[table] = namespace
        if hardware_only:
            robots.append({
                'namespace': namespace,
                'table_frame': table,
                'body_frame': f'{namespace}/base_link',
                'hardware': _hardware(_mapping(entry.get('hardware'), f'{where}.hardware',
                                               HARDWARE_KEYS), namespace, where, resolve),
            })
            continue
        trajectory_path = resolve(entry.get('trajectory_params') or defaults.get(
            'trajectory_params') or DEFAULT_TRAJECTORY, f'{where}.trajectory_params')
        thruster_path = resolve(entry.get('thruster_params') or defaults.get(
            'thruster_params') or DEFAULT_THRUSTERS, f'{where}.thruster_params')
        trajectory = ros_parameters(trajectory_path, f'{where}.trajectory_params')
        thrusters = ros_parameters(thruster_path, f'{where}.thruster_params')
        trajectory_overrides = _mapping(entry.get('trajectory_overrides'),
                                        f'{where}.trajectory_overrides')
        thruster_overrides = _mapping(entry.get('thruster_overrides'),
                                      f'{where}.thruster_overrides')
        for derived in ('reference_frame', 'body_frame', 'state_topic', 'start_enabled'):
            if derived in trajectory_overrides:
                raise ConfigError(f'{where}.trajectory_overrides.{derived} is derived by the '
                                  'launch; set table_frame/state_topic or launch arguments')
        _check_overrides(trajectory_overrides, trajectory, f'{where}.trajectory_overrides')
        _check_overrides(thruster_overrides, thrusters, f'{where}.thruster_overrides')
        sim = _mapping(entry.get('simulation'), f'{where}.simulation', ROBOT_SIM_KEYS)
        prefix = sim.get('element_prefix', f'{namespace}/')
        if not isinstance(prefix, str):
            raise ConfigError(f'{where}.simulation.element_prefix must be a string')
        if prefix in seen_prefixes:
            raise ConfigError(f'{where}.simulation.element_prefix {prefix!r} duplicates '
                              f'{seen_prefixes[prefix]}')
        seen_prefixes[prefix] = namespace
        initial = sim.get('initial_pose')
        tracking = _mapping(entry.get('tracking'), f'{where}.tracking', TRACKING_KEYS)
        hardware = _mapping(entry.get('hardware'), f'{where}.hardware', HARDWARE_KEYS)
        tracking_resolved = {
            key: tracking.get(key) for key in TRACKING_KEYS - {
                'params', 'simulation_params', 'simulation_camera'}}
        for key in ('image_topic', 'camera_info_topic'):
            if tracking_resolved[key] is not None:
                _topic(tracking_resolved[key], f'{where}.tracking.{key}')
        for key in ('params', 'simulation_params'):
            tracking_resolved[key] = None if tracking.get(key) is None else resolve(
                tracking[key], f'{where}.tracking.{key}')
        if tracking_resolved['params'] is not None:
            observed = ros_parameters(tracking_resolved['params'], f'{where}.tracking.params')
            if observed.get('table_frame') != table:
                raise ConfigError(f'{where}.tracking.params table_frame '
                                  f'{observed.get("table_frame")!r} differs from {table!r}')
            tracking_resolved['marker_ids'] = [
                int(item) for item in observed.get('fixed_marker_ids', [])]
            tracking_resolved['moving_marker_id'] = int(observed.get('moving_marker_id'))
        tracking_resolved['simulation'] = None
        if tracking_resolved['simulation_params'] is not None:
            tracking_resolved['simulation'] = _simulation_tracking(
                tracking_resolved['simulation_params'],
                tracking.get('simulation_camera', LEGACY_SIM_CAMERA), table, where)
        elif tracking.get('simulation_camera') is not None:
            raise ConfigError(f'{where}.tracking.simulation_camera needs simulation_params')
        robots.append({
            'namespace': namespace,
            'table_frame': table,
            'body_frame': f'{namespace}/base_link',
            'trajectory_params': trajectory_path,
            'thruster_params': thruster_path,
            'trajectory_base': trajectory,
            'thruster_base': thrusters,
            'trajectory_overrides': trajectory_overrides,
            'thruster_overrides': thruster_overrides,
            'state_topic': _topic(entry.get('state_topic', 'state/odometry'),
                                  f'{where}.state_topic'),
            'simulation': {
                'element_prefix': prefix,
                'initial_pose': None if initial is None else _finite_list(
                    initial, 3, f'{where}.simulation.initial_pose'),
                'state_topic': _topic(sim.get('state_topic', 'sim/ground_truth'),
                                      f'{where}.simulation.state_topic'),
            },
            'tracking': tracking_resolved,
            'hardware': _hardware(hardware, namespace, where, resolve),
        })
    if hardware_only:
        return {'path': path, 'model': None, 'tables': tables, 'robots': robots}
    _check_tracking_ids(robots)
    for robot in robots:
        effective_trajectory(robot)  # validate merged bounds/trajectory early
    return {'path': path, 'model': model, 'tables': tables, 'robots': robots}


def simulation_camera_names(camera):
    """
    Topic namespace and optical frame the simulator uses for one MJCF camera.

    - tracking_camera -> /sim/camera, sim_camera_infra1_optical_frame (original)
    - table_2/tracking_camera -> /sim/table_2/camera, sim_table_2_camera_infra1_optical_frame
    """
    scope = camera.rsplit('/', 1)[0] if '/' in camera else ''
    if camera != LEGACY_SIM_CAMERA and not scope:
        scope = camera
    namespace = '/sim/' + (f'{scope}/' if scope else '') + 'camera'
    frame = 'sim_' + (scope.replace('/', '_') + '_' if scope else '') + \
        'camera_infra1_optical_frame'
    return namespace, frame


def _simulation_tracking(path, camera, table, where):
    if not isinstance(camera, str) or not camera or camera.startswith('/'):
        raise ConfigError(f'{where}.tracking.simulation_camera must be an MJCF camera name')
    observed = ros_parameters(path, f'{where}.tracking.simulation_params')
    if observed.get('table_frame') != table:
        raise ConfigError(f'{where}.tracking.simulation_params table_frame '
                          f'{observed.get("table_frame")!r} differs from {table!r}')
    ids = [int(item) for item in observed.get('fixed_marker_ids', [])]
    positions = list(observed.get('fixed_marker_positions', []))
    if len(positions) != 3 * len(ids):
        raise ConfigError(f'{where}.tracking.simulation_params: fixed_marker_positions must '
                          'hold [x, y, z] per fixed_marker_ids entry')
    namespace, frame = simulation_camera_names(camera)
    return {
        'camera': camera,
        'camera_frame': frame,
        'image_topic': f'{namespace}/infra1/image_rect_raw',
        'camera_info_topic': f'{namespace}/infra1/camera_info',
        'namespace': namespace,
        'marker_size': SIM_MARKER_SIZE,
        'marker_ids': ids,
        'marker_positions': [positions[3 * i:3 * i + 3] for i in range(len(ids))],
        'moving_marker_id': int(observed.get('moving_marker_id')),
    }


def _check_tracking_ids(robots):
    """Require unique moving IDs and no shared IDs between robots sharing an image."""
    moving = {}
    for robot in robots:
        tracking = robot['tracking']
        if tracking.get('params') is None:
            continue
        mid = tracking['moving_marker_id']
        if mid in moving:
            raise ConfigError(f'moving marker ID {mid} is used by {moving[mid]} and '
                              f'{robot["namespace"]}')
        moving[mid] = robot['namespace']
    for index, first in enumerate(robots):
        for second in robots[index + 1:]:
            a, b = first['tracking'], second['tracking']
            if a.get('params') is None or b.get('params') is None:
                continue
            if a.get('image_topic') and a.get('image_topic') == b.get('image_topic'):
                common = set(a['marker_ids'] + [a['moving_marker_id']]) & set(
                    b['marker_ids'] + [b['moving_marker_id']])
                if common:
                    raise ConfigError(f'{first["namespace"]} and {second["namespace"]} share '
                                      f'{a["image_topic"]} and marker IDs {sorted(common)}')
    # Simulation: one camera per table, distinct moving markers.
    cameras, moving = {}, {}
    for robot in robots:
        sim = robot['tracking'].get('simulation')
        if sim is None:
            continue
        for seen, key, label in ((cameras, sim['camera'], 'simulation_camera'),
                                 (moving, sim['moving_marker_id'], 'simulation moving ID')):
            if key in seen:
                raise ConfigError(f'{label} {key!r} is used by {seen[key]} and '
                                  f'{robot["namespace"]}')
            seen[key] = robot['namespace']


def parse_selection(text):
    """Parse a comma/space-separated robot list; empty means all robots."""
    names = [item for item in re.split(r'[,\s]+', text or '') if item]
    if len(names) != len(set(names)):
        raise ConfigError(f'robots selection has duplicates: {names}')
    return names


def select_robots(config, selection):
    """Return selected robots in configuration order, rejecting unknown names."""
    names = parse_selection(selection) if isinstance(selection, str) else list(selection)
    available = [robot['namespace'] for robot in config['robots']]
    unknown = [name for name in names if name not in available]
    if unknown:
        raise ConfigError(f'unknown robots {unknown}; {config["path"]} defines {available}')
    if not names:
        return list(config['robots'])
    return [robot for robot in config['robots'] if robot['namespace'] in names]


TRAJECTORY_ARGUMENTS = {
    'trajectory': ('trajectory', 'type'), 'period': ('trajectory', 'period_s'),
    'amplitude': ('trajectory', 'amplitude_m'), 'radius': ('trajectory', 'radius_m'),
    'direction': ('trajectory', 'direction_rad'), 'fixed_yaw': ('trajectory', 'fixed_yaw_rad'),
}


# Launch arguments [s] for top-level parameters shared by the controller and pulse modulator.
PULSE_ARGUMENTS = {'pulse_duration': 'pulse_duration_s', 'pulse_period': 'pulse_period_s'}


def launch_trajectory_overrides(arguments):
    """Convert non-empty trajectory launch arguments to nested parameter overrides."""
    overrides = {}
    for argument, parameter in PULSE_ARGUMENTS.items():
        if arguments.get(argument, ''):
            overrides[parameter] = float(arguments[argument])
    for argument, (group, parameter) in TRAJECTORY_ARGUMENTS.items():
        value = arguments.get(argument, '')
        if value:
            overrides.setdefault(group, {})[parameter] = (
                value if parameter == 'type' else float(value))
    center = [arguments.get('circle_center_x', ''), arguments.get('circle_center_y', '')]
    if any(center):
        if not all(center):
            raise ConfigError('Specify both circle_center_x and circle_center_y')
        overrides.setdefault('trajectory', {})['circle_center_m'] = [float(v) for v in center]
    return overrides


def effective_trajectory(robot, launch_overrides=None):
    """Merge trajectory controller parameters (file < robot overrides < launch)."""
    params = merge(robot['trajectory_base'], robot['trajectory_overrides'])
    params = merge(params, launch_overrides or {})
    where = f'robots[{robot["namespace"]}]'
    trajectory = params.get('trajectory', {})
    if trajectory.get('type', 'line') not in ('line', 'circle'):
        raise ConfigError(f'{where}: trajectory.type must be line or circle')
    radius = trajectory.get('radius_m', 0.1)
    if not isinstance(radius, (int, float)) or not math.isfinite(radius) or radius <= 0:
        raise ConfigError(f'{where}: trajectory.radius_m must be finite and positive')
    center = trajectory.get('circle_center_m', [])
    if center:
        _finite_list(center, 2, f'{where}: trajectory.circle_center_m')
    width, period = pulse_timing(params)
    if not (math.isfinite(width) and math.isfinite(period) and 0.002 <= width <= 0.5 and
            width <= period <= 1.0):
        raise ConfigError(f'{where}: need 0.002 <= pulse_duration_s ({width}) <= '
                          f'pulse_period_s ({period}) <= 1 s')
    bounds = params.get('limits', {}).get('position_bounds_m')
    if bounds is not None:
        bounds = _finite_list(bounds, 4, f'{where}: limits.position_bounds_m')
        if bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
            raise ConfigError(f'{where}: limits.position_bounds_m must be [xmin,xmax,ymin,ymax]')
    return params


def pulse_timing(params):
    """Pulse width and period [s] of effective trajectory parameters (period: width)."""
    width = float(params.get('pulse_duration_s', 0.01))
    return width, float(params.get('pulse_period_s', width))


def pulse_thrust_caps(robot, params):
    """
    Lower max_thrust_n to the average a pulse train can deliver: on_thrust_n * width/period.

    Returns thruster overrides for the allocator and modulator (empty when nothing is capped),
    so the controller never asks for more than one pulse per period.
    """
    width, period = pulse_timing(params)
    thrusters = merge(robot['thruster_base'], robot['thruster_overrides'])['thrusters']
    caps = {}
    for name in thrusters['names']:
        spec = thrusters[name]
        maximum = float(spec.get('max_thrust_n', 0.5))
        limit = float(spec.get('on_thrust_n', maximum)) * min(1.0, width / period)
        if maximum > limit + 1e-12:
            caps[name] = {'max_thrust_n': limit}
    return {'thrusters': caps} if caps else {}


def initial_pose(robot, params, arguments=None):
    """Table-local start pose: launch args > robot YAML > circle edge > MJCF default."""
    arguments = arguments or {}
    where = f'robots[{robot["namespace"]}]'
    xy = [arguments.get('initial_x', ''), arguments.get('initial_y', '')]
    yaw_arg = arguments.get('initial_yaw', '')
    trajectory = params.get('trajectory', {})
    pose = None
    if any(xy):
        if not all(xy):
            raise ConfigError('Specify both initial_x and initial_y')
        pose = [float(xy[0]), float(xy[1]), float(yaw_arg or 0.0)]
    elif robot['simulation']['initial_pose'] is not None:
        pose = list(robot['simulation']['initial_pose'])
        if yaw_arg:
            pose[2] = float(yaw_arg)
    elif trajectory.get('type', 'line') == 'circle' and trajectory.get('circle_center_m'):
        center = trajectory['circle_center_m']
        offset = _finite_list(trajectory.get('offset_m', [0.0, 0.0]), 2,
                              f'{where}: trajectory.offset_m')
        pose = [center[0] + offset[0] + float(trajectory.get('radius_m', 0.1)),
                center[1] + offset[1], float(yaw_arg or 0.0)]
    elif yaw_arg:
        raise ConfigError('Set initial_x and initial_y when overriding a line initial yaw')
    if pose is None:
        return None
    if not all(math.isfinite(value) for value in pose):
        raise ConfigError(f'{where}: initial pose must be finite, got {pose}')
    bounds = params.get('limits', {}).get('position_bounds_m')
    if bounds is not None and not (bounds[0] <= pose[0] <= bounds[1]
                                   and bounds[2] <= pose[1] <= bounds[3]):
        raise ConfigError(f'{where}: initial pose {pose[:2]} is outside '
                          f'position_bounds_m {bounds} of {robot["table_frame"]}')
    return pose


def check_simulation_model(model_path, robots):
    """Check that each selected vehicle, its table and prefixed elements exist in MJCF."""
    try:
        root = ET.parse(model_path).getroot()
    except (ET.ParseError, OSError) as error:
        raise ConfigError(f'cannot parse MuJoCo model {model_path}: {error}') from error
    if root.find('.//include') is not None:
        raise ConfigError(f'{model_path}: <include> is not supported by the launch check')
    parents = {child: parent for parent in root.iter() for child in parent}
    bodies = {body.get('name'): body for body in root.iter('body')}
    names = {}
    for tag in ('actuator', 'sensor'):
        for section in root.findall(tag):
            for element in section:
                names[element.get('name')] = element.tag
    for robot in robots:
        namespace = robot['namespace']
        body = bodies.get(namespace)
        if body is None:
            raise ConfigError(f'{model_path}: vehicle body {namespace!r} not found; '
                              'choose the matching simulation.model or robots selection')
        parent = parents.get(body)
        if parent is None or parent.tag != 'body' or parent.get('name') != robot['table_frame']:
            actual = None if parent is None else parent.get('name')
            raise ConfigError(f'{model_path}: {namespace} is attached to {actual!r}, but the '
                              f'configuration says table_frame {robot["table_frame"]!r}')
        if parents.get(parent) is None or parents[parent].tag != 'worldbody':
            raise ConfigError(f'{model_path}: table {robot["table_frame"]} must be a '
                              'direct child of worldbody')
        prefix = robot['simulation']['element_prefix']
        thrusters = merge(robot['thruster_base'], robot['thruster_overrides'])['thrusters']
        if list(thrusters['names']) != list(SIM_THRUSTERS):
            raise ConfigError(f'robots[{namespace}]: simulator supports thrusters '
                              f'{list(SIM_THRUSTERS)}, configured {thrusters["names"]}')
        required = [f'{prefix}{name}_actuator' for name in SIM_THRUSTERS] + [
            prefix + name for name in SIM_ELEMENTS + SIM_SENSORS]
        missing = [name for name in required if name not in names]
        if missing:
            raise ConfigError(f'{model_path}: {namespace} (element_prefix {prefix!r}) is '
                              f'missing actuators/sensors {missing}')


def check_simulation_cameras(model_path, robots):
    """
    Check each tracked robot's camera, cards and IDs against the MJCF model.

    The camera must sit on the robot's own table; every fixed ID needs a card
    (material marker_<id>) on that table, outside the vehicle, at the configured
    table-frame position; the moving ID needs a card on the vehicle body.
    """
    try:
        root = ET.parse(model_path).getroot()
    except (ET.ParseError, OSError) as error:
        raise ConfigError(f'cannot parse MuJoCo model {model_path}: {error}') from error
    parents = {child: parent for parent in root.iter() for child in parent}
    bodies = {body.get('name'): body for body in root.iter('body')}
    cameras = {camera.get('name'): camera for camera in root.iter('camera')}
    for robot in robots:
        sim = robot['tracking'].get('simulation')
        namespace, table = robot['namespace'], robot['table_frame']
        if sim is None:
            raise ConfigError(f'robots[{namespace}]: tracking.simulation_params is not set')
        camera = cameras.get(sim['camera'])
        if camera is None:
            raise ConfigError(f'{model_path}: camera {sim["camera"]!r} for {namespace} not '
                              'found; tracking needs a model with cameras')
        if parents[camera].get('name') != table:
            raise ConfigError(f'{model_path}: camera {sim["camera"]!r} is on '
                              f'{parents[camera].get("name")!r}, not {namespace}\'s {table}')
        vehicle = bodies[namespace]
        on_vehicle = set(vehicle.iter())
        cards = {}
        for geom in bodies[table].iter('geom'):
            material = geom.get('material') or ''
            if material.startswith('marker_'):
                cards.setdefault(material, []).append((geom, geom in on_vehicle))
        for mid, expected in zip(sim['marker_ids'], sim['marker_positions']):
            fixed = [geom for geom, moving in cards.get(f'marker_{mid}', []) if not moving]
            if len(fixed) != 1 or parents[fixed[0]] is not bodies[table]:
                raise ConfigError(f'{model_path}: {table} needs exactly one fixed card '
                                  f'marker_{mid} directly on the table')
            actual = [float(v) for v in fixed[0].get('pos', '0 0 0').split()]
            if any(abs(a - b) > 1e-6 for a, b in zip(actual, expected)):
                raise ConfigError(f'{model_path}: marker_{mid} on {table} is at {actual}, '
                                  f'simulation_params say {expected}')
        mid = sim['moving_marker_id']
        if not any(moving for _, moving in cards.get(f'marker_{mid}', [])):
            raise ConfigError(f'{model_path}: {namespace} carries no card marker_{mid}')


def yaml_argument(value):
    """Serialize a parameter mapping for a string launch argument."""
    return yaml.safe_dump(value, default_flow_style=True, width=1 << 20).strip()


def parse_yaml_argument(text, where):
    """Parse a launch argument produced by yaml_argument; empty means no overrides."""
    if not text:
        return {}
    try:
        value = yaml.safe_load(text)
    except yaml.YAMLError as error:
        raise ConfigError(f'{where}: invalid YAML mapping: {error}') from error
    return flatten(_mapping(value, where))


def summary(config, robots):
    """Human-readable, reproducible summary of what will be launched."""
    lines = [f'robot_config={config["path"]}', f'model={config["model"]}']
    for robot in robots:
        lines.append(
            f'{robot["namespace"]}: table={robot["table_frame"]}, '
            f'trajectory={robot["trajectory_params"]}, thrusters={robot["thruster_params"]}, '
            f'trajectory_overrides={robot["trajectory_overrides"]}, '
            f'thruster_overrides={robot["thruster_overrides"]}')
    return '\n'.join(lines)
