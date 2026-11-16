"""Component: fixed-yaw trajectory, allocation and pulse chain of one robot (included)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def yaml_overrides(context, name):
    """Parse a YAML mapping launch argument into dotted ROS parameter names."""
    text = LaunchConfiguration(name).perform(context)
    if not text:
        return {}
    value = yaml.safe_load(text)
    if not isinstance(value, dict):
        raise ValueError(f'{name} must be a YAML mapping, got {text!r}')

    def flatten(params, prefix=''):
        flat = {}
        for key, item in params.items():
            if isinstance(item, dict):
                flat.update(flatten(item, f'{prefix}{key}.'))
            else:
                flat[f'{prefix}{key}'] = item
        return flat
    return flatten(value)


def control_nodes(context):
    """
    Apply only explicit trajectory arguments, preserving values from the YAML file.

    Precedence (lowest first): params_file, trajectory_overrides, explicit
    trajectory arguments, then reference_frame/body_frame/state_topic/enabled.
    """
    namespace = LaunchConfiguration('namespace')
    sim_time = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}
    body_frame = ParameterValue([namespace, '/base_link'], value_type=str)
    trajectory_params = LaunchConfiguration('params_file')
    thruster_params = LaunchConfiguration('thruster_params_file')
    trajectory_overrides = yaml_overrides(context, 'trajectory_overrides')
    thruster_overrides = yaml_overrides(context, 'thruster_overrides')
    overrides = {
        **trajectory_overrides,
        **sim_time,
        'body_frame': body_frame,
        'state_topic': LaunchConfiguration('state_topic'),
        'start_enabled': ParameterValue(LaunchConfiguration('enabled'), value_type=bool),
    }
    for argument, parameter in [
        ('trajectory', 'type'), ('period', 'period_s'), ('amplitude', 'amplitude_m'),
        ('radius', 'radius_m'), ('direction', 'direction_rad'), ('fixed_yaw', 'fixed_yaw_rad'),
    ]:
        value = LaunchConfiguration(argument).perform(context)
        if value:
            overrides['trajectory.' + parameter] = value if parameter == 'type' else float(value)
    center = [LaunchConfiguration(key).perform(context)
              for key in ('circle_center_x', 'circle_center_y')]
    if any(center):
        if not all(center):
            raise ValueError('Specify both circle_center_x and circle_center_y')
        overrides['trajectory.circle_center_m'] = [float(value) for value in center]
    reference_frame = LaunchConfiguration('reference_frame').perform(context)
    if reference_frame:
        overrides['reference_frame'] = reference_frame
    return [
        Node(package='airtable_controller', executable='airtable_trajectory_controller_node',
             name='airtable_trajectory_controller', namespace=namespace, output='screen',
             parameters=[trajectory_params, overrides]),
        Node(package='airtable_controller', executable='airtable_controller_node',
             name='airtable_controller', namespace=namespace, output='screen',
             parameters=[thruster_params, {
                 **thruster_overrides, **sim_time,
                 'manual_enabled': False, 'wrench_frame': body_frame}]),
        Node(package='airtable_controller', executable='airtable_pulse_modulator_node',
             name='airtable_pulse_modulator', namespace=namespace, output='screen',
             parameters=[thruster_params, thruster_overrides, trajectory_params,
                         trajectory_overrides, sim_time]),
    ]


def generate_launch_description():
    """Require external Odometry; leave output disabled by default for hardware use."""
    controller_share = FindPackageShare('airtable_controller')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('state_topic', default_value='state/odometry'),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('trajectory', default_value='', choices=['', 'line', 'circle']),
        DeclareLaunchArgument('period', default_value=''),
        DeclareLaunchArgument('amplitude', default_value=''),
        DeclareLaunchArgument('radius', default_value=''),
        DeclareLaunchArgument('direction', default_value=''),
        DeclareLaunchArgument('fixed_yaw', default_value=''),
        DeclareLaunchArgument('circle_center_x', default_value=''),
        DeclareLaunchArgument('circle_center_y', default_value=''),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            controller_share, 'config', 'trajectory.yaml'])),
        DeclareLaunchArgument('thruster_params_file', default_value=PathJoinSubstitution([
            controller_share, 'config', 'thrusters.yaml'])),
        # Empty keeps the params_file value (table_1 in the default trajectory.yaml).
        DeclareLaunchArgument('reference_frame', default_value=''),
        # YAML mappings, e.g. "{trajectory: {type: circle}}"; empty means none.
        DeclareLaunchArgument('trajectory_overrides', default_value=''),
        DeclareLaunchArgument('thruster_overrides', default_value=''),
        OpaqueFunction(function=control_nodes),
    ])
