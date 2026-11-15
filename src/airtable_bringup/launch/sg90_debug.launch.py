"""Run a debug SG90 RW locally, or split control and hardware between PC and Pi."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    start_control = LaunchConfiguration('start_control')
    start_hardware = LaunchConfiguration('start_hardware')
    start_joy = LaunchConfiguration('start_joy')
    backend = LaunchConfiguration('backend')
    pwm_path = LaunchConfiguration('pwm_path')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('start_control', default_value='true'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('start_joy', default_value='true'),
        DeclareLaunchArgument('backend', default_value='dummy', choices=['dummy', 'sysfs']),
        DeclareLaunchArgument('pwm_path', default_value=''),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('airtable_bringup'), 'config', 'sg90_debug.yaml',
            ]),
        ),
        Node(
            package='joy', executable='game_controller_node',
            name='game_controller_node', output='screen',
            condition=IfCondition(PythonExpression([
                "'", start_control, "'.lower() == 'true' and '", start_joy,
                "'.lower() == 'true'",
            ])),
        ),
        Node(
            package='airtable_teleop', executable='airtable_teleop_node',
            name='airtable_teleop', namespace=namespace, output='screen',
            condition=IfCondition(start_control),
            parameters=[PathJoinSubstitution([
                FindPackageShare('airtable_teleop'), 'config', 'teleop.yaml',
            ])],
        ),
        Node(
            package='airtable_controller', executable='airtable_controller_node',
            name='airtable_controller', namespace=namespace, output='screen',
            condition=IfCondition(start_control),
            parameters=[PathJoinSubstitution([
                FindPackageShare('airtable_controller'), 'config', 'thrusters.yaml',
            ]), {
                # Stable for virtual I=0.001 kg m² and 50 Hz state feedback.
                'reaction_wheels.rw_1.speed_kp': 0.02,
                'manual.reaction_wheel.speed_step_rad_s': 5.0,
            }],
        ),
        Node(
            package='airtable_hardware', executable='airtable_hardware',
            name='airtable_hardware', namespace=namespace, output='screen',
            condition=IfCondition(start_hardware),
            parameters=[params_file, {
                'rw.backend': ParameterValue(backend, value_type=str),
                'rw.pwm_path': ParameterValue(pwm_path, value_type=str),
            }],
        ),
    ])
