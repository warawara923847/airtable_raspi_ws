"""SG90-HV stand-in RW: manual speed levels and yaw-hold PID, split between PC and Pi.

PC (gamepad, teleop, RW attitude node; needs ArUco tracking for PID):
  ros2 launch airtable_bringup sg90hv_attitude.launch.py start_hardware:=false
  ros2 launch airtable_bringup robot_tracking.launch.py start_camera:=true
Pi (hardware node; PWM channel prepared by scripts/pi/airtable-pwm-setup.sh):
  ros2 launch airtable_bringup sg90hv_attitude.launch.py start_control:=false \
    pwm_path:=/sys/class/pwm/pwmchip0/pwm2 [thrusters_backend:=gpio]

See docs/sg90hv-rw.md and docs/adr/0018.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def rw_attitude_node(context):
    """Resolve the controller package only on the PC; the Pi does not build it."""
    if context.launch_configurations['start_control'].lower() != 'true':
        return []
    from ament_index_python.packages import get_package_share_directory
    params = context.launch_configurations['controller_params'] or str(
        Path(get_package_share_directory('airtable_controller'), 'config',
             'rw_attitude_sg90hv.yaml'))
    return [Node(
        package='airtable_controller', executable='airtable_rw_attitude_node',
        name='airtable_rw_attitude', namespace=context.launch_configurations['namespace'],
        output='screen', parameters=[params],
    )]


def generate_launch_description():
    """Declare the PC / Pi switches; each side starts only its own nodes."""
    namespace = LaunchConfiguration('namespace')
    start_control = LaunchConfiguration('start_control')
    start_hardware = LaunchConfiguration('start_hardware')
    start_joy = LaunchConfiguration('start_joy')
    pwm_path = LaunchConfiguration('pwm_path')
    thrusters_backend = LaunchConfiguration('thrusters_backend')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='robot_1'),
        DeclareLaunchArgument('start_control', default_value='true'),
        DeclareLaunchArgument('start_hardware', default_value='true'),
        DeclareLaunchArgument('start_joy', default_value='true'),
        DeclareLaunchArgument(
            'pwm_path', default_value='',
            description='Exported PWM channel directory; empty: dummy PWM (log only)'),
        DeclareLaunchArgument(
            'thrusters_backend', default_value='dummy', choices=['dummy', 'gpio'],
            description='Valve output of the same hardware node (gpio: hardware_pi.yaml lines)'),
        DeclareLaunchArgument(
            'controller_params', default_value='',
            description='Gains and manual levels of airtable_rw_attitude_node; '
                        'empty: airtable_controller/config/rw_attitude_sg90hv.yaml'),
        DeclareLaunchArgument(
            'hardware_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('airtable_bringup'), 'config', 'sg90hv.yaml',
            ]),
            description='Servo calibration for airtable_hardware'),
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
        OpaqueFunction(function=rw_attitude_node),
        Node(
            package='airtable_hardware', executable='airtable_hardware',
            name='airtable_hardware', namespace=namespace, output='screen',
            condition=IfCondition(start_hardware),
            parameters=[LaunchConfiguration('hardware_params'), {
                'rw.pwm_path': ParameterValue(pwm_path, value_type=str),
                'thrusters.backend': ParameterValue(thrusters_backend, value_type=str),
            }],
        ),
    ])
