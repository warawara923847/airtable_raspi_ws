"""Launch manual simulation control with optional rendered-camera tracking."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start one simulator, the control chain, and optional marker tracking."""
    tereop_params = PathJoinSubstitution(
        [
            FindPackageShare('airtable_teleop'),
            'config',
            'teleop.yaml',
        ]
    )

    controller_params = PathJoinSubstitution(
        [
            FindPackageShare('airtable_controller'),
            'config',
            'thrusters.yaml',
        ]
    )

    joy_node = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        output='screen',
    )

    teleop_node = Node(
        package='airtable_teleop',
        executable='airtable_teleop_node',
        name='airtable_teleop',
        namespace='robot_1',
        output='screen',
        parameters=[tereop_params],
    )

    controller_node = Node(
        package='airtable_controller',
        executable='airtable_controller_node',
        name='airtable_controller',
        namespace='robot_1',
        output='screen',
        parameters=[controller_params],
    )

    sim_node = Node(
        package='airtable_sim',
        executable='airtable_sim',
        name='airtable_sim',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('tracking')),
        parameters=[{'viewer': ParameterValue(LaunchConfiguration('viewer'), value_type=bool)}],
    )

    tracking_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('airtable_bringup'), 'launch', 'simulation_tracking.launch.py'
        ])),
        condition=IfCondition(LaunchConfiguration('tracking')),
        launch_arguments={
            'viewer': LaunchConfiguration('viewer'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('tracking', default_value='false',
                                  description='Enable rendered-camera marker tracking'),
            DeclareLaunchArgument('viewer', default_value='true',
                                  description='Open the MuJoCo viewer'),
            DeclareLaunchArgument('rviz', default_value='false',
                                  description='Open tracking RViz (requires tracking:=true)'),
            joy_node,
            teleop_node,
            controller_node,
            sim_node,
            tracking_simulation,
        ]
    )
