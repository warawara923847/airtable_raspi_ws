"""View tracking observations, simulation truth, camera TF, and camera images."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start RViz independently so it can attach to an already-running simulator."""
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([
            FindPackageShare('airtable_bringup'), 'rviz', 'tracking.rviz'
        ])),
        Node(package='rviz2', executable='rviz2', name='airtable_rviz', output='screen',
             arguments=['-d', LaunchConfiguration('rviz_config')],
             parameters=[{'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool
             )}]),
    ])
