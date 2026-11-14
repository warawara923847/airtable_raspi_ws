"""Connect the MuJoCo camera images to the same ArUco tracker used on hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Run physics, image rendering, image detection, and vehicle pose conversion."""
    tracking_share = FindPackageShare('airtable_tracking')
    sim_params = {
        # The simulator's wall timers produce /clock; they must not wait on that clock.
        'use_sim_time': False,
        'viewer': ParameterValue(LaunchConfiguration('viewer'), value_type=bool),
        'enable_camera': True,
        'camera_fps': ParameterValue(LaunchConfiguration('camera_fps'), value_type=float),
    }
    tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            tracking_share, 'launch', 'tracking.launch.py'
        ])),
        launch_arguments={
            'namespace': 'robot_1',
            'params_file': PathJoinSubstitution([
                tracking_share, 'config', 'simulation_tracking.yaml'
            ]),
            'start_camera': 'false',
            'start_detectors': 'true',
            'camera_frame': 'sim_camera_infra1_optical_frame',
            'image_topic': '/sim/camera/infra1/image_rect_raw',
            'camera_info_topic': '/sim/camera/infra1/camera_info',
            'marker_size': '0.1',
            'use_sim_time': 'true',
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('airtable_bringup'), 'launch', 'rviz.launch.py'
        ])),
        condition=IfCondition(LaunchConfiguration('rviz')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true', description='Open MuJoCo viewer'),
        DeclareLaunchArgument('camera_fps', default_value='30.0', description='Camera frame rate'),
        DeclareLaunchArgument('rviz', default_value='false', description='Open tracking RViz'),
        Node(package='airtable_sim', executable='airtable_sim', name='airtable_sim',
             parameters=[sim_params], output='screen'),
        tracking,
        rviz,
    ])
