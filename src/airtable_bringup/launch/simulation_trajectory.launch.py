"""Try fixed-yaw tracking with explicit initial pose and optional camera observations."""

import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def simulation_node(context):
    """Start at the circle's right edge by default, preserving line-model initial poses."""
    params_path = Path(LaunchConfiguration('params_file').perform(context))
    params = yaml.safe_load(params_path.read_text())['/**']['ros__parameters']
    trajectory = params.get('trajectory', {})

    def argument(name):
        return LaunchConfiguration(name).perform(context)

    kind = argument('trajectory') or trajectory.get('type', 'line')
    radius = float(argument('radius') or trajectory.get('radius_m', 0.1))
    center = trajectory.get('circle_center_m', [])
    center_args = [argument('circle_center_x'), argument('circle_center_y')]
    if any(center_args):
        if not all(center_args):
            raise ValueError('Specify both circle_center_x and circle_center_y')
        center = [float(value) for value in center_args]
    if len(center) not in (0, 2) or not all(math.isfinite(value) for value in center):
        raise ValueError('circle_center_m must be empty or two finite coordinates')
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('radius must be finite and positive')
    xy = [argument('initial_x'), argument('initial_y')]
    pose = []
    if any(xy):
        if not all(xy):
            raise ValueError('Specify both initial_x and initial_y')
        pose = [float(value) for value in xy]
    elif kind == 'circle' and center:
        offset = trajectory.get('offset_m', [0.0, 0.0])
        if len(offset) != 2 or not all(math.isfinite(value) for value in offset):
            raise ValueError('offset_m must contain two finite coordinates')
        pose = [center[0] + offset[0] + radius, center[1] + offset[1]]
    if pose:
        pose.append(float(argument('initial_yaw') or '0.0'))
        if not all(math.isfinite(value) for value in pose):
            raise ValueError('initial pose must be finite')
    elif argument('initial_yaw'):
        raise ValueError('Set initial_x and initial_y when overriding a line initial yaw')
    sim_params = {
        'use_sim_time': False,
        'enable_camera': ParameterValue(LaunchConfiguration('tracking'), value_type=bool),
        'camera_fps': ParameterValue(LaunchConfiguration('camera_fps'), value_type=float),
        'viewer': ParameterValue(LaunchConfiguration('viewer'), value_type=bool),
    }
    if pose:
        sim_params['initial_pose'] = pose
    if argument('model_path'):
        sim_params['model_path'] = argument('model_path')
    return [Node(package='airtable_sim', executable='airtable_sim', name='airtable_sim',
                 output='screen', parameters=[sim_params])]


def generate_launch_description():
    """Run a single simulator and the common control chain, without manual control."""
    share = FindPackageShare('airtable_bringup')
    forwarded = ['trajectory', 'period', 'amplitude', 'radius', 'direction', 'fixed_yaw',
                 'circle_center_x', 'circle_center_y',
                 'params_file', 'thruster_params_file', 'enabled']
    control_args = {key: LaunchConfiguration(key) for key in forwarded}
    control_args.update({'use_sim_time': 'true',
                         'state_topic': LaunchConfiguration('state_topic')})
    return LaunchDescription([
        DeclareLaunchArgument('trajectory', default_value='', choices=['', 'line', 'circle']),
        DeclareLaunchArgument('period', default_value=''),
        DeclareLaunchArgument('amplitude', default_value=''),
        DeclareLaunchArgument('radius', default_value=''),
        DeclareLaunchArgument('direction', default_value=''),
        DeclareLaunchArgument('fixed_yaw', default_value=''),
        DeclareLaunchArgument('circle_center_x', default_value=''),
        DeclareLaunchArgument('circle_center_y', default_value=''),
        DeclareLaunchArgument('initial_x', default_value=''),
        DeclareLaunchArgument('initial_y', default_value=''),
        DeclareLaunchArgument('initial_yaw', default_value=''),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('tracking', default_value='false'),
        DeclareLaunchArgument('camera_fps', default_value='30.0'),
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('state_topic', default_value='sim/ground_truth'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('disturbance', default_value='false'),
        DeclareLaunchArgument('disturbance_params_file', default_value=PathJoinSubstitution([
            share, 'config', 'disturbance.yaml'])),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('airtable_controller'), 'config', 'trajectory.yaml'])),
        DeclareLaunchArgument('thruster_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('airtable_controller'), 'config', 'thrusters.yaml'])),
        OpaqueFunction(function=simulation_node),
        Node(package='airtable_bringup', executable='sim_disturbance',
             name='airtable_sim_disturbance', namespace='robot_1', output='screen',
             condition=IfCondition(LaunchConfiguration('disturbance')),
             parameters=[LaunchConfiguration('disturbance_params_file'), {'use_sim_time': True}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            share, 'launch', 'trajectory_control.launch.py'])),
            launch_arguments=control_args.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('airtable_tracking'), 'launch', 'tracking.launch.py'])),
            condition=IfCondition(LaunchConfiguration('tracking')),
            launch_arguments={
                'namespace': 'robot_1', 'use_sim_time': 'true',
                'params_file': PathJoinSubstitution([
                    FindPackageShare('airtable_tracking'), 'config', 'simulation_tracking.yaml']),
                'start_camera': 'false', 'start_detectors': 'true',
                'camera_frame': 'sim_camera_infra1_optical_frame',
                'image_topic': '/sim/camera/infra1/image_rect_raw',
                'camera_info_topic': '/sim/camera/infra1/camera_info',
                'marker_size': '0.1',
            }.items()),
        Node(package='rviz2', executable='rviz2', name='trajectory_rviz',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'trajectory.rviz'])],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
