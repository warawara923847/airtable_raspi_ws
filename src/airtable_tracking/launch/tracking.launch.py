"""Launch the reusable marker observation adapter and optional image detectors."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _setup(context):
    config = Path(LaunchConfiguration('params_file').perform(context))
    with config.open() as stream:
        params = yaml.safe_load(stream)['/**']['ros__parameters']
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    camera_frame = LaunchConfiguration('camera_frame').perform(context)
    params['camera_frame'] = camera_frame
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    params['use_sim_time'] = use_sim_time
    # Shared with the detectors; the adapter only uses it to draw the expected squares.
    params['marker_size'] = float(LaunchConfiguration('marker_size').perform(context))
    nodes = [Node(
        package='airtable_tracking', executable='airtable_tracking_node',
        name='airtable_tracking', namespace=namespace, parameters=[params], output='screen'
    )]
    for mid in [*params['fixed_marker_ids'], params['moving_marker_id']]:
        nodes.append(Node(
            package='aruco_ros', executable='single', name=f'aruco_single_{mid}',
            namespace='/'.join(filter(None, [namespace, 'tracking'])), output='screen',
            condition=IfCondition(LaunchConfiguration('start_detectors')),
            parameters=[{
                'marker_id': mid,
                'marker_size': float(LaunchConfiguration('marker_size').perform(context)),
                'camera_frame': camera_frame,
                'reference_frame': camera_frame,
                'marker_frame': '/'.join(filter(None, [namespace, f'aruco_marker_{mid}'])),
                'image_is_rectified': True,
                'detection_mode': 'DM_NORMAL',
                # aruco_ros default 0.02 rejected a clearly visible ~40 px marker at 1280x720.
                'min_marker_size': float(LaunchConfiguration('min_marker_size').perform(context)),
                'use_sim_time': use_sim_time,
            }],
            remappings=[
                ('/image', LaunchConfiguration('image_topic')),
                ('/camera_info', LaunchConfiguration('camera_info_topic')),
            ],
        ))
    return nodes


def generate_launch_description():
    """Expose independent camera, detector, and observation-adapter startup."""
    share = get_package_share_directory('airtable_tracking')
    camera_share = get_package_share_directory('realsense2_camera')
    arguments = [
        ('namespace', 'robot_1', 'Vehicle topic namespace'),
        ('params_file', str(Path(share) / 'config/tracking.yaml'), 'Tracking configuration'),
        ('camera_frame', 'camera_infra1_optical_frame', 'Input optical frame'),
        ('marker_size', '0.1', 'Physical marker side length in meters'),
        ('image_topic', '/camera/camera/infra1/image_rect_raw', 'Rectified marker image'),
        ('camera_info_topic', '/camera/camera/infra1/camera_info', 'Matching CameraInfo'),
        ('start_camera', 'false', 'Start a physical RealSense camera'),
        ('start_detectors', 'true', 'Detect markers; disable for synthetic Pose inputs'),
        ('use_sim_time', 'false', 'Use /clock; requires a clock publisher'),
        ('camera_serial_no', "''", "RealSense serial, quoted like \"'123'\"; '' = any"),
        ('camera_name', 'camera', 'RealSense node name'),
        ('camera_namespace', 'camera', 'RealSense namespace (topics /<ns>/<name>/...)'),
        ('camera_profile', '848x480x90', 'RealSense infra1 WIDTHxHEIGHTxFPS'),
        ('camera_exposure', '5000', 'Manual infra exposure [us]; must fit the frame period'),
        ('camera_gain', '20', 'Manual infra gain'),
        ('min_marker_size', '0.0', 'aruco_ros minimum marker size relative to the image'),
    ]
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(camera_share) / 'launch/rs_launch.py')),
        condition=IfCondition(LaunchConfiguration('start_camera')),
        launch_arguments={
            'enable_infra1': 'true', 'enable_infra2': 'false',
            'enable_color': 'false', 'enable_depth': 'false',
            # Settings of the old airtable_ws full_system.launch.py. Auto exposure washed out
            # one glossy marker, and 640x480 crops the view; this detects all four at 65-90 Hz.
            'depth_module.infra_profile': LaunchConfiguration('camera_profile'),
            'depth_module.emitter_enabled': '0',
            'depth_module.enable_auto_exposure': 'false',
            'depth_module.exposure': LaunchConfiguration('camera_exposure'),
            'depth_module.gain': LaunchConfiguration('camera_gain'),
            # Select one of several cameras by serial; "''" (default) accepts any one.
            'serial_no': LaunchConfiguration('camera_serial_no'),
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_namespace': LaunchConfiguration('camera_namespace'),
        }.items(),
    )
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in arguments),
        camera,
        OpaqueFunction(function=_setup),
    ])
