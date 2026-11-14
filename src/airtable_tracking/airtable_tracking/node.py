"""Publish table-relative vehicle pose observations from aruCo PoseStamped inputs."""

from functools import partial
import math

from airtable_tracking.geometry import pose_matrix
from airtable_tracking.observations import MarkerObservations
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf_transformations import quaternion_from_euler


def _position(msg):
    return [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]


def _stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class TrackingNode(Node):
    """Adapt fixed-camera marker detections to the external vehicle pose interface."""

    def __init__(self, **kwargs):
        """Declare configuration and subscribe to the fixed and moving marker poses."""
        super().__init__('airtable_tracking', **kwargs)

        def parameter(name, default):
            return self.declare_parameter(name, default).value

        fixed_ids = parameter('fixed_marker_ids', [10, 11, 12, 13])
        moving_id = parameter('moving_marker_id', 50)
        table_points = parameter('fixed_marker_positions', [
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 2.0, 0.0, 1.0, 2.0, 0.0,
        ])
        camera_frame = parameter('camera_frame', 'camera_infra1_optical_frame')
        self.table_frame = parameter('table_frame', 'table_1')
        translation = parameter('base_to_marker_translation', [0.0, 0.0, 0.147])
        rpy = parameter('base_to_marker_rpy', [0.0, 0.0, 0.0])
        if len(rpy) != 3 or not np.isfinite(rpy).all():
            raise ValueError('base_to_marker_rpy must contain three finite radians')
        self.covariance = parameter(
            'pose_covariance_diagonal', [0.001, 0.001, 1e6, 1e6, 1e6, 0.01]
        )
        if (len(self.covariance) != 6 or not np.isfinite(self.covariance).all() or
                any(value <= 0 for value in self.covariance)):
            raise ValueError('pose_covariance_diagonal must contain six positive finite values')
        if not self.table_frame or self.table_frame == camera_frame:
            raise ValueError('table_frame must be nonempty and different from camera_frame')
        self.observations = MarkerObservations(
            fixed_ids, np.asarray(table_points).reshape((-1, 3)), moving_id, camera_frame,
            pose_matrix(translation, quaternion_from_euler(*rpy)),
            parameter('base_height', 0.0),
            parameter('calibration_max_time_span', 0.1),
            parameter('calibration_max_residual', 0.02),
        )
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'measurements/external_pose', 10
        )
        self.marker_subscriptions = [
            self.create_subscription(
                PoseStamped, f'tracking/aruco_single_{mid}/pose',
                partial(self.fixed_callback, marker_id=mid), qos_profile_sensor_data
            ) for mid in fixed_ids
        ]
        self.marker_subscriptions.append(self.create_subscription(
            PoseStamped, f'tracking/aruco_single_{moving_id}/pose',
            self.moving_callback, qos_profile_sensor_data
        ))
        self.get_logger().info('Waiting for fixed markers to calibrate the table frame')

    def fixed_callback(self, msg, marker_id):
        """Initialize a stationary camera calibration from a complete fixed-marker set."""
        try:
            calibrated = self.observations.add_fixed(
                marker_id, msg.header.frame_id, _stamp_ns(msg), _position(msg)
            )
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        if calibrated:
            self.get_logger().info(f'Table calibration complete: {self.table_frame}')

    def moving_callback(self, msg):
        """Publish each valid observation once, retaining its original measurement time."""
        orientation = msg.pose.orientation
        try:
            result = self.observations.convert_moving(
                msg.header.frame_id, _stamp_ns(msg), _position(msg),
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        if result is None:
            return
        x, y, z, yaw = result
        output = PoseWithCovarianceStamped()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.table_frame
        output.pose.pose.position.x = float(x)
        output.pose.pose.position.y = float(y)
        output.pose.pose.position.z = float(z)
        output.pose.pose.orientation.z = math.sin(yaw / 2)
        output.pose.pose.orientation.w = math.cos(yaw / 2)
        for index, variance in enumerate(self.covariance):
            output.pose.covariance[index * 7] = variance
        self.pose_publisher.publish(output)


def main(args=None):
    """Run the observation adapter until shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = TrackingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
