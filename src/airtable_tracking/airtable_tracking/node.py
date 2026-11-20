"""Publish table-relative vehicle pose observations from aruCo PoseStamped inputs."""

from functools import partial
import math

from airtable_tracking.geometry import pose_matrix, table_from_camera_root
from airtable_tracking.observations import MarkerObservations
from airtable_tracking.visualization import layout_markers
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformException, TransformListener
from tf_transformations import quaternion_from_euler, quaternion_from_matrix
from visualization_msgs.msg import MarkerArray


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
        # Configured table area and fixed markers for RViz; latched for late subscribers.
        self.layout_publisher = self.create_publisher(
            MarkerArray, 'tracking/layout',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.layout_publisher.publish(layout_markers(
            self.table_frame, parameter('table_size', [1.0, 2.0]), fixed_ids,
            self.observations.table_points, parameter('marker_size', 0.1)))
        # Root of the camera driver's TF tree (camera_link). After calibration it is attached
        # below table_frame so the real camera appears above the table. '' disables it, e.g.
        # in simulation where the simulator already publishes the camera frame.
        self.camera_tf_frame = parameter('camera_tf_frame', '')
        self.camera_tf_timer = None
        if self.camera_tf_frame:
            if self.camera_tf_frame == self.table_frame:
                raise ValueError('camera_tf_frame must differ from table_frame')
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.static_broadcaster = StaticTransformBroadcaster(self)
        # Dropped after calibration: aruco_ros single skips detection while its outputs have
        # no subscribers, so the fixed-marker detectors then idle (unless recorded/viewed).
        self.fixed_subscriptions = [
            self.create_subscription(
                PoseStamped, f'tracking/aruco_single_{mid}/pose',
                partial(self.fixed_callback, marker_id=mid), qos_profile_sensor_data
            ) for mid in fixed_ids
        ]
        self.moving_subscription = self.create_subscription(
            PoseStamped, f'tracking/aruco_single_{moving_id}/pose',
            self.moving_callback, qos_profile_sensor_data
        )
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
            self.get_logger().info(f'Table calibration complete: {self.table_frame}; '
                                   'unsubscribed from the fixed markers')
            for subscription in self.fixed_subscriptions:
                self.destroy_subscription(subscription)
            self.fixed_subscriptions = []
            if self.camera_tf_frame:
                self.camera_tf_timer = self.create_timer(0.5, self.publish_camera_tf)
                self.publish_camera_tf()

    def publish_camera_tf(self):
        """Attach the camera driver's TF root below the table once its tree is available."""
        camera_frame = self.observations.camera_frame
        if self.camera_tf_frame == camera_frame:
            root_from_camera = np.eye(4)
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.camera_tf_frame, camera_frame, Time()).transform
            except TransformException as error:
                self.get_logger().warning(
                    f'Waiting for TF {self.camera_tf_frame} -> {camera_frame}: {error}',
                    throttle_duration_sec=2.0)
                return
            root_from_camera = pose_matrix(
                [tf.translation.x, tf.translation.y, tf.translation.z],
                [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w])
        table_from_root = table_from_camera_root(
            self.observations.camera_from_table, root_from_camera)
        message = TransformStamped()
        message.header.stamp = Time(
            nanoseconds=self.observations.calibration_stamp_ns).to_msg()
        message.header.frame_id = self.table_frame
        message.child_frame_id = self.camera_tf_frame
        x, y, z = (float(value) for value in table_from_root[:3, 3])
        message.transform.translation.x = x
        message.transform.translation.y = y
        message.transform.translation.z = z
        qx, qy, qz, qw = (float(value) for value in quaternion_from_matrix(table_from_root))
        message.transform.rotation.x = qx
        message.transform.rotation.y = qy
        message.transform.rotation.z = qz
        message.transform.rotation.w = qw
        self.static_broadcaster.sendTransform(message)
        if self.camera_tf_timer is not None:
            self.camera_tf_timer.cancel()
        self.get_logger().info(
            f'Published TF {self.table_frame} -> {self.camera_tf_frame} '
            f'(camera at {x:.3f}, {y:.3f}, {z:.3f} m)')

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
