#!/usr/bin/python3
"""
Show the sun direction given by alpha as an arrow from the sun sensor (PC side).

Subscribes to measurements/sun_sensor (airtable_interfaces/SunSensorState) and publishes
visualization_msgs/Marker on measurements/sun_direction_marker, in the frame of the
measurement. The arrow lies in the sensor's X-Z plane, at alpha from +Z toward +X, and
ignores beta; with the sensor +Y along body +Z that is the robot's horizontal plane.
RViz and Foxglove place it on the robot through the body -> sensor TF of sun_sensor.mount.
An invalid measurement removes the arrow, and the marker lifetime removes it if the
measurements stop.
"""

import math
import sys


def arrow_end(valid, alpha_deg, length):
    """Return the arrow tip in the sensor frame, or None when there is no direction."""
    if not valid or not math.isfinite(length) or length <= 0:
        return None
    if not math.isfinite(alpha_deg) or abs(alpha_deg) >= 90:
        return None
    alpha = math.radians(alpha_deg)
    return (length * math.sin(alpha), 0.0, length * math.cos(alpha))


def main(argv=None):
    """Run the ROS node."""
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node

    from airtable_interfaces.msg import SunSensorState
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker

    class SunDirectionNode(Node):
        def __init__(self):
            super().__init__('airtable_sun_direction')
            self.length = self.declare_parameter('arrow_length_m', 0.4).value
            lifetime = self.declare_parameter('marker_lifetime_s', 0.5).value
            if self.length <= 0 or lifetime <= 0:
                raise ValueError('arrow_length_m and marker_lifetime_s must be positive')
            self.lifetime = Duration(seconds=lifetime).to_msg()
            self.publisher = self.create_publisher(Marker, 'measurements/sun_direction_marker', 10)
            self.create_subscription(
                SunSensorState, 'measurements/sun_sensor', self.on_state, 10)

        def on_state(self, state):
            marker = Marker()
            marker.header = state.header
            marker.ns = 'sun_direction'
            marker.id = 0
            # The Pi's clock stamps the message: draw with the latest TF, not at that time.
            marker.frame_locked = True
            tip = arrow_end(state.valid, state.alpha_deg, self.length)
            if tip is None:
                marker.action = Marker.DELETE
            else:
                marker.type = Marker.ARROW
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                marker.points = [Point(), Point(x=tip[0], y=tip[1], z=tip[2])]
                marker.scale.x = 0.015  # shaft diameter
                marker.scale.y = 0.04  # head diameter
                marker.scale.z = 0.07  # head length
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.8, 0.1
                marker.color.a = 1.0
                marker.lifetime = self.lifetime
            self.publisher.publish(marker)

    import signal

    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=argv)
    node = SunDirectionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C reaches this process twice (terminal and launch); finish cleanly.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
