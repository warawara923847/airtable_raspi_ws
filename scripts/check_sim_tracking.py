#!/usr/bin/python3
"""Check rendered-image ArUco tracking against MuJoCo ground truth over ROS topics."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from airtable_interfaces.msg import ThrusterPulseCommand
import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image


def stamp_ns(header):
    """Use observation time to pair estimates with independently published truth."""
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def yaw(pose):
    """Extract the heading from a ROS quaternion."""
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    """Launch the full image pipeline, collect matched observations, and report errors."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=5.0,
                        help='Collection time after first pose [s]')
    parser.add_argument('--timeout', type=float, default=30.0, help='Startup timeout [s]')
    parser.add_argument('--exercise-motion', action='store_true',
                        help='Fire thruster_1 for 0.25 s')
    parser.add_argument('--position-tolerance', type=float, default=0.03, help='Max XY error [m]')
    parser.add_argument('--yaw-tolerance', type=float, default=0.1, help='Max yaw error [rad]')
    parser.add_argument('--image-out', default='/tmp/airtable_sim_camera.png')
    parser.add_argument('--viewer', action='store_true', help='Also exercise the MuJoCo window')
    parser.add_argument('--control', action='store_true', help='Include joystick/control nodes')
    args = parser.parse_args()
    launch_file = (
        'simulation_control.launch.py' if args.control else 'simulation_tracking.launch.py'
    )
    launch_args = ['tracking:=true'] if args.control else []
    with tempfile.NamedTemporaryFile(
        prefix='airtable_sim_tracking_', suffix='.log', delete=False
    ) as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'airtable_bringup', launch_file,
             f'viewer:={str(args.viewer).lower()}', *launch_args],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
    print(f'Launch log: {log.name}', flush=True)
    node = None
    try:
        rclpy.init(args=[])
        node = rclpy.create_node('airtable_sim_tracking_check')
        truth = {}
        estimates = {}
        images = {}
        infos = {}
        detected_ids = set()
        subscriptions = [
            node.create_subscription(
                Odometry, '/robot_1/sim/ground_truth',
                lambda msg: truth.__setitem__(stamp_ns(msg.header), msg.pose.pose), 100
            ),
            node.create_subscription(
                PoseWithCovarianceStamped, '/robot_1/measurements/external_pose',
                lambda msg: estimates.__setitem__(stamp_ns(msg.header), msg), 100
            ),
            node.create_subscription(
                CameraInfo, '/sim/camera/infra1/camera_info',
                lambda msg: infos.__setitem__(stamp_ns(msg.header), msg), 10
            ),
        ]

        def image_callback(msg):
            images[stamp_ns(msg.header)] = (msg.header.frame_id, msg.width, msg.height)
            if detected_ids != {10, 11, 12, 13, 50}:
                assert msg.encoding == 'mono8'
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
                image = image[:, :msg.width]
                _, ids, _ = cv2.aruco.detectMarkers(
                    image, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
                )
                if ids is not None:
                    detected_ids.update(ids.flatten().tolist())
                cv2.imwrite(args.image_out, image)

        subscriptions.append(node.create_subscription(
            Image, '/sim/camera/infra1/image_rect_raw', image_callback, 10
        ))
        pulse_publisher = node.create_publisher(
            ThrusterPulseCommand, '/robot_1/actuators/thrusters/pulse', 10
        )
        deadline = time.monotonic() + args.timeout
        first_pose_time = None
        pulsed = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if process.poll() is not None:
                raise RuntimeError(f'Simulation launch exited; inspect {log.name}')
            if estimates and first_pose_time is None:
                first_pose_time = time.monotonic()
                deadline = first_pose_time + args.duration
            if estimates and args.exercise_motion and not pulsed:
                if pulse_publisher.get_subscription_count() > 0:
                    pulse = ThrusterPulseCommand()
                    pulse.names = ['thruster_1']
                    pulse.pulse_duration.nanosec = 250_000_000
                    pulse_publisher.publish(pulse)
                    pulsed = True
        if first_pose_time is None:
            raise RuntimeError(
                f'No vehicle pose received; inspect {log.name}; image: {args.image_out}'
            )
        assert detected_ids == {10, 11, 12, 13, 50}, f'Wrong rendered IDs: {detected_ids}'
        xy_errors, yaw_errors, headings = [], [], []
        for stamp in sorted(estimates.keys() & truth.keys() & images.keys() & infos.keys()):
            msg = estimates[stamp]
            actual = truth[stamp]
            observed = msg.pose.pose
            info = infos[stamp]
            assert msg.header.frame_id == 'table_1'
            assert images[stamp] == (info.header.frame_id, info.width, info.height)
            assert info.header.frame_id == 'sim_camera_infra1_optical_frame'
            assert list(info.d) == [0.0] * 5
            assert abs(actual.position.z - observed.position.z) < 1e-8
            xy_errors.append(math.hypot(
                actual.position.x - observed.position.x, actual.position.y - observed.position.y
            ))
            error = yaw(observed) - yaw(actual)
            yaw_errors.append(math.atan2(math.sin(error), math.cos(error)))
            headings.append(yaw(actual))
        assert len(xy_errors) >= 10, f'Only {len(xy_errors)} timestamp-matched samples'
        assert subscriptions[0].get_publisher_count() == 1, 'Multiple simulators publishing truth'
        report = {
            'matched_samples': len(xy_errors), 'rendered_ids': sorted(detected_ids),
            'xy_rmse_m': float(np.sqrt(np.mean(np.square(xy_errors)))),
            'xy_max_m': max(xy_errors),
            'yaw_rmse_rad': float(np.sqrt(np.mean(np.square(yaw_errors)))),
            'yaw_max_rad': max(abs(error) for error in yaw_errors),
            'image': str(Path(args.image_out).resolve()),
        }
        print(json.dumps(report, indent=2), flush=True)
        assert report['xy_max_m'] < args.position_tolerance, 'XY error exceeds tolerance'
        assert report['yaw_max_rad'] < args.yaw_tolerance, 'Yaw error exceeds tolerance'
        if args.exercise_motion:
            assert pulsed and np.ptp(np.unwrap(headings)) > 0.2, 'Vehicle did not rotate enough'
        print('Rendered-image tracking: PASS')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


if __name__ == '__main__':
    main()
