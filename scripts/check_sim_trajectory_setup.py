#!/usr/bin/python3
"""Check circle initialization, actual topic rates and optional camera observations."""

import argparse
from bisect import bisect_left
import json
import math
import os
from pathlib import Path
import tempfile
import time

from check_sim_model import require, running
import numpy as np


def stamp(msg):
    """Read the source timestamp, independent of wall reception time."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def rates(records):
    """Report observed wall rate and rate per simulation second separately."""
    wall, messages = zip(*records)
    duration = stamp(messages[-1]) - stamp(messages[0])
    return {'samples': len(messages),
            'wall_hz': (len(records) - 1) / (wall[-1] - wall[0]),
            'sim_hz': (len(records) - 1) / duration if duration > 0 else None}


def main():
    """Launch an isolated simulation and validate its startup and data sources."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tracking', action='store_true')
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--domain-id', type=int, default=194)
    parser.add_argument('--duration', type=float, default=8.0)
    parser.add_argument('--radius', type=float, default=0.2)
    parser.add_argument('--initial-x', type=float)
    parser.add_argument('--initial-y', type=float)
    parser.add_argument('--initial-yaw', type=float, default=0.0)
    args = parser.parse_args()
    require(0 <= args.domain_id <= 232 and math.isfinite(args.duration)
            and args.duration > 1 and math.isfinite(args.radius) and args.radius > 0,
            'Invalid test configuration')
    require((args.initial_x is None) == (args.initial_y is None), 'Specify both initial x/y')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    logs = Path(tempfile.mkdtemp(prefix='airtable_setup_check_'))
    os.environ['ROS_LOG_DIR'] = str(logs / 'ros')
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import Image

    rclpy.init(args=[])
    node = rclpy.create_node('trajectory_setup_check')
    records = {name: [] for name in ('truth', 'reference', 'path', 'image', 'observation')}

    def collect(name):
        return lambda msg: records[name].append((time.monotonic(), msg))

    subscriptions = [
        node.create_subscription(Odometry, '/robot_1/sim/ground_truth', collect('truth'), 100),
        node.create_subscription(Odometry, '/robot_1/trajectory/reference',
                                 collect('reference'), 100),
        node.create_subscription(RosPath, '/robot_1/trajectory/actual_path', collect('path'), 100),
        node.create_subscription(Image, '/sim/camera/infra1/image_rect_raw', collect('image'), 10),
        node.create_subscription(PoseWithCovarianceStamped, '/robot_1/measurements/external_pose',
                                 collect('observation'), 100),
    ]
    control_client = node.create_client(GetParameters,
                                        '/robot_1/airtable_trajectory_controller/get_parameters')
    sim_client = node.create_client(GetParameters, '/airtable_sim/get_parameters')
    command = ['ros2', 'launch', 'airtable_bringup', 'trajectory.launch.py',
               'trajectory:=circle', f'radius:={args.radius}',
               f'viewer:={str(args.viewer).lower()}', f'rviz:={str(args.rviz).lower()}',
               f'tracking:={str(args.tracking).lower()}']
    if args.initial_x is not None:
        command += [f'initial_x:={args.initial_x}', f'initial_y:={args.initial_y}',
                    f'initial_yaw:={args.initial_yaw}']

    def spin_until(predicate, process, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            require(process.poll() is None, f'Launch exited; inspect {logs}')
            if predicate():
                return
        raise RuntimeError(f'Timed out; inspect {logs}')

    print(f'Logs: {logs}; ROS domain: {args.domain_id}', flush=True)
    try:
        with running(command, logs, 'launch') as process:
            spin_until(lambda: bool(records['reference']), process)
            start = time.monotonic()
            spin_until(lambda: time.monotonic() - start >= args.duration, process,
                       timeout=args.duration + 5)
            require(subscriptions[0].get_publisher_count() == 1, 'Multiple simulators')
            spin_until(lambda: control_client.service_is_ready() and sim_client.service_is_ready(),
                       process)
            future = control_client.call_async(GetParameters.Request(names=[
                'state_topic', 'trajectory.circle_center_m']))
            spin_until(future.done, process)
            params = future.result().values
            require(params[0].string_value == 'sim/ground_truth', 'Control source changed')
            center = list(params[1].double_array_value)
            require(center == [0.5, 1.0], f'Wrong circle center: {center}')
            future = sim_client.call_async(GetParameters.Request(names=[
                'enable_camera', 'initial_pose_vehicles', 'initial_poses']))
            spin_until(future.done, process)
            params = future.result().values
            require(params[0].bool_value == args.tracking, 'Camera setting mismatch')
            expected = [0.5 + args.radius, 1.0, 0.0] if args.initial_x is None else [
                args.initial_x, args.initial_y, args.initial_yaw]
            require(list(params[1].string_array_value) == ['robot_1'], 'Wrong initial pose robot')
            require(np.allclose(params[2].double_array_value, expected), 'Wrong initial pose')
            initial = records['truth'][0][1].pose.pose.position
            require(np.linalg.norm(np.array([initial.x, initial.y]) - expected[:2]) < 1e-4,
                    'Initial truth differs from requested pose')
            reference_positions = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y]
                                            for _, msg in records['reference']])
            radial_errors = np.linalg.norm(reference_positions - center, axis=1) - args.radius
            if args.initial_x is None:
                require(np.max(np.abs(radial_errors)) < 1e-8,
                        'Reference radius changed during startup')
            report = {'initial_pose': expected, 'circle_center': center, 'radius_m': args.radius,
                      'control_source': 'sim/ground_truth', 'tracking': args.tracking}
            for name in ('truth', 'reference', 'path'):
                require(len(records[name]) > 10, f'Insufficient {name} messages')
                report[name] = rates(records[name])
            truth_duration = stamp(records['truth'][-1][1]) - stamp(records['truth'][0][1])
            report['real_time_factor'] = truth_duration / (
                records['truth'][-1][0] - records['truth'][0][0])
            if args.tracking:
                require(len(records['image']) >= 5 and len(records['observation']) >= 5,
                        'No rendered camera observations')
                times = [stamp(msg) for _, msg in records['truth']]
                errors = []
                for _, msg in records['observation']:
                    index = bisect_left(times, stamp(msg))
                    candidates = range(max(0, index - 1), min(len(times), index + 1))
                    match = min(candidates, key=lambda i: abs(times[i] - stamp(msg)))
                    if abs(times[match] - stamp(msg)) <= 0.021:
                        truth = records['truth'][match][1].pose.pose.position
                        pose = msg.pose.pose.position
                        errors.append(math.hypot(truth.x - pose.x, truth.y - pose.y))
                require(len(errors) >= 5, f'Only {len(errors)} timestamp-paired observations')
                report['image'] = rates(records['image'])
                report['observation'] = rates(records['observation'])
                report['observation_xy_rmse_m'] = float(np.sqrt(np.mean(np.square(errors))))
                report['observation_xy_max_m'] = max(errors)
            else:
                require(not records['image'] and not records['observation'],
                        'Unexpected camera/detector output')
            print(json.dumps(report, indent=2), flush=True)
            (logs / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
            if args.tracking:
                require(report['observation_xy_max_m'] < 0.03,
                        'Marker observation error exceeded tolerance; inspect report.json')
            print('Trajectory setup: PASS', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
