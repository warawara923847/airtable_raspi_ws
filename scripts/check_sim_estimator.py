#!/usr/bin/python3
"""
Close the trajectory loop through camera -> ArUco -> EKF in MuJoCo and compare with truth.

Runs trajectory.launch.py tracking:=true state_source:=estimator headless in a private ROS
domain (docs/adr/0024) and reports
- estimator error: state/odometry against sim/ground_truth at the same stamp
  (position, yaw, world-frame velocity, yaw rate) and its age on arrival,
- tracking error: ground truth against trajectory/reference, as check_sim_trajectory.py,
- observation and estimate rates.
--baseline also runs state_source:=truth with the same trajectory for comparison.

  scripts/check_sim_estimator.py                       # circle, one period after start-up
  scripts/check_sim_estimator.py --trajectory line --baseline
"""

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
    """Return the header time in seconds."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw(pose):
    """Extract yaw from a geometry_msgs Pose."""
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    """Wrap an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def world_velocity(msg):
    """Rotate the body-frame Odometry twist into the table frame."""
    heading = yaw(msg.pose.pose)
    v = msg.twist.twist.linear
    c, s = math.cos(heading), math.sin(heading)
    return np.array([c * v.x - s * v.y, s * v.x + c * v.y])


def nearest(times, messages, t, tolerance):
    """Message whose stamp is closest to t, or None beyond tolerance."""
    index = bisect_left(times, t)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(times[i] - t))
    return messages[best] if abs(times[best] - t) <= tolerance else None


def rms(values):
    """Root mean square of a sequence."""
    return float(np.sqrt(np.mean(np.square(values)))) if len(values) else float('nan')


def rate(messages, start, end):
    """Messages per second of stamps inside [start, end]."""
    count = sum(start <= stamp(msg) <= end for msg in messages)
    return count / (end - start) if end > start else float('nan')


def run(args, source, log_dir, domain):
    """Launch one configuration and return its metrics."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.parameter import Parameter

    os.environ['ROS_DOMAIN_ID'] = str(domain)
    rclpy.init(args=[])
    node = rclpy.create_node('estimator_check', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    truths, estimates, references, observations, ages = [], [], [], [], []

    def receive_estimate(msg):
        estimates.append(msg)
        ages.append(node.get_clock().now().nanoseconds * 1e-9 - stamp(msg))

    subscriptions = [
        node.create_subscription(Odometry, '/robot_1/sim/ground_truth', truths.append, 200),
        node.create_subscription(Odometry, '/robot_1/state/odometry', receive_estimate, 200),
        node.create_subscription(Odometry, '/robot_1/trajectory/reference',
                                 references.append, 200),
        node.create_subscription(PoseWithCovarianceStamped,
                                 '/robot_1/measurements/external_pose', observations.append, 200),
    ]
    command = ['ros2', 'launch', 'airtable_bringup', 'trajectory.launch.py',
               f'trajectory:={args.trajectory}', f'period:={args.period}', 'viewer:=false',
               f'state_source:={source}',
               f'tracking:={"true" if source == "estimator" else "false"}']
    if args.estimator_params and source == 'estimator':
        command.append(f'estimator_params:={args.estimator_params}')
    label = f'{args.trajectory}_{source}'
    try:
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        require(all(sub.get_publisher_count() == 0 for sub in subscriptions),
                f'ROS domain {domain} is occupied; use another --domain-id')
        with running(command, log_dir, label) as process:
            def spin_until(predicate, timeout):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    require(process.poll() is None, f'{label}: launch exited; see {log_dir}')
                    if predicate():
                        return
                raise RuntimeError(f'{label}: timed out; see {log_dir}')

            spin_until(lambda: bool(references), 60.0)
            first = stamp(references[0])
            duration = args.startup + args.cycles * args.period
            spin_until(lambda: stamp(references[-1]) - first >= duration,
                       max(60.0, 3 * duration))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    start, end = first + args.startup, first + duration
    truth_times = [stamp(msg) for msg in truths]
    tracking_xy, tracking_yaw = [], []
    for reference in references:
        if not start <= stamp(reference) <= end:
            continue
        truth = nearest(truth_times, truths, stamp(reference), 0.021)
        if truth is None:
            continue
        a, b = truth.pose.pose.position, reference.pose.pose.position
        tracking_xy.append(math.hypot(a.x - b.x, a.y - b.y))
        tracking_yaw.append(wrap(yaw(truth.pose.pose) - yaw(reference.pose.pose)))
    require(len(tracking_xy) > 20 * args.cycles * args.period * 0.8,
            f'{label}: only {len(tracking_xy)} tracking samples')
    report = {
        'trajectory': args.trajectory, 'state_source': source,
        'tracking_xy_rmse_m': rms(tracking_xy), 'tracking_xy_max_m': max(tracking_xy),
        'tracking_yaw_rmse_rad': rms(tracking_yaw),
        'tracking_yaw_max_rad': max(map(abs, tracking_yaw)),
    }
    if source == 'estimator':
        position, heading, velocity, yaw_rate, series = [], [], [], [], []
        for estimate in estimates:
            if not start <= stamp(estimate) <= end:
                continue
            truth = nearest(truth_times, truths, stamp(estimate), 0.006)
            if truth is None:
                continue
            a, b = estimate.pose.pose.position, truth.pose.pose.position
            position.append(math.hypot(a.x - b.x, a.y - b.y))
            heading.append(wrap(yaw(estimate.pose.pose) - yaw(truth.pose.pose)))
            velocity.append(float(np.linalg.norm(world_velocity(estimate) -
                                                 world_velocity(truth))))
            yaw_rate.append(estimate.twist.twist.angular.z - truth.twist.twist.angular.z)
            series.append([stamp(estimate), a.x, a.y, yaw(estimate.pose.pose),
                           *world_velocity(estimate), estimate.twist.twist.angular.z,
                           b.x, b.y, yaw(truth.pose.pose), *world_velocity(truth),
                           truth.twist.twist.angular.z])
        require(len(position) > 10 * args.cycles * args.period,
                f'{label}: only {len(position)} estimate samples paired with truth')
        # t, estimate x y yaw vx vy wz, truth x y yaw vx vy wz (velocities in the table frame)
        np.savetxt(log_dir / f'{label}_estimate_vs_truth.csv', np.array(series), delimiter=',',
                   header='t,x,y,yaw,vx,vy,wz,true_x,true_y,true_yaw,true_vx,true_vy,true_wz')
        report |= {
            'estimate_samples': len(position),
            'estimate_xy_rmse_m': rms(position), 'estimate_xy_max_m': max(position),
            'estimate_yaw_rmse_rad': rms(heading),
            'estimate_velocity_rmse_m_s': rms(velocity),
            'estimate_velocity_max_m_s': max(velocity),
            'estimate_yaw_rate_rmse_rad_s': rms(yaw_rate),
            'estimate_age_mean_s': float(np.mean(ages)), 'estimate_age_max_s': max(ages),
            'observation_rate_hz': rate(observations, start, end),
            'estimate_rate_hz': rate(estimates, start, end),
        }
    print(json.dumps(report, indent=2), flush=True)
    (log_dir / f'{label}_metrics.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    """Run the estimator loop (and optionally the truth baseline) in private domains."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--trajectory', choices=['line', 'circle'], default='circle')
    parser.add_argument('--period', type=float, default=20.0)
    parser.add_argument('--cycles', type=float, default=1.0)
    parser.add_argument('--startup', type=float, default=5.0,
                        help='Seconds after the first reference excluded from metrics')
    parser.add_argument('--domain-id', type=int, default=176)
    parser.add_argument('--estimator-params', default='', help='Override estimator_params:=')
    parser.add_argument('--baseline', action='store_true',
                        help='Also run state_source:=truth for comparison')
    parser.add_argument('--max-tracking-xy-rmse', type=float, default=0.02)
    parser.add_argument('--max-estimate-xy-rmse', type=float, default=0.01)
    parser.add_argument('--max-estimate-velocity-rmse', type=float, default=0.02)
    args = parser.parse_args()
    require(0 <= args.domain_id <= 231, 'Invalid ROS domain')
    require(args.period > 0 and args.cycles > 0 and args.startup >= 0, 'Invalid timing')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    log_dir = Path(tempfile.mkdtemp(prefix='airtable_estimator_check_'))
    os.environ['ROS_LOG_DIR'] = str(log_dir / 'ros')
    print(f'Logs: {log_dir}', flush=True)
    report = run(args, 'estimator', log_dir, args.domain_id)
    if args.baseline:
        run(args, 'truth', log_dir, args.domain_id + 1)
    require(report['tracking_xy_rmse_m'] <= args.max_tracking_xy_rmse,
            'Tracking through the estimator exceeded --max-tracking-xy-rmse')
    require(report['estimate_xy_rmse_m'] <= args.max_estimate_xy_rmse,
            'Estimated position exceeded --max-estimate-xy-rmse')
    require(report['estimate_velocity_rmse_m_s'] <= args.max_estimate_velocity_rmse,
            'Estimated velocity exceeded --max-estimate-velocity-rmse')
    print('Estimator closed loop: PASS', flush=True)


if __name__ == '__main__':
    main()
