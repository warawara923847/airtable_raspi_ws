#!/usr/bin/python3
"""Compare trajectory tracking with/without external loads and check the ROS force input."""

import argparse
from bisect import bisect_left
import copy
import csv
import json
import math
import os
from pathlib import Path
from runpy import run_path
import tempfile
import time

from check_sim_model import require, running
from check_sim_trajectory import stamp, yaw
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]


def spin_until(node, predicate, process, timeout=20):
    """Wait for observable state, failing if the owned process exits."""
    import rclpy
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.01)
        require(process.poll() is None, 'Simulation process exited; inspect the launch log')
        if predicate():
            return
    raise RuntimeError('Timed out waiting for simulation state')


def nonzero(msg):
    """Detect force or torque without combining their physical units in a norm."""
    return math.hypot(msg.wrench.force.x, msg.wrench.force.y) > 1e-12 or (
        abs(msg.wrench.torque.z) > 1e-12)


def check_input(logs):
    """Test actual ROS input, impulse response, timeout and malformed messages."""
    from geometry_msgs.msg import WrenchStamped
    from nav_msgs.msg import Odometry
    import rclpy

    rclpy.init(args=[])
    node = rclpy.create_node('disturbance_input_check')
    truth, applied = [], []
    subscriptions = [
        node.create_subscription(Odometry, '/robot_1/sim/ground_truth', truth.append, 100),
        node.create_subscription(WrenchStamped, '/robot_1/sim/disturbance/applied',
                                 applied.append, 100),
    ]
    publisher = node.create_publisher(WrenchStamped, '/robot_1/sim/disturbance/wrench', 1)
    command = ['ros2', 'run', 'airtable_sim', 'airtable_sim', '--ros-args',
               '-p', 'viewer:=false', '-p', 'initial_pose:=[0.5, 1.0, 1.5707963267948966]']

    def message(fx=0.0, mz=0.0, timed=False):
        msg = WrenchStamped()
        msg.header.frame_id = 'robot_1/base_link'
        if timed:
            msg.header.stamp = copy.deepcopy(truth[-1].header.stamp)
        msg.wrench.force.x = fx
        msg.wrench.torque.z = mz
        return msg

    def table_velocity():
        angle = yaw(truth[-1])
        twist = truth[-1].twist.twist.linear
        return np.array([math.cos(angle) * twist.x - math.sin(angle) * twist.y,
                         math.sin(angle) * twist.x + math.cos(angle) * twist.y])

    try:
        with running(command, logs, 'input') as process:
            spin_until(node, lambda: bool(truth) and bool(applied)
                       and publisher.get_subscription_count() == 1, process)
            require(subscriptions[0].get_publisher_count() == 1, 'Multiple simulators')
            publisher.publish(message(fx=0.04))
            spin_until(node, lambda: nonzero(applied[-1]), process)
            start = stamp(truth[-1])
            spin_until(node, lambda: stamp(truth[-1]) >= start + 0.4
                       and not nonzero(applied[-1]), process)
            require(np.allclose(table_velocity(), [0, 0.01], atol=1e-5),
                    f'Force impulse or body-frame mapping incorrect: {table_velocity()}')
            publisher.publish(message(mz=0.0004))
            spin_until(node, lambda: nonzero(applied[-1]), process)
            start = stamp(truth[-1])
            spin_until(node, lambda: stamp(truth[-1]) >= start + 0.4
                       and not nonzero(applied[-1]), process)
            require(abs(truth[-1].twist.twist.angular.z - 0.015) < 1e-5,
                    'Torque impulse or timeout incorrect')
            for kind in ('wrong_frame', 'nonplanar', 'nan', 'stale', 'future',
                         'duplicate', 'zero'):
                good = message(fx=0.01, timed=True)
                publisher.publish(good)
                spin_until(node, lambda: nonzero(applied[-1]), process)
                bad = message(fx=0.01)
                if kind == 'wrong_frame':
                    bad.header.frame_id = 'wrong'
                elif kind == 'nonplanar':
                    bad.wrench.force.z = 1.0
                elif kind == 'nan':
                    bad.wrench.force.x = float('nan')
                elif kind == 'stale':
                    bad.header.stamp.nanosec = 1
                elif kind == 'future':
                    bad.header.stamp.sec = truth[-1].header.stamp.sec + 1
                elif kind == 'duplicate':
                    bad = good
                else:
                    bad.wrench.force.x = 0.0
                publisher.publish(bad)
                spin_until(node, lambda: not nonzero(applied[-1]), process)
                target = stamp(truth[-1]) + 0.06
                spin_until(node, lambda: stamp(truth[-1]) >= target, process)
                require(not nonzero(applied[-1]), f'{kind} command did not remove the force')
            print('ROS disturbance input: PASS (force/torque impulses, body axes, watchdog, '
                  'invalid messages and zero cancellation)', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def metrics(rows, begin, end):
    """Compute errors over the same trajectory-time window for both trials."""
    samples = [row for row in rows if begin <= row[0] < end]
    require(len(samples) >= 20, 'Insufficient samples in evaluation window')
    data = np.array(samples)
    return {'samples': len(samples),
            'xy_rmse_m': float(np.sqrt(np.mean(data[:, 7] ** 2))),
            'xy_max_m': float(np.max(data[:, 7])),
            'yaw_rmse_rad': float(np.sqrt(np.mean(data[:, 8] ** 2))),
            'yaw_max_rad': float(np.max(np.abs(data[:, 8])))}


def check_gaussian(logs, label, commands, applied, epoch, config):
    """Verify seeded samples, sample holds and actual force application through ROS."""
    module = run_path(str(ROOT / 'src/airtable_bringup/scripts/sim_disturbance.py'))
    mean = [config[key] for key in ('force_x_n', 'force_y_n', 'torque_z_nm')]
    stddev = [config[key] for key in (
        'force_x_stddev_n', 'force_y_stddev_n', 'torque_z_stddev_nm')]
    generator = module['GaussianWrench'](
        mean, stddev, config['noise_sample_period_s'], config['noise_seed'])
    samples, command_rows = {}, []
    for msg in commands:
        # Match the generator's integer ROS-time conversion at interval boundaries.
        now = (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec) * 1e-9
        elapsed = now - epoch
        gain = module['profile_gain']('gaussian', elapsed, config['start_delay_s'],
                                      config['duration_s'], 1.0)
        expected = (0.0, 0.0, 0.0)
        if gain:
            expected = generator.sample(max(0, elapsed - config['start_delay_s']))
        actual = (msg.wrench.force.x, msg.wrench.force.y, msg.wrench.torque.z)
        require(msg.header.frame_id == config['frame_id'], 'Wrong random-force reference frame')
        require(np.allclose(actual, expected, rtol=0, atol=1e-14),
                f'Random command differs from seeded interval at t={elapsed}: {actual} != '
                f'{expected}')
        if gain:
            samples[generator.last_index] = actual
        command_rows.append([elapsed, *actual])
    count = math.ceil(config['duration_s'] / config['noise_sample_period_s'] - 1e-10)
    require(set(samples) == set(range(count)), 'Missing stochastic intervals in ROS output')
    command_times = [row[0] for row in command_rows]
    applied_count = 0
    for msg in applied:
        if not nonzero(msg):
            continue
        elapsed = stamp(msg) - epoch
        begin = bisect_left(command_times, elapsed - 0.1)
        end = bisect_left(command_times, elapsed + 0.011)
        force_norm = math.hypot(msg.wrench.force.x, msg.wrench.force.y)
        require(any(abs(force_norm - math.hypot(row[1], row[2])) < 1e-9
                    and abs(msg.wrench.torque.z - row[3]) < 1e-12
                    for row in command_rows[begin:end]),
                'Applied random load does not match a recent requested force/torque')
        applied_count += 1
    require(applied_count > count, 'Insufficient actual random force application')
    data = np.array([samples[i] for i in sorted(samples)])
    with (logs / f'{label}_commands.csv').open('w') as output:
        writer = csv.writer(output)
        writer.writerow(['time_s', 'fx_n', 'fy_n', 'mz_nm'])
        writer.writerows(command_rows)
    return {'seed': config['noise_seed'], 'sample_period_s': generator.sample_period,
            'frame_id': config['frame_id'], 'independent_samples': len(samples),
            'configured_mean': mean, 'configured_variance': [value**2 for value in stddev],
            'sample_mean': data.mean(axis=0).tolist(),
            'sample_variance': data.var(axis=0, ddof=1).tolist() if len(data) > 1 else None,
            'seeded_commands_verified': True, 'actual_application_verified': True}


def run_trial(args, logs, kind, load, config, duration):
    """Run identical trajectories with external loading switched off/on."""
    from geometry_msgs.msg import WrenchStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Bool

    label = kind + ('_loaded' if load else '_baseline')
    rclpy.init(args=[])
    node = rclpy.create_node('disturbance_tracking_check')
    truths, references, applied, paths, active, commands = [], [], [], [], [], []
    subscriptions = [
        node.create_subscription(Odometry, '/robot_1/sim/ground_truth', truths.append, 100),
        node.create_subscription(Odometry, '/robot_1/trajectory/reference',
                                 references.append, 100),
        node.create_subscription(WrenchStamped, '/robot_1/sim/disturbance/applied',
                                 applied.append, 100),
        node.create_subscription(WrenchStamped, '/robot_1/sim/disturbance/wrench',
                                 commands.append, 100),
        node.create_subscription(RosPath, '/robot_1/trajectory/path', paths.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(Bool, '/robot_1/trajectory/active', active.append,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]
    command = ['ros2', 'launch', 'airtable_bringup', 'trajectory.launch.py',
               f'trajectory:={kind}', 'viewer:=false', f'period:={args.period}',
               f'radius:={args.radius}', f'disturbance:={str(load).lower()}',
               f'disturbance_params_file:={args.disturbance_params}']
    try:
        with running(command, logs, label) as process:
            try:
                spin_until(node, lambda: bool(references) and bool(paths) and bool(applied)
                           and any(msg.data for msg in active), process)
            except RuntimeError as error:
                raise RuntimeError(f'{label}: truth={len(truths)}, reference={len(references)}, '
                                   f'path={len(paths)}, applied={len(applied)}, '
                                   f'active={[msg.data for msg in active]}') from error
            epoch = stamp(paths[0])
            spin_until(node, lambda: stamp(references[-1]) - epoch >= duration,
                       process, timeout=duration * 3 + 15)
            require(subscriptions[0].get_publisher_count() == 1, 'Multiple simulators')
            first_active = next(i for i, msg in enumerate(active) if msg.data)
            require(all(msg.data for msg in active[first_active:]),
                    f'{label}: trajectory was disabled under external loading')
            truth_times = [stamp(msg) for msg in truths]
            force_times = [stamp(msg) for msg in applied]
            rows = []
            for reference in references:
                t = stamp(reference)
                insertion = bisect_left(truth_times, t)
                options = range(max(0, insertion - 1), min(len(truth_times), insertion + 1))
                index = min(options, key=lambda i: abs(truth_times[i] - t))
                if abs(truth_times[index] - t) > 0.021:
                    continue
                actual = truths[index]
                force_index = min(max(0, bisect_left(force_times, t) - 1), len(applied) - 1)
                force = applied[force_index].wrench
                p, target = actual.pose.pose.position, reference.pose.pose.position
                heading = yaw(actual)
                desired_heading = yaw(reference)
                angle_error = math.atan2(math.sin(heading - desired_heading),
                                         math.cos(heading - desired_heading))
                rows.append([t - epoch, target.x, target.y, desired_heading, p.x, p.y,
                             heading, math.hypot(p.x - target.x, p.y - target.y), angle_error,
                             force.force.x, force.force.y, force.torque.z])
            require(len(rows) >= duration * 20, 'Insufficient paired trajectory samples')
            with (logs / f'{label}.csv').open('w') as output:
                writer = csv.writer(output)
                writer.writerow(['time_s', 'target_x', 'target_y', 'target_yaw', 'x', 'y', 'yaw',
                                 'xy_error_m', 'yaw_error_rad', 'body_fx_n', 'body_fy_n', 'mz_nm'])
                writer.writerows(rows)
            start = config['start_delay_s']
            end = start + config['duration_s']
            report = {'trajectory': kind, 'disturbance': load,
                      'all': metrics(rows, 5.0, duration),
                      'response': metrics(rows, start, end + 6),
                      'recovered': metrics(rows, end + 6, duration)}
            if load:
                excited = [msg for msg in applied if nonzero(msg)]
                require(excited, 'Profile did not apply any external load')
                require(stamp(excited[0]) - epoch >= start - 0.03
                        and stamp(excited[0]) - epoch < start + 0.1,
                        'Disturbance started outside its scheduled window')
                require(stamp(excited[-1]) - epoch <= end + 0.1,
                        'Disturbance continued beyond its scheduled window')
                if config['profile'] == 'gaussian':
                    report['noise'] = check_gaussian(logs, label, commands, applied, epoch, config)
                else:
                    expected_force = math.hypot(config['force_x_n'], config['force_y_n'])
                    require(all(
                        abs(math.hypot(msg.wrench.force.x, msg.wrench.force.y)
                            - expected_force) < 1e-9
                        and abs(msg.wrench.torque.z - config['torque_z_nm']) < 1e-12
                        for msg in excited),
                            'Applied external wrench differs from the requested pulse')
            else:
                require(not any(nonzero(msg) for msg in applied), 'Baseline had an external load')
            print(json.dumps(report, indent=2), flush=True)
            (logs / f'{label}.json').write_text(json.dumps(report, indent=2) + '\n')
            require(report['all']['xy_max_m'] < 0.06 and report['all']['yaw_max_rad'] < 0.12,
                    f'{label}: peak error exceeds test tolerance; inspect saved CSV/JSON')
            require(report['recovered']['xy_rmse_m'] < 0.01 and
                    report['recovered']['yaw_rmse_rad'] < 0.035,
                    f'{label}: tracking did not recover after the load; inspect saved CSV/JSON')
            return report, rows
    finally:
        node.destroy_node()
        rclpy.shutdown()


def plot_comparison(logs, kind, baseline, loaded, config):
    """Save a standalone error/force figure alongside the raw CSV files."""
    os.environ.setdefault('MPLCONFIGDIR', str(logs / 'matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for rows, label, color in [(baseline, 'No disturbance', '0.5'),
                               (loaded, f"External {config['profile']}", 'tab:blue')]:
        data = np.array(rows)
        axes[0].plot(data[:, 0], data[:, 7] * 1000, label=label, color=color)
        axes[1].plot(data[:, 0], np.degrees(data[:, 8]), label=label, color=color)
    data = np.array(loaded)
    axes[2].plot(data[:, 0], np.hypot(data[:, 9], data[:, 10]), color='tab:red', label='Force [N]')
    axes[3].plot(data[:, 0], data[:, 11], color='tab:orange', label='Torque [Nm]')
    axes[3].set_ylabel('External torque [Nm]', color='tab:orange')
    axes[0].set_ylabel('XY error [mm]')
    axes[1].set_ylabel('Yaw error [deg]')
    axes[2].set_ylabel('External force [N]', color='tab:red')
    axes[3].set_xlabel('Time from trajectory start [s]')
    for axis in axes:
        axis.axvspan(config['start_delay_s'], config['start_delay_s'] + config['duration_s'],
                     color='tab:red', alpha=0.1)
        axis.grid(alpha=0.3)
    axes[0].legend()
    axes[0].set_title(f'{kind}: fixed-yaw trajectory with/without an external force and torque')
    figure.tight_layout()
    figure.savefig(logs / f'{kind}_comparison.png', dpi=160)
    plt.close(figure)


def main():
    """Run the ROS protocol check and paired line/circle disturbance experiments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', choices=['both', 'line', 'circle'], default='both')
    parser.add_argument('--domain-id', type=int, default=210)
    parser.add_argument('--period', type=float, default=20.0)
    parser.add_argument('--radius', type=float, default=0.2)
    parser.add_argument('--plot', action='store_true', help='Also save matplotlib comparison PNGs')
    parser.add_argument('--input-only', action='store_true', help='Only ROS input/impulse checks')
    parser.add_argument('--disturbance-params', type=Path,
                        default=ROOT / 'src/airtable_bringup/config/disturbance.yaml')
    args = parser.parse_args()
    kinds = [] if args.input_only else (
        ['line', 'circle'] if args.trajectory == 'both' else [args.trajectory])
    require(0 <= args.domain_id <= 232 - 2 * len(kinds), 'Invalid test ROS domain range')
    require(math.isfinite(args.period) and args.period > 0 and
            math.isfinite(args.radius) and args.radius > 0, 'Invalid trajectory settings')
    config = yaml.safe_load(args.disturbance_params.read_text())['/**']['ros__parameters']
    require(config['profile'] in ('pulse', 'gaussian') and config['wait_for_trajectory'],
            'Automatic recovery validation requires a trajectory-timed pulse or Gaussian load')
    require(config['start_delay_s'] > 5 and config['duration_s'] > 0,
            'Automatic validation needs a finite load after the five-second startup')
    duration = max(5 + args.period, config['start_delay_s'] + config['duration_s'] + 8)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    logs = Path(tempfile.mkdtemp(prefix='airtable_disturbance_check_'))
    os.environ['ROS_LOG_DIR'] = str(logs / 'ros')
    print(f'Logs and results: {logs}', flush=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    check_input(logs)
    reports = []
    for i, kind in enumerate(kinds):
        rows = []
        for j, load in enumerate([False, True]):
            os.environ['ROS_DOMAIN_ID'] = str(args.domain_id + 1 + 2 * i + j)
            report, data = run_trial(args, logs, kind, load, config, duration)
            reports.append(report)
            rows.append(data)
        if args.plot:
            plot_comparison(logs, kind, rows[0], rows[1], config)
    (logs / 'summary.json').write_text(json.dumps({'profile': config, 'trials': reports}, indent=2)
                                       + '\n')
    print('External disturbance validation: PASS', flush=True)


if __name__ == '__main__':
    main()
