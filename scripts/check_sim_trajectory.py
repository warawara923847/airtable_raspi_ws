#!/usr/bin/python3
"""Measure fixed-yaw trajectory tracking and check output disable/timeout behavior."""

import argparse
from bisect import bisect_left
import copy
import json
import math
import os
from pathlib import Path
import tempfile
import time

from check_sim_model import require, running
import numpy as np


def stamp(msg):
    """Get the ROS observation timestamp in seconds."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw(msg):
    """Extract yaw from an Odometry pose."""
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def check_trajectory(args, kind, log_dir):
    """Run the production launch, relaying truth so observation loss is testable."""
    from airtable_interfaces.msg import ManualMotionCommand, ThrusterPulseCommand
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import WrenchStamped
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.parameter import Parameter
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool

    rclpy.init(args=[])
    node = rclpy.create_node('trajectory_check', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    truths, references, pulses, statuses, wrenches = [], [], [], [], []
    relay = [True]
    mutation = [None]
    state_pub = node.create_publisher(Odometry, '/robot_1/check/state', 1)

    def receive_truth(msg):
        truths.append(msg)
        if relay[0]:
            if mutation[0] is not None:
                msg = copy.deepcopy(msg)
                mutation[0](msg)
            state_pub.publish(msg)

    subscriptions = [
        node.create_subscription(Odometry, '/robot_1/sim/ground_truth', receive_truth, 100),
        node.create_subscription(Odometry, '/robot_1/trajectory/reference',
                                 references.append, 100),
        node.create_subscription(ThrusterPulseCommand, '/robot_1/actuators/thrusters/pulse',
                                 pulses.append, 100),
        node.create_subscription(Bool, '/robot_1/trajectory/active', statuses.append, 100),
        node.create_subscription(WrenchStamped, '/robot_1/wrench_cmd', wrenches.append, 100),
    ]
    manual_pub = node.create_publisher(ManualMotionCommand, '/robot_1/commands/manual/motion', 1)
    enable_client = node.create_client(SetBool, '/robot_1/trajectory/enable')

    def spin_until(predicate, process, timeout=20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            require(process.poll() is None, f'{kind} launch exited; inspect {log_dir}')
            if predicate():
                return
        raise RuntimeError(f'{kind}: condition timed out; inspect {log_dir}')

    def enable(value, process):
        require(enable_client.service_is_ready(), 'Enable service unavailable')
        future = enable_client.call_async(SetBool.Request(data=value))
        spin_until(future.done, process)
        require(future.result().success, future.result().message)

    try:
        # Allow discovery before sending anything into this dedicated test domain.
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        require(all(sub.get_publisher_count() == 0 for sub in subscriptions),
                'Test ROS domain is occupied; use another --domain-id')
        command = ['ros2', 'launch', 'airtable_bringup', 'trajectory.launch.py',
                   f'trajectory:={kind}', 'viewer:=false', 'state_topic:=check/state',
                   f'fixed_yaw:={args.fixed_yaw}', f'period:={args.period}']
        if args.radius is not None:
            command.append(f'radius:={args.radius}')
        with running(command, log_dir, kind) as process:
            spin_until(lambda: bool(references) and bool(statuses) and statuses[-1].data
                       and enable_client.service_is_ready()
                       and manual_pub.get_subscription_count() == 1, process)
            first_time = stamp(references[0])
            # Manual pulses must be rejected while autonomous mode owns the output.
            manual_pub.publish(ManualMotionCommand(
                command=ManualMotionCommand.FORWARD,
                pulse_duration=Duration(sec=0, nanosec=200_000_000)))
            duration = 5.0 + args.cycles * args.period
            spin_until(lambda: stamp(references[-1]) - first_time >= duration,
                       process, timeout=max(60.0, duration * 3))
            require(subscriptions[0].get_publisher_count() == 1, 'Multiple simulators')
            require(pulses, 'No actuator pulses')
            require(all(msg.pulse_duration.sec == 0 and
                        abs(msg.pulse_duration.nanosec - 10_000_000) <= 1 for msg in pulses),
                    'Wrong modulation quantum or manual command leaked through')
            times = [stamp(msg) for msg in truths]
            xy, heading = [], []
            actual_xy = []
            for reference in references:
                if stamp(reference) - first_time < 5.0:
                    continue
                insertion = bisect_left(times, stamp(reference))
                candidates = range(max(0, insertion - 1), min(len(times), insertion + 1))
                index = min(candidates, key=lambda i: abs(times[i] - stamp(reference)))
                if abs(times[index] - stamp(reference)) > 0.021:
                    continue
                truth = truths[index]
                a, b = truth.pose.pose.position, reference.pose.pose.position
                xy.append(math.hypot(a.x - b.x, a.y - b.y))
                error = yaw(truth) - yaw(reference)
                heading.append(math.atan2(math.sin(error), math.cos(error)))
                actual_xy.append([a.x, a.y])
            require(len(xy) >= args.cycles * args.period * 20,
                    f'Insufficient paired samples: {len(xy)}')
            report = {
                'trajectory': kind, 'fixed_yaw_rad': args.fixed_yaw, 'samples': len(xy),
                'xy_rmse_m': float(np.sqrt(np.mean(np.square(xy)))),
                'xy_max_m': max(xy),
                'yaw_rmse_rad': float(np.sqrt(np.mean(np.square(heading)))),
                'yaw_max_rad': max(abs(error) for error in heading),
                'pulse_messages': len(pulses),
            }
            print(json.dumps(report, indent=2), flush=True)
            (log_dir / f'{kind}_metrics.json').write_text(json.dumps(report, indent=2) + '\n')
            require(report['xy_rmse_m'] < 0.01 and report['xy_max_m'] < 0.03,
                    'Position tracking tolerance exceeded')
            require(report['yaw_rmse_rad'] < 0.02 and report['yaw_max_rad'] < 0.05,
                    'Fixed-yaw tolerance exceeded')
            require(float(np.max(np.ptp(actual_xy, axis=0))) > 0.15,
                    'Vehicle did not traverse the requested trajectory')

            def no_pulses_for(seconds):
                before = len(pulses)
                target = node.get_clock().now().nanoseconds * 1e-9 + seconds
                spin_until(lambda: node.get_clock().now().nanoseconds * 1e-9 >= target, process)
                require(len(pulses) == before, 'Pulses continued after disabling')

            enable(False, process)
            spin_until(lambda: bool(statuses) and not statuses[-1].data
                       and bool(wrenches) and wrenches[-1].wrench.force.x == 0.0
                       and wrenches[-1].wrench.force.y == 0.0
                       and wrenches[-1].wrench.torque.z == 0.0, process)
            # Drain any already transmitted bounded pulse before measuring silence.
            end = node.get_clock().now().nanoseconds * 1e-9 + 0.1
            spin_until(lambda: node.get_clock().now().nanoseconds * 1e-9 >= end, process)
            no_pulses_for(0.3)
            enable(True, process)
            spin_until(lambda: statuses[-1].data, process)
            relay[0] = False
            spin_until(lambda: not statuses[-1].data, process, timeout=3)
            end = node.get_clock().now().nanoseconds * 1e-9 + 0.1
            spin_until(lambda: node.get_clock().now().nanoseconds * 1e-9 >= end, process)
            no_pulses_for(0.3)
            relay[0] = True
            no_pulses_for(0.4)
            require(not statuses[-1].data, 'Observation recovery restarted output automatically')
            for description, corrupt in [
                ('pose frame', lambda msg: setattr(msg.header, 'frame_id', 'wrong')),
                ('twist frame', lambda msg: setattr(msg, 'child_frame_id', 'wrong')),
                ('stale observation', lambda msg: setattr(
                    msg.header.stamp, 'sec', msg.header.stamp.sec - 1)),
                ('future observation', lambda msg: setattr(
                    msg.header.stamp, 'sec', msg.header.stamp.sec + 1)),
                ('nonfinite position', lambda msg: setattr(
                    msg.pose.pose.position, 'x', float('nan'))),
                ('position bounds', lambda msg: setattr(msg.pose.pose.position, 'x', 0.99)),
            ]:
                enable(True, process)
                spin_until(lambda: statuses[-1].data, process)
                mutation[0] = corrupt
                spin_until(lambda: not statuses[-1].data, process, timeout=3)
                mutation[0] = None
                end = node.get_clock().now().nanoseconds * 1e-9 + 0.1
                spin_until(lambda: node.get_clock().now().nanoseconds * 1e-9 >= end, process)
                no_pulses_for(0.2)
                print(f'{kind}: rejected {description}', flush=True)
            print(f'{kind}: PASS (tracking, manual interlock, disable/restart, '
                  'invalid/stale observations and latched disable)', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def check_modulator_watchdog(log_dir):
    """Check lost, stale and malformed setpoints without any hardware node."""
    from airtable_interfaces.msg import ThrusterPulseCommand, ThrusterSetpoint
    import rclpy
    from rclpy.time import Time
    import yaml

    params_path = Path(__file__).resolve().parents[1] / (
        'src/airtable_controller/config/thrusters.yaml')
    config = yaml.safe_load(params_path.read_text())['/**']['ros__parameters']['thrusters']
    names = config['names']
    forces = [config[name]['max_thrust_n'] for name in names]
    rclpy.init(args=[])
    node = rclpy.create_node('pulse_watchdog_check')
    pulses = []
    subscription = node.create_subscription(
        ThrusterPulseCommand, '/pulse_guard/actuators/thrusters/pulse', pulses.append, 100)
    publisher = node.create_publisher(ThrusterSetpoint,
                                      '/pulse_guard/actuators/thrusters/setpoint', 1)
    stream = [None]
    stamp_offset = [0.0]

    def send():
        if stream[0] is not None:
            msg = ThrusterSetpoint(names=stream[0][0], thrust=stream[0][1])
            msg.stamp = Time(nanoseconds=node.get_clock().now().nanoseconds +
                             int(stamp_offset[0] * 1e9)).to_msg()
            publisher.publish(msg)

    timer = node.create_timer(0.02, send)

    def spin_for(seconds, process):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
            require(process.poll() is None, 'Pulse modulator exited')

    def quiet(process):
        spin_for(0.06, process)
        before = len(pulses)
        spin_for(0.2, process)
        require(len(pulses) == before, 'Invalid/stale setpoint produced pulses')

    command = ['ros2', 'run', 'airtable_controller', 'airtable_pulse_modulator_node',
               '--ros-args', '-r', '__ns:=/pulse_guard', '--params-file', str(params_path)]
    try:
        with running(command, log_dir, 'pulse_watchdog') as process:
            deadline = time.monotonic() + 15
            while publisher.get_subscription_count() != 1:
                spin_for(0.05, process)
                require(time.monotonic() < deadline, 'Modulator discovery timeout')
            stream[0] = (names, forces)
            spin_for(0.3, process)
            require(pulses, 'Fresh setpoint did not produce pulses')
            stream[0] = None
            spin_for(0.3, process)
            quiet(process)
            for offset in [-1.0, 1.0]:
                stream[0] = (names, forces)
                stamp_offset[0] = offset
                quiet(process)
            stamp_offset[0] = 0.0
            malformed = [
                (names, forces[:-1]),
                (names[:-1] + [names[0]], forces),
                (names[:-1] + ['unknown'], forces),
                (names, [float('nan')] + forces[1:]),
                (names, [-0.1] + forces[1:]),
                (names, [forces[0] + 0.1] + forces[1:]),
                (names, [0.0] * len(names)),
            ]
            for message in malformed:
                stream[0] = (names, forces)
                before = len(pulses)
                spin_for(0.1, process)
                require(len(pulses) > before, 'Fresh commands failed to resume modulation')
                stream[0] = message
                quiet(process)
            print('Pulse watchdog: PASS (lost, stale/future, invalid and zero setpoints)',
                  flush=True)
    finally:
        node.destroy_timer(timer)
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


def main():
    """Run one or both trajectories in an unused local ROS domain."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', choices=['both', 'line', 'circle'], default='both')
    parser.add_argument('--domain-id', type=int, default=174)
    parser.add_argument('--cycles', type=float, default=2)
    parser.add_argument('--period', type=float, default=20)
    parser.add_argument('--fixed-yaw', type=float, default=0)
    parser.add_argument('--radius', type=float, help='Override circle radius [m]')
    parser.add_argument('--guards-only', action='store_true', help='Only pulse watchdog checks')
    args = parser.parse_args()
    require(0 <= args.domain_id <= 232, 'Invalid ROS domain')
    require(math.isfinite(args.cycles) and args.cycles >= 1, 'At least one cycle required')
    require(math.isfinite(args.period) and args.period > 0, 'Period must be positive')
    require(math.isfinite(args.fixed_yaw), 'Yaw must be finite')
    require(args.radius is None or (math.isfinite(args.radius) and args.radius > 0),
            'Radius must be finite and positive')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    log_dir = Path(tempfile.mkdtemp(prefix='airtable_trajectory_check_'))
    os.environ['ROS_LOG_DIR'] = str(log_dir / 'ros')
    print(f'Logs: {log_dir}; ROS domain: {args.domain_id}', flush=True)
    kinds = [] if args.guards_only else (
        ['line', 'circle'] if args.trajectory == 'both' else [args.trajectory])
    # Use separate domains per launch to avoid stale DDS discovery from the prior run.
    require(args.domain_id + len(kinds) <= 232, 'Not enough consecutive ROS domains')
    for index, kind in enumerate(kinds):
        os.environ['ROS_DOMAIN_ID'] = str(args.domain_id + index)
        check_trajectory(args, kind, log_dir)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id + len(kinds))
    check_modulator_watchdog(log_dir)


if __name__ == '__main__':
    main()
