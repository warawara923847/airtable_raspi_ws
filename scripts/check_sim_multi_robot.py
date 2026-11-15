#!/usr/bin/python3
"""
Check robot-configuration launches: robot_2 alone, two robots, isolation, split launch.

The tracking part renders one camera per table in the dual model and compares
each robot's ArUco measurement with its own ground truth.

Runs only processes it starts, in dedicated local ROS domains, and stops only
their process groups. Results (metrics JSON, launch logs, bag) are kept in the
printed log directory.
"""

import argparse
from bisect import bisect_left
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from check_sim_model import require, running
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIGS = ROOT / 'src/airtable_bringup/config/robots'
STARTUP_S = 5.0


def stamp(msg):
    """ROS header time in seconds."""
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def yaw(msg):
    """Yaw of an Odometry pose."""
    q = msg.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def body_velocity(msg):
    """Body-frame planar velocity [vx, vy, wz] from Odometry."""
    t = msg.twist.twist
    return np.array([t.linear.x, t.linear.y, t.angular.z])


def table_velocity(msg):
    """Table-frame planar velocity; body twist rotated by the pose yaw."""
    vx, vy, wz = body_velocity(msg)
    c, s = math.cos(yaw(msg)), math.sin(yaw(msg))
    return np.array([c * vx - s * vy, s * vx + c * vy, wz])


class Session:
    """One rclpy node observing a set of robots in the current ROS domain."""

    def __init__(self, robots, relay=()):
        from airtable_interfaces.msg import ThrusterPulseCommand
        from geometry_msgs.msg import WrenchStamped
        from nav_msgs.msg import Odometry, Path as PathMsg
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Bool
        from std_srvs.srv import SetBool
        from tf2_msgs.msg import TFMessage

        self.rclpy = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node('multi_robot_check', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.data = {}
        self.relay = {name: {'on': True, 'mutation': None} for name in relay}
        self.subs = []
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf, self.tf_static = [], []
        self.subs.append(self.node.create_subscription(TFMessage, '/tf', self.tf.append, 100))
        self.subs.append(self.node.create_subscription(
            TFMessage, '/tf_static', self.tf_static.append, latched))
        self.clients = {}
        for name in robots:
            d = self.data[name] = {key: [] for key in (
                'truth', 'reference', 'pulse', 'active', 'wrench', 'applied', 'imu', 'path')}
            ns = f'/{name}'
            state_pub = None
            if name in self.relay:
                state_pub = self.node.create_publisher(Odometry, f'{ns}/check/state', 1)

            def relay_latest(msg, name=name, pub=state_pub):
                # Depth-1 subscription: a busy checker relays the newest state, never a
                # backlog that the controller would correctly reject as stale.
                relay = self.relay[name]
                relay['count'] += 1
                if relay['on']:
                    if relay['mutation'] is not None:
                        msg = copy.deepcopy(msg)
                        relay['mutation'](msg)
                    pub.publish(msg)
            if state_pub is not None:
                self.relay[name]['count'] = 0
                self.subs.append(self.node.create_subscription(
                    Odometry, f'{ns}/sim/ground_truth', relay_latest, 1))
            self.subs += [
                self.node.create_subscription(Odometry, f'{ns}/sim/ground_truth',
                                              d['truth'].append, 200),
                self.node.create_subscription(Odometry, f'{ns}/trajectory/reference',
                                              d['reference'].append, 200),
                self.node.create_subscription(ThrusterPulseCommand,
                                              f'{ns}/actuators/thrusters/pulse',
                                              d['pulse'].append, 200),
                self.node.create_subscription(Bool, f'{ns}/trajectory/active',
                                              d['active'].append, latched),
                self.node.create_subscription(WrenchStamped, f'{ns}/wrench_cmd',
                                              d['wrench'].append, 100),
                self.node.create_subscription(WrenchStamped, f'{ns}/sim/disturbance/applied',
                                              d['applied'].append, 200),
                self.node.create_subscription(Imu, f'{ns}/imu_1', d['imu'].append,
                                              rclpy.qos.qos_profile_sensor_data),
                self.node.create_subscription(PathMsg, f'{ns}/trajectory/path',
                                              d['path'].append, latched),
            ]
            self.clients[name] = self.node.create_client(SetBool, f'{ns}/trajectory/enable')
        self.SetBool = SetBool

    def close(self):
        """Destroy the node and shut rclpy down."""
        self.node.destroy_node()
        self.rclpy.shutdown()

    def now(self):
        """Return the current sim time [s]."""
        return self.node.get_clock().now().nanoseconds * 1e-9

    def spin_until(self, predicate, processes, timeout=20.0, what='condition'):
        """Spin until predicate() holds; fail if a launched process exits."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.02)
            for process in processes:
                require(process.poll() is None, f'launch exited while waiting for {what}')
            if predicate():
                return
        raise RuntimeError(f'timed out waiting for {what}')

    def spin_for(self, seconds, processes):
        """Spin for a span of simulation time."""
        end = self.now() + seconds
        self.spin_until(lambda: self.now() >= end, processes, timeout=max(10, seconds * 4),
                        what=f'{seconds}s of sim time')

    def enable(self, name, value, processes):
        """Call /<name>/trajectory/enable and require success."""
        client = self.clients[name]
        self.spin_until(client.service_is_ready, processes, what=f'{name} enable service')
        future = client.call_async(self.SetBool.Request(data=value))
        self.spin_until(future.done, processes, what=f'{name} enable response')
        require(future.result().success, f'{name}: {future.result().message}')

    def parameters(self, node_name, names, processes):
        """Read parameters of a running node."""
        from rcl_interfaces.srv import GetParameters
        client = self.node.create_client(GetParameters, f'{node_name}/get_parameters')
        self.spin_until(client.service_is_ready, processes, what=f'{node_name} parameters')
        future = client.call_async(GetParameters.Request(names=names))
        self.spin_until(future.done, processes, what=f'{node_name} parameters')
        from rclpy.parameter import parameter_value_to_python
        values = [parameter_value_to_python(v) for v in future.result().values]
        self.node.destroy_client(client)
        return dict(zip(names, values))

    def require_free_domain(self):
        """Refuse a domain that already has publishers on the observed topics."""
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        busy = [sub.topic_name for sub in self.subs if sub.get_publisher_count()]
        require(not busy, f'Test ROS domain is occupied ({busy}); use another --domain-id')


def metrics(d, start, duration):
    """Pair truth with references after startup; return tracking metrics."""
    truths = d['truth']
    times = [stamp(msg) for msg in truths]
    xy, heading, actual = [], [], []
    for reference in d['reference']:
        t = stamp(reference)
        if not start + STARTUP_S <= t <= start + STARTUP_S + duration:
            continue
        i = bisect_left(times, t)
        candidates = range(max(0, i - 1), min(len(times), i + 1))
        if not candidates:
            continue
        j = min(candidates, key=lambda k: abs(times[k] - t))
        if abs(times[j] - t) > 0.021:
            continue
        a, b = truths[j].pose.pose.position, reference.pose.pose.position
        xy.append(math.hypot(a.x - b.x, a.y - b.y))
        error = yaw(truths[j]) - yaw(reference)
        heading.append(math.atan2(math.sin(error), math.cos(error)))
        actual.append([a.x, a.y])
    require(xy, 'no paired samples')
    return {
        'samples': len(xy),
        'xy_rmse_m': float(np.sqrt(np.mean(np.square(xy)))),
        'xy_max_m': float(max(xy)),
        'yaw_rmse_rad': float(np.sqrt(np.mean(np.square(heading)))),
        'yaw_max_rad': float(max(abs(e) for e in heading)),
        'xy_extent_m': [float(v) for v in np.ptp(actual, axis=0)],
    }


def require_tracking(name, report, cycles, period):
    """Apply the existing single-robot acceptance thresholds unchanged."""
    require(report['samples'] >= cycles * period * 45,
            f'{name}: insufficient paired samples {report["samples"]}')
    require(report['xy_rmse_m'] < 0.01 and report['xy_max_m'] < 0.03,
            f'{name}: position tolerance exceeded: {report}')
    require(report['yaw_rmse_rad'] < 0.02 and report['yaw_max_rad'] < 0.05,
            f'{name}: yaw tolerance exceeded: {report}')
    require(max(report['xy_extent_m']) > 0.15, f'{name}: did not traverse the trajectory')


def launch(config, log_dir, label, *extra, file='simulation_robots.launch.py'):
    """Command line for a bringup launch with a robot configuration."""
    command = ['ros2', 'launch', 'airtable_bringup', file, f'robot_config:={config}', *extra]
    if file == 'simulation_robots.launch.py':
        command.append('viewer:=false')
    (log_dir / f'{label}.command').write_text(' '.join(command) + '\n')
    return command


def wait_tracking(session, names, processes, cycles, period):
    """Wait until every robot has run startup plus the requested cycles."""
    session.spin_until(lambda: all(session.data[n]['reference'] and session.data[n]['active']
                                   and session.data[n]['active'][-1].data for n in names),
                       processes, timeout=60, what='all robots active')
    starts = {n: stamp(session.data[n]['reference'][0]) for n in names}
    duration = cycles * period
    session.spin_until(
        lambda: all(stamp(session.data[n]['reference'][-1]) - starts[n] >= STARTUP_S + duration
                    for n in names),
        processes, timeout=max(90, 4 * (STARTUP_S + duration)), what='tracking cycles')
    return {n: metrics(session.data[n], starts[n], duration) for n in names}


def check_frames(session, names, tables):
    """Odometry, IMU, wrench, path frames and the TF tree of each robot."""
    for name, table in zip(names, tables):
        d = session.data[name]
        truth = d['truth'][-1]
        require(truth.header.frame_id == table and truth.child_frame_id == f'{name}/base_link',
                f'{name}: ground truth frames {truth.header.frame_id}/{truth.child_frame_id}')
        reference = d['reference'][-1]
        require(reference.header.frame_id == table, f'{name}: reference frame')
        require(d['path'] and d['path'][-1].header.frame_id == table, f'{name}: path frame')
        require(d['wrench'] and d['wrench'][-1].header.frame_id == f'{name}/base_link',
                f'{name}: wrench frame')
        require(d['imu'] and d['imu'][-1].header.frame_id == f'{name}/imu_1',
                f'{name}: IMU frame')
    dynamic = {(t.header.frame_id, t.child_frame_id) for m in session.tf for t in m.transforms}
    static = {(t.header.frame_id, t.child_frame_id)
              for m in session.tf_static for t in m.transforms}
    require(dynamic == {(table, f'{name}/base_link') for name, table in zip(names, tables)},
            f'unexpected dynamic TF pairs {dynamic}')
    require({('world', table) for table in tables} <= static, f'static TF {static}')
    require(len(session.node.get_publishers_info_by_topic('/clock')) == 1,
            'expected exactly one /clock publisher')
    require(len(session.node.get_publishers_info_by_topic('/tf')) == 1,
            'expected only the simulator on /tf')


def check_one_robot(args, log_dir, domain, config, robot, table, other, trajectory):
    """One robot from a configuration; the other robot must not exist."""
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    session = Session([robot, other])
    label = f'{robot}_{config.stem}'
    try:
        session.require_free_domain()
        command = launch(config, log_dir, label, f'trajectory:={trajectory}',
                         f'period:={args.period}')
        with running(command, log_dir, label) as process:
            report = wait_tracking(session, [robot], [process], args.cycles,
                                   args.period)[robot]
            report['trajectory'] = trajectory
            print(f'{config.name} {robot}:', json.dumps(report), flush=True)
            require_tracking(robot, report, args.cycles, args.period)
            check_frames(session, [robot], [table])
            topics = [name for name, _ in session.node.get_topic_names_and_types()]
            leaked = [topic for topic in topics if topic.startswith(f'/{other}/')
                      and session.node.get_publishers_info_by_topic(topic)]
            require(not leaked, f'{other} topics published: {leaked}')
            require(not session.data[other]['truth'], f'{other} truth published')
        return report
    finally:
        session.close()


def check_launch_disturbance(log_dir, domain):
    """disturbance:=true disturbance_robot:=robot_2 maps the frame and targets robot_2."""
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    names = ['robot_1', 'robot_2']
    session = Session(names)
    try:
        session.require_free_domain()
        command = launch(CONFIGS / 'dual_robot.yaml', log_dir, 'dual_disturbance',
                         'disturbance:=true', 'disturbance_robot:=robot_2')
        with running(command, log_dir, 'dual_disturbance') as process:
            d2 = session.data['robot_2']
            session.spin_until(lambda: any(m.wrench.force.x > 0.01 for m in d2['applied']),
                               [process], timeout=60, what='robot_2 scripted disturbance')
            require(all(m.wrench.force.x == m.wrench.force.y == m.wrench.torque.z == 0
                        for m in session.data['robot_1']['applied']),
                    'scripted robot_2 disturbance appeared on robot_1')
            require(d2['applied'][-1].header.frame_id == 'robot_2/base_link',
                    'applied disturbance frame')
            require(not session.node.get_publishers_info_by_topic(
                '/robot_1/sim/disturbance/wrench'), 'robot_1 disturbance publisher exists')
            require(session.data['robot_1']['active'][-1].data, 'robot_1 stopped')
        print('launch disturbance targeting robot_2: PASS', flush=True)
        return 'PASS'
    finally:
        session.close()


def write_dual_test_config(log_dir, fixed_yaw):
    """dual_robot.yaml with robot_1 state relayed by this checker and robot_2 yaw changed."""
    config = yaml.safe_load((CONFIGS / 'dual_robot.yaml').read_text())
    robots = {robot['namespace']: robot for robot in config['robots']}
    robots['robot_1']['simulation']['state_topic'] = 'check/state'
    robots['robot_2']['trajectory_overrides']['trajectory']['fixed_yaw_rad'] = fixed_yaw
    path = log_dir / 'dual_robot_check.yaml'
    path.write_text(yaml.safe_dump(config, sort_keys=False))
    return path


def record_bag(log_dir, config, seconds, processes, session):
    """Record the dual_sim profile for a span of sim time with the package CLI."""
    command = ['ros2', 'run', 'airtable_bringup', 'experiment_bag', 'record',
               '--profile', 'dual_sim', '--output-root', str(log_dir / 'recordings'),
               '--label', 'dual_check', '--snapshot', str(config),
               '--snapshot', str(ROOT / 'src/airtable_sim/mujoco/airtable_dual.xml')]
    log = (log_dir / 'recorder.log').open('w')
    recorder = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
    try:
        session.spin_for(seconds, processes)
    finally:
        os.killpg(recorder.pid, signal.SIGINT)
        try:
            recorder.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(recorder.pid, signal.SIGKILL)
            recorder.wait()
        log.close()
    runs = sorted((log_dir / 'recordings').glob('*/bag/metadata.yaml'))
    require(runs, f'no bag recorded; see {log_dir / "recorder.log"}')
    metadata = yaml.safe_load(runs[-1].read_text())['rosbag2_bagfile_information']
    counts = {item['topic_metadata']['name']: item['message_count']
              for item in metadata['topics_with_message_count']}
    expected = ['/clock', '/tf', '/tf_static'] + [
        f'/{n}/{topic}' for n in ('robot_1', 'robot_2') for topic in (
            'sim/ground_truth', 'trajectory/reference', 'trajectory/error',
            'trajectory/active', 'actuators/thrusters/setpoint', 'actuators/thrusters/pulse',
            'wrench_cmd', 'trajectory/path')]
    empty = [topic for topic in expected if not counts.get(topic)]
    require(not empty, f'bag topics without messages: {empty}')
    snapshots = list(runs[-1].parents[1].glob('inputs/*'))
    require(len(snapshots) >= 2, 'snapshots were not saved')
    return {'bag': str(runs[-1].parent), 'topics': len(counts),
            'messages': {topic: counts[topic] for topic in expected}}


def no_pulses(session, name, seconds, processes):
    """Require that a robot sends no pulses for a span of sim time."""
    session.spin_for(0.1, processes)  # drain pulses already in flight
    before = len(session.data[name]['pulse'])
    session.spin_for(seconds, processes)
    require(len(session.data[name]['pulse']) == before, f'{name}: pulses continued')


def still_tracking(session, name, seconds, processes):
    """Require active output, new pulses and small error during a span."""
    d = session.data[name]
    actives, pulses = len(d['active']), len(d['pulse'])
    start = session.now()
    session.spin_for(seconds, processes)
    require(all(msg.data for msg in d['active'][max(0, actives - 1):]),
            f'{name}: output was disabled by the other robot')
    require(len(d['pulse']) > pulses, f'{name}: no pulses while the other robot failed')
    report = metrics(d, stamp(d['reference'][0]), 1e9)
    recent = [msg for msg in d['reference'] if stamp(msg) >= start]
    require(recent, f'{name}: no references')
    window = metrics({'truth': d['truth'], 'reference': recent}, start - STARTUP_S, seconds)
    require(window['xy_max_m'] < 0.03 and window['yaw_max_rad'] < 0.05,
            f'{name}: tracking degraded while the other robot failed: {window}')
    return report


def check_dual(args, log_dir, domain):
    """Two robots, different trajectories, independent failures, commands and loads."""
    from airtable_interfaces.msg import ThrusterPulseCommand
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import WrenchStamped

    os.environ['ROS_DOMAIN_ID'] = str(domain)
    names, tables = ['robot_1', 'robot_2'], ['table_1', 'table_2']
    config = write_dual_test_config(log_dir, args.robot_2_fixed_yaw)
    session = Session(names, relay=['robot_1'])
    result = {}
    try:
        session.require_free_domain()
        command = launch(config, log_dir, 'dual', f'period:={args.period}')
        with running(command, log_dir, 'dual') as process:
            procs = [process]
            # robot_1's state comes from this checker. If discovery delayed the relay past
            # the controller's freshness check, restart robot_1 once the relay flows.
            session.spin_until(lambda: session.relay['robot_1']['count'] >= 50
                               and session.data['robot_1']['active'], procs,
                               timeout=30, what='robot_1 relay and active state')
            session.spin_for(0.3, procs)
            if not session.data['robot_1']['active'][-1].data:
                print('dual: robot_1 started before the relay; re-enabling', flush=True)
                session.enable('robot_1', True, procs)
                for key in ('reference', 'truth'):
                    session.data['robot_1'][key].clear()
            reports = wait_tracking(session, names, procs, args.cycles, args.period)
            for name in names:
                print(f'dual {name}:', json.dumps(reports[name]), flush=True)
                require_tracking(name, reports[name], args.cycles, args.period)
            result['tracking'] = reports
            check_frames(session, names, tables)
            params = {name: session.parameters(
                f'/{name}/airtable_trajectory_controller',
                ['reference_frame', 'body_frame', 'state_topic', 'trajectory.type',
                 'trajectory.fixed_yaw_rad', 'use_sim_time'], procs) for name in names}
            result['parameters'] = params
            require(params['robot_1']['trajectory.type'] == 'line'
                    and params['robot_2']['trajectory.type'] == 'circle',
                    f'per-robot trajectory types not applied: {params}')
            require(params['robot_2']['trajectory.fixed_yaw_rad'] == args.robot_2_fixed_yaw
                    and params['robot_1']['trajectory.fixed_yaw_rad'] == 0.0,
                    'per-robot override leaked or was not applied')
            require([params[n]['reference_frame'] for n in names] == tables
                    and [params[n]['body_frame'] for n in names]
                    == [f'{n}/base_link' for n in names], 'frames not derived per robot')
            require(abs(yaw(session.data['robot_2']['reference'][-1]) - args.robot_2_fixed_yaw)
                    < 1e-9 and abs(yaw(session.data['robot_1']['reference'][-1])) < 1e-9,
                    'reference yaw does not reflect the configuration')
            require(len(session.node.get_publishers_info_by_topic('/robot_1/check/state')) == 1,
                    'robot_1 does not use the configured state topic')
            if args.record_seconds > 0:
                result['recording'] = record_bag(log_dir, config, args.record_seconds,
                                                 procs, session)
                print('recording:', json.dumps(result['recording']), flush=True)

            # 1. robot_1 observation lost: only robot_1 stops.
            session.relay['robot_1']['on'] = False
            session.spin_until(lambda: not session.data['robot_1']['active'][-1].data, procs,
                               timeout=5, what='robot_1 disabled after state loss')
            no_pulses(session, 'robot_1', 0.5, procs)
            still_tracking(session, 'robot_2', 2.0, procs)
            session.relay['robot_1']['on'] = True
            no_pulses(session, 'robot_1', 0.4, procs)
            require(not session.data['robot_1']['active'][-1].data,
                    'robot_1 restarted without an explicit enable')
            print('isolation: robot_1 state loss stopped only robot_1', flush=True)
            # 2. robot_1 wrong frame (e.g. table_2): only robot_1 stops.
            session.enable('robot_1', True, procs)
            session.spin_until(lambda: session.data['robot_1']['active'][-1].data, procs,
                               what='robot_1 re-enabled')
            session.relay['robot_1']['mutation'] = lambda msg: setattr(
                msg.header, 'frame_id', 'table_2')
            session.spin_until(lambda: not session.data['robot_1']['active'][-1].data, procs,
                               timeout=5, what='robot_1 disabled after wrong frame')
            no_pulses(session, 'robot_1', 0.3, procs)
            still_tracking(session, 'robot_2', 1.0, procs)
            session.relay['robot_1']['mutation'] = None
            print('isolation: robot_1 table_2 frame rejected, robot_2 unaffected', flush=True)
            # 3. robot_2 disabled by service: robot_1 continues.
            session.spin_for(0.1, procs)  # a fresh valid observation is needed to re-enable
            session.enable('robot_1', True, procs)
            session.spin_until(lambda: session.data['robot_1']['active'][-1].data, procs,
                               what='robot_1 re-enabled')
            session.spin_for(1.0, procs)  # let the restarted robot settle
            session.enable('robot_2', False, procs)
            session.spin_until(lambda: not session.data['robot_2']['active'][-1].data, procs,
                               what='robot_2 disabled')
            no_pulses(session, 'robot_2', 0.5, procs)
            d = session.data['robot_1']
            pulses = len(d['pulse'])
            session.spin_for(1.0, procs)
            require(d['active'][-1].data and len(d['pulse']) > pulses,
                    'robot_1 stopped when robot_2 was disabled')
            print('isolation: robot_2 disable left robot_1 running', flush=True)
            # 4. direct actuator command to robot_1 only.
            session.enable('robot_1', False, procs)
            session.spin_until(lambda: not d['active'][-1].data, procs, what='robot_1 disabled')
            no_pulses(session, 'robot_1', 0.3, procs)
            before = {n: table_velocity(session.data[n]['truth'][-1]) for n in names}
            pub = session.node.create_publisher(
                ThrusterPulseCommand, '/robot_1/actuators/thrusters/pulse', 10)
            session.spin_until(lambda: pub.get_subscription_count() >= 1, procs,
                               what='pulse subscriber')
            pub.publish(ThrusterPulseCommand(names=['thruster_3', 'thruster_4'],
                                             pulse_duration=Duration(nanosec=200_000_000)))
            session.spin_for(0.4, procs)
            after = {n: table_velocity(session.data[n]['truth'][-1]) for n in names}
            change = {n: (after[n] - before[n]).tolist() for n in names}
            # thruster_3+4 push +X with 2 * 0.5 N * cos45 for 0.2 s on a 1 kg body.
            require(abs(change['robot_1'][0] - 0.2 * math.sqrt(2) * 0.5) < 0.01,
                    f'robot_1 did not respond to its command: {change}')
            require(np.allclose(change['robot_2'], 0, atol=1e-9),
                    f'robot_1 command moved robot_2: {change}')
            result['command_isolation'] = change
            print('isolation: robot_1 pulse moved only robot_1', json.dumps(change), flush=True)
            # 5. disturbance on robot_2 only; a table_1 frame for robot_2 is rejected.
            wrench_pub = session.node.create_publisher(
                WrenchStamped, '/robot_2/sim/disturbance/wrench', 10)
            session.spin_until(lambda: wrench_pub.get_subscription_count() >= 1, procs,
                               what='disturbance subscriber')
            before = {n: table_velocity(session.data[n]['truth'][-1]) for n in names}
            applied_1 = len(session.data['robot_1']['applied'])
            message = WrenchStamped()
            message.wrench.force.y = 0.05
            end = session.now() + 0.5
            accepted = []
            while session.now() < end:
                message.header.frame_id = 'table_1' if session.now() < end - 0.4 else 'table_2'
                if message.header.frame_id == 'table_2':
                    accepted.append(session.now())
                wrench_pub.publish(message)
                session.spin_for(0.02, procs)
            session.spin_for(0.6, procs)
            after = {n: table_velocity(session.data[n]['truth'][-1]) for n in names}
            change = {n: (after[n] - before[n]).tolist() for n in names}
            applied_2 = [m.wrench.force.y for m in session.data['robot_2']['applied']]
            require(max(applied_2) > 0.04, 'robot_2 disturbance was not applied')
            require(all(m.wrench.force.x == m.wrench.force.y == m.wrench.torque.z == 0
                        for m in session.data['robot_1']['applied'][applied_1:]),
                    'robot_2 disturbance appeared on robot_1')
            require(np.allclose(change['robot_1'], 0, atol=1e-9),
                    f'robot_2 disturbance moved robot_1: {change}')
            # Only table_2 messages are accepted; each lasts disturbance_timeout_s (0.25 s).
            expected = 0.05 * (accepted[-1] + 0.25 - accepted[0]) / 1.0
            require(abs(change['robot_2'][1] - expected) < 0.25 * expected,
                    f'robot_2 disturbance impulse {change}, expected vy {expected:.4f}')
            result['disturbance_isolation'] = change
            print('isolation: robot_2 disturbance applied only to robot_2',
                  json.dumps(change), flush=True)
        print('dual: PASS', flush=True)
        return result
    finally:
        session.close()


def node_names(session):
    """Fully qualified node names in the domain."""
    return {f'{ns.rstrip("/")}/{name}' for name, ns in
            session.node.get_node_names_and_namespaces()}


DUAL_CAMERAS = {
    'robot_1': ('table_1', '/sim/table_1/camera', 'sim_table_1_camera_infra1_optical_frame',
                {10, 11, 12, 13, 50}),
    'robot_2': ('table_2', '/sim/table_2/camera', 'sim_table_2_camera_infra1_optical_frame',
                {20, 21, 22, 23, 51}),
}
# single_robot.yaml keeps the original camera topics and frame.
SINGLE_CAMERAS = {
    'robot_1': ('table_1', '/sim/camera', 'sim_camera_infra1_optical_frame',
                {10, 11, 12, 13, 50}),
}


def measurement_metrics(truths, measurements):
    """Pair measurements with truth of the same stamp (both from one sim snapshot)."""
    by_stamp = {msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec: msg for msg in truths}
    xy, heading = [], []
    for msg in measurements:
        actual = by_stamp.get(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec)
        if actual is None:
            continue
        a, b = actual.pose.pose.position, msg.pose.pose.position
        xy.append(math.hypot(a.x - b.x, a.y - b.y))
        q = msg.pose.pose.orientation
        observed = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        error = observed - yaw(actual)
        heading.append(math.atan2(math.sin(error), math.cos(error)))
    require(len(xy) >= 30, f'only {len(xy)} stamp-matched measurements')
    return {
        'matched_samples': len(xy),
        'xy_rmse_m': float(np.sqrt(np.mean(np.square(xy)))),
        'xy_max_m': float(max(xy)),
        'xy_p95_m': float(np.percentile(xy, 95)),
        'yaw_rmse_rad': float(np.sqrt(np.mean(np.square(heading)))),
        'yaw_max_rad': float(max(abs(e) for e in heading)),
    }


def check_tracking(args, log_dir, domain, config, cameras, label, *extra):
    """Each table's camera: own IDs only, own frame, error against own truth."""
    import cv2
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import CameraInfo, Image

    os.environ['ROS_DOMAIN_ID'] = str(domain)
    names = list(cameras)
    session = Session(names)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    seen = {name: {'ids': set(), 'images': [], 'infos': [], 'measured': [], 'saved': False}
            for name in names}

    def image_callback(msg, name):
        record = seen[name]
        record['images'].append((stamp(msg), msg.header.frame_id, msg.width, msg.height))
        if len(record['images']) % 15 == 1:  # decode a subset; enough to list IDs
            image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
            _, ids, _ = cv2.aruco.detectMarkers(image[:, :msg.width], dictionary)
            if ids is not None:
                record['ids'].update(int(i) for i in ids.flatten())
            if not record['saved']:
                cv2.imwrite(str(log_dir / f'{label}_{name}_camera.png'), image[:, :msg.width])
                record['saved'] = True

    for name, (_, namespace, _, _) in cameras.items():
        session.subs += [
            session.node.create_subscription(
                Image, f'{namespace}/infra1/image_rect_raw',
                lambda msg, name=name: image_callback(msg, name), 5),
            session.node.create_subscription(
                CameraInfo, f'{namespace}/infra1/camera_info', seen[name]['infos'].append, 5),
            session.node.create_subscription(
                PoseWithCovarianceStamped, f'/{name}/measurements/external_pose',
                seen[name]['measured'].append, 50),
        ]
    result = {}
    try:
        session.require_free_domain()
        command = launch(config, log_dir, label, 'tracking:=true', f'period:={args.period}',
                         *extra)
        with running(command, log_dir, label) as process:
            procs = [process]
            reports = wait_tracking(session, names, procs, args.cycles, args.period)
            for name in names:
                require_tracking(name, reports[name], args.cycles, args.period)
            static = {(t.header.frame_id, t.child_frame_id)
                      for m in session.tf_static for t in m.transforms}
            for name, (table, namespace, frame, ids) in cameras.items():
                record = seen[name]
                require(('world', frame) in static, f'{name}: no static TF world->{frame}')
                require(record['ids'] == ids,
                        f'{namespace}: rendered IDs {sorted(record["ids"])}, expected {ids}')
                frames = {(f, w, h) for _, f, w, h in record['images']}
                require(frames == {(frame, 1280, 960)}, f'{namespace}: image frames {frames}')
                require({info.header.frame_id for info in record['infos']} == {frame},
                        f'{namespace}: CameraInfo frame')
                require({msg.header.frame_id for msg in record['measured']} == {table},
                        f'{name}: measurement frame is not {table}')
                times = [t for t, *_ in record['images']]
                rate = (len(times) - 1) / (times[-1] - times[0])
                report = measurement_metrics(session.data[name]['truth'], record['measured'])
                report.update({
                    'camera': namespace, 'rendered_ids': sorted(record['ids']),
                    'image_rate_hz_sim': rate,
                    'measurement_rate_hz_sim': len(record['measured']) / (
                        stamp(record['measured'][-1]) - stamp(record['measured'][0])),
                    'control': reports[name],
                    'image': str(log_dir / f'{label}_{name}_camera.png'),
                })
                print(f'{label} {name}:', json.dumps(report), flush=True)
                result[name] = report
            for name, report in result.items():
                # Same limits as check_sim_tracking.py.
                require(report['xy_max_m'] < 0.03, f'{name}: measurement position error')
                require(report['yaw_max_rad'] < 0.1, f'{name}: measurement yaw error')
        return result
    finally:
        session.close()


def check_split_launch(log_dir, domain):
    """PC control for both robots plus one dummy hardware launch per robot ("Pi")."""
    from sensor_msgs.msg import Imu

    os.environ['ROS_DOMAIN_ID'] = str(domain)
    names = ['robot_1', 'robot_2']
    session = Session(names)
    imus = {name: [] for name in names}
    for name in names:
        session.subs.append(session.node.create_subscription(
            Imu, f'/{name}/imu/data_raw', imus[name].append, 10))
    result = {}
    try:
        session.require_free_domain()
        config = CONFIGS / 'dual_robot.yaml'
        pc = launch(config, log_dir, 'pc', file='robots_trajectory_control.launch.py')
        pis = {name: launch(config, log_dir, f'pi_{name}', f'robot:={name}',
                            file='robot_hardware.launch.py') for name in names}
        with running(pc, log_dir, 'pc') as p0, \
                running(pis['robot_1'], log_dir, 'pi_robot_1') as p1, \
                running(pis['robot_2'], log_dir, 'pi_robot_2') as p2:
            procs = [p0, p1, p2]
            expected = {f'/{n}/{node}' for n in names for node in (
                'airtable_trajectory_controller', 'airtable_controller',
                'airtable_pulse_modulator', 'airtable_hardware')}
            session.spin_until(lambda: expected <= node_names(session)
                               and all(imus[n] for n in names)
                               and all(session.data[n]['active'] for n in names),
                               procs, timeout=30, what='PC and Pi nodes')
            nodes = node_names(session)
            extra = [n for n in nodes if any(k in n for k in ('teleop', 'game_controller',
                                                              'airtable_sim', 'joy'))]
            require(not extra, f'manual/sim nodes were started: {extra}')
            for name in names:
                count = sum(1 for n in nodes if n == f'/{name}/airtable_controller')
                require(count == 1, f'{name}: duplicate controllers')
                params = session.parameters(f'/{name}/airtable_trajectory_controller', [
                    'use_sim_time', 'start_enabled', 'state_topic', 'reference_frame'], procs)
                require(params == {'use_sim_time': False, 'start_enabled': False,
                                   'state_topic': 'state/odometry',
                                   'reference_frame': f'table_{name[-1]}'},
                        f'{name}: unexpected hardware control parameters {params}')
                hardware = session.parameters(f'/{name}/airtable_hardware',
                                              ['imu_frame_id', 'use_sim_time'], procs)
                require(hardware == {'imu_frame_id': f'{name}/imu_link', 'use_sim_time': False},
                        f'{name}: hardware parameters {hardware}')
                require(imus[name][-1].header.frame_id == f'{name}/imu_link',
                        f'{name}: IMU frame {imus[name][-1].header.frame_id}')
                state = session.node.get_subscriptions_info_by_topic(f'/{name}/state/odometry')
                truth = [info for info in session.node.get_subscriptions_info_by_topic(
                    f'/{name}/sim/ground_truth') if info.node_name != 'multi_robot_check']
                require(len(state) == 1 and not truth, f'{name}: wrong state input')
                pulse_subs = session.node.get_subscriptions_info_by_topic(
                    f'/{name}/actuators/thrusters/pulse')
                require(any(info.node_name == 'airtable_hardware' and
                            info.node_namespace == f'/{name}' for info in pulse_subs),
                        f'{name}: hardware does not receive its own pulses')
                require(not session.data[name]['active'][-1].data, f'{name}: enabled at start')
                result[name] = {'controller': params, 'hardware': hardware}
            require(not session.node.get_publishers_info_by_topic('/clock'),
                    'a /clock publisher exists on the hardware path')
            pulses = sum(len(session.data[n]['pulse']) for n in names)
            time.sleep(1.0)
            for _ in range(50):
                session.rclpy.spin_once(session.node, timeout_sec=0.02)
            require(sum(len(session.data[n]['pulse']) for n in names) == pulses,
                    'pulses were sent while disabled')
        print('split PC/Pi launch: PASS', json.dumps(result), flush=True)
        return result
    finally:
        session.close()


def check_launch_errors(log_dir):
    """Require configuration errors to stop ros2 launch before any node starts."""
    dual = CONFIGS / 'dual_robot.yaml'
    untracked = log_dir / 'dual_robot_2_untracked.yaml'
    text = dual.read_text()
    for line in ('      simulation_params: package://airtable_tracking/config/'
                 'simulation_tracking_robot_2.yaml\n',
                 '      simulation_camera: table_2/tracking_camera\n'):
        require(text.count(line) == 1, f'dual_robot.yaml no longer has {line!r}')
        text = text.replace(line, '')
    untracked.write_text(text)
    cases = [
        (['robot_hardware.launch.py', f'robot_config:={dual}'], 'robot:= is required'),
        (['robot_hardware.launch.py', f'robot_config:={dual}', 'robot:=robot_3'],
         'unknown robots'),
        (['simulation_robots.launch.py', f'robot_config:={untracked}', 'tracking:=true'],
         "['robot_2'] have no tracking.simulation_params"),
        (['simulation_robots.launch.py', f'robot_config:={dual}', 'disturbance:=true'],
         'disturbance_robot'),
        (['simulation_robots.launch.py', f'robot_config:={dual}', 'initial_x:=1.2',
          'initial_y:=1.0'], 'outside position_bounds_m'),
        (['simulation_robots.launch.py', f'robot_config:={CONFIGS / "single_robot.yaml"}',
          f'model_path:={ROOT / "src/airtable_sim/mujoco/airtable_robot_2.xml"}'],
         "vehicle body 'robot_1' not found"),
        (['robot_tracking.launch.py', f'robot_config:={dual}', 'robot:=robot_2'],
         'not configured'),
        (['robots_trajectory_control.launch.py', f'robot_config:={log_dir / "missing.yaml"}'],
         'robot configuration not found'),
    ]
    for index, (arguments, fragment) in enumerate(cases):
        output = subprocess.run(['ros2', 'launch', 'airtable_bringup', *arguments],
                                capture_output=True, text=True, timeout=60)
        text = output.stdout + output.stderr
        (log_dir / f'error_case_{index}.log').write_text(text)
        require(output.returncode != 0 and fragment in text,
                f'{arguments}: expected failure with {fragment!r}; got {output.returncode}')
        require('process started' not in text, f'{arguments}: a process started')
        print(f'launch error OK: {fragment}', flush=True)


def main():
    """Run the selected checks in consecutive dedicated ROS domains."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=186)
    parser.add_argument('--cycles', type=float, default=1.0)
    parser.add_argument('--period', type=float, default=20.0)
    parser.add_argument('--robot-2-only-trajectory', choices=['line', 'circle'],
                        default='line')
    parser.add_argument('--robot-2-fixed-yaw', type=float, default=0.3)
    parser.add_argument('--record-seconds', type=float, default=6.0,
                        help='Record the dual_sim profile; 0 disables')
    parser.add_argument('--only', choices=['errors', 'single', 'robot_2', 'dual',
                                           'disturbance', 'split', 'tracking'],
                        action='append', help='Run only these parts (repeatable)')
    args = parser.parse_args()
    require(0 <= args.domain_id <= 225, 'Invalid ROS domain')
    require(args.cycles >= 1 and args.period > 0, 'At least one full cycle is required')
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    log_dir = Path(tempfile.mkdtemp(prefix='airtable_multi_robot_check_'))
    os.environ['ROS_LOG_DIR'] = str(log_dir / 'ros')
    print(f'Logs: {log_dir}; ROS domains: {args.domain_id}..{args.domain_id + 6}', flush=True)
    parts = args.only or ['errors', 'single', 'robot_2', 'dual', 'disturbance', 'split',
                          'tracking']
    results = {'cycles': args.cycles, 'period_s': args.period}
    if 'errors' in parts:
        check_launch_errors(log_dir)
        results['launch_errors'] = 'PASS'
    if 'single' in parts:
        results['single_robot_1_circle'] = check_one_robot(
            args, log_dir, args.domain_id + 4, CONFIGS / 'single_robot.yaml', 'robot_1',
            'table_1', 'robot_2', 'circle')
    if 'robot_2' in parts:
        results['robot_2_only'] = check_one_robot(
            args, log_dir, args.domain_id + 1, CONFIGS / 'robot_2_only.yaml', 'robot_2',
            'table_2', 'robot_1', args.robot_2_only_trajectory)
    if 'dual' in parts:
        results['dual'] = check_dual(args, log_dir, args.domain_id + 2)
    if 'disturbance' in parts:
        results['launch_disturbance'] = check_launch_disturbance(log_dir, args.domain_id + 5)
    if 'split' in parts:
        results['split'] = check_split_launch(log_dir, args.domain_id + 3)
    if 'tracking' in parts:
        results['single_tracking'] = check_tracking(
            args, log_dir, args.domain_id + 6, CONFIGS / 'single_robot.yaml', SINGLE_CAMERAS,
            'single_tracking')
        results['single_tracking_circle'] = check_tracking(
            args, log_dir, args.domain_id + 6, CONFIGS / 'single_robot.yaml', SINGLE_CAMERAS,
            'single_tracking_circle', 'trajectory:=circle')
        results['robot_2_only_tracking'] = check_tracking(
            args, log_dir, args.domain_id + 6, CONFIGS / 'robot_2_only.yaml',
            {'robot_2': DUAL_CAMERAS['robot_2']}, 'robot_2_only_tracking')
        results['dual_tracking'] = check_tracking(
            args, log_dir, args.domain_id + 6, CONFIGS / 'dual_robot.yaml', DUAL_CAMERAS,
            'dual_tracking')
    (log_dir / 'metrics.json').write_text(json.dumps(results, indent=2) + '\n')
    print(f'ALL PASS; metrics: {log_dir / "metrics.json"}', flush=True)


if __name__ == '__main__':
    main()
