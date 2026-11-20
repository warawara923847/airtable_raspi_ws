#!/usr/bin/python3
"""
Measure the impulse of each thruster per pulse width from ArUco poses of the floating robot.

Bursts of identical pulses are fired on thruster pairs that push the robot one way and then
the opposite way (forward/backward, left/right, counterclockwise/clockwise), so the robot
stays near its start. The velocity and yaw rate are fitted on the observed pose before and
after each burst; the momentum change of all bursts of one width gives the impulse of every
thruster by least squares, and the yaw-rate changes give the yaw inertia. Over several widths
the fit J = F (w - d) gives the thrust F and the valve dead time d (docs/thruster-impulse.md).

Real robot (Pi: hardware.launch.py robot_config:=.../single_robot_thrusters.yaml,
PC: tracking.launch.py start_camera:=true; no controller may send pulses):
  scripts/measure_thruster_impulse.py --mass-kg 1.23 --thrust-n 0.5
Simulation (manual.launch.py tracking:=true start_joy:=false):
  scripts/measure_thruster_impulse.py --use-sim-time --yes

Nothing is sent before the plan is printed and confirmed (--yes skips the prompt).
--dry-run prints the plan only.
"""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import sys
import threading
import time

import numpy as np
import yaml

# Thruster pairs with a nominally pure force or torque, each followed by its opposite.
DEFAULT_SETS = [
    ('forward', ['thruster_3', 'thruster_4']), ('backward', ['thruster_1', 'thruster_2']),
    ('left', ['thruster_1', 'thruster_4']), ('right', ['thruster_2', 'thruster_3']),
    ('ccw', ['thruster_1', 'thruster_3']), ('cw', ['thruster_2', 'thruster_4']),
]


def fail(message):
    """Stop with a message; never continue firing after an unexpected state."""
    raise SystemExit(f'measure_thruster_impulse: {message}')


def ros_parameters(path):
    """Read the /** ros__parameters section of a parameter file."""
    return yaml.safe_load(Path(path).read_text())['/**']['ros__parameters']


def share(package, *parts):
    """Path inside an installed package."""
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory(package), *parts))


def thruster_geometry(path):
    """Names, unit force directions, and yaw arms (r x d) of the nominal thruster model."""
    thrusters = ros_parameters(path)['thrusters']
    names = list(thrusters['names'])
    directions, arms = [], []
    for name in names:
        r = np.asarray(thrusters[name]['position'], dtype=float)
        d = np.asarray(thrusters[name]['direction'], dtype=float)
        d = d / np.linalg.norm(d)
        directions.append(d)
        arms.append(float(r[0] * d[1] - r[1] * d[0]))
    return names, np.array(directions).T, np.array(arms), thrusters


def fit_rate(times, values):
    """Least-squares slope of values over times."""
    times = np.asarray(times) - np.mean(times)
    return float(np.polyfit(times, values, 1)[0])


class PoseLog:
    """Thread-safe record of (time, x, y, yaw) observations and of foreign pulses."""

    def __init__(self, use_receive_time):
        """Store observations from the subscription thread."""
        self.lock = threading.Lock()
        self.samples = []
        self.ages = []
        self.foreign_pulses = 0
        self.own_pending = 0
        self.use_receive_time = use_receive_time

    def add(self, node, msg):
        """Append one pose observation."""
        now = node.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = msg.pose.pose.orientation
        heading = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        with self.lock:
            self.samples.append((now if self.use_receive_time else stamp, p.x, p.y, heading))
            self.ages.append(now - stamp)

    def pulse_seen(self, _msg):
        """Count pulses that this script did not send."""
        with self.lock:
            if self.own_pending > 0:
                self.own_pending -= 1
            else:
                self.foreign_pulses += 1

    def window(self, start, end):
        """Return samples with start <= t <= end as an array (t, x, y, yaw)."""
        with self.lock:
            data = [s for s in self.samples if start <= s[0] <= end]
        return np.array(data).reshape(-1, 4)

    def latest(self):
        """Newest sample or None."""
        with self.lock:
            return self.samples[-1] if self.samples else None


def motion(log, start, end, minimum):
    """Velocity (table frame), yaw rate and samples over [start, end]."""
    data = log.window(start, end)
    if len(data) < minimum:
        fail(f'only {len(data)} pose samples in a {end - start:.2f} s window '
             f'(need {minimum}); is the marker visible and tracking running?')
    yaw = np.unwrap(data[:, 3])
    return np.array([fit_rate(data[:, 0], data[:, 1]), fit_rate(data[:, 0], data[:, 2])]), \
        fit_rate(data[:, 0], yaw), data


def rotation(angle):
    """Planar rotation matrix."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def solve_width(bursts, names, directions, arms, mass):
    """Impulse of every thruster [N s] and the yaw inertia from the bursts of one width."""
    rows, values = [], []
    for burst in bursts:
        # Momentum change in the table frame = sum over pulses of R(yaw_i) D_set J.
        selector = np.zeros((len(names), len(names)))
        for name in burst['thrusters']:
            index = names.index(name)
            selector[index, index] = 1.0
        a = sum(rotation(psi) for psi in burst['pulse_yaw']) @ directions @ selector
        rows += list(a)
        values += list(mass * np.asarray(burst['delta_v']))
    matrix = np.array(rows)
    rank = np.linalg.matrix_rank(matrix)
    if rank < len(names):
        fail(f'bursts excite only {rank} of {len(names)} thrusters; use more thruster sets')
    impulse, *_ = np.linalg.lstsq(matrix, np.array(values), rcond=None)
    predicted = matrix @ impulse
    residual = float(np.sqrt(np.mean((predicted - np.array(values)) ** 2)))
    torque_impulse = np.array([
        len(burst['pulse_yaw']) * sum(arms[names.index(n)] * impulse[names.index(n)]
                                      for n in burst['thrusters']) for burst in bursts])
    delta_w = np.array([burst['delta_yaw_rate'] for burst in bursts])
    strong = np.abs(torque_impulse) > 0.2 * np.max(np.abs(torque_impulse))
    inertia = float(np.sum(torque_impulse[strong] ** 2) /
                    np.sum(torque_impulse[strong] * delta_w[strong])) if strong.any() else None
    return impulse, residual, inertia


def main():
    """Plan, confirm, fire, fit."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument('--pose-topic', default='measurements/external_pose',
                        help='Raw ArUco pose (not the EKF output), relative to the namespace')
    parser.add_argument('--widths-ms', type=float, nargs='+', default=[10, 20, 30, 50])
    parser.add_argument('--open-ms', type=float, default=50.0,
                        help='Commanded open time per thruster and burst; sets pulses per burst')
    parser.add_argument('--gap-ms', type=float, default=60.0,
                        help='Closed time between pulses of a burst (isolated pulses)')
    parser.add_argument('--repeats', type=int, default=2, help='Thruster-set cycles per width')
    parser.add_argument('--window-s', type=float, default=1.0, help='Velocity fit window')
    parser.add_argument('--settle-s', type=float, default=0.3,
                        help='Wait after the last pulse before the after-window')
    parser.add_argument('--mass-kg', type=float,
                        help='Measured robot mass; default model.mass_kg of trajectory.yaml')
    parser.add_argument('--thrust-n', type=float,
                        help='Steady thrust F set by the user; also report d = w - J/F')
    parser.add_argument('--thruster-params', help='Default: airtable_controller thrusters.yaml')
    parser.add_argument('--trajectory-params', help='Default: airtable_controller trajectory.yaml')
    parser.add_argument('--margin-m', type=float, default=0.05,
                        help='Stop when closer than this to limits.position_bounds_m')
    parser.add_argument('--rest-speed', type=float, default=0.05,
                        help='Above this speed [m/s] wait (or ask) before the next burst')
    parser.add_argument('--rest-yaw-rate', type=float, default=0.5, help='[rad/s], as above')
    parser.add_argument('--max-speed', type=float, default=0.2, help='Abort above [m/s]')
    parser.add_argument('--max-yaw-rate', type=float, default=2.0, help='Abort above [rad/s]')
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--use-receive-time', action='store_true',
                        help='Time poses by arrival when their stamps use another clock')
    parser.add_argument('--output-dir', default='thruster_measurements')
    parser.add_argument('--yes', action='store_true', help='Do not ask before firing or '
                        'between bursts (simulation; on hardware only with a safe setup)')
    parser.add_argument('--dry-run', action='store_true', help='Print the plan and exit')
    args = parser.parse_args()

    names, directions, arms, thrusters = thruster_geometry(
        args.thruster_params or share('airtable_controller', 'config', 'thrusters.yaml'))
    trajectory = ros_parameters(
        args.trajectory_params or share('airtable_controller', 'config', 'trajectory.yaml'))
    mass = args.mass_kg or float(trajectory['model']['mass_kg'])
    bounds = trajectory['limits']['position_bounds_m']
    nominal_inertia = float(trajectory['model']['inertia_kg_m2'])
    for _, members in DEFAULT_SETS:
        missing = [n for n in members if n not in names]
        if missing:
            fail(f'thruster set needs {missing}, not in thrusters.names {names}')
    if any(not 2 <= w <= 1000 for w in args.widths_ms) or args.gap_ms < 20 \
            or args.open_ms <= 0 or args.repeats < 1 or mass <= 0:
        fail('widths must be 2-1000 ms, gap >= 20 ms, open time, repeats and mass positive')
    plan = []
    for width in args.widths_ms:
        count = max(1, round(args.open_ms / width))
        for _ in range(args.repeats):
            plan += [(width, count, label, members) for label, members in DEFAULT_SETS]
    # The second burst of each pair undoes the first, so only the first waits for rest.
    period = lambda w: (w + args.gap_ms) / 1000.0  # noqa: E731
    total = sum(args.window_s + c * period(w) + args.settle_s + args.window_s
                for w, c, _, _ in plan)
    print(f'mass {mass} kg, nominal on_thrust '
          f'{[thrusters[n].get("on_thrust_n") for n in names]} N, bounds {bounds}')
    print(f'{len(plan)} bursts, about {total:.0f} s without waiting for rest:')
    for width in args.widths_ms:
        count = max(1, round(args.open_ms / width))
        nominal = 2 * count * width / 1000 * float(thrusters[names[0]]['on_thrust_n'])
        print(f'  {width:6.1f} ms x {count:3d} pulses every {period(width) * 1000:.0f} ms per '
              f'pair; nominal pair impulse {nominal * 1000:.1f} mN s '
              f'(about {nominal * 0.7071 / mass * 100:.1f} cm/s)')
    if args.dry_run:
        return
    if not args.yes and input('Fire these pulses? Type yes: ').strip() != 'yes':
        fail('cancelled')

    from airtable_interfaces.msg import ThrusterPulseCommand
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import PoseWithCovarianceStamped
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=[])
    node = rclpy.create_node('thruster_impulse_measurement', namespace=args.namespace,
                             parameter_overrides=[
                                 Parameter('use_sim_time', value=args.use_sim_time)])
    log = PoseLog(args.use_receive_time)
    node.create_subscription(PoseWithCovarianceStamped, args.pose_topic,
                             lambda msg: log.add(node, msg), qos_profile_sensor_data)
    node.create_subscription(ThrusterPulseCommand, 'actuators/thrusters/pulse',
                             log.pulse_seen, 50)
    publisher = node.create_publisher(ThrusterPulseCommand, 'actuators/thrusters/pulse', 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def wait_until(t):
        while now() < t:
            time.sleep(0.002)

    def stop(message):
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        fail(message)

    output = Path(args.output_dir) / datetime.now().strftime('%Y%m%dT%H%M%S')
    bursts = []
    try:
        time.sleep(1.5)
        if log.latest() is None:
            stop(f'no pose on /{args.namespace}/{args.pose_topic}')
        if log.foreign_pulses:
            stop('another node is sending thruster pulses; stop the controller first')
        median_age = float(np.median(log.ages[-50:]))
        if not args.use_receive_time and not -0.05 <= median_age <= 0.5:
            stop(f'pose stamps are {median_age:.3f} s from this clock; '
                 'use --use-sim-time or --use-receive-time')
        rate = len(log.window(now() - 1.0, now())) / 1.0
        minimum = max(5, int(0.5 * rate * args.window_s))
        print(f'pose rate {rate:.0f} Hz, median age {median_age * 1000:.0f} ms', flush=True)
        for index, (width, count, label, members) in enumerate(plan):
            opposite = index % 2 == 1
            # Bounds and speed limits before every burst; rest before the first of a pair.
            while True:
                v, w, data = motion(log, now() - 0.3, now(), 3)
                speed = float(np.linalg.norm(v))
                x, y = data[-1, 1], data[-1, 2]
                if not (bounds[0] + args.margin_m <= x <= bounds[1] - args.margin_m and
                        bounds[2] + args.margin_m <= y <= bounds[3] - args.margin_m):
                    message = f'robot at ({x:.3f}, {y:.3f}) is near the bounds {bounds}'
                elif speed > args.max_speed or abs(w) > args.max_yaw_rate:
                    stop(f'robot moving too fast ({speed:.3f} m/s, {w:.2f} rad/s)')
                elif not opposite and (speed > args.rest_speed or abs(w) > args.rest_yaw_rate):
                    message = f'robot not at rest ({speed:.3f} m/s, {w:.2f} rad/s)'
                else:
                    break
                if args.yes:
                    stop(message)
                input(f'{message}. Stop/center it by hand, then press Enter: ')
                time.sleep(args.window_s)
            start = now()
            wait_until(start + args.window_s)
            pulse_times = []
            for _ in range(count):
                t = now()
                msg = ThrusterPulseCommand()
                msg.stamp = node.get_clock().now().to_msg()
                msg.names = list(members)
                ns = int(round(width * 1e6))
                msg.pulse_duration = Duration(sec=ns // 1_000_000_000,
                                              nanosec=ns % 1_000_000_000)
                with log.lock:
                    log.own_pending += 1
                publisher.publish(msg)
                pulse_times.append(t)
                wait_until(t + period(width))
            end = pulse_times[-1] + width / 1000.0
            wait_until(end + args.settle_s + args.window_s)
            if log.foreign_pulses:
                stop('another node sent thruster pulses during the measurement')
            v0, w0, before = motion(log, start, start + args.window_s, minimum)
            v1, w1, after = motion(log, end + args.settle_s,
                                   end + args.settle_s + args.window_s, minimum)
            span = log.window(start, end + args.settle_s + args.window_s)
            yaw = np.unwrap(span[:, 3])
            pulse_yaw = [float(np.interp(t + width / 2000.0, span[:, 0], yaw))
                         for t in pulse_times]
            burst = {'width_ms': width, 'pulses': count, 'set': label, 'thrusters': members,
                     'pulse_times': pulse_times, 'pulse_yaw': pulse_yaw,
                     'velocity_before': v0.tolist(), 'velocity_after': v1.tolist(),
                     'delta_v': (v1 - v0).tolist(), 'yaw_rate_before': w0,
                     'yaw_rate_after': w1, 'delta_yaw_rate': w1 - w0}
            bursts.append(burst)
            print(f'[{index + 1}/{len(plan)}] {width:5.1f} ms x{count} {label:8s} '
                  f'dv=({burst["delta_v"][0] * 100:+.2f}, {burst["delta_v"][1] * 100:+.2f}) '
                  f'cm/s, dw={burst["delta_yaw_rate"]:+.3f} rad/s', flush=True)
    except KeyboardInterrupt:
        print('interrupted; fitting the bursts measured so far', flush=True)
    finally:
        output.mkdir(parents=True, exist_ok=True)
        with log.lock:
            np.savetxt(output / 'poses.csv', np.array(log.samples).reshape(-1, 4),
                       delimiter=',', header='t,x,y,yaw')
        if rclpy.ok():
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()

    results = {'mass_kg': mass, 'names': names, 'nominal_inertia_kg_m2': nominal_inertia,
               'thrust_n_user': args.thrust_n, 'widths': []}
    impulses = {}
    for width in sorted({b['width_ms'] for b in bursts}):
        group = [b for b in bursts if b['width_ms'] == width]
        if len(group) < len(DEFAULT_SETS):
            continue
        impulse, residual, inertia = solve_width(group, names, directions, arms, mass)
        impulses[width] = impulse
        entry = {'width_ms': width, 'impulse_mN_s': (impulse * 1000).tolist(),
                 'effective_thrust_n': (impulse / (width / 1000)).tolist(),
                 'momentum_residual_mN_s': residual * 1000, 'inertia_kg_m2': inertia}
        if args.thrust_n:
            entry['dead_time_ms'] = (width - impulse / args.thrust_n * 1000).tolist()
        results['widths'].append(entry)
    if len(impulses) >= 2:
        widths = np.array(sorted(impulses)) / 1000.0
        fits = {}
        for index, name in enumerate(names):
            values = np.array([impulses[w][index] for w in sorted(impulses)])
            slope, intercept = np.polyfit(widths, values, 1)
            fits[name] = {'thrust_n': float(slope),
                          'dead_time_ms': float(-intercept / slope * 1000) if slope else None}
        results['fit_J_equals_F_times_w_minus_d'] = fits
    (output / 'bursts.json').write_text(json.dumps(bursts, indent=1) + '\n')
    (output / 'result.json').write_text(json.dumps(results, indent=2) + '\n')

    print(f'\nresults in {output}')
    header = 'width  ' + '  '.join(f'{n:>11s}' for n in names) + '   I [kg m^2]  residual'
    print('impulse per pulse [mN s] / effective thrust J/w [N]:\n' + header)
    for entry in results['widths']:
        cells = '  '.join(f'{j:5.2f}/{f:5.3f}' for j, f in
                          zip(entry['impulse_mN_s'], entry['effective_thrust_n']))
        inertia = entry['inertia_kg_m2']
        print(f'{entry["width_ms"]:5.1f}  {cells}   '
              f'{inertia if inertia is None else f"{inertia:.5f}":>10}  '
              f'{entry["momentum_residual_mN_s"]:.2f} mN s')
        if args.thrust_n:
            print('       dead time with F=%.3f N [ms]: ' % args.thrust_n +
                  ', '.join(f'{d:.1f}' for d in entry['dead_time_ms']))
    for name, fit in results.get('fit_J_equals_F_times_w_minus_d', {}).items():
        print(f'fit {name}: F = {fit["thrust_n"]:.3f} N, d = {fit["dead_time_ms"]:.1f} ms')
    return 0


if __name__ == '__main__':
    sys.exit(main())
