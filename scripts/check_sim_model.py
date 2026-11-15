#!/usr/bin/python3
"""Check thruster geometry, C++ command directions and headless MuJoCo dynamics."""

import argparse
from contextlib import contextmanager
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    """Fail checks even when Python assertions are disabled."""
    if not condition:
        raise RuntimeError(message)


def numbers(element, attribute, default):
    """Read an MJCF numeric attribute."""
    return np.array([float(value) for value in element.get(attribute, default).split()])


def rotation(angle):
    """Rotate a planar vector counterclockwise."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def check_geometry(model_path, params_path, description_path, control_params_path=None,
                   body='robot_1', prefix='', params=None, control_model=None):
    """Independently compare allocator YAML with the site/motor geometry in MJCF.

    ``params``/``control_model`` take already merged values (robot configurations);
    otherwise they are read from ``params_path``/``control_params_path``.
    """
    tree = ET.parse(model_path).getroot()
    if params is None:
        params = yaml.safe_load(params_path.read_text())['/**']['ros__parameters']['thrusters']
    robot = tree.find(f".//body[@name='{body}']")
    require(robot is not None, f'Missing {body} body')
    compiler = tree.find('compiler')
    angle_scale = math.pi / 180 if compiler is None or (
        compiler.get('angle', 'degree') == 'degree'
    ) else 1.0
    sites = {}

    def transform(element, position, angle):
        # This checker deliberately accepts the explicit planar MJCF used here.
        # Fail on other rotation encodings instead of silently misinterpreting them.
        require(not any(key in element.attrib for key in (
            'quat', 'axisangle', 'xyaxes', 'zaxis', 'childclass'
        )), f'Unsupported MJCF transform: {element.attrib}')
        euler = numbers(element, 'euler', '0 0 0') * angle_scale
        require(np.allclose(euler[:2], 0), 'Only planar site/mount rotations supported')
        return (position + rotation(angle) @ numbers(element, 'pos', '0 0 0')[:2],
                angle + euler[2])

    def collect(body, position, angle):
        for site in body.findall('site'):
            sites[site.get('name')] = transform(site, position, angle)
        for child in body.findall('body'):
            collect(child, *transform(child, position, angle))

    collect(robot, np.zeros(2), 0.0)
    motors = tree.findall('./actuator/motor')
    names = params['names']
    require(len(names) == len(set(names)) == 4, 'Expected four unique thrusters')
    sites = {name[len(prefix):]: value for name, value in sites.items()
             if name.startswith(prefix)}
    motors = [motor for motor in motors if motor.get('site', '').startswith(prefix)]
    for motor in motors:
        motor.set('site', motor.get('site')[len(prefix):])
    require({motor.get('site') for motor in motors if motor.get('site') in names}
            == set(names), 'MuJoCo/YAML thruster names differ')
    columns, limits = [], []
    for name in names:
        spec = params[name]
        matches = [motor for motor in motors if motor.get('site') == name]
        require(len(matches) == 1, f'{name}: expected one site motor')
        motor = matches[0]
        require(motor.get('refsite') is None, f'{name}: refsite unsupported')
        position, angle = sites[name]
        gear = numbers(motor, 'gear', '1 0 0 0 0 0')
        require(len(gear) == 6 and np.allclose(gear[2:], 0)
                and math.isclose(np.linalg.norm(gear[:2]), 1),
                f'{name}: expected a unit planar force gear')
        direction = rotation(angle) @ gear[:2]
        configured_direction = np.array(spec['direction'], dtype=float)
        require(np.linalg.norm(configured_direction) > 0, f'{name}: zero direction')
        configured_direction /= np.linalg.norm(configured_direction)
        require(np.allclose(spec['position'], position, atol=1e-10, rtol=0),
                f'{name}: position mismatch: YAML={spec["position"]}, MJCF={position}')
        require(np.allclose(configured_direction, direction, atol=1e-10, rtol=0),
                f'{name}: force direction mismatch: YAML={configured_direction}, '
                f'MJCF={direction}')
        limit = numbers(motor, 'ctrlrange', '0 0')
        require(motor.get('ctrllimited') == 'true' and np.allclose(
            limit, [spec['min_thrust_n'], spec['max_thrust_n']]
        ) and limit[0] == 0 and limit[1] > 0, f'{name}: thrust limits differ')
        moment = position[0] * direction[1] - position[1] * direction[0]
        columns.append([*direction, moment])
        limits.append(limit[1])
        print(f'{name}: r={position}, force={direction}, Mz/N={moment:+.6f}', flush=True)
    matrix = np.array(columns).T
    require(np.linalg.matrix_rank(matrix) == 3, 'Allocation matrix must have rank 3')
    require(np.allclose(matrix @ limits, 0, atol=1e-10),
            'Nominal symmetric thrusters should balance when all are ON')
    # The current dynamics model is one uniform box. Do not silently use this
    # inertia calculation if an inertial override or other massive geoms appear.
    require(not robot.findall('.//inertial'), 'Explicit inertia needs an updated checker')
    massive = [geom for geom in robot.findall('.//geom')
               if geom.get('mass') != '0']
    require(len(massive) == 1 and massive[0].get('type') == 'box',
            'Expected one massive uniform body box')
    geom = massive[0]
    require(np.allclose(numbers(geom, 'pos', '0 0 0'), 0), 'Expected centered body box')
    require(not any(key in geom.attrib for key in ('quat', 'euler', 'axisangle')),
            'Expected axis-aligned body box')
    mass = float(geom.get('mass'))
    half_size = numbers(geom, 'size', '')
    description = ET.parse(description_path).getroot()
    base_box = description.find("./link[@name='base_link']/visual/geometry/box")
    require(base_box is not None and np.allclose(
        numbers(base_box, 'size', ''), 2 * half_size, atol=1e-10, rtol=0),
        'URDF base visual dimensions differ from MuJoCo')
    iz = mass / 3 * (half_size[0] ** 2 + half_size[1] ** 2)
    model = control_model
    if model is None and control_params_path is not None:
        model = yaml.safe_load(control_params_path.read_text())[
            '/**']['ros__parameters']['model']
    if model is not None:
        require(math.isclose(model['mass_kg'], mass, rel_tol=1e-9)
                and math.isclose(model['inertia_kg_m2'], iz, rel_tol=1e-9),
                'Trajectory controller mass/inertia differ from MuJoCo')
    print(f'Geometry ({body}): PASS; mass={mass:g} kg, Iz={iz:.9f} kg m^2', flush=True)
    return names, matrix, np.array(limits), mass, iz


@contextmanager
def running(command, log_dir, label):
    """Own and stop only the process group started by this checker."""
    with (log_dir / f'{label}.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            yield process
        finally:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


def check_ros(args, geometry):
    """Exercise actual controller outputs and independent MuJoCo motion."""
    # A separate domain avoids sending test commands to a running robot/simulator.
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    from airtable_interfaces.msg import ManualMotionCommand, ThrusterPulseCommand
    from airtable_interfaces.msg import ThrusterSetpoint
    from geometry_msgs.msg import WrenchStamped
    from nav_msgs.msg import Odometry
    import rclpy

    names, matrix, limits, mass, iz = geometry
    log_dir = Path(tempfile.mkdtemp(prefix='airtable_model_check_'))
    os.environ['ROS_LOG_DIR'] = str(log_dir / 'ros')
    print(f'ROS domain: {args.domain_id}; logs: {log_dir}', flush=True)
    rclpy.init(args=[])
    node = rclpy.create_node('airtable_model_check')
    truth, pulses, setpoints = [], [], []
    subscriptions = [
        node.create_subscription(Odometry, '/robot_1/sim/ground_truth', truth.append, 100),
        node.create_subscription(ThrusterPulseCommand, '/robot_1/actuators/thrusters/pulse',
                                 pulses.append, 10),
        node.create_subscription(ThrusterSetpoint, '/robot_1/actuators/thrusters/setpoint',
                                 setpoints.append, 10),
    ]
    pulse_pub = node.create_publisher(
        ThrusterPulseCommand, '/robot_1/actuators/thrusters/pulse', 10)
    manual_pub = node.create_publisher(
        ManualMotionCommand, '/robot_1/commands/manual/motion', 10)
    wrench_pub = node.create_publisher(WrenchStamped, '/robot_1/wrench_cmd', 10)

    def wait_for(predicate, processes):
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            require(all(process.poll() is None for process in processes),
                    f'Child process exited; inspect {log_dir}')
            if predicate():
                return
        raise RuntimeError(f'Timed out; inspect {log_dir}')

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    axes = [('forward', ManualMotionCommand.FORWARD, [1, 0, 0]),
            ('backward', ManualMotionCommand.BACKWARD, [-1, 0, 0]),
            ('left', ManualMotionCommand.LEFT, [0, 1, 0]),
            ('right', ManualMotionCommand.RIGHT, [0, -1, 0]),
            ('ccw', ManualMotionCommand.THRUSTER_ROTATE_CCW, [0, 0, 1]),
            ('cw', ManualMotionCommand.THRUSTER_ROTATE_CW, [0, 0, -1])]
    try:
        # Refuse an occupied test domain before launching either node.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        require(subscriptions[0].get_publisher_count() == 0
                and subscriptions[1].get_publisher_count() == 1
                and subscriptions[2].get_publisher_count() == 0
                and manual_pub.get_subscription_count() == 0
                and pulse_pub.get_subscription_count() == 1,
                'Test domain is occupied; select another --domain-id')
        controller = ['ros2', 'run', 'airtable_controller', 'airtable_controller_node',
                      '--ros-args', '-r', '__ns:=/robot_1', '--params-file', str(args.params_file)]
        with running(controller, log_dir, 'controller') as control:
            wait_for(lambda: wrench_pub.get_subscription_count() == 1
                     and manual_pub.get_subscription_count() == 1
                     and subscriptions[2].get_publisher_count() == 1, [control])
            for label, _, axis in axes:
                setpoints.clear()
                command = WrenchStamped()
                command.header.frame_id = 'robot_1/base_link'
                requested = np.array(axis) * [0.1, 0.1, 0.01]
                command.wrench.force.x, command.wrench.force.y = requested[:2].tolist()
                command.wrench.torque.z = float(requested[2])
                wrench_pub.publish(command)
                wait_for(lambda: bool(setpoints), [control])
                output = setpoints[-1]
                require(len(output.names) == len(output.thrust) == len(names)
                        and set(output.names) == set(names), 'Invalid allocator output')
                forces = dict(zip(output.names, output.thrust))
                forces = np.array([forces[name] for name in names])
                require(np.all(np.isfinite(forces)) and np.all(forces >= 0)
                        and np.all(forces <= limits), 'Allocator exceeded thrust limits')
                realized = matrix @ forces
                index = int(np.argmax(np.abs(axis)))
                require(realized[index] * axis[index] > 0 and np.allclose(
                    realized, requested, atol=1e-10),
                    f'Allocator {label}: unexpected wrench {realized}')
                print(f'Allocation {label}: PASS; wrench={realized}', flush=True)

            cases = [(name, None, np.eye(4)[i]) for i, name in enumerate(names)]
            cases += [(label, command, np.array(axis)) for label, command, axis in axes]
            for label, command_id, expected in cases:
                truth.clear()
                pulses.clear()
                simulator = ['ros2', 'run', 'airtable_sim', 'airtable_sim', '--ros-args',
                             '-p', 'viewer:=false', '-p', 'enable_camera:=false',
                             '-p', f'model_path:={args.model}']
                with running(simulator, log_dir, label) as sim:
                    wait_for(lambda: bool(truth) and pulse_pub.get_subscription_count() == 2,
                             [control, sim])
                    require(subscriptions[0].get_publisher_count() == 1,
                            'Expected exactly one simulator')
                    require(abs(truth[-1].twist.twist.angular.z) < 1e-10
                            and np.linalg.norm([truth[-1].twist.twist.linear.x,
                                                truth[-1].twist.twist.linear.y]) < 1e-10,
                            'Simulator did not start at rest')
                    start_time = stamp(truth[-1])
                    duration = 0.2
                    if command_id is None:
                        msg = ThrusterPulseCommand()
                        msg.names = [label]
                        msg.pulse_duration.nanosec = 200_000_000
                        pulse_pub.publish(msg)
                    else:
                        msg = ManualMotionCommand()
                        msg.command = command_id
                        msg.pulse_duration.nanosec = 200_000_000
                        manual_pub.publish(msg)
                    wait_for(lambda: bool(pulses), [control, sim])
                    pulse = pulses[-1]
                    require(len(set(pulse.names)) == len(pulse.names)
                            and set(pulse.names) <= set(names), 'Invalid pulse names')
                    require(pulse.pulse_duration.sec == 0
                            and pulse.pulse_duration.nanosec == 200_000_000,
                            'Controller changed pulse duration')
                    selected = np.array([name in pulse.names for name in names])
                    wrench = matrix @ (selected * limits)
                    if command_id is None:
                        require(np.array_equal(selected, expected), 'Wrong single-thruster pulse')
                    else:
                        index = int(np.argmax(np.abs(expected)))
                        require(wrench[index] * expected[index] > 0 and np.allclose(
                            np.delete(wrench, index), 0, atol=1e-10),
                            f'Manual {label}: wrong wrench {wrench}')
                    wait_for(lambda: stamp(truth[-1]) >= start_time + duration + 0.3,
                             [control, sim])
                    sample = truth[-1]
                    q = sample.pose.pose.orientation
                    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                     1 - 2 * (q.y * q.y + q.z * q.z))
                    twist = sample.twist.twist
                    velocity = rotation(yaw) @ [twist.linear.x, twist.linear.y]
                    measured = np.array([*velocity, twist.angular.z])
                    # Integrate the independent planar rigid-body equations during
                    # the pulse, including rotation of a single thruster's force.
                    alpha = wrench[2] / iz
                    times = (np.arange(1000) + 0.5) * duration / 1000
                    expected_velocity = sum(
                        (rotation(0.5 * alpha * t * t) @ wrench[:2] for t in times),
                        np.zeros(2)) * duration / (1000 * mass)
                    predicted = np.array([*expected_velocity, alpha * duration])
                    require(sample.header.frame_id == 'table_1'
                            and sample.child_frame_id == 'robot_1/base_link',
                            'Unexpected ground-truth frames')
                    require(np.allclose(measured, predicted, rtol=0.03, atol=1e-5),
                            f'{label}: actual velocity={measured}, predicted={predicted}')
                    print(f'Motion {label}: PASS; [vx, vy, wz]={measured}', flush=True)
                wait_for(lambda: subscriptions[0].get_publisher_count() == 0
                         and pulse_pub.get_subscription_count() == 1, [control])
                # Drain already queued samples before starting the next fresh sim.
                while truth:
                    truth.clear()
                    rclpy.spin_once(node, timeout_sec=0.05)
        print('Model consistency: PASS (geometry, 6 allocations, 4 thrusters, 6 motions)',
              flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def check_robot_config(path, description):
    """Check each configured robot's merged thruster/model values against MJCF."""
    import sys
    sys.path.insert(0, str(ROOT / 'src/airtable_bringup'))
    from airtable_bringup import robot_config as rc

    config = rc.load_robot_config(path, lambda package: ROOT / 'src' / package)
    rc.check_simulation_model(config['model'], config['robots'])
    for robot in config['robots']:
        thrusters = rc.merge(robot['thruster_base'], robot['thruster_overrides'])['thrusters']
        check_geometry(config['model'], None, description, body=robot['namespace'],
                       prefix=robot['simulation']['element_prefix'], params=thrusters,
                       control_model=rc.effective_trajectory(robot)['model'])
    tracked = [robot for robot in config['robots'] if robot['tracking']['simulation']]
    rc.check_simulation_cameras(config['model'], tracked)
    print(f'Robot configuration {path}: PASS ({len(config["robots"])} robots; simulated '
          f'cameras/markers checked for {[r["namespace"] for r in tracked]})', flush=True)


def main():
    """Run static checks, optionally followed by ROS/MuJoCo integration checks."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path,
                        default=ROOT / 'src/airtable_sim/mujoco/airtable.xml')
    parser.add_argument('--params-file', type=Path,
                        default=ROOT / 'src/airtable_controller/config/thrusters.yaml')
    parser.add_argument('--description', type=Path,
                        default=ROOT / 'src/airtable_description/urdf/airtable.urdf.xacro')
    parser.add_argument('--control-params', type=Path,
                        default=ROOT / 'src/airtable_controller/config/trajectory.yaml')
    parser.add_argument('--static-only', action='store_true', help='No ROS nodes or physics run')
    parser.add_argument('--robot-config', type=Path,
                        help='Statically check every robot of a robot configuration against '
                             'its simulation.model (merged per-robot parameters)')
    parser.add_argument('--domain-id', type=int, default=173, help='Unused local test domain')
    parser.add_argument('--timeout', type=float, default=20, help='Timeout per condition [s]')
    args = parser.parse_args()
    require(0 <= args.domain_id <= 232, 'domain-id must be in 0..232')
    require(math.isfinite(args.timeout) and args.timeout > 0, 'timeout must be positive')
    if args.robot_config is not None:
        check_robot_config(args.robot_config, args.description)
        return
    geometry = check_geometry(args.model, args.params_file, args.description, args.control_params)
    if not args.static_only:
        check_ros(args, geometry)


if __name__ == '__main__':
    main()
