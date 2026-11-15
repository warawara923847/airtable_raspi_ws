#!/usr/bin/python3
"""Check recording, QoS, clocks and lossless conversion in an isolated ROS domain."""

import argparse
import collections
import os
import signal
import sqlite3
import subprocess
import tempfile
import time
from contextlib import contextmanager
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CLI = ROOT / 'src/airtable_bringup/scripts/experiment_bag.py'


def require(condition, message):
    """Fail with the preserved diagnostic location."""
    if not condition:
        raise RuntimeError(message)


@contextmanager
def running(command, log):
    """Stop only this check's own child process group on failure or completion."""
    with log.open('w') as stream:
        process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            yield process
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=10)


def read_bag(path):
    """Read serialized payloads, reception timestamps and embedded definitions."""
    if path.suffix == '.db3':
        messages = collections.defaultdict(list)
        with sqlite3.connect(path.as_uri() + '?mode=ro', uri=True) as database:
            for topic, payload, stamp in database.execute(
                    'SELECT topics.name, messages.data, messages.timestamp FROM messages '
                    'JOIN topics ON messages.topic_id = topics.id ORDER BY messages.timestamp'):
                messages[topic].append((stamp, payload))
        return dict(messages), {}
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path)), rosbag2_py.ConverterOptions('', ''))
    messages = collections.defaultdict(list)
    while reader.has_next():
        topic, payload, stamp = reader.read_next()
        messages[topic].append((stamp, payload))
    definitions = {item.topic_type: item.encoded_message_definition
                   for item in reader.get_all_message_definitions()}
    reader.close()
    return dict(messages), definitions


def main():
    """Run synthetic real-time checks and a headless simulation, never hardware."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=195)
    args = parser.parse_args()
    require(0 <= args.domain_id <= 232, 'Invalid ROS domain')
    temporary = Path(tempfile.mkdtemp(prefix='airtable_bag_check_'))
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ['ROS_LOG_DIR'] = str(temporary / 'ros_logs')
    print(f'Artifacts: {temporary}; ROS domain: {args.domain_id}', flush=True)
    from airtable_interfaces.msg import ThrusterPulseCommand
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    import rclpy
    import rosbag2_py

    rclpy.init()
    node = rclpy.create_node('bag_integration_check')
    received = {'truth': 0, 'clock': 0}

    def observe(name):
        def callback(_message):
            received[name] += 1
        return callback

    subscriptions = [
        node.create_subscription(Odometry, '/robot_1/sim/ground_truth', observe('truth'), 10),
        node.create_subscription(Clock, '/clock', observe('clock'), 10),
    ]

    def spin_until(predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise RuntimeError(f'Timed out; inspect {temporary}')

    image_publisher = node.create_publisher(Image, '/sim/camera/infra1/image_rect_raw', 10)
    imu_publisher = node.create_publisher(Imu, '/robot_2/imu/data_raw', qos_profile_sensor_data)

    def publish_synthetic():
        image = Image(height=1, width=1, encoding='mono8', step=1, data=[42])
        image_publisher.publish(image)
        imu_publisher.publish(Imu())

    timer = node.create_timer(0.05, publish_synthetic)

    def recording(profile, label, extra=(), stop_signal=signal.SIGINT):
        output = temporary / label
        command = ['/usr/bin/python3', str(CLI), 'record', '--profile', profile,
                   '--output-root', str(output), '--label', label, *extra]
        with running(command, temporary / f'{label}.log') as process:
            spin_until(lambda: list(output.glob('*/bag/metadata.yaml')) or (
                any(output.glob('*/bag/*.*')) and process.poll() is None))
            # Wait for discovery and traffic, including late-joining latched TF.
            start = time.monotonic()
            spin_until(lambda: time.monotonic() - start > 4)
            require(process.poll() is None, f'Recorder exited early: {label}')
            process.send_signal(stop_signal)
            spin_until(lambda: process.poll() is not None)
            require(process.returncode == 0, f'Recorder failed: {label}')
        runs = list(output.iterdir())
        require(len(runs) == 1, f'Unexpected outputs: {runs}')
        manifest = yaml.safe_load((runs[0] / 'experiment.yaml').read_text())
        require(manifest['status'] == 'completed', f'Incomplete {label}')
        return runs[0], manifest, read_bag(runs[0] / 'bag')

    try:
        command = ['ros2', 'launch', 'airtable_bringup', 'simulation_trajectory.launch.py',
                   'trajectory:=circle', 'radius:=0.20', 'viewer:=false', 'rviz:=false']
        with running(command, temporary / 'simulation.log') as simulation:
            spin_until(lambda: received['truth'] > 30 and received['clock'] > 30)
            run, manifest, (messages, definitions) = recording(
                'sim', 'circle', ['--meta', 'radius_m=0.20', '--snapshot', str(
                    ROOT / 'src/airtable_controller/config/trajectory.yaml')], signal.SIGTERM)
            require(simulation.poll() is None and subscriptions and timer, 'Simulation stopped')
            for topic in ('/clock', '/tf_static', '/robot_1/sim/ground_truth',
                          '/robot_1/imu_1', '/robot_1/actuators/thrusters/pulse'):
                require(messages.get(topic), f'Missing simulation topic {topic}')
            require('/sim/camera/infra1/image_rect_raw' not in messages, 'Images not opt-in')
            require(max(stamp for stamp, _ in messages['/robot_1/sim/ground_truth']) < 10**15,
                    'Simulation recorded wall time')
            require(definitions.get('airtable_interfaces/msg/ThrusterPulseCommand'),
                    'MCAP missing custom message schema')
            require(manifest['metadata']['radius_m'] == '0.20' and manifest['inputs'],
                    'Experiment conditions or snapshots missing')
            _, _, (with_images, _) = recording(
                'sim', 'images', ['--include-group', 'sim_images'])
            require(with_images.get('/sim/camera/infra1/image_rect_raw'), 'No opted-in images')
        _, real_manifest, (real_messages, _) = recording(
            'real', 'real_fixture', ['--namespace', 'robot_2'])
        require(real_messages.get('/robot_2/imu/data_raw'), 'Best-effort IMU not recorded')
        require('/clock' not in real_messages, 'Real profile captured /clock')
        require(min(stamp for stamp, _ in real_messages['/robot_2/imu/data_raw']) > 10**18,
                'Real profile did not record wall time')
        require(not real_manifest['use_sim_time'], 'Real profile has simulation time enabled')

        config_path = ROOT / 'src/airtable_bringup/config/recording.yaml'
        empty_config = yaml.safe_load(config_path.read_text())
        empty_config['profiles']['empty'] = {
            'use_sim_time': False, 'groups': [], 'topics': ['/not_published_for_bag_test']}
        empty_path = temporary / 'empty.yaml'
        empty_path.write_text(yaml.safe_dump(empty_config))
        output = temporary / 'empty_recording'
        command = ['/usr/bin/python3', str(CLI), 'record', '--config', str(empty_path),
                   '--profile', 'empty', '--output-root', str(output)]
        with running(command, temporary / 'empty.log') as process:
            spin_until(lambda: any(output.glob('*/bag/*.mcap')))
            start = time.monotonic()
            spin_until(lambda: time.monotonic() - start > 1)
            process.send_signal(signal.SIGINT)
            spin_until(lambda: process.poll() is not None)
            require(process.returncode == 1, 'Empty recording was accepted as successful')
        empty_manifest = yaml.safe_load(next(output.glob('*/experiment.yaml')).read_text())
        require(empty_manifest['status'] == 'failed' and
                empty_manifest['recording']['message_count'] == 0, 'Empty status not preserved')

        # Split SQLite fixture: custom type, hidden topic and exact timestamp/payload checks.
        source = temporary / 'sqlite_source'
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(source), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        for topic, type_name in [('/pulse', 'airtable_interfaces/msg/ThrusterPulseCommand'),
                                 ('/_hidden', 'std_msgs/msg/String')]:
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=0, name=topic, type=type_name, serialization_format='cdr'))
        for index in range(6):
            if index == 3:
                writer.split_bagfile()
            pulse = ThrusterPulseCommand(names=['thruster_1'])
            pulse.pulse_duration.nanosec = (index + 1) * 1000000
            writer.write('/pulse', serialize_message(pulse), 1000000000 + index)
            writer.write('/_hidden', serialize_message(String(data=str(index))), 2000000000 + index)
        writer.close()
        require(len(list(source.glob('*.db3'))) == 2, 'Fixture was not split')
        original, _ = read_bag(source)
        original_metadata = (source / 'metadata.yaml').read_bytes()
        for input_path in (source, next(source.glob('*.db3')), run / 'bag',
                           next((run / 'bag').glob('*.mcap'))):
            output = temporary / f'converted_{input_path.name}'
            result = subprocess.run([
                '/usr/bin/python3', str(CLI), 'convert', str(input_path),
                '--output-root', str(output)], capture_output=True, text=True, timeout=30)
            require(result.returncode == 0, f'Conversion failed: {result.stdout}\n{result.stderr}')
            converted = next(output.iterdir())
            actual, schemas = read_bag(converted / 'bag')
            expected = messages if input_path.suffix == '.mcap' else read_bag(input_path)[0]
            require(actual == expected, f'Conversion changed messages/timestamps: {input_path}')
            require(schemas.get('airtable_interfaces/msg/ThrusterPulseCommand'),
                    'Converted MCAP missing custom message schema')
            require((source / 'metadata.yaml').read_bytes() == original_metadata,
                    'Source metadata was changed')
        require(len(original['/pulse']) == 6 and len(original['/_hidden']) == 6,
                'Split fixture unexpectedly empty')
        print('Experiment bag: PASS (sim/real clocks, TF, QoS, images, stop, split conversion)',
              flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
