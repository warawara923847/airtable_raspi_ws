#!/usr/bin/python3
"""Record reusable experiment profiles and convert existing ROS 2 bags to MCAP."""

import argparse
import hashlib
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import uuid
from contextlib import contextmanager
from datetime import datetime
from datetime import timezone
from pathlib import Path

import yaml


def timestamp():
    """Return an unambiguous wall-clock timestamp, independent of ROS time."""
    return datetime.now(timezone.utc).isoformat()


def write_yaml(path, value):
    """Atomically replace a YAML document so status remains readable."""
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(yaml.safe_dump(value, allow_unicode=True, sort_keys=False))
    temporary.replace(path)


def mapping(value, name, allowed=None):
    """Reject misspelled options and non-mapping YAML values."""
    if not isinstance(value, dict) or not all(isinstance(k, str) for k in value):
        raise ValueError(f'{name} must be a mapping with string keys')
    if allowed is not None and set(value) - set(allowed):
        raise ValueError(f'Unknown {name} keys: {sorted(set(value) - set(allowed))}')
    return value


def string_list(value, name):
    """Validate topic/group lists before passing anything to rosbag2."""
    if not isinstance(value, list) or not all(isinstance(v, str) and v for v in value):
        raise ValueError(f'{name} must be a list of nonempty strings')
    return value


def ros_name(value):
    """Require fully qualified topic names without unresolved substitutions."""
    if not re.fullmatch(r'(/[A-Za-z_][A-Za-z0-9_]*)+', value):
        raise ValueError(f'Invalid absolute ROS topic name: {value!r}')
    return value


def default_config():
    """Find the source-tree configuration or the installed package share."""
    source = Path(__file__).resolve().parents[1] / 'config' / 'recording.yaml'
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('airtable_bringup')) / 'config' / 'recording.yaml'


def resolve_profile(args):
    """Resolve a strict YAML profile into concrete topics and recorder options."""
    path = Path(args.config).expanduser().resolve() if args.config else default_config()
    config = mapping(yaml.safe_load(path.read_text()), 'config', [
        'version', 'defaults', 'topic_groups', 'profiles', 'qos_overrides'])
    if type(config.get('version')) is not int or config['version'] != 1:
        raise ValueError('recording config version must be 1')
    defaults = mapping(config.get('defaults'), 'defaults', [
        'namespace', 'output_root', 'storage', 'storage_preset_profile',
        'max_bag_duration', 'max_bag_size'])
    groups = mapping(config.get('topic_groups'), 'topic_groups')
    profiles = mapping(config.get('profiles'), 'profiles')
    if args.profile not in profiles:
        raise ValueError(f'Unknown profile {args.profile!r}; choose from {list(profiles)}')
    profile = mapping(profiles[args.profile], f'profile {args.profile}', [
        'use_sim_time', 'groups', 'topics'])
    if type(profile.get('use_sim_time')) is not bool:
        raise ValueError('profile.use_sim_time must be true or false')
    namespace = args.namespace if args.namespace is not None else defaults.get('namespace')
    if not isinstance(namespace, str):
        raise ValueError('namespace must be a string')
    prefix = '/' + namespace.strip('/') if namespace.strip('/') else ''
    if prefix:
        ros_name(prefix)

    def expand(topic):
        return ros_name(topic.replace('{ns}', prefix))

    selected = string_list(profile.get('groups', []), 'profile.groups') + args.include_group
    topics = []
    for group in dict.fromkeys(selected):
        if group not in groups:
            raise ValueError(f'Unknown topic group {group!r}; choose from {list(groups)}')
        topics.extend(string_list(groups[group], f'topic_groups.{group}'))
    topics.extend(string_list(profile.get('topics', []), 'profile.topics'))
    topics = list(dict.fromkeys(expand(topic) for topic in topics))
    if not topics:
        raise ValueError('The selected profile has no topics')
    if profile['use_sim_time'] and '/clock' not in topics:
        raise ValueError('A simulation-time profile must include /clock for replay')
    storage = args.storage or defaults.get('storage', 'mcap')
    if storage not in ('mcap', 'sqlite3'):
        raise ValueError('storage must be mcap or sqlite3')
    preset = defaults.get('storage_preset_profile', 'zstd_fast')
    if preset not in ('none', 'zstd_fast', 'zstd_small'):
        raise ValueError('storage_preset_profile must be none, zstd_fast or zstd_small')
    for key in ('max_bag_duration', 'max_bag_size'):
        if type(defaults.get(key, 0)) is not int or defaults.get(key, 0) < 0:
            raise ValueError(f'{key} must be a nonnegative integer')
    qos = mapping(config.get('qos_overrides', {}), 'qos_overrides')
    resolved_qos = {expand(topic): mapping(value, f'qos_overrides.{topic}')
                    for topic, value in qos.items() if expand(topic) in topics}
    root = args.output_root or defaults.get('output_root', 'recordings')
    if not isinstance(root, str) or not root:
        raise ValueError('output_root must be a nonempty string')
    return {
        'config_path': str(path), 'profile': args.profile, 'namespace': prefix,
        'output_root': str(Path(root).expanduser().resolve()),
        'storage': storage, 'storage_preset_profile': preset,
        'use_sim_time': profile['use_sim_time'], 'groups': list(dict.fromkeys(selected)),
        'topics': topics, 'qos_overrides': resolved_qos,
        'max_bag_duration': defaults.get('max_bag_duration', 0),
        'max_bag_size': defaults.get('max_bag_size', 0),
    }


def record_command(config, bag, qos_path):
    """Build an argv list; shell interpolation is deliberately unnecessary."""
    command = ['ros2', 'bag', 'record', '--storage', config['storage'],
               '--output', str(bag), '--disable-keyboard-controls',
               '--max-bag-duration', str(config['max_bag_duration']),
               '--max-bag-size', str(config['max_bag_size'])]
    if config['storage'] == 'mcap':
        command += ['--storage-preset-profile', config['storage_preset_profile']]
    if config['use_sim_time']:
        command += ['--use-sim-time']
    if config['qos_overrides']:
        command += ['--qos-profile-overrides-path', str(qos_path)]
    return command + ['--topics', *config['topics']]


def run_process(command, log_path):
    """Forward stop requests as SIGINT and allow rosbag2 to flush its metadata."""
    stopped_by = None
    with log_path.open('w') as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   text=True, errors='replace', start_new_session=True)

        def stop(signum, _frame):
            nonlocal stopped_by
            if stopped_by is None:
                stopped_by = signal.Signals(signum).name
                print('\nStopping rosbag2; waiting for data and metadata to flush.', flush=True)
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass

        previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            for line in process.stdout:
                log.write(line)
                log.flush()
                print(line, end='', flush=True)
            code = process.wait()
        finally:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                process.communicate()
            process.stdout.close()
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    return code, stopped_by


def bag_summary(bag, requested=None):
    """Read actual recorded counts instead of assuming every requested topic exists."""
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    counts = {item['topic_metadata']['name']: item['message_count']
              for item in metadata['topics_with_message_count']}
    return {
        'storage': metadata['storage_identifier'], 'message_count': metadata['message_count'],
        'topic_message_counts': counts,
        'missing_or_empty_topics': [topic for topic in requested or [] if not counts.get(topic)],
        'files': metadata['relative_file_paths'],
    }


def execute(command, run_dir, manifest, requested=None):
    """Persist completion, failures and actual topic coverage beside the bag."""
    manifest.update(started_at=timestamp(), status='running', command=command)
    write_yaml(run_dir / 'experiment.yaml', manifest)
    print(f'Experiment: {run_dir}\nCommand: {shlex.join(command)}', flush=True)
    try:
        code, stopped_by = run_process(command, run_dir / 'rosbag2.log')
        manifest.update(returncode=code, stopped_by=stopped_by)
        summary = bag_summary(run_dir / 'bag', requested)
        manifest['recording'] = summary
        success = (code == 0 or (stopped_by and code in (-signal.SIGINT, 130)))
        if not success:
            raise RuntimeError(f'rosbag2 exited with status {code}')
        if summary['message_count'] == 0:
            raise RuntimeError('No messages recorded; check topics, ROS domain and /clock')
        expected = manifest.get('input_topic_message_counts')
        if expected is not None:
            actual = {name: count for name, count in summary['topic_message_counts'].items()
                      if count > 0}
            if actual != expected:
                raise RuntimeError('Converted topic counts differ from the input bag')
        manifest['status'] = 'completed'
        missing = summary['missing_or_empty_topics']
        if missing:
            print('Requested topics with no messages (may be optional): ' + ', '.join(missing))
        print(f"Saved {summary['message_count']} messages to {run_dir / 'bag'}")
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, yaml.YAMLError) as error:
        manifest.update(status='failed', error=str(error))
        print(f'Error: {error}. Inspect {run_dir}', file=sys.stderr)
        return 1
    finally:
        manifest['finished_at'] = timestamp()
        write_yaml(run_dir / 'experiment.yaml', manifest)


def git_context():
    """Capture repository identity without copying diffs or the environment."""
    if not shutil.which('git'):
        return None
    revision = subprocess.run(['git', 'rev-parse', 'HEAD'], capture_output=True, text=True)
    if revision.returncode:
        return None
    status = subprocess.run(['git', 'status', '--porcelain'], capture_output=True, text=True)
    return {'revision': revision.stdout.strip(), 'dirty': bool(status.stdout.strip())}


def new_run(root, label):
    """Use exclusive creation and a unique suffix to keep previous experiments intact."""
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', label):
        raise ValueError('label must be 1-80 letters, digits, underscores, dots or hyphens')
    name = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    root.mkdir(parents=True, exist_ok=True)
    destination = root / f'{name}_{label}_{uuid.uuid4().hex[:6]}'
    destination.mkdir()
    return destination


def record(args):
    """Record a profile, saving resolved settings and explicitly selected input files."""
    config = resolve_profile(args)
    metadata = {}
    for value in args.meta:
        key, separator, content = value.partition('=')
        if not separator or not key:
            raise ValueError('--meta requires KEY=VALUE')
        metadata[key] = content
    snapshots = [Path(value).expanduser().resolve(strict=True) for value in args.snapshot]
    if any(not path.is_file() for path in snapshots):
        raise ValueError('--snapshot accepts files, not directories')
    if args.dry_run:
        print(yaml.safe_dump(config, sort_keys=False), end='')
        print(shlex.join(record_command(config, Path('<run>/bag'), Path('<run>/qos.yaml'))))
        return 0
    run_dir = new_run(Path(config['output_root']), args.label)
    write_yaml(run_dir / 'resolved_recording.yaml', config)
    write_yaml(run_dir / 'qos.yaml', config['qos_overrides'])
    shutil.copyfile(config['config_path'], run_dir / 'recording_source.yaml')
    saved_inputs = []
    for index, source in enumerate(snapshots):
        destination = run_dir / 'inputs' / f'{index:03d}_{source.name}'
        destination.parent.mkdir(exist_ok=True)
        shutil.copyfile(source, destination)
        saved_inputs.append({'source': str(source), 'saved': str(destination.relative_to(run_dir)),
                             'sha256': hashlib.sha256(destination.read_bytes()).hexdigest()})
    manifest = {
        'version': 1, 'operation': 'record', 'label': args.label, 'note': args.note,
        'metadata': metadata, 'profile': config['profile'], 'namespace': config['namespace'],
        'use_sim_time': config['use_sim_time'], 'git': git_context(), 'inputs': saved_inputs,
        'environment': {key: os.environ.get(key) for key in (
            'ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY',
            'ROS_AUTOMATIC_DISCOVERY_RANGE')},
    }
    return execute(record_command(config, run_dir / 'bag', run_dir / 'qos.yaml'),
                   run_dir, manifest, config['topics'])


@contextmanager
def conversion_input(source):
    """Isolate single segments from Jazzy's embedded whole-bag metadata paths."""
    from rosbag2_py import Info
    from rosbag2_py import MetadataIo
    storage = ''
    if source.is_file():
        storage = {'.db3': 'sqlite3', '.mcap': 'mcap'}[source.suffix]
    metadata = Info().read_metadata(str(source), storage)
    if source.is_dir():
        yield source, metadata
        return
    # SQLite segments embed paths for the whole recording. Restrict this temporary
    # view to the explicitly requested file; leave all original metadata untouched.
    with tempfile.TemporaryDirectory(prefix='airtable_bag_convert_') as temporary:
        staging = Path(temporary)
        (staging / source.name).symlink_to(source)
        metadata.relative_file_paths = [source.name]
        metadata.files = []
        MetadataIo().write_metadata(str(staging), metadata)
        yield staging, metadata


def convert(args):
    """Convert all input topics to indexed MCAP without replaying robot commands."""
    source = Path(args.input).expanduser().resolve(strict=True)
    if not source.is_file() and not (source / 'metadata.yaml').is_file():
        raise ValueError('input must be a ROS 2 bag directory or storage file')
    if source.is_file() and source.suffix not in ('.mcap', '.db3'):
        raise ValueError('input file must be .mcap or .db3; use the directory for split bags')
    root = Path(args.output_root).expanduser().resolve()
    if root == source or source in root.parents:
        raise ValueError('Conversion output must be outside the input bag')
    with conversion_input(source) as (input_path, input_metadata):
        expected = {item.topic_metadata.name: item.message_count
                    for item in input_metadata.topics_with_message_count if item.message_count > 0}
        run_dir = new_run(root, args.label)
        options = {'output_bags': [{
            'uri': str(run_dir / 'bag'), 'storage_id': 'mcap', 'all_topics': True,
            'include_hidden_topics': True, 'storage_preset_profile': 'zstd_fast',
        }]}
        write_yaml(run_dir / 'convert.yaml', options)
        command = ['ros2', 'bag', 'convert', '--input', str(input_path),
                   '--output-options', str(run_dir / 'convert.yaml')]
        return execute(command, run_dir, {
            'version': 1, 'operation': 'convert', 'label': args.label, 'input': str(source),
            'input_topic_message_counts': expected})


def main(argv=None):
    """Expose independent recording and conversion commands through ros2 run."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='operation', required=True)
    recorder = subparsers.add_parser('record', help='Record a YAML topic profile')
    recorder.add_argument('--config', help='Custom recording YAML; default: package config')
    recorder.add_argument('--profile', required=True, help='Profile name in the YAML (sim/real)')
    recorder.add_argument('--namespace', help='Robot namespace; overrides the YAML default')
    recorder.add_argument('--include-group', action='append', default=[],
                          help='Additional topic group; repeat to add multiple groups')
    recorder.add_argument('--output-root', help='Output root, relative to the current directory')
    recorder.add_argument('--label', default='experiment', help='Short name for this experiment')
    recorder.add_argument('--storage', choices=['mcap', 'sqlite3'], help='Override storage format')
    recorder.add_argument('--meta', action='append', default=[], metavar='KEY=VALUE')
    recorder.add_argument('--note', default='', help='Free-form experiment notes')
    recorder.add_argument('--snapshot', action='append', default=[], metavar='FILE',
                          help='Copy an input YAML, launch file or model into the experiment')
    recorder.add_argument('--dry-run', action='store_true', help='Print settings; create no files')
    recorder.set_defaults(action=record)
    converter = subparsers.add_parser('convert', help='Convert an existing ROS 2 bag to MCAP')
    converter.add_argument('input', help='Bag directory (all split segments) or .db3/.mcap file')
    converter.add_argument('--output-root', default='recordings')
    converter.add_argument('--label', default='converted')
    converter.set_defaults(action=convert)
    args = parser.parse_args(argv)
    try:
        if not shutil.which('ros2') and not getattr(args, 'dry_run', False):
            raise ValueError('ros2 not found; source /opt/ros/jazzy/setup.bash first')
        return args.action(args)
    except (OSError, ValueError, RuntimeError, yaml.YAMLError) as error:
        parser.exit(2, f'Error: {error}\n')


if __name__ == '__main__':
    sys.exit(main())
