"""Validate recording profiles without starting ROS nodes or touching hardware."""

import argparse
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import re
import tempfile
import time
import unittest
from unittest import mock

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'experiment_bag', PACKAGE / 'scripts/experiment_bag.py')
bag = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bag)


class RecordingConfigTest(unittest.TestCase):
    """Exercise accidental over-recording, time mixups and output collisions."""

    def arguments(self, **overrides):
        """Build the options shared by profile resolution checks."""
        values = {'config': str(PACKAGE / 'config/recording.yaml'), 'profile': 'sim',
                  'namespace': [], 'include_group': [], 'output_root': None, 'storage': None,
                  'topic': []}
        return argparse.Namespace(**(values | overrides))

    def test_sim_and_real_keep_distinct_clocks(self):
        """Only simulations subscribe to /clock; images are opt-in."""
        simulation = bag.resolve_profile(self.arguments())
        hardware = bag.resolve_profile(self.arguments(profile='real'))
        self.assertTrue(simulation['use_sim_time'])
        self.assertIn('/clock', simulation['topics'])
        self.assertIn('/robot_1/sim/disturbance/wrench', simulation['topics'])
        self.assertIn('/robot_1/sim/disturbance/applied', simulation['topics'])
        self.assertNotIn('/robot_1/sim/disturbance/wrench', hardware['topics'])
        self.assertFalse(hardware['use_sim_time'])
        self.assertNotIn('/clock', hardware['topics'])
        self.assertIn('/robot_1/imu/data_raw', hardware['topics'])
        self.assertFalse(any('image' in topic for topic in simulation['topics']))

    def test_namespace_images_and_duplicate_groups(self):
        """Reuse a profile on another robot without changing global camera topics."""
        config = bag.resolve_profile(self.arguments(
            namespace=['/robot_2/'], include_group=['sim_images', 'control']))
        self.assertIn('/robot_2/actuators/thrusters/pulse', config['topics'])
        self.assertIn('/sim/camera/infra1/image_rect_raw', config['topics'])
        self.assertFalse(any('/robot_1/' in topic for topic in config['topics']))
        self.assertEqual(len(config['topics']), len(set(config['topics'])))
        self.assertEqual(config['qos_overrides']['/tf_static']['durability'], 'transient_local')

    def test_root_namespace(self):
        """An unnamespaced standalone hardware node can use the same groups."""
        config = bag.resolve_profile(self.arguments(namespace=['/'], profile='real'))
        self.assertIn('/imu/data_raw', config['topics'])

    def test_several_namespaces_regex_and_extra_topics(self):
        """record:=true of a two-robot launch: every {ns} topic and pattern once per robot."""
        recording = yaml.safe_load((PACKAGE / 'config/recording.yaml').read_text())
        recording['topic_groups']['tracking'].append(
            {'regex': '{ns}/tracking/aruco_single_[0-9]+/pose'})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory, 'recording.yaml')
            path.write_text(yaml.safe_dump(recording))
            config = bag.resolve_profile(self.arguments(
                config=str(path), namespace=['robot_1', 'robot_2', 'robot_1'],
                topic=['/sim/table_2/camera/x']))
        self.assertEqual(config['namespaces'], ['/robot_1', '/robot_2'])
        for robot in ('robot_1', 'robot_2'):
            self.assertIn(f'/{robot}/sim/ground_truth', config['topics'])
            self.assertIn(f'/{robot}/trajectory/path', config['qos_overrides'])
        self.assertEqual(config['topics'].count('/clock'), 1)
        self.assertIn('/sim/table_2/camera/x', config['topics'])
        command = bag.record_command(config, Path('/tmp/bag'), Path('/tmp/qos.yaml'))
        pattern = re.compile(command[command.index('--regex') + 1])
        self.assertTrue(pattern.search('/robot_2/tracking/aruco_single_21/pose'))
        for other in ('/robot_2/tracking/aruco_single_21/debug',
                      '/robot_3/tracking/aruco_single_21/pose', '/x/robot_1/tracking/'
                      'aruco_single_1/pose'):
            self.assertFalse(pattern.search(other), other)

    def test_fixed_markers_are_not_subscribed(self):
        """A recorded fixed-marker pose would keep its detector running (adr/0023)."""
        config = bag.resolve_profile(self.arguments())
        self.assertFalse(any('aruco_single' in topic for topic in config['topics']))
        self.assertNotIn('--regex', bag.record_command(config, Path('b'), Path('q')))

    def test_pi_hardware_profile(self):
        """The Pi records only its own inputs/outputs, with the system clock."""
        config = bag.resolve_profile(self.arguments(profile='hardware'))
        self.assertFalse(config['use_sim_time'])
        self.assertIn('/robot_1/actuators/thrusters/pulse', config['topics'])
        self.assertIn('/robot_1/actuators/reaction_wheels/speed_command', config['topics'])
        self.assertFalse(any('/trajectory/' in t or '/tracking/' in t for t in config['topics']))
        self.assertNotIn('--regex', bag.record_command(config, Path('b'), Path('q')))

    def test_reject_unknown_groups_and_invalid_names(self):
        """Catch misspellings before creating any experiment directory."""
        for values in ({'include_group': ['images_typo']}, {'namespace': ['../robot']},
                       {'topic': ['relative/topic']}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                bag.resolve_profile(self.arguments(**values))

    def test_reject_yaml_clock_and_option_typos(self):
        """Do not silently record a bad clock or ignore a misspelled split limit."""
        original = yaml.safe_load((PACKAGE / 'config/recording.yaml').read_text())
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'config.yaml'
            for mutation in ('clock', 'typo', 'boolean', 'empty', 'regex'):
                config = yaml.safe_load(yaml.safe_dump(original))
                if mutation == 'clock':
                    config['topic_groups']['simulation'].remove('/clock')
                elif mutation == 'typo':
                    config['defaults']['max_bag_duraton'] = 300
                elif mutation == 'boolean':
                    config['profiles']['sim']['use_sim_time'] = 'false'
                elif mutation == 'regex':
                    config['topic_groups']['tracking'].append({'regex': '{ns}/(unclosed'})
                else:
                    config['profiles']['sim']['groups'] = []
                path.write_text(yaml.safe_dump(config))
                with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                    bag.resolve_profile(self.arguments(config=str(path)))

    def test_dry_run_does_not_create_output(self):
        """A preview remains safe to run before an experiment."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'absent'
            with contextlib.redirect_stdout(io.StringIO()) as output:
                result = bag.main(['record', '--profile', 'sim', '--dry-run',
                                   '--output-root', str(root)])
            self.assertEqual(result, 0)
            self.assertFalse(root.exists())
            self.assertIn('--use-sim-time', output.getvalue())

    def test_record_sqlite_omits_mcap_options(self):
        """The optional SQLite recording path remains usable for older tools."""
        config = bag.resolve_profile(self.arguments(storage='sqlite3', profile='real'))
        command = bag.record_command(config, Path('/tmp/bag'), Path('/tmp/qos.yaml'))
        self.assertNotIn('--storage-preset-profile', command)
        self.assertNotIn('--use-sim-time', command)

    def test_unique_runs_and_path_traversal(self):
        """Repeated labels never overwrite existing recordings or escape the root."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = bag.new_run(root, 'circle_r020')
            (first / 'keep.txt').write_text('keep')
            second = bag.new_run(root, 'circle_r020')
            self.assertNotEqual(first, second)
            self.assertEqual((first / 'keep.txt').read_text(), 'keep')
            with self.assertRaises(ValueError):
                bag.new_run(root, '../escape')


class FoxgloveUploadTest(unittest.TestCase):
    """Upload through a stand-in CLI; nothing here reaches the network."""

    def setUp(self):
        """Put a fake foxglove first on PATH and create a two-segment run."""
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        tools = self.root / 'tools'
        tools.mkdir()
        self.calls = self.root / 'calls.txt'
        self.fail_on = self.root / 'fail_on'
        cli = tools / 'foxglove'
        self.setup_calls = self.root / 'setup_calls.txt'
        self.session_calls = self.root / 'session_calls.txt'
        self.sessions = self.root / 'sessions.txt'
        self.definitions = self.root / 'definitions.json'
        self.define_properties()
        cli.write_text('#!/bin/sh\n'
                       'case "$1 $2" in\n'
                       '  "custom-properties list") cat "$FAKE_DEFINITIONS"; exit 0 ;;\n'
                       '  "custom-properties add"|"devices add")\n'
                       '    echo "$*" >> "$FAKE_SETUP"; exit 0 ;;\n'
                       '  "devices get")\n'
                       '    if test "$3" = absent-device; then\n'
                       '      echo "Device not found: $3" >&2; exit 1\n'
                       '    fi\n'
                       '    echo \'{"id":"dev_1"}\'; exit 0 ;;\n'
                       '  "sessions get")\n'
                       '    grep -qx "$3" "$FAKE_SESSIONS" 2>/dev/null && exit 0\n'
                       '    echo "Session not found: $3" >&2; exit 1 ;;\n'
                       '  "sessions add")\n'
                       '    echo "$*" >> "$FAKE_SESSION_CALLS"; previous=\n'
                       '    for value; do\n'
                       '      test "$previous" = --key && echo "$value" >> "$FAKE_SESSIONS"\n'
                       '      previous=$value\n'
                       '    done\n'
                       '    exit 0 ;;\n'
                       '  "sessions edit") echo "$*" >> "$FAKE_SESSION_CALLS"; exit 0 ;;\n'
                       'esac\n'
                       'test "$1" = upload || exit 9\n'
                       'test -f "$2" || exit 3\n'
                       'name=$(basename "$2"); shift 2\n'
                       'echo "$name $*" >> "$FAKE_CALLS"\n'
                       'if test -f "$FAKE_FAIL" && grep -q "$name" "$FAKE_FAIL"; then\n'
                       '  echo "upload refused" >&2; exit 1\n'
                       'fi\n'
                       'echo "upload request req_1" >&2\n')
        cli.chmod(0o755)
        environment = {'PATH': f'{tools}:{os.environ["PATH"]}', 'FAKE_CALLS': str(self.calls),
                       'FAKE_FAIL': str(self.fail_on), 'FAKE_SETUP': str(self.setup_calls),
                       'FAKE_SESSION_CALLS': str(self.session_calls),
                       'FAKE_SESSIONS': str(self.sessions),
                       'FAKE_DEFINITIONS': str(self.definitions)}
        environment['TZ'] = 'Asia/Tokyo'
        patcher = mock.patch.dict(os.environ, environment)
        self.addCleanup(time.tzset)  # runs after the environment is restored
        patcher.start()
        self.addCleanup(patcher.stop)
        time.tzset()
        self.session = 'sim_circle_r020_2026-11-21_1132'
        self.run_dir = self.root / '20261121T000000.000000Z_circle_abc123'
        self.write_run(start=1_795_000_000 * 10**9)

    def define_properties(self, defined=True):
        """Write what the fake organization has defined."""
        items = [{'key': key, 'enumValues': values}
                 for key, (_, values) in bag.SESSION_PROPERTIES.items()] if defined else []
        self.definitions.write_text(json.dumps({'data': items}))

    def write_run(self, *, start, status='completed'):
        """Write the files experiment_bag leaves behind after a recording."""
        (self.run_dir / 'bag').mkdir(parents=True, exist_ok=True)
        (self.run_dir / 'experiment.yaml').write_text(yaml.safe_dump({
            'status': status, 'profile': 'sim', 'label': 'circle_r020',
            'started_at': '2026-11-21T02:32:36.629496+00:00', 'metadata': {
                'launch.file': 'trajectory.launch.py', 'session.plant': 'sim',
                'session.launch': 'trajectory', 'session.trajectory': 'circle',
                'session.state_source': 'estimator'}}, sort_keys=False))
        names = ['bag_0.mcap', 'bag_1.mcap']
        for name in names:
            (self.run_dir / 'bag' / name).write_bytes(b'mcap')
        (self.run_dir / 'bag' / 'metadata.yaml').write_text(yaml.safe_dump({
            'rosbag2_bagfile_information': {
                'starting_time': {'nanoseconds_since_epoch': start},
                'relative_file_paths': names}}))

    def upload(self, *extra):
        """Run the upload command quietly and return its status."""
        with contextlib.redirect_stdout(io.StringIO()), \
                contextlib.redirect_stderr(io.StringIO()):
            try:
                return bag.main(['upload', str(self.run_dir), *extra])
            except SystemExit as error:
                return error.code

    def called(self):
        """Return the calls the fake CLI received."""
        return self.calls.read_text().splitlines() if self.calls.exists() else []

    def test_segments_are_named_after_the_run_and_sent_once(self):
        """Every run's bag_0.mcap stays distinguishable and retries do not duplicate it."""
        self.assertEqual(self.upload('--device', 'lab-pc', '--project', 'prj_1'), 0)
        name = self.run_dir.name
        self.assertEqual(self.called(), [
            f'{name}_bag_0.mcap --device-name lab-pc --session-key {self.session} '
            '--project-id prj_1',
            f'{name}_bag_1.mcap --device-name lab-pc --session-key {self.session} '
            '--project-id prj_1'])
        self.assertEqual(self.upload(), 0)
        self.assertEqual(len(self.called()), 2)
        state = yaml.safe_load((self.run_dir / bag.UPLOAD_STATE).read_text())
        self.assertEqual(state['files']['bag_1.mcap']['status'], 'uploaded')
        self.assertIn('req_1', state['files']['bag_1.mcap']['output'])

    def test_failure_stops_and_retry_resumes(self):
        """A failed segment is reported; the retry sends only what is missing."""
        self.fail_on.write_text(f'{self.run_dir.name}_bag_1.mcap')
        self.assertEqual(self.upload(), 1)
        state = yaml.safe_load((self.run_dir / bag.UPLOAD_STATE).read_text())
        self.assertEqual(state['files']['bag_0.mcap']['status'], 'uploaded')
        self.assertEqual(state['files']['bag_1.mcap']['status'], 'failed')
        self.fail_on.unlink()
        self.assertEqual(self.upload(), 0)
        self.assertEqual([call.split()[0].rsplit('_', 2)[-2:] for call in self.called()],
                         [['bag', '0.mcap'], ['bag', '1.mcap'], ['bag', '1.mcap']])

    def test_refuses_runs_that_would_be_misplaced(self):
        """Unfinished runs and zero-based clocks are not sent by accident."""
        self.write_run(start=1_795_000_000 * 10**9, status='failed')
        self.assertNotEqual(self.upload(), 0)
        self.write_run(start=10_000_000)
        self.assertNotEqual(self.upload(), 0)
        self.assertEqual(self.called(), [])
        self.assertEqual(self.upload('--allow-epoch-time'), 0)
        self.assertEqual(len(self.called()), 2)

    def test_session_carries_the_conditions_and_retry_updates_it(self):
        """One session per run, created before the data and edited on a retry."""
        self.assertEqual(self.upload(), 0)
        conditions = ('--property plant:sim --property launch:trajectory '
                      '--property trajectory:circle --property state_source:estimator')
        self.assertEqual(self.session_calls.read_text().splitlines(),
                         [f'sessions add --device-id dev_1 --key {self.session} {conditions}'])
        self.assertEqual(self.upload(), 0)
        self.assertEqual(self.session_calls.read_text().splitlines()[1:],
                         [f'sessions edit {self.session} {conditions}'])
        state = yaml.safe_load((self.run_dir / bag.UPLOAD_STATE).read_text())
        self.assertEqual(state['session']['key'], self.session)
        self.assertEqual(state['session']['properties']['plant'], 'sim')

    def test_session_names_are_readable(self):
        """Plant, experiment and local time, without repeating what the label says."""
        started = {'started_at': '2026-11-21T02:32:36+00:00'}
        trajectory = {'plant': 'sim', 'launch': 'trajectory', 'trajectory': 'circle'}
        for label, properties, expected in [
                ('cloud_test', trajectory, 'sim_cloud_test_2026-11-21_1132'),
                ('trajectory_sim', trajectory, 'sim_circle_2026-11-21_1132'),
                ('manual_real', {'plant': 'real', 'launch': 'manual'},
                 'real_manual_2026-11-21_1132'),
                ('real_circle01', {'plant': 'real'}, 'real_circle01_2026-11-21_1132'),
                ('bench', {}, 'bench_2026-11-21_1132')]:
            with self.subTest(label=label):
                self.assertEqual(bag.session_name({**started, 'label': label}, properties),
                                 expected)

    def test_repeated_names_get_a_number(self):
        """Two runs of one experiment in the same minute keep separate sessions."""
        self.sessions.write_text(f'{self.session}\n{self.session}_2\n')
        self.assertEqual(self.upload(), 0)
        self.assertIn(f'--key {self.session}_3 ', self.session_calls.read_text())

    def test_session_named_after_the_directory_is_renamed(self):
        """Running upload again on an uploaded run only renames its session."""
        name = self.run_dir.name
        self.sessions.write_text(f'{name}\n')
        (self.run_dir / bag.UPLOAD_STATE).write_text(yaml.safe_dump({
            'session': {'key': name},
            'files': {segment: {'status': 'uploaded'}
                      for segment in ('bag_0.mcap', 'bag_1.mcap')}}))
        self.assertEqual(self.upload(), 0)
        self.assertTrue(self.session_calls.read_text().startswith(
            f'sessions edit {name} --key {self.session} '))
        self.assertEqual(self.called(), [])
        state = yaml.safe_load((self.run_dir / bag.UPLOAD_STATE).read_text())
        self.assertEqual(state['session']['key'], self.session)

    def test_uploaded_run_without_a_session_gets_no_empty_session(self):
        """A session created now would not contain the recording uploaded earlier."""
        (self.run_dir / bag.UPLOAD_STATE).write_text(yaml.safe_dump({
            'files': {segment: {'status': 'uploaded'}
                      for segment in ('bag_0.mcap', 'bag_1.mcap')}}))
        self.assertNotEqual(self.upload(), 0)
        self.assertFalse(self.session_calls.exists())
        self.assertEqual(self.called(), [])

    def test_undefined_properties_stop_before_any_data(self):
        """Without the definitions a recording would land outside its category."""
        self.define_properties(defined=False)
        self.assertNotEqual(self.upload(), 0)
        self.assertEqual(self.called(), [])
        self.assertFalse(self.session_calls.exists())

    def test_plant_falls_back_to_the_profile(self):
        """Runs recorded without the launch still get sim or real."""
        self.assertEqual(bag.session_properties({'profile': 'hardware'}), {'plant': 'real'})
        self.assertEqual(bag.session_properties({'profile': 'custom'}), {})
        with self.assertRaises(ValueError):
            bag.session_properties({'metadata': {'session.plant': 'bench'}})

    def test_setup_defines_properties_and_device_once(self):
        """foxglove-setup creates what is missing and leaves the rest alone."""
        self.define_properties(defined=False)
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(bag.main(['foxglove-setup', '--device', 'absent-device']), 0)
        calls = self.setup_calls.read_text().splitlines()
        self.assertEqual(len(calls), len(bag.SESSION_PROPERTIES) + 1)
        self.assertIn('--key plant --label Plant --resource-type session --value-type enum '
                      '--enum-value sim --enum-value real', calls[0])
        self.assertEqual(calls[-1], 'devices add --name absent-device')
        self.define_properties()
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(bag.main(['foxglove-setup']), 0)
        self.assertEqual(len(self.setup_calls.read_text().splitlines()), len(calls))

    def test_record_checks_the_cli_before_the_experiment(self):
        """A missing CLI stops the launch instead of losing the upload afterwards."""
        with mock.patch.object(bag.shutil, 'which', return_value=None), \
                contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            bag.main(['record', '--profile', 'sim', '--dry-run', '--upload'])


if __name__ == '__main__':
    unittest.main()
