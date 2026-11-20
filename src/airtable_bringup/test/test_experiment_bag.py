"""Validate recording profiles without starting ROS nodes or touching hardware."""

import argparse
import contextlib
import importlib.util
import io
from pathlib import Path
import re
import tempfile
import unittest

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


if __name__ == '__main__':
    unittest.main()
