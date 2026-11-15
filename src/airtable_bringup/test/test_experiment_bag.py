"""Validate recording profiles without starting ROS nodes or touching hardware."""

import argparse
import contextlib
import importlib.util
import io
from pathlib import Path
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
                  'namespace': None, 'include_group': [], 'output_root': None, 'storage': None}
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
            namespace='/robot_2/', include_group=['sim_images', 'control']))
        self.assertIn('/robot_2/actuators/thrusters/pulse', config['topics'])
        self.assertIn('/sim/camera/infra1/image_rect_raw', config['topics'])
        self.assertFalse(any('/robot_1/' in topic for topic in config['topics']))
        self.assertEqual(len(config['topics']), len(set(config['topics'])))
        self.assertEqual(config['qos_overrides']['/tf_static']['durability'], 'transient_local')

    def test_root_namespace(self):
        """An unnamespaced standalone hardware node can use the same groups."""
        config = bag.resolve_profile(self.arguments(namespace='/', profile='real'))
        self.assertIn('/imu/data_raw', config['topics'])

    def test_reject_unknown_groups_and_invalid_names(self):
        """Catch misspellings before creating any experiment directory."""
        for values in ({'include_group': ['images_typo']}, {'namespace': '../robot'}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                bag.resolve_profile(self.arguments(**values))

    def test_reject_yaml_clock_and_option_typos(self):
        """Do not silently record a bad clock or ignore a misspelled split limit."""
        original = yaml.safe_load((PACKAGE / 'config/recording.yaml').read_text())
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / 'config.yaml'
            for mutation in ('clock', 'typo', 'boolean', 'empty'):
                config = yaml.safe_load(yaml.safe_dump(original))
                if mutation == 'clock':
                    config['topic_groups']['simulation'].remove('/clock')
                elif mutation == 'typo':
                    config['defaults']['max_bag_duraton'] = 300
                elif mutation == 'boolean':
                    config['profiles']['sim']['use_sim_time'] = 'false'
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
