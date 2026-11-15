"""Validate robot configurations, path rules, precedence and model consistency."""

import math
import os
from pathlib import Path
import sys
import tempfile
import textwrap
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
SRC = PACKAGE.parent
sys.path.insert(0, str(PACKAGE))

from airtable_bringup import robot_config as rc  # noqa: E402
from airtable_bringup.launch_helpers import rviz_config  # noqa: E402


def share(package):
    """Source-tree stand-in for package share directories (same relative layout)."""
    path = SRC / package
    if not path.is_dir():
        raise rc.ConfigError(f'package://{package}: package not found')
    return path


CONFIGS = PACKAGE / 'config' / 'robots'


class RobotConfigTest(unittest.TestCase):
    """Load the shipped configurations and reject each kind of inconsistency."""

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def load(self, name):
        return rc.load_robot_config(CONFIGS / name, share)

    def write(self, text, name='robots.yaml'):
        path = self.dir / name
        path.write_text(textwrap.dedent(text))
        return path

    def dual_text(self, robot_2_extra='', robot_2_table='table_2'):
        return f"""
            version: 1
            simulation:
              model: package://airtable_sim/mujoco/airtable_dual.xml
            tables:
              table_1: {{world_pose: null}}
              table_2: {{world_pose: [1.5, 0.0, 0.0]}}
            robots:
              - namespace: robot_1
                table_frame: table_1
                simulation: {{element_prefix: robot_1/}}
              - namespace: robot_2
                table_frame: {robot_2_table}
                simulation: {{element_prefix: robot_2/}}
            {robot_2_extra}
        """

    def assert_error(self, text, fragment, check_model=False):
        with self.assertRaises(rc.ConfigError) as caught:
            config = rc.load_robot_config(self.write(text), share)
            if check_model:
                rc.check_simulation_model(config['model'], config['robots'])
        self.assertIn(fragment, str(caught.exception))

    def test_shipped_configurations_load_and_match_their_models(self):
        for name, robots, tables in [
            ('single_robot.yaml', ['robot_1'], ['table_1']),
            ('dual_robot.yaml', ['robot_1', 'robot_2'], ['table_1', 'table_2']),
            ('robot_2_only.yaml', ['robot_2'], ['table_2']),
        ]:
            config = self.load(name)
            self.assertEqual([r['namespace'] for r in config['robots']], robots)
            self.assertEqual([r['table_frame'] for r in config['robots']], tables)
            rc.check_simulation_model(config['model'], config['robots'])
        single = self.load('single_robot.yaml')['robots'][0]
        self.assertEqual(single['simulation']['element_prefix'], '')
        self.assertEqual(single['hardware']['imu_frame'], 'imu_link')
        dual = self.load('dual_robot.yaml')['robots']
        self.assertEqual([r['hardware']['imu_frame'] for r in dual],
                         ['robot_1/imu_link', 'robot_2/imu_link'])
        self.assertEqual([rc.effective_trajectory(r)['trajectory']['type'] for r in dual],
                         ['line', 'circle'])

    def test_configuration_path_forms(self):
        by_package = rc.load_robot_config(
            'package://airtable_bringup/config/robots/dual_robot.yaml', share)
        self.assertEqual(by_package['path'], (CONFIGS / 'dual_robot.yaml').resolve())
        with self.assertRaisesRegex(rc.ConfigError, 'relative to the current directory'):
            rc.load_robot_config('share/airtable_bringup/config/robots/dual_robot.yaml', share)
        with self.assertRaisesRegex(rc.ConfigError, 'package not found'):
            rc.load_robot_config('package://no_such_pkg/config/robots/dual_robot.yaml', share)

    def test_selection(self):
        config = self.load('dual_robot.yaml')
        self.assertEqual([r['namespace'] for r in rc.select_robots(config, 'robot_2')],
                         ['robot_2'])
        self.assertEqual(len(rc.select_robots(config, '')), 2)
        # Configuration order is kept regardless of the selection order.
        self.assertEqual([r['namespace'] for r in rc.select_robots(config, 'robot_2,robot_1')],
                         ['robot_1', 'robot_2'])
        with self.assertRaisesRegex(rc.ConfigError, 'unknown robots'):
            rc.select_robots(config, 'robot_3')
        with self.assertRaisesRegex(rc.ConfigError, 'duplicates'):
            rc.select_robots(config, 'robot_1,robot_1')

    def test_paths_are_relative_to_the_configuration_not_the_cwd(self):
        params = self.dir / 'params'
        params.mkdir()
        trajectory = (SRC / 'airtable_controller/config/trajectory.yaml').read_text()
        (params / 'traj.yaml').write_text(trajectory.replace('amplitude_m: 0.10',
                                                             'amplitude_m: 0.07'))
        path = self.write(self.dual_text().replace(
            'simulation: {element_prefix: robot_1/}',
            'trajectory_params: params/traj.yaml\n                simulation: '
            '{element_prefix: robot_1/}'))
        cwd = os.getcwd()
        try:
            os.chdir('/')
            config = rc.load_robot_config(path, share)
        finally:
            os.chdir(cwd)
        robot_1, robot_2 = config['robots']
        self.assertEqual(robot_1['trajectory_params'], (params / 'traj.yaml').resolve())
        self.assertEqual(rc.effective_trajectory(robot_1)['trajectory']['amplitude_m'], 0.07)
        self.assertEqual(rc.effective_trajectory(robot_2)['trajectory']['amplitude_m'], 0.10)
        self.assertEqual(config['tables']['table_2']['world_pose'], [1.5, 0.0, 0.0])

    def test_precedence_file_then_robot_then_launch(self):
        path = self.write(self.dual_text().replace(
            'simulation: {element_prefix: robot_2/}',
            'trajectory_overrides: {trajectory: {type: circle, radius_m: 0.2}}\n'
            '                simulation: {element_prefix: robot_2/}'))
        robot = rc.load_robot_config(path, share)['robots'][1]
        params = rc.effective_trajectory(robot)
        self.assertEqual(params['trajectory']['radius_m'], 0.2)
        self.assertEqual(params['trajectory']['period_s'], 20.0)  # from the file
        self.assertEqual(rc.initial_pose(robot, params), [0.7, 1.0, 0.0])
        launch = rc.launch_trajectory_overrides({'radius': '0.15', 'period': ''})
        params = rc.effective_trajectory(robot, launch)
        self.assertEqual(params['trajectory']['radius_m'], 0.15)
        self.assertEqual(rc.initial_pose(robot, params), [0.65, 1.0, 0.0])
        self.assertEqual(rc.initial_pose(robot, params, {'initial_x': '0.4',
                                                         'initial_y': '0.9'}),
                         [0.4, 0.9, 0.0])
        robot['simulation']['initial_pose'] = [0.3, 0.5, 0.1]
        self.assertEqual(rc.initial_pose(robot, params), [0.3, 0.5, 0.1])
        line = rc.load_robot_config(path, share)['robots'][0]
        self.assertIsNone(rc.initial_pose(line, rc.effective_trajectory(line)))

    def test_rejected_configurations(self):
        base = self.dual_text()
        cases = [
            (base.replace('namespace: robot_2', 'namespace: robot_1'),
             'duplicate robot namespace'),
            (base.replace('version: 1', 'version: 2'), 'version must be 1'),
            (base.replace('airtable_dual.xml', 'missing.xml'), 'file not found'),
            (base.replace('package://airtable_sim', 'package://no_such_pkg'),
             'package not found'),
            (self.dual_text(robot_2_table='table_3'), 'not listed in tables'),
            (self.dual_text(robot_2_table='table_1'), 'one robot per table'),
            (base.replace('table_frame: table_1', 'table_frame: table_1\n'
                          '                tabel: x'), 'unknown keys'),
            (base.replace('robot_2/}', 'robot_2/, initial_pose: [.nan, 1.0, 0.0]}'),
             'must be finite'),
            (base.replace('robot_2/}', 'robot_2/, initial_pose: [0.5, 1.0]}'),
             'list of 3 numbers'),
            (base.replace('robot_2/}', 'robot_1/}'), 'duplicates robot_1'),
            (base.replace('namespace: robot_2', 'namespace: robot-2'), 'ROS name segment'),
            (base.replace('robot_2/}', 'robot_2/}\n                trajectory_overrides: '
                          '{trajectory: {typo_m: 1.0}}'), 'not a parameter'),
            (base.replace('robot_2/}', 'robot_2/}\n                trajectory_overrides: '
                          '{reference_frame: table_1}'), 'derived by the launch'),
            (base.replace('robot_2/}', 'robot_2/}\n                trajectory_overrides: '
                          '{trajectory: {period_s: fast}}'), 'type str differs'),
            (base.replace('robot_2/}', 'robot_2/}\n                trajectory_overrides: '
                          '{trajectory: {type: square}}'), 'line or circle'),
            (base.replace('version: 1', 'version: 1\n            unused: 1'), 'unknown keys'),
            (base[:base.index('robots:')] + 'robots: []\n', 'non-empty list'),
        ]
        for text, fragment in cases:
            with self.subTest(fragment=fragment):
                self.assert_error(text, fragment)

    def test_initial_pose_must_be_inside_the_table_bounds(self):
        config = rc.load_robot_config(self.write(self.dual_text()), share)
        robot = config['robots'][1]
        params = rc.effective_trajectory(robot)
        with self.assertRaisesRegex(rc.ConfigError, 'outside position_bounds_m'):
            rc.initial_pose(robot, params, {'initial_x': '1.2', 'initial_y': '1.0'})
        with self.assertRaisesRegex(rc.ConfigError, 'both initial_x'):
            rc.initial_pose(robot, params, {'initial_x': '0.5'})
        with self.assertRaisesRegex(rc.ConfigError, 'must be finite'):
            rc.initial_pose(robot, params, {'initial_x': 'nan', 'initial_y': '1.0'})

    def test_model_consistency(self):
        # robot_2 really sits on table_2 in the dual model.
        text = self.dual_text().replace('tables:', 'tables:\n              table_3: {}')
        text = text.replace('table_frame: table_2', 'table_frame: table_3')
        self.assert_error(text, "configuration says table_frame 'table_3'", check_model=True)
        self.assert_error(self.dual_text().replace('robot_2/}', "''}"),
                          'missing actuators/sensors', check_model=True)
        self.assert_error(self.dual_text().replace('airtable_dual.xml', 'airtable_robot_2.xml'),
                          "vehicle body 'robot_1' not found", check_model=True)

    def test_tracking_marker_ids_and_tables(self):
        tracking = SRC / 'airtable_tracking/config/tracking.yaml'
        moved = self.dir / 'tracking_2.yaml'
        moved.write_text(tracking.read_text().replace('table_frame: table_1',
                                                      'table_frame: table_2'))
        entry = (f'tracking: {{params: {moved}, image_topic: /cam/cam/infra1/image_rect_raw}}')
        text = self.dual_text().replace('robot_2/}', f'robot_2/}}\n                {entry}')
        text = text.replace('robot_1/}', 'robot_1/}\n                tracking: {params: '
                            'package://airtable_tracking/config/tracking.yaml, '
                            'image_topic: /cam/cam/infra1/image_rect_raw}')
        self.assert_error(text, 'moving marker ID 50')
        self.assert_error(text.replace(str(moved), str(tracking)), 'differs from')
        moved.write_text(moved.read_text().replace('moving_marker_id: 50',
                                                   'moving_marker_id: 51'))
        # Same image: fixed IDs 10-13 would be seen twice.
        self.assert_error(text, 'share /cam/cam/infra1/image_rect_raw and marker IDs')
        separate = text.replace('image_topic: /cam/cam/infra1/image_rect_raw}}',
                                'image_topic: /cam2/cam2/infra1/image_rect_raw}}')
        config = rc.load_robot_config(self.write(separate.replace(
            f'{moved}, image_topic: /cam/', f'{moved}, image_topic: /cam2/')), share)
        self.assertEqual(config['robots'][1]['tracking']['moving_marker_id'], 51)

    def test_simulation_cameras_per_table(self):
        single = self.load('single_robot.yaml')
        sim = single['robots'][0]['tracking']['simulation']
        # The original model keeps its topics and frame.
        self.assertEqual(sim['image_topic'], '/sim/camera/infra1/image_rect_raw')
        self.assertEqual(sim['camera_frame'], 'sim_camera_infra1_optical_frame')
        dual = self.load('dual_robot.yaml')
        sims = [r['tracking']['simulation'] for r in dual['robots']]
        self.assertEqual([s['image_topic'] for s in sims],
                         ['/sim/table_1/camera/infra1/image_rect_raw',
                          '/sim/table_2/camera/infra1/image_rect_raw'])
        self.assertEqual([s['camera_frame'] for s in sims],
                         ['sim_table_1_camera_infra1_optical_frame',
                          'sim_table_2_camera_infra1_optical_frame'])
        self.assertEqual([s['marker_ids'] for s in sims], [[10, 11, 12, 13], [20, 21, 22, 23]])
        self.assertEqual([s['moving_marker_id'] for s in sims], [50, 51])
        for name in ('single_robot.yaml', 'dual_robot.yaml', 'robot_2_only.yaml'):
            config = self.load(name)
            rc.check_simulation_cameras(config['model'], config['robots'])
        # Real tracking of robot_2 stays unconfigured; only simulation is set.
        self.assertIsNone(dual['robots'][1]['tracking']['params'])

    def test_simulation_camera_mismatches(self):
        sim_2 = SRC / 'airtable_tracking/config/simulation_tracking_robot_2.yaml'
        entry = ('tracking: {{simulation_params: {params}, '
                 'simulation_camera: {camera}}}')

        def text(camera='table_2/tracking_camera', params=sim_2, robot_1=''):
            out = self.dual_text().replace('robot_2/}', 'robot_2/}\n                ' +
                                           entry.format(params=params, camera=camera))
            if robot_1:
                out = out.replace('robot_1/}', 'robot_1/}\n                ' + robot_1)
            return out

        def model_error(config_text, fragment):
            config = rc.load_robot_config(self.write(config_text), share)
            with self.assertRaises(rc.ConfigError) as caught:
                rc.check_simulation_cameras(config['model'], [
                    r for r in config['robots'] if r['tracking']['simulation']])
            self.assertIn(fragment, str(caught.exception))

        model_error(text(camera='table_1/tracking_camera'), "is on 'table_1'")
        model_error(text(camera='tracking_camera'), "camera 'tracking_camera'")
        shifted = self.dir / 'shifted.yaml'
        shifted.write_text(sim_2.read_text().replace('0.88, 1.88, 0.002', '0.86, 1.88, 0.002'))
        model_error(text(params=shifted), 'marker_23 on table_2 is at')
        other = self.dir / 'other_ids.yaml'
        other.write_text(sim_2.read_text().replace('moving_marker_id: 51',
                                                   'moving_marker_id: 52'))
        model_error(text(params=other), 'robot_2 carries no card marker_52')
        table_1_ids = self.dir / 'table_1_ids.yaml'
        table_1_ids.write_text(sim_2.read_text().replace('[20, 21, 22, 23]',
                                                         '[10, 11, 12, 13]'))
        model_error(text(params=table_1_ids), 'table_2 needs exactly one fixed card marker_10')
        robot_1 = ('tracking: {simulation_params: package://airtable_tracking/config/'
                   'simulation_tracking.yaml, simulation_camera: table_2/tracking_camera}')
        self.assert_error(text(robot_1=robot_1), "simulation_camera 'table_2/tracking_camera'")
        self.assert_error(text().replace(f'simulation_params: {sim_2}, ', ''),
                          'simulation_camera needs simulation_params')
        self.assert_error(text(params=SRC / 'airtable_tracking/config/'
                               'simulation_tracking.yaml'), 'differs from')

    def test_generated_rviz_separates_robots(self):
        config = self.load('dual_robot.yaml')
        data = rviz_config(config['robots'], simulation=True)
        manager = data['Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'world')
        groups = [d for d in manager['Displays'] if d['Class'] == 'rviz_common/Group']
        self.assertEqual([g['Name'] for g in groups],
                         ['robot_1 on table_1', 'robot_2 on table_2'])
        topics = [[d['Topic']['Value'] for d in g['Displays'] if 'Topic' in d] for g in groups]
        self.assertIn('/robot_2/sim/ground_truth', topics[1])
        self.assertTrue(all(t.startswith('/robot_1/') for t in topics[0]))
        colors = [[d.get('Color') for d in g['Displays'] if 'Color' in d] for g in groups]
        self.assertFalse(set(colors[0]) & set(colors[1]))
        hardware = rviz_config(config['robots'], simulation=False)
        names = [d['Topic']['Value'] for g in hardware['Visualization Manager']['Displays']
                 if g['Class'] == 'rviz_common/Group' for d in g['Displays'] if 'Topic' in d]
        self.assertIn('/robot_2/state/odometry', names)
        self.assertFalse([n for n in names if '/sim/' in n])

    def test_yaml_argument_round_trip(self):
        value = {'trajectory': {'type': 'circle', 'circle_center_m': [0.5, 1.0]}}
        text = rc.yaml_argument(value)
        self.assertEqual(rc.parse_yaml_argument(text, 'x'),
                         {'trajectory.type': 'circle', 'trajectory.circle_center_m': [0.5, 1.0]})
        self.assertTrue(math.isfinite(rc.parse_yaml_argument('{a: 1.0}', 'x')['a']))


if __name__ == '__main__':
    unittest.main()
