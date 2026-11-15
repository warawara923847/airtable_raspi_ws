#!/usr/bin/python3
"""
Check a robot configuration, or export its resolved parameters for recording.

- ros2 run airtable_bringup robot_config check <config.yaml> [--robots robot_2] [--simulation]
- ros2 run airtable_bringup robot_config export <config.yaml> --output <dir>
- ros2 run airtable_bringup robot_config rviz <config.yaml> --output <file.rviz>
"""

import argparse
from pathlib import Path
import shutil
import sys

from airtable_bringup.launch_helpers import model_table_positions, write_rviz_config
from airtable_bringup.robot_config import (
    check_simulation_model, ConfigError, effective_trajectory, load_robot_config, merge,
    select_robots, summary)
import yaml


def resolved(config, robots):
    """Effective per-robot values, as the launch files pass them to nodes."""
    result = {'robot_config': str(config['path']),
              'simulation_model': None if config['model'] is None else str(config['model']),
              'tables': config['tables'], 'robots': {}}
    for robot in robots:
        trajectory = effective_trajectory(robot)
        # Values derived by the launch. state_topic/start_enabled are the hardware-path
        # defaults; simulation uses simulation.state_topic and enabled:=true.
        trajectory.update({'reference_frame': robot['table_frame'],
                           'body_frame': robot['body_frame'],
                           'state_topic': robot['state_topic'], 'start_enabled': False})
        result['robots'][robot['namespace']] = {
            'table_frame': robot['table_frame'],
            'state_topic': {'hardware': robot['state_topic'],
                            'simulation': robot['simulation']['state_topic']},
            'trajectory_params_file': str(robot['trajectory_params']),
            'thruster_params_file': str(robot['thruster_params']),
            'trajectory_controller': trajectory,
            'thrusters': merge(robot['thruster_base'], robot['thruster_overrides']),
            'simulation': robot['simulation'],
            'tracking': {key: (str(value) if isinstance(value, Path) else value)
                         for key, value in robot['tracking'].items()},
            'hardware': {key: (str(value) if isinstance(value, Path) else value)
                         for key, value in robot['hardware'].items()},
        }
    return result


def main(argv=None):
    """Run one subcommand; exit 2 with a specific message on configuration errors."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('check', 'export', 'rviz'):
        command = sub.add_parser(name)
        command.add_argument('config')
        command.add_argument('--robots', default='', help='Comma-separated robots; default all')
        if name == 'check':
            command.add_argument('--simulation', action='store_true',
                                 help='Also check simulation.model names and table parents')
        if name == 'export':
            command.add_argument('--output', required=True, help='New or empty directory')
        if name == 'rviz':
            command.add_argument('--output', required=True)
            command.add_argument('--hardware', action='store_true',
                                 help='Use state_topic instead of simulation truth')
            command.add_argument('--fixed-frame', default='world')
    args = parser.parse_args(argv)
    try:
        config = load_robot_config(args.config)
        robots = select_robots(config, args.robots)
        if args.command == 'check':
            if args.simulation:
                if config['model'] is None:
                    raise ConfigError('simulation.model is not set')
                check_simulation_model(config['model'], robots)
            print(summary(config, robots))
            print(yaml.safe_dump(resolved(config, robots), sort_keys=False))
            print('OK')
        elif args.command == 'export':
            output = Path(args.output)
            if output.exists() and any(output.iterdir()):
                raise ConfigError(f'{output} is not empty')
            output.mkdir(parents=True, exist_ok=True)
            shutil.copy2(config['path'], output / config['path'].name)
            data = resolved(config, robots)
            (output / 'resolved_robot_config.yaml').write_text(
                yaml.safe_dump(data, sort_keys=False))
            for name, values in data['robots'].items():
                for key in ('trajectory_controller', 'thrusters'):
                    (output / f'{name}_{key}.yaml').write_text(yaml.safe_dump(
                        {'/**': {'ros__parameters': values[key]}}, sort_keys=False))
            print('\n'.join(str(path) for path in sorted(output.iterdir())))
        else:
            table_xy = model_table_positions(config['model']) if (
                config['model'] is not None and not args.hardware) else {
                name: spec['world_pose'][:2] for name, spec in config['tables'].items()
                if spec['world_pose'] is not None}
            print(write_rviz_config(robots, simulation=not args.hardware,
                                    fixed_frame=args.fixed_frame, table_xy=table_xy,
                                    path=args.output))
    except ConfigError as error:
        print(f'robot configuration error: {error}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
