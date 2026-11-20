#!/usr/bin/env python3
"""Fire the thruster valves one at a time through a running airtable_hardware node.

The ROS counterpart of the C sample's main.c (VALVE1..4 opened in turn): it publishes
ThrusterPulseCommand on <namespace>/actuators/thrusters/pulse, so it exercises the same
path as the controller (DDS -> hardware worker -> GPIO, deadline kept on the Pi).
Before each thruster it prints the BCM line (hardware_pi.yaml) and where the thruster sits
and which way it should push the robot (controller thrusters.yaml), so wiring, numbering
and direction can be checked by ear, by hand or with air. See docs/valve-check.md.

Examples (Pi: hardware.launch.py hardware_params:=.../hardware_pi.yaml must be running):
  scripts/check_valves.py                              # 1..4, one 500 ms pulse each
  scripts/check_valves.py --thrusters 2 --ask          # wait for Enter before firing
  scripts/check_valves.py --pulse-ms 10 --count 50 --period-ms 50   # 10 ms at 20 Hz
"""

import argparse
import os
from pathlib import Path
import sys
import time

from airtable_interfaces.msg import ThrusterPulseCommand
from builtin_interfaces.msg import Duration
import rclpy
import yaml

ROOT = Path(__file__).resolve().parents[1]
GEOMETRY = ROOT / 'src/airtable_controller/config/thrusters.yaml'
HARDWARE_PI = ROOT / 'src/airtable_bringup/config/hardware_pi.yaml'
# hardware.max_pulse_ms in hardware_pi.yaml; a longer pulse is shortened by the node.
DEFAULT_MAX_PULSE_MS = 1000


def side(value, positive, negative):
    if abs(value) < 1e-9:
        return ''
    return positive if value > 0 else negative


def describe(name, geometry):
    """'front-left, pushes the robot back-left (exhaust front-right)' in Japanese."""
    entry = geometry.get(name)
    if entry is None:
        return '幾何情報なし'
    px, py = entry['position']
    dx, dy = entry['direction']
    place = side(px, '前', '後') + side(py, '左', '右')
    push = side(dx, '前', '後') + side(dy, '左', '右')
    exhaust = side(-dx, '前', '後') + side(-dy, '左', '右')
    torque = px * dy - py * dx
    turn = side(torque, '反時計回り(CCW)', '時計回り(CW)') or '回転なし'
    return f'取付 {place}、機体を{push}へ押す（排気は{exhaust}）、単独では{turn}'


def load_parameters(path):
    data = yaml.safe_load(path.read_text())
    return data['/**']['ros__parameters']


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument('--thrusters', type=int, nargs='+', default=[1, 2, 3, 4],
                        help='thruster numbers to fire, in this order (default: 1 2 3 4)')
    parser.add_argument('--pulse-ms', type=float, default=500.0,
                        help='length of one pulse (default: 500)')
    parser.add_argument('--count', type=int, default=1, help='pulses per thruster')
    parser.add_argument('--period-ms', type=float, default=1000.0,
                        help='pulse start to next pulse start (default: 1000)')
    parser.add_argument('--gap-s', type=float, default=1.5,
                        help='pause between thrusters (default: 1.5)')
    parser.add_argument('--ask', action='store_true',
                        help='wait for Enter before each thruster')
    parser.add_argument('--discovery-timeout-s', type=float, default=10.0)
    args = parser.parse_args()

    if args.pulse_ms <= 0 or args.count < 1:
        parser.error('--pulse-ms must be positive and --count at least 1')
    if args.count > 1 and args.period_ms < args.pulse_ms:
        parser.error('--period-ms must not be shorter than --pulse-ms')
    names = [f'thruster_{number}' for number in args.thrusters]

    geometry = load_parameters(GEOMETRY)['thrusters']
    lines, max_pulse_ms = {}, DEFAULT_MAX_PULSE_MS
    if HARDWARE_PI.exists():
        thrusters = load_parameters(HARDWARE_PI)['thrusters']
        lines = {f'thruster_{i + 1}': line for i, line in enumerate(thrusters['gpio_lines'])}
        max_pulse_ms = thrusters.get('max_pulse_ms', max_pulse_ms)
    if args.pulse_ms > max_pulse_ms:
        print(f'[注意] {args.pulse_ms:g} ms は hardware の max_pulse_ms={max_pulse_ms} で'
              ' 切り詰められる', file=sys.stderr)

    rclpy.init()
    node = rclpy.create_node('airtable_check_valves')
    topic = f'/{args.namespace.strip("/")}/actuators/thrusters/pulse'
    publisher = node.create_publisher(ThrusterPulseCommand, topic, 10)
    try:
        deadline = time.monotonic() + args.discovery_timeout_s
        while publisher.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                peers = os.environ.get('ROS_STATIC_PEERS')
                print(f'{topic} の購読者が見つからない。Piで hardware.launch.py が動いているか、'
                      'ROS_DOMAIN_ID が合っているかを確認する。マルチキャストの通らない'
                      'ネットワークでは export ROS_STATIC_PEERS=<PiのIP> が必要'
                      f'（現在: {peers or "未設定"}）', file=sys.stderr)
                return 1
            rclpy.spin_once(node, timeout_sec=0.1)
        # Discovery of the reverse direction can lag behind; do not lose the first pulse.
        time.sleep(0.5)

        seconds, rest = divmod(args.pulse_ms / 1000.0, 1.0)
        duration = Duration(sec=int(seconds), nanosec=int(round(rest * 1e9)))
        print(f'{topic} へ {args.pulse_ms:g} ms × {args.count} 発'
              f'（周期 {args.period_ms:g} ms）')
        for index, name in enumerate(names):
            line = lines.get(name)
            print(f'\n== {name}  BCM {line if line is not None else "?"}  '
                  f'{describe(name, geometry)}')
            if args.ask:
                input('   Enterで噴射 > ')
            elif index > 0:
                time.sleep(args.gap_s)
            start = time.monotonic()
            for shot in range(args.count):
                target = start + shot * args.period_ms / 1000.0
                time.sleep(max(0.0, target - time.monotonic()))
                message = ThrusterPulseCommand()
                message.stamp = node.get_clock().now().to_msg()
                message.names = [name]
                message.pulse_duration = duration
                publisher.publish(message)
            # Let the last pulse finish before moving on.
            time.sleep(args.pulse_ms / 1000.0)
            print(f'   {args.count} 発送信')
        return 0
    except KeyboardInterrupt:
        # The Pi closes the valve at the pulse deadline; nothing to undo here.
        return 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
