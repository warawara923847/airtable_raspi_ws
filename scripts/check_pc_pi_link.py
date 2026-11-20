#!/usr/bin/env python3
"""
Measure whether pose-sized messages reach the Raspberry Pi within a control period.

The PC sends PoseWithCovarianceStamped (the ArUco observation type) at the camera rate;
the Pi echoes each one back. PC and Pi clocks differ by tens of ms (timesyncd), so the
one-way delay is bounded from the round trip on the PC clock alone (RTT/2 is the
symmetric estimate, RTT the worst case). The Pi reports its own arrival intervals.

  Pi: python3 check_pc_pi_link.py echo --duration 40
  PC: python3 scripts/check_pc_pi_link.py probe --rate 90 --duration 30 --qos best_effort

Use the same ROS_DOMAIN_ID on both, different from running robots. Needs only rclpy and
geometry_msgs (present on the Pi's ros-base).
"""

import argparse
import json
import statistics
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

PING = 'link_check/ping'
PONG = 'link_check/pong'


def qos(kind):
    reliability = (ReliabilityPolicy.BEST_EFFORT if kind == 'best_effort'
                   else ReliabilityPolicy.RELIABLE)
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=reliability)


def percentiles(values, points=(50, 95, 99)):
    if not values:
        return {}
    ordered = sorted(values)
    result = {f'p{p}': ordered[min(len(ordered) - 1, int(len(ordered) * p / 100))]
              for p in points}
    result['max'] = ordered[-1]
    result['mean'] = statistics.fmean(ordered)
    return result


def echo(args):
    """Republish every ping unchanged and report arrival intervals on exit."""
    node = rclpy.create_node('link_check_echo')
    publisher = node.create_publisher(PoseWithCovarianceStamped, PONG, qos(args.qos))
    arrivals = []

    def on_ping(msg):
        arrivals.append(time.monotonic())
        publisher.publish(msg)

    node.create_subscription(PoseWithCovarianceStamped, PING, on_ping, qos(args.qos))
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    gaps = [(b - a) * 1000 for a, b in zip(arrivals, arrivals[1:])]
    print(json.dumps({'role': 'echo', 'received': len(arrivals),
                      'interval_ms': percentiles(gaps)}, indent=2))
    node.destroy_node()


def probe(args):
    """Send pings at a fixed rate and measure round trips on this clock."""
    node = rclpy.create_node('link_check_probe')
    publisher = node.create_publisher(PoseWithCovarianceStamped, PING, qos(args.qos))
    sent = {}
    rtts = []

    def on_pong(msg):
        seq = int(msg.pose.covariance[0])
        if seq in sent:
            rtts.append((time.monotonic() - sent.pop(seq)) * 1000)

    node.create_subscription(PoseWithCovarianceStamped, PONG, on_pong, qos(args.qos))
    deadline = time.monotonic() + args.wait
    while publisher.get_subscription_count() == 0 or node.count_publishers(PONG) == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.monotonic() > deadline:
            raise SystemExit('No echo found; start the Pi side with the same ROS_DOMAIN_ID')
    period = 1.0 / args.rate
    count = int(args.duration * args.rate)
    start = time.monotonic()
    for seq in range(count):
        target = start + seq * period
        while time.monotonic() < target:
            rclpy.spin_once(node, timeout_sec=max(0.0, target - time.monotonic()))
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'table_1'
        msg.pose.covariance[0] = float(seq)
        sent[seq] = time.monotonic()
        publisher.publish(msg)
    drain = time.monotonic() + 1.0
    while time.monotonic() < drain:
        rclpy.spin_once(node, timeout_sec=0.02)
    rtt = percentiles(rtts)
    budget = args.budget_ms
    report = {
        'role': 'probe', 'rate_hz': args.rate, 'qos': args.qos, 'sent': count,
        'returned': len(rtts), 'lost_or_late': count - len(rtts),
        'rtt_ms': rtt,
        'one_way_estimate_ms': {key: value / 2 for key, value in rtt.items()},
        'budget_ms': budget,
        # RTT <= budget guarantees one way <= budget whatever the asymmetry.
        'rtt_within_budget': sum(value <= budget for value in rtts) / max(1, count),
        'half_rtt_within_budget': sum(value / 2 <= budget for value in rtts) / max(1, count),
    }
    print(json.dumps(report, indent=2))
    node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('role', choices=['echo', 'probe'])
    parser.add_argument('--duration', type=float, default=30.0, help='Run time [s]')
    parser.add_argument('--rate', type=float, default=90.0, help='probe: send rate [Hz]')
    parser.add_argument('--qos', choices=['best_effort', 'reliable'], default='best_effort')
    parser.add_argument('--budget-ms', type=float, default=20.0,
                        help='probe: control period the one-way delay must fit in [ms]')
    parser.add_argument('--wait', type=float, default=20.0, help='probe: discovery wait [s]')
    args = parser.parse_args()
    rclpy.init()
    try:
        echo(args) if args.role == 'echo' else probe(args)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
