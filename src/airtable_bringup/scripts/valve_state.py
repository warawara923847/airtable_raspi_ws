#!/usr/bin/python3
"""
Turn thruster pulse commands into a per-valve open/closed timeline for display.

ThrusterPulseCommand lists the valves to open and one duration, which a plot cannot
split per valve. This node keeps the commanded state of every valve and publishes
sensor_msgs/JointState on actuators/thrusters/valve_state: name = valve names (the
controller's thrusters.names order), position = 1.0 open / 0.0 closed.

- One message per change, stamped with the exact time of the edge (a pulse arrival or
  its scheduled end), plus a heartbeat so live plots keep extending.
- A new pulse restarts that valve's timer from now, as airtable_hardware and airtable_sim
  do (not extended), so the timeline matches what the plant is told to do.
- It is the commanded state seen on this computer, not a measurement: on the real robot
  the Pi opens the valve after the Wi-Fi delay and closes it within its 5 ms timer.
"""

import sys


class ValveTimeline:
    """Commanded valve state; times are seconds of the node clock (sim or system)."""

    def __init__(self, names):
        if not names or len(set(names)) != len(names) or not all(names):
            raise ValueError('valve names must be unique nonempty strings')
        self.names = list(names)
        self.until = [None] * len(names)  # scheduled close time while open
        self.last_time = None

    def states(self):
        """Return 1.0 for open and 0.0 for closed valves, in name order."""
        return [0.0 if until is None else 1.0 for until in self.until]

    def _rewind(self, now):
        """Forget everything if the clock went backwards (simulator restarted)."""
        if self.last_time is not None and now < self.last_time:
            self.until = [None] * len(self.names)
            self.last_time = None
            return True
        return False

    def close_due(self, now):
        """Close valves whose pulse ended by now; one (time, states) per distinct end."""
        self._rewind(now)
        events = []
        for end in sorted({u for u in self.until if u is not None and u <= now}):
            self.until = [None if u == end else u for u in self.until]
            events.append((end, self.states()))
            self.last_time = end
        return events

    def pulse(self, now, names, duration):
        """Apply one pulse at now. Return (events, unknown names)."""
        events = self.close_due(now)
        if duration <= 0:
            return events, []
        unknown = [name for name in names if name not in self.names]
        before = self.states()
        for name in names:
            if name in self.names:
                self.until[self.names.index(name)] = now + duration
        if self.states() != before:
            events.append((now, self.states()))
        self.last_time = now
        return events, unknown


def main(argv=None):
    """Run the ROS node."""
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import JointState

    from airtable_interfaces.msg import ThrusterPulseCommand

    class ValveStateNode(Node):
        def __init__(self):
            super().__init__('airtable_valve_state')
            names = self.declare_parameter(
                'thruster_names', ['thruster_1', 'thruster_2', 'thruster_3', 'thruster_4']).value
            check_rate = self.declare_parameter('check_rate_hz', 200.0).value
            heartbeat_rate = self.declare_parameter('heartbeat_rate_hz', 10.0).value
            if not 0 < heartbeat_rate <= check_rate:
                raise ValueError('require 0 < heartbeat_rate_hz <= check_rate_hz')
            self.timeline = ValveTimeline(names)
            self.heartbeat_period = 1.0 / heartbeat_rate
            self.last_published = None
            self.warned = set()
            self.publisher = self.create_publisher(
                JointState, 'actuators/thrusters/valve_state', 50)
            self.create_subscription(
                ThrusterPulseCommand, 'actuators/thrusters/pulse', self.on_pulse, 50)
            self.create_timer(1.0 / check_rate, self.on_timer)

        def now_s(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def publish(self, events):
            for time_s, states in events:
                message = JointState()
                message.header.stamp = Time(nanoseconds=round(time_s * 1e9)).to_msg()
                message.name = self.timeline.names
                message.position = states
                self.publisher.publish(message)
                self.last_published = time_s

        def on_pulse(self, message):
            if message.pulse_duration.sec < 0:
                return
            duration = message.pulse_duration.sec + message.pulse_duration.nanosec * 1e-9
            events, unknown = self.timeline.pulse(self.now_s(), list(message.names), duration)
            for name in set(unknown) - self.warned:
                self.get_logger().warn(f'pulse for unknown valve {name!r} ignored')
                self.warned.add(name)
            self.publish(events)

        def on_timer(self):
            now = self.now_s()
            if self.last_published is not None and now < self.last_published:
                self.last_published = None  # the simulator was restarted
            self.publish(self.timeline.close_due(now))
            if self.last_published is None or now - self.last_published >= \
                    self.heartbeat_period:
                self.publish([(now, self.timeline.states())])

    import signal

    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=argv)
    node = ValveStateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl+C reaches this process twice (terminal and launch); finish cleanly.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
