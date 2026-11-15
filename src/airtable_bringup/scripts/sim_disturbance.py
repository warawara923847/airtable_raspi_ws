#!/usr/bin/python3
"""Publish a timed planar disturbance to the simulator, independently of control."""

import math
import random


def validate_profile(profile, start, duration, period, rate, wrench):
    """Reject invalid experiment conditions before publishing a force."""
    if profile not in ('pulse', 'step', 'sine', 'gaussian'):
        raise ValueError('profile must be pulse, step, sine or gaussian')
    if not all(math.isfinite(value) for value in [start, duration, period, rate, *wrench]):
        raise ValueError('Disturbance parameters must be finite')
    if start < 0 or duration < 0 or period <= 0 or not 0 < rate <= 100:
        raise ValueError('Require start/duration >= 0, period > 0 and rate in (0, 100]')
    if profile == 'pulse' and duration <= 0:
        raise ValueError('A pulse needs a positive duration')


def profile_gain(profile, elapsed, start, duration, period):
    """Evaluate a pulse, persistent step or optionally finite sinusoid in ROS time."""
    t = elapsed - start
    if t < 0:
        return 0.0
    if profile != 'step' and duration > 0 and t >= duration:
        return 0.0
    if profile == 'sine':
        return math.sin(2 * math.pi * t / period)
    return 1.0


class GaussianWrench:
    """Generate independent normal loads held over fixed simulation-time intervals."""

    def __init__(self, mean, stddev, sample_period, seed):
        if len(mean) != 3 or len(stddev) != 3 or not all(
                math.isfinite(value) for value in [*mean, *stddev, sample_period]):
            raise ValueError('Require three finite means/stddevs and a finite sample period')
        if any(value < 0 for value in stddev) or sample_period <= 0:
            raise ValueError('Require stddev >= 0 and noise_sample_period_s > 0')
        if type(seed) is not int or not 0 <= seed <= 2**63 - 1:
            raise ValueError('noise_seed must be an integer in [0, 2^63-1]')
        self.mean = tuple(mean)
        self.stddev = tuple(stddev)
        self.sample_period = sample_period
        self.seed = seed
        self.last_index = None
        self.last_wrench = None

    def sample(self, elapsed):
        """Draw by interval index so skipped callbacks and late joining preserve samples."""
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError('Noise elapsed time must be finite and nonnegative')
        index = math.floor((elapsed + 1e-12) / self.sample_period)
        if index != self.last_index:
            # Random's string seed is stable; do not use Python's randomized hash().
            rng = random.Random(f'{self.seed}:{index}')
            self.last_wrench = tuple(
                mean + sigma * rng.gauss(0.0, 1.0)
                for mean, sigma in zip(self.mean, self.stddev))
            self.last_index = index
        return self.last_wrench


def main(args=None):
    """Use the trajectory epoch for reproducible timing, or start from node time."""
    from geometry_msgs.msg import WrenchStamped
    from nav_msgs.msg import Path
    from rcl_interfaces.msg import ParameterDescriptor
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Bool

    class SimDisturbance(Node):
        """Experiment input only; the simulator validates and applies the wrench."""

        def __init__(self):
            super().__init__('airtable_sim_disturbance')

            def parameter(name, default):
                """Declare an immutable experiment parameter."""
                return self.declare_parameter(name, default,
                                              ParameterDescriptor(read_only=True)).value

            self.profile = parameter('profile', 'pulse')
            self.start = parameter('start_delay_s', 8.0)
            self.duration = parameter('duration_s', 2.0)
            self.period = parameter('period_s', 5.0)
            self.rate = parameter('publish_rate_hz', 50.0)
            self.wrench = [parameter(name, fallback) for name, fallback in [
                ('force_x_n', 0.02), ('force_y_n', 0.0), ('torque_z_nm', 0.0005)]]
            self.frame = parameter('frame_id', 'table_1')
            self.wait = parameter('wait_for_trajectory', True)
            stddev = [parameter(name, fallback) for name, fallback in [
                ('force_x_stddev_n', 0.02), ('force_y_stddev_n', 0.02),
                ('torque_z_stddev_nm', 0.0005)]]
            sample_period = parameter('noise_sample_period_s', 0.1)
            seed = parameter('noise_seed', 42)
            validate_profile(self.profile, self.start, self.duration, self.period,
                             self.rate, self.wrench)
            self.noise = None
            if self.profile == 'gaussian':
                self.noise = GaussianWrench(self.wrench, stddev, sample_period, seed)
                if sample_period * self.rate < 1 - 1e-12:
                    raise ValueError('Noise sampling must not be faster than publishing')
            if not self.frame:
                raise ValueError('frame_id must not be empty')
            self.active = False
            self.epoch = None
            self.last_time = None
            self.last_sent = -1
            self.publisher = self.create_publisher(WrenchStamped, 'sim/disturbance/wrench', 1)
            self.subscriptions_kept = []
            if self.wait:
                qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                self.subscriptions_kept = [
                    self.create_subscription(Path, 'trajectory/path', self.on_path, qos),
                    self.create_subscription(Bool, 'trajectory/active', self.on_active, qos),
                ]
            self.timer = self.create_timer(1 / self.rate, self.update)
            self.get_logger().info(
                f'{self.profile}: {self.wrench} [N,N,Nm] in {self.frame}; '
                f'start={self.start}s, duration={self.duration}s, '
                f'wait_for_trajectory={self.wait}')
            if self.noise is not None:
                self.get_logger().info(
                    f'Gaussian mean={self.wrench}, stddev={stddev} [N,N,Nm], '
                    f'sample_period={sample_period}s, seed={seed}; independent axes')

        def on_path(self, msg):
            self.epoch = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        def on_active(self, msg):
            self.active = msg.data

        def publish(self, wrench, now):
            # ROS time advances at /clock's rate, even if the timer is faster.
            if now.nanoseconds == self.last_sent:
                return
            self.last_sent = now.nanoseconds
            msg = WrenchStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self.frame
            msg.wrench.force.x = wrench[0]
            msg.wrench.force.y = wrench[1]
            msg.wrench.torque.z = wrench[2]
            self.publisher.publish(msg)

        def update(self):
            now = self.get_clock().now()
            time = now.nanoseconds * 1e-9
            if self.last_time is not None and time < self.last_time:
                self.epoch = None
                self.active = False
                self.last_sent = -1
            self.last_time = time
            if not self.wait and self.epoch is None:
                self.epoch = time
            if self.epoch is None or (self.wait and not self.active):
                self.publish((0.0, 0.0, 0.0), now)
                return
            gain = profile_gain(self.profile, time - self.epoch, self.start,
                                self.duration, self.period)
            if self.noise is not None and gain:
                wrench = self.noise.sample(max(0.0, time - self.epoch - self.start))
            else:
                wrench = [value * gain for value in self.wrench]
            self.publish(wrench, now)

    rclpy.init(args=args)
    node = None
    try:
        node = SimDisturbance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # The simulator's watchdog removes the last nonzero command on shutdown.


if __name__ == '__main__':
    main()
