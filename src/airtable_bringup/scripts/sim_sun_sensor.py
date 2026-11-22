#!/usr/bin/python3
"""
Simulated nanoSSOC-D60 (docs/adr/0030).

Reads the simulator's true pose (sim/ground_truth) and publishes the same
airtable_interfaces/SunSensorState on measurements/sun_sensor as the real sensor, so the
display, the recording and later control use one interface. The light is parallel: only
its direction matters, set per table as azimuth (from table +X, counterclockwise) and
elevation of the direction from the table toward the light.

alpha = atan2(x, z) and beta = atan2(y, z) of the sun in the sensor frame. With the sun
behind the sensor or too dim the output is what the real sensor gave without light:
error 10 and alpha = beta = 60 deg.
"""

import collections
import math
import random
import sys

ERROR_NONE = 0
ERROR_NOT_ENOUGH_RADIATION = 10
ERROR_OUT_OF_FIELD_OF_VIEW = 13
ERROR_TRANSMISSION_FAILED = 20
NO_LIGHT_ANGLE_DEG = 60.0

Observation = collections.namedtuple(
    'Observation', 'valid error_code alpha_deg beta_deg intensity_percent sun_vector')


def rotation_matrix(roll, pitch, yaw):
    """Return Rz(yaw) Ry(pitch) Rx(roll), the convention of static_transform_publisher."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


class SunSensorModel:
    """Sensor geometry, field of view and errors for one robot."""

    def __init__(self, mount, azimuth_deg, elevation_deg, *, fov_deg=60.0, noise_stddev_deg=0.1,
                 bias_deg=(0.0, 0.0), peak_intensity_percent=100.0, min_intensity_percent=5.0,
                 dropout_probability=0.0, seed=42):
        values = [*mount, azimuth_deg, elevation_deg, fov_deg, noise_stddev_deg, *bias_deg,
                  peak_intensity_percent, min_intensity_percent, dropout_probability]
        if len(mount) != 6 or len(bias_deg) != 2 or not all(map(math.isfinite, values)):
            raise ValueError('mount needs 6 values, bias_deg 2, and all values must be finite')
        if abs(elevation_deg) > 90 or not 0 < fov_deg < 90 or noise_stddev_deg < 0:
            raise ValueError('require |elevation_deg| <= 90, 0 < fov_deg < 90, noise_stddev >= 0')
        if not 0 <= min_intensity_percent <= peak_intensity_percent or \
                not 0 <= dropout_probability <= 1:
            raise ValueError('require 0 <= min_intensity <= peak_intensity, 0 <= dropout <= 1')
        self.rotation = rotation_matrix(*mount[3:])
        azimuth, elevation = math.radians(azimuth_deg), math.radians(elevation_deg)
        self.sun = (math.cos(elevation) * math.cos(azimuth),
                    math.cos(elevation) * math.sin(azimuth), math.sin(elevation))
        self.fov_deg = fov_deg
        self.noise = noise_stddev_deg
        self.bias = tuple(bias_deg)
        self.peak = peak_intensity_percent
        self.minimum = min_intensity_percent
        self.dropout = dropout_probability
        self.random = random.Random(seed)

    def sun_in_sensor(self, yaw):
        """Return the unit vector toward the sun in the sensor frame at this body yaw."""
        c, s = math.cos(yaw), math.sin(yaw)
        body = (c * self.sun[0] + s * self.sun[1], -s * self.sun[0] + c * self.sun[1],
                self.sun[2])
        return tuple(sum(self.rotation[row][axis] * body[row] for row in range(3))
                     for axis in range(3))

    def observe(self, yaw):
        """Return one Observation for the true body yaw [rad]."""
        if self.random.random() < self.dropout:
            return Observation(False, ERROR_TRANSMISSION_FAILED, math.nan, math.nan, math.nan,
                               (0.0, 0.0, 0.0))
        x, y, z = self.sun_in_sensor(yaw)
        intensity = self.peak * z
        if z <= 0 or intensity < self.minimum:
            return Observation(False, ERROR_NOT_ENOUGH_RADIATION, NO_LIGHT_ANGLE_DEG,
                               NO_LIGHT_ANGLE_DEG, max(intensity, 0.0), (0.0, 0.0, 0.0))
        alpha = math.degrees(math.atan2(x, z)) + self.bias[0] + self.random.gauss(0, self.noise)
        beta = math.degrees(math.atan2(y, z)) + self.bias[1] + self.random.gauss(0, self.noise)
        if abs(alpha) > self.fov_deg or abs(beta) > self.fov_deg:
            return Observation(False, ERROR_OUT_OF_FIELD_OF_VIEW, alpha, beta, intensity,
                               (0.0, 0.0, 0.0))
        # Same conversion as the real driver.
        tan_alpha, tan_beta = math.tan(math.radians(alpha)), math.tan(math.radians(beta))
        norm = math.sqrt(tan_alpha ** 2 + tan_beta ** 2 + 1)
        return Observation(True, ERROR_NONE, alpha, beta, intensity,
                           (tan_alpha / norm, tan_beta / norm, 1 / norm))


class DelayedSampler:
    """Sample at a fixed rate and release each sample after the read latency."""

    def __init__(self, rate_hz, latency_s):
        if not math.isfinite(rate_hz) or rate_hz <= 0 or not math.isfinite(latency_s) or \
                latency_s < 0:
            raise ValueError('require rate_hz > 0 and latency_s >= 0')
        self.period = 1.0 / rate_hz
        self.latency = latency_s
        self.pending = collections.deque()
        self.next_sample = None
        self.last = None

    def update(self, now, sample):
        """Advance to now; return the [(publish time, sample)] that are due."""
        if self.last is not None and now < self.last:  # the simulator was restarted
            self.pending.clear()
            self.next_sample = None
        self.last = now
        if self.next_sample is None or now >= self.next_sample:
            self.pending.append((now + self.latency, sample()))
            self.next_sample = now + self.period if self.next_sample is None \
                else self.next_sample + self.period
            if self.next_sample <= now:
                self.next_sample = now + self.period
        due = []
        while self.pending and self.pending[0][0] <= now:
            due.append(self.pending.popleft())
        return due


def main(argv=None):
    """Run the ROS node."""
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time

    from airtable_interfaces.msg import SunSensorState
    from nav_msgs.msg import Odometry

    class SimSunSensorNode(Node):
        def __init__(self):
            super().__init__('airtable_sim_sun_sensor')
            value = {name: self.declare_parameter(f'sun_sensor.{name}', default).value
                     for name, default in (
                         ('frame_id', 'sun_sensor_link'), ('mount', [0.0] * 6),
                         ('azimuth_deg', 180.0), ('elevation_deg', 0.0), ('rate_hz', 10.0),
                         ('latency_s', 0.016), ('fov_deg', 60.0), ('noise_stddev_deg', 0.1),
                         ('bias_deg', [0.0, 0.0]), ('peak_intensity_percent', 100.0),
                         ('min_intensity_percent', 5.0), ('dropout_probability', 0.0),
                         ('seed', 42))}
            self.frame_id = value['frame_id']
            self.model = SunSensorModel(
                list(value['mount']), value['azimuth_deg'], value['elevation_deg'],
                fov_deg=value['fov_deg'], noise_stddev_deg=value['noise_stddev_deg'],
                bias_deg=tuple(value['bias_deg']),
                peak_intensity_percent=value['peak_intensity_percent'],
                min_intensity_percent=value['min_intensity_percent'],
                dropout_probability=value['dropout_probability'], seed=value['seed'])
            self.sampler = DelayedSampler(value['rate_hz'], value['latency_s'])
            self.publisher = self.create_publisher(SunSensorState, 'measurements/sun_sensor', 10)
            self.create_subscription(Odometry, 'sim/ground_truth', self.on_state, 10)

        def on_state(self, message):
            stamp = message.header.stamp
            orientation = message.pose.pose.orientation
            yaw = 2.0 * math.atan2(orientation.z, orientation.w)
            due = self.sampler.update(stamp.sec + stamp.nanosec * 1e-9,
                                      lambda: self.model.observe(yaw))
            for time_s, observation in due:
                state = SunSensorState()
                state.header.stamp = Time(nanoseconds=round(time_s * 1e9)).to_msg()
                state.header.frame_id = self.frame_id
                state.valid = observation.valid
                state.error_code = observation.error_code
                state.alpha_deg = observation.alpha_deg
                state.beta_deg = observation.beta_deg
                state.intensity_percent = observation.intensity_percent
                state.sun_vector.x, state.sun_vector.y, state.sun_vector.z = \
                    observation.sun_vector
                self.publisher.publish(state)

    import signal

    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=argv)
    node = SimSunSensorNode()
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
