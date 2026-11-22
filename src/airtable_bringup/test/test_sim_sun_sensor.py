"""Simulated sun sensor geometry, errors and timing, without ROS."""

import importlib.util
import math
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location(
    'sim_sun_sensor', Path(__file__).resolve().parents[1] / 'scripts/sim_sun_sensor.py')
sim = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sim)

# Sensor on the robot's right side: +Z toward body -Y, +X toward body +X.
MOUNT = [0.0, -0.1, 0.0, math.pi / 2, 0.0, 0.0]


def model(azimuth=180.0, elevation=0.0, **options):
    options.setdefault('noise_stddev_deg', 0.0)
    return sim.SunSensorModel(MOUNT, azimuth, elevation, **options)


class SunSensorModelTest(unittest.TestCase):
    """The light is at table -X unless a test says otherwise."""

    def test_mount_axes(self):
        rotation = sim.rotation_matrix(*MOUNT[3:])
        columns = [[rotation[row][axis] for row in range(3)] for axis in range(3)]
        for actual, expected in zip(columns, ([1, 0, 0], [0, 0, 1], [0, -1, 0])):
            for value, target in zip(actual, expected):
                self.assertAlmostEqual(value, target)

    def test_light_is_centered_at_yaw_minus_90(self):
        observation = model().observe(math.radians(-90))
        self.assertTrue(observation.valid)
        self.assertAlmostEqual(observation.alpha_deg, 0.0)
        self.assertAlmostEqual(observation.beta_deg, 0.0)
        self.assertAlmostEqual(observation.intensity_percent, 100.0)
        for value, target in zip(observation.sun_vector, (0, 0, 1)):
            self.assertAlmostEqual(value, target)

    def test_alpha_follows_yaw_inside_the_field_of_view(self):
        for yaw in (-145, -120, -100, -90, -80, -60, -35):
            observation = model().observe(math.radians(yaw))
            self.assertTrue(observation.valid, yaw)
            self.assertAlmostEqual(observation.alpha_deg, -(yaw + 90), places=9)
            self.assertAlmostEqual(math.dist(observation.sun_vector, (0, 0, 0)), 1.0)

    def test_outside_the_field_of_view_is_not_valid(self):
        observation = model().observe(math.radians(-20))
        self.assertFalse(observation.valid)
        self.assertEqual(observation.error_code, sim.ERROR_OUT_OF_FIELD_OF_VIEW)
        self.assertAlmostEqual(observation.alpha_deg, -70.0)
        self.assertEqual(observation.sun_vector, (0.0, 0.0, 0.0))

    def test_noise_past_grazing_is_not_a_direction(self):
        observation = model(min_intensity_percent=0.0, bias_deg=(5.0, 0.0)).observe(
            math.radians(-179))
        self.assertFalse(observation.valid)
        self.assertEqual(observation.error_code, sim.ERROR_NOT_ENOUGH_RADIATION)

    def test_light_at_or_behind_the_sensor_plane_is_no_radiation(self):
        for yaw in (0, 45, 90, 180):
            observation = model().observe(math.radians(yaw))
            self.assertEqual(observation.error_code, sim.ERROR_NOT_ENOUGH_RADIATION, yaw)
            self.assertEqual((observation.alpha_deg, observation.beta_deg), (60.0, 60.0))

    def test_light_direction_can_be_changed(self):
        # Light at table -Y: the sensor faces it at yaw 0.
        self.assertAlmostEqual(model(azimuth=-90).observe(0.0).alpha_deg, 0.0)
        # A raised light appears on sensor +Y, the body's up axis.
        self.assertAlmostEqual(model(elevation=10).observe(math.radians(-90)).beta_deg, 10.0)

    def test_dim_light_is_not_enough_radiation(self):
        observation = model(peak_intensity_percent=40, min_intensity_percent=30).observe(
            math.radians(-140))
        self.assertEqual(observation.error_code, sim.ERROR_NOT_ENOUGH_RADIATION)

    def test_bias_and_reproducible_noise(self):
        biased = model(bias_deg=(1.0, -2.0)).observe(math.radians(-90))
        self.assertAlmostEqual(biased.alpha_deg, 1.0)
        self.assertAlmostEqual(biased.beta_deg, -2.0)
        first, second = (model(noise_stddev_deg=0.1, seed=7) for _ in range(2))
        samples = [first.observe(math.radians(-90)).alpha_deg for _ in range(5)]
        self.assertEqual(samples, [second.observe(math.radians(-90)).alpha_deg for _ in range(5)])
        self.assertGreater(len(set(samples)), 1)

    def test_dropout_is_a_transmission_failure(self):
        observation = model(dropout_probability=1.0).observe(math.radians(-90))
        self.assertEqual(observation.error_code, sim.ERROR_TRANSMISSION_FAILED)
        self.assertTrue(math.isnan(observation.alpha_deg))

    def test_invalid_settings_are_rejected(self):
        for options in ({'fov_deg': 90}, {'noise_stddev_deg': -1}, {'dropout_probability': 2},
                        {'min_intensity_percent': 200}, {'bias_deg': (0.0,)}):
            with self.assertRaises(ValueError):
                model(**options)
        with self.assertRaises(ValueError):
            model(elevation=91)


class DelayedSamplerTest(unittest.TestCase):
    """Samples follow the rate and appear after the latency."""

    def test_rate_and_latency(self):
        sampler = sim.DelayedSampler(10.0, 0.016)
        count = []
        published = []
        for step in range(101):
            now = step * 0.01
            published += sampler.update(now, lambda: count.append(now) or now)
        self.assertEqual(len(count), 11)
        for time_s, sample_time in published:
            self.assertAlmostEqual(time_s, sample_time + 0.016)
        self.assertEqual(len(published), 10)  # the last sample is still in flight

    def test_zero_latency_is_released_at_once(self):
        sampler = sim.DelayedSampler(10.0, 0.0)
        self.assertEqual(sampler.update(1.0, lambda: 'a'), [(1.0, 'a')])

    def test_clock_rewind_forgets_pending_samples(self):
        sampler = sim.DelayedSampler(10.0, 0.5)
        sampler.update(5.0, lambda: 'old')
        self.assertEqual(sampler.update(0.0, lambda: 'new'), [])
        self.assertEqual(sampler.update(0.5, lambda: 'later'), [(0.5, 'new')])


if __name__ == '__main__':
    unittest.main()
