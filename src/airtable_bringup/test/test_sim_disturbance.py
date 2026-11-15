"""Check experiment waveform boundaries and parameter errors without ROS."""

import importlib.util
import math
from pathlib import Path
import statistics
import unittest


SPEC = importlib.util.spec_from_file_location(
    'sim_disturbance', Path(__file__).resolve().parents[1] / 'scripts/sim_disturbance.py')
disturbance = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(disturbance)


class DisturbanceProfileTest(unittest.TestCase):
    """Test finite pulses, persistent steps and signed sinusoidal loads."""

    def test_pulse_boundaries_and_step(self):
        """The end instant stops a pulse while a step stays on."""
        for t, expected in [(7.9, 0.0), (8.0, 1.0), (9.99, 1.0), (10.0, 0.0)]:
            self.assertEqual(disturbance.profile_gain('pulse', t, 8, 2, 5), expected)
        self.assertEqual(disturbance.profile_gain('step', 100, 8, 2, 5), 1)
        self.assertEqual(disturbance.profile_gain('step', 0, 8, 2, 5), 0)

    def test_sine_has_both_signs_and_optional_duration(self):
        """A sinusoidal load begins at zero and can reverse its force and torque."""
        self.assertAlmostEqual(disturbance.profile_gain('sine', 8, 8, 5, 4), 0)
        self.assertAlmostEqual(disturbance.profile_gain('sine', 9, 8, 5, 4), 1)
        self.assertAlmostEqual(disturbance.profile_gain('sine', 11, 8, 5, 4), -1)
        self.assertEqual(disturbance.profile_gain('sine', 13, 8, 5, 4), 0)
        self.assertAlmostEqual(disturbance.profile_gain('sine', 17, 8, 0, 4), 1)

    def test_invalid_profiles_are_rejected(self):
        """Do not publish non-finite commands or nonsensical waveform parameters."""
        good = ('pulse', 8.0, 2.0, 5.0, 50.0, [0.02, 0.0, 0.0005])
        disturbance.validate_profile(*good)
        for index, value in [(0, 'typo'), (1, -1), (2, 0), (3, 0), (4, 0),
                             (4, 200), (5, [math.nan, 0, 0])]:
            bad = list(good)
            bad[index] = value
            with self.subTest(index=index, value=value), self.assertRaises(ValueError):
                disturbance.validate_profile(*bad)


class GaussianWrenchTest(unittest.TestCase):
    """Check distribution, simulation-time sampling and repeatable experiment conditions."""

    def test_distribution_and_independent_axes(self):
        """Recover requested means and variances from independent interval samples."""
        means = [0.01, -0.02, 0.0001]
        sigmas = [0.02, 0.03, 0.0005]
        generator = disturbance.GaussianWrench(means, sigmas, 0.1, 42)
        rows = [generator.sample(i * 0.1) for i in range(12000)]
        axes = list(zip(*rows))
        for data, mean, sigma in zip(axes, means, sigmas):
            self.assertLess(abs(statistics.mean(data) - mean), 0.05 * sigma)
            self.assertAlmostEqual(statistics.variance(data) / sigma**2, 1, delta=0.06)
        for a, b in [(0, 1), (1, 2), (2, 0)]:
            self.assertLess(abs(statistics.correlation(axes[a], axes[b])), 0.05)
        self.assertLess(abs(statistics.correlation(axes[0][:-1], axes[0][1:])), 0.05)

    def test_sampling_is_held_and_independent_of_callback_order(self):
        """Preserve the same noise after pauses, skipped ticks, late joins and time rewinds."""
        regular = disturbance.GaussianWrench([0, 0, 0], [1, 1, 1], 0.1, 42)
        expected = [regular.sample(i * 0.1) for i in range(100)]
        sparse = disturbance.GaussianWrench([0, 0, 0], [1, 1, 1], 0.1, 42)
        for i in [99, 30, 30, 0, 2, 70, 99]:
            self.assertEqual(sparse.sample(i * 0.1), expected[i])
            self.assertEqual(sparse.sample(i * 0.1 + 0.09), expected[i])
        self.assertNotEqual(expected[0], expected[1])
        different = disturbance.GaussianWrench([0, 0, 0], [1, 1, 1], 0.1, 43)
        self.assertNotEqual(different.sample(0), expected[0])

    def test_zero_variance_and_nonzero_mean(self):
        """Allow a constant bias and axes with no stochastic excitation."""
        generator = disturbance.GaussianWrench([0.02, -0.01, 0.0002], [0, 0, 0], 0.1, 42)
        for t in [0, 0.05, 0.1, 1000]:
            self.assertEqual(generator.sample(t), (0.02, -0.01, 0.0002))

    def test_finite_and_infinite_noise_windows(self):
        """Switch all stochastic axes off outside the specified trajectory-time window."""
        for t, expected in [(7.9, 0), (8, 1), (27.99, 1), (28, 0)]:
            self.assertEqual(disturbance.profile_gain('gaussian', t, 8, 20, 5), expected)
        self.assertEqual(disturbance.profile_gain('gaussian', 1000, 8, 0, 5), 1)

    def test_invalid_noise_parameters(self):
        """Reject negative deviations, invalid seeds and invalid sample times."""
        good = [[0, 0, 0], [0.02, 0.02, 0.0005], 0.1, 42]
        for index, value in [(0, [math.inf, 0, 0]), (0, [0, 0]), (1, [1, -1, 1]),
                             (1, [math.nan, 0, 0]), (2, 0), (2, math.inf), (3, -1),
                             (3, 2**63), (3, 1.5), (3, True)]:
            bad = list(good)
            bad[index] = value
            with self.subTest(index=index, value=value), self.assertRaises(ValueError):
                disturbance.GaussianWrench(*bad)
        generator = disturbance.GaussianWrench(*good)
        for t in [-0.1, math.inf, math.nan]:
            with self.subTest(t=t), self.assertRaises(ValueError):
                generator.sample(t)
