"""Sun direction arrow geometry, without ROS."""

import importlib.util
import math
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location(
    'sun_direction', Path(__file__).resolve().parents[1] / 'scripts/sun_direction.py')
sun = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(sun)


class ArrowEndTest(unittest.TestCase):
    """The arrow has a fixed length along the measured direction, or does not exist."""

    def test_tip_is_along_the_vector_at_the_requested_length(self):
        tip = sun.arrow_end(True, (0.6, 0.0, 0.8), 0.5)
        for actual, expected in zip(tip, (0.3, 0.0, 0.4)):
            self.assertAlmostEqual(actual, expected)

    def test_vector_is_normalized(self):
        tip = sun.arrow_end(True, (0.0, 2.0, 0.0), 0.4)
        self.assertAlmostEqual(math.dist(tip, (0, 0, 0)), 0.4)

    def test_no_arrow_without_a_direction(self):
        for valid, vector in [(False, (0.0, 0.0, 1.0)), (True, (0.0, 0.0, 0.0)),
                              (True, (math.nan, 0.0, 1.0)), (True, (math.inf, 0.0, 1.0))]:
            self.assertIsNone(sun.arrow_end(valid, vector, 0.4))


if __name__ == '__main__':
    unittest.main()
