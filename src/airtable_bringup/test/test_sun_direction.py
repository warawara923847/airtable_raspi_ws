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
    """The arrow has a fixed length at alpha in the sensor X-Z plane, or does not exist."""

    def test_zero_alpha_points_along_the_boresight(self):
        for actual, expected in zip(sun.arrow_end(True, 0.0, 0.5), (0.0, 0.0, 0.5)):
            self.assertAlmostEqual(actual, expected)

    def test_alpha_turns_the_arrow_toward_sensor_x(self):
        tip = sun.arrow_end(True, 30.0, 0.4)
        for actual, expected in zip(tip, (0.2, 0.0, 0.4 * math.cos(math.radians(30)))):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(sun.arrow_end(True, -30.0, 0.4)[0], -0.2)
        self.assertAlmostEqual(math.dist(tip, (0, 0, 0)), 0.4)

    def test_arrow_stays_on_the_light_while_the_robot_turns(self):
        """Sensor on the right side, light at table -X: the arrow keeps pointing at -X."""
        for yaw_deg in (-140, -110, -90, -70, -40):
            alpha = -(yaw_deg + 90)
            x, _, z = sun.arrow_end(True, alpha, 1.0)
            body = (x, -z)  # sensor +X = body +X, sensor +Z = body -Y
            yaw = math.radians(yaw_deg)
            table = (math.cos(yaw) * body[0] - math.sin(yaw) * body[1],
                     math.sin(yaw) * body[0] + math.cos(yaw) * body[1])
            self.assertAlmostEqual(table[0], -1.0)
            self.assertAlmostEqual(table[1], 0.0)

    def test_no_arrow_without_a_direction(self):
        for valid, alpha in [(False, 0.0), (True, math.nan), (True, math.inf), (True, 90.0),
                             (True, -91.0)]:
            self.assertIsNone(sun.arrow_end(valid, alpha, 0.4))


if __name__ == '__main__':
    unittest.main()
