"""Per-valve timeline from pulse commands, without ROS."""

import importlib.util
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location(
    'valve_state', Path(__file__).resolve().parents[1] / 'scripts/valve_state.py')
valve = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(valve)
NAMES = ['thruster_1', 'thruster_2', 'thruster_3', 'thruster_4']


class ValveTimelineTest(unittest.TestCase):
    """Edges must sit at the pulse arrival and its scheduled end."""

    def test_open_and_close_at_exact_times(self):
        timeline = valve.ValveTimeline(NAMES)
        events, unknown = timeline.pulse(1.0, ['thruster_1', 'thruster_3'], 0.01)
        self.assertEqual(events, [(1.0, [1.0, 0.0, 1.0, 0.0])])
        self.assertEqual(unknown, [])
        self.assertEqual(timeline.close_due(1.005), [])
        self.assertEqual(timeline.close_due(1.02), [(1.01, [0.0] * 4)])

    def test_separate_ends_are_separate_edges(self):
        timeline = valve.ValveTimeline(NAMES)
        timeline.pulse(0.0, ['thruster_1'], 0.03)
        timeline.pulse(0.01, ['thruster_2'], 0.01)
        self.assertEqual(timeline.close_due(0.05), [
            (0.02, [1.0, 0.0, 0.0, 0.0]), (0.03, [0.0] * 4)])

    def test_new_pulse_restarts_like_the_plant(self):
        """airtable_hardware/sim set PulseUntil(now + duration); a shorter pulse shortens."""
        timeline = valve.ValveTimeline(NAMES)
        timeline.pulse(0.0, ['thruster_1'], 0.10)
        events, _ = timeline.pulse(0.02, ['thruster_1'], 0.01)
        self.assertEqual(events, [])  # already open: no edge
        self.assertEqual(timeline.close_due(0.2), [(0.03, [0.0] * 4)])

    def test_pending_close_before_a_later_pulse(self):
        """A close that the timer has not handled yet still comes before the next open."""
        timeline = valve.ValveTimeline(NAMES)
        timeline.pulse(0.0, ['thruster_1'], 0.01)
        events, _ = timeline.pulse(0.015, ['thruster_1'], 0.01)
        self.assertEqual(events, [(0.01, [0.0] * 4), (0.015, [1.0, 0.0, 0.0, 0.0])])

    def test_ignore_zero_unknown_and_clock_reset(self):
        timeline = valve.ValveTimeline(NAMES)
        self.assertEqual(timeline.pulse(0.0, ['thruster_1'], 0.0), ([], []))
        _, unknown = timeline.pulse(0.0, ['thruster_9', 'thruster_2'], 0.05)
        self.assertEqual(unknown, ['thruster_9'])
        timeline.pulse(5.0, ['thruster_1'], 1.0)
        # The simulator restarted at t=0: no stale open valves, no edge in the past.
        self.assertEqual(timeline.close_due(0.1), [])
        self.assertEqual(timeline.states(), [0.0] * 4)

    def test_reject_bad_names(self):
        for names in ([], ['a', 'a'], ['']):
            with self.subTest(names=names), self.assertRaises(ValueError):
                valve.ValveTimeline(names)


if __name__ == '__main__':
    unittest.main()
