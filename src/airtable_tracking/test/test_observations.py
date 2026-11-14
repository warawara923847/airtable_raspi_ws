"""Exercise missing, stale, mismatched, and reordered marker observations."""

from airtable_tracking.observations import MarkerObservations
import numpy as np
import pytest

IDS = [10, 11, 12, 13]
POINTS = [[0., 0., 0.], [1., 0., 0.], [0., 2., 0.], [1., 2., 0.]]
FRAME = 'camera_infra1_optical_frame'
STAMP = 1_000_000_000


def observations():
    """Create an identity-camera fixture with a marker at the vehicle origin."""
    return MarkerObservations(IDS, POINTS, 50, FRAME, np.eye(4), 0., 0.1, 0.02)


def calibrate(tracker):
    """Deliver a complete, contemporaneous set of fixed marker centers."""
    for mid, point in zip(IDS, POINTS):
        tracker.add_fixed(mid, FRAME, STAMP, point)


def test_missing_fixed_markers_produce_no_vehicle_pose():
    """Wait for all fixed observations rather than silently inventing a world frame."""
    tracker = observations()
    for mid, point in zip(IDS[:-1], POINTS[:-1]):
        tracker.add_fixed(mid, FRAME, STAMP, point)
    assert tracker.convert_moving(FRAME, STAMP, [0.4, 0.8, 0.], [0., 0., 0., 1.]) is None
    assert tracker.add_fixed(13, FRAME, STAMP, POINTS[-1])
    np.testing.assert_allclose(
        tracker.convert_moving(FRAME, STAMP, [0.4, 0.8, 0.], [0., 0., 0., 1.]),
        [0.4, 0.8, 0., 0.]
    )


def test_stale_fixed_batch_waits_for_fresh_observations():
    """Avoid combining old and new calibration frames; recover once the set is fresh."""
    tracker = observations()
    tracker.add_fixed(10, FRAME, 0, POINTS[0])
    for mid, point in zip(IDS[1:], POINTS[1:]):
        assert not tracker.add_fixed(mid, FRAME, STAMP, point)
    assert tracker.camera_from_table is None
    assert tracker.add_fixed(10, FRAME, STAMP, POINTS[0])


def test_old_fixed_observation_cannot_replace_newer_one():
    """An out-of-order fixed detection must not corrupt pending initialization."""
    tracker = observations()
    tracker.add_fixed(10, FRAME, STAMP, POINTS[0])
    tracker.add_fixed(10, FRAME, 0, [99., 99., 99.])
    for mid, point in zip(IDS[1:], POINTS[1:]):
        tracker.add_fixed(mid, FRAME, STAMP, point)
    np.testing.assert_allclose(tracker.camera_from_table, np.eye(4), atol=1e-12)


def test_wrong_camera_frame_is_rejected():
    """Do not mix IR, RGB, or unrelated camera coordinate systems."""
    tracker = observations()
    with pytest.raises(ValueError, match='Expected'):
        tracker.add_fixed(10, 'another_camera', STAMP, POINTS[0])
    calibrate(tracker)
    with pytest.raises(ValueError, match='Expected'):
        tracker.convert_moving('another_camera', STAMP, [0., 0., 0.], [0., 0., 0., 1.])


def test_duplicate_and_older_moving_observations_are_not_republished():
    """Each accepted measurement advances time; duplicate or stale ones do not."""
    tracker = observations()
    calibrate(tracker)
    assert tracker.convert_moving(FRAME, STAMP - 1, [0., 0., 0.], [0., 0., 0., 1.]) is None
    assert tracker.convert_moving(FRAME, STAMP, [0., 0., 0.], [0., 0., 0., 1.]) is not None
    assert tracker.convert_moving(FRAME, STAMP, [0., 0., 0.], [0., 0., 0., 1.]) is None
    assert tracker.convert_moving(FRAME, STAMP - 1, [0., 0., 0.], [0., 0., 0., 1.]) is None
    assert tracker.convert_moving(FRAME, STAMP + 1, [0., 0., 0.], [0., 0., 0., 1.]) is not None


def test_bad_moving_quaternion_does_not_consume_timestamp():
    """A rejected measurement must not prevent a valid replacement at the same time."""
    tracker = observations()
    calibrate(tracker)
    with pytest.raises(ValueError):
        tracker.convert_moving(FRAME, STAMP, [0., 0., 0.], [0., 0., 0., 0.])
    assert tracker.convert_moving(FRAME, STAMP, [0., 0., 0.], [0., 0., 0., 1.]) is not None
