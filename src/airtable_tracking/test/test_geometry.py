"""Check geometric correctness with independent camera and vehicle ground truth."""

from airtable_tracking.geometry import planar_base_pose, pose_matrix, register_markers
import numpy as np
import pytest
from tf_transformations import quaternion_from_euler

POINTS = np.array([[0., 0., 0.], [1., 0., 0.], [0., 2., 0.], [1., 2., 0.]])


@pytest.mark.parametrize('camera_rpy', [
    (0., 0., 0.), (np.pi, 0., 0.), (2.8, 0.35, -0.6), (-1.2, 0.7, 2.1),
])
@pytest.mark.parametrize('yaw', [-3.12, -0.4, 0., 1.7, 3.12])
def test_camera_and_mounting_do_not_change_vehicle_pose(camera_rpy, yaw):
    """Recover the same table pose from tilted cameras and a rotated offset marker."""
    camera_from_table = pose_matrix([0.3, -0.5, 2.4], quaternion_from_euler(*camera_rpy))
    camera_points = (
        POINTS @ camera_from_table[:3, :3].T + camera_from_table[:3, 3]
    )
    calibrated = register_markers(POINTS, camera_points, 0.02)
    np.testing.assert_allclose(calibrated, camera_from_table, atol=1e-12)
    assert np.linalg.det(calibrated[:3, :3]) == pytest.approx(1.)

    base_from_marker = pose_matrix([0.08, -0.03, 0.147], quaternion_from_euler(0.2, -0.3, 0.5))
    table_from_base = pose_matrix([0.4, 1.2, 0.01], quaternion_from_euler(0., 0., yaw))
    camera_from_marker = camera_from_table @ table_from_base @ base_from_marker
    result = planar_base_pose(calibrated, camera_from_marker, base_from_marker, 0.01)
    np.testing.assert_allclose(result, [0.4, 1.2, 0.01, yaw], atol=1e-12)


def test_calibration_uses_all_four_points():
    """Reject a bad fourth fixed marker even when the first three look correct."""
    camera_points = POINTS.copy()
    camera_points[3, 2] += 0.4
    with pytest.raises(ValueError, match='residual'):
        register_markers(POINTS, camera_points, 0.02)


def test_small_measurement_noise_is_accepted():
    """Keep calibration usable with modest independent localization noise."""
    noisy = POINTS + np.random.default_rng(42).normal(0., 0.001, POINTS.shape)
    transform = register_markers(POINTS, noisy, 0.02)
    np.testing.assert_allclose(transform[:3, :3], np.eye(3), atol=0.005)
    np.testing.assert_allclose(transform[:3, 3], np.zeros(3), atol=0.005)


@pytest.mark.parametrize('points', [np.zeros((4, 3)), [[0., 0., 0.], [1., 0., 0.], [2., 0., 0.]]])
def test_degenerate_fixed_layout_is_rejected(points):
    """Reject coincident or collinear points rather than creating undefined axes."""
    with pytest.raises(ValueError, match='collinear'):
        register_markers(points, points, 0.02)


@pytest.mark.parametrize('position,quaternion', [
    ([float('nan'), 0., 0.], [0., 0., 0., 1.]),
    ([0., 0., 0.], [0., 0., 0., 0.]),
    ([0., 0., 0.], [0., float('inf'), 0., 1.]),
])
def test_invalid_pose_is_rejected(position, quaternion):
    """Prevent malformed sensor data from propagating NaNs into an estimator."""
    with pytest.raises(ValueError):
        pose_matrix(position, quaternion)
