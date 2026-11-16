"""Rigid registration and planar vehicle pose conversion, independent of ROS nodes."""

import numpy as np
from tf_transformations import quaternion_matrix


def pose_matrix(position, quaternion):
    """Return T_parent_child from position and a ROS-order (x, y, z, w) quaternion."""
    position = np.asarray(position, dtype=float)
    quaternion = np.asarray(quaternion, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError('Position must contain three finite values')
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError('Quaternion must contain four finite values')
    norm = np.linalg.norm(quaternion)
    if norm < 1e-8:
        raise ValueError('Quaternion must have nonzero norm')
    transform = quaternion_matrix(quaternion / norm)
    transform[:3, 3] = position
    return transform


def table_from_camera_root(camera_from_table, root_from_camera):
    """Return T_table_root for a camera driver frame (camera_link) above the optical frame."""
    return np.linalg.inv(camera_from_table) @ np.linalg.inv(root_from_camera)


def register_markers(table_points, camera_points, max_residual):
    """Fit T_camera_table to corresponding marker centers with a proper rotation."""
    table_points = np.asarray(table_points, dtype=float)
    camera_points = np.asarray(camera_points, dtype=float)
    if (table_points.ndim != 2 or table_points.shape[1] != 3 or
            table_points.shape[0] < 3 or camera_points.shape != table_points.shape):
        raise ValueError('At least three corresponding 3D marker centers are required')
    if not np.isfinite(table_points).all() or not np.isfinite(camera_points).all():
        raise ValueError('Marker centers must be finite')
    table_center = table_points.mean(axis=0)
    camera_center = camera_points.mean(axis=0)
    table_offsets = table_points - table_center
    camera_offsets = camera_points - camera_center
    if (np.linalg.svd(table_offsets, compute_uv=False)[1] < 1e-8 or
            np.linalg.svd(camera_offsets, compute_uv=False)[1] < 1e-8):
        raise ValueError('Fixed markers must not be coincident or collinear')
    left, _, right = np.linalg.svd(table_offsets.T @ camera_offsets)
    rotation = right.T @ left.T
    if np.linalg.det(rotation) < 0:
        right[-1, :] *= -1
        rotation = right.T @ left.T
    translation = camera_center - rotation @ table_center
    predicted = table_points @ rotation.T + translation
    residual = float(np.linalg.norm(predicted - camera_points, axis=1).max())
    if residual > max_residual:
        raise ValueError(f'Fixed marker layout residual {residual:.4f} m exceeds limit')
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def planar_base_pose(camera_from_table, camera_from_marker, base_from_marker, base_height):
    """Convert a camera marker observation to (x, y, z, yaw) of the vehicle base."""
    table_from_base = (
        np.linalg.inv(camera_from_table) @ camera_from_marker @ np.linalg.inv(base_from_marker)
    )
    forward = table_from_base[:2, 0]
    if np.linalg.norm(forward) < 1e-8:
        raise ValueError('Vehicle forward axis cannot be perpendicular to the table')
    yaw = float(np.arctan2(forward[1], forward[0]))
    return (*table_from_base[:2, 3], base_height, yaw)
