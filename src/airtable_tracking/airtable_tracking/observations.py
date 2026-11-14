"""Validate marker observations and keep a fixed-camera table calibration."""

from airtable_tracking.geometry import planar_base_pose, pose_matrix, register_markers
import numpy as np


class MarkerObservations:
    """Calibrate once from contemporaneous fixed markers, then convert moving poses."""

    def __init__(self, fixed_ids, table_points, moving_id, camera_frame,
                 base_from_marker, base_height, max_time_span, max_residual):
        """Configure the measured marker layout and the rigid vehicle mounting."""
        self.fixed_ids = tuple(fixed_ids)
        self.table_points = np.asarray(table_points, dtype=float)
        if len(set(self.fixed_ids)) != len(self.fixed_ids) or moving_id in self.fixed_ids:
            raise ValueError('Fixed IDs must be unique and exclude the moving marker')
        if not camera_frame:
            raise ValueError('camera_frame must not be empty')
        if not np.isfinite([base_height, max_time_span, max_residual]).all():
            raise ValueError('Height and calibration thresholds must be finite')
        if max_time_span <= 0 or max_residual <= 0:
            raise ValueError('Calibration thresholds must be positive')
        if self.table_points.shape != (len(self.fixed_ids), 3):
            raise ValueError('fixed_marker_positions must contain XYZ for each fixed ID')
        # Check the configured layout before any messages arrive.
        register_markers(self.table_points, self.table_points, max_residual)
        self.camera_frame = camera_frame
        self.base_from_marker = base_from_marker
        self.base_height = base_height
        self.max_span_ns = int(max_time_span * 1e9)
        self.max_residual = max_residual
        self.fixed = {}
        self.camera_from_table = None
        self.calibration_stamp_ns = None
        self.last_moving_stamp_ns = None

    def _validate(self, frame_id, stamp_ns):
        if frame_id != self.camera_frame:
            raise ValueError(f'Expected {self.camera_frame}, received {frame_id}')
        if stamp_ns < 0:
            raise ValueError('Observation time must not be negative')

    def add_fixed(self, marker_id, frame_id, stamp_ns, position):
        """Collect fixed centers and return true exactly when calibration completes."""
        self._validate(frame_id, stamp_ns)
        if marker_id not in self.fixed_ids:
            raise ValueError('Unknown fixed marker ID')
        if self.camera_from_table is not None:
            return False
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('Fixed marker position must contain three finite values')
        previous = self.fixed.get(marker_id)
        if previous is not None and stamp_ns < previous[0]:
            return False
        self.fixed[marker_id] = (stamp_ns, position)
        if len(self.fixed) != len(self.fixed_ids):
            return False
        stamps = [self.fixed[mid][0] for mid in self.fixed_ids]
        if max(stamps) - min(stamps) > self.max_span_ns:
            return False
        self.camera_from_table = register_markers(
            self.table_points, [self.fixed[mid][1] for mid in self.fixed_ids], self.max_residual
        )
        self.calibration_stamp_ns = max(stamps)
        return True

    def convert_moving(self, frame_id, stamp_ns, position, quaternion):
        """Return a planar base pose, or None before calibration or for old observations."""
        self._validate(frame_id, stamp_ns)
        if self.camera_from_table is None:
            return None
        if stamp_ns < self.calibration_stamp_ns:
            return None
        if self.last_moving_stamp_ns is not None and stamp_ns <= self.last_moving_stamp_ns:
            return None
        camera_from_marker = pose_matrix(position, quaternion)
        result = planar_base_pose(
            self.camera_from_table, camera_from_marker, self.base_from_marker, self.base_height
        )
        self.last_moving_stamp_ns = stamp_ns
        return result
