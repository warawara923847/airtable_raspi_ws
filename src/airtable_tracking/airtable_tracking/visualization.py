"""RViz markers for the configured table area and fixed marker layout."""

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

TABLE_COLOR = ColorRGBA(r=0.2, g=0.5, b=1.0, a=1.0)
FIXED_COLOR = ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.6)
LABEL_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def _marker(frame_id, namespace, marker_id, marker_type, color):
    marker = Marker()
    marker.header.frame_id = frame_id  # zero stamp: use the latest TF in RViz
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.color = color
    return marker


def layout_markers(table_frame, table_size, fixed_ids, fixed_positions, marker_size):
    """Return the table outline from its origin corner and the expected fixed markers."""
    # Squares use the configured centers and side length, so detected aruco_ros marker TFs
    # can be compared against them in RViz.
    width, length = (float(value) for value in table_size)
    if len(fixed_ids) != len(fixed_positions):
        raise ValueError('Each fixed marker ID needs one position')
    if not (width > 0 and length > 0 and marker_size > 0):
        raise ValueError('table_size and marker_size must be positive')
    markers = MarkerArray()
    outline = _marker(table_frame, 'table', 0, Marker.LINE_STRIP, TABLE_COLOR)
    outline.scale.x = 0.01
    outline.points = [Point(x=x, y=y, z=0.0) for x, y in (
        (0.0, 0.0), (width, 0.0), (width, length), (0.0, length), (0.0, 0.0))]
    markers.markers.append(outline)
    for marker_id, position in zip(fixed_ids, fixed_positions):
        x, y, z = (float(value) for value in position)
        square = _marker(table_frame, 'fixed_markers', int(marker_id), Marker.CUBE, FIXED_COLOR)
        square.pose.position = Point(x=x, y=y, z=z)
        square.scale.x = square.scale.y = float(marker_size)
        square.scale.z = 0.002
        label = _marker(table_frame, 'fixed_marker_ids', int(marker_id),
                        Marker.TEXT_VIEW_FACING, LABEL_COLOR)
        label.pose.position = Point(x=x, y=y, z=z + 0.08)
        label.scale.z = 0.06
        label.text = str(marker_id)
        markers.markers.extend([square, label])
    return markers
