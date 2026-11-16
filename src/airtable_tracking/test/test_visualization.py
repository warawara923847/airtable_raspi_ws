"""Check the RViz layout markers drawn from the tracking configuration."""

from airtable_tracking.visualization import layout_markers
import pytest
from visualization_msgs.msg import Marker

IDS = [10, 11, 12, 13]
POSITIONS = [[0.075, 1.925, 0.002], [0.925, 1.925, 0.002],
             [0.075, 0.075, 0.002], [0.925, 0.075, 0.002]]


def test_layout_draws_table_outline_and_expected_markers():
    """Outline the table from its origin corner and place one square and label per ID."""
    markers = layout_markers('table_1', [1.0, 2.0], IDS, POSITIONS, 0.1).markers
    assert all(m.header.frame_id == 'table_1' for m in markers)
    outline = markers[0]
    assert outline.type == Marker.LINE_STRIP
    assert [(p.x, p.y) for p in outline.points] == [
        (0.0, 0.0), (1.0, 0.0), (1.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
    squares = [m for m in markers if m.ns == 'fixed_markers']
    labels = [m for m in markers if m.ns == 'fixed_marker_ids']
    assert [m.id for m in squares] == IDS and [m.text for m in labels] == ['10', '11', '12', '13']
    for square, position in zip(squares, POSITIONS):
        assert [square.pose.position.x, square.pose.position.y,
                square.pose.position.z] == pytest.approx(position)
        assert (square.scale.x, square.scale.y) == pytest.approx((0.1, 0.1))


@pytest.mark.parametrize('table_size, marker_size', [([0.0, 2.0], 0.1), ([1.0, 2.0], 0.0)])
def test_layout_rejects_nonpositive_sizes(table_size, marker_size):
    """Refuse sizes that would draw nothing instead of publishing an empty outline."""
    with pytest.raises(ValueError, match='positive'):
        layout_markers('table_1', table_size, IDS, POSITIONS, marker_size)
