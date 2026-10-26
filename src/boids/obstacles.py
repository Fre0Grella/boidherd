"""Where the walls are, from me: the map's walls against my GPS position.

Every wall is an axis-aligned box seen from above (the arena boundary is
built as such boxes, maps.course). Each gets its own shill agent at its
nearest point, as each obstacle does in robotsim; nearest points on a box
are a clamp, no geometry library needed in the controller.

The arena is not convex (the L hall), which robotsim never handled; split
into convex boxes it needs nothing special. At a concave corner of the
arena two boxes meet, and a drone right at it feels both.
"""

from util import Vec3

Box = tuple[float, float, float, float]  # x_min, y_min, x_max, y_max


def to_walls(position: Vec3, boxes: list[Box]) -> list[Vec3]:
    """From `position` to the nearest point of each box, metres, horizontal. Zero when
    inside one."""
    return [
        Vec3(_clamp(position.x, x0, x1) - position.x, _clamp(position.y, y0, y1) - position.y, 0)
        for x0, y0, x1, y1 in boxes
    ]


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)
