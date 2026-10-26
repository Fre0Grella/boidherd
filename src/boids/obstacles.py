"""Where the walls are, from me: the map's walls against my GPS position.

"Walls" are everything solid: the arena boundary and the map's walls, each
an axis-aligned box seen from above (maps.course), and pillars, disks. The
stopping limit (rules.py) looks at the nearest point of each, as robotsim's
shill agents do; nearest points on a box are a clamp and on a disk a step
along the radius, no geometry library needed in the controller. Until
phase 4 every one is full height, low ones too (as in the map's
reachability check).

The arena is not convex (the L hall), which robotsim never handled; split
into convex boxes it needs nothing special. At a concave corner of the
arena two boxes meet, and a drone right at it feels both.
"""

from util import Vec3

Box = tuple[float, float, float, float]  # x_min, y_min, x_max, y_max
Disk = tuple[float, float, float]  # x, y, radius


def to_walls(position: Vec3, boxes: list[Box], disks: list[Disk] | None = None) -> list[Vec3]:
    """From `position` to the nearest point of each box and disk, metres, horizontal. Zero
    when inside one."""
    to_boxes = [
        Vec3(_clamp(position.x, x0, x1) - position.x, _clamp(position.y, y0, y1) - position.y, 0)
        for x0, y0, x1, y1 in boxes
    ]
    return to_boxes + [_to_disk(position, d) for d in disks or []]


def _to_disk(position: Vec3, disk: Disk) -> Vec3:
    x, y, r = disk
    to_centre = Vec3(x - position.x, y - position.y, 0)
    d = to_centre.length()
    return to_centre * (1 - r / d) if d > r else Vec3()


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)
