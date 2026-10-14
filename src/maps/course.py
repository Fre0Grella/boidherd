"""From a `Map` to a `Course`: the concrete geometry Webots will build.

`build_course()` turns the arena rectangles into boundary walls, grows the
forests into pillars, and runs the checks that need geometry: a connected
arena, sectors inside it, obstacles clear of A and C, and a way from A to C.
"""

from dataclasses import dataclass

from shapely import box
from shapely.geometry import Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.geometry.polygon import orient
from shapely.ops import unary_union

from .forest import grow_forest
from .load import MapError
from .model import Map, Pillar, Rect, Sector, Wall

WALL_THICKNESS = 0.1  # boundary walls, built just outside the free space
DRONE_RADIUS = 0.1  # half of the narrowest passage a drone can fly through


@dataclass(frozen=True)
class Course:
    ceiling: float
    area: Polygon  # free space inside the arena, before obstacles
    floor: Rect  # the arena's bounding rectangle: floor tiles line up with its outer walls
    boundary: tuple[Wall, ...]  # around the arena, full height
    walls: tuple[Wall, ...]  # from the map
    pillars: tuple[Pillar, ...]  # placed by hand and grown from forests
    sector_a: Sector
    sector_c: Sector


def build_course(m: Map) -> Course:
    area = _arena_area(m)
    a, c = _rect_shape(m.sector_a.area), _rect_shape(m.sector_c.area)
    for name, sector in (("A", a), ("C", c)):
        if not area.covers(sector):
            raise MapError(f"sectors.{name}: not inside the arena")
    if a.intersection(c).area > 0:
        raise MapError("sectors: A and C overlap")

    for i, wall in enumerate(m.walls):
        _check_obstacle(_rect_shape(wall.rect), f"obstacles.walls[{i}]", area, a, c)
    for i, pillar in enumerate(m.pillars):
        _check_obstacle(_disk(pillar), f"obstacles.pillars[{i}]", area, a, c)

    outside = _rect_shape(_bounds(area, margin=1.0)).difference(area)
    pillars = list(m.pillars)
    for i, forest in enumerate(m.forests):
        blocked = unary_union([outside, a, c, *(_rect_shape(w.rect) for w in m.walls)])
        pillars += grow_forest(forest, blocked, pillars, f"obstacles.forests[{i}]")

    _check_reachable(area, m.walls, pillars, a, c)
    return Course(
        ceiling=m.arena.ceiling,
        area=area,
        floor=_bounds(area, margin=0.0),
        boundary=_boundary_walls(area),
        walls=m.walls,
        pillars=tuple(pillars),
        sector_a=m.sector_a,
        sector_c=m.sector_c,
    )


def _arena_area(m: Map) -> Polygon:
    union = unary_union([_rect_shape(r) for r in m.arena.rects]).simplify(0)
    if not isinstance(union, Polygon):
        raise MapError(
            "arena.rects: the rectangles don't all connect; "
            "each must overlap or share an edge with another"
        )
    return orient(union, sign=1.0)  # free space on the left of every edge


def _check_obstacle(shape: BaseGeometry, path: str, area: Polygon, a: Polygon, c: Polygon) -> None:
    if not area.covers(shape):
        raise MapError(f"{path}: not inside the arena")
    for name, sector in (("A", a), ("C", c)):
        if shape.intersection(sector).area > 0:
            raise MapError(f"{path}: overlaps sector {name}")


def _check_reachable(
    area: Polygon, walls: tuple[Wall, ...], pillars: list[Pillar], a: Polygon, c: Polygon
) -> None:
    """Obstacles must leave a passage a drone wide from A to C (seen from above).

    The free space minus the obstacles, shrunk by the drone's radius, is
    where a drone's centre can be; A and C must lie in the same piece of it.
    Every obstacle blocks, whatever its height: until phase 4 the flock
    flies at a fixed altitude, which the map doesn't know.
    """
    solid = [_rect_shape(w.rect) for w in walls] + [_disk(p) for p in pillars]
    free = area.difference(unary_union(solid)).buffer(-DRONE_RADIUS)
    pieces = list(getattr(free, "geoms", [free]))
    if not any(piece.intersects(a) and piece.intersects(c) for piece in pieces):
        raise MapError(
            f"sectors.C: can't be reached from A; obstacles leave "
            f"no passage wider than {2 * DRONE_RADIUS} m"
        )


def _boundary_walls(area: Polygon) -> tuple[Wall, ...]:
    """One box per edge of the arena outline, just outside the free space.

    Each box is stretched by the wall thickness at convex corners, so the
    corner is closed, but not at concave ones, where it would poke into the
    free space.
    """
    walls = []
    for ring in (area.exterior, *area.interiors):
        points = list(ring.coords)[:-1]
        n = len(points)
        for i in range(n):
            before, start, end, after = (points[(i + k) % n] for k in (-1, 0, 1, 2))
            convex_start = _is_convex(before, start, end)
            convex_end = _is_convex(start, end, after)
            walls.append(_wall_along(start, end, convex_start, convex_end))
    return tuple(walls)


def _wall_along(
    start: tuple[float, ...], end: tuple[float, ...], stretch_start: bool, stretch_end: bool
) -> Wall:
    t = WALL_THICKNESS
    length = abs(end[0] - start[0]) + abs(end[1] - start[1])  # edges are axis-aligned
    dx, dy = (end[0] - start[0]) / length, (end[1] - start[1]) / length
    x0, y0 = start[0] - dx * t * stretch_start, start[1] - dy * t * stretch_start
    x1, y1 = end[0] + dx * t * stretch_end, end[1] + dy * t * stretch_end
    nx, ny = dy * t, -dx * t  # the outside is on the right of the edge
    xs, ys = (x0, x1, x0 + nx, x1 + nx), (y0, y1, y0 + ny, y1 + ny)
    return Wall(rect=Rect(min(xs), min(ys), max(xs), max(ys)), height=None)


def _is_convex(p: tuple[float, ...], q: tuple[float, ...], r: tuple[float, ...]) -> bool:
    """True if the free space turns left at q (a convex corner of the free space)."""
    return (q[0] - p[0]) * (r[1] - q[1]) - (q[1] - p[1]) * (r[0] - q[0]) > 0


def _rect_shape(r: Rect) -> Polygon:
    return box(r.x_min, r.y_min, r.x_max, r.y_max)


def _disk(p: Pillar) -> Polygon:
    return Point(p.center).buffer(p.radius)


def _bounds(shape: BaseGeometry, margin: float) -> Rect:
    x_min, y_min, x_max, y_max = shape.bounds
    return Rect(x_min - margin, y_min - margin, x_max + margin, y_max + margin)
