"""Where the drones start and which of them lead.

Drones start on the floor of sector A and take off to their own altitude,
like a real Crazyflie. `place_swarm()` draws their floor positions at random
with the run's seed: the horizontal spacing keeps them apart, the clearance
keeps them off the walls and obstacles. Each drone draws a takeoff altitude
from A's altitude band, and j drones, chosen at random, become leaders.
Same course, n, j and seed: same swarm.
"""

import math
import random
from dataclasses import dataclass

from shapely import box
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

from maps.course import Course

ATTEMPTS_PER_DRONE = 500
ALTITUDE_MARGIN = 0.3  # no takeoff altitude closer than this to the floor or the ceiling


class SpawnError(Exception):
    """The run settings can't be satisfied on this course."""


@dataclass(frozen=True)
class Drone:
    id: int
    xy: tuple[float, float]  # where it stands on the floor, metres
    yaw: float  # radians
    altitude: float  # where it takes off to, metres
    leader: bool  # knows where sector C is


def place_swarm(
    course: Course, n: int, j: int, seed: int, spacing: float = 1.0, clearance: float = 0.5
) -> tuple[Drone, ...]:
    """n drones in sector A, j of them leaders.

    `spacing`: minimum horizontal distance between two drones.
    `clearance`: minimum distance from any wall or obstacle.
    """
    if n < 1:
        raise SpawnError(f"need at least one drone, got n = {n}")
    if not 0 <= j <= n:
        raise SpawnError(f"the leaders must be between 0 and n = {n}, got j = {j}")
    low, high = _altitude_band(course)
    area = course.sector_a.area

    obstacles = _obstacles(course)
    rng = random.Random(seed)
    spots: list[tuple[float, float]] = []
    for _ in range(n * ATTEMPTS_PER_DRONE):
        if len(spots) == n:
            break
        x = rng.uniform(area.x_min, area.x_max)
        y = rng.uniform(area.y_min, area.y_max)
        if Point(x, y).distance(obstacles) < clearance:
            continue
        if any(math.hypot(x - sx, y - sy) < spacing for sx, sy in spots):
            continue
        spots.append((x, y))
    if len(spots) < n:
        raise SpawnError(
            f"could only place {len(spots)} of {n} drones in sector A with {spacing} m "
            f"spacing and {clearance} m clearance; lower n or the spacing, or enlarge A"
        )

    leaders = set(rng.sample(range(n), j))
    return tuple(
        Drone(
            id=i,
            xy=xy,
            yaw=rng.uniform(-math.pi, math.pi),
            altitude=rng.uniform(low, high),
            leader=i in leaders,
        )
        for i, xy in enumerate(spots)
    )


def _altitude_band(course: Course) -> tuple[float, float]:
    """A's altitude band, kept ALTITUDE_MARGIN away from the floor and the ceiling."""
    z_min, z_max = course.sector_a.z
    low = max(z_min, ALTITUDE_MARGIN)
    high = min(z_max, course.ceiling - ALTITUDE_MARGIN)
    if low > high:
        raise SpawnError(
            f"sectors.A.z: takeoff altitudes must be at least {ALTITUDE_MARGIN} m from "
            f"the floor and the ceiling ({course.ceiling} m), got [{z_min}, {z_max}]"
        )
    return low, high


def _obstacles(course: Course) -> BaseGeometry:
    """Everything a drone must keep clear of: the arena edge, walls and pillars."""
    shapes = [course.area.boundary]
    shapes += [box(w.rect.x_min, w.rect.y_min, w.rect.x_max, w.rect.y_max) for w in course.walls]
    shapes += [Point(p.center).buffer(p.radius) for p in course.pillars]
    return unary_union(shapes)
