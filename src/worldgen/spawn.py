"""Where the drones start and which of them lead.

Drones start on the floor of sector A and take off to their own altitude,
like a real Crazyflie. `place_swarm()` draws their floor positions at random
with the run's seed, in a patch around A's centre sized for the swarm
(`area_per_drone` each), so every swarm starts at the same density whatever
n and A: a sweep over n then measures the swarm's size, not how sparse it
started (design log, the spawn). The horizontal spacing keeps them apart,
the clearance keeps them off the walls and obstacles, and a draw that falls
apart into groups is drawn again: the swarm starts as one flock. Each drone
draws a takeoff altitude from A's altitude band, and j drones, chosen at
random, become leaders. Same course, n, j and seed: same swarm.
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
SWARM_ATTEMPTS = 100  # draws, until one is a single group
PATCH_GROWTH = 0.1  # m: the patch's radius grows by this until it is large enough
ALTITUDE_MARGIN = 0.3  # no takeoff altitude closer than this to the floor or the ceiling


Point2 = tuple[float, float]


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
    course: Course,
    n: int,
    j: int,
    seed: int,
    spacing: float = 1.0,
    clearance: float = 0.5,
    area_per_drone: float = 2.0,
    link: float = 2.0,
) -> tuple[Drone, ...]:
    """n drones in sector A, j of them leaders.

    `spacing`: minimum horizontal distance between two drones.
    `clearance`: minimum distance from any wall or obstacle.
    `area_per_drone`: m^2 of floor per drone, the starting density (~1.2 m between
    neighbours at 2); denser only when A is too small for n.
    `link`: two drones closer than this (horizontally) are in the same group; every drone
    must be linked to the others through such pairs. Below the perception radius (2.5 m)
    with room for A's altitude band, which counts twice (boids' vertical_scale).
    """
    if n < 1:
        raise SpawnError(f"need at least one drone, got n = {n}")
    if not 0 <= j <= n:
        raise SpawnError(f"the leaders must be between 0 and n = {n}, got j = {j}")
    low, high = _altitude_band(course)
    area = course.sector_a.area

    free = box(area.x_min, area.y_min, area.x_max, area.y_max).difference(
        _obstacles(course).buffer(clearance)
    )
    patch = _patch(free, Point(area.center), n * area_per_drone)
    rng = random.Random(seed)
    for _ in range(SWARM_ATTEMPTS):
        spots = _draw(patch, n, spacing, rng)
        if len(spots) < n and patch.equals(free):
            raise SpawnError(
                f"could only place {len(spots)} of {n} drones in sector A with {spacing} m "
                f"spacing and {clearance} m clearance; lower n or the spacing, or enlarge A"
            )
        if len(spots) == n and _connected(spots, link):
            break
    else:
        raise SpawnError(
            f"no draw of {n} drones in sector A made a single group with {link} m links "
            f"in {SWARM_ATTEMPTS} attempts; lower area_per_drone or raise link"
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


def _patch(free: BaseGeometry, centre: Point, target: float) -> BaseGeometry:
    """The part of `free` within a disk around `centre`, grown until it covers `target` m^2,
    or all of `free` if it is smaller."""
    if free.area <= target:
        return free
    radius = math.sqrt(target / math.pi)
    while (patch := free.intersection(centre.buffer(radius))).area < target:
        radius += PATCH_GROWTH
    return patch


def _draw(patch: BaseGeometry, n: int, spacing: float, rng: random.Random) -> list[Point2]:
    """Up to n random spots in `patch`, at least `spacing` apart."""
    x_min, y_min, x_max, y_max = patch.bounds
    spots: list[Point2] = []
    for _ in range(n * ATTEMPTS_PER_DRONE):
        if len(spots) == n:
            break
        x, y = rng.uniform(x_min, x_max), rng.uniform(y_min, y_max)
        if not patch.contains(Point(x, y)):
            continue
        if any(math.hypot(x - sx, y - sy) < spacing for sx, sy in spots):
            continue
        spots.append((x, y))
    return spots


def _connected(spots: list[Point2], link: float) -> bool:
    """Whether every spot is linked to the first through spots closer than `link`."""
    reached, frontier = {0}, [0]
    while frontier:
        i = frontier.pop()
        for k, (x, y) in enumerate(spots):
            if k not in reached and math.hypot(x - spots[i][0], y - spots[i][1]) < link:
                reached.add(k)
                frontier.append(k)
    return len(reached) == len(spots)


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
