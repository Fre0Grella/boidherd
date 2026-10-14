"""The pillar forest generator: `count` random pillars in a region.

Pillars are placed by rejection sampling with the forest's own seed, so a
map always grows the same forest. Each pillar keeps `min_gap` of free space
(edge to edge) from the other pillars and from everything in `blocked`.
"""

import math
import random

from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry

from .load import MapError
from .model import Forest, Pillar

ATTEMPTS_PER_PILLAR = 500


def grow_forest(
    forest: Forest, blocked: BaseGeometry, neighbours: list[Pillar], path: str
) -> list[Pillar]:
    """Place the forest's pillars clear of `blocked` and of the `neighbours` already placed."""
    region = forest.region
    if min(region.width, region.depth) < 2 * forest.radius[1]:
        raise MapError(f"{path}.region: narrower than the largest pillar")
    rng = random.Random(forest.seed)
    grown: list[Pillar] = []
    for _ in range(forest.count * ATTEMPTS_PER_PILLAR):
        if len(grown) == forest.count:
            break
        r = rng.uniform(*forest.radius)
        h = rng.uniform(*forest.height) if forest.height else None
        x = rng.uniform(region.x_min + r, region.x_max - r)
        y = rng.uniform(region.y_min + r, region.y_max - r)
        if Point(x, y).distance(blocked) < r + forest.min_gap:
            continue
        if any(_gap(x, y, r, p) < forest.min_gap for p in [*neighbours, *grown]):
            continue
        grown.append(Pillar(center=(x, y), radius=r, height=h))
    if len(grown) < forest.count:
        raise MapError(
            f"{path}: could only fit {len(grown)} of {forest.count} pillars with "
            f"min_gap {forest.min_gap}; lower count or min_gap, or enlarge the region"
        )
    return grown


def _gap(x: float, y: float, r: float, other: Pillar) -> float:
    """Free space between a pillar at (x, y) of radius r and `other`."""
    ox, oy = other.center
    return math.hypot(x - ox, y - oy) - r - other.radius
