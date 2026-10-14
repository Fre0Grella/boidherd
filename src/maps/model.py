"""What a map file says, as plain immutable data.

These classes mirror the TOML format one to one (see maps/*.toml and
docs/design-log.md). Generators such as forests are kept as they were
written; turning them into concrete pillars is the job of the course
builder, not of this module. Lengths are in metres, z is up.
"""

from dataclasses import dataclass

Interval = tuple[float, float]  # (min, max), min <= max
Point = tuple[float, float]  # (x, y)


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle; the loader accepts its corners in any order."""

    x_min: float
    y_min: float
    x_max: float
    y_max: float

    @property
    def width(self) -> float:
        return self.x_max - self.x_min

    @property
    def depth(self) -> float:
        return self.y_max - self.y_min

    @property
    def center(self) -> Point:
        return ((self.x_min + self.x_max) / 2, (self.y_min + self.y_max) / 2)


@dataclass(frozen=True)
class Arena:
    ceiling: float  # flight ceiling; walls and full-height obstacles reach it
    rects: tuple[Rect, ...]  # the free space is their union


@dataclass(frozen=True)
class Sector:
    area: Rect
    z: Interval  # altitude band; a map without z gives floor to ceiling


@dataclass(frozen=True)
class Wall:
    rect: Rect
    height: float | None  # None: as tall as the ceiling


@dataclass(frozen=True)
class Pillar:
    center: Point
    radius: float
    height: float | None  # None: as tall as the ceiling


@dataclass(frozen=True)
class Forest:
    """Generator: `count` pillars placed at random in `region`."""

    region: Rect
    count: int
    radius: Interval  # each pillar draws its radius from here
    min_gap: float  # free space between pillar edges (and walls)
    seed: int  # fixes the layout for this map
    height: Interval | None  # None: as tall as the ceiling


@dataclass(frozen=True)
class Map:
    description: str
    arena: Arena
    sector_a: Sector  # spawn region
    sector_c: Sector  # goal, known only to the leaders
    walls: tuple[Wall, ...]
    pillars: tuple[Pillar, ...]
    forests: tuple[Forest, ...]
