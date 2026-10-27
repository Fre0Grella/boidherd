"""Following a planned path: pure pursuit.

The leader aims at the point `lookahead` metres ahead along its path, from
the point of the path nearest to it, wherever the flock has pushed it, so
it never needs to replan. Progress along the path only moves forward, and
the nearest point is looked for a little way ahead of it, so a path that
doubles back near itself can't make the leader skip a stretch.

The result is a `to_goal` the flocking models take as it is: it points at
the lookahead point, and its length is the distance still to go along the
path, so a leader slows down and stops at the end as it did aiming straight
at C (boids: arrive_radius).
"""

import math

from util import Vec3

from .plan import Point

SEARCH_AHEAD = 3.0  # x lookahead: how far ahead of the progress the nearest point may lie


class Pursuit:
    def __init__(self, path: list[Point], lookahead: float) -> None:
        self.path = path
        self.lookahead = lookahead
        self.segments = list(zip(path, path[1:], strict=False))
        self.lengths = [math.dist(a, b) for a, b in self.segments]
        self.total = sum(self.lengths)
        self.progress = 0.0  # m along the path

    def to_goal(self, position: Vec3) -> Vec3:
        """From `position` toward the lookahead point, as long as the way still to go."""
        here = (position.x, position.y)
        self.progress = self._nearest(here)
        s = min(self.total, self.progress + self.lookahead)
        target = self.point_at(s)
        aim = Vec3(target[0] - here[0], target[1] - here[1], 0)
        remaining = aim.length() + (self.total - s)
        return aim.unit() * remaining if aim.length() > 0 else Vec3()

    def point_at(self, s: float) -> Point:
        """The point `s` metres along the path (clamped to its ends)."""
        for (a, b), length in zip(self.segments, self.lengths, strict=True):
            if s <= length:
                t = s / length if length > 0 else 0.0
                return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            s -= length
        return self.path[-1]

    def _nearest(self, q: Point) -> float:
        """Arc length of the path point nearest to q, at or after the progress so far and at
        most SEARCH_AHEAD lookaheads beyond it."""
        best, best_s, start = math.inf, self.progress, 0.0
        window = self.progress + SEARCH_AHEAD * self.lookahead
        for (a, b), length in zip(self.segments, self.lengths, strict=True):
            end = start + length
            if end >= self.progress and start <= window:
                t = _projection(q, a, b)
                s = min(max(start + t * length, self.progress), window)
                d = math.dist(q, self.point_at(s))
                if d < best:
                    best, best_s = d, s
            start = end
        return best_s


def _projection(q: Point, a: Point, b: Point) -> float:
    """Where q falls along segment a-b, 0 at a and 1 at b, clamped."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return 0.0
    return min(1.0, max(0.0, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / length2))
