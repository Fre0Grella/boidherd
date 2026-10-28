"""A leader's way to C: plan, follow, and plan again when a wall cuts it off.

The leader plans at go time from where it is, then follows the path by
pure pursuit. The flock can push it off the path, behind a wall the path
goes round: the point it aims at is then on the other side of that wall,
and aiming straight at it presses the leader into the wall for good (the
cup of `dead_end`, design log 2026-10-28). So whenever a planned-round
obstacle stands between the leader and that point, it plans again from
where it is, at most every `replan_every` seconds.
"""

import math

from util import Vec3

from .params import PlanParams
from .plan import Planner, Point
from .pursuit import Pursuit


class Route:
    def __init__(self, planner: Planner, goal: Point, p: PlanParams) -> None:
        self.planner, self.goal, self.p = planner, goal, p
        self.pursuit: Pursuit | None = None
        self.planned_at = -math.inf
        self.plans = 0  # how many times it has planned, the first included

    def to_goal(self, position: Vec3, now: float) -> Vec3 | None:
        """Toward the lookahead point, as Pursuit.to_goal; None while there is no path."""
        here = (position.x, position.y)
        if self.pursuit is not None:
            aim = self.pursuit.to_goal(position)
            if self.planner.sees(here, self.pursuit.target) or not self._plan(here, now):
                return aim
        elif not self._plan(here, now):
            return None
        assert self.pursuit is not None  # _plan just made it
        return self.pursuit.to_goal(position)

    def _plan(self, here: Point, now: float) -> bool:
        """Plan from `here` unless it planned too recently; whether there's a new path.
        With no path, the old one (if any) stays."""
        if now - self.planned_at < self.p.replan_every:
            return False
        self.planned_at = now
        path = self.planner.plan(here, self.goal)
        if path is None:
            return False
        self.pursuit = Pursuit(path, self.p.lookahead)
        self.plans += 1
        return True
