"""When a drone cruises, and which way: alignment with the flock (Couzin 2005).

Self-propulsion needs a heading. A follower takes it from its neighbours:
the mean direction of those I see moving (above `wake_speed`), mine
included, as in Couzin's zone of orientation. So a follower has no
direction of its own: one deflected by a wall or a corner turns back to
where the flock goes, a small overshoot doesn't flip it unless its whole
neighbourhood flips, and a few leaders can steer the flock, since the
others only follow each other. Before anyone near me moves I have no
heading: the flock wakes up from its leaders outward. Alone, a follower
keeps the last heading it had.

A leader's heading is the same mean without itself and without memory:
the social half of Couzin's informed individual, which rules.seek mixes
with the way to the goal. Its own momentum would let a leader run away
from a goal it can't reach straight (the L hall: C is behind a wall from
hall 1, design log, step 4).

A drone stops cruising once it has cruised `settle_distance` into C
(timed: that distance at v_flock), not at once: a flock arrives behind its
leaders, and drones stopping right at the edge would pile up there, the
tail left outside (design log, step 2).
"""

from flight.sensing import Neighbour
from util import Vec3

from .params import BoidParams
from .zone import within


class Cruise:
    def __init__(self, p: BoidParams, leader: bool = False) -> None:
        self.p = p
        self.leader = leader
        self.heading: Vec3 | None = None  # None: asleep
        self._entered: float | None = None  # when I last entered C

    def update(
        self, velocity: Vec3, neighbours: list[Neighbour], now: float, go: bool, in_c: bool
    ) -> Vec3 | None:
        """The heading to cruise along this step (unit, horizontal), or None. `go`: the
        mission has started; `in_c`: I recognise C under me."""
        if in_c and self._entered is None:
            self._entered = now
        elif not in_c:
            self._entered = None
        settle_time = self.p.settle_distance / self.p.v_flock
        settled = self._entered is not None and now - self._entered >= settle_time
        if not go or settled:
            self.heading = None
            return None
        p = self.p
        seen = [
            n.velocity
            for n in neighbours
            if within(n.offset, p.perception_radius, p.vertical_scale)
        ]
        moving = [v.horizontal() for v in seen if v.horizontal().length() >= p.wake_speed]
        if not self.leader and velocity.horizontal().length() >= p.wake_speed:
            moving.append(velocity.horizontal())
        mean = Vec3.sum(v.unit() for v in moving)
        if mean.length() > 0:
            self.heading = mean.unit()
        elif self.leader:
            self.heading = None
        return self.heading
