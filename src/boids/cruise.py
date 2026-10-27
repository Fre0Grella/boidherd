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

When a drone may cruise at all (the mission clock, settling in C) is
mission.py's.
"""

from flight.sensing import Neighbour
from util import Vec3

from .mission import Mission
from .params import BoidParams
from .zone import within


class Cruise:
    def __init__(self, p: BoidParams, leader: bool = False) -> None:
        self.p = p
        self.leader = leader
        self.heading: Vec3 | None = None  # None: asleep
        self.mission = Mission(p)

    def update(
        self, velocity: Vec3, neighbours: list[Neighbour], now: float, go: bool, in_c: bool
    ) -> Vec3 | None:
        """The heading to cruise along this step (unit, horizontal), or None. `go`: the
        mission has started; `in_c`: I recognise C under me."""
        if not self.mission.active(now, go, in_c):
            self.heading = None
            return None
        p = self.p
        seen = [
            n.velocity
            for n in neighbours
            if within(n.offset, p.perception_radius, p.vertical_scale)
        ]
        wake = p.wake_speed * p.v_flock
        moving = [v.horizontal() for v in seen if v.horizontal().length() >= wake]
        if not self.leader and velocity.horizontal().length() >= wake:
            moving.append(velocity.horizontal())
        mean = Vec3.sum(v.unit() for v in moving)
        if mean.length() > 0:
            self.heading = mean.unit()
        elif self.leader:
            self.heading = None
        return self.heading
