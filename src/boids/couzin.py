"""Couzin et al. (2005): every drone picks a direction, and flies it at v_flock.

The model decides only where to go; how fast is safe stays with the
stopping limit (limit.py) and when to go with the mission (mission.py). A
drone keeps its direction as its own state, like Couzin's agents, so a
wall that stops it or an overshoot of the flight controller doesn't change
where it wants to go. Each step, among the neighbours it sees:

- repulsion first, and alone: if any is closer than `couzin_repulsion`, away
  from all of those, d = -sum unit(offset);
- otherwise attraction and alignment, each neighbour weighing the same:
  d = sum unit(offset) + sum unit(velocity of those moving) + my direction;
- a leader then mixes in the way to the goal, d = unit(d) + couzin_weight x
  unit(to goal), and slows down within arrive_radius so it stops there.

A follower has no direction until it sees a neighbour moving: Couzin's
agents start with random ones, which would scatter the swarm at the start,
so ours wait to be woken by their leaders (design log). Couzin's other
parts are left to the drone: its turning rate is the flight layer's, and
real sensors and air bring their own noise.
"""

from flight.sensing import Neighbour
from util import Vec3

from .limit import stopping_limit
from .mission import Mission
from .params import BoidParams
from .zone import ellipsoidal_distance, within


class Couzin:
    def __init__(self, p: BoidParams, leader: bool = False) -> None:
        self.p = p
        self.leader = leader
        self.direction: Vec3 | None = None  # unit, horizontal; None: no direction yet
        self.mission = Mission(p)

    def wanted(
        self,
        velocity: Vec3,
        neighbours: list[Neighbour],
        now: float,
        go: bool,
        in_c: bool,
        to_goal: Vec3 | None,
        to_walls: list[Vec3],
    ) -> Vec3:
        """The wanted velocity (m/s, world frame): v_flock along my direction, through the
        stopping limit. `to_goal`, for a leader on its way: from the drone to its goal."""
        p = self.p
        seen = [n for n in neighbours if within(n.offset, p.perception_radius, p.vertical_scale)]
        wanted = Vec3()
        if self.mission.active(now, go, in_c):
            self.direction = self.decide(seen, to_goal)
            if self.direction is not None:
                speed = p.v_flock
                if to_goal is not None:
                    speed *= min(1.0, to_goal.length() / p.arrive_radius)
                wanted = self.direction * speed
        else:
            self.direction = None
        return stopping_limit(wanted, seen, to_walls, p)

    def decide(self, seen: list[Neighbour], to_goal: Vec3 | None) -> Vec3 | None:
        """My new direction (unit, horizontal), or None if I have none yet."""
        p = self.p
        close = [
            n for n in seen if ellipsoidal_distance(n.offset, p.vertical_scale) < p.couzin_repulsion
        ]
        if close:
            social = -Vec3.sum(n.offset.horizontal().unit() for n in close)
        else:
            wake = p.wake_speed * p.v_flock
            moving = [n for n in seen if n.velocity.horizontal().length() >= wake]
            if self.direction is None and not moving and to_goal is None:
                return None  # asleep: nobody near me has set off yet
            social = (
                Vec3.sum(n.offset.horizontal().unit() for n in seen)
                + Vec3.sum(n.velocity.horizontal().unit() for n in moving)
                + (self.direction or Vec3())
            )
        direction = social.horizontal().unit()
        if to_goal is not None:
            direction = (direction + to_goal.horizontal().unit() * p.couzin_weight).unit()
        return direction if direction.length() > 0 else self.direction
