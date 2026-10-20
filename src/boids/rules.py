"""Reynolds' three rules, a speed limit and the leaders' goal: one world-frame acceleration.

Each rule only sees what IR tells a drone about its neighbours (where they
are relative to it, and how they move) and the drone's own velocity; a
follower never needs its absolute position.

- separation: away from neighbours that are too close, harder the closer
  they are (zero at the edge of the zone, so it doesn't switch on abruptly);
- alignment: toward the neighbours' mean velocity;
- cohesion: toward the neighbours' centre;
- goal, leaders only: toward a cruise velocity pointing at the goal, slowing
  down near it (Reynolds 1999: seek, then arrive). The leader works out the
  offset to the goal from its own position; the rules only get the offset.

There is no drag: the rules only damp motion relative to the neighbours, so
a leaderless flock keeps its drift. Kept on purpose: a drifting flock is
likelier to meet a leader than one that stands still (design log).

The sum is weighted (Reynolds 1987 also describes a prioritised sum; not
needed yet), its vertical part scaled by `vertical_gain`, then capped.
"""

from flight.sensing import Neighbour
from util import Vec3

from .params import BoidParams
from .zone import ellipsoidal_distance, within


def flock(
    velocity: Vec3, neighbours: list[Neighbour], p: BoidParams, to_goal: Vec3 | None = None
) -> Vec3:
    """The acceleration (m/s^2, world frame) the rules ask for. `to_goal`, for a leader
    on its way: from the drone to its goal, metres."""
    seen = [n for n in neighbours if within(n.offset, p.perception_radius, p.vertical_scale)]
    total = (
        separation(seen, p)
        + alignment(velocity, seen) * p.alignment
        + cohesion(seen) * p.cohesion
        + speed_limit(velocity, p)
    )
    if to_goal is not None:
        total = total + goal(velocity, to_goal, p)
    return total.scaled_z(p.vertical_gain).clamped(p.max_acceleration)


def separation(neighbours: list[Neighbour], p: BoidParams) -> Vec3:
    push = Vec3()
    for n in neighbours:
        d = ellipsoidal_distance(n.offset, p.vertical_scale)
        if 0 < d < p.separation_radius:
            push = push - n.offset.unit() * (p.separation * (p.separation_radius / d - 1))
    return push


def alignment(velocity: Vec3, neighbours: list[Neighbour]) -> Vec3:
    """Mean velocity of the neighbours minus mine; zero when alone."""
    if not neighbours:
        return Vec3()
    return Vec3.mean(n.velocity for n in neighbours) - velocity


def cohesion(neighbours: list[Neighbour]) -> Vec3:
    """From me to the neighbours' centre; zero when alone."""
    return Vec3.mean(n.offset for n in neighbours)


def goal(velocity: Vec3, to_goal: Vec3, p: BoidParams) -> Vec3:
    """Steer toward `goal_speed` aimed at the goal; within `arrive_radius` the wanted
    speed shrinks with the distance, so the leader stops there instead of overshooting."""
    slow_down = min(1.0, to_goal.length() / p.arrive_radius)
    wanted = to_goal.unit() * (p.goal_speed * slow_down)
    return (wanted - velocity) * p.goal_weight


def speed_limit(velocity: Vec3, p: BoidParams) -> Vec3:
    excess = velocity.length() - p.max_speed
    return -velocity.unit() * (p.speed_brake * excess) if excess > 0 else Vec3()
