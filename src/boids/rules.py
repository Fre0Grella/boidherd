"""Reynolds' three rules, plus drag and a speed limit: one world-frame acceleration.

Each rule only sees what IR tells a drone about its neighbours (where they
are relative to it, and how they move) and the drone's own velocity; a
follower never needs its absolute position.

- separation: away from neighbours that are too close, harder the closer
  they are (zero at the edge of the zone, so it doesn't switch on abruptly);
- alignment: toward the neighbours' mean velocity;
- cohesion: toward the neighbours' centre;
- drag: a little against the drone's own velocity. The three rules only
  damp motion *relative* to the neighbours, so without it a leaderless
  flock keeps whatever drift it has until it reaches a wall.

The sum is weighted (Reynolds 1987 also describes a prioritised sum; not
needed yet), its vertical part scaled by `vertical_gain`, then capped.
"""

from flight.sensing import Neighbour
from util import Vec3

from .params import BoidParams
from .zone import ellipsoidal_distance, within


def flock(velocity: Vec3, neighbours: list[Neighbour], p: BoidParams) -> Vec3:
    """The acceleration (m/s^2, world frame) the rules ask for."""
    seen = [n for n in neighbours if within(n.offset, p.perception_radius, p.vertical_scale)]
    total = (
        separation(seen, p)
        + alignment(velocity, seen) * p.alignment
        + cohesion(seen) * p.cohesion
        - velocity * p.drag
        + speed_limit(velocity, p)
    )
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


def speed_limit(velocity: Vec3, p: BoidParams) -> Vec3:
    excess = velocity.length() - p.max_speed
    return -velocity.unit() * (p.speed_brake * excess) if excess > 0 else Vec3()
