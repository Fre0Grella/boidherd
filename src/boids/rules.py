"""The hybrid model's rules: one wanted velocity (world frame) from what a drone hears.

Vásárhelyi et al. (2018), the model of the authors' simulator (csviragh/
robotsim), plus Reynolds' cohesion and Couzin's informed leaders: the model
phase 3 grew into (design log; models.py has the others). Each term is a
velocity; their sum is capped at `max_speed`, then goes through the stopping
limit (limit.py), and the flight layer turns it into an acceleration
(flight.acceleration_for). Each rule only
sees what IR tells a drone about its neighbours (where they are relative to
it, how they move) and its own velocity; a follower never needs its
absolute position.

- self-propulsion: cruise at `v_flock` along a heading, the flock's mean
  direction, once the mission has started and until I settle in C (when
  and which way: cruise.py);
- repulsion: away from neighbours inside `repulsion_radius`, linearly;
- friction: two neighbours may differ in velocity only by what they could
  still brake within their distance (the braking curve), the excess is
  corrected: velocity alignment that is strong up close, slack far away;
- cohesion (Reynolds): toward the neighbours' centre;
- leaders only: cruise toward the goal instead, the direction mixed with
  the flock's (Couzin 2005), slowing down near the goal.

Walls are the one thing a drone knows from the map rather than hears: it
gets them as offsets to their nearest points (obstacles.py, map + GPS).

Every speed and rate in BoidParams is in units of v_flock (params.py), so
each term multiplies by it here: speeds and rates once, decelerations
twice.

Nothing keeps a velocity by itself: a drone with no reason to move brakes
to a stop within ~tau (the flight layer). Cruising is what keeps a flock
moving.
"""

from flight.sensing import Neighbour
from util import Vec3

from .limit import braking_curve, stopping_limit
from .params import BoidParams
from .zone import ellipsoidal_distance, within


def flock(
    velocity: Vec3,
    neighbours: list[Neighbour],
    p: BoidParams,
    heading: Vec3 | None = None,
    to_goal: Vec3 | None = None,
    to_walls: list[Vec3] | None = None,
) -> Vec3:
    """The wanted velocity (m/s, world frame). `heading`: cruise along it (a unit vector,
    see cruise.py). `to_goal`, for a leader on its way: from the drone to its goal, metres
    (then it cruises toward it, mixed with `heading`, instead). `to_walls`: from the drone
    to the nearest point of each wall, metres."""
    seen = [n for n in neighbours if within(n.offset, p.perception_radius, p.vertical_scale)]
    total = repulsion(seen, p) + friction(velocity, seen, p)
    total = total + cohesion(seen) * (p.cohesion * p.v_flock)
    if to_goal is not None:
        total = total + seek(heading or Vec3(), to_goal, p)
    elif heading is not None:
        total = total + heading * p.v_flock
    wanted = total.scaled_z(p.vertical_gain).clamped(p.max_speed * p.v_flock)
    return stopping_limit(wanted, seen, to_walls or [], p)


def seek(heading: Vec3, to_goal: Vec3, p: BoidParams) -> Vec3:
    """A leader's cruise: toward the goal mixed with the flock's heading (a unit vector, or
    zero alone; goal_weight to 1), at v_flock, slowing down within arrive_radius so it stops
    there."""
    direction = (heading + to_goal.unit() * p.goal_weight).unit()
    return direction * (p.v_flock * min(1.0, to_goal.length() / p.arrive_radius))


def repulsion(neighbours: list[Neighbour], p: BoidParams) -> Vec3:
    push = Vec3()
    for n in neighbours:
        d = ellipsoidal_distance(n.offset, p.vertical_scale)
        if 0 < d < p.repulsion_radius:
            gain = p.repulsion_gain * p.v_flock
            push = push - n.offset.unit() * (gain * (p.repulsion_radius - d))
    return push


def friction(velocity: Vec3, neighbours: list[Neighbour], p: BoidParams) -> Vec3:
    """For each neighbour, the part of our velocity difference beyond what the braking
    curve allows at its distance (at least friction_slack), toward its velocity."""
    v = p.v_flock
    total = Vec3()
    for n in neighbours:
        gap = n.velocity - velocity
        allowed = max(
            p.friction_slack * v,
            braking_curve(
                ellipsoidal_distance(n.offset, p.vertical_scale) - p.friction_radius,
                p.friction_slope * v,
                p.friction_braking * v**2,
            ),
        )
        if gap.length() > allowed:
            total = total + gap.unit() * (p.friction_gain * (gap.length() - allowed))
    return total


def cohesion(neighbours: list[Neighbour]) -> Vec3:
    """From me to the neighbours' centre; zero when alone."""
    return Vec3.mean(n.offset for n in neighbours)
