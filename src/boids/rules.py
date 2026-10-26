"""The flocking rules: one wanted velocity (world frame) from what a drone hears.

Vásárhelyi et al. (2018), the model of the authors' simulator (csviragh/
robotsim), plus Reynolds' cohesion and Couzin's informed leaders. Each term
is a velocity; their sum is capped at `max_speed`, then limited so that it
never closes on a wall or a neighbour faster than the drone could still
stop, and the flight layer turns it into an acceleration
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
  the flock's (Couzin 2005), slowing down near the goal;
- the stopping limit, last: toward each wall, and each neighbour relative
  to its velocity, the wanted velocity may close only at the braking
  curve of the distance left (minus a margin). Not a push: it only takes
  speed away, so nothing bounces off a wall, and it bounds the sum, so
  no cruise can push a drone into a wall or another drone. Walls win
  where the two disagree. Ours: Vásárhelyi's shill agents push a drone
  away, which in a forest or at a gap makes the flock bounce back, and
  behind a stopped front a cruising drone outpushes repulsion (design
  log, step 4).

Walls are the one thing a drone knows from the map rather than hears: it
gets them as offsets to their nearest points (obstacles.py, map + GPS).

Every speed and rate in BoidParams is in units of v_flock (params.py), so
each term multiplies by it here: speeds and rates once, decelerations
twice.

Nothing keeps a velocity by itself: a drone with no reason to move brakes
to a stop within ~tau (the flight layer). Cruising is what keeps a flock
moving.
"""

import math

from flight.sensing import Neighbour
from util import Vec3

from .params import BoidParams
from .zone import ellipsoidal_distance, within

LIMIT_ROUNDS = 3  # enough for the few limits that bind at once (a corner, a crowd)


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


def braking_curve(distance: float, slope: float, deceleration: float) -> float:
    """The highest speed from which a drone still stops within `distance`: v^2 = 2 a d far
    away, a gentler straight line (slope * distance) over the last stretch, 0 at or below 0.
    Vásárhelyi et al. (2018), robotsim's VelDecayLinSqrt."""
    if distance <= 0 or slope <= 0 or deceleration <= 0:
        return 0.0
    linear = slope * distance
    if linear < deceleration / slope:
        return linear
    return math.sqrt(2 * deceleration * distance - (deceleration / slope) ** 2)


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


def stopping_limit(
    wanted: Vec3, neighbours: list[Neighbour], to_walls: list[Vec3], p: BoidParams
) -> Vec3:
    """`wanted`, its horizontal speed toward each neighbour (beyond the neighbour's own) and
    each wall cut down to the braking curve of the distance left, less stop_drone or
    stop_wall. Each limit is a half-plane of allowed velocities; a few rounds of projecting
    onto them in turn find a velocity in all of them, and the walls get the last rounds.
    Flat: walls are full height until phase 4."""
    drones = [
        (n.offset.horizontal(), ellipsoidal_distance(n.offset, p.vertical_scale), n.velocity,
         p.stop_drone)
        for n in neighbours
    ]  # fmt: skip
    walls = [(w.horizontal(), w.horizontal().length(), Vec3(), p.stop_wall) for w in to_walls]
    for limits in (drones + walls, walls):
        for _ in range(LIMIT_ROUNDS):
            for direction, distance, moving, margin in limits:
                if direction.length() == 0:
                    continue  # right above me, or I am on a wall: no way to tell which way
                toward = direction.unit()
                allowed = moving.dot(toward) + braking_curve(
                    distance - margin, p.stop_slope * p.v_flock, p.stop_braking * p.v_flock**2
                )
                excess = wanted.dot(toward) - allowed
                if excess > 0:
                    wanted = wanted - toward * excess
    return wanted


def cohesion(neighbours: list[Neighbour]) -> Vec3:
    """From me to the neighbours' centre; zero when alone."""
    return Vec3.mean(n.offset for n in neighbours)
