"""The stopping limit: what keeps any flocking model safe on a real drone.

Every model ends here. Toward each wall, and toward each neighbour relative
to its velocity, the wanted velocity may close only at the braking curve of
the distance left, minus a margin: never faster than the drone could still
stop. It only takes speed away, so nothing bounces off a wall, and it bounds
whatever the model asked for, so no rule can push a drone into a wall or
another drone. Walls win where the two disagree. Ours, built on Vásárhelyi's
braking curve: his shill agents push a drone away, which in a forest or at a
gap made the flock bounce back (design log, step 4).
"""

import math

from flight.sensing import Neighbour
from util import Vec3

from .params import BoidParams
from .zone import ellipsoidal_distance

LIMIT_ROUNDS = 3  # enough for the few limits that bind at once (a corner, a crowd)


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
