"""Ellipsoidal distance: how far a neighbour is, counting height differently.

A sphere of radius r becomes an ellipsoid r wide and r / vertical_scale
tall. With vertical_scale > 1 a neighbour above or below counts as closer
than one at the same distance sideways: drones disturb each other more
vertically (propeller downwash, which Webots doesn't simulate, is the real
reason) and flocks are flatter than they are wide.

One helper serves both zones of the Reynolds rules: who counts as a
neighbour at all, and who is too close (separation).
"""

import math

from util import Vec3


def ellipsoidal_distance(offset: Vec3, vertical_scale: float) -> float:
    return math.sqrt(offset.x**2 + offset.y**2 + (offset.z * vertical_scale) ** 2)


def within(offset: Vec3, radius: float, vertical_scale: float) -> bool:
    return ellipsoidal_distance(offset, vertical_scale) <= radius
