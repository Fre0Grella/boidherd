"""Boid settings: every number the Reynolds rules use, in one place.

Change a default here, or build `BoidParams(...)` with other values for a
run; nothing else in the boids code hard-codes these.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class BoidParams:
    # zones (ellipsoidal: see zone.py); the IR range (3 m) caps perception anyway
    perception_radius: float = 2.5  # m, who counts as a neighbour
    separation_radius: float = 1.0  # m, who is too close
    vertical_scale: float = 2.0  # a neighbour above or below counts as this x closer
    # rule weights
    separation: float = 1.0  # m/s^2 at half the separation radius, more when closer
    alignment: float = 1.0  # 1/s: m/s^2 per m/s of difference with the neighbours' mean
    cohesion: float = 0.3  # 1/s^2: m/s^2 per metre to the neighbours' centre
    # leaders only: the pull toward the goal (seek, then arrive)
    goal_speed: float = 0.5  # m/s, the cruise speed a leader aims for
    goal_weight: float = 1.0  # 1/s: m/s^2 per m/s between its velocity and the wanted one
    arrive_radius: float = 3.0  # m, within it the wanted speed shrinks to stop at the goal
    goal_after: float = 10.0  # s of hovering before leaders set off: the flock forms first
    # limits
    max_speed: float = 1.0  # m/s, above it the drone brakes
    speed_brake: float = 2.0  # 1/s: m/s^2 per m/s over max_speed
    max_acceleration: float = 2.5  # m/s^2, below g tan(max_tilt) = 5.7 so tilt keeps direction
    # 0: fixed altitude (z of the result dropped); 1: full 3D (phase 4)
    vertical_gain: float = 0.0
