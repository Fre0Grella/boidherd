"""Reynolds' boids: pure maths on neighbours' offsets and velocities. No Webots, no numpy."""

from .params import BoidParams
from .rules import flock
from .zone import ellipsoidal_distance, within

__all__ = ["BoidParams", "ellipsoidal_distance", "flock", "within"]
