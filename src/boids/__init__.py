"""Reynolds' boids: pure maths on neighbours' offsets and velocities. No Webots, no numpy."""

from .zone import ellipsoidal_distance, within

__all__ = ["ellipsoidal_distance", "within"]
