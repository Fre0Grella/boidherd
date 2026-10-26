"""Flocking rules (Vásárhelyi + Reynolds cohesion + Couzin leaders): pure maths. No Webots."""

from .cruise import Cruise
from .obstacles import Box, to_walls
from .params import BoidParams
from .rules import flock
from .zone import ellipsoidal_distance, within

__all__ = ["BoidParams", "Box", "Cruise", "ellipsoidal_distance", "flock", "to_walls", "within"]
