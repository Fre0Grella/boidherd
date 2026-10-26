"""Flocking rules (Vásárhelyi + Reynolds cohesion + Couzin leaders): pure maths. No Webots."""

from .cruise import Cruise
from .params import BoidParams
from .rules import flock
from .zone import ellipsoidal_distance, within

__all__ = ["BoidParams", "Cruise", "ellipsoidal_distance", "flock", "within"]
