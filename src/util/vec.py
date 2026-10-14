"""A small 3D vector, so steering and flight code reads like the maths it implements.

Immutable and dependency-free: the Reynolds rules use it without importing
numpy or Webots. Units are whatever the caller uses (metres, m/s, m/s^2).
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def __add__(self, other: Vec3) -> Vec3:
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: Vec3) -> Vec3:
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, k: float) -> Vec3:
        return Vec3(self.x * k, self.y * k, self.z * k)

    __rmul__ = __mul__

    def __truediv__(self, k: float) -> Vec3:
        return Vec3(self.x / k, self.y / k, self.z / k)

    def __neg__(self) -> Vec3:
        return Vec3(-self.x, -self.y, -self.z)

    def dot(self, other: Vec3) -> float:
        return self.x * other.x + self.y * other.y + self.z * other.z

    def length(self) -> float:
        return math.sqrt(self.dot(self))

    def unit(self) -> Vec3:
        """Same direction, length 1; the zero vector stays zero."""
        n = self.length()
        return self / n if n > 0 else Vec3()

    def clamped(self, max_length: float) -> Vec3:
        """Same direction, length at most `max_length`."""
        n = self.length()
        return self * (max_length / n) if n > max_length else self

    def horizontal(self) -> Vec3:
        """The same vector with z set to 0."""
        return Vec3(self.x, self.y, 0.0)

    def scaled_z(self, k: float) -> Vec3:
        """The same vector with z multiplied by k."""
        return Vec3(self.x, self.y, self.z * k)

    @staticmethod
    def sum(vectors: Iterable[Vec3]) -> Vec3:
        total = Vec3()
        for v in vectors:
            total = total + v
        return total

    @staticmethod
    def mean(vectors: Iterable[Vec3]) -> Vec3:
        """Average of the vectors; the zero vector if there are none."""
        items = list(vectors)
        return Vec3.sum(items) / len(items) if items else Vec3()


ZERO = Vec3()
