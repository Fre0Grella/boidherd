import math

import pytest

from util import ZERO, Vec3


def test_arithmetic():
    a, b = Vec3(1, 2, 3), Vec3(4, 5, 6)
    assert a + b == Vec3(5, 7, 9)
    assert b - a == Vec3(3, 3, 3)
    assert a * 2 == 2 * a == Vec3(2, 4, 6)
    assert b / 2 == Vec3(2, 2.5, 3)
    assert -a == Vec3(-1, -2, -3)
    assert a.dot(b) == 32


def test_length_and_unit():
    v = Vec3(3, 4, 0)
    assert v.length() == 5
    assert v.unit() == Vec3(0.6, 0.8, 0)
    assert ZERO.unit() == ZERO


def test_clamped_keeps_direction():
    assert Vec3(3, 4, 0).clamped(1).length() == pytest.approx(1)
    c = Vec3(3, 4, 0).clamped(1)
    assert (c.x, c.y, c.z) == pytest.approx((0.6, 0.8, 0))
    assert Vec3(0.3, 0.4, 0).clamped(1) == Vec3(0.3, 0.4, 0)


def test_horizontal_and_scaled_z():
    v = Vec3(1, 2, 3)
    assert v.horizontal() == Vec3(1, 2, 0)
    assert v.scaled_z(0.5) == Vec3(1, 2, 1.5)


def test_sum_and_mean():
    vs = [Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(-1, -1, 3)]
    assert Vec3.sum(vs) == Vec3(0, 0, 3)
    assert Vec3.mean(vs) == Vec3(0, 0, 1)
    assert Vec3.mean([]) == ZERO


def test_is_immutable():
    v = Vec3(1, 2, 3)
    with pytest.raises(AttributeError):
        v.x = 5  # type: ignore[misc]
    assert math.isclose(v.x, 1)
