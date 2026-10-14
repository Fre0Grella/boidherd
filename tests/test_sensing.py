import math

import pytest

from boids import ellipsoidal_distance, within
from flight import Attitude
from flight.radio import Packet, decode, encode
from flight.sensing import Reception, body_to_world, neighbours
from util import Vec3


def level(yaw: float = 0.0) -> Attitude:
    return Attitude(roll=0.0, pitch=0.0, yaw=yaw, yaw_rate=0.0, altitude=1.0)


def close(a: Vec3, b: Vec3) -> bool:
    return (a - b).length() < 1e-9


# --- packets -----------------------------------------------------------------


def test_packet_round_trip():
    assert decode(encode(7, Vec3(0.5, -1.25, 0.0))) == Packet(7, Vec3(0.5, -1.25, 0.0))


@pytest.mark.parametrize("text", ["", "hello", "1;2;3", "a;1;2;3", "1;x;2;3"])
def test_stray_messages_are_ignored(text: str):
    assert decode(text) is None


# --- body to world -----------------------------------------------------------


def test_level_and_facing_x_changes_nothing():
    assert close(body_to_world(Vec3(1, 2, 3), level()), Vec3(1, 2, 3))


def test_yaw_turns_about_z():
    assert close(body_to_world(Vec3(1, 0, 0), level(yaw=math.pi / 2)), Vec3(0, 1, 0))


def test_pitch_nose_down_tips_forward_down():
    a = Attitude(roll=0.0, pitch=0.3, yaw=0.0, yaw_rate=0.0, altitude=1.0)
    v = body_to_world(Vec3(1, 0, 0), a)  # the nose points ahead and down
    assert v.x == pytest.approx(math.cos(0.3)) and v.z == pytest.approx(-math.sin(0.3))


def test_roll_tips_the_left_side_up():
    a = Attitude(roll=0.3, pitch=0.0, yaw=0.0, yaw_rate=0.0, altitude=1.0)
    v = body_to_world(Vec3(0, 1, 0), a)
    assert v.y == pytest.approx(math.cos(0.3)) and v.z == pytest.approx(math.sin(0.3))


def test_rotation_keeps_lengths():
    a = Attitude(roll=0.2, pitch=-0.4, yaw=2.0, yaw_rate=0.0, altitude=1.0)
    assert body_to_world(Vec3(1, 2, 3), a).length() == pytest.approx(Vec3(1, 2, 3).length())


# --- neighbours --------------------------------------------------------------


def test_offset_from_direction_and_strength():
    # 2 m straight ahead of a drone facing +y: the neighbour is at world +y
    r = Reception(encode(4, Vec3(1, 0, 0)), direction=Vec3(1, 0, 0), strength=1 / 4)
    [n] = neighbours(0, [r], level(yaw=math.pi / 2))
    assert n.id == 4 and close(n.offset, Vec3(0, 2, 0)) and n.velocity == Vec3(1, 0, 0)


def test_own_and_stray_packets_are_dropped():
    receptions = [
        Reception(encode(0, Vec3()), Vec3(1, 0, 0), 1.0),  # my own
        Reception("noise", Vec3(1, 0, 0), 1.0),
        Reception(encode(5, Vec3()), Vec3(0, 1, 0), 1.0),
    ]
    assert [n.id for n in neighbours(0, receptions, level())] == [5]


def test_one_neighbour_per_sender():
    receptions = [Reception(encode(3, Vec3()), Vec3(1, 0, 0), 1.0)] * 2
    assert len(neighbours(0, receptions, level())) == 1


# --- ellipsoidal zone --------------------------------------------------------


def test_vertical_scale_stretches_height():
    assert ellipsoidal_distance(Vec3(3, 4, 0), 2.0) == 5
    assert ellipsoidal_distance(Vec3(0, 0, 1), 2.0) == 2
    assert within(Vec3(0, 0, 0.9), radius=2.0, vertical_scale=2.0)
    assert not within(Vec3(0, 0, 1.1), radius=2.0, vertical_scale=2.0)
    assert within(Vec3(1.9, 0, 0), radius=2.0, vertical_scale=2.0)


def test_scale_one_is_a_sphere():
    assert ellipsoidal_distance(Vec3(1, 2, 2), 1.0) == 3
