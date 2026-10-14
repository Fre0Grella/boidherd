"""From raw IR receptions to neighbours in the world frame. Pure maths, no Webots.

Webots' IR receiver gives, for each packet, the direction it came from (a
unit vector in the receiver's frame, which on the Crazyflie is the body
frame) and the signal strength, 1 / distance^2. The direction is rotated
into the world frame with the drone's roll, pitch and yaw: the drone tilts
whenever it accelerates, so the body frame is rarely level.
"""

import math
from dataclasses import dataclass

from util import Vec3

from .control import Attitude
from .radio import decode


@dataclass(frozen=True)
class Reception:
    """One packet as the receiver saw it."""

    text: str
    direction: Vec3  # unit vector towards the sender, body frame
    strength: float  # 1 / distance^2


@dataclass(frozen=True)
class Neighbour:
    id: int
    offset: Vec3  # where it is, relative to me, world frame, metres
    velocity: Vec3  # how it moves, world frame, m/s


def neighbours(me: int, receptions: list[Reception], attitude: Attitude) -> list[Neighbour]:
    """Every other drone heard this step; my own packets and stray messages are dropped."""
    found: dict[int, Neighbour] = {}
    for r in receptions:
        packet = decode(r.text)
        if packet is None or packet.sender == me or r.strength <= 0:
            continue
        distance = 1.0 / math.sqrt(r.strength)
        offset = body_to_world(r.direction.unit(), attitude) * distance
        found[packet.sender] = Neighbour(packet.sender, offset, packet.velocity)
    return list(found.values())


def body_to_world(v: Vec3, a: Attitude) -> Vec3:
    """Rotate a body-frame vector into the world frame: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(a.roll), math.sin(a.roll)
    cp, sp = math.cos(a.pitch), math.sin(a.pitch)
    cy, sy = math.cos(a.yaw), math.sin(a.yaw)
    x1, y1, z1 = v.x, cr * v.y - sr * v.z, sr * v.y + cr * v.z  # roll about x
    x2, z2 = cp * x1 + sp * z1, -sp * x1 + cp * z1  # pitch about y
    return Vec3(cy * x2 - sy * y1, sy * x2 + cy * y1, z2)  # yaw about z
