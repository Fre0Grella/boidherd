"""The IR packet every drone broadcasts each step: who it is and how it moves.

    "<id>;<vx>;<vy>;<vz>"     world-frame velocity, m/s, millimetre/s precision

Velocities are in the world frame, which every drone shares through its IMU
heading, so a neighbour's velocity means the same thing to everyone (a
quadrotor's yaw says nothing about where it is going). Where the sender is
comes from the IR signal itself, not from the packet.
"""

from dataclasses import dataclass

from util import Vec3


@dataclass(frozen=True)
class Packet:
    sender: int
    velocity: Vec3


def encode(sender: int, velocity: Vec3) -> str:
    return f"{sender};{velocity.x:.3f};{velocity.y:.3f};{velocity.z:.3f}"


def decode(text: str) -> Packet | None:
    """The packet, or None if the text isn't one (never crash on a stray message)."""
    try:
        sender, vx, vy, vz = text.split(";")
        return Packet(int(sender), Vec3(float(vx), float(vy), float(vz)))
    except ValueError:
        return None
