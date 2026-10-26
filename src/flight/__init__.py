"""The Crazyflie side: sensing, takeoff and hover, and turning wanted velocities into motor speeds.

`crazyflie` is the only module that imports Webots; import it directly
(`from flight.crazyflie import Crazyflie`) inside controllers only.
"""

from .autopilot import Autopilot, Phase
from .control import Attitude, G, MotorMixer, acceleration_for, tilt_for
from .params import FlightParams

__all__ = [
    "G",
    "Attitude",
    "Autopilot",
    "FlightParams",
    "MotorMixer",
    "Phase",
    "acceleration_for",
    "tilt_for",
]
