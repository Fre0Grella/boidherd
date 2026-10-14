"""From a wanted acceleration to the four motor speeds. Pure maths, no Webots.

A quadrotor accelerates sideways by tilting: the thrust that holds it up
then also pushes it horizontally, with a ~ g * tan(tilt). So a world-frame
acceleration is rotated into the drone's body frame (it can face any way)
and turned into a pitch (forward) and a roll (sideways). An attitude PID
then tilts the drone, while an altitude PID keeps it at its height.

The PID structure and gains come from Webots' Crazyflie example
(projects/robots/bitcraze/crazyflie/controllers/crazyflie_py_wallfollowing/
pid_controller.py, Apache 2.0), minus its velocity loop: the boids command
accelerations, so the drone's momentum is left to the physics.
"""

import math
from dataclasses import dataclass

from util import Vec3

from .params import FlightParams

G = 9.81  # m/s^2


@dataclass(frozen=True)
class Attitude:
    """What the drone measures about itself: angles in rad, rates in rad/s, metres."""

    roll: float
    pitch: float
    yaw: float
    yaw_rate: float
    altitude: float


def tilt_for(acceleration: Vec3, yaw: float, max_tilt: float) -> tuple[float, float]:
    """The (roll, pitch) that gives a horizontal world-frame acceleration.

    Signs as in the Crazyflie example: positive pitch accelerates forward
    (+x of the body), positive roll accelerates to the right (-y of the body).
    """
    c, s = math.cos(yaw), math.sin(yaw)
    forward = acceleration.x * c + acceleration.y * s  # world -> body frame
    left = -acceleration.x * s + acceleration.y * c
    pitch = math.atan2(forward, G)
    roll = -math.atan2(left, G)
    return _clip(roll, max_tilt), _clip(pitch, max_tilt)


class MotorMixer:
    """Altitude and attitude PIDs plus motor mixing; keeps the PIDs' memory between steps."""

    def __init__(self, params: FlightParams) -> None:
        self.p = params
        self._altitude_integral = 0.0
        self._last: tuple[float, float, float] | None = None  # altitude, roll, pitch errors

    def motors(
        self, dt: float, roll: float, pitch: float, altitude: float, now: Attitude
    ) -> tuple[float, float, float, float]:
        """Motor speeds (m1..m4) for the wanted roll, pitch and altitude, holding yaw still."""
        p = self.p
        errors = (altitude - now.altitude, roll - now.roll, pitch - now.pitch)
        last = self._last if self._last is not None and dt > 0 else errors
        rates = [(e - e0) / dt if dt > 0 else 0.0 for e, e0 in zip(errors, last, strict=True)]
        self._last = errors
        alt_err, roll_err, pitch_err = errors
        alt_rate, roll_rate, pitch_rate = rates

        self._altitude_integral = _clip(self._altitude_integral + alt_err * dt, 2.0)
        # tilted, only cos(roll) cos(pitch) of the thrust holds the drone up; thrust ~ speed^2
        hover = p.hover_speed / math.sqrt(max(math.cos(now.roll) * math.cos(now.pitch), 0.5))
        thrust = hover + p.kp_z * alt_err + p.ki_z * self._altitude_integral + p.kd_z * alt_rate
        roll_cmd = p.kp_tilt * _clip(roll_err, 1.0) + p.kd_tilt * roll_rate
        pitch_cmd = -p.kp_tilt * _clip(pitch_err, 1.0) - p.kd_tilt * pitch_rate
        yaw_cmd = p.kp_yaw * _clip(-now.yaw_rate, 1.0)

        mix = (
            thrust - roll_cmd + pitch_cmd + yaw_cmd,
            thrust - roll_cmd - pitch_cmd - yaw_cmd,
            thrust + roll_cmd - pitch_cmd + yaw_cmd,
            thrust + roll_cmd + pitch_cmd - yaw_cmd,
        )
        m1, m2, m3, m4 = (min(max(m, 0.0), p.max_motor) for m in mix)
        return m1, m2, m3, m4


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
