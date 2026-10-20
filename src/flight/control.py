"""From a wanted acceleration to the four motor speeds. Pure maths, no Webots.

A quadrotor accelerates sideways by tilting: the thrust that holds it up
then also pushes it horizontally, with a ~ g * tan(tilt). So a world-frame
acceleration is rotated into the drone's body frame (it can face any way)
and turned into a pitch (forward) and a roll (sideways), while an altitude
PID keeps the drone at its height.

The attitude is held by two nested loops, as in the Crazyflie firmware: the
angle error asks for a rotation rate, capped at `max_rate`, and the rate
error (measured by the gyro) sets the motor differential. A single loop from
angle straight to motors (Webots' example) was either slow or, when tuned
fast, flipped the drone on a 40 degree reversal of the wanted tilt; capping
the rate turns a big swing into a controlled ramp. Yaw is held the same way,
at the heading the drone took off with.

The altitude PID, the motor mixing and their signs come from Webots'
Crazyflie example (projects/robots/bitcraze/crazyflie/controllers/
crazyflie_py_wallfollowing/pid_controller.py, Apache 2.0).
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
    roll_rate: float = 0.0  # from the gyro, body frame: the inner (rate) loop
    pitch_rate: float = 0.0


def tilt_for(acceleration: Vec3, yaw: float, max_tilt: float) -> tuple[float, float]:
    """The (roll, pitch) that gives a horizontal world-frame acceleration.

    Signs as in the Crazyflie example: positive pitch accelerates forward
    (+x of the body), positive roll accelerates to the right (-y of the body).
    An acceleration beyond g tan(max_tilt) is shortened, not clipped per axis,
    so the drone still accelerates the way it was asked to.
    """
    acceleration = acceleration.horizontal().clamped(G * math.tan(max_tilt))
    c, s = math.cos(yaw), math.sin(yaw)
    forward = acceleration.x * c + acceleration.y * s  # world -> body frame
    left = -acceleration.x * s + acceleration.y * c
    pitch = math.atan2(forward, G)
    roll = -math.atan2(left, G)
    return roll, pitch


class MotorMixer:
    """Altitude PID, attitude loops and motor mixing; keeps their memory between steps."""

    def __init__(self, params: FlightParams) -> None:
        self.p = params
        self._altitude_integral = 0.0
        self._last_alt_err: float | None = None
        self._yaw_target: float | None = None  # the heading at the first step

    def motors(
        self, dt: float, roll: float, pitch: float, altitude: float, now: Attitude
    ) -> tuple[float, float, float, float]:
        """Motor speeds (m1..m4) for the wanted roll, pitch and altitude, holding the heading."""
        p = self.p
        alt_err, roll_err, pitch_err = altitude - now.altitude, roll - now.roll, pitch - now.pitch
        last = self._last_alt_err if self._last_alt_err is not None and dt > 0 else alt_err
        alt_rate = (alt_err - last) / dt if dt > 0 else 0.0
        self._last_alt_err = alt_err

        self._altitude_integral = _clip(self._altitude_integral + alt_err * dt, 2.0)
        # tilted, only cos(roll) cos(pitch) of the thrust holds the drone up; thrust ~ speed^2
        hover = p.hover_speed / math.sqrt(max(math.cos(now.roll) * math.cos(now.pitch), 0.5))
        thrust = hover + p.kp_z * alt_err + p.ki_z * self._altitude_integral + p.kd_z * alt_rate
        if self._yaw_target is None:
            self._yaw_target = now.yaw
        yaw_err = math.remainder(self._yaw_target - now.yaw, math.tau)  # the short way round
        # outer loop: angle error -> wanted rate (capped); inner loop: rate error -> motors.
        # Signs from the example: +roll_cmd and -pitch_cmd raise roll and pitch, +yaw_cmd yaw
        roll_cmd = p.kp_rate * (_clip(p.kp_angle * roll_err, p.max_rate) - now.roll_rate)
        pitch_cmd = -p.kp_rate * (_clip(p.kp_angle * pitch_err, p.max_rate) - now.pitch_rate)
        yaw_rate = _clip(p.kp_yaw_angle * yaw_err, p.max_yaw_rate)
        yaw_cmd = p.kp_yaw_rate * (yaw_rate - now.yaw_rate)

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
