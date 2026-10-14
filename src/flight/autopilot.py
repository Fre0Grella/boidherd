"""Takeoff, hover, then follow commanded accelerations. Pure logic, no Webots.

    GROUNDED --(off the floor)--> CLIMBING --(at altitude)--> HOVERING --(go)--> FOLLOWING

On the floor the motors run at a fixed liftoff speed (see FlightParams).
Once the drone is off the floor the PIDs take over, with a fresh memory,
and the altitude setpoint rises from there at `climb_rate` instead of
jumping to the target. While climbing and hovering the drone stays level;
only in FOLLOWING does it tilt for the commanded accelerations.
"""

from enum import Enum

from util import Vec3

from .control import Attitude, MotorMixer, tilt_for
from .params import FlightParams


class Phase(Enum):
    GROUNDED = "grounded"
    CLIMBING = "climbing"
    HOVERING = "hovering"
    FOLLOWING = "following"


class Autopilot:
    def __init__(self, altitude: float, params: FlightParams | None = None) -> None:
        self.p = params or FlightParams()
        self.target_altitude = altitude
        self.phase = Phase.GROUNDED
        self._floor: float | None = None  # altitude measured at the start
        self._setpoint = 0.0
        self._mixer = MotorMixer(self.p)

    def step(
        self, dt: float, now: Attitude, acceleration: Vec3 | None
    ) -> tuple[float, float, float, float]:
        """Motor speeds for this step. `acceleration` (world frame, m/s^2) is followed once
        the drone hovers; None means "not yet": keep climbing or hovering."""
        if self.phase is Phase.GROUNDED:
            if self._floor is None:
                self._floor = now.altitude
            if now.altitude - self._floor < self.p.liftoff_height:
                s = self.p.liftoff_speed
                return s, s, s, s
            self.phase = Phase.CLIMBING
            self._setpoint = now.altitude
            self._mixer = MotorMixer(self.p)  # no memory of the time on the floor

        # the setpoint keeps rising until it reaches the target, whatever the phase,
        # slowing down over the last metres so the drone doesn't overshoot
        gap = self.target_altitude - self._setpoint
        rate = min(self.p.climb_rate, self.p.climb_slowdown * gap)
        self._setpoint = min(self._setpoint + rate * dt, self.target_altitude)
        if gap < 0.005:
            self._setpoint = self.target_altitude
        if self.phase is Phase.CLIMBING:
            reached = self._setpoint == self.target_altitude
            if reached and abs(now.altitude - self.target_altitude) < self.p.hover_tolerance:
                self.phase = Phase.HOVERING
        elif self.phase is Phase.HOVERING and acceleration is not None:
            self.phase = Phase.FOLLOWING

        roll = pitch = 0.0
        if self.phase is Phase.FOLLOWING and acceleration is not None:
            roll, pitch = tilt_for(acceleration, now.yaw, self.p.max_tilt)
        return self._mixer.motors(dt, roll, pitch, self._setpoint, now)
