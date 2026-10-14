"""The only module that talks to Webots devices: one Crazyflie, read and driven.

Everything else in `flight` works on plain values, so it can be tested
without Webots. Device names are those of Webots' Crazyflie proto (R2025a)
plus the IR emitter and receiver that `make_world.py` mounts.
"""

import json
from typing import cast

from controller import (  # pyright: ignore[reportMissingImports]
    GPS,
    Gyro,
    InertialUnit,
    Motor,
    Robot,
)

from util import Vec3

from .control import Attitude

MOTOR_SIGNS = (-1.0, 1.0, -1.0, 1.0)  # m1 and m3 spin the other way (as in Webots' example)


class Crazyflie:
    def __init__(self) -> None:
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        # getDevice() is typed as a generic Device: cast() tells the type checker
        # which kind each one really is (no effect at runtime)
        self.motors = [cast(Motor, self.robot.getDevice(f"m{i}_motor")) for i in range(1, 5)]
        for motor in self.motors:
            motor.setPosition(float("inf"))  # velocity control
            motor.setVelocity(0.0)
        self.imu = cast(InertialUnit, self.robot.getDevice("inertial_unit"))
        self.gps = cast(GPS, self.robot.getDevice("gps"))
        self.gyro = cast(Gyro, self.robot.getDevice("gyro"))
        for sensor in (self.imu, self.gps, self.gyro):
            sensor.enable(self.time_step)
        self.config: dict[str, object] = json.loads(self.robot.getCustomData() or "{}")
        self.id = int(self.robot.getName().split("_")[-1])

    def step(self) -> bool:
        """Advance the simulation by one time step; False when Webots ends the run."""
        return self.robot.step(self.time_step) != -1

    @property
    def time(self) -> float:
        return self.robot.getTime()

    def attitude(self) -> Attitude:
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        return Attitude(roll, pitch, yaw, self.gyro.getValues()[2], self.gps.getValues()[2])

    def position(self) -> Vec3:
        return Vec3(*self.gps.getValues())

    def velocity(self) -> Vec3:
        """World-frame velocity, m/s (what a real GPS also measures)."""
        return Vec3(*self.gps.getSpeedVector())

    def set_motors(self, speeds: tuple[float, float, float, float]) -> None:
        for motor, sign, speed in zip(self.motors, MOTOR_SIGNS, speeds, strict=True):
            motor.setVelocity(sign * speed)
