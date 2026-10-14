"""The only module that talks to Webots devices: one Crazyflie, read and driven.

Everything else in `flight` works on plain values, so it can be tested
without Webots. Device names are those of Webots' Crazyflie proto (R2025a)
plus the IR emitter and receiver that `make_world.py` mounts.
"""

import json
from typing import cast

from controller import (  # pyright: ignore[reportMissingImports]
    GPS,
    Emitter,
    Gyro,
    InertialUnit,
    Motor,
    Receiver,
    Robot,
)

from util import Vec3

from .control import Attitude
from .sensing import Reception

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
        self.ir_out = cast(Emitter, self.robot.getDevice("ir emitter"))
        self.ir_in = cast(Receiver, self.robot.getDevice("ir receiver"))
        for sensor in (self.imu, self.gps, self.gyro, self.ir_in):
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

    def broadcast(self, text: str) -> None:
        """Send a packet over IR; neighbours in range receive it on their next step."""
        self.ir_out.send(text)

    def receptions(self) -> list[Reception]:
        """Every IR packet that arrived since the last step (my own ones included)."""
        received = []
        while self.ir_in.getQueueLength() > 0:
            received.append(
                Reception(
                    text=self.ir_in.getString(),
                    direction=Vec3(*self.ir_in.getEmitterDirection()),
                    strength=self.ir_in.getSignalStrength(),
                )
            )
            self.ir_in.nextPacket()
        return received

    def set_motors(self, speeds: tuple[float, float, float, float]) -> None:
        for motor, sign, speed in zip(self.motors, MOTOR_SIGNS, speeds, strict=True):
            motor.setVelocity(sign * speed)
