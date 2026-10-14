"""Flight settings: every number the flight layer uses, in one place.

Change a default here, or build `FlightParams(...)` with other values for a
run; nothing else in the flight code hard-codes these.
"""

import math
from dataclasses import dataclass

# From Webots' Crazyflie proto (R2025a): mass 0.05 kg, each propeller pushes
# 4e-5 * speed^2 newtons. Four of them hold the weight when 4 k w^2 = m g.
CRAZYFLIE_MASS = 0.05  # kg
THRUST_CONSTANT = 4e-5  # N / (rad/s)^2
HOVER_SPEED = math.sqrt(CRAZYFLIE_MASS * 9.81 / (4 * THRUST_CONSTANT))  # about 55.4 rad/s


@dataclass(frozen=True)
class FlightParams:
    # takeoff: on the floor the motors run at a fixed liftoff speed. In Webots the
    # drone leaves the floor only ~2 s after the motors start (measured with
    # tools/flight_test.py; cause not pinned down), and a PID would wind up its
    # integral while waiting, then overshoot
    liftoff_speed: float = 70.0  # rad/s, a little above the 55.4 that holds it in the air
    liftoff_height: float = 0.05  # m above the starting altitude: from here the PIDs fly
    climb_rate: float = 0.4  # m/s, how fast the altitude setpoint then rises
    climb_slowdown: float = 1.0  # 1/s: near the target the setpoint rises at this x the gap
    hover_tolerance: float = 0.05  # m, altitude error under which the drone counts as hovering
    # turning accelerations into tilt
    max_tilt: float = 0.35  # rad (about 20 degrees), caps the commanded roll and pitch
    # PID: structure and gains from Webots' Crazyflie example (pid_controller.py,
    # Apache 2.0), except the hover speed, computed from the proto's physics
    hover_speed: float = HOVER_SPEED
    kp_z: float = 10.0
    ki_z: float = 5.0
    kd_z: float = 5.0
    kp_tilt: float = 0.5
    kd_tilt: float = 0.1
    kp_yaw: float = 1.0
    max_motor: float = 600.0  # rad/s, the Crazyflie motors' limit
