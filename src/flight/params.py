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
# The proto's motors reach 600 rad/s: ~117 times the weight in thrust. A real Crazyflie 2.1
# lifts 60 g (Bitcraze) and weighs 29 g (firmware's CF_MASS): thrust/weight ~2.07. Capping
# the motors there keeps whatever the controllers do within what the real drone can do.
THRUST_TO_WEIGHT = 60 / 29
REAL_MAX_MOTOR = HOVER_SPEED * math.sqrt(THRUST_TO_WEIGHT)  # about 79.7 rad/s


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
    # following a wanted velocity (control.acceleration_for): the gap closes with this time
    # constant, at most at max_acceleration (below g tan(max_tilt) = 5.7 m/s^2). Measured with
    # tools/flight_test.py: the attitude loops alone take ~0.27 s to 63% of a step; at 0.3 s
    # the drone tracks the ideal curve without overshoot, at 0.2 s it overshoots 15%
    velocity_tau: float = 0.3  # s
    max_acceleration: float = 2.5  # m/s^2
    # turning accelerations into tilt
    # 30 deg: holding altitude needs 1/cos 30 = 1.15 x the weight, 56% of the real drone's
    # thrust, which leaves room to correct; a = g tan 30 = 5.7 m/s^2. (Bitcraze's firmware
    # stops at 20 deg in velocity mode; physics, at 61 deg with thrust/weight 2.07.)
    max_tilt: float = math.radians(30)  # rad, caps the commanded tilt
    # altitude PID: gains from Webots' Crazyflie example (pid_controller.py, Apache 2.0),
    # hover speed computed from the proto's physics
    hover_speed: float = HOVER_SPEED
    kp_z: float = 10.0
    ki_z: float = 5.0
    kd_z: float = 5.0
    # attitude, two nested loops (control.py). Units: the inner loop's output is a motor
    # speed difference, rad/s per rad/s of rate error
    # tuned with tools/flight_test.py (design log, "Attitude: two nested loops"). Above
    # ~2 rad/s the 31 Hz loop can't brake a rotation in time and the drone flips
    kp_angle: float = 10.0  # 1/s: wanted rotation rate per radian of tilt error
    max_rate: float = 1.5  # rad/s (86 deg/s), the fastest the drone is asked to rotate
    kp_rate: float = 1.0  # motor rad/s per rad/s of rate error
    kp_yaw_angle: float = 2.0  # 1/s
    max_yaw_rate: float = 1.0  # rad/s
    kp_yaw_rate: float = 2.0
    max_motor: float = REAL_MAX_MOTOR  # rad/s
