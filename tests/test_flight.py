import math

import pytest

from flight import Attitude, Autopilot, FlightParams, MotorMixer, Phase, tilt_for
from flight.params import HOVER_SPEED
from util import Vec3

LEVEL = Attitude(roll=0.0, pitch=0.0, yaw=0.0, yaw_rate=0.0, altitude=1.0)


# --- tilt_for: signs taken from Webots' Crazyflie example ------------------


def test_forward_acceleration_pitches_the_nose_down():
    roll, pitch = tilt_for(Vec3(1, 0, 0), yaw=0.0, max_tilt=1.0)
    assert pitch > 0 and roll == 0
    assert math.tan(pitch) * 9.81 == pytest.approx(1.0)  # a = g tan(tilt)


def test_leftward_acceleration_rolls_negative():
    roll, pitch = tilt_for(Vec3(0, 1, 0), yaw=0.0, max_tilt=1.0)
    assert roll < 0 and pitch == pytest.approx(0)


def test_world_acceleration_is_rotated_into_the_body_frame():
    # facing +y (yaw 90 deg), a push along world +y is "forward" for the drone
    roll, pitch = tilt_for(Vec3(0, 1, 0), yaw=math.pi / 2, max_tilt=1.0)
    assert pitch > 0 and roll == pytest.approx(0, abs=1e-12)
    # and a push along world +x is to its right
    roll, pitch = tilt_for(Vec3(1, 0, 0), yaw=math.pi / 2, max_tilt=1.0)
    assert roll > 0 and pitch == pytest.approx(0, abs=1e-12)


def test_tilt_is_capped():
    roll, pitch = tilt_for(Vec3(50, -50, 0), yaw=0.3, max_tilt=0.2)
    assert abs(roll) <= 0.2 and abs(pitch) <= 0.2


def test_vertical_part_does_not_tilt():
    assert tilt_for(Vec3(0, 0, 5), yaw=1.0, max_tilt=1.0) == (0.0, 0.0)


# --- motor mixer -------------------------------------------------------------


def test_level_at_the_setpoint_spins_all_motors_at_hover_speed():
    m = MotorMixer(FlightParams()).motors(0.032, 0.0, 0.0, 1.0, LEVEL)
    assert m == pytest.approx((HOVER_SPEED,) * 4)


def test_below_the_setpoint_spins_faster():
    m = MotorMixer(FlightParams()).motors(0.032, 0.0, 0.0, 1.5, LEVEL)
    assert all(s > HOVER_SPEED for s in m)


def test_first_step_with_zero_dt_is_safe():
    m = MotorMixer(FlightParams()).motors(0.0, 0.1, 0.1, 1.5, LEVEL)
    assert all(math.isfinite(s) for s in m)


def test_hover_speed_matches_the_proto_physics():
    assert 4 * 4e-5 * HOVER_SPEED**2 == pytest.approx(0.05 * 9.81)


# --- autopilot phases ------------------------------------------------------


def at(altitude: float) -> Attitude:
    return Attitude(roll=0.0, pitch=0.0, yaw=0.0, yaw_rate=0.0, altitude=altitude)


def test_on_the_floor_it_runs_the_liftoff_speed():
    pilot = Autopilot(altitude=1.0)
    p = FlightParams()
    assert pilot.step(0.032, at(0.015), None) == (p.liftoff_speed,) * 4
    assert pilot.phase is Phase.GROUNDED


def test_phases_follow_the_flight():
    pilot = Autopilot(altitude=1.0)
    pilot.step(0.032, at(0.015), None)
    pilot.step(0.032, at(0.1), None)  # off the floor
    assert pilot.phase is Phase.CLIMBING
    for _ in range(1000):  # let the setpoint reach the target
        pilot.step(0.032, at(1.0), None)
    assert pilot.phase is Phase.HOVERING
    pilot.step(0.032, at(1.0), None)  # no command yet: keep hovering
    assert pilot.phase is Phase.HOVERING
    pilot.step(0.032, at(1.0), Vec3(1, 0, 0))
    assert pilot.phase is Phase.FOLLOWING


def test_no_tilt_before_following():
    pilot = Autopilot(altitude=1.0)
    pilot.step(0.032, at(0.015), None)
    m = pilot.step(0.032, at(0.1), Vec3(5, 0, 0))  # climbing: the command is ignored
    assert m[0] == pytest.approx(m[1]) == pytest.approx(m[2]) == pytest.approx(m[3])
