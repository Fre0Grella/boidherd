import math

import pytest

from boids import BoidParams, Cruise, flock, to_walls
from boids.rules import braking_curve, cohesion, friction, repulsion, seek, walls
from flight.sensing import Neighbour
from util import ZERO, Vec3

P = BoidParams()
# only the term under test switched on
ONLY_REPULSION = BoidParams(friction_gain=0, cohesion=0)
ONLY_FRICTION = BoidParams(repulsion_gain=0, cohesion=0)
ONLY_COHESION = BoidParams(repulsion_gain=0, friction_gain=0)


def at(x: float, y: float = 0.0, z: float = 0.0, velocity: Vec3 = ZERO) -> Neighbour:
    return Neighbour(id=1, offset=Vec3(x, y, z), velocity=velocity)


def test_alone_and_still_means_stay():
    assert flock(Vec3(), [], P) == Vec3()


def test_alone_without_cruise_a_drone_wants_to_stop():
    assert flock(Vec3(0.5, 0, 0), [], P) == Vec3()


# --- braking curve -----------------------------------------------------------


def test_braking_curve_is_zero_at_or_below_zero_distance():
    assert braking_curve(0.0, 0.5, 0.4) == 0 and braking_curve(-1.0, 0.5, 0.4) == 0


def test_braking_curve_is_linear_close_and_constant_deceleration_far():
    slope, a = 0.5, 0.4
    assert braking_curve(0.5, slope, a) == pytest.approx(0.25)  # slope * d below a / slope
    far = 10.0
    assert braking_curve(far, slope, a) == pytest.approx(math.sqrt(2 * a * far - (a / slope) ** 2))


def test_braking_curve_is_continuous_where_the_two_parts_meet():
    slope, a = 0.5, 0.4
    joint = a / slope**2  # where slope * d reaches a / slope
    assert braking_curve(joint - 1e-9, slope, a) == pytest.approx(braking_curve(joint, slope, a))


# --- self-propulsion and leaders ---------------------------------------------


def test_cruise_goes_along_the_heading_at_v_flock():
    v = flock(Vec3(), [], P, heading=Vec3(0.6, 0.8, 0))
    assert (v.x, v.y) == pytest.approx((0.6 * P.v_flock, 0.8 * P.v_flock))
    assert flock(Vec3(0.5, 0, 0), [], P, heading=None) == Vec3()


def test_no_cruise_before_go():
    assert Cruise(P).update(Vec3(0.3, 0, 0), now=0.0, go=False, in_c=False) is None


def test_a_drone_wakes_only_once_moving():
    cruise = Cruise(P)
    assert cruise.update(Vec3(0.9 * P.wake_speed, 0, 0), now=0.0, go=True, in_c=False) is None
    assert cruise.update(Vec3(0, 0.3, 0), now=0.1, go=True, in_c=False) == Vec3(0, 1, 0)


def test_once_awake_it_keeps_its_last_heading_when_slowed_down():
    cruise = Cruise(P)
    cruise.update(Vec3(0.3, 0, 0), now=0.0, go=True, in_c=False)
    assert cruise.update(Vec3(0, 0.01, 0), now=0.1, go=True, in_c=False) == Vec3(1, 0, 0)


def test_it_stops_once_it_has_cruised_settle_distance_into_c():
    p = BoidParams(v_flock=0.5, settle_distance=2.0)  # 4 s
    cruise, v = Cruise(p), Vec3(0.3, 0, 0)
    cruise.update(v, now=0.0, go=True, in_c=False)
    assert cruise.update(v, now=1.0, go=True, in_c=True) == Vec3(1, 0, 0)  # just entered
    assert cruise.update(v, now=4.9, go=True, in_c=True) is not None
    assert cruise.update(v, now=5.0, go=True, in_c=True) is None
    assert cruise.update(Vec3(0.01, 0, 0), now=10.0, go=True, in_c=True) is None  # stays put


def test_leaving_c_restarts_the_clock():
    cruise, v = Cruise(P), Vec3(0.3, 0, 0)
    cruise.update(v, now=0.0, go=True, in_c=True)
    cruise.update(v, now=1.0, go=True, in_c=False)  # pushed out
    settle_time = P.settle_distance / P.v_flock
    assert cruise.update(v, now=settle_time + 0.5, go=True, in_c=True) is not None


def test_the_heading_is_horizontal():
    heading = Cruise(P).update(Vec3(0.3, 0, 0.3), now=0.0, go=True, in_c=False)
    assert heading == Vec3(1, 0, 0)


def test_a_leader_at_rest_heads_straight_for_its_goal():
    v = seek(Vec3(), Vec3(0, 20, 0), P)
    assert (v.x, v.y) == pytest.approx((0, P.v_flock))


def test_a_leader_mixes_its_heading_with_the_goal():
    v = seek(Vec3(0.5, 0, 0), Vec3(0, 20, 0), BoidParams(goal_weight=1.0))
    assert v.x == pytest.approx(v.y) and v.length() == pytest.approx(P.v_flock)


def test_a_leader_slows_down_near_its_goal():
    p = BoidParams(arrive_radius=3.0)
    assert seek(Vec3(), Vec3(1.5, 0, 0), p).length() == pytest.approx(p.v_flock / 2)
    assert seek(Vec3(), Vec3(), p) == Vec3()


# --- repulsion, friction, cohesion -------------------------------------------


def test_repulsion_pushes_away_linearly_inside_the_radius():
    assert repulsion([at(P.repulsion_radius)], P) == Vec3()
    push = repulsion([at(0.4)], P)
    assert push.x == pytest.approx(-P.repulsion_gain * (P.repulsion_radius - 0.4))
    assert flock(Vec3(), [at(0.4)], ONLY_REPULSION).x < 0


def test_repulsion_counts_height_with_the_vertical_scale():
    p = BoidParams(repulsion_radius=1.0, vertical_scale=2.0)
    assert repulsion([at(0, 0, 0.6)], p) == Vec3()  # 0.6 m above counts as 1.2 m
    assert repulsion([at(0, 0, 0.4)], p).z < 0


def test_friction_corrects_only_beyond_the_slack_up_close():
    close = at(1.0, velocity=Vec3(0, 0.5, 0))  # inside friction_radius: only the slack
    v = friction(Vec3(), [close], P)
    assert v.y == pytest.approx(P.friction_gain * (0.5 - P.friction_slack)) and v.x == 0
    assert friction(Vec3(0, 0.45, 0), [close], P) == Vec3()  # within the slack


def test_friction_allows_more_difference_farther_away():
    gap = Vec3(0, 0.3, 0)
    near = friction(Vec3(), [at(1.0, velocity=gap)], ONLY_FRICTION).length()
    far = friction(Vec3(), [at(2.4, velocity=gap)], ONLY_FRICTION).length()
    assert near > far


def test_cohesion_points_to_the_centre():
    assert cohesion([at(2, 0), at(0, 2)]) == Vec3(1, 1, 0)
    v = flock(Vec3(), [at(2, 0), at(0, 2)], ONLY_COHESION)
    assert v.x > 0 and v.x == pytest.approx(v.y)


def test_neighbours_out_of_sight_are_ignored():
    far = at(P.perception_radius + 0.1, velocity=Vec3(1, 0, 0))
    assert flock(Vec3(), [far], P) == Vec3()


# --- walls -------------------------------------------------------------------


def test_a_wall_far_away_is_ignored():
    assert walls(Vec3(0.5, 0, 0), [Vec3(10, 0, 0)], P) == Vec3()


def test_a_wall_pushes_a_drone_at_rest_close_to_it_away():
    push = walls(Vec3(), [Vec3(0.2, 0, 0)], P)
    allowed = braking_curve(0.2, P.wall_slope, P.wall_braking)
    assert push == Vec3(-(P.wall_speed - allowed), 0, 0)


def test_a_wall_brakes_a_drone_flying_at_it_from_farther_than_one_at_rest():
    toward = walls(Vec3(0.5, 0, 0), [Vec3(2.0, 0, 0)], P)
    assert toward.x < 0 and walls(Vec3(), [Vec3(2.0, 0, 0)], P) == Vec3()


def test_a_wall_ignores_height_and_a_drone_on_it():
    assert walls(Vec3(), [Vec3(0.2, 0, 1.0)], P) == walls(Vec3(), [Vec3(0.2, 0, 0)], P)
    assert walls(Vec3(), [Vec3()], P) == Vec3()


def test_flock_adds_the_walls():
    assert flock(Vec3(), [], P, to_walls=[Vec3(0.2, 0, 0)]).x < 0


def test_to_walls_points_to_the_nearest_point_of_each_box():
    boxes = [(2.0, -1.0, 3.0, 1.0), (-1.0, 4.0, 1.0, 5.0), (5.0, 5.0, 6.0, 6.0)]
    assert to_walls(Vec3(0, 0, 1.2), boxes) == [Vec3(2, 0, 0), Vec3(0, 4, 0), Vec3(5, 5, 0)]


def test_to_walls_is_zero_inside_a_box():
    assert to_walls(Vec3(2.5, 0, 1), [(2.0, -1.0, 3.0, 1.0)]) == [Vec3()]


# --- speed -------------------------------------------------------------------


@pytest.mark.parametrize("k", [0.5, 1.5])
def test_the_rules_are_speed_free(k: float):
    """At k times v_flock, with every velocity around k times as large, every term (and
    so the wanted velocity) is k times as large: the same flock, faster."""
    v = Vec3(0.8, 0.3, 0)  # well above wake_speed, also at k = 0.5
    neighbours = [(Vec3(0.5, 0.2, 0), Vec3(0.2, 0.9, 0)), (Vec3(-2.2, 0.4, 0), Vec3(-0.5, 0, 0))]
    walls_at = [Vec3(0, 0.6, 0), Vec3(2.0, 0, 0)]  # both parts of the braking curve
    for extra in ({"heading": Vec3(0, 1, 0)}, {"to_goal": Vec3(2.0, -1.0, 0)}):

        def wanted(s: float, p: BoidParams) -> Vec3:
            heard = [Neighbour(1, o, u * s) for o, u in neighbours]
            return flock(v * s, heard, p, to_walls=walls_at, **extra)  # noqa: B023

        slow, fast = wanted(1, P), wanted(k, BoidParams(v_flock=P.v_flock * k))
        assert (fast.x, fast.y) == pytest.approx((slow.x * k, slow.y * k))


# --- limits, vertical --------------------------------------------------------


def test_wanted_velocity_is_capped():
    p = BoidParams(v_flock=0.5, max_speed=1.6, repulsion_gain=10.0)  # 0.8 m/s
    assert flock(Vec3(), [at(0.05), at(0, 0.05)], p).length() == pytest.approx(0.8)


def test_flat_flocking_drops_vertical_velocity():
    above = [at(0.2, 0, 0.2)]
    assert flock(Vec3(), above, BoidParams(vertical_gain=0)).z == 0
    assert flock(Vec3(), above, BoidParams(vertical_gain=1)).z != 0
