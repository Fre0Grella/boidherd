import pytest

from boids import BoidParams, flock
from boids.rules import alignment, cohesion, goal, separation, speed_limit
from flight.sensing import Neighbour
from util import ZERO, Vec3

# only the rule under test switched on
ONLY_SEPARATION = BoidParams(alignment=0, cohesion=0)
ONLY_ALIGNMENT = BoidParams(separation=0, cohesion=0)
ONLY_COHESION = BoidParams(separation=0, alignment=0)


def at(x: float, y: float = 0.0, z: float = 0.0, velocity: Vec3 = ZERO) -> Neighbour:
    return Neighbour(id=1, offset=Vec3(x, y, z), velocity=velocity)


def test_alone_and_still_means_no_acceleration():
    assert flock(Vec3(), [], BoidParams()) == Vec3()


# --- separation --------------------------------------------------------------


def test_separation_pushes_away():
    a = flock(Vec3(), [at(0.5)], ONLY_SEPARATION)
    assert a.x < 0 and a.y == 0


def test_separation_is_zero_at_the_edge_and_grows_when_closer():
    p = BoidParams()
    assert separation([at(p.separation_radius)], p) == Vec3()
    assert separation([at(0.3)], p).length() > separation([at(0.6)], p).length()


def test_separation_weight_is_the_push_at_half_the_radius():
    p = BoidParams()
    assert separation([at(p.separation_radius / 2)], p).length() == pytest.approx(p.separation)


def test_separation_counts_height_with_the_vertical_scale():
    p = BoidParams(separation_radius=1.0, vertical_scale=2.0)
    assert separation([at(0, 0, 0.6)], p) == Vec3()  # 0.6 m above counts as 1.2 m
    assert separation([at(0, 0, 0.4)], p).z < 0


# --- alignment and cohesion --------------------------------------------------


def test_alignment_matches_the_neighbours_mean_velocity():
    others = [at(1.5, velocity=Vec3(1, 0, 0)), at(-1.5, velocity=Vec3(0, 1, 0))]
    assert alignment(Vec3(0.5, 0.5, 0), others) == Vec3()
    assert flock(Vec3(), others, ONLY_ALIGNMENT).x > 0


def test_cohesion_points_to_the_centre():
    assert cohesion([at(2, 0), at(0, 2)]) == Vec3(1, 1, 0)
    a = flock(Vec3(), [at(2, 0), at(0, 2)], ONLY_COHESION)
    assert a.x > 0 and a.x == pytest.approx(a.y)


def test_alone_gives_no_alignment_or_cohesion():
    assert alignment(Vec3(1, 0, 0), []) == Vec3() and cohesion([]) == Vec3()


def test_neighbours_out_of_sight_are_ignored():
    p = BoidParams()
    far = at(p.perception_radius + 0.1, velocity=Vec3(1, 0, 0))
    assert flock(Vec3(), [far], p) == Vec3()


# --- goal, limits, vertical --------------------------------------------------


def test_a_lone_drone_keeps_its_drift():
    assert flock(Vec3(0.5, 0, 0), [], BoidParams()) == Vec3()


def test_a_leader_heads_for_its_goal_at_cruise_speed():
    p = BoidParams()
    assert flock(Vec3(), [], p, to_goal=Vec3(0, 20, 0)).y > 0
    at_cruise = Vec3(0, p.goal_speed, 0)
    assert goal(at_cruise, Vec3(0, 20, 0), p) == Vec3()  # far away: keep cruising


def test_a_leader_slows_down_near_its_goal():
    p = BoidParams(arrive_radius=3.0)
    at_cruise = Vec3(p.goal_speed, 0, 0)
    assert goal(at_cruise, Vec3(1.5, 0, 0), p).x < 0  # half the radius: half the speed
    assert goal(Vec3(), Vec3(), p) == Vec3()  # there, and still: nothing to do


def test_speed_limit_brakes_only_above_max_speed():
    p = BoidParams(max_speed=1.0)
    assert speed_limit(Vec3(0.9, 0, 0), p) == Vec3()
    assert speed_limit(Vec3(2.0, 0, 0), p).x == pytest.approx(-p.speed_brake)


def test_acceleration_is_capped():
    p = BoidParams(max_acceleration=2.0)
    assert flock(Vec3(), [at(0.05)], p).length() == pytest.approx(2.0)


def test_flat_flocking_drops_vertical_acceleration():
    above = [at(0.2, 0, 0.2)]
    assert flock(Vec3(0, 0, 0.5), above, BoidParams(vertical_gain=0)).z == 0
    assert flock(Vec3(0, 0, 0.5), above, BoidParams(vertical_gain=1)).z != 0
