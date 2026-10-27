import math

import pytest

from boids import BoidParams, make_model
from boids.couzin import Couzin
from boids.models import Hybrid
from flight.sensing import Neighbour
from util import Vec3

P = BoidParams(model="couzin")
GO = {"now": 20.0, "go": True, "in_c": False}


def at(x: float, y: float = 0.0, velocity: Vec3 | None = None) -> Neighbour:
    return Neighbour(id=1, offset=Vec3(x, y, 0), velocity=velocity or Vec3())


def wanted(drone: Couzin, heard: list[Neighbour], to_goal: Vec3 | None = None) -> Vec3:
    return drone.wanted(Vec3(), heard, to_goal=to_goal, to_walls=[], **GO)


def test_nothing_before_go():
    drone = Couzin(P, leader=True)
    v = drone.wanted(Vec3(), [], 1.0, go=False, in_c=False, to_goal=Vec3(10, 0, 0), to_walls=[])
    assert v == Vec3() and drone.direction is None


def test_a_follower_waits_until_a_neighbour_moves():
    drone = Couzin(P)
    assert wanted(drone, [at(2.0)]) == Vec3() and drone.direction is None
    assert wanted(drone, [at(2.0, velocity=Vec3(0, 1, 0))]).length() > 0


def test_attraction_and_alignment_weigh_the_same():
    drone = Couzin(P)
    wanted(drone, [at(2.0, velocity=Vec3(0, 1, 0))])  # toward it (+x), along it (+y)
    assert drone.direction == Vec3(1, 1, 0).unit()


def test_repulsion_comes_first_and_alone():
    drone = Couzin(P)
    wanted(drone, [at(0.4, velocity=Vec3(0, 1, 0)), at(0, 2.0, velocity=Vec3(0, 1, 0))])
    assert drone.direction == Vec3(-1, 0, 0)  # away from the close one, nothing else


def test_it_flies_its_direction_at_v_flock():
    drone = Couzin(P)
    v = wanted(drone, [at(2.0, velocity=Vec3(1, 0, 0))])
    assert v.length() == pytest.approx(P.v_flock)


def test_alone_it_keeps_its_direction_whatever_its_velocity():
    drone = Couzin(P)
    drone.direction = Vec3(0, 1, 0)
    v = drone.wanted(Vec3(0, -0.8, 0), [], to_goal=None, to_walls=[], **GO)  # overshooting back
    assert drone.direction == Vec3(0, 1, 0) and v == Vec3(0, P.v_flock, 0)


def test_a_leader_alone_heads_straight_for_its_goal():
    drone = Couzin(P, leader=True)
    wanted(drone, [], to_goal=Vec3(0, 20, 0))
    assert drone.direction == Vec3(0, 1, 0)


def test_a_leader_adds_the_goal_with_couzin_weight():
    drone = Couzin(P, leader=True)
    wanted(drone, [at(0, 2.0, velocity=Vec3(0, 1, 0))], to_goal=Vec3(20, 0, 0))
    expected = (Vec3(0, 1, 0) + Vec3(1, 0, 0) * P.couzin_weight).unit()
    assert drone.direction is not None
    assert (drone.direction.x, drone.direction.y) == pytest.approx((expected.x, expected.y))


def test_a_leader_slows_down_near_its_goal():
    v = wanted(Couzin(P, leader=True), [], to_goal=Vec3(P.arrive_radius / 2, 0, 0))
    assert v.length() == pytest.approx(P.v_flock / 2)


def test_it_stops_once_settled_in_c():
    p = BoidParams(model="couzin", settle_distance=1.0)  # 1 s at 1 m/s
    drone = Couzin(p)
    drone.direction = Vec3(1, 0, 0)
    drone.wanted(Vec3(), [], 10.0, go=True, in_c=True, to_goal=None, to_walls=[])
    v = drone.wanted(Vec3(), [], 11.0, go=True, in_c=True, to_goal=None, to_walls=[])
    assert v == Vec3() and drone.direction is None


def test_the_stopping_limit_still_holds():
    drone = Couzin(P)
    drone.direction = Vec3(1, 0, 0)
    v = drone.wanted(Vec3(), [], to_goal=None, to_walls=[Vec3(0.3, 0, 0)], **GO)
    assert v.x == pytest.approx(0)


def test_make_model_picks_the_named_model():
    assert isinstance(make_model(P, leader=False), Couzin)
    assert isinstance(make_model(BoidParams(), leader=False), Hybrid)
    with pytest.raises(ValueError, match="unknown model 'vicsek'"):
        make_model(BoidParams(model="vicsek"), leader=False)


def test_a_direction_is_a_unit_vector():
    drone = Couzin(P)
    wanted(drone, [at(1.5, 0.7, velocity=Vec3(0.3, 0.9, 0)), at(-1.0, 1.2)])
    assert drone.direction is not None and math.isclose(drone.direction.length(), 1.0)
