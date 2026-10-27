import pytest

from boids import BoidParams, make_model
from boids.reynolds import Reynolds, alignment, cohesion, separation, steer
from flight.sensing import Neighbour
from util import Vec3

P = BoidParams(model="reynolds")
TAU = 0.3
GO = {"now": 20.0, "go": True, "in_c": False}


def at(x: float, y: float = 0.0, velocity: Vec3 | None = None) -> Neighbour:
    return Neighbour(id=1, offset=Vec3(x, y, 0), velocity=velocity or Vec3())


def test_alone_a_drifting_drone_keeps_drifting():
    v = Vec3(0.5, 0.2, 0)
    assert Reynolds(P, tau=TAU).wanted(v, [], to_goal=None, to_walls=[], **GO) == v


def test_the_wanted_velocity_asks_the_flight_layer_for_exactly_the_rules():
    heard = [at(2.0, velocity=Vec3(0, 0.4, 0))]
    v = Reynolds(P, tau=TAU).wanted(Vec3(), heard, to_goal=None, to_walls=[], **GO)
    a = steer(Vec3(), heard, P)
    assert (v.x, v.y) == pytest.approx((a.x * TAU, a.y * TAU))  # (wanted - v) / tau = a


def test_the_three_rules_point_the_right_way():
    assert separation([at(0.4)], P).x < 0
    assert separation([at(P.reynolds_separation_radius)], P) == Vec3()
    assert alignment(Vec3(), [at(1.0, velocity=Vec3(0, 1, 0))]) == Vec3(0, 1, 0)
    assert cohesion([at(2, 0), at(0, 2)]) == Vec3(1, 1, 0)


def test_the_rules_act_before_the_start_so_the_swarm_forms():
    v = Reynolds(P, tau=TAU).wanted(Vec3(), [at(2.0)], 1.0, False, False, None, [])
    assert v.x > 0


def test_a_leader_seeks_v_flock_at_its_goal():
    drone = Reynolds(P, leader=True, tau=TAU)
    v = drone.wanted(Vec3(), [], to_goal=Vec3(0, 20, 0), to_walls=[], **GO)
    expected = P.v_flock * P.reynolds_goal * P.v_flock * TAU  # (aim - 0) x weight x tau
    assert (v.x, v.y) == pytest.approx((0, expected))


def test_settled_in_c_it_holds_still():
    p = BoidParams(model="reynolds", settle_distance=1.0)
    drone = Reynolds(p, tau=TAU)
    drone.wanted(Vec3(0.5, 0, 0), [], 10.0, True, True, None, [])
    assert drone.wanted(Vec3(0.5, 0, 0), [], 11.0, True, True, None, []) == Vec3()


def test_capped_and_limited_like_the_others():
    fast = Reynolds(P, tau=TAU).wanted(Vec3(5, 0, 0), [], to_goal=None, to_walls=[], **GO)
    assert fast.length() == pytest.approx(P.max_speed * P.v_flock)
    walled = Reynolds(P, tau=TAU).wanted(
        Vec3(0.5, 0, 0), [], to_goal=None, to_walls=[Vec3(0.3, 0, 0)], **GO
    )
    assert walled.x == pytest.approx(0)


def test_make_model_builds_reynolds_with_the_flight_layers_tau():
    model = make_model(P, leader=False, tau=0.25)
    assert isinstance(model, Reynolds) and model.tau == 0.25
