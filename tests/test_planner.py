import math
from pathlib import Path

import pytest

from boids.obstacles import Box
from maps import load_map
from maps.course import build_course
from planner import Planner, PlanParams, Pursuit, Route, plan
from planner.plan import Grid, kept_obstacles
from util import Vec3

MAPS = Path(__file__).parent / "maps"  # frozen copies, not the maps/ to experiment with
P = PlanParams()


def room(x: float, y: float, t: float = 0.1) -> list[Box]:
    """An empty x by y room's outline, as boundary boxes just outside it."""
    return [(-t, -t, x + t, 0), (-t, y, x + t, y + t), (-t, 0, 0, y), (x, 0, x + t, y)]


def l_hall() -> tuple[list[Box], list[Box], list[tuple[float, float, float]]]:
    c = build_course(load_map(MAPS / "l_hall.toml"))

    def box(r) -> Box:
        return (r.x_min, r.y_min, r.x_max, r.y_max)

    return (
        [box(w.rect) for w in c.boundary],
        [box(w.rect) for w in c.walls],
        [(*p.center, p.radius) for p in c.pillars],
    )


def segments_clear(path: list[tuple[float, float]], grid: Grid, at_least: float) -> bool:
    for a, b in zip(path, path[1:], strict=False):
        for k in range(41):
            q = (a[0] + (b[0] - a[0]) * k / 40, a[1] + (b[1] - a[1]) * k / 40)
            if grid.clearance(q) < at_least:
                return False
    return True


# --- what a plan goes round ----------------------------------------------------


def test_small_obstacles_are_left_to_the_drones_and_the_outline_always_stays():
    boundary = room(10, 4)
    walls = [(4.0, 1.0, 4.5, 1.5), (6.0, 0.0, 8.0, 0.2)]  # 0.5 m: dropped; 2 m: kept
    pillars = [(5.0, 2.0, 0.15), (2.0, 2.0, 0.6)]  # 0.3 m across: dropped; 1.2 m: kept
    boxes, disks = kept_obstacles(boundary, walls, pillars, P)
    assert boxes == boundary + [walls[1]] and disks == [pillars[1]]


def test_in_an_empty_room_the_path_is_one_straight_line():
    assert plan((2.0, 5.0), (18.0, 5.0), room(20, 10), [], []) == [(2.0, 5.0), (18.0, 5.0)]


def test_a_small_obstacle_in_the_way_doesnt_bend_the_path():
    path = plan((2.0, 5.0), (18.0, 5.0), room(20, 10), [(9.8, 4.8, 10.2, 5.2)], [])
    assert path == [(2.0, 5.0), (18.0, 5.0)]


def test_a_wall_in_the_way_is_planned_round():
    wall = (9.9, 0.0, 10.1, 7.0)  # across the room, open at the top
    path = plan((2.0, 2.0), (18.0, 2.0), room(20, 10), [wall], [])
    assert path is not None and len(path) > 2
    assert any(q[1] > 7.0 for q in path)


def test_no_passage_no_plan():
    wall = (9.9, 0.0, 10.1, 10.0)  # wall to wall
    assert plan((2.0, 2.0), (18.0, 2.0), room(20, 10), [wall], []) is None


# --- clearance -------------------------------------------------------------------


def test_the_l_hall_path_rounds_the_inner_corner_and_crosses_the_gap_in_its_middle():
    boundary, walls, pillars = l_hall()
    path = plan((6.5, 4.0), (36.0, 27.0), boundary, walls, pillars)
    assert path is not None and path[0] == (6.5, 4.0) and path[-1] == (36.0, 27.0)
    corner = min(
        math.dist((32.0, 8.0), (a[0] + (b[0] - a[0]) * k / 40, a[1] + (b[1] - a[1]) * k / 40))
        for a, b in zip(path, path[1:], strict=False)
        for k in range(41)
    )
    assert corner >= 1.5  # soft: near the 2 m clearance, far beyond the 0.3 m margin
    crossing = [
        a[0] + (b[0] - a[0]) * (16.1 - a[1]) / (b[1] - a[1])
        for a, b in zip(path, path[1:], strict=False)
        if min(a[1], b[1]) <= 16.1 <= max(a[1], b[1])
    ]
    assert crossing and abs(crossing[0] - 36.0) < 0.5  # the gap is x 34-38


def test_a_passage_narrower_than_the_clearance_is_still_taken():
    gap = [(9.9, 0.0, 10.1, 4.4), (9.9, 5.6, 10.1, 10.0)]  # a 1.2 m gap, under 2 x 2 m
    path = plan((2.0, 5.0), (18.0, 5.0), room(20, 10), gap, [])
    assert path is not None


def test_pruning_never_brings_the_path_closer_to_a_wall():
    boundary, walls, pillars = l_hall()
    grid = Grid(*kept_obstacles(boundary, walls, pillars, P), P)
    path = plan((6.5, 4.0), (36.0, 27.0), boundary, walls, pillars)
    assert path is not None
    assert segments_clear(path[1:-1], grid, P.margin)  # inner segments keep the margin


def test_closer_to_a_wall_costs_more_and_far_away_nothing_extra():
    grid = Grid(room(4, 4), [], P)
    assert grid.penalty(P.clearance) == 1.0
    assert grid.penalty(P.margin) == pytest.approx(1.0 + P.clearance_weight)
    assert grid.penalty(1.0) > grid.penalty(1.5)


# --- following ---------------------------------------------------------------------


def test_pursuit_aims_ahead_along_the_path_with_the_way_left_as_length():
    pursuit = Pursuit([(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], lookahead=2.0)
    v = pursuit.to_goal(Vec3(1.0, 0.0, 1.2))
    assert (v.x, v.y) == pytest.approx((19.0, 0.0))  # toward (3, 0); 19 m left


def test_pushed_off_the_path_it_heads_back_to_it_ahead():
    pursuit = Pursuit([(0.0, 0.0), (10.0, 0.0)], lookahead=2.0)
    v = pursuit.to_goal(Vec3(4.0, 2.0, 0))
    assert v.x > 0 and v.y < 0  # back down to the path, still forward


def test_near_the_end_it_aims_at_the_goal_so_the_leader_can_stop():
    pursuit = Pursuit([(0.0, 0.0), (10.0, 0.0)], lookahead=2.0)
    for x in range(9):  # a leader moves along its path a little at a time
        pursuit.to_goal(Vec3(x, 0.5, 0))
    v = pursuit.to_goal(Vec3(9.0, 0.5, 0))
    assert (v.x, v.y) == pytest.approx((1.0, -0.5))


def test_progress_only_moves_forward_on_a_path_that_doubles_back():
    hairpin = [(0.0, 0.0), (10.0, 0.0), (10.0, 1.0), (0.0, 1.0)]
    pursuit = Pursuit(hairpin, lookahead=1.0)
    pursuit.to_goal(Vec3(9.5, 0.0, 0))
    pursuit.to_goal(Vec3(9.9, 0.6, 0))
    later = pursuit.progress
    pursuit.to_goal(Vec3(5.0, 0.4, 0))  # between both legs: must stay on the second
    assert pursuit.progress >= later


# --- planning again ----------------------------------------------------------------

# a cup open toward the start, as in dead_end: the straight line to the goal ends in it
CUP = [(11.9, 2.0, 12.1, 9.0), (7.0, 8.9, 11.9, 9.1), (7.0, 1.9, 11.9, 2.1)]
START, GOAL = (2.0, 5.5), (18.0, 5.5)


def cup_route() -> Route:
    return Route(Planner(room(20, 12), CUP, [], P), GOAL, P)


def test_a_wall_blocks_the_view_and_a_small_obstacle_doesnt():
    planner = Planner(room(20, 10), [(9.9, 0.0, 10.1, 7.0), (5.0, 4.8, 5.4, 5.2)], [], P)
    assert not planner.sees((2.0, 2.0), (18.0, 2.0))  # through the wall
    assert planner.sees((2.0, 8.0), (18.0, 8.0))  # over its end
    assert planner.sees((2.0, 5.0), (8.0, 5.0))  # through the 0.4 m box, left to the drones


def test_a_big_pillar_blocks_the_view():
    planner = Planner(room(20, 10), [], [(10.0, 5.0, 1.0)], P)
    assert not planner.sees((2.0, 5.0), (18.0, 5.0))
    assert planner.sees((2.0, 2.0), (18.0, 2.0))


def test_a_leader_on_its_path_plans_once():
    route = cup_route()
    path = None
    for k in range(60):  # 6 s along the path at 1 m/s
        if route.pursuit is not None:
            path = route.pursuit
            here = path.point_at(k * 0.1)
        else:
            here = START
        route.to_goal(Vec3(here[0], here[1], 0), now=k * 0.1)
    assert path is not None and route.plans == 1


def test_pushed_behind_a_wall_it_plans_again_from_where_it_is():
    route = cup_route()
    route.to_goal(Vec3(START[0], START[1], 0), now=0.0)
    first = route.pursuit
    assert first is not None and any(q[1] > 9.1 for q in first.path)  # round the top arm
    inside = Vec3(11.0, 8.0, 0)  # the flock pushed it into the cup, under an arm
    v = route.to_goal(inside, now=0.5)  # too soon to plan again
    assert route.plans == 1
    v = route.to_goal(inside, now=1.0)
    assert route.plans == 2 and route.pursuit is not first
    assert v is not None and v.x < 0  # back out of the cup's mouth first


def test_with_no_passage_there_is_no_route():
    route = Route(Planner(room(20, 10), [(9.9, 0.0, 10.1, 10.0)], [], P), (18.0, 2.0), P)
    assert route.to_goal(Vec3(2.0, 2.0, 0), now=0.0) is None
