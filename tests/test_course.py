import math
from pathlib import Path

import pytest
from shapely import box
from shapely.geometry import Point

from maps import MapError, Wall, load_map, parse_map
from maps.course import WALL_THICKNESS, Course, build_course

MAPS = Path(__file__).parent.parent / "maps"
L_RECTS = [[[0, 0], [20, 4]], [[16, 0], [20, 16]]]
L_SECTORS = {"A": {"x": [0, 4], "y": [0, 4]}, "C": {"x": [16, 20], "y": [12, 16]}}
# a square track around a 6 x 6 m hole
RING_RECTS = [[[0, 0], [10, 2]], [[0, 8], [10, 10]], [[0, 0], [2, 10]], [[8, 0], [10, 10]]]
RING_SECTORS = {"A": {"x": [0, 2], "y": [0, 2]}, "C": {"x": [8, 10], "y": [8, 10]}}


def course_of(rects: object = L_RECTS, sectors: object = L_SECTORS, **obstacles: object) -> Course:
    doc: dict[str, object] = {
        "version": 1,
        "arena": {"ceiling": 2.5, "rects": rects},
        "sectors": sectors,
        "obstacles": obstacles,
    }
    return build_course(parse_map(doc))


def error_of(rects: object = L_RECTS, sectors: object = L_SECTORS, **obstacles: object) -> str:
    with pytest.raises(MapError) as e:
        course_of(rects, sectors, **obstacles)
    return str(e.value)


def assert_boundary_closed(course: Course) -> None:
    """Every point just outside the free space, corners included, is inside a wall."""
    walls = solid(course.boundary)
    offset = course.area.buffer(WALL_THICKNESS / 2, join_style="mitre")
    for ring in (offset.exterior, *offset.interiors):
        for i in range(200):
            p = ring.interpolate(i / 200, normalized=True)
            assert any(w.covers(p) for w in walls), f"gap in the boundary at {p}"


def solid(walls: tuple[Wall, ...]):
    return [box(w.rect.x_min, w.rect.y_min, w.rect.x_max, w.rect.y_max) for w in walls]


# --- arena and boundary walls -----------------------------------------------


def test_l_hall_gets_one_wall_per_edge():
    course = course_of()
    assert len(course.boundary) == 6  # an L has 6 edges
    assert course.area.area == 20 * 4 + 4 * 12


def test_boundary_walls_never_enter_the_free_space():
    course = course_of()
    for wall in solid(course.boundary):
        assert wall.intersection(course.area).area == pytest.approx(0)


def test_boundary_is_closed_all_around():
    assert_boundary_closed(course_of())


def test_arena_with_a_hole_gets_walls_around_the_hole_too():
    course = course_of(RING_RECTS, RING_SECTORS)
    assert len(course.area.interiors) == 1
    assert len(course.boundary) == 8  # 4 outside, 4 around the hole
    for wall in solid(course.boundary):
        assert wall.intersection(course.area).area == pytest.approx(0)
    assert_boundary_closed(course)


def test_floor_covers_the_walls():
    course = course_of()
    floor = box(course.floor.x_min, course.floor.y_min, course.floor.x_max, course.floor.y_max)
    for wall in solid(course.boundary):
        assert floor.covers(wall)


def test_rectangles_that_dont_connect_are_rejected():
    msg = error_of([[[0, 0], [4, 4]], [[10, 0], [20, 16]]])
    assert msg.startswith("arena.rects: the rectangles don't all connect")


def test_rectangles_touching_only_at_a_corner_are_rejected():
    assert error_of([[[0, 0], [16, 4]], [[16, 4], [20, 16]]]).startswith("arena.rects")


# --- sectors and obstacles --------------------------------------------------


def test_sector_outside_the_arena_is_rejected():
    sectors = {"A": {"x": [0, 4], "y": [0, 4]}, "C": {"x": [5, 9], "y": [8, 12]}}
    assert error_of(sectors=sectors) == "sectors.C: not inside the arena"


def test_obstacle_on_a_sector_is_rejected():
    msg = error_of(pillars=[{"center": [3, 2], "radius": 0.2}])
    assert msg == "obstacles.pillars[0]: overlaps sector A"


def test_obstacle_outside_the_arena_is_rejected():
    assert error_of(walls=[[[10, 3], [10.2, 5]]]) == "obstacles.walls[0]: not inside the arena"


def test_wall_sealing_the_hall_makes_c_unreachable():
    msg = error_of(walls=[[[10, 0], [10.2, 4]]])
    assert msg.startswith("sectors.C: can't be reached from A")


def test_low_wall_blocks_too_until_phase_4():
    msg = error_of(walls=[{"rect": [[10, 0], [10.2, 4]], "height": 0.5}])
    assert msg.startswith("sectors.C: can't be reached from A")


def test_gap_narrower_than_a_drone_makes_c_unreachable():
    walls = [[[10, 0], [10.2, 1.95]], [[10, 2.05], [10.2, 4]]]  # 10 cm gap
    assert error_of(walls=walls).startswith("sectors.C: can't be reached from A")


# --- forests ----------------------------------------------------------------

FOREST = {"region": [[6, 0], [12, 4]], "count": 12, "radius": [0.05, 0.12], "min_gap": 0.6}


def test_forest_grows_count_pillars_with_the_gap():
    course = course_of(forests=[{**FOREST, "seed": 3}])
    pillars = course.pillars
    assert len(pillars) == 12
    for i, p in enumerate(pillars):
        assert 6 <= p.center[0] - p.radius and p.center[0] + p.radius <= 12
        assert 0.05 <= p.radius <= 0.12
        for q in pillars[i + 1 :]:
            gap = math.dist(p.center, q.center) - p.radius - q.radius
            assert gap >= 0.6
        assert Point(p.center).distance(course.area.exterior) >= p.radius + 0.6


def test_same_seed_same_forest_other_seed_other_forest():
    first = course_of(forests=[{**FOREST, "seed": 3}]).pillars
    again = course_of(forests=[{**FOREST, "seed": 3}]).pillars
    other = course_of(forests=[{**FOREST, "seed": 4}]).pillars
    assert first == again
    assert first != other


def test_forest_that_cant_fit_says_how_many_fit():
    msg = error_of(forests=[{**FOREST, "count": 200, "seed": 1}])
    assert msg.startswith("obstacles.forests[0]: could only fit ")
    assert "of 200 pillars" in msg


def test_example_map_builds():
    course = build_course(load_map(MAPS / "l_hall.toml"))
    assert len(course.pillars) == 1 + 15
    assert len(course.walls) == 2
