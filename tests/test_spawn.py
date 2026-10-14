import math
from pathlib import Path

import pytest
from shapely.geometry import Point

from maps import load_map, parse_map
from maps.course import Course, build_course
from worldgen.spawn import ALTITUDE_MARGIN, SpawnError, place_swarm

MAPS = Path(__file__).parent.parent / "maps"


@pytest.fixture(scope="module")
def course() -> Course:
    return build_course(load_map(MAPS / "l_hall.toml"))


def test_places_n_drones_inside_a(course: Course):
    swarm = place_swarm(course, n=20, j=2, seed=7)
    a = course.sector_a
    assert [d.id for d in swarm] == list(range(20))
    for d in swarm:
        x, y = d.xy
        assert a.area.x_min <= x <= a.area.x_max and a.area.y_min <= y <= a.area.y_max
        assert a.z[0] <= d.altitude <= a.z[1]
        assert -math.pi <= d.yaw <= math.pi


def test_spacing_and_clearance_are_respected(course: Course):
    swarm = place_swarm(course, n=20, j=2, seed=7, spacing=1.0, clearance=0.5)
    for i, d in enumerate(swarm):
        assert Point(d.xy).distance(course.area.boundary) >= 0.5
        for e in swarm[i + 1 :]:
            assert math.dist(d.xy, e.xy) >= 1.0


def test_exactly_j_leaders(course: Course):
    assert sum(d.leader for d in place_swarm(course, n=20, j=3, seed=7)) == 3
    assert not any(d.leader for d in place_swarm(course, n=5, j=0, seed=7))  # phase 1
    assert all(d.leader for d in place_swarm(course, n=5, j=5, seed=7))


def test_same_seed_same_swarm_other_seed_other_swarm(course: Course):
    assert place_swarm(course, n=10, j=2, seed=1) == place_swarm(course, n=10, j=2, seed=1)
    assert place_swarm(course, n=10, j=2, seed=1) != place_swarm(course, n=10, j=2, seed=2)


@pytest.mark.parametrize(
    ("n", "j", "message"),
    [
        (0, 0, "need at least one drone, got n = 0"),
        (5, 6, "the leaders must be between 0 and n = 5, got j = 6"),
        (5, -1, "the leaders must be between 0 and n = 5, got j = -1"),
    ],
)
def test_bad_n_or_j(course: Course, n: int, j: int, message: str):
    with pytest.raises(SpawnError, match=f"^{message}$"):
        place_swarm(course, n=n, j=j, seed=0)


def test_too_many_drones_for_a_says_how_many_fit(course: Course):
    with pytest.raises(SpawnError, match=r"^could only place \d+ of 200 drones in sector A"):
        place_swarm(course, n=200, j=1, seed=0)


def course_with_a_z(z: object | None) -> Course:
    a: dict[str, object] = {"x": [1, 5], "y": [1, 7]}
    if z is not None:
        a["z"] = z
    doc: dict[str, object] = {
        "version": 1,
        "arena": {"ceiling": 2.5, "rects": [[[0, 0], [20, 8]]]},
        "sectors": {"A": a, "C": {"x": [16, 20], "y": [0, 8]}},
    }
    return build_course(parse_map(doc))


def test_a_without_z_means_floor_to_ceiling_with_a_margin():
    swarm = place_swarm(course_with_a_z(None), n=15, j=1, seed=0)
    altitudes = [d.altitude for d in swarm]
    assert min(altitudes) >= ALTITUDE_MARGIN
    assert max(altitudes) <= 2.5 - ALTITUDE_MARGIN


def test_a_single_z_puts_every_drone_at_that_altitude():
    swarm = place_swarm(course_with_a_z(1.2), n=10, j=1, seed=0)
    assert {d.altitude for d in swarm} == {1.2}


def test_altitude_too_close_to_the_floor_is_rejected():
    with pytest.raises(SpawnError, match=r"^sectors\.A\.z: takeoff altitudes must be"):
        place_swarm(course_with_a_z(0.1), n=3, j=1, seed=0)
