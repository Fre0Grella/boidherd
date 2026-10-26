import json
import math
import re
from pathlib import Path

import pytest

from maps import load_map, parse_map
from maps.course import Course, build_course
from worldgen.spawn import place_swarm
from worldgen.wbt import (
    CRAZYFLIE_REST_Z,
    _axis_angle,
    _num,
    _string,
    drone_marker,
    render_world,
)

MAPS = Path(__file__).parent / "maps"  # frozen copies, not the maps/ to experiment with


@pytest.fixture(scope="module")
def course() -> Course:
    return build_course(load_map(MAPS / "l_hall.toml"))


@pytest.fixture(scope="module")
def world(course: Course) -> str:
    return render_world(course, place_swarm(course, n=12, j=3, seed=5), "l_hall n=12 j=3 s=5")


def custom_data(world: str) -> list[dict[str, object]]:
    """The customData of every drone, unescaped and parsed back from JSON."""
    raw = re.findall(r'customData "((?:[^"\\]|\\.)*)"', world)
    return [json.loads(r.replace('\\"', '"').replace("\\\\", "\\")) for r in raw]


def test_starts_with_the_webots_header(world: str):
    assert world.startswith("#VRML_SIM R2025a utf8\n")
    assert world.count("EXTERNPROTO") == 4
    assert 'title "l_hall n=12 j=3 s=5"' in world


def test_braces_and_brackets_balance(world: str):
    assert world.count("{") == world.count("}")
    assert world.count("[") == world.count("]")


def test_one_crazyflie_per_drone_standing_on_the_floor(world: str):
    assert len(re.findall(r"^DEF DRONE_\d+ Crazyflie \{$", world, re.M)) == 12
    assert world.count('controller "drone"') == 12
    for z in re.findall(r"^  translation \S+ \S+ (\S+)\n  rotation 0 0 1", world, re.M):
        assert float(z) == CRAZYFLIE_REST_Z


def test_every_drone_knows_c_and_the_obstacles_and_three_lead(world: str, course: Course):
    data = custom_data(world)
    assert len(data) == 12
    c = course.sector_c.area
    assert all(d["sector_c"] == [c.x_min, c.y_min, c.x_max, c.y_max] for d in data)
    assert sum(d.get("leader") is True for d in data) == 3
    solid = (*course.boundary, *course.walls)
    walls = [[w.rect.x_min, w.rect.y_min, w.rect.x_max, w.rect.y_max] for w in solid]
    pillars = [pytest.approx([*p.center, p.radius], abs=1e-4) for p in course.pillars]
    assert all(d["walls"] == walls and d["pillars"] == pillars for d in data)
    z_min, z_max = course.sector_a.z
    assert all(z_min <= float(str(d["altitude"])) <= z_max for d in data)


def test_every_wall_and_pillar_is_a_solid(world: str, course: Course):
    solids = len(course.boundary) + len(course.walls) + len(course.pillars)
    assert world.count("Solid {") == solids
    assert world.count("boundingObject") == solids


def test_referee_gets_the_sectors(world: str):
    assert (
        'controllerArgs [ "--sector-a" "1 1 12 7" "--sector-c" "32 24 40 30" "--ceiling" "2.5" ]'
        in world
    )


def test_low_obstacles_keep_their_height():
    doc: dict[str, object] = {
        "version": 1,
        "arena": {"ceiling": 2.5, "rects": [[[0, 0], [20, 8]]]},
        "sectors": {"A": {"rect": [[1, 1], [5, 7]]}, "C": {"rect": [[16, 0], [20, 8]]}},
        "obstacles": {"pillars": [{"center": [10, 2], "radius": 0.2, "height": 1.2}]},
    }
    course = build_course(parse_map(doc))
    world = render_world(course, place_swarm(course, n=2, j=1, seed=0), "low")
    assert "Cylinder { height 1.2 radius 0.2 }" in world
    assert "translation 10 2 0.6\n" in world  # centred at half its height


def test_viewpoint_looks_at_the_centre_of_the_floor(world: str, course: Course):
    match = re.search(r"orientation (\S+) (\S+) (\S+) (\S+)\n  position (\S+) (\S+) (\S+)", world)
    assert match
    ax, ay, az, angle, *position = map(float, match.groups())
    # Rodrigues: rotate the camera's forward axis (1, 0, 0) by the orientation
    c, s = math.cos(angle), math.sin(angle)
    forward = (
        c + ax * ax * (1 - c),
        ay * ax * (1 - c) + az * s,
        az * ax * (1 - c) - ay * s,
    )
    t = -position[2] / forward[2]  # where the line of sight meets the floor
    cx, cy = course.floor.center
    assert position[0] + t * forward[0] == pytest.approx(cx, abs=0.01)
    assert position[1] + t * forward[1] == pytest.approx(cy, abs=0.01)


def test_axis_angle_of_no_rotation_is_zero():
    assert _axis_angle(0.0, 0.0) == ((0.0, 0.0, 1.0), 0.0)


@pytest.mark.parametrize(
    ("value", "text"),
    [(2.0, "2"), (1.23456, "1.2346"), (-0.00001, "0"), (0.015, "0.015"), (-3.5, "-3.5")],
)
def test_numbers_are_short(value: float, text: str):
    assert _num(value) == text


def test_strings_are_escaped():
    assert _string('say "hi" \\ bye') == '"say \\"hi\\" \\\\ bye"'


def test_every_drone_carries_a_marker_red_for_leaders(world: str):
    drones = re.split(r"^DEF DRONE_\d+ Crazyflie \{$", world, flags=re.M)[1:]
    for drone in drones:
        leader = 'leader\\":true' in drone  # in customData, quotes are escaped
        assert drone_marker(leader) in drone
    assert world.count(drone_marker(True)) == 3
    assert world.count(drone_marker(False)) == 9
    assert drone_marker(True) != drone_marker(False)


def test_sector_boxes_show_the_altitude_band():
    # sector A: 4 x 6 m, z from 0.8 to 1.5; sector C: 4 x 8 m, floor to ceiling
    a = {"rect": [[1, 1], [5, 7]], "z": [0.8, 1.5]}
    doc: dict[str, object] = {
        "version": 1,
        "arena": {"ceiling": 2.5, "rects": [[[0, 0], [20, 8]]]},
        "sectors": {"A": a, "C": {"rect": [[16, 0], [20, 8]]}},
    }
    course = build_course(parse_map(doc))
    world = render_world(course, place_swarm(course, n=2, j=1, seed=0), "band")
    assert "translation 0 0 1.15\n" in world
    assert "geometry Box { size 4 6 0.7 }" in world
    assert "translation 0 0 1.25\n" in world
    assert "geometry Box { size 4 8 2.5 }" in world
