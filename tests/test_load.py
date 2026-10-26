# "heigth" below is a deliberate typo: the loader must catch misspelled keys.
# cspell:ignore heigth
from pathlib import Path

import pytest

from maps import MapError, Rect, load_map, parse_map

MAPS = Path(__file__).parent.parent / "maps"


def minimal(**changes: object) -> dict[str, object]:
    """The smallest valid map, with top-level sections replaced by `changes`."""
    doc: dict[str, object] = {
        "version": 1,
        "arena": {"ceiling": 2.5, "rects": [[[0, 0], [20, 4]]]},
        "sectors": {"A": {"rect": [[0, 0], [4, 4]]}, "C": {"rect": [[16, 0], [20, 4]]}},
    }
    doc.update(changes)
    return doc


def obstacles(**lists: object) -> dict[str, object]:
    return minimal(obstacles=lists)


def error_of(doc: dict[str, object]) -> str:
    with pytest.raises(MapError) as e:
        parse_map(doc)
    return str(e.value)


# --- valid maps -------------------------------------------------------------


def test_example_map_loads():
    m = load_map(MAPS / "l_hall.toml")
    assert len(m.arena.rects) == 2
    assert len(m.walls) == 2 and len(m.pillars) == 1 and len(m.forests) == 1
    assert m.sector_a.z == (1.5, 1.5)  # one altitude until phase 4
    assert m.sector_c.z == (0.0, 2.5)  # no z: floor to ceiling


def test_minimal_map_has_no_obstacles():
    m = parse_map(minimal())
    assert (m.walls, m.pillars, m.forests) == ((), (), ())
    assert m.description == ""


def test_rect_corners_in_any_order():
    m = parse_map(minimal(arena={"ceiling": 2.5, "rects": [[[20, 4], [0, 0]]]}))
    assert m.arena.rects[0] == Rect(0, 0, 20, 4)


def test_wall_plain_pair_is_full_height_table_has_height():
    m = parse_map(
        obstacles(walls=[[[1, 0], [1.2, 4]], {"rect": [[3, 0], [3.2, 4]], "height": 1.0}])
    )
    assert m.walls[0].height is None
    assert m.walls[1].height == 1.0


def test_forest_number_means_fixed_value():
    forest = {
        "region": [[5, 0], [10, 4]],
        "count": 3,
        "radius": 0.1,
        "min_gap": 0.5,
        "seed": 1,
        "height": [0.5, 1.5],
    }
    f = parse_map(obstacles(forests=[forest])).forests[0]
    assert f.radius == (0.1, 0.1)
    assert f.height == (0.5, 1.5)


def test_sector_a_z_can_be_a_band():
    sectors = {"A": {"rect": [[0, 0], [4, 4]], "z": [0.8, 1.5]}, "C": {"rect": [[16, 0], [20, 4]]}}
    assert parse_map(minimal(sectors=sectors)).sector_a.z == (0.8, 1.5)


def test_sector_a_z_can_be_one_altitude():
    sectors = {"A": {"rect": [[0, 0], [4, 4]], "z": 1.2}, "C": {"rect": [[16, 0], [20, 4]]}}
    assert parse_map(minimal(sectors=sectors)).sector_a.z == (1.2, 1.2)


# --- errors name the field --------------------------------------------------


def test_unknown_key_suggests_the_right_one():
    msg = error_of(obstacles(pillars=[{"center": [5, 2], "radius": 0.1, "heigth": 1}]))
    assert msg == "obstacles.pillars[0].heigth: unknown key (did you mean 'height'?)"


def test_unknown_sector_is_rejected():
    sectors = {
        "A": {"rect": [[0, 0], [4, 4]]},
        "B": {"rect": [[5, 0], [6, 4]]},
        "C": {"rect": [[16, 0], [20, 4]]},
    }
    assert error_of(minimal(sectors=sectors)) == "sectors.B: unknown key"


def test_missing_section():
    doc = minimal()
    del doc["arena"]
    assert error_of(doc) == "arena: missing"


@pytest.mark.parametrize("version", [2, 0])
def test_other_versions_are_rejected(version: int):
    assert "this code reads version 1" in error_of(minimal(version=version))


def test_boolean_is_not_a_number():
    msg = error_of(minimal(arena={"ceiling": True, "rects": [[[0, 0], [20, 4]]]}))
    assert msg == "arena.ceiling: expected a number, got true/false"


def test_flat_rectangle_is_rejected():
    msg = error_of(obstacles(walls=[[[1, 0], [1, 4]]]))
    assert msg == "obstacles.walls[0]: the corners must differ in both x and y"


def test_height_above_ceiling_is_rejected():
    msg = error_of(obstacles(pillars=[{"center": [5, 2], "radius": 0.1, "height": 3}]))
    assert msg == "obstacles.pillars[0].height: must be between 0 and the ceiling (2.5), got 3.0"


def test_reversed_range_is_rejected():
    forest = {
        "region": [[5, 0], [10, 4]],
        "count": 3,
        "radius": [0.2, 0.1],
        "min_gap": 0.5,
        "seed": 1,
    }
    assert "min can't be above max" in error_of(obstacles(forests=[forest]))


def test_sector_c_z_must_be_a_range():
    sectors = {"A": {"rect": [[0, 0], [4, 4]]}, "C": {"rect": [[16, 0], [20, 4]], "z": 1.2}}
    assert error_of(minimal(sectors=sectors)) == "sectors.C.z: expected a list [...], got a number"


def test_old_x_y_sectors_point_to_rect():
    sectors = {"A": {"x": [0, 4], "y": [0, 4]}, "C": {"rect": [[16, 0], [20, 4]]}}
    msg = error_of(minimal(sectors=sectors))
    assert msg == "sectors.A: write the sector as rect = [[x1, y1], [x2, y2]], not x/y ranges"


def test_file_errors_start_with_the_file_name(tmp_path: Path):
    bad = tmp_path / "bad.toml"
    bad.write_text("version = 2\n", encoding="utf-8")
    with pytest.raises(MapError, match=r"^.*bad\.toml: arena: missing$"):
        load_map(bad)
