"""Read a map file into a `Map`, checking every field on the way.

Only checks that need nothing but the field itself (and the ceiling) live
here: types, unknown keys, positive sizes, ranges in order. Checks that need
geometry (sectors inside the arena, obstacles clear of A and C, a connected
arena) belong to the course builder.

Every error names the field it is about, e.g.
    maps/l_hall.toml: obstacles.walls[1].heigth: unknown key (did you mean 'height'?)
"""

import difflib
import tomllib
from collections.abc import Callable
from pathlib import Path
from typing import TypeVar

from .model import Arena, Forest, Interval, Map, Pillar, Point, Rect, Sector, Wall

VERSION = 1

T = TypeVar("T")


class MapError(Exception):
    """A map file that can't be used, with a message saying where and why."""


def load_map(file: Path) -> Map:
    try:
        doc = tomllib.loads(file.read_text(encoding="utf-8"))
        return parse_map(doc)
    except tomllib.TOMLDecodeError as e:
        raise MapError(f"{file}: not valid TOML: {e}") from None
    except MapError as e:
        raise MapError(f"{file}: {e}") from None


def parse_map(doc: dict[str, object]) -> Map:
    doc = _table(
        doc, "", required={"version", "arena", "sectors"}, optional={"description", "obstacles"}
    )
    version = _integer(doc["version"], "version")
    if version != VERSION:
        raise MapError(f"version: this code reads version {VERSION}, not {version}")

    arena = _arena(doc["arena"], "arena")
    sectors = _table(doc["sectors"], "sectors", required={"A", "C"})
    obstacles = _table(
        doc.get("obstacles", {}), "obstacles", optional={"walls", "pillars", "forests"}
    )
    ceiling = arena.ceiling
    return Map(
        description=_string(doc.get("description", ""), "description"),
        arena=arena,
        sector_a=_sector(sectors["A"], "sectors.A", ceiling),
        sector_c=_sector(sectors["C"], "sectors.C", ceiling),
        walls=_items(obstacles.get("walls", []), "obstacles.walls", _wall, ceiling),
        pillars=_items(obstacles.get("pillars", []), "obstacles.pillars", _pillar, ceiling),
        forests=_items(obstacles.get("forests", []), "obstacles.forests", _forest, ceiling),
    )


# --- sections ---------------------------------------------------------------


def _arena(value: object, path: str) -> Arena:
    t = _table(value, path, required={"ceiling", "rects"})
    rects = _list(t["rects"], f"{path}.rects")
    if not rects:
        raise MapError(f"{path}.rects: needs at least one rectangle")
    return Arena(
        ceiling=_positive(t["ceiling"], f"{path}.ceiling"),
        rects=tuple(_rect(r, f"{path}.rects[{i}]") for i, r in enumerate(rects)),
    )


def _sector(value: object, path: str, ceiling: float) -> Sector:
    t = _table(value, path, required={"x", "y"}, optional={"z"})
    x = _interval(t["x"], f"{path}.x")
    y = _interval(t["y"], f"{path}.y")
    z = _height_interval(t["z"], f"{path}.z", ceiling) if "z" in t else None
    return Sector(area=Rect(x[0], y[0], x[1], y[1]), z=z)


def _wall(value: object, path: str, ceiling: float) -> Wall:
    if isinstance(value, list):  # plain corner pair: full height
        return Wall(rect=_rect(value, path), height=None)
    t = _table(value, path, required={"rect"}, optional={"height"})
    return Wall(
        rect=_rect(t["rect"], f"{path}.rect"),
        height=_height(t.get("height"), f"{path}.height", ceiling),
    )


def _pillar(value: object, path: str, ceiling: float) -> Pillar:
    t = _table(value, path, required={"center", "radius"}, optional={"height"})
    return Pillar(
        center=_point(t["center"], f"{path}.center"),
        radius=_positive(t["radius"], f"{path}.radius"),
        height=_height(t.get("height"), f"{path}.height", ceiling),
    )


def _forest(value: object, path: str, ceiling: float) -> Forest:
    t = _table(
        value, path, required={"region", "count", "radius", "min_gap", "seed"}, optional={"height"}
    )
    count = _integer(t["count"], f"{path}.count")
    if count < 1:
        raise MapError(f"{path}.count: must be at least 1, got {count}")
    radius = _number_or_range(t["radius"], f"{path}.radius")
    if radius[0] <= 0:
        raise MapError(f"{path}.radius: must be positive, got {radius[0]}")
    min_gap = _number(t["min_gap"], f"{path}.min_gap")
    if min_gap < 0:
        raise MapError(f"{path}.min_gap: can't be negative, got {min_gap}")
    height = None
    if "height" in t:
        height = _number_or_range(t["height"], f"{path}.height")
        _check_height(height[0], f"{path}.height", ceiling)
        _check_height(height[1], f"{path}.height", ceiling)
    return Forest(
        region=_rect(t["region"], f"{path}.region"),
        count=count,
        radius=radius,
        min_gap=min_gap,
        seed=_integer(t["seed"], f"{path}.seed"),
        height=height,
    )


# --- shapes -----------------------------------------------------------------


def _rect(value: object, path: str) -> Rect:
    corners = _list(value, path)
    if len(corners) != 2:
        raise MapError(f"{path}: a rectangle is two opposite corners [[x1, y1], [x2, y2]]")
    (x1, y1), (x2, y2) = _point(corners[0], f"{path}[0]"), _point(corners[1], f"{path}[1]")
    if x1 == x2 or y1 == y2:
        raise MapError(f"{path}: the corners must differ in both x and y")
    return Rect(min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))


def _point(value: object, path: str) -> Point:
    x, y = _numbers(value, path, 2, "a point is [x, y]")
    return (x, y)


def _interval(value: object, path: str) -> Interval:
    lo, hi = _numbers(value, path, 2, "expected [min, max]")
    if lo >= hi:
        raise MapError(f"{path}: min must be below max, got [{lo}, {hi}]")
    return (lo, hi)


def _number_or_range(value: object, path: str) -> Interval:
    """A single number n means the range [n, n]."""
    if not isinstance(value, list):
        n = _number(value, path)
        return (n, n)
    lo, hi = _numbers(value, path, 2, "expected a number or [min, max]")
    if lo > hi:
        raise MapError(f"{path}: min can't be above max, got [{lo}, {hi}]")
    return (lo, hi)


def _height(value: object, path: str, ceiling: float) -> float | None:
    if value is None:
        return None
    h = _number(value, path)
    _check_height(h, path, ceiling)
    return h


def _height_interval(value: object, path: str, ceiling: float) -> Interval:
    lo, hi = _interval(value, path)
    _check_height(lo, path, ceiling, allow_zero=True)
    _check_height(hi, path, ceiling)
    return (lo, hi)


def _check_height(h: float, path: str, ceiling: float, allow_zero: bool = False) -> None:
    if h < 0 or (h == 0 and not allow_zero) or h > ceiling:
        raise MapError(f"{path}: must be between 0 and the ceiling ({ceiling}), got {h}")


# --- TOML values ------------------------------------------------------------


def _table(
    value: object,
    path: str,
    required: frozenset[str] | set[str] = frozenset(),
    optional: frozenset[str] | set[str] = frozenset(),
) -> dict[str, object]:
    where = f"{path}: " if path else ""
    if not isinstance(value, dict):
        raise MapError(f"{where}expected a table {{...}}, got {_kind(value)}")
    for key in value:
        if key not in required | optional:
            hint = difflib.get_close_matches(key, sorted(required | optional), n=1)
            suggestion = f" (did you mean '{hint[0]}'?)" if hint else ""
            raise MapError(f"{_join(path, key)}: unknown key{suggestion}")
    for key in sorted(required):
        if key not in value:
            raise MapError(f"{_join(path, key)}: missing")
    return value


def _items(
    value: object, path: str, parse: Callable[[object, str, float], T], ceiling: float
) -> tuple[T, ...]:
    return tuple(parse(item, f"{path}[{i}]", ceiling) for i, item in enumerate(_list(value, path)))


def _list(value: object, path: str) -> list[object]:
    if not isinstance(value, list):
        raise MapError(f"{path}: expected a list [...], got {_kind(value)}")
    return value


def _numbers(value: object, path: str, count: int, shape: str) -> list[float]:
    items = _list(value, path)
    if len(items) != count:
        raise MapError(f"{path}: {shape}")
    return [_number(item, f"{path}[{i}]") for i, item in enumerate(items)]


def _number(value: object, path: str) -> float:
    # bool is a subclass of int in Python: `true` must not pass as 1
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MapError(f"{path}: expected a number, got {_kind(value)}")
    return float(value)


def _positive(value: object, path: str) -> float:
    n = _number(value, path)
    if n <= 0:
        raise MapError(f"{path}: must be positive, got {n}")
    return n


def _integer(value: object, path: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise MapError(f"{path}: expected a whole number, got {_kind(value)}")
    return value


def _string(value: object, path: str) -> str:
    if not isinstance(value, str):
        raise MapError(f"{path}: expected text in quotes, got {_kind(value)}")
    return value


def _kind(value: object) -> str:
    names = {
        bool: "true/false",
        int: "a number",
        float: "a number",
        str: "text",
        list: "a list",
        dict: "a table",
    }
    return names.get(type(value), type(value).__name__)


def _join(path: str, key: str) -> str:
    return f"{path}.{key}" if path else key
