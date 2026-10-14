"""Map files: the format (model) and how to read it (load)."""

from .load import MapError, load_map, parse_map
from .model import Arena, Forest, Map, Pillar, Rect, Sector, Wall

__all__ = [
    "Arena",
    "Forest",
    "Map",
    "MapError",
    "Pillar",
    "Rect",
    "Sector",
    "Wall",
    "load_map",
    "parse_map",
]
