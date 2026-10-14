"""Build a Webots world from a map: `uv run make_world.py l_hall --preview`.

See `uv run make_world.py --help`; the code lives in src/worldgen/cli.py.
"""

from pathlib import Path

from worldgen.cli import main

raise SystemExit(main(root=Path(__file__).parent))
