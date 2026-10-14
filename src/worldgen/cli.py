"""The `make_world.py` command: a map name in, a Webots world out.

    uv run make_world.py l_hall                    # maps/l_hall.toml -> worlds/l_hall.wbt
    uv run make_world.py l_hall -n 20 -j 2 -s 7    # drones, leaders, seed
    uv run make_world.py l_hall --preview          # also draw worlds/l_hall.png

`generate()` is the whole pipeline as one function, for scripts that build
many worlds; `main()` only reads the command line and reports.
"""

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

from maps import MapError, load_map
from maps.course import Course, build_course

from .runtime import write_runtime_inis
from .spawn import Drone, SpawnError, place_swarm
from .wbt import render_world


@dataclass(frozen=True)
class Generated:
    world: Path
    preview: Path | None
    course: Course
    swarm: tuple[Drone, ...]


def generate(
    map_file: Path,
    worlds: Path,
    controllers: Path,
    n: int,
    j: int,
    seed: int,
    preview: bool = False,
    name: str | None = None,
    controller: str = "drone",
    time_step: int = 32,
) -> Generated:
    """Load, check, build and write: worlds/<name>.wbt (and .png with `preview`).

    `name` defaults to the map's name; `controller` and `time_step` are for
    test tools (see `render_world`), the command line always uses the defaults.
    """
    course = build_course(load_map(map_file))
    swarm = place_swarm(course, n=n, j=j, seed=seed)
    title = f"{map_file.stem}  n={n} j={j} seed={seed}"

    worlds.mkdir(parents=True, exist_ok=True)
    world = worlds / f"{name or map_file.stem}.wbt"
    text = render_world(course, swarm, title, controller=controller, time_step=time_step)
    world.write_text(text, encoding="utf-8")
    write_runtime_inis(controllers)

    png = None
    if preview:
        from .preview import draw_preview  # matplotlib is slow to import: only when asked

        png = world.with_suffix(".png")
        draw_preview(course, swarm, title, png)
    return Generated(world=world, preview=png, course=course, swarm=swarm)


def main(argv: list[str] | None = None, root: Path | None = None) -> int:
    root = root or Path.cwd()
    args = _parser().parse_args(argv)
    map_file = _map_file(args.map, root / "maps")
    if map_file is None:
        names = ", ".join(sorted(p.stem for p in (root / "maps").glob("*.toml"))) or "none"
        print(f"error: no map called '{args.map}' (maps: {names})", file=sys.stderr)
        return 2
    try:
        result = generate(
            map_file, root / "worlds", root / "controllers",
            n=args.n, j=args.j, seed=args.seed, preview=args.preview,
        )  # fmt: skip
    except (MapError, SpawnError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    c, leaders = result.course, sum(d.leader for d in result.swarm)
    print(
        f"{result.world.relative_to(root).as_posix()}: {args.n} drones "
        f"({leaders} leader{'s' if leaders != 1 else ''}), {len(c.walls)} walls, "
        f"{len(c.pillars)} pillars"
    )
    if result.preview:
        print(f"{result.preview.relative_to(root).as_posix()}: preview")
    return 0


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="make_world.py", description="Build a Webots world from a map in maps/."
    )
    p.add_argument("map", help="map name (maps/<name>.toml) or path to a .toml file")
    p.add_argument("-n", type=int, default=10, help="number of drones (default 10)")
    p.add_argument("-j", type=int, default=1, help="number of leaders (default 1)")
    p.add_argument("-s", "--seed", type=int, default=0, help="spawn seed (default 0)")
    p.add_argument("--preview", action="store_true", help="also draw worlds/<map>.png")
    return p


def _map_file(name: str, maps: Path) -> Path | None:
    candidates = [Path(name), maps / name, maps / f"{name}.toml"]
    return next((p for p in candidates if p.suffix == ".toml" and p.is_file()), None)
