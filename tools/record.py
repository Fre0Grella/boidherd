"""Film one run, for the report's videos.

    uv run tools/record.py dead_end -n 20 -j 2 --seed 4 --model reynolds --plan 1
    uv run tools/record.py dead_end --view map --stills 30 60
    uv run tools/record.py l_hall --view tilted --heading 45 --duration 112

Builds the world, opens Webots with its window (a movie needs rendering),
and the referee (controllers/referee/film.py) films the run on a plain dark
floor. `--view follow`: from straight above, following the flock, never
closer than `--frame` metres across; `map`: the whole map from above, still;
`tilted`: following the flock from `--tilt` degrees above the horizon,
facing `--heading`. The referee records to `--duration` simulated seconds
and quits once the movie is written:
videos/<map>_<model>[_plan]_n<n>_j<j>_seed<seed>[_<view>].mp4 by default.
`--speed` is how many simulated seconds one second of movie shows. With
`--stills`, a frame at each of those simulated times goes next to the movie
as a PNG (needs ffmpeg). Run it from PowerShell; a Webots window you have
open is left alone.
"""

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

from headless import WEBOTS_EXE, add_settings, settings

from boids import BoidParams
from planner import PlanParams
from util.overrides import VARIABLE
from worldgen.cli import generate
from worldgen.wbt import _axis_angle

ROOT = Path(__file__).resolve().parents[1]
NAME = "record_run"  # the world, and the referee's log logs/record_run.csv
FIELD_OF_VIEW = 0.785  # rad, Webots' default Viewpoint, across the wider side of the frame


def main() -> int:
    parser = argparse.ArgumentParser(description="Film one run.", allow_abbrev=False)
    parser.add_argument("map")
    parser.add_argument("-n", type=int, default=20)
    parser.add_argument("-j", type=int, default=2)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--duration", type=float, default=83.0, help="simulated seconds")
    parser.add_argument("--speed", type=int, default=2, help="simulated seconds per movie second")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--view", choices=["follow", "map", "tilted"], default="follow")
    parser.add_argument("--frame", type=float, default=16.0, help="m across, at the closest")
    parser.add_argument("--heading", type=float, default=90, help="deg the camera faces, 90 north")
    parser.add_argument("--tilt", type=float, default=50, help="deg below the horizon, tilted")
    parser.add_argument("--stills", type=float, nargs="*", default=[], help="simulated seconds")
    parser.add_argument("--out", type=Path, help="the movie, .mp4")
    parser.add_argument("--timeout", type=float, default=1800, help="wall-clock seconds")
    add_settings(parser)
    args = parser.parse_args()
    params = settings(args)

    model = BoidParams(**params.get("boids", {})).model
    planned = "_plan" if PlanParams(**params.get("plan", {})).planning else ""
    view = "" if args.view == "follow" else f"_{args.view}"
    name = f"{args.map}_{model}{planned}_n{args.n}_j{args.j}_seed{args.seed}{view}"
    out = (args.out or ROOT / "videos" / f"{name}.mp4").resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.unlink(missing_ok=True)

    result = generate(
        ROOT / "maps" / f"{args.map}.toml", ROOT / "worlds", ROOT / "controllers",
        n=args.n, j=args.j, seed=args.seed, name=NAME,
    )  # fmt: skip
    heading = math.radians(args.heading)
    pitch = math.radians(args.tilt if args.view == "tilted" else 90)
    axis, angle = _axis_angle(heading, pitch)
    params["record"] = {
        "file": str(out),
        "width": args.width,
        "height": args.height,
        "speed": args.speed,
        "duration": args.duration,
        "orientation": [*axis, angle],
        "forward": [
            math.cos(pitch) * math.cos(heading),
            math.cos(pitch) * math.sin(heading),
            -math.sin(pitch),
        ],
        "field_of_view": FIELD_OF_VIEW,
        "min_width": args.frame,
    }
    if args.view == "map":
        floor = result.course.floor
        params["record"]["fixed"] = {
            "centre": list(floor.center),
            "width": 1.05 * max(floor.width, floor.depth * args.width / args.height),
        }
    webots = subprocess.run(
        [str(WEBOTS_EXE), "--batch", "--mode=fast", "--port=1298", str(result.world)],
        env={**os.environ, VARIABLE: json.dumps(params)},
        timeout=args.timeout,
        check=False,
    )
    if webots.returncode != 0 or not out.exists():
        print(f"no movie: Webots ended with {webots.returncode}", file=sys.stderr)
        return 1
    print(f"-> {out}")
    for t in args.stills:
        print(f"-> {still(out, t, args.speed)}")
    return 0


def still(movie: Path, t: float, speed: int) -> Path:
    """The movie's frame at simulated time `t`, as a PNG next to it."""
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg is None:
        raise SystemExit("stills need ffmpeg on the PATH")
    png = movie.with_name(f"{movie.stem}_t{t:.0f}.png")
    subprocess.run(
        [ffmpeg, "-y", "-loglevel", "error", "-ss", f"{t / speed:.2f}", "-i", str(movie),
         "-frames:v", "1", str(png)],
        check=True,
    )  # fmt: skip
    return png


if __name__ == "__main__":
    raise SystemExit(main())
