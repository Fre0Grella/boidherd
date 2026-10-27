"""Take off a whole swarm and check that it reaches its altitude without touching.

    uv run tools/takeoff_test.py                # 20 drones on the square map, 25 s
    uv run tools/takeoff_test.py --map l_hall -n 40 --duration 30

Builds worlds/takeoff_test.wbt with the `hover` controller (takeoff only, no
flocking), runs Webots
headless until the referee has logged `--duration` simulated seconds, then
reads logs/takeoff_test.csv and reports per-drone takeoff times, overshoot,
the closest two drones ever got, drift and crashes. Run it from PowerShell.
"""

import argparse
import csv
import itertools
import math
import sys
from collections import defaultdict
from pathlib import Path

from headless import RunError, add_settings, run_until, settings

from worldgen.cli import generate

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / "logs" / "takeoff_test.csv"
DRONE_DIAMETER = 0.1  # m, the Crazyflie's collision cylinder (radius 5 cm)
HOVER_TOLERANCE = 0.05  # m


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Take off a swarm and check it.", allow_abbrev=False
    )
    parser.add_argument("--map", default="square")
    parser.add_argument("-n", type=int, default=20, help="drones (default 20)")
    parser.add_argument("-s", "--seed", type=int, default=0)
    parser.add_argument("--duration", type=float, default=25.0, help="simulated seconds")
    parser.add_argument("--timeout", type=float, default=300, help="wall-clock seconds")
    add_settings(parser)
    args = parser.parse_args()

    result = generate(
        ROOT / "maps" / f"{args.map}.toml", ROOT / "worlds", ROOT / "controllers",
        n=args.n, j=0, seed=args.seed, name="takeoff_test", controller="hover",
    )  # fmt: skip
    LOG.unlink(missing_ok=True)
    try:
        wall = run_until(result.world, lambda: _last_time() >= args.duration, args.timeout,
                        settings(args))  # fmt: skip
    except RunError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"{args.n} drones on '{args.map}', {args.duration:.0f} s simulated in {wall:.0f} s")
    targets = {d.id: d.altitude for d in result.swarm}
    spawns = {d.id: d.xy for d in result.swarm}
    return report(targets, spawns, args.duration)


def report(
    targets: dict[int, float], spawns: dict[int, tuple[float, float]], duration: float
) -> int:
    track: dict[int, list[tuple[float, float, float, float]]] = defaultdict(list)
    for row in csv.DictReader(open(LOG, newline="")):
        if float(row["t"]) <= duration:
            track[int(row["id"])].append(
                (float(row["t"]), float(row["x"]), float(row["y"]), float(row["z"]))
            )

    hover_at: dict[int, float] = {}
    for i, samples in track.items():
        hovering = [t for t, _, _, z in samples if abs(z - targets[i]) < HOVER_TOLERANCE]
        if hovering:
            hover_at[i] = hovering[0]
    waiting = sorted(set(track) - set(hover_at))
    if hover_at:
        times = sorted(hover_at.values())
        print(f"takeoff:   {len(hover_at)}/{len(track)} hovering; first at {times[0]:.1f} s, "
              f"last at {times[-1]:.1f} s")  # fmt: skip
    if waiting:
        print(f"           never reached their altitude: {waiting}")

    overshoot = max(max(z for *_, z in s) - targets[i] for i, s in track.items())
    settled = [(i, z) for i, s in track.items() for t, _, _, z in s if t >= duration - 5]
    error = max(abs(z - targets[i]) for i, z in settled)
    print(f"altitude:  max overshoot {1000 * overshoot:.0f} mm; "
          f"max error in the last 5 s {1000 * error:.0f} mm")  # fmt: skip

    closest, when, pair = math.inf, 0.0, (0, 0)
    steps = list(zip(*track.values(), strict=False))  # the same time step for every drone
    ids = list(track)
    for step in steps:
        for (a, pa), (b, pb) in itertools.combinations(zip(ids, step, strict=True), 2):
            d = math.dist(pa[1:], pb[1:])
            if d < closest:
                closest, when, pair = d, pa[0], (a, b)
    touched = closest < DRONE_DIAMETER
    print(f"spacing:   closest two drones {closest:.2f} m apart (drones {pair[0]} and "
          f"{pair[1]} at {when:.1f} s){' - THEY TOUCHED' if touched else ''}")  # fmt: skip

    drift = max(math.dist(spawns[i], s[-1][1:3]) for i, s in track.items())
    print(f"drift:     furthest any drone moved sideways from its spawn point {drift:.2f} m")
    crashed = [i for i, s in track.items() if i in hover_at and s[-1][3] < 0.1]
    print("crashes:   " + (f"{crashed}" if crashed else "none"))
    return 1 if waiting or touched or crashed else 0


def _last_time() -> float:
    """Simulated time of the last row the referee has logged so far."""
    try:
        with open(LOG, "rb") as f:
            f.seek(0, 2)
            f.seek(max(f.tell() - 200, 0))
            last = f.read().decode(errors="ignore").strip().splitlines()[-1]
        return float(last.split(",")[0])
    except (OSError, IndexError, ValueError):
        return 0.0


if __name__ == "__main__":
    raise SystemExit(main())
