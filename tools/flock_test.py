"""Take off a leaderless swarm, let it flock, and measure the flock it forms.

    uv run tools/flock_test.py                  # 20 drones on the square map, 60 s
    uv run tools/flock_test.py -n 40 --duration 90 --seed 3
    uv run tools/flock_test.py --set boids.cohesion=0.5 flight.kp_tilt=2

Builds worlds/flock_test.wbt with the real `drone` controller (no leaders),
runs Webots headless until the referee has logged `--duration` simulated
seconds, then reads logs/flock_test.csv. From the moment every drone hovers
it reports, every 10 s: the number of groups (drones linked when within the
perception radius), the mean distance to the nearest neighbour, the mean
speed and the polarization (length of the mean heading: 1 = all fly the same
way). Then the closest two drones ever got, how close any came to a wall,
how far the flock's centre drifted, and crashes. Run it from PowerShell.
"""

import argparse
import csv
import itertools
import math
import sys
from collections import defaultdict
from pathlib import Path

from headless import RunError, run_until
from shapely.geometry import Point
from shapely.geometry.base import BaseGeometry

from boids import BoidParams
from util.overrides import parse
from worldgen.cli import generate
from worldgen.wbt import TIME_STEP

ROOT = Path(__file__).resolve().parents[1]
LOG = ROOT / "logs" / "flock_test.csv"
DRONE_DIAMETER = 0.1  # m, the Crazyflie's collision cylinder (radius 5 cm)
HOVER_TOLERANCE = 0.05  # m
REPORT_EVERY = 10.0  # s

State = tuple[float, float, float, float, float, float]  # x, y, z, vx, vy, vz


def main() -> int:
    parser = argparse.ArgumentParser(description="Let a leaderless swarm flock and measure it.")
    parser.add_argument("--map", default="square")
    parser.add_argument("-n", type=int, default=20, help="drones (default 20)")
    parser.add_argument("-s", "--seed", type=int, default=0)
    parser.add_argument("--duration", type=float, default=60.0, help="simulated seconds")
    parser.add_argument("--timeout", type=float, default=600, help="wall-clock seconds")
    parser.add_argument("--time-step", type=int, default=TIME_STEP, help="ms")
    parser.add_argument("--set", nargs="*", default=[], metavar="SECTION.NAME=VALUE",
                        help="override settings, e.g. boids.cohesion=0.5")  # fmt: skip
    args = parser.parse_args()
    params = parse(args.set)

    result = generate(
        ROOT / "maps" / f"{args.map}.toml", ROOT / "worlds", ROOT / "controllers",
        n=args.n, j=0, seed=args.seed, name="flock_test", time_step=args.time_step,
    )  # fmt: skip
    LOG.unlink(missing_ok=True)
    try:
        wall = run_until(result.world, lambda: _last_time() >= args.duration, args.timeout, params)
    except RunError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"{args.n} drones on '{args.map}' at {args.time_step} ms, "
          f"{args.duration:.0f} s simulated in {wall:.0f} s")  # fmt: skip
    targets = {d.id: d.altitude for d in result.swarm}
    link = BoidParams(**params.get("boids", {})).perception_radius
    return report(targets, result.course.area.exterior, link, args.duration)


def report(targets: dict[int, float], walls: BaseGeometry, link: float, duration: float) -> int:
    steps: dict[float, dict[int, State]] = defaultdict(dict)
    for row in csv.DictReader(open(LOG, newline="")):
        t = float(row["t"])
        if t <= duration:
            x, y, z, vx, vy, vz = (float(row[k]) for k in ("x", "y", "z", "vx", "vy", "vz"))
            steps[t][int(row["id"])] = (x, y, z, vx, vy, vz)
    times = sorted(steps)

    hover_at: dict[int, float] = {}
    for t in times:
        for i, s in steps[t].items():
            if i not in hover_at and abs(s[2] - targets[i]) < HOVER_TOLERANCE:
                hover_at[i] = t
    waiting = sorted(set(targets) - set(hover_at))
    if waiting:
        print(f"takeoff:   never reached their altitude: {waiting}")
        return 1
    start = max(hover_at.values())
    print(f"takeoff:   all {len(targets)} hovering by {start:.1f} s; flocking from then on")

    print("    t   groups  nearest  speed  polarization")
    next_report = start
    for t in times:
        if t >= next_report or t == times[-1]:
            s = steps[t]
            print(f"  {t:5.1f}  {len(_groups(s, link)):5d}  {_mean_nearest(s):6.2f} m"
                  f"  {_mean_speed(s):4.2f}  {_polarization(s):6.2f}")  # fmt: skip
            next_report = t + REPORT_EVERY

    closest, when, pair = math.inf, 0.0, (0, 0)
    for t in times:
        for (a, pa), (b, pb) in itertools.combinations(steps[t].items(), 2):
            d = math.dist(pa[:3], pb[:3])
            if d < closest:
                closest, when, pair = d, t, (a, b)
    touched = closest < DRONE_DIAMETER
    print(f"spacing:   closest two drones {closest:.2f} m apart (drones {pair[0]} and "
          f"{pair[1]} at {when:.1f} s){' - THEY TOUCHED' if touched else ''}")  # fmt: skip

    near_wall = min(walls.distance(Point(s[0], s[1])) for t in times for s in steps[t].values())
    print(f"walls:     closest any drone came to a wall {near_wall:.2f} m")
    first, last = _centre(steps[min(t for t in times if t >= start)]), _centre(steps[times[-1]])
    print(f"drift:     the flock's centre moved {math.dist(first, last):.2f} m while flocking")
    end = steps[times[-1]]
    crashed = sorted(i for i, s in end.items() if s[2] < 0.1)
    print("crashes:   " + (f"{crashed}" if crashed else "none"))
    return 1 if touched or crashed else 0


def _groups(s: dict[int, State], link: float) -> list[set[int]]:
    """Connected components: two drones are linked when closer than `link`."""
    unseen, groups = set(s), []
    while unseen:
        todo, group = [unseen.pop()], set()
        while todo:
            i = todo.pop()
            group.add(i)
            near = {j for j in unseen if math.dist(s[i][:3], s[j][:3]) < link}
            unseen -= near
            todo.extend(near)
        groups.append(group)
    return groups


def _mean_nearest(s: dict[int, State]) -> float:
    if len(s) < 2:
        return 0.0
    return sum(min(math.dist(a[:3], b[:3]) for j, b in s.items() if j != i)
               for i, a in s.items()) / len(s)  # fmt: skip


def _mean_speed(s: dict[int, State]) -> float:
    return sum(math.hypot(*v[3:5]) for v in s.values()) / len(s)


def _polarization(s: dict[int, State]) -> float:
    """Length of the mean horizontal heading (drones slower than 5 cm/s don't count)."""
    headings = [(v[3] / sp, v[4] / sp) for v in s.values() if (sp := math.hypot(*v[3:5])) > 0.05]
    if not headings:
        return 0.0
    return math.hypot(sum(h[0] for h in headings), sum(h[1] for h in headings)) / len(headings)


def _centre(s: dict[int, State]) -> tuple[float, float]:
    return (sum(v[0] for v in s.values()) / len(s), sum(v[1] for v in s.values()) / len(s))


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
