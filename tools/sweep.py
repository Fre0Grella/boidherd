"""The n x j experiment: how many leaders does a flock need to reach sector C?

    uv run tools/sweep.py                       # n 10 20 40, j 0 1 2 4 8, seeds 0 1 2
    uv run tools/sweep.py -n 20 -j 0 2 --seeds 0 --duration 90
    uv run tools/sweep.py --map l_hall --budget 3300
    uv run tools/sweep.py --map l_hall --set boids.model=couzin

Runs every (n, j, seed) on a map (open by default), seed by seed so a sweep
cut short still covers every n and j, each headless until `--duration`
simulated seconds, until every drone is in C, or until a drone crashes, and
appends one row per run to logs/sweep_<map>_<model>.csv as soon as it
finishes.
Runs already in that file are skipped, so a stopped sweep picks up where it
left off; with `--budget`, no run starts once that many wall-clock seconds
have gone. Per run: when the flock started (all hovering), when 90% and when
all drones were in C, and at the last step how many were in C, the groups,
the drones in a group without a leader (they never hear where C is), the
polarization; the closest two drones ever got, how close any came to a
wall, a pillar or the arena's outline, and the crashed drones. Run it
from PowerShell.
"""

import argparse
import csv
import itertools
import math
import sys
import time
from collections import defaultdict
from pathlib import Path

from flock_test import ARRIVED, HOVER_TOLERANCE, State, _groups, _in, _polarization, solid
from headless import RunError, add_settings, run_until, settings
from shapely.geometry import Point

from boids import BoidParams
from planner import PlanParams
from util.overrides import Value
from worldgen.cli import generate
from worldgen.wbt import TIME_STEP

ROOT = Path(__file__).resolve().parents[1]
NAME = "sweep_run"  # the world; the referee logs it to logs/sweep_run.csv
LOG = ROOT / "logs" / f"{NAME}.csv"
CRASHED = 0.1  # m: a drone this low after takeoff is down
ROW_BYTES = 80  # a referee row is ~60 bytes
COLUMNS = [
    "n", "j", "seed", "time_step", "simulated", "wall", "start", "arrived", "all_in",
    "in_c", "groups", "uninformed", "polarization", "closest", "near_wall", "crashed",
]  # fmt: skip


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Sweep drones x leaders x seeds.", allow_abbrev=False
    )
    parser.add_argument("--map", default="open")
    parser.add_argument("-n", type=int, nargs="+", default=[10, 20, 40])
    parser.add_argument("-j", type=int, nargs="+", default=[0, 1, 2, 4, 8])
    parser.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    parser.add_argument("--duration", type=float, default=150.0, help="simulated seconds")
    parser.add_argument("--timeout", type=float, default=1800, help="wall-clock seconds per run")
    parser.add_argument("--time-step", type=int, default=TIME_STEP, help="ms")
    parser.add_argument("--budget", type=float, help="wall-clock seconds; no run starts after")
    add_settings(parser)
    args = parser.parse_args()
    params = settings(args)
    boids = BoidParams(**params.get("boids", {}))
    planned = "_plan" if PlanParams(**params.get("plan", {})).planning else ""
    results = ROOT / "logs" / f"sweep_{args.map}_{boids.model}{planned}.csv"
    began = time.monotonic()

    done = _done_runs(results)
    runs = [(n, j, seed) for seed, n, j in itertools.product(args.seeds, args.n, args.j)
            if (n, j, seed) not in done and j <= n]  # fmt: skip
    print(f"{len(runs)} runs to go ({len(done)} already in {results.name})", flush=True)
    for k, (n, j, seed) in enumerate(runs, 1):
        if args.budget is not None and time.monotonic() - began > args.budget:
            print(f"budget of {args.budget:.0f} s used up, {len(runs) - k + 1} runs left")
            break
        try:
            row = run(args.map, n, j, seed, params, args.duration, args.timeout, args.time_step)
        except RunError as e:
            print(f"[{k}/{len(runs)}] n {n} j {j} seed {seed}: {e}", file=sys.stderr, flush=True)
            continue
        _append(results, row)
        print(f"[{k}/{len(runs)}] n {n} j {j} seed {seed}: {summary(row)}", flush=True)
    return 0


def run(
    map_name: str,
    n: int,
    j: int,
    seed: int,
    params: dict[str, dict[str, Value]],
    duration: float,
    timeout: float,
    time_step: int,
) -> dict[str, object]:
    """One run, headless, measured: a row of COLUMNS. Raises RunError if Webots fails."""
    result = generate(
        ROOT / "maps" / f"{map_name}.toml", ROOT / "worlds", ROOT / "controllers",
        n=n, j=j, seed=seed, name=NAME, time_step=time_step,
    )  # fmt: skip
    _remove_log()
    goal = result.course.sector_c.area
    wall = run_until(result.world, lambda: _over(n, goal, duration), timeout, params)
    leaders = {d.id for d in result.swarm if d.leader}
    targets = {d.id: d.altitude for d in result.swarm}
    link = BoidParams(**params.get("boids", {})).perception_radius
    row = measure(targets, leaders, solid(result.course), goal, link)
    row.update(n=n, j=j, seed=seed, time_step=time_step, wall=round(wall))
    return row


def summary(row: dict[str, object]) -> str:
    return (
        ", ".join(
            f"{c} {row[c]}"
            for c in ("simulated", "arrived", "all_in", "in_c", "closest", "crashed")
        )
        + f" ({row['wall']} s wall)"
    )


def measure(
    targets: dict[int, float], leaders: set[int], walls, goal, link: float
) -> dict[str, object]:
    steps: dict[float, dict[int, State]] = defaultdict(dict)
    for row in csv.DictReader(open(LOG, newline="")):
        x, y, z, vx, vy, vz = (float(row[k]) for k in ("x", "y", "z", "vx", "vy", "vz"))
        steps[float(row["t"])][int(row["id"])] = (x, y, z, vx, vy, vz)
    times = [t for t in sorted(steps) if len(steps[t]) == len(targets)]

    hover_at: dict[int, float] = {}
    for t in times:
        for i, s in steps[t].items():
            if i not in hover_at and abs(s[2] - targets[i]) < HOVER_TOLERANCE:
                hover_at[i] = t
    start = max(hover_at.values()) if len(hover_at) == len(targets) else None

    arrived = all_in = None
    for t in times:
        inside = sum(_in(goal, s) for s in steps[t].values())
        if arrived is None and inside >= ARRIVED * len(targets):
            arrived = t
        if all_in is None and inside == len(targets):
            all_in = t

    closest = min(
        math.dist(a[:3], b[:3])
        for t in times
        for a, b in itertools.combinations(steps[t].values(), 2)
    )
    near_wall = min(walls.distance(Point(s[0], s[1])) for t in times for s in steps[t].values())
    end = steps[times[-1]]
    groups = _groups(end, link)
    return {
        "simulated": times[-1],
        "start": start,
        "arrived": arrived,
        "all_in": all_in,
        "in_c": sum(_in(goal, s) for s in end.values()),
        "groups": len(groups),
        "uninformed": sum(len(g) for g in groups if not g & leaders),
        "polarization": round(_polarization(end), 2),
        "closest": round(closest, 2),
        "near_wall": round(near_wall, 2),
        "crashed": " ".join(
            str(i) for i, s in sorted(end.items()) if start is not None and s[2] < CRASHED
        ),
    }


def _over(n: int, goal, duration: float) -> bool:
    """The run can stop: time is up, every drone is in C, or one has crashed after takeoff."""
    t, s = _last_step(n)
    if t is None:
        return False
    if t >= duration or all(_in(goal, v) for v in s.values()):
        return True
    return t > 10 and any(v[2] < CRASHED for v in s.values())


def _last_step(n: int) -> tuple[float | None, dict[int, State]]:
    """The latest complete time step logged so far (the last one may be half written)."""
    try:
        with open(LOG, "rb") as f:
            f.seek(0, 2)
            f.seek(max(f.tell() - 3 * n * ROW_BYTES, 0))
            lines = f.read().decode(errors="ignore").splitlines()[1:]
    except OSError:
        return None, {}
    steps: dict[float, dict[int, State]] = defaultdict(dict)
    for line in lines:
        try:
            t, i, *v = line.split(",")
            steps[float(t)][int(i)] = tuple(float(x) for x in v)  # type: ignore[assignment]
        except ValueError:
            continue
    full = [t for t in steps if len(steps[t]) == n]
    if not full:
        return None, {}
    return max(full), steps[max(full)]


def _done_runs(results: Path) -> set[tuple[int, int, int]]:
    if not results.exists():
        return set()
    with open(results, newline="") as f:
        return {(int(r["n"]), int(r["j"]), int(r["seed"])) for r in csv.DictReader(f)}


def _append(results: Path, row: dict[str, object], columns: list[str] = COLUMNS) -> None:
    new = not results.exists()
    with open(results, "a", newline="") as f:
        out = csv.DictWriter(f, columns)
        if new:
            out.writeheader()
        out.writerow({c: "" if row[c] is None else row[c] for c in columns})


def _remove_log() -> None:
    """The Webots just stopped may hold the log for a moment longer."""
    for _ in range(20):
        try:
            LOG.unlink(missing_ok=True)
            return
        except PermissionError:
            time.sleep(0.5)
    LOG.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
