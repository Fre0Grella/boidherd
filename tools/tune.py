"""Tune a flocking model: every combination of a few settings, on seeds kept for tuning.

    uv run tools/tune.py --model couzin --grid couzin_weight=0.5,1,2,3 couzin_repulsion=0.4,0.6,0.8
    uv run tools/tune.py --model reynolds --grid reynolds_goal=1,2,4 --budget 3300

Each setting on the grid is a boids setting (or section.name); any other flag
(--plan 1, --velocity 0.5) holds for every run. Every combination runs on
each seed (100 and 101 by default, never used by the sweeps), seed by seed so
a tuning cut short has tried every combination once. One row per run goes to
logs/tune_<map>_<model>.csv: the combination, then the sweep's columns.
Runs already there are skipped. At the end the combinations are ranked by
the drones in C at the end of their runs, then by time to 90% in C (design
log: tuning the models). Run it from PowerShell.
"""

import argparse
import csv
import itertools
import statistics
import sys
import time
from pathlib import Path

from headless import RunError, add_settings, settings
from sweep import COLUMNS, ROOT, _append, run, summary

from boids import BoidParams
from util.overrides import Value, parse

Combination = dict[str, str]  # "boids.couzin_weight" -> "0.5", as typed


def main() -> int:
    parser = argparse.ArgumentParser(description="Tune a flocking model.", allow_abbrev=False)
    parser.add_argument("--map", default="l_hall")
    parser.add_argument("--grid", nargs="+", required=True, metavar="NAME=A,B,C",
                        help="settings to try, e.g. couzin_weight=0.5,1,2")  # fmt: skip
    parser.add_argument("-n", type=int, default=20)
    parser.add_argument("-j", type=int, default=2)
    parser.add_argument("--seeds", type=int, nargs="+", default=[100, 101])
    parser.add_argument("--duration", type=float, default=200.0, help="simulated seconds")
    parser.add_argument("--timeout", type=float, default=1800, help="wall-clock seconds per run")
    parser.add_argument("--time-step", type=int, default=16, help="ms")
    parser.add_argument("--budget", type=float, help="wall-clock seconds; no run starts after")
    add_settings(parser)
    args = parser.parse_args()
    fixed = settings(args)
    model = BoidParams(**fixed.get("boids", {})).model
    results = ROOT / "logs" / f"tune_{args.map}_{model}.csv"
    combinations = grid(args.grid)
    keys = list(combinations[0])
    columns = keys + COLUMNS
    began = time.monotonic()

    done = _done(results, keys)
    runs = [(seed, c) for seed, c in itertools.product(args.seeds, combinations)
            if (seed, *c.values()) not in done]  # fmt: skip
    print(f"{model}: {len(combinations)} combinations x {len(args.seeds)} seeds, "
          f"{len(runs)} runs to go ({len(done)} done in {results.name})", flush=True)  # fmt: skip
    for k, (seed, c) in enumerate(runs, 1):
        if args.budget is not None and time.monotonic() - began > args.budget:
            print(f"budget of {args.budget:.0f} s used up, {len(runs) - k + 1} runs left")
            break
        label = " ".join(f"{key.partition('.')[2]}={v}" for key, v in c.items())
        try:
            row = run(args.map, args.n, args.j, seed, with_combination(fixed, c),
                      args.duration, args.timeout, args.time_step)  # fmt: skip
        except RunError as e:
            print(f"[{k}/{len(runs)}] {label} seed {seed}: {e}", file=sys.stderr, flush=True)
            continue
        _append(results, {**c, **row}, columns)
        print(f"[{k}/{len(runs)}] {label} seed {seed}: {summary(row)}", flush=True)
    rank(results, keys)
    return 0


def grid(specs: list[str]) -> list[Combination]:
    """`name=a,b,c` per setting (a boids setting, or section.name): every combination."""
    axes = []
    for spec in specs:
        key, _, values = spec.partition("=")
        if not values:
            raise SystemExit(f"--grid: expected name=a,b,c, got {spec!r}")
        key = key if "." in key else f"boids.{key}"
        axes.append([(key, v) for v in values.split(",")])
    return [dict(c) for c in itertools.product(*axes)]


def with_combination(
    fixed: dict[str, dict[str, Value]], c: Combination
) -> dict[str, dict[str, Value]]:
    params = {section: dict(values) for section, values in fixed.items()}
    for section, values in parse([f"{k}={v}" for k, v in c.items()]).items():
        params.setdefault(section, {}).update(values)
    return params


def rank(results: Path, keys: list[str]) -> None:
    """Each combination's runs, best first: most drones in C, then sooner at 90%."""
    if not results.exists():
        return
    runs: dict[tuple[str, ...], list[dict[str, str]]] = {}
    for r in csv.DictReader(open(results, newline="")):
        runs.setdefault(tuple(r[k] for k in keys), []).append(r)

    def score(rows: list[dict[str, str]]) -> tuple[float, float]:
        in_c = statistics.mean(int(r["in_c"]) / int(r["n"]) for r in rows)
        arrived = [float(r["arrived"]) if r["arrived"] else 1e9 for r in rows]
        return (-in_c, statistics.median(arrived))

    print("\nbest first (in C at the end, 90% in C, closest pair, per seed):")
    for combo, rows in sorted(runs.items(), key=lambda item: score(item[1])):
        label = " ".join(f"{k.partition('.')[2]}={v}" for k, v in zip(keys, combo, strict=True))
        per_seed = " | ".join(
            f"{r['in_c']}/{r['n']} {r['arrived'] or 'never'} {r['closest']} m" for r in rows
        )
        print(f"  {label}: {per_seed}")


def _done(results: Path, keys: list[str]) -> set[tuple]:
    if not results.exists():
        return set()
    with open(results, newline="") as f:
        return {(int(r["seed"]), *(r[k] for k in keys)) for r in csv.DictReader(f)}


if __name__ == "__main__":
    raise SystemExit(main())
