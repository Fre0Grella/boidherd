"""Check IR neighbour sensing against the referee's ground truth.

    uv run tools/ir_test.py                 # 20 drones, IR range 3 m, 25 s
    uv run tools/ir_test.py -n 30 --duration 30

Builds a 20 x 20 m test room (written to logs/, not maps/) with the swarm in
the middle, runs it headless with the `ir_test` controller (hover, then the
whole swarm sways together, so every drone is tilted), and compares each
drone's measured neighbour offsets with the true ones:

- distance and direction errors of the measured offsets;
- detection: pairs truly within IR range that were heard, and pairs heard
  although out of range;
- whether a drone's own body blocks IR (missing pairs that depend on the
  direction would show it).

Run it from PowerShell.
"""

import argparse
import csv
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path

from headless import RunError, run_until

from worldgen.cli import generate

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / "logs"
IR_RANGE = 3.0  # m, the default in render_world
START = 9.0  # s, as in controllers/ir_test

TEST_MAP = """version = 1
description = "IR test room: written by tools/ir_test.py."

[arena]
ceiling = 3
rects = [ [[0, 0], [20, 20]] ]

[sectors]
A = { rect = [[6, 6], [14, 14]], z = 1.2 }
C = { rect = [[17, 17], [20, 20]] }
"""


def main() -> int:
    parser = argparse.ArgumentParser(description="Check IR sensing against ground truth.")
    parser.add_argument("-n", type=int, default=20, help="drones (default 20)")
    parser.add_argument("-s", "--seed", type=int, default=0)
    parser.add_argument("--duration", type=float, default=25.0, help="simulated seconds")
    parser.add_argument("--timeout", type=float, default=300, help="wall-clock seconds")
    args = parser.parse_args()

    LOGS.mkdir(exist_ok=True)
    map_file = LOGS / "ir_test_map.toml"
    map_file.write_text(TEST_MAP, encoding="utf-8")
    result = generate(
        map_file, ROOT / "worlds", ROOT / "controllers",
        n=args.n, j=0, seed=args.seed, name="ir_test", controller="ir_test",
    )  # fmt: skip
    truth_log = LOGS / "ir_test.csv"
    truth_log.unlink(missing_ok=True)
    for old in (LOGS / "ir_test").glob("drone_*.csv"):
        old.unlink()
    try:
        wall = run_until(result.world, lambda: _last_time(truth_log) >= args.duration, args.timeout)
    except RunError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(
        f"{args.n} drones, IR range {IR_RANGE} m, {args.duration:.0f} s simulated in {wall:.0f} s"
    )
    return report(truth_log, args.duration)


def report(truth_log: Path, duration: float) -> int:
    # true positions: time (ms) -> drone -> (x, y, z)
    truth: dict[int, dict[int, tuple[float, float, float]]] = defaultdict(dict)
    for row in csv.DictReader(open(truth_log, newline="")):
        truth[_ms(row["t"])][int(row["id"])] = (float(row["x"]), float(row["y"]), float(row["z"]))
    times = sorted(truth)
    step = times[1] - times[0]

    dist_err, angle_err, tilts = [], [], []
    heard: dict[int, set[tuple[int, int]]] = defaultdict(set)
    for log in sorted((LOGS / "ir_test").glob("drone_*.csv")):
        for row in csv.DictReader(open(log, newline="")):
            t = _ms(row["t"])
            if t / 1000 > duration or t - step not in truth:
                continue
            me, other = int(row["me"]), int(row["other"])
            heard[t].add((me, other))
            # the packet left the sender one step earlier
            mx, my, mz = truth[t][me]
            ox, oy, oz = truth[t - step][other]
            true = (ox - mx, oy - my, oz - mz)
            measured = (float(row["ox"]), float(row["oy"]), float(row["oz"]))
            d_true, d_meas = math.dist(true, (0, 0, 0)), math.dist(measured, (0, 0, 0))
            dist_err.append(abs(d_meas - d_true))
            cos = sum(a * b for a, b in zip(true, measured, strict=True)) / (d_true * d_meas)
            angle_err.append(math.degrees(math.acos(max(-1.0, min(1.0, cos)))))
            tilts.append(max(abs(float(row["roll"])), abs(float(row["pitch"]))))

    if not dist_err:
        print("error: no drone heard any other")
        return 1
    print(f"tilt while measuring: up to {math.degrees(max(tilts)):.1f} deg")
    print(f"distance error:  mean {100 * statistics.mean(dist_err):.1f} cm, "
          f"max {100 * max(dist_err):.1f} cm")  # fmt: skip
    print(f"direction error: mean {statistics.mean(angle_err):.2f} deg, "
          f"max {max(angle_err):.2f} deg")  # fmt: skip

    in_range = received = false = 0
    missed: dict[tuple[int, int], int] = defaultdict(int)
    for t in heard:  # only steps where the drones were logging
        if t - step not in truth:
            continue
        for me, p_me in truth[t].items():
            for other, p_other in truth[t - step].items():
                if other == me:
                    continue
                pair_heard = (me, other) in heard[t]
                if math.dist(p_me, p_other) <= IR_RANGE:
                    in_range += 1
                    received += pair_heard
                    if not pair_heard:
                        missed[(me, other)] += 1
                elif pair_heard:
                    false += 1
    rate = received / in_range if in_range else math.nan
    print(f"detection:       {received}/{in_range} in-range pairs heard ({rate:.1%}); "
          f"heard although out of range: {false}")  # fmt: skip
    if missed:
        worst = sorted(missed.items(), key=lambda kv: -kv[1])[:5]
        print("most missed pairs (listener, sender: steps): "
              + ", ".join(f"{a}<-{b}: {k}" for (a, b), k in worst))  # fmt: skip
    return 0


def _ms(t: str) -> int:
    return round(float(t) * 1000)


def _last_time(log: Path) -> float:
    try:
        with open(log, "rb") as f:
            f.seek(0, 2)
            f.seek(max(f.tell() - 200, 0))
            last = f.read().decode(errors="ignore").strip().splitlines()[-1]
        return float(last.split(",")[0])
    except (OSError, IndexError, ValueError):
        return 0.0


if __name__ == "__main__":
    raise SystemExit(main())
