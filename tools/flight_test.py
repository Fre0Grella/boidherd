"""Fly one drone through a fixed list of accelerations and report how well it follows.

    uv run tools/flight_test.py                 # the project's 16 ms time step
    uv run tools/flight_test.py --time-step 32  # Webots' default

Builds worlds/flight_test.wbt (map `square`, one drone, controller
`flight_test`), runs Webots headless until the controller writes
logs/flight_test.done, stops that Webots, and prints the numbers from
logs/flight_test_telemetry.csv. Run it from PowerShell: Webots crashes when started
from Git Bash.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

from headless import RunError, run_until

from util.overrides import parse
from worldgen.cli import generate
from worldgen.wbt import TIME_STEP

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / "logs"

# script times (s from the script start) and the world velocity expected there
CHECKPOINTS = [
    (2.0, (1.0, 0.0), "after +x push"),
    (5.0, (1.0, 0.0), "end of coast"),
    (7.0, (0.0, 0.0), "after braking"),
    (9.0, (0.0, 1.0), "after +y push"),
    (11.0, (0.0, 0.0), "after braking"),
]

# script windows (s from the script start) whose start is a step in the wanted tilt
TILT_STEPS = [(0.0, 2.0, "gentle push"), (16.0, 17.0, "hard push"), (17.0, 18.0, "reversal")]


def main() -> int:
    parser = argparse.ArgumentParser(description="Fly one drone through fixed accelerations.")
    parser.add_argument("--time-step", type=int, default=TIME_STEP, help="ms")
    parser.add_argument("--seed", type=int, default=3, help="spawn seed (sets the yaw)")
    parser.add_argument("--timeout", type=float, default=180, help="wall-clock seconds")
    parser.add_argument("--set", nargs="*", default=[], metavar="SECTION.NAME=VALUE",
                        help="override settings, e.g. flight.kp_tilt=2")  # fmt: skip
    args = parser.parse_args()

    result = generate(
        ROOT / "maps" / "square.toml", ROOT / "worlds", ROOT / "controllers",
        n=1, j=0, seed=args.seed, name="flight_test",
        controller="flight_test", time_step=args.time_step,
    )  # fmt: skip
    drone = result.swarm[0]
    print(f"drone yaw {math.degrees(drone.yaw):.0f} deg, target altitude {drone.altitude:.2f} m")

    done = LOGS / "flight_test.done"
    done.unlink(missing_ok=True)
    try:
        wall = run_until(result.world, done.exists, args.timeout, parse(args.set))
    except RunError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"ran in {wall:.0f} s wall clock")
    report(drone.altitude)
    return 0


def report(target: float) -> None:
    rows = list(csv.DictReader(open(LOGS / "flight_test_telemetry.csv", newline="")))
    f = [{k: (float(v) if k != "phase" else v) for k, v in r.items()} for r in rows]
    hover = [r for r in f if r["phase"] == "hovering"]
    follow = [r for r in f if r["phase"] == "following"]
    if not hover or not follow:
        print("error: the drone never hovered or never followed")
        return
    t_hover, t0 = float(hover[0]["t"]), float(follow[0]["t"])
    print(f"takeoff: hovering after {t_hover:.1f} s")
    err = [abs(float(r["z"]) - target) for r in hover]
    drift = [math.hypot(float(r["vx"]), float(r["vy"])) for r in hover]
    print(f"hover:   altitude error mean {1000 * sum(err) / len(err):.0f} mm, "
          f"max {1000 * max(err):.0f} mm; horizontal speed max {max(drift):.2f} m/s")  # fmt: skip

    def at(seconds: float) -> dict[str, float]:
        return min(f, key=lambda r: abs(float(r["t"]) - (t0 + seconds)))  # type: ignore[arg-type]

    print("follow:  world velocity (vx, vy) m/s, expected vs measured")
    for seconds, (ex, ey), label in CHECKPOINTS:
        r = at(seconds)
        print(f"  t+{seconds:>4.1f} s {label:<14} expected ({ex:+.2f}, {ey:+.2f})  "
              f"measured ({float(r['vx']):+.2f}, {float(r['vy']):+.2f})")  # fmt: skip
    for first, last, label in TILT_STEPS:
        window = [r for r in follow if first <= float(r["t"]) - t0 < last]
        for axis in ("roll", "pitch"):
            middle = window[len(window) // 2]  # the window's command, after the step
            step = [
                r
                for r in window
                if (r["ax_cmd"], r["ay_cmd"]) == (middle["ax_cmd"], middle["ay_cmd"])
            ]
            wanted = float(middle[f"{axis}_cmd"])  # moves a little with yaw: the middle one
            times = [float(r["t"]) - float(step[0]["t"]) for r in step]
            print(tilt_step(label, axis, times, wanted, [float(r[axis]) for r in step]))
    print(f"motors:  {min(float(r['motor_min']) for r in follow):.1f} to "
          f"{max(float(r['motor_max']) for r in follow):.1f} rad/s while following")  # fmt: skip
    start, end = at(11.0), at(16.0)
    moved = math.hypot(float(end["x"]) - float(start["x"]), float(end["y"]) - float(start["y"]))
    tilt = max(max(abs(float(r["roll"])), abs(float(r["pitch"]))) for r in follow)
    z_err = max(abs(float(r["z"]) - target) for r in follow)
    yaw_change = abs(float(follow[-1]["yaw"]) - float(follow[0]["yaw"]))
    print(f"hold:    drifted {moved:.2f} m in the last 5 s")
    print(f"overall: max tilt {math.degrees(tilt):.1f} deg, altitude error max "
          f"{1000 * z_err:.0f} mm, yaw changed {math.degrees(yaw_change):.1f} deg")  # fmt: skip
    crashed = any(float(r["z"]) < 0.1 for r in follow)
    print("crash:   " + ("YES, the drone hit the floor" if crashed else "none"))


def tilt_step(label: str, axis: str, t: list[float], target: float, measured: list[float]) -> str:
    """How one tilt axis followed a step to `target`: time to 90%, overshoot, and where it
    sat over the last 0.3 s of the window, all relative to the step."""
    if abs(target) < 1e-3:
        return f"tilt:    {label}, {axis}: no step on this axis"
    share = [m / target for m in measured]
    rise = next((ti for ti, s in zip(t, share, strict=True) if s >= 0.9), math.inf)
    late = [s for ti, s in zip(t, share, strict=True) if ti >= t[-1] - 0.3]
    return (f"tilt:    {label}, {axis} step to {math.degrees(target):+.1f} deg: "
            f"90% after {rise:.2f} s, overshoot {100 * max(max(share) - 1, 0):.0f}%, "
            f"settles at {100 * sum(late) / len(late):.0f}%")  # fmt: skip


if __name__ == "__main__":
    raise SystemExit(main())
