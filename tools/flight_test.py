"""Fly one drone through a fixed list of wanted velocities and report how well it follows.

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

from headless import RunError, add_settings, run_until, settings

from flight import FlightParams
from worldgen.cli import generate
from worldgen.wbt import TIME_STEP

ROOT = Path(__file__).resolve().parents[1]
LOGS = ROOT / "logs"

# script windows (s from the script start) that open with a step in the wanted velocity
VELOCITY_STEPS = [
    (0.0, 4.0, "+x 0.5 m/s"),
    (4.0, 7.0, "stop"),
    (7.0, 11.0, "+y 1 m/s"),
    (11.0, 15.0, "reversal"),
    (15.0, 20.0, "stop"),
]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fly one drone through fixed velocities.", allow_abbrev=False
    )
    parser.add_argument("--time-step", type=int, default=TIME_STEP, help="ms")
    parser.add_argument("--seed", type=int, default=3, help="spawn seed (sets the yaw)")
    parser.add_argument("--timeout", type=float, default=180, help="wall-clock seconds")
    add_settings(parser)
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
        wall = run_until(result.world, done.exists, args.timeout, settings(args))
    except RunError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    print(f"ran in {wall:.0f} s wall clock")
    report(drone.altitude, FlightParams(**settings(args).get("flight", {})))
    return 0


def report(target: float, p: FlightParams) -> None:
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

    print(f"follow:  velocity steps (tau {p.velocity_tau} s, a_max {p.max_acceleration} m/s^2);"
          " ideal = a point mass with the same tau and a_max")  # fmt: skip
    previous = (0.0, 0.0)
    for first, last, label in VELOCITY_STEPS:
        window = [r for r in follow if first <= float(r["t"]) - t0 < last]
        wanted = (float(window[-1]["vx_cmd"]), float(window[-1]["vy_cmd"]))
        print(velocity_step(label, window, previous, wanted, p))
        previous = wanted
    print(f"motors:  {min(float(r['motor_min']) for r in follow):.1f} to "
          f"{max(float(r['motor_max']) for r in follow):.1f} rad/s while following")  # fmt: skip
    hold = [r for r in follow if float(r["t"]) - t0 >= 17.0]
    moved = math.hypot(float(hold[-1]["x"]) - float(hold[0]["x"]),
                       float(hold[-1]["y"]) - float(hold[0]["y"]))  # fmt: skip
    tilt = max(max(abs(float(r["roll"])), abs(float(r["pitch"]))) for r in follow)
    z_err = max(abs(float(r["z"]) - target) for r in follow)
    yaw_change = abs(float(follow[-1]["yaw"]) - float(follow[0]["yaw"]))
    print(f"hold:    drifted {moved:.2f} m in the last 3 s")
    print(f"overall: max tilt {math.degrees(tilt):.1f} deg, altitude error max "
          f"{1000 * z_err:.0f} mm, yaw changed {math.degrees(yaw_change):.1f} deg")  # fmt: skip
    crashed = any(float(r["z"]) < 0.1 for r in follow)
    print("crash:   " + ("YES, the drone hit the floor" if crashed else "none"))


def velocity_step(
    label: str,
    window: list[dict[str, float | str]],
    before: tuple[float, float],
    after: tuple[float, float],
    p: FlightParams,
) -> str:
    """How the velocity followed a step from `before` to `after`, measured along the change:
    time to 63% and 90% (measured and ideal), overshoot, and the error at the window's end."""
    gap = (after[0] - before[0], after[1] - before[1])
    size = math.hypot(*gap)
    unit = (gap[0] / size, gap[1] / size)
    t = [float(r["t"]) - float(window[0]["t"]) for r in window]
    share = [((float(r["vx"]) - before[0]) * unit[0] + (float(r["vy"]) - before[1]) * unit[1])
             / size for r in window]  # fmt: skip
    ideal = _ideal(size, t, p)

    def reach(s: list[float], level: float) -> float:
        return next((ti for ti, si in zip(t, s, strict=True) if si >= level), math.inf)

    end_error = math.hypot(float(window[-1]["vx"]) - after[0], float(window[-1]["vy"]) - after[1])
    return (f"  {label:<11} {size:.1f} m/s: 63% after {reach(share, 0.63):.2f} s "
            f"(ideal {reach(ideal, 0.63):.2f}), 90% after {reach(share, 0.9):.2f} s "
            f"(ideal {reach(ideal, 0.9):.2f}), overshoot {100 * max(max(share) - 1, 0):.0f}%, "
            f"off by {end_error:.3f} m/s at the end")  # fmt: skip


def _ideal(size: float, t: list[float], p: FlightParams) -> list[float]:
    """Share of the step a point mass has covered at each time: dv/dt = min(gap/tau, a_max)."""
    v, out, last = 0.0, [], 0.0
    for ti in t:
        dt, last = ti - last, ti
        v += min((size - v) / p.velocity_tau, p.max_acceleration) * dt
        out.append(v / size)
    return out


if __name__ == "__main__":
    raise SystemExit(main())
