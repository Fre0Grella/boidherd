"""Run a Webots world without a window until a condition holds, then stop it.

Shared by the test tools. Start them from PowerShell: Webots crashes when
started from Git Bash. Only the Webots started here is stopped, together
with its controllers; a Webots window you have open is left alone.
"""

import argparse
import json
import os
import subprocess
import time
from collections.abc import Callable
from pathlib import Path

from boids import BoidParams
from flight import FlightParams
from util.overrides import VARIABLE, Value, add_flags, from_flags

SETTINGS = {"boids": BoidParams, "flight": FlightParams}
ALIASES = {"velocity": "boids.v_flock"}  # the cruise speed, by the name one reaches for

WEBOTS = Path(os.environ.get("WEBOTS_HOME", r"C:\Program Files\Webots"))
WEBOTS_EXE = WEBOTS / "msys64" / "mingw64" / "bin" / "webots.exe"


class RunError(Exception):
    pass


def run_until(
    world: Path,
    done: Callable[[], bool],
    timeout: float,
    params: dict[str, dict[str, Value]] | None = None,
    port: int = 1297,
) -> float:
    """Run `world` (fast, no rendering) until `done()` is true; return the wall-clock seconds.

    `params` overrides settings in every controller (see util/overrides.py)."""
    env = {**os.environ, VARIABLE: json.dumps(params or {})}
    webots = subprocess.Popen(
        [str(WEBOTS_EXE), "--batch", "--mode=fast", "--no-rendering", "--minimize",
         f"--port={port}", str(world)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env,
    )  # fmt: skip
    started = time.monotonic()
    try:
        while not done():
            if time.monotonic() - started > timeout:
                raise RunError(f"timed out after {timeout:.0f} s")
            if webots.poll() is not None:
                raise RunError("Webots stopped before the end of the test")
            time.sleep(0.5)
    finally:
        subprocess.run(["taskkill", "/PID", str(webots.pid), "/T", "/F"], capture_output=True)
    return time.monotonic() - started


def add_settings(parser: argparse.ArgumentParser) -> None:
    """Every boids and flight setting as a flag (--model couzin --velocity 0.5), and --set
    section.name=value."""
    parser.add_argument("--set", nargs="*", default=[], metavar="SECTION.NAME=VALUE",
                        help="override settings, e.g. boids.cohesion=0.5")  # fmt: skip
    add_flags(parser, SETTINGS, ALIASES)
    parser.formatter_class = argparse.RawDescriptionHelpFormatter
    parser.usage = "%(prog)s [options] [--SETTING X ...]"
    parser.epilog = (
        "Any setting above can be changed for this run, by its name:\n"
        "  --model couzin                 the flocking model: hybrid, couzin, reynolds\n"
        "  --velocity 0.5                 the cruise speed (v_flock), m/s\n"
        "  --cohesion 0.5 --kp-angle 8    any boids or flight setting\n"
        "  --set boids.cohesion=0.5       the same, section.name=value"
    )


def settings(args: argparse.Namespace) -> dict[str, dict[str, Value]]:
    """The settings the command line asked for, by section."""
    return from_flags(args, args.set)
