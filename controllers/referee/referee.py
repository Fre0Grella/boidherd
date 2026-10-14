"""Referee (supervisor): logs every drone's true pose, for the analysis tools.

Writes logs/<world name>.csv, one row per drone per time step, flushed as
it goes so a tool can read it while Webots runs. It never quits Webots: the
tools stop the Webots they started, and in the GUI you stop it yourself.
"""

import argparse
import csv
import sys
from pathlib import Path
from typing import cast

from controller import Node, Supervisor  # pyright: ignore[reportMissingImports]

parser = argparse.ArgumentParser()
parser.add_argument("--sector-a", required=True)
parser.add_argument("--sector-c", required=True)
parser.add_argument("--ceiling", type=float, required=True)
args = parser.parse_args(sys.argv[1:])

referee = Supervisor()
time_step = int(referee.getBasicTimeStep())

drones: list[Node] = []
while (node := referee.getFromDef(f"DRONE_{len(drones)}")) is not None:
    drones.append(cast(Node, node))

logs = Path(__file__).resolve().parents[2] / "logs"
logs.mkdir(exist_ok=True)
world = Path(referee.getWorldPath()).stem

with open(logs / f"{world}.csv", "w", newline="") as f:
    log = csv.writer(f)
    log.writerow(["t", "id", "x", "y", "z", "vx", "vy", "vz"])
    while referee.step(time_step) != -1:
        t = f"{referee.getTime():.3f}"
        for i, node in enumerate(drones):
            x, y, z = node.getPosition()
            vx, vy, vz = node.getVelocity()[:3]
            log.writerow([t, i, *(f"{v:.4f}" for v in (x, y, z, vx, vy, vz))])
        f.flush()
