"""IR test: hover, then sway back and forth while logging what IR says about the others.

Used by tools/ir_test.py. Every drone follows the same acceleration at the
same time, so the swarm moves as one rigid block: spacing never changes,
but every drone is tilted, which tests the rotation of IR directions from
the body frame into the world frame. Each drone writes
logs/ir_test/drone_<id>.csv: time, who it heard, and the measured offset.
"""

import csv
import math
from pathlib import Path

from flight import Autopilot, Phase
from flight.crazyflie import Crazyflie
from flight.radio import encode
from flight.sensing import neighbours
from util import Vec3

START = 9.0  # s, every drone is hovering by then (takeoff takes ~6 s)
SWAY = 0.6  # m/s^2: tilts the drones by about 3.5 degrees
OMEGA = 0.8  # rad/s
# a = SWAY (cos wt, cos 1.5wt): from rest, each axis goes out and back with no
# drift, at most 2 SWAY / w^2 = 1.9 m along x and 0.8 m along y

drone = Crazyflie()
pilot = Autopilot(altitude=float(str(drone.config["altitude"])))
out = Path(__file__).resolve().parents[2] / "logs" / "ir_test"
out.mkdir(parents=True, exist_ok=True)
last_time = drone.time

with open(out / f"drone_{drone.id}.csv", "w", newline="") as f:
    log = csv.writer(f)
    log.writerow(["t", "me", "other", "ox", "oy", "oz", "roll", "pitch"])
    while drone.step():
        t = drone.time
        now = drone.attitude()
        drone.broadcast(encode(drone.id, drone.velocity()))
        heard = neighbours(drone.id, drone.receptions(), now)

        command = None
        if t >= START and pilot.phase in (Phase.HOVERING, Phase.FOLLOWING):
            wt = OMEGA * (t - START)
            command = Vec3(SWAY * math.cos(wt), SWAY * math.cos(1.5 * wt), 0)
        drone.set_motors(pilot.step(t - last_time, now, command))
        last_time = t

        if t >= START:
            for n in heard:
                log.writerow([f"{t:.3f}", drone.id, n.id,
                              *(f"{v:.4f}" for v in (n.offset.x, n.offset.y, n.offset.z)),
                              f"{now.roll:.4f}", f"{now.pitch:.4f}"])  # fmt: skip
            f.flush()
