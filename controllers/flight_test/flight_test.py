"""Flight test: one drone takes off, hovers, then follows a fixed list of accelerations.

Used by tools/flight_test.py, which builds the world and reads the log. The
accelerations are in the world frame, so the drone (spawned at any yaw) must
move along world x and y whatever way it faces. It writes logs/flight_test_telemetry.csv
and, when the script is over, logs/flight_test.done; it never quits Webots.
"""

import csv
from pathlib import Path

from flight import Autopilot, Phase
from flight.crazyflie import Crazyflie
from util import Vec3

# (seconds from the start of the script, acceleration) - each holds until the next
SCRIPT = [
    (0.0, Vec3(0.5, 0, 0)),  # speed up along +x to 1 m/s
    (2.0, Vec3()),  # coast: no drag, so the speed should stay
    (5.0, Vec3(-0.5, 0, 0)),  # brake back to 0
    (7.0, Vec3(0, 0.5, 0)),  # speed up along +y
    (9.0, Vec3(0, -0.5, 0)),  # brake
    (11.0, Vec3()),  # hold: how much does it drift?
]
SCRIPT_END = 16.0
SETTLE = 2.0  # seconds of hovering before the script starts

logs = Path(__file__).resolve().parents[2] / "logs"
logs.mkdir(exist_ok=True)
(logs / "flight_test.done").unlink(missing_ok=True)

drone = Crazyflie()
pilot = Autopilot(altitude=float(str(drone.config["altitude"])))
hover_since: float | None = None
last_time = drone.time

with open(logs / "flight_test_telemetry.csv", "w", newline="") as f:
    log = csv.writer(f)
    log.writerow(["t", "phase", "x", "y", "z", "vx", "vy", "vz",
                  "roll", "pitch", "yaw", "ax_cmd", "ay_cmd", "motor"])  # fmt: skip
    while drone.step():
        t = drone.time
        now = drone.attitude()
        if pilot.phase is Phase.HOVERING and hover_since is None:
            hover_since = t
        command = None
        if hover_since is not None and t - hover_since >= SETTLE:
            elapsed = t - hover_since - SETTLE
            command = next(a for start, a in reversed(SCRIPT) if elapsed >= start)
            if elapsed >= SCRIPT_END and not (logs / "flight_test.done").exists():
                (logs / "flight_test.done").write_text(f"{t:.3f}\n")
        motors = pilot.step(t - last_time, now, command)
        drone.set_motors(motors)
        last_time = t

        p, v, c = drone.position(), drone.velocity(), command or Vec3()
        log.writerow([f"{t:.3f}", pilot.phase.value,
                      *(f"{x:.4f}" for x in (p.x, p.y, p.z, v.x, v.y, v.z)),
                      *(f"{a:.4f}" for a in (now.roll, now.pitch, now.yaw, c.x, c.y)),
                      f"{sum(motors) / 4:.2f}"])  # fmt: skip
        f.flush()
