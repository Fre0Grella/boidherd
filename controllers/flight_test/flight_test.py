"""Flight test: one drone takes off, hovers, then follows a fixed list of accelerations.

Used by tools/flight_test.py, which builds the world and reads the log. The
accelerations are in the world frame, so the drone (spawned at any yaw) must
move along world x and y whatever way it faces. It writes logs/flight_test_telemetry.csv
and, when the script is over, logs/flight_test.done; it never quits Webots.
"""

import csv
from pathlib import Path

from flight import Autopilot, FlightParams, Phase, tilt_for
from flight.crazyflie import Crazyflie
from util import Vec3
from util.overrides import overrides

# (seconds from the start of the script, acceleration) - each holds until the next
SCRIPT = [
    (0.0, Vec3(0.5, 0, 0)),  # speed up along +x to 1 m/s
    (2.0, Vec3()),  # coast: no drag, so the speed should stay
    (5.0, Vec3(-0.5, 0, 0)),  # brake back to 0
    (7.0, Vec3(0, 0.5, 0)),  # speed up along +y
    (9.0, Vec3(0, -0.5, 0)),  # brake
    (11.0, Vec3()),  # hold: how much does it drift?
    (16.0, Vec3(5.0, 0, 0)),  # hard push near the 30 deg limit (5.7 m/s^2): a big tilt step
    (17.0, Vec3(-5.0, 0, 0)),  # hard brake
    (18.0, Vec3()),
]
SCRIPT_END = 20.0
SETTLE = 2.0  # seconds of hovering before the script starts

logs = Path(__file__).resolve().parents[2] / "logs"
logs.mkdir(exist_ok=True)
(logs / "flight_test.done").unlink(missing_ok=True)

drone = Crazyflie()
pilot = Autopilot(float(str(drone.config["altitude"])), FlightParams(**overrides("flight")))
hover_since: float | None = None
last_time = drone.time

with open(logs / "flight_test_telemetry.csv", "w", newline="") as f:
    log = csv.writer(f)
    log.writerow(["t", "phase", "x", "y", "z", "vx", "vy", "vz",
                  "roll", "pitch", "yaw", "ax_cmd", "ay_cmd", "roll_cmd", "pitch_cmd",
                  "gx", "gy", "gz", "motor_min", "motor_max"])  # fmt: skip
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
        wanted = tilt_for(c, now.yaw, pilot.p.max_tilt) if command is not None else (0.0, 0.0)
        gx, gy, gz = drone.gyro.getValues()
        log.writerow([f"{t:.3f}", pilot.phase.value,
                      *(f"{x:.4f}" for x in (p.x, p.y, p.z, v.x, v.y, v.z)),
                      *(f"{a:.4f}" for a in (now.roll, now.pitch, now.yaw, c.x, c.y, *wanted)),
                      f"{gx:.4f}", f"{gy:.4f}", f"{gz:.4f}",
                      f"{min(motors):.2f}", f"{max(motors):.2f}"])  # fmt: skip
        f.flush()
