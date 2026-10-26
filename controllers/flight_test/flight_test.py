"""Flight test: one drone takes off, hovers, then follows a fixed list of wanted velocities.

Used by tools/flight_test.py, which builds the world and reads the log. The
velocities are in the world frame, so the drone (spawned at any yaw) must
move along world x and y whatever way it faces; `acceleration_for` turns
each into the acceleration the autopilot flies. It writes logs/flight_test_telemetry.csv
and, when the script is over, logs/flight_test.done; it never quits Webots.
"""

import csv
from pathlib import Path

from flight import Autopilot, FlightParams, Phase, acceleration_for, tilt_for
from flight.crazyflie import Crazyflie
from util import Vec3
from util.overrides import overrides

# (seconds from the start of the script, wanted velocity) - each holds until the next
SCRIPT = [
    (0.0, Vec3(0.5, 0, 0)),  # a gentle step: +x at 0.5 m/s
    (4.0, Vec3()),  # stop
    (7.0, Vec3(0, 1.0, 0)),  # +y at the flock's top speed
    (11.0, Vec3(0, -1.0, 0)),  # reversal: a 2 m/s change, so the acceleration cap acts
    (15.0, Vec3()),  # stop and hold: how much does it drift?
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
                  "roll", "pitch", "yaw", "vx_cmd", "vy_cmd", "ax_cmd", "ay_cmd",
                  "roll_cmd", "pitch_cmd",
                  "gx", "gy", "gz", "motor_min", "motor_max"])  # fmt: skip
    while drone.step():
        t = drone.time
        now = drone.attitude()
        if pilot.phase is Phase.HOVERING and hover_since is None:
            hover_since = t
        wanted = command = None
        if hover_since is not None and t - hover_since >= SETTLE:
            elapsed = t - hover_since - SETTLE
            wanted = next(v for start, v in reversed(SCRIPT) if elapsed >= start)
            command = acceleration_for(wanted, drone.velocity(), pilot.p.velocity_tau,
                                       pilot.p.max_acceleration)  # fmt: skip
            if elapsed >= SCRIPT_END and not (logs / "flight_test.done").exists():
                (logs / "flight_test.done").write_text(f"{t:.3f}\n")
        motors = pilot.step(t - last_time, now, command)
        drone.set_motors(motors)
        last_time = t

        p, v, w, c = drone.position(), drone.velocity(), wanted or Vec3(), command or Vec3()
        tilt = tilt_for(c, now.yaw, pilot.p.max_tilt) if command is not None else (0.0, 0.0)
        gx, gy, gz = drone.gyro.getValues()
        log.writerow([f"{t:.3f}", pilot.phase.value,
                      *(f"{x:.4f}" for x in (p.x, p.y, p.z, v.x, v.y, v.z)),
                      *(f"{a:.4f}" for a in (now.roll, now.pitch, now.yaw)),
                      *(f"{a:.4f}" for a in (w.x, w.y, c.x, c.y, *tilt)),
                      f"{gx:.4f}", f"{gy:.4f}", f"{gz:.4f}",
                      f"{min(motors):.2f}", f"{max(motors):.2f}"])  # fmt: skip
        f.flush()
