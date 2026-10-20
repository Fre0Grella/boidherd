"""Drone controller: lift off, climb to the takeoff altitude, then flock.

The world gives each drone its altitude in customData, and leaders the goal
(sector C, as x_min, y_min, x_max, y_max). Every step the drone broadcasts
its velocity over IR and turns what it hears into a Reynolds acceleration;
the autopilot follows it once the drone hovers. A leader also pulls toward
the centre of C, from `goal_after` seconds after it starts hovering, so the
flock forms first. Nothing marks a leader to the others (Couzin 2005).
"""

from boids import BoidParams, flock
from flight import Autopilot, FlightParams, Phase
from flight.crazyflie import Crazyflie
from flight.radio import encode
from flight.sensing import neighbours
from util import Vec3
from util.overrides import overrides

drone = Crazyflie()
altitude = float(str(drone.config["altitude"]))
pilot = Autopilot(altitude, FlightParams(**overrides("flight")))
rules = BoidParams(**overrides("boids"))
goal: Vec3 | None = None
if isinstance(area := drone.config.get("goal"), list):
    x_min, y_min, x_max, y_max = (float(v) for v in area)
    goal = Vec3((x_min + x_max) / 2, (y_min + y_max) / 2, altitude)
hovering_since: float | None = None
last_time = drone.time

while drone.step():
    now = drone.time
    attitude = drone.attitude()
    velocity = drone.velocity()
    drone.broadcast(encode(drone.id, velocity))
    heard = neighbours(drone.id, drone.receptions(), attitude)
    if hovering_since is None and pilot.phase in (Phase.HOVERING, Phase.FOLLOWING):
        hovering_since = now
    to_goal = None
    if goal is not None and hovering_since is not None and now - hovering_since >= rules.goal_after:
        to_goal = goal - drone.position()
    command = flock(velocity, heard, rules, to_goal)
    drone.set_motors(pilot.step(now - last_time, attitude, command))
    last_time = now
