"""Drone controller: lift off, climb to the takeoff altitude, then flock.

The world gives each drone its altitude and sector C in customData, and
marks the leaders. Every step the drone broadcasts its velocity over IR and
turns what it hears into a wanted velocity (boids.flock), which the flight
layer follows once the drone hovers. The mission clock starts the trip:
before `go_time` the flock forms; from then leaders head for the centre of
C and every drone may cruise; once woken it keeps cruising until a few
seconds after it reaches C (boids.Cruise). Followers don't know where C
is, they only recognise it when they are in it (a ground marker; here GPS
against C's rectangle). Nothing marks a leader to the others (Couzin 2005).
"""

from boids import BoidParams, Cruise, flock
from flight import Autopilot, FlightParams, acceleration_for
from flight.crazyflie import Crazyflie
from flight.radio import encode
from flight.sensing import neighbours
from util import Vec3
from util.overrides import overrides

drone = Crazyflie()
altitude = float(str(drone.config["altitude"]))
pilot = Autopilot(altitude, FlightParams(**overrides("flight")))
rules = BoidParams(**overrides("boids"))
x_min, y_min, x_max, y_max = (float(v) for v in drone.config["sector_c"])  # type: ignore[union-attr]
centre = Vec3((x_min + x_max) / 2, (y_min + y_max) / 2, altitude)
leader = drone.config.get("leader") is True
cruise = Cruise(rules)
last_time = drone.time

while drone.step():
    now = drone.time
    attitude = drone.attitude()
    velocity = drone.velocity()
    position = drone.position()
    drone.broadcast(encode(drone.id, velocity))
    heard = neighbours(drone.id, drone.receptions(), attitude)
    go = now >= rules.go_time
    in_c = x_min <= position.x <= x_max and y_min <= position.y <= y_max
    to_goal = centre - position if leader and go else None
    heading = cruise.update(velocity, now, go, in_c)
    wanted = flock(velocity, heard, rules, heading=heading, to_goal=to_goal)
    command = acceleration_for(wanted, velocity, pilot.p.velocity_tau, pilot.p.max_acceleration)
    drone.set_motors(pilot.step(now - last_time, attitude, command))
    last_time = now
