"""Drone controller: lift off, climb to the takeoff altitude, then flock.

The world gives each drone its altitude in customData. Every step the drone
broadcasts its velocity over IR and turns what it hears into a Reynolds
acceleration; the autopilot follows it once the drone hovers. Leaders get
the goal in customData too, but nothing uses it yet (phase 2).
"""

from boids import BoidParams, flock
from flight import Autopilot, FlightParams
from flight.crazyflie import Crazyflie
from flight.radio import encode
from flight.sensing import neighbours
from util.overrides import overrides

drone = Crazyflie()
pilot = Autopilot(float(str(drone.config["altitude"])), FlightParams(**overrides("flight")))
rules = BoidParams(**overrides("boids"))
last_time = drone.time

while drone.step():
    now = drone.time
    attitude = drone.attitude()
    velocity = drone.velocity()
    drone.broadcast(encode(drone.id, velocity))
    heard = neighbours(drone.id, drone.receptions(), attitude)
    drone.set_motors(pilot.step(now - last_time, attitude, flock(velocity, heard, rules)))
    last_time = now
