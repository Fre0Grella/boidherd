"""Hover controller: lift off, climb to the takeoff altitude, hover. Nothing else.

Used by tools/takeoff_test.py, so takeoff is measured on its own, without
flocking moving the drones afterwards. The world gives each drone its
altitude in customData.
"""

from flight import Autopilot, FlightParams
from flight.crazyflie import Crazyflie
from util.overrides import overrides

drone = Crazyflie()
pilot = Autopilot(float(str(drone.config["altitude"])), FlightParams(**overrides("flight")))
last_time = drone.time

while drone.step():
    now = drone.time
    drone.set_motors(pilot.step(now - last_time, drone.attitude(), None))
    last_time = now
