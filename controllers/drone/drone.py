"""Drone controller: lift off, climb to the takeoff altitude, hover.

The world gives each drone its altitude in customData. Flocking is not
attached yet: the drone hovers in place (phase 1, step 2).
"""

from flight import Autopilot
from flight.crazyflie import Crazyflie

drone = Crazyflie()
pilot = Autopilot(altitude=float(str(drone.config["altitude"])))
last_time = drone.time

while drone.step():
    now = drone.time
    drone.set_motors(pilot.step(now - last_time, drone.attitude(), None))
    last_time = now
