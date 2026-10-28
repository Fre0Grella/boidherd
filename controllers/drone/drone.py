"""Drone controller: lift off, climb to the takeoff altitude, then flock.

The world gives each drone its altitude and sector C in customData, and
marks the leaders. Every step the drone broadcasts its velocity over IR and
turns what it hears into a wanted velocity with the flocking model the run
chose (boids.make_model: hybrid, Couzin or Reynolds), which the flight layer follows
once the drone hovers. The mission clock starts the trip: before `go_time`
the flock forms; from then leaders head for the centre of C and the others
follow, until a few metres into C. Followers don't know where C is, they
only recognise it when they are in it (a ground marker; here GPS against
C's rectangle). Nothing marks a leader to the others (Couzin 2005).
Every drone knows the walls from the map and where it is from GPS
(boids.to_walls). With `plan.planning` on, a leader plans its own path
round the walls at go time, from where it is, follows it, and plans again
whenever the flock has pushed it behind a wall (planner.route); off, or
while no passage is wide enough, it aims straight at C.
"""

from boids import BoidParams, Box, Disk, make_model, to_walls
from flight import Autopilot, FlightParams, acceleration_for
from flight.crazyflie import Crazyflie
from flight.radio import encode
from flight.sensing import neighbours
from planner import Planner, PlanParams, Route
from util import Vec3
from util.overrides import overrides

drone = Crazyflie()
altitude = float(str(drone.config["altitude"]))
pilot = Autopilot(altitude, FlightParams(**overrides("flight")))
rules = BoidParams(**overrides("boids"))
x_min, y_min, x_max, y_max = (float(v) for v in drone.config["sector_c"])  # type: ignore[union-attr]
centre = Vec3((x_min + x_max) / 2, (y_min + y_max) / 2, altitude)
leader = drone.config.get("leader") is True
boundary: list[Box] = [tuple(float(v) for v in w) for w in drone.config["boundary"]]  # type: ignore
walls: list[Box] = [tuple(float(v) for v in w) for w in drone.config["walls"]]  # type: ignore
pillars: list[Disk] = [tuple(float(v) for v in d) for d in drone.config["pillars"]]  # type: ignore
solid = boundary + walls  # everything the stopping limit keeps clear of, with the pillars
planning = PlanParams(**overrides("plan"))
model = make_model(rules, leader, pilot.p.velocity_tau)
route: Route | None = None  # a leader's way to C, from go time on
plans = 0
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
    if leader and go and planning.planning and route is None:
        route = Route(Planner(boundary, walls, pillars, planning), (centre.x, centre.y), planning)
    to_goal = None
    if leader and go:
        if route is not None:
            to_goal = route.to_goal(position, now)
            if route.plans != plans:
                plans = route.plans
                assert route.pursuit is not None
                print(f"drone {drone.id}: plan {plans} at {now:.1f} s: {route.pursuit.path}")
        if to_goal is None:
            to_goal = centre - position
    wanted = model.wanted(
        velocity, heard, now, go, in_c, to_goal, to_walls(position, solid, pillars)
    )
    command = acceleration_for(wanted, velocity, pilot.p.velocity_tau, pilot.p.max_acceleration)
    drone.set_motors(pilot.step(now - last_time, attitude, command))
    last_time = now
