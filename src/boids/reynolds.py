"""Reynolds (1987) boids, as in phase 2: three steering rules and a leader's goal.

The rules give an acceleration (m/s^2, world frame), from what a drone hears:

- separation: away from neighbours inside `reynolds_separation_radius`,
  harder the closer they are (zero at the edge of the zone);
- alignment: toward the neighbours' mean velocity;
- cohesion: toward the neighbours' centre;
- goal, leaders only: toward v_flock aimed at the goal, slowing down within
  arrive_radius (Reynolds 1999: seek, then arrive).

Nothing propels a follower: it moves only as its neighbours pull it, and
nothing brakes a drifting flock (no drag), as Reynolds' boids. To share the
rest with the other models, the acceleration becomes a wanted velocity,
v + a x tau: the flight layer, which steers at (wanted - v) / tau, then
asks for exactly a, as phase 2 did directly; capped at max_speed (Reynolds
truncates the speed) and through the stopping limit. Before the start the
rules already act, so the swarm forms, as in phase 2; once settled in C a
drone holds still.
"""

from flight.sensing import Neighbour
from util import Vec3

from .limit import stopping_limit
from .mission import Mission
from .params import BoidParams
from .zone import ellipsoidal_distance, within


class Reynolds:
    def __init__(self, p: BoidParams, leader: bool = False, tau: float = 0.3) -> None:
        self.p = p
        self.leader = leader
        self.tau = tau  # s, the flight layer's velocity_tau
        self.mission = Mission(p)

    def wanted(
        self,
        velocity: Vec3,
        neighbours: list[Neighbour],
        now: float,
        go: bool,
        in_c: bool,
        to_goal: Vec3 | None,
        to_walls: list[Vec3],
    ) -> Vec3:
        p = self.p
        seen = [n for n in neighbours if within(n.offset, p.perception_radius, p.vertical_scale)]
        active = self.mission.active(now, go, in_c)
        if go and not active:  # settled in C
            return stopping_limit(Vec3(), seen, to_walls, p)
        a = steer(velocity, seen, p)
        if to_goal is not None and active:
            a = a + goal(velocity, to_goal, p)
        wanted = velocity.horizontal() + a.scaled_z(p.vertical_gain) * self.tau
        return stopping_limit(wanted.clamped(p.max_speed * p.v_flock), seen, to_walls, p)


def steer(velocity: Vec3, neighbours: list[Neighbour], p: BoidParams) -> Vec3:
    """Separation + alignment + cohesion, weighted (m/s^2)."""
    v = p.v_flock
    return (
        separation(neighbours, p) * (p.reynolds_separation * v**2)
        + alignment(velocity, neighbours) * (p.reynolds_alignment * v)
        + cohesion(neighbours) * (p.reynolds_cohesion * v**2)
    )


def separation(neighbours: list[Neighbour], p: BoidParams) -> Vec3:
    """Away from each neighbour inside the radius, by radius / distance - 1 (1 at half)."""
    push = Vec3()
    r = p.reynolds_separation_radius
    for n in neighbours:
        d = ellipsoidal_distance(n.offset, p.vertical_scale)
        if 0 < d < r:
            push = push - n.offset.unit() * (r / d - 1)
    return push


def alignment(velocity: Vec3, neighbours: list[Neighbour]) -> Vec3:
    """Mean velocity of the neighbours minus mine; zero when alone."""
    if not neighbours:
        return Vec3()
    return Vec3.mean(n.velocity for n in neighbours) - velocity


def cohesion(neighbours: list[Neighbour]) -> Vec3:
    """From me to the neighbours' centre; zero when alone."""
    return Vec3.mean(n.offset for n in neighbours)


def goal(velocity: Vec3, to_goal: Vec3, p: BoidParams) -> Vec3:
    """Steer toward v_flock aimed at the goal; within arrive_radius the wanted speed shrinks
    with the distance, so the leader stops there instead of overshooting."""
    slow_down = min(1.0, to_goal.length() / p.arrive_radius)
    aim = to_goal.unit() * (p.v_flock * slow_down)
    return (aim - velocity) * (p.reynolds_goal * p.v_flock)
