"""The flocking models a run can choose (`BoidParams.model`), behind one interface.

Each model turns what a drone hears into a wanted velocity; everything else
is shared and identical, so runs compare the models and nothing else: the
mission clock and settling in C (mission.py), the stopping limit (limit.py),
the flight layer, perception, the cruise speed and which drones lead.
"""

from typing import Protocol

from flight.sensing import Neighbour
from util import Vec3

from .couzin import Couzin
from .cruise import Cruise
from .params import BoidParams
from .reynolds import Reynolds
from .rules import flock


class Model(Protocol):
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
        """The wanted velocity (m/s, world frame). `to_goal`, for a leader on its way: from
        the drone to its goal, metres. `to_walls`: from the drone to the nearest point of
        each wall, metres."""
        ...


class Hybrid:
    """Vásárhelyi + Reynolds' cohesion + Couzin's leaders (rules.py, cruise.py)."""

    def __init__(self, p: BoidParams, leader: bool = False) -> None:
        self.p = p
        self.cruise = Cruise(p, leader)

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
        heading = self.cruise.update(velocity, neighbours, now, go, in_c)
        return flock(
            velocity, neighbours, self.p, heading=heading, to_goal=to_goal, to_walls=to_walls
        )


MODELS = ("hybrid", "couzin", "reynolds")


def make_model(p: BoidParams, leader: bool, tau: float = 0.3) -> Model:
    """The model `p.model` names, for one drone. `tau`: the flight layer's velocity_tau,
    which Reynolds needs to turn its acceleration into a wanted velocity."""
    if p.model == "hybrid":
        return Hybrid(p, leader)
    if p.model == "couzin":
        return Couzin(p, leader)
    if p.model == "reynolds":
        return Reynolds(p, leader, tau)
    raise ValueError(f"boids.model: unknown model {p.model!r}; one of {', '.join(MODELS)}")
