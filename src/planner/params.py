"""Planner settings: every number the leaders' path planning uses, in one place."""

from dataclasses import dataclass


@dataclass(frozen=True)
class PlanParams:
    # obstacles smaller than this across (a box's longest side, a pillar's diameter) are
    # left to the drones' own avoidance; the arena outline is always kept
    min_size: float = 1.0  # m
    # the grid A* searches
    cell: float = 0.25  # m
    # clearance: a cell closer than `margin` to a wall is forbidden; closer than `clearance`
    # it costs up to 1 + clearance_weight times its length, rising toward the wall
    margin: float = 0.3  # m, the drones' stop_wall
    clearance: float = 2.0  # m
    clearance_weight: float = 4.0
    # following: aim at the point this far ahead along the path (pure pursuit)
    lookahead: float = 2.0  # m
