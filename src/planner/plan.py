"""A leader's path to C: A* on a grid with a clearance cost, pruned to its corners.

The map is the one every drone already has (boids.obstacles: boxes and
disks). Obstacles smaller than `min_size` across are dropped, so the plan
goes round walls and leaves pillars to the stopping limit; the arena
outline is always kept. Each grid cell knows its distance to the nearest
kept obstacle. A cell closer than `margin` is forbidden; closer than
`clearance` a step through it costs more, rising toward the wall. So the
path keeps to the middle of wide halls and gaps, and still takes a narrow
passage when it must, at a price: one fixed clearance would either close
narrow passages or hug every corner (design log, phase 5).

The cells are then pruned to a few waypoints. A shortcut from one waypoint
to a later one is kept only if it never comes closer to a wall than the
planned path between them did, so pruning can't cut a corner the cost
kept the path away from.
"""

import heapq
import math

from boids.obstacles import Box, Disk, to_walls
from util import Vec3

from .params import PlanParams

Point = tuple[float, float]
Cell = tuple[int, int]
STEPS = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)]


def kept_obstacles(
    boundary: list[Box], walls: list[Box], pillars: list[Disk], p: PlanParams
) -> tuple[list[Box], list[Disk]]:
    """The obstacles a plan goes round: the whole arena outline, and the walls and pillars
    at least `min_size` across."""
    big_walls = [b for b in walls if max(b[2] - b[0], b[3] - b[1]) >= p.min_size]
    big_pillars = [d for d in pillars if 2 * d[2] >= p.min_size]
    return boundary + big_walls, big_pillars


class Grid:
    """The cells over the arena's bounding box, each with its distance to the nearest kept
    obstacle (0 inside one)."""

    def __init__(self, boxes: list[Box], disks: list[Disk], p: PlanParams) -> None:
        self.p = p
        self.x0 = min(b[0] for b in boxes)
        self.y0 = min(b[1] for b in boxes)
        self.nx = math.ceil((max(b[2] for b in boxes) - self.x0) / p.cell)
        self.ny = math.ceil((max(b[3] for b in boxes) - self.y0) / p.cell)
        self.clear = [
            [_clearance(self.centre((i, j)), boxes, disks) for j in range(self.ny)]
            for i in range(self.nx)
        ]

    def centre(self, c: Cell) -> Point:
        return (self.x0 + (c[0] + 0.5) * self.p.cell, self.y0 + (c[1] + 0.5) * self.p.cell)

    def cell(self, q: Point) -> Cell:
        i = int((q[0] - self.x0) / self.p.cell)
        j = int((q[1] - self.y0) / self.p.cell)
        return (min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1))

    def clearance(self, q: Point) -> float:
        i, j = self.cell(q)
        return self.clear[i][j]

    def penalty(self, distance: float) -> float:
        """How much more a step costs at this distance from a wall: 1 far away, up to
        1 + clearance_weight at the margin."""
        p = self.p
        if distance >= p.clearance:
            return 1.0
        closeness = (p.clearance - distance) / (p.clearance - p.margin)
        return 1.0 + p.clearance_weight * min(1.0, closeness) ** 2


def plan(
    start: Point,
    goal: Point,
    boundary: list[Box],
    walls: list[Box],
    pillars: list[Disk],
    p: PlanParams | None = None,
) -> list[Point] | None:
    """Waypoints from `start` to `goal` (both included), or None if no passage is wide
    enough. `boundary`: the arena outline as boxes; `walls`, `pillars`: the map's
    obstacles, of which only those at least min_size across are planned round."""
    p = p or PlanParams()
    boxes, disks = kept_obstacles(boundary, walls, pillars, p)
    grid = Grid(boxes, disks, p)
    cells = _astar(grid, grid.cell(start), grid.cell(goal))
    if cells is None:
        return None
    points = [start] + [grid.centre(c) for c in cells[1:-1]] + [goal]
    return _prune(points, grid)


def _clearance(q: Point, boxes: list[Box], disks: list[Disk]) -> float:
    return min(off.length() for off in to_walls(Vec3(q[0], q[1], 0), boxes, disks))


def _astar(grid: Grid, start: Cell, goal: Cell) -> list[Cell] | None:
    """The cheapest 8-connected route; the start and goal cells may be inside the margin."""

    def free(c: Cell) -> bool:
        inside = 0 <= c[0] < grid.nx and 0 <= c[1] < grid.ny
        return inside and (c in (start, goal) or grid.clear[c[0]][c[1]] >= grid.p.margin)

    def h(c: Cell) -> float:
        return math.dist(c, goal) * grid.p.cell  # never more than the cost: admissible

    best = {start: 0.0}
    came: dict[Cell, Cell] = {}
    frontier = [(h(start), start)]
    while frontier:
        _, c = heapq.heappop(frontier)
        if c == goal:
            route = [c]
            while route[-1] in came:
                route.append(came[route[-1]])
            return route[::-1]
        for dx, dy in STEPS:
            n = (c[0] + dx, c[1] + dy)
            if not free(n):
                continue
            length = math.hypot(dx, dy) * grid.p.cell
            cost = best[c] + length * grid.penalty(grid.clear[n[0]][n[1]])
            if cost < best.get(n, math.inf):
                best[n], came[n] = cost, c
                heapq.heappush(frontier, (cost + h(n), n))
    return None


def _prune(points: list[Point], grid: Grid) -> list[Point]:
    """Keep only the corners: from each waypoint, jump to the farthest later point whose
    straight segment stays at least as far from walls as the path between them did."""
    kept = [points[0]]
    i = 0
    while i < len(points) - 1:
        j = len(points) - 1
        while j > i + 1:
            closest = min(grid.clearance(q) for q in points[i : j + 1])
            if _segment_clearance(points[i], points[j], grid) >= closest:
                break
            j -= 1
        kept.append(points[j])
        i = j
    return kept


def _segment_clearance(a: Point, b: Point, grid: Grid) -> float:
    steps = max(1, math.ceil(math.dist(a, b) / (grid.p.cell / 2)))
    return min(
        grid.clearance((a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps))
        for k in range(steps + 1)
    )
