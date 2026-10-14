"""Draw a course and a swarm from above, as a PNG: check a map without Webots."""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # files only, no window

import matplotlib.pyplot as plt  # noqa: E402 - after choosing the backend
from matplotlib.axes import Axes  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402

from maps.course import Course  # noqa: E402
from maps.model import Rect  # noqa: E402

from .spawn import Drone  # noqa: E402

FREE = "#f4f1ea"
BOUNDARY = "#8c8c8c"
OBSTACLE = "#e07a1f"
SECTOR_A = "#2e9e44"
SECTOR_C = "#d9a800"
FOLLOWER = "#2b5fad"
LEADER = "#c8102e"


def draw_preview(course: Course, swarm: tuple[Drone, ...], title: str, file: Path) -> None:
    fig = plt.figure(figsize=(10, 10 * course.floor.depth / course.floor.width + 1))
    ax = fig.add_subplot()
    ax.add_patch(Polygon(list(course.area.exterior.coords), color=FREE, zorder=0))
    for hole in course.area.interiors:
        ax.add_patch(Polygon(list(hole.coords), color="white", zorder=0))
    _rect(ax, course.sector_a.area, SECTOR_A, alpha=0.3, label="sector A (spawn)")
    _rect(ax, course.sector_c.area, SECTOR_C, alpha=0.3, label="sector C (goal)")
    for wall in course.boundary:
        _rect(ax, wall.rect, BOUNDARY)
    for wall in course.walls:
        _rect(ax, wall.rect, OBSTACLE, alpha=_alpha(wall.height))
    for pillar in course.pillars:
        ax.add_patch(
            Circle(pillar.center, pillar.radius, color=OBSTACLE, alpha=_alpha(pillar.height))
        )

    for leader, color, marker, size, label in (
        (False, FOLLOWER, "o", 25, "follower"),
        (True, LEADER, "*", 140, "leader"),
    ):
        group = [d for d in swarm if d.leader == leader]
        if group:
            xs, ys = zip(*(d.xy for d in group), strict=True)
            ax.scatter(xs, ys, s=size, c=color, marker=marker, zorder=3, label=label)

    floor = course.floor
    ax.set_xlim(floor.x_min, floor.x_max)
    ax.set_ylim(floor.y_min, floor.y_max)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    ax.grid(True, linewidth=0.3, alpha=0.5)
    ax.legend(loc="upper left", fontsize="small", framealpha=0.9)
    fig.tight_layout()
    fig.savefig(file, dpi=150)
    plt.close(fig)


def _rect(ax: Axes, r: Rect, color: str, alpha: float = 1.0, label: str | None = None) -> None:
    patch = Rectangle((r.x_min, r.y_min), r.width, r.depth, color=color, alpha=alpha, label=label)
    ax.add_patch(patch)


def _alpha(height: float | None) -> float:
    """Full-height obstacles solid, low ones (phase 4) lighter."""
    return 1.0 if height is None else 0.45
