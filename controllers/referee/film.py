"""The movie tools/record.py asks the referee for.

The camera looks along `forward` (straight down, or tilted) at a frame
`width` metres across. With a fixed frame (the whole map) it stays put;
otherwise it follows the flock, framing every drone with a margin but never
closer than `min_width` metres across, so drones stay visible at their true
size and none leaves the frame when the flock spreads. It eases toward that
frame over `EASE` seconds, so it glides instead of jittering. For the movie
the floor turns plain dark grey, so the markers stand out; a world opened
by hand keeps its checkerboard.
"""

import math

from controller import Node, Supervisor  # pyright: ignore[reportMissingImports]

EASE = 1.5  # s: how slowly the camera follows the flock
MARGIN = 1.3  # the frame is this much wider than the flock
FLOOR = [0.12, 0.12, 0.12]


class Film:
    def __init__(self, referee: Supervisor, drones: list[Node], movie: dict) -> None:
        self.referee, self.drones, self.movie = referee, drones, movie
        self.aspect = movie["width"] / movie["height"]
        self.half = math.tan(movie["field_of_view"] / 2)  # across the frame's width
        self.view = self._node("Viewpoint")
        self.view.getField("orientation").setSFRotation(movie["orientation"])
        floor = self._node("Floor").getField("appearance").getSFNode()
        floor.getField("baseColorMap").removeSF()
        floor.getField("baseColor").setSFColor(FLOOR)
        fixed = movie.get("fixed")
        self.centre, self.width = (fixed["centre"], fixed["width"]) if fixed else self._frame()
        self._place()
        referee.movieStartRecording(
            movie["file"], movie["width"], movie["height"], 0, 90, movie["speed"], True
        )

    def step(self, dt: float) -> None:
        """Ease the camera toward the frame that holds every drone, unless it is fixed."""
        if self.movie.get("fixed"):
            return
        centre, width = self._frame()
        a = min(1.0, dt / EASE)
        self.centre = [c + a * (t - c) for c, t in zip(self.centre, centre, strict=True)]
        self.width += a * (width - self.width)
        self._place()

    def cut(self, time_step: int) -> None:
        """End the movie, wait until Webots has written it, and quit."""
        self.referee.movieStopRecording()
        while not self.referee.movieIsReady() and self.referee.step(time_step) != -1:
            pass
        self.referee.simulationQuit(1 if self.referee.movieFailed() else 0)

    def _frame(self) -> tuple[list[float], float]:
        xs, ys = zip(*(d.getPosition()[:2] for d in self.drones), strict=True)
        across = MARGIN * max(max(xs) - min(xs), (max(ys) - min(ys)) * self.aspect)
        centre = [(min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2]
        return centre, max(self.movie["min_width"], across)

    def _place(self) -> None:
        """Back away from the frame's centre along the view, far enough to see it all."""
        distance = self.width / 2 / self.half
        target = [*self.centre, 0.0]
        position = [c - distance * f for c, f in zip(target, self.movie["forward"], strict=True)]
        self.view.getField("position").setSFVec3f(position)

    def _node(self, kind: str) -> Node:
        children = self.referee.getRoot().getField("children")
        for k in range(children.getCount()):
            node = children.getMFNode(k)
            if node.getTypeName() == kind:
                return node
        raise LookupError(f"the world has no {kind}")
