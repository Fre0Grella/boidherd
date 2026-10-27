"""When a drone may move on its own: the mission clock and settling in C.

Shared by every model. Before `go_time` the swarm only forms. From then a
drone may move until it has cruised `settle_distance` into C (timed: that
distance at v_flock), not the moment it enters: a flock arrives behind its
leaders, and drones stopping right at the edge would pile up there, the
tail left outside (design log, step 2). Leaving C again restarts the count.
"""

from .params import BoidParams


class Mission:
    def __init__(self, p: BoidParams) -> None:
        self.p = p
        self._entered: float | None = None  # when I last entered C

    def active(self, now: float, go: bool, in_c: bool) -> bool:
        """Whether I may move on my own now. `go`: the mission has started; `in_c`: I
        recognise C under me."""
        if in_c and self._entered is None:
            self._entered = now
        elif not in_c:
            self._entered = None
        settle_time = self.p.settle_distance / self.p.v_flock
        settled = self._entered is not None and now - self._entered >= settle_time
        return go and not settled
