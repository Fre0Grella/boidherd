"""When a drone cruises, and which way: the one rule with a memory.

Self-propulsion needs a heading, and a hovering drone's velocity points
nowhere in particular. So a drone wakes up only once something (a leader,
the flock) has dragged it above `wake_speed`, and from then it keeps
cruising, along its last heading taken above that speed. A drone slowed
down by the flock ahead of it keeps pushing the way it was going instead of
stalling outside C.

It stops `settle_time` after it recognises C, not at once: a flock arrives
behind its leaders, and drones stopping right at the edge would pile up
there, the tail left outside. A few more seconds carry the flock across C,
whatever its size, and need nothing but recognising C (design log, step 2).
"""

from util import Vec3

from .params import BoidParams


class Cruise:
    def __init__(self, p: BoidParams) -> None:
        self.p = p
        self.heading: Vec3 | None = None  # None: asleep
        self._entered: float | None = None  # when I last entered C

    def update(self, velocity: Vec3, now: float, go: bool, in_c: bool) -> Vec3 | None:
        """The heading to cruise along this step (unit, horizontal), or None. `go`: the
        mission has started; `in_c`: I recognise C under me."""
        if in_c and self._entered is None:
            self._entered = now
        elif not in_c:
            self._entered = None
        settled = self._entered is not None and now - self._entered >= self.p.settle_time
        if not go or settled:
            self.heading = None
            return None
        flat = velocity.horizontal()
        if flat.length() >= self.p.wake_speed:
            self.heading = flat.unit()
        return self.heading
