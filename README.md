# boidherd

Reynolds boids on simulated Crazyflie drones in Webots: a few informed
leaders guide a flock through an obstacle course to a goal that only
they know.

Status: phase 1 (flat flocking). Drones take off, sense each other over
IR and flock with Reynolds' rules; adapting the rules to the robot is
under way. Decisions and their reasons are in
[docs/design-log.md](docs/design-log.md).

## Setup

Needs [uv](https://docs.astral.sh/uv/) and Webots R2025a.

```
uv sync          # creates .venv with Python 3.11 and installs everything
```

For the editor to resolve imports, point it at `.venv` and add `src` and
Webots' Python library (for `from controller import ...`) to the analysis
paths, e.g. in VS Code `.vscode/settings.json`:

```json
"python.defaultInterpreterPath": "${workspaceFolder}/.venv/Scripts/python.exe",
"python.analysis.extraPaths": ["src", "C:/Program Files/Webots/lib/controller/python"]
```

## Make a world

```
uv run make_world.py l_hall                    # maps/l_hall.toml -> worlds/l_hall.wbt
uv run make_world.py l_hall -n 20 -j 2 -s 7    # drones, leaders, seed (defaults 10, 1, 0)
uv run make_world.py l_hall --preview          # also draw worlds/l_hall.png
uv run make_world.py l_hall --no-markers       # without the coloured plates on the drones
```

Then open `worlds/l_hall.wbt` in Webots. Run `make_world.py` once before
opening any world: it also tells Webots to run the controllers with the
project's `.venv` (see `docs/design-log.md`, "How Webots starts Python
controllers").

The maps in `maps/` are yours to edit and add to. The tests use their own
frozen copies in `tests/maps/`, so an experiment never breaks them.

## Test in Webots

Headless runs that report numbers (start them from PowerShell: Webots
crashes when started from Git Bash):

```
uv run tools/flight_test.py      # one drone follows scripted velocities
uv run tools/takeoff_test.py     # a whole swarm takes off; -n, --map, --duration
uv run tools/ir_test.py          # IR neighbour sensing against ground truth
uv run tools/flock_test.py       # a leaderless swarm flocks: groups, spacing, speed
```

Any setting can be changed for one run, e.g.
`uv run tools/flock_test.py --set boids.cohesion=0.5 flight.kp_angle=8`.

## Checks

The same four run on GitHub for every push:

```
uv run ruff check           # lint
uv run ruff format --check  # formatting (drop --check to fix it)
uv run pyright              # types
uv run pytest               # tests
```
