# boidherd

A flock of simulated Crazyflie drones in Webots: a few informed leaders,
which the others cannot recognise, guide the flock through an obstacle
course to a goal that only they know.

Three flocking models fly on the same robot behind one interface:
Reynolds' boids, Couzin's zone model and a hybrid built on Vásárhelyi's
model for real drones. Leaders can also plan a path on the known map
(A*) and follow it (pure pursuit).

Status: done for the Intelligent Robotic Systems exam. The experiments
(450 runs) and the report are finished; 3D flight is future work.

- The report: [report/](report/) (build with `tectonic main.tex` from
  that folder). It covers the design choices, the experiments and the
  results.
- Every decision, with its reason and the alternatives turned down:
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

Any setting can be changed for one run by its name, e.g.
`uv run tools/flock_test.py --model couzin --velocity 0.5 --kp-angle 8`
(`--model`: the flocking model, `hybrid`, `couzin` or `reynolds`;
`--velocity`: the cruise speed; `--plan 1`: leaders plan a path round the
walls instead of aiming straight at C). `-h` lists every setting and what it does;
`--set boids.cohesion=0.5` still works.

## Experiments

```
uv run tools/sweep.py --map dead_end -n 10 20 30 -j 0 1 2 4 \
    --seeds 0 1 2 3 4 5 6 7 8 9 --duration 83 --model hybrid --plan 1
uv run tools/tune.py --model couzin --grid couzin_weight=0.5,1,2   # tuning seeds only
uv run tools/analyse.py all     # every number in the report; figures to report/figures
```

A sweep appends one row per run to `logs/sweep_<map>_<model>[_plan].csv`
and resumes where it stopped. A run succeeds when 90% of the drones are
in C within the map's limit (83 s on `dead_end`, 112 s on `l_turn`).

## Checks

The same four run on GitHub for every push:

```
uv run ruff check           # lint
uv run ruff format --check  # formatting (drop --check to fix it)
uv run pyright              # types
uv run pytest               # tests
```
