# boidherd

Reynolds boids on simulated Crazyflie drones in Webots: a few informed
leaders guide a flock through an obstacle course to a goal that only
they know.

Status: designing the architecture. Decisions and their reasons are in
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

## Checks

The same four run on GitHub for every push:

```
uv run ruff check           # lint
uv run ruff format --check  # formatting (drop --check to fix it)
uv run pyright              # types
uv run pytest               # tests
```
