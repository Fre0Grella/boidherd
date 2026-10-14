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

## Checks

The same four run on GitHub for every push:

```
uv run ruff check           # lint
uv run ruff format --check  # formatting (drop --check to fix it)
uv run pyright              # types
uv run pytest               # tests
```
