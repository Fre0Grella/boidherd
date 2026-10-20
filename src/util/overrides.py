"""Per-run settings from the test tools: `BOIDHERD_PARAMS` in the environment.

A tool sets it to JSON such as {"flight": {"kp_tilt": 2.0}, "boids": {...}}
before starting Webots; Webots passes its environment on to the controllers,
which build their settings with `Params(**overrides("flight"))`. Unset (a
world opened by hand), every section is empty and the defaults apply.
"""

import json
import os

VARIABLE = "BOIDHERD_PARAMS"


def overrides(section: str) -> dict[str, float]:
    return json.loads(os.environ.get(VARIABLE) or "{}").get(section, {})


def parse(assignments: list[str]) -> dict[str, dict[str, float]]:
    """Command-line `section.name=value` pairs, e.g. `flight.kp_tilt=2`, as sections."""
    sections: dict[str, dict[str, float]] = {}
    for a in assignments:
        key, _, value = a.partition("=")
        section, _, name = key.partition(".")
        if not (section and name and value):
            raise ValueError(f"expected section.name=value, got {a!r}")
        sections.setdefault(section, {})[name] = float(value)
    return sections
