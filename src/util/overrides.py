"""Per-run settings from the test tools: `BOIDHERD_PARAMS` in the environment.

A tool sets it to JSON such as {"flight": {"kp_tilt": 2.0}, "boids": {...}}
before starting Webots; Webots passes its environment on to the controllers,
which build their settings with `Params(**overrides("flight"))`. Unset (a
world opened by hand), every section is empty and the defaults apply.

On a tool's command line every setting is a flag of its own (`add_flags`):
--model couzin --cohesion 0.5. `--set section.name=value` does the same.
"""

import argparse
import inspect
import json
import os
import re
from collections.abc import Mapping
from dataclasses import fields
from typing import Any

VARIABLE = "BOIDHERD_PARAMS"


# a number, or a name such as boids.model=couzin; checked by the settings class it fills
Value = Any


def overrides(section: str) -> dict[str, Value]:
    return json.loads(os.environ.get(VARIABLE) or "{}").get(section, {})


def parse(assignments: list[str]) -> dict[str, dict[str, Value]]:
    """Command-line `section.name=value` pairs, e.g. `flight.kp_tilt=2`, as sections; a
    value that isn't a number stays text."""
    sections: dict[str, dict[str, Value]] = {}
    for a in assignments:
        key, _, value = a.partition("=")
        section, _, name = key.partition(".")
        if not (section and name and value):
            raise ValueError(f"expected section.name=value, got {a!r}")
        sections.setdefault(section, {})[name] = _value(value)
    return sections


def _value(text: str) -> Value:
    if text.lower() in SWITCH:
        return SWITCH[text.lower()]
    try:
        return float(text)
    except ValueError:
        return text


SWITCH = {"true": True, "on": True, "yes": True, "false": False, "off": False, "no": False}


def _switch(text: str) -> bool:
    """A yes/no setting from the command line: 1, true, on, yes or 0, false, off, no
    (bool("0") would be True)."""
    value = _value(text)
    if value not in (0, 1):
        raise argparse.ArgumentTypeError(f"expected 1/0, true/false, on/off; got {text!r}")
    return bool(value)


def add_flags(
    parser: argparse.ArgumentParser,
    sections: Mapping[str, type],
    aliases: Mapping[str, str] | None = None,
) -> None:
    """One flag per field of each settings dataclass, named after it: --v-flock (or
    --v_flock) for `boids.v_flock`. `aliases`: extra flag names, {"velocity":
    "boids.v_flock"}. Unset flags leave the default in the code."""
    for section, settings in sections.items():
        group = parser.add_argument_group(f"{section} settings (unset: as in the code)")
        comments = _comments(settings)
        for f in fields(settings):
            key = f"{section}.{f.name}"
            names = [f"--{f.name.replace('_', '-')}"]
            names += [f"--{a}" for a, target in (aliases or {}).items() if target == key]
            names += [f"--{f.name}"] if "_" in f.name else []
            about = comments.get(f.name, "")
            group.add_argument(
                *names,
                dest=key,
                type=_switch if isinstance(f.default, bool) else type(f.default),
                default=None,
                metavar="X",
                help=f"{about} (default {f.default})" if about else f"default {f.default}",
            )


def _comments(settings: type) -> dict[str, str]:
    """What each field of a settings class is, from its comments in the source: the one on
    its line, else the block just above it ("%" escaped for argparse)."""
    found: dict[str, str] = {}
    block: list[str] = []
    for line in inspect.getsource(settings).splitlines():
        text = line.strip()
        if text.startswith("#"):
            block.append(text.lstrip("# "))
            continue
        field = re.match(r"(\w+): [^=]+=[^#]*(?:#\s*(.*))?$", text)
        if field:
            found[field.group(1)] = (field.group(2) or " ".join(block)).replace("%", "%%")
        block = []
    return found


def from_flags(args: argparse.Namespace, assignments: list[str]) -> dict[str, dict[str, Value]]:
    """The settings a run asked for: `--set` assignments, then the flags (they win)."""
    sections = parse(assignments)
    for key, value in vars(args).items():
        section, dot, name = key.partition(".")
        if dot and value is not None:
            sections.setdefault(section, {})[name] = value
    return sections
