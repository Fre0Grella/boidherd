import argparse
from dataclasses import dataclass

import pytest

from util.overrides import add_flags, from_flags, parse


def test_numbers_become_floats_and_names_stay_text():
    assert parse(["boids.cohesion=0.5", "boids.model=couzin", "flight.kp_tilt=2"]) == {
        "boids": {"cohesion": 0.5, "model": "couzin"},
        "flight": {"kp_tilt": 2.0},
    }


def test_a_malformed_assignment_is_rejected():
    with pytest.raises(ValueError, match="expected section.name=value"):
        parse(["cohesion=0.5"])


@dataclass(frozen=True)
class Toy:
    # how the toy is called
    name: str = "a"
    rate_hz: float = 2.0  # how fast it ticks
    count: int = 3


def toy_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(allow_abbrev=False)
    add_flags(parser, {"toy": Toy}, {"speed": "toy.rate_hz"})
    return parser


def test_every_field_is_a_flag_with_hyphens_underscores_and_aliases():
    for flag in ("--rate-hz", "--rate_hz", "--speed"):
        args = toy_parser().parse_args([flag, "5"])
        assert from_flags(args, []) == {"toy": {"rate_hz": 5.0}}


def test_flags_keep_the_fields_types_and_unset_ones_stay_out():
    args = toy_parser().parse_args(["--name", "b", "--count", "7"])
    assert from_flags(args, []) == {"toy": {"name": "b", "count": 7}}


def test_flags_win_over_set():
    args = toy_parser().parse_args(["--count", "7"])
    assert from_flags(args, ["toy.count=1", "toy.name=c"]) == {"toy": {"count": 7, "name": "c"}}


def test_the_help_says_what_each_setting_is():
    text = toy_parser().format_help()
    assert "how fast it ticks (default 2.0)" in text
    assert "how the toy is called (default a)" in text
