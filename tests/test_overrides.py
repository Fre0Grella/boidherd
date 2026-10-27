import pytest

from util.overrides import parse


def test_numbers_become_floats_and_names_stay_text():
    assert parse(["boids.cohesion=0.5", "boids.model=couzin", "flight.kp_tilt=2"]) == {
        "boids": {"cohesion": 0.5, "model": "couzin"},
        "flight": {"kp_tilt": 2.0},
    }


def test_a_malformed_assignment_is_rejected():
    with pytest.raises(ValueError, match="expected section.name=value"):
        parse(["cohesion=0.5"])
