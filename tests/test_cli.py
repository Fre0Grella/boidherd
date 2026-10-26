import shutil
from pathlib import Path

import pytest

from worldgen.cli import main

REPO = Path(__file__).parent.parent


@pytest.fixture
def project(tmp_path: Path) -> Path:
    """A project with only the example map, so the test writes nothing in the repo."""
    (tmp_path / "maps").mkdir()
    shutil.copy(REPO / "tests" / "maps" / "l_hall.toml", tmp_path / "maps")
    for name in ("drone", "referee"):
        (tmp_path / "controllers" / name).mkdir(parents=True)
        (tmp_path / "controllers" / name / f"{name}.py").write_text("", encoding="utf-8")
    return tmp_path


def test_builds_the_world_and_the_controller_settings(
    project: Path, capsys: pytest.CaptureFixture[str]
):
    assert main(["l_hall", "-n", "12", "-j", "2", "-s", "3"], root=project) == 0
    assert (
        capsys.readouterr().out == "worlds/l_hall.wbt: 12 drones (2 leaders), 2 walls, 16 pillars\n"
    )
    assert (project / "worlds" / "l_hall.wbt").read_text(encoding="utf-8").count(
        "Crazyflie {"
    ) == 12
    assert (project / "controllers" / "drone" / "runtime.ini").exists()
    assert not (project / "worlds" / "l_hall.png").exists()


def test_preview_draws_a_png(project: Path, capsys: pytest.CaptureFixture[str]):
    assert main(["l_hall", "--preview"], root=project) == 0
    assert "worlds/l_hall.png: preview" in capsys.readouterr().out
    assert (project / "worlds" / "l_hall.png").read_bytes()[:4] == b"\x89PNG"


def test_a_path_works_as_well_as_a_name(project: Path):
    assert main([str(project / "maps" / "l_hall.toml")], root=project) == 0


def test_unknown_map_lists_the_maps(project: Path, capsys: pytest.CaptureFixture[str]):
    assert main(["nope"], root=project) == 2
    assert capsys.readouterr().err == "error: no map called 'nope' (maps: l_hall)\n"


def test_errors_are_one_line_and_write_nothing(project: Path, capsys: pytest.CaptureFixture[str]):
    assert main(["l_hall", "-n", "300"], root=project) == 1
    err = capsys.readouterr().err
    assert err.startswith("error: could only place ") and err.count("\n") == 1
    assert not (project / "worlds" / "l_hall.wbt").exists()
