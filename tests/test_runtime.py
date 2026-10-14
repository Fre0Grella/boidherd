from pathlib import Path

from worldgen import write_runtime_inis


def make_controllers(root: Path) -> Path:
    controllers = root / "controllers"
    for name in ("drone", "referee"):
        (controllers / name).mkdir(parents=True)
        (controllers / name / f"{name}.py").write_text("", encoding="utf-8")
    (controllers / "notes").mkdir()  # a folder without a Python controller
    return controllers


def test_writes_an_absolute_command_for_each_python_controller(tmp_path: Path):
    controllers = make_controllers(tmp_path)
    python = tmp_path / "venv" / "python.exe"

    changed = write_runtime_inis(controllers, python)

    assert [p.parent.name for p in changed] == ["drone", "referee"]
    assert not (controllers / "notes" / "runtime.ini").exists()
    command = (controllers / "drone" / "runtime.ini").read_text(encoding="utf-8")
    assert f"[python]\nCOMMAND = {python.resolve().as_posix()}\n" in command
    assert Path(command.split("COMMAND = ")[1].strip()).is_absolute()


def test_second_run_changes_nothing(tmp_path: Path):
    controllers = make_controllers(tmp_path)
    python = tmp_path / "python.exe"
    write_runtime_inis(controllers, python)

    assert write_runtime_inis(controllers, python) == []
