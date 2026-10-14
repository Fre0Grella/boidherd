"""Run a Webots world without a window until a condition holds, then stop it.

Shared by the test tools. Start them from PowerShell: Webots crashes when
started from Git Bash. Only the Webots started here is stopped, together
with its controllers; a Webots window you have open is left alone.
"""

import os
import subprocess
import time
from collections.abc import Callable
from pathlib import Path

WEBOTS = Path(os.environ.get("WEBOTS_HOME", r"C:\Program Files\Webots"))
WEBOTS_EXE = WEBOTS / "msys64" / "mingw64" / "bin" / "webots.exe"


class RunError(Exception):
    pass


def run_until(world: Path, done: Callable[[], bool], timeout: float, port: int = 1297) -> float:
    """Run `world` (fast, no rendering) until `done()` is true; return the wall-clock seconds."""
    webots = subprocess.Popen(
        [str(WEBOTS_EXE), "--batch", "--mode=fast", "--no-rendering", "--minimize",
         f"--port={port}", str(world)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )  # fmt: skip
    started = time.monotonic()
    try:
        while not done():
            if time.monotonic() - started > timeout:
                raise RunError(f"timed out after {timeout:.0f} s")
            if webots.poll() is not None:
                raise RunError("Webots stopped before the end of the test")
            time.sleep(0.5)
    finally:
        subprocess.run(["taskkill", "/PID", str(webots.pid), "/T", "/F"], capture_output=True)
    return time.monotonic() - started
