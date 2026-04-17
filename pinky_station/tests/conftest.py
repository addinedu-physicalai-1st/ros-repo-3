"""Shared pytest fixtures for pinky_station tests."""

import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

# Ensure pinky_station is importable
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


def _find_pinky_binary() -> Path | None:
    """Locate the pinky_robot binary in common build directories."""
    repo_root = Path(__file__).resolve().parent.parent.parent
    candidates = [
        repo_root / "pinky_core" / "build" / "pinky_robot",
        repo_root / "build" / "pinky_robot",
    ]
    for candidate in candidates:
        if candidate.exists() and os.access(candidate, os.X_OK):
            return candidate
    return None


def _wait_for_tcp(host: str, port: int, timeout: float = 5.0) -> bool:
    """Poll until a TCP port is accepting connections."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            sock = socket.create_connection((host, port), timeout=0.2)
            sock.close()
            return True
        except (ConnectionRefusedError, OSError):
            time.sleep(0.1)
    return False


@pytest.fixture(scope="session")
def qapp():
    """Provides a QApplication instance for GUI tests."""
    from PyQt6.QtWidgets import QApplication
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    yield app
    # No explicit quit needed for tests, it cleans up when Python exits


@pytest.fixture(scope="session")
def cpp_server():
    """Start ``pinky_robot --mock`` as a subprocess for the test session.

    Yields the process object.  The server is terminated on teardown.
    Skips the entire session if the binary is not found.
    """
    binary = _find_pinky_binary()
    if binary is None:
        pytest.skip("pinky_robot binary not found (run cmake --build first)")

    proc = subprocess.Popen(
        [str(binary), "--mock"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    if not _wait_for_tcp("127.0.0.1", 9100):
        proc.terminate()
        proc.wait(timeout=5)
        pytest.fail("pinky_robot did not start within 5 seconds")

    yield proc

    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
