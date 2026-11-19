"""Fast checks of machina's INAV fork (machina/README.md). Tests marked ``firmware`` run the SITL
binary (``machina/build.sh sitl``; INAV_SITL overrides its path) and are skipped without it."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from harness import sitl  # noqa: E402


def pytest_configure(config):
    config.addinivalue_line("markers", "firmware: runs this fork's SITL binary")


def pytest_collection_modifyitems(config, items):
    if Path(sitl.inav_bin()).exists():
        return
    skip = pytest.mark.skip(reason=f"no SITL binary at {sitl.inav_bin()}: run machina/build.sh sitl")
    for item in items:
        if "firmware" in item.keywords:
            item.add_marker(skip)


@pytest.fixture(scope="session")
def eeprom(tmp_path_factory) -> Path:
    """The quad's eeprom, written once over INAV's CLI (sitl.CLI_QUAD) for the low-level tests;
    flights configure their own."""
    path = tmp_path_factory.mktemp("inav") / "eeprom.bin"
    errors = sitl.configure(path)
    assert errors == []
    return path
