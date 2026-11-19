"""Determinism: identical-seed flights on the native backend in lockstep are bit-identical (every
output frame, byte for byte, and the trajectory), also when they run side by side; a different
seed gives a different flight."""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor

import pytest

from harness import sitl

pytestmark = pytest.mark.firmware


def _hover(seed: int, tag: str) -> sitl.Flight:
    return sitl.fly("hover", seed=seed, hold=4.0, tag=tag)


def test_identical_seeds_are_bit_identical_side_by_side(tmp_path):
    with ThreadPoolExecutor(3) as ex:
        flights = list(ex.map(lambda i: _hover(0, f"test-det-{i}"), range(3)))
    assert all(f.result == "done" and sitl.completed(f) for f in flights), [f.result for f in flights]
    s = sitl.spread(flights)
    assert s["bit_identical_outputs"] and s["bit_identical_trajectories"]
    assert s["pairwise_max_m"] == 0.0
    other = _hover(1, "test-det-seed1")
    assert other.result == "done"
    assert other.outputs_sha256 != flights[0].outputs_sha256
    for f in (*flights, other):
        sitl.remove(f.tag)
