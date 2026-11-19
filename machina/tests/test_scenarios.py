"""The G0 probe's scenarios on this fork: hover and return-to-home on the native backend in
lockstep, the receiver failsafe, and the X-Plane path that machina's w1-s7-inav-bridge card uses."""

from __future__ import annotations

import numpy as np
import pytest

from harness import frames, sitl, xplane

pytestmark = pytest.mark.firmware


def test_hover_holds_position_and_altitude():
    f = sitl.fly("hover", hold=15.0, tag="test-hover")
    s = f.summary()
    assert f.result == "done" and sitl.completed(f), s
    assert f.clock_checks["clock_start_ns"] == 0 and f.clock_checks["steps_checked"] == f.steps
    assert s["events"]["climb"] == pytest.approx(8.06, abs=0.05)  # INAV armed at its 8 s
    # The card's criterion: altitude within +-0.2 m over 10 s, from 5 s into the hold.
    R = f.rows
    w = R[(R[:, 0] >= s["events"]["hold"] + 5.0)]
    assert np.ptp(w[:, 0]) >= 9.9
    assert np.ptp(w[:, 3]) <= 0.4 and abs(w[:, 3] - w[:, 3].mean()).max() <= 0.2
    assert s["hold_drift_m"] < 0.1
    assert max(abs(e) for e in s["est_pos_err_m"]["mean"]) < 0.15
    sitl.remove(f.tag)


def test_return_to_home_lands_within_a_metre():
    f = sitl.fly("rth", hold=5.0, tag="test-rth", tlimit_s=150.0)
    s = f.summary()
    assert f.result == "done" and sitl.completed(f), s
    assert s["max_dist_m"] > 10.0 and s["land_dist_m"] < 1.0 and s["land_alt_m"] < 0.2
    sitl.remove(f.tag)


def test_receiver_loss_enters_inav_failsafe():
    f = sitl.fly("rcloss", hold=3.0, tag="test-rcloss")
    assert f.result == "done", f.summary()
    assert "failsafe" in f.events and f.events["failsafe"] - f.events["rcloss"] < 2.0
    assert f.last.modes & frames.MODES["FAILSAFE"]
    sitl.remove(f.tag)


def test_xplane_path_in_lockstep_counts_every_packet_and_hovers():
    """--sim=xp --lockstep: the clock starts at 0 and steps dt per packet, the probe packet counts,
    the MOTR echoes the packet number, and the quad hovers (the bridge card's interface)."""
    f = xplane.fly("hover", hold=6.0, tag="test-xp")
    assert f.result == "done" and sitl.completed(f), f.summary()
    assert f.clock_checks["clock_start_ns"] == 0 and f.clock_checks["steps_checked"] == f.steps
    sitl.remove(f.tag)
