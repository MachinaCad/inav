"""Lockstep on the native backend: the clock from process start, exact integer-ns ticks, numbered
and echoed packets, no step on held sensors, and refusal of gaps, repeats and wrong ticks."""

from __future__ import annotations

import select
import time
from fractions import Fraction

import pytest

from harness import frames, plant, sitl

pytestmark = pytest.mark.firmware


def _boot(eeprom, tmp_path, **kw) -> sitl.NativeLink:
    link = sitl.NativeLink(eeprom, log=tmp_path / "inav.log", **kw)
    link.wait_hello()
    return link


def _frame(k: int, rate=1000, t0: int = 0) -> frames.Sensors:
    t = plant.tick(k, rate, t0)
    return frames.Sensors(seq=k, t_ns=t, step_ns=plant.tick(k + 1, rate, t0) - t)


def test_hello_declares_lockstep_and_the_build(eeprom, tmp_path):
    link = _boot(eeprom, tmp_path)
    try:
        assert link.hello.lockstep and link.hello.fc_version == (9, 1, 0)
        assert len(link.hello.git_revision) >= 7
    finally:
        link.close()


@pytest.mark.parametrize(("rate", "t0"), [(1000, 0), (Fraction(7000, 3), 5_000_000_123)])
def test_clock_starts_at_the_scenario_start_and_follows_every_tick(eeprom, tmp_path, rate, t0):
    """The firmware clock is 0 at process start and at the first packet, whatever that packet's
    tick (t0), and after packet k it is exactly t_k+1 - t0 in integer ns, also for a rate whose
    period is not a whole number of ns (2,333.33 Hz: 428,571.43 ns)."""
    link = _boot(eeprom, tmp_path)
    try:
        for k in range(3000):
            out = link.step(_frame(k, rate, t0))
            t_k = plant.tick(k, rate, t0)
            assert (out.seq, out.t_ns, out.applied) == (k, t_k, k + 1)
            assert out.clock_ns == plant.tick(k + 1, rate, t0) - t0
            assert out.lockstep and not out.is_error
    finally:
        link.close()


def test_non_integer_periods_step_by_floor_ticks(eeprom, tmp_path):
    rate = 3000  # 333,333.33 ns: steps of 333,333 and 333,334 ns
    link = _boot(eeprom, tmp_path)
    try:
        steps = set()
        for k in range(3000):
            out = link.step(_frame(k, rate))
            steps.add(out.clock_ns - plant.tick(k, rate))
        assert out.clock_ns == 1_000_000_000  # one second exactly, no drift
        assert steps == {333_333, 333_334}
    finally:
        link.close()


def test_no_step_on_held_sensors(eeprom, tmp_path):
    """The receive blocks without a timeout: while the simulator holds back packet k + 1 for 2.5 s
    (the X-Plane path used to step after 1 s), the firmware sends nothing and its clock stands."""
    link = _boot(eeprom, tmp_path)
    try:
        for k in range(200):
            out = link.step(_frame(k))
        held = out.clock_ns
        end = time.monotonic() + 2.5
        while time.monotonic() < end:
            assert not select.select([link.sock], [], [], 0.1)[0], "the firmware answered without a packet"
        assert link.proc.poll() is None
        out = link.step(_frame(200))
        assert (out.applied, out.clock_ns) == (201, held + 1_000_000)
    finally:
        link.close()


@pytest.mark.parametrize(
    ("bad", "error"),
    [
        (lambda: _frame(12), "sequence"),  # a gap: 12 after 9
        (lambda: _frame(9), "sequence"),  # a repeat
        (lambda: frames.Sensors(seq=10, t_ns=10_000_001, step_ns=1_000_000), "time"),  # 1 ns off its tick
        (lambda: frames.Sensors(seq=10, t_ns=10_000_000, step_ns=0), "time"),  # no step
    ],
)
def test_gaps_repeats_and_wrong_ticks_stop_the_firmware(eeprom, tmp_path, bad, error):
    link = _boot(eeprom, tmp_path)
    try:
        for k in range(10):
            link.step(_frame(k))
        link.send(frames.encode_sensors(bad()))
        r, _, _ = select.select([link.sock], [], [], 10.0)
        assert r, "no error frame"
        out = frames.decode_outputs(link.sock.recv(4096))
        assert out.is_error and frames.ERRORS[out.error] == error and out.seq == 10 and out.applied == 10
        assert link.proc.wait(10) == 3
    finally:
        link.close()


def test_a_malformed_frame_stops_the_firmware(eeprom, tmp_path):
    link = _boot(eeprom, tmp_path)
    try:
        link.send(frames.encode_sensors(_frame(0))[:-1])
        assert select.select([link.sock], [], [], 10.0)[0]
        out = frames.decode_outputs(link.sock.recv(4096))
        assert out.is_error and frames.ERRORS[out.error] == "size"
        assert link.proc.wait(10) == 3
    finally:
        link.close()


def test_wall_clock_mode_declares_itself(eeprom, tmp_path):
    """The fallback (C5's implementation.clock = wall_clock): hello and every answer say so, and
    the firmware clock follows the host's."""
    link = _boot(eeprom, tmp_path, lockstep=False)
    try:
        assert not link.hello.lockstep
        t0 = time.monotonic()
        outs = []
        for k in range(200):
            outs.append(link.step(_frame(k)))
            time.sleep(0.002)
        wall = time.monotonic() - t0
        assert not any(o.lockstep for o in outs)
        assert [o.applied for o in outs] == list(range(1, 201))
        span = (outs[-1].clock_ns - outs[0].clock_ns) * 1e-9
        assert 0.5 * wall < span < 1.5 * wall
    finally:
        link.close()


def test_xplane_path_numbers_packets_and_stops_on_a_gap(eeprom, tmp_path):
    """--sim=xp --lockstep with numbered packets (drefs 42, 43): each MOTR reports rx = k + 1,
    seq k and the clock at (k + 1) dt; a gap stops the firmware with status 3."""
    from harness import xplane

    link = xplane.XplaneLink(eeprom, log=tmp_path / "xp.log")
    quad = plant.Quad(1e-3, noise=plant.Noise.off(), site=plant.Site(lat_deg=0.01, lon_deg=0.01, alt_m=100.0))
    rc = (0.0,) * 8
    try:
        link.wait_first()
        for k in range(10):
            s = quad.sense(k, k * 1_000_000, 1_000_000, rc)
            m = link.step(xplane.make_drefs(quad, s, rc, k, k))
            assert (m.rx, m.seq, m.clock_ns, m.lockstep) == (k + 1, k, (k + 1) * 1_000_000, 1)
        s = quad.sense(10, 10_000_000, 1_000_000, rc)
        link.sock.sendto(frames.rref_packet(xplane.make_drefs(quad, s, rc, 10, 12)), link.peer)
        assert link.proc.wait(10) == 3
    finally:
        link.close()
