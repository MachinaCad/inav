"""The X-Plane path in the INAV bridge's form (w1-s7-inav-bridge, Machina's plugins/core_autopilot/inav):
a Unix datagram socket (--simip=unix:<path>), packets numbered in 16-bit words with their tick
(drefs 42 to 47), MCH1 replies, the clock at each packet's tick, init whole at the first packet,
and a gap latched in every later reply's flags. FW2's own form is tested in test_lockstep.py."""

from __future__ import annotations

import os
import socket
import subprocess
import time

import pytest

from harness import frames, plant, sitl, xplane

pytestmark = pytest.mark.firmware
SITE = plant.Site(lat_deg=0.01, lon_deg=0.01, alt_m=100.0)


class BridgeLink:
    """INAV launched as the bridge launches it, on a Unix datagram socket in `where`."""

    def __init__(self, eeprom, where, *, lockstep: bool = True) -> None:
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.path = str(where / "sim.sock")
        self.sock.bind(self.path)
        self.peer = self.path + ".fw"
        args = [sitl.inav_bin(), "--sim=xp", f"--simip=unix:{self.path}", f"--path={eeprom}",
                f"--tcpbaseport={sitl.free_tcp_base()}", "--useimu"] + ["--lockstep"] * lockstep
        self.log = open(where / "inav.log", "w")  # noqa: SIM115 (closed in close())
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT, cwd=eeprom.parent)
        self.sock.settimeout(30.0)
        self.sock.recv(65536)  # INAV's first registration: it is bound and listening

    def step(self, drefs: dict[int, float], timeout: float = sitl.DEADLOCK_S) -> frames.Mch1:
        self.sock.sendto(frames.rref_packet(drefs), self.peer)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.sock.settimeout(max(0.01, end - time.monotonic()))
            m = frames.parse_mch1(self.sock.recv(65536))
            if m is not None:
                return m
        raise sitl.FirmwareError(f"no MCH1 within {timeout:.0f} s (exit {self.proc.poll()})")

    def close(self) -> None:
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.sock.close()
        self.log.close()
        os.unlink(self.path)


def _drefs(quad, k: int, seq: int, t_ns: int) -> dict[int, float]:
    rc = (0.0, 0.0, -1.0, 0.0, -1.0, -1.0, -1.0, -1.0)
    s = quad.sense(k, k * 1_000_000, 1_000_000, rc)
    d = xplane.make_drefs(quad, s, rc, k, None)
    d.update(frames.bridge_number_drefs(seq, t_ns))
    return d


def test_bridge_form_ticks_counts_the_first_and_flags_a_gap(eeprom, tmp_path):
    """Packet k at tick k ms: MCH1 counts k + 1 packets, echoes seq k and tick k ms, and INAV's
    clock reads the tick, from reply 0 on (init ran whole at the first packet: four motors).
    A gap is answered, with the sequence flag, and every later reply keeps it."""
    quad = plant.Quad(1e-3, noise=plant.Noise.off(), site=SITE)
    link = BridgeLink(eeprom, tmp_path)
    try:
        for k in range(300):
            m = link.step(_drefs(quad, k, k, k * 1_000_000))
            assert (m.rx, m.seq, m.t_ns, m.clock_ns) == (k + 1, k, k * 1_000_000, k * 1_000_000), k
            assert m.flags & frames.MCH1_LOCKSTEP and not m.flags & frames.MCH1_SEQ_ERROR, k
            assert len(m.motors) == 4, k
        m = link.step(_drefs(quad, 300, 302, 300_000_000))
        assert m.flags & frames.MCH1_SEQ_ERROR and m.rx == 301 and m.seq == 302
        m = link.step(_drefs(quad, 301, 301, 301_000_000))
        assert m.flags & frames.MCH1_SEQ_ERROR and link.proc.poll() is None
    finally:
        link.close()


def test_bridge_form_counts_packets_past_16_bits(eeprom, tmp_path):
    """65,600 packets at rest: the bridge's numbering passes 2^16 (its high word 1) with no
    sequence error, every packet counted and the clock at the last tick."""
    quad = plant.Quad(1e-3, noise=plant.Noise.off(), site=SITE)
    link = BridgeLink(eeprom, tmp_path)
    try:
        base = _drefs(quad, 0, 0, 0)
        n = 65_600
        for k in range(n):
            base.update(frames.bridge_number_drefs(k, k * 1_000_000))
            m = link.step(base)
            assert not m.flags & frames.MCH1_SEQ_ERROR, k
        assert (m.rx, m.seq, m.t_ns, m.clock_ns) == (n, n - 1, (n - 1) * 1_000_000, (n - 1) * 1_000_000)
        assert frames.bridge_number_drefs(n - 1, 0)[frames.DREF_SEQ_HI] == 1.0
    finally:
        link.close()


def test_bridge_form_paced_counts_the_first_packet(eeprom, tmp_path):
    """Paced (the bridge's wall-clock fallback): the connection probe only peeks, so the first
    packet is applied and counted, and the replies are MCH1."""
    quad = plant.Quad(1e-3, noise=plant.Noise.off(), site=SITE)
    link = BridgeLink(eeprom, tmp_path, lockstep=False)
    try:
        seen = []
        for k in range(400):
            link.sock.sendto(frames.rref_packet(_drefs(quad, k, k, k * 1_000_000)), link.peer)
            time.sleep(0.001)
            link.sock.settimeout(0.0)
            try:
                while True:
                    m = frames.parse_mch1(link.sock.recv(65536))
                    if m is not None:
                        seen.append(m)
            except BlockingIOError:
                pass
        assert seen, "no MCH1 reply"
        assert not any(m.flags & frames.MCH1_SEQ_ERROR for m in seen)
        assert all(m.seq == m.rx - 1 for m in seen if m.rx)
    finally:
        link.close()
