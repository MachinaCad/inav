"""The X-Plane path (--sim=xp), kept for machina's w1-s7-inav-bridge card, in this fork's
lockstep: virtual time from process start, a blocking receive, every packet a step (the
connection probe no longer consumes one), the reply after the step, and the packet number
echoed when the bridge sends it (drefs 42 and 43, two 24-bit halves).

``make_drefs`` is the G0 probe's mapping (spikes/cl_p01d_inav/p01d.py), which backend.py's
``InavXplane`` convention freezes: gyro (wx, -wy, -wz) deg/s, acceleration (-fx, fy, fz) in g,
magnetometer (-m_y, -m_x, -m_z) in units of 50 uT, X-Plane Euler angles and local velocity.
"""

from __future__ import annotations

import hashlib
import math
import os
import select
import socket
import subprocess
import time
from pathlib import Path

import numpy as np

from . import frames, plant, sitl
from .pilot import Pilot

INHG_PA = 3386.39


def make_drefs(quad: plant.Quad, s: frames.Sensors, rc, beat: int, seq: int | None) -> dict[int, float]:
    phi, theta, psi = frames.xplane_euler(quad.rot())
    fx, fy, fz = s.specific_force_mps2
    wx, wy, wz = s.gyro_rads
    mx, my, mz = (b / frames.MAG_T_PER_1024 for b in s.field_T)
    ve, vn, vu = s.vel_enu_mps
    gs = math.hypot(ve, vn)
    d = {
        0: frames.XITL_VERSION,
        1: s.lat_deg,
        2: s.lon_deg,
        3: s.alt_m,
        4: s.alt_m - quad.site.alt_m,
        5: ve,
        6: vu,
        7: -vn,
        8: gs,
        9: gs,
        10: phi,
        11: theta,
        12: psi,
        13: math.degrees(math.atan2(ve, vn)) % 360.0,
        14: -fx / frames.G0,
        15: fy / frames.G0,
        16: fz / frames.G0,
        17: math.degrees(wx),
        18: -math.degrees(wy),
        19: -math.degrees(wz),
        20: s.pressure_Pa / INHG_PA,
        21: 0.0,
        22: 1.0,
        31: float(beat % 1000),
        32: float(s.num_sat),
        33: 2.0,  # INAV's GPS_FIX_3D
        34: -my,
        35: -mx,
        36: -mz,
        37: 65535.0,
        38: s.battery_V,
        39: s.battery_A,
        40: 1023.0,
        41: 1.0 if s.flags & frames.RC_FAILSAFE else 0.0,
    }
    # X-Plane joystick: roll, pitch, yaw in -1..1; throttle and CH5..CH8 in 0..1.
    rc8 = list(rc[:8])
    for i in (2, 4, 5, 6, 7):
        rc8[i] = (rc8[i] + 1.0) / 2.0
    d.update({23 + i: float(x) for i, x in enumerate(rc8)})
    if seq is not None:
        d[frames.DREF_SEQ_LO] = float(seq & 0xFFFFFF)
        d[frames.DREF_SEQ_HI] = float(seq >> 24)
    return d


class XplaneLink:
    def __init__(
        self, eeprom: Path, *, dt_us: int = 1000, lockstep: bool = True, useimu: bool = True, log: Path | None = None
    ):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.port = self.sock.getsockname()[1]
        self.log = open(log or os.devnull, "w")  # noqa: SIM115 (closed in close())
        args = [sitl.inav_bin(), "--sim=xp", "--simip=127.0.0.1", f"--simport={self.port}"]
        args += [f"--path={eeprom}", f"--tcpbaseport={sitl.free_tcp_base()}"]
        args += ["--useimu"] * useimu + ([f"--lockstep={dt_us}"] if lockstep else [])
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT, cwd=eeprom.parent)
        self.peer = None

    def wait_first(self, timeout: float = 30.0) -> None:
        self.sock.settimeout(timeout)
        try:
            _, self.peer = self.sock.recvfrom(4096)  # INAV's RREF requests
        except TimeoutError:
            raise sitl.FirmwareError(f"no datagram within {timeout} s (exit {self.proc.poll()})") from None

    def step(self, drefs: dict[int, float], timeout: float = sitl.DEADLOCK_S) -> frames.Motr:
        self.sock.sendto(frames.rref_packet(drefs), self.peer)
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left <= 0:
                raise sitl.FirmwareError(f"deadlock: no MOTR within {timeout:.0f} s")
            if select.select([self.sock], [], [], min(left, 0.5))[0]:
                m = frames.parse_motr(self.sock.recv(4096))
                if m is not None:
                    return m
            elif self.proc.poll() is not None:
                raise sitl.FirmwareError(f"INAV exited with status {self.proc.returncode}")

    def close(self) -> int | None:
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.sock.close()
        self.log.close()
        return self.proc.returncode


def fly(
    scenario: str = "hover",
    *,
    seed: int = 0,
    dt_us: int = 1000,
    hold: float = 10.0,
    number: bool = True,
    tlimit_s: float = 200.0,
    tag: str = "",
    site: plant.Site | None = None,
) -> sitl.Flight:
    """One lockstep flight on the X-Plane path. Packet k must be answered by a MOTR reporting
    k + 1 packets applied, INAV's clock at (k + 1) * dt and, when numbered, seq k."""
    sitl.RUNS.mkdir(parents=True, exist_ok=True)
    tag = tag or f"xp-{scenario}-{os.getpid()}-{time.monotonic_ns()}"
    eeprom = sitl.RUNS / f"{tag}.bin"
    errors = sitl.configure(eeprom)
    if errors:
        raise RuntimeError(f"CLI errors: {errors}")
    # The X-Plane path carries lat/lon as float32: keep home near (0, 0), as the G0 probe did.
    site = site or plant.Site(lat_deg=0.01, lon_deg=0.01, alt_m=100.0)
    quad = plant.Quad(dt_us * 1e-6, seed, site=site, rates=plant.SensorRates(1000, 1000, 10))
    pilot = Pilot(scenario, hold)
    link = XplaneLink(eeprom, dt_us=dt_us, log=sitl.RUNS / f"{tag}.log")
    h_out, h_traj = hashlib.sha256(), hashlib.sha256()
    rows, m, motors = [], None, np.zeros(4)
    every = max(1, round(20000 / dt_us))
    checks = {"clock_start_ns": None, "steps_checked": 0}
    rc = (0.0, 0.0, -1.0, 0.0, -1.0, -1.0, -1.0, -1.0)
    result, k, w0 = None, 0, time.perf_counter()
    dt_ns = dt_us * 1000
    try:
        link.wait_first()
        while result is None:
            t_ns = k * dt_ns
            t_s = t_ns * 1e-9
            out = _as_outputs(m)
            if k % every == 0:
                cmd, result = pilot.rc(t_s, quad.pos()[2], out)
                if result:
                    break
                rc = cmd
                rows.append(sitl._row(t_s, quad, out, pilot))
            s = quad.sense(k, t_ns, dt_ns, rc, rc_failsafe=pilot.rc_failsafe)
            m = link.step(make_drefs(quad, s, rc, k, k if number else None))
            expect = (k + 1, (k + 1) * dt_ns, k if number else m.seq, 1)
            got = (m.rx, m.clock_ns, m.seq, m.lockstep)
            if got != expect:
                raise sitl.FirmwareError(f"packet {k}: MOTR (rx, clock_ns, seq, lockstep) {got}, expected {expect}")
            if checks["clock_start_ns"] is None:
                checks["clock_start_ns"] = m.clock_ns - dt_ns
            checks["steps_checked"] += 1
            quad.step(motors)
            motors = m.motors[:4].copy()
            h_out.update(np.array(m.motors, dtype=np.float32).tobytes())
            h_traj.update(quad.d.qpos.tobytes())
            k += 1
            if t_s > tlimit_s:
                result = "tlimit"
    except sitl.FirmwareError as e:
        result = f"firmware: {e}"
    finally:
        link.close()
        eeprom.unlink(missing_ok=True)
    return sitl.Flight(
        tag=tag,
        result=result,
        steps=k,
        sim_s=k * dt_us * 1e-6,
        wall_s=time.perf_counter() - w0,
        events=pilot.events,
        rows=np.array(rows) if rows else np.zeros((0, 17)),
        outputs_sha256=h_out.hexdigest(),
        trajectory_sha256=h_traj.hexdigest(),
        clock_checks=checks,
    )


def _as_outputs(m: frames.Motr | None) -> frames.Outputs | None:
    """A MOTR in the shape the pilot reads."""
    if m is None:
        return None
    return frames.Outputs(
        flags=frames.OUT_LOCKSTEP if m.lockstep else 0,
        seq=m.seq or 0,
        t_ns=0,
        clock_ns=m.clock_ns or m.us * 1000,
        applied=m.rx,
        motor_count=len(m.motors),
        error=0,
        failsafe_phase=0,
        motors=m.motors,
        motor_raw=(),
        servo_raw=(),
        att_decideg=m.att_decideg,
        rates_dps=(0.0, 0.0, 0.0),
        pos_neu_cm=m.pos_neu_cm,
        vel_neu_cms=m.vel_neu_cms,
        arming=m.arming,
        modes=m.modes,
        state=0,
        nav_state=m.nav_state,
        gyro_raw_dps=(),
        acc_g=(),
        mag_adc=(),
        baro_pa=0,
        gnss_lat_e7=0,
        gnss_lon_e7=0,
    )
