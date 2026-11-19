"""Running this fork's SITL binary: configuration over INAV's CLI, then flights on the native
backend (lockstep or wall clock) or on the X-Plane path, closed around the plant.

Every launch binds OS-chosen UDP ports and records every failure as a result, as the G0 probe
learnt to. Logs, time series and summaries go under ``$MACHINA_CACHE/inav-fork/runs``.
"""

from __future__ import annotations

import hashlib
import json
import os
import select
import socket
import subprocess
import time
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path

import numpy as np

from . import frames, plant
from .pilot import PHASES, Pilot

CACHE = Path(os.environ.get("MACHINA_CACHE", Path.home() / ".cache" / "machina")) / "inav-fork"
RUNS = CACHE / "runs"
DEADLOCK_S = 30.0  # a lockstep answer later than this ends the run as a deadlock

# The quad's configuration, written over INAV's CLI before each flight: simulated receiver,
# fake GNSS, compass and barometer, the QUADX mixer, ARM on CH5, ANGLE always, ALTHOLD and POSHOLD
# on CH7, RTH on CH8 (the G0 probe's settings).
CLI_QUAD = [
    "set receiver_type = SIM (SITL)",
    "set gps_provider = FAKE",
    "feature GPS",
    "set mag_hardware = FAKE",
    "set baro_hardware = FAKE",
    "set platform_type = MULTIROTOR",
    "set model_preview_type = 3",
    "mmix reset",
    "mmix 0 1.0 -1.0 1.0 -1.0",
    "mmix 1 1.0 -1.0 -1.0 1.0",
    "mmix 2 1.0 1.0 1.0 1.0",
    "mmix 3 1.0 1.0 -1.0 -1.0",
    "aux 0 0 0 1700 2100",
    "aux 1 1 1 900 2100",
    "aux 2 3 2 1300 2100",
    "aux 3 11 2 1700 2100",
    "aux 4 10 3 1700 2100",
    "set nav_mc_althold_throttle = MID_STICK",
    "set nav_mc_hover_thr = 1400",
    "set acczero_x = 1",
    "set magzero_x = 1",
]


def inav_bin() -> str:
    return os.environ.get("INAV_SITL", str(CACHE / "inav-sitl"))


def free_tcp_base(count: int = 8) -> int:
    """A base port with ``count`` free TCP ports above it (INAV binds one per UART), below the
    kernel's ephemeral range so that no OS-chosen port collides with it later."""
    rng = np.random.default_rng()
    for _ in range(200):
        base = int(rng.integers(20000, 32000 - count))
        socks = []
        try:
            for i in range(count):
                s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
                s.bind(("::", base + i))
                socks.append(s)
            return base
        except OSError:
            continue
        finally:
            for s in socks:
                s.close()
    raise RuntimeError("no free TCP port range for INAV's UARTs")


def _cli(sock: socket.socket, cmd: str, timeout: float = 5.0) -> str:
    sock.sendall(cmd.encode() + (b"" if cmd == "#" else b"\n"))
    buf, end = b"", time.monotonic() + timeout
    while not buf.endswith(b"# ") and time.monotonic() < end:
        r, _, _ = select.select([sock], [], [], 0.2)
        if r:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk
    return buf.decode(errors="replace")


def configure(eeprom: Path, cli: list[str] = CLI_QUAD, timeout: float = 60.0) -> list[str]:
    """Write ``eeprom`` from INAV's defaults and ``cli`` over the CLI (INAV started without a
    simulator: configurator mode); returns the CLI's error lines (none expected)."""
    eeprom.unlink(missing_ok=True)
    base = free_tcp_base()
    log = eeprom.with_suffix(".configure.log")
    args = [inav_bin(), f"--path={eeprom}", f"--tcpbaseport={base}"]
    with log.open("w") as fh:
        proc = subprocess.Popen(args, stdout=fh, stderr=subprocess.STDOUT, cwd=eeprom.parent)
    out: list[str] = []
    try:
        deadline = time.monotonic() + timeout
        while True:
            try:
                sock = socket.create_connection(("127.0.0.1", base), timeout=5)
                break
            except OSError:
                if time.monotonic() > deadline or proc.poll() is not None:
                    raise RuntimeError(f"INAV's CLI did not open (see {log})") from None
                time.sleep(0.05)
        with sock:
            out = [_cli(sock, c) for c in ["#", *cli]]
            before = eeprom.stat().st_mtime_ns if eeprom.exists() else 0
            sock.sendall(b"save\n")
            while time.monotonic() < deadline:  # INAV writes the eeprom, then reboots
                if eeprom.exists() and eeprom.stat().st_mtime_ns != before and eeprom.stat().st_size == 32768:
                    break
                time.sleep(0.05)
            time.sleep(0.3)
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except subprocess.TimeoutExpired:
            proc.kill()
    if not eeprom.exists():
        raise RuntimeError(f"INAV did not save its eeprom (see {log})")
    return [line for o in out for line in o.splitlines() if "nvalid" in line or "rror" in line]


class FirmwareError(RuntimeError):
    """The firmware refused a frame, exited or stopped answering."""


class NativeLink:
    """The simulator side of --sim=machina: one UDP socket; INAV is launched against it."""

    def __init__(self, eeprom: Path, *, lockstep: bool = True, useimu: bool = True, log: Path | None = None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.port = self.sock.getsockname()[1]
        self.lockstep = lockstep
        self.log = open(log or os.devnull, "w")  # noqa: SIM115 (closed in close())
        args = [inav_bin(), "--sim=machina", "--simip=127.0.0.1", f"--simport={self.port}"]
        args += [f"--path={eeprom}", f"--tcpbaseport={free_tcp_base()}"]
        args += ["--useimu"] * useimu + ["--lockstep"] * lockstep
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT, cwd=eeprom.parent)
        self.peer = None
        self.hello: frames.Hello | None = None

    def wait_hello(self, timeout: float = 30.0) -> frames.Hello:
        self.sock.settimeout(timeout)
        try:
            buf, self.peer = self.sock.recvfrom(4096)
        except TimeoutError:
            raise FirmwareError(f"no hello within {timeout} s (exit {self.proc.poll()})") from None
        self.hello = frames.decode_hello(buf)
        return self.hello

    def send(self, frame: bytes) -> None:
        self.sock.sendto(frame, self.peer)

    def recv(self, timeout: float = DEADLOCK_S) -> frames.Outputs:
        end = time.monotonic() + timeout
        while True:
            left = end - time.monotonic()
            if left <= 0:
                raise FirmwareError(f"deadlock: no answer within {timeout:.0f} s")
            r, _, _ = select.select([self.sock], [], [], min(left, 0.5))
            if r:
                out = frames.decode_outputs(self.sock.recv(4096))
                if out.is_error:
                    raise FirmwareError(f"frame refused: {frames.ERRORS[out.error]}, expected seq {out.seq}")
                return out
            if self.proc.poll() is not None:
                raise FirmwareError(f"INAV exited with status {self.proc.returncode}")

    def step(self, s: frames.Sensors) -> frames.Outputs:
        self.send(frames.encode_sensors(s))
        return self.recv()

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


@dataclass
class Flight:
    """One flight's record: the result, events, 50 Hz rows, and digests of every output frame
    and of the trajectory (bit-identical runs have identical digests)."""

    tag: str
    result: str
    steps: int
    sim_s: float
    wall_s: float
    events: dict
    rows: np.ndarray
    outputs_sha256: str
    trajectory_sha256: str
    clock_checks: dict = field(default_factory=dict)
    first: frames.Outputs | None = None
    last: frames.Outputs | None = None

    def summary(self) -> dict:
        return {
            "tag": self.tag,
            "result": self.result,
            "steps": self.steps,
            "sim_s": round(self.sim_s, 6),
            "wall_s": round(self.wall_s, 3),
            "speedup": round(self.sim_s / self.wall_s, 3) if self.wall_s else None,
            "events": self.events,
            "outputs_sha256": self.outputs_sha256,
            "trajectory_sha256": self.trajectory_sha256,
            "clock_checks": self.clock_checks,
            **metrics(self.rows, self.events),
        }


def fly(
    scenario: str = "hover",
    *,
    seed: int = 0,
    rate_hz: int | Fraction = 1000,
    t0_ns: int = 0,
    hold: float = 10.0,
    lockstep: bool = True,
    noise: plant.Noise | None = None,
    site: plant.Site | None = None,
    yaw_deg: float = 0.0,
    field_rotate_deg: float = 0.0,
    tlimit_s: float = 200.0,
    tag: str = "",
    eeprom: Path | None = None,
    cli: list[str] = CLI_QUAD,
    arm_after_s: float = 8.0,
) -> Flight:
    """One flight on the native backend, closed around the plant. In lockstep, step k sends
    sensors(k) at tick t0 + floor(k * 1e9 / rate) and must get back seq k, the same tick, the
    firmware clock at the end of the step (ns since its start) and k + 1 frames applied; any
    other answer ends the run. The plant then integrates over [t_k, t_k+1] with the motors the
    firmware had decided by t_k (one step of computation delay). On the wall clock, frames go out
    every period of wall time and the plant uses INAV's latest answer."""
    RUNS.mkdir(parents=True, exist_ok=True)
    tag = tag or f"{scenario}-{'ls' if lockstep else 'wc'}-{os.getpid()}-{time.monotonic_ns()}"
    own_eeprom = eeprom is None
    eeprom = eeprom or RUNS / f"{tag}.bin"
    if own_eeprom:
        errors = configure(eeprom, cli)
        if errors:
            raise RuntimeError(f"CLI errors: {errors}")
    period = Fraction(1_000_000_000) / Fraction(rate_hz)
    quad = plant.Quad(
        float(period) * 1e-9, seed, noise=noise, site=site, yaw_deg=yaw_deg, field_rotate_deg=field_rotate_deg
    )
    pilot = Pilot(scenario, hold, arm_after_s=arm_after_s)
    link = NativeLink(eeprom, lockstep=lockstep, log=RUNS / f"{tag}.log")
    h_out, h_traj = hashlib.sha256(), hashlib.sha256()
    rows: list[list[float]] = []
    every = max(1, round(Fraction(1, 50) / (period / 1_000_000_000)))  # 50 Hz rows
    out: frames.Outputs | None = None
    first = None
    motors = np.zeros(4)
    checks = {"clock_start_ns": None, "steps_checked": 0, "late_sends": 0}
    rc = (0.0, 0.0, -1.0, 0.0, -1.0, -1.0, -1.0, -1.0) + (-1.0,) * 8
    result, k, w0 = None, 0, time.perf_counter()
    try:
        link.wait_hello()
        t_next = time.perf_counter()
        while result is None:
            t_k = plant.tick(k, rate_hz, t0_ns)
            step = plant.tick(k + 1, rate_hz, t0_ns) - t_k
            t_s = (t_k - t0_ns) * 1e-9
            if k % every == 0:
                cmd, result = pilot.rc(t_s, quad.pos()[2], out)
                if result:
                    break
                rc = cmd
                rows.append(_row(t_s, quad, out, pilot))
            s = quad.sense(k, t_k, step, rc, rc_failsafe=pilot.rc_failsafe)
            if lockstep:
                out = link.step(s)
                expect = (k, t_k, t_k - t0_ns + step, k + 1)
                got = (out.seq, out.t_ns, out.clock_ns, out.applied)
                if got != expect:
                    raise FirmwareError(f"step {k}: answered (seq, t_ns, clock_ns, applied) {got}, expected {expect}")
                if first is None:
                    first, checks["clock_start_ns"] = out, out.clock_ns - step
                checks["steps_checked"] += 1
                quad.step(motors)
                motors = out.motors[:4].copy()
            else:
                while (rem := t_next - time.perf_counter()) > 0:
                    if select.select([link.sock], [], [], rem)[0]:
                        out = frames.decode_outputs(link.sock.recv(4096))
                checks["late_sends"] += time.perf_counter() - t_next > 0.010
                link.send(frames.encode_sensors(s))
                t_next += float(period) * 1e-9
                quad.step(out.motors[:4] if out is not None else np.zeros(4))
                first = first or out
            h_out.update(out.raw if out is not None else b"")
            h_traj.update(quad.d.qpos.tobytes())
            k += 1
            if t_s > tlimit_s:
                result = "tlimit"
    except FirmwareError as e:
        result = f"firmware: {e}"
    finally:
        code = link.close()
        if own_eeprom:
            eeprom.unlink(missing_ok=True)
    if code not in (None, 0, -15) and not result.startswith("firmware"):
        result = f"inav-exit-{code}"
    flight = Flight(
        tag=tag,
        result=result,
        steps=k,
        sim_s=float((plant.tick(k, rate_hz)) * 1e-9),
        wall_s=time.perf_counter() - w0,
        events=pilot.events,
        rows=np.array(rows) if rows else np.zeros((0, 17)),
        outputs_sha256=h_out.hexdigest(),
        trajectory_sha256=h_traj.hexdigest(),
        clock_checks=checks,
        first=first,
        last=out,
    )
    np.save(RUNS / f"{tag}.npy", flight.rows)
    (RUNS / f"{tag}.json").write_text(json.dumps(flight.summary(), indent=1, default=float))
    return flight


def _row(t: float, quad: plant.Quad, out: frames.Outputs | None, pilot: Pilot) -> list[float]:
    """t, true position ENU (3), true X-Plane-style phi/theta/psi (3), INAV's roll/pitch/heading
    in the same sense (3), INAV's position ENU (3), armed, flight modes, phase, INAV's clock (s)."""
    ia = (0.0, 0.0, 0.0)
    ip = np.zeros(3)
    if out is not None:
        ia = (out.att_decideg[0] / 10, -out.att_decideg[1] / 10, out.att_decideg[2] / 10)
        ip = out.pos_enu()
    return [
        t,
        *quad.pos(),
        *frames.xplane_euler(quad.rot()),
        *ia,
        *ip,
        float(out.armed) if out else 0.0,
        float(out.modes) if out else 0.0,
        float(PHASES.index(pilot.phase)),
        out.clock_ns * 1e-9 if out else 0.0,
    ]


def wrap(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def metrics(R: np.ndarray, ev: dict) -> dict:
    """The G0 probe's flight metrics over the 50 Hz rows."""
    out: dict = {}
    if "hold" in ev and len(R):
        h = R[(R[:, 0] >= ev["hold"]) & (R[:, 0] <= ev.get("out", ev.get("rcloss", R[-1, 0])))]
        if len(h):
            out["hold_drift_m"] = round(float(np.hypot(*(h[:, 1:3] - h[0, 1:3]).T).max()), 4)
            out["hold_alt_m_min_max"] = [round(float(h[:, 3].min()), 3), round(float(h[:, 3].max()), 3)]
        w = R[(R[:, 0] >= ev["hold"] + 5.0) & (R[:, 0] <= ev.get("out", ev.get("rcloss", R[-1, 0])))]
        if len(w):
            e = wrap(w[:, 7:10] - w[:, 4:7])
            out["att_err_deg"] = {
                "rms_roll": round(float(np.sqrt((e[:, 0] ** 2).mean())), 4),
                "rms_pitch": round(float(np.sqrt((e[:, 1] ** 2).mean())), 4),
                "hdg_mean": round(float(e[:, 2].mean()), 4),
                "hdg_std": round(float(e[:, 2].std()), 4),
            }
            pe = w[:, 10:13] - w[:, 1:4]
            out["est_pos_err_m"] = {"mean": pe.mean(0).round(4).tolist(), "std": pe.std(0).round(4).tolist()}
            out["hover_std_m"] = w[:, 1:4].std(0).round(4).tolist()
            out["hover_alt_m"] = round(float(w[:, 3].mean()), 4)
    if "rth" in ev and len(R):
        pre = R[R[:, 0] < ev["rth"]]
        out["max_dist_m"] = round(float(np.hypot(pre[:, 1], pre[:, 2]).max()), 3)
        out["land_dist_m"] = round(float(np.hypot(R[-1, 1], R[-1, 2])), 3)
        out["rth_s"] = round(ev["landed"] - ev["rth"], 3) if "landed" in ev else None
        out["land_alt_m"] = round(float(R[-1, 3]), 4)
    return out


def completed(f: Flight | dict) -> bool:
    """The G0 probe's completion test: armed, held within 1 m at 1.5 to 4 m; for rth also flew
    out at least 10 m and landed within 1 m of home below 0.2 m, disarmed by INAV."""
    r = f.summary() if isinstance(f, Flight) else f
    ev, alt = r.get("events", {}), r.get("hold_alt_m_min_max", [0.0, 0.0])
    if r.get("result") != "done" or "hold" not in ev:
        return False
    if r.get("hold_drift_m", 99.0) > 1.0 or not 1.5 <= alt[0] <= alt[1] <= 4.0:
        return False
    if "rth" in ev:
        return (
            r.get("max_dist_m", 0.0) >= 10.0
            and "landed" in ev
            and r.get("land_dist_m", 99.0) <= 1.0
            and r.get("land_alt_m", 9.0) < 0.2
        )
    return True


def spread(flights: list[Flight]) -> dict:
    """Spread of identical-seed flights over their common rows (true positions): radius (largest
    distance from the ensemble mean) and pairwise (largest distance between two flights)."""
    n = min(len(f.rows) for f in flights)
    X = np.stack([f.rows[:n, 1:4] for f in flights])
    rad = np.linalg.norm(X - X.mean(0), axis=2).max()
    pw = max(float(np.linalg.norm(X[i] - X[j], axis=1).max()) for i in range(len(X)) for j in range(i))
    return {
        "flights": len(flights),
        "rows": n,
        "radius_max_m": float(rad),
        "pairwise_max_m": pw,
        "bit_identical_outputs": len({f.outputs_sha256 for f in flights}) == 1,
        "bit_identical_trajectories": len({f.trajectory_sha256 for f in flights}) == 1,
    }


def remove(tag: str) -> None:
    for ext in ("json", "npy", "log", "bin", "configure.log"):
        (RUNS / f"{tag}.{ext}").unlink(missing_ok=True)


__all__ = [
    "CLI_QUAD",
    "Flight",
    "FirmwareError",
    "NativeLink",
    "completed",
    "configure",
    "fly",
    "inav_bin",
    "metrics",
    "remove",
    "spread",
]
