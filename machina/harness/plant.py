"""The plant: a 1 kg X quad in MuJoCo with seeded sensor noise, ported from the G0 probe
(spikes/cl_p01d_inav/p01d.py ``Quad``), and machina's integer-nanosecond ticks (C5).

Sensors are produced in machina's frames and SI only: FLU body rates and specific force, the
field in tesla, static pressure in pascal, WGS-84 position and ENU velocity. Each sensor has its
own rate, and a sample is flagged new only at its tick, as from hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

import numpy as np

from . import frames

G0 = frames.G0
M_PER_DEG = 111319.5  # INAV's metres per degree of latitude (navigation_private.h)


def tick(k: int, rate_hz: Fraction | int, t0: int = 0) -> int:
    """Tick k of a rational rate in integer ns: t0 + floor(k * 1e9 / rate) (C5's phase
    accumulator; no drift however long the run)."""
    period = Fraction(1_000_000_000) / Fraction(rate_hz)
    return t0 + (k * period.numerator) // period.denominator


def isa_pressure(h_m: float) -> float:
    """ISA static pressure (Pa) at geopotential height h (m), troposphere."""
    return 101325.0 * (1.0 - 2.25577e-5 * h_m) ** 5.25588


@dataclass(frozen=True)
class Site:
    """Where the run flies: home on WGS-84 and the Earth's field there."""

    lat_deg: float = 47.397742  # a realistic home: float32 would resolve it only to ~0.4 m
    lon_deg: float = 8.545594
    alt_m: float = 488.0
    field_uT: float = 50.0
    inclination_deg: float = 60.0
    declination_deg: float = 0.0  # INAV's SITL forces its own declination to 0

    def field_enu_T(self, rotate_deg: float = 0.0) -> np.ndarray:
        """The Earth's field in ENU (pointing north and down), optionally turned about up."""
        a = math.radians(self.declination_deg + rotate_deg)
        inc = math.radians(self.inclination_deg)
        h = math.cos(inc) * self.field_uT * 1e-6
        return np.array([math.sin(a) * h, math.cos(a) * h, -math.sin(inc) * self.field_uT * 1e-6])

    def to_lla(self, p_enu: np.ndarray) -> tuple[float, float, float]:
        lat = self.lat_deg + p_enu[1] / M_PER_DEG
        lon = self.lon_deg + p_enu[0] / (M_PER_DEG * math.cos(math.radians(self.lat_deg)))
        return lat, lon, self.alt_m + p_enu[2]


@dataclass(frozen=True)
class SensorRates:
    """Sample rates (Hz): IMU at every step; the others at their own ticks."""

    mag_hz: int = 100
    baro_hz: int = 50
    gnss_hz: int = 10


@dataclass(frozen=True)
class Noise:
    """1-sigma white noise per sample: the G0 probe's values in SI."""

    acc_mps2: float = 0.05
    gyro_rads: float = 0.002
    field_T: float = 0.005 * 50e-6
    baro_Pa: float = 1.2  # about 0.1 m
    gnss_m: float = 0.05

    @classmethod
    def off(cls) -> Noise:
        return cls(0.0, 0.0, 0.0, 0.0, 0.0)


QUAD_XML = """<mujoco><option timestep="{dt}" gravity="0 0 -9.80665" integrator="RK4"/>
<worldbody><geom type="plane" size="500 500 0.1"/>
<body name="quad" pos="0 0 0.03"><freejoint/>
<inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.018"/>
<geom type="box" size="0.1 0.1 0.03" mass="0"/><site name="imu"/></body></worldbody>
<sensor><accelerometer site="imu"/><gyro site="imu"/></sensor></mujoco>"""


class Quad:
    """1 kg X quad: thrust Tmax*u (hover at u = 0.4), 30 ms motor lag, yaw torque kq*T, linear
    drag; MuJoCo RK4 at the step. Body axes are FLU; it starts facing ``yaw_deg`` (ENU, from east,
    counter-clockwise)."""

    D = 0.12 / math.sqrt(2)
    XY = np.array([[-D, -D], [D, -D], [-D, D], [D, D]])  # INAV QUADX order RR, FR, RL, FL (FLU)
    SPIN = np.array([-1.0, 1.0, 1.0, -1.0])  # +1 = CCW seen from above (props in)

    def __init__(
        self,
        dt_s: float,
        seed: int = 0,
        *,
        site: Site | None = None,
        noise: Noise | None = None,
        rates: SensorRates | None = None,
        yaw_deg: float = 0.0,
        field_rotate_deg: float = 0.0,
        hover_u: float = 0.4,
        tau_s: float = 0.03,
        kq: float = 0.016,
        drag: float = 0.25,
        settle_s: float = 0.5,
    ) -> None:
        import mujoco

        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_string(QUAD_XML.format(dt=dt_s))
        self.d = mujoco.MjData(self.m)
        half = math.radians(yaw_deg) / 2
        self.d.qpos[3:7] = [math.cos(half), 0.0, 0.0, math.sin(half)]
        mujoco.mj_forward(self.m, self.d)
        self.dt = dt_s
        self.tmax = G0 / (4 * hover_u)
        self.alpha = 1 - math.exp(-dt_s / tau_s)
        self.kq, self.drag = kq, drag
        self.u = np.zeros(4)
        self.rng = np.random.default_rng(seed)
        self.site = site or Site()
        self.noise = noise or Noise()
        self.rates = rates or SensorRates()
        self.b_enu = self.site.field_enu_T(field_rotate_deg)
        self.gnss: tuple | None = None
        # Settle on the ground before t = 0: the box starts just touching the plane, in free fall.
        for _ in range(round(settle_s / dt_s)):
            mujoco.mj_step(self.m, self.d)

    def rot(self) -> np.ndarray:
        """FLU body to ENU world."""
        return self.d.xmat[1].reshape(3, 3).copy()

    def pos(self) -> np.ndarray:
        return self.d.qpos[:3].copy()

    def vel(self) -> np.ndarray:
        return self.d.qvel[:3].copy()

    def step(self, cmd: np.ndarray) -> None:
        self.u += (np.clip(cmd, 0.0, 1.0) - self.u) * self.alpha
        thrust = self.tmax * self.u
        tau_b = np.array([self.XY[:, 1] @ thrust, -self.XY[:, 0] @ thrust, -self.kq * (self.SPIN @ thrust)])
        r = self.rot()
        self.d.xfrc_applied[1, :3] = r @ np.array([0.0, 0.0, thrust.sum()]) - self.drag * self.d.qvel[:3]
        self.d.xfrc_applied[1, 3:] = r @ tau_b
        self.mj.mj_step(self.m, self.d)

    def sense(self, seq: int, t_ns: int, step_ns: int, rc, *, rc_failsafe: bool = False) -> frames.Sensors:
        """The sensor frame for tick ``t_ns``: IMU every step; magnetometer, barometer and GNSS
        flagged new only at their own ticks (an integer number of ns since t = 0)."""
        n = self.noise
        r, p = self.rot(), self.pos()
        rng = self.rng
        flags = frames.HAS_IMU | frames.HAS_RC | frames.HAS_BATTERY
        if rc_failsafe:
            flags |= frames.RC_FAILSAFE
        s = frames.Sensors(seq=seq, t_ns=t_ns, step_ns=step_ns, flags=flags, rc=tuple(rc))
        s.gyro_rads = tuple(self.d.sensordata[3:6] + rng.normal(0.0, n.gyro_rads, 3))
        s.specific_force_mps2 = tuple(self.d.sensordata[0:3] + rng.normal(0.0, n.acc_mps2, 3))
        if _due(t_ns, step_ns, self.rates.mag_hz):
            s.flags |= frames.HAS_MAG
            s.field_T = tuple(r.T @ self.b_enu + rng.normal(0.0, n.field_T, 3))
        if _due(t_ns, step_ns, self.rates.baro_hz):
            s.flags |= frames.HAS_BARO
            s.pressure_Pa = isa_pressure(self.site.alt_m + p[2]) + rng.normal(0.0, n.baro_Pa)
        if _due(t_ns, step_ns, self.rates.gnss_hz):
            s.flags |= frames.HAS_GNSS
            self.gnss = (*self.site.to_lla(p + rng.normal(0.0, n.gnss_m, 3)), tuple(self.vel()))
        if self.gnss is not None:
            s.lat_deg, s.lon_deg, s.alt_m, s.vel_enu_mps = self.gnss
        return s


def _due(t_ns: int, step_ns: int, rate_hz: int) -> bool:
    """A sensor at rate_hz has a sample due within [t_ns, t_ns + step_ns)."""
    period = Fraction(1_000_000_000, rate_hz)
    k = -(-t_ns * period.denominator // period.numerator)  # first tick index at or after t_ns
    return tick(k, rate_hz) < t_ns + step_ns
