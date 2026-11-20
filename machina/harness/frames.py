"""Wire frames: machina's native simulator frames (src/main/fc/machina_sim.h), the MSP v2 framing
of the HITL frame, and the X-Plane RREF packets and MOTR replies of the G0 probe (P1d).

Every layout here is checked against the C structs' sizes (272, 174, 214 and 24 bytes), and the
tests round-trip each one. All frames are little-endian.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

import numpy as np

G0 = 9.80665  # m/s^2 per g (INAV's GRAVITY_MSS)
MAG_T_PER_1024 = 5.0e-5  # T per 1024 INAV magnetometer counts (backend.py INAV_MAG_T_PER_UNIT)
VERSION = 1
MAGIC_SENSORS = 0x3153434D  # "MCS1"
MAGIC_OUTPUTS = 0x314F434D  # "MCO1"
MAGIC_HELLO = 0x3148434D  # "MCH1"
RC_CHANNELS = 16

HAS_IMU = 1 << 0
HAS_MAG = 1 << 1
HAS_BARO = 1 << 2
HAS_GNSS = 1 << 3
HAS_RC = 1 << 4
RC_FAILSAFE = 1 << 5
HAS_BATTERY = 1 << 6
HAS_RANGEFINDER = 1 << 7
HAS_AIRSPEED = 1 << 8
HITL_ENABLE = 1 << 15

OUT_LOCKSTEP = 1 << 0
OUT_ERROR = 1 << 1
OUT_HITL = 1 << 2

ERRORS = ("ok", "size", "magic", "version", "value", "sequence", "time")

SENSOR = struct.Struct("<IHHQQQ3d3d3dddddd3dffBBHq16fHHffff")
HITL = struct.Struct("<BBHIQ3f3f3fffiii3fBBHHH16fHffff")
OUTPUT = struct.Struct("<IHHQQQQBBBB8f8H8H3h3f3f3fIIII3f3f3fiii")
HELLO = struct.Struct("<IHH3BB12s")
assert (SENSOR.size, HITL.size, OUTPUT.size, HELLO.size) == (272, 174, 214, 24)

MSP2_MACHINA_SIMULATOR = 0x2F01  # this fork's MSP2 command (src/main/msp/msp_protocol_v2_inav.h)

# INAV runtime flags (src/main/fc/runtime_config.h)
ARMED = 1 << 2
SIMULATOR_MODE_HITL = 1 << 4
SIMULATOR_MODE_SITL = 1 << 5
ARMING_BLOCKERS = 0x7FFFFFC0
MODES = {"ANGLE": 1 << 0, "ALTHOLD": 1 << 3, "RTH": 1 << 4, "POSHOLD": 1 << 5, "FAILSAFE": 1 << 9}


@dataclass
class Sensors:
    """One sensor sample in machina's frames and SI: FLU body vectors, ENU velocity, WGS-84
    position. ``flags`` says which parts are new in this frame (HAS_*)."""

    seq: int
    t_ns: int
    step_ns: int
    flags: int = HAS_IMU
    gyro_rads: tuple = (0.0, 0.0, 0.0)
    specific_force_mps2: tuple = (0.0, 0.0, G0)
    field_T: tuple = (0.0, 0.0, 0.0)
    pressure_Pa: float = 101325.0
    temperature_K: float = 293.15
    lat_deg: float = 0.0
    lon_deg: float = 0.0
    alt_m: float = 0.0
    vel_enu_mps: tuple = (0.0, 0.0, 0.0)
    eph_m: float = 1.0
    epv_m: float = 1.5
    fix_type: int = 3
    num_sat: int = 16
    hdop: int = 100
    gnss_unix_s: int = 0
    rc: tuple = (0.0,) * RC_CHANNELS
    rssi: int = 1023
    battery_V: float = 16.8
    battery_A: float = 0.0
    range_m: float = -1.0
    airspeed_mps: float = 0.0


def encode_sensors(s: Sensors) -> bytes:
    """The SITL sensor frame (machinaSimSensorFrame_t)."""
    rc = tuple(s.rc) + (0.0,) * (RC_CHANNELS - len(s.rc))
    return SENSOR.pack(
        MAGIC_SENSORS,
        VERSION,
        s.flags,
        s.seq,
        s.t_ns,
        s.step_ns,
        *s.gyro_rads,
        *s.specific_force_mps2,
        *s.field_T,
        s.pressure_Pa,
        s.temperature_K,
        s.lat_deg,
        s.lon_deg,
        s.alt_m,
        *s.vel_enu_mps,
        s.eph_m,
        s.epv_m,
        s.fix_type,
        s.num_sat,
        s.hdop,
        s.gnss_unix_s,
        *rc,
        s.rssi,
        0,
        s.battery_V,
        s.battery_A,
        s.range_m,
        s.airspeed_mps,
    )


def decode_sensors(buf: bytes) -> Sensors:
    f = SENSOR.unpack(buf)
    assert f[0] == MAGIC_SENSORS and f[1] == VERSION
    return Sensors(
        seq=f[3],
        t_ns=f[4],
        step_ns=f[5],
        flags=f[2],
        gyro_rads=f[6:9],
        specific_force_mps2=f[9:12],
        field_T=f[12:15],
        pressure_Pa=f[15],
        temperature_K=f[16],
        lat_deg=f[17],
        lon_deg=f[18],
        alt_m=f[19],
        vel_enu_mps=f[20:23],
        eph_m=f[23],
        epv_m=f[24],
        fix_type=f[25],
        num_sat=f[26],
        hdop=f[27],
        gnss_unix_s=f[28],
        rc=f[29:45],
        rssi=f[45],
        battery_V=f[47],
        battery_A=f[48],
        range_m=f[49],
        airspeed_mps=f[50],
    )


def encode_hitl(s: Sensors) -> bytes:
    """The HITL sensor frame (machinaSimHitlFrame_t): float32, GNSS position in 1e-7 deg and cm."""
    rc = tuple(s.rc) + (0.0,) * (RC_CHANNELS - len(s.rc))
    return HITL.pack(
        VERSION,
        0,
        s.flags,
        s.seq & 0xFFFFFFFF,
        s.t_ns,
        *s.gyro_rads,
        *s.specific_force_mps2,
        *s.field_T,
        s.pressure_Pa,
        s.temperature_K,
        round(s.lat_deg * 1e7),
        round(s.lon_deg * 1e7),
        round(s.alt_m * 100),
        *s.vel_enu_mps,
        s.fix_type,
        s.num_sat,
        s.hdop,
        min(65535, round(s.eph_m * 100)),
        min(65535, round(s.epv_m * 100)),
        *rc,
        s.rssi,
        s.battery_V,
        s.battery_A,
        s.range_m,
        s.airspeed_mps,
    )


@dataclass
class Outputs:
    """One output frame (machinaSimOutputFrame_t): INAV's answer to a sensor frame."""

    flags: int
    seq: int
    t_ns: int
    clock_ns: int
    applied: int
    motor_count: int
    error: int
    failsafe_phase: int
    motors: np.ndarray
    motor_raw: tuple
    servo_raw: tuple
    att_decideg: tuple  # roll (right down +), pitch (nose down +), heading (cw from north)
    rates_dps: tuple
    pos_neu_cm: tuple
    vel_neu_cms: tuple
    arming: int
    modes: int
    state: int
    nav_state: int
    gyro_raw_dps: tuple
    acc_g: tuple
    mag_adc: tuple
    baro_pa: int
    gnss_lat_e7: int
    gnss_lon_e7: int
    raw: bytes = field(repr=False, default=b"")

    @property
    def lockstep(self) -> bool:
        return bool(self.flags & OUT_LOCKSTEP)

    @property
    def is_error(self) -> bool:
        return bool(self.flags & OUT_ERROR)

    @property
    def armed(self) -> bool:
        return bool(self.arming & ARMED)

    def pos_enu(self) -> np.ndarray:
        return np.array([self.pos_neu_cm[1], self.pos_neu_cm[0], self.pos_neu_cm[2]]) / 100.0

    def heading_deg(self) -> float:
        return self.att_decideg[2] / 10.0


def decode_outputs(buf: bytes) -> Outputs:
    if len(buf) != OUTPUT.size:
        raise ValueError(f"output frame of {len(buf)} bytes, expected {OUTPUT.size}")
    f = OUTPUT.unpack(buf)
    if f[0] != MAGIC_OUTPUTS or f[1] != VERSION:
        raise ValueError("not a machina output frame")
    return Outputs(
        flags=f[2],
        seq=f[3],
        t_ns=f[4],
        clock_ns=f[5],
        applied=f[6],
        motor_count=f[7],
        error=f[9],
        failsafe_phase=f[10],
        motors=np.array(f[11:19]),
        motor_raw=f[19:27],
        servo_raw=f[27:35],
        att_decideg=f[35:38],
        rates_dps=f[38:41],
        pos_neu_cm=f[41:44],
        vel_neu_cms=f[44:47],
        arming=f[47],
        modes=f[48],
        state=f[49],
        nav_state=f[50],
        gyro_raw_dps=f[51:54],
        acc_g=f[54:57],
        mag_adc=f[57:60],
        baro_pa=f[60],
        gnss_lat_e7=f[61],
        gnss_lon_e7=f[62],
        raw=bytes(buf),
    )


@dataclass
class Hello:
    lockstep: bool
    fc_version: tuple
    git_revision: str


def decode_hello(buf: bytes) -> Hello:
    if len(buf) != HELLO.size:
        raise ValueError(f"hello frame of {len(buf)} bytes, expected {HELLO.size}")
    magic, version, flags, a, b, c, _, rev = HELLO.unpack(buf)
    if magic != MAGIC_HELLO or version != VERSION:
        raise ValueError("not a machina hello frame")
    return Hello(bool(flags & OUT_LOCKSTEP), (a, b, c), rev.rstrip(b"\0").decode())


# ------------------------------------------------------------------------------------------------
# MSP v2 (for the HITL frame over a serial or TCP port)
# ------------------------------------------------------------------------------------------------


def crc8_dvb_s2(data: bytes, crc: int = 0) -> int:
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def msp_v2(cmd: int, payload: bytes, direction: bytes = b"<") -> bytes:
    body = struct.pack("<BHH", 0, cmd, len(payload)) + payload
    return b"$X" + direction + body + bytes([crc8_dvb_s2(body)])


def msp_v2_parse(buf: bytes) -> tuple[int, bytes, bytes] | None:
    """One MSP v2 reply from the start of ``buf``: (cmd, payload, rest), or None if incomplete.
    Raises ValueError on a bad checksum or an error reply ('!')."""
    start = buf.find(b"$X")
    if start < 0 or len(buf) < start + 9:
        return None
    direction = buf[start + 2 : start + 3]
    _, cmd, size = struct.unpack_from("<BHH", buf, start + 3)
    end = start + 8 + size + 1
    if len(buf) < end:
        return None
    body = buf[start + 3 : end - 1]
    if crc8_dvb_s2(body) != buf[end - 1]:
        raise ValueError("MSP v2 checksum mismatch")
    if direction == b"!":
        raise ValueError(f"MSP v2 error reply to command {cmd:#x}")
    return cmd, bytes(buf[start + 8 : end - 1]), bytes(buf[end:])


# ------------------------------------------------------------------------------------------------
# X-Plane (the G0 probe's interface, kept for w1-s7-inav-bridge)
# ------------------------------------------------------------------------------------------------

XITL_VERSION = 2.0  # dref 0: >= 2 makes INAV read mag, battery, RSSI, sats and fix from us
DREF_SEQ_LO, DREF_SEQ_HI = 42, 43  # this fork: the packet number in two 24-bit halves
DREF_TICK = (44, 45, 46, 47)  # the bridge's form: the tick in four 16-bit words (and 42, 43 in 16)
MOTR = struct.Struct("<5sIIB8f3h3f3fIIIB")  # patch 0001 (+ the lockstep byte of 0002)
MOTR_EXT = struct.Struct("<QQ")  # this fork: the echoed packet number and the clock in ns
MCH1 = struct.Struct("<4sIIQQBB8f3h3f3f3f3fIIIBBiii")  # the bridge's reply (w1-s7-inav-bridge)
MCH1_LOCKSTEP, MCH1_SEQ_ERROR, MCH1_FAILSAFE = 0x01, 0x02, 0x04


def bridge_number_drefs(seq: int, t_ns: int) -> dict[int, float]:
    """The bridge's numbering (plugins/core_autopilot/inav/packets.py `number_drefs`): the packet
    number in two 16-bit words (drefs 42, 43) and its tick in four (44 to 47)."""
    d = {DREF_SEQ_LO: float(seq & 0xFFFF), DREF_SEQ_HI: float(seq >> 16)}
    d.update({dref: float((t_ns >> (16 * i)) & 0xFFFF) for i, dref in enumerate(DREF_TICK)})
    return d


@dataclass
class Mch1:
    """The bridge's reply: packets applied, the last one's echoed number and tick, the clock (ns),
    flags, motors (INAV's motor count), arming and mode flags, navigation state."""

    rx: int
    seq: int
    t_ns: int
    clock_ns: int
    flags: int
    motors: np.ndarray
    arming: int
    modes: int
    nav_state: int


def parse_mch1(buf: bytes) -> Mch1 | None:
    if len(buf) < MCH1.size or buf[:4] != b"MCH1":
        return None
    f = MCH1.unpack_from(buf)
    return Mch1(f[1], f[2], f[3], f[4], f[5], np.array(f[7 : 7 + min(f[6], 8)]), f[30], f[31], f[32])


def rref_packet(values: dict[int, float]) -> bytes:
    """An X-Plane ``RREF,`` packet: (int32 id, float32 value) pairs."""
    return b"RREF," + b"".join(struct.pack("<if", i, v) for i, v in sorted(values.items()))


@dataclass
class Motr:
    rx: int
    us: int
    motors: np.ndarray
    att_decideg: tuple
    pos_neu_cm: tuple
    vel_neu_cms: tuple
    arming: int
    modes: int
    nav_state: int
    lockstep: int
    seq: int | None
    clock_ns: int | None


def parse_motr(buf: bytes) -> Motr | None:
    if len(buf) < MOTR.size or buf[:5] != b"MOTR\0":
        return None
    f = MOTR.unpack_from(buf)
    seq = clock = None
    if len(buf) >= MOTR.size + MOTR_EXT.size:
        seq, clock = MOTR_EXT.unpack_from(buf, MOTR.size)
    return Motr(f[1], f[2], np.array(f[4 : 4 + f[3]]), f[12:15], f[15:18], f[18:21], *f[21:25], seq, clock)


def xplane_euler(q: np.ndarray) -> tuple[float, float, float]:
    """X-Plane phi (right wing down +), theta (nose up +), psi (cw from north), degrees, from the
    FLU-body to ENU-world rotation matrix ``q`` (3x3)."""
    r = q
    phi = math.degrees(math.atan2(r[2, 1], r[2, 2]))
    theta = math.degrees(math.asin(max(-1.0, min(1.0, r[2, 0]))))
    psi = math.degrees(math.atan2(r[0, 0], r[1, 0])) % 360.0
    return phi, theta, psi
