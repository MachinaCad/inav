"""Frames, ticks and the plant's sensors, without the firmware."""

from __future__ import annotations

import math
import struct
from fractions import Fraction

import numpy as np
import pytest

from harness import frames, plant, xplane


def test_layouts_match_machina_sim_h():
    # src/main/fc/machina_sim.c static-asserts the same sizes on the C side.
    assert (frames.SENSOR.size, frames.HITL.size, frames.OUTPUT.size, frames.HELLO.size) == (272, 174, 214, 24)
    assert frames.HITL.size + 2 <= 192  # INAV's MSP receive buffer (msp_serial.h)


def test_sensor_frame_round_trips_every_field():
    s = frames.Sensors(
        seq=7,
        t_ns=123_456_789,
        step_ns=333_334,
        flags=frames.HAS_IMU | frames.HAS_GNSS,
        gyro_rads=(0.1, -0.2, 0.3),
        specific_force_mps2=(1.0, 2.0, 9.0),
        field_T=(2e-5, -1e-5, -4e-5),
        pressure_Pa=95123.25,
        temperature_K=290.5,
        lat_deg=47.3977421,
        lon_deg=8.5455938,
        alt_m=488.25,
        vel_enu_mps=(1.0, -2.0, 0.5),
        eph_m=0.5,
        epv_m=0.75,
        fix_type=3,
        num_sat=12,
        hdop=90,
        gnss_unix_s=1_790_000_000,
        rc=tuple(np.linspace(-1, 1, 16)),
        rssi=1000,
        battery_V=24.5,
        battery_A=12.25,
        range_m=3.5,
        airspeed_mps=11.0,
    )
    back = frames.decode_sensors(frames.encode_sensors(s))
    for name in (
        "seq",
        "t_ns",
        "step_ns",
        "flags",
        "lat_deg",
        "lon_deg",
        "alt_m",
        "pressure_Pa",
        "fix_type",
        "gnss_unix_s",
    ):
        assert getattr(back, name) == getattr(s, name), name
    assert np.allclose(back.rc, s.rc, atol=1e-7)
    assert back.gyro_rads == s.gyro_rads and back.field_T == s.field_T


def test_crc8_dvb_s2_check_value_and_msp_v2_round_trip():
    assert frames.crc8_dvb_s2(b"123456789") == 0xBC  # the CRC-8/DVB-S2 check value
    payload = bytes(range(40))
    cmd, body, rest = frames.msp_v2_parse(frames.msp_v2(0x2F01, payload, b">") + b"tail")
    assert (cmd, body, rest) == (0x2F01, payload, b"tail")
    bad = bytearray(frames.msp_v2(0x2F01, payload))
    bad[-1] ^= 1
    with pytest.raises(ValueError, match="checksum"):
        frames.msp_v2_parse(bytes(bad))


@pytest.mark.parametrize("rate", [1000, 2000, 3000, Fraction(8000, 3)])
def test_ticks_are_integer_ns_without_drift(rate):
    period = Fraction(1_000_000_000) / Fraction(rate)
    k = 3_600 * int(rate)  # one hour of steps
    assert plant.tick(k, rate) == math.floor(k * period)
    steps = {plant.tick(i + 1, rate) - plant.tick(i, rate) for i in range(10_000)}
    assert steps <= {math.floor(period), math.ceil(period)}
    assert plant.tick(5, rate, t0=7) == 7 + plant.tick(5, rate)


def test_slow_sensors_are_new_only_at_their_ticks():
    hits = [k for k in range(100) if plant._due(plant.tick(k, 1000), 1_000_000, 100)]
    assert hits == list(range(0, 100, 10))
    hits = [
        k for k in range(30) if plant._due(plant.tick(k, 3000), plant.tick(k + 1, 3000) - plant.tick(k, 3000), 1000)
    ]
    assert hits == list(range(0, 30, 3))


def test_plant_senses_c2_at_rest():
    q = plant.Quad(1e-3, noise=plant.Noise.off(), yaw_deg=0.0)  # body x along east
    s = q.sense(0, 0, 1_000_000, (0.0,) * 16)
    assert np.allclose(s.specific_force_mps2, (0.0, 0.0, plant.G0), atol=1e-6)
    b = q.site.field_enu_T()  # ENU: north and down
    assert b[0] == pytest.approx(0.0, abs=1e-15) and b[1] > 0 and b[2] < 0
    # Facing east, north is the body's left (+y).
    assert s.field_T[1] == pytest.approx(b[1]) and s.field_T[0] == pytest.approx(0.0, abs=1e-12)
    assert s.flags & frames.HAS_MAG and s.flags & frames.HAS_BARO and s.flags & frames.HAS_GNSS
    assert s.pressure_Pa == pytest.approx(plant.isa_pressure(q.site.alt_m + q.pos()[2]), rel=1e-12)
    assert q.pos()[2] == pytest.approx(0.03, abs=2e-4)  # settled on the ground before t = 0


def test_xplane_drefs_follow_the_frozen_inav_xplane_convention():
    # backend.py InavXplane: gyro (wx, -wy, -wz) deg/s; accel (-fx, fy, fz) / g0; mag (-my, -mx, -mz)
    # in units of 50 uT; local velocity (east, up, south).
    q = plant.Quad(1e-3, noise=plant.Noise.off())
    s = frames.Sensors(
        seq=0,
        t_ns=0,
        step_ns=1,
        gyro_rads=(0.1, 0.2, 0.3),
        specific_force_mps2=(1.0, 2.0, 3.0),
        field_T=(1e-5, 2e-5, 3e-5),
        vel_enu_mps=(4.0, 5.0, 6.0),
    )
    d = xplane.make_drefs(q, s, (0.0,) * 8, 0, 5)
    assert [d[17], d[18], d[19]] == pytest.approx([math.degrees(0.1), -math.degrees(0.2), -math.degrees(0.3)])
    assert [d[14], d[15], d[16]] == pytest.approx([-1.0 / plant.G0, 2.0 / plant.G0, 3.0 / plant.G0])
    assert [d[34], d[35], d[36]] == pytest.approx([-0.4, -0.2, -0.6])
    assert [d[5], d[6], d[7]] == [4.0, 6.0, -5.0]
    assert (d[frames.DREF_SEQ_LO], d[frames.DREF_SEQ_HI]) == (5.0, 0.0)
    big = xplane.make_drefs(q, s, (0.0,) * 8, 0, (3 << 24) | 77)
    assert (big[frames.DREF_SEQ_LO], big[frames.DREF_SEQ_HI]) == (77.0, 3.0)


def test_float32_cannot_carry_a_real_home_but_the_native_frame_can():
    # Why the native frame carries degrees as float64: at 47.4 deg a float32 step is ~0.4 m.
    lat = 47.3977421
    f32 = struct.unpack("<f", struct.pack("<f", lat))[0]
    assert abs(f32 - lat) * 111_319.5 > 0.05
    back = frames.decode_sensors(frames.encode_sensors(frames.Sensors(seq=0, t_ns=0, step_ns=1, lat_deg=lat)))
    assert round(back.lat_deg * 1e7) == round(lat * 1e7)
