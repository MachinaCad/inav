"""Sensor conformance through INAV's own sensor code (the output frame echoes what INAV holds):
neither clipped nor quantised beyond INAV's representation of real sensors, in INAV's body axes
after its own calibration and alignment chain."""

from __future__ import annotations

import math
import struct

import pytest

from harness import frames, plant, sitl

pytestmark = pytest.mark.firmware

HOME = plant.Site()  # 47.397742 N, 8.545594 E, 488 m: float32 cannot carry it (test_frames)
FIELD_FLU = (12e-6, 20e-6, -43e-6)  # T


def _frame(k: int, **kw) -> frames.Sensors:
    s = frames.Sensors(
        seq=k,
        t_ns=k * 1_000_000,
        step_ns=1_000_000,
        flags=frames.HAS_IMU | frames.HAS_MAG | frames.HAS_BARO | frames.HAS_GNSS,
        field_T=FIELD_FLU,
        pressure_Pa=95597.4,
        lat_deg=HOME.lat_deg + 1.234567e-5,
        lon_deg=HOME.lon_deg - 2.345678e-5,
        alt_m=HOME.alt_m,
    )
    for name, v in kw.items():
        setattr(s, name, v)
    return s


@pytest.fixture(scope="module")
def answers(tmp_path_factory):
    """Boot on still sensors for 9 s (INAV calibrates its gyro on them and is ready to arm by
    8 s), then 1.5 s of each test case, long enough for INAV's accelerometer filter (a 15 Hz
    biquad that overshoots a step by 4 %) to settle. Returns the last answer of each phase."""
    tmp = tmp_path_factory.mktemp("sensors")
    eeprom = tmp / "eeprom.bin"
    assert sitl.configure(eeprom) == []
    link = sitl.NativeLink(eeprom, log=tmp / "inav.log")
    link.wait_hello()
    out = {}
    try:
        k = 0
        for _ in range(9000):
            link.step(_frame(k))
            k += 1
        cases = {
            "fast": dict(
                gyro_rads=(math.radians(2500.0), math.radians(-0.0004), 0.0),
                specific_force_mps2=(5 * frames.G0, -20 * frames.G0, 0.001),
            ),
            "baro": dict(pressure_Pa=95597.6),
        }
        for name, kw in cases.items():
            for _ in range(1500):
                last = link.step(_frame(k, **kw))
                k += 1
            out[name] = last
    finally:
        link.close()
    return out


def test_gyro_is_neither_clipped_nor_quantised(answers):
    # The X-Plane path stops at 2,048 deg/s and steps by 1/16 deg/s.
    raw = answers["fast"].gyro_raw_dps
    assert raw[0] == pytest.approx(2500.0, rel=1e-6)
    assert raw[1] == pytest.approx(-0.0004, rel=1e-4)
    assert raw[2] == 0.0


def test_accelerometer_is_not_clipped(answers):
    # The X-Plane path clips at 3.34 g in 1 mm/s^2 steps; INAV itself flags clipping above
    # 15.9 g (a real sensor's +-16 g), but the value passes unclipped. acc.accADCf is after
    # INAV's accelerometer low-pass filter, whose float32 coefficients leave a DC gain within
    # 3e-5 of one.
    acc = answers["fast"].acc_g
    assert acc[0] == pytest.approx(5.0, rel=1e-4)
    assert acc[1] == pytest.approx(-20.0, rel=1e-4)
    assert acc[2] == pytest.approx(0.001 / frames.G0, rel=1e-3)


def test_magnetometer_reaches_the_estimator_in_body_axes(answers):
    """The field goes in as FLU; the fake compass reads it in its own axes (the inverse of the
    default CW270_DEG_FLIP alignment), and INAV's calibration and alignment turn it back. With
    the test configuration's magzero_x = 1 (it marks the compass calibrated), INAV subtracts one
    count from the sensor's x axis, which is the body's -y."""
    counts = [round(b / frames.MAG_T_PER_1024 * 1024) for b in FIELD_FLU]
    assert answers["fast"].mag_adc == (counts[0], counts[1] + 1, counts[2])


def test_barometer_is_rounded_to_integer_pascal(answers):
    assert answers["fast"].baro_pa == 95597
    assert answers["baro"].baro_pa == 95598


def test_gnss_position_is_exact_in_1e7_degrees(answers):
    lat = HOME.lat_deg + 1.234567e-5
    lon = HOME.lon_deg - 2.345678e-5
    out = answers["fast"]
    assert (out.gnss_lat_e7, out.gnss_lon_e7) == (round(lat * 1e7), round(lon * 1e7))
    f32 = struct.unpack("<f", struct.pack("<f", lat))[0]
    assert round(f32 * 1e7) != out.gnss_lat_e7  # what the X-Plane path would have delivered
