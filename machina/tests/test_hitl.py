"""The HITL frame (MSP2_MACHINA_SIMULATOR) on the wall clock, with this fork's SITL binary in
configurator mode standing in for the board: sensors in machina's frames go in, the board's
answer comes back with the sequence echoed, and a gap takes the board out of simulator mode."""

from __future__ import annotations

import math

import pytest

from harness import frames, hitl

pytestmark = pytest.mark.firmware


@pytest.fixture
def board(eeprom, tmp_path):
    b = hitl.SitlBoard(eeprom, log=tmp_path / "board.log")
    yield b
    b.close()


def _frame(seq: int, **kw) -> frames.Sensors:
    s = frames.Sensors(
        seq=seq,
        t_ns=seq * 2_000_000,
        step_ns=0,
        flags=frames.HITL_ENABLE | frames.HAS_IMU | frames.HAS_MAG | frames.HAS_BARO | frames.HAS_BATTERY,
        gyro_rads=(0.5, -0.25, 0.125),
        specific_force_mps2=(0.0, 0.0, frames.G0),
        field_T=(25e-6, 0.0, -43.3e-6),
        pressure_Pa=95123.4,
        battery_V=25.2,
        battery_A=3.5,
    )
    for k, v in kw.items():
        setattr(s, k, v)
    return s


def test_hitl_frames_drive_the_board_on_its_own_clock(board):
    outs = [board.msp.hitl(_frame(k)) for k in range(50)]
    last = outs[-1]
    assert all(o.flags & frames.OUT_HITL and not o.is_error for o in outs)
    assert [o.seq for o in outs] == list(range(50))
    assert [o.applied for o in outs] == list(range(1, 51))
    assert [o.t_ns for o in outs] == [k * 2_000_000 for k in range(50)]
    assert last.arming & frames.SIMULATOR_MODE_HITL
    clocks = [o.clock_ns for o in outs]
    assert clocks == sorted(clocks) and clocks[-1] > clocks[0]  # the board's wall clock
    # The sensors as INAV holds them: rates in deg/s and specific force in g, unquantised, FLU;
    # the field in body axes at 1024 counts per 50 uT (HITL bypasses the compass alignment).
    assert last.gyro_raw_dps == pytest.approx(tuple(math.degrees(x) for x in (0.5, -0.25, 0.125)), rel=1e-6)
    assert last.acc_g == pytest.approx((0.0, 0.0, 1.0), abs=1e-6)
    assert last.mag_adc == pytest.approx((512.0, 0.0, -887.0))
    assert last.baro_pa == 95123


def test_a_gap_is_refused_and_ends_simulator_mode(board):
    for k in range(5):
        assert not board.msp.hitl(_frame(k)).is_error
    gap = board.msp.hitl(_frame(7))
    assert gap.is_error and frames.ERRORS[gap.error] == "sequence" and gap.seq == 5
    assert not gap.arming & frames.SIMULATOR_MODE_HITL and gap.applied == 0
    # The simulator starts again from seq 0.
    again = board.msp.hitl(_frame(0))
    assert not again.is_error and again.applied == 1 and again.arming & frames.SIMULATOR_MODE_HITL


def test_a_frame_without_enable_leaves_hitl(board):
    assert board.msp.hitl(_frame(0)).arming & frames.SIMULATOR_MODE_HITL
    off = board.msp.hitl(_frame(1, flags=0))
    assert not off.is_error and not off.arming & frames.SIMULATOR_MODE_HITL and off.applied == 0


def test_a_malformed_frame_is_refused(board):
    reply = board.msp.request(frames.MSP2_MACHINA_SIMULATOR, frames.encode_hitl(_frame(0))[:-3])
    out = frames.decode_outputs(reply)
    assert out.is_error and frames.ERRORS[out.error] == "size"
