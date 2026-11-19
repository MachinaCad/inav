"""The scripted pilot of the G0 probe (spikes/cl_p01d_inav/p01d.py ``Pilot``), on normalised RC.

Channels (INAV's AETR order, then AUX): roll, pitch, throttle, yaw, CH5 arm, CH6 (ANGLE always on),
CH7 ALTHOLD + POSHOLD, CH8 RTH; each -1 to 1 (1000 to 2000 us). Scenarios:

- ``hover``: arm once INAV's clock passes 8 s with no arming blocker, climb in ANGLE to 2.5 m,
  hold in POSHOLD for ``hold`` s.
- ``rth``: hover, then pitch out for 8 s, brake for 4 s, switch to RTH and wait for INAV to land
  and disarm itself.
- ``rcloss``: hover, then the receiver reports signal loss: INAV must enter its failsafe.
"""

from __future__ import annotations

from . import frames

PHASES = ["boot", "arm", "climb", "hold", "out", "brake", "rth", "landed", "rcloss"]


def _ch(x01: float) -> float:
    """A 0-to-1 channel (throttle, switches) as a normalised -1-to-1 channel."""
    return 2.0 * x01 - 1.0


class Pilot:
    def __init__(self, scenario: str = "hover", hold: float = 10.0, arm_after_s: float = 8.0):
        if scenario not in ("hover", "rth", "rcloss"):
            raise ValueError(f"unknown scenario {scenario!r}")
        self.sc, self.hold, self.arm_after_s = scenario, hold, arm_after_s
        self.phase, self.t0, self.events = "boot", 0.0, {}
        self.rc_failsafe = False

    def go(self, phase: str, t: float) -> None:
        self.phase, self.t0 = phase, t
        self.events[phase] = round(t, 4)

    def rc(self, t: float, alt_m: float, out: frames.Outputs | None) -> tuple[tuple | None, str | None]:
        """RC for simulated time ``t`` (s) given the true altitude and INAV's last answer:
        (channels, None) or (None, result) when the flight ends."""
        pitch, thr, arm, nav, rth = 0.0, 0.0, 0.0, 0.0, 0.0
        armed = bool(out and out.armed)
        dtp, ph = t - self.t0, self.phase
        if ph == "boot":
            if out and out.clock_ns > self.arm_after_s * 1e9 and not out.arming & frames.ARMING_BLOCKERS:
                self.go("arm", t)
            elif t > self.arm_after_s + 82:
                self.events["blockers"] = out.arming if out else None
                return None, "boot-timeout"
        elif ph == "arm":
            arm = 1.0
            if armed:
                self.go("climb", t)
            elif dtp > 3:
                self.events["blockers"] = out.arming if out else None
                return None, "arm-failed"
        elif ph == "climb":
            arm, thr = 1.0, 0.5
            if alt_m > 2.5:
                self.go("hold", t)
            elif dtp > 10:
                return None, "climb-failed"
        elif ph == "hold":
            arm, thr, nav = 1.0, 0.5, 1.0
            if dtp > self.hold:
                if self.sc == "hover":
                    return None, "done"
                self.go("out" if self.sc == "rth" else "rcloss", t)
        elif ph == "out":
            arm, thr, nav, pitch = 1.0, 0.5, 1.0, 0.3
            if dtp > 8:
                self.go("brake", t)
        elif ph == "brake":
            arm, thr, nav = 1.0, 0.5, 1.0
            if dtp > 4:
                self.go("rth", t)
        elif ph == "rth":
            arm, thr, nav, rth = 1.0, 0.5, 1.0, 1.0
            if not armed:
                self.go("landed", t)
            elif dtp > 120:
                return None, "rth-timeout"
        elif ph == "landed":
            arm, nav, rth = 1.0, 1.0, 1.0
            if dtp > 1:
                return None, "done"
        elif ph == "rcloss":
            arm, thr, nav = 1.0, 0.5, 1.0
            self.rc_failsafe = True
            if out and out.modes & frames.MODES["FAILSAFE"] and "failsafe" not in self.events:
                self.events["failsafe"] = round(t, 4)
            if dtp > 5:
                return None, "done"
        return (0.0, pitch, _ch(thr), 0.0, _ch(arm), _ch(0.0), _ch(nav), _ch(rth)) + (-1.0,) * 8, None
