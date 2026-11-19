"""The firmware half of hardware-in-the-loop: machina's HITL frame over MSP v2
(MSP2_MACHINA_SIMULATOR), on the wall clock. On the bench it goes to a real board's USB port; the
checks here send it to this fork's SITL binary in configurator mode (no --sim), whose UART1 is a
TCP socket and whose clock is the wall clock, as a board's is.
"""

from __future__ import annotations

import socket
import subprocess
import time
from pathlib import Path

from . import frames, sitl


class MspLink:
    """MSP v2 over a byte stream (TCP to SITL's UART1 here; a serial port on a board)."""

    def __init__(self, sock: socket.socket):
        self.sock = sock
        self.buf = b""

    def request(self, cmd: int, payload: bytes, timeout: float = 2.0) -> bytes:
        self.sock.sendall(frames.msp_v2(cmd, payload))
        end = time.monotonic() + timeout
        while True:
            parsed = frames.msp_v2_parse(self.buf)
            if parsed is not None:
                got, body, self.buf = parsed
                if got == cmd:
                    return body
                continue
            left = end - time.monotonic()
            if left <= 0:
                raise TimeoutError(f"no MSP reply to {cmd:#x}")
            self.sock.settimeout(left)
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("MSP link closed")
            self.buf += chunk

    def hitl(self, s: frames.Sensors) -> frames.Outputs:
        """One HITL frame and the board's answer (errors are returned, not raised)."""
        return frames.decode_outputs(self.request(frames.MSP2_MACHINA_SIMULATOR, frames.encode_hitl(s)))


class SitlBoard:
    """This fork's SITL binary without a simulator: a stand-in board on the wall clock."""

    def __init__(self, eeprom: Path, log: Path | None = None):
        self.base = sitl.free_tcp_base()
        self.log = open(log or "/dev/null", "w")  # noqa: SIM115 (closed in close())
        args = [sitl.inav_bin(), f"--path={eeprom}", f"--tcpbaseport={self.base}"]
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT, cwd=eeprom.parent)
        end = time.monotonic() + 30
        while True:
            try:
                self.sock = socket.create_connection(("127.0.0.1", self.base), timeout=5)
                break
            except OSError:
                if time.monotonic() > end or self.proc.poll() is not None:
                    self.close()
                    raise RuntimeError("SITL's UART1 did not open") from None
                time.sleep(0.05)
        self.msp = MspLink(self.sock)

    def close(self) -> None:
        if getattr(self, "sock", None):
            self.sock.close()
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.log.close()
