"""
Vehicle Actuator Interface via USB Serial.

Talks to the Master Teensy over USB CDC serial using the frozen line protocol
(see ../firmware/PROTOCOL.md). This is the shared Hardware Abstraction Layer:
- today `keyboard_teleop.py` uses it,
- later a ROS2 node wraps this SAME class.

Because the protocol never changes, moving to ROS2 does not touch the firmware
or this driver.

Protocol (downlink):
    E 1|0      -> estop on/off
    T 0..1     -> throttle
    M N|D|S|R  -> mode (Neutral/Drive/Sport/Reverse)
    B 0..1     -> brake
    S -1..1    -> steer (left -, right +)
    C          -> center steering
"""

from __future__ import annotations
import logging
import threading
import time
from typing import Optional

import serial  # pip install pyserial

logger = logging.getLogger(__name__)

VALID_MODES = ("N", "D", "S", "R")


class VehicleActuator:
    """Vehicle actuator interface via USB Serial to the Master Teensy."""

    def __init__(self, port: str = "COM3", baud: int = 115200, timeout: float = 0.1,
                 write_timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.write_timeout = write_timeout
        self._ser: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self.open()

    def open(self):
        """Open serial connection."""
        if self._ser and self._ser.is_open:
            return
        # write_timeout guards against a silent/hung master: if the Teensy stops
        # draining USB input, write() raises SerialTimeoutException instead of
        # blocking forever (which used to freeze the tool and strand the COM port).
        self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout,
                                  write_timeout=self.write_timeout)
        time.sleep(1.5)  # Teensy USB CDC setup time
        self._ser.reset_input_buffer()

    def close(self):
        """Close serial connection."""
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def __enter__(self) -> "VehicleActuator":
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def _send(self, cmd: str):
        """Send one command line (thread-safe)."""
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial port not open")
        with self._lock:
            self._ser.write((cmd + "\n").encode("ascii"))

    def set_throttle(self, value: float):
        """Set throttle 0..1."""
        self._send(f"T {max(0.0, min(1.0, float(value))):.3f}")

    def set_brake(self, value: float):
        """Set brake 0..1."""
        self._send(f"B {max(0.0, min(1.0, float(value))):.3f}")

    def set_mode(self, mode: str):
        """Set drive mode: N, D, S, or R."""
        m = mode.upper()
        if m not in VALID_MODES:
            raise ValueError(f"Invalid mode '{mode}', use: {VALID_MODES}")
        self._send(f"M {m}")

    def set_steer_norm(self, value: float):
        """Set steering -1..1 (left=-1, right=+1)."""
        self._send(f"S {max(-1.0, min(1.0, float(value))):.3f}")

    def set_steer_deg(self, degrees: float, mech_limit_deg: float = 28.0):
        """Set steering in degrees. Converts to normalized using mechanical limit."""
        d = max(-mech_limit_deg, min(mech_limit_deg, float(degrees)))
        self.set_steer_norm(d / mech_limit_deg)

    def center(self):
        """One-shot center steering."""
        self._send("C")

    def estop(self, on: bool = True):
        """Emergency stop."""
        self._send(f"E {1 if on else 0}")

    def send_all(self, estop: bool, throttle: float, mode: str,
                 brake: float, steer: float):
        """Send the whole state in one atomic 'A' line."""
        m = mode.upper() if mode.upper() in VALID_MODES else "N"
        self._send(
            f"A E={1 if estop else 0} "
            f"T={max(0.0, min(1.0, float(throttle))):.3f} "
            f"M={m} "
            f"B={max(0.0, min(1.0, float(brake))):.3f} "
            f"S={max(-1.0, min(1.0, float(steer))):.3f}"
        )

    def read_telemetry_line(self) -> Optional[str]:
        """Read one pending telemetry line if available (non-blocking-ish)."""
        if not self._ser or not self._ser.is_open:
            return None
        with self._lock:
            if self._ser.in_waiting:
                raw = self._ser.readline()
                try:
                    return raw.decode("ascii", errors="ignore").strip()
                except Exception:
                    return None
        return None
