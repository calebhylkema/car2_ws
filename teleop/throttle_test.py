"""
Throttle-only test for the car2 Master Teensy.

Same key mappings as keyboard_teleop.py, but it ONLY commands throttle + gear +
e-stop -- steer and brake are held at 0, so it can't move the (still-being-fixed)
steering. Talks to the MASTER over USB; the master relays to the throttle node
over CAN. A background thread re-sends at a fixed rate to feed the master's
watchdog.

Controls (same as the main teleop):
  1 / 2 / 3 / 4   HOLD for 25 / 50 / 75 / 100 % throttle; release -> 0 (idle)
  q               cycle drive mode  Neutral -> Drive -> Reverse -> Neutral
  e               toggle E-STOP
  Esc             quit (sends a safe stop first)

Note: throttle only produces output in Drive/Reverse -- press q to select a gear,
then hold 1-4. In Neutral the throttle node stays at idle.

Usage:
  python throttle_test.py --port COM4                  # master's COM port
  python throttle_test.py --port COM4 --throttle-scale 0.3   # gentler for first test
"""

from __future__ import annotations
import argparse
import json
import sys
import threading
import time

from pynput import keyboard

from teensy_serial import VehicleActuator

MODE_CYCLE = ("N", "D", "R")
SPEED_LEVELS = {"1": 0.25, "2": 0.50, "3": 0.75, "4": 1.00}


class ThrottleTest:
    def __init__(self, actuator, rate_hz, throttle_scale):
        self.act = actuator
        self.period = 1.0 / rate_hz
        self.scale = throttle_scale
        self._lock = threading.Lock()
        self._pressed = set()
        self._running = True
        self.estop = False
        self.mode_idx = 0
        self.telem = {}

    @staticmethod
    def _kid(key):
        if isinstance(key, keyboard.KeyCode) and key.char is not None:
            return key.char.lower()
        return key

    def on_press(self, key):
        k = self._kid(key)
        with self._lock:
            edge = k not in self._pressed
            self._pressed.add(k)
            if not edge:
                return
            if k == "e":
                self.estop = not self.estop
            elif k == "q":
                self.mode_idx = (self.mode_idx + 1) % len(MODE_CYCLE)
            elif k == keyboard.Key.esc:
                self._running = False
                return False

    def on_release(self, key):
        with self._lock:
            self._pressed.discard(self._kid(key))

    def _throttle(self):
        for d in ("4", "3", "2", "1"):     # highest held digit wins
            if d in self._pressed:
                return SPEED_LEVELS[d] * self.scale
        return 0.0

    def _status(self, estop, throttle, mode):
        tag = "ESTOP" if estop else "  run"
        t = self.telem
        car = ""
        if t:
            # w=master watchdog, t=cmd throttle at master, tv=node volts, tm=node gear
            car = (f" | car: w={t.get('w','?')} t={t.get('t','?')} "
                   f"tv={t.get('tv','?')} tm={t.get('tm','?')}")
        sys.stdout.write(f"\r[{tag}] cmd mode={mode} thr={throttle:4.2f}{car}      ")
        sys.stdout.flush()

    def run(self):
        print(__doc__)
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        try:
            while self._running:
                with self._lock:
                    estop = self.estop
                    throttle = self._throttle()
                    mode = MODE_CYCLE[self.mode_idx]
                try:
                    self.act.send_all(estop=estop, throttle=throttle, mode=mode,
                                      brake=0.0, steer=0.0)
                except Exception as e:
                    sys.stderr.write(f"\n[send error] {e}\n")
                # Drain any telemetry lines; keep the latest parsed one.
                for _ in range(4):
                    line = self.act.read_telemetry_line()
                    if not line:
                        break
                    if line.startswith("{"):
                        try:
                            self.telem = json.loads(line)
                        except Exception:
                            pass
                self._status(estop, throttle, mode)
                time.sleep(self.period)
        finally:
            listener.stop()

    def shutdown(self):
        self._running = False
        time.sleep(self.period * 2)
        try:
            self.act.send_all(estop=True, throttle=0.0, mode="N", brake=1.0, steer=0.0)
            self.act.estop(True)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="Throttle-only keyboard test for car2")
    ap.add_argument("--port", default="COM4", help="MASTER's serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=float, default=30.0)
    ap.add_argument("--throttle-scale", type=float, default=1.0,
                    help="multiplier on the 1-4 levels (use <1 for a gentle first test)")
    args = ap.parse_args()

    print(f"Connecting to {args.port} @ {args.baud} ...")
    actuator = VehicleActuator(port=args.port, baud=args.baud)
    test = ThrottleTest(actuator, rate_hz=args.rate, throttle_scale=args.throttle_scale)
    try:
        test.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping (safe e-stop)...")
        test.shutdown()
        actuator.close()
        print("Done.")


if __name__ == "__main__":
    main()
