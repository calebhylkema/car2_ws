"""
Keyboard teleop for the car2 Master Teensy (pre-ROS2).

Throwaway PC-side tool. Talks to the master over USB via VehicleActuator; the
master relays to the throttle/steer/brake nodes over CAN. A background thread
re-sends state at a fixed rate to feed the master's watchdog. Shows the master's
live telemetry so we can see what each node is actually doing.

Controls:
  1 / 2 / 3 / 4   SELECT throttle level (25 / 50 / 75 / 100 %) -- latched
  w               HOLD to apply the selected throttle; release -> coast (0)
  a / Left        HOLD to steer left   (release -> returns to center)
  d / Right       HOLD to steer right  (release -> returns to center)
  c               snap steering to center
  space           HOLD to brake        (release -> 0)
  q               cycle mode  Neutral -> Drive -> Reverse -> Neutral
  e               toggle E-STOP
  Esc             quit (safe stop)

Telemetry shown (from the master): w=watchdog, t=cmd throttle, tv=throttle node
volts, tm=gear, sp=steer pos%, sf=steer flags, bl=brake length in, bf=brake flags.

Usage:
  python keyboard_teleop.py --port COM4
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


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class KeyboardTeleop:
    def __init__(self, actuator, rate_hz, throttle_scale, steer_step):
        self.act = actuator
        self.period = 1.0 / rate_hz
        self.throttle_scale = throttle_scale
        self.steer_step = steer_step

        self._lock = threading.Lock()
        self._pressed = set()
        self._running = True

        self.estop = False
        self.mode_idx = 0
        self.level = 0.25        # selected throttle level (keys 1-4)
        self.steer = 0.0
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
            if k in SPEED_LEVELS:
                self.level = SPEED_LEVELS[k]      # select level (latched)
            elif k == "e":
                self.estop = not self.estop
            elif k == "q":
                self.mode_idx = (self.mode_idx + 1) % len(MODE_CYCLE)
            elif k == "c":
                self.steer = 0.0
            elif k == keyboard.Key.esc:
                self._running = False
                return False

    def on_release(self, key):
        with self._lock:
            self._pressed.discard(self._kid(key))

    def _compute(self):
        p = self._pressed
        # Throttle: only while 'w' held, at the selected level.
        throttle = (self.level * self.throttle_scale) if ("w" in p) else 0.0
        # Brake: full while space held.
        brake = 1.0 if keyboard.Key.space in p else 0.0
        # Steering: ramp while held, ease back to center on release.
        left = ("a" in p) or (keyboard.Key.left in p)
        right = ("d" in p) or (keyboard.Key.right in p)
        direction = (1 if right else 0) - (1 if left else 0)
        if direction != 0:
            self.steer = clamp(self.steer + direction * self.steer_step, -1.0, 1.0)
        elif self.steer > 0.0:
            self.steer = max(0.0, self.steer - self.steer_step)
        elif self.steer < 0.0:
            self.steer = min(0.0, self.steer + self.steer_step)
        return self.estop, throttle, MODE_CYCLE[self.mode_idx], brake, self.steer

    def _status(self, estop, throttle, mode, brake, steer):
        tag = "ESTOP" if estop else "  run"
        t = self.telem
        car = ""
        if t:
            car = (f" | car w={t.get('w','?')} t={t.get('t','?')} tv={t.get('tv','?')} "
                   f"tm={t.get('tm','?')} sp={t.get('sp','?')} sf={t.get('sf','?')} "
                   f"bl={t.get('bl','?')} bf={t.get('bf','?')}")
        sys.stdout.write(
            f"\r[{tag}] mode={mode} lvl={self.level:.2f} thr={throttle:4.2f} "
            f"str={steer:+4.2f} brk={brake:3.1f}{car}      "
        )
        sys.stdout.flush()

    def run(self):
        print(__doc__)
        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()
        try:
            while self._running:
                with self._lock:
                    estop, throttle, mode, brake, steer = self._compute()
                try:
                    self.act.send_all(estop=estop, throttle=throttle, mode=mode,
                                      brake=brake, steer=steer)
                except Exception as e:
                    sys.stderr.write(f"\n[send error] {e}\n")
                for _ in range(4):
                    line = self.act.read_telemetry_line()
                    if not line:
                        break
                    if line.startswith("{"):
                        try:
                            self.telem = json.loads(line)
                        except Exception:
                            pass
                self._status(estop, throttle, mode, brake, steer)
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
    ap = argparse.ArgumentParser(description="Keyboard teleop for car2")
    ap.add_argument("--port", default="COM4", help="master's serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--rate", type=float, default=30.0)
    ap.add_argument("--throttle-scale", type=float, default=1.0)
    ap.add_argument("--steer-step", type=float, default=0.20)
    args = ap.parse_args()

    print(f"Connecting to {args.port} @ {args.baud} ...")
    actuator = VehicleActuator(port=args.port, baud=args.baud)
    teleop = KeyboardTeleop(actuator, rate_hz=args.rate,
                            throttle_scale=args.throttle_scale, steer_step=args.steer_step)
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping (safe e-stop)...")
        teleop.shutdown()
        actuator.close()
        print("Done.")


if __name__ == "__main__":
    main()
