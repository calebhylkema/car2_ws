"""
Game-style keyboard teleop for the car2 Master Teensy.

Controls:
  1 / 2 / 3 / 4   select one of four configured throttle levels
  W               hold for forward
  S               hold for reverse
  A / Left        hold to steer left
  D / Right       hold to steer right
  C               center steering
  Space           hold full brake
  Q               toggle Neutral / active driving
  E               toggle E-STOP
  Esc             safe stop and quit

Throttle is inhibited until telemetry confirms the requested gear. A direction
change commands Neutral and full brake, requires W/S to be released, waits for
confirmed Neutral, and enforces a minimum brake interval. Press the desired
direction again only after the vehicle is physically stopped.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time

from pynput import keyboard

from teensy_serial import VehicleActuator


DEFAULT_SPEED_LEVELS = (0.32, 0.40, 0.55, 0.75)
GEAR_CODES = {"N": 0, "D": 1, "R": 3}


def clamp(value, low, high):
    return max(low, min(high, value))


def parse_speed_levels(value):
    try:
        levels = tuple(float(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("speed levels must be four decimal values") from exc
    if len(levels) != 4:
        raise argparse.ArgumentTypeError("speed levels must contain exactly four values")
    if any(level <= 0.0 or level > 1.0 for level in levels):
        raise argparse.ArgumentTypeError("each speed level must be greater than 0 and at most 1")
    if any(left >= right for left, right in zip(levels, levels[1:])):
        raise argparse.ArgumentTypeError("speed levels must be strictly increasing")
    return levels


class WasdTeleop:
    def __init__(self, actuator, rate_hz, throttle_scale, speed_levels, steer_step,
                 steer_trim, direction_change_brake_s):
        self.act = actuator
        self.period = 1.0 / rate_hz
        self.throttle_scale = throttle_scale
        self.speed_levels = dict(zip(("1", "2", "3", "4"), speed_levels))
        self.steer_step = steer_step
        self.steer_trim = clamp(steer_trim, -0.25, 0.25)
        self.direction_change_brake_s = max(0.5, direction_change_brake_s)

        self._lock = threading.Lock()
        self._pressed = set()
        self._running = True

        self.estop = False
        self.drive_enabled = False
        self.level = self.speed_levels["1"]
        self.steer = 0.0
        self.commanded_mode = "N"
        self.last_motion_gear = None
        self.guard_until = 0.0
        self.guard_release_seen = False
        self.preselect_drive = False
        self.drive_state = "NEUTRAL"
        self.telem = {}

    @staticmethod
    def _key_id(key):
        if isinstance(key, keyboard.KeyCode) and key.char is not None:
            return key.char.lower()
        return key

    def on_press(self, key):
        key_id = self._key_id(key)
        with self._lock:
            edge = key_id not in self._pressed
            self._pressed.add(key_id)
            if not edge:
                return

            if key_id in self.speed_levels:
                self.level = self.speed_levels[key_id]
            elif key_id == "e":
                self.estop = not self.estop
                if self.estop:
                    self.drive_enabled = False
                    self.commanded_mode = "N"
                    self.guard_until = 0.0
                    self.drive_state = "ESTOP"
            elif key_id == "q":
                self.drive_enabled = not self.drive_enabled
                if self.drive_enabled:
                    self.preselect_drive = True
                    self.drive_state = "SELECTING DRIVE"
                else:
                    self.commanded_mode = "N"
                    self.preselect_drive = False
                    self.drive_state = "NEUTRAL"
            elif key_id == "c":
                self.steer = 0.0
            elif key_id == keyboard.Key.esc:
                self._running = False
                return False

    def on_release(self, key):
        with self._lock:
            self._pressed.discard(self._key_id(key))

    def _actual_gear(self):
        try:
            return int(self.telem["tm"])
        except (KeyError, TypeError, ValueError):
            return None

    def _start_direction_guard(self, now):
        self.commanded_mode = "N"
        self.preselect_drive = False
        self.guard_until = now + self.direction_change_brake_s
        self.guard_release_seen = False
        self.drive_state = "DIRECTION GUARD"

    def _compute_drive(self, now, forward, reverse, brake):
        actual_gear = self._actual_gear()

        if self.estop:
            self.drive_state = "ESTOP"
            return 0.0, "N", 1.0

        if not self.drive_enabled:
            self.commanded_mode = "N"
            self.drive_state = "NEUTRAL"
            return 0.0, "N", brake

        if self.guard_until:
            direction_keys_released = not forward and not reverse
            if direction_keys_released:
                self.guard_release_seen = True

            guard_complete = (
                now >= self.guard_until
                and self.guard_release_seen
                and direction_keys_released
                and actual_gear == GEAR_CODES["N"]
            )
            if guard_complete:
                self.guard_until = 0.0
                self.guard_release_seen = False
                self.last_motion_gear = None
                self.drive_state = "READY"
            else:
                self.drive_state = "DIRECTION GUARD"
            return 0.0, "N", 1.0

        if self.preselect_drive and not forward and not reverse:
            if self.last_motion_gear == "R" or actual_gear == GEAR_CODES["R"]:
                self._start_direction_guard(now)
                return 0.0, "N", 1.0
            if actual_gear in (GEAR_CODES["N"], GEAR_CODES["D"]):
                self.commanded_mode = "D"
                self.preselect_drive = False
            else:
                self.drive_state = "WAIT-TELEMETRY"
                return 0.0, "N", brake

        if forward and reverse:
            self._start_direction_guard(now)
            return 0.0, "N", 1.0

        if forward or reverse:
            requested = "D" if forward else "R"
            self.preselect_drive = False
            opposite = "R" if requested == "D" else "D"
            if self.last_motion_gear == opposite or actual_gear == GEAR_CODES[opposite]:
                self._start_direction_guard(now)
                return 0.0, "N", 1.0

            self.commanded_mode = requested
            if actual_gear == GEAR_CODES[requested]:
                self.last_motion_gear = requested
                self.drive_state = "FORWARD" if requested == "D" else "REVERSE"
                throttle = self.level * self.throttle_scale
            else:
                self.drive_state = f"WAIT-{requested}"
                throttle = 0.0
        else:
            throttle = 0.0
            if self.commanded_mode in GEAR_CODES:
                if actual_gear == GEAR_CODES[self.commanded_mode]:
                    self.drive_state = f"READY-{self.commanded_mode}"
                else:
                    self.drive_state = f"WAIT-{self.commanded_mode}"
            else:
                self.drive_state = "READY"

        if brake > 0.0:
            throttle = 0.0
            self.drive_state = "BRAKE"

        return throttle, self.commanded_mode, brake

    def _compute(self):
        pressed = self._pressed
        forward = "w" in pressed
        reverse = "s" in pressed
        brake = 1.0 if keyboard.Key.space in pressed else 0.0

        throttle, mode, brake = self._compute_drive(
            time.monotonic(), forward, reverse, brake
        )

        left = ("a" in pressed) or (keyboard.Key.left in pressed)
        right = ("d" in pressed) or (keyboard.Key.right in pressed)
        direction = (1 if right else 0) - (1 if left else 0)
        if direction:
            self.steer = clamp(self.steer + direction * self.steer_step, -1.0, 1.0)
        elif self.steer > 0.0:
            self.steer = max(0.0, self.steer - self.steer_step)
        elif self.steer < 0.0:
            self.steer = min(0.0, self.steer + self.steer_step)

        steer_cmd = clamp(self.steer + self.steer_trim, -1.0, 1.0)
        return self.estop, throttle, mode, brake, steer_cmd

    def _status(self, estop, throttle, mode, brake, steer):
        tag = "ESTOP" if estop else ("ACTIVE" if self.drive_enabled else " NEUT ")
        telemetry = self.telem
        car = ""
        if telemetry:
            car = (
                f" | car w={telemetry.get('w', '?')} t={telemetry.get('t', '?')} "
                f"tv={telemetry.get('tv', '?')} tm={telemetry.get('tm', '?')} "
                f"sp={telemetry.get('sp', '?')} sf={telemetry.get('sf', '?')} "
                f"bl={telemetry.get('bl', '?')} bf={telemetry.get('bf', '?')}"
            )
        sys.stdout.write(
            f"\r[{tag}] state={self.drive_state:<15} mode={mode} lvl={self.level:.2f} "
            f"thr={throttle:4.2f} str={steer:+4.2f} brk={brake:3.1f}{car}      "
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
                    self.act.send_all(
                        estop=estop,
                        throttle=throttle,
                        mode=mode,
                        brake=brake,
                        steer=steer,
                    )
                except Exception as exc:
                    sys.stderr.write(f"\n[send error] {exc}\n")

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
    parser = argparse.ArgumentParser(description="Game-style keyboard teleop for car2")
    parser.add_argument("--port", default="COM4", help="master Teensy serial port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=60.0)
    parser.add_argument("--throttle-scale", type=float, default=1.0)
    parser.add_argument("--speed-levels", type=parse_speed_levels,
                        default=DEFAULT_SPEED_LEVELS,
                        metavar="L1,L2,L3,L4",
                        help="four increasing normalized throttle levels")
    parser.add_argument("--steer-step", type=float, default=0.25)
    parser.add_argument("--steer-trim", type=float, default=0.0,
                        help="normalized steering center trim; positive moves center right")
    parser.add_argument("--direction-change-brake", type=float, default=1.0,
                        help="minimum full-brake Neutral guard before changing direction")
    args = parser.parse_args()

    print(f"Connecting to {args.port} @ {args.baud} ...")
    actuator = VehicleActuator(port=args.port, baud=args.baud)
    teleop = WasdTeleop(
        actuator,
        rate_hz=args.rate,
        throttle_scale=args.throttle_scale,
        speed_levels=args.speed_levels,
        steer_step=args.steer_step,
        steer_trim=args.steer_trim,
        direction_change_brake_s=args.direction_change_brake,
    )
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
