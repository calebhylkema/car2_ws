"""
Guarded game-style keyboard teleop for the car2 Master Teensy.

Controls:
  1 / 2 / 3 / 4   select one of four configured throttle levels
  W               hold for forward
  S               hold for reverse
  A / Left        hold to steer left
  D / Right       hold to steer right
  C               center steering
  Z               record manually centered steering reference (E-stop only)
  F2              enable/disable manual Z reference (E-stop only)
  Space           hold full brake
  Q               toggle Neutral / active driving
  G               toggle Drive / Sport for forward motion
  E               toggle E-STOP
  Esc             safe stop and quit

The controller starts in E-STOP. Physically straighten the wheels and press Z
at every startup, then E to clear E-STOP and Q to enable driving.
All speed levels ramp from zero: 10 percentage points per second up
to 30%, then the configurable upper throttle rate (default 7.5 points/second).
Holding A/D builds steering over 1 second from center to full travel;
releasing it returns from full travel to center over 2 seconds.

Throttle is inhibited until telemetry confirms the requested gear. A direction
change commands Neutral and full brake, waits for confirmed Neutral, and
enforces a minimum brake interval. Release the old direction key, then hold
the desired direction after the vehicle is physically stopped. The held key
continues automatically after the interval; W and S together inhibit driving.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time

from pynput import keyboard

from teensy_serial import VehicleActuator


DEFAULT_SPEED_LEVELS = (0.15, 0.30, 0.55, 0.85)
DEFAULT_STEER_TRIM = 0.0
DEFAULT_STEER_STEP = 1.0 / 60.0
STEER_RETURN_RATE = 1.0 / 2.0
MAX_STEER_TRIM = 0.50
DEFAULT_HIGH_SPEED_RAMP_RATE = 0.075
DEFAULT_DIRECTION_CHANGE_BRAKE_S = 0.5
LOW_SPEED_RAMP_RATE = 0.10
LOW_SPEED_RAMP_THRESHOLD = 0.30
GEAR_CODES = {"N": 0, "D": 1, "S": 2, "R": 3}


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


def positive_float(value):
    try:
        number = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("value must be a decimal number") from exc
    if number <= 0.0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return number


class WasdTeleop:
    def __init__(self, actuator, rate_hz, throttle_scale, speed_levels, steer_step,
                 steer_trim, direction_change_brake_s, high_speed_ramp_rate,
                 manual_reference_enabled=True):
        self.act = actuator
        self.period = 1.0 / rate_hz
        self.throttle_scale = throttle_scale
        self.speed_levels = dict(zip(("1", "2", "3", "4"), speed_levels))
        self.steer_step = steer_step
        self.steer_trim = clamp(steer_trim, -MAX_STEER_TRIM, MAX_STEER_TRIM)
        self.direction_change_brake_s = max(0.5, direction_change_brake_s)
        self.high_speed_ramp_rate = high_speed_ramp_rate
        self.manual_reference_enabled = bool(manual_reference_enabled)
        self.startup_center_set = False

        self._lock = threading.Lock()
        self._pressed = set()
        self._running = True

        self.estop = True
        self.drive_enabled = False
        self.sport_enabled = False
        self.level = self.speed_levels["1"]
        self.steer = 0.0
        self.steer_last_at = None
        self.commanded_mode = "N"
        self.last_motion_gear = None
        self.guard_until = 0.0
        self.preselect_drive = False
        self.drive_state = "ESTOP"
        self.telem = {}
        self.telem_at = 0.0

        self.ramp_command = 0.0
        self.ramp_last_at = None
        self.ramp_gear = None
        self.ramp_active = False

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
                if self.estop and self._inhibit_reason():
                    self.drive_state = self._inhibit_reason()
                    return
                self.estop = not self.estop
                self.drive_enabled = False
                self.commanded_mode = "N"
                self.preselect_drive = False
                self.guard_until = 0.0
                self._reset_high_speed_ramp()
                if self.estop:
                    self.drive_state = "ESTOP"
                else:
                    self.drive_state = "NEUTRAL"
            elif key_id == "q":
                if self.estop:
                    self.drive_enabled = False
                    self.drive_state = "ESTOP"
                    return
                self.drive_enabled = not self.drive_enabled
                if self.drive_enabled:
                    self.preselect_drive = True
                    self.drive_state = f"SELECTING {self._forward_mode_name()}"
                else:
                    self.commanded_mode = "N"
                    self.preselect_drive = False
                    self._reset_high_speed_ramp()
                    self.drive_state = "NEUTRAL"
            elif key_id == "g":
                self.sport_enabled = not self.sport_enabled
                self._reset_high_speed_ramp()
                if self.drive_enabled and self.commanded_mode in ("D", "S"):
                    self.commanded_mode = self._forward_mode()
                    self.preselect_drive = False
                    self.drive_state = f"SELECTING {self._forward_mode_name()}"
            elif key_id == "c":
                self.steer = 0.0
            elif key_id == keyboard.Key.f2 and self.estop:
                self.manual_reference_enabled = not self.manual_reference_enabled
            elif key_id == "z" and self.estop and self.manual_reference_enabled:
                self.act.reference_steering()
                self.steer = 0.0
                self.startup_center_set = True
            elif key_id == keyboard.Key.esc:
                self._running = False
                return False

    def on_release(self, key):
        with self._lock:
            self._pressed.discard(self._key_id(key))

    def _actual_gear(self):
        if self.telem.get("tr") != 1:
            return None
        try:
            return int(self.telem["tm"])
        except (KeyError, TypeError, ValueError):
            return None

    def _inhibit_reason(self):
        if time.monotonic() - self.telem_at > 0.5:
            return "NO-MASTER"
        if not self.startup_center_set or self.telem.get("sh") != 1:
            return "NEED-REF (Z)" if self.manual_reference_enabled else "Z-OFF (F2)"
        if int(self.telem.get("sf", 0)) & 0x10:
            return "STEER-FAULT"
        return ""

    def _forward_mode(self):
        return "S" if self.sport_enabled else "D"

    def _forward_mode_name(self):
        return "SPORT" if self.sport_enabled else "DRIVE"

    @staticmethod
    def _motion_direction(mode):
        if mode in ("D", "S"):
            return "F"
        if mode == "R":
            return "R"
        return "N"

    @staticmethod
    def _gear_from_code(code):
        for gear, gear_code in GEAR_CODES.items():
            if code == gear_code:
                return gear
        return None

    def _reset_high_speed_ramp(self):
        self.ramp_command = 0.0
        self.ramp_last_at = None
        self.ramp_gear = None
        self.ramp_active = False

    def _apply_high_speed_ramp(self, now, gear, target):
        if self.ramp_last_at is None or self.ramp_gear != gear:
            self.ramp_command = 0.0
            self.ramp_last_at = now
            self.ramp_gear = gear
        else:
            elapsed = max(0.0, now - self.ramp_last_at)
            self.ramp_last_at = now
            if target <= self.ramp_command:
                self.ramp_command = target
            else:
                # Spend elapsed time below 30% first, then use the existing
                # upper ramp rate for any time remaining in this update.
                low_target = min(target, LOW_SPEED_RAMP_THRESHOLD)
                if self.ramp_command < low_target:
                    increase = min(low_target - self.ramp_command,
                                   LOW_SPEED_RAMP_RATE * elapsed)
                    self.ramp_command += increase
                    elapsed = max(0.0, elapsed - increase / LOW_SPEED_RAMP_RATE)
                self.ramp_command = min(
                    target,
                    self.ramp_command + self.high_speed_ramp_rate * elapsed,
                )

        self.ramp_active = self.ramp_command < target
        return self.ramp_command

    def _start_direction_guard(self, now):
        self._reset_high_speed_ramp()
        self.commanded_mode = "N"
        self.preselect_drive = False
        self.guard_until = now + self.direction_change_brake_s
        self.drive_state = "DIRECTION GUARD"

    def _compute_drive(self, now, forward, reverse, brake):
        actual_gear = self._actual_gear()

        if self.estop:
            self._reset_high_speed_ramp()
            self.drive_state = "ESTOP"
            return 0.0, "N", 1.0

        if not self.drive_enabled:
            self._reset_high_speed_ramp()
            self.commanded_mode = "N"
            self.drive_state = "NEUTRAL"
            return 0.0, "N", brake

        if self.guard_until:
            guard_complete = (
                now >= self.guard_until
                and not (forward and reverse)
                and actual_gear == GEAR_CODES["N"]
            )
            if guard_complete:
                self.guard_until = 0.0
                self.last_motion_gear = None
                self.drive_state = "READY"
            else:
                self.drive_state = "DIRECTION GUARD"
            self._reset_high_speed_ramp()
            return 0.0, "N", 1.0

        if self.preselect_drive and not forward and not reverse:
            target = self._forward_mode()
            actual_mode = self._gear_from_code(actual_gear)
            if (self._motion_direction(self.last_motion_gear) == "R" or
                    self._motion_direction(actual_mode) == "R"):
                self._start_direction_guard(now)
                return 0.0, "N", 1.0
            if actual_gear in (GEAR_CODES["N"], GEAR_CODES["D"], GEAR_CODES["S"]):
                self.commanded_mode = target
                self.preselect_drive = False
            else:
                self._reset_high_speed_ramp()
                self.drive_state = "WAIT-TELEMETRY"
                return 0.0, "N", brake

        if forward and reverse:
            self._start_direction_guard(now)
            return 0.0, "N", 1.0

        if forward or reverse:
            requested = self._forward_mode() if forward else "R"
            self.preselect_drive = False
            requested_direction = self._motion_direction(requested)
            actual_mode = self._gear_from_code(actual_gear)
            last_direction = self._motion_direction(self.last_motion_gear)
            actual_direction = self._motion_direction(actual_mode)
            if ((last_direction != "N" and last_direction != requested_direction) or
                    (actual_direction != "N" and actual_direction != requested_direction)):
                self._start_direction_guard(now)
                return 0.0, "N", 1.0

            self.commanded_mode = requested
            if actual_gear == GEAR_CODES[requested]:
                self.last_motion_gear = requested
                target = self.level * self.throttle_scale
                throttle = self._apply_high_speed_ramp(now, requested, target)
                if self.ramp_active:
                    self.drive_state = f"RAMP-{requested}"
                else:
                    if requested == "S":
                        self.drive_state = "SPORT"
                    else:
                        self.drive_state = "FORWARD" if requested == "D" else "REVERSE"
            else:
                self._reset_high_speed_ramp()
                self.drive_state = f"WAIT-{requested}"
                throttle = 0.0
        else:
            self._reset_high_speed_ramp()
            throttle = 0.0
            if self.commanded_mode in GEAR_CODES:
                if actual_gear == GEAR_CODES[self.commanded_mode]:
                    self.drive_state = f"READY-{self.commanded_mode}"
                else:
                    self.drive_state = f"WAIT-{self.commanded_mode}"
            else:
                self.drive_state = "READY"

        if brake > 0.0:
            self._reset_high_speed_ramp()
            throttle = 0.0
            self.drive_state = "BRAKE"

        return throttle, self.commanded_mode, brake

    def _apply_steering_input(self, now, direction):
        elapsed = 0.0 if self.steer_last_at is None else max(0.0, now - self.steer_last_at)
        self.steer_last_at = now
        # Preserve --steer-step as a 60 Hz equivalent, using actual elapsed
        # time so keyboard steering does not depend on the update frequency.
        change = self.steer_step * 60.0 * elapsed
        if direction:
            self.steer = clamp(self.steer + direction * change, -1.0, 1.0)
        elif self.steer > 0.0:
            self.steer = max(0.0, self.steer - STEER_RETURN_RATE * elapsed)
        elif self.steer < 0.0:
            self.steer = min(0.0, self.steer + STEER_RETURN_RATE * elapsed)
        return self.steer

    def _compute(self):
        if not self.estop and self._inhibit_reason():
            self.estop = True
            self.drive_enabled = False
            self.commanded_mode = "N"
            self.preselect_drive = False
            self.guard_until = 0.0
        pressed = self._pressed
        forward = "w" in pressed
        reverse = "s" in pressed
        brake = 1.0 if keyboard.Key.space in pressed else 0.0

        now = time.monotonic()
        throttle, mode, brake = self._compute_drive(now, forward, reverse, brake)

        if self.estop:
            self.steer = 0.0
            self.steer_last_at = now
            self.drive_state = self._inhibit_reason() or "ESTOP"
            return True, 0.0, "N", 1.0, 0.0

        left = ("a" in pressed) or (keyboard.Key.left in pressed)
        right = ("d" in pressed) or (keyboard.Key.right in pressed)
        direction = (1 if right else 0) - (1 if left else 0)
        self._apply_steering_input(now, direction)

        steer_cmd = clamp(self.steer + self.steer_trim, -1.0, 1.0)
        return self.estop, throttle, mode, brake, steer_cmd

    def _status(self, estop, throttle, mode, brake, steer):
        tag = "ESTOP" if estop else ("ACTIVE" if self.drive_enabled else " NEUT ")
        telemetry = self.telem
        alignment = ""
        if not self.startup_center_set or telemetry.get("sh") != 1:
            action = "press Z" if self.manual_reference_enabled else "press F2, then Z"
            alignment = f"ALIGN: straighten wheels; {action} | "
        car = ""
        if telemetry:
            car = (
                f" | car w={telemetry.get('w', '?')} t={telemetry.get('t', '?')} "
                f"tv={telemetry.get('tv', '?')} tm={telemetry.get('tm', '?')} "
                f"sp={telemetry.get('sp', '?')} sf={telemetry.get('sf', '?')} "
                f"ref={telemetry.get('sh', '?')} "
                f"CAN(T/S/B)={telemetry.get('tr', '?')}/{telemetry.get('sr', '?')}/{telemetry.get('br', '?')} "
                f"bl={telemetry.get('bl', '?')} bf={telemetry.get('bf', '?')}"
            )
        sys.stdout.write(
            f"\r{alignment}[{tag}] state={self.drive_state:<15} mode={mode} lvl={self.level:.2f} "
            f"sport={'ON' if self.sport_enabled else 'OFF'} "
            f"manualZ={'ON' if self.manual_reference_enabled else 'OFF'} "
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
                            self.telem_at = time.monotonic()
                        except Exception:
                            pass

                self._status(estop, throttle, mode, brake, steer)
                time.sleep(self.period)
        finally:
            listener.stop()

    def shutdown(self):
        self._running = False
        # Repeat the final safe state so a single USB write cannot strand the
        # car active. The master latches E-stop, and its watchdog independently
        # enters E-stop if the process or PC disappears before cleanup runs.
        for _ in range(5):
            try:
                self.act.send_all(
                    estop=True,
                    throttle=0.0,
                    mode="N",
                    brake=1.0,
                    steer=0.0,
                )
            except Exception:
                break
            time.sleep(0.05)


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
    parser.add_argument("--steer-step", type=positive_float, default=DEFAULT_STEER_STEP,
                        help="60 Hz equivalent steering increment; default 1s center-to-full, 2s return")
    parser.add_argument("--disable-manual-reference", action="store_true",
                        help="start with Z disabled; F2 can enable it while in E-stop")
    parser.add_argument("--steer-trim", type=float, default=DEFAULT_STEER_TRIM,
                        help="temporary override for the installed steering-center correction")
    parser.add_argument("--direction-change-brake", type=float,
                        default=DEFAULT_DIRECTION_CHANGE_BRAKE_S,
                        help="minimum full-brake Neutral guard before changing direction")
    parser.add_argument("--high-speed-ramp-rate", type=positive_float,
                        default=DEFAULT_HIGH_SPEED_RAMP_RATE,
                        help="normalized throttle increase per second above 30%")
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
        high_speed_ramp_rate=args.high_speed_ramp_rate,
        manual_reference_enabled=not args.disable_manual_reference,
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
