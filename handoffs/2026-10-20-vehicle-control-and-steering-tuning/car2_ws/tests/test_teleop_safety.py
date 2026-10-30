"""Regression tests for actual teleop logic; no USB or keyboard hooks opened."""
import importlib.util
import io
import pathlib
import sys
import time
import types
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "teleop"))
keys = types.SimpleNamespace(space="SPACE", left="LEFT", right="RIGHT", esc="ESC", f2="F2")
keyboard = types.SimpleNamespace(Key=keys, KeyCode=type("KeyCode", (), {}))
spec = importlib.util.spec_from_file_location("tested_teleop", ROOT / "teleop/keyboard_teleop_wasd_guarded.py")
teleop = importlib.util.module_from_spec(spec)
with mock.patch.dict(sys.modules, {"pynput": types.SimpleNamespace(keyboard=keyboard)}):
    spec.loader.exec_module(teleop)


class TeleopSafety(unittest.TestCase):
    def setUp(self):
        self.act = mock.Mock()
        self.app = teleop.WasdTeleop(self.act, 50, 1, teleop.DEFAULT_SPEED_LEVELS,
                                     teleop.DEFAULT_STEER_STEP, teleop.DEFAULT_STEER_TRIM,
                                     teleop.DEFAULT_DIRECTION_CHANGE_BRAKE_S,
                                     teleop.DEFAULT_HIGH_SPEED_RAMP_RATE)

    def ready(self):
        # Existing active-control cases start after the operator selected Z.
        self.app.startup_center_set = True
        self.app.telem = dict(tr=1, sr=1, br=1, sh=1, sf=0, tm=0)
        self.app.telem_at = time.monotonic()

    def test_startup_has_no_left_trim(self):
        self.assertEqual(self.app._compute(), (True, 0, "N", 1, 0))
        self.assertEqual(teleop.DEFAULT_STEER_TRIM, 0)

    def test_startup_requires_z_even_when_teensy_retains_old_reference(self):
        self.ready()
        self.app.startup_center_set = False
        self.app.on_press("e")
        self.app.on_release("e")
        self.assertTrue(self.app.estop)
        self.assertEqual(self.app._inhibit_reason(), "NEED-REF (Z)")
        self.app.on_press("z")
        self.act.reference_steering.assert_called_once_with()
        self.app.on_release("z")
        self.app.on_press("e")
        self.assertFalse(self.app.estop)
        self.assertFalse(self.app.drive_enabled)

    def test_startup_alignment_prompt_is_visible_until_z_is_confirmed(self):
        self.ready()
        self.app.startup_center_set = False
        for _ in range(2):
            with mock.patch.object(teleop.sys, "stdout", new_callable=io.StringIO) as output:
                self.app._status(True, 0, "N", 1, 0)
                self.assertTrue(output.getvalue().startswith("\rALIGN: straighten wheels; press Z"))
        self.app.on_press("z")
        with mock.patch.object(teleop.sys, "stdout", new_callable=io.StringIO) as output:
            self.app._status(True, 0, "N", 1, 0)
            self.assertNotIn("ALIGN:", output.getvalue())
        self.app.telem["sh"] = 0
        with mock.patch.object(teleop.sys, "stdout", new_callable=io.StringIO) as output:
            self.app._status(True, 0, "N", 1, 0)
            self.assertIn("ALIGN:", output.getvalue())

    def test_startup_alignment_prompt_explains_disabled_manual_z(self):
        self.app.manual_reference_enabled = False
        with mock.patch.object(teleop.sys, "stdout", new_callable=io.StringIO) as output:
            self.app._status(True, 0, "N", 1, 0)
            self.assertIn("press F2, then Z", output.getvalue())

    def test_estop_suppresses_held_steering_and_explicit_trim(self):
        self.app.steer_trim = -0.4
        self.app._pressed.add("a")
        for _ in range(20):
            self.assertEqual(self.app._compute(), (True, 0, "N", 1, 0))

    def test_estop_can_clear_with_missing_node_feedback(self):
        for missing in ("tr", "sr", "br"):
            self.ready()
            self.app.telem[missing] = 0
            self.app.estop = True
            self.app.on_press("e")
            self.app.on_release("e")
            self.assertFalse(self.app.estop)

    def test_estop_cannot_clear_without_steering_reference(self):
        self.ready()
        self.app.telem["sh"] = 0
        self.app.on_press("e")
        self.assertTrue(self.app.estop)

    def test_clear_estop_does_not_enable_drive(self):
        self.ready()
        self.app.on_press("e")
        self.assertFalse(self.app.estop)
        self.assertFalse(self.app.drive_enabled)
        self.assertEqual(self.app._compute()[1:3], (0, "N"))

    def test_z_reference_is_only_sent_in_estop(self):
        self.app.on_press("z")
        self.act.reference_steering.assert_called_once_with()
        self.app.on_release("z")
        self.ready()
        self.app.on_press("e")
        self.app.on_press("z")
        self.act.reference_steering.assert_called_once_with()

    def test_manual_reference_toggle_blocks_z_and_ignores_key_repeat(self):
        self.app.on_press(keys.f2)
        self.app.on_press(keys.f2)  # OS key repeat must not re-enable it.
        self.assertFalse(self.app.manual_reference_enabled)
        self.app.on_press("z")
        self.app.on_release("z")
        self.act.reference_steering.assert_not_called()
        self.app.on_release(keys.f2)
        self.app.on_press(keys.f2)
        self.app.on_press("z")
        self.act.reference_steering.assert_called_once_with()

    def test_manual_reference_toggle_cannot_change_while_active(self):
        self.ready()
        self.app.estop = False
        self.app.on_press(keys.f2)
        self.assertTrue(self.app.manual_reference_enabled)
        self.app.on_press("z")
        self.act.reference_steering.assert_not_called()

    def test_disabled_manual_reference_explains_reference_inhibit(self):
        self.ready()
        self.app.telem["sh"] = 0
        self.app.manual_reference_enabled = False
        self.assertEqual(self.app._inhibit_reason(), "Z-OFF (F2)")
        self.app.on_press("e")
        self.assertTrue(self.app.estop)
        self.assertEqual(self.app._compute(), (True, 0, "N", 1, 0))

    def test_missing_brake_feedback_does_not_stop_driving(self):
        self.ready()
        self.app.estop = False
        self.app.drive_enabled = True
        self.app._pressed.update(("w", "d"))
        self.app.telem["br"] = 0
        self.app.telem["tm"] = 1
        with mock.patch.object(teleop.time, "monotonic", return_value=self.app.telem_at):
            self.app._compute()
        with mock.patch.object(teleop.time, "monotonic", return_value=self.app.telem_at + 0.1):
            estop, throttle, mode, brake, steer = self.app._compute()
        self.assertFalse(estop)
        self.assertGreater(throttle, 0)
        self.assertEqual((mode, brake), ("D", 0))
        self.assertGreater(steer, 0)
        self.assertTrue(self.app.drive_enabled)

    def test_frozen_master_telemetry_cannot_keep_driving(self):
        self.ready()
        self.app.estop = False
        self.app.telem_at -= 1
        self.assertTrue(self.app._compute()[0])

    def test_fault_blocks_enable(self):
        self.ready()
        self.app.telem["sf"] = 0x10
        self.app.on_press("e")
        self.assertTrue(self.app.estop)

    def test_single_limit_with_preserved_reference_does_not_inhibit(self):
        for flag in (0x01, 0x02):
            self.ready()
            self.app.estop = True
            self.app.telem["sf"] = 0x60 | flag
            self.app.on_press("e")
            self.app.on_release("e")
            self.assertFalse(self.app.estop)
            self.assertFalse(self.app._compute()[0])

    def test_active_steering_remains_controllable(self):
        self.ready()
        self.app.estop = False
        self.app._pressed.add("d")
        with mock.patch.object(teleop.time, "monotonic", return_value=self.app.telem_at):
            self.assertEqual(self.app._compute()[4], 0)
        with mock.patch.object(teleop.time, "monotonic", return_value=self.app.telem_at + 0.075):
            self.assertAlmostEqual(self.app._compute()[4], 0.075)

    def test_steering_reaches_each_end_in_one_second(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.app.steer = 0
                self.app.steer_last_at = None
                self.assertEqual(self.app._apply_steering_input(10, direction), 0)
                self.assertAlmostEqual(self.app._apply_steering_input(10.5, direction), direction * 0.5)
                self.assertAlmostEqual(self.app._apply_steering_input(11, direction), direction)
                self.assertAlmostEqual(self.app._apply_steering_input(12, direction), direction)

    def test_steering_returns_to_center_in_two_seconds(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction):
                self.app.steer = direction
                self.app.steer_last_at = 10
                self.assertAlmostEqual(self.app._apply_steering_input(11, 0), direction * 0.5)
                self.assertAlmostEqual(self.app._apply_steering_input(12, 0), 0)
                self.assertEqual(self.app._apply_steering_input(13, 0), 0)

    def test_short_steering_tap_makes_small_correction_then_returns(self):
        self.app._apply_steering_input(10, 1)
        self.assertAlmostEqual(self.app._apply_steering_input(10.1, 1), 0.1)
        self.assertAlmostEqual(self.app._apply_steering_input(10.2, 0), 0.05)
        self.assertAlmostEqual(self.app._apply_steering_input(10.3, 0), 0)

    def test_steering_direction_change_passes_through_center(self):
        self.app.steer = 1
        self.app.steer_last_at = 10
        self.assertAlmostEqual(self.app._apply_steering_input(10.5, -1), 0.5)
        self.assertAlmostEqual(self.app._apply_steering_input(11, -1), 0)
        self.assertAlmostEqual(self.app._apply_steering_input(11.5, -1), -0.5)
        self.assertAlmostEqual(self.app._apply_steering_input(12, -1), -1)

    def test_steering_timing_is_independent_of_update_rate(self):
        for rate in (30, 60, 120):
            with self.subTest(rate=rate):
                self.app.period = 1 / rate
                self.app.steer = 0
                self.app.steer_last_at = None
                self.app._apply_steering_input(10, 1)
                for tick in range(1, rate // 2 + 1):
                    self.app._apply_steering_input(10 + tick / rate, 1)
                self.assertAlmostEqual(self.app.steer, 0.5)
                for tick in range(1, rate * 3 // 2 + 1):
                    self.app._apply_steering_input(10.5 + tick / rate, 0)
                self.assertEqual(self.app.steer, 0)

    def test_estop_resets_steering_timing_before_reenable(self):
        self.app.steer = 0.5
        self.app.steer_last_at = 10
        self.app._pressed.add("d")
        with mock.patch.object(teleop.time, "monotonic", return_value=20):
            self.assertEqual(self.app._compute()[4], 0)
        self.ready()
        self.app.telem_at = 20
        self.app.estop = False
        with mock.patch.object(teleop.time, "monotonic", return_value=20.075):
            self.assertAlmostEqual(self.app._compute()[4], 0.075)

    def test_stale_throttle_feedback_cannot_look_like_neutral(self):
        self.ready()
        self.app.telem["tr"] = 0
        self.assertIsNone(self.app._actual_gear())

    def test_shutdown_never_commands_center_offset(self):
        self.app.steer_trim = -0.4
        self.app.shutdown()
        self.assertEqual(self.act.send_all.call_count, 5)
        for call in self.act.send_all.call_args_list:
            self.assertTrue(call.kwargs["estop"])
            self.assertEqual(call.kwargs["steer"], 0)

    def test_all_levels_start_at_zero_and_reach_their_targets(self):
        for target, duration in zip(teleop.DEFAULT_SPEED_LEVELS, (1.5, 3, 19 / 3, 31 / 3)):
            with self.subTest(target=target):
                self.app._reset_high_speed_ramp()
                self.assertEqual(self.app._apply_high_speed_ramp(10, "D", target), 0)
                self.assertAlmostEqual(self.app._apply_high_speed_ramp(10.5, "D", target), 0.05)
                self.assertAlmostEqual(self.app._apply_high_speed_ramp(10 + duration, "D", target), target)
                self.assertAlmostEqual(self.app._apply_high_speed_ramp(30, "D", target), target)

    def test_upper_ramp_rate_and_threshold_crossing(self):
        self.app._apply_high_speed_ramp(10, "D", 0.85)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(11.5, "D", 0.85), 0.15)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(13.5, "D", 0.85), 0.3375)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(14.5, "D", 0.85), 0.4125)

    def test_upper_level_changes_take_three_to_five_seconds(self):
        self.app._apply_high_speed_ramp(10, "D", 0.30)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(13, "D", 0.30), 0.30)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(13, "D", 0.55), 0.30)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(15, "D", 0.55), 0.45)
        level_three_at = 13 + 10 / 3
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(level_three_at, "D", 0.55), 0.55)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(level_three_at, "D", 0.85), 0.55)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(level_three_at + 2, "D", 0.85), 0.70)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(level_three_at + 4, "D", 0.85), 0.85)

    def test_ramp_depends_on_elapsed_time_not_update_count(self):
        self.app._apply_high_speed_ramp(10, "D", 0.85)
        for tick in range(1, 36):
            self.app._apply_high_speed_ramp(10 + tick / 10, "D", 0.85)
        frequent = self.app.ramp_command
        self.app._reset_high_speed_ramp()
        self.app._apply_high_speed_ramp(10, "D", 0.85)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(13.5, "D", 0.85), frequent)

    def test_level_changes_continue_from_current_command(self):
        self.app._apply_high_speed_ramp(10, "D", 0.15)
        self.app._apply_high_speed_ramp(11.5, "D", 0.15)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(11.5, "D", 0.85), 0.15)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(13, "D", 0.85), 0.30)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(14, "D", 0.85), 0.375)
        self.assertAlmostEqual(self.app._apply_high_speed_ramp(14, "D", 0.15), 0.15)

    def test_release_brake_and_estop_cut_throttle_then_restart_from_zero(self):
        for action in ("release", "brake", "estop", "disable"):
            with self.subTest(action=action):
                self.ready()
                self.app.estop = False
                self.app.drive_enabled = True
                self.app.telem["tm"] = 1
                self.app.level = 0.85
                self.app._reset_high_speed_ramp()
                self.app._compute_drive(10, True, False, 0)
                self.assertAlmostEqual(self.app._compute_drive(13, True, False, 0)[0], 0.30)
                self.app.estop = action == "estop"
                self.app.drive_enabled = action != "disable"
                result = self.app._compute_drive(13.1, action != "release", False,
                                                 1 if action == "brake" else 0)
                self.assertEqual(result[0], 0)
                self.assertIsNone(self.app.ramp_last_at)
                self.app.estop = False
                self.app.drive_enabled = True
                self.assertEqual(self.app._compute_drive(13.2, True, False, 0)[0], 0)

    def test_gear_confirmation_is_required_before_starting_ramp(self):
        self.ready()
        self.app.estop = False
        self.app.drive_enabled = True
        self.assertEqual(self.app._compute_drive(10, True, False, 0)[0], 0)
        self.assertIsNone(self.app.ramp_last_at)
        self.app.telem["tm"] = 1
        self.assertEqual(self.app._compute_drive(12, True, False, 0)[0], 0)
        self.assertAlmostEqual(self.app._compute_drive(12.5, True, False, 0)[0], 0.05)

    def test_held_reverse_resumes_without_a_second_press(self):
        self.ready()
        self.app.estop = False
        self.app.drive_enabled = True
        self.app.last_motion_gear = "D"
        self.app.telem["tm"] = 1
        self.assertEqual(self.app._compute_drive(10, False, True, 0), (0, "N", 1))
        self.app.telem["tm"] = 0
        self.assertEqual(self.app._compute_drive(12, False, True, 0), (0, "N", 1))
        self.assertEqual(self.app.guard_until, 0)
        self.assertEqual(self.app._compute_drive(12.1, False, True, 0), (0, "R", 0))
        self.app.telem["tm"] = 3
        self.assertEqual(self.app._compute_drive(12.2, False, True, 0)[0], 0)
        self.assertGreater(self.app._compute_drive(12.3, False, True, 0)[0], 0)

    def test_direction_hold_is_half_second_in_both_directions(self):
        for old_gear, old_code, forward, reverse, target_code in (
                ("D", 1, False, True, 3), ("R", 3, True, False, 1)):
            with self.subTest(old_gear=old_gear):
                self.ready()
                self.app.estop = False
                self.app.drive_enabled = True
                self.app.last_motion_gear = old_gear
                self.app.telem["tm"] = old_code
                self.assertEqual(self.app._compute_drive(10, forward, reverse, 0), (0, "N", 1))
                self.assertAlmostEqual(self.app.guard_until, 10.5)
                self.app.telem["tm"] = 0
                self.assertEqual(self.app._compute_drive(10.499, forward, reverse, 0), (0, "N", 1))
                self.assertTrue(self.app.guard_until)
                self.app._compute_drive(10.5, forward, reverse, 0)
                self.assertEqual(self.app.guard_until, 0)
                # After the dwell, torque still waits for the requested gear.
                self.assertEqual(self.app._compute_drive(10.6, forward, reverse, 0)[0], 0)
                self.app.telem["tm"] = target_code
                self.assertEqual(self.app._compute_drive(10.7, forward, reverse, 0)[0], 0)
                self.assertGreater(self.app._compute_drive(10.8, forward, reverse, 0)[0], 0)

    def test_direction_hold_waits_for_neutral_even_after_timer_expires(self):
        self.ready()
        self.app.estop = False
        self.app.drive_enabled = True
        self.app.last_motion_gear = "D"
        self.app.telem["tm"] = 1
        self.app._compute_drive(10, False, True, 0)
        self.assertEqual(self.app._compute_drive(10.6, False, True, 0), (0, "N", 1))
        self.assertTrue(self.app.guard_until)
        self.app.telem["tm"] = 0
        self.app._compute_drive(10.7, False, True, 0)
        self.assertEqual(self.app.guard_until, 0)

    def test_both_direction_keys_keep_guard_active_until_one_is_released(self):
        self.ready()
        self.app.estop = False
        self.app.drive_enabled = True
        self.app.last_motion_gear = "D"
        self.app.telem["tm"] = 1
        self.assertEqual(self.app._compute_drive(10, True, True, 0), (0, "N", 1))
        self.app.telem["tm"] = 0
        self.assertEqual(self.app._compute_drive(16, True, True, 0), (0, "N", 1))
        self.assertTrue(self.app.guard_until)
        self.app._compute_drive(16.1, False, True, 0)
        self.assertEqual(self.app.guard_until, 0)
        self.assertEqual(self.app._compute_drive(16.2, False, True, 0), (0, "R", 0))

    def test_brake_still_cancels_throttle_after_held_reverse_change(self):
        self.ready()
        self.app.estop = False
        self.app.drive_enabled = True
        self.app.last_motion_gear = "D"
        self.app.telem["tm"] = 1
        self.app._compute_drive(10, False, True, 1)
        self.app.telem["tm"] = 0
        self.app._compute_drive(10.5, False, True, 1)
        self.app.telem["tm"] = 3
        self.assertEqual(self.app._compute_drive(10.6, False, True, 1), (0, "R", 1))
        self.assertEqual(self.app._compute_drive(10.7, False, True, 0)[0], 0)
        self.assertGreater(self.app._compute_drive(10.8, False, True, 0)[0], 0)


if __name__ == "__main__":
    unittest.main()
