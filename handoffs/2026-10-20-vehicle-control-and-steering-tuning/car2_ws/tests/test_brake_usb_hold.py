import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'teleop'))
from brake_usb_hold_test import command_for


class BrakeHoldKeys(unittest.TestCase):
    def test_apply_release_and_keyup(self):
        for keys, command in [({'a'}, b'a'), ({'space'}, b'a'),
                              ({'a', 'space'}, b'a'), ({'r'}, b'r'), (set(), b'x')]:
            with self.subTest(keys=keys):
                self.assertEqual(command_for(keys), command)

    def test_conflicting_directions_stop(self):
        self.assertEqual(command_for({'a', 'r'}), b'x')
        self.assertEqual(command_for({'space', 'r'}), b'x')

    def test_stop_overrides_held_keys(self):
        for keys in ({'a', 'x'}, {'space', 'x'}, {'r', 'x'}):
            self.assertEqual(command_for(keys), b'x')
