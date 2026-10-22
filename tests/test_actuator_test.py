"""Hardware-free tests for actuator_test.py."""

import sys
import unittest
from pathlib import Path

# Add project root, src/, and tools/ to sys.path
PROJECT_DIR = Path(__file__).resolve().parent.parent
for p in (str(PROJECT_DIR), str(PROJECT_DIR / 'src'), str(PROJECT_DIR / 'tools')):
    if p not in sys.path:
        sys.path.insert(0, p)

from actuator_test import parse_args, steering_pulse, test_plan, throttle_pulse


SETTINGS = {
    'PWM_STEERING_PIN': 'PCA9685.1:40.0',
    'PWM_STEERING_SCALE': 1.0,
    'PWM_STEERING_INVERTED': False,
    'PWM_THROTTLE_PIN': 'PCA9685.1:40.1',
    'PWM_THROTTLE_SCALE': 1.0,
    'PWM_THROTTLE_INVERTED': False,
    'STEERING_LEFT_PWM': 460,
    'STEERING_CENTER_PWM': 420,
    'STEERING_RIGHT_PWM': 290,
    'THROTTLE_STOPPED_PWM': 370,
    'THROTTLE_FORWARD_START_PWM': 380,
    'THROTTLE_FORWARD_PWM': 400,
}


class PulseMathTest(unittest.TestCase):
    def test_steering_endpoints_and_center(self):
        self.assertEqual(steering_pulse(-1, 460, 420, 290), 460)
        self.assertEqual(steering_pulse(0, 460, 420, 290), 420)
        self.assertEqual(steering_pulse(1, 460, 420, 290), 290)
        self.assertEqual(steering_pulse(-0.5, 460, 420, 290), 440)
        self.assertEqual(steering_pulse(0.5, 460, 420, 290), 355)

    def test_throttle_speed(self):
        self.assertEqual(throttle_pulse(0.0, 370, 380, 400), 370)
        self.assertEqual(throttle_pulse(0.1, 370, 380, 400), 386)
        self.assertEqual(throttle_pulse(0.3, 370, 380, 400), 400)

    def test_steering_plan_uses_configured_channel(self):
        args = parse_args(['steering', '--value', '-0.5'])
        plan = test_plan(args, SETTINGS)
        self.assertEqual(plan['pin'], 'PCA9685.1:40.0')
        self.assertEqual(plan['pulse'], 440)
        self.assertIsNone(plan['safe_pulse'])

    def test_return_center_is_explicit(self):
        args = parse_args(['steering', '--value', '-1', '--return-center'])
        plan = test_plan(args, SETTINGS)
        self.assertEqual(plan['pulse'], 460)
        self.assertEqual(plan['safe_pulse'], 420)

    def test_raw_throttle_is_locked_to_calibrated_range(self):
        args = parse_args(['throttle', '--pulse', '380'])
        plan = test_plan(args, SETTINGS)
        self.assertEqual(plan['pin'], 'PCA9685.1:40.1')
        self.assertEqual(plan['pulse'], 380)
        self.assertEqual(plan['safe_pulse'], 370)
        with self.assertRaises(ValueError):
            test_plan(parse_args(['throttle', '--pulse', '48']), SETTINGS)


if __name__ == '__main__':
    unittest.main()
