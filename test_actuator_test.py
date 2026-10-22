"""Hardware-free tests for actuator_test.py."""

import contextlib
import io
import unittest

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
        for pulse in ('48', '401'):
            with self.subTest(pulse=pulse), self.assertRaises(ValueError):
                test_plan(parse_args(['throttle', '--pulse', pulse]), SETTINGS)
        for pulse in ('370', '400'):
            with self.subTest(pulse=pulse):
                plan = test_plan(parse_args(['throttle', '--pulse', pulse]), SETTINGS)
                self.assertEqual(plan['pulse'], int(pulse))

    def test_throttle_rejects_invalid_cli_calibration_overrides(self):
        for mode in (['--pulse', '385'], ['--speed', '0.1']):
            for option in ('--neutral-pwm', '--start-pwm', '--forward-pwm'):
                for value in ('-1', '0', '4095', 'nan', 'inf', '-inf'):
                    with self.subTest(mode=mode, option=option, value=value):
                        with contextlib.redirect_stderr(io.StringIO()):
                            with self.assertRaises(SystemExit) as error:
                                parse_args(['throttle'] + mode + [option + '=' + value])
                        self.assertEqual(error.exception.code, 2)

    def test_throttle_rejects_invalid_loaded_calibration(self):
        for mode in (['--pulse', '385'], ['--speed', '0.1']):
            args = parse_args(['throttle'] + mode)
            for key in ('THROTTLE_STOPPED_PWM', 'THROTTLE_FORWARD_START_PWM',
                        'THROTTLE_FORWARD_PWM'):
                for value in (-1, 0, 4095, float('nan'), float('inf')):
                    with self.subTest(mode=mode, key=key, value=value):
                        settings = dict(SETTINGS)
                        settings[key] = value
                        with self.assertRaises(ValueError):
                            test_plan(args, settings)

    def test_throttle_rejects_inconsistent_calibration_overrides(self):
        for mode in (['--pulse', '385'], ['--speed', '0.1']):
            for overrides in (['--neutral-pwm', '380'],
                              ['--start-pwm', '370'],
                              ['--forward-pwm', '375']):
                with self.subTest(mode=mode, overrides=overrides):
                    args = parse_args(['throttle'] + mode + overrides)
                    with self.assertRaises(ValueError):
                        test_plan(args, SETTINGS)

    def test_throttle_accepts_valid_calibration_overrides(self):
        for mode, expected in ((['--pulse', '405'], 405),
                               (['--speed', '0.3'], 410)):
            with self.subTest(mode=mode):
                args = parse_args(['throttle'] + mode + [
                    '--neutral-pwm', '360', '--start-pwm', '380',
                    '--forward-pwm', '410'])
                plan = test_plan(args, SETTINGS)
                self.assertEqual(plan['pulse'], expected)
                self.assertEqual(plan['safe_pulse'], 360)


if __name__ == '__main__':
    unittest.main()
