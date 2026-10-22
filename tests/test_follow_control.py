"""Hardware-free checks for the target-following controller."""

import sys
import unittest
from pathlib import Path

# Add project root and src/ to sys.path
PROJECT_DIR = Path(__file__).resolve().parent.parent
for p in (str(PROJECT_DIR), str(PROJECT_DIR / 'src')):
    if p not in sys.path:
        sys.path.insert(0, p)

from follow_control import Box, FollowController
from donkey_drive import CalibratedPWMDrive, load_pwm_settings, SteeringOnlyDrive
from red_detector import detect_largest_red

import numpy as np


class FollowControllerTest(unittest.TestCase):
    def setUp(self):
        self.control = FollowController(speed=0.12, target_height=0.55)

    def test_requires_three_stable_frames(self):
        target = Box(0.35, 0.20, 0.65, 0.60)
        self.assertEqual(self.control.update([target]).state, 'ACQUIRING')
        self.assertEqual(self.control.update([target]).state, 'ACQUIRING')
        command = self.control.update([target])
        self.assertEqual(command.state, 'FOLLOW')
        self.assertEqual(command.throttle, 0.12)

    def test_stops_for_unsafe_target_counts(self):
        target = Box(0.35, 0.20, 0.65, 0.60)
        self.assertEqual(self.control.update([]).state, 'NO_PERSON')
        self.assertEqual(self.control.update([target, target]).state, 'MULTIPLE_PEOPLE')

    def test_holds_when_target_is_close(self):
        target = Box(0.25, 0.15, 0.75, 0.75)
        self.control.update([target])
        self.control.update([target])
        command = self.control.update([target])
        self.assertEqual(command.state, 'HOLD')
        self.assertEqual(command.throttle, 0.0)

    def test_steers_towards_target(self):
        target = Box(0.55, 0.20, 0.85, 0.60)
        self.control.update([target])
        self.control.update([target])
        command = self.control.update([target])
        self.assertEqual(command.state, 'FOLLOW')
        self.assertGreater(command.steering, 0.0)

    def test_ignore_distance_tracks_partial_close_target(self):
        control = FollowController(speed=0.08, target_height=0.2,
                                   ignore_distance=True)
        target = Box(0.0, 0.02, 0.60, 1.0)
        control.update([target])
        control.update([target])
        command = control.update([target])
        self.assertEqual(command.state, 'FOLLOW_ANY_DISTANCE')
        self.assertEqual(command.throttle, 0.08)
        self.assertLess(command.steering, 0.0)

    def test_allow_partial_still_stops_at_half_height(self):
        control = FollowController(speed=0.08, target_height=0.5,
                                   allow_partial=True)
        target = Box(0.0, 0.1, 0.6, 0.7)
        control.update([target])
        control.update([target])
        command = control.update([target])
        self.assertEqual(command.state, 'HOLD')
        self.assertEqual(command.throttle, 0.0)

    def test_allow_partial_tracks_small_target(self):
        control = FollowController(speed=0.08, target_height=0.5,
                                   allow_partial=True, distance_hysteresis=0.0)
        target = Box(0.65, 0.45, 0.72, 0.50)
        control.update([target])
        control.update([target])
        command = control.update([target])
        self.assertEqual(command.state, 'FOLLOW')
        self.assertGreater(command.steering, 0.0)

    def test_red_mode_follows_until_half_height(self):
        control = FollowController(speed=0.08, target_height=0.5,
                                   allow_partial=True, distance_hysteresis=0.0)
        target = Box(0.2, 0.2, 0.7, 0.69)
        control.update([target])
        control.update([target])
        command = control.update([target])
        self.assertEqual(command.state, 'FOLLOW')
        self.assertEqual(command.throttle, 0.08)


class RedDetectorTest(unittest.TestCase):
    def test_detects_largest_red_object(self):
        image = np.zeros((100, 160, 3), dtype=np.uint8)
        image[20:80, 10:70] = (220, 20, 25)
        image[5:10, 130:135] = (255, 0, 0)
        detection = detect_largest_red(image, min_area_fraction=0.002)
        self.assertIsNotNone(detection)
        self.assertAlmostEqual(detection.left, 10 / 160, places=2)
        self.assertAlmostEqual(detection.right, 70 / 160, places=2)
        self.assertAlmostEqual(detection.top, 20 / 100, places=2)
        self.assertAlmostEqual(detection.bottom, 80 / 100, places=2)

    def test_rejects_small_red_noise(self):
        image = np.zeros((100, 160, 3), dtype=np.uint8)
        image[5:7, 5:7] = (255, 0, 0)
        self.assertIsNone(detect_largest_red(image, min_area_fraction=0.01))

    def test_red_thresholds_are_strict(self):
        image = np.zeros((100, 160, 3), dtype=np.uint8)
        image[20:80, 20:100] = (85, 20, 20)
        self.assertIsNone(detect_largest_red(image))

        image[20:80, 20:100] = (86, 45, 40)
        self.assertIsNotNone(detect_largest_red(image))


class CalibrationTest(unittest.TestCase):
    def test_rejects_invalid_active_throttle_calibration(self):
        class Config(object):
            PWM_STEERING_THROTTLE = {
                'PWM_STEERING_PIN': 'PCA9685.1:40.1',
                'PWM_STEERING_SCALE': 1.0,
                'PWM_STEERING_INVERTED': False,
                'PWM_THROTTLE_PIN': 'PCA9685.1:40.0',
                'PWM_THROTTLE_SCALE': 1.0,
                'PWM_THROTTLE_INVERTED': False,
                'STEERING_LEFT_PWM': 460,
                'STEERING_CENTER_PWM': 420,
                'STEERING_RIGHT_PWM': 290,
                'THROTTLE_FORWARD_START_PWM': 395,
                'THROTTLE_FORWARD_PWM': 50,
                'THROTTLE_STOPPED_PWM': 370,
                'THROTTLE_REVERSE_PWM': 50,
            }
            STEERING_LEFT_PWM = 460
            STEERING_RIGHT_PWM = 290
            THROTTLE_FORWARD_PWM = 500
            THROTTLE_STOPPED_PWM = 370
            THROTTLE_REVERSE_PWM = 220

        with self.assertRaises(ValueError):
            load_pwm_settings(Config())

    def test_accepts_forward_only_safe_calibration(self):
        class Config(object):
            PWM_STEERING_THROTTLE = {
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
                'THROTTLE_FORWARD_START_PWM': 395,
                'THROTTLE_FORWARD_PWM': 400,
            }

        settings = load_pwm_settings(Config())
        self.assertEqual(settings['THROTTLE_FORWARD_START_PWM'], 395)

    def test_positive_follow_throttle_skips_deadband_and_stays_capped(self):
        class Controller(object):
            def __init__(self):
                self.pulses = []

            def set_pulse(self, pulse):
                self.pulses.append(pulse)

        settings = {
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
        steering = Controller()
        throttle = Controller()
        drive = CalibratedPWMDrive(steering, throttle, settings)
        drive.write(0.0, 0.0)
        drive.write(0.0, 0.08)
        drive.write(0.0, 0.3)
        self.assertEqual(throttle.pulses[-3:], [370, 385, 400])

    def test_steering_only_never_writes_throttle(self):
        class Controller(object):
            def __init__(self):
                self.pulses = []

            def set_pulse(self, pulse):
                self.pulses.append(pulse)

        settings = {
            'PWM_STEERING_PIN': 'PCA9685.1:40.1',
            'PWM_STEERING_SCALE': 1.0,
            'PWM_STEERING_INVERTED': False,
            'STEERING_LEFT_PWM': 460,
            'STEERING_CENTER_PWM': 420,
            'STEERING_RIGHT_PWM': 290,
        }
        steering = Controller()
        drive = SteeringOnlyDrive(steering, settings)
        drive.write(-0.5, 0.0)
        self.assertEqual(len(steering.pulses), 2)
        with self.assertRaises(ValueError):
            drive.write(0.0, 0.01)


if __name__ == '__main__':
    unittest.main()
