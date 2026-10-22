"""Calibrated hardware configuration for this Jetson/DonkeyCar vehicle."""

DRIVE_TRAIN_TYPE = 'PWM_STEERING_THROTTLE'
DONKEY_GYM = False

PWM_STEERING_THROTTLE = {
    # Observed wiring: channel 0 is steering, channel 1 is the ESC.
    'PWM_STEERING_PIN': 'PCA9685.1:40.0',
    'PWM_STEERING_SCALE': 1.0,
    'PWM_STEERING_INVERTED': False,
    'PWM_THROTTLE_PIN': 'PCA9685.1:40.1',
    'PWM_THROTTLE_SCALE': 1.0,
    'PWM_THROTTLE_INVERTED': False,

    # Independently calibrated asymmetric steering positions.
    'STEERING_LEFT_PWM': 525,
    'STEERING_CENTER_PWM': 420,
    'STEERING_RIGHT_PWM': 280,

    # Forward-only ESC range.  Reverse is intentionally disabled.
    'THROTTLE_STOPPED_PWM': 370,
    'THROTTLE_FORWARD_START_PWM': 380,
    'THROTTLE_FORWARD_PWM': 400,
    'THROTTLE_REVERSE_PWM': 370,
}
