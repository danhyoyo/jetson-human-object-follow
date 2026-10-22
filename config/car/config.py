"""Minimal DonkeyCar configuration required by human_follow.

This project does not use DonkeyCar's manage.py vehicle pipeline.  It loads
only the drivetrain type and the PWM dictionary through dk.load_config().
"""

DRIVE_TRAIN_TYPE = 'PWM_STEERING_THROTTLE'
DONKEY_GYM = False

# Replaced by car/myconfig.py.  Keeping the key here makes the relationship
# explicit and produces a clear failure if the override file is missing.
PWM_STEERING_THROTTLE = {}
