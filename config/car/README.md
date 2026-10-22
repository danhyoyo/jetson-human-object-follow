# Bundled vehicle configuration

`config.py` and `myconfig.py` contain the complete DonkeyCar configuration
used by `person_follow.py` and `actuator_test.py`.  The application defaults to
this directory, so the project does not depend on `/home/jetson/mycar`.

The configuration is specific to the calibrated vehicle:

- PCA9685 channel 0: steering servo
- PCA9685 channel 1: throttle ESC
- steering left/center/right: 525/420/280
- throttle stopped/start/max: 370/380/400
- reverse: disabled

Keep the wheels raised and cut ESC power before changing these values.
