# Human Follow

Complete Jetson Nano/DonkeyCar project for following either one detected person
or the largest red object.  Person mode uses `ssd-mobilenet-v2`; red-object
mode uses a fast NumPy RGB detector and does not use an AI model.  Preview is
the default and never opens the motor controller.

## Project contents

```text
human_follow/
├── person_follow.py         main camera, detection and follow application
├── follow_control.py        hardware-independent target controller
├── red_detector.py          connected-component red-object detector
├── donkey_drive.py          guarded PCA9685 steering/throttle output
├── actuator_test.py         dry-run-by-default actuator diagnostic
├── car/
│   ├── config.py            minimal bundled DonkeyCar configuration
│   └── myconfig.py          calibrated pins and PWM values
├── models/SSD-Mobilenet-v2/
│   ├── ssd_mobilenet_v2_coco.uff  portable person-detection model
│   └── ssd_coco_labels.txt        MS-COCO class labels
├── test_follow_control.py   controller, detector and drive tests
├── test_actuator_test.py    PWM mapping and diagnostic tests
├── requirements.txt         PyPI dependencies
└── README.md
```

`jetson_myconfig.py` is retained as a snapshot of the original full DonkeyCar
configuration.  Runtime code uses the smaller bundled `car/` configuration.

## Requirements

- Jetson Nano with JetPack 4 and a CSI camera.
- Python 3.6+.
- DonkeyCar 4.5.1 and NumPy.
- `jetson_inference` and `jetson_utils` Python bindings.  The portable
  `ssd-mobilenet-v2` UFF model and labels are bundled in this project.
- PCA9685 at I2C bus 1, address `0x40`.
- Steering servo on PCA9685 channel 0 and ESC on channel 1.
- An HDMI display for preview, unless `--headless` is used.

The `jetson_inference`/`jetson_utils` bindings come from the Jetson inference
installation and are not normal PyPI packages.  On the configured vehicle the
remaining dependencies are already available in:

```bash
/home/jetson/.virtualenv/donkeycar/bin/python
```

For a fresh compatible environment, install the PyPI dependencies with:

```bash
python -m pip install -r requirements.txt
```

## Copy from Windows to the Jetson

From the directory containing `human_follow`:

```powershell
scp -r .\human_follow jetson@10.95.212.115:/home/jetson/
```

The project is self-contained after copying.  It no longer requires
`/home/jetson/mycar`; both applications default to `human_follow/car`.  An
alternative DonkeyCar configuration can still be selected with
`--donkey-dir /path/to/car`.

If using the packaged archive instead, copy and extract it:

```powershell
scp .\human_follow_jetson.tar.gz jetson@10.95.212.115:/home/jetson/
```

```bash
cd /home/jetson
tar -xzf human_follow_jetson.tar.gz
cd human_follow
```

Run the bundled setup check.  It verifies the bundled model checksum, installs
the model into jetson-inference's network directory if missing, verifies
imports, runs all hardware-free tests, validates `car/myconfig.py`, and
performs steering/throttle dry-runs; it does not apply actuator PWM:

```bash
bash setup_jetson.sh
```

On a fresh DonkeyCar virtual environment, install the listed PyPI packages and
then check with:

```bash
bash setup_jetson.sh --install-python-deps
```

This cannot install the Jetson-specific `jetson_inference` and `jetson_utils`
bindings.  Install those as part of jetson-inference/JetPack first if the setup
check reports them missing.  A TensorRT `.engine` is not portable and is not
bundled; the first person-mode run on another Jetson will build its own engine
and can take several minutes.

## Safety

- Preview mode is the first test to run because it never opens PWM hardware.
- Raise all driven wheels before using `--drive`, `--steering-test`, or
  `actuator_test.py --apply`.
- Be ready to disconnect ESC power.  Stop immediately if the steering servo
  buzzes, binds, or reaches a mechanical stop.
- Reverse is disabled.  Raw throttle tests are locked to PWM 370..400.
- Do not swap the two PCA9685 connections unless the bundled pin mapping is
  also changed.

## Preview on the Jetson HDMI display

```bash
cd /home/jetson/human_follow
/home/jetson/.virtualenv/donkeycar/bin/python person_follow.py
```

The preview shows SSD-MobileNet-v2 person boxes, labels, confidence, and the
proposed steering/throttle state.  Press Ctrl+C to stop.

The JetPack 4 CSI backend on this device is stable only on its default
zero-copy path, which produces a 180-degree camera image.  The application
therefore rotates each captured frame before both inference and display.  Use
`--no-software-rotate` only if the physical camera is mounted in the opposite
orientation.  `--input-flip` remains available for non-CSI inputs.

Use `--headless` only when no video window is wanted.  `--output` accepts a
different jetson-utils output URI.

## Motor modes

Do not use these modes until the car is raised so its wheels cannot touch the
ground and the PWM calibration has been checked.

```bash
# Steering only; the throttle PWM channel is disabled (no ESC pulse)
/home/jetson/.virtualenv/donkeycar/bin/python person_follow.py --steering-test

# Follow at a deliberately limited throttle (370 stop, 380 range start, 400 max)
/home/jetson/.virtualenv/donkeycar/bin/python person_follow.py --drive --speed 0.08
```

Motor control is never enabled without `--drive` or `--steering-test`.
The watchdog stops output if a capture/inference/render cycle becomes stale.
Drive mode requires a valid active DonkeyCar PWM dictionary and never falls
back to older calibration values.  This prevents an unverified legacy neutral
value from being sent to the ESC.

Both person and red-object tracking use the same forward-only ESC mapping:
zero throttle sends PWM 370, while any positive throttle is mapped into the
configured range 380..400.  The normalized `--speed` range remains
0..0.3; 0.3 now means the configured safe cap of PWM 400, not full ESC power.

Steering uses the calibrated asymmetric range left/center/right =
525/420/280.  The default tracking gain is 1.0 and the normalized steering
limit is 0.70 for stronger turns under vehicle load.  Reduce it at runtime
with `--steering-limit 0.5` if the steering oscillates or the servo strains.

For a raised-wheel/handheld bench test, `--ignore-distance` keeps tracking one
person regardless of bounding-box height or whether the box touches an image
edge.  This disables distance and partial-target stops, so do not use it with
the car on the ground near people.

## Red object mode (no AI detector)

`--red-object` tracks the largest connected red region using RGB color
dominance.  Its box may touch the image edge.  Forward throttle stops when the
box reaches 50% of image height, while steering continues to use the box's
horizontal center.

The current color rule requires `red > 85`, red to exceed green and blue by
more than 40, and red to be at least 1.4 times each of green and blue.  The
minimum red area remains 0.002 of the image.

```bash
# Preview only
/home/jetson/.virtualenv/donkeycar/bin/python person_follow.py --red-object

# Steering-only bench test (throttle PWM disabled)
/home/jetson/.virtualenv/donkeycar/bin/python person_follow.py \
  --red-object --steering-test --steering-gain 1.0 --steering-limit 0.6
```

## Hardware-free tests

```bash
cd /home/jetson/human_follow
PYTHONDONTWRITEBYTECODE=1 /home/jetson/.virtualenv/donkeycar/bin/python \
  -m unittest -v test_follow_control.py test_actuator_test.py
```

These tests do not open the camera, PCA9685, servo, or ESC.

## Timed actuator diagnostics

`actuator_test.py` is a dry-run by default.  It controls only the selected
PCA9685 channel, runs for at most five seconds, and disables that output in a
`finally` block.  On this car the observed wiring is channel 0 for steering
and channel 1 for the ESC.  Keep the wheels raised before using `--apply`.

```bash
# Dry-run: center steering calculation, no hardware write
python actuator_test.py steering --value 0

# Apply steering left for one second, then disable without a second movement
python actuator_test.py --apply --duration 1 steering --value -0.5

# Add --return-center only when a deliberate second move back to center is wanted
python actuator_test.py --apply --duration 1 steering --value -0.5 --return-center

# Raw output is locked to the configured safe interval 370..400
python actuator_test.py --apply --duration 0.5 throttle --pulse 370

# Normalized test uses the configured 370/380/400 mapping
python actuator_test.py --apply --duration 1 throttle \
  --speed 0.05
```

## Useful runtime options

```text
--red-object              use the RGB red-object detector
--drive                   enable steering and forward throttle
--steering-test           enable steering while disabling ESC PWM output
--speed 0..0.3            constant throttle command while FOLLOW is active
--target-height VALUE     stop at this normalized box height
--ignore-distance         ignore distance/edge stops for a raised-wheel test
--steering-gain VALUE     horizontal-error gain; default 1.0
--steering-limit VALUE    absolute steering limit; default 0.70
--headless                do not open an HDMI preview window
--no-software-rotate      disable the default 180-degree correction
--donkey-dir PATH         use another config.py/myconfig.py directory
```

Throttle mapping is intentionally quantized to integer PCA9685 pulses.  With
the current 370/380/400 calibration, `--speed 0.08` maps to PWM 385 and
`--speed 0.30` maps to PWM 400.  Earlier physical testing observed movement
near PWM 395, so low speed settings may remain inside the ESC deadband.
