"""DonkeyCar PWM output and command expiry, without ESC calibration sweeps."""

import math
import re
import threading
import time
from pathlib import Path


def validate_settings(settings):
    if not isinstance(settings, dict):
        raise ValueError('PWM_STEERING_THROTTLE must be a dictionary')
    try:
        pin_ids = []
        for kind in ('STEERING', 'THROTTLE'):
            pin = settings['PWM_{}_PIN'.format(kind)]
            match = re.fullmatch(r'PCA9685\.(\d+):([0-9a-fA-F]+)\.(\d+)', pin)
            if not match:
                raise ValueError('This adapter requires PCA9685 pin identifiers')
            bus, address, channel = match.groups()
            pin_id = (int(bus), int(address, 16), int(channel))
            if not 1 <= pin_id[1] <= 127 or not 0 <= pin_id[2] <= 15:
                raise ValueError('Invalid PCA9685 address/channel')
            pin_ids.append(pin_id)
            scale = settings['PWM_{}_SCALE'.format(kind)]
            inverted = settings['PWM_{}_INVERTED'.format(kind)]
            if not math.isfinite(scale) or not 0 < scale <= 2:
                raise ValueError('PWM scale must be finite and in (0, 2]')
            if not isinstance(inverted, bool):
                raise ValueError('PWM inverted settings must be booleans')
            keys = (['STEERING_LEFT_PWM', 'STEERING_CENTER_PWM',
                     'STEERING_RIGHT_PWM'] if kind == 'STEERING'
                    else ['THROTTLE_FORWARD_START_PWM', 'THROTTLE_FORWARD_PWM',
                          'THROTTLE_STOPPED_PWM'])
            for key in keys:
                pulse = settings[key]
                if not math.isfinite(pulse) or not 0 < pulse < 4095:
                    raise ValueError('Invalid PWM pulse: ' + key)
                scaled = (4095 - pulse if inverted else pulse) * scale
                if not 0 < scaled < 4095:
                    raise ValueError('Scaled PWM pulse is out of range: ' + key)
        if pin_ids[0] == pin_ids[1]:
            raise ValueError('Steering and throttle cannot use the same pin')
        if settings['STEERING_LEFT_PWM'] == settings['STEERING_RIGHT_PWM']:
            raise ValueError('Steering endpoints must differ')
        if not (min(settings['STEERING_LEFT_PWM'], settings['STEERING_RIGHT_PWM'])
                < settings['STEERING_CENTER_PWM']
                < max(settings['STEERING_LEFT_PWM'], settings['STEERING_RIGHT_PWM'])):
            raise ValueError('Steering center must be strictly between endpoints')
        if not (settings['THROTTLE_STOPPED_PWM']
                < settings['THROTTLE_FORWARD_START_PWM']
                <= settings['THROTTLE_FORWARD_PWM']):
            raise ValueError('Expected stopped < forward-start <= forward PWM calibration')
    except (KeyError, TypeError) as error:
        raise ValueError('Missing or invalid drivetrain setting: {}'.format(error))


def validate_steering_settings(settings):
    """Validate only steering fields, without trusting throttle calibration."""
    if not isinstance(settings, dict):
        raise ValueError('PWM_STEERING_THROTTLE must be a dictionary')
    try:
        pin = settings['PWM_STEERING_PIN']
        match = re.fullmatch(r'PCA9685\.(\d+):([0-9a-fA-F]+)\.(\d+)', pin)
        if not match:
            raise ValueError('Steering test requires a PCA9685 pin identifier')
        _, address, channel = match.groups()
        if not 1 <= int(address, 16) <= 127 or not 0 <= int(channel) <= 15:
            raise ValueError('Invalid steering PCA9685 address/channel')
        scale = settings['PWM_STEERING_SCALE']
        inverted = settings['PWM_STEERING_INVERTED']
        if not math.isfinite(scale) or not 0 < scale <= 2:
            raise ValueError('Steering PWM scale must be finite and in (0, 2]')
        if not isinstance(inverted, bool):
            raise ValueError('Steering PWM inverted setting must be boolean')
        for key in ('STEERING_LEFT_PWM', 'STEERING_CENTER_PWM', 'STEERING_RIGHT_PWM'):
            pulse = settings[key]
            if not math.isfinite(pulse) or not 0 < pulse < 4095:
                raise ValueError('Invalid PWM pulse: ' + key)
        if settings['STEERING_LEFT_PWM'] == settings['STEERING_RIGHT_PWM']:
            raise ValueError('Steering endpoints must differ')
        if not (min(settings['STEERING_LEFT_PWM'], settings['STEERING_RIGHT_PWM'])
                < settings['STEERING_CENTER_PWM']
                < max(settings['STEERING_LEFT_PWM'], settings['STEERING_RIGHT_PWM'])):
            raise ValueError('Steering center must be strictly between endpoints')
    except (KeyError, TypeError) as error:
        raise ValueError('Missing or invalid steering setting: {}'.format(error))


def load_pwm_settings(cfg):
    """Return only the active, fully validated drivetrain configuration."""
    settings = dict(cfg.PWM_STEERING_THROTTLE)
    validate_settings(settings)
    return settings


def steering_to_pulse(steering, settings):
    """Map each steering half around the independently calibrated center."""
    if not math.isfinite(steering) or not -1.0 <= steering <= 1.0:
        raise ValueError('Steering command must be finite and in [-1, 1]')
    center = settings['STEERING_CENTER_PWM']
    endpoint = (settings['STEERING_LEFT_PWM'] if steering < 0
                else settings['STEERING_RIGHT_PWM'])
    return int(center + abs(steering) * (endpoint - center))


class CalibratedPWMDrive:
    def __init__(self, steering_controller, throttle_controller, settings):
        validate_settings(settings)
        self.steering_controller = steering_controller
        self.throttle_controller = throttle_controller
        self.settings = dict(settings)
        self.stop()

    def stop(self):
        # Attempt neutral throttle first, and centering even if throttle fails.
        try:
            self.throttle_controller.set_pulse(int(self.settings['THROTTLE_STOPPED_PWM']))
        finally:
            self.steering_controller.set_pulse(int(self.settings['STEERING_CENTER_PWM']))

    def write(self, steering, throttle):
        if (not math.isfinite(steering) or not -1.0 <= steering <= 1.0
                or not math.isfinite(throttle) or not 0 <= throttle <= 0.3):
            raise ValueError('Command exceeds following limits or contains invalid values')
        steering_pulse = steering_to_pulse(steering, self.settings)
        neutral = self.settings['THROTTLE_STOPPED_PWM']
        forward_start = self.settings['THROTTLE_FORWARD_START_PWM']
        forward = self.settings['THROTTLE_FORWARD_PWM']
        # Zero stays at true neutral.  Positive application throttle 0..0.3
        # spans the configured limited forward range rather than full ESC power.
        throttle_pulse = (neutral if throttle == 0 else int(
            forward_start + throttle / 0.3 * (forward - forward_start)))
        # Stopping takes priority over steering output.
        if throttle == 0:
            self.throttle_controller.set_pulse(throttle_pulse)
            self.steering_controller.set_pulse(steering_pulse)
        else:
            self.steering_controller.set_pulse(steering_pulse)
            self.throttle_controller.set_pulse(throttle_pulse)


class SteeringOnlyDrive:
    """Drive adapter that controls the steering servo and never pulses the ESC."""
    def __init__(self, steering_controller, settings):
        validate_steering_settings(settings)
        self.steering_controller = steering_controller
        self.settings = dict(settings)
        self.stop()

    def _set_steering(self, steering):
        if not math.isfinite(steering) or not -1.0 <= steering <= 1.0:
            raise ValueError('Steering command must be finite and in [-1, 1]')
        pulse = steering_to_pulse(steering, self.settings)
        self.steering_controller.set_pulse(pulse)

    def stop(self):
        self._set_steering(0.0)

    def write(self, steering, throttle):
        if throttle != 0:
            raise ValueError('Steering-only mode refuses non-zero throttle')
        self._set_steering(steering)


class CheckedPwmPin:
    """Preserve DonkeyCar pin setup but propagate PCA9685 write failures.

    DonkeyCar 4.5.1's PCA9685.set_duty_cycle logs and swallows I2C errors.
    Forward the final write to its existing I2C object so DriveGuard can stop
    and latch the failure. Only non-extreme duty cycles are used by this app.
    """
    def __init__(self, pin):
        self.pin = pin

    def state(self):
        return self.pin.state()

    def start(self):
        self.pin.start()

    def duty_cycle(self, duty):
        if not math.isfinite(duty) or not 0 < duty < 1:
            raise ValueError('Expected a servo/ESC pulse strictly between 0 and 1 duty')
        self.pin.pca9685.pwm.set_pwm(self.pin.pin_number, 0, int(4096 * duty))


def build_donkey_drive(car_dir):
    """Load the existing config; instantiate only DonkeyCar's PulseController."""
    import donkeycar as dk
    from donkeycar.parts import pins
    from donkeycar.parts.actuator import PulseController

    directory = Path(car_dir).expanduser().resolve()
    if not (directory / 'config.py').is_file() or not (directory / 'myconfig.py').is_file():
        raise ValueError('--donkey-dir must contain config.py and myconfig.py')
    cfg = dk.load_config(config_path=str(directory / 'config.py'), myconfig='myconfig.py')
    if getattr(cfg, 'DONKEY_GYM', False):
        raise ValueError('The loaded configuration uses a simulator')
    if getattr(cfg, 'DRIVE_TRAIN_TYPE', None) != 'PWM_STEERING_THROTTLE':
        raise ValueError('This adapter supports PWM_STEERING_THROTTLE only')
    settings = load_pwm_settings(cfg)
    print('Motor configuration:', settings, flush=True)

    def controller(kind):
        return PulseController(
            pwm_pin=CheckedPwmPin(pins.pwm_pin_by_id(settings['PWM_{}_PIN'.format(kind)])),
            pwm_scale=settings['PWM_{}_SCALE'.format(kind)],
            pwm_inverted=settings['PWM_{}_INVERTED'.format(kind)])

    throttle = controller('THROTTLE')
    throttle.set_pulse(int(settings['THROTTLE_STOPPED_PWM']))
    try:
        steering = controller('STEERING')
        return CalibratedPWMDrive(steering, throttle, settings)
    except BaseException:
        throttle.set_pulse(int(settings['THROTTLE_STOPPED_PWM']))
        raise


def build_steering_drive(car_dir):
    """Disable the ESC channel, then instantiate only the steering controller."""
    import donkeycar as dk
    from donkeycar.parts import pins
    from donkeycar.parts.actuator import PulseController

    directory = Path(car_dir).expanduser().resolve()
    if not (directory / 'config.py').is_file() or not (directory / 'myconfig.py').is_file():
        raise ValueError('--donkey-dir must contain config.py and myconfig.py')
    cfg = dk.load_config(config_path=str(directory / 'config.py'), myconfig='myconfig.py')
    if getattr(cfg, 'DONKEY_GYM', False):
        raise ValueError('The loaded configuration uses a simulator')
    if getattr(cfg, 'DRIVE_TRAIN_TYPE', None) != 'PWM_STEERING_THROTTLE':
        raise ValueError('This adapter supports PWM_STEERING_THROTTLE only')
    settings = dict(cfg.PWM_STEERING_THROTTLE)
    validate_steering_settings(settings)

    throttle_pin = settings.get('PWM_THROTTLE_PIN')
    if not isinstance(throttle_pin, str) or not re.fullmatch(
            r'PCA9685\.(\d+):([0-9a-fA-F]+)\.(\d+)', throttle_pin):
        raise ValueError('Cannot safely disable the configured throttle output')
    raw_throttle = pins.pwm_pin_by_id(throttle_pin)
    raw_throttle.pca9685.pwm.set_pwm(raw_throttle.pin_number, 0, 0)
    print('STEERING: throttle PWM output is disabled (no ESC pulse).', flush=True)

    steering = PulseController(
        pwm_pin=CheckedPwmPin(pins.pwm_pin_by_id(settings['PWM_STEERING_PIN'])),
        pwm_scale=settings['PWM_STEERING_SCALE'],
        pwm_inverted=settings['PWM_STEERING_INVERTED'])
    print('Steering configuration: left={} center={} right={}'.format(
        settings['STEERING_LEFT_PWM'],
        settings['STEERING_CENTER_PWM'],
        settings['STEERING_RIGHT_PWM']), flush=True)
    return SteeringOnlyDrive(steering, settings)


class DriveGuard:
    """Expire commands in a separate thread while capture/inference is blocked."""
    def __init__(self, drive, timeout=0.75):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Watchdog timeout must be positive and finite')
        self.drive = drive
        self.timeout = timeout
        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.closed = False
        self.fault = None
        self.expired = False
        self.deadline = time.monotonic() + timeout
        drive.stop()
        self.thread = threading.Thread(target=self._watch, daemon=True)
        self.thread.start()

    def _record_failure(self, error):
        self.fault = error
        try:
            self.drive.stop()
        except Exception:
            pass  # Preserve the original failure; caller will exit.

    def raise_if_faulted(self):
        if self.fault is not None:
            raise RuntimeError('Motor output failed; restart only after inspection') from self.fault

    def apply(self, steering, throttle, observed_at):
        with self.lock:
            self.raise_if_faulted()
            if self.closed:
                raise RuntimeError('Motor output is closed')
            now = time.monotonic()
            try:
                if (self.expired or not math.isfinite(observed_at)
                        or observed_at > now or now - observed_at >= self.timeout):
                    self.drive.stop()
                    self.expired = False
                    self.deadline = now + self.timeout
                    return False
                self.drive.write(steering, throttle)
                self.deadline = observed_at + self.timeout
                return True
            except Exception as error:
                self._record_failure(error)
                self.raise_if_faulted()

    def _watch(self):
        interval = min(0.05, self.timeout / 4)
        while not self.shutdown_event.wait(interval):
            with self.lock:
                if self.closed or self.fault is not None:
                    return
                if not self.expired and time.monotonic() >= self.deadline:
                    try:
                        self.expired = True
                        self.drive.stop()
                    except Exception as error:
                        self._record_failure(error)
                        return

    def close(self):
        self.shutdown_event.set()
        try:
            with self.lock:
                if not self.closed:
                    self.closed = True
                    self.drive.stop()
        finally:
            self.thread.join(timeout=1)
