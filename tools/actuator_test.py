#!/usr/bin/env python3
"""Short, explicit PCA9685 actuator tests.  Dry-run unless --apply is given."""

import argparse
import math
import signal
import sys
import time
from pathlib import Path

# Add src and project root to sys.path
PROJECT_DIR = Path(__file__).resolve().parent.parent
SRC_DIR = PROJECT_DIR / 'src'
for p in (str(SRC_DIR), str(PROJECT_DIR)):
    if p not in sys.path:
        sys.path.insert(0, p)


def finite_in_range(value, low, high, name):
    if not math.isfinite(value) or not low <= value <= high:
        raise ValueError('{} must be finite and in [{}, {}]'.format(name, low, high))
    return value


def steering_pulse(value, left_pwm, center_pwm, right_pwm):
    finite_in_range(value, -1.0, 1.0, 'steering value')
    finite_in_range(left_pwm, 1, 4094, 'left PWM')
    finite_in_range(center_pwm, 1, 4094, 'center PWM')
    finite_in_range(right_pwm, 1, 4094, 'right PWM')
    if left_pwm == right_pwm:
        raise ValueError('left and right PWM must differ')
    if not min(left_pwm, right_pwm) < center_pwm < max(left_pwm, right_pwm):
        raise ValueError('center PWM must be strictly between endpoints')
    endpoint = left_pwm if value < 0 else right_pwm
    return int(center_pwm + abs(value) * (endpoint - center_pwm))


def throttle_pulse(speed, neutral_pwm, forward_start_pwm, forward_pwm):
    finite_in_range(speed, 0.0, 0.3, 'throttle speed')
    finite_in_range(neutral_pwm, 1, 4094, 'neutral PWM')
    finite_in_range(forward_start_pwm, 1, 4094, 'forward-start PWM')
    finite_in_range(forward_pwm, 1, 4094, 'forward PWM')
    if not neutral_pwm < forward_start_pwm <= forward_pwm:
        raise ValueError('expected neutral < forward-start <= forward PWM')
    if speed == 0:
        return int(neutral_pwm)
    return int(forward_start_pwm + speed / 0.3 * (forward_pwm - forward_start_pwm))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Timed PCA9685 steering/throttle test; dry-run by default.')
    default_car_dir = (PROJECT_DIR / 'config' / 'car'
                       if (PROJECT_DIR / 'config' / 'car').exists()
                       else PROJECT_DIR / 'car')
    parser.add_argument('--donkey-dir',
                        default=str(default_car_dir),
                        help='Directory containing config.py/myconfig.py '
                             '(default: bundled car directory)')
    parser.add_argument('--apply', action='store_true',
                        help='Actually write PWM. Without this flag, only print the plan.')
    parser.add_argument('--duration', type=float, default=1.0,
                        help='Seconds to hold the test output, 0.1..5 (default: 1)')
    sub = parser.add_subparsers(dest='command')

    steering = sub.add_parser('steering', help='Test a normalized steering position')
    steering.add_argument('--value', type=float, required=True,
                          help='-1=left endpoint, 0=center, +1=right endpoint')
    steering.add_argument('--pin', help='PCA9685 pin id; defaults to configured steering pin')
    steering.add_argument('--left-pwm', type=float,
                          help='Override configured left endpoint')
    steering.add_argument('--center-pwm', type=float,
                          help='Override configured center')
    steering.add_argument('--right-pwm', type=float,
                          help='Override configured right endpoint')
    steering.add_argument('--return-center', action='store_true',
                          help='Return to center before disabling PWM (off by default)')

    throttle = sub.add_parser('throttle', help='Test one explicit throttle output')
    throttle.add_argument('--pin', help='PCA9685 pin id; defaults to configured throttle pin')
    choice = throttle.add_mutually_exclusive_group(required=True)
    choice.add_argument('--pulse', type=float,
                        help='Raw PWM pulse to test (recommended for finding neutral)')
    choice.add_argument('--speed', type=float,
                        help='Normalized forward speed 0..0.3')
    throttle.add_argument('--neutral-pwm', type=float,
                          help='Override configured neutral; sent before/after the test')
    throttle.add_argument('--start-pwm', type=float,
                          help='Override configured first-moving forward pulse')
    throttle.add_argument('--forward-pwm', type=float,
                          help='Override configured safe maximum forward pulse')
    throttle.add_argument('--arm-seconds', type=float, default=1.0,
                          help='Neutral arming time for --speed, 0..5 (default: 1)')

    args = parser.parse_args(argv)
    if args.command is None:
        parser.error('choose steering or throttle')
    try:
        finite_in_range(args.duration, 0.1, 5.0, 'duration')
        if args.command == 'steering':
            finite_in_range(args.value, -1.0, 1.0, 'steering value')
        else:
            if args.pulse is not None:
                finite_in_range(args.pulse, 1, 4094, 'pulse')
            else:
                for value, name in ((args.neutral_pwm, 'neutral PWM'),
                                    (args.start_pwm, 'forward-start PWM'),
                                    (args.forward_pwm, 'forward PWM')):
                    if value is not None:
                        finite_in_range(value, 1, 4094, name)
                finite_in_range(args.arm_seconds, 0.0, 5.0, 'arm-seconds')
    except ValueError as error:
        parser.error(str(error))
    return args


def load_config(car_dir):
    import donkeycar as dk

    directory = Path(car_dir).expanduser().resolve()
    if not (directory / 'config.py').is_file() or not (directory / 'myconfig.py').is_file():
        raise ValueError('--donkey-dir must contain config.py and myconfig.py')
    cfg = dk.load_config(config_path=str(directory / 'config.py'), myconfig='myconfig.py')
    if getattr(cfg, 'DONKEY_GYM', False):
        raise ValueError('Simulator configuration cannot drive hardware')
    settings = dict(cfg.PWM_STEERING_THROTTLE)
    return settings


def test_plan(args, settings):
    if args.command == 'steering':
        pin = args.pin or settings['PWM_STEERING_PIN']
        left = args.left_pwm if args.left_pwm is not None else settings['STEERING_LEFT_PWM']
        center = (args.center_pwm if args.center_pwm is not None
                  else settings['STEERING_CENTER_PWM'])
        right = args.right_pwm if args.right_pwm is not None else settings['STEERING_RIGHT_PWM']
        pulse = steering_pulse(args.value, left, center, right)
        return {
            'pin': pin,
            'pulse': pulse,
            'safe_pulse': int(center) if args.return_center else None,
            'scale': settings['PWM_STEERING_SCALE'],
            'inverted': settings['PWM_STEERING_INVERTED'],
            'description': 'steering value {:+.3f} (left={}, center={}, right={})'.format(
                args.value, int(left), int(center), int(right)),
        }

    pin = args.pin or settings['PWM_THROTTLE_PIN']
    neutral = (args.neutral_pwm if args.neutral_pwm is not None
               else settings['THROTTLE_STOPPED_PWM'])
    forward_start = (args.start_pwm if args.start_pwm is not None
                     else settings['THROTTLE_FORWARD_START_PWM'])
    forward = (args.forward_pwm if args.forward_pwm is not None
               else settings['THROTTLE_FORWARD_PWM'])
    if args.pulse is not None:
        finite_in_range(args.pulse, neutral, forward, 'raw throttle pulse')
        pulse = int(args.pulse)
        safe_pulse = int(neutral)
        description = 'raw throttle pulse {} (locked to {}..{})'.format(
            pulse, int(neutral), int(forward))
    else:
        pulse = throttle_pulse(args.speed, neutral, forward_start, forward)
        safe_pulse = int(neutral)
        description = ('throttle speed {:.3f} '
                       '(neutral={}, start={}, forward={})').format(
                           args.speed, int(neutral), int(forward_start), int(forward))
    return {
        'pin': pin,
        'pulse': pulse,
        'safe_pulse': safe_pulse,
        'scale': settings['PWM_THROTTLE_SCALE'],
        'inverted': settings['PWM_THROTTLE_INVERTED'],
        'description': description,
    }


def apply_plan(args, plan):
    from donkeycar.parts import pins
    from donkeycar.parts.actuator import PulseController
    from donkey_drive import CheckedPwmPin

    raw_pin = pins.pwm_pin_by_id(plan['pin'])
    controller = PulseController(
        pwm_pin=CheckedPwmPin(raw_pin),
        pwm_scale=plan['scale'],
        pwm_inverted=plan['inverted'])

    interrupted = [False]

    def request_stop(signum, frame):
        interrupted[0] = True
        raise KeyboardInterrupt

    previous_sigint = signal.signal(signal.SIGINT, request_stop)
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        if args.command == 'throttle' and args.speed is not None:
            controller.set_pulse(plan['safe_pulse'])
            time.sleep(args.arm_seconds)
        controller.set_pulse(plan['pulse'])
        time.sleep(args.duration)
    finally:
        try:
            if plan['safe_pulse'] is not None:
                controller.set_pulse(plan['safe_pulse'])
                time.sleep(0.25)
        finally:
            # Constant-low output: do not leave a stale servo/ESC pulse behind.
            raw_pin.pca9685.pwm.set_pwm(raw_pin.pin_number, 0, 0)
            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)
    return not interrupted[0]


def main():
    args = parse_args()
    settings = load_config(args.donkey_dir)
    try:
        plan = test_plan(args, settings)
    except ValueError as error:
        raise SystemExit('REFUSED: {}'.format(error))
    print('Mode: {}'.format(args.command))
    print('Pin: {}'.format(plan['pin']))
    print('Command: {}'.format(plan['description']))
    print('PWM pulse: {}'.format(plan['pulse']))
    print('Duration: {:.2f}s'.format(args.duration))
    if not args.apply:
        print('DRY-RUN: no PWM was written. Add --apply before the subcommand to execute.')
        return
    print('APPLYING PWM -- keep wheels raised and be ready to cut power.', flush=True)
    try:
        apply_plan(args, plan)
    except KeyboardInterrupt:
        print('\nInterrupted; output disabled.', flush=True)
    else:
        print('Test complete; output disabled.', flush=True)


if __name__ == '__main__':
    main()
