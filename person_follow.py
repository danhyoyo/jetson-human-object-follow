#!/usr/bin/env python3
"""One-person following; preview by default, DonkeyCar PWM output when enabled."""

import argparse
import math
import os
import signal
import sys
import time
from pathlib import Path

from follow_control import Box, FollowController
from red_detector import detect_largest_red, draw_detection


PROJECT_DIR = Path(__file__).resolve().parent


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='One-person following: preview by default (Python 3.6+).')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--drive', action='store_true', help='Enable steering and forward throttle')
    mode.add_argument('--steering-test', action='store_true',
                      help='Enable steering; keep throttle at neutral')
    parser.add_argument('--donkey-dir', default=str(PROJECT_DIR / 'car'),
                        help='Directory containing config.py/myconfig.py '
                             '(default: bundled car directory)')
    parser.add_argument('--invert-steering', action='store_true',
                        help='Reverse steering sign if the lifted-wheel test shows the wrong direction')
    parser.add_argument('--headless', action='store_true',
                        help='Only print commands, without the HDMI preview')
    parser.add_argument('--camera', default='csi://0', help='Camera URI')
    parser.add_argument('--input-flip', default='none',
                        choices=('none', 'counterclockwise', 'rotate-180', 'clockwise',
                                 'horizontal', 'vertical', 'upper-right-diagonal',
                                 'upper-left-diagonal'),
                        help='Low-level camera flip (JetPack 4 CSI default: none)')
    parser.add_argument('--no-software-rotate', action='store_true',
                        help='Disable the default 180-degree frame correction')
    parser.add_argument('--output', default='display://0',
                        help='Preview output URI (default: display://0)')
    parser.add_argument('--threshold', type=float, default=0.6)
    parser.add_argument('--red-object', action='store_true',
                        help='Track the largest red object without an AI model')
    parser.add_argument('--red-min-area', type=float, default=0.002,
                        help='Minimum red image-area fraction (default: 0.002)')
    parser.add_argument('--speed', type=float, default=None,
                        help='Normalized throttle, 0..0.3; required with --drive')
    parser.add_argument('--target-height', type=float, default=None,
                        help='Stop at this box-height fraction (person: 0.55, red: 0.50)')
    parser.add_argument('--ignore-distance', action='store_true',
                        help='Bench-test mode: follow one person at any box size/distance')
    parser.add_argument('--steering-gain', type=float, default=1.0,
                        help='Horizontal error multiplier (default: 1.0)')
    parser.add_argument('--steering-limit', type=float, default=0.70,
                        help='Maximum absolute steering command, 0..1 (default: 0.70)')
    parser.add_argument('--max-frame-age', type=float, default=0.75,
                        help='Reject capture/inference cycles exceeding this many seconds')
    args = parser.parse_args(argv)
    if args.drive and (args.speed is None or args.speed <= 0):
        parser.error('--drive requires an explicit positive --speed')
    if args.steering_test and args.speed not in (None, 0):
        parser.error('--steering-test requires zero throttle; omit --speed')
    if args.speed is None:
        args.speed = 0.0 if args.steering_test else 0.12
    if args.target_height is None:
        args.target_height = 0.50 if args.red_object else 0.55
    if not math.isfinite(args.threshold) or not 0 < args.threshold <= 1:
        parser.error('--threshold must be between 0 (exclusive) and 1')
    if not math.isfinite(args.max_frame_age) or args.max_frame_age <= 0:
        parser.error('--max-frame-age must be positive and finite')
    if not math.isfinite(args.red_min_area) or not 0 < args.red_min_area <= 0.25:
        parser.error('--red-min-area must be in (0, 0.25]')
    try:
        FollowController(args.speed, args.target_height, args.ignore_distance,
                         args.steering_gain, args.steering_limit,
                         allow_partial=args.red_object,
                         distance_hysteresis=0.0 if args.red_object else 0.05)
    except ValueError as error:
        parser.error(str(error))
    return args


def main():
    args = parse_args()
    # Delayed imports allow --help and controller tests on a normal computer.
    import numpy as np
    from jetson_utils import cudaFont, cudaFromNumpy, cudaToNumpy, videoSource, videoOutput

    control = FollowController(args.speed, args.target_height, args.ignore_distance,
                               args.steering_gain, args.steering_limit,
                               allow_partial=args.red_object,
                               distance_hysteresis=0.0 if args.red_object else 0.05)
    net = None
    person_ids = set()
    if not args.red_object:
        from jetson_inference import detectNet
        net = detectNet('ssd-mobilenet-v2', threshold=args.threshold)
        person_ids = {i for i in range(net.GetNumClasses())
                      if net.GetClassDesc(i).strip().lower() == 'person'}
        if not person_ids:
            raise RuntimeError('Model label list does not contain person')
    camera = None
    output = None
    font = None
    font_failed = False
    drive_output = None
    guard = None
    mode = 'DRIVE' if args.drive else ('STEERING' if args.steering_test else 'PREVIEW')
    target_name = 'RED' if args.red_object else 'PERSON'

    def request_stop(signum, frame):
        raise KeyboardInterrupt

    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        camera = videoSource(args.camera, argv=[
            # JetPack 4's gstCamera swaps NONE and ROTATE_180 for CSI sources.
            # Its flip-method=0 zero-copy path crashes on this installation, so
            # keep the stable default pipeline and correct orientation after
            # capture, before inference and rendering.
            'person_follow.py', '--input-width=640', '--input-height=480', '--input-rate=30',
            '--input-flip={}'.format(args.input_flip)])
        camera.Open()
        if not args.headless:
            # An SSH shell often has no DISPLAY even though the Jetson has an
            # HDMI desktop.  display://0 should target the physical screen.
            if args.output.startswith('display://'):
                os.environ.setdefault('DISPLAY', ':0')
            output = videoOutput(args.output)
            output.Open()
            font = cudaFont()
        if args.drive or args.steering_test:
            from donkey_drive import DriveGuard, build_donkey_drive, build_steering_drive
            drive_output = (build_donkey_drive(args.donkey_dir) if args.drive
                            else build_steering_drive(args.donkey_dir))
            guard = DriveGuard(drive_output, timeout=args.max_frame_age)
            print('{}: motor output enabled; Ctrl+C stops the program.'.format(mode), flush=True)
            if args.ignore_distance:
                print('WARNING: ignore-distance is active; distance and image-edge stops are disabled.',
                      flush=True)
        else:
            print('PREVIEW: detection video -> {}; no motor hardware is opened.'.format(
                args.output), flush=True)
        last_report = time.monotonic()
        frame_count = 0
        while camera.IsStreaming() and (output is None or output.IsStreaming()):
            if guard is not None:
                guard.raise_if_faulted()
            started = time.monotonic()
            image = camera.Capture()
            boxes = []
            if image is None:
                command = control.stop('NO_FRAME')
            else:
                pixels = cudaToNumpy(image)
                if not args.no_software_rotate:
                    pixels = np.ascontiguousarray(pixels[::-1, ::-1])
                    image = cudaFromNumpy(pixels)
                if args.red_object:
                    # Work on a copy so the detector box can be drawn before
                    # uploading the RGB frame back to CUDA for display.
                    pixels = np.ascontiguousarray(pixels)
                    red = detect_largest_red(pixels, args.red_min_area)
                    if red is not None:
                        boxes = [Box(red.left, red.top, red.right, red.bottom)]
                        draw_detection(pixels, red)
                    image = cudaFromNumpy(pixels)
                else:
                    detections = net.Detect(image, overlay='box,labels,conf')
                    boxes = [Box(d.Left / image.width, d.Top / image.height,
                                 d.Right / image.width, d.Bottom / image.height)
                             for d in detections if d.ClassID in person_ids]
                if time.monotonic() - started > args.max_frame_age:
                    command = control.stop('SLOW_FRAME')
                else:
                    command = control.update(boxes)
                frame_count += 1
            if args.invert_steering:
                command = command._replace(steering=-command.steering)
            if guard is not None and not guard.apply(command.steering, command.throttle, started):
                command = control.stop('WATCHDOG_RESET')
            # Apply commands before rendering. The watchdog also covers blocked rendering.
            if image is not None and output is not None:
                if font is not None:
                    status = '{} {} | {} | targets={} | steer={:+.2f} | throttle={:.2f}'.format(
                        mode, target_name, command.state, len(boxes),
                        command.steering, command.throttle)
                    try:
                        font.OverlayText(image, image.width, image.height, status,
                                         8, 8, font.White, font.Gray40)
                    except Exception as error:
                        if not font_failed:
                            print('On-video status overlay disabled: {}'.format(error),
                                  file=sys.stderr, flush=True)
                            font_failed = True
                        font = None
                output.Render(image)
                output.SetStatus('{} | {} | steer={:+.2f} throttle={:.2f}'.format(
                    mode, command.state, command.steering, command.throttle))
            now = time.monotonic()
            if now - last_report >= 0.5:
                print('{} {} | {} | targets={} | error={:+.2f} height={:.2f} | '
                      'steer={:+.2f} throttle={:.2f} | loop={:.1f} FPS'.format(
                          mode, target_name, command.state, len(boxes),
                          command.x_error, command.height,
                          command.steering, command.throttle,
                          frame_count / (now - last_report)), flush=True)
                last_report, frame_count = now, 0
    except KeyboardInterrupt:
        print('\nStopped.', flush=True)
    finally:
        # Neutral comes before camera/display cleanup, even on inference errors.
        if guard is not None or drive_output is not None:
            try:
                guard.close() if guard is not None else drive_output.stop()
            except Exception as error:
                print('Failed to send neutral motor output: {}'.format(error), file=sys.stderr)
        for stream in (output, camera):
            if stream is not None:
                try:
                    stream.Close()
                except Exception as error:
                    print('Failed to close video stream: {}'.format(error), file=sys.stderr)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == '__main__':
    main()
