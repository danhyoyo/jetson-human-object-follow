"""Hardware-independent controller for a one-person demo (Python 3.6+)."""

import math
from collections import namedtuple


Box = namedtuple('Box', 'left top right bottom')
Command = namedtuple('Command', 'state steering throttle x_error height')


def overlap(a, b):
    width = max(0, min(a.right, b.right) - max(a.left, b.left))
    height = max(0, min(a.bottom, b.bottom) - max(a.top, b.top))
    intersection = width * height
    union = ((a.right - a.left) * (a.bottom - a.top)
             + (b.right - b.left) * (b.bottom - b.top) - intersection)
    return intersection / union if union > 0 else 0


class FollowController:
    def __init__(self, speed=0.12, target_height=0.55, ignore_distance=False,
                 steering_gain=1.0, steering_limit=0.70, allow_partial=False,
                 distance_hysteresis=0.05):
        if not math.isfinite(speed) or not 0 <= speed <= 0.3:
            raise ValueError('speed must be finite and between 0 and 0.3')
        if not math.isfinite(target_height) or not 0.2 <= target_height <= 0.85:
            raise ValueError('target_height must be between 0.2 and 0.85')
        if not isinstance(ignore_distance, bool):
            raise ValueError('ignore_distance must be a boolean')
        if not isinstance(allow_partial, bool):
            raise ValueError('allow_partial must be a boolean')
        if not math.isfinite(steering_gain) or not 0 < steering_gain <= 2:
            raise ValueError('steering_gain must be finite and in (0, 2]')
        if not math.isfinite(steering_limit) or not 0 < steering_limit <= 1:
            raise ValueError('steering_limit must be finite and in (0, 1]')
        if (not math.isfinite(distance_hysteresis)
                or not 0 <= distance_hysteresis < target_height):
            raise ValueError('distance_hysteresis must be finite and non-negative')
        self.speed = speed
        self.target_height = target_height
        self.ignore_distance = ignore_distance
        self.steering_gain = steering_gain
        self.steering_limit = steering_limit
        self.allow_partial = allow_partial
        self.distance_hysteresis = distance_hysteresis
        self.reset()

    def reset(self):
        self.previous = None
        self.stable_frames = 0
        self.steering = 0.0
        self.moving = False

    def stop(self, state):
        self.reset()
        return Command(state, 0.0, 0.0, 0.0, 0.0)

    def update(self, boxes):
        """Accept normalized person boxes; negative steering means left."""
        if not boxes:
            return self.stop('NO_PERSON')
        if len(boxes) != 1:
            return self.stop('MULTIPLE_PEOPLE')
        box = boxes[0]
        if (not all(math.isfinite(v) for v in box)
                or not 0 <= box.left < box.right <= 1
                or not 0 <= box.top < box.bottom <= 1):
            return self.stop('INVALID_TARGET')
        if (not self.ignore_distance and not self.allow_partial
                and (box.left <= 0.01 or box.top <= 0.01
                     or box.right >= 0.99 or box.bottom >= 0.99)):
            return self.stop('PARTIAL_TARGET')
        height = box.bottom - box.top
        if not self.ignore_distance and not self.allow_partial and height < 0.08:
            return self.stop('TOO_SMALL')

        x_error = box.left + box.right - 1.0
        if self.previous is None or overlap(self.previous, box) < 0.1:
            self.stable_frames = 1
            self.steering = 0.0
            self.moving = False
        else:
            self.stable_frames += 1
        self.previous = box
        if self.stable_frames < 3:
            return Command('ACQUIRING', 0.0, 0.0, x_error, height)

        # Ignore-distance is an explicit bench-test mode.  The normal mode uses
        # two thresholds to prevent flickering around the stop position.
        if self.ignore_distance:
            self.moving = True
        elif height >= self.target_height or abs(x_error) > 0.75:
            self.moving = False
        elif height < self.target_height - self.distance_hysteresis:
            self.moving = True
        if not self.moving:
            self.steering = 0.0
            return Command('HOLD', 0.0, 0.0, x_error, height)

        desired = 0.0 if abs(x_error) < 0.08 else self.steering_gain * x_error
        desired = max(-self.steering_limit, min(self.steering_limit, desired))
        self.steering = 0.7 * self.steering + 0.3 * desired
        state = 'FOLLOW_ANY_DISTANCE' if self.ignore_distance else 'FOLLOW'
        return Command(state, self.steering, self.speed, x_error, height)
