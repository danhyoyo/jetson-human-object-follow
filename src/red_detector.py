"""Fast NumPy-only detector for one dominant red object."""

from collections import deque, namedtuple

import numpy as np


RedDetection = namedtuple('RedDetection',
                          'left top right bottom area_fraction')


def detect_largest_red(rgb, min_area_fraction=0.002, block_size=4):
    """Return the largest connected red region as normalized coordinates.

    The color rule uses both an absolute red level and red/green-blue
    dominance, which is more tolerant of lighting than a single RGB cutoff.
    Connected components are found on a small block mask for Jetson Nano speed.
    """
    pixels = np.asarray(rgb)
    if pixels.ndim != 3 or pixels.shape[2] < 3:
        raise ValueError('Expected an RGB/RGBA image')
    if not 0 < min_area_fraction <= 0.25:
        raise ValueError('min_area_fraction must be in (0, 0.25]')
    if block_size < 1:
        raise ValueError('block_size must be positive')
    colors = pixels[:, :, :3].astype(np.int16, copy=False)
    red, green, blue = colors[:, :, 0], colors[:, :, 1], colors[:, :, 2]
    mask = ((red > 85)
            & (red - green > 40)
            & (red - blue > 40)
            & (red * 10 >= green * 14)
            & (red * 10 >= blue * 14))

    height, width = mask.shape
    coarse_h, coarse_w = height // block_size, width // block_size
    if coarse_h == 0 or coarse_w == 0:
        return None
    trimmed = mask[:coarse_h * block_size, :coarse_w * block_size]
    coarse = (trimmed.reshape(coarse_h, block_size, coarse_w, block_size)
              .mean(axis=(1, 3)) >= 0.20)

    visited = np.zeros_like(coarse, dtype=np.bool_)
    best = None
    for start_y, start_x in zip(*np.nonzero(coarse)):
        if visited[start_y, start_x]:
            continue
        queue = deque([(int(start_y), int(start_x))])
        visited[start_y, start_x] = True
        min_y = max_y = int(start_y)
        min_x = max_x = int(start_x)
        cells = 0
        while queue:
            y, x = queue.popleft()
            cells += 1
            min_y, max_y = min(min_y, y), max(max_y, y)
            min_x, max_x = min(min_x, x), max(max_x, x)
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = y + dy, x + dx
                    if (0 <= ny < coarse_h and 0 <= nx < coarse_w
                            and coarse[ny, nx] and not visited[ny, nx]):
                        visited[ny, nx] = True
                        queue.append((ny, nx))
        if best is None or cells > best[0]:
            best = (cells, min_y, min_x, max_y, max_x)

    if best is None:
        return None
    _, coarse_top, coarse_left, coarse_bottom, coarse_right = best
    roi_top = coarse_top * block_size
    roi_left = coarse_left * block_size
    roi_bottom = min(height, (coarse_bottom + 1) * block_size)
    roi_right = min(width, (coarse_right + 1) * block_size)
    ys, xs = np.nonzero(mask[roi_top:roi_bottom, roi_left:roi_right])
    if len(xs) < height * width * min_area_fraction:
        return None

    left = roi_left + int(xs.min())
    right = roi_left + int(xs.max()) + 1
    top = roi_top + int(ys.min())
    bottom = roi_top + int(ys.max()) + 1
    return RedDetection(left / width, top / height, right / width,
                        bottom / height, len(xs) / float(height * width))


def draw_detection(rgb, detection, color=(0, 255, 0), thickness=3):
    """Draw a box in-place around a normalized RedDetection."""
    if detection is None:
        return rgb
    height, width = rgb.shape[:2]
    left = max(0, min(width - 1, int(detection.left * width)))
    right = max(left + 1, min(width, int(detection.right * width)))
    top = max(0, min(height - 1, int(detection.top * height)))
    bottom = max(top + 1, min(height, int(detection.bottom * height)))
    size = max(1, min(thickness, right - left, bottom - top))
    rgb[top:top + size, left:right, :3] = color
    rgb[bottom - size:bottom, left:right, :3] = color
    rgb[top:bottom, left:left + size, :3] = color
    rgb[top:bottom, right - size:right, :3] = color
    return rgb
