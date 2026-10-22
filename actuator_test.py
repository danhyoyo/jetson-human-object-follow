#!/usr/bin/env python3
"""Convenience root launcher for tools/actuator_test.py."""
import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent
TOOLS_DIR = PROJECT_DIR / 'tools'
SRC_DIR = PROJECT_DIR / 'src'

for path in (str(TOOLS_DIR), str(SRC_DIR), str(PROJECT_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from tools.actuator_test import (
    main,
    steering_pulse,
    throttle_pulse,
    parse_args,
    load_config,
    test_plan,
    finite_in_range,
    apply_plan,
)

if __name__ == '__main__':
    main()
