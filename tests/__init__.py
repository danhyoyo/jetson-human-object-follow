"""Test package initialization."""

import sys
from pathlib import Path

PROJECT_DIR = Path(__file__).resolve().parent.parent
for p in (str(PROJECT_DIR), str(PROJECT_DIR / 'src'), str(PROJECT_DIR / 'tools')):
    if p not in sys.path:
        sys.path.insert(0, p)
