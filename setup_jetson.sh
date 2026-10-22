#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-/home/jetson/.virtualenv/donkeycar/bin/python}"
MODEL_SOURCE="$PROJECT_DIR/models/SSD-Mobilenet-v2"
MODEL_DEST="/usr/local/bin/networks/SSD-Mobilenet-v2"

if [[ ! -x "$PYTHON_BIN" ]]; then
    echo "Python environment not found or not executable: $PYTHON_BIN" >&2
    echo "Set PYTHON_BIN to the Jetson Python that has DonkeyCar installed." >&2
    exit 1
fi

if [[ "${1:-}" == "--install-python-deps" ]]; then
    "$PYTHON_BIN" -m pip install -r "$PROJECT_DIR/requirements.txt"
fi

if [[ ! -f "$MODEL_SOURCE/ssd_mobilenet_v2_coco.uff" ||
      ! -f "$MODEL_SOURCE/ssd_coco_labels.txt" ]]; then
    echo "Bundled SSD-MobileNet-v2 model assets are missing." >&2
    exit 1
fi

EXPECTED_MODEL_SHA="bcc956cfbdcff28e7d6e52e6fea12e6892c3c183b313e0685206ad86800ff364"
ACTUAL_MODEL_SHA="$(sha256sum "$MODEL_SOURCE/ssd_mobilenet_v2_coco.uff" | awk '{print $1}')"
if [[ "$ACTUAL_MODEL_SHA" != "$EXPECTED_MODEL_SHA" ]]; then
    echo "Bundled SSD-MobileNet-v2 checksum mismatch." >&2
    exit 1
fi

if [[ ! -f "$MODEL_DEST/ssd_mobilenet_v2_coco.uff" ||
      ! -f "$MODEL_DEST/ssd_coco_labels.txt" ]]; then
    echo "Installing bundled SSD-MobileNet-v2 assets into $MODEL_DEST"
    if mkdir -p "$MODEL_DEST" 2>/dev/null &&
       cp "$MODEL_SOURCE/ssd_mobilenet_v2_coco.uff" "$MODEL_DEST/" 2>/dev/null &&
       cp "$MODEL_SOURCE/ssd_coco_labels.txt" "$MODEL_DEST/" 2>/dev/null; then
        :
    elif command -v sudo >/dev/null 2>&1; then
        sudo mkdir -p "$MODEL_DEST"
        sudo cp "$MODEL_SOURCE/ssd_mobilenet_v2_coco.uff" "$MODEL_DEST/"
        sudo cp "$MODEL_SOURCE/ssd_coco_labels.txt" "$MODEL_DEST/"
    else
        echo "Cannot install model into $MODEL_DEST (permission denied)." >&2
        exit 1
    fi
fi

"$PYTHON_BIN" - <<'PY'
import importlib

required = ('numpy', 'donkeycar', 'jetson_utils', 'jetson_inference')
missing = []
for name in required:
    try:
        importlib.import_module(name)
    except Exception as error:
        missing.append('{}: {}'.format(name, error))

if missing:
    raise SystemExit('Missing runtime dependencies:\n  ' + '\n  '.join(missing))
print('Runtime imports: OK')
PY

cd "$PROJECT_DIR"
PYTHONDONTWRITEBYTECODE=1 "$PYTHON_BIN" -m unittest -q \
    test_follow_control.py test_actuator_test.py

# Both commands below are dry-runs; they validate the bundled car config and
# print the planned PWM without opening PCA9685 output.
"$PYTHON_BIN" actuator_test.py steering --value 0
"$PYTHON_BIN" actuator_test.py throttle --speed 0.30

echo
echo "Project check complete. No actuator PWM was applied."
echo "Bundled SSD-MobileNet-v2 model: OK"
echo "Preview person mode: $PYTHON_BIN $PROJECT_DIR/person_follow.py"
echo "Preview red mode:    $PYTHON_BIN $PROJECT_DIR/person_follow.py --red-object"
