#!/usr/bin/env bash
set -euo pipefail

# Convenience root forwarder for scripts/setup_jetson.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$SCRIPT_DIR/scripts/setup_jetson.sh" "$@"
