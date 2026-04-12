#!/bin/bash
# Reset device to UF2 bootloader mode via serial
# Creates a venv and installs pyserial automatically

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

if [ -z "$1" ]; then
    echo "Usage: ./reset.sh /dev/tty.usbmodemXXXX"
    exit 1
fi

if [ ! -d "$VENV_DIR" ]; then
    echo "[INFO] Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
    "$VENV_DIR/bin/pip" install --quiet pyserial
fi

"$VENV_DIR/bin/python3" "$SCRIPT_DIR/reset.py" "$1"
