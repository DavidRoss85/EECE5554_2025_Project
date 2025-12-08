#!/bin/bash

# -----------------------------
# AUTOMATIC ENV + BUILD SCRIPT
# -----------------------------

# Get absolute path of the folder containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
MAIN_WS="$PROJECT_ROOT/main_ws"
VENV_DIR="$PROJECT_ROOT/venv"

echo "[INFO] Script directory: $SCRIPT_DIR"
echo "[INFO] Project root: $PROJECT_ROOT"
echo "[INFO] Workspace: $MAIN_WS"

# Sanity check
if [ ! -d "$MAIN_WS" ]; then
    echo "[ERROR] main_ws not found at $MAIN_WS"
    exit 1
fi

# -------------------------------------
# 1. Create or update virtual environment
# -------------------------------------
if [ ! -d "$VENV_DIR" ]; then
    echo "[INFO] Creating virtual environment at $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
else
    echo "[INFO] Virtual environment already exists."
fi

echo "[INFO] Upgrading pip inside venv..."
"$VENV_DIR/bin/pip" install --upgrade pip

# -------------------------------------
# 2. Install requirements.txt if present
# -------------------------------------
REQ_FILE="$MAIN_WS/requirements.txt"

if [ -f "$REQ_FILE" ]; then
    echo "[INFO] Installing requirements.txt packages..."
    "$VENV_DIR/bin/pip" install -r "$REQ_FILE"
else
    echo "[WARNING] requirements.txt not found at $REQ_FILE"
fi

# -------------------------------------
# 3. Install additional Python dependencies (venv only)
# -------------------------------------
echo "[INFO] Installing additional Python packages (typeguard, ultralytics, opencv-python, scipy, message-filters)..."

"$VENV_DIR/bin/pip" install \
    typeguard \
    ultralytics \
    opencv-python \
    scipy \
    message-filters

# -------------------------------------
# 4. Launch gnome-terminal to run colcon build (WITHOUT VENV)
# -------------------------------------
echo "[INFO] Opening gnome-terminal for colcon build..."

gnome-terminal -- bash -c "
    echo \"[TERMINAL] Sourcing ROS...\";
    source /opt/ros/\$ROS_DISTRO/setup.bash;

    echo \"[TERMINAL] Entering workspace...\";
    cd \"$MAIN_WS\";

    echo \"[TERMINAL] Running colcon build...\";
    colcon build;

    echo \"[TERMINAL] Build complete.\";
    exec bash
"

echo "[INFO] Environment setup complete!"
