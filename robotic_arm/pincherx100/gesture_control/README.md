# Gesture Control

Hand gesture control system for robot arm using MediaPipe and OpenCV.

## Requirements

- Python 3.8-3.12 (MediaPipe does not support Python 3.13)
- Webcam
- OpenCV, MediaPipe, NumPy

## Installation

### Option 1: Using pyenv (Recommended for Raspberry Pi)

If you only have Python 3.13 installed, you'll need to install a compatible version using pyenv:

```bash
# Install pyenv if not already installed
curl https://pyenv.run | bash

# Add to your ~/.bashrc or ~/.zshrc
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init -)"' >> ~/.bashrc
source ~/.bashrc

# Install Python 3.12
pyenv install 3.12.0

# Set it as local version for this project
cd gesture_control
pyenv local 3.12.0

# Now run setup
./setup.sh
```

### Option 2: Install Python 3.12 from Debian/Ubuntu repositories

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa  # For Ubuntu
sudo apt update
sudo apt install python3.12 python3.12-venv python3.12-dev

# Then modify setup.sh to use python3.12 explicitly
```

### Option 3: Use existing Python 3.11 or 3.12 (if available)

If you have Python 3.11 or 3.12 already installed, the setup script will automatically detect and use it.

## Usage

```bash
# Activate virtual environment
source venv/bin/activate

# Run the gesture control
python gesture_control.py
```

## Controls

- `q`: Quit
- `r`: Reset gesture state
- `+`/`-`: Adjust smoothing factor
- `t`/`g`: Adjust gripper grasping position

## Features

- Real-time hand tracking using MediaPipe
- Joint angle calculation from hand position
- Pinch detection for gripper control
- Smoothing algorithms to reduce jitter
- Visual feedback with hand landmarks and status indicators

