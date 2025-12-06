# Installation Instructions for Raspberry Pi 5

## Step 1: Install Build Dependencies (if not already installed)

```bash
sudo apt update
sudo apt install -y make build-essential libssl-dev zlib1g-dev \
  libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm \
  libncurses5-dev libncursesw5-dev xz-utils tk-dev libffi-dev \
  liblzma-dev python3-openssl git
```

## Step 2: Install pyenv (if not already installed)

```bash
curl https://pyenv.run | bash
```

## Step 3: Add pyenv to Your Shell Configuration

Add these lines to your `~/.bashrc` file:

```bash
export PYENV_ROOT="$HOME/.pyenv"
[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init - bash)"
```

To add them automatically, run:

```bash
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo '[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init - bash)"' >> ~/.bashrc
```

Then reload your shell configuration:

```bash
source ~/.bashrc
```

Or simply open a new terminal window.

## Step 4: Install Python 3.12.0

```bash
pyenv install 3.12.0
```

**Note:** This will take 10-15 minutes to compile. Be patient!

## Step 5: Set Python 3.12 for the Gesture Control Project

Navigate to the gesture_control directory and set Python 3.12 as the local version:

```bash
cd /home/jx/Dev/Robotics/pincherx100/gesture_control
pyenv local 3.12.0
```

This creates a `.python-version` file that tells pyenv to use Python 3.12.0 for this directory.

## Step 6: Verify Python Version

```bash
python --version
```

You should see: `Python 3.12.0`

## Step 7: Run the Setup Script

```bash
./setup.sh
```

This will:
- Create a virtual environment using Python 3.12
- Install OpenCV, MediaPipe, and NumPy

## Step 8: Activate and Use

```bash
# Activate the virtual environment
source venv/bin/activate

# Run the gesture control script
python gesture_control.py
```

## Troubleshooting

### If pyenv command is not found after adding to .bashrc:
- Make sure you've run `source ~/.bashrc` or opened a new terminal
- Verify pyenv is installed: `ls -la ~/.pyenv`

### If Python 3.12 installation fails:
- Make sure all build dependencies from Step 1 are installed
- Check available disk space: `df -h`
- Try installing a different Python 3.12 version: `pyenv install 3.12.7`

### If MediaPipe still fails to install:
- Verify Python version: `python --version` (should be 3.12.x)
- Check pip version: `pip --version`
- Try upgrading pip: `pip install --upgrade pip`
- Try installing MediaPipe with specific version: `pip install mediapipe==0.10.8`

### To check which Python versions are available via pyenv:
```bash
pyenv versions
```

### To see which Python version is currently active:
```bash
pyenv version
```

## Quick Reference Commands

```bash
# Check Python version
python --version

# Check pyenv versions
pyenv versions

# Set local Python version for current directory
pyenv local 3.12.0

# Activate virtual environment
source venv/bin/activate

# Deactivate virtual environment
deactivate

# Run gesture control
python gesture_control.py
```

