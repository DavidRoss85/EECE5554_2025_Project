# Quick Start Guide - Raspberry Pi 5

## Complete Setup (Copy & Paste)

```bash
# 1. Install build dependencies
sudo apt update
sudo apt install -y make build-essential libssl-dev zlib1g-dev \
  libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm \
  libncurses5-dev libncursesw5-dev xz-utils tk-dev libffi-dev \
  liblzma-dev python3-openssl git

# 2. Install pyenv
curl https://pyenv.run | bash

# 3. Add pyenv to shell (adds to ~/.bashrc)
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo '[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init - bash)"' >> ~/.bashrc

# 4. Reload shell configuration
source ~/.bashrc

# 5. Install Python 3.12.0 (takes 10-15 minutes)
pyenv install 3.12.0

# 6. Navigate to project and set Python version
cd /home/jx/Dev/Robotics/pincherx100/gesture_control
pyenv local 3.12.0

# 7. Verify Python version
python --version  # Should show: Python 3.12.0

# 8. Run setup script
./setup.sh

# 9. Activate and run
source venv/bin/activate
python gesture_control.py
```

## What Each Step Does

1. **Build dependencies**: Libraries needed to compile Python from source
2. **pyenv**: Tool to manage multiple Python versions
3. **Shell config**: Makes pyenv available in your terminal
4. **Reload**: Applies the new configuration
5. **Install Python**: Compiles Python 3.12.0 (MediaPipe compatible)
6. **Set local version**: Tells this project to use Python 3.12
7. **Verify**: Confirms correct Python version
8. **Setup**: Creates virtual environment and installs packages
9. **Run**: Starts the gesture control application

## After Setup

Every time you want to use the gesture control:

```bash
cd /home/jx/Dev/Robotics/pincherx100/gesture_control
source venv/bin/activate
python gesture_control.py
```

Press `q` to quit the application.

## Troubleshooting

**Problem**: `pyenv: command not found`  
**Solution**: Run `source ~/.bashrc` or open a new terminal

**Problem**: Python 3.12 installation fails  
**Solution**: Make sure all Step 1 packages installed successfully

**Problem**: MediaPipe installation fails  
**Solution**: Verify `python --version` shows 3.12.x (not 3.13)

**Problem**: Setup script can't find Python 3.12  
**Solution**: Make sure you ran `pyenv local 3.12.0` in the gesture_control directory

For more details, see `INSTALL.md`.

