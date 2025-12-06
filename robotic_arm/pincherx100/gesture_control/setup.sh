#!/bin/bash

# Setup script for gesture control virtual environment

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Function to find a compatible Python version
find_python() {
    # First, check if pyenv is available and has a local version set
    if command -v pyenv &> /dev/null; then
        # Initialize pyenv if not already done
        export PYENV_ROOT="$HOME/.pyenv"
        [[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"
        eval "$(pyenv init - bash 2>/dev/null)" || true
        
        # Check if there's a local .python-version file
        if [ -f ".python-version" ]; then
            LOCAL_VERSION=$(cat .python-version | head -1)
            PYENV_PYTHON="$PYENV_ROOT/versions/$LOCAL_VERSION/bin/python"
            if [ -f "$PYENV_PYTHON" ]; then
                PYTHON_VERSION=$($PYENV_PYTHON --version 2>&1 | grep -oP '\d+\.\d+' | head -1)
                MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
                MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)
                
                if [ "$MAJOR" -eq 3 ] && [ "$MINOR" -ge 8 ] && [ "$MINOR" -le 12 ]; then
                    echo "$PYENV_PYTHON"
                    return 0
                fi
            fi
        fi
        
        # Check pyenv versions for compatible Python
        for pyenv_version in $(pyenv versions --bare 2>/dev/null); do
            PYENV_PYTHON="$PYENV_ROOT/versions/$pyenv_version/bin/python"
            if [ -f "$PYENV_PYTHON" ]; then
                PYTHON_VERSION=$($PYENV_PYTHON --version 2>&1 | grep -oP '\d+\.\d+' | head -1)
                MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
                MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)
                
                if [ "$MAJOR" -eq 3 ] && [ "$MINOR" -ge 8 ] && [ "$MINOR" -le 12 ]; then
                    echo "$PYENV_PYTHON"
                    return 0
                fi
            fi
        done
    fi
    
    # Try system Python versions in order of preference (MediaPipe supports 3.8-3.12)
    for version in python3.12 python3.11 python3.10 python3.9 python3.8 python3; do
        if command -v "$version" &> /dev/null; then
            PYTHON_VERSION=$($version --version 2>&1 | grep -oP '\d+\.\d+' | head -1)
            MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
            MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)
            
            # Check if version is 3.8-3.12 (MediaPipe compatible)
            if [ "$MAJOR" -eq 3 ] && [ "$MINOR" -ge 8 ] && [ "$MINOR" -le 12 ]; then
                echo "$version"
                return 0
            fi
        fi
    done
    return 1
}

# Find compatible Python version
PYTHON_CMD=$(find_python)

if [ -z "$PYTHON_CMD" ]; then
    echo "ERROR: No compatible Python version found (MediaPipe requires Python 3.8-3.12)"
    echo ""
    echo "Current Python version: $(python3 --version)"
    echo ""
    echo "Please install Python 3.12 or earlier. Options:"
    echo "  1. Install Python 3.12:"
    echo "     sudo apt update"
    echo "     sudo apt install python3.12 python3.12-venv"
    echo ""
    echo "  2. Use pyenv to manage Python versions (RECOMMENDED):"
    echo "     # Install pyenv if needed:"
    echo "     curl https://pyenv.run | bash"
    echo "     # Add to ~/.bashrc:"
    echo "     echo 'export PYENV_ROOT=\"\$HOME/.pyenv\"' >> ~/.bashrc"
    echo "     echo '[[ -d \$PYENV_ROOT/bin ]] && export PATH=\"\$PYENV_ROOT/bin:\$PATH\"' >> ~/.bashrc"
    echo "     echo 'eval \"\$(pyenv init - bash)\"' >> ~/.bashrc"
    echo "     source ~/.bashrc"
    echo "     # Install Python 3.12:"
    echo "     pyenv install 3.12.0"
    echo "     cd $(pwd)"
    echo "     pyenv local 3.12.0"
    echo ""
    echo "   See INSTALL.md for detailed instructions."
    echo ""
    exit 1
fi

PYTHON_VERSION=$($PYTHON_CMD --version)
echo "Using Python: $PYTHON_VERSION"

# Remove existing venv if it was created with wrong Python version
if [ -d "venv" ]; then
    VENV_PYTHON=$(venv/bin/python --version 2>&1 | grep -oP '\d+\.\d+' | head -1)
    VENV_MAJOR=$(echo $VENV_PYTHON | cut -d. -f1)
    VENV_MINOR=$(echo $VENV_PYTHON | cut -d. -f2)
    
    # If venv was created with Python 3.13, remove it
    if [ "$VENV_MAJOR" -eq 3 ] && [ "$VENV_MINOR" -eq 13 ]; then
        echo "Removing existing virtual environment (created with Python 3.13)..."
        rm -rf venv
    fi
fi

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
    echo "Creating virtual environment with $PYTHON_CMD..."
    $PYTHON_CMD -m venv venv
    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to create virtual environment"
        exit 1
    fi
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install required packages
echo "Installing required packages..."
pip install opencv-python>=4.8.0
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to install opencv-python"
    exit 1
fi

pip install mediapipe>=0.10.0
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to install mediapipe"
    echo "MediaPipe requires Python 3.8-3.12. Current version: $(python --version)"
    exit 1
fi

pip install numpy>=1.24.0
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to install numpy"
    exit 1
fi

echo ""
echo "Setup complete!"
echo "To activate the virtual environment, run:"
echo "  source venv/bin/activate"
echo ""
echo "To run the script:"
echo "  python gesture_control.py"

