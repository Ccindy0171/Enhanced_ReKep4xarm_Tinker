#!/bin/bash

# Setup script for TAPnet and Point Tracker dependencies
# This script handles the installation of TAPnet and downloads required checkpoints

set -e

echo "Setting up TAPnet for Point Tracker..."

# Navigate to the parent directory to find tapnet
cd "$(dirname "$0")/.."

# Check if tapnet directory exists
if [ ! -d "../tapnet" ]; then
    echo "Error: TAPnet directory not found at ../tapnet"
    echo "Please clone TAPnet repository first:"
    echo "git clone https://github.com/google-deepmind/tapnet.git ../tapnet"
    exit 1
fi

# Install TAPnet with a more robust approach
echo "Installing TAPnet..."
cd ../tapnet

# First, let's try to fix the setup.py if it exists
if [ -f "setup.py" ]; then
    echo "Found setup.py, attempting installation..."
    python3 -m pip install -e . --user
else
    echo "No setup.py found, trying direct installation..."
    # Install dependencies first
    python3 -m pip install torch torchvision numpy opencv-python tree --user
    
    # Add the tapnet directory to Python path via a .pth file
    PYTHON_PATH=$(python3 -c "import site; print(site.getusersitepackages())")
    echo "$(pwd)" > "$PYTHON_PATH/tapnet.pth"
    echo "Added TAPnet to Python path: $PYTHON_PATH/tapnet.pth"
fi

# Create checkpoints directory if it doesn't exist
mkdir -p checkpoints

# Download the required checkpoint if it doesn't exist
if [ ! -f "checkpoints/causal_bootstapir_checkpoint.pt" ]; then
    echo "Downloading TAPnet checkpoint..."
    cd checkpoints
    
    # Check if wget or curl is available
    if command -v wget > /dev/null; then
        wget -O causal_bootstapir_checkpoint.pt "https://storage.googleapis.com/dm-tapnet/causal_bootstapir_checkpoint.pt"
    elif command -v curl > /dev/null; then
        curl -L -o causal_bootstapir_checkpoint.pt "https://storage.googleapis.com/dm-tapnet/causal_bootstapir_checkpoint.pt"
    else
        echo "Error: Neither wget nor curl found. Please install one of them or download manually:"
        echo "https://storage.googleapis.com/dm-tapnet/causal_bootstapir_checkpoint.pt"
        echo "Save it as: $(pwd)/causal_bootstapir_checkpoint.pt"
        exit 1
    fi
    
    cd ..
else
    echo "Checkpoint already exists."
fi

# Copy or link the checkpoint to the Enhanced_ReKep4xarm_Tinker directory
cd "../Enhanced_ReKep4xarm_Tinker"
if [ ! -d "checkpoints" ]; then
    mkdir -p checkpoints
fi

if [ ! -f "checkpoints/causal_bootstapir_checkpoint.pt" ]; then
    echo "Linking checkpoint to Enhanced_ReKep4xarm_Tinker/checkpoints/"
    ln -s "$(pwd)/../tapnet/checkpoints/causal_bootstapir_checkpoint.pt" "checkpoints/causal_bootstapir_checkpoint.pt"
fi

echo "TAPnet setup complete!"
echo ""
echo "To test the installation, run:"
echo "python3 -c 'from tapnet.torch1 import tapir_model; print(\"TAPnet import successful!\")'"
