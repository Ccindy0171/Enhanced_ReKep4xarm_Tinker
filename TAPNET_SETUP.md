# TAPnet Installation Guide

This guide helps you fix the TAPnet installation issues for the Point Tracker.

## Problem

You're seeing "UNKNOWN-0.0.0" when installing TAPnet, which indicates issues with the package metadata in the TAPnet repository.

## Solutions

### Option 1: Automated Setup (Recommended)

Run the automated setup script:

```bash
./setup_tapnet.sh
```

This script will:
- Install TAPnet dependencies
- Add TAPnet to your Python path
- Download the required checkpoint file
- Set up proper symlinks

### Option 2: Manual Setup

1. **Install dependencies:**
```bash
python3 -m pip install torch torchvision numpy opencv-python tree --user
```

2. **Add TAPnet to Python path:**
```bash
# Navigate to TAPnet directory
cd ../tapnet

# Add to Python path
PYTHON_PATH=$(python3 -c "import site; print(site.getusersitepackages())")
echo "$(pwd)" > "$PYTHON_PATH/tapnet.pth"
```

3. **Download checkpoint:**
```bash
# Create checkpoints directory
mkdir -p checkpoints

# Download checkpoint (choose one method)
# Using wget:
wget -O checkpoints/causal_bootstapir_checkpoint.pt "https://storage.googleapis.com/dm-tapnet/causal_bootstapir_checkpoint.pt"

# Using curl:
curl -L -o checkpoints/causal_bootstapir_checkpoint.pt "https://storage.googleapis.com/dm-tapnet/causal_bootstapir_checkpoint.pt"
```

4. **Link checkpoint to Enhanced_ReKep4xarm_Tinker:**
```bash
cd ../Enhanced_ReKep4xarm_Tinker
mkdir -p checkpoints
ln -s "$(pwd)/../tapnet/checkpoints/causal_bootstapir_checkpoint.pt" "checkpoints/causal_bootstapir_checkpoint.pt"
```

### Option 3: Diagnostic Tool

Run the diagnostic tool to check what's missing:

```bash
python3 tapnet_diagnostic.py
```

This will tell you exactly what needs to be fixed.

## Verification

Test that everything works:

```bash
python3 -c "from tapnet.torch1 import tapir_model; print('TAPnet import successful!')"
```

## Alternative: Using Git Submodules

If you want a more permanent solution, consider adding TAPnet as a git submodule:

```bash
git submodule add https://github.com/google-deepmind/tapnet.git tapnet
git submodule update --init --recursive
```

Then modify your Python path or installation accordingly.

## Troubleshooting

### Issue: "Import tapnet.torch1 could not be resolved"

This means TAPnet is not in your Python path. Run the setup script or manually add it to your path.

### Issue: "FileNotFoundError: checkpoints/causal_bootstapir_checkpoint.pt"

The checkpoint file is missing. Download it manually or run the setup script.

### Issue: "UNKNOWN-0.0.0" package

This is a metadata issue in the TAPnet repository. Our solutions bypass the pip installation and use direct path manipulation.

## Directory Structure

After successful setup, you should have:

```
Enhanced_ReKep4xarm_Tinker/
├── checkpoints/
│   └── causal_bootstapir_checkpoint.pt  # Symlink to ../tapnet/checkpoints/
├── point_tracker/
│   └── point_track_ros.py
└── ...

../tapnet/                               # TAPnet repository
├── checkpoints/
│   └── causal_bootstapir_checkpoint.pt  # Downloaded checkpoint
├── tapnet/
│   └── torch1/
│       └── tapir_model.py
└── ...
```
