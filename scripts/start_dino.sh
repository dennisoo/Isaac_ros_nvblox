#!/bin/bash
# start_dino.sh - The "One-Click" Setup Script
# Includes AGGRESSIVE fixes for NumPy and PyTorch

set -e

# Determine script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_DIR"

echo "==========================================="
echo "   STARTING DINO/SAM SETUP"
echo "==========================================="
echo "Working directory: $WORKSPACE_DIR"

# 1. Make scripts executable
chmod +x scripts/*.sh

# 2. Run basic setup
echo "-------------------------------------------"
echo "1/4: Running base setup..."
./scripts/setup_dino.sh || echo "Base setup warning (ignoring)..."

# --- AUTOMATIC FIXES START ---
echo "-------------------------------------------"
echo "2/4: FORCING CRITICAL FIXES..."

# [Fix A] INTELLIGENT PyTorch Downgrade
echo "-> [Fix A] Checking GPU Compatibility..."
NEEDS_DOWNGRADE="false"

if command -v nvidia-smi &> /dev/null; then
    CC_STR=$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader | head -n 1)
    CC_INT=$(echo "$CC_STR" | tr -d '.')
    if [[ "$CC_INT" =~ ^[0-9]+$ ]]; then
        if [ "$CC_INT" -lt 75 ]; then
            echo "Old GPU ($CC_STR). Downgrade required."
            NEEDS_DOWNGRADE="true"
        else
            echo "Modern GPU ($CC_STR). Skipping PyTorch downgrade."
        fi
    else
        NEEDS_DOWNGRADE="true" # Fallback
    fi
else
    NEEDS_DOWNGRADE="false" # No GPU found or container limitation
fi

if [ "$NEEDS_DOWNGRADE" == "true" ]; then
    echo "Forcing PyTorch (CUDA 11.8)..."
    pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 --force-reinstall --break-system-packages
fi

# [Fix B] MobileSAM
echo "-> [Fix B] Ensuring MobileSAM..."
pip3 install git+https://github.com/ChaoningZhang/MobileSAM.git --break-system-packages --no-deps

# [Fix C] Supervision
echo "-> [Fix C] Locking Supervision to v0.18.0..."
pip3 install "supervision==0.18.0" --force-reinstall --break-system-packages --no-deps

# [Fix D] AGGRESSIVE NUMPY DOWNGRADE
# We uninstall first to ensure no traces of 2.x remain.
echo "-> [Fix D] NUKING NumPy 2.x and installing 1.x..."
pip3 uninstall -y numpy --break-system-packages || true
pip3 install "numpy<2.0.0" --force-reinstall --break-system-packages --no-deps

# Verify NumPy version immediately
echo "Verifying NumPy version:"
python3 -c "import numpy; print(f'   -> Installed: {numpy.__version__}')"

# --- AUTOMATIC FIXES END ---

# 3. Download Weights
echo "-------------------------------------------"
echo "3/4: Downloading Weights..."
./scripts/download_weights.sh

# 4. Build Workspace
echo "-------------------------------------------"
echo "4/4: Building ROS package..."
colcon build --packages-select my_dino_package --symlink-install

echo "-------------------------------------------"
echo "SETUP COMPLETE!"
echo "  Please run your preprocessing script now."
echo "==========================================="