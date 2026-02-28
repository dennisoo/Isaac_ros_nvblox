#!/bin/bash
# setup_dino.sh - Installs dependencies incl. MobileSAM & GPU Fixes


#gitconfig fix - redirect HOME so git never touches the mounted /home/admin/.gitconfig
export HOME=/tmp/fakehome
mkdir -p "$HOME"
git config --global user.name "admin"
git config --global user.email "admin@localhost"
set -e

SETUP_MARKER="/tmp/.dino_setup_done"
if [ -f "$SETUP_MARKER" ]; then
    echo "Setup already completed (marker found). Skipping."
    echo "Delete $SETUP_MARKER to force reinstall."
    exit 0
fi

# Helper: only install if missing or wrong version
pkg_ok() {
    python3 -c "import $1" 2>/dev/null
}
pkg_ver_ok() {
    python3 -c "import $1; assert $1.__version__=='$2'" 2>/dev/null
}

# Define PIP command with break-system-packages for Docker
sudo apt install -y python3-pip
PIP="pip3 install --break-system-packages"

# 1. Update pip
$PIP --upgrade pip

# 2. Transformers
pkg_ver_ok transformers 4.36.2 || $PIP transformers==4.36.2

# 3. Basic dependencies (only install if missing, don't upgrade CUDA deps)
echo "Checking basic packages..."
pkg_ok numpy || $PIP "numpy<2.0.0"
pkg_ok cv2 || $PIP opencv-python
pkg_ok PIL || $PIP pillow
pkg_ok yaml || $PIP pyyaml

# 4. Segment Anything (Standard)
pkg_ok segment_anything || { echo "Installing Segment Anything..."; $PIP segment-anything; }

# 5. MobileSAM (Fast & Lightweight)
pkg_ok mobile_sam || { echo "Installing MobileSAM..."; $PIP git+https://github.com/ChaoningZhang/MobileSAM.git; }

# 6. Supervision (Pinned version for compatibility)
pkg_ver_ok supervision 0.18.0 || { echo "Installing Supervision 0.18.0..."; $PIP supervision==0.18.0; }

# 7. Grounding DINO
echo "Checking Grounding DINO..."
if python3 -c "from groundingdino.util.inference import load_model" 2>/dev/null; then
    echo "GroundingDINO already installed. ✓"
else
    # Try pre-built wheel first (faster/easier)
    if $PIP groundingdino-py 2>/dev/null; then
        echo "Installed groundingdino-py successfully."
    else
    echo "groundingdino-py not available, installing from source WITHOUT CUDA compilation..."
    
    # Clone repo to temp dir
    TEMP_DIR="/tmp/groundingdino_install"
    rm -rf "$TEMP_DIR"
    git clone https://github.com/IDEA-Research/GroundingDINO.git "$TEMP_DIR/GroundingDINO"
    cd "$TEMP_DIR/GroundingDINO"
    
    # Install WITHOUT building CUDA extension (CPU-only ops, but model still uses GPU)
    # This avoids compilation errors if nvcc is missing
    FORCE_CUDA=0 PIP_BREAK_SYSTEM_PACKAGES=1 pip3 install --no-build-isolation -e .
    
    # Cleanup
    rm -rf "$TEMP_DIR"
fi
fi

# 7. Verify Installation
echo "Verifying installation..."
python3 << 'EOF'
try:
    import torch
    print(f"PyTorch Version: {torch.__version__}")
    print(f"CUDA Available: {torch.cuda.is_available()}")
    
    if torch.cuda.is_available():
        print(f"   Device: {torch.cuda.get_device_name(0)}")
        
    from groundingdino.util.inference import load_model
    print("GroundingDINO imported")
    
    from segment_anything import sam_model_registry
    print("Standard SAM imported")
    
    from mobile_sam import sam_model_registry as mobile_registry
    print("MobileSAM imported")
    
    import supervision as sv
    print(f"Supervision imported (v{sv.__version__})")
    
    print("\n ALL SYSTEMS GO!")
    
except ImportError as e:
    print(f"\n Import failed: {e}")
    exit(1)
EOF

# Mark setup as done
touch "$SETUP_MARKER"
echo "Setup complete!"  