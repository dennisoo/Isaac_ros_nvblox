#!/bin/bash
# setup_dino.sh - Installs dependencies incl. MobileSAM & GPU Fixes

set -e

echo "🔧 Installing Python dependencies..."

# Define PIP command with break-system-packages for Docker
PIP="pip3 install --break-system-packages"

# 1. Update pip
$PIP --upgrade pip

# 2. Basic dependencies
echo "Installing basic packages..."
$PIP numpy opencv-python pillow pyyaml

# 3. Segment Anything (Standard)
echo "Installing Segment Anything (Standard)..."
$PIP segment-anything

# 4. MobileSAM (Fast & Lightweight)
echo "Installing MobileSAM..."
$PIP git+https://github.com/ChaoningZhang/MobileSAM.git

# 5. Supervision (Pinned version for compatibility)
echo "Installing Supervision..."
$PIP supervision==0.18.0

# 6. Grounding DINO
echo "Installing Grounding DINO..."

# Try pre-built wheel first (faster/easier)
if $PIP groundingdino-py 2>/dev/null; then
    echo "✔ Installed groundingdino-py successfully."
else
    echo "⚠ groundingdino-py not available, installing from source WITHOUT CUDA compilation..."
    
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

# 7. Verify Installation
echo "Verifying installation..."
python3 << 'EOF'
try:
    import torch
    print(f"✅ PyTorch Version: {torch.__version__}")
    print(f"✅ CUDA Available: {torch.cuda.is_available()}")
    
    if torch.cuda.is_available():
        print(f"   Device: {torch.cuda.get_device_name(0)}")
        
    from groundingdino.util.inference import load_model
    print("✅ GroundingDINO imported")
    
    from segment_anything import sam_model_registry
    print("✅ Standard SAM imported")
    
    from mobile_sam import sam_model_registry as mobile_registry
    print("✅ MobileSAM imported")
    
    import supervision as sv
    print(f"✅ Supervision imported (v{sv.__version__})")
    
    print("\n🎉 ALL SYSTEMS GO!")
    
except ImportError as e:
    print(f"\n❌ Import failed: {e}")
    exit(1)
EOF

echo "Setup complete!"