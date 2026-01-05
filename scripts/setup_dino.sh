#!/bin/bash
# setup_dino.sh - Installs dependencies WITHOUT compiling CUDA extensions

set -e

echo "🔧 Installing Python dependencies..."

PIP="pip3 install --break-system-packages"

# Update pip
$PIP --upgrade pip

# Basic dependencies
echo "📦 Installing basic packages..."
$PIP numpy opencv-python pillow pyyaml

# Segment Anything
echo "📦 Installing Segment Anything..."
$PIP segment-anything

# Supervision
echo "📦 Installing Supervision..."
$PIP supervision

# Grounding DINO - Use pre-built wheel (NO CUDA compilation)
echo "🦖 Installing Grounding DINO (pre-built, no compilation)..."

# Option 1: Try groundingdino-py (if available)
if $PIP groundingdino-py 2>/dev/null; then
    echo "✅ Installed groundingdino-py"
else
    echo "⚠️  groundingdino-py not available, installing from source WITHOUT CUDA..."
    
    # Clone repo
    TEMP_DIR="/tmp/groundingdino_install"
    rm -rf "$TEMP_DIR"
    git clone https://github.com/IDEA-Research/GroundingDINO.git "$TEMP_DIR/GroundingDINO"
    cd "$TEMP_DIR/GroundingDINO"
    
    # Install WITHOUT building CUDA extension (CPU-only, but models still run on GPU)
    echo "🔨 Installing GroundingDINO (skipping CUDA compilation)..."
    FORCE_CUDA=0 PIP_BREAK_SYSTEM_PACKAGES=1 pip3 install --no-build-isolation -e .
    
    # Cleanup
    rm -rf "$TEMP_DIR"
fi

# Verify
echo "✅ Verifying installation..."
python3 << 'EOF'
try:
    from groundingdino.util.inference import load_model, predict
    from segment_anything import sam_model_registry
    import supervision as sv
    import torch
    print("✅ All imports successful!")
    print(f"   PyTorch:  {torch.__version__}")
    print(f"   CUDA available: {torch.cuda.is_available()}")
except ImportError as e:
    print(f"❌ Import failed: {e}")
    exit(1)
EOF

echo "✅ Installation complete!"