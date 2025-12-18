#!/bin/bash
# setup_dino.sh - Installs dependencies for DINO/SAM inside the container

echo "🔧 Installing Python dependencies..."
# Pinning versions for compatibility (NumPy 1.x, OpenCV < 4.10)
pip install "numpy<2.0" "opencv-python<4.10" segment-anything supervision

echo "🦖 Cloning and patching GroundingDINO..."
# Use a temporary directory to keep the src folder clean
TEMP_DIR=$(mktemp -d)
git clone https://github.com/IDEA-Research/GroundingDINO.git "$TEMP_DIR/GroundingDINO"

cd "$TEMP_DIR/GroundingDINO" || exit

# Apply automatic patches for PyTorch/CUDA compatibility (fixes 'value.type()' errors)
sed -i 's/value.type().is_cuda()/value.is_cuda()/g' groundingdino/models/GroundingDINO/csrc/MsDeformAttn/ms_deform_attn_cuda.cu
sed -i 's/value.type()/value.scalar_type()/g' groundingdino/models/GroundingDINO/csrc/MsDeformAttn/ms_deform_attn_cuda.cu

echo "🔨 Compiling GroundingDINO (this might take a moment)..."
export CUDA_HOME=/usr/local/cuda
pip install --no-build-isolation .

# Cleanup
rm -rf "$TEMP_DIR"

echo "✅ Installation complete!"