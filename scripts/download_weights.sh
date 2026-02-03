#!/bin/bash
# download_weights.sh - Downloads weights for GroundingDINO, SAM, and MobileSAM

set -e

# Define paths
# (Assuming script is run from workspace root via start_dino.sh, or directly from scripts/)
if [ -d "data/weights" ]; then
    DATA_DIR="data/weights"
else
    # Fallback if run from scripts/ dir
    DATA_DIR="../data/weights"
fi
mkdir -p "$DATA_DIR"

echo "Downloading weights to $DATA_DIR..."

# 1. GroundingDINO
echo "--- GroundingDINO ---"
if [ ! -f "$DATA_DIR/groundingdino_swint_ogc.pth" ]; then
    echo "Downloading GroundingDINO weights..."
    wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth -P "$DATA_DIR"
fi
if [ ! -f "$DATA_DIR/GroundingDINO_SwinT_OGC.py" ]; then
     echo "Downloading GroundingDINO config..."
     wget -q https://raw.githubusercontent.com/IDEA-Research/GroundingDINO/main/groundingdino/config/GroundingDINO_SwinT_OGC.py -P "$DATA_DIR"
fi

# 2. SAM ViT-H (Huge)
echo "--- SAM ViT-H (Huge) ---"
if [ ! -f "$DATA_DIR/sam_vit_h_4b8939.pth" ]; then
    echo "Downloading SAM ViT-H weights..."
    wget -q https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth -P "$DATA_DIR"
fi

# 3. SAM ViT-B (Base)
echo "--- SAM ViT-B (Base) ---"
if [ ! -f "$DATA_DIR/sam_vit_b_01ec64.pth" ]; then
    echo "Downloading SAM ViT-B weights..."
    wget -q https://dl.fbaipublicfiles.com/segment_anything/sam_vit_b_01ec64.pth -P "$DATA_DIR"
fi

# 4. MobileSAM (Tiny) - NEW!
echo "--- MobileSAM ---"
if [ ! -f "$DATA_DIR/mobile_sam.pt" ]; then
    echo "Downloading MobileSAM weights..."
    wget -q https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt -P "$DATA_DIR"
else
    echo "✔ MobileSAM already present."
fi

echo "✅ All weights ready!"