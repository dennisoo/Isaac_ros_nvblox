#!/bin/bash
# download_weights.sh - Downloads model weights to data/weights

set -e

WEIGHTS_DIR="data/weights"
mkdir -p "$WEIGHTS_DIR"

echo "📥 Downloading model weights to $WEIGHTS_DIR..."

# SAM Model
if [ !  -f "$WEIGHTS_DIR/sam_vit_h_4b8939.pth" ]; then
    echo "📥 Downloading SAM Model..."
    wget -P "$WEIGHTS_DIR" https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth
else
    echo "✅ SAM Model already downloaded"
fi

# GroundingDINO Model
if [ ! -f "$WEIGHTS_DIR/groundingdino_swint_ogc.pth" ]; then
    echo "📥 Downloading GroundingDINO Model..."
    wget -P "$WEIGHTS_DIR" https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
else
    echo "✅ GroundingDINO Model already downloaded"
fi

# GroundingDINO Config
if [ ! -f "$WEIGHTS_DIR/GroundingDINO_SwinT_OGC.py" ]; then
    echo "📥 Downloading GroundingDINO Config..."
    wget -P "$WEIGHTS_DIR" https://raw.githubusercontent.com/IDEA-Research/GroundingDINO/main/groundingdino/config/GroundingDINO_SwinT_OGC.py
else
    echo "✅ GroundingDINO Config already downloaded"
fi

echo "✅ All weights are ready in $WEIGHTS_DIR"