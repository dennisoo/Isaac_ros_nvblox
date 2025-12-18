#!/bin/bash
# download_weights.sh - Downloads model weights to data/weights

WEIGHTS_DIR="data/weights"
mkdir -p "$WEIGHTS_DIR"

echo "📥 Downloading SAM Model..."
wget -nc -P "$WEIGHTS_DIR" https://dl.fbaipublicfiles.com/segment_anything/sam_vit_h_4b8939.pth

echo "📥 Downloading GroundingDINO Model..."
wget -nc -P "$WEIGHTS_DIR" https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth

echo "📥 Downloading GroundingDINO Config..."
wget -nc -P "$WEIGHTS_DIR" https://raw.githubusercontent.com/IDEA-Research/GroundingDINO/main/groundingdino/config/GroundingDINO_SwinT_OGC.py

echo "✅ All weights are ready in $WEIGHTS_DIR." 
