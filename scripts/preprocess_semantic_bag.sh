#!/bin/bash
# Preprocessing: Creates a bag with semantic images using DINO/SAM
# Usage: ./preprocess_semantic_bag.sh <input_bag> <output_bag>

set -e

# --- MEMORY OPTIMIZATION FOR 8GB CARDS ---
# Hilft gegen "CUDA out of memory" durch Fragmentierung
export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True

# Kill old lingering processes to free VRAM before starting
echo "🧹 Cleaning up old processes..."
pkill -f dino_nvblox_node || true
sleep 1

# --- 1. INTERACTIVE SELECTION (Model) ---
echo "=========================================="
echo "   SEMANTIC PREPROCESSING CONFIG"
echo "=========================================="
echo ""
echo "--- Step 1: Model Selection ---"
echo "Which model do you want to use?"
echo "  1) MobileSAM (Fastest - Recommended for Dev)"
echo "  2) SAM ViT-B (Balanced - Good Quality)"
echo "  3) SAM ViT-H (Slowest - Best Quality)"
echo ""
read -p "Your choice [1-3]: " choice

if [ "$choice" == "3" ]; then
    MODEL_TYPE="vit_h"
    PLAY_RATE="0.05"
    echo "-> Selected: SAM ViT-H (Slow)"
elif [ "$choice" == "2" ]; then
    MODEL_TYPE="vit_b"
    PLAY_RATE="0.1"
    echo "-> Selected: SAM ViT-B (Balanced)"
else
    MODEL_TYPE="mobile"
    PLAY_RATE="0.5"
    echo "-> Selected: MobileSAM (Fast)"
fi

# --- 2. INTERACTIVE SELECTION (Threshold) ---
echo ""
echo "--- Step 2: Detection Sensitivity (Threshold) ---"
echo "Select the box threshold (0.25 - 0.40):"
echo "  0.25 : High Sensitivity (Detects almost everything, but high risk of hallucinations)"
echo "  0.30 : Balanced (Recommended default)"
echo "  0.40 : High Precision (No hallucinations, but might miss objects)"
echo ""
read -p "Enter threshold [default: 0.30] (or 1=0.25, 2=0.30, 3=0.40): " THRESH_INPUT

if [ "$THRESH_INPUT" == "1" ]; then
    BOX_THRESHOLD="0.25"
elif [ "$THRESH_INPUT" == "2" ]; then
    BOX_THRESHOLD="0.30"
elif [ "$THRESH_INPUT" == "3" ]; then
    BOX_THRESHOLD="0.40"
else
    BOX_THRESHOLD="${THRESH_INPUT:-0.30}"
fi

echo "-> Selected Threshold: $BOX_THRESHOLD"
echo "=========================================="

# Setup Paths
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WORKSPACE_DIR"

# Source ROS
source install/setup.bash

# Define Input/Output
INPUT_BAG="${1:-/workspaces/isaac_ros-dev/bags/tugbot_slam_bag_point}"
OUTPUT_BAG="${2:-/workspaces/isaac_ros-dev/bags/tugbot_semantic_bag}"

echo "Input:  $INPUT_BAG"
echo "Output: $OUTPUT_BAG"
echo ""

# Cleanup old output bag
if [ -d "$OUTPUT_BAG" ]; then
    echo "Removing old output bag..."
    rm -rf "$OUTPUT_BAG"
fi

# Cleanup function
cleanup() {
    echo ""
    echo "Stopping processes..."
    if [ ! -z "$RECORD_PID" ]; then
        kill $RECORD_PID 2>/dev/null || true
    fi
    if [ ! -z "$DINO_PID" ]; then
        kill $DINO_PID 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# --- 3. START DINO NODE ---
echo "[1/3] Starting DINO/SAM node..."

ros2 run my_dino_package dino_nvblox_node --ros-args \
    -p use_sim_time:=true \
    -p model_type:=$MODEL_TYPE \
    -p box_threshold:=$BOX_THRESHOLD \
    -p text_threshold:=$BOX_THRESHOLD \
    -r image:=/head_front_camera/color/image_raw \
    -r camera_info:=/head_front_camera/color/camera_info &

DINO_PID=$!
echo "DINO PID: $DINO_PID"

# Wait briefly for the model to load
if [ "$MODEL_TYPE" == "mobile" ]; then
    sleep 5
else
    echo "Waiting for heavy model to load..."
    sleep 10
fi

# --- 4. START RECORDING ---
echo "[2/3] Starting bag recording..."
ros2 bag record -o "$OUTPUT_BAG" --all &

RECORD_PID=$!
echo "Recording PID: $RECORD_PID"
sleep 2

# --- 5. PLAY BAG ---
echo "[3/3] Playing input bag at rate $PLAY_RATE..."
ros2 bag play "$INPUT_BAG" --clock --rate $PLAY_RATE

echo ""
echo "Bag playback finished. Waiting a few seconds for final messages..."
sleep 5

echo ""
echo "=== Done! ==="
echo "Semantic bag created at: $OUTPUT_BAG"
echo "You can now run the pipeline using this new bag."