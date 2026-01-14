#!/bin/bash
# Vorverarbeitung: Erstellt eine Bag mit semantischen Bildern
#
# Nutzung: ./preprocess_semantic_bag.sh <input_bag> <output_bag>
#
# Dieser Prozess:
# 1. Spielt die originale Bag mit 0.1x Speed ab
# 2. DINO/SAM verarbeitet die Bilder
# 3. Nimmt alle Topics + semantic topics in neue Bag auf

set -e

INPUT_BAG="${1:-/workspaces/isaac_ros-dev/bags/tugbot_slam_bag_point}"
OUTPUT_BAG="${2:-/workspaces/isaac_ros-dev/bags/tugbot_semantic_bag}"

echo "=== Semantic Bag Preprocessing ==="
echo "Input:  $INPUT_BAG"
echo "Output: $OUTPUT_BAG"
echo ""

# Cleanup alte output bag falls vorhanden
rm -rf "$OUTPUT_BAG"

# Cleanup function
cleanup() {
    echo ""
    echo "Stopping processes..."
    if [ ! -z "$RECORD_PID" ]; then
        kill $RECORD_PID 2>/dev/null || true
        wait $RECORD_PID 2>/dev/null || true
    fi
    if [ ! -z "$DINO_PID" ]; then
        kill $DINO_PID 2>/dev/null || true
        wait $DINO_PID 2>/dev/null || true
    fi
}

# Register cleanup on exit
trap cleanup EXIT INT TERM

# Terminal 1: DINO Node starten (mit sim time)
echo "[1/3] Starting DINO/SAM node..."
ros2 run my_dino_package dino_nvblox_node --ros-args -p use_sim_time:=true \
    -r image:=/camera/color/image \
    -r camera_info:=/camera/color/camera_info &
DINO_PID=$!
echo "DINO PID: $DINO_PID"
sleep 5

# Terminal 2: Bag Recording starten (alle relevanten Topics)
echo "[2/3] Starting bag recording..."
ros2 bag record -o "$OUTPUT_BAG" \
    /camera/color/image \
    /camera/color/camera_info \
    /camera/depth/image \
    /camera/depth/camera_info \
    /semantic/image_rgb8 \
    /semantic/image_mono8 \
    /semantic/camera_info \
    /tf \
    /odom \
    /imu \
    /pointcloud \
    /clock &
RECORD_PID=$!
echo "Recording PID: $RECORD_PID"
sleep 2

# Terminal 3: Bag abspielen mit sehr niedriger Rate
echo "[3/3] Playing input bag at 0.1x speed (this will take a while)..."
echo "Press Ctrl+C to stop early"
echo ""
ros2 bag play "$INPUT_BAG" --clock --rate 0.1

echo ""
echo "Bag playback finished, waiting for final messages..."
sleep 5

# Cleanup wird automatisch durch trap aufgerufen

echo ""
echo "=== Done! ==="
echo "Semantic bag created at: $OUTPUT_BAG"
echo ""
echo "To use with nvblox, update your launch file to:"
echo "  - Color input: /semantic/image_rgb8"
echo "  - Camera info: /semantic/camera_info"
