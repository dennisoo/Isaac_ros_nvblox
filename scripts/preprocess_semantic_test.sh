#!/bin/bash
set -e

# Pfade definieren
INPUT_BAG="${1:-/workspaces/isaac_ros-dev/bags/tugbot_slam_bag_point}"
OUTPUT_BAG="${2:-/workspaces/isaac_ros-dev/bags/tugbot_semantic_bag}"
echo : "HERE"
LAUNCH_FILE="/workspaces/isaac_ros-dev/src/my_dino_packageNew/launch/dino_sam_nodes_launch.py"

echo "=== Semantic Bag Preprocessing (Isaac ROS SAM2 Edition) ==="
echo "Input:  $INPUT_BAG"
echo "Output: $OUTPUT_BAG"

# Alte Output Bag löschen
rm -rf "$OUTPUT_BAG"

cleanup() {
    echo "Stopping processes..."
    # Beendet die gesamte Prozessgruppe (Launch + alle Nodes)
    if [ ! -z "$LAUNCH_PID" ]; then kill -$LAUNCH_PID 2>/dev/null || true; fi
    if [ ! -z "$RECORD_PID" ]; then kill $RECORD_PID 2>/dev/null || true; fi
}
trap cleanup EXIT INT TERM

# 1. Launch der NVIDIA Nodes + Fusion Node
echo "[1/3] Launching DINO, SAM2 and Fusion Node..."
# Wir starten den Launch in einer eigenen Prozessgruppe (setsid)
setsid ros2 launch "$LAUNCH_FILE" \
    image_topic:=/camera/color/image \
    use_sim_time:=true &
LAUNCH_PID=$!

echo "Waiting for models to load (TensorRT can take a moment)..."
sleep 15 # Erhöht auf 15s, da SAM2/DINO etwas brauchen zum Initialisieren

# 2. Bag Recording starten
echo "[2/3] Starting bag recording..."
# WICHTIG: --clock ist nötig, damit alle Nodes synchron auf die Bag-Zeit hören
ros2 bag record -o "$OUTPUT_BAG" \
    /camera/color/image \
    /camera/color/camera_info \
    /camera/depth/image \
    /camera/depth/camera_info \
    /semantic/image_rgb8 \
    /semantic/image_mono8 \
    /tf \
    /tf_static \
    /odom \
    /imu \
    /pointcloud \
    /clock &
RECORD_PID=$!
sleep 2

# 3. Bag abspielen
echo "[3/3] Playing input bag at 0.1x speed..."
# WICHTIG: --clock ist nötig, damit alle Nodes synchron auf die Bag-Zeit hören
ros2 bag play "$INPUT_BAG" --clock --rate 0.1

echo "Playback finished. Saving..."
sleep 5