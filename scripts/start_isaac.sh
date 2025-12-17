#!/bin/bash
# start_isaac.sh - Für Teammitglieder (vorgebautes Image)

IMAGE="animeeye/isaac_ros:latest"
CONTAINER_NAME="isaac_workspace"

# WICHTIG: Finde das Projektverzeichnis automatisch
# basierend auf dem Ort DIESES Skripts
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"  # Geht einen Ordner hoch von scripts/

echo "📦 Pulling Isaac ROS workspace image (this may take a while)..."
docker pull "$IMAGE"

xhost +local:root 2>/dev/null

echo "🚀 Starting container from project: $PROJECT_DIR"
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --gpus all \
    --privileged \
    --net host \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PROJECT_DIR/config:/workspaces/isaac_ros-dev/config" \
    -v "$PROJECT_DIR/data:/workspaces/isaac_ros-dev/data" \
    -w /workspaces/isaac_ros-dev \
    "$IMAGE" \
    bash -c "source install/setup.bash && exec bash"