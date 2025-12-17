#!/bin/bash
# start_isaac.sh - Für Teammitglieder (vorgebautes Image)

IMAGE="animeeye/isaac_ros:latest"
CONTAINER_NAME="isaac_workspace_${USER}"

echo "📦 Pulling Isaac ROS workspace image (this may take a while)..."
docker pull "$IMAGE"

xhost +local:root 2>/dev/null

echo "🚀 Starting container..."
docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --gpus all \
    --privileged \
    --net host \
    -e DISPLAY="$DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$(pwd)/config:/workspaces/isaac_ros-dev/config" \
    -v "$(pwd)/data:/workspaces/isaac_ros-dev/data" \
    -w /workspaces/isaac_ros-dev \
    "$IMAGE" \
    bash -c "source install/setup.bash && exec bash"