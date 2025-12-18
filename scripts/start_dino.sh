#!/bin/bash
# setup_all.sh - Installs dependencies, downloads weights, builds the package, and sets up ROS.

# Exit immediately if a command exits with a non-zero status
set -e

echo "🚀 Starting full setup..."

# 1. Ensure helper scripts are executable
chmod +x scripts/setup_dino.sh
chmod +x scripts/download_weights.sh

# 2. Install dependencies
echo "-------------------------------------------"
echo "📦 1/3: Installing Python dependencies..."
./scripts/setup_dino.sh

# 3. Download weights
echo "-------------------------------------------"
echo "🏋️ 2/3: Downloading weights..."
./scripts/download_weights.sh

# 4. Build package
echo "-------------------------------------------"
echo "🔨 3/3: Building ROS package..."
# Ensure we are in the workspace root
cd /workspaces/isaac_ros-dev
colcon build --symlink-install --packages-select my_dino_package

echo "-------------------------------------------"
echo "✅ DONE! Everything installed and built."
echo "💡 NOTE: To apply the ROS environment changes to your current terminal,"
echo "         you MUST run this script using 'source' or '.' :"
echo "         Example: source scripts/setup_all.sh"
echo "-------------------------------------------"

# Source the setup file (only works if script is sourced)
source install/setup.bash
echo "🌍 ROS 2 environment loaded."