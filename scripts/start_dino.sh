#!/bin/bash
# start_dino.sh - Complete setup: dependencies, weights, and build

set -e  # Exit on error

echo "🚀 Starting full DINO setup..."

# Get script directory and go to workspace root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"  # Parent of scripts/
cd "$WORKSPACE_DIR" || exit 1

echo "📂 Working directory: $(pwd)"

# 1. Make scripts executable
echo "-------------------------------------------"
echo "🔧 Making scripts executable..."
chmod +x scripts/setup_dino.sh
chmod +x scripts/download_weights.sh

# 2. Install Python dependencies
echo "-------------------------------------------"
echo "📦 1/3: Installing Python dependencies..."
if !  bash scripts/setup_dino.sh; then
    echo "❌ Failed to install dependencies!"
    exit 1
fi

# 3. Download weights
echo "-------------------------------------------"
echo "🏋️  2/3: Downloading weights..."
if ! bash scripts/download_weights.sh; then
    echo "❌ Failed to download weights!"
    exit 1
fi

# 4. Build ROS package
echo "-------------------------------------------"
echo "🔨 3/3: Building ROS package..."
if ! colcon build --packages-select my_dino_package --symlink-install; then
    echo "❌ Build failed!"
    exit 1
fi

echo "-------------------------------------------"
echo "✅ SETUP COMPLETE!"
echo ""
echo "📝 Next steps:"
echo "   Launch pipeline: ros2 launch my_dino_package semantic_pipeline. launch.py"
echo "-------------------------------------------"

# If script was sourced, also source the setup file
if [ "${BASH_SOURCE[0]}" != "${0}" ]; then
    echo "🌍 Sourcing ROS environment..."
    source install/setup.bash
    echo "✅ Environment ready!  You can now launch the pipeline."
else
    echo "💡 NOTE: To apply ROS environment, run:  source install/setup.bash"
fi