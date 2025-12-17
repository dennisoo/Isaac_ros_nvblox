#!/bin/bash
# run_nvblox_with_bag.sh - Startet Nvblox + TF + RViz + Bag in einem Skript


#wenn ihr die Rosbag im data folder habt also im data ordner einen ordner mein_lidar_datenset dann (sollte) dieses skript klappen.

CONTAINER_NAME="isaac_workspace"

echo "🚀 Starte Nvblox-Komponenten..."

# 1. Nvblox Node (im Hintergrund)
docker exec -d $CONTAINER_NAME bash -c "source install/setup.bash && ros2 run nvblox_ros nvblox_node --ros-args -p use_sim_time:=true -p global_frame:=World -p pose_frame:=PandarXT_32_10hz -p base_frame:=PandarXT_32_10hz -p use_lidar:=true -p use_depth:=false -p use_color:=false -p lidar_max_range:=30.0 -r pointcloud:=/point_cloud"

# 2. TF Publisher (im Hintergrund)
docker exec -d $CONTAINER_NAME bash -c "source install/setup.bash && ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 PandarXT_32_10hz base_link"

# 3. RViz (im Hintergrund)
docker exec -d $CONTAINER_NAME bash -c "source install/setup.bash && rviz2"

echo "⏳ Warte 3 Sekunden für RViz..."
sleep 3

# 4. Bag abspielen (im Vordergrund)
echo "▶️ Spiele Bag-Datei ab..."
docker exec -it $CONTAINER_NAME bash -c "source install/setup.bash && ros2 bag play /workspaces/isaac_ros-dev/data/mein_lidar_dataset --clock"

echo "✅ Fertig! Drücke Strg+C zum Beenden."
