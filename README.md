# Isaac ROS Nvblox Workspace
Folgt dem Dokument fürs Setup vom Docker
## DINO+SAM Setup

Also wenn ihr den Nvidia cli ros container habt, müsst ihr folgendes tun:

1. Geht in den docker:
   ```bash
   isaac-ros activate
   ```

2. Dann sourcen sam dino holen und package builden:
   ```bash
   source scripts/start_dino.sh
   ```

3. Versionen downgraden aufgrund von konflikten:
   ```bash
   pip3 install --break-system-packages 'supervision==0.18.0'
   pip install --force-reinstall --break-system-packages "numpy<2"
   ```
4. Wenn ihr eine Aufnahme machen wollt (um die pipleine vorzubereiten tut dies)
    ```bash
    chmod +x scripts/preprocess_semantic_bag.sh
     ```
     ```bash
     ./scripts/preprocess_semantic_bag.sh \
     /workspaces/isaac_ros-dev/bags/tugbot_slam_bag_point \
     /workspaces/isaac_ros-dev/bags/tugbot_semantic_bag
     ```
## Pipeline ausführen

So jetzt runnen (sam/dino geht mesh noch nicht richtig liegt evtl. an rosbag, schaut das ihr die rosbag natürlich am richtigen path habt.):

```bash
ros2 launch my_dino_package semantic_pipeline.launch.py   bag_path:=/workspaces/isaac_ros-dev/bags/tugbot_semantic_bag_test/   rate:=5
```
speichern geht mit: 
```bash
ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath   "{file_path: '/workspaces/isaac_ros-dev/semantic_mesh.ply'}"
```
Wenn ihr was am skript ändert immer neu builden
```bash
colcon build --packages-select my_dino_package && source install/setup.bash
```

