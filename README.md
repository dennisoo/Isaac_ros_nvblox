# Isaac ROS Nvblox Workspace

Ein fertig eingerichteter Docker-Container für Isaac ROS mit Nvblox, perfekt für Teammitglieder.



1. **Repository klonen**:
   ```bash
   git clone https://github.com/dennisoo/Isaac_ros_nvblox.git
   cd Isaac_ros_nvblox
   
2. **Startskript ausführbar machen**
    chmod +x scripts/start_isaac.sh
3. **Container Starten**
    ./scripts/start_isaac.sh
4. **Testen**
    Das sollte jetzt laufen überprüft mal ob folgende Befehle klappen:
    ros2 (Es kommt einfach so eine riesen Liste an befehlen)
    rviz2 (Es sollte sich rviz öffnen)
    ros2 pkg list | grep nvblox (Hier sollten nvblox pakete angezeigt werden (hoffentlich))
5. **Informationen zur Struktur**
    Aktuell ist nur ros2/rviz2/nvblox usw. aufgesetzt wenn ihr das von git bekommen habt, solltet ihr eine ordnerstruktur haben.
    Ich kann mal erklären was ich mir gedacht habe:
    1. **src**
    im src Ordner kommt unser Main Development hin hab da jetzt mal nen home ordner reingetan, auf ne richtige Struktur können wir uns ja am freitag einigen.
    2. **scripts**
    Hier würd ich hilfsskripts reinmachen wie den start, evtl. auch noch sachen für rosbags etc.
    3. **data**
    Hier würden rosbags bspw. reinkommen dieser Ordner ist im gitignore um das git kleinzuhalten, die rosbags müssten wir uns dan hin und her schicken.
    4. **install/build**
    da sind einfach ein paar build related sachen drin einfach ignorieren
    5. **config**

    hier kommen später unsere .yml (oder was auch immer) Dateien rein um die Pipeline zu konfigurieren
Also wenn ihr den Nvidia cli ros container habt, müsst ihr folgendes tun:
geht in den docker: isaac-ros activate
dann sourcen sam dino holen und package builden: source scripts/start_dino.sh
versionen downgraden aufgrund von konflikten:
pip3 install --break-system-packages 'supervision==0.18.0'
pip install --force-reinstall --break-system-packages "numpy<2"
So jetzt runnen (sam/dino geht mesh noch nicht richtig liegt evtl. an rosbag, schaut das ihr die rosbag natürlich am richtigen path habt.)
ros2 launch my_dino_package semantic_pipeline.launch.py \
  bag_path:=/workspaces/isaac_ros-dev/bags/mein_lidar_dataset \
  rate:=0.5
