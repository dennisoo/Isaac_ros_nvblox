# Info
This Git-Repo is used for the PSE project: "Incremental Semantic Mapping of Dynamic Indoor Enviornments". It tries to create a labelled 3D enviornment from ROS2-Data Streams. 

# Setup
## WSL
Führe in der Powershell (Administrator) folgendes aus und folge den Anweisungen von Windows:

```
wsl --install
```

## Docker
Gehe zu https://www.docker.com/ und installiere die AMD64 version. Starte anschließend deinen Rechner neu.


# Getting Started with ROS
## Vorwort
Was machen wir?
Wir folgen folgendem Tutroial: https://nvidia-isaac-ros.github.io/getting_started/index.html
Der Inhalt dieses Dokuments folgt diesem Tutorial, und gibt den Inhalt in vereinfachter weise wieder.

## Vorraussetzungen
### Hardware
**Speicherplatz:** 32+ GB
**GPU:** (Zitat Nvidia) 'Ampere or higher NVIDIA GPU Architecture with 8 GB RAM or higher'

### Software
**Ubuntu 24.04**
**Nvidia Driver 580**
**CUDA Version 13.0+**
Nutzt die App https://www.nvidia.com/de-at/software/nvidia-app/ zum Updaten (Anschließend natürlich neustarten)
Geht ins Windows-Terminal und gebt folgenden Befehl ein

```
nvidia-smi
```

Der Output sollte ungefähr so aussehen (Natürlich mit noch mehr infos die ich hier ausgelassen habe)

```
Fri Dec 26 02:43:33 2025
+-------------------------------------------------------------+
| NVIDIA-SMI 591.59 Driver Version: 591.59  CUDA Version: 13.1   |
+-------------------------------------------------------------+
```

Eure Ubuntu version könnt ihr wie folgt im Ubuntu terminal überprüfen, nutzt folgenden Befehl

```
lsb_release -a
```

Der Output sollte ungefähr so aus sehen:

```
No LSB modules are available.
Distributor ID: Ubuntu
Description:    Ubuntu 24.04.3 LTS
Release:        24.04
Codename:       noble
```

## Einen Workspace erstellen
Um einen ROS 2 Workspace zu erstellen, müsst ihr folgenden Befehl in eurem *WSL* terminal eingeben:

```
mkdir -p  ~/workspaces/isaac_ros-dev/src
echo 'export ISAAC_ROS_WS="${ISAAC_ROS_WS:-${HOME}/workspaces/isaac_ros-dev/}"' >> ~/.bashrc
source ~/.bashrc
```

Nun müssen wir unser Isaac Ros Apt Repsitory einstellen. 
Setzt den locale auf eurem WSL System zu UTF-8 indem ihr folgendes ausführt:

```
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

Am ende sollte euer locale befehl so aussehen:

```
user@Username:~$ locale
LANG=en_US.UTF-8
LANGUAGE=
LC_CTYPE="en_US.UTF-8"
LC_NUMERIC="en_US.UTF-8"
LC_TIME="en_US.UTF-8"
LC_COLLATE="en_US.UTF-8"
LC_MONETARY="en_US.UTF-8"
LC_MESSAGES="en_US.UTF-8"
LC_PAPER="en_US.UTF-8"
LC_NAME="en_US.UTF-8"
LC_ADDRESS="en_US.UTF-8"
LC_TELEPHONE="en_US.UTF-8"
LC_MEASUREMENT="en_US.UTF-8"
LC_IDENTIFICATION="en_US.UTF-8"
LC_ALL=
user@Username:~$
```

Installiert dependecies:

```
sudo apt update && sudo apt install curl gnupg
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Nun Sourcen wie das Isaac Ros Repository, aber erst updaten:

```
sudo apt-get update
```

```
k="/usr/share/keyrings/nvidia-isaac-ros.gpg"
curl -fsSL https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo gpg --dearmor \
    | sudo tee -a $k > /dev/null
f="/etc/apt/sources.list.d/nvidia-isaac-ros.list"
sudo touch $f
s="deb [signed-by=$k] https://isaac.download.nvidia.com/isaac-ros/release-4.2 noble main"
grep -qxF "$s" $f || echo "$s" | sudo tee -a $f

sudo apt-get update
```

## Die Isaac RSO CLI initialisieren
Installiert erstmal pyhton3-pip:

```
sudo apt install python3-pip
```

Installiert abhängigkeiten:

```
pip install termcolor --break-system-packages
```

Installiert die Isaac ROS CLI:

```
sudo apt-get install isaac-ros-cli
```

Nun installieren wir das NVIDIA Container Toolkit:
Installiert abhängigkeiten:

```
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
   ca-certificates \
   curl \
   gnupg2
```

Konfiguiriert das Production repository:

```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Updated die packages aus dem repo:

```
sudo apt-get update
```

und installiert NVIDIA Container Toolkit packages:

```
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.2-1
  sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
```

Konfiguriert den Docker:

```
sudo nvidia-ctk runtime configure --runtime=docker
```

Startet ihn anschließend neu:

```
sudo systemctl daemon-reload && sudo systemctl restart docker
```

Initialisiert nun die ROS CLI:

```
sudo isaac-ros init docker
```

startet nun den docker mit:

```
isaac-ros activate
```

Testes ob ros2 geht mit:

```
ros2
```

Klappt isaac-ros activate nicht müsst ihr euch evtl noch zum docker hinzufügen.

```
sudo usermod -aG docker $USER 
```

Wenn der befehl klappt sieht 21:38 19/01/2026dies ungefähr wie folgt aus:

```
user@Username:~ isaac-ros activate
# ---Einige Warnungen---
admin@Username:/workspaces/isaac_ros-dev$ ros2
usage: ros2 [-h] [--use-python-default-buffering] Call ros2 <command> -h for more detailed usage. ...

ros2 is an extensible command-line tool for ROS 2.

options:
  -h, --help            show this help message and exit
  --use-python-default-buffering
                        Do not force line buffering in stdout and instead use the python default buffering, which
                        might be affected by PYTHONUNBUFFERED/-u and depends on whatever stdout is interactive or not

Commands:
  action     Various action related sub-commands
  bag        Various rosbag related sub-commands
  component  Various component related sub-commands
  daemon     Various daemon related sub-commands
  doctor     Check ROS setup and other potential issues
  interface  Show information about ROS interfaces
  launch     Run a launch file
  lifecycle  Various lifecycle related sub-commands
  multicast  Various multicast related sub-commands
  node       Various node related sub-commands
  param      Various param related sub-commands
  pkg        Various package related sub-commands
  run        Run a package specific executable
  security   Various security related sub-commands
  service    Various service related sub-commands
  topic      Various topic related sub-commands
  wtf        Use wtf as alias to doctor

  Call ros2 <command> -h for more detailed usage.
```

Den Docker könnt ihr mit folgendem Befehl verlassen:

```
exit
```


# Packages
## Vorwort
Wenn ihr hier angekommen seid, sollte ros2 im isaac docker Laufen. Nun wollen wir noch einige
Packages Installieren, die für unser Projekt notwendig sind.

## Nvblox
Wir wollen nun Nvblox für die mesh generierung installieren. Wir folgen dem Quickstarte guide
https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html
Stellt sicher das ihr aktuell nicht im Docker seid. (exited also) Stellt sicher das die libaries installiert sind:

```
sudo apt-get install -y curl jq tar
```

nun müssen wir die assets downloaden dies nimmt einiges an Zeit in anspruch:

```
NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_nvblox"
NGC_RESOURCE="isaac_ros_nvblox_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=4
MINOR_VERSION=2
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi
```

Da wir eigentlich incht immer wieder neu installieren wollen, müssen wir ein paar sachen Konfigurieren. Hierzu modifizieren wir die dockerfile config. Da die CLI wohl recht neu ist, ist die Offizielle Dokumentation hier etwas uneindeutig. Ich meine aber einige einfache Schritte gefunden zu haben.
Stoppt hierfür euren container:

```
docker stop isaac_ros_dev_container
```

erstellt den config folder:

```
mkdir -p ~/.config/isaac-ros-cli
```
Habt ihr Probleme mit rechten, gebt euch diese:
```
sudo chown -R $USER:$USER ~/.config
```
Erstellt hier nun manuell eine config.yaml der Inhalt sieht folgendermaßen aus:

```
cat <<EOF > ~/.config/isaac-ros-cli/config.yaml
docker:
  image:
    additional_image_keys:
      - user_pkgs
EOF
```

Wir erstellen nun eine dockerfile zum builden.

```
sudo mkdir -p /etc/isaac-ros-cli/docker
sudo tee /etc/isaac-ros-cli/docker/Dockerfile.user_pkgs > /dev/null <<EOF
ARG BASE_IMAGE
FROM \${BASE_IMAGE}

ENV ISAAC_ROS_WS=/workspaces/isaac_ros-dev
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
        ros-jazzy-isaac-ros-nvblox 
EOF
```
Im /etc/isaac-ros-cli/docker folder sollte sich nun eure Datei befinden. Wir builden nun (Dies nimmt einiges an Zeit in anspruch)

```
isaac-ros activate --build-local
```

Ihr solltet nun im Docker sein, überprüft nun ob die nvblox packages vorhanden sind:

```
ros2 pkg list | grep nvblox
#Der Output sollte ungefaehr so aussehen: 
admin@AnimeEye:/workspaces/isaac_ros-dev$ ros2 pkg list | grep nvblox
isaac_ros_nvblox
nvblox_examples_bringup
nvblox_image_padding
nvblox_msgs
nvblox_nav2
nvblox_ros
nvblox_ros_common
nvblox_ros_python_utils
nvblox_rviz_plugin
admin@AnimeEye:/workspaces/isaac_ros-dev$
```

Wenn ihr irgendwann nochmal in den docker wollt (wenn ihr draußen seid), müsst ihr nur den Befehl nutzen (ohhne --build-local)

```
isaac-ros activate
```

## Repo Pullen:
```
cd ~/workspaces/isaac_ros-dev
git clone https://github.com/dennisoo/Isaac_ros_nvblox.git temp
mv temp/* .
mv temp/.* . 2>/dev/null
rm -rf temp
mkdir -p bags meshes
```
Bekommt ihr eine Fehlermeldung, so liegt dies an eurer gitconfig, ändert diese.

.gitconfig ist bei installation oft fälschlicherweise ein Verzeichnis – löschen und neu erstellen.

Passt natürlich eure namen noch an.
```
rm -rf /home/denni/.gitconfig
git config --global user.name "DeinName"
git config --global user.email "deine@email.com"
```
## Informationen
Mesh Objekte werden in den meshes Ordner von den Skripts geladen.
Jegliche Rosbags, die ihr nutzen wollt, sollten bitte ind den bags Ordner.


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
4. Wenn ihr eine Aufnahme machen wollt (um die pipleine vorzubereiten tut dies, das skript ist aber nur dafür da, falls bereits eine unsegmentierte Bag (die point bag) vorhanden ist (output ist die semantic bag)
    ```bash
    chmod +x scripts/preprocess_semantic_bag.sh
     ```
     ```bash
     ./scripts/preprocess_semantic_bag.sh \
     /workspaces/isaac_ros-dev/bags/tugbot_slam_bag_point \
     /workspaces/isaac_ros-dev/bags/tugbot_semantic_bag
     ```
So jetzt runnen (sam/dino geht mesh noch nicht richtig liegt evtl. an rosbag, schaut das ihr die rosbag natürlich am richtigen path habt.):

```bash
ros2 launch my_dino_package semantic_pipeline.launch.py   bag_path:=/workspaces/isaac_ros-dev/bags/tugbot_semantic_bag_test/   rate:=5
```
```
Wenn ihr was am skript ändert immer neu builden
```bash
colcon build --packages-select my_dino_package && source install/setup.bash
```


