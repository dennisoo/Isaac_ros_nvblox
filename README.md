# Info
This Git repo is used for the PSE project: "Incremental Semantic Mapping of Dynamic Indoor Environments". It attempts to create a labelled 3D environment from ROS2 data streams.

This guide was finalized on 28.02.2026. If NVIDIA updates their download paths or changes the installation process, this guide may become outdated. In case of installation issues, please refer to the official NVIDIA documentation: https://nvidia-isaac-ros.github.io/getting_started/index.html. We are currently using Container Version 4.2.

# Setup
## WSL
Run the following in PowerShell (Administrator) and follow the Windows instructions:

```
wsl --install
```

## Docker
Go to https://www.docker.com/ and install the AMD64 version. Then restart your computer.


# Getting Started with ROS
## Foreword
What are we doing?
We are following this tutorial: https://nvidia-isaac-ros.github.io/getting_started/index.html
The content of this document follows the tutorial and presents it in a simplified manner.

## Prerequisites
### Hardware
**Storage:** 32+ GB
**GPU:** (Quote from NVIDIA) 'Ampere or higher NVIDIA GPU Architecture with 8 GB RAM or higher'

### Software
**Ubuntu 24.04**
**NVIDIA Driver 580**
**CUDA Version 13.0+**
Use the app https://www.nvidia.com/de-at/software/nvidia-app/ to update (and restart afterwards, of course).
Open the Windows terminal and enter the following command:

```
nvidia-smi
```

The output should look roughly like this (with more info that has been omitted here):

```
Fri Dec 26 02:43:33 2025
+-------------------------------------------------------------+
| NVIDIA-SMI 591.59 Driver Version: 591.59  CUDA Version: 13.1   |
+-------------------------------------------------------------+
```

You can check your Ubuntu version in the Ubuntu terminal with the following command:

```
lsb_release -a
```

The output should look roughly like this:

```
No LSB modules are available.
Distributor ID: Ubuntu
Description:    Ubuntu 24.04.3 LTS
Release:        24.04
Codename:       noble
```

## Creating a Workspace
To create a ROS 2 workspace, enter the following command in your *WSL* terminal:

```
mkdir -p  ~/workspaces/isaac_ros-dev/src
echo 'export ISAAC_ROS_WS="${ISAAC_ROS_WS:-${HOME}/workspaces/isaac_ros-dev/}"' >> ~/.bashrc
source ~/.bashrc
```

Now we need to set up our Isaac ROS Apt repository.
Set the locale on your WSL system to UTF-8 by running the following:

```
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings
```

At the end, your locale command output should look like this:

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

Install dependencies:

```
sudo apt update && sudo apt install curl gnupg
sudo apt install software-properties-common
sudo add-apt-repository universe
```

Now source the Isaac ROS repository, but update first:

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

## Initializing the Isaac ROS CLI
First, install python3-pip:

```
sudo apt install python3-pip
```

Install dependencies:

```
pip install termcolor --break-system-packages
```

Install the Isaac ROS CLI:

```
sudo apt-get install isaac-ros-cli
```

Now we install the NVIDIA Container Toolkit.
Install dependencies:

```
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
   ca-certificates \
   curl \
   gnupg2
```

Configure the production repository:

```
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Update packages from the repository:

```
sudo apt-get update
```

And install the NVIDIA Container Toolkit packages:

```
export NVIDIA_CONTAINER_TOOLKIT_VERSION=1.18.2-1
  sudo apt-get install -y \
      nvidia-container-toolkit=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      nvidia-container-toolkit-base=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container-tools=${NVIDIA_CONTAINER_TOOLKIT_VERSION} \
      libnvidia-container1=${NVIDIA_CONTAINER_TOOLKIT_VERSION}
```

Configure Docker:

```
sudo nvidia-ctk runtime configure --runtime=docker
```

Then restart it:

```
sudo systemctl daemon-reload && sudo systemctl restart docker
```

Now initialize the ROS CLI:

```
sudo isaac-ros init docker
```

Start the Docker container with:

```
isaac-ros activate
```

Test if ros2 works with:

```
ros2
```

If `isaac-ros activate` does not work, you may need to add yourself to the docker group:

```
sudo usermod -aG docker $USER 
```

If the command works, the output should look roughly like this:

```
user@Username:~ isaac-ros activate
# ---Some warnings---
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

You can leave the Docker container with the following command:

```
exit
```


# Packages
## Foreword
If you have reached this point, ros2 should be running inside the Isaac Docker container. Now we want to install some additional packages that are necessary for our project.

## Nvblox
We now want to install Nvblox for mesh generation. We follow the quickstart guide:
https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html
Make sure you are currently **not** inside the Docker container (i.e. you have exited it). Make sure the required libraries are installed:

```
sudo apt-get install -y curl jq tar
```

Now we need to download the assets — this will take some time:

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

Since we don't want to reinstall everything every time, we need to configure a few things. For this, we modify the Dockerfile config. Since the CLI is fairly new, the official documentation is somewhat unclear here. However, I believe I've found some simple steps.
First, stop your container:

```
docker stop isaac_ros_dev_container
```

Create the config folder:

```
mkdir -p ~/.config/isaac-ros-cli
```
If you have permission issues, grant yourself the rights:
```
sudo chown -R $USER:$USER ~/.config
```
Now manually create a config.yaml with the following content:

```
cat <<EOF > ~/.config/isaac-ros-cli/config.yaml
docker:
  image:
    additional_image_keys:
      - user_pkgs
EOF
```

We now create a Dockerfile for building:

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
Your file should now be in the `/etc/isaac-ros-cli/docker` folder. Now build (this will take some time):

```
isaac-ros activate --build-local
```

You should now be inside the Docker container. Verify that the nvblox packages are available:

```
ros2 pkg list | grep nvblox
#The output should look roughly like this: 
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

If you want to re-enter the Docker container later (after exiting), simply use the command (without --build-local):

```
isaac-ros activate
```

## Cloning the Repo
```
cd ~/workspaces/isaac_ros-dev
git clone https://github.com/dennisoo/Isaac_ros_nvblox.git temp
mv temp/* .
mv temp/.* . 2>/dev/null
rm -rf temp
mkdir -p bags meshes
```
If you get an error, it may be due to your gitconfig. Modify it accordingly.

The .gitconfig is often mistakenly a directory after installation — delete it and recreate it.

Adjust your name and email accordingly:
```
rm -rf /home/denni/.gitconfig
git config --global user.name "YourName"
git config --global user.email "your@email.com"
```
## Information
Mesh objects are saved to the meshes folder by the scripts.
Any rosbags you want to use should be placed in the bags folder.


## DINO+SAM Setup

Once you have the NVIDIA CLI ROS container, follow these steps:

1. Enter the Docker container:
   ```bash
   isaac-ros activate
   ```

2. Source SAM/DINO and build the package:
   ```bash
   source scripts/start_dino.sh
   ```

3. Versionen downgraden aufgrund von konflikten:
   ```bash
   pip3 install --break-system-packages 'supervision==0.18.0'
   pip install --force-reinstall --break-system-packages "numpy<2"
   ```
4. If you want to make a recording (to prepare the pipeline, do this, but the script is only there if an unsegmented bag (the point bag) already exists (output is the semantic bag)
    ```bash
    chmod +x scripts/preprocess_semantic_bag.sh
     ```
     ```bash
     ./scripts/preprocess_semantic_bag.sh \
     /workspaces/isaac_ros-dev/bags/tugbot_slam_bag_point \
     /workspaces/isaac_ros-dev/bags/tugbot_semantic_bag
     ```
## Additional folders
Create a bags folder in the workspace folder.
Now create a meshes folder, where the saved glb mesh files will end up.
## Run pipeline

Now run (sam/dino mesh is not working properly yet, possibly due to rosbag; make sure you have rosbag in the correct path):
```bash
ros2 launch my_dino_package semantic_pipeline.launch.py   bag_path:=/workspaces/isaac_ros-dev/bags/tugbot_semantic_bag_test/   rate:=5
```

If you change anything in the scripts, always rebuild:
```bash
colcon build --packages-select my_dino_package && source install/setup.bash
```

If you don't run the DINO install script, you need to manually install pip for the Tiago Pro launch file with nvblox (due to the newer NVIDIA ROS version):
```bash
colcon build --packages-select my_dino_package && source install/setup.bash && sudo apt-get install -y python3-pip
```

## Unity
If you want to view the mesh in Unity, use the LoadGLB.cs file in the scripts folder. To add it to Unity, do the following:

Window --> Package Management --> Package Manager. Click the plus on the left --> Install Package from git URL, then paste the URL: https://github.com/Siccity/GLTFUtility

Now drag the script into the Assets folder, and from there drag it into the Scene. Then adjust the path in the script to point to your mesh file, and start Unity. The mesh should be imported within a few seconds. You can also start Unity ahead of time before the script runs — the mesh will be automatically imported into Unity after processing is complete.
