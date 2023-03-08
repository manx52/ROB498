## Setup Instructions
Setup software in the loop gazebo simulation for your drone. Should help development of your drone. Transferring to physical drone is pretty painless. These instructions are for Ubuntu 20 with Ros noetic. You can find other versions here https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html

### 1. Install librealsense for T265
```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```

#### Optionally install the developer and debug packages:

```bash
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

### 2. Install realsense2 ROS 

```bash
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```

### 3. Install MAVROS Dependencies

```bash
sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras
sudo apt install ros-$ROS_DISTRO-pcl-ros
```

### 4. Install Geographiclib Datasets

- found in ``` cd ~/catkin_ws/src/ROB498/drone/scripts/```
- or download https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh\
#### After script is found/downloaded run
```bash
sudo bash ./install_geographiclib_datasets.sh
```

### 5. setup PX toolchain development environment for drone simulations

```bash
sudo apt update
sudo apt upgrade
sudo apt install git

cd ~/catkin_ws/src
git clone https://github.com/PX4/Firmware.git --recursive

cd Firmware
bash ./Tools/setup/ubuntu.sh
```

#### reboot computer
- found in ``` cd ~/catkin_ws/src/ROB498/drone/scripts/```
- or download wget https://raw.githubusercontent.com/ktelegenov/scripts/main/ubuntu_sim_ros_noetic.sh
#### After script is found/downloaded run
```bash
bash ubuntu_sim_ros_noetic.sh
```

#### close the terminal and open it again

```bash
cd ~/catkin_ws/src/Firmware
git submodule update --init --recursive
sudo apt-get install libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev libxml2-utils python3-rospkg python-jinja2
sudo apt-get install libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly -y
DONT_RUN=1 make px4_sitl_default gazebo-classic
```

#### Add this to your ~/.bashrc if you want to run it everytime from the terminal.

```bash
export PX_HOME=$HOME/catkin_ws/src/Firmware
source ${PX_HOME}/Tools/simulation/gazebo-classic/setup_gazebo.bash ${PX_HOME} ${PX_HOME}/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX_HOME}:${PX_HOME}/Tools/simulation/gazebo-classic/sitl_gazebo-classic
```

#### Source your bash
```bash
source ~/.bashrc
```
## Other Instructions
- Sometimes you need to ```bashsource ~/.bashrc``` in a terminal to get it to work

#### Launching simulation
```bash
cd ~/catkin_ws
catkin build drone
source ~/devel/setup.bash
roslaunch drone drone.launch
```
### VIO Instructions
```bash
cd ~/catkin_ws/src
git clone git@github.com:dbaldwin/VIO.git

cd ~/catkin_ws/
catkin build px4_realsense_bridge
```
- there is a way to get VIO working in Sim, but I have not had enough time to implement it.

