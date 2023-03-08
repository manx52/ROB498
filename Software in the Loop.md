
1. install librealsense

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

Optionally install the developer and debug packages:

sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg

2. install realsens2 ros

sudo apt-get install ros-$ROS_DISTRO-realsense2-camera

3. Install MAVROS Dependencies

sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras

4. install ros pcl

sudo apt install ros-$ROS_DISTRO-pcl-ros

5. Install install_geographiclib_datasets

found in cd ~/catkin_ws/src/ROB498/drone/scripts/
or download https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

sudo bash ./install_geographiclib_datasets.sh

6. setup PX toolchain development environment for drone simulations

sudo apt update
sudo apt upgrade
sudo apt install git
git clone https://github.com/PX4/Firmware.git --recursive
cd Firmware
bash ./Tools/setup/ubuntu.sh

## reboot computer
found in cd ~/catkin_ws/src/ROB498/drone/scripts/
or download wget https://raw.githubusercontent.com/ktelegenov/scripts/main/ubuntu_sim_ros_noetic.sh
bash ubuntu_sim_ros_noetic.sh

## close the terminal and open it again
cd ~/catkin_ws/src/Firmware
git submodule update --init --recursive
DONT_RUN=1 make px4_sitl_default gazebo-classic

Add this to
export PX_HOME=/home/manx52/catkin_ws/src/Firmware
source ${PX_HOME}/Tools/simulation/gazebo-classic/setup_gazebo.bash ${PX_HOME} ${PX_HOME}/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${PX_HOME}:${PX_HOME}/Tools/simulation/gazebo-classic/sitl_gazebo-classic

## Make sure to add the above inside the .bashrc file if you want to run it everytime from the terminal. The $pwd should be replaced with the path to Firmware folder.

roslaunch px4 multi_uav_mavros_sitl.launch

VIO

cd ~/catkin_ws
catkin build px4_realsense_bridge


