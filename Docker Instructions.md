### Setup Instructions
The docker setup is only available for computers with NVIDIA-capable GPUs. TO verify this, run ```nvidia-smi``` and verify if you have any problems. If you haven't installed it, go to Software & Updates >> Additional Drivers and install the recommended NVIDIA driver

#### Verify CUDA on Jetson Nano 
```bash
cd /usr/bin
sudo ./tegrastats
```

Setup your ssh-keys
```bash
ssh-keygen # Then keep entering, don't set a password
cd ~/.ssh &&  cat id_rsa.pub # Get contents of public key
# on github > Settings > SSH and GPG Keys > New SSH Key, paste the key
```

Create a catkin workspace:
```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
catkin init
cd src
git clone git@github.com:manx52/ROB498.git

```


#### Install necessary packages
```bash
sudo apt-get install apt-utils python3-pip
pip3 install --upgrade setuptools
sudo apt-get install -y curl

```

#### Install docker (https://docs.docker.com/engine/install/ubuntu/)
```bash
curl -sSL https://get.docker.com/ | sh
```

#### Install docker-compose using pip install method (https://docs.docker.com/compose/install/)
```bash
sudo python3 -m pip install docker-compose
```

#### Install docker NVIDIA tools
```bash
# Make sure you have nvidia drivers (nvidia-smi), if not https://linuxconfig.org/how-to-install-the-nvidia-drivers-on-ubuntu-20-04-focal-fossa-linux
curl -s -L https://nvidia.github.io/nvidia-container-runtime/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-container-runtime/$distribution/nvidia-container-runtime.list | sudo tee /etc/apt/sources.list.d/nvidia-container-runtime.list
sudo apt-get update
sudo apt-get install -y nvidia-container-runtime nvidia-container-toolkit mesa-utils
sudo systemctl restart docker
echo "xhost +local:docker" >> ~/.bashrc && source ~/.bashrc
```


### Running Instructions for computer
```bash
docker login -u utrasoccer # Password dckr_pat_r8_EwMV7RmiYNG2VbnculJUqc1w
docker-compose pull # Use docker-compose build if you want to build locally
docker-compose up
# docker-compose -f docker-compose.robot.yaml up
```

### Pull and Run the ARM image for Jetson Nano
```bash
docker-compose -f docker-compose.robot.yaml pull
docker-compose -f docker-compose.robot.yaml up
```

### Notes
- To run just simulator run docker-compose up simulator
- You never need to build the images a second time as all the code is mounted
- To run the full game run docker-compose -f docker-compose.full.yaml up
- To avoid opening rviz everytime, run the following
```bash
sudo rm /opt/ros/noetic/share/rviz/default.rviz
sudo ln -s /home/$USER/catkin_ws/src/soccerbot/soccerbot/rviz/soccerbot.rviz /opt/ros/noetic/share/rviz/default.rviz
```
