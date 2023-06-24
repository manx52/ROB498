[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
[![CodeFactor](https://www.codefactor.io/repository/github/manx52/rob498/badge/main)](https://www.codefactor.io/repository/github/manx52/rob498/overview/main)
[![Docker Image CI](https://github.com/manx52/ROB498/actions/workflows/docker_image.yml/badge.svg)](https://github.com/manx52/ROB498/actions/workflows/docker_image.yml)
[![Docker Image Arm64 CI](https://github.com/manx52/ROB498/actions/workflows/docker_image_arm.yml/badge.svg)](https://github.com/manx52/ROB498/actions/workflows/docker_image_arm.yml)
[![Documentation Status](https://readthedocs.org/projects/rob498/badge/?version=latest)](https://manx52.github.io/ROB498/api.html)
[![Docker Image Size](https://badgen.net/docker/size/utrarobosoccer/rob498?icon=docker&label=image%20size)](https://hub.docker.com/r/utrarobosoccer/rob498/)
[![Docker Pulls](https://badgen.net/docker/pulls/utrarobosoccer/rob498?icon=docker&label=pulls)](https://hub.docker.com/r/utrarobosoccer/rob498/)



### Running Instructions for Computer
- docker-compose.yaml is for a docker container that runs on a normal computer
- docker-compose.robot.yaml  is for a docker container that runs on a Jetson Nano. Building from scratch will take more then an hour.
```bash
roslaunch drone mavros_posix_sitl.launch # run simulation
roslaunch drone gui.launch # For visualization

docker-compose -f docker-compose.yaml pull # Use docker-compose build if you want to build locally
docker-compose -f docker-compose.yaml up

```

### Pull and Run the ARM image for Jetson Nano
```bash
docker-compose -f docker-compose.robot.yaml pull
docker-compose -f docker-compose.robot.yaml up
```

#### Test Waypoints and Vicon
```bash
rostopic pub /rob498_drone_07/comm/test_env std_msgs/Int8 "data: 8" # 4 or 8 depending on waypoints
```
or 
```bash
rostopic pub /vicon/ROB498_Drone/ROB498_Drone geometry_msgs/TransformStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
child_frame_id: ''
transform:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 
    
rostopic pub /rob498_drone_07/comm/test_waypoints std_msgs/Int8 "data: 8" # 4 or 8 depending on waypoints

```

#### Services
```bash
rosservice call /rob498_drone_07/comm/launch
rosservice call /rob498_drone_07/comm/test
rosservice call /rob498_drone_07/comm/land
rosservice call /rob498_drone_07/comm/abort
```

### Controling Drone from remote computer
Make sure your computer and the robot are connected to the same network, then use your personal computer as the ROS_MASTER_URI. You can find the IP address using ```ifconfig```
 - On your personal computer (or a dedicated server computer)
```bash
export ROS_MASTER_URI=http://<your computer ip>:11311
export ROS_IP=<your computer ip>
```
- On your person computer edit ```/etc/hosts``` and add ```<robot1_ipaddress> <robot1_hostname>``` as an a line
- On the robot add this to the bashrc
```bash
export ROS_MASTER_URI=http://<your computer ip>:11311
```
- Verify creating a publisher and subscriber on robot and your computer using ```rostopic pub```, ```rostopic echo``` and verify that the nodes can connect to each other using ```rosnode info <subscriber/publisher> node
