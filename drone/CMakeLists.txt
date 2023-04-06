cmake_minimum_required(VERSION 3.0.2)
project(drone)

find_package(
    catkin
    REQUIRED
    COMPONENTS
      gazebo_ros
      gazebo_msgs
      drone_description
      drone_mapping
      drone_control
      drone_common
      px4_realsense_bridge
      drone_perception
      drone_msgs
      realsense2_camera

)
find_package(gazebo REQUIRED)

catkin_package(DEPENDS
      gazebo_ros
      gazebo_msgs
    CATKIN_DEPENDS
       drone_msgs
    INCLUDE_DIRS
    LIBRARIES)


