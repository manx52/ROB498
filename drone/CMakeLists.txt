cmake_minimum_required(VERSION 3.0.2)
project(drone)

find_package(
    catkin
    REQUIRED
    COMPONENTS
    rospy
    roscpp
    dynamic_reconfigure
      tf
      pcl_ros
      mavros
      mavros_extras
      mavros_msgs
      mavlink
      gazebo_ros
      gazebo_msgs
      drone_description
      drone_mapping
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

catkin_install_python(PROGRAMS src/drone/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()