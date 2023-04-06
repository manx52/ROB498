cmake_minimum_required(VERSION 3.0.2)
project(drone_control)

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

)
find_package(gazebo REQUIRED)

catkin_package(DEPENDS
      gazebo_ros
      gazebo_msgs
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES)

catkin_install_python(PROGRAMS src/drone_control/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()