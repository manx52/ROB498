cmake_minimum_required(VERSION 3.0.2)
project(drone)

find_package(
    catkin
    REQUIRED
    COMPONENTS rospy
    dynamic_reconfigure
      tf
      pcl_ros
      mavros
      mavros_extras
      mavros_msgs
      mavlink

)
catkin_package()
catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})