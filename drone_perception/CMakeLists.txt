cmake_minimum_required(VERSION 3.0.2)
project(drone_perception)

add_compile_options(-std=c++17)

set(PKG_DEPS
    rospy geometry_msgs drone_msgs cv_bridge sensor_msgs
)

find_package(
    catkin
    REQUIRED ${PKG_DEPS}
)

catkin_package(CATKIN_DEPENDS drone_msgs )

# ##############################################################################
# Build ##
# ##############################################################################
include_directories(${catkin_INCLUDE_DIRS})

catkin_python_setup()
