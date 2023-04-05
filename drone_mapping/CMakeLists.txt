cmake_minimum_required(VERSION 3.0.2)
project(drone_mapping)

add_compile_options(-std=c++17)

set(PKG_DEPS
    rospy nav_msgs ros_numpy
)

find_package(
    catkin
    REQUIRED ${PKG_DEPS}
)

catkin_package(CATKIN_DEPENDS nav_msgs )

# ##############################################################################
# Build ##
# ##############################################################################
include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS src/drone_mapping/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()
