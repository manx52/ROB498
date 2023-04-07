cmake_minimum_required(VERSION 3.0.2)
project(drone_perception)

add_compile_options(-std=c++17)

set(PKG_DEPS
    rospy geometry_msgs drone_msgs cv_bridge sensor_msgs jetson_camera image_proc stereo_image_proc camera_calibration
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

catkin_install_python(PROGRAMS src/drone_perception/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_python_setup()
